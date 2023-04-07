/*
 * sound/soc/amlogic/auge/tdm.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <linux/amlogic/clk_measure.h>
#include <linux/amlogic/cpu_version.h>

#include <linux/amlogic/media/sound/aout_notify.h>

#include "ddr_mngr.h"
#include "tdm_hw.h"
#include "sharebuffer.h"
#include "vad.h"
#include "spdif_hw.h"
#include "tdm_match_table.c"
#include "effects_v2.h"
#include "spdif.h"


/*#define __PTM_TDM_CLK__*/


#define DRV_NAME "snd_tdm"

static int aml_dai_set_tdm_sysclk(struct snd_soc_dai *cpu_dai,
				int clk_id, unsigned int freq, int dir);

static void dump_pcm_setting(struct pcm_setting *setting)
{
	if (setting == NULL)
		return;

	pr_debug("dump_pcm_setting(%p)\n", setting);
	pr_debug("\tpcm_mode(%d)\n", setting->pcm_mode);
	pr_debug("\tsysclk(%d)\n", setting->sysclk);
	pr_debug("\tsysclk_bclk_ratio(%d)\n", setting->sysclk_bclk_ratio);
	pr_debug("\tbclk(%d)\n", setting->bclk);
	pr_debug("\tbclk_lrclk_ratio(%d)\n", setting->bclk_lrclk_ratio);
	pr_debug("\tlrclk(%d)\n", setting->lrclk);
	pr_debug("\ttx_mask(%#x)\n", setting->tx_mask);
	pr_debug("\trx_mask(%#x)\n", setting->rx_mask);
	pr_debug("\tslots(%d)\n", setting->slots);
	pr_debug("\tslot_width(%d)\n", setting->slot_width);
	pr_debug("\tlane_mask_in(%#x)\n", setting->lane_mask_in);
	pr_debug("\tlane_mask_out(%#x)\n", setting->lane_mask_out);
	pr_debug("\tlane_oe_mask_in(%#x)\n", setting->lane_oe_mask_in);
	pr_debug("\tlane_oe_mask_out(%#x)\n", setting->lane_oe_mask_out);
	pr_debug("\tlane_lb_mask_in(%#x)\n", setting->lane_lb_mask_in);
}

struct aml_tdm {
	struct pcm_setting setting;
	struct pinctrl *pin_ctl;
	struct aml_audio_controller *actrl;
	struct device *dev;
	struct clk *clk;
	struct clk *clk_gate;
	struct clk *mclk;
	/* mclk mux out to pad */
	struct clk *mclk2pad;
	struct clk *samesrc_srcpll;
	struct clk *samesrc_clk;
	bool contns_clk;
	unsigned int id;
	/* bclk src selection */
	unsigned int clk_sel;
	struct toddr *tddr;
	struct frddr *fddr;

	struct tdm_chipinfo *chipinfo;
	/* share buffer with module */
	int samesource_sel;
	/* share buffer lane setting from DTS */
	int lane_ss;
	int mclk_pad;
	/* virtual link for i2s to hdmitx */
	int i2s2hdmitx;
	int acodec_adc;
	uint last_mpll_freq;
	uint last_mclk_freq;
	uint last_fmt;

	bool en_share;
	unsigned int lane_cnt;

	/* tdmin_lb src sel */
	int tdmin_lb_src;
	int start_clk_enable;
	int clk_tuning_enable;
	int last_rate;
};

static const struct snd_pcm_hardware aml_tdm_hardware = {
	.info =
	SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_MMAP_VALID |
	SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_PAUSE,
	.formats =
	    SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
	    SNDRV_PCM_FMTBIT_S32_LE,

	.period_bytes_min = 64,
	.period_bytes_max = 256 * 1024 * 2,
	.periods_min = 2,
	.periods_max = 1024,
	.buffer_bytes_max = 512 * 1024 * 2,

	.rate_min = 8000,
	.rate_max = 384000,
	.channels_min = 1,
	.channels_max = 32,
};

static int tdm_clk_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);

	ucontrol->value.enumerated.item[0] = clk_get_rate(p_tdm->mclk);
	return 0;
}

static int tdm_clk_set(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);

	int mclk_rate = p_tdm->last_mclk_freq;
	int value = ucontrol->value.enumerated.item[0];

	if (value > 2000000 || value < 0) {
		pr_err("Fine tdm clk setting range (0~2000000), %d\n", value);
		return 0;
	}
	mclk_rate += (value - 1000000);

	mclk_rate >>= 1;
	mclk_rate <<= 1;

	aml_dai_set_tdm_sysclk(cpu_dai, 0, mclk_rate, 0);

	return 0;
}

static int tdmin_clk_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int clk;
	int value;

	clk = meson_clk_measure(70);
	if (clk >= 11000000)
		value = 3;
	else if (clk >= 6000000)
		value = 2;
	else if (clk >= 2000000)
		value = 1;
	else
		value = 0;


	ucontrol->value.integer.value[0] = value;

	return 0;
}

/* current sample mode and its sample rate */
static const char *const i2sin_clk[] = {
	"0",
	"3000000",
	"6000000",
	"12000000"
};

static const struct soc_enum i2sin_clk_enum[] = {
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(i2sin_clk),
			i2sin_clk),
};

static const struct snd_kcontrol_new snd_tdm_controls[] = {
	SOC_ENUM_EXT("I2SIn CLK", i2sin_clk_enum,
				tdmin_clk_get,
				NULL),
};

static const struct snd_kcontrol_new snd_tdm_clk_controls[] = {
	SOC_SINGLE_EXT("TDM MCLK Fine Setting",
				0, 0, 2000000, 0,
				tdm_clk_get,
				tdm_clk_set),
};

static irqreturn_t aml_tdm_ddr_isr(int irq, void *devid)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)devid;

	if (!snd_pcm_running(substream))
		return IRQ_HANDLED;

	snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

/* get counts of '1's in val */
static unsigned int pop_count(unsigned int val)
{
	unsigned int count = 0;

	while (val) {
		count++;
		val = val & (val - 1);
	}

	return count;
}

static int snd_soc_of_get_slot_mask(struct device_node *np,
				    const char *prop_name,
				    unsigned int *mask)
{
	u32 val;
	const __be32 *of_slot_mask = of_get_property(np, prop_name, &val);
	int i;

	if (!of_slot_mask)
		return -EINVAL;

	val /= sizeof(u32);
	for (i = 0; i < val; i++)
		if (be32_to_cpup(&of_slot_mask[i]))
			*mask |= (1 << i);

	return val;
}

static int of_parse_tdm_lane_slot_in(struct device_node *np,
			      unsigned int *lane_mask)
{
	if (lane_mask)
		return snd_soc_of_get_slot_mask(np,
			"dai-tdm-lane-slot-mask-in", lane_mask);

	return -EINVAL;
}

static int of_parse_tdm_lane_slot_out(struct device_node *np,
			      unsigned int *lane_mask)
{
	if (lane_mask)
		return snd_soc_of_get_slot_mask(np,
			"dai-tdm-lane-slot-mask-out", lane_mask);

	return -EINVAL;
}

static int of_parse_tdm_lane_oe_slot_in(struct device_node *np,
			      unsigned int *lane_mask)
{
	if (lane_mask)
		return snd_soc_of_get_slot_mask(np,
			"dai-tdm-lane-oe-slot-mask-in", lane_mask);

	return -EINVAL;
}

static int of_parse_tdm_lane_oe_slot_out(struct device_node *np,
			      unsigned int *lane_mask)
{
	if (lane_mask)
		return snd_soc_of_get_slot_mask(np,
			"dai-tdm-lane-oe-slot-mask-out", lane_mask);

	return -EINVAL;
}

static int of_parse_tdm_lane_lb_slot_in(struct device_node *np,
			      unsigned int *lane_mask)
{
	if (lane_mask)
		return snd_soc_of_get_slot_mask(np,
			"dai-tdm-lane-lb-slot-mask-in", lane_mask);

	return -EINVAL;
}

static int aml_tdm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct aml_tdm *p_tdm;

	pr_info("%s\n", __func__);
	p_tdm = (struct aml_tdm *)dev_get_drvdata(dev);

	snd_soc_set_runtime_hwparams(substream, &aml_tdm_hardware);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		int dst_id = get_aed_dst();
		bool aed_dst_status = false;

		if (dst_id == p_tdm->id)
			aed_dst_status = true;
		p_tdm->fddr = aml_audio_register_frddr(dev,
			p_tdm->actrl, aml_tdm_ddr_isr,
			substream, aed_dst_status);
		if (p_tdm->fddr == NULL) {
			dev_err(dev, "failed to claim from ddr\n");
			return -ENXIO;
		}
	} else {
		p_tdm->tddr = aml_audio_register_toddr(dev,
			p_tdm->actrl, aml_tdm_ddr_isr, substream);
		if (p_tdm->tddr == NULL) {
			dev_err(dev, "failed to claim to ddr\n");
			return -ENXIO;
		}
	}

	runtime->private_data = p_tdm;
	return 0;
}

static int aml_tdm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_tdm *p_tdm = runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aml_audio_unregister_frddr(p_tdm->dev,
				substream);
	else
		aml_audio_unregister_toddr(p_tdm->dev,
				substream);

	return 0;
}

static int aml_tdm_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int aml_tdm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int aml_tdm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_tdm *p_tdm = runtime->private_data;
	unsigned int start_addr, end_addr, int_addr;

	start_addr = runtime->dma_addr;
	end_addr = start_addr + runtime->dma_bytes - 8;
	int_addr = frames_to_bytes(runtime, runtime->period_size)/8;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		struct frddr *fr = p_tdm->fddr;

		if (p_tdm->chipinfo && p_tdm->chipinfo->async_fifo) {
			int offset = p_tdm->chipinfo->reset_reg_offset;

			pr_debug("%s(), reset fddr\n", __func__);
			aml_frddr_reset(p_tdm->fddr, offset);
			aml_tdm_out_reset(p_tdm->id, offset);

			if (p_tdm->chipinfo->same_src_fn
				&& (p_tdm->samesource_sel >= 0)
				&& (aml_check_sharebuffer_valid(p_tdm->fddr,
					p_tdm->samesource_sel))
				&& p_tdm->en_share)
				aml_spdif_out_reset(p_tdm->samesource_sel - 3,
						offset);
		}
		aml_frddr_set_buf(fr, start_addr, end_addr);
		aml_frddr_set_intrpt(fr, int_addr);
	} else {
		struct toddr *to = p_tdm->tddr;

		aml_toddr_set_buf(to, start_addr, end_addr);
		aml_toddr_set_intrpt(to, int_addr);
	}

	return 0;
}

static snd_pcm_uframes_t aml_tdm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_tdm *p_tdm = runtime->private_data;
	unsigned int addr, start_addr;
	snd_pcm_uframes_t frames;

	start_addr = runtime->dma_addr;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		addr = aml_frddr_get_position(p_tdm->fddr);
	else
		addr = aml_toddr_get_position(p_tdm->tddr);

	frames = bytes_to_frames(runtime, addr - start_addr);
	if (frames > runtime->buffer_size)
		frames = 0;

	return frames;
}

static int aml_tdm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *vma)
{
	return snd_pcm_lib_default_mmap(substream, vma);
}

static struct snd_pcm_ops aml_tdm_ops = {
	.open = aml_tdm_open,
	.close = aml_tdm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = aml_tdm_hw_params,
	.hw_free = aml_tdm_hw_free,
	.prepare = aml_tdm_prepare,
	.pointer = aml_tdm_pointer,
	.mmap = aml_tdm_mmap,
};

#define PREALLOC_BUFFER		(256 * 1024)
#define PREALLOC_BUFFER_MAX	(256 * 1024)
static int aml_tdm_new(struct snd_soc_pcm_runtime *rtd)
{
	return snd_pcm_lib_preallocate_pages_for_all(
		rtd->pcm, SNDRV_DMA_TYPE_DEV,
		rtd->card->snd_card->dev, PREALLOC_BUFFER, PREALLOC_BUFFER_MAX);
}

struct snd_soc_platform_driver aml_tdm_platform = {
	.ops = &aml_tdm_ops,
	.pcm_new = aml_tdm_new,
};

static int aml_dai_tdm_prepare(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *cpu_dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	int bit_depth;

	bit_depth = snd_pcm_format_width(runtime->format);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		struct frddr *fr = p_tdm->fddr;
		enum frddr_dest dst;
		unsigned int fifo_id;

		/* share buffer prepare */
		if (p_tdm->chipinfo &&
			p_tdm->chipinfo->same_src_fn
			&& (p_tdm->samesource_sel >= 0)
			&& (aml_check_sharebuffer_valid(p_tdm->fddr,
				p_tdm->samesource_sel))
			&& p_tdm->en_share) {
				sharebuffer_prepare(substream,
					fr, p_tdm->samesource_sel,
					p_tdm->lane_ss,
					p_tdm->chipinfo->reset_reg_offset);
					/* sharebuffer default uses spdif_a */
				spdif_set_audio_clk(p_tdm->samesource_sel - 3,
					p_tdm->clk,
					(p_tdm->last_mclk_freq >> 1), 1);
		}

		/* i2s source to hdmix */
		if (p_tdm->i2s2hdmitx) {
			i2s_to_hdmitx_ctrl(p_tdm->id);
			aout_notifier_call_chain(AOUT_EVENT_IEC_60958_PCM,
				substream);
		}

		fifo_id = aml_frddr_get_fifo_id(fr);
		aml_tdm_fifo_ctrl(p_tdm->actrl,
			bit_depth,
			substream->stream,
			p_tdm->id,
			fifo_id);

		switch (p_tdm->id) {
		case 0:
			dst = TDMOUT_A;
			break;
		case 1:
			dst = TDMOUT_B;
			break;
		case 2:
			dst = TDMOUT_C;
			break;
		default:
			dev_err(p_tdm->dev,	"invalid id: %d\n",
					p_tdm->id);
			return -EINVAL;
		}
		aml_frddr_set_format(fr,
			runtime->channels,
			bit_depth - 1,
			tdmout_get_frddr_type(bit_depth));
		aml_frddr_select_dst(fr, dst);
		aml_frddr_set_fifos(fr, 0x40, 0x20);
	} else {
		struct toddr *to = p_tdm->tddr;
		enum toddr_src src;
		unsigned int lsb = 32 - bit_depth;
		unsigned int toddr_type;
		struct toddr_fmt fmt;

		if (vad_tdm_is_running(p_tdm->id)
			&& pm_audio_is_suspend())
			return 0;

		switch (bit_depth) {
		case 8:
		case 16:
		case 32:
			toddr_type = 0;
			break;
		case 24:
			toddr_type = 4;
			break;
		default:
			dev_err(p_tdm->dev, "invalid bit_depth: %d\n",
					bit_depth);
			return -EINVAL;
		}

		dev_info(substream->pcm->card->dev, "tdm prepare capture\n");
		switch (p_tdm->id) {
		case 0:
			src = TDMIN_A;
		break;
		case 1:
			src = TDMIN_B;
		break;
		case 2:
			src = TDMIN_C;
		break;
		case 3:
			src = TDMIN_LB;
		break;
		default:
			dev_err(p_tdm->dev, "invalid id: %d\n",
					p_tdm->id);
			return -EINVAL;
		}

		if (toddr_src_get() == FRHDMIRX) {
			src = FRHDMIRX;

			aml_update_tdmin_src(p_tdm->actrl,
				p_tdm->id,
				HDMIRX_I2S);
		}  else if (p_tdm->chipinfo
			&& p_tdm->chipinfo->adc_fn
			&& p_tdm->acodec_adc)
			aml_update_tdmin_src(p_tdm->actrl,
				p_tdm->id,
				ACODEC_ADC);

		pr_info("%s Expected toddr src:%s\n",
			__func__,
			toddr_src_get_str(src));

		fmt.type      = toddr_type;
		fmt.msb       = 31;
		fmt.lsb       = lsb;
		fmt.endian    = 0;
		fmt.bit_depth = bit_depth;
		fmt.ch_num    = runtime->channels;
		fmt.rate      = runtime->rate;
		aml_toddr_select_src(to, src);
		aml_toddr_set_format(to, &fmt);
		aml_toddr_set_fifos(to, 0x40);
	}

	return 0;
}

static int aml_dai_tdm_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *cpu_dai)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

		if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			&& vad_tdm_is_running(p_tdm->id)
			&& pm_audio_is_suspend()) {
			pm_audio_set_suspend(false);
			/* VAD switch to alsa buffer */
			vad_update_buffer(0);
			break;
		}

		/* reset fifo here.
		 * If not, xrun will cause channel mapping mismatch
		 */
		aml_tdm_fifo_reset(p_tdm->actrl, substream->stream, p_tdm->id);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* output START sequence:
			 * 1. Frddr/TDMOUT/SPDIF reset(may cause the AVR mute)
			 * 2. ctrl0 set to 0
			 * 3. TDMOUT enable
			 * 4. SPDIFOUT enable
			 * 5. FRDDR enable
			 */
			dev_info(substream->pcm->card->dev, "tdm playback enable\n");
			/*don't change this flow*/
			aml_aed_top_enable(p_tdm->fddr, true);
			aml_tdm_enable(p_tdm->actrl,
				substream->stream, p_tdm->id, true);
			if (p_tdm->chipinfo
				&& p_tdm->chipinfo->same_src_fn
				&& (p_tdm->samesource_sel >= 0)
				&& (aml_check_sharebuffer_valid(p_tdm->fddr,
						p_tdm->samesource_sel))
				&& p_tdm->en_share)
				sharebuffer_trigger(cmd,
					p_tdm->samesource_sel,
					p_tdm->chipinfo->same_src_spdif_reen);

			aml_frddr_enable(p_tdm->fddr, true);
			udelay(100);
			aml_tdmout_enable_gain(p_tdm->id, false);
			if (p_tdm->chipinfo
				&& p_tdm->chipinfo->same_src_fn
				&& (p_tdm->samesource_sel >= 0)
				&& (aml_check_sharebuffer_valid(p_tdm->fddr,
						p_tdm->samesource_sel))
				&& p_tdm->en_share) {
				aml_spdifout_mute_without_actrl(0, false);
			}
		} else {
			dev_info(substream->pcm->card->dev, "tdm capture enable\n");
			aml_toddr_enable(p_tdm->tddr, 1);
			aml_tdm_enable(p_tdm->actrl,
				substream->stream, p_tdm->id, true);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			&& vad_tdm_is_running(p_tdm->id)
			&& pm_audio_is_suspend()) {
			/* switch to VAD buffer */
			vad_update_buffer(1);
			break;
		}

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* output STOP sequence:
			 * 1. TDMOUT->muteval
			 * 2. SPDIFOUT->muteval
			 * 3. TDMOUT/SPDIF Disable
			 * 4. FRDDR Disable
			 */
			dev_info(substream->pcm->card->dev, "tdm playback stop\n");
			/*don't change this flow*/
			aml_tdmout_enable_gain(p_tdm->id, true);
			if (p_tdm->chipinfo
				&& p_tdm->chipinfo->same_src_fn
				&& (p_tdm->samesource_sel >= 0)
				&& (aml_check_sharebuffer_valid(p_tdm->fddr,
						p_tdm->samesource_sel))
				&& p_tdm->en_share) {
				aml_spdifout_mute_without_actrl(0, true);
			}
			aml_aed_top_enable(p_tdm->fddr, false);
			aml_tdm_enable(p_tdm->actrl,
				substream->stream, p_tdm->id, false);
			if (p_tdm->chipinfo	&& p_tdm->chipinfo->same_src_fn
				&& (p_tdm->samesource_sel >= 0)
				&& (aml_check_sharebuffer_valid(p_tdm->fddr,
						p_tdm->samesource_sel))
				&& p_tdm->en_share)
				sharebuffer_trigger(cmd,
					p_tdm->samesource_sel,
					p_tdm->chipinfo->same_src_spdif_reen);

			if (p_tdm->chipinfo	&&
				p_tdm->chipinfo->async_fifo)
				aml_frddr_check(p_tdm->fddr);

			aml_frddr_enable(p_tdm->fddr, false);
		} else {
			bool toddr_stopped = false;

			aml_tdm_enable(p_tdm->actrl,
				substream->stream, p_tdm->id, false);
			dev_info(substream->pcm->card->dev, "tdm capture stop\n");

			toddr_stopped = aml_toddr_burst_finished(p_tdm->tddr);
			if (toddr_stopped)
				aml_toddr_enable(p_tdm->tddr, false);
			else
				pr_err("%s(), toddr may be stuck\n", __func__);
		}

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pcm_setting_init(struct pcm_setting *setting, unsigned int rate,
			unsigned int channels)
{
	unsigned int ratio = 0;

	setting->lrclk = rate;
	setting->bclk_lrclk_ratio = setting->slots * setting->slot_width;
	setting->bclk = setting->lrclk * setting->bclk_lrclk_ratio;

	/* calculate mclk */
	if (setting->pcm_mode == SND_SOC_DAIFMT_DSP_A ||
		setting->pcm_mode == SND_SOC_DAIFMT_DSP_B) {
		/* for some TDM codec, mclk limites */
		ratio = 2;
	} else {
		if (rate > 192000)
			ratio = 2;
		else
			ratio = 4;
	}
	setting->sysclk_bclk_ratio = ratio;
	setting->sysclk = ratio * setting->bclk;

	return 0;
}
static int aml_tdm_set_lanes(struct aml_tdm *p_tdm,
				unsigned int channels, int stream)
{
	struct pcm_setting *setting = &p_tdm->setting;
	unsigned int lanes, swap_val = 0, swap_val1 = 0;
	unsigned int lane_mask;
	unsigned int set_num = 0;
	unsigned int i;

	/* calc lanes by channels and slots */
	lanes = (channels - 1) / setting->slots + 1;
	if (lanes > p_tdm->lane_cnt) {
		pr_err("set lane error! asoc channels:%d, slots:%d, lane_cnt:%d\n",
		channels, setting->slots, p_tdm->lane_cnt);
		return -EINVAL;
	}

#if 1
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set lanes mask acordingly */
		lane_mask = setting->lane_mask_out;
		/* compatible using oe masks */
		if (!lane_mask && setting->lane_oe_mask_out)
			lane_mask = setting->lane_oe_mask_out;

		for (i = 0; i < p_tdm->lane_cnt; i++) {
			if (((1 << i) & lane_mask) && lanes) {
				aml_tdm_set_channel_mask(p_tdm->actrl,
					stream, p_tdm->id, i, setting->tx_mask);
				lanes--;
			}
		}
		swap_val = 0x76543210;
		if (p_tdm->lane_cnt > LANE_MAX1)
			swap_val1 = 0xfedcba98;
		aml_tdm_set_lane_channel_swap(p_tdm->actrl,
			stream, p_tdm->id, swap_val, swap_val1);
	} else {
		if (p_tdm->chipinfo
			&& p_tdm->chipinfo->oe_fn
			&& p_tdm->setting.lane_oe_mask_in)
			lane_mask = setting->lane_oe_mask_in;
		else
			lane_mask = setting->lane_mask_in;

		for (i = 0; i < p_tdm->lane_cnt; i++) {
			if (i < lanes)
				aml_tdm_set_channel_mask(p_tdm->actrl,
					stream, p_tdm->id, i, setting->rx_mask);
			if (((1 << i) & lane_mask) && (i < LANE_MAX1)) {
				/* each lane only L/R masked */
				pr_debug("tdmin set lane %d\n", i);
				swap_val |= (i * 2) <<
						(set_num++ * LANE_MAX1);
				swap_val |= (i * 2 + 1) <<
						(set_num++ * LANE_MAX1);
			}
			if (((1 << i) & lane_mask) && (i >= LANE_MAX1)
					&& (i < LANE_MAX3)) {
				/* each lane only L/R masked */
				pr_debug("tdmin set lane %d\n", i);
				swap_val1 |= (i * 2) <<
						(set_num++ * LANE_MAX1);
				swap_val1 |= (i * 2 + 1) <<
						(set_num++ * LANE_MAX1);
			}
		}

		aml_tdm_set_lane_channel_swap(p_tdm->actrl,
			stream, p_tdm->id, swap_val, swap_val1);
	}

#else
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (p_tdm->chipinfo
			&& p_tdm->chipinfo->oe_fn
			&& p_tdm->setting.lane_oe_mask_out)
			lane_mask = setting->lane_oe_mask_out;
		else
			lane_mask = setting->lane_mask_out;
	} else {
		if (p_tdm->chipinfo
			&& p_tdm->chipinfo->oe_fn
			&& p_tdm->setting.lane_oe_mask_in)
			lane_mask = setting->lane_oe_mask_in;
		else
			lane_mask = setting->lane_mask_in;
	}

	/* lane mask */
	for (i = 0; i < 4; i++) {
		if (((1 << i) & lane_mask) && lanes) {
			aml_tdm_set_channel_mask(p_tdm->actrl,
				stream, p_tdm->id, i, setting->tx_mask);
			lanes--;
		}
	}

	/* channel swap */
	for (i = 0; i < p_tdm->lane_cnt; i++) {
		if ((1 << i) & lane_mask) {
			swap0_val |= (1 << (2 * i)) - 1;
			lane_chs += 1;
			if (lane_chs >= channels)
				break;
			swap0_val |= (1 << (2 * i + 1)) - 1;
			lane_chs += 1;
			if (lane_chs >= channels)
				break;

			if (i >= LANE_MAX1) {
				swap1_val |= (1 << (2 * (i - LANE_MAX1))) - 1;
				lane_chs += 1;
				if (lane_chs >= channels)
					break;
				swap1_val |= (1 << (2 * (i - LANE_MAX1) + 1))
								- 1;
				lane_chs += 1;
				if (lane_chs >= channels)
					break;
			}
		}
	}
	aml_tdm_set_lane_channel_swap(p_tdm->actrl,
		stream, p_tdm->id, swap0_val, swap1_val);
#endif
	return 0;
}

static int aml_tdm_set_clk_pad(struct aml_tdm *p_tdm)
{
	int mpad_offset = 0;
	/* mclk pad
	 * does mclk need?
	 * mclk from which mclk source,  mclk_a/b/c/d/e/f
	 * mclk pad controlled by dts, mclk source according to id
	 */
	if (p_tdm->chipinfo && (!p_tdm->chipinfo->mclkpad_no_offset))
		mpad_offset = 1;

	if (p_tdm->mclk_pad >= 0) {
		aml_tdm_mclk_pad_select(p_tdm->actrl,
					p_tdm->mclk_pad,
					mpad_offset,
					p_tdm->clk_sel);
	}

#ifdef CONFIG_ARCH_MESON64_ODROID_COMMON
	aml_tdm_sclk_pad_select(p_tdm->actrl,
				mpad_offset,
				p_tdm->id,
				p_tdm->clk_sel);
#else
	aml_tdm_sclk_pad_select(p_tdm->actrl,
				mpad_offset,
				p_tdm->id,
				p_tdm->id);
#endif

	return 0;
}

static int aml_dai_tdm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	struct pcm_setting *setting = &p_tdm->setting;
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	int ret;

	ret = pcm_setting_init(setting, rate, channels);
	if (ret)
		return ret;

	dump_pcm_setting(setting);

	/* set pcm dai hw params */
	// TODO: add clk_id
	snd_soc_dai_set_sysclk(cpu_dai,
		0, setting->sysclk, SND_SOC_CLOCK_OUT);
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, setting->sysclk_bclk_ratio);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, setting->bclk_lrclk_ratio);
	if (ret)
		return ret;

	ret = aml_tdm_set_lanes(p_tdm, channels, substream->stream);
	if (ret)
		return ret;

	/* Must enabe channel number for VAD */
	if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		&& (vad_tdm_is_running(p_tdm->id)))
		tdmin_set_chnum_en(p_tdm->actrl, p_tdm->id, true);

	/* share buffer trigger */
	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		&& p_tdm->chipinfo
		&& (p_tdm->chipinfo->same_src_fn)
		&& (p_tdm->samesource_sel >= 0)
		&& (aml_check_sharebuffer_valid(p_tdm->fddr,
				p_tdm->samesource_sel))
		&& p_tdm->en_share) {
#if 0
		int mux = 0, ratio = 0;

			sharebuffer_get_mclk_fs_ratio(p_tdm->samesource_sel,
				&mux, &ratio);
			pr_info("samesource sysclk:%d\n", rate * ratio * mux);
			if (!IS_ERR(p_tdm->samesrc_srcpll)) {
				clk_set_rate(p_tdm->samesrc_srcpll,
					rate * ratio * mux);
				clk_prepare_enable(p_tdm->samesrc_srcpll);
			}
			if (!IS_ERR(p_tdm->samesrc_clk)) {
				clk_set_rate(p_tdm->samesrc_clk,
					rate * ratio);
				clk_prepare_enable(p_tdm->samesrc_clk);
			}
#endif
	}

	if (!p_tdm->contns_clk && !IS_ERR(p_tdm->mclk)) {
		pr_debug("%s(), enable mclk for %s\n", __func__, cpu_dai->name);
		ret = clk_prepare_enable(p_tdm->mclk);
		if (ret) {
			pr_err("Can't enable mclk: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int aml_dai_tdm_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *cpu_dai)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	struct frddr *fr = p_tdm->fddr;
	int i;

	for (i = 0; i < p_tdm->lane_cnt; i++)
		aml_tdm_set_channel_mask(p_tdm->actrl,
			substream->stream, p_tdm->id, i, 0);

	/* Disable channel number for VAD */
	if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		&& (vad_tdm_is_running(p_tdm->id)))
		tdmin_set_chnum_en(p_tdm->actrl, p_tdm->id, false);

	/* share buffer free */
	if ((substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		&& p_tdm->chipinfo
		&& p_tdm->chipinfo->same_src_fn
		&& (p_tdm->samesource_sel >= 0)
		&& fr
		&& (aml_check_sharebuffer_valid(fr, p_tdm->samesource_sel))
		&& p_tdm->en_share) {
			sharebuffer_free(substream,
				fr, p_tdm->samesource_sel);
	}

	/* disable clock and gate */
	if (!p_tdm->contns_clk && !IS_ERR(p_tdm->mclk)) {
		pr_info("%s(), disable mclk for %s", __func__, cpu_dai->name);
		clk_disable_unprepare(p_tdm->mclk);
	}

	return 0;
}

static int aml_dai_set_tdm_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);

	pr_debug("asoc aml_dai_set_tdm_fmt, %#x, %p, id(%d), clksel(%d)\n",
		fmt, p_tdm, p_tdm->id, p_tdm->clk_sel);
	if (p_tdm->last_fmt == fmt) {
		pr_debug("%s(), fmt not change\n", __func__);
		goto capture;
	} else
		p_tdm->last_fmt = fmt;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_MASK) {
	case SND_SOC_DAIFMT_CONT:
		p_tdm->contns_clk = true;
		break;
	case SND_SOC_DAIFMT_GATED:
		p_tdm->contns_clk = false;
		break;
	default:
		return -EINVAL;
	}

	if (p_tdm->chipinfo)
		p_tdm->setting.sclk_ws_inv = p_tdm->chipinfo->sclk_ws_inv;

	aml_tdm_set_format(p_tdm->actrl,
		&(p_tdm->setting), p_tdm->clk_sel, p_tdm->id, fmt,
		1, 1);
	if (p_tdm->contns_clk && !IS_ERR(p_tdm->mclk)) {
		int ret = clk_prepare_enable(p_tdm->mclk);

		if (ret) {
			pr_err("Can't enable mclk: %d\n", ret);
			return ret;
		}
	}

capture:
	/* update skew for ACODEC_ADC */
	if (cpu_dai->capture_active
		&& p_tdm->chipinfo
		&& p_tdm->chipinfo->adc_fn
		&& p_tdm->acodec_adc){
		aml_update_tdmin_skew(p_tdm->actrl, p_tdm->id, 4);
		aml_update_tdmin_rev_ws(p_tdm->actrl, p_tdm->id, 0);
	}

	return 0;
}

/* mpll clk range from 5M to 500M */
#define AML_MPLL_FREQ_MIN	5000000
static unsigned int aml_mpll_mclk_ratio(unsigned int freq)
{
	unsigned int i, ratio = 2;
	unsigned int mpll_freq = 0;

	for (i = 1; i < 15; i++) {
		ratio = 1 << i;
		mpll_freq = freq * ratio;

		if (mpll_freq > AML_MPLL_FREQ_MIN)
			break;
	}

	return ratio;
}

static int aml_dai_set_tdm_sysclk(struct snd_soc_dai *cpu_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int ratio = aml_mpll_mclk_ratio(freq);
	unsigned int mpll_freq = 0;

	p_tdm->setting.sysclk = freq;

#ifdef __PTM_TDM_CLK__
	if (p_tdm->id == 0)
		ratio = 14;
	else if (p_tdm->id == 1)
		ratio = 18 * 2;
	else if (p_tdm->id == 2)
		ratio = 20;
#endif

	mpll_freq = freq * ratio;
	if (mpll_freq != p_tdm->last_mpll_freq) {
		clk_set_rate(p_tdm->clk, mpll_freq);
		p_tdm->last_mpll_freq = mpll_freq;
	} else {
		pr_debug("%s(), mpll no change, keep clk\n", __func__);
	}

	if (freq != p_tdm->last_mclk_freq) {
		clk_set_rate(p_tdm->mclk, freq);
		p_tdm->last_mclk_freq = freq;
	} else {
		pr_debug("%s(), mclk no change, keep clk\n", __func__);
	}

	pr_debug("set mclk:%d, mpll:%d, get mclk:%lu, mpll:%lu\n",
		freq,
		freq * ratio,
		clk_get_rate(p_tdm->mclk),
		clk_get_rate(p_tdm->clk));

	return 0;
}

static int aml_dai_set_bclk_ratio(struct snd_soc_dai *cpu_dai,
						unsigned int ratio)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int bclk_ratio, lrclk_hi;

	p_tdm->setting.bclk_lrclk_ratio = ratio;
	bclk_ratio = ratio - 1;
	lrclk_hi = 0;

	if (p_tdm->setting.pcm_mode == SND_SOC_DAIFMT_I2S ||
		p_tdm->setting.pcm_mode == SND_SOC_DAIFMT_LEFT_J) {
		pr_debug("aml_dai_set_bclk_ratio, select I2S mode\n");
		lrclk_hi = bclk_ratio / 2;
	} else {
		pr_debug("aml_dai_set_bclk_ratio, select TDM mode\n");
	}
	aml_tdm_set_bclk_ratio(p_tdm->actrl,
		p_tdm->clk_sel, lrclk_hi, bclk_ratio);

	return 0;
}

static int aml_dai_set_clkdiv(struct snd_soc_dai *cpu_dai,
						int div_id, int div)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int mclk_ratio;

	pr_debug("aml_dai_set_clkdiv, div %d, clksel(%d)\n",
			div, p_tdm->clk_sel);

	p_tdm->setting.sysclk_bclk_ratio = div;
	mclk_ratio = div - 1;
	aml_tdm_set_lrclkdiv(p_tdm->actrl, p_tdm->clk_sel, mclk_ratio);

	return 0;
}

static int aml_dai_set_tdm_slot(struct snd_soc_dai *cpu_dai,
				unsigned int tx_mask, unsigned int rx_mask,
				int slots, int slot_width)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);
	struct snd_soc_dai_driver *drv = cpu_dai->driver;
	unsigned int lanes_out_cnt = 0, lanes_in_cnt = 0;
	unsigned int lanes_oe_out_cnt = 0, lanes_oe_in_cnt = 0;
	unsigned int force_oe = 0, oe_val = 0;
	unsigned int lanes_lb_cnt = 0;
	int out_lanes = 0, in_lanes = 0;
	int in_src = -1;

	lanes_out_cnt = pop_count(p_tdm->setting.lane_mask_out);
	lanes_in_cnt = pop_count(p_tdm->setting.lane_mask_in);
	lanes_lb_cnt = pop_count(p_tdm->setting.lane_lb_mask_in);

	pr_debug("%s(), txmask(%#x), rxmask(%#x)\n",
		__func__, tx_mask, rx_mask);
	pr_debug("\tlanes_out_cnt(%d), lanes_in_cnt(%d)\n",
		lanes_out_cnt, lanes_in_cnt);
	pr_debug("\tlanes_lb_cnt(%d)\n",
		lanes_lb_cnt);
	pr_debug("\tslots(%d), slot_width(%d)\n",
		slots, slot_width);

	p_tdm->setting.tx_mask = tx_mask;
	p_tdm->setting.rx_mask = rx_mask;
	p_tdm->setting.slots = slots;
	p_tdm->setting.slot_width = slot_width;

	if (p_tdm->setting.lane_mask_in
			& p_tdm->setting.lane_lb_mask_in)
		pr_err("pin(%x) should be selected for only one usage\n",
			p_tdm->setting.lane_mask_in
				& p_tdm->setting.lane_lb_mask_in);

	if (p_tdm->chipinfo && p_tdm->chipinfo->oe_fn) {
		if (p_tdm->setting.lane_mask_out
				& p_tdm->setting.lane_oe_mask_out)
			pr_err("pin(%x) should be selected for only one usage\n",
				p_tdm->setting.lane_mask_out
					& p_tdm->setting.lane_oe_mask_out);

		if ((p_tdm->setting.lane_mask_in
				& p_tdm->setting.lane_oe_mask_in)
			|| (p_tdm->setting.lane_lb_mask_in
				& p_tdm->setting.lane_oe_mask_in))
			pr_err("pin(%x:%x) should be selected for only one usage\n",
				p_tdm->setting.lane_mask_in
					& p_tdm->setting.lane_oe_mask_in,
				p_tdm->setting.lane_lb_mask_in
					& p_tdm->setting.lane_oe_mask_in);

		lanes_oe_out_cnt = pop_count(p_tdm->setting.lane_oe_mask_out);
		lanes_oe_in_cnt = pop_count(p_tdm->setting.lane_oe_mask_in);
		pr_debug
			("\tlanes_oe_out_cnt(%d), lanes_oe_in_cnt(%d)\n",
			lanes_oe_out_cnt, lanes_oe_in_cnt);

		if (lanes_oe_out_cnt) {
			unsigned int oe_fn_version = p_tdm->chipinfo->oe_fn;

			force_oe = (1 << p_tdm->chipinfo->lane_cnt) - 1;
			oe_val = p_tdm->setting.lane_oe_mask_out;
			if (oe_fn_version == OE_FUNCTION_V1) {
				aml_tdm_set_oe_v1
					(p_tdm->actrl, p_tdm->id,
					force_oe, oe_val);
			} else if (oe_fn_version == OE_FUNCTION_V2) {
				aml_tdm_set_oe_v2
					(p_tdm->actrl, p_tdm->id,
					force_oe, oe_val);
			} else {
				pr_err
					("%s(), oe version(%d) not support\n",
					__func__, oe_fn_version);
			}
		}

		if (lanes_lb_cnt)
			in_src = p_tdm->tdmin_lb_src;
		if (lanes_oe_in_cnt)
			in_src = p_tdm->id + 3;
		if (lanes_in_cnt)
			in_src = p_tdm->id;
	} else {
		if (lanes_lb_cnt)
			in_src = p_tdm->tdmin_lb_src;
		if (lanes_in_cnt && lanes_in_cnt <= 4)
			in_src = p_tdm->id;
		if (in_src > 5) {
			pr_err("unknown src(%d) for tdmin\n", in_src);
			return -EINVAL;
		}
	}

	out_lanes = lanes_out_cnt;
	in_lanes = lanes_in_cnt + lanes_oe_in_cnt + lanes_lb_cnt;

	if (p_tdm->chipinfo
		&& p_tdm->chipinfo->adc_fn
		&& p_tdm->acodec_adc) {
		in_src = ACODEC_ADC;
	}

	if (in_lanes > 0 && in_lanes <= LANE_MAX3)
		aml_tdm_set_slot_in(p_tdm->actrl,
			p_tdm->id, in_src, slot_width);

	if (out_lanes > 0 && out_lanes <= LANE_MAX3)
		aml_tdm_set_slot_out(p_tdm->actrl,
			p_tdm->id, slots, slot_width);

	/* constrains hw channels_max by DTS configs */
	drv->playback.channels_max = slots * out_lanes;
	drv->capture.channels_max = slots * in_lanes;

	return 0;
}

static int aml_dai_tdm_probe(struct snd_soc_dai *cpu_dai)
{
	int ret = 0;
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);

	ret = snd_soc_add_dai_controls(cpu_dai, snd_tdm_controls,
					ARRAY_SIZE(snd_tdm_controls));
	if (ret < 0)
		pr_err("%s, failed add snd tdm controls\n", __func__);

	if (p_tdm->clk_tuning_enable == 1) {
		ret = snd_soc_add_dai_controls(cpu_dai,
				snd_tdm_clk_controls,
				ARRAY_SIZE(snd_tdm_clk_controls));
		if (ret < 0)
			pr_err("%s, failed add snd tdm clk controls\n",
				__func__);
	}

	/* config ddr arb */
	aml_tdm_arb_config(p_tdm->actrl);

	return 0;
}

static int aml_dai_tdm_remove(struct snd_soc_dai *cpu_dai)
{

	return 0;
}

static int aml_dai_tdm_mute_stream(struct snd_soc_dai *cpu_dai,
				int mute, int stream)
{
	struct aml_tdm *p_tdm = snd_soc_dai_get_drvdata(cpu_dai);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("tdm playback mute: %d, lane_cnt = %d\n",
				mute, p_tdm->lane_cnt);
		//aml_tdm_mute_playback(p_tdm->actrl, p_tdm->id,
		//		mute, p_tdm->lane_cnt);
	} else if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		pr_debug("tdm capture mute: %d\n", mute);
		aml_tdm_mute_capture(p_tdm->actrl, p_tdm->id,
				mute, p_tdm->lane_cnt);
	}
	return 0;
}

static int aml_set_default_tdm_clk(struct aml_tdm *tdm)
{
	unsigned int mclk = 12288000;
	unsigned int ratio = aml_mpll_mclk_ratio(mclk);
	unsigned int lrclk_hi;
	unsigned int pll = mclk * ratio;

	/*set default i2s clk for codec sequence*/
	tdm->setting.bclk_lrclk_ratio = 64;
	tdm->setting.sysclk_bclk_ratio = 4;
	lrclk_hi = tdm->setting.bclk_lrclk_ratio - 1;

	aml_tdm_set_lrclkdiv(tdm->actrl, tdm->clk_sel,
		tdm->setting.sysclk_bclk_ratio - 1);

	aml_tdm_set_bclk_ratio(tdm->actrl,
		tdm->clk_sel, lrclk_hi/2, lrclk_hi);

	clk_prepare_enable(tdm->mclk);
	clk_set_rate(tdm->clk, pll);
	clk_set_rate(tdm->mclk, mclk);

	tdm->last_mclk_freq = mclk;
	tdm->last_mpll_freq = pll;

	return 0;
}


static struct snd_soc_dai_ops aml_dai_tdm_ops = {
	.prepare = aml_dai_tdm_prepare,
	.trigger = aml_dai_tdm_trigger,
	.hw_params = aml_dai_tdm_hw_params,
	.hw_free = aml_dai_tdm_hw_free,
	.set_fmt = aml_dai_set_tdm_fmt,
	.set_sysclk = aml_dai_set_tdm_sysclk,
	.set_bclk_ratio = aml_dai_set_bclk_ratio,
	.set_clkdiv = aml_dai_set_clkdiv,
	.set_tdm_slot = aml_dai_set_tdm_slot,
	.mute_stream = aml_dai_tdm_mute_stream,
};

#define AML_DAI_TDM_RATES		(SNDRV_PCM_RATE_8000_384000)
#define AML_DAI_TDM_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver aml_tdm_dai[] = {
	{
		.name = "TDM-A",
		.id = 1,
		.probe = aml_dai_tdm_probe,
		.remove = aml_dai_tdm_remove,
		.playback = {
		      .channels_min = 1,
		      .channels_max = 32,
		      .rates = AML_DAI_TDM_RATES,
		      .formats = AML_DAI_TDM_FORMATS,
		},
		.capture = {
		     .channels_min = 1,
		     .channels_max = 32,
		     .rates = AML_DAI_TDM_RATES,
		     .formats = AML_DAI_TDM_FORMATS,
		},
		.ops = &aml_dai_tdm_ops,
		.symmetric_rates = 1,
	},
	{

		.name = "TDM-B",
		.id = 2,
		.probe = aml_dai_tdm_probe,
		.remove = aml_dai_tdm_remove,
		.playback = {
		      .channels_min = 1,
		      .channels_max = 32,
		      .rates = AML_DAI_TDM_RATES,
		      .formats = AML_DAI_TDM_FORMATS,
		},
		.capture = {
		     .channels_min = 1,
		     .channels_max = 32,
		     .rates = AML_DAI_TDM_RATES,
		     .formats = AML_DAI_TDM_FORMATS,
		},
		.ops = &aml_dai_tdm_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "TDM-C",
		.id = 3,
		.probe = aml_dai_tdm_probe,
		.remove = aml_dai_tdm_remove,
		.playback = {
		      .channels_min = 1,
		      .channels_max = 32,
		      .rates = AML_DAI_TDM_RATES,
		      .formats = AML_DAI_TDM_FORMATS,
		},
		.capture = {
		     .channels_min = 1,
		     .channels_max = 32,
		     .rates = AML_DAI_TDM_RATES,
		     .formats = AML_DAI_TDM_FORMATS,
		},
		.ops = &aml_dai_tdm_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "TDMIN-LB",
		.id = 4,
		.probe = aml_dai_tdm_probe,
		.remove = aml_dai_tdm_remove,

		.capture = {
			.channels_min = 1,
			.channels_max = 32,
			.rates = AML_DAI_TDM_RATES,
			.formats = AML_DAI_TDM_FORMATS,
		},
		.ops = &aml_dai_tdm_ops,
		.symmetric_rates = 1,
	}
};

static const struct snd_soc_component_driver aml_tdm_component = {
	.name		= DRV_NAME,
};

static int check_channel_mask(const char *str)
{
	int ret = -1;

	if (!strncmp(str, "i2s_0/1", 7))
		ret = 0;
	else if (!strncmp(str, "i2s_2/3", 7))
		ret = 1;
	else if (!strncmp(str, "i2s_4/5", 7))
		ret = 2;
	else if (!strncmp(str, "i2s_6/7", 7))
		ret = 3;
	return ret;
}

/* spdif same source with i2s */
static void parse_samesrc_channel_mask(struct aml_tdm *p_tdm)
{
	struct device_node *node = p_tdm->dev->of_node;
	struct device_node *np = NULL;
	const char *str = NULL;
	int ret = 0;

	/* channel mask */
	np = of_get_child_by_name(node, "Channel_Mask");
	if (np == NULL) {
		pr_info("No channel mask node %s\n",
				"Channel_Mask");
		return;
	}

	/* If spdif is same source to i2s,
	 * it can be muxed to i2s 2 channels
	 */
	ret = of_property_read_string(np,
			"Spdif_samesource_Channel_Mask", &str);
	if (ret) {
		pr_err("error:read Spdif_samesource_Channel_Mask\n");
		return;
	}
	p_tdm->lane_ss = check_channel_mask(str);

	pr_info("Channel_Mask: lane_ss = %d\n", p_tdm->lane_ss);
}

static int aml_tdm_platform_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *node_prt = NULL;
	struct platform_device *pdev_parent;
	struct device *dev = &pdev->dev;
	struct aml_audio_controller *actrl = NULL;
	struct aml_tdm *p_tdm = NULL;
	struct tdm_chipinfo *p_chipinfo;
	int ret = 0;

	p_tdm = devm_kzalloc(dev, sizeof(struct aml_tdm), GFP_KERNEL);
	if (!p_tdm)
		return -ENOMEM;

	/* match data */
	p_chipinfo = (struct tdm_chipinfo *)
		of_device_get_match_data(dev);
	if (!p_chipinfo) {
		dev_warn_once(dev, "check whether to update tdm chipinfo\n");
		return -ENOMEM;
	}
	p_tdm->chipinfo = p_chipinfo;
	p_tdm->id = p_chipinfo->id;
	if (!p_chipinfo->lane_cnt)
		p_chipinfo->lane_cnt = LANE_MAX1;

	p_tdm->lane_cnt = p_chipinfo->lane_cnt;
	pr_info("%s, tdm ID = %u, lane_cnt = %d\n", __func__,
			p_tdm->id, p_tdm->lane_cnt);

	/* get audio controller */
	node_prt = of_get_parent(node);
	if (node_prt == NULL)
		return -ENXIO;

	pdev_parent = of_find_device_by_node(node_prt);
	of_node_put(node_prt);
	actrl = (struct aml_audio_controller *)
				platform_get_drvdata(pdev_parent);
	p_tdm->actrl = actrl;

	/* get tdm mclk sel configs */
	ret = of_property_read_u32(node, "dai-tdm-clk-sel",
			&p_tdm->clk_sel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't retrieve dai-tdm-clk-sel\n");
		return -ENXIO;
	}

	/* default no same source */
	if (p_tdm->chipinfo &&
		p_tdm->chipinfo->same_src_fn) {

		ret = of_property_read_u32(node, "samesource_sel",
				&p_tdm->samesource_sel);
		if (ret < 0)
			p_tdm->samesource_sel = -1;
		else {
			p_tdm->samesrc_srcpll = devm_clk_get(&pdev->dev,
				"samesource_srcpll");
			if (IS_ERR(p_tdm->samesrc_srcpll)) {
				dev_err(&pdev->dev,
					"Can't retrieve samesrc_srcpll clock\n");
				return PTR_ERR(p_tdm->samesrc_srcpll);
			}
			p_tdm->samesrc_clk = devm_clk_get(&pdev->dev,
				"samesource_clk");
			if (IS_ERR(p_tdm->samesrc_clk)) {
				dev_err(&pdev->dev,
					"Can't retrieve samesrc_clk clock\n");
				return PTR_ERR(p_tdm->samesrc_clk);
			}
			ret = clk_set_parent(p_tdm->samesrc_clk,
				p_tdm->samesrc_srcpll);
			if (ret) {
				dev_err(dev, "can't set samesource clock\n");
				return ret;
			}
			pr_info("TDM id %d samesource_sel:%d\n",
				p_tdm->id,
				p_tdm->samesource_sel);
		}
	}
	/* default no acodec_adc */
	if (p_tdm->chipinfo &&
		p_tdm->chipinfo->adc_fn) {

		ret = of_property_read_u32(node, "acodec_adc",
				&p_tdm->acodec_adc);
		if (ret < 0)
			p_tdm->acodec_adc = 0;
		else
			pr_info("TDM id %d supports ACODEC_ADC\n", p_tdm->id);
	}

	ret = of_property_read_u32(node, "i2s2hdmi",
			&p_tdm->i2s2hdmitx);
	if (ret < 0)
		p_tdm->i2s2hdmitx = 0;
	else
		pr_info("TDM id %d i2s2hdmi:%d\n",
			p_tdm->id,
			p_tdm->i2s2hdmitx);

	if (p_tdm->id == TDM_LB) {
		ret = of_property_read_u32(node, "lb-src-sel",
				&p_tdm->tdmin_lb_src);
		if (ret < 0 || (p_tdm->tdmin_lb_src > 7)) {
			dev_err(&pdev->dev, "invalid lb-src-sel:%d\n",
				p_tdm->tdmin_lb_src);
			return -EINVAL;
		}
		pr_info("TDM id %d lb-src-sel:%d\n",
			p_tdm->id,
			p_tdm->tdmin_lb_src);
	}

	/* get tdm lanes info. if not, set to default 0 */
	ret = of_parse_tdm_lane_slot_in(node,
			&p_tdm->setting.lane_mask_in);
	if (ret < 0)
		p_tdm->setting.lane_mask_in = 0x0;

	ret = of_parse_tdm_lane_slot_out(node,
			&p_tdm->setting.lane_mask_out);
	if (ret < 0)
		p_tdm->setting.lane_mask_out = 0x1;

	/* get tdm lanes oe info. if not, set to default 0 */
	ret = of_parse_tdm_lane_oe_slot_in(node,
			&p_tdm->setting.lane_oe_mask_in);
	if (ret < 0)
		p_tdm->setting.lane_oe_mask_in = 0x0;

	ret = of_parse_tdm_lane_oe_slot_out(node,
			&p_tdm->setting.lane_oe_mask_out);
	if (ret < 0)
		p_tdm->setting.lane_oe_mask_out = 0x0;

	/* get tdm lanes lb info. if not, set to default 0 */
	ret = of_parse_tdm_lane_lb_slot_in(node,
			&p_tdm->setting.lane_lb_mask_in);
	if (ret < 0)
		p_tdm->setting.lane_lb_mask_in = 0x0;

	dev_info(&pdev->dev,
	    "lane_mask_out = %x, lane_oe_mask_out = %x\n",
	    p_tdm->setting.lane_mask_out,
	    p_tdm->setting.lane_oe_mask_out);

	p_tdm->clk = devm_clk_get(&pdev->dev, "clk_srcpll");
	if (IS_ERR(p_tdm->clk)) {
		dev_err(&pdev->dev, "Can't retrieve mpll2 clock\n");
		return PTR_ERR(p_tdm->clk);
	}

	p_tdm->mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(p_tdm->mclk)) {
		dev_err(&pdev->dev, "Can't retrieve mclk\n");
		return PTR_ERR(p_tdm->mclk);
	}

	ret = clk_set_parent(p_tdm->mclk, p_tdm->clk);
	if (ret) {
		dev_err(dev, "can't set tdm parent clock\n");
		return ret;
	}

	/* clk tree style after SM1, instead of legacy prop */
	p_tdm->mclk2pad = devm_clk_get(&pdev->dev, "mclk_pad");
	if (!IS_ERR(p_tdm->mclk2pad)) {
		ret = clk_set_parent(p_tdm->mclk2pad, p_tdm->mclk);
		if (ret) {
			dev_err(&pdev->dev, "Can't set tdm mclk_pad parent\n");
			return -EINVAL;
		}
		clk_prepare_enable(p_tdm->mclk2pad);
		p_tdm->mclk_pad = -1;
	} else {
		/* mclk pad ctrl */
		ret = of_property_read_u32(node, "mclk_pad",
					   &p_tdm->mclk_pad);
		if (ret < 0) {
			/* No mclk in defalut if chip needs mclk pad mux. */
			p_tdm->mclk_pad = -1;
			dev_warn_once(&pdev->dev,
				      "neither mclk_pad nor mclk2pad set\n");
		}
	}

	if (p_tdm->chipinfo && (!p_tdm->chipinfo->no_mclkpad_ctrl)) {
		ret = aml_tdm_set_clk_pad(p_tdm);
		if (ret)
			dev_warn_once(&pdev->dev, "clk_pad set failed\n");
	}

	/* complete mclk for tdm */
	if (get_meson_cpu_version(MESON_CPU_VERSION_LVL_MINOR) == 0xa)
		meson_clk_measure((1<<16) | 0x67);

	p_tdm->pin_ctl = devm_pinctrl_get_select(dev, "tdm_pins");
	if (IS_ERR(p_tdm->pin_ctl)) {
		dev_info(dev, "aml_tdm_get_pins error!\n");
		/*return PTR_ERR(p_tdm->pin_ctl);*/
	}

	ret = of_property_read_u32(node, "start_clk_enable",
				&p_tdm->start_clk_enable);
	if (ret < 0)
		p_tdm->start_clk_enable = 0;
	else
		pr_info("TDM id %d output clk enable:%d\n",
			p_tdm->id, p_tdm->start_clk_enable);

	/*set default clk for output*/
	if (p_tdm->start_clk_enable == 1)
		aml_set_default_tdm_clk(p_tdm);

	p_tdm->dev = dev;
	/* For debug to disable share buffer */
	p_tdm->en_share = 1;
	pr_info("%s(), share en = %d", __func__, p_tdm->en_share);
	dev_set_drvdata(dev, p_tdm);

	/* spdif same source with i2s */
	parse_samesrc_channel_mask(p_tdm);

	ret = devm_snd_soc_register_component(dev, &aml_tdm_component,
					 &aml_tdm_dai[p_tdm->id], 1);
	if (ret) {
		dev_err(dev, "devm_snd_soc_register_component failed\n");
		return ret;
	}

	ret = of_property_read_u32(node, "clk_tuning_enable",
				&p_tdm->clk_tuning_enable);
	if (ret < 0)
		p_tdm->clk_tuning_enable = 0;
	else
		pr_info("TDM id %d tuning clk enable:%d\n",
			p_tdm->id, p_tdm->clk_tuning_enable);

	return devm_snd_soc_register_platform(dev, &aml_tdm_platform);
}

static int aml_tdm_platform_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct aml_tdm *p_tdm = dev_get_drvdata(&pdev->dev);

	/*mute default clk */
	if (p_tdm->start_clk_enable == 1 && p_tdm->pin_ctl) {
		struct pinctrl_state *ps = NULL;

		ps = pinctrl_lookup_state(p_tdm->pin_ctl, "tdmout_a_gpio");
		if (!IS_ERR_OR_NULL(ps)) {
			pinctrl_select_state(p_tdm->pin_ctl, ps);
			pr_info("%s tdm pins disable!\n", __func__);
		}
	}

	pr_info("%s tdm:(%d)\n", __func__, p_tdm->id);
	return 0;
}

static int aml_tdm_platform_resume(struct platform_device *pdev)
{
	struct aml_tdm *p_tdm = dev_get_drvdata(&pdev->dev);

	/* complete mclk for tdm */
	if (get_meson_cpu_version(MESON_CPU_VERSION_LVL_MINOR) == 0xa)
		meson_clk_measure((1<<16) | 0x67);

	/*set default clk for output*/
	if (p_tdm->start_clk_enable == 1 && p_tdm->pin_ctl) {
		struct pinctrl_state *state = NULL;

		aml_set_default_tdm_clk(p_tdm);
		state = pinctrl_lookup_state(p_tdm->pin_ctl, "tdm_pins");
		if (!IS_ERR_OR_NULL(state)) {
			pinctrl_select_state(p_tdm->pin_ctl, state);
			pr_info("%s tdm pins enable!\n", __func__);
		}
	}

	pr_info("%s tdm:(%d)\n", __func__, p_tdm->id);
	return 0;
}

static void aml_tdm_platform_shutdown(struct platform_device *pdev)
{
}

struct platform_driver aml_tdm_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = aml_tdm_device_id,
	},
	.probe	 = aml_tdm_platform_probe,
	.suspend = aml_tdm_platform_suspend,
	.resume  = aml_tdm_platform_resume,
	.shutdown = aml_tdm_platform_shutdown,
};
module_platform_driver(aml_tdm_driver);

MODULE_AUTHOR("Amlogic, Inc.");
MODULE_DESCRIPTION("Amlogic TDM ASoc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, aml_tdm_device_id);
