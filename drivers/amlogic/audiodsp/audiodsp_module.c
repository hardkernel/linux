/*
 * drivers/amlogic/audiodsp/audiodsp_module.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <linux/amlogic/major.h>
#include <linux/slab.h>
#include <linux/reset.h>
#include <linux/amlogic/sound/aiu_regs.h>

/* #include <asm/dsp/audiodsp_control.h> */
/* #include "audiodsp_control.h" */

#include <linux/uaccess.h>
#include <linux/amlogic/amports/amstream.h>


#include "audiodsp_module.h"
#if 0
#include "dsp_control.h"
#include "dsp_microcode.h"
#include "dsp_mailbox.h"
#include "dsp_monitor.h"
#include "dsp_codec.h"
#endif
#include <linux/dma-mapping.h>
#include <linux/amlogic/amports/ptsserv.h>
#include <linux/amlogic/amports/timestamp.h>
#include <linux/amlogic/amports/tsync.h>

unsigned dsp_debug_flag = 1;

MODULE_DESCRIPTION("AMLOGIC APOLLO Audio dsp driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhou Zhi <zhi.zhou@amlogic.com>");
MODULE_VERSION("1.0.0");
/* static int IEC958_mode_raw_last; */
/* static int IEC958_mode_codec_last; */
static unsigned   audio_samesource = 1;
static int decopt  = 1;
/* code for DD/DD+ DRC control  */
/* Dynamic range compression mode */
enum DDP_DEC_DRC_MODE {
	GBL_COMP_CUSTOM_0 = 0,	/* custom mode, analog  dialnorm */
	GBL_COMP_CUSTOM_1,	/* custom mode, digital dialnorm */
	GBL_COMP_LINE,		/* line out mode (default)       */
	GBL_COMP_RF		/* RF mode                       */
};
#define DRC_MODE_BIT  0
#define DRC_HIGH_CUT_BIT 3
#define DRC_LOW_BST_BIT 16
static unsigned ac3_drc_control =
(GBL_COMP_LINE << DRC_MODE_BIT) |
(100 << DRC_HIGH_CUT_BIT) | (100 << DRC_LOW_BST_BIT);
/* code for DTS dial norm/downmix mode control */
enum DTS_DMX_MODE {
	DTS_DMX_LoRo = 0,
	DTS_DMX_LtRt,
};
#define DTS_DMX_MODE_BIT  0
#define DTS_DIAL_NORM_BIT  1
#define DTS_DRC_SCALE_BIT  2
static unsigned int dts_dec_control =
(DTS_DMX_LoRo << DTS_DMX_MODE_BIT) |
(1 << DTS_DIAL_NORM_BIT) | (0 << DTS_DRC_SCALE_BIT);

static struct audiodsp_priv *audiodsp_p;
#define  DSP_DRIVER_NAME	"audiodsp"
#define  DSP_NAME	"dsp"

/**
 *  Audio codec necessary MIPS (KHz)
 */
#if 0
static unsigned int audiodsp_mips[] = {
	200000,			/* #define MCODEC_FMT_MPEG123 (1<<0) */
	200000,			/* #define MCODEC_FMT_AAC      (1<<1) */
	200000,			/* #define MCODEC_FMT_AC3      (1<<2) */
	200000,			/* #define MCODEC_FMT_DTS              (1<<3) */
	200000,			/* #define MCODEC_FMT_FLAC     (1<<4) */
	200000,			/* #define MCODEC_FMT_COOK           (1<<5) */
	200000,			/* #define MCODEC_FMT_AMR            (1<<6) */
	200000,			/* #define MCODEC_FMT_RAAC     (1<<7) */
	200000,			/* #define MCODEC_FMT_ADPCM    (1<<8) */
	200000,			/* #define MCODEC_FMT_WMA     (1<<9) */
	200000,			/* #define MCODEC_FMT_PCM      (1<<10) */
	200000,			/* #define MCODEC_FMT_WMAPRO      (1<<11) */
};
#endif
#ifdef CONFIG_PM
struct audiodsp_pm_state_t {
	int event;
	/*  */
};

/* static struct audiodsp_pm_state_t pm_state; */

#endif
#if 0
static void audiodsp_prevent_sleep(void)
{
	/*struct audiodsp_priv* priv = */ audiodsp_privdata();
	pr_info("audiodsp prevent sleep\n");
	/* wake_lock(&priv->wakelock); */
}

static void audiodsp_allow_sleep(void)
{
	/*struct audiodsp_priv *priv= */ audiodsp_privdata();
	pr_info("audiodsp allow sleep\n");
	/* wake_unlock(&priv->wakelock); */
}

int audiodsp_start(void)
{
	struct audiodsp_priv *priv = audiodsp_privdata();
	struct audiodsp_microcode *pmcode;
	struct audio_info *audio_info;
	int ret, i;
	priv->frame_format.valid = 0;
	priv->decode_error_count = 0;
	priv->last_valid_pts = 0;
	priv->out_len_after_last_valid_pts = 0;
	priv->decode_fatal_err = 0;
	priv->first_lookup_over = 0;
	pmcode = audiodsp_find_supoort_mcode(priv, priv->stream_fmt);
	if (pmcode == NULL) {
		DSP_PRNT("have not find a valid mcode for fmt(0x%x)\n",
			 priv->stream_fmt);
		return -1;
	}

	stop_audiodsp_monitor(priv);
	dsp_stop(priv);
	ret = dsp_start(priv, pmcode);
	if (ret == 0) {
#if 0
		if (priv->stream_fmt == MCODEC_FMT_DTS
		    || priv->stream_fmt == MCODEC_FMT_AC3
		    || priv->stream_fmt == MCODEC_FMT_EAC3) {
			dsp_get_debug_interface(1);
		}
#endif
		start_audiodsp_monitor(priv);

#ifdef CONFIG_AM_VDEC_REAL
		if ((pmcode->fmt == MCODEC_FMT_COOK) ||
		    (pmcode->fmt == MCODEC_FMT_RAAC) ||
		    (pmcode->fmt == MCODEC_FMT_AMR) ||
		    (pmcode->fmt == MCODEC_FMT_WMA) ||
		    (pmcode->fmt == MCODEC_FMT_ADPCM) ||
		    (pmcode->fmt == MCODEC_FMT_PCM) ||
		    (pmcode->fmt == MCODEC_FMT_WMAPRO) ||
		    (pmcode->fmt == MCODEC_FMT_ALAC) ||
		    (pmcode->fmt & MCODEC_FMT_AC3) ||
		    (pmcode->fmt & MCODEC_FMT_EAC3) ||
		    (pmcode->fmt == MCODEC_FMT_APE) ||
		    (pmcode->fmt == MCODEC_FMT_FLAC)) {
			DSP_PRNT("dsp send audio info\n");
			for (i = 0; i < 2000; i++) {
				/*
				 * maybe at audiodsp side,
				 * INT not enabled yet,so wait a while
				 */
				if (DSP_RD(DSP_AUDIOINFO_STATUS) ==
				    DSP_AUDIOINFO_READY)
					break;
				udelay(1000);
			}
			if (i == 2000)
				DSP_PRNT("audiodsp not ready for info\n");
			DSP_WD(DSP_AUDIOINFO_STATUS, 0);
			audio_info = get_audio_info();
			DSP_PRNT
			    ("kernel sent info[0x%x],[0x%x],[0x%x],[0x%x]\n\t",
			     audio_info->extradata[0], audio_info->extradata[1],
			     audio_info->extradata[2],
			     audio_info->extradata[3]);
			DSP_WD(DSP_GET_EXTRA_INFO_FINISH, 0);
			while (1) {
				dsp_mailbox_send(priv, 1, M2B_IRQ4_AUDIO_INFO,
						 0, (const char *)audio_info,
						 sizeof(struct audio_info));
				msleep(100);

				if (DSP_RD(DSP_GET_EXTRA_INFO_FINISH) ==
				    0x12345678)
					break;
			}
		}
#endif
	}
	return ret;
}

static int audiodsp_open(struct inode *node, struct file *file)
{
	DSP_PRNT("dsp_open\n");

#if 0				/* MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6 */
	switch_mod_gate_by_type(MOD_MEDIA_CPU, 1);
	/* Audio DSP firmware uses mailbox registers for communications
	 * with host processor. And these mailbox registers unfortunately
	 * falls into the assistant module, which is in vdec DOS clock domain.
	 * So we have to make sure the clock domain is enabled/disabled when
	 * audio firmware start/stop running.
	 * Note the module_gating has ref count so a flag 0 does
	 * not mean the vdec clock is gated off immediately.
	 */
	switch_mod_gate_by_type(MOD_VDEC, 1);
#endif
	audiodsp_prevent_sleep();
	return 0;

}

static unsigned long audiodsp_drop_pcm(unsigned long size)
{
	struct audiodsp_priv *priv = audiodsp_privdata();
	size_t len;
	int count = 0;
	unsigned long drop_bytes = size;

	mutex_lock(&priv->stream_buffer_mutex);
	if (priv->stream_buffer_mem == NULL || !priv->dsp_is_started)
		goto err;

	while (drop_bytes > 0) {
		len = dsp_codec_get_bufer_data_len(priv);
		if (drop_bytes >= len) {
			dsp_codec_inc_rd_addr(priv, len);
			drop_bytes -= len;
			msleep(50);
			count++;
			if (count > 20)
				break;
		} else {
			dsp_codec_inc_rd_addr(priv, drop_bytes);
			drop_bytes = 0;
		}
	}

	mutex_unlock(&priv->stream_buffer_mutex);
	if (count > 10)
		pr_info("drop pcm data timeout! count = %d\n", count);

	return size - drop_bytes;

 err:
	mutex_unlock(&priv->stream_buffer_mutex);
	pr_info("error, can not drop pcm data!\n");
	return 0;
}

static long audiodsp_ioctl(struct file *file, unsigned int cmd,
			   unsigned long args)
{
	struct audiodsp_priv *priv = audiodsp_privdata();
	struct audiodsp_cmd *a_cmd;
	char name[64];
	int len;
	unsigned long pts;
	int ret = 0;
	unsigned long drop_size;
	static int wait_format_times;
	switch (cmd) {
	case AUDIODSP_SET_FMT:
		priv->stream_fmt = args;
		if (IEC958_mode_raw) {	/* raw data pass through */
			if (args == MCODEC_FMT_DTS)
				/* dts PCM/RAW mode */
				IEC958_mode_codec = ((decopt >> 5) & 1) ? 3 : 1;
			else if (args == MCODEC_FMT_AC3)
				IEC958_mode_codec = 2;	/* ac3 */
			else if (args == MCODEC_FMT_EAC3) {
				if (IEC958_mode_raw == 2)
					/* 958 dd+ package */
					IEC958_mode_codec = 4;
				else
					/* 958 dd package */
					IEC958_mode_codec = 2;
			} else
				IEC958_mode_codec = 0;
		} else
			IEC958_mode_codec = 0;

		/* for dd+ certification */
		if (args == MCODEC_FMT_AC3 || args == MCODEC_FMT_EAC3)
			DSP_WD(DSP_AC3_DRC_INFO, ac3_drc_control | (1 << 31));
		else if (args == MCODEC_FMT_DTS)
			DSP_WD(DSP_DTS_DEC_INFO, dts_dec_control | (1 << 31));
		break;
	case AUDIODSP_START:
		if (IEC958_mode_codec
		    || (IEC958_mode_codec_last != IEC958_mode_codec)) {
			IEC958_mode_raw_last = IEC958_mode_raw;
			IEC958_mode_codec_last = IEC958_mode_codec;
			aml_alsa_hw_reprepare();
		}
		priv->decoded_nb_frames = 0;
		priv->format_wait_count = 0;
		if (priv->stream_fmt <= 0) {
			DSP_PRNT("Audio dsp steam format have not set!\n");
		} else {
#if 0
			if (priv->stream_fmt == MCODEC_FMT_DTS
			    || priv->stream_fmt == MCODEC_FMT_AC3
			    || priv->stream_fmt == MCODEC_FMT_EAC3)
				dsp_get_debug_interface(0);
#endif
			ret = audiodsp_start();
		}
		break;
	case AUDIODSP_STOP:
		/* restore aiu958 setting to pcm */
		if (IEC958_mode_codec_last)
			aml_alsa_hw_reprepare();
		IEC958_mode_codec = 0;
		/* DSP_PRNT("audiodsp command stop\n"); */
		stop_audiodsp_monitor(priv);
		dsp_stop(priv);
		priv->decoded_nb_frames = 0;
		priv->format_wait_count = 0;
		break;
	case AUDIODSP_DECODE_START:
		if (priv->dsp_is_started) {
			dsp_codec_start(priv);
			wait_format_times = 0;
		} else {
			DSP_PRNT("Audio dsp have not started\n");
		}
		break;
	case AUDIODSP_WAIT_FORMAT:
		if (priv->dsp_is_started) {
			ret = audiodsp_get_audioinfo(priv);
		} else {
			ret = -1;
			DSP_PRNT("Audio dsp have not started\n");
		}
		/*Reset the PLL. Added by GK */
		tsync_pcr_recover();
		break;
	case AUDIODSP_DECODE_STOP:
		if (priv->dsp_is_started)
			dsp_codec_stop(priv);
		else
			DSP_PRNT("Audio dsp have not started\n");

		break;
	case AUDIODSP_REGISTER_FIRMWARE:
		a_cmd = (struct audiodsp_cmd *)args;
		/*
		 * DSP_PRNT("register firware,%d,%s\n",a_cmd->fmt,a_cmd->data);
		 */
		len = a_cmd->data_len > 64 ? 64 : a_cmd->data_len;
		if (copy_from_user(name, a_cmd->data, len))
			return -EFAULT;

		name[len] = '\0';
		ret = audiodsp_microcode_register(priv, a_cmd->fmt, name);
		break;
	case AUDIODSP_UNREGISTER_ALLFIRMWARE:
		audiodsp_microcode_free(priv);
		break;
	case AUDIODSP_GET_CHANNELS_NUM:
		/*mask data is not valid */
		put_user(-1, (__s32 __user *) args);
		if (priv->frame_format.valid & CHANNEL_VALID)
			put_user(priv->frame_format.channel_num,
				 (__s32 __user *) args);
		break;
	case AUDIODSP_GET_SAMPLERATE:
		/*mask data is not valid */
		put_user(-1, (__s32 __user *) args);
		if (priv->frame_format.valid & SAMPLE_RATE_VALID)
			put_user(priv->frame_format.sample_rate,
				 (__s32 __user *) args);
		break;
	case AUDIODSP_GET_DECODED_NB_FRAMES:
		put_user(priv->decoded_nb_frames, (__s32 __user *) args);
		break;
	case AUDIODSP_GET_BITS_PER_SAMPLE:
		/*mask data is not valid */
		put_user(-1, (__s32 __user *) args);
		if (priv->frame_format.valid & DATA_WIDTH_VALID) {
			put_user(priv->frame_format.data_width,
				 (__s32 __user *) args);
		}
		break;
	case AUDIODSP_GET_PTS:
		/*val=-1 is not valid */
		put_user(dsp_codec_get_current_pts(priv),
			 (__u32 __user *) args);
		break;
	case AUDIODSP_LOOKUP_APTS:
		{
			u32 pts, offset;
			get_user(offset, (__u32 __user *) args);
			pts_lookup_offset(PTS_TYPE_AUDIO, offset, &pts, 300);
			put_user(pts, (__u32 __user *) args);
		}
		break;
	case AUDIODSP_GET_FIRST_PTS_FLAG:
		if (priv->stream_fmt == MCODEC_FMT_COOK
		    || priv->stream_fmt == MCODEC_FMT_RAAC)
			put_user(1, (__s32 __user *) args);
		else
			put_user(first_pts_checkin_complete(PTS_TYPE_AUDIO),
				 (__s32 __user *) args);
		break;

	case AUDIODSP_SYNC_AUDIO_START:

		if (get_user(pts, (unsigned long __user *)args)) {
			pr_info("Get start pts from user space fault!\n");
			return -EFAULT;
		}
		tsync_avevent(AUDIO_START, pts);

		break;

	case AUDIODSP_SYNC_AUDIO_PAUSE:

		tsync_avevent(AUDIO_PAUSE, 0);
		break;

	case AUDIODSP_SYNC_AUDIO_RESUME:

		tsync_avevent(AUDIO_RESUME, 0);
		break;

	case AUDIODSP_SYNC_AUDIO_TSTAMP_DISCONTINUITY:

		if (get_user(pts, (unsigned long __user *)args)) {
			pr_info("Get audio discontinuity pts fault!\n");
			return -EFAULT;
		}
		tsync_avevent(AUDIO_TSTAMP_DISCONTINUITY, pts);

		break;

	case AUDIODSP_SYNC_GET_APTS:

		pts = timestamp_apts_get();

		if (put_user(pts, (unsigned long __user *)args)) {
			pr_info("Put audio pts to user space fault!\n");
			return -EFAULT;
		}

		break;

	case AUDIODSP_SYNC_GET_PCRSCR:

		pts = timestamp_pcrscr_get();

		if (put_user(pts, (unsigned long __user *)args)) {
			pr_info("Put pcrscr to user space fault!\n");
			return -EFAULT;
		}

		break;

	case AUDIODSP_SYNC_SET_APTS:

		if (get_user(pts, (unsigned long __user *)args)) {
			pr_info("Get audio pts from user space fault!\n");
			return -EFAULT;
		}
		tsync_set_apts(pts);

		break;

	case AUDIODSP_DROP_PCMDATA:

		if (get_user(drop_size, (unsigned long __user *)args)) {
			pr_info("Get pcm drop size from user space fault!\n");
			return -EFAULT;
		}
		audiodsp_drop_pcm(drop_size);
		break;

	case AUDIODSP_AUTOMUTE_ON:
		tsync_set_automute_on(1);
		break;

	case AUDIODSP_AUTOMUTE_OFF:
		tsync_set_automute_on(0);
		break;

	case AUDIODSP_GET_PCM_LEVEL:
		{
			int len = dsp_codec_get_bufer_data_len(priv);
			if (put_user(len, (unsigned long __user *)args)) {
				pr_info("Put pcm level to user space fault!\n");
				return -EFAULT;
			}
			break;
		}

	case AUDIODSP_SET_PCM_BUF_SIZE:
		if ((int)args > 0)
			priv->stream_buffer_mem_size = args;
		break;
	case AUDIODSP_SKIP_BYTES:
		DSP_WD(DSP_SKIP_BYTES, (unsigned int)args);
		break;
	default:
		DSP_PRNT("unsupport cmd number%d\n", cmd);
		ret = -1;
	}
	return ret;
}

static int audiodsp_release(struct inode *node, struct file *file)
{
	DSP_PRNT("dsp_release\n");
#if 0				/* MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6 */
	switch_mod_gate_by_type(MOD_VDEC, 0);
	switch_mod_gate_by_type(MOD_MEDIA_CPU, 0);
#endif

	audiodsp_allow_sleep();
	return 0;
}

ssize_t audiodsp_read(struct file *file, char __user *ubuf, size_t size,
		      loff_t *loff)
{
	struct audiodsp_priv *priv = audiodsp_privdata();
	unsigned long rp, orp;
	size_t len;
	size_t else_len;
	size_t wlen;
	size_t w_else_len;
	int wait = 0;
	char __user *pubuf = ubuf;
	dma_addr_t buf_map;

#define MIN_READ	2	/* 1 channel * 2 bytes per sample */
#define PCM_DATA_MIN	2
#define PCM_DATA_ALGIN(x) (x & (~(PCM_DATA_MIN-1)))
#define MAX_WAIT		(HZ/1000)

	mutex_lock(&priv->stream_buffer_mutex);
	if (priv->stream_buffer_mem == NULL || !priv->dsp_is_started)
		goto error_out;

	do {
		len = dsp_codec_get_bufer_data_len(priv);
		if (len > MIN_READ)
			break;
		else {
			if (wait > 0)
				break;
			wait++;
			init_completion(&priv->decode_completion);
			wait_for_completion_timeout(&priv->decode_completion,
						    MAX_WAIT);
		}
	} while (len < MIN_READ);
	if (len > priv->stream_buffer_size || len < 0) {
		DSP_PRNT("audio stream buffer is bad len=%d\n", len);
		goto error_out;
	}
	len = min(len, size);
	len = PCM_DATA_ALGIN(len);
	else_len = len;
	rp = dsp_codec_get_rd_addr(priv);
	orp = rp;
	while (else_len > 0) {

		wlen = priv->stream_buffer_end - rp;
		wlen = min(wlen, else_len);
		/* dma_cache_inv((unsigned long)rp,wlen); */
		buf_map =
		    dma_map_single(NULL, (void *)rp, wlen, DMA_FROM_DEVICE);
		w_else_len =
		    copy_to_user((void *)pubuf, (const char *)(rp), wlen);
		if (w_else_len != 0) {
			DSP_PRNT("copyed error,%d,%d,[%p]<---[%lx]\n",
				 w_else_len, wlen, pubuf, rp);
			wlen -= w_else_len;
		}
		dma_unmap_single(NULL, buf_map, wlen, DMA_FROM_DEVICE);
		else_len -= wlen;
		pubuf += wlen;
		rp = dsp_codec_inc_rd_addr(priv, wlen);
	}
	priv->out_len_after_last_valid_pts += len;
	mutex_unlock(&priv->stream_buffer_mutex);
	/* u32 timestamp_pcrscr_get(void); */
	/*
	 *pr_info("current pts=%ld,src=%ld\n",
	 *dsp_codec_get_current_pts(priv),timestamp_pcrscr_get());
	 */
	return len;
 error_out:
	mutex_unlock(&priv->stream_buffer_mutex);
	pr_info("audiodsp_read failed\n");
	return -EINVAL;
}

ssize_t audiodsp_write(struct file *file, const char __user *ubuf,
		       size_t size, loff_t *loff)
{
	struct audiodsp_priv *priv = audiodsp_privdata();
	/* int dsp_codec_start( struct audiodsp_priv *priv); */
	/* int dsp_codec_stop( struct audiodsp_priv *priv); */
	audiodsp_microcode_register(priv,
				    MCODEC_FMT_COOK, "audiodsp_codec_cook.bin");
	priv->stream_fmt = MCODEC_FMT_COOK;
	audiodsp_start();
	dsp_codec_start(priv);
	/* dsp_codec_stop(priv); */

	return size;
}

static const struct file_operations audiodsp_fops = {
	.owner = THIS_MODULE,
	.open = audiodsp_open,
	.read = audiodsp_read,
	.write = audiodsp_write,
	.release = audiodsp_release,
	.unlocked_ioctl = audiodsp_ioctl,
};

static int audiodsp_get_status(struct adec_status *astatus)
{
	struct audiodsp_priv *priv = audiodsp_privdata();
	if (!astatus)
		return -EINVAL;
	if (priv->frame_format.valid & CHANNEL_VALID)
		astatus->channels = priv->frame_format.channel_num;
	else
		astatus->channels = 0;
	if (priv->frame_format.valid & SAMPLE_RATE_VALID)
		astatus->sample_rate = priv->frame_format.sample_rate;
	else
		astatus->sample_rate = 0;
	if (priv->frame_format.valid & DATA_WIDTH_VALID)
		astatus->resolution = priv->frame_format.data_width;
	else
		astatus->resolution = 0;
	astatus->error_count = priv->decode_error_count;
	astatus->status = priv->dsp_is_started ? 0 : 1;
	return 0;
}

static int audiodsp_init_mcode(struct audiodsp_priv *priv)
{
	spin_lock_init(&priv->mcode_lock);
	priv->mcode_id = 0;
	priv->dsp_stack_start = 0;
	priv->dsp_gstack_start = 0;
	priv->dsp_heap_start = 0;
	priv->code_mem_size = AUDIO_DSP_MEM_SIZE - REG_MEM_SIZE;
	priv->dsp_code_start = AUDIO_DSP_START_ADDR;
	DSP_PRNT("DSP start addr 0x%x\n", AUDIO_DSP_START_ADDR);
	priv->dsp_stack_size = 1024 * 64;
	priv->dsp_gstack_size = 512;
	priv->dsp_heap_size = 1024 * 1024;
	priv->stream_buffer_mem = NULL;
	priv->stream_buffer_mem_size = 32 * 1024;
	priv->stream_fmt = -1;
	INIT_LIST_HEAD(&priv->mcode_list);
	init_completion(&priv->decode_completion);
	mutex_init(&priv->stream_buffer_mutex);
	mutex_init(&priv->dsp_mutex);
	priv->last_stream_fmt = -1;
	priv->last_valid_pts = 0;
	priv->out_len_after_last_valid_pts = 0;
	priv->dsp_work_details = (struct dsp_working_info *)DSP_WORK_INFO;
	return 0;
}
#endif
static ssize_t codec_fmt_show(struct class *cla, struct class_attribute *attr,
			      char *buf)
{
	size_t ret = 0;
	struct audiodsp_priv *priv = audiodsp_privdata();
	ret = sprintf(buf, "The codec Format %d\n", priv->stream_fmt);
	return ret;
}

#if 0
static ssize_t codec_mips_show(struct class *cla, struct class_attribute *attr,
			       char *buf)
{
	size_t ret = 0;
	struct audiodsp_priv *priv = audiodsp_privdata();
	if (priv->stream_fmt < sizeof(audiodsp_mips)) {
		ret =
		    sprintf(buf, "%d\n",
			    audiodsp_mips[__builtin_ffs(priv->stream_fmt)]);
	} else {
		ret = sprintf(buf, "%d\n", 200000);
	}
	return ret;
}
#endif
static const struct file_operations audiodsp_fops = {
.owner = THIS_MODULE,
.open = NULL,
.read = NULL,
.write = NULL,
.release = NULL,
.unlocked_ioctl = NULL,
};

static ssize_t codec_fatal_err_show(struct class *cla,
				    struct class_attribute *attr, char *buf)
{
	struct audiodsp_priv *priv = audiodsp_privdata();

	return sprintf(buf, "%d\n", priv->decode_fatal_err);
}

static ssize_t codec_fatal_err_store(struct class *cla,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	struct audiodsp_priv *priv = audiodsp_privdata();

	if (buf[0] == '0')
		priv->decode_fatal_err = 0;
	else if (buf[0] == '1')
		priv->decode_fatal_err = 1;
	else if (buf[0] == '2')
		priv->decode_fatal_err = 2;

	pr_info("codec_fatal_err value:%d\n ", priv->decode_fatal_err);
	return count;
}

static ssize_t digital_raw_show(struct class *cla, struct class_attribute *attr,
				char *buf)
{
	char *pbuf = buf;
	pbuf += sprintf(pbuf, "%d\n", IEC958_mode_raw);
	return pbuf - buf;
}

static ssize_t digital_raw_store(struct class *class,
				 struct class_attribute *attr, const char *buf,
				 size_t count)
{
	pr_info("buf=%s\n", buf);
	if (buf[0] == '0')
		IEC958_mode_raw = 0;	/* PCM */
	else if (buf[0] == '1')
		IEC958_mode_raw = 1;	/* RAW without over clock */
	else if (buf[0] == '2')
		IEC958_mode_raw = 2;	/* RAW with over clock */

	pr_info("IEC958_mode_raw=%d\n", IEC958_mode_raw);
	return count;
}

#define SUPPORT_TYPE_NUM  9
static unsigned char *codec_str[SUPPORT_TYPE_NUM] = {
	"2 CH PCM", "DTS RAW Mode", "Dolby Digital", "DTS",
	"DD+", "DTSHD", "8 CH PCM", "TrueHD", "DTSLL"
};

static ssize_t digital_codec_show(struct class *cla,
				  struct class_attribute *attr, char *buf)
{
	char *pbuf = buf;

	pbuf += sprintf(pbuf, "%d\n", IEC958_mode_codec);
	return pbuf - buf;
}

static ssize_t digital_codec_store(struct class *class,
				   struct class_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned int digital_codec = 0;
	unsigned mode_codec = IEC958_mode_codec;
	if (buf) {
		if (kstrtoint(buf, 10, &digital_codec))
			pr_info("kstrtoint err %s\n", __func__);
		if (digital_codec == 5) {
		    digital_codec = 8;
		    pr_info("fixed IEC958 type %s to %s\n", codec_str[5], codec_str[digital_codec]);
		}
		if (digital_codec < SUPPORT_TYPE_NUM) {
			IEC958_mode_codec = digital_codec;
			pr_info("IEC958_mode_codec= %d, IEC958 type %s\n",
				digital_codec, codec_str[digital_codec]);
		} else {
			pr_info("IEC958 type set exceed supported range\n");
		}
	}
	/*
	 * when raw output switch to pcm output,
	 * need trigger pcm hw prepare to re-init hw configuration
	 */
	pr_info("last mode %d,now %d\n", mode_codec, IEC958_mode_codec);

	return count;
}

/*
code to force enable none-samesource,
to trigger alsa reset audio hw,only once available
*/
static ssize_t audio_samesource_store(struct class *class,
				      struct class_attribute *attr,
				      const char *buf, size_t count)
{
	if (kstrtoint(buf, 16, (unsigned int *)&audio_samesource)) {
		pr_info("audio_samesource_store failed\n");
		return -EINVAL;
	}
	pr_info("same source set to %d\n", audio_samesource);
	return count;

}

static ssize_t audio_samesource_show(struct class *cla,
				     struct class_attribute *attr, char *buf)
{
	char *pbuf = buf;
	int samesource = 0;
	int i2s_enable = !!(aml_read_cbus(AIU_MEM_I2S_CONTROL) & (3 << 1));
	int iec958_enable =
	    !!(aml_read_cbus(AIU_MEM_IEC958_CONTROL) & (3 << 1));
	samesource =
	    (aml_read_cbus(AIU_MEM_IEC958_START_PTR) ==
	     aml_read_cbus(AIU_MEM_I2S_START_PTR));

	/* make sure i2s/958 same source.and both enabled */
	if (samesource == 0) {
		if ((i2s_enable && iec958_enable) || !audio_samesource) {
			samesource = 2;
			if (audio_samesource == 0)
				audio_samesource = 1;
		}
	}
	pbuf += sprintf(pbuf, "%d\n", samesource);
	return pbuf - buf;
}

static ssize_t print_flag_show(struct class *cla,
			       struct class_attribute *attr, char *buf)
{
	static char *dec_format[] = {
		"0 - disable arc dsp print",
		"1 - enable arc dsp print",
	};
	char *pbuf = buf;
	pbuf +=
	    sprintf(pbuf, "audiodsp decode option: %s\n",
		    dec_format[(decopt & 0x5) >> 2]);
	return pbuf - buf;
}

static ssize_t print_flag_store(struct class *class,
				struct class_attribute *attr, const char *buf,
				size_t count)
{
	unsigned dec_opt = 0x1;
	pr_info("buf=%s\n", buf);
	if (buf[0] == '0')
		dec_opt = 0;	/* disable print flag */
	else if (buf[0] == '1')
		dec_opt = 1;	/* enable print flag */

	decopt = (decopt & (~4)) | (dec_opt << 2);
	pr_info("dec option=%d, decopt = %x\n", dec_opt, decopt);
	return count;
}

static ssize_t dec_option_show(struct class *cla, struct class_attribute *attr,
			       char *buf)
{
	static char *dec_format[] = {
		"0 - mute dts and ac3 ",
		"1 - mute dts.ac3 with noise ",
		"2 - mute ac3.dts with noise",
		"3 - both ac3 and dts with noise",
	};
	char *pbuf = buf;
	pbuf +=
	    sprintf(pbuf, "audiodsp decode option: %s\n",
		    dec_format[decopt & 0x3]);
	return pbuf - buf;
}

static ssize_t dec_option_store(struct class *class,
				struct class_attribute *attr, const char *buf,
				size_t count)
{
	unsigned dec_opt = 0x3;
	pr_info("buf=%s\n", buf);
	if (buf[0] == '0') {
		dec_opt = 0;	/* mute ac3/dts */
	} else if (buf[0] == '1') {
		dec_opt = 1;	/* mute dts */
	} else if (buf[0] == '2') {
		dec_opt = 2;	/* mute ac3 */
	} else if (buf[0] == '3') {
		dec_opt = 3;	/* with noise */
	} else if (buf[0] == '4') {
		pr_info("digital mode :PCM-raw\n");
		decopt |= (1 << 5);
	} else if (buf[0] == '5') {
		pr_info("digital mode :raw\n");
		decopt &= ~(1 << 5);
	}
	decopt = (decopt & (~3)) | dec_opt;
	pr_info("dec option=%d\n", dec_opt);
	return count;
}

static ssize_t ac3_drc_control_show(struct class *cla,
				    struct class_attribute *attr, char *buf)
{
	char *drcmode[] = { "CUSTOM_0", "CUSTOM_1", "LINE", "RF" };
	char *pbuf = buf;
#if 0
	pbuf +=
	    sprintf(pbuf, "\tdd+ drc mode : %s\n",
		    drcmode[ac3_drc_control & 0x3]);
	pbuf +=
	    sprintf(pbuf, "\tdd+ drc high cut scale : %d%%\n",
		    (ac3_drc_control >> DRC_HIGH_CUT_BIT) & 0xff);
	pbuf +=
	    sprintf(pbuf, "\tdd+ drc low boost scale : %d%%\n",
		    (ac3_drc_control >> DRC_LOW_BST_BIT) & 0xff);
#else
	pr_info("\tdd+ drc mode : %s\n", drcmode[ac3_drc_control & 0x3]);
	pr_info("\tdd+ drc high cut scale : %d%%\n",
		(ac3_drc_control >> DRC_HIGH_CUT_BIT) & 0xff);
	pr_info("\tdd+ drc low boost scale : %d%%\n",
		(ac3_drc_control >> DRC_LOW_BST_BIT) & 0xff);
	pbuf += sprintf(pbuf, "%d\n", ac3_drc_control);
#endif
	return pbuf - buf;
}

static ssize_t ac3_drc_control_store(struct class *class,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	char tmpbuf[128];
	char *drcmode[] = { "CUSTOM_0", "CUSTOM_1", "LINE", "RF" };
	int i = 0;
	unsigned int val;
	while ((buf[i]) && (buf[i] != ',') && (buf[i] != ' ')) {
		tmpbuf[i] = buf[i];
		i++;
	}
	tmpbuf[i] = 0;
	if (strncmp(tmpbuf, "drcmode", 7) == 0) {
		if (kstrtoint(buf + i + 1, 16, &val))
			pr_info("kstrtoint err %s\n", __func__);
		val = val & 0x3;
		pr_info("drc mode set to %s\n", drcmode[val]);
		ac3_drc_control = (ac3_drc_control & (~3)) | val;
	} else if (strncmp(tmpbuf, "drchighcutscale", 15) == 0) {
		if (kstrtoint(buf + i + 1, 16, &val))
			pr_info("kstrtoint err %s\n", __func__);
		val = val & 0xff;
		pr_info("drc high cut scale set to %d%%\n", val);
		ac3_drc_control =
		    (ac3_drc_control & (~(0xff << DRC_HIGH_CUT_BIT))) |
		    (val << DRC_HIGH_CUT_BIT);
	} else if (strncmp(tmpbuf, "drclowboostscale", 16) == 0) {
		if (kstrtoint(buf + i + 1, 16, &val))
			pr_info("kstrtoint err %s\n", __func__);
		val = val & 0xff;
		pr_info("drc low boost scale set to %d%%\n", val);
		ac3_drc_control =
		    (ac3_drc_control & (~(0xff << DRC_LOW_BST_BIT))) |
		    (val << DRC_LOW_BST_BIT);
	} else
		pr_info("invalid args\n");
	return count;
}

static ssize_t dts_dec_control_show(struct class *cla,
				    struct class_attribute *attr, char *buf)
{
	/* char *dmxmode[] = {"Lo/Ro","Lt/Rt"}; */
	/* char *dialnorm[] = {"disable","enable"}; */
	char *pbuf = buf;
	pbuf += sprintf(pbuf, "%d\n", dts_dec_control);
	return pbuf - buf;
}

static ssize_t dts_dec_control_store(struct class *class,
				     struct class_attribute *attr,
				     const char *buf, size_t count)
{
	char tmpbuf[128];
	char *dmxmode[] = { "Lo/Ro", "Lt/Rt" };
	char *dialnorm[] = { "disable", "enable" };
	int i = 0;
	unsigned int val;

	while ((buf[i]) && (buf[i] != ',') && (buf[i] != ' ')) {
		tmpbuf[i] = buf[i];
		i++;
	}
	tmpbuf[i] = 0;
	if (strncmp(tmpbuf, "dtsdmxmode", 10) == 0) {
		if (kstrtoint(buf + i + 1, 16, &val))
			pr_info("kstrtoint err %s\n", __func__);
		val = val & 0x1;
		pr_info("dts dmx mode set to %s\n", dmxmode[val]);
		dts_dec_control = (dts_dec_control & (~1)) | val;
	} else if (strncmp(tmpbuf, "dtsdrcscale", 11) == 0) {
		if (kstrtoint(buf + i + 1, 16, &val))
			pr_info("kstrtoint err %s\n", __func__);
		val = val & 0xff;
		pr_info("dts drc  scale set to %d\n", val);
		dts_dec_control =
		    (dts_dec_control & (~(0xff << DTS_DRC_SCALE_BIT))) |
		    (val << DTS_DRC_SCALE_BIT);
	} else if (strncmp(tmpbuf, "dtsdialnorm", 11) == 0) {
		if (kstrtoint(buf + i + 1, 16, &val))
			pr_info("kstrtoint err %s\n", __func__);
		val = val & 0x1;
		pr_info("dts  dial norm : set to %s\n", dialnorm[val]);
		dts_dec_control =
		    (dts_dec_control & (~(0x1 << DTS_DIAL_NORM_BIT))) |
		    (val << DTS_DIAL_NORM_BIT);
	} else {
		if (kstrtoint(tmpbuf, 16, &dts_dec_control))
			pr_info("kstrtoint err %s\n", __func__);
		pr_info("dts_dec_control/0x%x\n", dts_dec_control);
	}
	return count;

}

static ssize_t dsp_debug_show(struct class *cla, struct class_attribute *attr,
			      char *buf)
{
	char *pbuf = buf;
	pbuf +=
	    sprintf(pbuf, "DSP Debug Flag: %s\n",
		    (dsp_debug_flag == 0) ? "0 - OFF" : "1 - ON");
	return pbuf - buf;
}

static ssize_t dsp_debug_store(struct class *class,
			       struct class_attribute *attr, const char *buf,
			       size_t count)
{
	if (buf[0] == '0')
		dsp_debug_flag = 0;
	else if (buf[0] == '1')
		dsp_debug_flag = 1;

	pr_info("DSP Debug Flag: %d\n", dsp_debug_flag);
	return count;
}
#if 0
static ssize_t skip_rawbytes_show(struct class *cla,
				  struct class_attribute *attr, char *buf)
{
	unsigned long bytes = DSP_RD(DSP_SKIP_BYTES);
	return sprintf(buf, "%ld\n", bytes);
}

static ssize_t skip_rawbytes_store(struct class *class,
				   struct class_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned int bytes = 0;
	if (kstrtoint(buf, 16, &bytes))
		return count;

	DSP_WD(DSP_SKIP_BYTES, bytes);
	pr_info("audio stream SKIP when ablevel>0x%x\n", bytes);
	return count;
}
#endif
#if 0
static ssize_t pcm_left_len_show(struct class *cla,
				 struct class_attribute *attr, char *buf)
{

	struct audiodsp_priv *priv = audiodsp_privdata();
	int len = dsp_codec_get_bufer_data_len(priv);
	return sprintf(buf, "%d\n", len);
}
#endif
static struct class_attribute audiodsp_attrs[] = {
	__ATTR_RO(codec_fmt),
#ifdef CONFIG_ARCH_MESON1
	__ATTR_RO(codec_mips),
#endif
	__ATTR(codec_fatal_err, S_IRUGO | S_IWUSR | S_IWGRP,
	       codec_fatal_err_show, codec_fatal_err_store),
	/* __ATTR_RO(swap_buf_ptr), */
	/* __ATTR_RO(dsp_working_status), */
	__ATTR(digital_raw, 0666, digital_raw_show,
	       digital_raw_store),
	__ATTR(digital_codec, S_IRUGO | S_IWUSR | S_IWGRP, digital_codec_show,
	       digital_codec_store),
	__ATTR(dec_option, S_IRUGO | S_IWUSR, dec_option_show,
	       dec_option_store),
	__ATTR(print_flag, S_IRUGO | S_IWUSR, print_flag_show,
	       print_flag_store),
	__ATTR(ac3_drc_control, S_IRUGO | S_IWUSR | S_IWGRP,
	       ac3_drc_control_show, ac3_drc_control_store),
	__ATTR(dsp_debug, S_IRUGO | S_IWUSR, dsp_debug_show, dsp_debug_store),
	__ATTR(dts_dec_control, S_IRUGO | S_IWUSR, dts_dec_control_show,
	       dts_dec_control_store),
	/* __ATTR(skip_rawbytes, S_IRUGO | S_IWUSR, skip_rawbytes_show, */
	/*       skip_rawbytes_store), */
	/* __ATTR_RO(pcm_left_len),   */
	__ATTR(audio_samesource, S_IRUGO | S_IWUSR, audio_samesource_show,
	       audio_samesource_store),
	__ATTR_NULL
};
#if 0
#ifdef CONFIG_PM
static int audiodsp_suspend(struct device *dev, pm_message_t state)
{
	/* struct audiodsp_priv *priv = */ audiodsp_privdata();
#if 0
	if (wake_lock_active(&priv->wakelock))
		return -1;	/* please stop dsp first */
#endif
	pm_state.event = state.event;
	if (state.event == PM_EVENT_SUSPEND) {
		/* should sleep cpu2 here after RevC chip */
		/* msleep(50); */
	}
	pr_info("audiodsp suspend\n");
	return 0;
}

static int audiodsp_resume(struct device *dev)
{
	if (pm_state.event == PM_EVENT_SUSPEND)
		/* wakeup cpu2 */
		pm_state.event = -1;

	pr_info("audiodsp resumed\n");
	return 0;
}
#endif
#endif
static struct class audiodsp_class = {
	.name = DSP_DRIVER_NAME,
	.class_attrs = audiodsp_attrs,
#if 0
	.suspend = audiodsp_suspend,
	.resume = audiodsp_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
};

int audiodsp_probe(void)
{
	int res = 0;
	struct audiodsp_priv *priv;
	dsp_debug_flag = 1;
	priv = kmalloc(sizeof(struct audiodsp_priv), GFP_KERNEL);
	if (priv == NULL) {
		DSP_PRNT("Out of memory for audiodsp register\n");
		return -1;
	}
	memset(priv, 0, sizeof(struct audiodsp_priv));
	priv->dsp_is_started = 0;
	/*
	   priv->p = ioremap_nocache(AUDIO_DSP_START_PHY_ADDR, S_1M);
	   if(priv->p)
	   DSP_PRNT("DSP IOREMAP to addr 0x%x\n",(unsigned)priv->p);
	   else{
	   DSP_PRNT("DSP IOREMAP error\n");
	   goto error1;
	   }
	 */
	audiodsp_p = priv;
#if 0
	audiodsp_init_mcode(priv);
#endif
	if (priv->dsp_heap_size) {
		if (priv->dsp_heap_start == 0)
			priv->dsp_heap_start =
			    (unsigned long)kmalloc(priv->dsp_heap_size,
						   GFP_KERNEL);
		if (priv->dsp_heap_start == 0) {
			DSP_PRNT("no memory for audio dsp dsp_set_heap\n");
			kfree(priv);
			return -ENOMEM;
		}
	}
	res = register_chrdev(AUDIODSP_MAJOR, DSP_NAME, &audiodsp_fops);
	if (res < 0) {
		DSP_PRNT("Can't register  char devie for " DSP_NAME "\n");
		goto error1;
	} else
		DSP_PRNT("register " DSP_NAME " to char divece(%d)\n",
			 AUDIODSP_MAJOR);

	res = class_register(&audiodsp_class);
	if (res < 0) {
		DSP_PRNT("Create audiodsp class failed\n");
		res = -EEXIST;
		goto error2;
	}

	priv->class = &audiodsp_class;
	priv->dev = device_create(priv->class,
				  NULL, MKDEV(AUDIODSP_MAJOR, 0),
				  NULL, "audiodsp0");
	if (priv->dev == NULL) {
		res = -EEXIST;
		goto error3;
	}
#if 0
	audiodsp_init_mailbox(priv);
	init_audiodsp_monitor(priv);
#endif
	wake_lock_init(&priv->wakelock, WAKE_LOCK_SUSPEND, "audiodsp");
#ifdef CONFIG_AM_STREAMING
	/* set_adec_func(audiodsp_get_status); */
#endif
	/*memset((void *)DSP_REG_OFFSET, 0, REG_MEM_SIZE); */
#if 0
	dsp_get_debug_interface(0);
#endif

	return res;

	device_destroy(priv->class, MKDEV(AUDIODSP_MAJOR, 0));
 error3:
	class_destroy(priv->class);
 error2:
	unregister_chrdev(AUDIODSP_MAJOR, DSP_NAME);
 error1:
	kfree(priv);
	return res;
}

struct audiodsp_priv *audiodsp_privdata(void)
{
	return audiodsp_p;
}

static int __init audiodsp_init_module(void)
{

	return audiodsp_probe();
}

static void __exit audiodsp_exit_module(void)
{
	struct audiodsp_priv *priv;
	priv = audiodsp_privdata();
#ifdef CONFIG_AM_STREAMING
	set_adec_func(NULL);
#endif
#if 0
	dsp_stop(priv);
	stop_audiodsp_monitor(priv);
	audiodsp_release_mailbox(priv);
	release_audiodsp_monitor(priv);
	audiodsp_microcode_free(priv);
#endif
	/*
	   iounmap(priv->p);
	 */
	device_destroy(priv->class, MKDEV(AUDIODSP_MAJOR, 0));
	class_destroy(priv->class);
	unregister_chrdev(AUDIODSP_MAJOR, DSP_NAME);
	kfree(priv);
	priv = NULL;
	return;
}

module_init(audiodsp_init_module);
module_exit(audiodsp_exit_module);
