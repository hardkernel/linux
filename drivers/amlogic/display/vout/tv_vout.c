/*
 * drivers/amlogic/display/vout/tv_vout.c
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


/* Linux Headers */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ctype.h>
#include <linux/major.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#ifdef CONFIG_INSTABOOT
#include <linux/syscore_ops.h>
#endif

/* Amlogic Headers */
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/vout/vinfo.h>
#ifdef CONFIG_AML_VPU
#include <linux/amlogic/vpu.h>
#endif

#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

/* Local Headers */
#include "tvregs.h"
#include "tv_vout.h"
#include "vout_io.h"
#include "vout_log.h"
#include "enc_clk_config.h"
#include "tv_out_reg.h"

#define PIN_MUX_REG_0 0x202c
#define P_PIN_MUX_REG_0 CBUS_REG_ADDR(PIN_MUX_REG_0)
static struct disp_module_info_s disp_module_info __nosavedata;
static struct disp_module_info_s *info __nosavedata;

static void vdac_power_level_store(char *para);
SET_TV_CLASS_ATTR(vdac_power_level, vdac_power_level_store)

static void bist_test_store(char *para);

static void cvbs_debug_store(char *para);
SET_TV_CLASS_ATTR(debug, cvbs_debug_store)

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
#define DEFAULT_POLICY_FR_AUTO	1
static enum vmode_e mode_by_user = VMODE_INIT_NULL;
static int fr_auto_policy = DEFAULT_POLICY_FR_AUTO;
static int fr_auto_policy_hold = DEFAULT_POLICY_FR_AUTO;
static int fps_playing_flag;
static enum vmode_e fps_target_mode = VMODE_INIT_NULL;
static void policy_framerate_automation_switch_store(char *para);
static void policy_framerate_automation_store(char *para);
SET_TV_CLASS_ATTR(policy_fr_auto, policy_framerate_automation_store)
SET_TV_CLASS_ATTR(policy_fr_auto_switch,
		policy_framerate_automation_switch_store)
static char *get_name_from_vmode(enum vmode_e mode);
static struct vinfo_s *update_tv_info_duration(
	enum vmode_e target_vmode, enum fine_tune_mode_e fine_tune_mode);
static int hdmitx_is_vmode_supported_process(char *mode_name);

#endif

static int tv_vdac_power_level;

static DEFINE_MUTEX(setmode_mutex);

static enum tvmode_e vmode_to_tvmode(enum vmode_e mode);
static void cvbs_config_vdac(unsigned int flag, unsigned int cfg);

#ifdef CONFIG_CVBS_PERFORMANCE_COMPATIBLITY_SUPPORT
static void cvbs_performance_enhancement(enum tvmode_e mode);
#endif
static void cvbs_cntl_output(unsigned int open);
static void cvbs_performance_config(unsigned int index);

static int get_vdac_power_level(void)
{
	return tv_vdac_power_level;
}

static void set_tvmode_misc(enum tvmode_e mode)
{
	/* for hdmi mode, leave the hpll setting to be done by hdmi module. */
	if ((get_cpu_type() == MESON_CPU_MAJOR_ID_M8) ||
	    (get_cpu_type() == MESON_CPU_MAJOR_ID_M8M2) ||
	    (get_cpu_type() == MESON_CPU_MAJOR_ID_GXBB)) {
		if ((mode == TVMODE_480CVBS) || (mode == TVMODE_576CVBS))
			set_vmode_clk(mode);
	} else
		set_vmode_clk(mode);
}

/*
 * uboot_display_already() uses to judge whether display has already
 * be set in uboot.
 * Here, first read the value of reg P_ENCP_VIDEO_MAX_PXCNT and
 * P_ENCP_VIDEO_MAX_LNCNT, then compare with value of tvregsTab[mode]
 */
static int uboot_display_already(enum tvmode_e mode)
{
	enum tvmode_e source = TVMODE_MAX;

	source = vmode_to_tvmode(get_logo_vmode());
	if (source == mode)
		return 1;
	else
		return 0;
}

static unsigned int vdac_cfg_valid = 0, vdac_cfg_value;
static unsigned int cvbs_get_trimming_version(unsigned int flag)
{
	unsigned int version = 0xff;
	if ((flag & 0xf0) == 0xa0)
		version = 5;
	else if ((flag & 0xf0) == 0x40)
		version = 2;
	else if ((flag & 0xc0) == 0x80)
		version = 1;
	else if ((flag & 0xc0) == 0x00)
		version = 0;
	return version;
}

static void cvbs_config_vdac(unsigned int flag, unsigned int cfg)
{
	unsigned char version = 0;
	vdac_cfg_value = cfg & 0x7;
	version = cvbs_get_trimming_version(flag);
	/* flag 1/0 for validity of vdac config */
	if ((version == 1) || (version == 2) || (version == 5))
		vdac_cfg_valid = 1;
	else
		vdac_cfg_valid = 0;
	vout_log_info("cvbs trimming.%d.v%d: 0x%x, 0x%x\n",
		      vdac_cfg_valid, version, flag, cfg);
	return;
}

static void cvbs_cntl_output(unsigned int open)
{
	unsigned int cntl0 = 0, cntl1 = 0;
	if (open == 0) { /* close */
		cntl0 = 0;
		cntl1 = 8;
		tv_out_hiu_write(HHI_VDAC_CNTL0, cntl0);
		tv_out_hiu_write(HHI_VDAC_CNTL1, cntl1);
	} else if (open == 1) { /* open */
		cntl0 = 0x1;
		cntl1 = (vdac_cfg_valid == 0) ? 0 : vdac_cfg_value;
		vout_log_info("vdac open.%d = 0x%x, 0x%x\n",
			      vdac_cfg_valid, cntl0, cntl1);
		tv_out_hiu_write(HHI_VDAC_CNTL1, cntl1);
		tv_out_hiu_write(HHI_VDAC_CNTL0, cntl0);
	}
	return;
}

/* 0xff for none config from uboot */
static unsigned int cvbs_performance_index = 0xff;
static void cvbs_performance_config(unsigned int index)
{
	cvbs_performance_index = index;
	return;
}

#ifdef CONFIG_CVBS_PERFORMANCE_COMPATIBLITY_SUPPORT
static void cvbs_performance_enhancement(enum tvmode_e mode)
{
	unsigned int index = cvbs_performance_index;
	unsigned int max = 0;
	unsigned int type = 0;
	const struct reg_s *s = NULL;
	if (TVMODE_576CVBS != mode)
		return;
	if (0xff == index)
		return;
	if (is_meson_m8b_cpu()) {
		max = sizeof(tvregs_576cvbs_performance_m8b)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = tvregs_576cvbs_performance_m8b[index];
		type = 1;
	} else if (is_meson_m8m2_cpu()) {
		max = sizeof(tvregs_576cvbs_performance_m8m2)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = tvregs_576cvbs_performance_m8m2[index];
		type = 2;
	} else if (is_meson_m8_cpu()) {
		max = sizeof(tvregs_576cvbs_performance_m8)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = tvregs_576cvbs_performance_m8[index];
		type = 0;
	} else if (is_meson_gxbb_cpu()) {
		max = sizeof(tvregs_576cvbs_performance_gxbb)
			/ sizeof(struct reg_s *);
		index = (index >= max) ? 0 : index;
		s = tvregs_576cvbs_performance_gxbb[index];
		type = 0;
	}

	vout_log_info("cvbs performance type = %d, table = %d\n", type, index);
	while (s && MREG_END_MARKER != s->reg) {
		vout_vcbus_write(s->reg, s->val);
		s++;
	}
	return;
}
#endif/* end of CVBS_PERFORMANCE_COMPATIBLITY_SUPPORT */

#if 0
static const struct reg_s *tvregs_get_clk_setting_by_mode(enum tvmode_e mode)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(tvregsTab); i++) {
		if (mode == tvregsTab[i].tvmode)
			return tvregsTab[i].clk_reg_setting;
	}
	return NULL;
}
#endif
static const struct reg_s *tvregs_get_enc_setting_by_mode(enum tvmode_e mode)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(tvregsTab); i++) {
		if (mode == tvregsTab[i].tvmode)
			return tvregsTab[i].enc_reg_setting;
	}
	return NULL;
}

static const struct tvinfo_s *tvinfo_mode(enum tvmode_e mode)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(tvinfoTab); i++) {
		if (mode == tvinfoTab[i].tvmode)
			return &tvinfoTab[i];
	}
	return NULL;
}

static void tv_out_init_off(enum tvmode_e mode)
{
	/* turn off cvbs clock */
	if (get_cpu_type() == MESON_CPU_MAJOR_ID_MG9TV)
		cvbs_cntl_output(0);
	else if (get_cpu_type() >= MESON_CPU_MAJOR_ID_M8) {
		/* for hdmi mode, disable HPLL as soon as possible */
		if ((mode == TVMODE_480I) || (mode == TVMODE_480P) ||
		    (mode == TVMODE_576I) || (mode == TVMODE_576P) ||
		    (mode == TVMODE_720P) || (mode == TVMODE_720P_50HZ) ||
		    (mode == TVMODE_1080I) || (mode == TVMODE_1080I_50HZ) ||
		    (mode == TVMODE_1080P) || (mode == TVMODE_1080P_50HZ) ||
		    (mode == TVMODE_1080P_24HZ) || (mode == TVMODE_4K2K_24HZ) ||
		    (mode == TVMODE_4K2K_25HZ) || (mode == TVMODE_4K2K_30HZ) ||
		    (mode == TVMODE_4K2K_FAKE_5G) ||
		    (mode == TVMODE_4K2K_SMPTE) || (mode == TVMODE_4K2K_60HZ))
			/* vout_cbus_set_bits(HHI_VID_PLL_CNTL, 0x0, 30, 1); */
			/* vout_cbus_set_bits(HHI_VID_PLL_CNTL, 0x0, 30, 1); */
		cvbs_cntl_output(0);
	}
	/* init encoding */
#if 0
	/* casue flick */
	tv_out_reg_write(ENCP_VIDEO_EN, 0);
	tv_out_reg_write(ENCI_VIDEO_EN, 0);
#endif
	tv_out_reg_write(VENC_VDAC_SETTING, 0xff);
}

static void tv_out_set_clk_gate(enum tvmode_e mode)
{
	/* remove it, use clk_gate API (resets) in dts */
#if 0
	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_M8B) {
		if ((mode != TVMODE_480CVBS) && (mode != TVMODE_576CVBS)) {
			/* TODO */
			/* CLK_GATE_OFF(CTS_VDAC); */
			vout_cbus_set_bits(HHI_VID_CLK_CNTL2, 0x0, 4, 1);
			/* CLK_GATE_OFF(DAC_CLK); */
			vout_cbus_set_bits(HHI_GCLK_OTHER, 0x0, 10, 1);
		}
		if ((mode != TVMODE_480I) && (mode != TVMODE_480CVBS) &&
		    (mode != TVMODE_576I) && (mode != TVMODE_576CVBS)) {
			/* TODO */
			/* CLK_GATE_OFF(CTS_ENCI); */
			vout_cbus_set_bits(HHI_VID_CLK_CNTL2, 0x0, 0, 1);
			/* CLK_GATE_OFF(VCLK2_ENCI); */
			vout_cbus_set_bits(HHI_GCLK_OTHER, 0x0, 8, 1);
			/* CLK_GATE_OFF(VCLK2_VENCI1); */
			vout_cbus_set_bits(HHI_GCLK_OTHER, 0x0, 2, 1);
		}
	}
	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_M8) {
		if ((mode == TVMODE_480CVBS) || (mode == TVMODE_576CVBS)) {
			msleep(1000);
			/* TODO */
			/* CLK_GATE_ON(VCLK2_ENCI); */
			vout_cbus_set_bits(HHI_GCLK_OTHER, 0x1, 8, 1);
			/* CLK_GATE_ON(VCLK2_VENCI1); */
			vout_cbus_set_bits(HHI_GCLK_OTHER, 0x1, 2, 1);
			/* CLK_GATE_ON(CTS_ENCI); */
			vout_cbus_set_bits(HHI_VID_CLK_CNTL2, 0x1, 0, 1);
			/* CLK_GATE_ON(CTS_VDAC); */
			vout_cbus_set_bits(HHI_VID_CLK_CNTL2, 0x1, 4, 1);
			/* CLK_GATE_ON(DAC_CLK); */
			vout_cbus_set_bits(HHI_GCLK_OTHER, 0x1, 10, 1);
		}
	}
#endif
}

static void tv_out_set_clk(enum tvmode_e mode)
{
#if 0
	const struct reg_s *s;

	/* setup clock regs */
	s = tvregs_get_clk_setting_by_mode(mode);
	if (!s) {
		vout_log_info("display mode %d get clk set NULL\n", mode);
	} else {
		while (MREG_END_MARKER != s->reg) {
			tv_out_hiu_write(s->reg, s->val);
			s++;
		}
	}
#endif
	set_tvmode_misc(mode);
}

static int tv_out_set_venc(enum tvmode_e mode)
{
	const struct reg_s *s;

	/* setup encoding regs */
	s = tvregs_get_enc_setting_by_mode(mode);
	if (!s) {
		vout_log_info("display mode %d get enc set NULL\n", mode);
		return -1;
	} else {
		while (MREG_END_MARKER != s->reg) {
			tv_out_reg_write(s->reg, s->val);
			s++;
		}
	}
	/* vout_log_info("%s[%d]\n", __func__, __LINE__); */
	return 0;
}

static void tv_out_set_enc_viu_mux(enum tvmode_e mode)
{
#ifdef CONFIG_AM_TV_OUTPUT2
	switch (mode) {
	case TVMODE_480I:
	case TVMODE_480I_RPT:
	case TVMODE_480CVBS:
	case TVMODE_576I:
	case TVMODE_576I_RPT:
	case TVMODE_576CVBS:
		/* reg0x271a, select ENCI to VIU1 */
		tv_out_reg_setb(VPU_VIU_VENC_MUX_CTRL, 1, 0, 2);
		/* reg0x271a, Select encI clock to VDIN */
		tv_out_reg_setb(VPU_VIU_VENC_MUX_CTRL, 1, 4, 4);
		/* reg0x271a, Enable VIU of ENC_I domain to VDIN; */
		tv_out_reg_setb(VPU_VIU_VENC_MUX_CTRL, 1, 8, 4);
		break;
	default:
		/* reg0x271a, select ENCP to VIU1 */
		tv_out_reg_setb(VPU_VIU_VENC_MUX_CTRL, 2, 0, 2);
		/* reg0x271a, Select encP clock to VDIN */
		tv_out_reg_setb(VPU_VIU_VENC_MUX_CTRL, 2, 4, 4);
		/* reg0x271a, Enable VIU of ENC_P domain to VDIN; */
		tv_out_reg_setb(VPU_VIU_VENC_MUX_CTRL, 2, 8, 4);
		break;
	}
#endif
}

static void tv_out_set_pinmux(enum tvmode_e mode)
{
	/* only m6 support VGA */
	if (get_cpu_type() == MESON_CPU_MAJOR_ID_M6) {
		if (mode >= TVMODE_VGA && mode <= TVMODE_FHDVGA)
			tv_out_cbus_set_mask(PERIPHS_PIN_MUX_0, (3 << 20));
		else
			tv_out_cbus_clr_mask(PERIPHS_PIN_MUX_0, (3 << 20));
	}
	/* vout_log_info("%s[%d] mode is %d\n", __func__, __LINE__, mode); */
}

static void tv_out_pre_close_vdac(enum tvmode_e mode)
{
	if (get_cpu_type() == MESON_CPU_MAJOR_ID_MG9TV) {
		if ((mode == TVMODE_480CVBS) || (mode == TVMODE_576CVBS))
			cvbs_cntl_output(0);
	} else if (get_cpu_type() >= MESON_CPU_MAJOR_ID_M8)
		cvbs_cntl_output(0);

	return;
}

static void tv_out_late_open_vdac(enum tvmode_e mode)
{
	if (get_cpu_type() == MESON_CPU_MAJOR_ID_M6) {
		if ((mode == TVMODE_480CVBS) || (mode == TVMODE_576CVBS)) {
			msleep(1000);
			if (get_vdac_power_level() == 0)
				tv_out_reg_write(VENC_VDAC_SETTING, 0x5);
			else
				tv_out_reg_write(VENC_VDAC_SETTING, 0x7);
		} else {
			if (get_vdac_power_level() == 0)
				tv_out_reg_write(VENC_VDAC_SETTING, 0x0);
			else
				tv_out_reg_write(VENC_VDAC_SETTING, 0x7);
		}
	} else if (get_cpu_type() >= MESON_CPU_MAJOR_ID_M8) {
		if ((mode == TVMODE_480CVBS) || (mode == TVMODE_576CVBS)) {
			msleep(1000);
			cvbs_cntl_output(1);
		}

	}

	return;
}

static void tv_out_enable_cvbs_gate(void)
{
	tv_out_hiu_set_mask(HHI_GCLK_OTHER,
		(1<<DAC_CLK) | (1<<GCLK_VENCI_INT) |
		(1<<VCLK2_VENCI1) | (1<<VCLK2_VENCI));
	tv_out_hiu_set_mask(HHI_VID_CLK_CNTL2,
		(1<<ENCI_GATE_VCLK) | (1<<VDAC_GATE_VCLK));
}

static char *tv_out_bist_str[] = {
	"Fix Value",   /* 0 */
	"Color Bar",   /* 1 */
	"Thin Line",   /* 2 */
	"Dot Grid",    /* 3 */
};

int tv_out_setmode(enum tvmode_e mode)
{
	const struct tvinfo_s *tvinfo;
	static int uboot_display_flag = 1;
	int ret;

	if (mode >= TVMODE_MAX) {
		vout_log_info(KERN_ERR "Invalid video output modes.\n");
		return -ENODEV;
	}
	mutex_lock(&setmode_mutex);
	tvinfo = tvinfo_mode(mode);
	if (!tvinfo) {
		vout_log_info(KERN_ERR "tvinfo %d not find\n", mode);
		mutex_unlock(&setmode_mutex);
		return 0;
	}
	vout_log_info("TV mode %s selected.\n", tvinfo->id);

	if (uboot_display_flag) {
		uboot_display_flag = 0;
		if (uboot_display_already(mode)) {
			vout_log_info("already display in uboot\n");
			mutex_unlock(&setmode_mutex);
			return 0;
		}
	}

	tv_out_pre_close_vdac(mode);

	tv_out_set_clk_gate(mode);
	tv_out_init_off(mode);
	tv_out_set_clk(mode);
	ret = tv_out_set_venc(mode);
	if (ret) {
		mutex_lock(&setmode_mutex);
		return -1;
	}
	tv_out_set_enc_viu_mux(mode);
	tv_out_set_pinmux(mode);
#ifdef CONFIG_CVBS_PERFORMANCE_COMPATIBLITY_SUPPORT
	cvbs_performance_enhancement(mode);
#endif
	tv_out_enable_cvbs_gate();
	tv_out_late_open_vdac(mode);

	mutex_unlock(&setmode_mutex);
	return 0;
}

static const enum tvmode_e vmode_tvmode_map(enum vmode_e mode)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(mode_tab); i++) {
		if (mode == mode_tab[i].mode)
			return mode_tab[i].tvmode;
	}
	return TVMODE_MAX;
}

static const struct file_operations am_tv_fops = {
	.open	= NULL,
	.read	= NULL,/* am_tv_read, */
	.write	= NULL,
	.unlocked_ioctl	= NULL,/* am_tv_ioctl, */
	.release	= NULL,
	.poll		= NULL,
};

static const struct vinfo_s *get_valid_vinfo(char  *mode)
{
	const struct vinfo_s *vinfo = NULL;
	int  i, count = ARRAY_SIZE(tv_info);
	int mode_name_len = 0;
	for (i = 0; i < count; i++) {
		if (strncmp(tv_info[i].name, mode,
			    strlen(tv_info[i].name)) == 0) {
			if ((vinfo == NULL)
			    || (strlen(tv_info[i].name) > mode_name_len)) {
				vinfo = &tv_info[i];
				mode_name_len = strlen(tv_info[i].name);
			}
		}
	}
	return vinfo;
}

static const struct vinfo_s *tv_get_current_info(void)
{
	return info->vinfo;
}

static enum tvmode_e vmode_to_tvmode(enum vmode_e mode)
{
	return vmode_tvmode_map(mode);
}

static struct vinfo_s *get_tv_info(enum vmode_e mode)
{
	struct hdmi_cea_timing *t = get_custom_timing();
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(tv_info); i++) {
		if (mode == tv_info[i].mode) {
			if (mode == VMODE_CUSTOMBUILT) {
				tv_info[i].width = t->h_active;
				tv_info[i].height = t->v_active;
				/* check progressive or interlaced */
				/* if progressive */
				tv_info[i].field_height
					= (t->v_active);

				tv_info[i].aspect_ratio_num = 16;
				tv_info[i].aspect_ratio_den = 9;

				tv_info[i].sync_duration_num
					= t->v_freq / 1000;
				tv_info[i].sync_duration_den = 1;

				/* in Hz unit */
				tv_info[i].video_clk
					= (t->pixel_freq * 1000);
			}
			return &tv_info[i];
		}
	}
	return NULL;
}

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
/* for hdmi (un)plug during fps automation */
static int want_hdmi_mode(enum vmode_e mode)
{
	int ret = 0;
	if ((mode == VMODE_480I)
	    || (mode == VMODE_480I_RPT)
	    || (mode == VMODE_480P)
	    || (mode == VMODE_480P_RPT)
	    || (mode == VMODE_576I)
	    || (mode == VMODE_576I_RPT)
	    || (mode == VMODE_576P)
	    || (mode == VMODE_576P_RPT)
	    || (mode == VMODE_720P)
	    || (mode == VMODE_720P_50HZ)
	    || (mode == VMODE_1080I)
	    || (mode == VMODE_1080I_50HZ)
	    || (mode == VMODE_1080P)
	    || (mode == VMODE_1080P_50HZ)
	    || (mode == VMODE_1080P_24HZ)
	    || (mode == VMODE_4K2K_24HZ)
	    || (mode == VMODE_4K2K_25HZ)
	    || (mode == VMODE_4K2K_30HZ)
	    || (mode == VMODE_4K2K_SMPTE)
	    || (mode == VMODE_4K2K_FAKE_5G)
	    || (mode == VMODE_4K2K_5G)
	    || (mode == VMODE_4K2K_50HZ)
	    || (mode == VMODE_4K2K_50HZ_Y420)
	    || (mode == VMODE_4K2K_60HZ)
	    || (mode == VMODE_4K2K_60HZ_Y420)
	   )
		ret = 1;
	return ret;
}

/* if plug hdmi during fps (stream is playing), then adjust mode to fps vmode */
static void fps_auto_adjust_mode(enum vmode_e *pmode)
{
	if (fps_playing_flag == 1) {
		if (want_hdmi_mode(*pmode) == 1) {
			if ((hdmitx_is_vmode_supported_process(
				get_name_from_vmode(mode_by_user)) == 1)
				&& (hdmitx_is_vmode_supported_process(
				get_name_from_vmode(fps_target_mode)) == 1)) {
				*pmode = fps_target_mode;
			} else {
				mode_by_user = *pmode;
				fps_target_mode = *pmode;
			}
		}
	}
}

static enum fine_tune_mode_e fine_tune_mode = KEEP_HPLL;
#endif

static int is_enci_required(enum vmode_e mode)
{
	if ((mode == VMODE_576I) ||
		(mode == VMODE_576I_RPT) ||
		(mode == VMODE_480I) ||
		(mode == VMODE_480I_RPT) ||
		(mode == VMODE_576CVBS) ||
		(mode == VMODE_480CVBS))
		return 1;
	return 0;
}

static void vout_change_mode_preprocess(enum vmode_e vmode_new)
{
	if (!is_enci_required(vmode_new))
		tv_out_hiu_setb(HHI_VID_CLK_CNTL2, 0, ENCI_GATE_VCLK, 1);

	return;
}

static int tv_set_current_vmode(enum vmode_e mode)
{
	if ((mode & VMODE_MODE_BIT_MASK) > VMODE_MAX)
		return -EINVAL;
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	/*
	 * if plug hdmi during fps (stream is playing)
	 * then adjust mode to fps vmode
	 */
	if (!want_hdmi_mode(mode)) {
		if (DOWN_HPLL == fine_tune_mode)
			update_tv_info_duration(fps_target_mode, UP_HPLL);
	}
	fps_auto_adjust_mode(&mode);
	update_vmode_status(get_name_from_vmode(mode));
	vout_log_info("%s[%d]fps_target_mode=%d\n",
		      __func__, __LINE__, mode);

	info->vinfo = update_tv_info_duration(mode, fine_tune_mode);
#else
	info->vinfo = get_tv_info(mode & VMODE_MODE_BIT_MASK);
#endif
	if (!info->vinfo) {
		vout_log_info("don't get tv_info, mode is %d\n", mode);
		return 1;
	}
	vout_log_info("mode is %d,sync_duration_den=%d,sync_duration_num=%d\n",
		      mode, info->vinfo->sync_duration_den,
		      info->vinfo->sync_duration_num);
	if (mode & VMODE_LOGO_BIT_MASK)
		return 0;

	vout_change_mode_preprocess(mode);

#ifdef CONFIG_AML_VPU
	switch_vpu_mem_pd_vmod(info->vinfo->mode, VPU_MEM_POWER_ON);
	request_vpu_clk_vmod(info->vinfo->video_clk, info->vinfo->mode);
#endif
	tv_out_reg_write(VPP_POSTBLEND_H_SIZE, info->vinfo->width);
	tv_out_setmode(vmode_to_tvmode(mode));

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	vout_log_info("new mode =%s set ok\n", get_name_from_vmode(mode));
#endif

	return 0;
}

static enum vmode_e tv_validate_vmode(char *mode)
{
	const struct vinfo_s *info = get_valid_vinfo(mode);
	if (info)
		return info->mode;
	return VMODE_MAX;
}
static int tv_vmode_is_supported(enum vmode_e mode)
{
	int  i, count = ARRAY_SIZE(tv_info);
	mode &= VMODE_MODE_BIT_MASK;
	for (i = 0; i < count; i++) {
		if (tv_info[i].mode == mode)
			return true;
	}
	return false;
}
static int tv_module_disable(enum vmode_e cur_vmod)
{
	if (get_cpu_type() < MESON_CPU_MAJOR_ID_M8)
		return 0;

#ifdef CONFIG_AML_VPU
	if (info->vinfo) {
		release_vpu_clk_vmod(info->vinfo->mode);
		switch_vpu_mem_pd_vmod(info->vinfo->mode, VPU_MEM_POWER_DOWN);
	}
#endif
	/* video_dac_disable(); */
	return 0;
}

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
static char *get_name_from_vmode(enum vmode_e mode)
{
	int i = 0, count = 0;
	count = ARRAY_SIZE(tv_info);
	for (i = 0; i < count; i++) {
		if (tv_info[i].mode == mode)
			break;
	}
	if (i == count)
		return NULL;
	return tv_info[i].name;
}

/* frame_rate = 9600/duration/100 hz */
static int get_vsource_frame_rate(int duration)
{
	int frame_rate = 0;
	switch (duration) {
	case 1600:
		frame_rate = 6000;
		break;
	case 1601:
	case 1602:
		frame_rate = 5994;
		break;
	case 1920:
		frame_rate = 5000;
		break;
	case 3200:
		frame_rate = 3000;
		break;
	case 3203:
		frame_rate = 2997;
		break;
	case 3840:
		frame_rate = 2500;
		break;
	case 4000:
		frame_rate = 2400;
		break;
	case 4004:
		frame_rate = 2397;
		break;
	default:
		break;
	}
	return frame_rate;
}

static int (*hdmi_edid_supported_func)(char *mode_name);
void register_hdmi_edid_supported_func(int (*pfunc)(char *mode_name))
{
	hdmi_edid_supported_func = pfunc;
}
EXPORT_SYMBOL(register_hdmi_edid_supported_func);
static int hdmitx_is_vmode_supported_process(char *mode_name)
{
	if (!mode_name) {
		vout_log_err("mode name is null\n");
		return 0;
	}
	if (hdmi_edid_supported_func)
		return hdmi_edid_supported_func(mode_name);
	else
		return 0;
}

static enum fine_tune_mode_e get_fine_tune_mode(
	enum vmode_e mode, int fr_vsource)
{
	enum fine_tune_mode_e tune_mode = KEEP_HPLL;
	switch (mode) {
	case VMODE_720P:
	case VMODE_1080P:
	case VMODE_1080P_24HZ:
	case VMODE_1080I:
	case VMODE_4K2K_30HZ:
	case VMODE_4K2K_24HZ:
	case VMODE_4K2K_60HZ:
	case VMODE_4K2K_60HZ_Y420:
		if ((fr_vsource == 2397) || (fr_vsource == 2997)
			|| (fr_vsource == 5994))
			tune_mode = DOWN_HPLL;
		break;
	case VMODE_480P:
		if ((get_cpu_type() == MESON_CPU_MAJOR_ID_M8) ||
			(get_cpu_type() == MESON_CPU_MAJOR_ID_M8B) ||
			(get_cpu_type() == MESON_CPU_MAJOR_ID_M8M2)) {
			if ((fr_vsource == 2397) || (fr_vsource == 2997)
				|| (fr_vsource == 5994))
				tune_mode = DOWN_HPLL;
		}
		break;
	default:
		break;
	}
	return tune_mode;
}

static enum vmode_e get_target_vmode(int fr_vsource)
{
	int is_receiver_supported = 0;
	const struct vinfo_s *pvinfo;
	enum vmode_e mode_target = VMODE_INIT_NULL;
	int i = 0, count = 0;
	struct fps_mode_conv *s = NULL;
	vout_log_info("fr_vsource = %d\n", fr_vsource);
	pvinfo = tv_get_current_info();
	if (!pvinfo) {
		vout_log_info("don't get vinfo!\n");
		return mode_target;
	}
	mode_target = pvinfo->mode;
	mode_by_user = mode_target;
	if (fr_auto_policy == 1) {/* for amlogic */
		fine_tune_mode = get_fine_tune_mode(mode_target, fr_vsource);
	} else if (fr_auto_policy == 2) {/* for jiewei */
		switch (fr_vsource) {
		case 2397:
			s = fps_mode_map_23;
			count = ARRAY_SIZE(fps_mode_map_23);
			break;
		case 2400:
			s = fps_mode_map_24;
			count = ARRAY_SIZE(fps_mode_map_24);
			break;
		case 2500:
			s = fps_mode_map_25;
			count = ARRAY_SIZE(fps_mode_map_25);
			break;
		case 2997:
			s = fps_mode_map_29;
			count = ARRAY_SIZE(fps_mode_map_29);
			break;
		case 3000:
			s = fps_mode_map_30;
			count = ARRAY_SIZE(fps_mode_map_30);
			break;
		case 5000:
			s = fps_mode_map_50;
			count = ARRAY_SIZE(fps_mode_map_50);
			break;
		case 5994:
			s = fps_mode_map_59;
			count = ARRAY_SIZE(fps_mode_map_59);
			break;
		case 6000:
			s = fps_mode_map_60;
			count = ARRAY_SIZE(fps_mode_map_60);
			break;
		default:
			break;
		}
	}
	for (i = 0; i < count; i++) {
		if (s[i].cur_mode == mode_target)
			break;
	}
	if (i < count) {
		mode_target = s[i].target_mode;
		/*for jiewei policy=2, if mode not support
		in edid, then will use Amloigc policy*/
		fine_tune_mode = get_fine_tune_mode(mode_target, fr_vsource);
	}

	is_receiver_supported = hdmitx_is_vmode_supported_process(
		get_name_from_vmode(mode_target));
	vout_log_info("mode_target=%d,is_receiver_supported=%d\n",
		mode_target, is_receiver_supported);
	switch (is_receiver_supported) {
	case 0: /* not supported in edid */
	case 2: /* no edid */
		mode_target = pvinfo->mode;
		break;
	case 1:/* supported in edid */
	default:
		break;
	}
	fps_target_mode = mode_target;
	return mode_target;
}

static struct vinfo_s *update_tv_info_duration(
	enum vmode_e target_vmode, enum fine_tune_mode_e fine_tune_mode)
{
	struct vinfo_s *vinfo;
	if ((target_vmode&VMODE_MODE_BIT_MASK) > VMODE_MAX)
		return NULL;
	vinfo = get_tv_info(target_vmode & VMODE_MODE_BIT_MASK);
	if (!vinfo) {
		vout_log_err("don't get tv_info, mode is %d\n", target_vmode);
		return NULL;
	}
	if (fine_tune_mode == DOWN_HPLL) {
		if ((target_vmode == VMODE_720P)
			|| (target_vmode == VMODE_1080I)
			|| (target_vmode == VMODE_1080P)
			|| (target_vmode == VMODE_4K2K_60HZ)
			|| (target_vmode == VMODE_4K2K_60HZ_Y420)) {
			vinfo->sync_duration_den = 1001;
			vinfo->sync_duration_num = 60000;
		} else if ((target_vmode == VMODE_1080P_24HZ)
			|| (target_vmode == VMODE_4K2K_24HZ)) {
			vinfo->sync_duration_den = 1001;
			vinfo->sync_duration_num = 24000;
		} else if (target_vmode == VMODE_4K2K_30HZ) {
			vinfo->sync_duration_den = 1001;
			vinfo->sync_duration_num = 30000;
		} else if (target_vmode == VMODE_480P) {
			if ((get_cpu_type() == MESON_CPU_MAJOR_ID_M8) ||
				(get_cpu_type() == MESON_CPU_MAJOR_ID_M8B) ||
				(get_cpu_type() == MESON_CPU_MAJOR_ID_M8M2)) {
				vinfo->sync_duration_den = 1001;
				vinfo->sync_duration_num = 60000;
			}
		}
	}  else if (fine_tune_mode == UP_HPLL) {
		if ((target_vmode == VMODE_720P)
			|| (target_vmode == VMODE_1080I)
			|| (target_vmode == VMODE_1080P)
			|| (target_vmode == VMODE_4K2K_60HZ)
			|| (target_vmode == VMODE_4K2K_60HZ_Y420)) {
			vinfo->sync_duration_den = 1;
			vinfo->sync_duration_num = 60;
		} else if ((target_vmode == VMODE_1080P_24HZ)
		|| (target_vmode == VMODE_4K2K_24HZ)) {
			vinfo->sync_duration_den = 1;
			vinfo->sync_duration_num = 24;
		} else if (target_vmode == VMODE_4K2K_30HZ) {
			vinfo->sync_duration_den = 1;
			vinfo->sync_duration_num = 30;
		} else if (target_vmode == VMODE_480P) {
			if ((get_cpu_type() == MESON_CPU_MAJOR_ID_M8) ||
				(get_cpu_type() == MESON_CPU_MAJOR_ID_M8B) ||
				(get_cpu_type() == MESON_CPU_MAJOR_ID_M8M2)) {
				vinfo->sync_duration_den = 1;
				vinfo->sync_duration_num = 60;
			}
		}
	}
	return vinfo;
}

static int framerate_automation_set_mode(
	enum vmode_e mode_target, enum hint_mode_e hint_mode)
{
	const struct vinfo_s *pvinfo;
	enum vmode_e mode_current = VMODE_INIT_NULL;
	if ((mode_target&VMODE_MODE_BIT_MASK) > VMODE_MAX)
		return 1;
	pvinfo = tv_get_current_info();
	if (!pvinfo) {
		vout_log_err("don't get tv_info, mode is %d\n", mode_target);
		return 1;
	}
	mode_current = pvinfo->mode;
	if (!want_hdmi_mode(mode_current)) {
		vout_log_info("not in valid HDMI mode!\n");
		return 1;
	}

	if (mode_current == mode_target) {
		if (fine_tune_mode == KEEP_HPLL)
			return 0;
	}
	vout_log_info("vout [%s] mode_target = %d\n", __func__, mode_target);
	if (START_HINT == hint_mode) {
		info->vinfo = update_tv_info_duration(mode_target,
			fine_tune_mode);
	} else if (END_HINT == hint_mode) {
		/*recovery tv_info array, e.g. 1080p59hz->1080p50hz
		or 1080p59hz->1080p60hz, recovery 1080p60hz vinfo*/
		update_tv_info_duration(fps_target_mode, fine_tune_mode);
		/*get current vinfo, maybe change is 1080p->1080p50,
		then get info->vinfo  for 1080p50hz*/
		info->vinfo = get_tv_info(mode_target & VMODE_MODE_BIT_MASK);
	}
	update_vmode_status(get_name_from_vmode(mode_target));
	vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE, &mode_target);
	return 0;
}

static int framerate_automation_process(int duration)
{
	int policy = 0, fr_vsource = 0;
	enum vmode_e mode_target = VMODE_INIT_NULL;

	vout_log_info("vout [%s] duration = %d\n", __func__, duration);
	policy = fr_auto_policy;
	if (policy == 0) {
		vout_log_info("vout frame rate automation disabled!\n");
		return 1;
	}
	fr_vsource = get_vsource_frame_rate(duration);
	fps_playing_flag = 0;
	if ((fr_vsource == 5994)
		|| (fr_vsource == 2997)
		|| (fr_vsource == 2397))
		fps_playing_flag = 1;
	else if (fr_auto_policy == 2) {/* fr_auto_policy: 2 for jiewei */
		if ((fr_vsource == 6000)
			|| (fr_vsource == 5000)
			|| (fr_vsource == 3000)
			|| (fr_vsource == 2500)
			|| (fr_vsource == 2400))
			fps_playing_flag = 1;
	}

	vout_log_info("%s[%d] fps_playing_flag = %d\n", __func__, __LINE__,
		      fps_playing_flag);
	mode_target = get_target_vmode(fr_vsource);
	framerate_automation_set_mode(mode_target, START_HINT);
	return 0;
}

enum fine_tune_mode_e get_hpll_tune_mode(void)
{
	return fine_tune_mode;
}
EXPORT_SYMBOL(get_hpll_tune_mode);

#endif

static int tv_set_vframe_rate_hint(int duration)
{
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	const struct vinfo_s *pvinfo;
	vout_log_info("vout [%s] duration = %d, policy = %d!\n",
		      __func__, duration, fr_auto_policy);
	pvinfo = tv_get_current_info();
	if (!pvinfo) {
		vout_log_err("don't get vinfo!\n");
		return 1;
	}
	/*if in CVBS mode, don't do frame rate automation*/
	if (!want_hdmi_mode(pvinfo->mode)) {
		vout_log_info("not in valid HDMI mode!\n");
		return 1;
	}

	framerate_automation_process(duration);
#endif
	return 0;
}

static int tv_set_vframe_rate_end_hint(void)
{
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	vout_log_info("vout [%s] return mode = %d, policy = %d!\n", __func__,
		      mode_by_user, fr_auto_policy);
	if (fr_auto_policy != 0) {
		fps_playing_flag = 0;
		if (DOWN_HPLL == fine_tune_mode)
			fine_tune_mode = UP_HPLL;
		framerate_automation_set_mode(mode_by_user, END_HINT);
		fine_tune_mode = KEEP_HPLL;
		fps_target_mode = VMODE_INIT_NULL;
		mode_by_user = VMODE_INIT_NULL;
	}
#endif
	return 0;
}

#ifdef CONFIG_PM
static int tv_suspend(void)
{
	/* TODO */
	/* video_dac_disable(); */
	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_M8)
		cvbs_cntl_output(0);
	return 0;
}

static int tv_resume(void)
{
	/* TODO */
	/* video_dac_enable(0xff); */
	tv_set_current_vmode(info->vinfo->mode);
	return 0;
}
#endif

static struct vout_server_s tv_server = {
	.name = "vout_tv_server",
	.op = {
		.get_vinfo = tv_get_current_info,
		.set_vmode = tv_set_current_vmode,
		.validate_vmode = tv_validate_vmode,
		.vmode_is_supported = tv_vmode_is_supported,
		.disable = tv_module_disable,
		.set_vframe_rate_hint = tv_set_vframe_rate_hint,
		.set_vframe_rate_end_hint = tv_set_vframe_rate_end_hint,
#ifdef CONFIG_PM
		.vout_suspend = tv_suspend,
		.vout_resume = tv_resume,
#endif
	},
};

static void _init_vout(void)
{
	if (info->vinfo == NULL)
		info->vinfo = &tv_info[TVMODE_720P];
}

/* **************************************************** */
static void bist_test_store(char *para)
{
	unsigned long num;
	enum tvmode_e mode;
	unsigned int start, width;
	int ret;

	mode = info->vinfo->mode;
	ret = kstrtoul(para, 10, (unsigned long *)&num);
	if (num > 3) {
		switch (mode) {
		case TVMODE_480I:
		case TVMODE_480I_RPT:
		case TVMODE_480CVBS:
		case TVMODE_576I:
		case TVMODE_576I_RPT:
		case TVMODE_576CVBS:
			tv_out_reg_write(ENCI_TST_EN, 0);
			break;
		default:
			tv_out_reg_setb(ENCP_VIDEO_MODE_ADV, 1, 3, 1);
			tv_out_reg_write(VENC_VIDEO_TST_EN, 0);
			break;
		}
		pr_info("disable bist pattern\n");
		return;
	} else {
		switch (mode) {
		case TVMODE_480I:
		case TVMODE_480I_RPT:
		case TVMODE_480CVBS:
		case TVMODE_576I:
		case TVMODE_576I_RPT:
		case TVMODE_576CVBS:
			tv_out_reg_write(ENCI_TST_CLRBAR_STRT, 0x112);
			tv_out_reg_write(ENCI_TST_CLRBAR_WIDTH, 0xb4);
			tv_out_reg_write(ENCI_TST_MDSEL, (unsigned int)num);
			tv_out_reg_write(ENCI_TST_EN, 1);
			break;
		default:
			start = tv_out_reg_read(ENCP_VIDEO_HAVON_BEGIN);
			width = info->vinfo->width / 9;
			tv_out_reg_write(VENC_VIDEO_TST_CLRBAR_STRT, start);
			tv_out_reg_write(VENC_VIDEO_TST_CLRBAR_WIDTH, width);
			tv_out_reg_write(VENC_VIDEO_TST_MDSEL, 1);
			tv_out_reg_setb(ENCP_VIDEO_MODE_ADV, 0, 3, 1);
			tv_out_reg_write(VENC_VIDEO_TST_EN, 1);
			break;
		}
		pr_info("show bist pattern %ld: %s\n",
			num, tv_out_bist_str[num]);
	}
}

static void vdac_power_level_store(char *para)
{
	unsigned long level = 0;
	int ret = 0;
	ret = kstrtoul(para, 10, (unsigned long *)&level);
	tv_vdac_power_level = level;
	return;
}

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
/*
 * 0: disable frame_rate_automation feature
 * 1: enable frame_rate_automation feature
 *    same with frame rate of video source
 * 2: enable frame_rate_automation feature
 *    use 59.94 instead of 23.97/29.97
 */
static void policy_framerate_automation_store(char *para)
{
	int policy = 0;
	int ret = 0;
	ret = kstrtoul(para, 10, (unsigned long *)&policy);
	if ((policy >= 0) && (policy < 3)) {
		fr_auto_policy_hold = policy;
		fr_auto_policy = fr_auto_policy_hold;
		snprintf(policy_fr_auto_switch, 40, "%d\n", fr_auto_policy);
	}
	snprintf(policy_fr_auto, 40, "%d\n", fr_auto_policy_hold);

	return;
}

static void policy_framerate_automation_switch_store(char *para)
{
	int policy = 0;
	int ret = 0;
	ret = kstrtoul(para, 10, (unsigned long *)&policy);
	if ((policy >= 0) && (policy < 3)) {
		fr_auto_policy = policy;
	} else if (policy == 3) {
		fr_auto_policy = fr_auto_policy_hold;
		tv_set_vframe_rate_end_hint();
	}
	snprintf(policy_fr_auto_switch, 40, "%d\n", fr_auto_policy);
	return;
}


#endif

static void dump_clk_registers(void)
{
	unsigned int clk_regs[] = {
		/* hiu 10c8 ~ 10cd */
		HHI_HDMI_PLL_CNTL,
		HHI_HDMI_PLL_CNTL2,
		HHI_HDMI_PLL_CNTL3,
		HHI_HDMI_PLL_CNTL4,
		HHI_HDMI_PLL_CNTL5,
		HHI_HDMI_PLL_CNTL6,

		/* hiu 1068 */
		HHI_VID_PLL_CLK_DIV,

		/* hiu 104a, 104b*/
		HHI_VIID_CLK_DIV,
		HHI_VIID_CLK_CNTL,

		/* hiu 1059, 105f */
		HHI_VID_CLK_DIV,
		HHI_VID_CLK_CNTL,
	};
	unsigned int i, max;

	max = sizeof(clk_regs)/sizeof(unsigned int);
	pr_info("\n total %d registers of clock path for hdmi pll:\n", max);
	for (i = 0; i < max; i++) {
		pr_info("hiu [0x%x] = 0x%x\n", clk_regs[i],
			tv_out_hiu_read(clk_regs[i]));
	}

	return;
}

enum {
	CMD_REG_READ,
	CMD_REG_READ_BITS,
	CMD_REG_DUMP,
	CMD_REG_WRITE,
	CMD_REG_WRITE_BITS,

	CMD_CLK_DUMP,
	CMD_CLK_MSR,

	CMD_BIST,

	CMD_HELP,

	CMD_MAX
} debug_cmd_t;

#define func_type_map(a) {\
	if (!strcmp(a, "c")) {\
		str_type = "cbus";\
		func_read = tv_out_cbus_read;\
		func_write = tv_out_cbus_write;\
		func_getb = tv_out_cbus_getb;\
		func_setb = tv_out_cbus_setb;\
	} \
	else if (!strcmp(a, "h")) {\
		str_type = "hiu";\
		func_read = tv_out_hiu_read;\
		func_write = tv_out_hiu_write;\
		func_getb = tv_out_hiu_getb;\
		func_setb = tv_out_hiu_setb;\
	} \
	else if (!strcmp(a, "v")) {\
		str_type = "vcbus";\
		func_read = tv_out_reg_read;\
		func_write = tv_out_reg_write;\
		func_getb = tv_out_reg_getb;\
		func_setb = tv_out_reg_setb;\
	} \
}

static void cvbs_debug_store(char *buf)
{
	unsigned int ret = 0;
	unsigned long addr, start, end, value, length, old;
	unsigned int argc;
	char *p = NULL, *para = NULL,
		*argv[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
	char *str_type = NULL;
	unsigned int i, cmd;
	unsigned int (*func_read)(unsigned int)  = NULL;
	void (*func_write)(unsigned int, unsigned int) = NULL;
	unsigned int (*func_getb)(unsigned int,
		unsigned int, unsigned int) = NULL;
	void (*func_setb)(unsigned int, unsigned int,
		unsigned int, unsigned int) = NULL;

	p = kstrdup(buf, GFP_KERNEL);
	for (argc = 0; argc < 6; argc++) {
		para = strsep(&p, " ");
		if (para == NULL)
			break;
		argv[argc] = para;
	}

	if (!strcmp(argv[0], "r"))
		cmd = CMD_REG_READ;
	else if (!strcmp(argv[0], "rb"))
		cmd = CMD_REG_READ_BITS;
	else if (!strcmp(argv[0], "dump"))
		cmd = CMD_REG_DUMP;
	else if (!strcmp(argv[0], "w"))
		cmd = CMD_REG_WRITE;
	else if (!strcmp(argv[0], "wb"))
		cmd = CMD_REG_WRITE_BITS;
	else if (!strncmp(argv[0], "clkdump", strlen("clkdump")))
		cmd = CMD_CLK_DUMP;
	else if (!strncmp(argv[0], "clkmsr", strlen("clkmsr")))
		cmd = CMD_CLK_MSR;
	else if (!strncmp(argv[0], "bist", strlen("bist")))
		cmd = CMD_BIST;
	else if (!strncmp(argv[0], "help", strlen("help")))
		cmd = CMD_HELP;
	else {
		print_info("[%s] invalid cmd = %s!\n", __func__, argv[0]);
		goto DEBUG_END;
	}

	switch (cmd) {
	case CMD_REG_READ:
		if (argc != 3) {
			print_info("[%s] cmd_reg_read format: r c/h/v address_hex\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[1]);
		ret = kstrtoul(argv[2], 16, &addr);

		print_info("read %s[0x%x] = 0x%x\n",
			str_type, (unsigned int)addr, func_read(addr));

		break;

	case CMD_REG_READ_BITS:
		if (argc != 5) {
			print_info("[%s] cmd_reg_read_bits format:\n"
			"\trb c/h/v address_hex start_dec length_dec\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[1]);
		ret = kstrtoul(argv[2], 16, &addr);
		ret = kstrtoul(argv[3], 10, &start);
		ret = kstrtoul(argv[4], 10, &length);

		if (length == 1)
			print_info("read_bits %s[0x%x] = 0x%x, bit[%d] = 0x%x\n",
			str_type, (unsigned int)addr,
			func_read(addr), (unsigned int)start,
			func_getb(addr, start, length));
		else
			print_info("read_bits %s[0x%x] = 0x%x, bit[%d-%d] = 0x%x\n",
			str_type, (unsigned int)addr,
			func_read(addr),
			(unsigned int)start+(unsigned int)length-1,
			(unsigned int)start,
			func_getb(addr, start, length));
		break;

	case CMD_REG_DUMP:
		if (argc != 4) {
			print_info("[%s] cmd_reg_dump format: dump c/h/v start_dec end_dec\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[1]);

		ret = kstrtoul(argv[2], 16, &start);
		ret = kstrtoul(argv[3], 16, &end);

		for (i = start; i <= end; i++)
			print_info("%s[0x%x] = 0x%x\n",
				str_type, i, func_read(i));

		break;

	case CMD_REG_WRITE:
		if (argc != 4) {
			print_info("[%s] cmd_reg_write format: w value_hex c/h/v address_hex\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[2]);

		ret = kstrtoul(argv[1], 16, &value);
		ret = kstrtoul(argv[3], 16, &addr);

		func_write(addr, value);
		print_info("write %s[0x%x] = 0x%x\n", str_type,
			(unsigned int)addr, (unsigned int)value);
		break;

	case CMD_REG_WRITE_BITS:
		if (argc != 6) {
			print_info("[%s] cmd_reg_wrute_bits format:\n"
			"\twb value_hex c/h/v address_hex start_dec length_dec\n",
				__func__);
			goto DEBUG_END;
		}

		func_type_map(argv[2]);

		ret = kstrtoul(argv[1], 16, &value);
		ret = kstrtoul(argv[3], 16, &addr);
		ret = kstrtoul(argv[4], 10, &start);
		ret = kstrtoul(argv[5], 10, &length);

		old = func_read(addr);
		func_setb(addr, value, start, length);
		print_info("write_bits %s[0x%x] old = 0x%x, new = 0x%x\n",
			str_type, (unsigned int)addr,
			(unsigned int)old, func_read(addr));
		break;

	case CMD_CLK_DUMP:
		dump_clk_registers();
		break;

	case CMD_CLK_MSR:
		/* todo */
		print_info("cvbs: debug_store: clk_msr todo!\n");
		break;

	case CMD_BIST:
		if (argc != 2) {
			print_info("[%s] cmd_bist format:\n"
			"\tbist 0/1/2/3/off\n", __func__);
			goto DEBUG_END;
		}

		bist_test_store(argv[1]);
		break;

	case CMD_HELP:
		print_info("command format:\n"
		"\tr c/h/v address_hex\n"
		"\trb c/h/v address_hex start_dec length_dec\n"
		"\tdump c/h/v start_dec end_dec\n"
		"\tw value_hex c/h/v address_hex\n"
		"\twb value_hex c/h/v address_hex start_dec length_dec\n"
		"\tbist 0/1/2/3/off\n"
		"\tclkdump\n");
		break;
	default:
		break;
	}

DEBUG_END:
	kfree(p);
	return;
}

static  struct  class_attribute   *tv_attr[] = {
	&class_TV_attr_vdac_power_level,
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	&class_TV_attr_policy_fr_auto,
	&class_TV_attr_policy_fr_auto_switch,
#endif
	&class_TV_attr_debug,
};



static int create_tv_attr(struct disp_module_info_s *info)
{
	/* create base class for display */
	int i;
	info->base_class = class_create(THIS_MODULE, info->name);
	if (IS_ERR(info->base_class)) {
		vout_log_err("create tv display class fail\n");
		return  -1;
	}
	/* create class attr */
	for (i = 0; i < ARRAY_SIZE(tv_attr); i++) {
		if (class_create_file(info->base_class, tv_attr[i]))
			vout_log_err("create disp attribute %s fail\n",
				     tv_attr[i]->attr.name);
	}

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	sprintf(policy_fr_auto, "%d", DEFAULT_POLICY_FR_AUTO);
	sprintf(policy_fr_auto_switch, "%d", DEFAULT_POLICY_FR_AUTO);
#endif
	return 0;
}
/* **************************************************** */

#ifdef CONFIG_INSTABOOT
struct class  *info_base_class;
static int tvconf_suspend(void)
{
	info_base_class = info->base_class;
	return 0;
}

static void tvconf_resume(void)
{
	info->base_class = info_base_class;
}

static struct syscore_ops tvconf_ops = {
	.suspend = tvconf_suspend,
	.resume = tvconf_resume,
	.shutdown = NULL,
};
#endif

static int __init tv_init_module(void)
{
	int  ret;
#ifdef CONFIG_INSTABOOT
	INIT_LIST_HEAD(&tvconf_ops.node);
	register_syscore_ops(&tvconf_ops);
#endif
	tv_out_ioremap();
	info = &disp_module_info;
	vout_log_info("%s\n", __func__);
	sprintf(info->name, TV_CLASS_NAME);
	ret = register_chrdev(0, info->name, &am_tv_fops);
	if (ret < 0) {
		vout_log_err("register char dev tv error\n");
		return  ret;
	}
	info->major = ret;
	_init_vout();
	vout_log_err("major number %d for disp\n", ret);
	if (vout_register_server(&tv_server))
		vout_log_err("register tv module server fail\n");
	else
		vout_log_info("register tv module server ok\n");
	create_tv_attr(info);
	return 0;
}

static __exit void tv_exit_module(void)
{
	int i;
	if (info->base_class) {
		for (i = 0; i < ARRAY_SIZE(tv_attr); i++)
			class_remove_file(info->base_class, tv_attr[i]);
		class_destroy(info->base_class);
	}
	if (info) {
		unregister_chrdev(info->major, info->name);
		kfree(info);
	}
	vout_unregister_server(&tv_server);
	vout_log_info("exit tv module\n");
}

static int __init vdac_config_bootargs_setup(char *line)
{
	unsigned int cfg = 0x00;
	int ret = 0;
	vout_log_info("cvbs trimming line = %s\n", line);
	ret = kstrtoul(line, 16, (unsigned long *)&cfg);
	cvbs_config_vdac((cfg & 0xff00) >> 8, cfg & 0xff);
	return 1;
}

__setup("vdaccfg=", vdac_config_bootargs_setup);

static int __init cvbs_performance_setup(char *line)
{
	unsigned int cfg = 0x1;
	int ret = 0;
	vout_log_info("cvbs performance line = %s\n", line);
	ret = kstrtoul(line, 10, (unsigned long *)&cfg);
	cvbs_performance_config(cfg);
	return 0;
}
__setup("cvbsdrv=", cvbs_performance_setup);

arch_initcall(tv_init_module);
module_exit(tv_exit_module);

MODULE_AUTHOR("Platform-BJ <platform.bj@amlogic.com>");
MODULE_DESCRIPTION("TV Output Module");
MODULE_LICENSE("GPL");
