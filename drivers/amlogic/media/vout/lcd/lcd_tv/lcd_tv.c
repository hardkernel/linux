/*
 * drivers/amlogic/media/vout/lcd/lcd_tv/lcd_tv.c
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

#include <linux/init.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/clk.h>
#ifdef CONFIG_AMLOGIC_VPU
#include <linux/amlogic/media/vpu/vpu.h>
#endif
#include <linux/amlogic/media/vout/vinfo.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#include <linux/amlogic/media/vout/lcd/lcd_notify.h>
#include "lcd_tv.h"
#include "../lcd_reg.h"
#include "../lcd_common.h"

static unsigned int lcd_output_vmode;
static char lcd_output_name[30];
static int lcd_init_on_flag;

/* ************************************************** *
 * lcd mode function
 * **************************************************
 */
static unsigned int lcd_std_frame_rate[] = {
	50,
	60,
	48,
};

struct lcd_vmode_info_s {
	char *name;
	enum vmode_e mode;
	unsigned int width;
	unsigned int height;
	unsigned int frame_rate;
};

enum lcd_vmode_e {
	LCD_VMODE_600P = 0,
	LCD_VMODE_768P,
	LCD_VMODE_1080P,
	LCD_VMODE_2160P,
	LCD_VMODE_MAX,
};

static struct lcd_vmode_info_s lcd_vmode_info[] = {
	{
		.name              = "600p",
		.mode              = VMODE_LCD,
		.width             = 1024,
		.height            = 600,
		.frame_rate        = 60,
	},
	{
		.name              = "768p",
		.mode              = VMODE_LCD,
		.width             = 1366,
		.height            = 768,
		.frame_rate        = 60,
	},
	{
		.name              = "1080p",
		.mode              = VMODE_LCD,
		.width             = 1920,
		.height            = 1080,
		.frame_rate        = 60,
	},
	{
		.name              = "2160p",
		.mode              = VMODE_LCD,
		.width             = 3840,
		.height            = 2160,
		.frame_rate        = 60,
	},
	{
		.name              = "invalid",
		.mode              = VMODE_INIT_NULL,
		.width             = 1920,
		.height            = 1080,
		.frame_rate        = 60,
	},
};

static int lcd_vmode_is_mached(int index)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;

	pconf = lcd_drv->lcd_config;
	if ((pconf->lcd_basic.h_active == lcd_vmode_info[index].width) &&
		(pconf->lcd_basic.v_active == lcd_vmode_info[index].height))
		return 0;
	else
		return -1;
}

static int lcd_get_vmode(enum vmode_e mode)
{
	int lcd_vmode = LCD_VMODE_MAX;
	int i, count = ARRAY_SIZE(lcd_vmode_info) - 1;
	int ret;

	for (i = 0; i < count; i++) {
		if (mode == lcd_vmode_info[i].mode) {
			ret = lcd_vmode_is_mached(i);
			if (ret == 0) {
				lcd_vmode = i;
				break;
			}
		}
	}
	return lcd_vmode;
}

static int lcd_outputmode_to_lcd_vmode(const char *mode)
{
	int lcd_vmode = LCD_VMODE_MAX;
	int i, count = ARRAY_SIZE(lcd_vmode_info) - 1;
	char temp[30], *p;
	int n;

	p = strchr(mode, 'p');
	if (p == NULL)
		return LCD_VMODE_MAX;
	n = p - mode + 1;
	strncpy(temp, mode, n);
	temp[n] = '\0';
	if (lcd_debug_print_flag)
		LCDPR("outputmode=%s, lcd_vmode=%s\n", mode, temp);

	for (i = 0; i < count; i++) {
		if (strcmp(temp, lcd_vmode_info[i].name) == 0) {
			lcd_vmode = i;
			break;
		}
	}
	return lcd_vmode;
}

static int lcd_outputmode_to_lcd_frame_rate(const char *mode)
{
	int frame_rate = 0;
	char temp[30], *p;
	int n, i, ret = 0;

	p = strchr(mode, 'p');
	if (p == NULL)
		return 0;
	n = p - mode + 1;
	strncpy(temp, mode+n, (strlen(mode)-n));
	p = strchr(temp, 'h');
	if (p == NULL)
		return 0;
	*p = '\0';
	ret = kstrtoint(temp, 10, &n);
	if (lcd_debug_print_flag)
		LCDPR("outputmode=%s, frame_rate=%d\n", mode, n);

	for (i = 0; i < ARRAY_SIZE(lcd_std_frame_rate); i++) {
		if (n == lcd_std_frame_rate[i]) {
			frame_rate = n;
			break;
		}
	}
	return frame_rate;
}

static void lcd_vmode_vinfo_update(enum vmode_e mode)
{
	struct lcd_vmode_info_s *info;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;

	pconf = lcd_drv->lcd_config;
	lcd_output_vmode = lcd_get_vmode(mode);
	info = &lcd_vmode_info[lcd_output_vmode];
	sprintf(lcd_output_name, "%s%dhz", info->name, info->frame_rate);
	if (lcd_debug_print_flag) {
		LCDPR("%s vmode = %d, lcd_vmode = %d, outputmode = %s\n",
			__func__, mode, lcd_output_vmode, lcd_output_name);
	}

	/* store standard duration */
	if (pconf->lcd_timing.fr_adjust_type == 0xff) {
		lcd_drv->std_duration.duration_num =
			((pconf->lcd_timing.lcd_clk /
			pconf->lcd_basic.h_period) * 100) /
			pconf->lcd_basic.v_period;
		lcd_drv->std_duration.duration_den = 100;
	} else {
		lcd_drv->std_duration.duration_num = info->frame_rate;
		lcd_drv->std_duration.duration_den = 1;
	}

	/* update vinfo */
	pconf = lcd_drv->lcd_config;
	lcd_drv->lcd_info->name = lcd_output_name;
	lcd_drv->lcd_info->mode = info->mode;
	lcd_drv->lcd_info->width = info->width;
	lcd_drv->lcd_info->height = info->height;
	lcd_drv->lcd_info->field_height = info->height;
	lcd_drv->lcd_info->aspect_ratio_num = pconf->lcd_basic.screen_width;
	lcd_drv->lcd_info->aspect_ratio_den = pconf->lcd_basic.screen_height;
	lcd_drv->lcd_info->screen_real_width = pconf->lcd_basic.screen_width;
	lcd_drv->lcd_info->screen_real_height = pconf->lcd_basic.screen_height;
	lcd_drv->lcd_info->sync_duration_num =
		lcd_drv->std_duration.duration_num;
	lcd_drv->lcd_info->sync_duration_den =
		lcd_drv->std_duration.duration_den;
	lcd_drv->lcd_info->video_clk = pconf->lcd_timing.lcd_clk;
	lcd_drv->lcd_info->htotal = pconf->lcd_basic.h_period;
	lcd_drv->lcd_info->vtotal = pconf->lcd_basic.v_period;
	lcd_drv->lcd_info->viu_mux = VIU_MUX_ENCL;
	switch (pconf->lcd_timing.fr_adjust_type) {
	case 0:
		lcd_drv->lcd_info->fr_adj_type = VOUT_FR_ADJ_CLK;
		break;
	case 1:
		lcd_drv->lcd_info->fr_adj_type = VOUT_FR_ADJ_HTOTAL;
		break;
	case 2:
		lcd_drv->lcd_info->fr_adj_type = VOUT_FR_ADJ_VTOTAL;
		break;
	case 3:
		lcd_drv->lcd_info->fr_adj_type = VOUT_FR_ADJ_COMBO;
		break;
	case 4:
		lcd_drv->lcd_info->fr_adj_type = VOUT_FR_ADJ_HDMI;
		break;
	default:
		lcd_drv->lcd_info->fr_adj_type = VOUT_FR_ADJ_NONE;
		break;
	}

	lcd_optical_vinfo_update();
}

/* ************************************************** *
 * vout server api
 * **************************************************
 */
static enum vmode_e lcd_validate_vmode(char *mode)
{
	int lcd_vmode, frame_rate;
	int ret;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_vout_serve_bypass) {
		LCDPR("vout_serve bypass\n");
		return VMODE_MAX;
	}
	if (mode == NULL)
		return VMODE_MAX;

	lcd_vmode = lcd_outputmode_to_lcd_vmode(mode);
	ret = lcd_vmode_is_mached(lcd_vmode);
	if (ret) {
		LCDERR("%s: outputmode is not support\n", __func__);
		return VMODE_MAX;
	}
	frame_rate = lcd_outputmode_to_lcd_frame_rate(mode);
	if (lcd_drv->lcd_config->lcd_timing.fr_adjust_type == 0xff)
		return lcd_vmode_info[lcd_vmode].mode;
	if (frame_rate == 0) {
		LCDERR("%s: frame_rate is not support\n", __func__);
		return VMODE_MAX;
	} else {
		lcd_vmode_info[lcd_vmode].frame_rate = frame_rate;
		if (lcd_vmode < LCD_VMODE_MAX)
			return lcd_vmode_info[lcd_vmode].mode;
		else
			return VMODE_MAX;
	}

	return VMODE_MAX;
}

static struct vinfo_s *lcd_get_current_info(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return lcd_drv->lcd_info;
}

static int lcd_set_current_vmode(enum vmode_e mode)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_vout_serve_bypass) {
		LCDPR("vout_serve bypass\n");
		return 0;
	}
	if (lcd_drv->lcd_config->lcd_timing.fr_adjust_type == 0xff) {
		LCDPR("fixed timing, exit vmode change\n");
		return -1;
	}
	mutex_lock(&lcd_drv->power_mutex);

	/* do not change mode value here, for bit mask is useful */
	lcd_vmode_vinfo_update(mode & VMODE_MODE_BIT_MASK);

	if (VMODE_LCD == (mode & VMODE_MODE_BIT_MASK)) {
		if (mode & VMODE_INIT_BIT_MASK) {
			lcd_clk_gate_switch(1);
		} else if (lcd_init_on_flag == 0) {
			lcd_init_on_flag = 1;
			if ((lcd_drv->lcd_key_valid == 0)
			&& !(lcd_drv->lcd_status & LCD_STATUS_ENCL_ON)) {
				aml_lcd_notifier_call_chain
					(LCD_EVENT_POWER_ON, NULL);
				lcd_if_enable_retry(lcd_drv->lcd_config);
			} else if (lcd_drv->driver_change != NULL) {
				mutex_lock(&lcd_vout_mutex);
				ret = lcd_drv->driver_change();
				mutex_unlock(&lcd_vout_mutex);
			}
		} else if (lcd_init_on_flag == 1) {
			mutex_lock(&lcd_vout_mutex);
			ret = lcd_drv->driver_change();
			mutex_unlock(&lcd_vout_mutex);
		}
	} else {
		ret = -EINVAL;
	}

	lcd_drv->lcd_status |= LCD_STATUS_VMODE_ACTIVE;

	mutex_unlock(&lcd_drv->power_mutex);
	return ret;
}

static int lcd_vmode_is_supported(enum vmode_e mode)
{
	int lcd_vmode;

	mode &= VMODE_MODE_BIT_MASK;
	lcd_vmode = lcd_get_vmode(mode);
	if (lcd_debug_print_flag) {
		LCDPR("%s vmode = %d, lcd_vmode = %d(%s)\n",
			__func__, mode, lcd_vmode,
			lcd_vmode_info[lcd_vmode].name);
	}

	if (lcd_vmode < LCD_VMODE_MAX)
		return true;
	else
		return false;
}

static int lcd_vout_disable(enum vmode_e cur_vmod)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_drv->lcd_status &= ~LCD_STATUS_VMODE_ACTIVE;

	return 0;
}

static int lcd_vout_state;
static int lcd_vout_set_state(int index)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_vout_state |= (1 << index);
	lcd_drv->viu_sel = index;

	return 0;
}

static int lcd_vout_clr_state(int index)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_vout_state &= ~(1 << index);
	if (lcd_drv->viu_sel == index)
		lcd_drv->viu_sel = LCD_VIU_SEL_NONE;

	return 0;
}

static int lcd_vout_get_state(void)
{
	return lcd_vout_state;
}

#ifdef CONFIG_AMLOGIC_VOUT_SERVE
struct lcd_vframe_match_s {
	int fps;
	int frame_rate; /* *100 */
	unsigned int duration_num;
	unsigned int duration_den;
};

static struct lcd_vframe_match_s lcd_vframe_match_table_1[] = {
	{5000, 5000, 50, 1},
	{2500, 5000, 50, 1},
	{6000, 6000, 60, 1},
	{3000, 6000, 60, 1},
	{2400, 6000, 60, 1},
	{2397, 5994, 5994, 100},
	{2997, 5994, 5994, 100},
	{5994, 5994, 5994, 100},
};

static struct lcd_vframe_match_s lcd_vframe_match_table_2[] = {
	{5000, 5000, 50, 1},
	{2500, 5000, 50, 1},
	{6000, 6000, 60, 1},
	{3000, 6000, 60, 1},
	{2400, 4800, 48, 1},
	{2397, 5994, 5994, 100},
	{2997, 5994, 5994, 100},
	{5994, 5994, 5994, 100},
};

static int lcd_framerate_automation_set_mode(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	LCDPR("%s\n", __func__);

	/* update interface timing */
	lcd_tv_config_update(lcd_drv->lcd_config);
#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(
		lcd_drv->lcd_config->lcd_timing.lcd_clk, VPU_VENCL);
#endif

	if (lcd_drv->lcd_config->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_vbyone_interrupt_enable(0);
	/* change clk parameter */
	lcd_clk_change(lcd_drv->lcd_config);
	lcd_venc_change(lcd_drv->lcd_config);
	if (lcd_drv->lcd_config->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_vbyone_wait_stable();

	lcd_vout_notify_mode_change();

	return 0;
}
#endif

static int lcd_set_vframe_rate_hint(int duration)
{
#ifdef CONFIG_AMLOGIC_VOUT_SERVE
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vinfo_s *info;
	int fr_policy;
	unsigned int frame_rate = 6000;
	unsigned int duration_num = 60, duration_den = 1;
	struct lcd_vframe_match_s *vtable = lcd_vframe_match_table_1;
	int fps, i, n;

	if (lcd_vout_serve_bypass) {
		LCDPR("vout_serve bypass\n");
		return 0;
	}
	if ((lcd_drv->lcd_status & LCD_STATUS_ENCL_ON) == 0) {
		LCDPR("%s: lcd is disabled, exit\n", __func__);
		return 0;
	}

	if (lcd_drv->lcd_config->lcd_timing.fr_adjust_type == 0xff) {
		LCDPR("%s: fixed timing, exit\n", __func__);
		return 0;
	}
	info = lcd_drv->lcd_info;

	fr_policy = lcd_drv->fr_auto_policy;
	switch (fr_policy) {
	case 1:
		vtable = lcd_vframe_match_table_1;
		n = ARRAY_SIZE(lcd_vframe_match_table_1);
		break;
	case 2:
		vtable = lcd_vframe_match_table_2;
		n = ARRAY_SIZE(lcd_vframe_match_table_2);
		break;
	default:
		LCDPR("%s: fr_auto_policy = %d, disabled\n",
			__func__, fr_policy);
		return 0;
	}
	fps = vout_get_vsource_fps(duration);
	for (i = 0; i < n; i++) {
		if (fps == vtable[i].fps) {
			frame_rate = vtable[i].frame_rate;
			duration_num = vtable[i].duration_num;
			duration_den = vtable[i].duration_den;
		}
	}
	LCDPR("%s: policy = %d, duration = %d, fps = %d, frame_rate = %d\n",
		__func__, fr_policy, duration, fps, frame_rate);

	/* if the sync_duration is same as current */
	if ((duration_num == info->sync_duration_num) &&
		(duration_den == info->sync_duration_den)) {
		LCDPR("%s: sync_duration is the same, exit\n", __func__);
		return 0;
	}

	/* update vinfo */
	info->sync_duration_num = duration_num;
	info->sync_duration_den = duration_den;
	lcd_drv->fr_mode = 1;

	lcd_framerate_automation_set_mode();
#endif
	return 0;
}

static int lcd_set_vframe_rate_end_hint(void)
{
#ifdef CONFIG_AMLOGIC_VOUT_SERVE
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vinfo_s *info;

	if (lcd_vout_serve_bypass) {
		LCDPR("vout_serve bypass\n");
		return 0;
	}
	if ((lcd_drv->lcd_status & LCD_STATUS_ENCL_ON) == 0) {
		LCDPR("%s: lcd is disabled, exit\n", __func__);
		return 0;
	}

	if (lcd_drv->lcd_config->lcd_timing.fr_adjust_type == 0xff) {
		LCDPR("%s: fixed timing, exit\n", __func__);
		return 0;
	}
	if (lcd_debug_print_flag)
		LCDPR("fr_auto_policy = %d\n", lcd_drv->fr_auto_policy);
	if (lcd_drv->fr_auto_policy) {
		info = lcd_drv->lcd_info;
		LCDPR("%s: return mode = %s, policy = %d\n", __func__,
			info->name, lcd_drv->fr_auto_policy);

		if (lcd_drv->fr_mode == 0) {
			LCDPR("%s: fr_mode is invalid, exit\n", __func__);
			return 0;
		}

		/* update vinfo */
		info->sync_duration_num = lcd_drv->std_duration.duration_num;
		info->sync_duration_den = lcd_drv->std_duration.duration_den;
		lcd_drv->fr_mode = 0;

		lcd_framerate_automation_set_mode();
	}
#endif
	return 0;
}

static int lcd_set_vframe_rate_policy(int policy)
{
#ifdef CONFIG_AMLOGIC_VOUT_SERVE
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_vout_serve_bypass) {
		LCDPR("vout_serve bypass\n");
		return 0;
	}
	lcd_drv->fr_auto_policy = policy;
	LCDPR("%s: %d\n", __func__, lcd_drv->fr_auto_policy);
#endif
	return 0;
}

static int lcd_get_vframe_rate_policy(void)
{
#ifdef CONFIG_AMLOGIC_VOUT_SERVE
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return lcd_drv->fr_auto_policy;
#else
	return 0;
#endif
}

#ifdef CONFIG_PM
static int lcd_suspend(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	mutex_lock(&lcd_drv->power_mutex);
	aml_lcd_notifier_call_chain(LCD_EVENT_POWER_OFF, NULL);
	lcd_resume_flag = 0;
	LCDPR("%s finished\n", __func__);
	mutex_unlock(&lcd_drv->power_mutex);
	return 0;
}

static int lcd_resume(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if ((lcd_drv->lcd_status & LCD_STATUS_VMODE_ACTIVE) == 0)
		return 0;

	if (lcd_resume_flag)
		return 0;

	if (lcd_drv->lcd_resume_type) {
		if (lcd_drv->workqueue) {
			queue_work(lcd_drv->workqueue,
				&(lcd_drv->lcd_resume_work));
		} else {
			schedule_work(&(lcd_drv->lcd_resume_work));
		}
	} else {
		mutex_lock(&lcd_drv->power_mutex);
		LCDPR("directly lcd late resume\n");
		lcd_resume_flag = 1;
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_ON, NULL);
		lcd_if_enable_retry(lcd_drv->lcd_config);
		LCDPR("%s finished\n", __func__);
		mutex_unlock(&lcd_drv->power_mutex);
	}

	return 0;
}

#endif

static struct vout_server_s lcd_vout_server = {
	.name = "lcd_vout_server",
	.op = {
		.get_vinfo = lcd_get_current_info,
		.set_vmode = lcd_set_current_vmode,
		.validate_vmode = lcd_validate_vmode,
		.vmode_is_supported = lcd_vmode_is_supported,
		.disable = lcd_vout_disable,
		.set_state = lcd_vout_set_state,
		.clr_state = lcd_vout_clr_state,
		.get_state = lcd_vout_get_state,
		.set_vframe_rate_hint = lcd_set_vframe_rate_hint,
		.set_vframe_rate_end_hint = lcd_set_vframe_rate_end_hint,
		.set_vframe_rate_policy = lcd_set_vframe_rate_policy,
		.get_vframe_rate_policy = lcd_get_vframe_rate_policy,
		.set_bist = lcd_debug_test,
#ifdef CONFIG_PM
		.vout_suspend = lcd_suspend,
		.vout_resume = lcd_resume,
#endif
	},
};

static void lcd_vinfo_update_default(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vinfo_s *vinfo;
	unsigned int h_active, v_active, h_total, v_total;
	char *mode;

	if (lcd_drv->lcd_info == NULL) {
		LCDERR("no lcd_info exist\n");
		return;
	}

	mode = get_vout_mode_uboot();
	h_active = lcd_vcbus_read(ENCL_VIDEO_HAVON_END)
			- lcd_vcbus_read(ENCL_VIDEO_HAVON_BEGIN) + 1;
	v_active = lcd_vcbus_read(ENCL_VIDEO_VAVON_ELINE)
			- lcd_vcbus_read(ENCL_VIDEO_VAVON_BLINE) + 1;
	h_total = lcd_vcbus_read(ENCL_VIDEO_MAX_PXCNT) + 1;
	v_total = lcd_vcbus_read(ENCL_VIDEO_MAX_LNCNT) + 1;

	vinfo = lcd_drv->lcd_info;
	if (vinfo) {
		vinfo->name = mode;
		vinfo->mode = VMODE_LCD;
		vinfo->width = h_active;
		vinfo->height = v_active;
		vinfo->field_height = v_active;
		vinfo->aspect_ratio_num = h_active;
		vinfo->aspect_ratio_den = v_active;
		vinfo->screen_real_width = h_active;
		vinfo->screen_real_height = v_active;
		vinfo->sync_duration_num = 60;
		vinfo->sync_duration_den = 1;
		vinfo->video_clk = 0;
		vinfo->htotal = h_total;
		vinfo->vtotal = v_total;
		vinfo->fr_adj_type = VOUT_FR_ADJ_NONE;
	}
}

void lcd_tv_vout_server_init(void)
{
	lcd_vinfo_update_default();

	vout_register_server(&lcd_vout_server);
}

void lcd_tv_vout_server_remove(void)
{
	vout_register_server(&lcd_vout_server);
}

/* ************************************************** *
 * lcd tv config
 * **************************************************
 */
static void lcd_config_print(struct lcd_config_s *pconf)
{
	LCDPR("%s, %s, %dbit, %dx%d\n",
		pconf->lcd_basic.model_name,
		lcd_type_type_to_str(pconf->lcd_basic.lcd_type),
		pconf->lcd_basic.lcd_bits,
		pconf->lcd_basic.h_active, pconf->lcd_basic.v_active);

	if (lcd_debug_print_flag == 0)
		return;

	LCDPR("h_period = %d\n", pconf->lcd_basic.h_period);
	LCDPR("v_period = %d\n", pconf->lcd_basic.v_period);
	LCDPR("screen_width = %d\n", pconf->lcd_basic.screen_width);
	LCDPR("screen_height = %d\n", pconf->lcd_basic.screen_height);

	LCDPR("h_period_min = %d\n", pconf->lcd_basic.h_period_min);
	LCDPR("h_period_max = %d\n", pconf->lcd_basic.h_period_max);
	LCDPR("v_period_min = %d\n", pconf->lcd_basic.v_period_min);
	LCDPR("v_period_max = %d\n", pconf->lcd_basic.v_period_max);
	LCDPR("pclk_min = %d\n", pconf->lcd_basic.lcd_clk_min);
	LCDPR("pclk_max = %d\n", pconf->lcd_basic.lcd_clk_max);

	LCDPR("hsync_width = %d\n", pconf->lcd_timing.hsync_width);
	LCDPR("hsync_bp = %d\n", pconf->lcd_timing.hsync_bp);
	LCDPR("hsync_pol = %d\n", pconf->lcd_timing.hsync_pol);
	LCDPR("vsync_width = %d\n", pconf->lcd_timing.vsync_width);
	LCDPR("vsync_bp = %d\n", pconf->lcd_timing.vsync_bp);
	LCDPR("vsync_pol = %d\n", pconf->lcd_timing.vsync_pol);

	LCDPR("fr_adjust_type = %d\n", pconf->lcd_timing.fr_adjust_type);
	LCDPR("ss_level = %d\n", pconf->lcd_timing.ss_level);
	LCDPR("clk_auto = %d\n", pconf->lcd_timing.clk_auto);

	if (pconf->lcd_basic.lcd_type == LCD_VBYONE) {
		LCDPR("lane_count = %d\n",
			pconf->lcd_control.vbyone_config->lane_count);
		LCDPR("byte_mode = %d\n",
			pconf->lcd_control.vbyone_config->byte_mode);
		LCDPR("region_num = %d\n",
			pconf->lcd_control.vbyone_config->region_num);
		LCDPR("color_fmt = %d\n",
			pconf->lcd_control.vbyone_config->color_fmt);
	} else if (pconf->lcd_basic.lcd_type == LCD_LVDS) {
		LCDPR("lvds_repack = %d\n",
			pconf->lcd_control.lvds_config->lvds_repack);
		LCDPR("pn_swap = %d\n",
			pconf->lcd_control.lvds_config->pn_swap);
		LCDPR("dual_port = %d\n",
			pconf->lcd_control.lvds_config->dual_port);
		LCDPR("port_swap = %d\n",
			pconf->lcd_control.lvds_config->port_swap);
		LCDPR("lane_reverse = %d\n",
			pconf->lcd_control.lvds_config->lane_reverse);
	}
}

static int lcd_init_load_from_dts(struct lcd_config_s *pconf,
		struct device *dev)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	/* lock pinmux if lcd in on */
	if (lcd_drv->lcd_status & LCD_STATUS_IF_ON) {
		switch (pconf->lcd_basic.lcd_type) {
		case LCD_VBYONE:
			lcd_vbyone_pinmux_set(1);
			break;
		case LCD_P2P:
			lcd_tcon_pinmux_set(1);
			break;
		default:
			break;
		}
	}

	return ret;
}

static int lcd_config_load_from_dts(struct lcd_config_s *pconf,
		struct device *dev)
{
	int ret = 0;
	const char *str;
	unsigned int para[10], val;
	struct device_node *child;
	struct lvds_config_s *lvdsconf;
	struct vbyone_config_s *vx1_conf;
	struct mlvds_config_s *mlvds_conf;
	struct p2p_config_s *p2p_conf;

	child = of_get_child_by_name(dev->of_node, pconf->lcd_propname);
	if (child == NULL) {
		LCDERR("failed to get %s\n", pconf->lcd_propname);
		return -1;
	}

	ret = of_property_read_string(child, "model_name", &str);
	if (ret) {
		LCDERR("failed to get model_name\n");
		strncpy(pconf->lcd_basic.model_name, pconf->lcd_propname,
			MOD_LEN_MAX);
	} else {
		strncpy(pconf->lcd_basic.model_name, str, MOD_LEN_MAX);
	}
	/* ensure string ending */
	pconf->lcd_basic.model_name[MOD_LEN_MAX-1] = '\0';

	ret = of_property_read_string(child, "interface", &str);
	if (ret) {
		LCDERR("failed to get interface\n");
		str = "invalid";
	}
	pconf->lcd_basic.lcd_type = lcd_type_str_to_type(str);

	ret = of_property_read_u32_array(child, "basic_setting", &para[0], 7);
	if (ret) {
		LCDERR("failed to get basic_setting\n");
	} else {
		pconf->lcd_basic.h_active = para[0];
		pconf->lcd_basic.v_active = para[1];
		pconf->lcd_basic.h_period = para[2];
		pconf->lcd_basic.v_period = para[3];
		pconf->lcd_basic.lcd_bits = para[4];
		pconf->lcd_basic.screen_width = para[5];
		pconf->lcd_basic.screen_height = para[6];
	}
	ret = of_property_read_u32_array(child, "range_setting", &para[0], 6);
	if (ret) {
		LCDPR("no range_setting\n");
		pconf->lcd_basic.h_period_min = pconf->lcd_basic.h_period;
		pconf->lcd_basic.h_period_max = pconf->lcd_basic.h_period;
		pconf->lcd_basic.v_period_min = pconf->lcd_basic.v_period;
		pconf->lcd_basic.v_period_max = pconf->lcd_basic.v_period;
		pconf->lcd_basic.lcd_clk_min = 0;
		pconf->lcd_basic.lcd_clk_max = 0;
	} else {
		pconf->lcd_basic.h_period_min = para[0];
		pconf->lcd_basic.h_period_max = para[1];
		pconf->lcd_basic.v_period_min = para[2];
		pconf->lcd_basic.v_period_max = para[3];
		pconf->lcd_basic.lcd_clk_min = para[4];
		pconf->lcd_basic.lcd_clk_max = para[5];
	}

	ret = of_property_read_u32_array(child, "lcd_timing", &para[0], 6);
	if (ret) {
		LCDERR("failed to get lcd_timing\n");
	} else {
		pconf->lcd_timing.hsync_width = (unsigned short)(para[0]);
		pconf->lcd_timing.hsync_bp = (unsigned short)(para[1]);
		pconf->lcd_timing.hsync_pol = (unsigned short)(para[2]);
		pconf->lcd_timing.vsync_width = (unsigned short)(para[3]);
		pconf->lcd_timing.vsync_bp = (unsigned short)(para[4]);
		pconf->lcd_timing.vsync_pol = (unsigned short)(para[5]);
	}

	ret = of_property_read_u32_array(child, "clk_attr", &para[0], 4);
	if (ret) {
		LCDERR("failed to get clk_attr\n");
		pconf->lcd_timing.fr_adjust_type = 0;
		pconf->lcd_timing.ss_level = 0;
		pconf->lcd_timing.clk_auto = 1;
		pconf->lcd_timing.lcd_clk = 60;
	} else {
		pconf->lcd_timing.fr_adjust_type = (unsigned char)(para[0]);
		pconf->lcd_timing.ss_level = para[1];
		pconf->lcd_timing.clk_auto = (unsigned char)(para[2]);
		pconf->lcd_timing.lcd_clk = para[3];
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		lvdsconf = pconf->lcd_control.lvds_config;
		ret = of_property_read_u32_array(child, "lvds_attr",
			&para[0], 5);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDERR("load 4 parameters in lvds_attr\n");
			ret = of_property_read_u32_array(child, "lvds_attr",
				&para[0], 4);
			if (ret) {
				if (lcd_debug_print_flag)
					LCDPR("failed to get lvds_attr\n");
			} else {
				lvdsconf->lvds_repack = para[0];
				lvdsconf->dual_port = para[1];
				lvdsconf->pn_swap = para[2];
				lvdsconf->port_swap = para[3];
			}
		} else {
			lvdsconf->lvds_repack = para[0];
			lvdsconf->dual_port = para[1];
			lvdsconf->pn_swap = para[2];
			lvdsconf->port_swap = para[3];
			lvdsconf->lane_reverse = para[4];
		}
		ret = of_property_read_u32_array(child, "phy_attr",
			&para[0], 4);
		if (ret) { /* old parameters */
			if (lcd_debug_print_flag)
				LCDPR("load 2 parameters in phy_attr\n");
			ret = of_property_read_u32_array(child, "phy_attr",
				&para[0], 2);
			if (ret) {
				if (lcd_debug_print_flag)
					LCDPR("failed to get phy_attr\n");
			} else {
				lvdsconf->phy_vswing = para[0];
				lvdsconf->phy_preem = para[1];
				lvdsconf->phy_clk_vswing = 0;
				lvdsconf->phy_clk_preem = 0;
				if (lcd_debug_print_flag) {
					LCDPR("phy vswing=0x%x, preem=0x%x\n",
						lvdsconf->phy_vswing,
						lvdsconf->phy_preem);
				}
			}
		} else {
			lvdsconf->phy_vswing = para[0];
			lvdsconf->phy_preem = para[1];
			lvdsconf->phy_clk_vswing = para[2];
			lvdsconf->phy_clk_preem = para[3];
			if (lcd_debug_print_flag) {
				LCDPR("phy vswing=0x%x, preemphasis=0x%x\n",
					lvdsconf->phy_vswing,
					lvdsconf->phy_preem);
				LCDPR("phy_clk vswing=0x%x, preemphasis=0x%x\n",
					lvdsconf->phy_clk_vswing,
					lvdsconf->phy_clk_preem);
			}
		}
		break;
	case LCD_VBYONE:
		vx1_conf = pconf->lcd_control.vbyone_config;
		ret = of_property_read_u32_array(child, "vbyone_attr",
			&para[0], 4);
		if (ret) {
			LCDERR("failed to get vbyone_attr\n");
		} else {
			vx1_conf->lane_count = para[0];
			vx1_conf->region_num = para[1];
			vx1_conf->byte_mode = para[2];
			vx1_conf->color_fmt = para[3];
		}
		ret = of_property_read_u32_array(child, "vbyone_intr_enable",
			&para[0], 2);
		if (ret) {
			LCDERR("failed to get vbyone_intr_enable\n");
		} else {
			vx1_conf->intr_en = para[0];
			vx1_conf->vsync_intr_en = para[1];
		}
		ret = of_property_read_u32_array(child, "phy_attr",
			&para[0], 2);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDPR("failed to get phy_attr\n");
		} else {
			vx1_conf->phy_vswing = para[0];
			vx1_conf->phy_preem = para[1];
			if (lcd_debug_print_flag) {
				LCDPR("phy vswing=0x%x, preemphasis=0x%x\n",
				vx1_conf->phy_vswing, vx1_conf->phy_preem);
			}
		}
		ret = of_property_read_u32(child, "vbyone_ctrl_flag", &val);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDPR("failed to get vbyone_ctrl_flag\n");
		} else {
			vx1_conf->ctrl_flag = val;
			LCDPR("vbyone ctrl_flag=0x%x\n", vx1_conf->ctrl_flag);
		}
		if (vx1_conf->ctrl_flag & 0x7) {
			ret = of_property_read_u32_array(child,
				"vbyone_ctrl_timing", &para[0], 3);
			if (ret) {
				LCDPR("failed to get vbyone_ctrl_timing\n");
			} else {
				vx1_conf->power_on_reset_delay = para[0];
				vx1_conf->hpd_data_delay = para[1];
				vx1_conf->cdr_training_hold = para[2];
			}
			if (lcd_debug_print_flag) {
				LCDPR("power_on_reset_delay: %d\n",
					vx1_conf->power_on_reset_delay);
				LCDPR("hpd_data_delay: %d\n",
					vx1_conf->hpd_data_delay);
				LCDPR("cdr_training_hold: %d\n",
					vx1_conf->cdr_training_hold);
			}
		}
		ret = of_property_read_u32_array(child, "hw_filter",
			&para[0], 2);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDPR("failed to get hw_filter\n");
		} else {
			vx1_conf->hw_filter_time = para[0];
			vx1_conf->hw_filter_cnt = para[1];
			if (lcd_debug_print_flag) {
				LCDPR("vbyone hw_filter=0x%x 0x%x\n",
					vx1_conf->hw_filter_time,
					vx1_conf->hw_filter_cnt);
			}
		}
		break;
	case LCD_MLVDS:
		mlvds_conf = pconf->lcd_control.mlvds_config;
		ret = of_property_read_u32_array(child, "minilvds_attr",
			&para[0], 6);
		if (ret) {
			LCDERR("failed to get minilvds_attr\n");
		} else {
			mlvds_conf->channel_num = para[0];
			mlvds_conf->channel_sel0 = para[1];
			mlvds_conf->channel_sel1 = para[2];
			mlvds_conf->clk_phase = para[3];
			mlvds_conf->pn_swap = para[4];
			mlvds_conf->bit_swap = para[5];
		}
		ret = of_property_read_u32_array(child, "phy_attr",
			&para[0], 2);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDPR("failed to get phy_attr\n");
		} else {
			mlvds_conf->phy_vswing = para[0];
			mlvds_conf->phy_preem = para[1];
			if (lcd_debug_print_flag) {
				LCDPR("phy vswing=0x%x, preem=0x%x\n",
					mlvds_conf->phy_vswing,
					mlvds_conf->phy_preem);
			}
		}
		break;
	case LCD_P2P:
		p2p_conf = pconf->lcd_control.p2p_config;
		ret = of_property_read_u32_array(child, "p2p_attr",
			&para[0], 6);
		if (ret) {
			LCDERR("failed to get p2p_attr\n");
		} else {
			p2p_conf->p2p_type = para[0];
			p2p_conf->lane_num = para[1];
			p2p_conf->channel_sel0 = para[2];
			p2p_conf->channel_sel1 = para[3];
			p2p_conf->pn_swap = para[4];
			p2p_conf->bit_swap = para[5];
		}
		ret = of_property_read_u32_array(child, "phy_attr",
			&para[0], 2);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDPR("failed to get phy_attr\n");
		} else {
			p2p_conf->phy_vswing = para[0];
			p2p_conf->phy_preem = para[1];
			if (lcd_debug_print_flag) {
				LCDPR("phy vswing=0x%x, preem=0x%x\n",
					p2p_conf->phy_vswing,
					p2p_conf->phy_preem);
			}
		}
		break;
	default:
		LCDERR("invalid lcd type\n");
		break;
	}

	lcd_vlock_param_load_from_dts(pconf, child);
	ret = lcd_power_load_from_dts(pconf, child);

	return ret;
}

static int lcd_config_load_from_unifykey(struct lcd_config_s *pconf)
{
	unsigned char *para;
	int key_len, len;
	unsigned char *p;
	const char *str;
	struct aml_lcd_unifykey_header_s lcd_header;
	struct lvds_config_s *lvdsconf = pconf->lcd_control.lvds_config;
	struct vbyone_config_s *vx1_conf = pconf->lcd_control.vbyone_config;
	struct mlvds_config_s *mlvds_conf = pconf->lcd_control.mlvds_config;
	struct p2p_config_s *p2p_conf = pconf->lcd_control.p2p_config;
	int ret;

	para = kmalloc((sizeof(unsigned char) * LCD_UKEY_LCD_SIZE), GFP_KERNEL);
	if (!para) {
		LCDERR("%s: Not enough memory\n", __func__);
		return -1;
	}
	key_len = LCD_UKEY_LCD_SIZE;
	memset(para, 0, (sizeof(unsigned char) * key_len));
	ret = lcd_unifykey_get("lcd", para, &key_len);
	if (ret < 0) {
		kfree(para);
		return -1;
	}

	/* step 1: check header */
	len = LCD_UKEY_HEAD_SIZE;
	ret = lcd_unifykey_len_check(key_len, len);
	if (ret < 0) {
		LCDERR("unifykey header length is incorrect\n");
		kfree(para);
		return -1;
	}

	lcd_unifykey_header_check(para, &lcd_header);
	LCDPR("unifykey version: 0x%04x\n", lcd_header.version);
	switch (lcd_header.version) {
	case 2:
		len = LCD_UKEY_DATA_LEN_V2; /*10+36+18+31+20+44+10*/
		break;
	default:
		len = LCD_UKEY_DATA_LEN_V1; /*10+36+18+31+20*/
		break;
	}
	if (lcd_debug_print_flag) {
		LCDPR("unifykey header:\n");
		LCDPR("crc32             = 0x%08x\n", lcd_header.crc32);
		LCDPR("data_len          = %d\n", lcd_header.data_len);
		LCDPR("reserved          = 0x%04x\n", lcd_header.reserved);
	}

	/* step 2: check lcd parameters */
	ret = lcd_unifykey_len_check(key_len, len);
	if (ret < 0) {
		LCDERR("unifykey parameters length is incorrect\n");
		kfree(para);
		return -1;
	}

	/* panel_type update */
	sprintf(pconf->lcd_propname, "%s", "unifykey");

	/* basic: 36byte */
	p = para;
	str = (const char *)(p + LCD_UKEY_HEAD_SIZE);
	strncpy(pconf->lcd_basic.model_name, str, MOD_LEN_MAX);
	/* ensure string ending */
	pconf->lcd_basic.model_name[MOD_LEN_MAX-1] = '\0';
	pconf->lcd_basic.lcd_type = *(p + LCD_UKEY_INTERFACE);
	pconf->lcd_basic.lcd_bits = *(p + LCD_UKEY_LCD_BITS);
	pconf->lcd_basic.screen_width = (*(p + LCD_UKEY_SCREEN_WIDTH) |
		((*(p + LCD_UKEY_SCREEN_WIDTH + 1)) << 8));
	pconf->lcd_basic.screen_height = (*(p + LCD_UKEY_SCREEN_HEIGHT) |
		((*(p + LCD_UKEY_SCREEN_HEIGHT + 1)) << 8));

	/* timing: 18byte */
	pconf->lcd_basic.h_active = (*(p + LCD_UKEY_H_ACTIVE) |
		((*(p + LCD_UKEY_H_ACTIVE + 1)) << 8));
	pconf->lcd_basic.v_active = (*(p + LCD_UKEY_V_ACTIVE)) |
		((*(p + LCD_UKEY_V_ACTIVE + 1)) << 8);
	pconf->lcd_basic.h_period = (*(p + LCD_UKEY_H_PERIOD)) |
		((*(p + LCD_UKEY_H_PERIOD + 1)) << 8);
	pconf->lcd_basic.v_period = (*(p + LCD_UKEY_V_PERIOD)) |
		((*(p + LCD_UKEY_V_PERIOD + 1)) << 8);
	pconf->lcd_timing.hsync_width = (*(p + LCD_UKEY_HS_WIDTH) |
		((*(p + LCD_UKEY_HS_WIDTH + 1)) << 8));
	pconf->lcd_timing.hsync_bp = (*(p + LCD_UKEY_HS_BP) |
		((*(p + LCD_UKEY_HS_BP + 1)) << 8));
	pconf->lcd_timing.hsync_pol = *(p + LCD_UKEY_HS_POL);
	pconf->lcd_timing.vsync_width = (*(p + LCD_UKEY_VS_WIDTH) |
		((*(p + LCD_UKEY_VS_WIDTH + 1)) << 8));
	pconf->lcd_timing.vsync_bp = (*(p + LCD_UKEY_VS_BP) |
		((*(p + LCD_UKEY_VS_BP + 1)) << 8));
	pconf->lcd_timing.vsync_pol = *(p + LCD_UKEY_VS_POL);

	/* customer: 31byte */
	pconf->lcd_timing.fr_adjust_type = *(p + LCD_UKEY_FR_ADJ_TYPE);
	pconf->lcd_timing.ss_level = *(p + LCD_UKEY_SS_LEVEL);
	pconf->lcd_timing.clk_auto = *(p + LCD_UKEY_CLK_AUTO_GEN);
	pconf->lcd_timing.lcd_clk = (*(p + LCD_UKEY_PCLK) |
		((*(p + LCD_UKEY_PCLK + 1)) << 8) |
		((*(p + LCD_UKEY_PCLK + 2)) << 16) |
		((*(p + LCD_UKEY_PCLK + 3)) << 24));
	pconf->lcd_basic.h_period_min = (*(p + LCD_UKEY_H_PERIOD_MIN) |
		((*(p + LCD_UKEY_H_PERIOD_MIN + 1)) << 8));
	pconf->lcd_basic.h_period_max = (*(p + LCD_UKEY_H_PERIOD_MAX) |
		((*(p + LCD_UKEY_H_PERIOD_MAX + 1)) << 8));
	pconf->lcd_basic.v_period_min = (*(p + LCD_UKEY_V_PERIOD_MIN) |
		((*(p + LCD_UKEY_V_PERIOD_MIN + 1)) << 8));
	pconf->lcd_basic.v_period_max = (*(p + LCD_UKEY_V_PERIOD_MAX) |
		((*(p + LCD_UKEY_V_PERIOD_MAX + 1)) << 8));
	pconf->lcd_basic.lcd_clk_min = (*(p + LCD_UKEY_PCLK_MIN) |
		((*(p + LCD_UKEY_PCLK_MIN + 1)) << 8) |
		((*(p + LCD_UKEY_PCLK_MIN + 2)) << 16) |
		((*(p + LCD_UKEY_PCLK_MIN + 3)) << 24));
	pconf->lcd_basic.lcd_clk_max = (*(p + LCD_UKEY_PCLK_MAX) |
		((*(p + LCD_UKEY_PCLK_MAX + 1)) << 8) |
		((*(p + LCD_UKEY_PCLK_MAX + 2)) << 16) |
		((*(p + LCD_UKEY_PCLK_MAX + 3)) << 24));

	/* interface: 20byte */
	if (pconf->lcd_basic.lcd_type == LCD_VBYONE) {
		if (lcd_header.version == 2) {
			vx1_conf->lane_count = (*(p + LCD_UKEY_IF_ATTR_0) |
				((*(p + LCD_UKEY_IF_ATTR_0 + 1)) << 8)) & 0xff;
			vx1_conf->region_num = (*(p + LCD_UKEY_IF_ATTR_1) |
				((*(p + LCD_UKEY_IF_ATTR_1 + 1)) << 8)) & 0xff;
			vx1_conf->byte_mode = (*(p + LCD_UKEY_IF_ATTR_2) |
				((*(p + LCD_UKEY_IF_ATTR_2 + 1)) << 8)) & 0xff;
			vx1_conf->color_fmt = (*(p + LCD_UKEY_IF_ATTR_3) |
				((*(p + LCD_UKEY_IF_ATTR_3 + 1)) << 8)) & 0xff;
			vx1_conf->intr_en = (*(p + LCD_UKEY_IF_ATTR_4) |
				((*(p + LCD_UKEY_IF_ATTR_4 + 1)) << 8)) & 0xff;
			vx1_conf->vsync_intr_en = (*(p + LCD_UKEY_IF_ATTR_5) |
				((*(p + LCD_UKEY_IF_ATTR_5 + 1)) << 8)) & 0xff;
		} else {
			vx1_conf->lane_count = (*(p + LCD_UKEY_IF_ATTR_0) |
				((*(p + LCD_UKEY_IF_ATTR_0 + 1)) << 8)) & 0xff;
			vx1_conf->region_num = (*(p + LCD_UKEY_IF_ATTR_1) |
				((*(p + LCD_UKEY_IF_ATTR_1 + 1)) << 8)) & 0xff;
			vx1_conf->byte_mode = (*(p + LCD_UKEY_IF_ATTR_2) |
				((*(p + LCD_UKEY_IF_ATTR_2 + 1)) << 8)) & 0xff;
			vx1_conf->color_fmt = (*(p + LCD_UKEY_IF_ATTR_3) |
				((*(p + LCD_UKEY_IF_ATTR_3 + 1)) << 8)) & 0xff;
			vx1_conf->phy_vswing = (*(p + LCD_UKEY_IF_ATTR_4) |
				((*(p + LCD_UKEY_IF_ATTR_4 + 1)) << 8)) & 0xff;
			vx1_conf->phy_preem = (*(p + LCD_UKEY_IF_ATTR_5) |
				((*(p + LCD_UKEY_IF_ATTR_5 + 1)) << 8)) & 0xff;
			vx1_conf->intr_en = (*(p + LCD_UKEY_IF_ATTR_6) |
				((*(p + LCD_UKEY_IF_ATTR_6 + 1)) << 8)) & 0xff;
			vx1_conf->vsync_intr_en = (*(p + LCD_UKEY_IF_ATTR_7) |
				((*(p + LCD_UKEY_IF_ATTR_7 + 1)) << 8)) & 0xff;
			vx1_conf->hw_filter_time = *(p + LCD_UKEY_IF_ATTR_8) |
				((*(p + LCD_UKEY_IF_ATTR_8 + 1)) << 8);
			vx1_conf->hw_filter_cnt = *(p + LCD_UKEY_IF_ATTR_9) |
				((*(p + LCD_UKEY_IF_ATTR_9 + 1)) << 8);
		}
	} else if (pconf->lcd_basic.lcd_type == LCD_LVDS) {
		if (lcd_header.version == 2) {
			lvdsconf->lvds_repack = (*(p + LCD_UKEY_IF_ATTR_0) |
				((*(p + LCD_UKEY_IF_ATTR_0 + 1)) << 8)) & 0xff;
			lvdsconf->dual_port = (*(p + LCD_UKEY_IF_ATTR_1) |
				((*(p + LCD_UKEY_IF_ATTR_1 + 1)) << 8)) & 0xff;
			lvdsconf->pn_swap = (*(p + LCD_UKEY_IF_ATTR_2) |
				((*(p + LCD_UKEY_IF_ATTR_2 + 1)) << 8)) & 0xff;
			lvdsconf->port_swap = (*(p + LCD_UKEY_IF_ATTR_3) |
				((*(p + LCD_UKEY_IF_ATTR_3 + 1)) << 8)) & 0xff;
			lvdsconf->lane_reverse = (*(p + LCD_UKEY_IF_ATTR_4) |
				((*(p + LCD_UKEY_IF_ATTR_4 + 1)) << 8)) & 0xff;
		} else {
			lvdsconf->lvds_repack = (*(p + LCD_UKEY_IF_ATTR_0) |
				((*(p + LCD_UKEY_IF_ATTR_0 + 1)) << 8)) & 0xff;
			lvdsconf->dual_port = (*(p + LCD_UKEY_IF_ATTR_1) |
				((*(p + LCD_UKEY_IF_ATTR_1 + 1)) << 8)) & 0xff;
			lvdsconf->pn_swap = (*(p + LCD_UKEY_IF_ATTR_2) |
				((*(p + LCD_UKEY_IF_ATTR_2 + 1)) << 8)) & 0xff;
			lvdsconf->port_swap = (*(p + LCD_UKEY_IF_ATTR_3) |
				((*(p + LCD_UKEY_IF_ATTR_3 + 1)) << 8)) & 0xff;
			lvdsconf->phy_vswing = (*(p + LCD_UKEY_IF_ATTR_4) |
				((*(p + LCD_UKEY_IF_ATTR_4 + 1)) << 8)) & 0xff;
			lvdsconf->phy_preem = (*(p + LCD_UKEY_IF_ATTR_5) |
				((*(p + LCD_UKEY_IF_ATTR_5 + 1)) << 8)) & 0xff;
			lvdsconf->phy_clk_vswing = (*(p + LCD_UKEY_IF_ATTR_6) |
				((*(p + LCD_UKEY_IF_ATTR_6 + 1)) << 8)) & 0xff;
			lvdsconf->phy_clk_preem = (*(p + LCD_UKEY_IF_ATTR_7) |
				((*(p + LCD_UKEY_IF_ATTR_7 + 1)) << 8)) & 0xff;
			lvdsconf->lane_reverse = 0;
		}
	} else if (pconf->lcd_basic.lcd_type == LCD_MLVDS) {
		mlvds_conf->channel_num = (*(p + LCD_UKEY_IF_ATTR_0) |
			((*(p + LCD_UKEY_IF_ATTR_0 + 1)) << 8)) & 0xff;
		mlvds_conf->channel_sel0 = (*(p + LCD_UKEY_IF_ATTR_1) |
			((*(p + LCD_UKEY_IF_ATTR_1 + 1)) << 8) |
			(*(p + LCD_UKEY_IF_ATTR_2) << 16) |
			((*(p + LCD_UKEY_IF_ATTR_2 + 1)) << 24));
		mlvds_conf->channel_sel1 = (*(p + LCD_UKEY_IF_ATTR_3) |
			((*(p + LCD_UKEY_IF_ATTR_3 + 1)) << 8) |
			(*(p + LCD_UKEY_IF_ATTR_4) << 16) |
			((*(p + LCD_UKEY_IF_ATTR_4 + 1)) << 24));
		mlvds_conf->clk_phase = (*(p + LCD_UKEY_IF_ATTR_5) |
			((*(p + LCD_UKEY_IF_ATTR_5 + 1)) << 8));
		mlvds_conf->pn_swap = (*(p + LCD_UKEY_IF_ATTR_6) |
			((*(p + LCD_UKEY_IF_ATTR_6 + 1)) << 8)) & 0xff;
		mlvds_conf->bit_swap = (*(p + LCD_UKEY_IF_ATTR_7) |
			((*(p + LCD_UKEY_IF_ATTR_7 + 1)) << 8)) & 0xff;
		mlvds_conf->phy_vswing = (*(p + LCD_UKEY_IF_ATTR_8) |
			((*(p + LCD_UKEY_IF_ATTR_8 + 1)) << 8)) & 0xff;
		mlvds_conf->phy_preem = (*(p + LCD_UKEY_IF_ATTR_9) |
			((*(p + LCD_UKEY_IF_ATTR_9 + 1)) << 8)) & 0xff;
	} else if (pconf->lcd_basic.lcd_type == LCD_P2P) {
		p2p_conf->p2p_type = (*(p + LCD_UKEY_IF_ATTR_0) |
			((*(p + LCD_UKEY_IF_ATTR_0 + 1)) << 8));
		p2p_conf->lane_num = (*(p + LCD_UKEY_IF_ATTR_1) |
			((*(p + LCD_UKEY_IF_ATTR_1 + 1)) << 8));
		p2p_conf->channel_sel0 = (*(p + LCD_UKEY_IF_ATTR_2) |
			((*(p + LCD_UKEY_IF_ATTR_2 + 1)) << 8) |
			(*(p + LCD_UKEY_IF_ATTR_3) << 16) |
			((*(p + LCD_UKEY_IF_ATTR_3 + 1)) << 24));
		p2p_conf->channel_sel1 = (*(p + LCD_UKEY_IF_ATTR_4) |
			((*(p + LCD_UKEY_IF_ATTR_4 + 1)) << 8) |
			(*(p + LCD_UKEY_IF_ATTR_5) << 16) |
			((*(p + LCD_UKEY_IF_ATTR_5 + 1)) << 24));
		p2p_conf->pn_swap = (*(p + LCD_UKEY_IF_ATTR_6) |
			((*(p + LCD_UKEY_IF_ATTR_6 + 1)) << 8));
		p2p_conf->bit_swap = (*(p + LCD_UKEY_IF_ATTR_7) |
			((*(p + LCD_UKEY_IF_ATTR_7 + 1)) << 8));
		p2p_conf->phy_vswing = (*(p + LCD_UKEY_IF_ATTR_8) |
			((*(p + LCD_UKEY_IF_ATTR_8 + 1)) << 8));
		p2p_conf->phy_preem = (*(p + LCD_UKEY_IF_ATTR_9) |
			((*(p + LCD_UKEY_IF_ATTR_9 + 1)) << 8));
	} else
		LCDERR("unsupport lcd_type: %d\n", pconf->lcd_basic.lcd_type);

	if (lcd_header.version == 2) {
		/* ctrl: 44byte */ /* v2 */
		if (pconf->lcd_basic.lcd_type == LCD_VBYONE) {
			vx1_conf->ctrl_flag = (*(p + LCD_UKEY_CTRL_FLAG) |
				((*(p + LCD_UKEY_CTRL_FLAG + 1)) << 8) |
				((*(p + LCD_UKEY_CTRL_FLAG + 2)) << 16) |
				((*(p + LCD_UKEY_CTRL_FLAG + 3)) << 24));
			vx1_conf->power_on_reset_delay =
				(*(p + LCD_UKEY_CTRL_ATTR_0) |
				((*(p + LCD_UKEY_CTRL_ATTR_0 + 1)) << 8));
			vx1_conf->hpd_data_delay =
				(*(p + LCD_UKEY_CTRL_ATTR_1) |
				((*(p  + LCD_UKEY_CTRL_ATTR_1 + 1)) << 8));
			vx1_conf->cdr_training_hold =
				(*(p + LCD_UKEY_CTRL_ATTR_2) |
				((*(p + LCD_UKEY_CTRL_ATTR_2 + 1)) << 8));
		}

		/* phy: 10byte */ /* v2 */
		if (pconf->lcd_basic.lcd_type == LCD_VBYONE) {
			vx1_conf->phy_vswing = *(p + LCD_UKEY_PHY_ATTR_0);
			vx1_conf->phy_preem = *(p + LCD_UKEY_PHY_ATTR_1);
		} else if (pconf->lcd_basic.lcd_type == LCD_LVDS) {
			lvdsconf->phy_vswing = *(p + LCD_UKEY_PHY_ATTR_0);
			lvdsconf->phy_preem = *(p + LCD_UKEY_PHY_ATTR_1);
			lvdsconf->phy_clk_vswing = *(p + LCD_UKEY_PHY_ATTR_2);
			lvdsconf->phy_clk_preem = *(p + LCD_UKEY_PHY_ATTR_3);
		}
	}

	lcd_vlock_param_load_from_unifykey(pconf, para);

	/* step 3: check power sequence */
	ret = lcd_power_load_from_unifykey(pconf, para, key_len, len);
	if (ret < 0) {
		kfree(para);
		return -1;
	}

	kfree(para);
	return 0;
}

static void lcd_vmode_init(struct lcd_config_s *pconf)
{
	char *mode;
	enum vmode_e vmode;

	mode = get_vout_mode_uboot();
	LCDPR("%s mode: %s\n", __func__, mode);
	vmode = lcd_validate_vmode(mode);
	if (vmode >= VMODE_MAX) {
		LCDERR("%s: invalid vout_init_mode: %s\n", __func__, mode);
		vmode = VMODE_LCD;
	}
	lcd_vmode_vinfo_update(vmode & VMODE_MODE_BIT_MASK);
}

static void lcd_config_init(struct lcd_config_s *pconf)
{
	struct lcd_clk_config_s *cconf = get_lcd_clk_config();
	unsigned int temp;

	if (pconf->lcd_timing.lcd_clk == 0) /* default 0 for 60hz */
		pconf->lcd_timing.lcd_clk = 60;
	else
		LCDPR("custome clk: %d\n", pconf->lcd_timing.lcd_clk);
	temp = pconf->lcd_timing.lcd_clk;
	if (temp < 200) { /* regard as frame_rate */
		pconf->lcd_timing.lcd_clk = temp * pconf->lcd_basic.h_period *
			pconf->lcd_basic.v_period;
	} else { /* regard as pixel clock */
		pconf->lcd_timing.lcd_clk = temp;
	}

	pconf->lcd_timing.lcd_clk_dft = pconf->lcd_timing.lcd_clk;
	pconf->lcd_timing.h_period_dft = pconf->lcd_basic.h_period;
	pconf->lcd_timing.v_period_dft = pconf->lcd_basic.v_period;
	/* before vmode_init to avoid period changing */
	lcd_timing_init_config(pconf);
	lcd_vmode_init(pconf);
	lcd_tv_config_update(pconf);
	lcd_clk_generate_parameter(pconf);
	if (cconf->data) {
		temp = pconf->lcd_timing.ss_level & 0xff;
		cconf->ss_level = (temp >= cconf->data->ss_level_max) ?
					0 : temp;
		temp = (pconf->lcd_timing.ss_level >> 8) & 0xff;
		temp = (temp >> LCD_CLK_SS_BIT_FREQ) & 0xf;
		cconf->ss_freq = (temp >= cconf->data->ss_freq_max) ?
					0 : temp;
		temp = (pconf->lcd_timing.ss_level >> 8) & 0xff;
		temp = (temp >> LCD_CLK_SS_BIT_MODE) & 0xf;
		cconf->ss_mode = (temp >= cconf->data->ss_mode_max) ?
					0 : temp;
	} else {
		LCDERR("%s: clk config data is null\n", __func__);
		cconf->ss_level = 0;
		cconf->ss_freq = 0;
		cconf->ss_mode = 0;
	}
}

static int lcd_get_config(struct lcd_config_s *pconf, struct device *dev)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int load_id = 0;
	int ret;

	if (dev->of_node == NULL) {
		LCDERR("dev of_node is null\n");
		return -1;
	}
	if (lcd_drv->lcd_key_valid) {
		ret = lcd_unifykey_check("lcd");
		if (ret < 0)
			load_id = 0;
		else
			load_id = 1;
	}
	if (load_id) {
		if (lcd_debug_print_flag)
			LCDPR("%s from unifykey\n", __func__);
		lcd_drv->lcd_config_load = 1;
		lcd_config_load_from_unifykey(pconf);
	} else {
		if (lcd_debug_print_flag)
			LCDPR("%s from dts\n", __func__);
		lcd_drv->lcd_config_load = 0;
		lcd_config_load_from_dts(pconf, dev);
	}
	lcd_init_load_from_dts(pconf, dev);
	lcd_config_print(pconf);
	lcd_config_init(pconf);

	return 0;
}

/* ************************************************** *
 * lcd notify
 * **************************************************
 */
/* sync_duration is real_value * 100 */
static void lcd_set_vinfo(unsigned int sync_duration)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	LCDPR("%s: sync_duration=%d\n", __func__, sync_duration);

	lcd_vout_notify_mode_change_pre();

	/* update vinfo */
	lcd_drv->lcd_info->sync_duration_num = sync_duration;
	lcd_drv->lcd_info->sync_duration_den = 100;

	/* update interface timing */
	lcd_tv_config_update(lcd_drv->lcd_config);
#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(
		lcd_drv->lcd_config->lcd_timing.lcd_clk, VPU_VENCL);
#endif

	if (lcd_drv->lcd_config->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_vbyone_interrupt_enable(0);
	/* change clk parameter */
	lcd_clk_change(lcd_drv->lcd_config);
	lcd_venc_change(lcd_drv->lcd_config);
	if (lcd_drv->lcd_config->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_vbyone_wait_stable();

	lcd_vout_notify_mode_change();
}

static int lcd_frame_rate_adjust_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	unsigned int *sync_duration;

	/* LCDPR("%s: 0x%lx\n", __func__, event); */
	if ((event & LCD_EVENT_FRAME_RATE_ADJUST) == 0)
		return NOTIFY_DONE;

	if (data == NULL) {
		LCDERR("%s: data is NULL\n", __func__);
		return NOTIFY_DONE;
	}
	sync_duration = (unsigned int *)data;
	lcd_set_vinfo(*sync_duration);

	return NOTIFY_OK;
}

static struct notifier_block lcd_frame_rate_adjust_nb = {
	.notifier_call = lcd_frame_rate_adjust_notifier,
};

/* ************************************************** *
 * lcd tv
 * **************************************************
 */
int lcd_tv_probe(struct device *dev)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int ret;

	memset(lcd_output_name, 0, sizeof(lcd_output_name));
	lcd_drv->driver_init_pre = lcd_tv_driver_init_pre;
	lcd_drv->driver_disable_post = lcd_tv_driver_disable_post;
	lcd_drv->driver_init = lcd_tv_driver_init;
	lcd_drv->driver_disable = lcd_tv_driver_disable;
	lcd_drv->driver_change = lcd_tv_driver_change;

	lcd_get_config(lcd_drv->lcd_config, dev);

	switch (lcd_drv->lcd_config->lcd_basic.lcd_type) {
	case LCD_VBYONE:
		lcd_vbyone_interrupt_up();
		break;
	default:
		break;
	}

	ret = aml_lcd_notifier_register(&lcd_frame_rate_adjust_nb);
	if (ret)
		LCDERR("register lcd_frame_rate_adjust_nb failed\n");

	return 0;
}

int lcd_tv_remove(struct device *dev)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	aml_lcd_notifier_unregister(&lcd_frame_rate_adjust_nb);
	switch (lcd_drv->lcd_config->lcd_basic.lcd_type) {
	case LCD_VBYONE:
		lcd_vbyone_interrupt_down();
		break;
	default:
		break;
	}

	kfree(lcd_drv->lcd_info);
	lcd_drv->lcd_info = NULL;
	return 0;
}

