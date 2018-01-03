/*
 * drivers/amlogic/display/logo/logo.c
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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ctype.h>

/* Amlogic Headers */
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

/* Local Headers */
#include <osd/osd_hw.h>


#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define LOGO_DEV_OSD0 0x0
#define LOGO_DEV_OSD1 0x1
#define LOGO_DEBUG    0x1001
#define LOGO_LOADED   0x1002

static enum vmode_e hdmimode = VMODE_1080P;
static enum vmode_e cvbsmode = VMODE_480CVBS;
static enum vmode_e last_mode = VMODE_MAX;

struct delayed_work logo_work;
static DEFINE_MUTEX(logo_lock);

struct para_pair_s {
	char *name;
	int value;
};

static struct para_pair_s logo_args[] = {
	{"osd0", LOGO_DEV_OSD0},
	{"osd1", LOGO_DEV_OSD1},
	{"debug", LOGO_DEBUG},
	{"loaded", LOGO_LOADED},
};

/* FIXME : need to cover all of resolutions using general table */
static struct para_pair_s mode_infos[] = {
	{"640x480p60hz", TVMODE_640x480p60hz},
	{"800x480p60hz", TVMODE_800x480p60hz},
	{"480x800p60hz", TVMODE_480x800p60hz},
	{"800x600p60hz", TVMODE_800x600p60hz},
	{"1024x600p60hz", TVMODE_1024x600p60hz},
	{"1024x768p60hz", TVMODE_1024x768p60hz},
	{"1280x800p60hz", TVMODE_1280x800p60hz},
	{"1280x1024p60hz", TVMODE_1280x1024p60hz},
	{"1360x768p60hz", TVMODE_1360x768p60hz},
	{"1366x768p60hz", TVMODE_1366x768p60hz},
	{"1440x900p60hz", TVMODE_1440x900p60hz},
	{"1600x900p60hz", TVMODE_1600x900p60hz},
	{"1680x1050p60hz", TVMODE_1680x1050p60hz},
	{"1920x1200p60hz", TVMODE_1920x1200p60hz},
	{"2560x1440p60hz", TVMODE_2560x1440p60hz},
	{"2560x1600p60hz", TVMODE_2560x1600p60hz},
	{"2560x1080p60hz", TVMODE_2560x1080p60hz},
	{"3440x1440p60hz", TVMODE_3440x1440p60hz},
	{"480x320p60hz", TVMODE_480x320p60hz},
	{"480cvbs", VMODE_480CVBS},
	{"576cvbs", VMODE_576CVBS},
	{"480i60hz", VMODE_480I},
	{"480p60hz", VMODE_480P},
	{"576i50hz", VMODE_576I},
	{"576p50hz", VMODE_576P},
	{"720p60hz", VMODE_720P},
	{"1080i60hz", VMODE_1080I},
	{"1080p60hz", VMODE_1080P},
	{"720p50hz", VMODE_720P_50HZ},
	{"1080i50hz", VMODE_1080I_50HZ},
	{"1080p50hz", VMODE_1080P_50HZ},
	{"1080p24hz", VMODE_1080P_24HZ},
	{"2160p24hz", VMODE_4K2K_24HZ},
	{"2160p25hz", VMODE_4K2K_25HZ},
	{"2160p30hz", VMODE_4K2K_30HZ},
	{"smpte24hz", VMODE_4K2K_SMPTE},
	{"2160p50hz420", VMODE_4K2K_50HZ_Y420},
	{"2160p60hz420", VMODE_4K2K_60HZ_Y420},
	{"custombuilt", VMODE_CUSTOMBUILT},
};

struct logo_info_s {
	u32 index;
	u32 vmode;
	u32 debug;
	u32 loaded;
} logo_info = {
	.index = -1,
	.vmode = VMODE_MAX,
	.debug = 0,
	.loaded = 0,
};


static int get_value_by_name(char *name, struct para_pair_s *pair, u32 cnt)
{
	u32 i = 0;
	int found = -1;

	for (i = 0; i < cnt; i++, pair++) {
		if (strcmp(pair->name, name) == 0) {
			found = pair->value;
			break;
		}
	}

	return found;
}

static char *get_name_by_value(u32 value, struct para_pair_s *pair, u32 cnt)
{
	u32 i = 0;
	char *found = NULL;

	for (i = 0; i < cnt; i++, pair++) {
		if (value == pair->value) {
			found = pair->name;
			break;
		}
	}

	return found;
}

int set_osd_freescaler(int index, enum vmode_e new_mode)
{
	int cnt = sizeof(mode_infos) / sizeof(mode_infos[0]);
	pr_info("outputmode changed to %s, reset osd%d scaler.\n",
		get_name_by_value(new_mode, mode_infos, cnt), index);
	osd_set_free_scale_mode_hw(index, 1);
	osd_set_free_scale_enable_hw(index, 0);

	osd_set_free_scale_axis_hw(index, 0, 0, 1919, 1079);
	osd_update_disp_axis_hw(index, 0, 1919, 0, 1079, 0, 0, 0);
	switch (new_mode) {
	case VMODE_480I:
	case VMODE_480CVBS:
	case VMODE_480P:
		osd_set_window_axis_hw(index, 0, 0, 719, 479);
		break;
	case VMODE_576I:
	case VMODE_576CVBS:
	case VMODE_576P:
		osd_set_window_axis_hw(index, 0, 0, 719, 575);
		break;
	case VMODE_720P:
	case VMODE_720P_50HZ:
		osd_set_window_axis_hw(index, 0, 0, 1279, 719);
		break;
	case VMODE_1080I:
	case VMODE_1080I_50HZ:
	case VMODE_1080P:
	case VMODE_1080P_50HZ:
	case VMODE_1080P_24HZ:
		osd_set_window_axis_hw(index, 0, 0, 1919, 1079);
		break;
	case VMODE_4K2K_24HZ:
	case VMODE_4K2K_25HZ:
	case VMODE_4K2K_30HZ:
	case VMODE_4K2K_50HZ_Y420:
	case VMODE_4K2K_60HZ_Y420:
		osd_set_window_axis_hw(index, 0, 0, 3839, 2159);
		break;
	case VMODE_4K2K_SMPTE:
		osd_set_window_axis_hw(index, 0, 0, 4095, 2159);
		break;
	default:
		break;
	}

	if (osd_get_logo_index() != logo_info.index) {
		pr_info("logo changed, return");
		return -1;
	}
	osd_set_free_scale_enable_hw(index, 0x10001);
	osd_enable_hw(index, 1);
	pr_info("finish");
	return 0;
}

enum vmode_e get_initial_vmode(void)
{
	enum vmode_e cur_mode = VMODE_MAX;

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
	int hdp_state = 1;
#else
	int hdp_state = get_hpd_state();
#endif

	if (hdp_state)
		cur_mode = hdmimode;
	else
		cur_mode = cvbsmode;

	return cur_mode;
}
EXPORT_SYMBOL(get_initial_vmode);

static int refresh_mode_and_logo(bool first)
{
	enum vmode_e cur_mode = VMODE_MAX;
	int cnt = sizeof(mode_infos) / sizeof(mode_infos[0]);
#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
	int hdp_state = 1;
#else
	int hdp_state = get_hpd_state();
#endif
	if (!first && osd_get_logo_index() != logo_info.index)
		return -1;

	if (hdp_state)
		cur_mode = hdmimode;
	else
		cur_mode = cvbsmode;

	if (first) {
		last_mode = logo_info.vmode;
		set_logo_vmode(cur_mode);
		pr_info("set vmode: %s\n",
			get_name_by_value(cur_mode, mode_infos, cnt));

		if ((logo_info.index >= 0)) {
			osd_set_logo_index(logo_info.index);
			osd_init_hw(logo_info.loaded);
		}
	}

	if ((cur_mode != last_mode) && (cur_mode != VMODE_CUSTOMBUILT)) {
		pr_info("mode chang\n");
		osd_enable_hw(logo_info.index, 0);
		if (!first) {
			set_logo_vmode(cur_mode);
			pr_info("set vmode: %s\n",
				get_name_by_value(cur_mode, mode_infos, cnt));
		}
		last_mode = cur_mode;
		vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE, &cur_mode);
		set_osd_freescaler(logo_info.index, cur_mode);
	}

	return 0;
}

void aml_logo_work(struct work_struct *work)
{
	mutex_lock(&logo_lock);

	refresh_mode_and_logo(false);

	mutex_unlock(&logo_lock);

	if (osd_get_logo_index() == logo_info.index)
		schedule_delayed_work(&logo_work, 1*HZ/2);

}

static int logo_info_init(char *para)
{
	u32 count = 0;
	int value = -1;

	count = sizeof(logo_args) / sizeof(logo_args[0]);
	value = get_value_by_name(para, logo_args, count);
	if (value >= 0) {
		switch (value) {
		case LOGO_DEV_OSD0:
			logo_info.index = LOGO_DEV_OSD0;
			break;
		case LOGO_DEV_OSD1:
			logo_info.index = LOGO_DEV_OSD1;
			break;
		case LOGO_DEBUG:
			logo_info.debug = 1;
			break;
		case LOGO_LOADED:
			logo_info.loaded = 1;
			break;
		default:
			break;
		}
		return 0;
	}

	count = sizeof(mode_infos) / sizeof(mode_infos[0]);
	value = get_value_by_name(para, mode_infos, count);
	if (value >= 0)
		logo_info.vmode = value;

	return 0;
}

static int str2lower(char *str)
{
	while (*str != '\0') {
		*str = tolower(*str);
		str++;
	}
	return 0;
}

static int __init logo_setup(char *str)
{
	char *ptr = str;
	char sep[2];
	char *option;
	int count = 5;
	char find = 0;

	if (NULL == str)
		return -EINVAL;

	do {
		if (!isalpha(*ptr) && !isdigit(*ptr)) {
			find = 1;
			break;
		}
	} while (*++ptr != '\0');

	if (!find)
		return -EINVAL;

	logo_info.index = -1;
	logo_info.debug = 0;
	logo_info.loaded = 0;
	logo_info.vmode = VMODE_MAX;

	sep[0] = *ptr;
	sep[1] = '\0';
	while ((count--) && (option = strsep(&str, sep))) {
		pr_info("%s\n", option);
		str2lower(option);
		logo_info_init(option);
	}
	return 0;
}
__setup("logo=", logo_setup);

static int __init get_hdmi_mode(char *str)
{
	u32 i;
	u32 cnt;

	cnt = sizeof(mode_infos) / sizeof(mode_infos[0]);
	for (i = 0; i < cnt; i++) {
		if (strcmp(mode_infos[i].name, str) == 0) {
			hdmimode = mode_infos[i].value;
			break;
		}
	}

	pr_info("get hdmimode: %s\n", str);
	return 1;
}
__setup("hdmimode=", get_hdmi_mode);

static int __init get_cvbs_mode(char *str)
{
	if (strncmp("480", str, 3) == 0)
		cvbsmode = VMODE_480CVBS;
	else if (strncmp("576", str, 3) == 0)
		cvbsmode = VMODE_576CVBS;
	else if (strncmp("nocvbs", str, 6) == 0)
		cvbsmode = hdmimode;
	pr_info("get cvbsmode: %s\n", str);
	return 1;
}
__setup("cvbsmode=", get_cvbs_mode);

static int __init logo_init(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	if (logo_info.loaded == 0)
		return 0;

	refresh_mode_and_logo(true);

	INIT_DELAYED_WORK(&logo_work, aml_logo_work);
	schedule_delayed_work(&logo_work, 1*HZ/2);

	return ret;
}
subsys_initcall(logo_init);
