/*
 * include/linux/amlogic/vout/vinfo.h
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


#ifndef _VINFO_H_
#define _VINFO_H_

/* the MSB is represent vmode set by logo */
#define	VMODE_LOGO_BIT_MASK	0x8000
#define	VMODE_MODE_BIT_MASK	0xff

enum vmode_e {
	VMODE_480I  = 0,
	VMODE_480I_RPT,
	VMODE_480CVBS,
	VMODE_480P,
	VMODE_480P_RPT,
	VMODE_576I,
	VMODE_576I_RPT,
	VMODE_576CVBS,
	VMODE_576P,
	VMODE_576P_RPT,
	VMODE_720P,
	VMODE_1080I,
	VMODE_1080P,
	VMODE_720P_50HZ,
	VMODE_1080I_50HZ,
	VMODE_1080P_50HZ,
	VMODE_1080P_24HZ,
	VMODE_4K2K_30HZ,
	VMODE_4K2K_25HZ,
	VMODE_4K2K_24HZ,
	VMODE_4K2K_SMPTE,
	VMODE_4K2K_FAKE_5G,
	VMODE_4K2K_60HZ,
	VMODE_4K2K_60HZ_Y420,
	VMODE_4K2K_50HZ,
	VMODE_4K2K_50HZ_Y420,
	VMODE_4K2K_5G,
	VMODE_4K1K_120HZ,
	VMODE_4K1K_120HZ_Y420,
	VMODE_4K1K_100HZ,
	VMODE_4K1K_100HZ_Y420,
	VMODE_4K05K_240HZ,
	VMODE_4K05K_240HZ_Y420,
	VMODE_4K05K_200HZ,
	VMODE_4K05K_200HZ_Y420,
	VMODE_640x480p60hz,
	VMODE_800x480p60hz,
	VMODE_480x800p60hz,
	VMODE_800x600p60hz,
	VMODE_1024x600p60hz,
	VMODE_1024x768p60hz,
	VMODE_1280x800p60hz,
	VMODE_1280x1024p60hz,
	VMODE_1360x768p60hz,
	VMODE_1366x768p60hz,
	VMODE_1440x900p60hz,
	VMODE_1600x900p60hz,
	VMODE_1600x1200p60hz,
	VMODE_1680x1050p60hz,
	VMODE_1920x1200p60hz,
	VMODE_2560x1440p60hz,
	VMODE_2560x1600p60hz,
	VMODE_2560x1080p60hz,
	VMODE_3440x1440p60hz,
	VMODE_480x320p60hz,
	VMODE_VGA,
	VMODE_SVGA,
	VMODE_XGA,
	VMODE_SXGA,
	VMODE_WSXGA,
	VMODE_FHDVGA,
	VMODE_LCD,
	VMODE_LVDS_1080P,
	VMODE_LVDS_1080P_50HZ,
	VMODE_LVDS_768P,
	VMODE_VX1_4K2K_60HZ,
	VMODE_CUSTOMBUILT,
	VMODE_MAX,
	VMODE_INIT_NULL,
	VMODE_MASK = 0xFF,
};

enum tvmode_e {
	TVMODE_480I = 0,
	TVMODE_480I_RPT,
	TVMODE_480CVBS,
	TVMODE_480P,
	TVMODE_480P_RPT,
	TVMODE_576I,
	TVMODE_576I_RPT,
	TVMODE_576CVBS,
	TVMODE_576P,
	TVMODE_576P_RPT,
	TVMODE_720P,
	TVMODE_1080I,
	TVMODE_1080P,
	TVMODE_720P_50HZ,
	TVMODE_1080I_50HZ,
	TVMODE_1080P_50HZ,
	TVMODE_1080P_24HZ,
	TVMODE_4K2K_30HZ,
	TVMODE_4K2K_25HZ,
	TVMODE_4K2K_24HZ,
	TVMODE_4K2K_SMPTE,
	TVMODE_4K2K_FAKE_5G,
	TVMODE_4K2K_60HZ,
	TVMODE_4K2K_60HZ_Y420,
	TVMODE_4K2K_50HZ,
	TVMODE_4K2K_50HZ_Y420,
	TVMODE_4K2K_5G,
	TVMODE_4K1K_120HZ,
	TVMODE_4K1K_120HZ_Y420,
	TVMODE_4K1K_100HZ,
	TVMODE_4K1K_100HZ_Y420,
	TVMODE_4K05K_240HZ,
	TVMODE_4K05K_240HZ_Y420,
	TVMODE_4K05K_200HZ,
	TVMODE_4K05K_200HZ_Y420,
	TVMODE_640x480p60hz,
	TVMODE_800x480p60hz,
	TVMODE_480x800p60hz,
	TVMODE_800x600p60hz,
	TVMODE_1024x600p60hz,
	TVMODE_1024x768p60hz,
	TVMODE_1280x800p60hz,
	TVMODE_1280x1024p60hz,
	TVMODE_1360x768p60hz,
	TVMODE_1366x768p60hz,
	TVMODE_1440x900p60hz,
	TVMODE_1600x900p60hz,
	TVMODE_1600x1200p60hz,
	TVMODE_1680x1050p60hz,
	TVMODE_1920x1200p60hz,
	TVMODE_2560x1440p60hz,
	TVMODE_2560x1600p60hz,
	TVMODE_2560x1080p60hz,
	TVMODE_3440x1440p60hz,
	TVMODE_480x320p60hz,
	TVMODE_VGA ,
	TVMODE_SVGA,
	TVMODE_XGA,
	TVMODE_SXGA,
	TVMODE_WSXGA,
	TVMODE_FHDVGA,
	TVMODE_CUSTOMBUILT,
	TVMODE_MAX
};

struct vinfo_s {
	char *name;
	enum vmode_e mode;
	u32 width;
	u32 height;
	u32 field_height;
	u32 aspect_ratio_num;
	u32 aspect_ratio_den;
	u32 sync_duration_num;
	u32 sync_duration_den;
	u32 screen_real_width;
	u32 screen_real_height;
	u32 video_clk;
};

struct disp_rect_s {
	int x;
	int y;
	int w;
	int h;
};

struct reg_s {
	unsigned int reg;
	unsigned int val;
};

struct tvregs_set_t {
	enum tvmode_e tvmode;
	const struct reg_s *clk_reg_setting;
	const struct reg_s *enc_reg_setting;
};

struct tvinfo_s {
	enum tvmode_e tvmode;
	uint xres;
	uint yres;
	const char *id;
};

#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
enum fine_tune_mode_e {
	KEEP_HPLL,
	UP_HPLL,
	DOWN_HPLL,
};
#endif
#endif /* _VINFO_H_ */
