/*
 * drivers/amlogic/display/osd/osd_fb.c
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
#include <linux/compat.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
/* Amlogic Headers */
#include <linux/amlogic/vout/vout_notify.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

/* Local Headers */
#include "osd.h"
#include "osd_fb.h"
#include "osd_hw.h"
#include "osd_log.h"
#include "osd_sync.h"

static __u32 var_screeninfo[5];

struct osd_info_s osd_info = {
	.index = 0,
	.osd_reverse = 0,
};

extern void osd_wait_vsync_hw(void);

const struct color_bit_define_s default_color_format_array[] = {
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	{
		COLOR_INDEX_02_PAL4, 0, 0,
		0, 2, 0, 0, 2, 0, 0, 2, 0, 0, 0, 0,
		FB_VISUAL_PSEUDOCOLOR, 2,
	},
	INVALID_BPP_ITEM,
	{
		COLOR_INDEX_04_PAL16, 0, 1,
		0, 4, 0, 0, 4, 0, 0, 4, 0, 0, 0, 0,
		FB_VISUAL_PSEUDOCOLOR, 4,
	},
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	{
		COLOR_INDEX_08_PAL256, 0, 2,
		0, 8, 0, 0, 8, 0, 0, 8, 0, 0, 0, 0,
		FB_VISUAL_PSEUDOCOLOR, 8,
	},
	/*16 bit color*/
	{
		COLOR_INDEX_16_655, 0, 4,
		10, 6, 0, 5, 5, 0, 0, 5, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_844, 1, 4,
		8, 8, 0, 4, 4, 0, 0, 4, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_6442, 2, 4,
		10, 6, 0, 6, 4, 0, 2, 4, 0, 0, 2, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_4444_R, 3, 4,
		12, 4, 0, 8, 4, 0, 4, 4, 0, 0, 4, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_4642_R, 7, 4,
		12, 4, 0, 6, 6, 0, 2, 4, 0, 0, 2, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_1555_A, 6, 4,
		10, 5, 0, 5, 5, 0, 0, 5, 0, 15, 1, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_4444_A, 5, 4,
		8, 4, 0, 4, 4, 0, 0, 4, 0, 12, 4, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	{
		COLOR_INDEX_16_565, 4, 4,
		11, 5, 0, 5, 6, 0, 0, 5, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 16
	},
	/*24 bit color*/
	INVALID_BPP_ITEM,
	INVALID_BPP_ITEM,
	{
		COLOR_INDEX_24_6666_A, 4, 7,
		12, 6, 0, 6, 6, 0, 0, 6, 0, 18, 6, 0,
		FB_VISUAL_TRUECOLOR, 24
	},
	{
		COLOR_INDEX_24_6666_R, 3, 7,
		18, 6, 0, 12, 6, 0, 6, 6, 0, 0, 6, 0,
		FB_VISUAL_TRUECOLOR, 24
	},
	{
		COLOR_INDEX_24_8565, 2, 7,
		11, 5, 0, 5, 6, 0, 0, 5, 0, 16, 8, 0,
		FB_VISUAL_TRUECOLOR, 24
	},
	{
		COLOR_INDEX_24_5658, 1, 7,
		19, 5, 0, 13, 6, 0, 8, 5, 0, 0, 8, 0,
		FB_VISUAL_TRUECOLOR, 24
	},
	{
		COLOR_INDEX_24_888_B, 5, 7,
		0, 8, 0, 8, 8, 0, 16, 8, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 24
	},
	{
		COLOR_INDEX_24_RGB, 0, 7,
		16, 8, 0, 8, 8, 0, 0, 8, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 24
	},
	/*32 bit color*/
	{
		COLOR_INDEX_32_BGRX, 3, 5,
		8, 8, 0, 16, 8, 0, 24, 8, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_XBGR, 2, 5,
		0, 8, 0, 8, 8, 0, 16, 8, 0, 24, 0, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_RGBX, 0, 5,
		24, 8, 0, 16, 8, 0, 8, 8, 0, 0, 0, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_XRGB, 1, 5,
		16, 8, 0, 8, 8, 0, 0, 8, 0, 24, 0, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_BGRA, 3, 5,
		8, 8, 0, 16, 8, 0, 24, 8, 0, 0, 8, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_ABGR, 2, 5,
		0, 8, 0, 8, 8, 0, 16, 8, 0, 24, 8, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_RGBA, 0, 5,
		24, 8, 0, 16, 8, 0, 8, 8, 0, 0, 8, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	{
		COLOR_INDEX_32_ARGB, 1, 5,
		16, 8, 0, 8, 8, 0, 0, 8, 0, 24, 8, 0,
		FB_VISUAL_TRUECOLOR, 32
	},
	/*YUV color*/
	{COLOR_INDEX_YUV_422, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16},
};

static struct fb_var_screeninfo fb_def_var[] = {
	{
		.xres            = 1920,
		.yres            = 1080,
		.xres_virtual    = 1920,
		.yres_virtual    = 2160,
		.xoffset         = 0,
		.yoffset         = 0,
		.bits_per_pixel = 32,
		.grayscale       = 0,
		.red             = {16, 8, 0},
		.green           = {8, 8, 0},
		.blue            = {0, 8, 0},
		.transp          = {24, 0, 0},
		.nonstd          = 1,
		.activate        = FB_ACTIVATE_NOW,
		.height          = -1,
		.width           = -1,
		.accel_flags     = 0,
		.pixclock        = 0,
		.left_margin     = 0,
		.right_margin    = 0,
		.upper_margin    = 0,
		.lower_margin    = 0,
		.hsync_len       = 0,
		.vsync_len       = 0,
		.sync            = 0,
		.vmode           = FB_VMODE_NONINTERLACED,
		.rotate          = 0,
	}
#ifdef CONFIG_FB_OSD2_ENABLE
	,
	{
		.xres            = 32,
		.yres            = 32,
		.xres_virtual    = 32,
		.yres_virtual    = 32,
		.xoffset         = 0,
		.yoffset         = 0,
		.bits_per_pixel = 32,
		.grayscale       = 0,
		/* leave as it is ,set by system. */
		.red             = {0, 0, 0},
		.green           = {0, 0, 0},
		.blue            = {0, 0, 0},
		.transp          = {0, 0, 0},
		.nonstd          = 0,
		.activate        = FB_ACTIVATE_NOW,
		.height          = -1,
		.width           = -1,
		.accel_flags     = 0,
		.pixclock        = 0,
		.left_margin     = 0,
		.right_margin    = 0,
		.upper_margin    = 0,
		.lower_margin    = 0,
		.hsync_len       = 0,
		.vsync_len       = 0,
		.sync            = 0,
		.vmode           = FB_VMODE_NONINTERLACED,
		.rotate          = 0,
	}
#endif
};

static struct fb_fix_screeninfo fb_def_fix = {
	.id         = "OSD FB",
	.xpanstep   = 1,
	.ypanstep   = 1,
	.type       = FB_TYPE_PACKED_PIXELS,
	.visual     = FB_VISUAL_TRUECOLOR,
	.accel      = FB_ACCEL_NONE,
};

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
static int monitor_onoff_flag;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
static int early_suspend_flag;
#endif
#ifdef CONFIG_SCREEN_ON_EARLY
static int early_resume_flag;
#endif

unsigned int osd_log_level;
int int_viu_vsync = -ENXIO;
#ifdef CONFIG_FB_OSD_VSYNC_RDMA
int int_rdma = INT_RDMA;
#endif
struct osd_fb_dev_s *gp_fbdev_list[OSD_COUNT] = {};
static struct reserved_mem fb_rmem;
static phys_addr_t fb_rmem_paddr[2];
static void __iomem *fb_rmem_vaddr[2];
static u32 fb_rmem_size[2];

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
static int osd_set_res_bootargs(int index, enum vmode_e mode)
{
	struct hdmi_cea_timing *custom_timing = get_custom_timing();

	osd_log_info("%s : mode %d\n", __func__, mode);

	/* FIXME : need to adjust this routine */
	switch (mode) {
	case TVMODE_640x480p60hz:
		fb_def_var[index].xres = 640;
		fb_def_var[index].yres = 480;
		fb_def_var[index].xres_virtual = 640;
		fb_def_var[index].yres_virtual = 960;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_800x480p60hz:
		fb_def_var[index].xres = 800;
		fb_def_var[index].yres = 480;
		fb_def_var[index].xres_virtual = 800;
		fb_def_var[index].yres_virtual = 960;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_800x600p60hz:
		fb_def_var[index].xres = 800;
		fb_def_var[index].yres = 600;
		fb_def_var[index].xres_virtual = 800;
		fb_def_var[index].yres_virtual = 1200;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1024x600p60hz:
		fb_def_var[index].xres = 1024;
		fb_def_var[index].yres = 600;
		fb_def_var[index].xres_virtual = 1024;
		fb_def_var[index].yres_virtual = 1200;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1024x768p60hz:
		fb_def_var[index].xres = 1024;
		fb_def_var[index].yres = 768;
		fb_def_var[index].xres_virtual = 1024;
		fb_def_var[index].yres_virtual = 1536;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1280x800p60hz:
		fb_def_var[index].xres = 1280;
		fb_def_var[index].yres = 800;
		fb_def_var[index].xres_virtual = 1280;
		fb_def_var[index].yres_virtual = 1600;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1280x1024p60hz:
		fb_def_var[index].xres = 1280;
		fb_def_var[index].yres = 1024;
		fb_def_var[index].xres_virtual = 1280;
		fb_def_var[index].yres_virtual = 2048;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1360x768p60hz:
		fb_def_var[index].xres = 1360;
		fb_def_var[index].yres = 768;
		fb_def_var[index].xres_virtual = 1360;
		fb_def_var[index].yres_virtual = 1536;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1366x768p60hz:
		fb_def_var[index].xres = 1366;
		fb_def_var[index].yres = 768;
		fb_def_var[index].xres_virtual = 1366;
		fb_def_var[index].yres_virtual = 1536;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1440x900p60hz:
		fb_def_var[index].xres = 1440;
		fb_def_var[index].yres = 900;
		fb_def_var[index].xres_virtual = 1440;
		fb_def_var[index].yres_virtual = 1800;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1600x900p60hz:
		fb_def_var[index].xres = 1600;
		fb_def_var[index].yres = 900;
		fb_def_var[index].xres_virtual = 1600;
		fb_def_var[index].yres_virtual = 1800;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1680x1050p60hz:
		fb_def_var[index].xres = 1680;
		fb_def_var[index].yres = 1050;
		fb_def_var[index].xres_virtual = 1680;
		fb_def_var[index].yres_virtual = 2100;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_1920x1200p60hz:
		fb_def_var[index].xres = 1920;
		fb_def_var[index].yres = 1200;
		fb_def_var[index].xres_virtual = 1920;
		fb_def_var[index].yres_virtual = 2400;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_2560x1440p60hz:
		fb_def_var[index].xres = 2560;
		fb_def_var[index].yres = 1440;
		fb_def_var[index].xres_virtual = 2560;
		fb_def_var[index].yres_virtual = 2880;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_2560x1600p60hz:
		fb_def_var[index].xres = 2560;
		fb_def_var[index].yres = 1600;
		fb_def_var[index].xres_virtual = 2560;
		fb_def_var[index].yres_virtual = 3200;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_2560x1080p60hz:
		fb_def_var[index].xres = 2560;
		fb_def_var[index].yres = 1080;
		fb_def_var[index].xres_virtual = 2560;
		fb_def_var[index].yres_virtual = 2160;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_3440x1440p60hz:
		fb_def_var[index].xres = 3440;
		fb_def_var[index].yres = 1440;
		fb_def_var[index].xres_virtual = 3440;
		fb_def_var[index].yres_virtual = 2880;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case TVMODE_480x320p60hz:
		fb_def_var[index].xres = 480;
		fb_def_var[index].yres = 320;
		fb_def_var[index].xres_virtual = 480;
		fb_def_var[index].yres_virtual = 320;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case VMODE_480P:
	case VMODE_480I:
		fb_def_var[index].xres = 720;
		fb_def_var[index].yres = 480;
		fb_def_var[index].xres_virtual = 720;
		fb_def_var[index].yres_virtual = 960;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case VMODE_576P:
	case VMODE_576I:
		fb_def_var[index].xres = 720;
		fb_def_var[index].yres = 576;
		fb_def_var[index].xres_virtual = 720;
		fb_def_var[index].yres_virtual = 1152;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case VMODE_720P:
	case VMODE_720P_50HZ:
		fb_def_var[index].xres = 1280;
		fb_def_var[index].yres = 720;
		fb_def_var[index].xres_virtual = 1280;
		fb_def_var[index].yres_virtual = 1440;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case VMODE_1080P:
	case VMODE_1080P_50HZ:
	case VMODE_1080P_24HZ:
	case VMODE_1080I:
	case VMODE_1080I_50HZ:
		fb_def_var[index].xres = 1920;
		fb_def_var[index].yres = 1080;
		fb_def_var[index].xres_virtual = 1920;
		fb_def_var[index].yres_virtual = 2160;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case VMODE_4K2K_30HZ:
	case VMODE_4K2K_25HZ:
	case VMODE_4K2K_60HZ:
	case VMODE_4K2K_50HZ:
	case VMODE_4K2K_60HZ_Y420:
	case VMODE_4K2K_50HZ_Y420:
		fb_def_var[index].xres = 1920;
		fb_def_var[index].yres = 1080;
		fb_def_var[index].xres_virtual = 1920;
		fb_def_var[index].yres_virtual = 3240;
		fb_def_var[index].bits_per_pixel = 32;
		break;
	case VMODE_CUSTOMBUILT:
		fb_def_var[index].xres = custom_timing->h_active;
		fb_def_var[index].yres = custom_timing->v_active;
		fb_def_var[index].xres_virtual = custom_timing->h_active;
		fb_def_var[index].yres_virtual = (custom_timing->v_active * 2);
		fb_def_var[index].bits_per_pixel = 32;
		break;
	default:
		osd_log_info("%s, no available resolution info.", __func__);
		return 1;
	}

	return 0;
}
#endif /* CONFIG_ARCH_MESON64_ODROIDC2 */

phys_addr_t get_fb_rmem_paddr(int index)
{
	if (index < 0 || index > 1)
		return 0;
	return fb_rmem_paddr[index];
}

static void osddev_setup(struct osd_fb_dev_s *fbdev)
{
	mutex_lock(&fbdev->lock);
	osd_setup_hw(fbdev->fb_info->node,
		     &fbdev->osd_ctl,
		     fbdev->fb_info->var.xoffset,
		     fbdev->fb_info->var.yoffset,
		     fbdev->fb_info->var.xres,
		     fbdev->fb_info->var.yres,
		     fbdev->fb_info->var.xres_virtual,
		     fbdev->fb_info->var.yres_virtual,
		     fbdev->osd_ctl.disp_start_x,
		     fbdev->osd_ctl.disp_start_y,
		     fbdev->osd_ctl.disp_end_x,
		     fbdev->osd_ctl.disp_end_y,
		     fbdev->fb_mem_paddr,
		     fbdev->color);
	mutex_unlock(&fbdev->lock);

	return;
}

static void osddev_update_disp_axis(struct osd_fb_dev_s *fbdev,
					int mode_change)
{
	osd_update_disp_axis_hw(fbdev->fb_info->node,
				fbdev->osd_ctl.disp_start_x,
				fbdev->osd_ctl.disp_end_x,
				fbdev->osd_ctl.disp_start_y,
				fbdev->osd_ctl.disp_end_y,
				fbdev->fb_info->var.xoffset,
				fbdev->fb_info->var.yoffset,
				mode_change);
}

static int osddev_setcolreg(unsigned regno, u16 red, u16 green, u16 blue,
			    u16 transp, struct osd_fb_dev_s *fbdev)
{
	struct fb_info *info = fbdev->fb_info;
	if ((fbdev->color->color_index == COLOR_INDEX_02_PAL4) ||
	    (fbdev->color->color_index == COLOR_INDEX_04_PAL16) ||
	    (fbdev->color->color_index == COLOR_INDEX_08_PAL256)) {
		mutex_lock(&fbdev->lock);
		osd_setpal_hw(fbdev->fb_info->node, regno, red, green,
				blue, transp);
		mutex_lock(&fbdev->lock);
	}
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v, r, g, b, a;
		if (regno >= 16)
			return 1;
		r = red    >> (16 - info->var.red.length);
		g = green  >> (16 - info->var.green.length);
		b = blue   >> (16 - info->var.blue.length);
		a = transp >> (16 - info->var.transp.length);
		v = (r << info->var.red.offset)   |
		    (g << info->var.green.offset) |
		    (b << info->var.blue.offset)  |
		    (a << info->var.transp.offset);
		((u32 *)(info->pseudo_palette))[regno] = v;
	}
	return 0;
}

static const struct color_bit_define_s *
_find_color_format(struct fb_var_screeninfo *var)
{
	u32 upper_margin, lower_margin, i, level;
	const struct color_bit_define_s *found = NULL;
	const struct color_bit_define_s *color = NULL;

	level = (var->bits_per_pixel - 1) / 8;
	switch (level) {
	case 0:
		upper_margin = COLOR_INDEX_08_PAL256;
		lower_margin = COLOR_INDEX_02_PAL4;
		break;
	case 1:
		upper_margin = COLOR_INDEX_16_565;
		lower_margin = COLOR_INDEX_16_655;
		break;
	case 2:
		upper_margin = COLOR_INDEX_24_RGB;
		lower_margin = COLOR_INDEX_24_6666_A;
		break;
	case 3:
		if ((var->nonstd != 0)
		    && (var->transp.length == 0)) {
			/* RGBX Mode */
			upper_margin = COLOR_INDEX_32_XRGB;
			lower_margin = COLOR_INDEX_32_BGRX;
		} else {
			upper_margin = COLOR_INDEX_32_ARGB;
			lower_margin = COLOR_INDEX_32_BGRA;
		}
		break;
	case 4:
		upper_margin = COLOR_INDEX_YUV_422;
		lower_margin = COLOR_INDEX_YUV_422;
		break;
	default:
		osd_log_err("unsupported color format.");
		return NULL;
	}
	/*
	 * if not provide color component length
	 * then we find the first depth match.
	 */

	if ((var->nonstd != 0) && (level == 3)
	    && (var->transp.length == 0)) {
		/* RGBX Mode */
		for (i = upper_margin; i >= lower_margin; i--) {
			color = &default_color_format_array[i];
			if ((color->red_length == var->red.length) &&
			    (color->green_length == var->green.length) &&
			    (color->blue_length == var->blue.length) &&
			    (color->transp_offset == var->transp.offset) &&
			    (color->green_offset == var->green.offset) &&
			    (color->blue_offset == var->blue.offset) &&
			    (color->red_offset == var->red.offset)) {
				found = color;
				break;
			}
			color--;
		}
	} else if ((var->red.length == 0) || (var->green.length == 0)
	    || (var->blue.length == 0) ||
	    var->bits_per_pixel != (var->red.length + var->green.length +
		    var->blue.length + var->transp.length)) {
		osd_log_dbg("not provide color length, use default color\n");
		found = &default_color_format_array[upper_margin];
	} else {
		for (i = upper_margin; i >= lower_margin; i--) {
			color = &default_color_format_array[i];
			if ((color->red_length == var->red.length) &&
			    (color->green_length == var->green.length) &&
			    (color->blue_length == var->blue.length) &&
			    (color->transp_length == var->transp.length) &&
			    (color->transp_offset == var->transp.offset) &&
			    (color->green_offset == var->green.offset) &&
			    (color->blue_offset == var->blue.offset) &&
			    (color->red_offset == var->red.offset)) {
				found = color;
				break;
			}
			color--;
		}
	}
	return found;
}

static void __init _fbdev_set_default(struct osd_fb_dev_s *fbdev, int index)
{
	/* setup default value */
	fbdev->fb_info->var = fb_def_var[index];
	fbdev->fb_info->fix = fb_def_fix;
	fbdev->color = _find_color_format(&fbdev->fb_info->var);
}

static int osd_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct fb_fix_screeninfo *fix;
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)info->par;
	const struct color_bit_define_s *color_format_pt;

	fix = &info->fix;
	color_format_pt = _find_color_format(var);
	if (color_format_pt == NULL || color_format_pt->color_index == 0)
		return -EFAULT;

	osd_log_dbg("select color format :index %d, bpp %d\n",
		    color_format_pt->color_index,
		    color_format_pt->bpp);
	fbdev->color = color_format_pt;
	var->red.offset = color_format_pt->red_offset;
	var->red.length = color_format_pt->red_length;
	var->red.msb_right = color_format_pt->red_msb_right;
	var->green.offset  = color_format_pt->green_offset;
	var->green.length  = color_format_pt->green_length;
	var->green.msb_right = color_format_pt->green_msb_right;
	var->blue.offset   = color_format_pt->blue_offset;
	var->blue.length   = color_format_pt->blue_length;
	var->blue.msb_right = color_format_pt->blue_msb_right;
	var->transp.offset = color_format_pt->transp_offset;
	var->transp.length = color_format_pt->transp_length;
	var->transp.msb_right = color_format_pt->transp_msb_right;
	var->bits_per_pixel = color_format_pt->bpp;
	osd_log_dbg("rgba(L/O):%d/%d-%d/%d-%d/%d-%d/%d\n",
		    var->red.length, var->red.offset,
		    var->green.length, var->green.offset,
		    var->blue.length, var->blue.offset,
		    var->transp.length, var->transp.offset);
	fix->visual = color_format_pt->color_type;
	/* adjust memory length. */
	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;
	osd_log_dbg("xvirtual=%d, bpp:%d, line_length=%d\n",
		var->xres_virtual, var->bits_per_pixel, fix->line_length);
	if (var->xres_virtual * var->yres_virtual * var->bits_per_pixel / 8 >
			fbdev->fb_len) {
		osd_log_err("no enough memory for %d*%d*%d\n", var->xres,
			var->yres, var->bits_per_pixel);
		return  -ENOMEM;
	}
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	var->left_margin = var->right_margin = 0;
	var->upper_margin = var->lower_margin = 0;
	if (var->xres + var->xoffset > var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres;
	if (var->yres + var->yoffset > var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres;
	return 0;
}

static int osd_set_par(struct fb_info *info)
{
	const struct vinfo_s *vinfo;
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)info->par;
	struct osd_ctl_s *osd_ctrl = &fbdev->osd_ctl;
	u32 virt_end_x, virt_end_y;

	vinfo = get_current_vinfo();
	if (!vinfo) {
		osd_log_err("current vinfo NULL\n");
		return -1;
	}
	virt_end_x = osd_ctrl->disp_start_x + info->var.xres;
	virt_end_y = osd_ctrl->disp_start_y + info->var.yres;
	if (virt_end_x > vinfo->width)
		osd_ctrl->disp_end_x = vinfo->width - 1;
	else
		osd_ctrl->disp_end_x = virt_end_x - 1;
	if (virt_end_y  > vinfo->height)
		osd_ctrl->disp_end_y = vinfo->height - 1;
	else
		osd_ctrl->disp_end_y = virt_end_y - 1;
	osddev_setup((struct osd_fb_dev_s *)info->par);

	return 0;
}

static int
osd_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue,
	      unsigned transp, struct fb_info *info)
{
	return osddev_setcolreg(regno, red, green, blue,
				transp, (struct osd_fb_dev_s *)info->par);
}

static int
osd_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	int count, index, r;
	u16 *red, *green, *blue, *transp;
	u16 trans = 0xffff;
	red     = cmap->red;
	green   = cmap->green;
	blue    = cmap->blue;
	transp  = cmap->transp;
	index   = cmap->start;
	for (count = 0; count < cmap->len; count++) {
		if (transp)
			trans = *transp++;
		r = osddev_setcolreg(index++, *red++, *green++, *blue++, trans,
				     (struct osd_fb_dev_s *)info->par);
		if (r != 0)
			return r;
	}
	return 0;
}

static int osd_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)info->par;
	void __user *argp = (void __user *)arg;
	u32 src_colorkey;/* 16 bit or 24 bit */
	u32 srckey_enable;
	u32 gbl_alpha;
	u32 osd_order;
	s32 osd_axis[4] = {0};
	s32 osd_dst_axis[4] = {0};
	u32 block_windows[8] = {0};
	u32 block_mode;
	unsigned long ret;
	u32 flush_rate;
	struct fb_sync_request_s sync_request;

	switch (cmd) {
	case  FBIOPUT_OSD_SRCKEY_ENABLE:
		ret = copy_from_user(&srckey_enable, argp, sizeof(u32));
		break;
	case  FBIOPUT_OSD_SRCCOLORKEY:
		ret = copy_from_user(&src_colorkey, argp, sizeof(u32));
		break;
	case FBIOPUT_OSD_SET_GBL_ALPHA:
		ret = copy_from_user(&gbl_alpha, argp, sizeof(u32));
		break;
	case FBIOPUT_OSD_SCALE_AXIS:
		ret = copy_from_user(&osd_axis, argp, 4 * sizeof(s32));
		break;
	case FBIOPUT_OSD_SYNC_ADD:
		ret = copy_from_user(&sync_request, argp,
				sizeof(struct fb_sync_request_s));
		break;
	case FBIO_WAITFORVSYNC:
	case FBIOGET_OSD_SCALE_AXIS:
	case FBIOPUT_OSD_ORDER:
	case FBIOGET_OSD_ORDER:
	case FBIOGET_OSD_GET_GBL_ALPHA:
	case FBIOPUT_OSD_2X_SCALE:
	case FBIOPUT_OSD_ENABLE_3D_MODE:
	case FBIOPUT_OSD_FREE_SCALE_ENABLE:
	case FBIOPUT_OSD_FREE_SCALE_MODE:
	case FBIOPUT_OSD_FREE_SCALE_WIDTH:
	case FBIOPUT_OSD_FREE_SCALE_HEIGHT:
	case FBIOGET_OSD_BLOCK_WINDOWS:
	case FBIOGET_OSD_BLOCK_MODE:
	case FBIOGET_OSD_FREE_SCALE_AXIS:
	case FBIOGET_OSD_WINDOW_AXIS:
	case FBIOPUT_OSD_REVERSE:
	case FBIOPUT_OSD_ROTATE_ON:
	case FBIOPUT_OSD_ROTATE_ANGLE:
		break;
	case FBIOPUT_OSD_BLOCK_MODE:
		ret = copy_from_user(&block_mode, argp, sizeof(u32));
		break;
	case FBIOPUT_OSD_BLOCK_WINDOWS:
		ret = copy_from_user(&block_windows, argp, 8 * sizeof(u32));
		break;
	case FBIOPUT_OSD_FREE_SCALE_AXIS:
		ret = copy_from_user(&osd_axis, argp, 4 * sizeof(s32));
		break;
	case FBIOPUT_OSD_WINDOW_AXIS:
		ret = copy_from_user(&osd_dst_axis, argp, 4 * sizeof(s32));
		break;
#if defined(CONFIG_ARCH_MESON64_ODROIDC2) && defined(CONFIG_UMP)
	case GET_UMP_SECURE_ID_BUF1:
		return disp_get_ump_secure_id(info, fbdev, arg, 0);
		break;
	case GET_UMP_SECURE_ID_BUF2:
		return disp_get_ump_secure_id(info, fbdev, arg, 1);
		break;
#endif
	default:
		return -1;
	}
	mutex_lock(&fbdev->lock);
	switch (cmd) {
	case FBIOPUT_OSD_ORDER:
		osd_set_order_hw(info->node, arg);
		break;
	case FBIOGET_OSD_ORDER:
		osd_get_order_hw(info->node, &osd_order);
		ret = copy_to_user(argp, &osd_order, sizeof(u32));
		break;
	case FBIOPUT_OSD_FREE_SCALE_WIDTH:
		osd_set_free_scale_width_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_FREE_SCALE_HEIGHT:
		osd_set_free_scale_height_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_FREE_SCALE_ENABLE:
		osd_set_free_scale_enable_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_FREE_SCALE_MODE:
		osd_set_free_scale_mode_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_ENABLE_3D_MODE:
		osd_enable_3d_mode_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_2X_SCALE:
		/*
		 * high 16 bits for h_scale_enable;
		 * low 16 bits for v_scale_enable
		 */
		osd_set_2x_scale_hw(info->node, arg & 0xffff0000 ? 1 : 0,
				arg & 0xffff ? 1 : 0);
		break;
	case FBIOGET_OSD_FLUSH_RATE:
		osd_get_flush_rate_hw(&flush_rate);
		if (copy_to_user(argp, &flush_rate, sizeof(u32)))
			return -EFAULT;
		break;
	case FBIOPUT_OSD_REVERSE:
		osd_set_reverse_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_ROTATE_ON:
		osd_set_rotate_on_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_ROTATE_ANGLE:
		osd_set_rotate_angle_hw(info->node, arg);
		break;
	case FBIOPUT_OSD_SRCCOLORKEY:
		switch (fbdev->color->color_index) {
		case COLOR_INDEX_16_655:
		case COLOR_INDEX_16_844:
		case COLOR_INDEX_16_565:
		case COLOR_INDEX_24_888_B:
		case COLOR_INDEX_24_RGB:
		case COLOR_INDEX_YUV_422:
			osd_log_dbg("set osd color key 0x%x\n", src_colorkey);
			fbdev->color_key = src_colorkey;
			osd_set_color_key_hw(info->node,
				fbdev->color->color_index, src_colorkey);
			break;
		default:
			break;
		}
		break;
	case FBIOPUT_OSD_SRCKEY_ENABLE:
		switch (fbdev->color->color_index) {
		case COLOR_INDEX_16_655:
		case COLOR_INDEX_16_844:
		case COLOR_INDEX_16_565:
		case COLOR_INDEX_24_888_B:
		case COLOR_INDEX_24_RGB:
		case COLOR_INDEX_YUV_422:
			osd_log_dbg("set osd color key %s\n",
					srckey_enable ? "enable" : "disable");
			if (srckey_enable != 0) {
				fbdev->enable_key_flag |= KEYCOLOR_FLAG_TARGET;
				if (!(fbdev->enable_key_flag &
						KEYCOLOR_FLAG_ONHOLD)) {
					osd_srckey_enable_hw(info->node, 1);
					fbdev->enable_key_flag |=
						KEYCOLOR_FLAG_CURRENT;
				}
			} else {
				osd_srckey_enable_hw(info->node, 0);
				fbdev->enable_key_flag &= ~(KEYCOLOR_FLAG_TARGET
						| KEYCOLOR_FLAG_CURRENT);
			}
			break;
		default:
			break;
		}
		break;
	case FBIOPUT_OSD_SET_GBL_ALPHA:
		osd_set_gbl_alpha_hw(info->node, gbl_alpha);
		break;
	case  FBIOGET_OSD_GET_GBL_ALPHA:
		gbl_alpha = osd_get_gbl_alpha_hw(info->node);
		ret = copy_to_user(argp, &gbl_alpha, sizeof(u32));
		break;
	case FBIOGET_OSD_SCALE_AXIS:
		osd_get_scale_axis_hw(info->node, &osd_axis[0],
			&osd_axis[1], &osd_axis[2], &osd_axis[3]);
		ret = copy_to_user(argp, &osd_axis, 4 * sizeof(s32));
		break;
	case FBIOPUT_OSD_SCALE_AXIS:
		osd_set_scale_axis_hw(info->node, osd_axis[0], osd_axis[1],
				osd_axis[2], osd_axis[3]);
		break;
	case FBIOGET_OSD_BLOCK_WINDOWS:
		osd_get_block_windows_hw(info->node, block_windows);
		ret = copy_to_user(argp, &block_windows, 8 * sizeof(u32));
		break;
	case FBIOPUT_OSD_BLOCK_WINDOWS:
		osd_set_block_windows_hw(info->node, block_windows);
		break;
	case FBIOPUT_OSD_BLOCK_MODE:
		osd_set_block_mode_hw(info->node, block_mode);
		break;
	case FBIOGET_OSD_BLOCK_MODE:
		osd_get_block_mode_hw(info->node, &block_mode);
		ret = copy_to_user(argp, &block_mode, sizeof(u32));
		break;
	case FBIOGET_OSD_FREE_SCALE_AXIS:
		osd_get_free_scale_axis_hw(info->node, &osd_axis[0],
			&osd_axis[1], &osd_axis[2], &osd_axis[3]);
		ret = copy_to_user(argp, &osd_axis, 4 * sizeof(s32));
		break;
	case FBIOGET_OSD_WINDOW_AXIS:
		osd_get_window_axis_hw(info->node, &osd_dst_axis[0],
			&osd_dst_axis[1], &osd_dst_axis[2], &osd_dst_axis[3]);
		ret = copy_to_user(argp, &osd_dst_axis, 4 * sizeof(s32));
		break;
	case FBIOPUT_OSD_FREE_SCALE_AXIS:
		osd_set_free_scale_axis_hw(info->node, osd_axis[0],
			osd_axis[1], osd_axis[2], osd_axis[3]);
		break;
	case FBIOPUT_OSD_WINDOW_AXIS:
		osd_set_window_axis_hw(info->node, osd_dst_axis[0],
			osd_dst_axis[1], osd_dst_axis[2], osd_dst_axis[3]);
		break;
	case FBIOPUT_OSD_SYNC_ADD:
		sync_request.out_fen_fd =
			osd_sync_request(info->node, info->var.yres,
					sync_request.xoffset,
					sync_request.yoffset,
					sync_request.in_fen_fd);
		ret = copy_to_user(argp, &sync_request,
				sizeof(struct fb_sync_request_s));
		if (sync_request.out_fen_fd  < 0)
			/* fence create fail. */
			ret = -1;
		break;
	case FBIO_WAITFORVSYNC:
		osd_wait_vsync_hw();
		ret = copy_to_user(argp, &ret, sizeof(u32));
	default:
		break;
	}
	mutex_unlock(&fbdev->lock);
	return  ret;
}

#ifdef CONFIG_COMPAT
struct fb_cursor_user32 {
	__u16 set;		/* what to set */
	__u16 enable;		/* cursor on/off */
	__u16 rop;		/* bitop operation */
	compat_caddr_t mask;
	struct fbcurpos hot;	/* cursor hot spot */
	struct fb_image image;	/* Cursor image */
};

static int osd_compat_cursor(struct fb_info *info, unsigned long arg)
{
	unsigned long ret;
	struct fb_cursor_user __user *ucursor;
	struct fb_cursor_user32 __user *ucursor32;
	struct fb_cursor cursor;
	void __user *argp;
	__u32 data;

	ucursor = compat_alloc_user_space(sizeof(*ucursor));
	ucursor32 = compat_ptr(arg);

	if (copy_in_user(&ucursor->set, &ucursor32->set, 3 * sizeof(__u16)))
		return -EFAULT;
	if (get_user(data, &ucursor32->mask) ||
	    put_user(compat_ptr(data), &ucursor->mask))
		return -EFAULT;
	if (copy_in_user(&ucursor->hot, &ucursor32->hot, 2 * sizeof(__u16)))
		return -EFAULT;

	argp = (void __user *)ucursor;
	if (copy_from_user(&cursor, argp, sizeof(cursor)))
		return -EFAULT;

	if (!lock_fb_info(info))
		return -ENODEV;
	if (!info->fbops->fb_cursor)
		return -EINVAL;
	console_lock();
	ret = info->fbops->fb_cursor(info, &cursor);
	console_unlock();
	unlock_fb_info(info);

	if (ret == 0 && copy_to_user(argp, &cursor, sizeof(cursor)))
		return -EFAULT;

	return ret;
}

static int osd_compat_ioctl(struct fb_info *info,
		unsigned int cmd, unsigned long arg)
{
	unsigned long ret;

	arg = (unsigned long)compat_ptr(arg);

	/* handle fbio cursor command for 32-bit app */
	if ((cmd & 0xFFFF) == (FBIO_CURSOR & 0xFFFF)) {
		ret = osd_compat_cursor(info, arg);
	} else
		ret = osd_ioctl(info, cmd, arg);

	return ret;
}
#endif

static int osd_open(struct fb_info *info, int arg)
{
	return 0;
}

int osd_blank(int blank_mode, struct fb_info *info)
{
	osd_enable_hw(info->node, (blank_mode != 0) ? 0 : 1);
#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
	if (!monitor_onoff_flag)
		return 0;

	switch (blank_mode) {
	/* Display: On; HSync: On, VSync: On */
	case FB_BLANK_UNBLANK:
		control_hdmiphy(1);
		break;
	/* Display: Off; HSync: Off, VSync: Off */
	case FB_BLANK_POWERDOWN:
		control_hdmiphy(0);
		break;
	/* Display: Off; HSync: On, VSync: On */
	case FB_BLANK_NORMAL:
	/* Display: Off; HSync: Off, VSync: On */
	case FB_BLANK_HSYNC_SUSPEND:
	/* Display: Off; HSync: On, VSync: Off */
	case FB_BLANK_VSYNC_SUSPEND:
		break;
	}
#endif
	return 0;
}

static int osd_pan_display(struct fb_var_screeninfo *var,
			   struct fb_info *fbi)
{
	osd_pan_display_hw(fbi->node, var->xoffset, var->yoffset);
	osd_log_dbg("osd_pan_display:=>osd%d xoff=%d, yoff=%d\n",
			fbi->node, var->xoffset, var->yoffset);
	return 0;
}

#if defined(CONFIG_FB_OSD2_CURSOR)
static int osd_cursor(struct fb_info *fbi, struct fb_cursor *var)
{
	s16 startx = 0, starty = 0;
	struct osd_fb_dev_s *fb_dev = gp_fbdev_list[1];
	if (fb_dev) {
		startx = fb_dev->osd_ctl.disp_start_x;
		starty = fb_dev->osd_ctl.disp_start_y;
	}
	osd_cursor_hw(fbi->node, (s16)var->hot.x, (s16)var->hot.y, (s16)startx,
		      (s16)starty, fbi->var.xres, fbi->var.yres);
	return 0;
}
#endif

static int osd_sync(struct fb_info *info)
{
	return 0;
}


/* fb_ops structures */
static struct fb_ops osd_ops = {
	.owner          = THIS_MODULE,
	.fb_check_var   = osd_check_var,
	.fb_set_par     = osd_set_par,
	.fb_setcolreg   = osd_setcolreg,
	.fb_setcmap     = osd_setcmap,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
#ifdef CONFIG_FB_SOFT_CURSOR
	.fb_cursor      = soft_cursor,
#elif defined(CONFIG_FB_OSD2_CURSOR)
	.fb_cursor      = osd_cursor,
#endif
	.fb_ioctl       = osd_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = osd_compat_ioctl,
#endif
	.fb_open        = osd_open,
	.fb_blank       = osd_blank,
	.fb_pan_display = osd_pan_display,
	.fb_sync        = osd_sync,
};

static void set_default_display_axis(struct fb_var_screeninfo *var,
		struct osd_ctl_s *osd_ctrl, const struct vinfo_s *vinfo)
{
	u32  virt_end_x = osd_ctrl->disp_start_x + var->xres;
	u32  virt_end_y = osd_ctrl->disp_start_y + var->yres;
	if (virt_end_x > vinfo->width) {
		/* screen axis */
		osd_ctrl->disp_end_x = vinfo->width - 1;
	} else {
		/* screen axis */
		osd_ctrl->disp_end_x = virt_end_x - 1;
	}
	if (virt_end_y > vinfo->height)
		osd_ctrl->disp_end_y = vinfo->height - 1;
	else {
		/* screen axis */
		osd_ctrl->disp_end_y = virt_end_y - 1;
	}
	return;
}

int osd_notify_callback(struct notifier_block *block, unsigned long cmd,
			void *para)
{
	const struct vinfo_s *vinfo;
	struct osd_fb_dev_s *fb_dev;
	int  i, blank;
	struct disp_rect_s *disp_rect;

	vinfo = get_current_vinfo();
	if (!vinfo) {
		osd_log_err("current vinfo NULL\n");
		return -1;
	}
	osd_log_info("current vmode=%s\n", vinfo->name);
	switch (cmd) {
	case  VOUT_EVENT_MODE_CHANGE:
		for (i = 0; i < OSD_COUNT; i++) {
			fb_dev = gp_fbdev_list[i];
			if (NULL == fb_dev)
				continue;
			set_default_display_axis(&fb_dev->fb_info->var,
					&fb_dev->osd_ctl, vinfo);
			console_lock();
			osddev_update_disp_axis(fb_dev, 1);
#ifdef CONFIG_FB_OSD2_ENABLE
			osd_set_antiflicker_hw(DEV_OSD1, vinfo->mode,
			       gp_fbdev_list[DEV_OSD1]->fb_info->var.yres);
#endif
			console_unlock();
		}
		break;
	case VOUT_EVENT_OSD_BLANK:
		blank = *(int *)para;
		for (i = 0; i < OSD_COUNT; i++) {
			fb_dev = gp_fbdev_list[i];
			if (NULL == fb_dev)
				continue;
			console_lock();
			osd_blank(blank, fb_dev->fb_info);
			console_unlock();
		}
		break;
	case VOUT_EVENT_OSD_DISP_AXIS:
		disp_rect = (struct disp_rect_s *)para;
		for (i = 0; i < OSD_COUNT; i++) {
			if (!disp_rect)
				break;
			fb_dev = gp_fbdev_list[i];
			/*
			 * if osd layer preblend,
			 * it's position is controlled by vpp.
			 */
			if (fb_dev->preblend_enable)
				break;
			fb_dev->osd_ctl.disp_start_x = disp_rect->x;
			fb_dev->osd_ctl.disp_start_y = disp_rect->y;
			osd_log_dbg("set disp axis: x:%d y:%d w:%d h:%d\n",
				    disp_rect->x, disp_rect->y,
				    disp_rect->w, disp_rect->h);
			if (disp_rect->x + disp_rect->w > vinfo->width)
				fb_dev->osd_ctl.disp_end_x = vinfo->width - 1;
			else
				fb_dev->osd_ctl.disp_end_x =
					fb_dev->osd_ctl.disp_start_x +
					disp_rect->w - 1;
			if (disp_rect->y + disp_rect->h > vinfo->height)
				fb_dev->osd_ctl.disp_end_y = vinfo->height - 1;
			else
				fb_dev->osd_ctl.disp_end_y =
					fb_dev->osd_ctl.disp_start_y +
					disp_rect->h - 1;
			disp_rect++;
			osd_log_dbg("new disp axis: x0:%d y0:%d x1:%d y1:%d\n",
				    fb_dev->osd_ctl.disp_start_x,
				    fb_dev->osd_ctl.disp_start_y,
				    fb_dev->osd_ctl.disp_end_x,
				    fb_dev->osd_ctl.disp_end_y);
			console_lock();
			osddev_update_disp_axis(fb_dev, 0);
			console_unlock();
		}
		break;
	}
	return 0;
}

static struct notifier_block osd_notifier_nb = {
	.notifier_call	= osd_notify_callback,
};
static ssize_t store_preblend_enable(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	fbdev->preblend_enable = res;
	vout_notifier_call_chain(VOUT_EVENT_OSD_PREBLEND_ENABLE,
				 &fbdev->preblend_enable);
	return count;
}

static ssize_t show_preblend_enable(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	return snprintf(buf, PAGE_SIZE, "preblend[%s]\n",
			fbdev->preblend_enable ? "enable" : "disable");
}

static ssize_t store_enable_3d(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	fbdev->enable_3d = res;
	osd_enable_3d_mode_hw(fb_info->node, fbdev->enable_3d);
	return count;
}

static ssize_t show_enable_3d(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	return snprintf(buf, PAGE_SIZE, "3d_enable:[0x%x]\n", fbdev->enable_3d);
}

static ssize_t store_color_key(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 16, &res);
	switch (fbdev->color->color_index) {
	case COLOR_INDEX_16_655:
	case COLOR_INDEX_16_844:
	case COLOR_INDEX_16_565:
	case COLOR_INDEX_24_888_B:
	case COLOR_INDEX_24_RGB:
	case COLOR_INDEX_YUV_422:
		fbdev->color_key = res;
		osd_set_color_key_hw(fb_info->node, fbdev->color->color_index,
				     fbdev->color_key);
		break;
	default:
		break;
	}

	return count;
}

static ssize_t show_color_key(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	return snprintf(buf, PAGE_SIZE, "0x%x\n", fbdev->color_key);
}

static ssize_t store_enable_key(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	mutex_lock(&fbdev->lock);
	if (res != 0) {
		fbdev->enable_key_flag |= KEYCOLOR_FLAG_TARGET;
		if (!(fbdev->enable_key_flag & KEYCOLOR_FLAG_ONHOLD)) {
			osd_srckey_enable_hw(fb_info->node, 1);
			fbdev->enable_key_flag |= KEYCOLOR_FLAG_CURRENT;
		}
	} else {
		fbdev->enable_key_flag &=
			~(KEYCOLOR_FLAG_TARGET | KEYCOLOR_FLAG_CURRENT);
		osd_srckey_enable_hw(fb_info->node, 0);
	}
	mutex_unlock(&fbdev->lock);
	return count;
}

static ssize_t show_enable_key(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	return snprintf(buf, PAGE_SIZE, (fbdev->enable_key_flag
			& KEYCOLOR_FLAG_TARGET) ? "1\n" : "0\n");
}

static ssize_t store_enable_key_onhold(struct device *device,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0 , &res);
	mutex_lock(&fbdev->lock);
	if (res != 0) {
		/* hold all the calls to enable color key */
		fbdev->enable_key_flag |= KEYCOLOR_FLAG_ONHOLD;
		fbdev->enable_key_flag &= ~KEYCOLOR_FLAG_CURRENT;
		osd_srckey_enable_hw(fb_info->node, 0);
	} else {
		fbdev->enable_key_flag &= ~KEYCOLOR_FLAG_ONHOLD;
		/*
		 * if target and current mistach
		 * then recover the pending key settings
		 */
		if (fbdev->enable_key_flag & KEYCOLOR_FLAG_TARGET) {
			if ((fbdev->enable_key_flag & KEYCOLOR_FLAG_CURRENT)
					== 0) {
				fbdev->enable_key_flag |= KEYCOLOR_FLAG_CURRENT;
				osd_srckey_enable_hw(fb_info->node, 1);
			}
		} else {
			if (fbdev->enable_key_flag & KEYCOLOR_FLAG_CURRENT) {
				fbdev->enable_key_flag &=
					~KEYCOLOR_FLAG_CURRENT;
				osd_srckey_enable_hw(fb_info->node, 0);
			}
		}
	}
	mutex_unlock(&fbdev->lock);
	return count;
}

static ssize_t show_enable_key_onhold(struct device *device,
				      struct device_attribute *attr,
				      char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	return snprintf(buf, PAGE_SIZE,
			(fbdev->enable_key_flag & KEYCOLOR_FLAG_ONHOLD) ?
			"1\n" : "0\n");
}

static int parse_para(const char *para, int para_num, int *result)
{
	char *token = NULL;
	char *params, *params_base;
	int *out = result;
	int len = 0, count = 0;
	int res = 0;
	int ret = 0;

	if (!para)
		return 0;

	params = kstrdup(para, GFP_KERNEL);
	params_base = params;
	token = params;
	len = strlen(token);
	do {
		token = strsep(&params, " ");
		while (token && (isspace(*token)
				|| !isgraph(*token)) && len) {
			token++;
			len--;
		}
		if (len == 0)
			break;
		ret = kstrtoint(token, 0, &res);
		if (ret < 0)
			break;
		len = strlen(token);
		*out++ = res;
		count++;
	} while ((token) && (count < para_num) && (len > 0));

	kfree(params_base);
	return count;
}

static ssize_t show_free_scale_axis(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int x, y, w, h;
	osd_get_free_scale_axis_hw(fb_info->node, &x, &y, &w, &h);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", x, y, w, h);
}

static ssize_t store_free_scale_axis(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int parsed[4];
	if (likely(parse_para(buf, 4, parsed) == 4))
		osd_set_free_scale_axis_hw(fb_info->node,
				parsed[0], parsed[1], parsed[2], parsed[3]);
	else
		osd_log_err("set free scale axis error\n");
	return count;
}

static ssize_t show_scale_axis(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int x0, y0, x1, y1;
	osd_get_scale_axis_hw(fb_info->node, &x0, &y0, &x1, &y1);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n", x0, y0, x1, y1);
}

static ssize_t store_scale_axis(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int parsed[4];
	if (likely(parse_para(buf, 4, parsed) == 4))
		osd_set_scale_axis_hw(fb_info->node,
				parsed[0], parsed[1], parsed[2], parsed[3]);
	else
		osd_log_err("set scale axis error\n");
	return count;
}

static ssize_t store_scale_width(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_width = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0 , &res);
	free_scale_width = res;
	osd_set_free_scale_width_hw(fb_info->node, free_scale_width);

	return count;
}

static ssize_t show_scale_width(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_width = 0;
	osd_get_free_scale_width_hw(fb_info->node, &free_scale_width);
	return snprintf(buf, PAGE_SIZE, "free_scale_width:%d\n",
			free_scale_width);
}

static ssize_t store_scale_height(struct device *device,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_height = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0 , &res);
	free_scale_height = res;
	osd_set_free_scale_height_hw(fb_info->node, free_scale_height);
	return count;
}

static ssize_t show_scale_height(struct device *device,
				 struct device_attribute *attr,
				 char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_height = 0;
	osd_get_free_scale_height_hw(fb_info->node, &free_scale_height);
	return snprintf(buf, PAGE_SIZE, "free_scale_height:%d\n",
			free_scale_height);
}

static ssize_t store_free_scale(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_enable = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	free_scale_enable = res;
	osd_set_free_scale_enable_hw(fb_info->node, free_scale_enable);
	return count;
}

static ssize_t show_free_scale(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_enable = 0;
	osd_get_free_scale_enable_hw(fb_info->node, &free_scale_enable);
	return snprintf(buf, PAGE_SIZE, "free_scale_enable:[0x%x]\n",
			free_scale_enable);
}

static ssize_t store_freescale_mode(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_mode = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	free_scale_mode = res;
	osd_set_free_scale_mode_hw(fb_info->node, free_scale_mode);
	return count;
}

static ssize_t show_freescale_mode(struct device *device,
				   struct device_attribute *attr,
				   char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int free_scale_mode = 0;
	char *help_info = "free scale mode:\n"
			  "    0: VPU free scaler\n"
			  "    1: OSD free scaler\n"
			  "    2: OSD super scaler\n";
	osd_get_free_scale_mode_hw(fb_info->node, &free_scale_mode);
	return snprintf(buf, PAGE_SIZE, "%scurrent free_scale_mode:%d\n",
			help_info, free_scale_mode);
}

static ssize_t store_scale(struct device *device, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	fbdev->scale = res;
	osd_set_2x_scale_hw(fb_info->node, fbdev->scale & 0xffff0000 ? 1 : 0,
			    fbdev->scale & 0xffff ? 1 : 0);
	return count;
}

static ssize_t show_scale(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	return snprintf(buf, PAGE_SIZE, "scale:[0x%x]\n", fbdev->scale);
}

static ssize_t show_window_axis(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int x0, y0, x1, y1;
	osd_get_window_axis_hw(fb_info->node, &x0, &y0, &x1, &y1);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			x0, y0, x1, y1);
}

static ssize_t store_window_axis(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	s32 parsed[4];
	if (likely(parse_para(buf, 4, parsed) == 4))
		osd_set_window_axis_hw(fb_info->node,
				parsed[0], parsed[1], parsed[2], parsed[3]);
	else
		osd_log_err("set window axis error\n");
	return count;
}

static ssize_t show_debug(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	char *help = osd_get_debug_hw();

	return snprintf(buf, strlen(help), "%s", help);
}

static ssize_t store_debug(struct device *device, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int ret = -EINVAL;

	ret = osd_set_debug_hw(buf);
	if (ret == 0)
		ret = count;

	return ret;
}

static ssize_t show_log_level(struct device *device,
			  struct device_attribute *attr,
			  char *buf)
{
	return snprintf(buf, 40, "%d\n", osd_log_level);
}

static ssize_t store_log_level(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_log_info("log_level: %d->%d\n", osd_log_level, res);
	osd_log_level = res;

	return count;
}

static ssize_t store_order(struct device *device, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	fbdev->order = res;
	osd_set_order_hw(fb_info->node, fbdev->order);
	return count;
}

static ssize_t show_order(struct device *device, struct device_attribute *attr,
			  char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct osd_fb_dev_s *fbdev = (struct osd_fb_dev_s *)fb_info->par;
	osd_get_order_hw(fb_info->node, &(fbdev->order));
	return snprintf(buf, PAGE_SIZE, "order:[0x%x]\n", fbdev->order);
}

static ssize_t show_block_windows(struct device *device,
				  struct device_attribute *attr,
				  char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	u32 wins[8];
	osd_get_block_windows_hw(fb_info->node, wins);
	return snprintf(buf, PAGE_SIZE,
			"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
			wins[0], wins[1], wins[2], wins[3],
			wins[4], wins[5], wins[6], wins[7]);
}

static ssize_t store_block_windows(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int parsed[8];
	if (likely(parse_para(buf, 8, parsed) == 8))
		osd_set_block_windows_hw(fb_info->node, parsed);
	else
		osd_log_err("set block windows error\n");
	return count;
}

static ssize_t show_block_mode(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	u32 mode;
	osd_get_block_mode_hw(fb_info->node, &mode);
	return snprintf(buf, PAGE_SIZE, "0x%x\n", mode);
}

static ssize_t store_block_mode(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	u32 mode;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	mode = res;
	osd_set_block_mode_hw(fb_info->node, mode);
	return count;
}

static ssize_t show_flush_rate(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	u32 flush_rate = 0;
	osd_get_flush_rate_hw(&flush_rate);
	return snprintf(buf, PAGE_SIZE, "flush_rate:[%d]\n", flush_rate);
}

static ssize_t show_osd_reverse(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int osd_reverse = 0;
	osd_get_reverse_hw(fb_info->node, &osd_reverse);
	return snprintf(buf, PAGE_SIZE, "osd_reverse:[%s]\n",
			osd_reverse ? "TRUE" : "FALSE");
}

static ssize_t store_osd_reverse(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int osd_reverse = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_reverse = res;
	osd_set_reverse_hw(fb_info->node, osd_reverse);

	return count;
}
static ssize_t show_rotate_on(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int osd_rotate = 0;
	osd_get_rotate_on_hw(fb_info->node, &osd_rotate);
	return snprintf(buf, PAGE_SIZE, "osd_rotate:[%s]\n",
			osd_rotate ? "ON" : "OFF");
}

static ssize_t store_rotate_on(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int osd_rotate = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_rotate = res;
	osd_set_rotate_on_hw(fb_info->node, osd_rotate);
	return count;
}

static ssize_t show_prot_state(struct device *device,
			       struct device_attribute *attr,
			       char *buf)
{
	int pos = 0;
	unsigned int osd_rotate = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	osd_get_rotate_on_hw(fb_info->node, &osd_rotate);
	pos += snprintf(buf + pos, PAGE_SIZE, "%d", osd_rotate);
	return pos;
}

static ssize_t show_rotate_angle(struct device *device,
				 struct device_attribute *attr,
				 char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int osd_rotate_angle = 0;
	osd_get_rotate_angle_hw(fb_info->node, &osd_rotate_angle);
	return snprintf(buf, PAGE_SIZE, "osd_rotate:%d\n", osd_rotate_angle);
}

static ssize_t store_rotate_angle(struct device *device,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int osd_rotate_angle = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_rotate_angle = res;
	osd_set_rotate_angle_hw(fb_info->node, osd_rotate_angle);
	return count;
}

static ssize_t show_prot_canvas(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int x_start, y_start, x_end, y_end;
	osd_get_prot_canvas_hw(fb_info->node,
			&x_start, &y_start, &x_end, &y_end);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			x_start, y_start, x_end, y_end);
}

static ssize_t store_prot_canvas(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	int parsed[4];
	if (likely(parse_para(buf, 4, parsed) == 4))
		osd_set_prot_canvas_hw(fb_info->node, parsed[0],
				parsed[1], parsed[2], parsed[3]);
	else
		osd_log_err("set prot canvas error\n");
	return count;
}

static ssize_t show_antiflicker(struct device *device,
				struct device_attribute *attr,
				char *buf)
{
	unsigned int osd_antiflicker = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	osd_get_antiflicker_hw(fb_info->node, &osd_antiflicker);
	return snprintf(buf, PAGE_SIZE, "osd_antifliter:[%s]\n",
			osd_antiflicker ? "ON" : "OFF");
}

static ssize_t store_antiflicker(struct device *device,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	const struct vinfo_s *vinfo;
	unsigned int osd_antiflicker = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_antiflicker = res;
	vinfo = get_current_vinfo();
	if (!vinfo) {
		osd_log_err("get current vinfo NULL\n");
		return 0;
	}
	if (osd_antiflicker == 2)
		osd_set_antiflicker_hw(fb_info->node,
				osd_antiflicker, fb_info->var.yres);
	else
		osd_set_antiflicker_hw(fb_info->node,
				vinfo->mode, fb_info->var.yres);
	return count;
}

static ssize_t show_update_freescale(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int update_state = 0;
	osd_get_update_state_hw(fb_info->node, &update_state);
	return snprintf(buf, PAGE_SIZE, "update_state:[%s]\n",
			update_state ? "TRUE" : "FALSE");
}

static ssize_t store_update_freescale(struct device *device,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	unsigned int update_state = 0;
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	update_state = res;
	osd_set_update_state_hw(fb_info->node, update_state);
	return count;
}

static ssize_t show_ver_angle(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	unsigned int osd_angle = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	osd_get_angle_hw(fb_info->node, &osd_angle);
	return snprintf(buf, PAGE_SIZE, "osd_angle:[%d]\n", osd_angle);
}

static ssize_t store_ver_angle(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int osd_angle = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_angle = res;
	memset((char *)fb_info->screen_base, 0x80, fb_info->screen_size);
#ifdef CONFIG_FB_OSD2_ENABLE
	osd_set_angle_hw(fb_info->node, osd_angle, fb_def_var[DEV_OSD1].yres,
			 fb_info->var.yres);
#endif
	return count;
}

static ssize_t show_ver_clone(struct device *device,
			      struct device_attribute *attr,
			      char *buf)
{
	unsigned int osd_clone = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	osd_get_clone_hw(fb_info->node, &osd_clone);
	return snprintf(buf, PAGE_SIZE, "osd_clone:[%s]\n",
			osd_clone ? "ON" : "OFF");
}

static ssize_t store_ver_clone(struct device *device,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int osd_clone = 0;
	struct fb_info *fb_info = dev_get_drvdata(device);
	int res = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &res);
	osd_clone = res;
	osd_set_clone_hw(fb_info->node, osd_clone);
	return count;
}

static ssize_t store_ver_update_pan(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	osd_set_update_pan_hw(fb_info->node);
	return count;
}

static inline  int str2lower(char *str)
{
	while (*str != '\0') {
		*str = tolower(*str);
		str++;
	}
	return 0;
}

static struct para_osd_info_s para_osd_info[OSD_END + 2] = {
	/* head */
	{
		"head", OSD_INVALID_INFO,
		OSD_END + 1, 1,
		0, OSD_END + 1
	},
	/* dev */
	{
		"osd0",	DEV_OSD0,
		OSD_FIRST_GROUP_START - 1, OSD_FIRST_GROUP_START + 1,
		OSD_FIRST_GROUP_START, OSD_SECOND_GROUP_START - 1
	},
	{
		"osd1",	DEV_OSD1,
		OSD_FIRST_GROUP_START, OSD_FIRST_GROUP_START + 2,
		OSD_FIRST_GROUP_START, OSD_SECOND_GROUP_START - 1
	},
	{
		"all", DEV_ALL,
		OSD_FIRST_GROUP_START + 1, OSD_FIRST_GROUP_START + 3,
		OSD_FIRST_GROUP_START, OSD_SECOND_GROUP_START - 1
	},
	/* reverse_mode */
	{
		"true",	REVERSE_TRUE,
		OSD_SECOND_GROUP_START - 1, OSD_SECOND_GROUP_START + 1,
		OSD_SECOND_GROUP_START,	OSD_END
	},
	{
		"false", REVERSE_FALSE,
		OSD_SECOND_GROUP_START,	OSD_SECOND_GROUP_START + 2,
		OSD_SECOND_GROUP_START, OSD_END
	},
	{
		"tail",	OSD_INVALID_INFO, OSD_END,
		0, 0,
		OSD_END + 1
	},
};

static inline int install_osd_reverse_info(struct osd_info_s *init_osd_info,
		char *para)
{
	u32 i = 0;
	static u32 tail = OSD_END + 1;
	u32 first = para_osd_info[0].next_idx;
	for (i = first; i < tail; i = para_osd_info[i].next_idx) {
		if (strcmp(para_osd_info[i].name, para) == 0) {
			u32 group_start = para_osd_info[i].cur_group_start;
			u32 group_end = para_osd_info[i].cur_group_end;
			u32	prev = para_osd_info[group_start].prev_idx;
			u32  next = para_osd_info[group_end].next_idx;
			switch (para_osd_info[i].cur_group_start) {
			case OSD_FIRST_GROUP_START:
				init_osd_info->index = para_osd_info[i].info;
				break;
			case OSD_SECOND_GROUP_START:
				init_osd_info->osd_reverse =
					para_osd_info[i].info;
				break;
			}
			para_osd_info[prev].next_idx = next;
			para_osd_info[next].prev_idx = prev;
			return 0;
		}
	}
	return 0;
}

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
static int __init osd_setup_monitor_onoff(char *str)
{
	if (!strcmp(str, "true") || !strcmp(str, "1"))
		monitor_onoff_flag = 1;
	else
		monitor_onoff_flag = 0;

	return 0;
}
__setup("monitor_onoff=", osd_setup_monitor_onoff);
#endif

static int __init osd_info_setup(char *str)
{
	char	*ptr = str;
	char	sep[2];
	char	*option;
	int count = 2;
	char find = 0;

	struct osd_info_s *init_osd_info;
	if (NULL == str)
		return -EINVAL;

	init_osd_info = &osd_info;
	memset(init_osd_info, 0, sizeof(struct osd_info_s));
	do {
		if (!isalpha(*ptr) && !isdigit(*ptr)) {
			find = 1;
			break;
		}
	} while (*++ptr != '\0');
	if (!find)
		return -EINVAL;
	sep[0] = *ptr;
	sep[1] = '\0';
	while ((count--) && (option = strsep(&str, sep))) {
		str2lower(option);
		install_osd_reverse_info(init_osd_info, option);
	}
	return 0;
}

__setup("osd_reverse=", osd_info_setup);
static struct device_attribute osd_attrs[] = {
	__ATTR(scale, S_IRUGO | S_IWUSR | S_IWGRP,
			show_scale, store_scale),
	__ATTR(order, S_IRUGO | S_IWUSR | S_IWGRP,
			show_order, store_order),
	__ATTR(enable_3d, S_IRUGO | S_IWUSR,
			show_enable_3d, store_enable_3d),
	__ATTR(preblend_enable, S_IRUGO | S_IWUSR,
			show_preblend_enable, store_preblend_enable),
	__ATTR(free_scale, S_IRUGO | S_IWUSR | S_IWGRP,
			show_free_scale, store_free_scale),
	__ATTR(scale_axis, S_IRUGO | S_IWUSR,
			show_scale_axis, store_scale_axis),
	__ATTR(scale_width, S_IRUGO | S_IWUSR | S_IWGRP,
			show_scale_width, store_scale_width),
	__ATTR(scale_height, S_IRUGO | S_IWUSR | S_IWGRP,
			show_scale_height, store_scale_height),
	__ATTR(color_key, S_IRUGO | S_IWUSR,
			show_color_key, store_color_key),
	__ATTR(enable_key, S_IRUGO | S_IWUSR | S_IWGRP,
			show_enable_key, store_enable_key),
	__ATTR(enable_key_onhold, S_IRUGO | S_IWUSR | S_IWGRP,
			show_enable_key_onhold, store_enable_key_onhold),
	__ATTR(block_windows, S_IRUGO | S_IWUSR,
			show_block_windows, store_block_windows),
	__ATTR(block_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			show_block_mode, store_block_mode),
	__ATTR(free_scale_axis, S_IRUGO | S_IWUSR | S_IWGRP,
			show_free_scale_axis, store_free_scale_axis),
	__ATTR(debug, S_IRUGO | S_IWUSR,
			show_debug, store_debug),
	__ATTR(log_level, S_IRUGO | S_IWUSR,
			show_log_level, store_log_level),
	__ATTR(window_axis, S_IRUGO | S_IWUSR | S_IWGRP,
			show_window_axis, store_window_axis),
	__ATTR(freescale_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			show_freescale_mode, store_freescale_mode),
	__ATTR(flush_rate, S_IRUGO | S_IRUSR,
			show_flush_rate, NULL),
	__ATTR(prot_on, S_IRUGO | S_IWUSR | S_IWGRP,
			show_rotate_on, store_rotate_on),
	__ATTR(prot_angle, S_IRUGO | S_IWUSR | S_IWGRP,
			show_rotate_angle, store_rotate_angle),
	__ATTR(prot_canvas, S_IRUGO | S_IWUSR | S_IWGRP,
			show_prot_canvas, store_prot_canvas),
	__ATTR(osd_reverse, S_IRUGO | S_IWUSR,
			show_osd_reverse, store_osd_reverse),
	__ATTR(prot_state, S_IRUGO | S_IRUSR,
			show_prot_state, NULL),
	__ATTR(osd_antiflicker, S_IRUGO | S_IWUSR,
			show_antiflicker, store_antiflicker),
	__ATTR(update_freescale, S_IRUGO | S_IWUSR,
			show_update_freescale, store_update_freescale),
	__ATTR(ver_angle, S_IRUGO | S_IWUSR,
			show_ver_angle, store_ver_angle),
	__ATTR(ver_clone, S_IRUGO | S_IWUSR,
			show_ver_clone, store_ver_clone),
	__ATTR(ver_update_pan, S_IWUGO | S_IWUSR, NULL, store_ver_update_pan),
};

#ifdef CONFIG_PM
static int osd_suspend(struct platform_device *dev, pm_message_t state)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif
	osd_suspend_hw();
	return 0;
}

static int osd_resume(struct platform_device *dev)
{
#ifdef CONFIG_SCREEN_ON_EARLY
	if (early_resume_flag) {
		early_resume_flag = 0;
		return 0;
	}
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (early_suspend_flag)
		return 0;
#endif
	osd_resume_hw();
	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void osd_early_suspend(struct early_suspend *h)
{
	if (early_suspend_flag)
		return;
	osd_suspend_hw();
	early_suspend_flag = 1;
}

static void osd_late_resume(struct early_suspend *h)
{
	if (!early_suspend_flag)
		return;
	early_suspend_flag = 0;
	osd_resume_hw();
}
#endif

#ifdef CONFIG_SCREEN_ON_EARLY
void osd_resume_early(void)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	osd_resume_hw();
	early_suspend_flag = 0;
#endif
	early_resume_flag = 1;
	return;
}
EXPORT_SYMBOL(osd_resume_early);
#endif

#ifdef CONFIG_HIBERNATION
static int osd_freeze(struct device *dev)
{
	osd_freeze_hw();
	return 0;
}

static int osd_thaw(struct device *dev)
{
	osd_thaw_hw();
	return 0;
}

static int osd_restore(struct device *dev)
{
	osd_restore_hw();
	return 0;
}
#endif

static int osd_probe(struct platform_device *pdev)
{
	struct fb_info *fbi = NULL;
	const struct vinfo_s *vinfo;
	struct fb_var_screeninfo *var;
	struct fb_fix_screeninfo *fix;
	int  index, bpp;
	struct osd_fb_dev_s *fbdev = NULL;
	enum vmode_e current_mode = VMODE_MASK;
	enum vmode_e initial_mode = VMODE_MASK;
	enum vmode_e logo_mode = VMODE_MASK;
	int logo_index = -1;
	const void *prop;
	int prop_idx = 0;
	int rotation = 0;
	u32 memsize[2];
	int i;
	int ret = 0;

	/* get interrupt resource */
	int_viu_vsync = platform_get_irq_byname(pdev, "viu-vsync");
	if (int_viu_vsync  == -ENXIO) {
		osd_log_err("cannot get viu irq resource\n");
		goto failed1;
	} else
		osd_log_info("viu vsync irq: %d\n", int_viu_vsync);
#ifdef CONFIG_FB_OSD_VSYNC_RDMA
	int_rdma = platform_get_irq_byname(pdev, "rdma");
	if (int_viu_vsync  == -ENXIO) {
		osd_log_err("cannot get osd rdma irq resource\n");
		goto failed1;
	}
#endif
	/* get buffer size from dt */
	ret = of_property_read_u32_array(pdev->dev.of_node,
			"mem_size", memsize, 2);
	if (ret) {
		osd_log_err("not found mem_size from dtd\n");
		goto failed1;
	} else {
		osd_log_dbg("mem_size: 0x%x, 0x%x\n",
				memsize[0], memsize[1]);
		fb_rmem_size[0] = memsize[0];
		fb_rmem_paddr[0] = fb_rmem.base;
		if ((OSD_COUNT == 2) &&
				((memsize[0] + memsize[1]) <= fb_rmem.size)) {
			fb_rmem_size[1] = memsize[1];
			fb_rmem_paddr[1] = fb_rmem.base + memsize[0];
		}
	}
	/* init reserved memory */
	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret != 0) {
		osd_log_err("failed to init reserved memory\n");
		goto failed1;
	}
	/* get meson-fb resource from dt */
	prop = of_get_property(pdev->dev.of_node, "scale_mode", NULL);
	if (prop)
		prop_idx = of_read_ulong(prop, 1);
	osd_set_free_scale_mode_hw(DEV_OSD0, prop_idx);
	prop = of_get_property(pdev->dev.of_node, "4k2k_fb", NULL);
	if (prop)
		prop_idx = of_read_ulong(prop, 1);
	osd_set_4k2k_fb_mode_hw(prop_idx);
	/* get vmode from dt */
	prop = of_get_property(pdev->dev.of_node, "vmode", NULL);
	if (prop)
		prop_idx = of_read_ulong(prop, 1);
	switch (prop_idx) {
	case 1:
		current_mode = VMODE_LCD;
		break;
	case 2:
		current_mode = VMODE_LVDS_1080P;
		break;
	case 3:
		current_mode = VMODE_1080P;
		break;
	case 4:
		current_mode = VMODE_720P;
		break;
	default:
		current_mode = VMODE_MASK;
		break;
	}
	/* if logo vmode not set, set vmode and init osd hw */
	logo_mode = get_logo_vmode();
	logo_index = osd_get_logo_index();
	if (logo_mode >= VMODE_MAX) {
		initial_mode = get_initial_vmode();
		if (initial_mode < VMODE_MAX) {
			current_mode = initial_mode;
			set_current_vmode(current_mode);
		}
		osd_init_hw(0);
		if (current_mode < VMODE_MAX)
			vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE,
				&current_mode);
	}

	vinfo = get_current_vinfo();
	osd_log_info("%s vinfo:%p\n", __func__, vinfo);

	for (index = 0; index < OSD_COUNT; index++) {
		/* register frame buffer memory */
		fbi = framebuffer_alloc(sizeof(struct osd_fb_dev_s),
				&pdev->dev);
		if (!fbi) {
			ret = -ENOMEM;
			goto failed1;
		}
		fbdev = (struct osd_fb_dev_s *)fbi->par;
		fbdev->fb_info = fbi;
		fbdev->dev = pdev;
		mutex_init(&fbdev->lock);
		var = &fbi->var;
		fix = &fbi->fix;
		gp_fbdev_list[index] = fbdev;
		fbdev->fb_len = fb_rmem_size[index];
		fbdev->fb_mem_paddr = fb_rmem_paddr[index];
		fbdev->fb_mem_vaddr = fb_rmem_vaddr[index];

		if (!fbdev->fb_mem_vaddr) {
			osd_log_err("failed to ioremap frame buffer\n");
			ret = -ENOMEM;
			goto failed1;
		}
		osd_log_info("Frame buffer memory assigned at");
		osd_log_info("    phy: 0x%08x, vir:0x%p, size=%dK\n",
			     fbdev->fb_mem_paddr,
			     fbdev->fb_mem_vaddr,
			     fbdev->fb_len >> 10);
		if (vinfo) {
			fb_def_var[index].width = vinfo->screen_real_width;
			fb_def_var[index].height = vinfo->screen_real_height;
		}

		/* setup fb0 display size */
		if (index == DEV_OSD0) {
			ret = of_property_read_u32_array(pdev->dev.of_node,
					"display_size_default",
					&var_screeninfo[0], 5);
			if (ret)
				osd_log_info("not found display_size_default\n");
			else {
#if !defined(CONFIG_ARCH_MESON64_ODROIDC2)
				fb_def_var[index].xres = var_screeninfo[0];
				fb_def_var[index].yres = var_screeninfo[1];
				fb_def_var[index].xres_virtual =
					var_screeninfo[2];
				fb_def_var[index].yres_virtual =
					var_screeninfo[3];
				fb_def_var[index].bits_per_pixel =
					var_screeninfo[4];
				osd_log_info("init fbdev bpp is:%d\n",
					fb_def_var[index].bits_per_pixel);
				if (fb_def_var[index].bits_per_pixel > 32)
					fb_def_var[index].bits_per_pixel = 32;

#else /* CONFIG_ARCH_MESON64_ODROIDC2 */

				if (osd_set_res_bootargs(index, current_mode)) {
					fb_def_var[index].xres =
						var_screeninfo[0];
					fb_def_var[index].yres =
						var_screeninfo[1];
					fb_def_var[index].xres_virtual =
						var_screeninfo[2];
					fb_def_var[index].yres_virtual =
						var_screeninfo[3];
					fb_def_var[index].bits_per_pixel =
						var_screeninfo[4];
				}
				osd_log_info("fb def : %d %d %d %d %d\n",
					fb_def_var[index].xres,
					fb_def_var[index].yres,
					fb_def_var[index].xres_virtual,
					fb_def_var[index].yres_virtual,
					fb_def_var[index].bits_per_pixel);

				osd_log_info("init fbdev bpp is:%d\n",
					fb_def_var[index].bits_per_pixel);

				if (fb_def_var[index].bits_per_pixel > 32)
					fb_def_var[index].bits_per_pixel = 32;
#endif /* CONFIG_ARCH_MESON64_ODROIDC2 */
			}
		}
		/* clear osd buffer if not logo layer */
		if ((logo_index < 0) || (logo_index != index)) {
			osd_log_info("---------------clear fb%d memory\n",
					index);
			memset((char *)fbdev->fb_mem_vaddr, 0x0,
					fbdev->fb_len);
		}

		/* get roataion from dtd */
		if (index == DEV_OSD0) {
			prop = of_get_property(pdev->dev.of_node,
					"rotation", NULL);
			if (prop)
				prop_idx = of_read_ulong(prop, 1);
			rotation = prop_idx;
		}
		_fbdev_set_default(fbdev, index);
		if (NULL == fbdev->color) {
			osd_log_err("fbdev->color NULL\n");
			ret = -ENOENT;
			goto failed1;
		}
		bpp = (fbdev->color->color_index > 8 ?
				(fbdev->color->color_index > 16 ?
				(fbdev->color->color_index > 24 ?
				 4 : 3) : 2) : 1);
		fix->line_length = var->xres_virtual * bpp;
		fix->smem_start = fbdev->fb_mem_paddr;
		fix->smem_len = fbdev->fb_len;
		if (fb_alloc_cmap(&fbi->cmap, 16, 0) != 0) {
			osd_log_err("unable to allocate color map memory\n");
			ret = -ENOMEM;
			goto failed2;
		}
		fbi->pseudo_palette = kmalloc(sizeof(u32) * 16, GFP_KERNEL);
		if (!(fbi->pseudo_palette)) {
			osd_log_err("unable to allocate pseudo palette mem\n");
			ret = -ENOMEM;
			goto failed2;
		}
		memset(fbi->pseudo_palette, 0, sizeof(u32) * 16);
		fbi->fbops = &osd_ops;
		fbi->screen_base = (char __iomem *)fbdev->fb_mem_vaddr;
		fbi->screen_size = fix->smem_len;
		if (vinfo)
			set_default_display_axis(&fbdev->fb_info->var,
					&fbdev->osd_ctl, vinfo);
		osd_check_var(var, fbi);
		/* register frame buffer */
		register_framebuffer(fbi);

		/* setup osd if not logo layer */
		if ((logo_index < 0) || (logo_index != index))
			osddev_setup(fbdev);

		/* create device attribute files */
		for (i = 0; i < ARRAY_SIZE(osd_attrs); i++)
			ret = device_create_file(fbi->dev, &osd_attrs[i]);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	early_suspend.suspend = osd_early_suspend;
	early_suspend.resume = osd_late_resume;
	register_early_suspend(&early_suspend);
#endif
	/* init osd rotate */
	if (rotation == 90 || rotation == 270) {
		osd_set_prot_canvas_hw(0, 0, 0,
				var_screeninfo[0] - 1, var_screeninfo[1] - 1);
		if (rotation == 90)
			osd_set_rotate_angle_hw(0, 1);
		else {
			osd_set_rotate_angle_hw(0, 2);
			osd_set_rotate_on_hw(0, 1);
		}
	}
	/* init osd reverse */
	if (osd_info.index == DEV_ALL) {
		osd_set_reverse_hw(0, osd_info.osd_reverse);
		osd_set_reverse_hw(1, osd_info.osd_reverse);
	} else
		osd_set_reverse_hw(osd_info.index, osd_info.osd_reverse);

	/* register vout client */
	vout_register_client(&osd_notifier_nb);

	osd_log_info("osd probe OK\n");
	return 0;
failed2:
	fb_dealloc_cmap(&fbi->cmap);
failed1:
	osd_log_err("osd probe failed.\n");
	return ret;
}

static int osd_remove(struct platform_device *pdev)
{
	struct fb_info *fbi;
	int i = 0;
	osd_log_info("osd_remove.\n");
	if (!pdev)
		return -ENODEV;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif
	vout_unregister_client(&osd_notifier_nb);
	for (i = 0; i < OSD_COUNT; i++) {
		int j;
		if (gp_fbdev_list[i]) {
			struct osd_fb_dev_s *fbdev = gp_fbdev_list[i];
			fbi = fbdev->fb_info;
			for (j = 0; j < ARRAY_SIZE(osd_attrs); j++)
				device_remove_file(fbi->dev, &osd_attrs[j]);
			iounmap(fbdev->fb_mem_vaddr);
			kfree(fbi->pseudo_palette);
			fb_dealloc_cmap(&fbi->cmap);
			unregister_framebuffer(fbi);
			framebuffer_release(fbi);
		}
	}
	return 0;
}

/* Process kernel command line parameters */
static int __init osd_setup_attribute(char *options)
{
	char *this_opt = NULL;
	int r = 0;
	unsigned long res;

	if (!options || !*options)
		goto exit;
	while (!r && (this_opt = strsep(&options, ",")) != NULL) {
		if (!strncmp(this_opt, "xres:", 5)) {
			r = kstrtol(this_opt + 5, 0, &res);
			fb_def_var[0].xres = fb_def_var[0].xres_virtual = res;
		} else if (!strncmp(this_opt, "yres:", 5)) {
			r = kstrtol(this_opt + 5, 0, &res);
			fb_def_var[0].yres = fb_def_var[0].yres_virtual = res;
		} else {
			osd_log_info("invalid option\n");
			r = -1;
		}
	}
exit:
	return r;
}

static int rmem_fb_device_init(struct reserved_mem *rmem, struct device *dev)
{
	int i = 0;
	for (i = 0; i < OSD_COUNT; i++) {
		if ((fb_rmem_paddr[i] > 0) && (fb_rmem_size[i] > 0)) {
			fb_rmem_vaddr[i] =
				ioremap_wc(fb_rmem_paddr[i], fb_rmem_size[i]);
			if (!fb_rmem_vaddr[i])
				osd_log_err("fb[%d] ioremap error", i);
		}
	}
	return 0;
}

static const struct reserved_mem_ops rmem_fb_ops = {
	.device_init = rmem_fb_device_init,
};

static int __init rmem_fb_setup(struct reserved_mem *rmem)
{
	phys_addr_t align = PAGE_SIZE;
	phys_addr_t mask = align - 1;
	if ((rmem->base & mask) || (rmem->size & mask)) {
		pr_err("Reserved memory: incorrect alignment of region\n");
		return -EINVAL;
	}
	fb_rmem.base = rmem->base;
	fb_rmem.size = rmem->size;
	rmem->ops = &rmem_fb_ops;
	osd_log_info("Reserved memory: created fb at 0x%p, size %ld MiB\n",
		     (void *)rmem->base, (unsigned long)rmem->size / SZ_1M);
	return 0;
}
RESERVEDMEM_OF_DECLARE(fb, "amlogic, fb-memory", rmem_fb_setup);

static const struct of_device_id meson_fb_dt_match[] = {
	{.compatible = "amlogic, meson-fb",},
	{},
};

#ifdef CONFIG_HIBERNATION
const struct dev_pm_ops osd_pm = {
	.freeze		= osd_freeze,
	.thaw		= osd_thaw,
	.restore	= osd_restore,
};
#endif

static struct platform_driver osd_driver = {
	.probe      = osd_probe,
	.remove     = osd_remove,
#ifdef CONFIG_PM
	.suspend  = osd_suspend,
	.resume    = osd_resume,
#endif
	.driver     = {
		.name   = "meson-fb",
		.of_match_table = meson_fb_dt_match,
#ifdef CONFIG_HIBERNATION
		.pm     = &osd_pm,
#endif
	}
};

static int __init osd_init_module(void)
{
	char *option;
	osd_log_info("%s\n", __func__);
	if (fb_get_options("meson-fb", &option)) {
		osd_log_err("failed to get meson-fb options!\n");
		return -ENODEV;
	}
	osd_setup_attribute(option);
	if (platform_driver_register(&osd_driver)) {
		osd_log_err("failed to register OSD driver!\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit osd_exit_module(void)
{
	osd_log_info("%s\n", __func__);
	platform_driver_unregister(&osd_driver);
}

module_init(osd_init_module);
module_exit(osd_exit_module);

MODULE_AUTHOR("Platform-BJ <platform.bj@amlogic.com>");
MODULE_DESCRIPTION("OSD Module");
MODULE_LICENSE("GPL");
