// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#ifndef __EBC_DEV_H__
#define __EBC_DEV_H__

#include <linux/ebc.h>
#include <linux/notifier.h>

/*
* max support panel size 2560x1920
* ebc module display buf use 4bit per pixel
* ebook module display buf use 8bit per pixel
* ebc module direct mode display buf use 2bit per pixel
*/
#define LUT_TABLE_SIZE		0x100000 /* 1M */

#define MAX_FB_NUM		4

#define EBC_SUCCESS		(0)
#define EBC_ERROR		(-1)

#define WF_4BIT		16
#define WF_5BIT		32

/*
* ebc buf format
*/
#define EBC_Y4 (0)
#define EBC_Y8 (1)

/*
 * ebc system ioctl command
 */
#define EBC_GET_BUFFER				(0x7000)
#define EBC_SEND_BUFFER			(0x7001)
#define EBC_GET_BUFFER_INFO		(0x7002)
#define EBC_SET_FULL_MODE_NUM	(0x7003)
#define EBC_ENABLE_OVERLAY		(0x7004)
#define EBC_DISABLE_OVERLAY		(0x7005)
#define EBC_GET_OSD_BUFFER		(0x7006)
#define EBC_SEND_OSD_BUFFER		(0x7007)
#define EBC_NEW_BUF_PREPARE		(0x7008)
#define EBC_SET_DIFF_PERCENT		(0x7009)
#define EBC_WAIT_NEW_BUF_TIME	(0x700a)
#define EBC_GET_OVERLAY_STATUS	(0x700b)
#define EBC_ENABLE_BG_CONTROL	(0x700c)
#define EBC_DISABLE_BG_CONTROL	(0x700d)
#define EBC_ENABLE_RESUME_COUNT	(0x700e)
#define EBC_DISABLE_RESUME_COUNT	(0x700f)
#define EBC_GET_BUF_FORMAT		(0x7010)
#define EBC_DROP_PREV_BUFFER		(0x7011)
#define EBC_GET_STATUS				(0x7012)
#define EBC_SET_FB_BLANK				(0x7013)
#define EBC_SET_FB_UNBLANK			(0x7014)
#define EBC_BW_ENABLE_REPAIR			(0x7015)
#define EBC_BW_GHOST_RM_LEVEL			(0x7016)
#define EBC_GET_NORMAL_REPAIR		(0x7017)
#define EBC_SET_TRANSITION_WIDTH	(0x7018)
#define EBC_SET_TRANSITION_TYPE		(0x7019)
#define EBC_SET_FORCE_TEMPERATURE	(0x701a)
#define EBC_ENABLE_WAVEFORM_FIX		(0x701b)
#define EBC_SET_REGAL_TYPE		(0x701c)
#define EBC_FORCE_FULL_USE_REGAL	(0x701d)
#define EBC_GET_PREV_BUFFER			(0x701e)
#define EBC_GET_SUPPORT_REGAL			(0x701f)
#define EBC_ENABLE_NORMAL_REPAIR		(0x7020)
#define EBC_BW_BALANCE_CTL		(0x7021)
#define EBC_SET_TCON_UPDATE_MODE	(0x7022)
#define EBC_GET_WAVEFORM_VERSION	(0x7023)
#define EBC_BW_GHOST_RM_TYPE		(0x7024)
#define EBC_PEN_PROXIMITY		(0x7025)

enum transition_type {
	DISABLE_TRANSITION_REFRESH,
	CIRCULAR_OUTWARD_REFRESH,
	DIAMOND_OUTWARD_REFRESH,
	RECTANGLE_OUTWARD_REFRESH,
	CROSS_OUTWARD_REFRESH,
	SPIRAL_OUTWARD_REFRESH,
	LINE_0_DEGREES_REFRESH,
	LINE_90_DEGREES_REFRESH,
	LINE_180_DEGREES_REFRESH,
	LINE_270_DEGREES_REFRESH,
	LUT_REFRESH,
	MAX_TRANSITION_REFRESH,
};

/*
 * IMPORTANT: Those values is corresponding to android hardware program,
 * so *FORBID* to changes bellow values, unless you know what you're doing.
 * And if you want to add new refresh modes, please appended to the tail.
 */
enum panel_refresh_mode {
	EPD_AUTO		= 0,
	EPD_OVERLAY		= 1,
	EPD_FULL_GC16		= 2,
	EPD_FULL_GL16		= 3,
	EPD_FULL_GLR16		= 4,
	EPD_FULL_GLD16		= 5,
	EPD_FULL_GCC16		= 6,
	EPD_PART_GC16		= 7,
	EPD_PART_GL16		= 8,
	EPD_PART_GLR16		= 9,
	EPD_PART_GLD16		= 10,
	EPD_PART_GCC16		= 11,
	EPD_A2			= 12,
	EPD_A2_FAST	        = 13,
	EPD_DU			= 14,
	EPD_DU4			= 15,
	EPD_A2_ENTER		= 16,
	EPD_RESET		= 17,
	EPD_SUSPEND		= 18,
	EPD_RESUME		= 19,
	EPD_POWER_OFF		= 20,
	EPD_FORCE_FULL		= 21,
	EPD_AUTO_DU		= 22,
	EPD_AUTO_DU4		= 23,
};

/*
 * IMPORTANT: android hardware use struct, so *FORBID* to changes this, unless you know what you're doing.
 */
struct ebc_buf_info {
	int offset;
	int epd_mode;
	int height;
	int width;
	int panel_color;
	int win_x1;
	int win_y1;
	int win_x2;
	int win_y2;
	int width_mm;
	int height_mm;
	int undroppable;	//0: can be drop by userspace, 1: can't be drop by userspace
	char tid_name[16];
	int dma_buf_fd;
};

enum ebc_debug_level {
	DEBUG_LEVEL_0,/*no debug info*/
	DEBUG_LEVEL_1,/*only print dev_dbg info*/
	DEBUG_LEVEL_2,/*print dev_dbg and dev_vdbg info*/
	DEBUG_LEVEL_NUM
};

int ebc_get_init_log_level(void);
int ebc_overlay_enable_control(void);
int ebc_overlay_enable_timeout_ms(void);
#endif
