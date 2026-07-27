// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#ifndef _EBC_TCON_H_
#define _EBC_TCON_H_

#include "../ebc_panel.h"

/* update mode */
#define NORMAL_UPDATE	0
#define DIFF_UPDATE	1

/* display mode */
#define DIRECT_MODE	0
#define LUT_MODE	1
#define THREE_WIN_MODE	1
#define EINK_MODE	1

enum ebc_tcon_data_format {
	EBC_Y4_FORMAT = 0,
	EBC_Y8_FORMAT = 1,
	EBC_RGB565_FORMAT = 2,
	EBC_XBGR8888_FORMAT = 3,
	EBC_Y5_FORMAT = 4,
	EBC_Y1_FORMAT = 5,
};

enum ebc_tcon_version {
	EBC_VERSION_RK3568 = 0,
	EBC_VERSION_RK3576 = 1,
	EBC_VERSION_RK3572 = 2,
	EBC_VERSION_RK3506 = 3,
	EBC_VERSION_PX30 = 4,
};

struct ebc_tcon {
	struct device *dev;
	void __iomem *regs;
	unsigned int *regcache; /* register cache */
	unsigned int len;
	int irq;

	struct clk *aclk;
	struct clk *hclk;
	struct clk *dclk;
	struct regmap *regmap_base;
	struct regmap *grf;
	struct phy *phy;

	struct ebc_panel *panel;
	int display_mode;
	u32 version;

	int low_8bit_offset;
	int high_8bit_offset;

	u32 line_rel;
	u32 width;
	u32 height;
	u32 bytes_per_pixel;
	u32 bytes_per_row;

	struct panel_buffer buf;
	void *priv;

	u32 lut_offset;

	int (*enable)(struct ebc_tcon *tcon, struct ebc_panel *panel);
	void (*disable)(struct ebc_tcon *tcon);
	void (*dsp_mode_set)(struct ebc_tcon *tcon, int update_mode, int display_mode, int three_win_mode, int eink_mode);
	void (*image_addr_set)(struct ebc_tcon *tcon, u32 pre_image_addr, u32 cur_image_addr);
	void (*frame_addr_set)(struct ebc_tcon *tcon, u32 frame_addr);
	int (*lut_data_set)(struct ebc_tcon *tcon, unsigned int *lut_data, int frame_count, int lut_32);
	void (*data_format_set)(struct ebc_tcon *tcon, enum ebc_tcon_data_format format);
	void (*frame_start)(struct ebc_tcon *tcon, int frame_total);
	void (*dsp_end_callback)(void);
	void (*set_line_flag_event)(struct ebc_tcon *tcon, u32 line, bool enable);
	void (*line_flag_callback)(void);
	int (*get_version)(struct ebc_tcon *tcon);
	int (*clk_set_rate)(struct clk *clk, unsigned long rate);
};

static inline int ebc_tcon_get_version(struct ebc_tcon *tcon)
{
	return tcon->get_version(tcon);
}

static inline int ebc_tcon_enable(struct ebc_tcon *tcon, struct ebc_panel *panel)
{
	return tcon->enable(tcon, panel);
}

static inline void ebc_tcon_disable(struct ebc_tcon *tcon)
{
	tcon->disable(tcon);
}

static inline void ebc_tcon_dsp_mode_set(struct ebc_tcon *tcon, int update_mode,
					 int display_mode, int three_win_mode, int eink_mode)
{
	return tcon->dsp_mode_set(tcon, update_mode, display_mode, three_win_mode, eink_mode);
}

static inline void ebc_tcon_image_addr_set(struct ebc_tcon *tcon, u32 pre_image_addr, u32 cur_image_addr)
{
	tcon->image_addr_set(tcon, pre_image_addr, cur_image_addr);
}

static inline void ebc_tcon_frame_addr_set(struct ebc_tcon *tcon, u32 frame_addr)
{
	tcon->frame_addr_set(tcon, frame_addr);
}

static inline int ebc_tcon_lut_data_set(struct ebc_tcon *tcon, unsigned int *lut_data, int frame_count, int lut_32)
{
	return tcon->lut_data_set(tcon, lut_data, frame_count, lut_32);
}

static inline void ebc_tcon_frame_start(struct ebc_tcon *tcon, int frame_total)
{
	tcon->frame_start(tcon, frame_total);
}

static inline void ebc_tcon_data_format_set(struct ebc_tcon *tcon, enum ebc_tcon_data_format format)
{
	tcon->data_format_set(tcon, format);
}

static inline void ebc_tcon_set_line_flag_event(struct ebc_tcon *tcon, u32 line, bool enable)
{
	tcon->set_line_flag_event(tcon, line, enable);
}
#endif
