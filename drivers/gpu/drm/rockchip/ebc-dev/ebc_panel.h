// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#ifndef _EBC_PANEL_H_
#define _EBC_PANEL_H_

#define DIRECT_FB_NUM	2

enum ebc_panel_bit_depth {
	EBC_PANEL_8BIT = 0,
	EBC_PANEL_16BIT = 1,
	EBC_PANEL_24BIT = 2,
};

#define PANEL_IS_8BIT(panel) ((panel)->panel_bit_depth == EBC_PANEL_8BIT)
#define PANEL_IS_16BIT(panel) ((panel)->panel_bit_depth == EBC_PANEL_16BIT)
#define PANEL_IS_24BIT(panel) ((panel)->panel_bit_depth == EBC_PANEL_24BIT)

struct panel_buffer {
	void *virt_addr;
	unsigned long phy_addr;
	size_t size;
};

struct ebc_panel {
	struct device *dev;
	struct ebc_tcon *tcon;
	struct ebc_pmic *pmic;
	struct panel_buffer fb[DIRECT_FB_NUM]; //for direct mode, one pixel 2bit
	int current_buffer;

	u32 width;
	u32 height;
	u32 vir_width;
	u32 vir_height;
	u32 width_mm;
	u32 height_mm;
	u32 direct_mode;
	u32 sdck;
	u32 lsl;
	u32 lbl;
	u32 ldl;
	u32 lel;
	u32 gdck_sta;
	u32 lgonl;
	u32 fsl;
	u32 fbl;
	u32 fdl;
	u32 fel;
	u32 panel_bit_depth;
	u32 panel_color;
	u32 mirror;
	u32 rearrange;
	u32 gate_dummy_start;
	u32 gate_dummy_lenth;
	u32 sdoe_mode;
	u32 sdce_width;
	u32 lel_keep_clk;
	bool pmic_early_power_on;
	u32 data_rate;
};
#endif
