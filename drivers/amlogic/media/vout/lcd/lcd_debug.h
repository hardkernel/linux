/*
 * drivers/amlogic/media/vout/lcd/lcd_debug.h
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

#ifndef __AML_LCD_DEBUG_H__
#define __AML_LCD_DEBUG_H__
#include "lcd_reg.h"

#define LCD_DEBUG_REG_CNT_MAX    30
#define LCD_DEBUG_REG_END        0xffffffff

struct lcd_debug_info_reg_s {
	unsigned int *reg_clk_table;
	unsigned int *reg_encl_table;
	unsigned int *reg_pinmux_table;
};

struct lcd_debug_info_if_s {
	int (*interface_print)(char *buf, int offset);
	int (*reg_dump_interface)(char *buf, int offset);
	int (*reg_dump_phy)(char *buf, int offset);
	struct class_attribute *class_attrs;
};

static unsigned int lcd_reg_dump_clk_dft[] = {
	HHI_HDMI_PLL_CNTL,
	HHI_HDMI_PLL_CNTL2,
	HHI_HDMI_PLL_CNTL3,
	HHI_HDMI_PLL_CNTL4,
	HHI_HDMI_PLL_CNTL5,
	HHI_HDMI_PLL_CNTL6,
	HHI_VID_PLL_CLK_DIV,
	HHI_VIID_CLK_DIV,
	HHI_VIID_CLK_CNTL,
	HHI_VID_CLK_CNTL2,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_clk_axg[] = {
	HHI_GP0_PLL_CNTL_AXG,
	HHI_GP0_PLL_CNTL2_AXG,
	HHI_GP0_PLL_CNTL3_AXG,
	HHI_GP0_PLL_CNTL4_AXG,
	HHI_GP0_PLL_CNTL5_AXG,
	HHI_GP0_PLL_CNTL1_AXG,
	HHI_VIID_CLK_DIV,
	HHI_VIID_CLK_CNTL,
	HHI_VID_CLK_CNTL2,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_clk_gp0_g12a[] = {
	HHI_GP0_PLL_CNTL0_G12A,
	HHI_GP0_PLL_CNTL1_G12A,
	HHI_GP0_PLL_CNTL2_G12A,
	HHI_GP0_PLL_CNTL3_G12A,
	HHI_GP0_PLL_CNTL4_G12A,
	HHI_GP0_PLL_CNTL5_G12A,
	HHI_GP0_PLL_CNTL6_G12A,
	HHI_VIID_CLK_DIV,
	HHI_VIID_CLK_CNTL,
	HHI_VID_CLK_CNTL2,
	HHI_MIPIDSI_PHY_CLK_CNTL,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_clk_hpll_g12a[] = {
	HHI_HDMI_PLL_CNTL,
	HHI_HDMI_PLL_CNTL2,
	HHI_HDMI_PLL_CNTL3,
	HHI_HDMI_PLL_CNTL4,
	HHI_HDMI_PLL_CNTL5,
	HHI_HDMI_PLL_CNTL6,
	HHI_HDMI_PLL_CNTL7,
	HHI_VID_PLL_CLK_DIV,
	HHI_VIID_CLK_DIV,
	HHI_VIID_CLK_CNTL,
	HHI_VID_CLK_CNTL2,
	HHI_MIPIDSI_PHY_CLK_CNTL,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_clk_tl1[] = {
	HHI_TCON_PLL_CNTL0,
	HHI_TCON_PLL_CNTL1,
	HHI_TCON_PLL_CNTL2,
	HHI_TCON_PLL_CNTL3,
	HHI_TCON_PLL_CNTL4,
	HHI_VID_PLL_CLK_DIV,
	HHI_VIID_CLK_DIV,
	HHI_VIID_CLK_CNTL,
	HHI_VID_CLK_CNTL2,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_encl_dft[] = {
	VPU_VIU_VENC_MUX_CTRL,
	ENCL_VIDEO_EN,
	ENCL_VIDEO_MODE,
	ENCL_VIDEO_MODE_ADV,
	ENCL_VIDEO_MAX_PXCNT,
	ENCL_VIDEO_MAX_LNCNT,
	ENCL_VIDEO_HAVON_BEGIN,
	ENCL_VIDEO_HAVON_END,
	ENCL_VIDEO_VAVON_BLINE,
	ENCL_VIDEO_VAVON_ELINE,
	ENCL_VIDEO_HSO_BEGIN,
	ENCL_VIDEO_HSO_END,
	ENCL_VIDEO_VSO_BEGIN,
	ENCL_VIDEO_VSO_END,
	ENCL_VIDEO_VSO_BLINE,
	ENCL_VIDEO_VSO_ELINE,
	ENCL_VIDEO_RGBIN_CTRL,
	L_GAMMA_CNTL_PORT,
	L_RGB_BASE_ADDR,
	L_RGB_COEFF_ADDR,
	L_POL_CNTL_ADDR,
	L_DITH_CNTL_ADDR,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_encl_tl1[] = {
	VPU_VIU_VENC_MUX_CTRL,
	ENCL_VIDEO_EN,
	ENCL_VIDEO_MODE,
	ENCL_VIDEO_MODE_ADV,
	ENCL_VIDEO_MAX_PXCNT,
	ENCL_VIDEO_MAX_LNCNT,
	ENCL_VIDEO_HAVON_BEGIN,
	ENCL_VIDEO_HAVON_END,
	ENCL_VIDEO_VAVON_BLINE,
	ENCL_VIDEO_VAVON_ELINE,
	ENCL_VIDEO_HSO_BEGIN,
	ENCL_VIDEO_HSO_END,
	ENCL_VIDEO_VSO_BEGIN,
	ENCL_VIDEO_VSO_END,
	ENCL_VIDEO_VSO_BLINE,
	ENCL_VIDEO_VSO_ELINE,
	ENCL_VIDEO_RGBIN_CTRL,
	ENCL_INBUF_CNTL0,
	ENCL_INBUF_CNTL1,
	L_GAMMA_CNTL_PORT,
	L_RGB_BASE_ADDR,
	L_RGB_COEFF_ADDR,
	L_POL_CNTL_ADDR,
	L_DITH_CNTL_ADDR,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_pinmux_gxl[] = {
	PERIPHS_PIN_MUX_1,
	PERIPHS_PIN_MUX_3,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_pinmux_txl[] = {
	PERIPHS_PIN_MUX_0,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_pinmux_txlx[] = {
	PERIPHS_PIN_MUX_0,
	PERIPHS_PIN_MUX_8,
	LCD_DEBUG_REG_END,
};

static unsigned int lcd_reg_dump_pinmux_tl1[] = {
	PERIPHS_PIN_MUX_7_TL1,
	PERIPHS_PIN_MUX_8_TL1,
	PERIPHS_PIN_MUX_9_TL1,
	LCD_DEBUG_REG_END,
};

#endif
