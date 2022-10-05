/*
 * drivers/amlogic/media/vout/lcd/lcd_tablet/lcd_drv.c
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
#ifdef CONFIG_AMLOGIC_VPU
#include <linux/amlogic/media/vpu/vpu.h>
#endif
#include <linux/amlogic/media/vout/vinfo.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/media/vout/lcd/lcd_notify.h>
#include "lcd_tablet.h"
#include "mipi_dsi_util.h"
#include "../lcd_reg.h"
#include "../lcd_common.h"

static int lcd_type_supported(struct lcd_config_s *pconf)
{
	int lcd_type = pconf->lcd_basic.lcd_type;
	int ret = -1;

	switch (lcd_type) {
	case LCD_TTL:
	case LCD_LVDS:
	case LCD_VBYONE:
	case LCD_MIPI:
		ret = 0;
		break;
	default:
		LCDERR("invalid lcd type: %s(%d)\n",
			lcd_type_type_to_str(lcd_type), lcd_type);
		break;
	}
	return ret;
}

static void lcd_encl_tcon_set(struct lcd_config_s *pconf)
{
	struct lcd_timing_s *tcon_adr = &pconf->lcd_timing;

	lcd_vcbus_write(L_RGB_BASE_ADDR, 0);
	lcd_vcbus_write(L_RGB_COEFF_ADDR, 0x400);
	aml_lcd_notifier_call_chain(LCD_EVENT_GAMMA_UPDATE, NULL);

	switch (pconf->lcd_basic.lcd_bits) {
	case 6:
		lcd_vcbus_write(L_DITH_CNTL_ADDR,  0x600);
		break;
	case 8:
		lcd_vcbus_write(L_DITH_CNTL_ADDR,  0x400);
		break;
	case 10:
	default:
		lcd_vcbus_write(L_DITH_CNTL_ADDR,  0x0);
		break;
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		lcd_vcbus_setb(L_POL_CNTL_ADDR, 1, 0, 1);
		if (pconf->lcd_timing.vsync_pol)
			lcd_vcbus_setb(L_POL_CNTL_ADDR, 1, 1, 1);
		break;
	case LCD_VBYONE:
		if (pconf->lcd_timing.hsync_pol)
			lcd_vcbus_setb(L_POL_CNTL_ADDR, 1, 0, 1);
		if (pconf->lcd_timing.vsync_pol)
			lcd_vcbus_setb(L_POL_CNTL_ADDR, 1, 1, 1);
		break;
	case LCD_MIPI:
		/* lcd_vcbus_setb(L_POL_CNTL_ADDR, 3, 0, 2); */
		break;
	default:
		break;
	}

	/* DE signal for TTL m8,m8m2 */
	lcd_vcbus_write(L_OEH_HS_ADDR, tcon_adr->de_hs_addr);
	lcd_vcbus_write(L_OEH_HE_ADDR, tcon_adr->de_he_addr);
	lcd_vcbus_write(L_OEH_VS_ADDR, tcon_adr->de_vs_addr);
	lcd_vcbus_write(L_OEH_VE_ADDR, tcon_adr->de_ve_addr);
	/* DE signal for TTL m8b */
	lcd_vcbus_write(L_OEV1_HS_ADDR,  tcon_adr->de_hs_addr);
	lcd_vcbus_write(L_OEV1_HE_ADDR,  tcon_adr->de_he_addr);
	lcd_vcbus_write(L_OEV1_VS_ADDR,  tcon_adr->de_vs_addr);
	lcd_vcbus_write(L_OEV1_VE_ADDR,  tcon_adr->de_ve_addr);

	/* Hsync signal for TTL m8,m8m2 */
	if (tcon_adr->hsync_pol == 0) {
		lcd_vcbus_write(L_STH1_HS_ADDR, tcon_adr->hs_he_addr);
		lcd_vcbus_write(L_STH1_HE_ADDR, tcon_adr->hs_hs_addr);
	} else {
		lcd_vcbus_write(L_STH1_HS_ADDR, tcon_adr->hs_hs_addr);
		lcd_vcbus_write(L_STH1_HE_ADDR, tcon_adr->hs_he_addr);
	}
	lcd_vcbus_write(L_STH1_VS_ADDR, tcon_adr->hs_vs_addr);
	lcd_vcbus_write(L_STH1_VE_ADDR, tcon_adr->hs_ve_addr);

	/* Vsync signal for TTL m8,m8m2 */
	lcd_vcbus_write(L_STV1_HS_ADDR, tcon_adr->vs_hs_addr);
	lcd_vcbus_write(L_STV1_HE_ADDR, tcon_adr->vs_he_addr);
	if (tcon_adr->vsync_pol == 0) {
		lcd_vcbus_write(L_STV1_VS_ADDR, tcon_adr->vs_ve_addr);
		lcd_vcbus_write(L_STV1_VE_ADDR, tcon_adr->vs_vs_addr);
	} else {
		lcd_vcbus_write(L_STV1_VS_ADDR, tcon_adr->vs_vs_addr);
		lcd_vcbus_write(L_STV1_VE_ADDR, tcon_adr->vs_ve_addr);
	}

	/* DE signal */
	lcd_vcbus_write(L_DE_HS_ADDR,    tcon_adr->de_hs_addr);
	lcd_vcbus_write(L_DE_HE_ADDR,    tcon_adr->de_he_addr);
	lcd_vcbus_write(L_DE_VS_ADDR,    tcon_adr->de_vs_addr);
	lcd_vcbus_write(L_DE_VE_ADDR,    tcon_adr->de_ve_addr);

	/* Hsync signal */
	lcd_vcbus_write(L_HSYNC_HS_ADDR,  tcon_adr->hs_hs_addr);
	lcd_vcbus_write(L_HSYNC_HE_ADDR,  tcon_adr->hs_he_addr);
	lcd_vcbus_write(L_HSYNC_VS_ADDR,  tcon_adr->hs_vs_addr);
	lcd_vcbus_write(L_HSYNC_VE_ADDR,  tcon_adr->hs_ve_addr);

	/* Vsync signal */
	lcd_vcbus_write(L_VSYNC_HS_ADDR,  tcon_adr->vs_hs_addr);
	lcd_vcbus_write(L_VSYNC_HE_ADDR,  tcon_adr->vs_he_addr);
	lcd_vcbus_write(L_VSYNC_VS_ADDR,  tcon_adr->vs_vs_addr);
	lcd_vcbus_write(L_VSYNC_VE_ADDR,  tcon_adr->vs_ve_addr);

	lcd_vcbus_write(L_INV_CNT_ADDR, 0);
	lcd_vcbus_write(L_TCON_MISC_SEL_ADDR,
		((1 << STV1_SEL) | (1 << STV2_SEL)));

	if (lcd_vcbus_read(VPP_MISC) & VPP_OUT_SATURATE)
		lcd_vcbus_write(VPP_MISC,
			lcd_vcbus_read(VPP_MISC) & ~(VPP_OUT_SATURATE));
}

static void lcd_venc_set(struct lcd_config_s *pconf)
{
	unsigned int h_active, v_active;
	unsigned int video_on_pixel, video_on_line;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);
	h_active = pconf->lcd_basic.h_active;
	v_active = pconf->lcd_basic.v_active;
	video_on_pixel = pconf->lcd_timing.video_on_pixel;
	video_on_line = pconf->lcd_timing.video_on_line;

	lcd_vcbus_write(ENCL_VIDEO_EN, 0);

	lcd_vcbus_write(ENCL_VIDEO_MODE, 0x8000);/*bit[15] shadown en*/
	lcd_vcbus_write(ENCL_VIDEO_MODE_ADV, 0x0418); /* Sampling rate: 1 */

	lcd_vcbus_write(ENCL_VIDEO_FILT_CTRL, 0x1000); /* bypass filter */
	lcd_vcbus_write(ENCL_VIDEO_MAX_PXCNT, pconf->lcd_basic.h_period - 1);
	lcd_vcbus_write(ENCL_VIDEO_MAX_LNCNT, pconf->lcd_basic.v_period - 1);
	lcd_vcbus_write(ENCL_VIDEO_HAVON_BEGIN, video_on_pixel);
	lcd_vcbus_write(ENCL_VIDEO_HAVON_END,   h_active - 1 + video_on_pixel);
	lcd_vcbus_write(ENCL_VIDEO_VAVON_BLINE, video_on_line);
	lcd_vcbus_write(ENCL_VIDEO_VAVON_ELINE, v_active - 1  + video_on_line);

	lcd_vcbus_write(ENCL_VIDEO_HSO_BEGIN, pconf->lcd_timing.hs_hs_addr);
	lcd_vcbus_write(ENCL_VIDEO_HSO_END,   pconf->lcd_timing.hs_he_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_BEGIN, pconf->lcd_timing.vs_hs_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_END,   pconf->lcd_timing.vs_he_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_BLINE, pconf->lcd_timing.vs_vs_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_ELINE, pconf->lcd_timing.vs_ve_addr);
	lcd_vcbus_write(ENCL_VIDEO_RGBIN_CTRL, 3);

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
		/*[15:14]: 2'b10 or 2'b01*/
		lcd_vcbus_write(ENCL_INBUF_CNTL1, (2 << 14) | (h_active - 1));
		lcd_vcbus_write(ENCL_INBUF_CNTL0, 0x200);
		break;
	case LCD_CHIP_TM2:
		/*[15:14]: 2'b10 or 2'b01 bit13:1*/
		lcd_vcbus_write(ENCL_INBUF_CNTL1, (5 << 13) | (h_active - 1));
		lcd_vcbus_write(ENCL_INBUF_CNTL0, 0x200);
		break;
	default:
		break;
	}

	/* default black pattern */
	lcd_vcbus_write(ENCL_TST_MDSEL, 0);
	lcd_vcbus_write(ENCL_TST_Y, 0);
	lcd_vcbus_write(ENCL_TST_CB, 0);
	lcd_vcbus_write(ENCL_TST_CR, 0);
	lcd_vcbus_write(ENCL_TST_EN, 1);
	lcd_vcbus_setb(ENCL_VIDEO_MODE_ADV, 0, 3, 1);

	lcd_vcbus_write(ENCL_VIDEO_EN, 1);

	aml_lcd_notifier_call_chain(LCD_EVENT_BACKLIGHT_UPDATE, NULL);
}

static void lcd_ttl_control_set(struct lcd_config_s *pconf)
{
	unsigned int clk_pol, rb_swap, bit_swap;

	clk_pol = pconf->lcd_control.ttl_config->clk_pol;
	rb_swap = (pconf->lcd_control.ttl_config->swap_ctrl >> 1) & 1;
	bit_swap = (pconf->lcd_control.ttl_config->swap_ctrl >> 0) & 1;

	lcd_vcbus_setb(L_POL_CNTL_ADDR, clk_pol, 6, 1);
	lcd_vcbus_setb(L_DUAL_PORT_CNTL_ADDR, rb_swap, 1, 1);
	lcd_vcbus_setb(L_DUAL_PORT_CNTL_ADDR, bit_swap, 0, 1);
}

static void lcd_lvds_clk_util_set(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int phy_div;
	unsigned int reg_cntl0, reg_cntl1;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		reg_cntl0 = HHI_LVDS_TX_PHY_CNTL0_TL1;
		reg_cntl1 = HHI_LVDS_TX_PHY_CNTL1_TL1;
		break;
	default:
		reg_cntl0 = HHI_LVDS_TX_PHY_CNTL0;
		reg_cntl1 = HHI_LVDS_TX_PHY_CNTL1;
		break;
	}

	if (pconf->lcd_control.lvds_config->dual_port)
		phy_div = 2;
	else
		phy_div = 1;

	/* set fifo_clk_sel: div 7 */
	lcd_hiu_write(reg_cntl0, (1 << 6));
	/* set cntl_ser_en:  8-channel to 1 */
	lcd_hiu_setb(reg_cntl0, 0xfff, 16, 12);
	switch (lcd_drv->data->chip_type) { /* pn swap */
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		lcd_hiu_setb(reg_cntl0, 1, 2, 1);
		break;
	default:
		break;
	}

	/* decoupling fifo enable, gated clock enable */
	lcd_hiu_write(reg_cntl1,
		(1 << 30) | ((phy_div - 1) << 25) | (1 << 24));
	/* decoupling fifo write enable after fifo enable */
	lcd_hiu_setb(reg_cntl1, 1, 31, 1);
}

static void lcd_lvds_control_set(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int bit_num = 1;
	unsigned int pn_swap, port_swap, lane_reverse;
	unsigned int dual_port, fifo_mode;
	unsigned int lvds_repack = 1;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	lcd_lvds_clk_util_set(pconf);

	lvds_repack = (pconf->lcd_control.lvds_config->lvds_repack) & 0x3;
	pn_swap   = (pconf->lcd_control.lvds_config->pn_swap) & 0x1;
	dual_port = (pconf->lcd_control.lvds_config->dual_port) & 0x1;
	port_swap = (pconf->lcd_control.lvds_config->port_swap) & 0x1;
	lane_reverse = (pconf->lcd_control.lvds_config->lane_reverse) & 0x1;

	switch (pconf->lcd_basic.lcd_bits) {
	case 10:
		bit_num = 0;
		break;
	case 8:
		bit_num = 1;
		break;
	case 6:
		bit_num = 2;
		break;
	case 4:
		bit_num = 3;
		break;
	default:
		bit_num = 1;
		break;
	}
	if (dual_port)
		fifo_mode = 0x3;
	else
		fifo_mode = 0x1;

	lcd_vcbus_write(LVDS_PACK_CNTL_ADDR,
		(lvds_repack << 0) | /* repack //[1:0] */
		(0 << 3) |	/* reserve */
		(0 << 4) |	/* lsb first */
		(pn_swap << 5) | /* pn swap */
		(dual_port << 6) | /* dual port */
		(0 << 7) |	/* use tcon control */
		(bit_num << 8) | /* 0:10bits, 1:8bits, 2:6bits, 3:4bits */
		(0 << 10) |	/* r_select  //0:R, 1:G, 2:B, 3:0 */
		(1 << 12) |	/* g_select  //0:R, 1:G, 2:B, 3:0 */
		(2 << 14));	/* b_select  //0:R, 1:G, 2:B, 3:0 */

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		lcd_vcbus_write(P2P_CH_SWAP0, 0x76543210);
		lcd_vcbus_write(P2P_CH_SWAP1, 0xba98);
		break;
	default:
		lcd_vcbus_setb(LCD_PORT_SWAP, port_swap, 12, 1);
		if (lane_reverse)
			lcd_vcbus_setb(LVDS_GEN_CNTL, 0x03, 13, 2);
		break;
	}

	lcd_vcbus_write(LVDS_GEN_CNTL,
		(lcd_vcbus_read(LVDS_GEN_CNTL) |
		(1 << 4) | (fifo_mode << 0)));

	lcd_vcbus_setb(LVDS_GEN_CNTL, 1, 3, 1);
}

static void lcd_lvds_disable(void)
{
	lcd_vcbus_setb(LVDS_GEN_CNTL, 0, 3, 1); /* disable lvds fifo */
}

static void lcd_vbyone_sync_pol(int hsync_pol, int vsync_pol)
{
	lcd_vcbus_setb(VBO_VIN_CTRL, hsync_pol, 4, 1);
	lcd_vcbus_setb(VBO_VIN_CTRL, vsync_pol, 5, 1);

	lcd_vcbus_setb(VBO_VIN_CTRL, hsync_pol, 6, 1);
	lcd_vcbus_setb(VBO_VIN_CTRL, vsync_pol, 7, 1);
}

static void lcd_vbyone_clk_util_set(struct lcd_config_s *pconf)
{
	unsigned int lcd_bits;
	unsigned int div_sel, phy_div;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int reg_cntl0, reg_cntl1;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		reg_cntl0 = HHI_LVDS_TX_PHY_CNTL0_TL1;
		reg_cntl1 = HHI_LVDS_TX_PHY_CNTL1_TL1;
		break;
	default:
		reg_cntl0 = HHI_LVDS_TX_PHY_CNTL0;
		reg_cntl1 = HHI_LVDS_TX_PHY_CNTL1;
		break;
	}

	phy_div = pconf->lcd_control.vbyone_config->phy_div;
	lcd_bits = pconf->lcd_basic.lcd_bits;

	switch (lcd_bits) {
	case 6:
		div_sel = 0;
		break;
	case 8:
		div_sel = 2;
		break;
	case 10:
		div_sel = 3;
		break;
	default:
		div_sel = 3;
		break;
	}
	/* set fifo_clk_sel */
	lcd_hiu_write(reg_cntl0, (div_sel << 6));
	/* set cntl_ser_en:  8-channel to 1 */
	lcd_hiu_setb(reg_cntl0, 0xfff, 16, 12);
	switch (lcd_drv->data->chip_type) { /* pn swap */
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		lcd_hiu_setb(reg_cntl0, 1, 2, 1);
		break;
	default:
		break;
	}

	/* decoupling fifo enable, gated clock enable */
	lcd_hiu_write(reg_cntl1,
		(1 << 30) | ((phy_div - 1) << 25) | (1 << 24));
	/* decoupling fifo write enable after fifo enable */
	lcd_hiu_setb(reg_cntl1, 1, 31, 1);
}

static int lcd_vbyone_lanes_set(int lane_num, int byte_mode, int region_num,
		int hsize, int vsize)
{
	int sublane_num;
	int region_size[4];
	int tmp;

	switch (lane_num) {
	case 1:
	case 2:
	case 4:
	case 8:
		break;
	default:
		return -1;
	}
	switch (region_num) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		return -1;
	}
	if (lane_num % region_num)
		return -1;
	switch (byte_mode) {
	case 3:
	case 4:
		break;
	default:
		return -1;
	}
	if (lcd_debug_print_flag) {
		LCDPR("byte_mode=%d, lane_num=%d, region_num=%d\n",
			byte_mode, lane_num, region_num);
	}

	sublane_num = lane_num / region_num; /* lane num in each region */
	lcd_vcbus_setb(VBO_LANES, (lane_num - 1), 0, 3);
	lcd_vcbus_setb(VBO_LANES, (region_num - 1), 4, 2);
	lcd_vcbus_setb(VBO_LANES, (sublane_num - 1), 8, 3);
	lcd_vcbus_setb(VBO_LANES, (byte_mode - 1), 11, 2);

	if (region_num > 1) {
		region_size[3] = (hsize / lane_num) * sublane_num;
		tmp = (hsize % lane_num);
		region_size[0] = region_size[3] + (((tmp / sublane_num) > 0) ?
			sublane_num : (tmp % sublane_num));
		region_size[1] = region_size[3] + (((tmp / sublane_num) > 1) ?
			sublane_num : (tmp % sublane_num));
		region_size[2] = region_size[3] + (((tmp / sublane_num) > 2) ?
			sublane_num : (tmp % sublane_num));
		lcd_vcbus_write(VBO_REGION_00, region_size[0]);
		lcd_vcbus_write(VBO_REGION_01, region_size[1]);
		lcd_vcbus_write(VBO_REGION_02, region_size[2]);
		lcd_vcbus_write(VBO_REGION_03, region_size[3]);
	}
	lcd_vcbus_write(VBO_ACT_VSIZE, vsize);
	/* different from FBC code!!! */
	/* lcd_vcbus_setb(VBO_CTRL_H,0x80,11,5); */
	/* different from simulation code!!! */
	lcd_vcbus_setb(VBO_CTRL_H, 0x0, 0, 4);
	lcd_vcbus_setb(VBO_CTRL_H, 0x1, 9, 1);
	/* lcd_vcbus_setb(VBO_CTRL_L,enable,0,1); */

	return 0;
}

static void lcd_vbyone_sw_reset(void)
{
	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	/* force PHY to 0 */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 3, 8, 2);
	lcd_vcbus_write(VBO_SOFT_RST, 0x1ff);
	udelay(5);
	/* realease PHY */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 0, 8, 2);
	lcd_vcbus_write(VBO_SOFT_RST, 0);
}

static void lcd_vbyone_wait_timing_stable(void)
{
	unsigned int timing_state;
	int i = 200;

	timing_state = lcd_vcbus_read(VBO_INTR_STATE) & 0x1ff;
	while ((timing_state) && (i > 0)) {
		/* clear video timing error intr */
		lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0x7, 0, 3);
		lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 3);
		mdelay(2);
		timing_state = lcd_vcbus_read(VBO_INTR_STATE) & 0x1ff;
		i--;
	};
	if (lcd_debug_print_flag) {
		LCDPR("vbyone timing state: 0x%03x, i=%d\n",
			timing_state, (200 - i));
	}
	mdelay(2);
}

static void lcd_vbyone_control_set(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int lane_count, byte_mode, region_num, hsize, vsize, color_fmt;
	int vin_color, vin_bpp;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	hsize = pconf->lcd_basic.h_active;
	vsize = pconf->lcd_basic.v_active;
	lane_count = pconf->lcd_control.vbyone_config->lane_count; /* 8 */
	region_num = pconf->lcd_control.vbyone_config->region_num; /* 2 */
	byte_mode = pconf->lcd_control.vbyone_config->byte_mode; /* 4 */
	color_fmt = pconf->lcd_control.vbyone_config->color_fmt; /* 4 */

	lcd_vbyone_clk_util_set(pconf);
#if 0
	switch (color_fmt) {
	case 0:/* SDVT_VBYONE_18BPP_RGB */
		vin_color = 4;
		vin_bpp   = 2;
		break;
	case 1:/* SDVT_VBYONE_18BPP_YCBCR444 */
		vin_color = 0;
		vin_bpp   = 2;
		break;
	case 2:/* SDVT_VBYONE_24BPP_RGB */
		vin_color = 4;
		vin_bpp   = 1;
		break;
	case 3:/* SDVT_VBYONE_24BPP_YCBCR444 */
		vin_color = 0;
		vin_bpp   = 1;
		break;
	case 4:/* SDVT_VBYONE_30BPP_RGB */
		vin_color = 4;
		vin_bpp   = 0;
		break;
	case 5:/* SDVT_VBYONE_30BPP_YCBCR444 */
		vin_color = 0;
		vin_bpp   = 0;
		break;
	default:
		LCDERR("vbyone COLOR_FORMAT unsupport\n");
		return;
	}
#else
	vin_color = 4; /* fixed RGB */
	vin_bpp   = 0; /* fixed 30bbp 4:4:4 */
#endif

	/* set Vbyone vin color format */
	lcd_vcbus_setb(VBO_VIN_CTRL, vin_color, 8, 3);
	lcd_vcbus_setb(VBO_VIN_CTRL, vin_bpp, 11, 2);

	lcd_vbyone_lanes_set(lane_count, byte_mode, region_num, hsize, vsize);
	/*set hsync/vsync polarity to let the polarity is low active
	inside the VbyOne */
	lcd_vbyone_sync_pol(0, 0);

	/* below line copy from simulation */
	/* gate the input when vsync asserted */
	lcd_vcbus_setb(VBO_VIN_CTRL, 1, 0, 2);
	/* lcd_vcbus_write(VBO_VBK_CTRL_0,0x13);
	//lcd_vcbus_write(VBO_VBK_CTRL_1,0x56);
	//lcd_vcbus_write(VBO_HBK_CTRL,0x3478);
	//lcd_vcbus_setb(VBO_PXL_CTRL,0x2,0,4);
	//lcd_vcbus_setb(VBO_PXL_CTRL,0x3,VBO_PXL_CTR1_BIT,VBO_PXL_CTR1_WID);
	//set_vbyone_ctlbits(1,0,0); */

	/* PAD select: */
	if ((lane_count == 1) || (lane_count == 2))
		lcd_vcbus_setb(LCD_PORT_SWAP, 1, 9, 2);
	else if (lane_count == 4)
		lcd_vcbus_setb(LCD_PORT_SWAP, 2, 9, 2);
	else
		lcd_vcbus_setb(LCD_PORT_SWAP, 0, 9, 2);
	/* lcd_vcbus_setb(LCD_PORT_SWAP, 1, 8, 1);//reverse lane output order */

	/* Mux pads in combo-phy: 0 for dsi; 1 for lvds or vbyone; 2 for edp */
	/*lcd_hiu_write(HHI_DSI_LVDS_EDP_CNTL0, 0x1);*/

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		lcd_vcbus_write(VBO_INFILTER_TICK_PERIOD_L, 0xff);
		lcd_vcbus_write(VBO_INFILTER_TICK_PERIOD_H, 0x0);
		lcd_vcbus_setb(VBO_INSGN_CTRL, 0x7, 8, 4);
		lcd_vcbus_setb(VBO_INSGN_CTRL, 0x7, 12, 4);
		break;
	default:
		lcd_vcbus_write(VBO_INFILTER_CTRL, 0xff77);
		break;
	}
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 2, 2);

	lcd_vcbus_setb(VBO_CTRL_L, 1, 0, 1);

	/*force vencl clk enable, otherwise, it might auto turn off by mipi DSI
	//lcd_vcbus_setb(VPU_MISC_CTRL, 1, 0, 1); */

	lcd_vbyone_wait_timing_stable();
	lcd_vbyone_sw_reset();
}

static void lcd_vbyone_disable(void)
{
	lcd_vcbus_setb(VBO_CTRL_L, 0, 0, 1);
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 2, 1);
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 0, 1);
}

static void lcd_tablet_vbyone_wait_stable(void)
{
	int i = 5000;

	while (((lcd_vcbus_read(VBO_STATUS_L) & 0x3f) != 0x20) && (i > 0)) {
		udelay(50);
		i--;
	}
	LCDPR("%s status: 0x%x, i=%d\n",
		__func__, lcd_vcbus_read(VBO_STATUS_L), (5000 - i));
}

static void lcd_vbyone_wait_hpd(void)
{
	int i = 0;

	if (lcd_debug_print_flag)
		LCDPR("vx1 wait hpd to low ...\n");

	while (lcd_vcbus_read(VBO_STATUS_L) & 0x40) {
		if (i++ >= 10000)
			break;
		udelay(50);
	}
	mdelay(10);
	if (lcd_vcbus_read(VBO_STATUS_L) & 0x40) {
		LCDPR("%s: hpd=%d\n", __func__,
			((lcd_vcbus_read(VBO_STATUS_L) >> 6) & 0x1));
	} else {
		LCDPR("%s: hpd=%d, i=%d\n", __func__,
			((lcd_vcbus_read(VBO_STATUS_L) >> 6) & 0x1), i);
		/* force low only actived for actual hpd is low */
		lcd_vcbus_setb(VBO_INSGN_CTRL, 1, 2, 2);
	}

	usleep_range(10000, 10500); /* add 10ms delay for compatibility */
}

static unsigned int vbyone_lane_num[] = {
	1,
	2,
	4,
	8,
	8,
};

#define VBYONE_BIT_RATE_MAX		3100 /* MHz */
#define VBYONE_BIT_RATE_MIN		600
static void lcd_vbyone_config_set(struct lcd_config_s *pconf)
{
	unsigned int band_width, bit_rate, pclk, phy_div;
	unsigned int byte_mode, lane_count, minlane;
	unsigned int temp, i;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	/* auto calculate bandwidth, clock */
	lane_count = pconf->lcd_control.vbyone_config->lane_count;
	byte_mode = pconf->lcd_control.vbyone_config->byte_mode;
	/* byte_mode * byte2bit * 8/10_encoding * pclk =
	   byte_mode * 8 * 10 / 8 * pclk */
	pclk = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	band_width = byte_mode * 10 * pclk;

	temp = VBYONE_BIT_RATE_MAX * 1000;
	temp = (band_width + temp - 1) / temp;
	for (i = 0; i < 4; i++) {
		if (temp <= vbyone_lane_num[i])
			break;
	}
	minlane = vbyone_lane_num[i];
	if (lane_count < minlane) {
		LCDERR("vbyone lane_num(%d) is less than min(%d)\n",
			lane_count, minlane);
		lane_count = minlane;
		pconf->lcd_control.vbyone_config->lane_count = lane_count;
		LCDPR("change to min lane_num %d\n", minlane);
	}

	bit_rate = band_width / lane_count;
	phy_div = lane_count / lane_count;
	if (phy_div == 8) {
		phy_div /= 2;
		bit_rate /= 2;
	}
	if (bit_rate > (VBYONE_BIT_RATE_MAX * 1000)) {
		LCDERR("vbyone bit rate(%dKHz) is out of max(%dKHz)\n",
			bit_rate, (VBYONE_BIT_RATE_MAX * 1000));
	}
	if (bit_rate < (VBYONE_BIT_RATE_MIN * 1000)) {
		LCDERR("vbyone bit rate(%dKHz) is out of min(%dKHz)\n",
			bit_rate, (VBYONE_BIT_RATE_MIN * 1000));
	}
	bit_rate = bit_rate * 1000; /* Hz */

	pconf->lcd_control.vbyone_config->phy_div = phy_div;
	pconf->lcd_timing.bit_rate = bit_rate;

	if (lcd_debug_print_flag) {
		LCDPR("lane_count=%u, bit_rate = %uMHz, pclk=%u.%03uMhz\n",
			lane_count, (bit_rate / 1000000),
			(pclk / 1000), (pclk % 1000));
	}
}

void lcd_tablet_clk_config_change(struct lcd_config_s *pconf)
{
#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(pconf->lcd_timing.lcd_clk, VPU_VENCL);
#endif

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_VBYONE:
		lcd_vbyone_config_set(pconf);
		break;
	case LCD_MIPI:
		lcd_mipi_dsi_config_set(pconf);
		break;
	default:
		break;
	}

	lcd_clk_generate_parameter(pconf);
}

void lcd_tablet_clk_update(struct lcd_config_s *pconf)
{
	lcd_tablet_clk_config_change(pconf);

	lcd_clk_set(pconf);
	if (pconf->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_tablet_vbyone_wait_stable();
}

void lcd_tablet_config_update(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vinfo_s *info;

	/* update lcd config sync_duration */
	info = lcd_drv->lcd_info;
	pconf->lcd_timing.sync_duration_num = info->sync_duration_num;
	pconf->lcd_timing.sync_duration_den = info->sync_duration_den;

	/* update clk & timing config */
	lcd_vmode_change(pconf);
	info->video_clk = pconf->lcd_timing.lcd_clk;
	info->htotal = pconf->lcd_basic.h_period;
	info->vtotal = pconf->lcd_basic.v_period;
	/* update interface timing */
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_VBYONE:
		lcd_vbyone_config_set(pconf);
		break;
	case LCD_MIPI:
		lcd_mipi_dsi_config_set(pconf);
		break;
	default:
		break;
	}
}

void lcd_tablet_config_post_update(struct lcd_config_s *pconf)
{
	/* update interface timing */
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MIPI:
		lcd_mipi_dsi_config_post(pconf);
		break;
	default:
		break;
	}
}

void lcd_tablet_driver_init_pre(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	pconf = lcd_drv->lcd_config;
	LCDPR("tablet driver init(ver %s): %s\n", lcd_drv->version,
		lcd_type_type_to_str(pconf->lcd_basic.lcd_type));
	ret = lcd_type_supported(pconf);
	if (ret)
		return;

	lcd_tablet_config_update(pconf);
	lcd_tablet_config_post_update(pconf);
#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(pconf->lcd_timing.lcd_clk, VPU_VENCL);
	switch_vpu_mem_pd_vmod(VPU_VENCL, VPU_MEM_POWER_ON);
#endif
	lcd_clk_gate_switch(1);

	lcd_clk_set(pconf);
	lcd_venc_set(pconf);
	lcd_encl_tcon_set(pconf);
	lcd_drv->lcd_mute_state = 1;

	lcd_vcbus_write(VENC_INTCTRL, 0x200);

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
}

void lcd_tablet_driver_disable_post(void)
{
	lcd_vcbus_write(ENCL_VIDEO_EN, 0); /* disable encl */

	lcd_clk_disable();
	lcd_clk_gate_switch(0);
#ifdef CONFIG_AMLOGIC_VPU
	switch_vpu_mem_pd_vmod(VPU_VENCL, VPU_MEM_POWER_DOWN);
	release_vpu_clk_vmod(VPU_VENCL);
#endif

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
}

int lcd_tablet_driver_init(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	pconf = lcd_drv->lcd_config;
	ret = lcd_type_supported(pconf);
	if (ret)
		return -1;

	/* init driver */
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_TTL:
		lcd_ttl_control_set(pconf);
		lcd_ttl_pinmux_set(1);
		break;
	case LCD_LVDS:
		lcd_lvds_control_set(pconf);
		lcd_lvds_phy_set(pconf, 1);
		break;
	case LCD_VBYONE:
		lcd_vbyone_pinmux_set(1);
		lcd_vbyone_control_set(pconf);
		lcd_vbyone_wait_hpd();
		lcd_vbyone_phy_set(pconf, 1);
		lcd_tablet_vbyone_wait_stable();
		break;
	case LCD_MIPI:
		lcd_mipi_phy_set(pconf, 1);
		lcd_mipi_control_set(pconf, 1);
		break;
	default:
		break;
	}

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);

	return 0;
}

void lcd_tablet_driver_disable(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	LCDPR("disable driver\n");
	pconf = lcd_drv->lcd_config;
	ret = lcd_type_supported(pconf);
	if (ret)
		return;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_TTL:
		lcd_ttl_pinmux_set(0);
		break;
	case LCD_LVDS:
		lcd_lvds_phy_set(pconf, 0);
		lcd_lvds_disable();
		break;
	case LCD_VBYONE:
		lcd_vbyone_phy_set(pconf, 0);
		lcd_vbyone_pinmux_set(0);
		lcd_vbyone_disable();
		break;
	case LCD_MIPI:
		mipi_dsi_link_off(pconf);
		lcd_mipi_phy_set(pconf, 0);
		lcd_mipi_control_set(pconf, 0);
		break;
	default:
		break;
	}

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
}

