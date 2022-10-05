/*
 * drivers/amlogic/media/vout/lcd/lcd_tv/lcd_drv.c
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
#include "lcd_tv.h"
#include "../lcd_reg.h"
#include "../lcd_common.h"

static struct delayed_work lcd_vx1_stable_delayed_work;

static int vx1_fsm_acq_st;

#define VX1_TRAINING_TIMEOUT    60  /* vsync cnt */
static int vx1_training_wait_cnt;
static int vx1_training_stable_cnt;
static int vx1_timeout_reset_flag;
static struct work_struct lcd_vx1_reset_work;
static int lcd_vx1_intr_request;

#define VX1_HPLL_INTERVAL (HZ)
static struct timer_list vx1_hpll_timer;

static int lcd_type_supported(struct lcd_config_s *pconf)
{
	int lcd_type = pconf->lcd_basic.lcd_type;
	int ret = -1;

	switch (lcd_type) {
	case LCD_LVDS:
	case LCD_VBYONE:
	case LCD_MLVDS:
	case LCD_P2P:
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
	unsigned int lcd_bits;

	lcd_vcbus_write(L_RGB_BASE_ADDR, 0);
	lcd_vcbus_write(L_RGB_COEFF_ADDR, 0x400);
	aml_lcd_notifier_call_chain(LCD_EVENT_GAMMA_UPDATE, NULL);

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MLVDS:
	case LCD_P2P:
		lcd_bits = 10;
		break;
	default:
		lcd_bits = pconf->lcd_basic.lcd_bits;
		break;
	}
	switch (lcd_bits) {
	case 8:
		lcd_vcbus_write(L_DITH_CNTL_ADDR,  0x400);
		break;
	case 6:
		lcd_vcbus_write(L_DITH_CNTL_ADDR,  0x600);
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
	default:
		break;
	}

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

	/* Enable Hsync and equalization pulse switch in center;
	 * bit[14] cfg_de_v = 1
	 */
	lcd_vcbus_write(ENCL_VIDEO_MODE, 0x8000);/*bit[15] shadown en*/
	lcd_vcbus_write(ENCL_VIDEO_MODE_ADV,   0x18); /* Sampling rate: 1 */

	/* bypass filter */
	lcd_vcbus_write(ENCL_VIDEO_FILT_CTRL, 0x1000);
	lcd_vcbus_write(ENCL_VIDEO_MAX_PXCNT, pconf->lcd_basic.h_period - 1);
	lcd_vcbus_write(ENCL_VIDEO_MAX_LNCNT, pconf->lcd_basic.v_period - 1);

	lcd_vcbus_write(ENCL_VIDEO_HAVON_BEGIN, video_on_pixel);
	lcd_vcbus_write(ENCL_VIDEO_HAVON_END,   h_active - 1 + video_on_pixel);
	lcd_vcbus_write(ENCL_VIDEO_VAVON_BLINE, video_on_line);
	lcd_vcbus_write(ENCL_VIDEO_VAVON_ELINE, v_active - 1  + video_on_line);
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_P2P:
		lcd_vcbus_write(ENCL_VIDEO_V_PRE_DE_BLINE,
			video_on_line - 1 - 4);
		lcd_vcbus_write(ENCL_VIDEO_V_PRE_DE_ELINE,
			video_on_line - 1);
		lcd_vcbus_write(ENCL_VIDEO_H_PRE_DE_BEGIN,
			video_on_pixel + PRE_DE_DELAY);
		lcd_vcbus_write(ENCL_VIDEO_H_PRE_DE_END,
			h_active - 1 + video_on_pixel + PRE_DE_DELAY);
		break;
	default:
		break;
	}

	lcd_vcbus_write(ENCL_VIDEO_HSO_BEGIN,   pconf->lcd_timing.hs_hs_addr);
	lcd_vcbus_write(ENCL_VIDEO_HSO_END,     pconf->lcd_timing.hs_he_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_BEGIN,   pconf->lcd_timing.vs_hs_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_END,     pconf->lcd_timing.vs_he_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_BLINE,   pconf->lcd_timing.vs_vs_addr);
	lcd_vcbus_write(ENCL_VIDEO_VSO_ELINE,   pconf->lcd_timing.vs_ve_addr);

	lcd_vcbus_write(ENCL_VIDEO_RGBIN_CTRL, 3);

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
		/*[15:14]: 2'b10 or 2'b01*/
		lcd_vcbus_write(ENCL_INBUF_CNTL1, (2 << 14) | (h_active - 1));
		lcd_vcbus_write(ENCL_INBUF_CNTL0, 0x200);
		break;
	case LCD_CHIP_TM2:
		/*[15:14]: 2'b10 or 2'b01, bit13:1*/
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
		(lcd_vcbus_read(LVDS_GEN_CNTL) | (1 << 4) | (fifo_mode << 0)));

	lcd_vcbus_setb(LVDS_GEN_CNTL, 1, 3, 1);
}

static void lcd_lvds_disable(void)
{
	lcd_vcbus_setb(LVDS_GEN_CNTL, 0, 3, 1); /* disable lvds fifo */
}

static void lcd_mlvds_control_set(struct lcd_config_s *pconf)
{
	unsigned int div_sel;
	unsigned int channel_sel0, channel_sel1;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	/* phy_div: 0=div6, 1=div 7, 2=div8, 3=div10 */
	switch (pconf->lcd_basic.lcd_bits) {
	case 6:
		div_sel = 0;
		break;
	case 8:
		div_sel = 2;
		break;
	default:
		div_sel = 2;
		break;
	}

	/* fifo_clk_sel[7:6]: 0=div6, 1=div 7, 2=div8, 3=div10 */
	lcd_hiu_write(HHI_LVDS_TX_PHY_CNTL0_TL1, (div_sel << 6));
	/* serializer_en[27:16] */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0_TL1, 0xfff, 16, 12);
	/* pn swap[2] */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0_TL1, 1, 2, 1);

	/* fifo enable[30], phy_clock gating[24] */
	lcd_hiu_write(HHI_LVDS_TX_PHY_CNTL1_TL1, (1 << 30) | (1 << 24));
	/* fifo write enable[31] */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL1_TL1, 1, 31, 1);

	channel_sel0 = pconf->lcd_control.mlvds_config->channel_sel0;
	channel_sel1 = pconf->lcd_control.mlvds_config->channel_sel1;
	lcd_vcbus_write(P2P_CH_SWAP0, channel_sel0);
	lcd_vcbus_write(P2P_CH_SWAP1, channel_sel1);

	lcd_tcon_enable(pconf);
}

static void lcd_mlvds_disable(void)
{
	lcd_tcon_disable();
}

#if 0
static void lcd_vbyone_ctlbits(int p3d_en, int p3d_lr, int mode)
{
	if (mode == 0) { /* insert at the first pixel */
		lcd_vcbus_setb(VBO_PXL_CTRL,
			(1 << p3d_en) | (p3d_lr & 0x1), 0, 4);
	} else {
		lcd_vcbus_setb(VBO_VBK_CTRL_0,
			(1 << p3d_en) | (p3d_lr & 0x1), 0, 2);
	}
}
#endif

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

static void lcd_vbyone_hw_filter(int flag)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vx1_conf;
	unsigned int temp, period;
	unsigned int tick_period[] = {
		0xfff,
		0xff,    /* 1: 0.8us */
		0x1ff,   /* 2: 1.7us */
		0x3ff,   /* 3: 3.4us */
		0x7ff,   /* 4: 6.9us */
		0xfff,   /* 5: 13.8us */
		0x1fff,  /* 6: 27us */
		0x3fff,  /* 7: 55us */
		0x7fff,  /* 8: 110us */
		0xffff,  /* 9: 221us */
		0x1ffff, /* 10: 441us */
		0x3ffff, /* 11: 883us */
		0x7ffff, /* 12: 1.76ms */
		0xfffff, /* 13: 3.53ms */
	};

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;
	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		if (flag) {
			period = vx1_conf->hw_filter_time & 0xff;
			if (period >=
				(sizeof(tick_period) / sizeof(unsigned int)))
				period = tick_period[0];
			else
				period = tick_period[period];
			temp = period & 0xffff;
			lcd_vcbus_write(VBO_INFILTER_TICK_PERIOD_L, temp);
			temp = (period >> 16) & 0xf;
			lcd_vcbus_write(VBO_INFILTER_TICK_PERIOD_H, temp);
			/* hpd */
			temp = vx1_conf->hw_filter_cnt & 0xf;
			temp = (temp == 0) ? 0x7 : temp;
			lcd_vcbus_setb(VBO_INSGN_CTRL, temp, 8, 4);
			/* lockn */
			temp = (vx1_conf->hw_filter_cnt >> 8) & 0xf;
			temp = (temp == 0) ? 0x7 : temp;
			lcd_vcbus_setb(VBO_INSGN_CTRL, temp, 12, 4);
		} else {
			temp = (vx1_conf->hw_filter_time >> 8) & 0x1;
			if (temp) {
				lcd_vcbus_write(VBO_INFILTER_TICK_PERIOD_L,
						0xff);
				lcd_vcbus_write(VBO_INFILTER_TICK_PERIOD_H,
						0x0);
				lcd_vcbus_setb(VBO_INSGN_CTRL, 0x7, 8, 4);
				lcd_vcbus_setb(VBO_INSGN_CTRL, 0x7, 12, 4);
				LCDPR("%s: %d change to min for debug\n",
				       __func__, flag);
			}
		}
		break;
	default:
		lcd_vcbus_write(VBO_INFILTER_CTRL, 0xff77);
		break;
	}
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

static void lcd_vbyone_cdr_training_hold(struct vbyone_config_s *vx1_conf,
		int flag)
{
	if (flag) {
		LCDPR("ctrl_flag for cdr_training_hold\n");
		lcd_vcbus_setb(VBO_FSM_HOLDER_H, 0xffff, 0, 16);
	} else {
		msleep(vx1_conf->cdr_training_hold);
		lcd_vcbus_setb(VBO_FSM_HOLDER_H, 0, 0, 16);
	}
}

static void lcd_vbyone_control_set(struct lcd_config_s *pconf)
{
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
	/*set hsync/vsync polarity to let the polarity is low active*/
	/*inside the VbyOne */
	lcd_vbyone_sync_pol(0, 0);

	/* below line copy from simulation */
	/* gate the input when vsync asserted */
	lcd_vcbus_setb(VBO_VIN_CTRL, 1, 0, 2);
	/* lcd_vcbus_write(VBO_VBK_CTRL_0,0x13);
	 * lcd_vcbus_write(VBO_VBK_CTRL_1,0x56);
	 * lcd_vcbus_write(VBO_HBK_CTRL,0x3478);
	 * lcd_vcbus_setb(VBO_PXL_CTRL,0x2,0,4);
	 * lcd_vcbus_setb(VBO_PXL_CTRL,0x3,VBO_PXL_CTR1_BIT,VBO_PXL_CTR1_WID);
	 * set_vbyone_ctlbits(1,0,0);
	 */

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

	lcd_vbyone_hw_filter(0);
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 2, 2);
	lcd_vcbus_setb(VBO_CTRL_L, 1, 0, 1);

	/* force vencl clk enable, otherwise,
	 * it might auto turn off by mipi DSI
	 */
	/* lcd_vcbus_setb(VPU_MISC_CTRL, 1, 0, 1); */

	lcd_vbyone_wait_timing_stable();
	lcd_vbyone_sw_reset();

	/* training hold */
	if ((pconf->lcd_control.vbyone_config->ctrl_flag) & 0x4) {
		lcd_vbyone_cdr_training_hold(
			pconf->lcd_control.vbyone_config, 1);
	}
}

static void lcd_vbyone_disable(void)
{
	lcd_vcbus_setb(VBO_CTRL_L, 0, 0, 1);
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 2, 1);
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 0, 1);
}

#define VBYONE_INTR_UNMASK   0x2bff /* 0x2a00 */
			/* enable htpdn_fail,lockn_fail,acq_hold */
void lcd_vbyone_interrupt_enable(int flag)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vx1_conf;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, flag);

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;
	if (flag) {
		lcd_vbyone_hw_filter(1);
		if (vx1_conf->intr_en) {
			vx1_fsm_acq_st = 0;
			/* clear interrupt */
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0x01ff, 0, 9);
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 9);

			/* set hold in FSM_ACQ */
			if (vx1_conf->vsync_intr_en == 3)
				lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0, 0, 16);
			else
				lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0xffff, 0, 16);
			/* enable interrupt */
			lcd_vcbus_setb(VBO_INTR_UNMASK,
				VBYONE_INTR_UNMASK, 0, 15);
		} else {
			/* mask interrupt */
			lcd_vcbus_write(VBO_INTR_UNMASK, 0x0);
			if (vx1_conf->vsync_intr_en) {
				/* keep holder for vsync monitor enabled */
				/* set hold in FSM_ACQ */
				if (vx1_conf->vsync_intr_en == 3)
					lcd_vcbus_setb(VBO_FSM_HOLDER_L,
						0, 0, 16);
				else
					lcd_vcbus_setb(VBO_FSM_HOLDER_L,
					0xffff, 0, 16);
			} else {
				/* release holder for vsync monitor disabled */
				/* release hold in FSM_ACQ */
				lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0, 0, 16);
			}

			vx1_fsm_acq_st = 0;
			/* clear interrupt */
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0x01ff, 0, 9);
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 9);
		}
	} else {
		/* mask interrupt */
		lcd_vcbus_write(VBO_INTR_UNMASK, 0x0);
		/* release hold in FSM_ACQ */
		lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0, 0, 16);
		lcd_vbyone_hw_filter(0);
		lcd_vx1_intr_request = 0;
	}
}

static void lcd_vbyone_interrupt_init(struct aml_lcd_drv_s *lcd_drv)
{
	struct vbyone_config_s *vx1_conf;

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;

	/* release sw filter ctrl in uboot */
	lcd_vcbus_setb(VBO_INSGN_CTRL, 0, 0, 1);
	lcd_vbyone_hw_filter(1);

	/* set hold in FSM_ACQ */
	if (vx1_conf->vsync_intr_en == 3)
		lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0, 0, 16);
	else
	lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0xffff, 0, 16);
	/* set hold in FSM_CDR */
	lcd_vcbus_setb(VBO_FSM_HOLDER_H, 0, 0, 16);
	/* not wait lockn to 1 in FSM_ACQ */
	lcd_vcbus_setb(VBO_CTRL_L, 1, 10, 1);
	/* lcd_vcbus_setb(VBO_CTRL_L, 0, 9, 1);*/   /*use sw pll_lock */
	/* reg_pll_lock = 1 to realease force to FSM_ACQ*/
	/*lcd_vcbus_setb(VBO_CTRL_H, 1, 13, 1); */

	/* vx1 interrupt setting */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 12, 1);    /* intr pulse width */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0x01ff, 0, 9); /* clear interrupt */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 9);

	vx1_fsm_acq_st = 0;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);
}

#define VX1_LOCKN_WAIT_TIMEOUT    5000 /* 250ms */
void lcd_vbyone_wait_stable(void)
{
	int i = VX1_LOCKN_WAIT_TIMEOUT;

	while (((lcd_vcbus_read(VBO_STATUS_L) & 0x3f) != 0x20) && (i > 0)) {
		udelay(50);
		i--;
	}
	LCDPR("%s status: 0x%x, i=%d\n",
		__func__, lcd_vcbus_read(VBO_STATUS_L),
		(VX1_LOCKN_WAIT_TIMEOUT - i));
	vx1_training_wait_cnt = 0;
	vx1_training_stable_cnt = 0;
	vx1_fsm_acq_st = 0;
	lcd_vbyone_interrupt_enable(1);
}

static void lcd_vbyone_power_on_wait_stable(struct lcd_config_s *pconf)
{
	int i = VX1_LOCKN_WAIT_TIMEOUT;

	/* training hold release */
	if ((pconf->lcd_control.vbyone_config->ctrl_flag) & 0x4) {
		lcd_vbyone_cdr_training_hold(
			pconf->lcd_control.vbyone_config, 0);
	}
	while (((lcd_vcbus_read(VBO_STATUS_L) & 0x3f) != 0x20) && (i > 0)) {
		udelay(50);
		i--;
	}
	LCDPR("%s status: 0x%x, i=%d\n",
		__func__, lcd_vcbus_read(VBO_STATUS_L),
		(VX1_LOCKN_WAIT_TIMEOUT - i));

	/* power on reset */
	if ((pconf->lcd_control.vbyone_config->ctrl_flag) & 0x1) {
		LCDPR("ctrl_flag for power on reset\n");
		msleep(pconf->lcd_control.vbyone_config->power_on_reset_delay);
		lcd_vbyone_sw_reset();
		i = VX1_LOCKN_WAIT_TIMEOUT;
		while (((lcd_vcbus_read(VBO_STATUS_L) & 0x3f) != 0x20) &&
			(i > 0)) {
			udelay(50);
			i--;
		}
		LCDPR("%s status: 0x%x, i=%d\n",
			__func__, lcd_vcbus_read(VBO_STATUS_L),
			(VX1_LOCKN_WAIT_TIMEOUT - i));
	}

	vx1_training_wait_cnt = 0;
	vx1_training_stable_cnt = 0;
	vx1_fsm_acq_st = 0;
	lcd_vbyone_interrupt_enable(1);
}

#define LCD_VX1_WAIT_STABLE_DELAY    300 /* ms */
static void lcd_vx1_wait_stable_delayed(struct work_struct *work)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_vx1_intr_request == 0)
		return;

	lcd_vbyone_power_on_wait_stable(lcd_drv->lcd_config);
}

static void lcd_vbyone_wait_hpd(struct lcd_config_s *pconf)
{
	int i = 0;

	if (lcd_debug_print_flag)
		LCDPR("vx1 wait hpd to low ...\n");

	while (lcd_vcbus_read(VBO_STATUS_L) & 0x40) {
		if (i++ >= 10000)
			break;
		udelay(50);
	}
	if (lcd_vcbus_read(VBO_STATUS_L) & 0x40) {
		LCDPR("%s: hpd=%d\n", __func__,
			((lcd_vcbus_read(VBO_STATUS_L) >> 6) & 0x1));
	} else {
		LCDPR("%s: hpd=%d, i=%d\n", __func__,
			((lcd_vcbus_read(VBO_STATUS_L) >> 6) & 0x1), i);
		/* force low only actived for actual hpd is low */
		lcd_vcbus_setb(VBO_INSGN_CTRL, 1, 2, 2);
	}

	if ((pconf->lcd_control.vbyone_config->ctrl_flag) & 0x2) {
		LCDPR("ctrl_flag for hpd_data delay\n");
		msleep(pconf->lcd_control.vbyone_config->hpd_data_delay);
	} else
		usleep_range(10000, 10500);
		/* add 10ms delay for compatibility */
}

#define LCD_PCLK_TOLERANCE          2000000  /* 2M */
#define LCD_ENCL_CLK_ERR_CNT_MAX    3
static unsigned char lcd_encl_clk_err_cnt;
static void lcd_vx1_hpll_timer_handler(unsigned long arg)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int encl_clk;
#if 0
	int pclk, pclk_min, pclk_max;
#endif

	if ((lcd_drv->lcd_status & LCD_STATUS_ENCL_ON) == 0)
		goto vx1_hpll_timer_end;

#if 0
	pclk = lcd_drv->lcd_config->lcd_timing.lcd_clk;
	pclk_min = pclk - LCD_PCLK_TOLERANCE;
	pclk_max = pclk + LCD_PCLK_TOLERANCE;
	encl_clk = lcd_encl_clk_msr();
	if ((encl_clk < pclk_min) || (encl_clk > pclk_max)) {
		LCDPR("%s: pll frequency error: %d\n", __func__, encl_clk);
		lcd_pll_reset();
	}
#else
	encl_clk = lcd_encl_clk_msr();
	if (encl_clk == 0)
		lcd_encl_clk_err_cnt++;
	else
		lcd_encl_clk_err_cnt = 0;
	if (lcd_encl_clk_err_cnt >= LCD_ENCL_CLK_ERR_CNT_MAX) {
		LCDPR("%s: pll frequency error: %d\n", __func__, encl_clk);
		lcd_pll_reset();
		lcd_encl_clk_err_cnt = 0;
	}
#endif

vx1_hpll_timer_end:
	vx1_hpll_timer.expires = jiffies + VX1_HPLL_INTERVAL;
	add_timer(&vx1_hpll_timer);
}

static void lcd_vx1_hold_reset(void)
{
	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	vx1_fsm_acq_st = 0;
	lcd_vcbus_write(VBO_INTR_UNMASK, 0x0); /* mask interrupt */
	/* clear interrupt */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0x01ff, 0, 9);
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 9);

	/* clear FSM_continue */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 15, 1);

	/* force PHY to 0 */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 3, 8, 2);
	lcd_vcbus_write(VBO_SOFT_RST, 0x1ff);
	udelay(5);
	/* clear lockn raising flag */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 7, 1);
	/* realease PHY */
	lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 0, 8, 2);
	/* clear lockn raising flag */
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 7, 1);
	lcd_vcbus_write(VBO_SOFT_RST, 0);

	/* enable interrupt */
	lcd_vcbus_setb(VBO_INTR_UNMASK, VBYONE_INTR_UNMASK, 0, 15);
}

static void lcd_vx1_timeout_reset(struct work_struct *p_work)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (vx1_timeout_reset_flag == 0)
		return;

	LCDPR("%s\n", __func__);
	lcd_drv->module_reset();
	if (lcd_drv->lcd_config->lcd_control.vbyone_config->intr_en)
		lcd_vx1_hold_reset();
	vx1_timeout_reset_flag = 0;
}

#define VSYNC_CNT_VX1_RESET   5
#define VSYNC_CNT_VX1_STABLE  20
static unsigned short vsync_cnt = VSYNC_CNT_VX1_STABLE;
static irqreturn_t lcd_vbyone_vsync_isr(int irq, void *dev_id)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vx1_conf;

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;
	if ((lcd_drv->lcd_status & LCD_STATUS_IF_ON) == 0)
		return IRQ_HANDLED;
	if (lcd_vcbus_read(VBO_STATUS_L) & 0x40) /* hpd detect */
		return IRQ_HANDLED;

	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 0, 1);
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 1);

	if (vx1_conf->vsync_intr_en == 0) {
		vx1_training_wait_cnt = 0;
		return IRQ_HANDLED;
	}

	if (vx1_conf->vsync_intr_en == 3) {
		if (vsync_cnt < VSYNC_CNT_VX1_RESET)
			vsync_cnt++;
		else if (vsync_cnt == VSYNC_CNT_VX1_RESET) {
			lcd_vbyone_hw_filter(0);
			lcd_vbyone_sw_reset();
			vsync_cnt++;
		} else if ((vsync_cnt > VSYNC_CNT_VX1_RESET) &&
			(vsync_cnt < VSYNC_CNT_VX1_STABLE)) {
			if (lcd_vcbus_read(VBO_STATUS_L) & 0x20) {
				vsync_cnt = VSYNC_CNT_VX1_STABLE;
				lcd_vbyone_hw_filter(1);
			} else {
				vsync_cnt++;
			}
		}
	} else if (vx1_conf->vsync_intr_en == 2) {
		if (vsync_cnt >= 5) {
			vsync_cnt = 0;
			if (!(lcd_vcbus_read(VBO_STATUS_L) & 0x20)) {
				lcd_vbyone_hw_filter(0);
				lcd_vbyone_sw_reset();
				LCDPR("vx1 sw_reset 2\n");
				while (lcd_vcbus_read(VBO_STATUS_L) & 0x4)
					break;

				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 15, 1);
				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 15, 1);
			} else {
				lcd_vbyone_hw_filter(1);
			}
		} else
			vsync_cnt++;
	} else {
		if (vx1_training_wait_cnt >= VX1_TRAINING_TIMEOUT) {
			if ((lcd_vcbus_read(VBO_STATUS_L) & 0x3f) != 0x20) {
				if (vx1_timeout_reset_flag == 0) {
					vx1_timeout_reset_flag = 1;
					if (lcd_drv->workqueue) {
						queue_work(lcd_drv->workqueue,
							&lcd_vx1_reset_work);
					} else {
						schedule_work(
							&lcd_vx1_reset_work);
					}
				}
			} else {
				vx1_training_stable_cnt++;
				if (vx1_training_stable_cnt >= 5) {
					vx1_training_wait_cnt = 0;
					vx1_training_stable_cnt = 0;
				}
			}
		} else {
			vx1_training_wait_cnt++;
		}
	}

	return IRQ_HANDLED;
}

#define VX1_LOCKN_WAIT_CNT_MAX    50
static int vx1_lockn_wait_cnt;

#define VX1_FSM_ACQ_NEXT_STEP_CONTINUE     0
#define VX1_FSM_ACQ_NEXT_RELEASE_HOLDER    1
#define VX1_FSM_ACQ_NEXT                   VX1_FSM_ACQ_NEXT_STEP_CONTINUE
static irqreturn_t lcd_vbyone_interrupt_handler(int irq, void *dev_id)
{
	unsigned int data32, data32_1;
	int encl_clk;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vx1_conf;

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;

	lcd_vcbus_write(VBO_INTR_UNMASK, 0x0);  /* mask interrupt */

	encl_clk = lcd_encl_clk_msr();
	data32 = (lcd_vcbus_read(VBO_INTR_STATE) & 0x7fff);
	/* clear the interrupt */
	data32_1 = ((data32 >> 9) << 3);
	if (data32 & 0x1c0)
		data32_1 |= (1 << 2);
	if (data32 & 0x38)
		data32_1 |= (1 << 1);
	if (data32 & 0x7)
		data32_1 |= (1 << 0);
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, data32_1, 0, 9);
	lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 0, 9);
	if (lcd_debug_print_flag) {
		LCDPR("vx1 intr status = 0x%04x, encl_clkmsr = %d",
			data32, encl_clk);
	}

	if (vx1_conf->vsync_intr_en == 3) {
		if (data32 & 0x1000) {
			if (vsync_cnt >= (VSYNC_CNT_VX1_RESET + 1)) {
				vsync_cnt = 0;
				LCDPR("vx1 lockn rise edge occurred\n");
			}
		}
	} else {
		if (data32 & 0x200) {
			LCDPR("vx1 htpdn fall occurred\n");
			vx1_fsm_acq_st = 0;
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 15, 1);
		}
		if (data32 & 0x800) {
			if (lcd_debug_print_flag)
				LCDPR("vx1 lockn fall occurred\n");
			vx1_fsm_acq_st = 0;
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 15, 1);
			if (vx1_lockn_wait_cnt++ > VX1_LOCKN_WAIT_CNT_MAX) {
				if (vx1_timeout_reset_flag == 0) {
					vx1_timeout_reset_flag = 1;
					if (lcd_drv->workqueue) {
						queue_work(lcd_drv->workqueue,
							&lcd_vx1_reset_work);
					} else {
						schedule_work(
							&lcd_vx1_reset_work);
					}
					vx1_lockn_wait_cnt = 0;
					return IRQ_HANDLED;
				}
			}
		}
		if (data32 & 0x2000) {
			/* LCDPR("vx1 fsm_acq wait end\n"); */
			if (lcd_debug_print_flag) {
				LCDPR("vx1 status 0: 0x%x, fsm_acq_st: %d\n",
					lcd_vcbus_read(VBO_STATUS_L),
					vx1_fsm_acq_st);
			}
			if (vx1_fsm_acq_st == 0) {
				/* clear FSM_continue */
				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 15, 1);
				LCDPR("vx1 sw reset\n");
				/* force PHY to 0 */
				lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 3, 8, 2);
				lcd_vcbus_write(VBO_SOFT_RST, 0x1ff);
				udelay(5);
				/* clear lockn raising flag */
				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 7, 1);
				/* realease PHY */
				lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 0, 8, 2);
				/* clear lockn raising flag */
				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 7, 1);
				lcd_vcbus_write(VBO_SOFT_RST, 0);
				vx1_fsm_acq_st = 1;
			} else {
				vx1_fsm_acq_st = 2;
				/* set FSM_continue */
#if (VX1_FSM_ACQ_NEXT == VX1_FSM_ACQ_NEXT_RELEASE_HOLDER)
				lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0, 0, 16);
#else
				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 15, 1);
				lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 15, 1);
#endif
			}
			if (lcd_debug_print_flag) {
				LCDPR("vx1 status 1: 0x%x, fsm_acq_st: %d\n",
					lcd_vcbus_read(VBO_STATUS_L),
					vx1_fsm_acq_st);
			}
		}

		if (data32 & 0x1ff) {
			if (lcd_debug_print_flag)
				LCDPR("vx1 reset for timing err\n");
			vx1_fsm_acq_st = 0;
			/* force PHY to 0 */
			lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 3, 8, 2);
			lcd_vcbus_write(VBO_SOFT_RST, 0x1ff);
			udelay(5);
			/* clear lockn raising flag */
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 1, 7, 1);
			/* realease PHY */
			lcd_hiu_setb(HHI_LVDS_TX_PHY_CNTL0, 0, 8, 2);
			/* clear lockn raising flag */
			lcd_vcbus_setb(VBO_INTR_STATE_CTRL, 0, 7, 1);
			lcd_vcbus_write(VBO_SOFT_RST, 0);
		}

		udelay(20);
		if ((lcd_vcbus_read(VBO_STATUS_L) & 0x3f) == 0x20) {
			vx1_lockn_wait_cnt = 0;
			/* vx1_training_wait_cnt = 0; */
#if (VX1_FSM_ACQ_NEXT == VX1_FSM_ACQ_NEXT_RELEASE_HOLDER)
			lcd_vcbus_setb(VBO_FSM_HOLDER_L, 0xffff, 0, 16);
#endif
			lcd_vbyone_hw_filter(1);
			LCDPR("vx1 fsm stable\n");
		}
	}
	/* enable interrupt */
	lcd_vcbus_setb(VBO_INTR_UNMASK, VBYONE_INTR_UNMASK, 0, 15);

	return IRQ_HANDLED;
}

static void lcd_p2p_control_set(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int reg_cntl0, reg_cntl1;
	unsigned int phy_div;
	unsigned int channel_sel0, channel_sel1;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	/* phy_div: 0=div6, 1=div 7, 2=div8, 3=div10 */
	switch (pconf->lcd_control.p2p_config->p2p_type) {
	case P2P_CHPI: /* 8/10 coding */
		phy_div = 3;
		break;
	default:
		phy_div = 2;
		break;
	}

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

	/* fifo_clk_sel[7:6]: 0=div6, 1=div 7, 2=div8, 3=div10 */
	lcd_hiu_write(reg_cntl0, (phy_div << 6));
	/* serializer_en[27:16] */
	lcd_hiu_setb(reg_cntl0, 0xfff, 16, 12);
	/* pn swap[2] */
	lcd_hiu_setb(reg_cntl0, 1, 2, 1);

	/* fifo enable[30], phy_clock gating[24] */
	lcd_hiu_write(reg_cntl1, (1 << 30) | (1 << 24));
	/* fifo write enable[31] */
	lcd_hiu_setb(reg_cntl1, 1, 31, 1);

	channel_sel0 = pconf->lcd_control.p2p_config->channel_sel0;
	channel_sel1 = pconf->lcd_control.p2p_config->channel_sel1;
	lcd_vcbus_write(P2P_CH_SWAP0, channel_sel0);
	lcd_vcbus_write(P2P_CH_SWAP1, channel_sel1);

	lcd_tcon_enable(pconf);
}

static void lcd_p2p_disable(void)
{
	lcd_tcon_disable();
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
	 * byte_mode * 8 * 10 / 8 * pclk
	 */
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

static void lcd_mlvds_config_set(struct lcd_config_s *pconf)
{
	unsigned int bit_rate, pclk;
	unsigned int lcd_bits, channel_num;
	unsigned int channel_sel0, channel_sel1, pi_clk_sel = 0;
	unsigned int i, temp;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	lcd_bits = pconf->lcd_basic.lcd_bits;
	channel_num = pconf->lcd_control.mlvds_config->channel_num;
	pclk = pconf->lcd_timing.lcd_clk / 1000;
	bit_rate = lcd_bits * 3 * pclk / channel_num;

	pconf->lcd_timing.bit_rate = bit_rate * 1000;

	if (lcd_debug_print_flag) {
		LCDPR("channel_num=%u, bit_rate=%u.%03uMHz, pclk=%u.%03uMhz\n",
			channel_num, (bit_rate / 1000), (bit_rate % 1000),
			(pclk / 1000), (pclk % 1000));
	}

	/* pi_clk select */
	channel_sel0 = pconf->lcd_control.mlvds_config->channel_sel0;
	channel_sel1 = pconf->lcd_control.mlvds_config->channel_sel1;
	/* mlvds channel:    //tx 12 channels
	 *    0: clk_a
	 *    1: d0_a
	 *    2: d1_a
	 *    3: d2_a
	 *    4: d3_a
	 *    5: d4_a
	 *    6: clk_b
	 *    7: d0_b
	 *    8: d1_b
	 *    9: d2_b
	 *   10: d3_b
	 *   11: d4_b
	 */
	for (i = 0; i < 8; i++) {
		temp = (channel_sel0 >> (i*4)) & 0xf;
		if ((temp == 0) || (temp == 6))
			pi_clk_sel |= (1 << i);
	}
	for (i = 0; i < 4; i++) {
		temp = (channel_sel1 >> (i*4)) & 0xf;
		if ((temp == 0) || (temp == 6))
			pi_clk_sel |= (1 << (i + 8));
	}
	pconf->lcd_control.mlvds_config->pi_clk_sel = pi_clk_sel;
	if (lcd_debug_print_flag) {
		LCDPR(
		"channel_sel0=0x%08x, channel_sel1=0x%08x, pi_clk_sel=0x%03x\n",
			channel_sel0, channel_sel1, pi_clk_sel);
	}
}

static void lcd_p2p_config_set(struct lcd_config_s *pconf)
{
	unsigned int bit_rate, pclk;
	unsigned int lcd_bits, lane_num;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	lcd_bits = pconf->lcd_basic.lcd_bits;
	lane_num = pconf->lcd_control.p2p_config->lane_num;
	pclk = pconf->lcd_timing.lcd_clk / 1000;
	switch (pconf->lcd_control.p2p_config->p2p_type) {
	case P2P_CEDS:
		if (pclk >= 600000)
			bit_rate = pclk * 3 * lcd_bits / lane_num;
		else
			bit_rate = pclk * (3 * lcd_bits + 4) / lane_num;
		break;
	case P2P_CHPI: /* 8/10 coding */
		bit_rate = (pclk * 3 * lcd_bits * 10 / 8) / lane_num;
		break;
	default:
		bit_rate = pclk * 3 * lcd_bits / lane_num;
		break;
	}

	pconf->lcd_timing.bit_rate = bit_rate * 1000;

	if (lcd_debug_print_flag) {
		LCDPR("lane_num=%u, bit_rate=%u.%03uMHz, pclk=%u.%03uMhz\n",
			lane_num, (bit_rate / 1000), (bit_rate % 1000),
			(pclk / 1000), (pclk % 1000));
	}
}

void lcd_tv_clk_config_change(struct lcd_config_s *pconf)
{
#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(pconf->lcd_timing.lcd_clk, VPU_VENCL);
#endif
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_VBYONE:
		lcd_vbyone_config_set(pconf);
		break;
	case LCD_MLVDS:
		lcd_mlvds_config_set(pconf);
		break;
	case LCD_P2P:
		lcd_p2p_config_set(pconf);
		break;
	default:
		break;
	}
	lcd_clk_generate_parameter(pconf);
}

void lcd_tv_clk_update(struct lcd_config_s *pconf)
{
	lcd_tv_clk_config_change(pconf);

	if (pconf->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_vbyone_interrupt_enable(0);
	lcd_clk_set(pconf);
	if (pconf->lcd_basic.lcd_type == LCD_VBYONE)
		lcd_vbyone_wait_stable();
}

void lcd_tv_config_update(struct lcd_config_s *pconf)
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
	case LCD_MLVDS:
		lcd_mlvds_config_set(pconf);
		break;
	case LCD_P2P:
		lcd_p2p_config_set(pconf);
		break;
	default:
		break;
	}
}

void lcd_tv_driver_init_pre(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	pconf = lcd_drv->lcd_config;
	LCDPR("tv driver init(ver %s): %s\n", lcd_drv->version,
		lcd_type_type_to_str(pconf->lcd_basic.lcd_type));
	ret = lcd_type_supported(pconf);
	if (ret)
		return;

#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(pconf->lcd_timing.lcd_clk, VPU_VENCL);
	switch_vpu_mem_pd_vmod(VPU_VENCL, VPU_MEM_POWER_ON);
#endif
	lcd_clk_gate_switch(1);

	/* init driver */
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_VBYONE:
		lcd_vbyone_interrupt_enable(0);
		break;
	default:
		break;
	}

	lcd_clk_set(pconf);
	lcd_venc_set(pconf);
	lcd_encl_tcon_set(pconf);
	lcd_drv->lcd_mute_state = 1;

	lcd_vcbus_write(VENC_INTCTRL, 0x200);

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
}

void lcd_tv_driver_disable_post(void)
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

int lcd_tv_driver_init(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	pconf = lcd_drv->lcd_config;
	ret = lcd_type_supported(pconf);
	if (ret)
		return -1;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		lcd_lvds_control_set(pconf);
		lcd_lvds_phy_set(pconf, 1);
		break;
	case LCD_VBYONE:
		lcd_vbyone_pinmux_set(1);
		lcd_vbyone_control_set(pconf);
		lcd_vbyone_wait_hpd(pconf);
		lcd_vbyone_phy_set(pconf, 1);
		lcd_vx1_intr_request = 1;
		if (lcd_drv->workqueue) {
			queue_delayed_work(lcd_drv->workqueue,
				&lcd_vx1_stable_delayed_work,
				msecs_to_jiffies(LCD_VX1_WAIT_STABLE_DELAY));
		} else {
			schedule_delayed_work(&lcd_vx1_stable_delayed_work,
				msecs_to_jiffies(LCD_VX1_WAIT_STABLE_DELAY));
		}
		break;
	case LCD_MLVDS:
		lcd_mlvds_control_set(pconf);
		lcd_tcon_pinmux_set(1);
		lcd_mlvds_phy_set(pconf, 1);
		break;
	case LCD_P2P:
		lcd_tcon_pinmux_set(1);
		lcd_p2p_phy_set(pconf, 1);
		lcd_p2p_control_set(pconf);
		break;
	default:
		break;
	}

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
	return 0;
}

void lcd_tv_driver_disable(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	vx1_training_wait_cnt = 0;
	vx1_timeout_reset_flag = 0;

	LCDPR("disable driver\n");
	pconf = lcd_drv->lcd_config;
	ret = lcd_type_supported(pconf);
	if (ret)
		return;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		lcd_lvds_phy_set(pconf, 0);
		lcd_lvds_disable();
		break;
	case LCD_VBYONE:
		lcd_vbyone_interrupt_enable(0);
		lcd_vbyone_phy_set(pconf, 0);
		lcd_vbyone_pinmux_set(0);
		lcd_vbyone_disable();
		break;
	case LCD_MLVDS:
		lcd_mlvds_disable();
		lcd_mlvds_phy_set(pconf, 0);
		lcd_tcon_pinmux_set(0);
		break;
	case LCD_P2P:
		lcd_p2p_disable();
		lcd_p2p_phy_set(pconf, 0);
		lcd_tcon_pinmux_set(0);
		break;
	default:
		break;
	}

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
}

int lcd_tv_driver_change(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int ret;

	pconf = lcd_drv->lcd_config;
	LCDPR("tv driver change(ver %s): %s\n", lcd_drv->version,
		lcd_type_type_to_str(pconf->lcd_basic.lcd_type));
	ret = lcd_type_supported(pconf);
	if (ret)
		return -1;

	lcd_tv_config_update(pconf);
#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(pconf->lcd_timing.lcd_clk, VPU_VENCL);
#endif

	if (lcd_drv->lcd_status & LCD_STATUS_ENCL_ON) {
		if (pconf->lcd_basic.lcd_type == LCD_VBYONE) {
			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_vbyone_interrupt_enable(0);
		}

		lcd_clk_change(pconf);
		lcd_venc_change(pconf);

		if (pconf->lcd_basic.lcd_type == LCD_VBYONE) {
			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_vbyone_wait_stable();
		}
	} else {
		/* only change parameters when panel is off */
		switch (pconf->lcd_timing.clk_change) {
		case LCD_CLK_PLL_CHANGE:
			lcd_clk_generate_parameter(pconf);
			break;
		default:
			break;
		}
	}

	if (lcd_debug_print_flag)
		LCDPR("%s finished\n", __func__);
	return 0;
}

#define VBYONE_IRQF   IRQF_SHARED /* IRQF_DISABLED */ /* IRQF_SHARED */

int lcd_vbyone_interrupt_up(void)
{
	unsigned int viu_vsync_irq = 0, venc_vx1_irq = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_vbyone_interrupt_init(lcd_drv);
	vx1_lockn_wait_cnt = 0;
	vx1_training_wait_cnt = 0;
	vx1_timeout_reset_flag = 0;
	vx1_training_stable_cnt = 0;
	lcd_vx1_intr_request = 0;
	lcd_encl_clk_err_cnt = 0;

	INIT_WORK(&lcd_vx1_reset_work, lcd_vx1_timeout_reset);

	INIT_DELAYED_WORK(&lcd_vx1_stable_delayed_work,
		lcd_vx1_wait_stable_delayed);

	if (!lcd_drv->res_vsync_irq) {
		LCDERR("res_vsync_irq is null\n");
		return -1;
	}
	viu_vsync_irq = lcd_drv->res_vsync_irq->start;
	LCDPR("viu_vsync_irq: %d\n", viu_vsync_irq);

	if (request_irq(viu_vsync_irq, lcd_vbyone_vsync_isr,
		IRQF_SHARED, "vbyone_vsync", (void *)"vbyone_vsync"))
		LCDERR("can't request viu_vsync_irq\n");
	else {
		if (lcd_debug_print_flag)
			LCDPR("request viu_vsync_irq successful\n");
	}

	if (!lcd_drv->res_vx1_irq) {
		LCDERR("res_vx1_irq is null\n");
		return -1;
	}
	venc_vx1_irq = lcd_drv->res_vx1_irq->start;
	LCDPR("venc_vx1_irq: %d\n", venc_vx1_irq);

	if (request_irq(venc_vx1_irq, lcd_vbyone_interrupt_handler, 0,
		"vbyone", (void *)"vbyone"))
		LCDERR("can't request venc_vx1_irq\n");
	else {
		if (lcd_debug_print_flag)
			LCDPR("request venc_vx1_irq successful\n");
	}

	lcd_vx1_intr_request = 1;
	lcd_vbyone_interrupt_enable(1);

	/* add timer to monitor hpll frequency */
	init_timer(&vx1_hpll_timer);
	/* vx1_hpll_timer.data = NULL; */
	vx1_hpll_timer.function = lcd_vx1_hpll_timer_handler;
	vx1_hpll_timer.expires = jiffies + VX1_HPLL_INTERVAL;
	add_timer(&vx1_hpll_timer);
	LCDPR("add vbyone hpll timer handler\n");

	return 0;
}

void lcd_vbyone_interrupt_down(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	del_timer_sync(&vx1_hpll_timer);

	lcd_vbyone_interrupt_enable(0);
	free_irq(lcd_drv->res_vx1_irq->start, (void *)"vbyone");
	free_irq(lcd_drv->res_vsync_irq->start, (void *)"vbyone_vsync");
	cancel_work_sync(&lcd_vx1_reset_work);
	cancel_delayed_work(&lcd_vx1_stable_delayed_work);

	if (lcd_debug_print_flag)
		LCDPR("free vbyone irq\n");
}
