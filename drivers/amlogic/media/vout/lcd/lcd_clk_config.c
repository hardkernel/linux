/*
 * drivers/amlogic/media/vout/lcd/lcd_clk_config.c
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
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/clk.h>
#ifdef CONFIG_AMLOGIC_VPU
#include <linux/amlogic/media/vpu/vpu.h>
#endif
#include "lcd_reg.h"
#include "lcd_clk_config.h"
#include "lcd_clk_ctrl.h"

static spinlock_t lcd_clk_lock;

static struct lcd_clk_config_s clk_conf = { /* unit: kHz */
	/* IN-OUT parameters */
	.fin = FIN_FREQ,
	.fout = 0,

	/* pll parameters */
	.pll_mode = 0, /* txl */
	.pll_od_fb = 0,
	.pll_m = 0,
	.pll_n = 0,
	.pll_od1_sel = 0,
	.pll_od2_sel = 0,
	.pll_od3_sel = 0,
	.pll_tcon_div_sel = 0,
	.pll_level = 0,
	.ss_level = 0,
	.ss_freq = 0,
	.ss_mode = 0,
	.div_sel = 0,
	.xd = 0,
	.pll_fout = 0,

	/* clk path node parameters */
	.div_sel_max = 0,
	.xd_max = 0,

	.data = NULL,
};

static struct lcd_clktree_s lcd_clktree = {
	.clk_gate_state = 0,
	.clk_gate_optional_state = 0, /* by interface */

	.encl_top_gate = NULL,
	.encl_int_gate = NULL,

	.dsi_host_gate = NULL,
	.dsi_phy_gate = NULL,
	.dsi_meas = NULL,
	.mipi_enable_gate = NULL,
	.mipi_bandgap_gate = NULL,
	.gp0_pll = NULL,
	.tcon_gate = NULL,
	.tcon_clk = NULL,
};

struct lcd_clk_config_s *get_lcd_clk_config(void)
{
	return &clk_conf;
}

/* ****************************************************
 * lcd pll & clk operation
 * ****************************************************
 */
static int lcd_pll_wait_lock(unsigned int reg, unsigned int lock_bit)
{
	unsigned int pll_lock;
	int wait_loop = PLL_WAIT_LOCK_CNT; /* 200 */
	int ret = 0;

	do {
		udelay(50);
		pll_lock = lcd_hiu_getb(reg, lock_bit, 1);
		wait_loop--;
	} while ((pll_lock == 0) && (wait_loop > 0));
	if (pll_lock == 0)
		ret = -1;
	LCDPR("%s: pll_lock=%d, wait_loop=%d\n",
		__func__, pll_lock, (PLL_WAIT_LOCK_CNT - wait_loop));
	return ret;
}

#define PLL_WAIT_LOCK_CNT_G12A    1000
static int lcd_pll_wait_lock_g12a(int path)
{
	unsigned int pll_ctrl, pll_ctrl3, pll_ctrl6;
	unsigned int pll_lock;
	int wait_loop = PLL_WAIT_LOCK_CNT_G12A; /* 200 */
	int ret = 0;

	if (path) {
		pll_ctrl = HHI_GP0_PLL_CNTL0_G12A;
		pll_ctrl3 = HHI_GP0_PLL_CNTL3_G12A;
		pll_ctrl6 = HHI_GP0_PLL_CNTL6_G12A;
	} else {
		pll_ctrl = HHI_HDMI_PLL_CNTL;
		pll_ctrl3 = HHI_HDMI_PLL_CNTL4;
		pll_ctrl6 = HHI_HDMI_PLL_CNTL7;
	}
	do {
		udelay(50);
		pll_lock = lcd_hiu_getb(pll_ctrl, 31, 1);
		wait_loop--;
	} while ((pll_lock != 1) && (wait_loop > 0));

	if (pll_lock == 1) {
		goto pll_lock_end_g12a;
	} else {
		LCDPR("path: %d, pll try 1, lock: %d\n", path, pll_lock);
		lcd_hiu_setb(pll_ctrl3, 1, 31, 1);
		wait_loop = PLL_WAIT_LOCK_CNT_G12A;
		do {
			udelay(50);
			pll_lock = lcd_hiu_getb(pll_ctrl, 31, 1);
			wait_loop--;
		} while ((pll_lock != 1) && (wait_loop > 0));
	}

	if (pll_lock == 1) {
		goto pll_lock_end_g12a;
	} else {
		LCDPR("path: %d, pll try 2, lock: %d\n", path, pll_lock);
		lcd_hiu_write(pll_ctrl6, 0x55540000);
		wait_loop = PLL_WAIT_LOCK_CNT_G12A;
		do {
			udelay(50);
			pll_lock = lcd_hiu_getb(pll_ctrl, 31, 1);
			wait_loop--;
		} while ((pll_lock != 1) && (wait_loop > 0));
	}

	if (pll_lock != 1)
		ret = -1;

pll_lock_end_g12a:
	LCDPR("%s: path=%d, pll_lock=%d, wait_loop=%d\n",
		__func__, path, pll_lock, (PLL_WAIT_LOCK_CNT_G12A - wait_loop));
	return ret;
}

static void lcd_set_pll_ss_level_txl(unsigned int level)
{
	unsigned int pll_ctrl3, pll_ctrl4;

	pll_ctrl3 = lcd_hiu_read(HHI_HDMI_PLL_CNTL3);
	pll_ctrl4 = lcd_hiu_read(HHI_HDMI_PLL_CNTL4);
	pll_ctrl3 &= ~((0xf << 10) | (1 << 14));
	pll_ctrl4 &= ~(0x3 << 2);

	pll_ctrl3 |= pll_ss_reg_txl[level][0];
	pll_ctrl4 |= pll_ss_reg_txl[level][1];

	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, pll_ctrl4);

	LCDPR("set pll spread spectrum: %s\n", lcd_ss_level_table_txl[level]);
}

static void lcd_set_pll_txl(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl3;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((1 << LCD_PLL_EN_TXL) |
		(cConf->pll_n << LCD_PLL_N_TXL) |
		(cConf->pll_m << LCD_PLL_M_TXL));
	pll_ctrl2 = 0x800ca000;
	pll_ctrl2 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl3 = 0x860330c4 | (cConf->pll_od_fb << 30);
	pll_ctrl3 |= ((cConf->pll_od3_sel << LCD_PLL_OD3_TXL) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_TXL) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_TXL));

set_pll_retry_txl:
	lcd_hiu_write(HHI_HDMI_PLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, pll_ctrl3);
	if (cConf->pll_mode)
		lcd_hiu_write(HHI_HDMI_PLL_CNTL4, 0x0d160000);
	else
		lcd_hiu_write(HHI_HDMI_PLL_CNTL4, 0x0c8e0000);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, 0x001fa729);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL6, 0x01a31500);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 1, LCD_PLL_RST_TXL, 1);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 0, LCD_PLL_RST_TXL, 1);

	ret = lcd_pll_wait_lock(HHI_HDMI_PLL_CNTL, LCD_PLL_LOCK_TXL);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_pll_retry_txl;
		LCDERR("hpll lock failed\n");
	}

	if (cConf->ss_level > 0)
		lcd_set_pll_ss_level_txl(cConf->ss_level);
}

static void lcd_set_pll_ss_level_txlx(unsigned int level)
{
	unsigned int pll_ctrl3, pll_ctrl4, pll_ctrl5;

	pll_ctrl3 = lcd_hiu_read(HHI_HDMI_PLL_CNTL3);
	pll_ctrl4 = lcd_hiu_read(HHI_HDMI_PLL_CNTL4);
	pll_ctrl5 = lcd_hiu_read(HHI_HDMI_PLL_CNTL5);
	pll_ctrl3 &= ~((0xf << 10) | (1 << 14));
	pll_ctrl4 &= ~(0x3 << 2);
	pll_ctrl5 &= ~(0x3 << 30);

	pll_ctrl3 |= pll_ss_reg_txlx[level][0];
	pll_ctrl4 |= pll_ss_reg_txlx[level][1];
	pll_ctrl5 |= pll_ss_reg_txlx[level][2];

	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, pll_ctrl5);

	LCDPR("set pll spread spectrum: %s\n", lcd_ss_level_table_txlx[level]);
}

static void lcd_set_pll_txlx(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl3;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((1 << LCD_PLL_EN_TXL) |
		(cConf->pll_n << LCD_PLL_N_TXL) |
		(cConf->pll_m << LCD_PLL_M_TXL));
	pll_ctrl2 = 0x800ca000;
	pll_ctrl2 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl3 = 0x860030c4 | (cConf->pll_od_fb << 30);
	pll_ctrl3 |= ((cConf->pll_od3_sel << LCD_PLL_OD3_TXL) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_TXL) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_TXL));

set_pll_retry_txlx:
	lcd_hiu_write(HHI_HDMI_PLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, pll_ctrl3);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, 0x0c8e0000);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, 0x001fa729);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL6, 0x01a31500);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 1, LCD_PLL_RST_TXL, 1);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 0, LCD_PLL_RST_TXL, 1);

	ret = lcd_pll_wait_lock(HHI_HDMI_PLL_CNTL, LCD_PLL_LOCK_TXL);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_pll_retry_txlx;
		LCDERR("hpll lock failed\n");
	}

	if (cConf->ss_level > 0)
		lcd_set_pll_ss_level_txlx(cConf->ss_level);
}

static void lcd_set_pll_axg(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl2;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_AXG) |
		(cConf->pll_n << LCD_PLL_N_AXG) |
		(cConf->pll_m << LCD_PLL_M_AXG) |
		(cConf->pll_od1_sel << LCD_PLL_OD_AXG));
	pll_ctrl1 = 0xc084a000;
	pll_ctrl1 |= ((1 << 12) | (cConf->pll_frac << 0));
	pll_ctrl2 = 0xb75020be | (cConf->pll_od_fb << 19);

set_pll_retry_axg:
	lcd_hiu_write(HHI_GP0_PLL_CNTL_AXG, pll_ctrl);
	lcd_hiu_write(HHI_GP0_PLL_CNTL1_AXG, pll_ctrl1);
	lcd_hiu_write(HHI_GP0_PLL_CNTL2_AXG, pll_ctrl2);
	lcd_hiu_write(HHI_GP0_PLL_CNTL3_AXG, 0x0a59a288);
	lcd_hiu_write(HHI_GP0_PLL_CNTL4_AXG, 0xc000004d);
	if (cConf->pll_fvco >= 1632000)
		lcd_hiu_write(HHI_GP0_PLL_CNTL5_AXG, 0x00058000);
	else
		lcd_hiu_write(HHI_GP0_PLL_CNTL5_AXG, 0x00078000);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL_AXG, 1, LCD_PLL_RST_AXG, 1);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL_AXG, 0, LCD_PLL_RST_AXG, 1);

	ret = lcd_pll_wait_lock(HHI_GP0_PLL_CNTL_AXG, LCD_PLL_LOCK_AXG);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_pll_retry_axg;
		LCDERR("gp0_pll lock failed\n");
	}
}

static void lcd_set_gp0_pll_g12a(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl3, pll_ctrl4, pll_ctrl6;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_GP0_G12A) |
		(cConf->pll_n << LCD_PLL_N_GP0_G12A) |
		(cConf->pll_m << LCD_PLL_M_GP0_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD_GP0_G12A));
	pll_ctrl1 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl3 = 0x6a285c00;
		pll_ctrl4 = 0x65771290;
		pll_ctrl6 = 0x56540000;
	} else {
		pll_ctrl3 = 0x48681c00;
		pll_ctrl4 = 0x33771290;
		pll_ctrl6 = 0x56540000;
	}

set_gp0_pll_retry_g12a:
	lcd_hiu_write(HHI_GP0_PLL_CNTL0_G12A, pll_ctrl);
	lcd_hiu_write(HHI_GP0_PLL_CNTL1_G12A, pll_ctrl1);
	lcd_hiu_write(HHI_GP0_PLL_CNTL2_G12A, 0x00);
	lcd_hiu_write(HHI_GP0_PLL_CNTL3_G12A, pll_ctrl3);
	lcd_hiu_write(HHI_GP0_PLL_CNTL4_G12A, pll_ctrl4);
	lcd_hiu_write(HHI_GP0_PLL_CNTL5_G12A, 0x39272000);
	lcd_hiu_write(HHI_GP0_PLL_CNTL6_G12A, pll_ctrl6);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0_G12A, 1, LCD_PLL_RST_GP0_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0_G12A, 0, LCD_PLL_RST_GP0_G12A, 1);

	ret = lcd_pll_wait_lock_g12a(1);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_gp0_pll_retry_g12a;
		LCDERR("gp0_pll lock failed\n");
	}
}

static void lcd_set_hpll_g12a(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl4, pll_ctrl5, pll_ctrl7;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_HPLL_G12A) |
		(1 << 25) | /* clk out gate */
		(cConf->pll_n << LCD_PLL_N_HPLL_G12A) |
		(cConf->pll_m << LCD_PLL_M_HPLL_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_HPLL_G12A) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_HPLL_G12A) |
		(cConf->pll_od3_sel << LCD_PLL_OD3_HPLL_G12A));
	pll_ctrl2 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl4 = 0x6a285c00;
		pll_ctrl5 = 0x65771290;
		pll_ctrl7 = 0x56540000;
	} else {
		pll_ctrl4 = 0x48681c00;
		pll_ctrl5 = 0x33771290;
		pll_ctrl7 = 0x56540000;
	}

set_hpll_pll_retry_g12a:
	lcd_hiu_write(HHI_HDMI_PLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, 0x00);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, pll_ctrl5);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL6, 0x39272000);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL7, pll_ctrl7);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 1, LCD_PLL_RST_HPLL_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 0, LCD_PLL_RST_HPLL_G12A, 1);

	ret = lcd_pll_wait_lock_g12a(0);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_hpll_pll_retry_g12a;
		LCDERR("hpll lock failed\n");
	}
}

static void lcd_set_gp0_pll_g12b(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1, pll_ctrl3, pll_ctrl4, pll_ctrl6;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_GP0_G12A) |
		(cConf->pll_n << LCD_PLL_N_GP0_G12A) |
		(cConf->pll_m << LCD_PLL_M_GP0_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD_GP0_G12A));
	pll_ctrl1 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl3 = 0x6a285c00;
		pll_ctrl4 = 0x65771290;
		pll_ctrl6 = 0x56540000;
	} else {
		pll_ctrl3 = 0x48681c00;
		pll_ctrl4 = 0x33771290;
		pll_ctrl6 = 0x56540000;
	}

set_gp0_pll_retry_g12b:
	lcd_hiu_write(HHI_GP0_PLL_CNTL0_G12A, pll_ctrl);
	lcd_hiu_write(HHI_GP0_PLL_CNTL1_G12A, pll_ctrl1);
	lcd_hiu_write(HHI_GP0_PLL_CNTL2_G12A, 0x00);
	lcd_hiu_write(HHI_GP0_PLL_CNTL3_G12A, pll_ctrl3);
	lcd_hiu_write(HHI_GP0_PLL_CNTL4_G12A, pll_ctrl4);
	lcd_hiu_write(HHI_GP0_PLL_CNTL5_G12A, 0x39272000);
	lcd_hiu_write(HHI_GP0_PLL_CNTL6_G12A, pll_ctrl6);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0_G12A, 1, LCD_PLL_RST_GP0_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_GP0_PLL_CNTL0_G12A, 0, LCD_PLL_RST_GP0_G12A, 1);

	ret = lcd_pll_wait_lock(HHI_GP0_PLL_CNTL0_G12A, LCD_PLL_LOCK_GP0_G12A);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_gp0_pll_retry_g12b;
		LCDERR("gp0_pll lock failed\n");
	}
}

static void lcd_set_hpll_g12b(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl2, pll_ctrl4, pll_ctrl5, pll_ctrl7;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	pll_ctrl = ((1 << LCD_PLL_EN_HPLL_G12A) |
		(1 << 25) | /* clk out gate */
		(cConf->pll_n << LCD_PLL_N_HPLL_G12A) |
		(cConf->pll_m << LCD_PLL_M_HPLL_G12A) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_HPLL_G12A) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_HPLL_G12A) |
		(cConf->pll_od3_sel << LCD_PLL_OD3_HPLL_G12A));
	pll_ctrl2 = (cConf->pll_frac << 0);
	if (cConf->pll_frac) {
		pll_ctrl |= (1 << 27);
		pll_ctrl4 = 0x6a285c00;
		pll_ctrl5 = 0x65771290;
		pll_ctrl7 = 0x56540000;
	} else {
		pll_ctrl4 = 0x48681c00;
		pll_ctrl5 = 0x33771290;
		pll_ctrl7 = 0x56540000;
	}

set_hpll_pll_retry_g12b:
	lcd_hiu_write(HHI_HDMI_PLL_CNTL, pll_ctrl);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL2, pll_ctrl2);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL3, 0x00);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL4, pll_ctrl4);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL5, pll_ctrl5);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL6, 0x39272000);
	lcd_hiu_write(HHI_HDMI_PLL_CNTL7, pll_ctrl7);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 1, LCD_PLL_RST_HPLL_G12A, 1);
	udelay(100);
	lcd_hiu_setb(HHI_HDMI_PLL_CNTL, 0, LCD_PLL_RST_HPLL_G12A, 1);

	ret = lcd_pll_wait_lock(HHI_HDMI_PLL_CNTL, LCD_PLL_LOCK_HPLL_G12A);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_hpll_pll_retry_g12b;
		LCDERR("hpll lock failed\n");
	}
}

static void lcd_set_pll_ss_level_tl1(unsigned int level)
{
	unsigned int pll_ctrl2;
	unsigned int dep_sel, str_m;

	pll_ctrl2 = lcd_hiu_read(HHI_TCON_PLL_CNTL2);
	pll_ctrl2 &= ~((1 << 15) | (0xf << 16) | (0xf << 28));

	if (level > 0) {
		dep_sel = pll_ss_reg_tl1[level][0];
		str_m = pll_ss_reg_tl1[level][1];
		dep_sel = (dep_sel > 10) ? 10 : dep_sel;
		str_m = (str_m > 10) ? 10 : str_m;
		pll_ctrl2 |= ((1 << 15) | (dep_sel << 28) | (str_m << 16));
	}
	lcd_hiu_write(HHI_TCON_PLL_CNTL2, pll_ctrl2);

	LCDPR("set pll spread spectrum: %s\n", lcd_ss_level_table_tl1[level]);
}

static void lcd_set_pll_ss_advance_tl1(unsigned int freq, unsigned int mode)
{
	unsigned int pll_ctrl2;

	pll_ctrl2 = lcd_hiu_read(HHI_TCON_PLL_CNTL2);
	pll_ctrl2 &= ~(0x7 << 24); /* ss_freq */
	pll_ctrl2 |= (freq << 24);
	pll_ctrl2 &= ~(0x3 << 22); /* ss_mode */
	pll_ctrl2 |= (mode << 22);
	lcd_hiu_write(HHI_TCON_PLL_CNTL2, pll_ctrl2);

	LCDPR("set pll spread spectrum: freq=%d, mode=%d\n", freq, mode);
}

static void lcd_set_pll_tl1(struct lcd_clk_config_s *cConf)
{
	unsigned int pll_ctrl, pll_ctrl1;
	unsigned int tcon_div[5][3] = {
		/* div_mux, div2/4_sel, div4_bypass */
		{1, 0, 1},  /* div1 */
		{0, 0, 1},  /* div2 */
		{0, 1, 1},  /* div4 */
		{0, 0, 0},  /* div8 */
		{0, 1, 0},  /* div16 */
	};
	unsigned int tcon_div_sel = cConf->pll_tcon_div_sel;
	int ret, cnt = 0;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);
	pll_ctrl = ((0x3 << 17) | /* gate ctrl */
		(tcon_div[tcon_div_sel][2] << 16) |
		(cConf->pll_n << LCD_PLL_N_TL1) |
		(cConf->pll_m << LCD_PLL_M_TL1) |
		(cConf->pll_od3_sel << LCD_PLL_OD3_TL1) |
		(cConf->pll_od2_sel << LCD_PLL_OD2_TL1) |
		(cConf->pll_od1_sel << LCD_PLL_OD1_TL1));
	pll_ctrl1 = (1 << 28) |
		(tcon_div[tcon_div_sel][0] << 22) |
		(tcon_div[tcon_div_sel][1] << 21) |
		((1 << 20) | /* sdm_en */
		(cConf->pll_frac << 0));

set_pll_retry_tl1:
	lcd_hiu_write(HHI_TCON_PLL_CNTL0, pll_ctrl);
	udelay(10);
	lcd_hiu_setb(HHI_TCON_PLL_CNTL0, 1, LCD_PLL_RST_TL1, 1);
	udelay(10);
	lcd_hiu_setb(HHI_TCON_PLL_CNTL0, 1, LCD_PLL_EN_TL1, 1);
	udelay(10);
	lcd_hiu_write(HHI_TCON_PLL_CNTL1, pll_ctrl1);
	udelay(10);
	lcd_hiu_write(HHI_TCON_PLL_CNTL2, 0x0000110c);
	udelay(10);
	lcd_hiu_write(HHI_TCON_PLL_CNTL3, 0x10051400);
	udelay(10);
	lcd_hiu_setb(HHI_TCON_PLL_CNTL4, 0x0100c0, 0, 24);
	udelay(10);
	lcd_hiu_setb(HHI_TCON_PLL_CNTL4, 0x8300c0, 0, 24);
	udelay(10);
	lcd_hiu_setb(HHI_TCON_PLL_CNTL0, 1, 26, 1);
	udelay(10);
	lcd_hiu_setb(HHI_TCON_PLL_CNTL0, 0, LCD_PLL_RST_TL1, 1);
	udelay(10);
	lcd_hiu_write(HHI_TCON_PLL_CNTL2, 0x0000300c);

	ret = lcd_pll_wait_lock(HHI_TCON_PLL_CNTL0, LCD_PLL_LOCK_TL1);
	if (ret) {
		if (cnt++ < PLL_RETRY_MAX)
			goto set_pll_retry_tl1;
		LCDERR("hpll lock failed\n");
	} else {
		udelay(100);
		lcd_hiu_setb(HHI_TCON_PLL_CNTL2, 1, 5, 1);
	}

	if (cConf->ss_level > 0) {
		lcd_set_pll_ss_level_tl1(cConf->ss_level);
		lcd_set_pll_ss_advance_tl1(cConf->ss_freq, cConf->ss_mode);
	}
}

static void lcd_set_vid_pll_div(struct lcd_clk_config_s *cConf)
{
	unsigned int shift_val, shift_sel;
	int i;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_EN, 1);
	udelay(5);

	/* Disable the div output clock */
	lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 19, 1);
	lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 15, 1);

	i = 0;
	while (lcd_clk_div_table[i][0] != CLK_DIV_SEL_MAX) {
		if (cConf->div_sel == lcd_clk_div_table[i][0])
			break;
		i++;
	}
	if (lcd_clk_div_table[i][0] == CLK_DIV_SEL_MAX)
		LCDERR("invalid clk divider\n");
	shift_val = lcd_clk_div_table[i][1];
	shift_sel = lcd_clk_div_table[i][2];

	if (shift_val == 0xffff) { /* if divide by 1 */
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 1, 18, 1);
	} else {
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 18, 1);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 16, 2);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 15, 1);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 0, 14);

		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, shift_sel, 16, 2);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 1, 15, 1);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, shift_val, 0, 15);
		lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 0, 15, 1);
	}
	/* Enable the final output clock */
	lcd_hiu_setb(HHI_VID_PLL_CLK_DIV, 1, 19, 1);
}

static void lcd_set_vclk_crt(int lcd_type, struct lcd_clk_config_s *cConf)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	/* setup the XD divider value */
	lcd_hiu_setb(HHI_VIID_CLK_DIV, (cConf->xd-1), VCLK2_XD, 8);
	udelay(5);

	/* select vid_pll_clk */
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, cConf->data->vclk_sel,
		VCLK2_CLK_IN_SEL, 3);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_EN, 1);
	udelay(2);

	/* [15:12] encl_clk_sel, select vclk2_div1 */
	lcd_hiu_setb(HHI_VIID_CLK_DIV, 8, ENCL_CLK_SEL, 4);
	/* release vclk2_div_reset and enable vclk2_div */
	lcd_hiu_setb(HHI_VIID_CLK_DIV, 1, VCLK2_XD_EN, 2);
	udelay(5);

	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_DIV1_EN, 1);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 1, VCLK2_SOFT_RST, 1);
	udelay(10);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_SOFT_RST, 1);
	udelay(5);

	/* enable CTS_ENCL clk gate */
	lcd_hiu_setb(HHI_VID_CLK_CNTL2, 1, ENCL_GATE_VCLK, 1);
}

static void lcd_set_dsi_phy_clk(int sel)
{
	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	lcd_hiu_setb(HHI_MIPIDSI_PHY_CLK_CNTL, sel, 12, 3);
	lcd_hiu_setb(HHI_MIPIDSI_PHY_CLK_CNTL, 1, 8, 1);
	lcd_hiu_setb(HHI_MIPIDSI_PHY_CLK_CNTL, 0, 0, 7);
}

static void lcd_set_tcon_clk(struct lcd_config_s *pconf)
{
	unsigned int freq, val;

	if (lcd_debug_print_flag == 2)
		LCDPR("%s\n", __func__);

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MLVDS:
		val = pconf->lcd_control.mlvds_config->clk_phase & 0xfff;
		lcd_hiu_setb(HHI_TCON_PLL_CNTL1, (val & 0xf), 24, 4);
		lcd_hiu_setb(HHI_TCON_PLL_CNTL4, ((val >> 4) & 0xf), 28, 4);
		lcd_hiu_setb(HHI_TCON_PLL_CNTL4, ((val >> 8) & 0xf), 24, 4);

		/* tcon_clk */
		if (pconf->lcd_timing.lcd_clk >= 100000000) /* 25M */
			freq = 25000000;
		else /* 12.5M */
			freq = 12500000;
		if (!IS_ERR_OR_NULL(lcd_clktree.tcon_clk)) {
			clk_set_rate(lcd_clktree.tcon_clk, freq);
			clk_prepare_enable(lcd_clktree.tcon_clk);
		}
		break;
	case LCD_P2P:
		if (!IS_ERR_OR_NULL(lcd_clktree.tcon_clk)) {
			clk_set_rate(lcd_clktree.tcon_clk, 50000000);
			clk_prepare_enable(lcd_clktree.tcon_clk);
		}
		break;
	default:
		break;
	}
}

/* ****************************************************
 * lcd clk parameters calculate
 * ****************************************************
 */
static int error_abs(int a, int b)
{
	if (a >= b)
		return (a - b);
	else
		return (b - a);
}

static unsigned int clk_vid_pll_div_calc(unsigned int clk,
		unsigned int div_sel, int dir)
{
	unsigned int clk_ret;

	switch (div_sel) {
	case CLK_DIV_SEL_1:
		clk_ret = clk;
		break;
	case CLK_DIV_SEL_2:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 2;
		else
			clk_ret = clk * 2;
		break;
	case CLK_DIV_SEL_3:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 3;
		else
			clk_ret = clk * 3;
		break;
	case CLK_DIV_SEL_3p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 7;
		else
			clk_ret = clk * 7 / 2;
		break;
	case CLK_DIV_SEL_3p75:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 4 / 15;
		else
			clk_ret = clk * 15 / 4;
		break;
	case CLK_DIV_SEL_4:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 4;
		else
			clk_ret = clk * 4;
		break;
	case CLK_DIV_SEL_5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 5;
		else
			clk_ret = clk * 5;
		break;
	case CLK_DIV_SEL_6:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 6;
		else
			clk_ret = clk * 6;
		break;
	case CLK_DIV_SEL_6p25:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 4 / 25;
		else
			clk_ret = clk * 25 / 4;
		break;
	case CLK_DIV_SEL_7:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 7;
		else
			clk_ret = clk * 7;
		break;
	case CLK_DIV_SEL_7p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 15;
		else
			clk_ret = clk * 15 / 2;
		break;
	case CLK_DIV_SEL_12:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 12;
		else
			clk_ret = clk * 12;
		break;
	case CLK_DIV_SEL_14:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 14;
		else
			clk_ret = clk * 14;
		break;
	case CLK_DIV_SEL_15:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk / 15;
		else
			clk_ret = clk * 15;
		break;
	case CLK_DIV_SEL_2p5:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 2 / 5;
		else
			clk_ret = clk * 5 / 2;
		break;
	case CLK_DIV_SEL_4p67:
		if (dir == CLK_DIV_I2O)
			clk_ret = clk * 3 / 14;
		else
			clk_ret = clk * 14 / 3;
		break;
	default:
		clk_ret = clk;
		LCDERR("clk_div_sel: Invalid parameter\n");
		break;
	}

	return clk_ret;
}

static unsigned int clk_vid_pll_div_get(unsigned int clk_div)
{
	unsigned int div_sel;

	/* div * 100 */
	switch (clk_div) {
	case 375:
		div_sel = CLK_DIV_SEL_3p75;
		break;
	case 750:
		div_sel = CLK_DIV_SEL_7p5;
		break;
	case 1500:
		div_sel = CLK_DIV_SEL_15;
		break;
	case 500:
		div_sel = CLK_DIV_SEL_5;
		break;
	default:
		div_sel = CLK_DIV_SEL_MAX;
		break;
	}
	return div_sel;
}

static int check_pll_txl(struct lcd_clk_config_s *cConf,
		unsigned int pll_fout)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_clk_data_s *data = cConf->data;
	unsigned int m, n;
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	unsigned int od_fb = 0, pll_frac;
	int done;

	done = 0;
	if ((pll_fout > data->pll_out_fmax) ||
		(pll_fout < data->pll_out_fmin)) {
		return done;
	}
	for (od3_sel = data->pll_od_sel_max; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = od2_sel; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;
				if ((pll_fvco < data->pll_vco_fmin) ||
					(pll_fvco > data->pll_vco_fmax)) {
					continue;
				}
				cConf->pll_od1_sel = od1_sel - 1;
				cConf->pll_od2_sel = od2_sel - 1;
				cConf->pll_od3_sel = od3_sel - 1;
				cConf->pll_fout = pll_fout;
				if (lcd_debug_print_flag == 2) {
					LCDPR("od1=%d, od2=%d, od3=%d\n",
						(od1_sel - 1), (od2_sel - 1),
						(od3_sel - 1));
					LCDPR("pll_fvco=%d\n", pll_fvco);
				}
				cConf->pll_fvco = pll_fvco;
				n = 1;
				if (lcd_drv->data->chip_type == LCD_CHIP_TXL) {
					if (pll_fvco < 3700000)
						od_fb = 0;
					else
						od_fb = 1;
					cConf->pll_od_fb = od_fb;
				} else {
					od_fb = cConf->pll_od_fb;
				}
				pll_fvco = pll_fvco / od_fb_table[od_fb];
				m = pll_fvco / cConf->fin;
				pll_frac = (pll_fvco % cConf->fin) *
					data->pll_frac_range / cConf->fin;
				cConf->pll_m = m;
				cConf->pll_n = n;
				cConf->pll_frac = pll_frac;
				if (lcd_debug_print_flag == 2) {
					LCDPR("m=%d, n=%d, frac=0x%x\n",
						m, n, pll_frac);
				}
				done = 1;
				break;
			}
		}
	}
	return done;
}

static int check_pll_vco(struct lcd_clk_config_s *cConf, unsigned int pll_fvco)
{
	struct lcd_clk_data_s *data = cConf->data;
	unsigned int m, n;
	unsigned int od_fb = 0, pll_frac;
	int done = 0;

	if ((pll_fvco < data->pll_vco_fmin) ||
		(pll_fvco > data->pll_vco_fmax)) {
		if (lcd_debug_print_flag == 2)
			LCDPR("pll_fvco %d is out of range\n", pll_fvco);
		return done;
	}

	cConf->pll_fvco = pll_fvco;
	n = 1;
	od_fb = cConf->pll_od_fb;
	pll_fvco = pll_fvco / od_fb_table[od_fb];
	m = pll_fvco / cConf->fin;
	pll_frac = (pll_fvco % cConf->fin) * data->pll_frac_range / cConf->fin;
	cConf->pll_m = m;
	cConf->pll_n = n;
	cConf->pll_frac = pll_frac;
	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, frac=0x%x, pll_fvco=%d\n",
			m, n, pll_frac, pll_fvco);
	}
	done = 1;

	return done;
}

#define PLL_FVCO_ERR_MAX    2 /* kHz */
static int check_pll_od(struct lcd_clk_config_s *cConf, unsigned int pll_fout)
{
	struct lcd_clk_data_s *data = cConf->data;
	unsigned int od1_sel, od2_sel, od3_sel, od1, od2, od3;
	unsigned int pll_fod2_in, pll_fod3_in, pll_fvco;
	int done = 0;

	if ((pll_fout > data->pll_out_fmax) ||
		(pll_fout < data->pll_out_fmin)) {
		return done;
	}
	for (od3_sel = data->pll_od_sel_max; od3_sel > 0; od3_sel--) {
		od3 = od_table[od3_sel - 1];
		pll_fod3_in = pll_fout * od3;
		for (od2_sel = od3_sel; od2_sel > 0; od2_sel--) {
			od2 = od_table[od2_sel - 1];
			pll_fod2_in = pll_fod3_in * od2;
			for (od1_sel = od2_sel; od1_sel > 0; od1_sel--) {
				od1 = od_table[od1_sel - 1];
				pll_fvco = pll_fod2_in * od1;
				if ((pll_fvco < data->pll_vco_fmin) ||
					(pll_fvco > data->pll_vco_fmax)) {
					continue;
				}
				if (error_abs(pll_fvco, cConf->pll_fvco) <
					PLL_FVCO_ERR_MAX) {
					cConf->pll_od1_sel = od1_sel - 1;
					cConf->pll_od2_sel = od2_sel - 1;
					cConf->pll_od3_sel = od3_sel - 1;
					cConf->pll_fout = pll_fout;

					if (lcd_debug_print_flag == 2) {
						LCDPR(
						"od1=%d, od2=%d, od3=%d\n",
							(od1_sel - 1),
							(od2_sel - 1),
							(od3_sel - 1));
					}
					done = 1;
					break;
				}
			}
		}
	}
	return done;
}

static void lcd_clk_generate_txl(struct lcd_config_s *pconf)
{
	unsigned int pll_fout, pll_fvco, bit_rate;
	unsigned int clk_div_in, clk_div_out;
	unsigned int clk_div_sel, xd, tcon_div_sel = 0, phy_div = 1;
	unsigned int od1, od2, od3;
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();
	int done;

	done = 0;
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->data->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		goto generate_clk_done_txl;
	}

	bit_rate = pconf->lcd_timing.bit_rate / 1000;

	if (pconf->lcd_timing.clk_auto == 2)
		cConf->pll_mode = 1;
	else
		cConf->pll_mode = 0;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_TTL:
		clk_div_sel = CLK_DIV_SEL_1;
		cConf->xd_max = CRT_VID_DIV_MAX;
		for (xd = 1; xd <= cConf->xd_max; xd++) {
			clk_div_out = cConf->fout * xd;
			if (clk_div_out > cConf->data->div_out_fmax)
				continue;
			if (lcd_debug_print_flag == 2) {
				LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
					cConf->fout, xd, clk_div_out);
			}
			clk_div_in = clk_vid_pll_div_calc(clk_div_out,
					clk_div_sel, CLK_DIV_O2I);
			if (clk_div_in > cConf->data->div_in_fmax)
				continue;
			cConf->xd = xd;
			cConf->div_sel = clk_div_sel;
			pll_fout = clk_div_in;
			if (lcd_debug_print_flag == 2) {
				LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
					lcd_clk_div_sel_table[clk_div_sel],
					clk_div_sel, pll_fout);
			}
			done = check_pll_txl(cConf, pll_fout);
			if (done)
				goto generate_clk_done_txl;
		}
		break;
	case LCD_LVDS:
		clk_div_sel = CLK_DIV_SEL_7;
		xd = 1;
		clk_div_out = cConf->fout * xd;
		if (clk_div_out > cConf->data->div_out_fmax)
			goto generate_clk_done_txl;
		if (lcd_debug_print_flag == 2) {
			LCDPR("fout=%d, xd=%d, clk_div_out=%d\n",
				cConf->fout, xd, clk_div_out);
		}
		clk_div_in = clk_vid_pll_div_calc(clk_div_out,
				clk_div_sel, CLK_DIV_O2I);
		if (clk_div_in > cConf->data->div_in_fmax)
			goto generate_clk_done_txl;
		cConf->xd = xd;
		cConf->div_sel = clk_div_sel;
		pll_fout = clk_div_in;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_sel=%s(index %d), pll_fout=%d\n",
				lcd_clk_div_sel_table[clk_div_sel],
				clk_div_sel, pll_fout);
		}
		done = check_pll_txl(cConf, pll_fout);
		if (done == 0)
			goto generate_clk_done_txl;
		done = 0;
		if (pconf->lcd_control.lvds_config->dual_port)
			phy_div = 2;
		else
			phy_div = 1;
		od1 = od_table[cConf->pll_od1_sel];
		od2 = od_table[cConf->pll_od2_sel];
		od3 = od_table[cConf->pll_od3_sel];
		for (tcon_div_sel = 0; tcon_div_sel < 5; tcon_div_sel++) {
			if (tcon_div_table[tcon_div_sel] ==
				phy_div * od1 * od2 * od3) {
				cConf->pll_tcon_div_sel = tcon_div_sel;
				done = 1;
				break;
			}
		}
		break;
	case LCD_VBYONE:
		cConf->div_sel_max = CLK_DIV_SEL_MAX;
		cConf->xd_max = CRT_VID_DIV_MAX;
		pll_fout = bit_rate;
		clk_div_in = pll_fout;
		if (clk_div_in > cConf->data->div_in_fmax)
			goto generate_clk_done_txl;
		if (lcd_debug_print_flag == 2)
			LCDPR("pll_fout=%d\n", pll_fout);
		if ((clk_div_in / cConf->fout) > 15)
			cConf->xd = 4;
		else
			cConf->xd = 1;
		clk_div_out = cConf->fout * cConf->xd;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_in=%d, fout=%d, xd=%d, clk_div_out=%d\n",
				clk_div_in, cConf->fout,
				cConf->xd, clk_div_out);
		}
		if (clk_div_out > cConf->data->div_out_fmax)
			goto generate_clk_done_txl;
		clk_div_sel = clk_vid_pll_div_get(
				clk_div_in * 100 / clk_div_out);
		if (clk_div_sel == CLK_DIV_SEL_MAX) {
			clk_div_sel = CLK_DIV_SEL_1;
			cConf->xd *= clk_div_in / clk_div_out;
		} else
			cConf->div_sel = clk_div_sel;
		if (lcd_debug_print_flag == 2) {
			LCDPR("clk_div_sel=%s(index %d), xd=%d\n",
				lcd_clk_div_sel_table[clk_div_sel],
				cConf->div_sel, cConf->xd);
		}
		done = check_pll_txl(cConf, pll_fout);
		if (done == 0)
			goto generate_clk_done_txl;
		done = 0;
		od1 = od_table[cConf->pll_od1_sel];
		od2 = od_table[cConf->pll_od2_sel];
		od3 = od_table[cConf->pll_od3_sel];
		for (tcon_div_sel = 0; tcon_div_sel < 5; tcon_div_sel++) {
			if (tcon_div_table[tcon_div_sel] == od1 * od2 * od3) {
				cConf->pll_tcon_div_sel = tcon_div_sel;
				done = 1;
				break;
			}
		}
		break;
	case LCD_MLVDS:
		/* must go through div4 for clk phase */
		for (tcon_div_sel = 3; tcon_div_sel < 5; tcon_div_sel++) {
			pll_fvco = bit_rate * tcon_div_table[tcon_div_sel];
			done = check_pll_vco(cConf, pll_fvco);
			if (done == 0)
				continue;
			cConf->xd_max = CRT_VID_DIV_MAX;
			for (xd = 1; xd <= cConf->xd_max; xd++) {
				clk_div_out = cConf->fout * xd;
				if (clk_div_out > cConf->data->div_out_fmax)
					continue;
				if (lcd_debug_print_flag == 2) {
					LCDPR(
					"fout=%d, xd=%d, clk_div_out=%d\n",
						cConf->fout, xd, clk_div_out);
				}
				for (clk_div_sel = CLK_DIV_SEL_1;
					clk_div_sel < CLK_DIV_SEL_MAX;
					clk_div_sel++) {
					clk_div_in = clk_vid_pll_div_calc(
						clk_div_out, clk_div_sel,
						CLK_DIV_O2I);
					if (clk_div_in >
						cConf->data->div_in_fmax)
						continue;
					cConf->xd = xd;
					cConf->div_sel = clk_div_sel;
					cConf->pll_tcon_div_sel = tcon_div_sel;
					pll_fout = clk_div_in;
					if (lcd_debug_print_flag == 2) {
						LCDPR("clk_div_sel=%s(%d)\n",
					lcd_clk_div_sel_table[clk_div_sel],
							clk_div_sel);
						LCDPR(
					"pll_fout=%d, tcon_div_sel=%d\n",
							pll_fout, tcon_div_sel);
					}
					done = check_pll_od(cConf, pll_fout);
					if (done)
						goto generate_clk_done_txl;
				}
			}
		}
		break;
	case LCD_P2P:
		for (tcon_div_sel = 0; tcon_div_sel < 5; tcon_div_sel++) {
			pll_fvco = bit_rate * tcon_div_table[tcon_div_sel];
			done = check_pll_vco(cConf, pll_fvco);
			if (done == 0)
				continue;
			cConf->xd_max = CRT_VID_DIV_MAX;
			for (xd = 1; xd <= cConf->xd_max; xd++) {
				clk_div_out = cConf->fout * xd;
				if (clk_div_out > cConf->data->div_out_fmax)
					continue;
				if (lcd_debug_print_flag == 2) {
					LCDPR(
					"fout=%d, xd=%d, clk_div_out=%d\n",
						cConf->fout, xd, clk_div_out);
				}
				for (clk_div_sel = CLK_DIV_SEL_1;
					clk_div_sel < CLK_DIV_SEL_MAX;
					clk_div_sel++) {
					clk_div_in = clk_vid_pll_div_calc(
						clk_div_out, clk_div_sel,
						CLK_DIV_O2I);
					if (clk_div_in >
						cConf->data->div_in_fmax)
						continue;
					cConf->xd = xd;
					cConf->div_sel = clk_div_sel;
					cConf->pll_tcon_div_sel = tcon_div_sel;
					pll_fout = clk_div_in;
					if (lcd_debug_print_flag == 2) {
						LCDPR("clk_div_sel=%s(%d)\n",
					lcd_clk_div_sel_table[clk_div_sel],
							clk_div_sel);
						LCDPR(
					"pll_fout=%d, tcon_div_sel=%d\n",
							pll_fout, tcon_div_sel);
					}
					done = check_pll_od(cConf, pll_fout);
					if (done)
						goto generate_clk_done_txl;
				}
			}
		}
		if (done)
			goto generate_clk_done_txl;
		break;
	default:
		break;
	}

generate_clk_done_txl:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_od2_sel << PLL_CTRL_OD2) |
			(cConf->pll_od3_sel << PLL_CTRL_OD3) |
			(cConf->pll_n << PLL_CTRL_N)         |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(cConf->div_sel << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) |
			(1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting\n");
	}
}

static void lcd_pll_frac_generate_txl(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int clk_div_in, clk_div_out, clk_div_sel;
	unsigned int od1, od2, od3, pll_fvco;
	unsigned int m, n, od_fb, frac, offset, temp;
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();

	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	clk_div_sel = cConf->div_sel;
	od1 = od_table[cConf->pll_od1_sel];
	od2 = od_table[cConf->pll_od2_sel];
	od3 = od_table[cConf->pll_od3_sel];
	m = cConf->pll_m;
	n = cConf->pll_n;
	od_fb = cConf->pll_od_fb;

	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, od1=%d, od2=%d, od3=%d\n",
			m, n, cConf->pll_od1_sel, cConf->pll_od2_sel,
			cConf->pll_od3_sel);
		LCDPR("clk_div_sel=%s(index %d), xd=%d\n",
			lcd_clk_div_sel_table[clk_div_sel],
			clk_div_sel, cConf->xd);
	}
	if (cConf->fout > cConf->data->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pclk=%d\n", __func__, cConf->fout);

	clk_div_out = cConf->fout * cConf->xd;
	if (clk_div_out > cConf->data->div_out_fmax) {
		LCDERR("%s: wrong clk_div_out value %dkHz\n",
			__func__, clk_div_out);
		return;
	}

	clk_div_in =
		clk_vid_pll_div_calc(clk_div_out, clk_div_sel, CLK_DIV_O2I);
	if (clk_div_in > cConf->data->div_in_fmax) {
		LCDERR("%s: wrong clk_div_in value %dkHz\n",
			__func__, clk_div_in);
		return;
	}

	pll_fout = clk_div_in;
	if ((pll_fout > cConf->data->pll_out_fmax) ||
		(pll_fout < cConf->data->pll_out_fmin)) {
		LCDERR("%s: wrong pll_fout value %dkHz\n", __func__, pll_fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fout=%d\n", __func__, pll_fout);

	pll_fvco = pll_fout * od1 * od2 * od3;
	if ((pll_fvco < cConf->data->pll_vco_fmin) ||
		(pll_fvco > cConf->data->pll_vco_fmax)) {
		LCDERR("%s: wrong pll_fvco value %dkHz\n", __func__, pll_fvco);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fvco=%d\n", __func__, pll_fvco);

	cConf->pll_fvco = pll_fvco;
	pll_fvco = pll_fvco / od_fb_table[od_fb];
	temp = cConf->fin * m / n;
	if (pll_fvco >= temp) {
		temp = pll_fvco - temp;
		offset = 0;
	} else {
		temp = temp - pll_fvco;
		offset = 1;
	}
	if (temp >= (2 * cConf->fin)) {
		LCDERR("%s: pll changing %dkHz is too much\n",
			__func__, temp);
		return;
	}
	frac = temp * cConf->data->pll_frac_range * n / cConf->fin;
	cConf->pll_frac = frac | (offset << 11);
	if (lcd_debug_print_flag)
		LCDPR("lcd_pll_frac_generate: frac=0x%x\n", frac);
}

static int check_pll_axg(struct lcd_clk_config_s *cConf,
		unsigned int pll_fout)
{
	struct lcd_clk_data_s *data = cConf->data;
	unsigned int m, n, od_sel, od;
	unsigned int pll_fvco;
	unsigned int od_fb = 0, pll_frac;
	int done = 0;

	if ((pll_fout > data->pll_out_fmax) ||
		(pll_fout < data->pll_out_fmin)) {
		return done;
	}
	for (od_sel = data->pll_od_sel_max; od_sel > 0; od_sel--) {
		od = od_table[od_sel - 1];
		pll_fvco = pll_fout * od;
		if ((pll_fvco < data->pll_vco_fmin) ||
			(pll_fvco > data->pll_vco_fmax)) {
			continue;
		}
		cConf->pll_od1_sel = od_sel - 1;
		cConf->pll_fout = pll_fout;
		if (lcd_debug_print_flag == 2) {
			LCDPR("od_sel=%d, pll_fvco=%d\n",
				(od_sel - 1), pll_fvco);
		}

		cConf->pll_fvco = pll_fvco;
		n = 1;
		od_fb = cConf->pll_od_fb;
		pll_fvco = pll_fvco / od_fb_table[od_fb];
		m = pll_fvco / cConf->fin;
		pll_frac = (pll_fvco % cConf->fin) *
				data->pll_frac_range / cConf->fin;
		cConf->pll_m = m;
		cConf->pll_n = n;
		cConf->pll_frac = pll_frac;
		if (lcd_debug_print_flag == 2) {
			LCDPR("pll_m=%d, pll_n=%d, pll_frac=0x%x\n",
				m, n, pll_frac);
		}
		done = 1;
		break;
	}
	return done;
}

static void lcd_clk_generate_axg(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int xd;
	unsigned int dsi_bit_rate_max = 0, dsi_bit_rate_min = 0;
	unsigned int tmp;
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();
	int done;

	done = 0;
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->data->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		goto generate_clk_done_axg;
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MIPI:
		cConf->xd_max = CRT_VID_DIV_MAX;
		tmp = pconf->lcd_control.mipi_config->bit_rate_max;
		dsi_bit_rate_max = tmp * 1000; /* change to kHz */
		dsi_bit_rate_min = dsi_bit_rate_max - cConf->fout;

		for (xd = 1; xd <= cConf->xd_max; xd++) {
			pll_fout = cConf->fout * xd;
			if ((pll_fout > dsi_bit_rate_max) ||
				(pll_fout < dsi_bit_rate_min)) {
				continue;
			}
			if (lcd_debug_print_flag == 2)
				LCDPR("fout=%d, xd=%d\n", cConf->fout, xd);

			pconf->lcd_timing.bit_rate = pll_fout * 1000;
			pconf->lcd_control.mipi_config->clk_factor = xd;
			cConf->xd = xd;
			done = check_pll_axg(cConf, pll_fout);
			if (done)
				goto generate_clk_done_axg;
		}
		break;
	default:
		break;
	}

generate_clk_done_axg:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_n << PLL_CTRL_N) |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting\n");
	}
}

static void lcd_pll_frac_generate_axg(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int od, pll_fvco;
	unsigned int m, n, od_fb, frac, offset, temp;
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();

	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	od = od_table[cConf->pll_od1_sel];
	m = cConf->pll_m;
	n = cConf->pll_n;
	od_fb = cConf->pll_od_fb;

	if (lcd_debug_print_flag == 2) {
		LCDPR("m=%d, n=%d, od=%d, xd=%d\n",
			m, n, cConf->pll_od1_sel, cConf->xd);
	}
	if (cConf->fout > cConf->data->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pclk=%d\n", __func__, cConf->fout);

	pll_fout = cConf->fout * cConf->xd;
	if ((pll_fout > cConf->data->pll_out_fmax) ||
		(pll_fout < cConf->data->pll_out_fmin)) {
		LCDERR("%s: wrong pll_fout value %dkHz\n", __func__, pll_fout);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fout=%d\n", __func__, pll_fout);

	pll_fvco = pll_fout * od;
	if ((pll_fvco < cConf->data->pll_vco_fmin) ||
		(pll_fvco > cConf->data->pll_vco_fmax)) {
		LCDERR("%s: wrong pll_fvco value %dkHz\n", __func__, pll_fvco);
		return;
	}
	if (lcd_debug_print_flag == 2)
		LCDPR("%s pll_fvco=%d\n", __func__, pll_fvco);

	cConf->pll_fvco = pll_fvco;
	pll_fvco = pll_fvco / od_fb_table[od_fb];
	temp = cConf->fin * m / n;
	if (pll_fvco >= temp) {
		temp = pll_fvco - temp;
		offset = 0;
	} else {
		temp = temp - pll_fvco;
		offset = 1;
	}
	if (temp >= (2 * cConf->fin)) {
		LCDERR("%s: pll changing %dkHz is too much\n",
			__func__, temp);
		return;
	}
	frac = temp * cConf->data->pll_frac_range * n / cConf->fin;
	cConf->pll_frac = frac | (offset << 11);
	if (lcd_debug_print_flag)
		LCDPR("lcd_pll_frac_generate: frac=0x%x\n", frac);
}

static void lcd_clk_generate_hpll_g12a(struct lcd_config_s *pconf)
{
	unsigned int pll_fout;
	unsigned int clk_div_sel, xd;
	unsigned int dsi_bit_rate_max = 0, dsi_bit_rate_min = 0;
	unsigned int tmp;
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();
	int done;

	done = 0;
	cConf->fout = pconf->lcd_timing.lcd_clk / 1000; /* kHz */
	cConf->err_fmin = MAX_ERROR;

	if (cConf->fout > cConf->data->xd_out_fmax) {
		LCDERR("%s: wrong lcd_clk value %dkHz\n",
			__func__, cConf->fout);
	}

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_MIPI:
		cConf->xd_max = CRT_VID_DIV_MAX;
		tmp = pconf->lcd_control.mipi_config->bit_rate_max;
		dsi_bit_rate_max = tmp * 1000; /* change to kHz */
		dsi_bit_rate_min = dsi_bit_rate_max - cConf->fout;

		clk_div_sel = CLK_DIV_SEL_1;
		for (xd = 1; xd <= cConf->xd_max; xd++) {
			pll_fout = cConf->fout * xd;
			if ((pll_fout > dsi_bit_rate_max) ||
				(pll_fout < dsi_bit_rate_min)) {
				continue;
			}
			if (lcd_debug_print_flag == 2)
				LCDPR("fout=%d, xd=%d\n", cConf->fout, xd);

			pconf->lcd_timing.bit_rate = pll_fout * 1000;
			pconf->lcd_control.mipi_config->clk_factor = xd;
			cConf->xd = xd;
			cConf->div_sel = clk_div_sel;
			done = check_pll_txl(cConf, pll_fout);
			if (done)
				goto generate_clk_done_g12a;
		}
		break;
	default:
		break;
	}

generate_clk_done_g12a:
	if (done) {
		pconf->lcd_timing.pll_ctrl =
			(cConf->pll_od1_sel << PLL_CTRL_OD1) |
			(cConf->pll_od2_sel << PLL_CTRL_OD2) |
			(cConf->pll_od3_sel << PLL_CTRL_OD3) |
			(cConf->pll_n << PLL_CTRL_N)         |
			(cConf->pll_m << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(cConf->div_sel << DIV_CTRL_DIV_SEL) |
			(cConf->xd << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (cConf->pll_frac << CLK_CTRL_FRAC);
	} else {
		pconf->lcd_timing.pll_ctrl =
			(1 << PLL_CTRL_OD1) |
			(1 << PLL_CTRL_OD2) |
			(1 << PLL_CTRL_OD3) |
			(1 << PLL_CTRL_N)   |
			(50 << PLL_CTRL_M);
		pconf->lcd_timing.div_ctrl =
			(CLK_DIV_SEL_1 << DIV_CTRL_DIV_SEL) |
			(7 << DIV_CTRL_XD);
		pconf->lcd_timing.clk_ctrl = (0 << CLK_CTRL_FRAC);
		LCDERR("Out of clock range, reset to default setting\n");
	}
}

/* ****************************************************
 * lcd clk match function
 * ****************************************************
 */
void lcd_clk_set_txl(struct lcd_config_s *pconf)
{
	lcd_set_pll_txl(&clk_conf);
	lcd_set_vid_pll_div(&clk_conf);
}

static void lcd_clk_set_txlx(struct lcd_config_s *pconf)
{
	lcd_set_pll_txlx(&clk_conf);
	lcd_set_vid_pll_div(&clk_conf);
}

static void lcd_clk_set_axg(struct lcd_config_s *pconf)
{
	lcd_set_pll_axg(&clk_conf);
}

static void lcd_clk_set_g12a_path0(struct lcd_config_s *pconf)
{
	/* hpll */
	lcd_set_hpll_g12a(&clk_conf);
	lcd_set_vid_pll_div(&clk_conf);
	lcd_set_dsi_phy_clk(0);
}

static void lcd_clk_set_g12a_path1(struct lcd_config_s *pconf)
{
	/* gp0_pll */
	lcd_set_gp0_pll_g12a(&clk_conf);
	lcd_set_dsi_phy_clk(1);
}

static void lcd_clk_set_g12b_path0(struct lcd_config_s *pconf)
{
	/* hpll */
	lcd_set_hpll_g12b(&clk_conf);
	lcd_set_vid_pll_div(&clk_conf);
	lcd_set_dsi_phy_clk(0);
}

static void lcd_clk_set_g12b_path1(struct lcd_config_s *pconf)
{
	/* gp0_pll */
	lcd_set_gp0_pll_g12b(&clk_conf);
	lcd_set_dsi_phy_clk(1);
}

static void lcd_clk_set_tl1(struct lcd_config_s *pconf)
{
	lcd_set_tcon_clk(pconf);
	lcd_set_pll_tl1(&clk_conf);
	lcd_set_vid_pll_div(&clk_conf);
}

static void lcd_clk_gate_switch_dft(struct aml_lcd_drv_s *lcd_drv, int status)
{
	if (status) {
		if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
			LCDERR("%s: encl_top_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.encl_top_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
			LCDERR("%s: encl_int_gata\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.encl_int_gate);
	} else {
		if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
			LCDERR("%s: encl_int_gata\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.encl_int_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
			LCDERR("%s: encl_top_gata\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.encl_top_gate);
	}
}

static void lcd_clk_gate_switch_axg(struct aml_lcd_drv_s *lcd_drv, int status)
{
	if (status) {
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
			LCDERR("%s: dsi_host_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.dsi_host_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
			LCDERR("%s: dsi_phy_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.dsi_phy_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
			LCDERR("%s: dsi_meas\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.dsi_meas);
		if (IS_ERR_OR_NULL(lcd_clktree.mipi_enable_gate))
			LCDERR("%s: mipi_enable_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.mipi_enable_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.mipi_bandgap_gate))
			LCDERR("%s: mipi_bandgap_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.mipi_bandgap_gate);
	} else {
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
			LCDERR("%s: dsi_host_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.dsi_host_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
			LCDERR("%s: dsi_phy_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.dsi_phy_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
			LCDERR("%s: dsi_meas\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.dsi_meas);
		if (IS_ERR_OR_NULL(lcd_clktree.mipi_enable_gate))
			LCDERR("%s: mipi_enable_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.mipi_enable_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.mipi_bandgap_gate))
			LCDERR("%s: mipi_bandgap_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.mipi_bandgap_gate);
	}
}

static void lcd_clk_gate_switch_g12a(struct aml_lcd_drv_s *lcd_drv, int status)
{
	if (status) {
		if (clk_conf.data->vclk_sel) {
			if (IS_ERR_OR_NULL(lcd_clktree.gp0_pll))
				LCDERR("%s: gp0_pll\n", __func__);
			else
				clk_prepare_enable(lcd_clktree.gp0_pll);
		}

		if (IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
			LCDERR("%s: dsi_host_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.dsi_host_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
			LCDERR("%s: dsi_phy_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.dsi_phy_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
			LCDERR("%s: dsi_meas\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.dsi_meas);
		if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
			LCDERR("%s: encl_top_gate\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.encl_top_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
			LCDERR("%s: encl_int_gata\n", __func__);
		else
			clk_prepare_enable(lcd_clktree.encl_int_gate);
	} else {
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
			LCDERR("%s: dsi_host_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.dsi_host_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
			LCDERR("%s: dsi_phy_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.dsi_phy_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
			LCDERR("%s: dsi_meas\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.dsi_meas);
		if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
			LCDERR("%s: encl_int_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.encl_int_gate);
		if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
			LCDERR("%s: encl_top_gate\n", __func__);
		else
			clk_disable_unprepare(lcd_clktree.encl_top_gate);

		if (clk_conf.data->vclk_sel) {
			if (IS_ERR_OR_NULL(lcd_clktree.gp0_pll))
				LCDERR("%s: gp0_pll\n", __func__);
			else
				clk_disable_unprepare(lcd_clktree.gp0_pll);
		}
	}
}

static void lcd_clk_gate_optional_switch_tl1(struct aml_lcd_drv_s *lcd_drv,
		int status)
{
	if (status) {
		switch (lcd_drv->lcd_config->lcd_basic.lcd_type) {
		case LCD_MLVDS:
		case LCD_P2P:
			if (IS_ERR_OR_NULL(lcd_clktree.tcon_gate))
				LCDERR("%s: tcon_gate\n", __func__);
			else
				clk_prepare_enable(lcd_clktree.tcon_gate);
			if (IS_ERR_OR_NULL(lcd_clktree.tcon_clk))
				LCDERR("%s: tcon_clk\n", __func__);
			else
				clk_prepare_enable(lcd_clktree.tcon_clk);
			lcd_clktree.clk_gate_optional_state = 1;
			break;
		default:
			break;
		}
	} else {
		switch (lcd_drv->lcd_config->lcd_basic.lcd_type) {
		case LCD_MLVDS:
		case LCD_P2P:
			if (IS_ERR_OR_NULL(lcd_clktree.tcon_clk))
				LCDERR("%s: tcon_clk\n", __func__);
			else
				clk_disable_unprepare(lcd_clktree.tcon_clk);
			if (IS_ERR_OR_NULL(lcd_clktree.tcon_gate))
				LCDERR("%s: tcon_gate\n", __func__);
			else
				clk_disable_unprepare(lcd_clktree.tcon_gate);
			lcd_clktree.clk_gate_optional_state = 0;
			break;
		default:
			break;
		}
	}
}

static void lcd_clktree_probe_dft(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_clktree.clk_gate_state = 0;

	lcd_clktree.encl_top_gate = devm_clk_get(lcd_drv->dev, "encl_top_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
		LCDERR("%s: get encl_top_gate error\n", __func__);

	lcd_clktree.encl_int_gate = devm_clk_get(lcd_drv->dev, "encl_int_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
		LCDERR("%s: get encl_int_gate error\n", __func__);

	LCDPR("lcd_clktree_probe\n");
}

static void lcd_clktree_probe_axg(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_clktree.clk_gate_state = 0;

	lcd_clktree.dsi_host_gate = devm_clk_get(lcd_drv->dev, "dsi_host_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
		LCDERR("%s: clk dsi_host_gate\n", __func__);

	lcd_clktree.dsi_phy_gate = devm_clk_get(lcd_drv->dev, "dsi_phy_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
		LCDERR("%s: clk dsi_phy_gate\n", __func__);

	lcd_clktree.dsi_meas = devm_clk_get(lcd_drv->dev, "dsi_meas");
	if (IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
		LCDERR("%s: clk dsi_meas\n", __func__);

	lcd_clktree.mipi_enable_gate = devm_clk_get(lcd_drv->dev,
		"mipi_enable_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.mipi_enable_gate))
		LCDERR("%s: clk mipi_enable_gate\n", __func__);

	lcd_clktree.mipi_bandgap_gate = devm_clk_get(lcd_drv->dev,
		"mipi_bandgap_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.mipi_bandgap_gate))
		LCDERR("%s: clk mipi_bandgap_gate\n", __func__);

	LCDPR("lcd_clktree_probe\n");
}

static void lcd_clktree_probe_g12a(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_clktree.clk_gate_state = 0;

	lcd_clktree.dsi_host_gate = devm_clk_get(lcd_drv->dev, "dsi_host_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
		LCDERR("%s: clk dsi_host_gate\n", __func__);

	lcd_clktree.dsi_phy_gate = devm_clk_get(lcd_drv->dev, "dsi_phy_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
		LCDERR("%s: clk dsi_phy_gate\n", __func__);

	lcd_clktree.dsi_meas = devm_clk_get(lcd_drv->dev, "dsi_meas");
	if (IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
		LCDERR("%s: clk dsi_meas\n", __func__);

	lcd_clktree.encl_top_gate = devm_clk_get(lcd_drv->dev, "encl_top_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
		LCDERR("%s: clk encl_top_gate\n", __func__);

	lcd_clktree.encl_int_gate = devm_clk_get(lcd_drv->dev, "encl_int_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
		LCDERR("%s: clk encl_int_gate\n", __func__);

	lcd_clktree.gp0_pll = devm_clk_get(lcd_drv->dev, "gp0_pll");
	if (IS_ERR_OR_NULL(lcd_clktree.gp0_pll))
		LCDERR("%s: clk gp0_pll\n", __func__);

	LCDPR("lcd_clktree_probe\n");
}

static void lcd_clktree_probe_tl1(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct clk *temp_clk;

	lcd_clktree.clk_gate_state = 0;

	lcd_clktree.encl_top_gate = devm_clk_get(lcd_drv->dev, "encl_top_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
		LCDERR("%s: get encl_top_gate error\n", __func__);

	lcd_clktree.encl_int_gate = devm_clk_get(lcd_drv->dev, "encl_int_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
		LCDERR("%s: get encl_int_gate error\n", __func__);

	lcd_clktree.tcon_gate = devm_clk_get(lcd_drv->dev, "tcon_gate");
	if (IS_ERR_OR_NULL(lcd_clktree.tcon_gate))
		LCDERR("%s: get tcon_gate error\n", __func__);

	temp_clk = devm_clk_get(lcd_drv->dev, "fclk_div5");
	if (IS_ERR_OR_NULL(temp_clk)) {
		LCDERR("%s: clk fclk_div5\n", __func__);
		return;
	}
	lcd_clktree.tcon_clk = devm_clk_get(lcd_drv->dev, "clk_tcon");
	if (IS_ERR_OR_NULL(lcd_clktree.tcon_clk))
		LCDERR("%s: clk clk_tcon\n", __func__);
	else
		clk_set_parent(lcd_clktree.tcon_clk, temp_clk);

	LCDPR("lcd_clktree_probe\n");
}

static void lcd_clktree_remove_dft(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("lcd_clktree_remove\n");

	if (!IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.encl_top_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.encl_int_gate);
}

static void lcd_clktree_remove_axg(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("lcd_clktree_remove\n");

	if (!IS_ERR_OR_NULL(lcd_clktree.mipi_bandgap_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.mipi_bandgap_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.mipi_enable_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.mipi_enable_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
		devm_clk_put(lcd_drv->dev, lcd_clktree.dsi_meas);
	if (!IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.dsi_phy_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.dsi_host_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.gp0_pll))
		devm_clk_put(lcd_drv->dev, lcd_clktree.gp0_pll);
}

static void lcd_clktree_remove_g12a(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("lcd_clktree_remove\n");

	if (!IS_ERR_OR_NULL(lcd_clktree.dsi_host_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.dsi_host_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.dsi_phy_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.dsi_phy_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.dsi_meas))
		devm_clk_put(lcd_drv->dev, lcd_clktree.dsi_meas);
	if (!IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.encl_top_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.encl_int_gate);
}

static void lcd_clktree_remove_tl1(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_debug_print_flag)
		LCDPR("lcd_clktree_remove\n");

	if (!IS_ERR_OR_NULL(lcd_clktree.encl_top_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.encl_top_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.encl_int_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.encl_int_gate);
	if (!IS_ERR_OR_NULL(lcd_clktree.tcon_clk))
		devm_clk_put(lcd_drv->dev, lcd_clktree.tcon_clk);
	if (IS_ERR_OR_NULL(lcd_clktree.tcon_gate))
		devm_clk_put(lcd_drv->dev, lcd_clktree.tcon_gate);
}

static void lcd_clk_config_init_print_dft(void)
{
	struct lcd_clk_data_s *data = clk_conf.data;

	LCDPR("lcd clk config data init:\n"
		"pll_m_max:         %d\n"
		"pll_m_min:         %d\n"
		"pll_n_max:         %d\n"
		"pll_n_min:         %d\n"
		"pll_od_fb:         %d\n"
		"pll_frac_range:    %d\n"
		"pll_od_sel_max:    %d\n"
		"pll_ref_fmax:      %d\n"
		"pll_ref_fmin:      %d\n"
		"pll_vco_fmax:      %d\n"
		"pll_vco_fmin:      %d\n"
		"pll_out_fmax:      %d\n"
		"pll_out_fmin:      %d\n"
		"div_in_fmax:       %d\n"
		"div_out_fmax:      %d\n"
		"xd_out_fmax:       %d\n"
		"ss_level_max:      %d\n"
		"ss_freq_max:       %d\n"
		"ss_mode_max:       %d\n\n",
		data->pll_m_max, data->pll_m_min,
		data->pll_n_max, data->pll_n_min,
		data->pll_od_fb, data->pll_frac_range,
		data->pll_od_sel_max,
		data->pll_ref_fmax, data->pll_ref_fmin,
		data->pll_vco_fmax, data->pll_vco_fmin,
		data->pll_out_fmax, data->pll_out_fmin,
		data->div_in_fmax, data->div_out_fmax,
		data->xd_out_fmax, data->ss_level_max,
		data->ss_freq_max, data->ss_mode_max);
}

static void lcd_clk_config_init_print_axg(void)
{
	struct lcd_clk_data_s *data = clk_conf.data;

	LCDPR("lcd clk config data init:\n"
		"vclk_sel:          %d\n"
		"pll_m_max:         %d\n"
		"pll_m_min:         %d\n"
		"pll_n_max:         %d\n"
		"pll_n_min:         %d\n"
		"pll_od_fb:         %d\n"
		"pll_frac_range:    %d\n"
		"pll_od_sel_max:    %d\n"
		"pll_ref_fmax:      %d\n"
		"pll_ref_fmin:      %d\n"
		"pll_vco_fmax:      %d\n"
		"pll_vco_fmin:      %d\n"
		"pll_out_fmax:      %d\n"
		"pll_out_fmin:      %d\n"
		"xd_out_fmax:       %d\n\n",
		data->vclk_sel,
		data->pll_m_max, data->pll_m_min,
		data->pll_n_max, data->pll_n_min,
		data->pll_od_fb, data->pll_frac_range,
		data->pll_od_sel_max,
		data->pll_ref_fmax, data->pll_ref_fmin,
		data->pll_vco_fmax, data->pll_vco_fmin,
		data->pll_out_fmax, data->pll_out_fmin,
		data->xd_out_fmax);
}

static int lcd_clk_config_print_dft(char *buf, int offset)
{
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"lcd clk config:\n"
		"pll_mode:         %d\n"
		"pll_m:            %d\n"
		"pll_n:            %d\n"
		"pll_frac:         0x%03x\n"
		"pll_fvco:         %dkHz\n"
		"pll_od1:          %d\n"
		"pll_od2:          %d\n"
		"pll_od3:          %d\n"
		"pll_tcon_div_sel: %d\n"
		"pll_out:          %dkHz\n"
		"div_sel:          %s(index %d)\n"
		"xd:               %d\n"
		"fout:             %dkHz\n"
		"ss_level:         %d\n"
		"ss_freq:          %d\n"
		"ss_mode:          %d\n\n",
		clk_conf.pll_mode, clk_conf.pll_m, clk_conf.pll_n,
		clk_conf.pll_frac, clk_conf.pll_fvco,
		clk_conf.pll_od1_sel, clk_conf.pll_od2_sel,
		clk_conf.pll_od3_sel, clk_conf.pll_tcon_div_sel,
		clk_conf.pll_fout,
		lcd_clk_div_sel_table[clk_conf.div_sel],
		clk_conf.div_sel, clk_conf.xd,
		clk_conf.fout, clk_conf.ss_level,
		clk_conf.ss_freq, clk_conf.ss_mode);

	return len;
}

static int lcd_clk_config_print_axg(char *buf, int offset)
{
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"lcd clk config:\n"
		"pll_m:        %d\n"
		"pll_n:        %d\n"
		"pll_frac:     0x%03x\n"
		"pll_fvco:     %dkHz\n"
		"pll_od:       %d\n"
		"pll_out:      %dkHz\n"
		"xd:           %d\n"
		"fout:         %dkHz\n\n",
		clk_conf.pll_m, clk_conf.pll_n,
		clk_conf.pll_frac, clk_conf.pll_fvco,
		clk_conf.pll_od1_sel, clk_conf.pll_fout,
		clk_conf.xd, clk_conf.fout);

	return len;
}

static int lcd_clk_config_print_g12a(char *buf, int offset)
{
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	if (clk_conf.data->vclk_sel) {
		len += snprintf((buf+len), n,
			"lcd clk config:\n"
			"vclk_sel      %d\n"
			"pll_m:        %d\n"
			"pll_n:        %d\n"
			"pll_frac:     0x%03x\n"
			"pll_fvco:     %dkHz\n"
			"pll_od:       %d\n"
			"pll_out:      %dkHz\n"
			"xd:           %d\n"
			"fout:         %dkHz\n\n",
			clk_conf.data->vclk_sel,
			clk_conf.pll_m, clk_conf.pll_n,
			clk_conf.pll_frac, clk_conf.pll_fvco,
			clk_conf.pll_od1_sel, clk_conf.pll_fout,
			clk_conf.xd, clk_conf.fout);
	} else {
		len += snprintf((buf+len), n,
			"lcd clk config:\n"
			"vclk_sel        %d\n"
			"pll_m:          %d\n"
			"pll_n:          %d\n"
			"pll_frac:       0x%03x\n"
			"pll_fvco:       %dkHz\n"
			"pll_od1:        %d\n"
			"pll_od2:        %d\n"
			"pll_od3:        %d\n"
			"pll_out:        %dkHz\n"
			"div_sel:        %s(index %d)\n"
			"xd:             %d\n"
			"fout:           %dkHz\n\n",
			clk_conf.data->vclk_sel,
			clk_conf.pll_m, clk_conf.pll_n,
			clk_conf.pll_frac, clk_conf.pll_fvco,
			clk_conf.pll_od1_sel, clk_conf.pll_od2_sel,
			clk_conf.pll_od3_sel, clk_conf.pll_fout,
			lcd_clk_div_sel_table[clk_conf.div_sel],
			clk_conf.div_sel, clk_conf.xd,
			clk_conf.fout);
	}

	return len;
}

/* ****************************************************
 * lcd clk function api
 * ****************************************************
 */
void lcd_clk_generate_parameter(struct lcd_config_s *pconf)
{
	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}

	if (clk_conf.data->clk_generate_parameter)
		clk_conf.data->clk_generate_parameter(pconf);
}

int lcd_get_ss(char *buf)
{
	unsigned int temp;
	int len = 0;

	if (clk_conf.data == NULL) {
		len += sprintf(buf+len, "lcd clk config data is null\n");
		return len;
	}
	if (clk_conf.data->ss_level_max == 0) {
		len += sprintf(buf+len, "lcd spread spectrum is invalid\n");
		return len;
	}

	temp = (clk_conf.ss_level >= clk_conf.data->ss_level_max) ?
		0 : clk_conf.ss_level;
	if (clk_conf.data->ss_level_table) {
		len += sprintf(buf+len, "ss_level: %s\n",
			clk_conf.data->ss_level_table[temp]);
	}
	temp = (clk_conf.ss_freq >= clk_conf.data->ss_freq_max) ?
		0 : clk_conf.ss_freq;
	if (clk_conf.data->ss_freq_table) {
		len += sprintf(buf+len, "ss_freq: %s\n",
			clk_conf.data->ss_freq_table[temp]);
	}
	temp = (clk_conf.ss_mode >= clk_conf.data->ss_mode_max) ?
		0 : clk_conf.ss_mode;
	if (clk_conf.data->ss_mode_table) {
		len += sprintf(buf+len, "ss_mode: %s\n",
			clk_conf.data->ss_mode_table[temp]);
	}

	return len;
}

int lcd_set_ss(unsigned int level, unsigned int freq, unsigned int mode)
{
	unsigned long flags = 0;
	int ret = -1;

	spin_lock_irqsave(&lcd_clk_lock, flags);

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		goto lcd_set_ss_end;
	}
	if (level < 0xff) {
		if (level >= clk_conf.data->ss_level_max) {
			LCDERR("%s: ss_level %d is out of support (max %d)\n",
				__func__, level,
				(clk_conf.data->ss_level_max - 1));
			goto lcd_set_ss_end;
		}
	}
	if (freq < 0xff) {
		if (freq >= clk_conf.data->ss_freq_max) {
			LCDERR("%s: ss_freq %d is out of support (max %d)\n",
				__func__, freq,
				(clk_conf.data->ss_freq_max - 1));
			goto lcd_set_ss_end;
		}
	}
	if (mode < 0xff) {
		if (mode >= clk_conf.data->ss_mode_max) {
			LCDERR("%s: ss_mode %d is out of support (max %d)\n",
				__func__, mode,
				(clk_conf.data->ss_mode_max - 1));
			goto lcd_set_ss_end;
		}
	}

	if (clk_conf.data->set_ss_level) {
		if (level < 0xff) {
			clk_conf.ss_level = level;
			clk_conf.data->set_ss_level(level);
		}
	}

	ret = 0;
	if (clk_conf.data->set_ss_advance) {
		if ((freq == 0xff) && (mode == 0xff))
			goto lcd_set_ss_end;
		if (freq < 0xff)
			clk_conf.ss_freq = freq;
		if (mode < 0xff)
			clk_conf.ss_mode = mode;
		clk_conf.data->set_ss_advance(clk_conf.ss_freq,
			clk_conf.ss_mode);
	}

lcd_set_ss_end:
	spin_unlock_irqrestore(&lcd_clk_lock, flags);

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);
	return ret;
}

/* design for vlock, don't save ss_level to clk_config */
int lcd_ss_enable(unsigned int flag)
{
	unsigned int level;
	unsigned long flags = 0;
	int ret = -1;

	spin_lock_irqsave(&lcd_clk_lock, flags);

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		goto lcd_ss_enable_end;
	}

	if (clk_conf.data->set_ss_level) {
		level = flag ? clk_conf.ss_level : 0;
		clk_conf.data->set_ss_level(level);
	}

lcd_ss_enable_end:
	spin_unlock_irqrestore(&lcd_clk_lock, flags);

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);
	return ret;
}

int lcd_encl_clk_msr(void)
{
	unsigned int clk_mux;
	int encl_clk = 0;

	clk_mux = 9;
	encl_clk = meson_clk_measure(clk_mux);

	return encl_clk;
}

void lcd_pll_reset(void)
{
	struct lcd_clk_ctrl_s *table;
	int i = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&lcd_clk_lock, flags);

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		goto lcd_pll_reset_end;
	}
	if (clk_conf.data->pll_ctrl_table == NULL)
		goto lcd_pll_reset_end;

	table = clk_conf.data->pll_ctrl_table;
	while (i < LCD_CLK_CTRL_CNT_MAX) {
		if (table[i].flag == LCD_CLK_CTRL_END)
			break;
		if (table[i].flag == LCD_CLK_CTRL_RST) {
			lcd_hiu_setb(table[i].reg, 1,
				table[i].bit, table[i].len);
			udelay(10);
			lcd_hiu_setb(table[i].reg, 0,
				table[i].bit, table[i].len);
		}
		i++;
	}

lcd_pll_reset_end:
	spin_unlock_irqrestore(&lcd_clk_lock, flags);
	LCDPR("%s\n", __func__);
}

void lcd_vlock_m_update(unsigned int vlock_m)
{
	struct lcd_clk_ctrl_s *table;
	int i = 0;
	unsigned long flags = 0;

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}

	if (clk_conf.data->pll_ctrl_table == NULL)
		return;

	spin_lock_irqsave(&lcd_clk_lock, flags);
	vlock_m &= 0xff;
	if (lcd_debug_print_flag == 2)
		LCDPR("%s,vlcok_m: 0x%x,", __func__, vlock_m);

	table = clk_conf.data->pll_ctrl_table;
	while (i < LCD_CLK_CTRL_CNT_MAX) {
		if (table[i].flag == LCD_CLK_CTRL_M) {
			lcd_hiu_setb(table[i].reg, vlock_m,
				table[i].bit, table[i].len);
			break;
		}
		i++;
	}
	spin_unlock_irqrestore(&lcd_clk_lock, flags);
}

void lcd_vlock_farc_update(unsigned int vlock_farc)
{
	struct lcd_clk_ctrl_s *table;
	int i = 0;
	unsigned long flags = 0;

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}

	if (clk_conf.data->pll_ctrl_table == NULL)
		return;

	spin_lock_irqsave(&lcd_clk_lock, flags);
	vlock_farc &= 0x1ffff;
	if (lcd_debug_print_flag == 2)
		LCDPR("%s,vlock_farc: 0x%x\n", __func__, vlock_farc);

	table = clk_conf.data->pll_ctrl_table;
	while (i < LCD_CLK_CTRL_CNT_MAX) {
		if (table[i].flag == LCD_CLK_CTRL_FRAC) {
			lcd_hiu_setb(table[i].reg, vlock_farc,
				table[i].bit, table[i].len);
			break;
		}
		i++;
	}
	spin_unlock_irqrestore(&lcd_clk_lock, flags);
}

/* for frame rate change */
void lcd_clk_update(struct lcd_config_s *pconf)
{
	struct lcd_clk_ctrl_s *table;
	int i = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&lcd_clk_lock, flags);

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		goto lcd_clk_update_end;
	}

	if (clk_conf.data->pll_frac_generate)
		clk_conf.data->pll_frac_generate(pconf);

	if (clk_conf.data->pll_ctrl_table == NULL)
		goto lcd_clk_update_end;
	table = clk_conf.data->pll_ctrl_table;
	while (i < LCD_CLK_CTRL_CNT_MAX) {
		if (table[i].flag == LCD_CLK_CTRL_END)
			break;
		if (table[i].flag == LCD_CLK_CTRL_FRAC) {
			lcd_hiu_setb(table[i].reg, clk_conf.pll_frac,
				table[i].bit, table[i].len);
		}
		i++;
	}

lcd_clk_update_end:
	spin_unlock_irqrestore(&lcd_clk_lock, flags);
	LCDPR("%s\n", __func__);
}

/* for timing change */
void lcd_clk_set(struct lcd_config_s *pconf)
{
	unsigned long flags = 0;

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}

	spin_lock_irqsave(&lcd_clk_lock, flags);
	if (clk_conf.data->clk_set)
		clk_conf.data->clk_set(pconf);

	lcd_set_vclk_crt(pconf->lcd_basic.lcd_type, &clk_conf);
	mdelay(10);
	spin_unlock_irqrestore(&lcd_clk_lock, flags);

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);
}

void lcd_clk_disable(void)
{
	struct lcd_clk_ctrl_s *table;
	int i = 0;

	lcd_hiu_setb(HHI_VID_CLK_CNTL2, 0, ENCL_GATE_VCLK, 1);

	/* close vclk2_div gate: 0x104b[4:0] */
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, 0, 5);
	lcd_hiu_setb(HHI_VIID_CLK_CNTL, 0, VCLK2_EN, 1);

	if (clk_conf.data == NULL)
		return;
	if (clk_conf.data->pll_ctrl_table == NULL)
		return;
	table = clk_conf.data->pll_ctrl_table;
	while (i < LCD_CLK_CTRL_CNT_MAX) {
		if (table[i].flag == LCD_CLK_CTRL_END)
			break;
		if (table[i].flag == LCD_CLK_CTRL_EN) {
			lcd_hiu_setb(table[i].reg, 0,
				table[i].bit, table[i].len);
		} else if (table[i].flag == LCD_CLK_CTRL_RST) {
			lcd_hiu_setb(table[i].reg, 1,
				table[i].bit, table[i].len);
		}
		i++;
	}

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);
}

static void lcd_clk_gate_optional_switch(int status)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}
	if (clk_conf.data->clk_gate_optional_switch == NULL)
		return;

	if (status) {
		if (lcd_clktree.clk_gate_optional_state)
			LCDPR("clk_gate_optional is already on\n");
		else
			clk_conf.data->clk_gate_optional_switch(lcd_drv, 1);
	} else {
		if (lcd_clktree.clk_gate_optional_state == 0)
			LCDPR("clk_gate_optional is already off\n");
		else
			clk_conf.data->clk_gate_optional_switch(lcd_drv, 0);
	}
}

void lcd_clk_gate_switch(int status)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}

	if (status) {
		if (lcd_clktree.clk_gate_state) {
			LCDPR("clk_gate is already on\n");
		} else {
#ifdef CONFIG_AMLOGIC_VPU
			switch_vpu_clk_gate_vmod(VPU_VENCL, VPU_CLK_GATE_ON);
#endif
			if (clk_conf.data->clk_gate_switch)
				clk_conf.data->clk_gate_switch(lcd_drv, 1);
			lcd_clktree.clk_gate_state = 1;
		}
		lcd_clk_gate_optional_switch(1);
	} else {
		lcd_clk_gate_optional_switch(0);
		if (lcd_clktree.clk_gate_state == 0) {
			LCDPR("clk_gate is already off\n");
		} else {
			if (clk_conf.data->clk_gate_switch)
				clk_conf.data->clk_gate_switch(lcd_drv, 0);
#ifdef CONFIG_AMLOGIC_VPU
			switch_vpu_clk_gate_vmod(VPU_VENCL, VPU_CLK_GATE_OFF);
#endif
			lcd_clktree.clk_gate_state = 0;
		}
	}
}

static void lcd_clk_config_init_print(void)
{
	if (clk_conf.data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return;
	}

	if (clk_conf.data->clk_config_init_print)
		clk_conf.data->clk_config_init_print();
}

int lcd_clk_config_print(char *buf, int offset)
{
	int n, len = 0;

	if (clk_conf.data == NULL) {
		n = lcd_debug_info_len(len + offset);
		len += snprintf((buf+len), n,
			"%s: clk config data is null\n",
			__func__);
		return len;
	}

	if (clk_conf.data->clk_config_print)
		len = clk_conf.data->clk_config_print(buf, offset);

	return len;
}

/* ****************************************************
 * lcd clk config
 * ****************************************************
 */
static struct lcd_clk_data_s lcd_clk_data_gxl = {
	.pll_od_fb = PLL_OD_FB_GXL,
	.pll_m_max = PLL_M_MAX_GXL,
	.pll_m_min = PLL_M_MIN_GXL,
	.pll_n_max = PLL_N_MAX_GXL,
	.pll_n_min = PLL_N_MIN_GXL,
	.pll_frac_range = PLL_FRAC_RANGE_GXL,
	.pll_od_sel_max = PLL_OD_SEL_MAX_GXL,
	.pll_ref_fmax = PLL_FREF_MAX_GXL,
	.pll_ref_fmin = PLL_FREF_MIN_GXL,
	.pll_vco_fmax = PLL_VCO_MAX_GXL,
	.pll_vco_fmin = PLL_VCO_MIN_GXL,
	.pll_out_fmax = CLK_DIV_IN_MAX_GXL,
	.pll_out_fmin = PLL_VCO_MIN_GXL / 16,
	.div_in_fmax = CLK_DIV_IN_MAX_GXL,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_GXL,
	.xd_out_fmax = ENCL_CLK_IN_MAX_GXL,

	.clk_path_valid = 0,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_txl,

	.ss_level_max = 0,
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = NULL,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_txl,
	.pll_frac_generate = lcd_pll_frac_generate_txl,
	.set_ss_level = NULL,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_txl,
	.clk_gate_switch = lcd_clk_gate_switch_dft,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_dft,
	.clktree_remove = lcd_clktree_remove_dft,
	.clk_config_init_print = lcd_clk_config_init_print_dft,
	.clk_config_print = lcd_clk_config_print_dft,
};

static struct lcd_clk_data_s lcd_clk_data_txl = {
	.pll_od_fb = PLL_OD_FB_TXL,
	.pll_m_max = PLL_M_MAX_TXL,
	.pll_m_min = PLL_M_MIN_TXL,
	.pll_n_max = PLL_N_MAX_TXL,
	.pll_n_min = PLL_N_MIN_TXL,
	.pll_frac_range = PLL_FRAC_RANGE_TXL,
	.pll_od_sel_max = PLL_OD_SEL_MAX_TXL,
	.pll_ref_fmax = PLL_FREF_MAX_TXL,
	.pll_ref_fmin = PLL_FREF_MIN_TXL,
	.pll_vco_fmax = PLL_VCO_MAX_TXL,
	.pll_vco_fmin = PLL_VCO_MIN_TXL,
	.pll_out_fmax = CLK_DIV_IN_MAX_TXL,
	.pll_out_fmin = PLL_VCO_MIN_TXL / 16,
	.div_in_fmax = CLK_DIV_IN_MAX_TXL,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_TXL,
	.xd_out_fmax = ENCL_CLK_IN_MAX_TXL,

	.clk_path_valid = 0,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_txl,

	.ss_level_max = sizeof(lcd_ss_level_table_txl) / sizeof(char *),
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = lcd_ss_level_table_txl,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_txl,
	.pll_frac_generate = lcd_pll_frac_generate_txl,
	.set_ss_level = lcd_set_pll_ss_level_txl,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_txl,
	.clk_gate_switch = lcd_clk_gate_switch_dft,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_dft,
	.clktree_remove = lcd_clktree_remove_dft,
	.clk_config_init_print = lcd_clk_config_init_print_dft,
	.clk_config_print = lcd_clk_config_print_dft,
};

static struct lcd_clk_data_s lcd_clk_data_txlx = {
	.pll_od_fb = PLL_OD_FB_TXLX,
	.pll_m_max = PLL_M_MAX_TXLX,
	.pll_m_min = PLL_M_MIN_TXLX,
	.pll_n_max = PLL_N_MAX_TXLX,
	.pll_n_min = PLL_N_MIN_TXLX,
	.pll_frac_range = PLL_FRAC_RANGE_TXLX,
	.pll_od_sel_max = PLL_OD_SEL_MAX_TXLX,
	.pll_ref_fmax = PLL_FREF_MAX_TXLX,
	.pll_ref_fmin = PLL_FREF_MIN_TXLX,
	.pll_vco_fmax = PLL_VCO_MAX_TXLX,
	.pll_vco_fmin = PLL_VCO_MIN_TXLX,
	.pll_out_fmax = CLK_DIV_IN_MAX_TXLX,
	.pll_out_fmin = PLL_VCO_MIN_TXLX / 16,
	.div_in_fmax = CLK_DIV_IN_MAX_TXLX,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_TXLX,
	.xd_out_fmax = ENCL_CLK_IN_MAX_TXLX,

	.clk_path_valid = 0,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_txl,

	.ss_level_max = sizeof(lcd_ss_level_table_txlx) / sizeof(char *),
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = lcd_ss_level_table_txlx,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_txl,
	.pll_frac_generate = lcd_pll_frac_generate_txl,
	.set_ss_level = lcd_set_pll_ss_level_txlx,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_txlx,
	.clk_gate_switch = lcd_clk_gate_switch_dft,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_dft,
	.clktree_remove = lcd_clktree_remove_dft,
	.clk_config_init_print = lcd_clk_config_init_print_dft,
	.clk_config_print = lcd_clk_config_print_dft,
};

static struct lcd_clk_data_s lcd_clk_data_axg = {
	.pll_od_fb = PLL_OD_FB_AXG,
	.pll_m_max = PLL_M_MAX_AXG,
	.pll_m_min = PLL_M_MIN_AXG,
	.pll_n_max = PLL_N_MAX_AXG,
	.pll_n_min = PLL_N_MIN_AXG,
	.pll_frac_range = PLL_FRAC_RANGE_AXG,
	.pll_od_sel_max = PLL_OD_SEL_MAX_AXG,
	.pll_ref_fmax = PLL_FREF_MAX_AXG,
	.pll_ref_fmin = PLL_FREF_MIN_AXG,
	.pll_vco_fmax = PLL_VCO_MAX_AXG,
	.pll_vco_fmin = PLL_VCO_MIN_AXG,
	.pll_out_fmax = CRT_VID_CLK_IN_MAX_AXG,
	.pll_out_fmin = PLL_VCO_MIN_AXG / 4,
	.div_in_fmax = 0,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_AXG,
	.xd_out_fmax = ENCL_CLK_IN_MAX_AXG,

	.clk_path_valid = 0,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_axg,

	.ss_level_max = 0,
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = NULL,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_axg,
	.pll_frac_generate = lcd_pll_frac_generate_axg,
	.set_ss_level = NULL,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_axg,
	.clk_gate_switch = lcd_clk_gate_switch_axg,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_axg,
	.clktree_remove = lcd_clktree_remove_axg,
	.clk_config_init_print = lcd_clk_config_init_print_axg,
	.clk_config_print = lcd_clk_config_print_axg,
};

static struct lcd_clk_data_s lcd_clk_data_g12a_path0 = {
	.pll_od_fb = PLL_OD_FB_HPLL_G12A,
	.pll_m_max = PLL_M_MAX_G12A,
	.pll_m_min = PLL_M_MIN_G12A,
	.pll_n_max = PLL_N_MAX_G12A,
	.pll_n_min = PLL_N_MIN_G12A,
	.pll_frac_range = PLL_FRAC_RANGE_HPLL_G12A,
	.pll_od_sel_max = PLL_OD_SEL_MAX_HPLL_G12A,
	.pll_ref_fmax = PLL_FREF_MAX_G12A,
	.pll_ref_fmin = PLL_FREF_MIN_G12A,
	.pll_vco_fmax = PLL_VCO_MAX_HPLL_G12A,
	.pll_vco_fmin = PLL_VCO_MIN_HPLL_G12A,
	.pll_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.pll_out_fmin = PLL_VCO_MIN_HPLL_G12A / 16,
	.div_in_fmax = 0,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.xd_out_fmax = ENCL_CLK_IN_MAX_G12A,

	.clk_path_valid = 1,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_g12a_path0,

	.ss_level_max = 0,
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = NULL,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_hpll_g12a,
	.pll_frac_generate = lcd_pll_frac_generate_txl,
	.set_ss_level = NULL,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_g12a_path0,
	.clk_gate_switch = lcd_clk_gate_switch_g12a,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_g12a,
	.clktree_remove = lcd_clktree_remove_g12a,
	.clk_config_init_print = lcd_clk_config_init_print_axg,
	.clk_config_print = lcd_clk_config_print_g12a,
};

static struct lcd_clk_data_s lcd_clk_data_g12a_path1 = {
	.pll_od_fb = PLL_OD_FB_GP0_G12A,
	.pll_m_max = PLL_M_MAX_G12A,
	.pll_m_min = PLL_M_MIN_G12A,
	.pll_n_max = PLL_N_MAX_G12A,
	.pll_n_min = PLL_N_MIN_G12A,
	.pll_frac_range = PLL_FRAC_RANGE_GP0_G12A,
	.pll_od_sel_max = PLL_OD_SEL_MAX_GP0_G12A,
	.pll_ref_fmax = PLL_FREF_MAX_G12A,
	.pll_ref_fmin = PLL_FREF_MIN_G12A,
	.pll_vco_fmax = PLL_VCO_MAX_GP0_G12A,
	.pll_vco_fmin = PLL_VCO_MIN_GP0_G12A,
	.pll_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.pll_out_fmin = PLL_VCO_MIN_GP0_G12A / 16,
	.div_in_fmax = 0,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.xd_out_fmax = ENCL_CLK_IN_MAX_G12A,

	.clk_path_valid = 1,
	.vclk_sel = 1,
	.pll_ctrl_table = pll_ctrl_table_g12a_path1,

	.ss_level_max = 0,
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = NULL,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_axg,
	.pll_frac_generate = lcd_pll_frac_generate_axg,
	.set_ss_level = NULL,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_g12a_path1,
	.clk_gate_switch = lcd_clk_gate_switch_g12a,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_g12a,
	.clktree_remove = lcd_clktree_remove_g12a,
	.clk_config_init_print = lcd_clk_config_init_print_axg,
	.clk_config_print = lcd_clk_config_print_g12a,
};

static struct lcd_clk_data_s lcd_clk_data_g12b_path0 = {
	.pll_od_fb = PLL_OD_FB_HPLL_G12A,
	.pll_m_max = PLL_M_MAX_G12A,
	.pll_m_min = PLL_M_MIN_G12A,
	.pll_n_max = PLL_N_MAX_G12A,
	.pll_n_min = PLL_N_MIN_G12A,
	.pll_frac_range = PLL_FRAC_RANGE_HPLL_G12A,
	.pll_od_sel_max = PLL_OD_SEL_MAX_HPLL_G12A,
	.pll_ref_fmax = PLL_FREF_MAX_G12A,
	.pll_ref_fmin = PLL_FREF_MIN_G12A,
	.pll_vco_fmax = PLL_VCO_MAX_HPLL_G12A,
	.pll_vco_fmin = PLL_VCO_MIN_HPLL_G12A,
	.pll_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.pll_out_fmin = PLL_VCO_MIN_HPLL_G12A / 16,
	.div_in_fmax = 0,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.xd_out_fmax = ENCL_CLK_IN_MAX_G12A,

	.clk_path_valid = 1,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_g12a_path0,

	.ss_level_max = 0,
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = NULL,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_hpll_g12a,
	.pll_frac_generate = lcd_pll_frac_generate_txl,
	.set_ss_level = NULL,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_g12b_path0,
	.clk_gate_switch = lcd_clk_gate_switch_g12a,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_g12a,
	.clktree_remove = lcd_clktree_remove_g12a,
	.clk_config_init_print = lcd_clk_config_init_print_axg,
	.clk_config_print = lcd_clk_config_print_g12a,
};

static struct lcd_clk_data_s lcd_clk_data_g12b_path1 = {
	.pll_od_fb = PLL_OD_FB_GP0_G12A,
	.pll_m_max = PLL_M_MAX_G12A,
	.pll_m_min = PLL_M_MIN_G12A,
	.pll_n_max = PLL_N_MAX_G12A,
	.pll_n_min = PLL_N_MIN_G12A,
	.pll_frac_range = PLL_FRAC_RANGE_GP0_G12A,
	.pll_od_sel_max = PLL_OD_SEL_MAX_GP0_G12A,
	.pll_ref_fmax = PLL_FREF_MAX_G12A,
	.pll_ref_fmin = PLL_FREF_MIN_G12A,
	.pll_vco_fmax = PLL_VCO_MAX_GP0_G12A,
	.pll_vco_fmin = PLL_VCO_MIN_GP0_G12A,
	.pll_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.pll_out_fmin = PLL_VCO_MIN_GP0_G12A / 16,
	.div_in_fmax = 0,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_G12A,
	.xd_out_fmax = ENCL_CLK_IN_MAX_G12A,

	.clk_path_valid = 1,
	.vclk_sel = 1,
	.pll_ctrl_table = pll_ctrl_table_g12a_path1,

	.ss_level_max = 0,
	.ss_freq_max = 0,
	.ss_mode_max = 0,
	.ss_level_table = NULL,
	.ss_freq_table = NULL,
	.ss_mode_table = NULL,

	.clk_generate_parameter = lcd_clk_generate_axg,
	.pll_frac_generate = lcd_pll_frac_generate_axg,
	.set_ss_level = NULL,
	.set_ss_advance = NULL,
	.clk_set = lcd_clk_set_g12b_path1,
	.clk_gate_switch = lcd_clk_gate_switch_g12a,
	.clk_gate_optional_switch = NULL,
	.clktree_probe = lcd_clktree_probe_g12a,
	.clktree_remove = lcd_clktree_remove_g12a,
	.clk_config_init_print = lcd_clk_config_init_print_axg,
	.clk_config_print = lcd_clk_config_print_g12a,
};

static struct lcd_clk_data_s lcd_clk_data_tl1 = {
	.pll_od_fb = PLL_OD_FB_TL1,
	.pll_m_max = PLL_M_MAX_TL1,
	.pll_m_min = PLL_M_MIN_TL1,
	.pll_n_max = PLL_N_MAX_TL1,
	.pll_n_min = PLL_N_MIN_TL1,
	.pll_frac_range = PLL_FRAC_RANGE_TL1,
	.pll_od_sel_max = PLL_OD_SEL_MAX_TL1,
	.pll_ref_fmax = PLL_FREF_MAX_TL1,
	.pll_ref_fmin = PLL_FREF_MIN_TL1,
	.pll_vco_fmax = PLL_VCO_MAX_TL1,
	.pll_vco_fmin = PLL_VCO_MIN_TL1,
	.pll_out_fmax = CLK_DIV_IN_MAX_TL1,
	.pll_out_fmin = PLL_VCO_MIN_TL1 / 16,
	.div_in_fmax = CLK_DIV_IN_MAX_TL1,
	.div_out_fmax = CRT_VID_CLK_IN_MAX_TL1,
	.xd_out_fmax = ENCL_CLK_IN_MAX_TL1,

	.clk_path_valid = 0,
	.vclk_sel = 0,
	.pll_ctrl_table = pll_ctrl_table_tl1,

	.ss_level_max = sizeof(lcd_ss_level_table_tl1) / sizeof(char *),
	.ss_freq_max = sizeof(lcd_ss_freq_table_tl1) / sizeof(char *),
	.ss_mode_max = sizeof(lcd_ss_mode_table_tl1) / sizeof(char *),
	.ss_level_table = lcd_ss_level_table_tl1,
	.ss_freq_table = lcd_ss_freq_table_tl1,
	.ss_mode_table = lcd_ss_mode_table_tl1,

	.clk_generate_parameter = lcd_clk_generate_txl,
	.pll_frac_generate = lcd_pll_frac_generate_txl,
	.set_ss_level = lcd_set_pll_ss_level_tl1,
	.set_ss_advance = lcd_set_pll_ss_advance_tl1,
	.clk_set = lcd_clk_set_tl1,
	.clk_gate_switch = lcd_clk_gate_switch_dft,
	.clk_gate_optional_switch = lcd_clk_gate_optional_switch_tl1,
	.clktree_probe = lcd_clktree_probe_tl1,
	.clktree_remove = lcd_clktree_remove_tl1,
	.clk_config_init_print = lcd_clk_config_init_print_dft,
	.clk_config_print = lcd_clk_config_print_dft,
};

static void lcd_clk_config_chip_init(struct lcd_clk_config_s *cConf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
		cConf->data = &lcd_clk_data_gxl;
		break;
	case LCD_CHIP_TXL:
		cConf->data = &lcd_clk_data_txl;
		break;
	case LCD_CHIP_TXLX:
		cConf->data = &lcd_clk_data_txlx;
		break;
	case LCD_CHIP_AXG:
		cConf->data = &lcd_clk_data_axg;
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_clk_path)
			cConf->data = &lcd_clk_data_g12a_path1;
		else
			cConf->data = &lcd_clk_data_g12a_path0;
		break;
	case LCD_CHIP_G12B:
		if (lcd_drv->lcd_clk_path)
			cConf->data = &lcd_clk_data_g12b_path1;
		else
			cConf->data = &lcd_clk_data_g12b_path0;
		break;
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		cConf->data = &lcd_clk_data_tl1;
		break;
	default:
		LCDPR("%s: invalid chip type\n", __func__);
		break;
	}

	if (cConf->data)
		cConf->pll_od_fb = cConf->data->pll_od_fb;
	if (lcd_debug_print_flag > 0)
		lcd_clk_config_init_print();
}

int lcd_clk_path_change(int sel)
{
	struct lcd_clk_config_s *cConf = get_lcd_clk_config();
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (cConf->data == NULL) {
		LCDERR("%s: clk config data is null\n", __func__);
		return -1;
	}

	if (cConf->data->clk_path_valid == 0) {
		LCDPR("%s: current chip not support\n", __func__);
		return -1;
	}

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (sel)
			cConf->data = &lcd_clk_data_g12a_path1;
		else
			cConf->data = &lcd_clk_data_g12a_path0;
		cConf->pll_od_fb = cConf->data->pll_od_fb;

		if (lcd_debug_print_flag > 0)
			lcd_clk_config_init_print();
		break;
	default:
		LCDERR("%s: current chip not support\n", __func__);
		return -1;
	}

	return 0;
}

void lcd_clk_config_probe(void)
{
	spin_lock_init(&lcd_clk_lock);
	lcd_clk_config_chip_init(&clk_conf);
	if (clk_conf.data == NULL)
		return;

	if (clk_conf.data->clktree_probe)
		clk_conf.data->clktree_probe();
}

void lcd_clk_config_remove(void)
{
	if (clk_conf.data == NULL)
		return;

	if (clk_conf.data->clktree_remove)
		clk_conf.data->clktree_remove();
}

