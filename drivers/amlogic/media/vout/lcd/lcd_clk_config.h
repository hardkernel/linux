/*
 * drivers/amlogic/media/vout/lcd/lcd_clk_config.h
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

#ifndef _LCD_CLK_CONFIG_H
#define _LCD_CLK_CONFIG_H

#include <linux/types.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>

/* **********************************
 * clk config
 * **********************************
 */
#define PLL_RETRY_MAX		20
#define LCD_CLK_CTRL_EN      0
#define LCD_CLK_CTRL_RST     1
#define LCD_CLK_CTRL_M	     2
#define LCD_CLK_CTRL_FRAC    3
#define LCD_CLK_CTRL_END     0xffff

#define LCD_CLK_REG_END      0xffff
#define LCD_CLK_CTRL_CNT_MAX 10
struct lcd_clk_ctrl_s {
	unsigned int flag;
	unsigned int reg;
	unsigned int bit;
	unsigned int len;
};

struct lcd_clk_data_s {
	/* clk path node parameters */
	unsigned int pll_od_fb;
	unsigned int pll_m_max;
	unsigned int pll_m_min;
	unsigned int pll_n_max;
	unsigned int pll_n_min;
	unsigned int pll_frac_range;
	unsigned int pll_od_sel_max;
	unsigned int pll_ref_fmax;
	unsigned int pll_ref_fmin;
	unsigned int pll_vco_fmax;
	unsigned int pll_vco_fmin;
	unsigned int pll_out_fmax;
	unsigned int pll_out_fmin;
	unsigned int div_in_fmax;
	unsigned int div_out_fmax;
	unsigned int xd_out_fmax;

	unsigned char clk_path_valid;
	unsigned char vclk_sel;
	struct lcd_clk_ctrl_s *pll_ctrl_table;

	unsigned int ss_level_max;
	unsigned int ss_freq_max;
	unsigned int ss_mode_max;
	char **ss_level_table;
	char **ss_freq_table;
	char **ss_mode_table;

	void (*clk_generate_parameter)(struct lcd_config_s *pconf);
	void (*pll_frac_generate)(struct lcd_config_s *pconf);
	void (*set_ss_level)(unsigned int level);
	void (*set_ss_advance)(unsigned int freq, unsigned int mode);
	void (*clk_set)(struct lcd_config_s *pconf);
	void (*clk_gate_switch)(struct aml_lcd_drv_s *lcd_drv, int status);
	void (*clk_gate_optional_switch)(struct aml_lcd_drv_s *lcd_drv,
			int status);
	void (*clktree_probe)(void);
	void (*clktree_remove)(void);
	void (*clk_config_init_print)(void);
	int (*clk_config_print)(char *buf, int offset);
};

struct lcd_clk_config_s { /* unit: kHz */
	/* IN-OUT parameters */
	unsigned int fin;
	unsigned int fout;

	/* pll parameters */
	unsigned int pll_mode; /* txl */
	unsigned int pll_od_fb;
	unsigned int pll_m;
	unsigned int pll_n;
	unsigned int pll_fvco;
	unsigned int pll_od1_sel;
	unsigned int pll_od2_sel;
	unsigned int pll_od3_sel;
	unsigned int pll_tcon_div_sel;
	unsigned int pll_level;
	unsigned int pll_frac;
	unsigned int pll_fout;
	unsigned int ss_level;
	unsigned int ss_freq;
	unsigned int ss_mode;
	unsigned int div_sel;
	unsigned int xd;
	unsigned int div_sel_max;
	unsigned int xd_max;
	unsigned int err_fmin;

	struct lcd_clk_data_s *data;
};

struct lcd_clktree_s {
	unsigned char clk_gate_state;
	unsigned char clk_gate_optional_state;

	struct clk *encl_top_gate;
	struct clk *encl_int_gate;

	struct clk *dsi_host_gate;
	struct clk *dsi_phy_gate;
	struct clk *dsi_meas;
	struct clk *mipi_enable_gate;
	struct clk *mipi_bandgap_gate;
	struct clk *gp0_pll;
	struct clk *tcon_gate;
	struct clk *tcon_clk;
};

/* ******** api ******** */
extern int meson_clk_measure(unsigned int clk_mux);
extern int lcd_debug_info_len(int num);

extern struct lcd_clk_config_s *get_lcd_clk_config(void);
extern int lcd_clk_config_print(char *buf, int offset);
extern int lcd_encl_clk_msr(void);
extern void lcd_pll_reset(void);
extern int lcd_get_ss(char *buf);
extern int lcd_set_ss(unsigned int level, unsigned int freq, unsigned int mode);
extern void lcd_clk_update(struct lcd_config_s *pconf);
extern void lcd_clk_set(struct lcd_config_s *pconf);
extern void lcd_clk_disable(void);
extern void lcd_clk_generate_parameter(struct lcd_config_s *pconf);
extern void lcd_clk_gate_switch(int status);

extern int lcd_clk_path_change(int sel);
extern void lcd_clk_config_probe(void);
extern void lcd_clk_config_remove(void);

#endif
