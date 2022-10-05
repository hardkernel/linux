/*
 * drivers/amlogic/media/vout/lcd/lcd_common.h
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

#ifndef __AML_LCD_COMMON_H__
#define __AML_LCD_COMMON_H__
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include "lcd_clk_config.h"

/* 20170505: add a113 support to linux4.9 */
/* 20170905: fix coverity errors */
/* 20180122: support txlx, optimize lcd noitfier event */
/* 20180226: g12a support */
/* 20180425: tvconfig suuport */
/* 20180620: fix coverity errors */
/* 20180626: txl suuport */
/* 20180718: mute: wait vsync for display shadow */
/* 20180827: add pinmux off support */
/* 20180928: tl1 support, optimize clk config */
/* 20181012: tl1 support tcon */
/* 20181212: tl1 update p2p config and pll setting */
/* 20181225: update phy config */
/* 20190108: tl1 support tablet mode */
/* 20190115: tl1 tcon all interface support */
/* 20190225: optimize unifykey read flow to avoid crash */
/* 20190308: add more panel clk_ss_level step for tl1 */
/* 20190520: add vbyone hw filter user define support */
/* 20190911: add lcd_init_level for tl1 */
/* 20191025: tcon chpi phy setting update */
#define LCD_DRV_VERSION    "20191025"

#define VPP_OUT_SATURATE            (1 << 0)

extern struct mutex lcd_vout_mutex;
extern unsigned char lcd_resume_flag;
extern int lcd_vout_serve_bypass;

/* lcd common */
extern int lcd_type_str_to_type(const char *str);
extern char *lcd_type_type_to_str(int type);
extern unsigned char lcd_mode_str_to_mode(const char *str);
extern char *lcd_mode_mode_to_str(int mode);

extern void lcd_cpu_gpio_probe(unsigned int index);
extern void lcd_cpu_gpio_set(unsigned int index, int value);
extern unsigned int lcd_cpu_gpio_get(unsigned int index);
extern void lcd_ttl_pinmux_set(int status);
extern void lcd_vbyone_pinmux_set(int status);
extern void lcd_tcon_pinmux_set(int status);

extern int lcd_power_load_from_dts(struct lcd_config_s *pconf,
		struct device_node *child);
extern int lcd_power_load_from_unifykey(struct lcd_config_s *pconf,
		unsigned char *buf, int key_len, int len);
extern int lcd_vlock_param_load_from_dts(struct lcd_config_s *pconf,
		struct device_node *child);
extern int lcd_vlock_param_load_from_unifykey(struct lcd_config_s *pconf,
		unsigned char *buf);

extern void lcd_optical_vinfo_update(void);
extern void lcd_timing_init_config(struct lcd_config_s *pconf);
extern int lcd_vmode_change(struct lcd_config_s *pconf);
extern void lcd_clk_change(struct lcd_config_s *pconf);
extern void lcd_venc_change(struct lcd_config_s *pconf);
extern void lcd_if_enable_retry(struct lcd_config_s *pconf);
extern void lcd_vout_notify_mode_change_pre(void);
extern void lcd_vout_notify_mode_change(void);

/* lcd phy */
extern void lcd_lvds_phy_set(struct lcd_config_s *pconf, int status);
extern void lcd_vbyone_phy_set(struct lcd_config_s *pconf, int status);
extern void lcd_mlvds_phy_set(struct lcd_config_s *pconf, int status);
extern void lcd_p2p_phy_set(struct lcd_config_s *pconf, int status);
extern void lcd_mipi_phy_set(struct lcd_config_s *pconf, int status);
int lcd_phy_probe(void);
void lcd_phy_tcon_chpi_bbc_init_tl1(int delay);

/* lcd tcon */
extern unsigned int lcd_tcon_reg_read(unsigned int addr);
extern void lcd_tcon_reg_write(unsigned int addr, unsigned int val);
extern void lcd_tcon_reg_table_print(void);
extern void lcd_tcon_reg_readback_print(void);
extern int lcd_tcon_info_print(char *buf, int offset);
extern int lcd_tcon_od_set(int flag);
extern int lcd_tcon_od_get(void);
extern int lcd_tcon_reg_table_size_get(void);
extern unsigned char *lcd_tcon_reg_table_get(void);

extern int lcd_tcon_core_reg_get(unsigned char *buf, unsigned int size);
extern void lcd_tcon_core_reg_update(void);
extern int lcd_tcon_enable(struct lcd_config_s *pconf);
extern void lcd_tcon_disable(void);
extern int lcd_tcon_probe(struct aml_lcd_drv_s *lcd_drv);

/* lcd debug */
extern int lcd_debug_info_len(int num);
extern void lcd_debug_test(unsigned int num);
extern void lcd_mute_setting(unsigned char flag);
extern int lcd_debug_probe(void);
extern int lcd_debug_remove(void);

/* lcd driver */
#ifdef CONFIG_AMLOGIC_LCD_TV
extern void lcd_tv_vout_server_init(void);
extern void lcd_tv_vout_server_remove(void);
extern void lcd_vbyone_interrupt_enable(int flag);
extern void lcd_tv_clk_config_change(struct lcd_config_s *pconf);
extern void lcd_tv_clk_update(struct lcd_config_s *pconf);
extern int lcd_tv_probe(struct device *dev);
extern int lcd_tv_remove(struct device *dev);
#endif
#ifdef CONFIG_AMLOGIC_LCD_TABLET
extern int lcd_mipi_test_read(struct dsi_read_s *dread);
extern void lcd_tablet_vout_server_init(void);
extern void lcd_tablet_vout_server_remove(void);
extern void lcd_tablet_clk_config_change(struct lcd_config_s *pconf);
extern void lcd_tablet_clk_update(struct lcd_config_s *pconf);
extern int lcd_tablet_probe(struct device *dev);
extern int lcd_tablet_remove(struct device *dev);
#endif

#endif
