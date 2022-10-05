/*
 * drivers/amlogic/media/vout/lcd/lcd_debug.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/amlogic/media/vout/lcd/aml_bl.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/media/vout/lcd/lcd_notify.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#include "lcd_reg.h"
#include "lcd_common.h"
#ifdef CONFIG_AMLOGIC_LCD_TABLET
#include <linux/amlogic/media/vout/lcd/lcd_mipi.h>
#include "lcd_tablet/mipi_dsi_util.h"
#endif
#include "lcd_debug.h"

static struct lcd_debug_info_reg_s *lcd_debug_info_reg;
static struct lcd_debug_info_if_s *lcd_debug_info_if;

#define PR_BUF_MAX          4096

static void lcd_debug_parse_param(char *buf_orig, char **parm)
{
	char *ps, *token;
	char str[3] = {' ', '\n', '\0'};
	unsigned int n = 0;

	ps = buf_orig;
	while (1) {
		token = strsep(&ps, str);
		if (token == NULL)
			break;
		if (*token == '\0')
			continue;
		parm[n++] = token;
	}
}

static void lcd_debug_info_print(char *print_buf)
{
	char *ps, *token;
	char str[3] = {'\n', '\0'};

	ps = print_buf;
	while (1) {
		token = strsep(&ps, str);
		if (token == NULL)
			break;
		if (*token == '\0') {
			pr_info("\n");
			continue;
		}
		pr_info("%s\n", token);
	}
}

int lcd_debug_info_len(int num)
{
	int ret = 0;

	if (num >= (PR_BUF_MAX - 1)) {
		pr_info("%s: string length %d is out of support\n",
			__func__, num);
		return 0;
	}

	ret = PR_BUF_MAX - 1 - num;
	return ret;
}

static const char *lcd_common_usage_str = {
"Usage:\n"
"    echo <0|1> > enable ; 0=disable lcd; 1=enable lcd\n"
"\n"
"    echo type <adj_type> > frame_rate ; set lcd frame rate adjust type\n"
"    echo set <frame_rate> > frame_rate ; set lcd frame rate(unit in 1/100Hz)\n"
"    cat frame_rate ; read current lcd frame rate\n"
"\n"
"    echo <num> > test ; show lcd bist pattern(1~7), 0=disable bist\n"
"\n"
"    echo level <val> > ss ; set lcd clk spread spectrum level\n"
"    echo freq <val> > ss ; set lcd clk spread spectrum freq\n"
"    echo mode <val> > ss ; set lcd clk spread spectrum mode\n"
"    cat ss ; show lcd clk spread spectrum information\n"
"\n"
"    echo w<v|h|c|p|mh|mp|t> <reg> <data> > reg ; write data to vcbus|hiu|cbus|periphs|mipi host|mipi phy reg\n"
"    echo r<v|h|c|p|mh|mp|t> <reg> > reg ; read vcbus|hiu|cbus|periphs|mipi host|mipi phy reg\n"
"    echo d<v|h|c|p|mh|mp|t> <reg> <num> > reg ; dump vcbus|hiu|cbus|periphs|mipi host|mipi phy regs\n"
"\n"
"    echo <0|1> > print ; 0=disable debug print; 1=enable debug print\n"
"    cat print ; read current debug print flag\n"
"\n"
"    echo <cmd> > dump ; change dump info\n"
"    cat dump ; read specified info\n"
"data format:\n"
"    <cmd>   : info, interface, reg, reg2, power, hdr\n"
"\n"
"    echo <cmd> ... > debug ; lcd common debug, use 'cat debug' for help\n"
"    cat debug ; print help information for debug command\n"
"\n"
"    echo <0|1> > power; lcd power control: 0=lcd power off, 1=lcd power on\n"
"    echo <on|off> <step_num> <delay> > power_step ; set power on/off step delay(unit: ms)\n"
"    cat power ; print lcd power on/off step\n"
"\n"
"    cat key_valid ; print lcd_key_valid setting\n"
"    cat config_load ; print lcd_config load_id(0=dts, 1=unifykey)\n"
};

static const char *lcd_debug_usage_str = {
"Usage:\n"
"    echo clk <freq> > debug ; set lcd pixel clock, unit in Hz\n"
"    echo bit <lcd_bits> > debug ; set lcd bits\n"
"    echo basic <h_active> <v_active> <h_period> <v_period> <lcd_bits> > debug ; set lcd basic config\n"
"    echo sync <hs_width> <hs_bp> <hs_pol> <vs_width> <vs_bp> <vs_pol> > debug ; set lcd sync timing\n"
"data format:\n"
"    <xx_pol>       : 0=negative, 1=positive\n"
"\n"
"    echo info > debug ; show lcd information\n"
"    echo reg > debug ; show lcd registers\n"
"    echo dump > debug ; show lcd information & registers\n"
"    echo dith <dither_en> <rounding_en> <dither_md>  > debug ; set vpu_vencl_dith_ctrl\n"
"    echo key > debug ; show lcd_key_valid config, and lcd unifykey raw data\n"
"\n"
"    echo reset > debug; reset lcd driver\n"
"    echo power <0|1> > debug ; lcd power control: 0=power off, 1=power on\n"
};

static const char *lcd_debug_change_usage_str = {
"Usage:\n"
"    echo clk <freq> > change ; change lcd pixel clock, unit in Hz\n"
"    echo bit <lcd_bits> > change ; change lcd bits\n"
"    echo basic <h_active> <v_active> <h_period> <v_period> <lcd_bits> > change ; change lcd basic config\n"
"    echo sync <hs_width> <hs_bp> <hs_pol> <vs_width> <vs_bp> <vs_pol> > change ; change lcd sync timing\n"
"data format:\n"
"    <xx_pol>       : 0=negative, 1=positive\n"
"\n"
"    echo set > change; apply lcd config changing\n"
};

static ssize_t lcd_debug_common_help(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_common_usage_str);
}

static ssize_t lcd_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_debug_usage_str);
}

static int lcd_cpu_gpio_register_print(struct lcd_config_s *pconf,
		char *buf, int offset)
{
	int i, n, len = 0;
	struct lcd_cpu_gpio_s *cpu_gpio;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "cpu_gpio register:\n");

	i = 0;
	while (i < LCD_CPU_GPIO_NUM_MAX) {
		cpu_gpio = &pconf->lcd_power->cpu_gpio[i];
		if (cpu_gpio->probe_flag == 0) {
			i++;
			continue;
		}

		if (cpu_gpio->register_flag) {
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n,
				"%d: name=%s, gpio=%p\n",
				i, cpu_gpio->name, cpu_gpio->gpio);
		} else {
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n,
				"%d: name=%s, no registered\n",
				i, cpu_gpio->name);
		}
		i++;
	}

	return len;
}

static int lcd_power_step_print(struct lcd_config_s *pconf, int status,
		char *buf, int offset)
{
	int i, n, len = 0;
	struct lcd_power_step_s *power_step;

	n = lcd_debug_info_len(len + offset);
	if (status)
		len += snprintf((buf+len), n, "power on step:\n");
	else
		len += snprintf((buf+len), n, "power off step:\n");

	i = 0;
	while (i < LCD_PWR_STEP_MAX) {
		if (status)
			power_step = &pconf->lcd_power->power_on_step[i];
		else
			power_step = &pconf->lcd_power->power_off_step[i];

		if (power_step->type >= LCD_POWER_TYPE_MAX)
			break;
		switch (power_step->type) {
		case LCD_POWER_TYPE_CPU:
		case LCD_POWER_TYPE_PMU:
		case LCD_POWER_TYPE_WAIT_GPIO:
		case LCD_POWER_TYPE_CLK_SS:
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n,
				"%d: type=%d, index=%d, value=%d, delay=%d\n",
				i, power_step->type, power_step->index,
				power_step->value, power_step->delay);
			break;
		case LCD_POWER_TYPE_EXTERN:
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n,
				"%d: type=%d, index=%d, delay=%d\n",
				i, power_step->type, power_step->index,
				power_step->delay);
			break;
		case LCD_POWER_TYPE_SIGNAL:
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n,
				"%d: type=%d, delay=%d\n",
				i, power_step->type, power_step->delay);
			break;
		default:
			break;
		}
		i++;
	}

	return len;
}

static int lcd_power_step_info_print(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int len = 0;

	len += lcd_power_step_print(lcd_drv->lcd_config, 1,
		(buf+len), (len + offset));
	len += lcd_power_step_print(lcd_drv->lcd_config, 0,
		(buf+len), (len + offset));
	len += lcd_cpu_gpio_register_print(lcd_drv->lcd_config,
		(buf+len), (len + offset));

	return len;
}

static int lcd_info_print_ttl(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"clk_pol         %u\n"
		"de_valid        %u\n"
		"hvsync_valid    %u\n"
		"rb_swap         %u\n"
		"bit_swap        %u\n\n",
		pconf->lcd_control.ttl_config->clk_pol,
		((pconf->lcd_control.ttl_config->sync_valid >> 1) & 1),
		((pconf->lcd_control.ttl_config->sync_valid >> 0) & 1),
		((pconf->lcd_control.ttl_config->swap_ctrl >> 1) & 1),
		((pconf->lcd_control.ttl_config->swap_ctrl >> 0) & 1));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"pinmux_flag     %d\n"
		"pinmux_pointer  0x%p\n\n",
		pconf->pinmux_flag,
		pconf->pin);

	return len;
}

static int lcd_info_print_lvds(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"lvds_repack     %u\n"
		"dual_port       %u\n"
		"pn_swap         %u\n"
		"port_swap       %u\n"
		"lane_reverse    %u\n"
		"phy_vswing      0x%x\n"
		"phy_preem       0x%x\n"
		"phy_clk_vswing  0x%x\n"
		"phy_clk_preem   0x%x\n\n",
		pconf->lcd_control.lvds_config->lvds_repack,
		pconf->lcd_control.lvds_config->dual_port,
		pconf->lcd_control.lvds_config->pn_swap,
		pconf->lcd_control.lvds_config->port_swap,
		pconf->lcd_control.lvds_config->lane_reverse,
		pconf->lcd_control.lvds_config->phy_vswing,
		pconf->lcd_control.lvds_config->phy_preem,
		pconf->lcd_control.lvds_config->phy_clk_vswing,
		pconf->lcd_control.lvds_config->phy_clk_preem);

	return len;
}

static int lcd_info_print_vbyone(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	struct vbyone_config_s *vx1_conf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;
	vx1_conf = pconf->lcd_control.vbyone_config;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"lane_count      %u\n"
		"region_num      %u\n"
		"byte_mode       %u\n"
		"color_fmt       %u\n"
		"bit_rate        %u\n"
		"phy_vswing      0x%x\n"
		"phy_preem       0x%x\n"
		"intr_en         %u\n"
		"vsync_intr_en   %u\n"
		"hw_filter_time  0x%x\n"
		"hw_filter_cnt   0x%x\n"
		"ctrl_flag       0x%x\n\n",
		vx1_conf->lane_count,
		vx1_conf->region_num,
		vx1_conf->byte_mode,
		vx1_conf->color_fmt,
		pconf->lcd_timing.bit_rate,
		vx1_conf->phy_vswing,
		vx1_conf->phy_preem,
		vx1_conf->intr_en,
		vx1_conf->vsync_intr_en,
		vx1_conf->hw_filter_time,
		vx1_conf->hw_filter_cnt,
		vx1_conf->ctrl_flag);
	if (vx1_conf->ctrl_flag & 0x1) {
		n = lcd_debug_info_len(len + offset);
		len += snprintf((buf+len), n,
			"power_on_reset_en    %u\n"
			"power_on_reset_delay %ums\n\n",
			(vx1_conf->ctrl_flag & 0x1),
			vx1_conf->power_on_reset_delay);
	}
	if (vx1_conf->ctrl_flag & 0x2) {
		n = lcd_debug_info_len(len + offset);
		len += snprintf((buf+len), n,
			"hpd_data_delay_en    %u\n"
			"hpd_data_delay       %ums\n\n",
			((vx1_conf->ctrl_flag >> 1) & 0x1),
			vx1_conf->hpd_data_delay);
	}
	if (vx1_conf->ctrl_flag & 0x4) {
		n = lcd_debug_info_len(len + offset);
		len += snprintf((buf+len), n,
			"cdr_training_hold_en %u\n"
			"cdr_training_hold    %ums\n\n",
			((vx1_conf->ctrl_flag >> 2) & 0x1),
			vx1_conf->cdr_training_hold);
	}

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"pinmux_flag          %d\n"
		"pinmux_pointer       0x%p\n\n",
		pconf->pinmux_flag,
		pconf->pin);

	return len;
}

static int lcd_info_print_mipi(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int len = 0;

#ifdef CONFIG_AMLOGIC_LCD_TABLET
	mipi_dsi_print_info(lcd_drv->lcd_config);
#endif

	return len;
}

static int lcd_info_print_mlvds(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"channel_num       %d\n"
		"channel_sel0      0x%08x\n"
		"channel_sel1      0x%08x\n"
		"clk_phase         0x%04x\n"
		"pn_swap           %u\n"
		"bit_swap          %u\n"
		"phy_vswing        0x%x\n"
		"phy_preem         0x%x\n"
		"bit_rate          %dHz\n"
		"pi_clk_sel        0x%03x\n\n",
		pconf->lcd_control.mlvds_config->channel_num,
		pconf->lcd_control.mlvds_config->channel_sel0,
		pconf->lcd_control.mlvds_config->channel_sel1,
		pconf->lcd_control.mlvds_config->clk_phase,
		pconf->lcd_control.mlvds_config->pn_swap,
		pconf->lcd_control.mlvds_config->bit_swap,
		pconf->lcd_control.mlvds_config->phy_vswing,
		pconf->lcd_control.mlvds_config->phy_preem,
		pconf->lcd_timing.bit_rate,
		pconf->lcd_control.mlvds_config->pi_clk_sel);

	len += lcd_tcon_info_print((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"pinmux_flag       %d\n"
		"pinmux_pointer    0x%p\n\n",
		pconf->pinmux_flag,
		pconf->pin);

	return len;
}

static int lcd_info_print_p2p(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"p2p_type          0x%x\n"
		"lane_num          %d\n"
		"channel_sel0      0x%08x\n"
		"channel_sel1      0x%08x\n"
		"pn_swap           %u\n"
		"bit_swap          %u\n"
		"bit_rate          %dHz\n"
		"phy_vswing        0x%x\n"
		"phy_preem         0x%x\n\n",
		pconf->lcd_control.p2p_config->p2p_type,
		pconf->lcd_control.p2p_config->lane_num,
		pconf->lcd_control.p2p_config->channel_sel0,
		pconf->lcd_control.p2p_config->channel_sel1,
		pconf->lcd_control.p2p_config->pn_swap,
		pconf->lcd_control.p2p_config->bit_swap,
		pconf->lcd_timing.bit_rate,
		pconf->lcd_control.p2p_config->phy_vswing,
		pconf->lcd_control.p2p_config->phy_preem);

	len += lcd_tcon_info_print((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"pinmux_flag       %d\n"
		"pinmux_pointer    0x%p\n\n",
		pconf->pinmux_flag,
		pconf->pin);

	return len;
}

static int lcd_info_print(char *buf, int offset)
{
	unsigned int lcd_clk, sync_duration;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;
	lcd_clk = (pconf->lcd_timing.lcd_clk / 1000);
	sync_duration = pconf->lcd_timing.sync_duration_num * 100;
	sync_duration = sync_duration / pconf->lcd_timing.sync_duration_den;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"driver version: %s\n"
		"panel_type: %s, chip: %d, mode: %s, status: %d, viu_sel: %d\n"
		"key_valid: %d, config_load: %d\n"
		"fr_auto_policy: %d\n",
		lcd_drv->version,
		pconf->lcd_propname, lcd_drv->data->chip_type,
		lcd_mode_mode_to_str(lcd_drv->lcd_mode),
		lcd_drv->lcd_status, lcd_drv->viu_sel,
		lcd_drv->lcd_key_valid, lcd_drv->lcd_config_load,
		lcd_drv->fr_auto_policy);

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"%s, %s %ubit, %ux%u@%u.%02uHz\n",
		pconf->lcd_basic.model_name,
		lcd_type_type_to_str(pconf->lcd_basic.lcd_type),
		pconf->lcd_basic.lcd_bits,
		pconf->lcd_basic.h_active, pconf->lcd_basic.v_active,
		(sync_duration / 100), (sync_duration % 100));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"lcd_clk         %u.%03uMHz\n"
		"ss_level        %d\n"
		"clk_auto        %d\n"
		"fr_adj_type     %d\n\n",
		(lcd_clk / 1000), (lcd_clk % 1000),
		pconf->lcd_timing.ss_level, pconf->lcd_timing.clk_auto,
		pconf->lcd_timing.fr_adjust_type);

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"h_period        %d\n"
		"v_period        %d\n"
		"hs_width        %d\n"
		"hs_backporch    %d\n"
		"hs_pol          %d\n"
		"vs_width        %d\n"
		"vs_backporch    %d\n"
		"vs_pol          %d\n\n",
		pconf->lcd_basic.h_period, pconf->lcd_basic.v_period,
		pconf->lcd_timing.hsync_width, pconf->lcd_timing.hsync_bp,
		pconf->lcd_timing.hsync_pol,
		pconf->lcd_timing.vsync_width, pconf->lcd_timing.vsync_bp,
		pconf->lcd_timing.vsync_pol);
	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"h_period_min    %d\n"
		"h_period_max    %d\n"
		"v_period_min    %d\n"
		"v_period_max    %d\n"
		"pclk_min        %d\n"
		"pclk_max        %d\n\n",
		pconf->lcd_basic.h_period_min, pconf->lcd_basic.h_period_max,
		pconf->lcd_basic.v_period_min, pconf->lcd_basic.v_period_max,
		pconf->lcd_basic.lcd_clk_min, pconf->lcd_basic.lcd_clk_max);

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"pll_ctrl        0x%08x\n"
		"div_ctrl        0x%08x\n"
		"clk_ctrl        0x%08x\n"
		"video_on_pixel  %d\n"
		"video_on_line   %d\n\n",
		pconf->lcd_timing.pll_ctrl, pconf->lcd_timing.div_ctrl,
		pconf->lcd_timing.clk_ctrl,
		pconf->lcd_timing.video_on_pixel,
		pconf->lcd_timing.video_on_line);

	if (lcd_debug_info_if) {
		if (lcd_debug_info_if->interface_print) {
			len += lcd_debug_info_if->interface_print((buf+len),
				(len+offset));
		} else {
			LCDERR("%s: interface_print is null\n", __func__);
		}
	} else {
		LCDERR("%s: lcd_debug_info_if is null\n", __func__);
	}

	return len;
}

static void lcd_reg_print_serializer(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int reg0, reg1;
	int n, len = 0;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		reg0 = HHI_LVDS_TX_PHY_CNTL0_TL1;
		reg1 = HHI_LVDS_TX_PHY_CNTL1_TL1;
		break;
	default:
		reg0 = HHI_LVDS_TX_PHY_CNTL0;
		reg1 = HHI_LVDS_TX_PHY_CNTL1;
		break;
	}

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nserializer regs:\n");
	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"HHI_LVDS_TX_PHY_CNTL0    [0x%04x] = 0x%08x\n",
		reg0, lcd_hiu_read(reg0));
	len += snprintf((buf+len), n,
		"HHI_LVDS_TX_PHY_CNTL1    [0x%04x] = 0x%08x\n",
		reg1, lcd_hiu_read(reg1));
}

static int lcd_reg_print_ttl(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nttl regs:\n");
	reg = L_DUAL_PORT_CNTL_ADDR;
	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"PORT_CNTL     [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STH1_HS_ADDR;
	len += snprintf((buf+len), n,
		"STH1_HS_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STH1_HE_ADDR;
	len += snprintf((buf+len), n,
		"STH1_HE_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STH1_VS_ADDR;
	len += snprintf((buf+len), n,
		"STH1_VS_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STH1_VE_ADDR;
	len += snprintf((buf+len), n,
		"STH1_VE_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STV1_HS_ADDR;
	len += snprintf((buf+len), n,
		"STV1_HS_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STV1_HE_ADDR;
	len += snprintf((buf+len), n,
		"STV1_HE_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STV1_VS_ADDR;
	len += snprintf((buf+len), n,
		"STV1_VS_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_STV1_VE_ADDR;
	len += snprintf((buf+len), n,
		"STV1_VE_ADDR  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_OEH_HS_ADDR;
	len += snprintf((buf+len), n,
		"OEH_HS_ADDR   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_OEH_HE_ADDR;
	len += snprintf((buf+len), n,
		"OEH_HE_ADDR   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_OEH_VS_ADDR;
	len += snprintf((buf+len), n,
		"OEH_VS_ADDR   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = L_OEH_VE_ADDR;
	len += snprintf((buf+len), n,
		"OEH_VE_ADDR   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	return len;
}

static int lcd_reg_print_lvds(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	lcd_reg_print_serializer((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nlvds regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = LVDS_PACK_CNTL_ADDR;
	len += snprintf((buf+len), n,
		"LVDS_PACK_CNTL  [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = LVDS_GEN_CNTL;
	len += snprintf((buf+len), n,
		"LVDS_GEN_CNTL   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = LCD_PORT_SWAP;
	len += snprintf((buf+len), n,
		"LCD_PORT_SWAP   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	return len;
}

static int lcd_reg_print_vbyone_txl(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	lcd_reg_print_serializer((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nvbyone regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = VBO_STATUS_L;
	len += snprintf((buf+len), n,
		"VX1_STATUS          [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	n = lcd_debug_info_len(len + offset);
	reg = VBO_INFILTER_CTRL;
	len += snprintf((buf+len), n,
		"VBO_INFILTER_CTRL   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INSGN_CTRL;
	len += snprintf((buf+len), n,
		"VBO_INSGN_CTRL      [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	n = lcd_debug_info_len(len + offset);
	reg = VBO_FSM_HOLDER_L;
	len += snprintf((buf+len), n,
		"VX1_FSM_HOLDER_L    [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_FSM_HOLDER_H;
	len += snprintf((buf+len), n,
		"VX1_FSM_HOLDER_H    [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INTR_STATE_CTRL;
	len += snprintf((buf+len), n,
		"VX1_INTR_STATE_CTRL [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INTR_UNMASK;
	len += snprintf((buf+len), n,
		"VX1_INTR_UNMASK     [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INTR_STATE;
	len += snprintf((buf+len), n,
		"VX1_INTR_STATE      [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	return len;
}

static int lcd_reg_print_vbyone_tl1(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	lcd_reg_print_serializer((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nvbyone regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = VBO_STATUS_L;
	len += snprintf((buf+len), n,
		"VX1_STATUS                   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	n = lcd_debug_info_len(len + offset);
	reg = VBO_INFILTER_TICK_PERIOD_H;
	len += snprintf((buf+len), n,
		"VBO_INFILTER_TICK_PERIOD_H   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INFILTER_TICK_PERIOD_L;
	len += snprintf((buf+len), n,
		"VBO_INFILTER_TICK_PERIOD_L   [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INSGN_CTRL;
	len += snprintf((buf+len), n,
		"VBO_INSGN_CTRL               [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	n = lcd_debug_info_len(len + offset);
	reg = VBO_FSM_HOLDER_L;
	len += snprintf((buf+len), n,
		"VX1_FSM_HOLDER_L             [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_FSM_HOLDER_H;
	len += snprintf((buf+len), n,
		"VX1_FSM_HOLDER_H             [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INTR_STATE_CTRL;
	len += snprintf((buf+len), n,
		"VX1_INTR_STATE_CTRL          [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INTR_UNMASK;
	len += snprintf((buf+len), n,
		"VX1_INTR_UNMASK              [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = VBO_INTR_STATE;
	len += snprintf((buf+len), n,
		"VX1_INTR_STATE               [0x%04x] = 0x%08x\n",
		reg, lcd_vcbus_read(reg));

	return len;
}

static int lcd_reg_print_mipi(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nmipi_dsi regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_TOP_CNTL;
	len += snprintf((buf+len), n,
		"MIPI_DSI_TOP_CNTL               [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_TOP_CLK_CNTL;
	len += snprintf((buf+len), n,
		"MIPI_DSI_TOP_CLK_CNTL           [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_PWR_UP_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_PWR_UP_OS          [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_PCKHDL_CFG_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_PCKHDL_CFG_OS      [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_LPCLK_CTRL_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_LPCLK_CTRL_OS      [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_CMD_MODE_CFG_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_CMD_MODE_CFG_OS    [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_VID_MODE_CFG_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_VID_MODE_CFG_OS    [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_MODE_CFG_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_MODE_CFG_OS        [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_PHY_STATUS_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_PHY_STATUS_OS      [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_INT_ST0_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_INT_ST0_OS         [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_DWC_INT_ST1_OS;
	len += snprintf((buf+len), n,
		"MIPI_DSI_DWC_INT_ST1_OS         [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_TOP_STAT;
	len += snprintf((buf+len), n,
		"MIPI_DSI_TOP_STAT               [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_TOP_INTR_CNTL_STAT;
	len += snprintf((buf+len), n,
		"MIPI_DSI_TOP_INTR_CNTL_STAT     [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = MIPI_DSI_TOP_MEM_PD;
	len += snprintf((buf+len), n,
		"MIPI_DSI_TOP_MEM_PD             [0x%04x] = 0x%08x\n",
		reg, dsi_host_read(reg));

	return len;
}

static int lcd_reg_print_mlvds(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	lcd_reg_print_serializer((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nmlvds regs:\n");

	n = lcd_debug_info_len(len + offset);
	reg = HHI_TCON_CLK_CNTL;
	len += snprintf((buf+len), n,
		"HHI_TCON_CLK_CNTL   [0x%04x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_TCON_CNTL0;
	len += snprintf((buf+len), n,
		"HHI_DIF_TCON_CNTL0  [0x%04x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_TCON_CNTL1;
	len += snprintf((buf+len), n,
		"HHI_DIF_TCON_CNTL1  [0x%04x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_TCON_CNTL2;
	len += snprintf((buf+len), n,
		"HHI_DIF_TCON_CNTL2  [0x%04x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));

	n = lcd_debug_info_len(len + offset);
	reg = TCON_TOP_CTRL;
	len += snprintf((buf+len), n,
		"TCON_TOP_CTRL       [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_RGB_IN_MUX;
	len += snprintf((buf+len), n,
		"TCON_RGB_IN_MUX     [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_OUT_CH_SEL0;
	len += snprintf((buf+len), n,
		"TCON_OUT_CH_SEL0    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_OUT_CH_SEL1;
	len += snprintf((buf+len), n,
		"TCON_OUT_CH_SEL1    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_STATUS0;
	len += snprintf((buf+len), n,
		"TCON_STATUS0        [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_PLLLOCK_CNTL;
	len += snprintf((buf+len), n,
		"TCON_PLLLOCK_CNTL   [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_RST_CTRL;
	len += snprintf((buf+len), n,
		"TCON_RST_CTRL       [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_AXI_OFST0;
	len += snprintf((buf+len), n,
		"TCON_AXI_OFST0      [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_AXI_OFST1;
	len += snprintf((buf+len), n,
		"TCON_AXI_OFST1      [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_AXI_OFST2;
	len += snprintf((buf+len), n,
		"TCON_AXI_OFST2      [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_CLK_CTRL;
	len += snprintf((buf+len), n,
		"TCON_CLK_CTRL       [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_STATUS1;
	len += snprintf((buf+len), n,
		"TCON_STATUS1        [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_DDRIF_CTRL1;
	len += snprintf((buf+len), n,
		"TCON_DDRIF_CTRL1    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	reg = TCON_DDRIF_CTRL2;
	len += snprintf((buf+len), n,
		"TCON_DDRIF_CTRL2    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_INTR_MASKN;
	len += snprintf((buf+len), n,
		"TCON_INTR_MASKN     [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	reg = TCON_INTR_RO;
	len += snprintf((buf+len), n,
		"TCON_INTR_RO        [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));

	return len;
}

static int lcd_reg_print_p2p(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	lcd_reg_print_serializer((buf+len), (len+offset));

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\np2p regs:\n");

	n = lcd_debug_info_len(len + offset);
	reg = HHI_TCON_CLK_CNTL;
	len += snprintf((buf+len), n,
		"HHI_TCON_CLK_CNTL   [0x%04x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));

	n = lcd_debug_info_len(len + offset);
	reg = TCON_TOP_CTRL;
	len += snprintf((buf+len), n,
		"TCON_TOP_CTRL       [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_RGB_IN_MUX;
	len += snprintf((buf+len), n,
		"TCON_RGB_IN_MUX     [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_OUT_CH_SEL0;
	len += snprintf((buf+len), n,
		"TCON_OUT_CH_SEL0    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_OUT_CH_SEL1;
	len += snprintf((buf+len), n,
		"TCON_OUT_CH_SEL1    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_STATUS0;
	len += snprintf((buf+len), n,
		"TCON_STATUS0        [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_PLLLOCK_CNTL;
	len += snprintf((buf+len), n,
		"TCON_PLLLOCK_CNTL   [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_RST_CTRL;
	len += snprintf((buf+len), n,
		"TCON_RST_CTRL       [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_AXI_OFST0;
	len += snprintf((buf+len), n,
		"TCON_AXI_OFST0      [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_AXI_OFST1;
	len += snprintf((buf+len), n,
		"TCON_AXI_OFST1      [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_AXI_OFST2;
	len += snprintf((buf+len), n,
		"TCON_AXI_OFST2      [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_CLK_CTRL;
	len += snprintf((buf+len), n,
		"TCON_CLK_CTRL       [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_STATUS1;
	len += snprintf((buf+len), n,
		"TCON_STATUS1        [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_DDRIF_CTRL1;
	len += snprintf((buf+len), n,
		"TCON_DDRIF_CTRL1    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	reg = TCON_DDRIF_CTRL2;
	len += snprintf((buf+len), n,
		"TCON_DDRIF_CTRL2    [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = TCON_INTR_MASKN;
	len += snprintf((buf+len), n,
		"TCON_INTR_MASKN     [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));
	reg = TCON_INTR_RO;
	len += snprintf((buf+len), n,
		"TCON_INTR_RO        [0x%04x] = 0x%08x\n",
		reg, lcd_tcon_read(reg));

	return len;
}

static int lcd_reg_print_phy_analog(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nphy analog regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL1;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL1  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL2;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL2  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL3;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL3  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));

	return len;
}

static int lcd_reg_print_phy_analog_tl1(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nphy analog regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL1;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL1  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL2;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL2  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL3;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL3  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL4;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL4  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL6;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL6  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL7;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL7  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL8;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL8  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL9;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL9  [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL10;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL10 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL11;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL11 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL12;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL12 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL13;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL13 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL14;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL14 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL15;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL15 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_DIF_CSI_PHY_CNTL16;
	len += snprintf((buf+len), n,
		"HHI_DIF_CSI_PHY_CNTL16 [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));

	return len;
}

static int lcd_reg_print_mipi_phy_analog(char *buf, int offset)
{
	unsigned int reg;
	int n, len = 0;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nmipi_dsi_phy analog regs:\n");
	n = lcd_debug_info_len(len + offset);
	reg = HHI_MIPI_CNTL0;
	len += snprintf((buf+len), n,
		"HHI_MIPI_CNTL0   [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_MIPI_CNTL1;
	len += snprintf((buf+len), n,
		"HHI_MIPI_CNTL1   [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));
	n = lcd_debug_info_len(len + offset);
	reg = HHI_MIPI_CNTL2;
	len += snprintf((buf+len), n,
		"HHI_MIPI_CNTL2   [0x%02x] = 0x%08x\n",
		reg, lcd_hiu_read(reg));

	return len;
}

static int lcd_reg_print(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int i, n, len = 0;
	struct lcd_config_s *pconf;
	unsigned int *table;

	pconf = lcd_drv->lcd_config;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n, "\nclk regs:\n");
	if (lcd_debug_info_reg) {
		if (lcd_debug_info_reg->reg_clk_table) {
			table = lcd_debug_info_reg->reg_clk_table;
			i = 0;
			while (i < LCD_DEBUG_REG_CNT_MAX) {
				if (table[i] == LCD_DEBUG_REG_END)
					break;
				n = lcd_debug_info_len(len + offset);
				len += snprintf((buf+len), n,
					"hiu     [0x%02x] = 0x%08x\n",
					table[i], lcd_hiu_read(table[i]));
				i++;
			}
		} else {
			LCDERR("%s: reg_clk_table is null\n", __func__);
		}

		if (lcd_debug_info_reg->reg_encl_table) {
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n, "\nencl regs:\n");
			table = lcd_debug_info_reg->reg_encl_table;
			i = 0;
			while (i < LCD_DEBUG_REG_CNT_MAX) {
				if (table[i] == LCD_DEBUG_REG_END)
					break;
				n = lcd_debug_info_len(len + offset);
				len += snprintf((buf+len), n,
					"vcbus   [0x%04x] = 0x%08x\n",
					table[i], lcd_vcbus_read(table[i]));
				i++;
			}
		} else {
			LCDERR("%s: reg_encl_table is null\n", __func__);
		}

		if (lcd_debug_info_reg->reg_pinmux_table) {
			n = lcd_debug_info_len(len + offset);
			len += snprintf((buf+len), n, "\npinmux regs:\n");
			table = lcd_debug_info_reg->reg_pinmux_table;
			i = 0;
			while (i < LCD_DEBUG_REG_CNT_MAX) {
				if (table[i] == LCD_DEBUG_REG_END)
					break;
				len += snprintf((buf+len), n,
					"PERIPHS_PIN_MUX  [0x%02x] = 0x%08x\n",
					table[i], lcd_periphs_read(table[i]));
				i++;
			}
		}
	} else {
		LCDERR("%s: lcd_debug_info_reg is null\n", __func__);
	}

	if (lcd_debug_info_if) {
		if (lcd_debug_info_if->reg_dump_interface) {
			len += lcd_debug_info_if->reg_dump_interface((buf+len),
				(len+offset));
		} else {
			LCDERR("%s: reg_dump_interface is null\n", __func__);
		}

		if (lcd_debug_info_if->reg_dump_phy) {
			len += lcd_debug_info_if->reg_dump_phy((buf+len),
				(len+offset));
		}
	} else {
		LCDERR("%s: lcd_debug_info_if is null\n", __func__);
	}

	return len;
}

static int lcd_optical_info_print(char *buf, int offset)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	int n, len = 0;

	pconf = lcd_drv->lcd_config;
	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"\nlcd optical info:\n"
		"hdr_support          %d\n"
		"features             %d\n"
		"primaries_r_x        %d\n"
		"primaries_r_y        %d\n"
		"primaries_g_x        %d\n"
		"primaries_g_y        %d\n"
		"primaries_b_x        %d\n"
		"primaries_b_y        %d\n"
		"white_point_x        %d\n"
		"white_point_y        %d\n"
		"luma_max             %d\n"
		"luma_min             %d\n\n",
		pconf->optical_info.hdr_support,
		pconf->optical_info.features,
		pconf->optical_info.primaries_r_x,
		pconf->optical_info.primaries_r_y,
		pconf->optical_info.primaries_g_x,
		pconf->optical_info.primaries_g_y,
		pconf->optical_info.primaries_b_x,
		pconf->optical_info.primaries_b_y,
		pconf->optical_info.white_point_x,
		pconf->optical_info.white_point_y,
		pconf->optical_info.luma_max,
		pconf->optical_info.luma_min);

	return len;
}

static struct work_struct lcd_test_check_work;
static void lcd_test_pattern_check(struct work_struct *p_work)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int flag;

	flag = (lcd_drv->lcd_test_state > 0) ? 1 : 0;
	aml_lcd_notifier_call_chain(LCD_EVENT_TEST_PATTERN, &flag);
}

#define LCD_ENC_TST_NUM_MAX    9
static char *lcd_enc_tst_str[] = {
	"0-None",        /* 0 */
	"1-Color Bar",   /* 1 */
	"2-Thin Line",   /* 2 */
	"3-Dot Grid",    /* 3 */
	"4-Gray",        /* 4 */
	"5-Red",         /* 5 */
	"6-Green",       /* 6 */
	"7-Blue",        /* 7 */
	"8-Black",       /* 8 */
};

static unsigned int lcd_enc_tst[][7] = {
/*tst_mode,    Y,       Cb,     Cr,     tst_en,  vfifo_en  rgbin*/
	{0,    0x200,   0x200,  0x200,   0,      1,        3},  /* 0 */
	{1,    0x200,   0x200,  0x200,   1,      0,        1},  /* 1 */
	{2,    0x200,   0x200,  0x200,   1,      0,        1},  /* 2 */
	{3,    0x200,   0x200,  0x200,   1,      0,        1},  /* 3 */
	{0,    0x1ff,   0x1ff,  0x1ff,   1,      0,        3},  /* 4 */
	{0,    0x3ff,     0x0,    0x0,   1,      0,        3},  /* 5 */
	{0,      0x0,   0x3ff,    0x0,   1,      0,        3},  /* 6 */
	{0,      0x0,     0x0,  0x3ff,   1,      0,        3},  /* 7 */
	{0,      0x0,     0x0,    0x0,   1,      0,        3},  /* 8 */
};

void lcd_debug_test(unsigned int num)
{
	unsigned int h_active, video_on_pixel;
	static int change_flag;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	num = (num >= LCD_ENC_TST_NUM_MAX) ? 0 : num;

	if (lcd_drv->workqueue)
		queue_work(lcd_drv->workqueue, &lcd_test_check_work);
	else
		schedule_work(&lcd_test_check_work);

	h_active = lcd_drv->lcd_config->lcd_basic.h_active;
	video_on_pixel = lcd_drv->lcd_config->lcd_timing.video_on_pixel;
	if (num > 0) {
		if (!change_flag) {
			if (lcd_vcbus_getb(L_GAMMA_CNTL_PORT, 0, 1)) {
				change_flag = 1;
				lcd_vcbus_setb(L_GAMMA_CNTL_PORT, 0, 0, 1);
			}
		}
	} else {
		if (change_flag) {
			change_flag = 0;
			lcd_vcbus_setb(L_GAMMA_CNTL_PORT, 1, 0, 1);
		}
	}
	lcd_vcbus_write(ENCL_VIDEO_RGBIN_CTRL, lcd_enc_tst[num][6]);
	lcd_vcbus_write(ENCL_TST_MDSEL, lcd_enc_tst[num][0]);
	lcd_vcbus_write(ENCL_TST_Y, lcd_enc_tst[num][1]);
	lcd_vcbus_write(ENCL_TST_CB, lcd_enc_tst[num][2]);
	lcd_vcbus_write(ENCL_TST_CR, lcd_enc_tst[num][3]);
	lcd_vcbus_write(ENCL_TST_CLRBAR_STRT, video_on_pixel);
	lcd_vcbus_write(ENCL_TST_CLRBAR_WIDTH, (h_active / 9));
	lcd_vcbus_write(ENCL_TST_EN, lcd_enc_tst[num][4]);
	lcd_vcbus_setb(ENCL_VIDEO_MODE_ADV, lcd_enc_tst[num][5], 3, 1);
	if (num > 0)
		LCDPR("show test pattern: %s\n", lcd_enc_tst_str[num]);
	else
		LCDPR("disable test pattern\n");
}

void lcd_mute_setting(unsigned char flag)
{
	if (flag) {
		lcd_vcbus_write(ENCL_VIDEO_RGBIN_CTRL, 3);
		lcd_vcbus_write(ENCL_TST_MDSEL, 0);
		lcd_vcbus_write(ENCL_TST_Y, 0);
		lcd_vcbus_write(ENCL_TST_CB, 0);
		lcd_vcbus_write(ENCL_TST_CR, 0);
		lcd_vcbus_write(ENCL_TST_EN, 1);
		lcd_vcbus_setb(ENCL_VIDEO_MODE_ADV, 0, 3, 1);
	} else {
		lcd_vcbus_setb(ENCL_VIDEO_MODE_ADV, 1, 3, 1);
		lcd_vcbus_write(ENCL_TST_EN, 0);
	}
}

static void lcd_screen_restore(void)
{
	unsigned int h_active, video_on_pixel;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int num;

	num = lcd_drv->lcd_test_state;
	num = (num >= LCD_ENC_TST_NUM_MAX) ? 0 : num;

	if (lcd_drv->workqueue)
		queue_work(lcd_drv->workqueue, &lcd_test_check_work);
	else
		schedule_work(&lcd_test_check_work);

	h_active = lcd_drv->lcd_config->lcd_basic.h_active;
	video_on_pixel = lcd_drv->lcd_config->lcd_timing.video_on_pixel;

	lcd_vcbus_write(ENCL_VIDEO_RGBIN_CTRL, lcd_enc_tst[num][6]);
	lcd_vcbus_write(ENCL_TST_MDSEL, lcd_enc_tst[num][0]);
	lcd_vcbus_write(ENCL_TST_Y, lcd_enc_tst[num][1]);
	lcd_vcbus_write(ENCL_TST_CB, lcd_enc_tst[num][2]);
	lcd_vcbus_write(ENCL_TST_CR, lcd_enc_tst[num][3]);
	lcd_vcbus_write(ENCL_TST_CLRBAR_STRT, video_on_pixel);
	lcd_vcbus_write(ENCL_TST_CLRBAR_WIDTH, (h_active / 9));
	lcd_vcbus_write(ENCL_TST_EN, lcd_enc_tst[num][4]);
	lcd_vcbus_setb(ENCL_VIDEO_MODE_ADV, lcd_enc_tst[num][5], 3, 1);
	if (num > 0)
		LCDPR("show test pattern: %s\n", lcd_enc_tst_str[num]);
}

static void lcd_screen_black(void)
{
	lcd_mute_setting(1);
}

static unsigned int lcd_prbs_flag;
static unsigned int lcd_prbs_cnt;
static void aml_lcd_prbs_test(unsigned int s)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int reg0, reg1;
	unsigned int val1, val2, cnt = 0, timeout;
	int i, ret;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
	case LCD_CHIP_AXG:
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		LCDERR("not support\n");
		return;
	case LCD_CHIP_TXL:
	case LCD_CHIP_TXLX:
		reg0 = HHI_LVDS_TX_PHY_CNTL0;
		reg1 = HHI_LVDS_TX_PHY_CNTL1;
		break;
	default:
		reg0 = HHI_LVDS_TX_PHY_CNTL0_TL1;
		reg1 = HHI_LVDS_TX_PHY_CNTL1_TL1;
		break;
	}

	lcd_hiu_write(reg0, 0xfff20c4);
	lcd_hiu_setb(reg0, 1, 12, 1);
	val1 = lcd_hiu_getb(reg1, 12, 12);

	s = (s > 1800) ? 1800 : s;
	timeout = s * 200;
	while (lcd_prbs_flag) {
		if (s > 1) { /* when s=1, means always run */
			if (cnt++ >= timeout)
				break;
		}
		usleep_range(5000, 5001);
		ret = 1;
		for (i = 0; i < 5; i++) {
			val2 = lcd_hiu_getb(reg1, 12, 12);
			if (val2 != val1) {
				ret = 0;
				break;
			}
		}
		if (ret) {
			LCDERR("lcd prbs check error 1, val:0x%03x, cnt:%d\n",
			       val2, lcd_prbs_cnt);
			return;
		}
		val1 = val2;
		if (lcd_hiu_getb(reg1, 0, 12)) {
			LCDERR("lcd prbs check error 2, cnt:%d\n",
			       lcd_prbs_cnt);
			return;
		}
		if (lcd_prbs_cnt >= 0xffffffff)
			lcd_prbs_cnt = 0;
		else
			lcd_prbs_cnt++;
	}

	lcd_prbs_cnt = 0;
	LCDPR("lcd prbs check ok\n");
}

static ssize_t lcd_debug_prbs_show(struct class *class,
				   struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "lcd prbs flag: %d, cnt: %d\n",
		       lcd_prbs_flag, lcd_prbs_cnt);
}

static ssize_t lcd_debug_prbs_store(struct class *class,
				    struct class_attribute *attr,
				    const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 1;

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		LCDERR("invalid data\n");
		return -EINVAL;
	}
	if (temp) {
		if (lcd_prbs_flag) {
			LCDPR("lcd prbs check is already running\n");
			return count;
		}
		lcd_prbs_cnt = 0;
		lcd_prbs_flag = 1;
		aml_lcd_prbs_test(temp);
	} else {
		if (lcd_prbs_flag == 0) {
			LCDPR("lcd prbs check is already stopped\n");
			return count;
		}
		lcd_prbs_flag = 0;
	}

	return count;
}

static void lcd_vinfo_update(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vinfo_s *vinfo;
	struct lcd_config_s *pconf;

	vinfo = lcd_drv->lcd_info;
	pconf = lcd_drv->lcd_config;
	if (vinfo) {
		vinfo->width = pconf->lcd_basic.h_active;
		vinfo->height = pconf->lcd_basic.v_active;
		vinfo->field_height = pconf->lcd_basic.v_active;
		vinfo->aspect_ratio_num = pconf->lcd_basic.screen_width;
		vinfo->aspect_ratio_den = pconf->lcd_basic.screen_height;
		vinfo->screen_real_width = pconf->lcd_basic.screen_width;
		vinfo->screen_real_height = pconf->lcd_basic.screen_height;
		vinfo->sync_duration_num = pconf->lcd_timing.sync_duration_num;
		vinfo->sync_duration_den = pconf->lcd_timing.sync_duration_den;
		vinfo->video_clk = pconf->lcd_timing.lcd_clk;
		vinfo->htotal = pconf->lcd_basic.h_period;
		vinfo->vtotal = pconf->lcd_basic.v_period;
	}
	lcd_vout_notify_mode_change();
}

static void lcd_debug_config_update(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_drv->module_reset();

	lcd_vinfo_update();
}

static void lcd_debug_clk_change(unsigned int pclk)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	unsigned int sync_duration;

	lcd_vout_notify_mode_change_pre();

	pconf = lcd_drv->lcd_config;
	sync_duration = pclk / pconf->lcd_basic.h_period;
	sync_duration = sync_duration * 100 / pconf->lcd_basic.v_period;
	pconf->lcd_timing.lcd_clk = pclk;
	pconf->lcd_timing.lcd_clk_dft = pconf->lcd_timing.lcd_clk;
	pconf->lcd_timing.sync_duration_num = sync_duration;
	pconf->lcd_timing.sync_duration_den = 100;

	/* update vinfo */
	lcd_drv->lcd_info->sync_duration_num = sync_duration;
	lcd_drv->lcd_info->sync_duration_den = 100;
	lcd_drv->lcd_info->video_clk = pclk;

	switch (lcd_drv->lcd_mode) {
#ifdef CONFIG_AMLOGIC_LCD_TV
	case LCD_MODE_TV:
		lcd_tv_clk_update(pconf);
		break;
#endif
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	case LCD_MODE_TABLET:
		lcd_tablet_clk_update(pconf);
		break;
#endif
	default:
		LCDPR("invalid lcd mode\n");
		break;
	}

	lcd_vout_notify_mode_change();
}

static void lcd_power_interface_ctrl(int state)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	mutex_lock(&lcd_drv->power_mutex);
	LCDPR("%s: %d\n", __func__, state);
	if (state) {
		if (lcd_drv->lcd_status & LCD_STATUS_ENCL_ON) {
			aml_lcd_notifier_call_chain(
				LCD_EVENT_IF_POWER_ON, NULL);
			lcd_if_enable_retry(lcd_drv->lcd_config);
		} else {
			LCDERR("%s: can't power on when controller is off\n",
				__func__);
		}
	} else {
		aml_lcd_notifier_call_chain(LCD_EVENT_IF_POWER_OFF, NULL);
	}
	mutex_unlock(&lcd_drv->power_mutex);
}

static ssize_t lcd_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp, val[6];
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	char *print_buf;

	pconf = lcd_drv->lcd_config;
	switch (buf[0]) {
	case 'c': /* clk */
		ret = sscanf(buf, "clk %d", &temp);
		if (ret == 1) {
			if (temp > 200) {
				pr_info("set clk: %dHz\n", temp);
			} else {
				pr_info("set frame_rate: %dHz\n", temp);
				temp = pconf->lcd_basic.h_period *
					pconf->lcd_basic.v_period * temp;
				pr_info("set clk: %dHz\n", temp);
			}
			lcd_debug_clk_change(temp);
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'b':
		if (buf[1] == 'a') { /* basic */
			ret = sscanf(buf, "basic %d %d %d %d %d",
				&val[0], &val[1], &val[2], &val[3], &val[4]);
			if (ret == 4) {
				pconf->lcd_basic.h_active = val[0];
				pconf->lcd_basic.v_active = val[1];
				pconf->lcd_basic.h_period = val[2];
				pconf->lcd_basic.v_period = val[3];
				pconf->lcd_timing.h_period_dft = val[2];
				pconf->lcd_timing.v_period_dft = val[3];
				pr_info("set h_active=%d, v_active=%d\n",
					val[0], val[1]);
				pr_info("set h_period=%d, v_period=%d\n",
					val[2], val[3]);
				lcd_timing_init_config(pconf);
				lcd_debug_config_update();
			} else if (ret == 5) {
				pconf->lcd_basic.h_active = val[0];
				pconf->lcd_basic.v_active = val[1];
				pconf->lcd_basic.h_period = val[2];
				pconf->lcd_basic.v_period = val[3];
				pconf->lcd_timing.h_period_dft = val[2];
				pconf->lcd_timing.v_period_dft = val[3];
				pconf->lcd_basic.lcd_bits = val[4];
				pr_info("set h_active=%d, v_active=%d\n",
					val[0], val[1]);
				pr_info("set h_period=%d, v_period=%d\n",
					val[2], val[3]);
				pr_info("set lcd_bits=%d\n", val[4]);
				lcd_timing_init_config(pconf);
				lcd_debug_config_update();
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		} else if (buf[1] == 'i') { /* bit */
			ret = sscanf(buf, "bit %d", &val[0]);
			if (ret == 1) {
				pconf->lcd_basic.lcd_bits = val[4];
				pr_info("set lcd_bits=%d\n", val[4]);
				lcd_debug_config_update();
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		}
		break;
	case 's': /* sync */
		ret = sscanf(buf, "sync %d %d %d %d %d %d",
			&val[0], &val[1], &val[2], &val[3], &val[4], &val[5]);
		if (ret == 6) {
			pconf->lcd_timing.hsync_width = val[0];
			pconf->lcd_timing.hsync_bp =    val[1];
			pconf->lcd_timing.hsync_pol =   val[2];
			pconf->lcd_timing.vsync_width = val[3];
			pconf->lcd_timing.vsync_bp =    val[4];
			pconf->lcd_timing.vsync_pol =   val[5];
			pr_info("set hsync width=%d, bp=%d, pol=%d\n",
				val[0], val[1], val[2]);
			pr_info("set vsync width=%d, bp=%d, pol=%d\n",
				val[3], val[4], val[5]);
			lcd_timing_init_config(pconf);
			lcd_debug_config_update();
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 't': /* test */
		ret = sscanf(buf, "test %d", &temp);
		if (ret == 1) {
			lcd_drv->lcd_test_flag =
				(unsigned char)(temp | LCD_TEST_UPDATE);
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'i': /* info */
		print_buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
		if (print_buf == NULL) {
			LCDERR("%s: buf malloc error\n", __func__);
			return -EINVAL;
		}
		lcd_info_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		memset(print_buf, 0, PR_BUF_MAX);
		lcd_power_step_info_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		kfree(print_buf);
		break;
	case 'r':
		if (buf[2] == 'g') { /* reg */
			print_buf = kcalloc(PR_BUF_MAX, sizeof(char),
				GFP_KERNEL);
			if (print_buf == NULL) {
				LCDERR("%s: buf malloc error\n", __func__);
				return -EINVAL;
			}
			lcd_reg_print(print_buf, 0);
			lcd_debug_info_print(print_buf);
			kfree(print_buf);
		} else if (buf[2] == 's') { /* reset */
			lcd_drv->module_reset();
		} else if (buf[2] == 'n') { /* range */
			ret = sscanf(buf, "range %d %d %d %d %d %d",
				&val[0], &val[1], &val[2], &val[3],
				&val[4], &val[5]);
			if (ret == 6) {
				pconf->lcd_basic.h_period_min = val[0];
				pconf->lcd_basic.h_period_max = val[1];
				pconf->lcd_basic.h_period_min = val[2];
				pconf->lcd_basic.v_period_max = val[3];
				pconf->lcd_basic.lcd_clk_min  = val[4];
				pconf->lcd_basic.lcd_clk_max  = val[5];
				pr_info("set h_period min=%d, max=%d\n",
					pconf->lcd_basic.h_period_min,
					pconf->lcd_basic.h_period_max);
				pr_info("set v_period min=%d, max=%d\n",
					pconf->lcd_basic.v_period_min,
					pconf->lcd_basic.v_period_max);
				pr_info("set pclk min=%d, max=%d\n",
					pconf->lcd_basic.lcd_clk_min,
					pconf->lcd_basic.lcd_clk_max);
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		}
		break;
	case 'd': /* dump */
		print_buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
		if (print_buf == NULL) {
			LCDERR("%s: buf malloc error\n", __func__);
			return -EINVAL;
		}
		lcd_info_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		memset(print_buf, 0, PR_BUF_MAX);
		lcd_power_step_info_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		memset(print_buf, 0, PR_BUF_MAX);
		lcd_reg_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		memset(print_buf, 0, PR_BUF_MAX);
		lcd_optical_info_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		kfree(print_buf);
		break;
	case 'k': /* key */
		LCDPR("key_valid: %d, config_load: %d\n",
			lcd_drv->lcd_key_valid, lcd_drv->lcd_config_load);
		if (lcd_drv->lcd_key_valid)
			lcd_unifykey_print();
		break;
	case 'h': /* hdr */
		print_buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
		if (print_buf == NULL) {
			LCDERR("%s: buf malloc error\n", __func__);
			return -EINVAL;
		}
		lcd_optical_info_print(print_buf, 0);
		lcd_debug_info_print(print_buf);
		kfree(print_buf);
		break;
	case 'p': /* power */
		ret = sscanf(buf, "power %d", &temp);
		if (ret == 1) {
			lcd_power_interface_ctrl(temp);
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'v':
		ret = sscanf(buf, "vout %d", &temp);
		if (ret == 1) {
			LCDPR("vout_serve bypass: %d\n", temp);
			lcd_vout_serve_bypass = temp;
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		LCDERR("wrong command\n");
		break;
	}
	return count;
}

static ssize_t lcd_debug_change_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_debug_change_usage_str);
}

static void lcd_debug_change_clk_change(unsigned int pclk)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	unsigned int sync_duration;

	pconf = lcd_drv->lcd_config;
	sync_duration = pclk / pconf->lcd_basic.h_period;
	sync_duration = sync_duration * 100 / pconf->lcd_basic.v_period;
	pconf->lcd_timing.lcd_clk = pclk;
	pconf->lcd_timing.lcd_clk_dft = pconf->lcd_timing.lcd_clk;
	pconf->lcd_timing.sync_duration_num = sync_duration;
	pconf->lcd_timing.sync_duration_den = 100;

	switch (lcd_drv->lcd_mode) {
#ifdef CONFIG_AMLOGIC_LCD_TV
	case LCD_MODE_TV:
		lcd_tv_clk_config_change(pconf);
		break;
#endif
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	case LCD_MODE_TABLET:
		lcd_tablet_clk_config_change(pconf);
		break;
#endif
	default:
		LCDPR("invalid lcd mode\n");
		break;
	}
}

static ssize_t lcd_debug_change_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp, val[10];
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	struct ttl_config_s *ttl_conf;
	struct lvds_config_s *lvds_conf;
	struct vbyone_config_s *vx1_conf;
	struct dsi_config_s *dsi_conf;
	struct mlvds_config_s *mlvds_conf;
	struct p2p_config_s *p2p_conf;

	pconf = lcd_drv->lcd_config;
	switch (buf[0]) {
	case 'c': /* clk */
		ret = sscanf(buf, "clk %d", &temp);
		if (ret == 1) {
			if (temp > 200) {
				pr_info("change clk=%dHz\n", temp);
			} else {
				pr_info("change frame_rate=%dHz\n", temp);
				temp = pconf->lcd_basic.h_period *
					pconf->lcd_basic.v_period * temp;
				pr_info("change clk=%dHz\n", temp);
			}
			lcd_debug_change_clk_change(temp);
			pconf->change_flag = 1;
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'b':
		if (buf[1] == 'a') { /* basic */
			ret = sscanf(buf, "basic %d %d %d %d %d",
				&val[0], &val[1], &val[2], &val[3], &val[4]);
			if (ret == 4) {
				pconf->lcd_basic.h_active = val[0];
				pconf->lcd_basic.v_active = val[1];
				pconf->lcd_basic.h_period = val[2];
				pconf->lcd_basic.v_period = val[3];
				pconf->lcd_timing.h_period_dft = val[2];
				pconf->lcd_timing.v_period_dft = val[3];
				pr_info("change h_active=%d, v_active=%d\n",
					val[0], val[1]);
				pr_info("change h_period=%d, v_period=%d\n",
					val[2], val[3]);
				lcd_timing_init_config(pconf);
				pconf->change_flag = 1;
			} else if (ret == 5) {
				pconf->lcd_basic.h_active = val[0];
				pconf->lcd_basic.v_active = val[1];
				pconf->lcd_basic.h_period = val[2];
				pconf->lcd_basic.v_period = val[3];
				pconf->lcd_timing.h_period_dft = val[2];
				pconf->lcd_timing.v_period_dft = val[3];
				pconf->lcd_basic.lcd_bits = val[4];
				pr_info("change h_active=%d, v_active=%d\n",
					val[0], val[1]);
				pr_info("change h_period=%d, v_period=%d\n",
					val[2], val[3]);
				pr_info("change lcd_bits=%d\n", val[4]);
				lcd_timing_init_config(pconf);
				pconf->change_flag = 1;
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		} else if (buf[1] == 'i') { /* bit */
			ret = sscanf(buf, "bit %d", &val[0]);
			if (ret == 1) {
				pconf->lcd_basic.lcd_bits = val[4];
				pr_info("change lcd_bits=%d\n", val[4]);
				pconf->change_flag = 1;
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		}
		break;
	case 's':
		if (buf[1] == 'e') { /* set */
			if (pconf->change_flag) {
				LCDPR("apply config changing\n");
				lcd_debug_config_update();
			} else {
				LCDPR("config is no changing\n");
			}
		} else if (buf[1] == 'y') { /* sync */
			ret = sscanf(buf, "sync %d %d %d %d %d %d",
				&val[0], &val[1], &val[2], &val[3],
				&val[4], &val[5]);
			if (ret == 6) {
				pconf->lcd_timing.hsync_width = val[0];
				pconf->lcd_timing.hsync_bp =    val[1];
				pconf->lcd_timing.hsync_pol =   val[2];
				pconf->lcd_timing.vsync_width = val[3];
				pconf->lcd_timing.vsync_bp =    val[4];
				pconf->lcd_timing.vsync_pol =   val[5];
				pr_info("change hs width=%d, bp=%d, pol=%d\n",
					val[0], val[1], val[2]);
				pr_info("change vs width=%d, bp=%d, pol=%d\n",
					val[3], val[4], val[5]);
				lcd_timing_init_config(pconf);
				pconf->change_flag = 1;
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		}
		break;
	case 't':
		ttl_conf = pconf->lcd_control.ttl_config;
		ret = sscanf(buf, "ttl %d %d %d %d %d",
			&val[0], &val[1], &val[2], &val[3], &val[4]);
		if (ret == 5) {
			ttl_conf->clk_pol = val[0];
			ttl_conf->sync_valid = ((val[1] << 1) | val[2]);
			ttl_conf->swap_ctrl = ((val[3] << 1) | val[4]);
			pr_info("set ttl config:\n"
	"clk_pol=%d, de_valid=%d, de_valid=%d, rb_swap=%d, bit_swap=%d\n",
				val[0], val[1], val[2], val[3], val[4]);
			lcd_debug_change_clk_change(pconf->lcd_timing.lcd_clk);
			pconf->change_flag = 1;
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'l':
		lvds_conf = pconf->lcd_control.lvds_config;
		ret = sscanf(buf, "lvds %d %d %d %d %d",
			&val[0], &val[1], &val[2], &val[3], &val[4]);
		if (ret == 5) {
			lvds_conf->lvds_repack = val[0];
			lvds_conf->dual_port = val[1];
			lvds_conf->pn_swap = val[2];
			lvds_conf->port_swap = val[3];
			lvds_conf->lane_reverse = val[4];
			pr_info("set lvds config:\n"
	"repack=%d, dual_port=%d, pn_swap=%d, port_swap=%d, lane_reverse=%d\n",
				lvds_conf->lvds_repack, lvds_conf->dual_port,
				lvds_conf->pn_swap, lvds_conf->port_swap,
				lvds_conf->lane_reverse);
			lcd_debug_change_clk_change(pconf->lcd_timing.lcd_clk);
			pconf->change_flag = 1;
		} else if (ret == 4) {
			lvds_conf->lvds_repack = val[0];
			lvds_conf->dual_port = val[1];
			lvds_conf->pn_swap = val[2];
			lvds_conf->port_swap = val[3];
			pr_info("set lvds config:\n"
			"repack=%d, dual_port=%d, pn_swap=%d, port_swap=%d\n",
				lvds_conf->lvds_repack, lvds_conf->dual_port,
				lvds_conf->pn_swap, lvds_conf->port_swap);
			lcd_debug_change_clk_change(pconf->lcd_timing.lcd_clk);
			pconf->change_flag = 1;
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'v':
		vx1_conf = pconf->lcd_control.vbyone_config;
		ret = sscanf(buf, "vbyone %d %d %d %d",
			&val[0], &val[1], &val[2], &val[3]);
		if (ret == 4 || ret == 3) {
			vx1_conf->lane_count = val[0];
			vx1_conf->region_num = val[1];
			vx1_conf->byte_mode = val[2];
			pr_info("set vbyone config:\n"
				"lane_count=%d, region_num=%d, byte_mode=%d\n",
				vx1_conf->lane_count, vx1_conf->region_num,
				vx1_conf->byte_mode);
			lcd_debug_change_clk_change(pconf->lcd_timing.lcd_clk);
			pconf->change_flag = 1;
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'm':
		if (buf[1] == 'i') {
			dsi_conf = pconf->lcd_control.mipi_config;
			ret = sscanf(buf, "mipi %d %d %d %d %d %d %d %d",
				&val[0], &val[1], &val[2], &val[3],
				&val[4], &val[5], &val[6], &val[7]);
			if (ret == 8) {
				dsi_conf->lane_num = (unsigned char)val[0];
				dsi_conf->bit_rate_max = val[1];
				dsi_conf->factor_numerator = val[2];
				dsi_conf->operation_mode_init =
					(unsigned char)val[3];
				dsi_conf->operation_mode_display =
					(unsigned char)val[4];
				dsi_conf->video_mode_type =
					(unsigned char)val[5];
				dsi_conf->clk_always_hs = (unsigned char)val[6];
				dsi_conf->phy_switch = (unsigned char)val[7];
				pr_info("change mipi_dsi config:\n"
			"lane_num=%d, bit_rate_max=%dMhz, factor_numerator=%d\n"
			"operation_mode_init=%d, operation_mode_display=%d\n"
			"video_mode_type=%d, clk_always_hs=%d, phy_switch=%d\n",
					dsi_conf->lane_num,
					dsi_conf->bit_rate_max,
					dsi_conf->factor_numerator,
					dsi_conf->operation_mode_init,
					dsi_conf->operation_mode_display,
					dsi_conf->video_mode_type,
					dsi_conf->clk_always_hs,
					dsi_conf->phy_switch);
				lcd_debug_change_clk_change(
					pconf->lcd_timing.lcd_clk);
				pconf->change_flag = 1;
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		} else if (buf[1] == 'l') {
			mlvds_conf = pconf->lcd_control.mlvds_config;
			ret = sscanf(buf, "mlvds %d %x %x %x %d %d",
				&val[0], &val[1], &val[2], &val[3],
				&val[4], &val[5]);
			if (ret == 6) {
				mlvds_conf->channel_num = val[0];
				mlvds_conf->channel_sel0 = val[1];
				mlvds_conf->channel_sel1 = val[2];
				mlvds_conf->clk_phase = val[3];
				mlvds_conf->pn_swap = val[4];
				mlvds_conf->bit_swap = val[5];
				pr_info("change mlvds config:\n"
					"channel_num=%d,\n"
				"channel_sel0=0x%08x, channel_sel1=0x%08x,\n"
					"clk_phase=0x%04x,\n"
					"pn_swap=%d, bit_swap=%d\n",
					mlvds_conf->channel_num,
					mlvds_conf->channel_sel0,
					mlvds_conf->channel_sel1,
					mlvds_conf->clk_phase,
					mlvds_conf->pn_swap,
					mlvds_conf->bit_swap);
				lcd_debug_change_clk_change(
					pconf->lcd_timing.lcd_clk);
				pconf->change_flag = 1;
			} else {
				LCDERR("invalid data\n");
				return -EINVAL;
			}
		}
		break;
	case 'p':
		p2p_conf = pconf->lcd_control.p2p_config;
		ret = sscanf(buf, "p2p %x %d %x %x %d %d",
			&val[0], &val[1], &val[2], &val[3], &val[4], &val[5]);
		if (ret == 6) {
			p2p_conf->p2p_type = val[0];
			p2p_conf->lane_num = val[1];
			p2p_conf->channel_sel0 = val[2];
			p2p_conf->channel_sel1 = val[3];
			p2p_conf->pn_swap = val[4];
			p2p_conf->bit_swap = val[5];
			pr_info("change p2p config:\n"
				"p2p_type=0x%x, lane_num=%d,\n"
				"channel_sel0=0x%08x, channel_sel1=0x%08x,\n"
				"pn_swap=%d, bit_swap=%d\n",
				p2p_conf->p2p_type, p2p_conf->lane_num,
				p2p_conf->channel_sel0, p2p_conf->channel_sel1,
				p2p_conf->pn_swap, p2p_conf->bit_swap);
			lcd_debug_change_clk_change(pconf->lcd_timing.lcd_clk);
			pconf->change_flag = 1;
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'u': /* update */
		if (pconf->change_flag) {
			LCDPR("apply config changing\n");
			lcd_debug_config_update();
		} else {
			LCDPR("config is no changing\n");
		}
		break;
	default:
		LCDERR("wrong command\n");
		break;
	}

	return count;
}

static ssize_t lcd_debug_enable_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return sprintf(buf, "lcd_status: %d\n",
		lcd_drv->lcd_status);
}

static ssize_t lcd_debug_enable_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int ret = 0;
	unsigned int temp = 1;

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		LCDERR("invalid data\n");
		return -EINVAL;
	}
	if (temp) {
		mutex_lock(&lcd_drv->power_mutex);
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_ON, NULL);
		lcd_if_enable_retry(lcd_drv->lcd_config);
		mutex_unlock(&lcd_drv->power_mutex);
	} else {
		mutex_lock(&lcd_drv->power_mutex);
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_OFF, NULL);
		mutex_unlock(&lcd_drv->power_mutex);
	}

	return count;
}

static ssize_t lcd_debug_resume_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return sprintf(buf, "lcd resume type: %d(%s)\n",
		lcd_drv->lcd_resume_type,
		lcd_drv->lcd_resume_type ? "workqueue" : "directly");
}

static ssize_t lcd_debug_resume_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int temp = 1;

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		LCDERR("invalid data\n");
		return -EINVAL;
	}
	lcd_drv->lcd_resume_type = (unsigned char)temp;
	LCDPR("set lcd resume flag: %d\n", lcd_drv->lcd_resume_type);

	return count;
}

static ssize_t lcd_debug_power_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int state;

	if ((lcd_drv->lcd_status & LCD_STATUS_ON) == 0) {
		state = 0;
	} else {
		if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
			state = 1;
		else
			state = 0;
	}
	return sprintf(buf, "lcd power state: %d\n", state);
}

static ssize_t lcd_debug_power_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int ret = 0;
	unsigned int temp = 1;

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		LCDERR("invalid data\n");
		return -EINVAL;
	}
	if (temp) {
		mutex_lock(&lcd_drv->power_mutex);
		aml_lcd_notifier_call_chain(LCD_EVENT_IF_POWER_ON, NULL);
		lcd_if_enable_retry(lcd_drv->lcd_config);
		mutex_unlock(&lcd_drv->power_mutex);
	} else {
		mutex_lock(&lcd_drv->power_mutex);
		aml_lcd_notifier_call_chain(LCD_EVENT_IF_POWER_OFF, NULL);
		mutex_unlock(&lcd_drv->power_mutex);
	}

	return count;
}

static ssize_t lcd_debug_power_step_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	char *print_buf;
	int n = 0;

	print_buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
	if (print_buf == NULL)
		return sprintf(buf, "%s: buf malloc error\n", __func__);

	lcd_power_step_info_print(print_buf, 0);

	n = sprintf(buf, "%s\n", print_buf);
	kfree(print_buf);

	return n;
}

static ssize_t lcd_debug_power_step_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int i;
	unsigned int tmp[2];
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_power_ctrl_s *lcd_power_step;

	lcd_power_step = lcd_drv->lcd_config->lcd_power;
	switch (buf[1]) {
	case 'n': /* on */
		ret = sscanf(buf, "on %d %d %d", &i, &tmp[0], &tmp[1]);
		if (ret == 3) {
			if (i >= lcd_power_step->power_on_step_max) {
				pr_info("invalid power_on step: %d, step_max: %d\n",
				i, lcd_power_step->power_on_step_max);
				return -EINVAL;
			}
			lcd_power_step->power_on_step[i].value = tmp[0];
			lcd_power_step->power_on_step[i].delay = tmp[1];
			pr_info(
				"set power_on step %d value %d delay: %dms\n",
				i, tmp[0], tmp[1]);
		} else if (ret == 2) {
			if (i >= lcd_power_step->power_on_step_max) {
				pr_info("invalid power_on step: %d\n", i);
				return -EINVAL;
			}
			lcd_power_step->power_on_step[i].delay = tmp[0];
			pr_info("set power_on step %d delay: %dms\n",
				i, tmp[0]);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'f': /* off */
		ret = sscanf(buf, "off %d %d %d\n", &i, &tmp[0], &tmp[1]);
		if (ret == 3) {
			if (i >= lcd_power_step->power_off_step_max) {
				pr_info("invalid power_off step: %d\n", i);
				return -EINVAL;
			}
			lcd_power_step->power_off_step[i].value = tmp[0];
			lcd_power_step->power_off_step[i].delay = tmp[1];
			pr_info(
				"set power_off step %d value %d delay: %dms\n",
				i, tmp[0], tmp[1]);
		} else if (ret == 2) {
			if (i >= lcd_power_step->power_off_step_max) {
				pr_info("invalid power_off step: %d\n", i);
				return -EINVAL;
			}
			lcd_power_step->power_off_step[i].delay = tmp[0];
			pr_info("set power_off step %d delay: %dms\n",
				i, tmp[0]);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		pr_info("wrong command\n");
		break;
	}

	return count;
}

static ssize_t lcd_debug_frame_rate_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	unsigned int sync_duration;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;

	pconf = lcd_drv->lcd_config;
	sync_duration = pconf->lcd_timing.sync_duration_num * 100;
	sync_duration = sync_duration / pconf->lcd_timing.sync_duration_den;

	return sprintf(buf, "get frame_rate: %u.%02uHz, fr_adjust_type: %d\n",
		(sync_duration / 100), (sync_duration % 100),
		pconf->lcd_timing.fr_adjust_type);
}

static ssize_t lcd_debug_frame_rate_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (buf[0]) {
	case 't':
		ret = sscanf(buf, "type %d", &temp);
		if (ret == 1) {
			lcd_drv->lcd_config->lcd_timing.fr_adjust_type = temp;
			pr_info("set fr_adjust_type: %d\n", temp);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 's':
		ret = sscanf(buf, "set %d", &temp);
		if (ret == 1) {
			pr_info("set frame rate(*100): %d\n", temp);
			aml_lcd_notifier_call_chain(
				LCD_EVENT_FRAME_RATE_ADJUST, &temp);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		pr_info("wrong command\n");
		break;
	}

	return count;
}

static ssize_t lcd_debug_fr_policy_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return sprintf(buf, "fr_auto_policy: %d\n", lcd_drv->fr_auto_policy);
}

static ssize_t lcd_debug_fr_policy_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		pr_info("invalid data\n");
		return -EINVAL;
	}
	lcd_drv->fr_auto_policy = temp;
	pr_info("set fr_auto_policy: %d\n", temp);

	return count;
}

static ssize_t lcd_debug_ss_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len;

	len = lcd_get_ss(buf);
	return len;
}

static ssize_t lcd_debug_ss_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int value = 0, temp;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	temp = lcd_drv->lcd_config->lcd_timing.ss_level;
	switch (buf[0]) {
	case 'l':
		ret = sscanf(buf, "level %d", &value);
		if (ret == 1) {
			value &= 0xff;
			ret = lcd_set_ss(value, 0xff, 0xff);
			if (ret == 0) {
				temp &= ~(0xff);
				temp |= value;
				lcd_drv->lcd_config->lcd_timing.ss_level = temp;
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'f':
		ret = sscanf(buf, "freq %d", &value);
		if (ret == 1) {
			value &= 0xf;
			ret = lcd_set_ss(0xff, value, 0xff);
			if (ret == 0) {
				temp &= ~((0xf << LCD_CLK_SS_BIT_FREQ) << 8);
				temp |= ((value << LCD_CLK_SS_BIT_FREQ) << 8);
				lcd_drv->lcd_config->lcd_timing.ss_level = temp;
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'm':
		ret = sscanf(buf, "mode %d", &value);
		if (ret == 1) {
			value &= 0xf;
			ret = lcd_set_ss(0xff, 0xff, value);
			if (ret == 0) {
				temp &= ~((0xf << LCD_CLK_SS_BIT_MODE) << 8);
				temp |= ((value << LCD_CLK_SS_BIT_MODE) << 8);
				lcd_drv->lcd_config->lcd_timing.ss_level = temp;
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		ret = kstrtouint(buf, 16, &value);
		if (ret) {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		value &= 0xffff;
		temp = value >> 8;
		ret = lcd_set_ss((value & 0xff),
			((temp >> LCD_CLK_SS_BIT_FREQ) & 0xf),
			((temp >> LCD_CLK_SS_BIT_MODE) & 0xf));
		if (ret == 0)
			lcd_drv->lcd_config->lcd_timing.ss_level = value;
		break;
	}

	return count;
}

static ssize_t lcd_debug_clk_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	char *print_buf;
	int n = 0;

	print_buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
	if (print_buf == NULL)
		return sprintf(buf, "%s: buf malloc error\n", __func__);

	lcd_clk_config_print(print_buf, 0);

	n = sprintf(buf, "%s\n", print_buf);
	kfree(print_buf);

	return n;
}

static ssize_t lcd_debug_clk_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (buf[0]) {
	case 'p':
		ret = sscanf(buf, "path %d", &temp);
		if (ret == 1) {
			ret = lcd_clk_path_change(temp);
			if (ret) {
				pr_info("change clk_path error\n");
			} else {
				lcd_drv->lcd_clk_path = temp;
				lcd_clk_generate_parameter(lcd_drv->lcd_config);
				pr_info("change clk_path: %d\n", temp);
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		pr_info("wrong command\n");
		break;
	}

	return count;
}

static ssize_t lcd_debug_test_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return sprintf(buf, "test pattern: %d\n", lcd_drv->lcd_test_state);
}

static ssize_t lcd_debug_test_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 0, i = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		pr_info("invalid data\n");
		return -EINVAL;
	}
	temp = (temp >= LCD_ENC_TST_NUM_MAX) ? 0 : temp;
	lcd_drv->lcd_test_flag = (unsigned char)(temp | LCD_TEST_UPDATE);

	while (i++ < 5000) {
		if (lcd_drv->lcd_test_state == temp)
			break;
		udelay(20);
	}

	return count;
}

static ssize_t lcd_debug_mute_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return sprintf(buf, "get lcd mute state: %d\n",
		lcd_drv->lcd_mute_state);
}

static ssize_t lcd_debug_mute_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 0, i = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		pr_info("invalid data\n");
		return -EINVAL;
	}
	temp = temp ? 1 : 0;
	lcd_drv->lcd_mute_flag = (unsigned char)(temp | LCD_MUTE_UPDATE);
	if (temp)
		LCDPR("set mute\n");
	else
		LCDPR("clear mute\n");
	while (i++ < 5000) {
		if (lcd_drv->lcd_mute_state == temp)
			break;
		udelay(20);
	}

	return count;
}

static void lcd_debug_reg_write(unsigned int reg, unsigned int data,
		unsigned int bus)
{
	switch (bus) {
	case 0: /* vcbus */
		lcd_vcbus_write(reg, data);
		pr_info("write vcbus [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, lcd_vcbus_read(reg));
		break;
	case 1: /* hiu */
		lcd_hiu_write(reg, data);
		pr_info("write hiu [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, lcd_hiu_read(reg));
		break;
	case 2: /* cbus */
		lcd_cbus_write(reg, data);
		pr_info("write cbus [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, lcd_cbus_read(reg));
		break;
	case 3: /* periphs */
		lcd_periphs_write(reg, data);
		pr_info("write periphs [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, lcd_periphs_read(reg));
		break;
	case 4: /* mipi_dsi_host */
		dsi_host_write(reg, data);
		pr_info("write mipi_dsi_host [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, dsi_host_read(reg));
		break;
	case 5: /* mipi_dsi_phy */
		dsi_phy_write(reg, data);
		pr_info("write mipi_dsi_phy [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, dsi_phy_read(reg));
		break;
	case 6: /* tcon */
		lcd_tcon_reg_write(reg, data);
		if (reg < TCON_TOP_BASE) {
			pr_info("write tcon [0x%04x] = 0x%02x, readback 0x%02x\n",
				reg, data, lcd_tcon_reg_read(reg));
		} else {
			pr_info("write tcon [0x%04x] = 0x%08x, readback 0x%08x\n",
			reg, data, lcd_tcon_reg_read(reg));
		}
		break;
	default:
		break;
	}
}

static void lcd_debug_reg_read(unsigned int reg, unsigned int bus)
{
	switch (bus) {
	case 0: /* vcbus */
		pr_info("read vcbus [0x%04x] = 0x%08x\n",
			reg, lcd_vcbus_read(reg));
		break;
	case 1: /* hiu */
		pr_info("read hiu [0x%04x] = 0x%08x\n",
			reg, lcd_hiu_read(reg));
		break;
	case 2: /* cbus */
		pr_info("read cbus [0x%04x] = 0x%08x\n",
			reg, lcd_cbus_read(reg));
		break;
	case 3: /* periphs */
		pr_info("read periphs [0x%04x] = 0x%08x\n",
			reg, lcd_periphs_read(reg));
		break;
	case 4: /* mipi_dsi_host */
		pr_info("read mipi_dsi_host [0x%04x] = 0x%08x\n",
			reg, dsi_host_read(reg));
		break;
	case 5: /* mipi_dsi_phy */
		pr_info("read mipi_dsi_phy [0x%04x] = 0x%08x\n",
			reg, dsi_phy_read(reg));
		break;
	case 6:
		if (reg < TCON_TOP_BASE) {
			pr_info("read tcon [0x%04x] = 0x%02x\n",
				reg, lcd_tcon_reg_read(reg));
		} else {
			pr_info("read tcon [0x%04x] = 0x%08x\n",
			reg, lcd_tcon_reg_read(reg));
		}
		break;
	default:
		break;
	}
}

static void lcd_debug_reg_dump(unsigned int reg, unsigned int num,
		unsigned int bus)
{
	int i;

	switch (bus) {
	case 0: /* vcbus */
		pr_info("dump vcbus regs:\n");
		for (i = 0; i < num; i++) {
			pr_info("[0x%04x] = 0x%08x\n",
				(reg + i), lcd_vcbus_read(reg + i));
		}
		break;
	case 1: /* hiu */
		pr_info("dump hiu-bus regs:\n");
		for (i = 0; i < num; i++) {
			pr_info("[0x%04x] = 0x%08x\n",
				(reg + i), lcd_hiu_read(reg + i));
		}
		break;
	case 2: /* cbus */
		pr_info("dump cbus regs:\n");
		for (i = 0; i < num; i++) {
			pr_info("[0x%04x] = 0x%08x\n",
				(reg + i), lcd_cbus_read(reg + i));
		}
		break;
	case 3: /* periphs */
		pr_info("dump periphs-bus regs:\n");
		for (i = 0; i < num; i++) {
			pr_info("[0x%04x] = 0x%08x\n",
				(reg + i), lcd_periphs_read(reg + i));
		}
		break;
	case 4: /* mipi_dsi_host */
		pr_info("dump mipi_dsi_host regs:\n");
		for (i = 0; i < num; i++) {
			pr_info("[0x%04x] = 0x%08x\n",
				(reg + i), dsi_host_read(reg + i));
		}
		break;
	case 5: /* mipi_dsi_phy */
		pr_info("dump mipi_dsi_phy regs:\n");
		for (i = 0; i < num; i++) {
			pr_info("[0x%04x] = 0x%08x\n",
				(reg + i), dsi_phy_read(reg + i));
		}
		break;
	case 6:
		pr_info("dump tcon regs:\n");
		if (reg < TCON_TOP_BASE) {
			for (i = 0; i < num; i++) {
				pr_info("[0x%04x] = 0x%02x\n",
					(reg + i), lcd_tcon_reg_read(reg + i));
			}
		} else {
			for (i = 0; i < num; i++) {
				pr_info("[0x%04x] = 0x%08x\n",
					(reg + i), lcd_tcon_reg_read(reg + i));
			}
		}
		break;
	default:
		break;
	}
}

static ssize_t lcd_debug_reg_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int bus = 0;
	unsigned int reg32 = 0, data32 = 0;

	switch (buf[0]) {
	case 'w':
		if (buf[1] == 'v') {
			ret = sscanf(buf, "wv %x %x", &reg32, &data32);
			bus = 0;
		} else if (buf[1] == 'h') {
			ret = sscanf(buf, "wh %x %x", &reg32, &data32);
			bus = 1;
		} else if (buf[1] == 'c') {
			ret = sscanf(buf, "wc %x %x", &reg32, &data32);
			bus = 2;
		} else if (buf[1] == 'p') {
			ret = sscanf(buf, "wp %x %x", &reg32, &data32);
			bus = 3;
		} else if (buf[1] == 'm') {
			if (buf[2] == 'h') { /* mipi host */
				ret = sscanf(buf, "wmh %x %x", &reg32, &data32);
				bus = 4;
			} else if (buf[2] == 'p') { /* mipi phy */
				ret = sscanf(buf, "wmp %x %x", &reg32, &data32);
				bus = 5;
			}
		} else if (buf[1] == 't') {
			ret = sscanf(buf, "wt %x %x", &reg32, &data32);
			bus = 6;
		}
		if (ret == 2) {
			lcd_debug_reg_write(reg32, data32, bus);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'r':
		if (buf[1] == 'v') {
			ret = sscanf(buf, "rv %x", &reg32);
			bus = 0;
		} else if (buf[1] == 'h') {
			ret = sscanf(buf, "rh %x", &reg32);
			bus = 1;
		} else if (buf[1] == 'c') {
			ret = sscanf(buf, "rc %x", &reg32);
			bus = 2;
		} else if (buf[1] == 'p') {
			ret = sscanf(buf, "rp %x", &reg32);
			bus = 3;
		} else if (buf[1] == 'm') {
			if (buf[2] == 'h') { /* mipi host */
				ret = sscanf(buf, "rmh %x", &reg32);
				bus = 4;
			} else if (buf[2] == 'p') { /* mipi phy */
				ret = sscanf(buf, "rmp %x", &reg32);
				bus = 5;
			}
		} else if (buf[1] == 't') {
			ret = sscanf(buf, "rt %x", &reg32);
			bus = 6;
		}
		if (ret == 1) {
			lcd_debug_reg_read(reg32, bus);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'd':
		if (buf[1] == 'v') {
			ret = sscanf(buf, "dv %x %d", &reg32, &data32);
			bus = 0;
		} else if (buf[1] == 'h') {
			ret = sscanf(buf, "dh %x %d", &reg32, &data32);
			bus = 1;
		} else if (buf[1] == 'c') {
			ret = sscanf(buf, "dc %x %d", &reg32, &data32);
			bus = 2;
		} else if (buf[1] == 'p') {
			ret = sscanf(buf, "dp %x %d", &reg32, &data32);
			bus = 3;
		} else if (buf[1] == 'm') {
			if (buf[2] == 'h') { /* mipi host */
				ret = sscanf(buf, "dmh %x %d", &reg32, &data32);
				bus = 4;
			} else if (buf[2] == 'p') { /* mipi phy */
				ret = sscanf(buf, "dmp %x %d", &reg32, &data32);
				bus = 5;
			}
		} else if (buf[1] == 't') {
			ret = sscanf(buf, "dt %x %d", &reg32, &data32);
			bus = 6;
		}
		if (ret == 2) {
			lcd_debug_reg_dump(reg32, data32, bus);
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		pr_info("wrong command\n");
		break;
	}

	return count;
}

static unsigned int lcd_dither_en = 1;
static unsigned int lcd_dither_round_en = 0;
static unsigned int lcd_dither_md = 0;
static void lcd_vpu_dither_setting(unsigned int lcd_dither_en,
		unsigned int lcd_dither_round_en, unsigned int lcd_dither_md)
{
	unsigned int data32;

	data32 = lcd_vcbus_read(VPU_VENCL_DITH_CTRL);
	data32 &= ~((1 << 0) | (1 << 1) | (1 << 2));
	data32 |= ((lcd_dither_en << 0) |
		(lcd_dither_round_en << 1) |
		(lcd_dither_md << 2));
	lcd_vcbus_write(VPU_VENCL_DITH_CTRL, data32);
}

static ssize_t lcd_debug_dither_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	ssize_t len = 0;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TXLX:
		len = sprintf(buf, "get dither status:\n"
			"dither_en:        %d\n"
			"dither_round_en:  %d\n"
			"dither_md:        %d\n",
			lcd_dither_en, lcd_dither_round_en, lcd_dither_md);
		break;
	default:
		len = sprintf(buf,
			"don't support dither function for current chip\n");
		break;
	}

	return len;
}

static ssize_t lcd_debug_dither_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int ret = -1;
	unsigned int temp = 0;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TXLX:
		ret = 0;
		break;
	default:
		ret = -1;
		LCDPR("don't support dither function for current chip\n");
		break;
	}
	if (ret)
		return count;

	switch (buf[0]) {
	case 'e': /* en */
		ret = sscanf(buf, "en %d", &temp);
		if (ret == 1) {
			if (temp)
				lcd_dither_en = 1;
			else
				lcd_dither_en = 0;
			lcd_vpu_dither_setting(lcd_dither_en,
				lcd_dither_round_en, lcd_dither_md);
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'r': /* round */
		ret = sscanf(buf, "round %d", &temp);
		if (ret == 1) {
			if (temp)
				lcd_dither_round_en = 1;
			else
				lcd_dither_round_en = 0;
			lcd_vpu_dither_setting(lcd_dither_en,
				lcd_dither_round_en, lcd_dither_md);
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'm': /* md */
		ret = sscanf(buf, "method %d", &temp);
		if (ret == 1) {
			if (temp)
				lcd_dither_md = 1;
			else
				lcd_dither_md = 0;
			lcd_vpu_dither_setting(lcd_dither_en,
				lcd_dither_round_en, lcd_dither_md);
		} else {
			LCDERR("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		LCDERR("wrong command\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t lcd_debug_vlock_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	ssize_t len = 0;

	len = sprintf(buf, "custome vlock attr:\n"
		"vlock_valid:        %d\n"
		"vlock_en:           %d\n"
		"vlock_work_mode:    %d\n"
		"vlock_pll_m_limit:  %d\n"
		"vlock_line_limit:   %d\n",
		lcd_drv->lcd_config->lcd_control.vlock_param[0],
		lcd_drv->lcd_config->lcd_control.vlock_param[1],
		lcd_drv->lcd_config->lcd_control.vlock_param[2],
		lcd_drv->lcd_config->lcd_control.vlock_param[3],
		lcd_drv->lcd_config->lcd_control.vlock_param[4]);

	return len;
}

#define LCD_DEBUG_DUMP_INFO         0
#define LCD_DEBUG_DUMP_REG          1
#define LCD_DEBUG_DUMP_HDR          2
static int lcd_debug_dump_state;
static ssize_t lcd_debug_dump_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	char *print_buf;
	int len = 0;

	print_buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
	if (print_buf == NULL)
		return sprintf(buf, "%s: buf malloc error\n", __func__);

	switch (lcd_debug_dump_state) {
	case LCD_DEBUG_DUMP_INFO:
		len = lcd_info_print(print_buf, 0);
		lcd_power_step_info_print((print_buf+len), len);
		break;
	case LCD_DEBUG_DUMP_REG:
		lcd_reg_print(print_buf, 0);
		break;
	case LCD_DEBUG_DUMP_HDR:
		lcd_optical_info_print(print_buf, 0);
		break;
	default:
		sprintf(print_buf, "%s: invalid command\n", __func__);
		break;
	}
	len = sprintf(buf, "%s\n", print_buf);
	kfree(print_buf);

	return len;
}

static ssize_t lcd_debug_dump_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	char *buf_orig;
	char *parm[47] = {NULL};

	if (!buf)
		return count;
	buf_orig = kstrdup(buf, GFP_KERNEL);
	if (buf_orig == NULL) {
		LCDERR("%s: buf malloc error\n", __func__);
		return count;
	}
	lcd_debug_parse_param(buf_orig, (char **)&parm);

	if (strcmp(parm[0], "info") == 0)
		lcd_debug_dump_state = LCD_DEBUG_DUMP_INFO;
	else if (strcmp(parm[0], "reg") == 0)
		lcd_debug_dump_state = LCD_DEBUG_DUMP_REG;
	else if (strcmp(parm[0], "hdr") == 0)
		lcd_debug_dump_state = LCD_DEBUG_DUMP_HDR;
	else {
		LCDERR("invalid command\n");
		kfree(buf_orig);
		return -EINVAL;
	}

	kfree(buf_orig);
	return count;
}

static ssize_t lcd_debug_print_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "get debug print flag: %d\n", lcd_debug_print_flag);
}

static ssize_t lcd_debug_print_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int temp = 0;

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		pr_info("invalid data\n");
		return -EINVAL;
	}
	lcd_debug_print_flag = (unsigned char)temp;
	LCDPR("set debug print flag: %d\n", lcd_debug_print_flag);

	return count;
}

static struct class_attribute lcd_debug_class_attrs[] = {
	__ATTR(help,        0444, lcd_debug_common_help, NULL),
	__ATTR(debug,       0644, lcd_debug_show, lcd_debug_store),
	__ATTR(change,      0644, lcd_debug_change_show,
		lcd_debug_change_store),
	__ATTR(enable,      0644,
		lcd_debug_enable_show, lcd_debug_enable_store),
	__ATTR(resume_type, 0644,
		lcd_debug_resume_show, lcd_debug_resume_store),
	__ATTR(power,       0644,  lcd_debug_power_show,
		lcd_debug_power_store),
	__ATTR(power_step,       0644, lcd_debug_power_step_show,
		 lcd_debug_power_step_store),
	__ATTR(frame_rate,  0644,
		lcd_debug_frame_rate_show, lcd_debug_frame_rate_store),
	__ATTR(fr_policy,   0644,
		lcd_debug_fr_policy_show, lcd_debug_fr_policy_store),
	__ATTR(ss,          0644, lcd_debug_ss_show, lcd_debug_ss_store),
	__ATTR(clk,         0644, lcd_debug_clk_show, lcd_debug_clk_store),
	__ATTR(test,        0644, lcd_debug_test_show, lcd_debug_test_store),
	__ATTR(mute,        0644, lcd_debug_mute_show, lcd_debug_mute_store),
	__ATTR(prbs,        0644, lcd_debug_prbs_show, lcd_debug_prbs_store),
	__ATTR(reg,         0200, NULL, lcd_debug_reg_store),
	__ATTR(dither,      0644,
		lcd_debug_dither_show, lcd_debug_dither_store),
	__ATTR(vlock,       0644, lcd_debug_vlock_show, NULL),
	__ATTR(dump,        0644,
		lcd_debug_dump_show, lcd_debug_dump_store),
	__ATTR(print,       0644, lcd_debug_print_show, lcd_debug_print_store),
};

static const char *lcd_ttl_debug_usage_str = {
"Usage:\n"
"    echo <clk_pol> <de_valid> <hvsync_valid> <rb_swpa> <bit_swap> > ttl ; set ttl config\n"
"data format:\n"
"    <clk_pol>      : 0=negative, 1=positive\n"
"    <de_valid>     : for DE, 0=invalid, 1=valid\n"
"    <hvsync_valid> : for hvsync, 0=invalid, 1=valid\n"
"    <rb_swpa>      : for R/B port, 0=normal, 1=swap\n"
"    <bit_swap>     : for RGB MSB/LSB, 0=normal, 1=swap\n"
"\n"
};

static const char *lcd_lvds_debug_usage_str = {
"Usage:\n"
"    echo <repack> <dual_port> <pn_swap> <port_swap> <lane_reverse> > lvds ; set lvds config\n"
"data format:\n"
"    <repack>    : 0=JEIDA mode, 1=VESA mode(8bit), 2=VESA mode(10bit)\n"
"    <dual_port> : 0=single port, 1=dual port\n"
"    <pn_swap>   : 0=normal, 1=swap p/n channels\n"
"    <port_swap> : 0=normal, 1=swap A/B port\n"
"	 <lane_reverse> : 0=normal, 1=swap A0-A4/B0-B4\n"
"\n"
"    echo <vswing> <preem> > phy ; set vbyone phy config\n"
"data format:\n"
"    <vswing> : vswing level, support 0~7\n"
"    <preem>  : preemphasis level, support 0~7\n"
"\n"
};

static const char *lcd_vbyone_debug_usage_str = {
"Usage:\n"
"    echo <lane_count> <region_num> <byte_mode> > vbyone ; set vbyone config\n"
"data format:\n"
"    <lane_count> : 4/8/16\n"
"    <region_num> : 1/2\n"
"    <byte_mode>  : 3/4/5\n"
"\n"
"    echo <vswing> <preem> > phy ; set vbyone phy config\n"
"data format:\n"
"    <vswing> : vswing level, support 0~7\n"
"    <preem>  : preemphasis level, support 0~7\n"
"    <byte_mode>  : 3/4/5\n"
"\n"
"    echo intr <state> <en> > vbyone; enable or disable vbyone interrupt\n"
"data format:\n"
"    <state> : 0=temp no use intr, 1=temp use intr. keep effect until reset lcd driver\n"
"    <en>    : 0=disable intr, 1=enable intr\n"
"\n"
"    echo vintr <en> > vbyone; enable or disable vbyone interrupt\n"
"data format:\n"
"    <en>    : 0=disable vsync monitor intr, 1=enable vsync monitor intr\n"
"\n"
"    echo ctrl <ctrl_flag> <power_on_reset_delay> <hpd_data_delay> <cdr_training_hold> > vbyone; set ctrl adjust\n"
"data format:\n"
"    <ctrl_flag>    : bit[0]:power_on_reset_en, bit[1]:hpd_data_delay_en, bit[2]:cdr_training_hold_en\n"
"    others         : unit in ms\n"
"\n"
};

static const char *lcd_mipi_debug_usage_str = {
"Usage:\n"
"    echo <lane_num> <bit_rate_max> <factor> <op_mode_init> <op_mode_disp> <vid_mode_type> <clk_always_hs> <phy_switch> > mipi ; set mpi config\n"
"data format:\n"
"    <lane_num>          : 1/2/3/4\n"
"    <bit_rate_max>      : unit in MHz\n"
"    <factor>:           : special adjust, 0 for default\n"
"    <op_mode_init>      : operation mode for init (0=video mode, 1=command mode)\n"
"    <op_mode_disp>      : operation mode for display (0=video mode, 1=command mode)\n"
"    <vid_mode_type>     : video mode type (0=sync_pulse, 1=sync_event, 2=burst)\n"
"    <clk_always_hs>     : 0=disable, 1=enable\n"
"    <phy_switch>        : 0=auto, 1=standard, 2=slow\n"
"\n"
};

static const char *lcd_mipi_cmd_debug_usage_str = {
"Usage:\n"
"   echo <data_type> <N> <data0> <data1> <data2> ...... <dataN-1> > mpcmd ; send mipi cmd\n"
"   support data_type:\n"
"	DT_SHUT_DOWN            = 0x22\n"
"	DT_TURN_ON              = 0x32\n"
"	DT_GEN_SHORT_WR_0       = 0x03\n"
"	DT_GEN_SHORT_WR_1       = 0x13\n"
"	DT_GEN_SHORT_WR_2       = 0x23\n"
"	DT_DCS_SHORT_WR_0       = 0x05\n"
"	DT_DCS_SHORT_WR_1       = 0x15\n"
"	DT_GEN_LONG_WR          = 0x29\n"
"	DT_DCS_LONG_WR          = 0x39\n"
"\n"
};

static const char *lcd_mlvds_debug_usage_str = {
"Usage:\n"
"    echo <channel_num> <channel_sel0> <channel_sel1> <clk_phase> <pn_swap> <bit_swap> > minilvds ; set minilvds config\n"
"data format:\n"
"    <channel_sel> : minilvds 8 channels mapping in tx 10 channels\n"
"    <clk_phase>   : bit[13:12]=clk01_pi_sel, bit[11:8]=pi2, bit[7:4]=pi1, bit[3:0]=pi0\n"
"    <pn_swap>     : 0=normal, 1=swap p/n channels\n"
"    <bit_swap>    : 0=normal, 1=swap bit LSB/MSB\n"
"\n"
};

static const char *lcd_p2p_debug_usage_str = {
"Usage:\n"
"    echo <p2p_type> <lane_num> <channel_sel0> <channel_sel1> <pn_swap> <bit_swap> > p2p ; set p2p config\n"
"data format:\n"
"    <p2p_type>    : 0x0=ceds, 0x1=cmpi, 0x2=isp, 0x3=epi,\n"
"                    0x10=chpi, 0x11=cspi, 0x12=usit\n"
"    <channel_sel> : 12 channels mapping\n"
"    <pn_swap>     : 0=normal, 1=swap p/n channels\n"
"    <bit_swap>    : 0=normal, 1=swap bit LSB/MSB\n"
"\n"
};

static const char *lcd_debug_tcon_usage_str = {
	"Usage:\n"
	"    echo reg > tcon ; print tcon system regs\n"
	"    echo reg save <path> > tcon ; save tcon system regs to bin file\n"
	"\n"
	"    echo table > tcon ; print tcon reg table\n"
	"    echo table r <index> > tcon ; read tcon reg table by specified index\n"
	"    echo table w <index> <value> > tcon ; write tcon reg table by specified index\n"
	"    echo table d <index> <len> > tcon ; dump tcon reg table\n"
	"data format:\n"
	"    <index>    : hex number\n"
	"    <value>    : hex number\n"
	"    <len>      : dec number\n"
	"\n"
	"    echo table update > tcon ; update tcon reg table into tcon system regs\n"
	"    echo table save <path> > tcon ; save tcon reg table to bin file\n"
	"\n"
	"    echo od <en> > tcon ; tcon over driver control\n"
	"data format:\n"
	"    <en>       : 0=disable, 1=enable\n"
};

static ssize_t lcd_ttl_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len = 0;

	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct ttl_config_s *ttl_conf;

	ttl_conf = lcd_drv->lcd_config->lcd_control.ttl_config;

	len += sprintf(buf+len,
		"ttl config: clk_pol=%d, de_valid=%d, hvsync_valid=%d,",
		ttl_conf->clk_pol,
		(ttl_conf->sync_valid >> 1) & 0x1,
		(ttl_conf->sync_valid >> 0) & 0x1);
	len += sprintf(buf+len, "rb_swap=%d, bit_swap=%d\n\n",
		(ttl_conf->swap_ctrl >> 1) & 0x1,
		(ttl_conf->swap_ctrl >> 0) & 0x1);
	len += sprintf(buf+len, "%s\n", lcd_ttl_debug_usage_str);

	return len;
}

static ssize_t lcd_lvds_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lvds_config_s *lvds_conf;

	lvds_conf = lcd_drv->lcd_config->lcd_control.lvds_config;

	len += sprintf(buf+len, "lvds config: repack=%d, dual_port=%d,",
		lvds_conf->lvds_repack, lvds_conf->dual_port);
	len += sprintf(buf+len, "pn_swap=%d, port_swap=%d, lane_reverse=%d\n\n",
		lvds_conf->pn_swap, lvds_conf->port_swap,
		lvds_conf->lane_reverse);
	len += sprintf(buf+len, "%s\n", lcd_lvds_debug_usage_str);

	return len;
}

static ssize_t lcd_vx1_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vx1_conf;

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;

	len += sprintf(buf+len, "vbyone config: lane_count=%d,",
		vx1_conf->lane_count);
	len += sprintf(buf+len, "region_num=%d, byte_mode=%d\n\n",
		vx1_conf->region_num, vx1_conf->byte_mode);
	len += sprintf(buf+len, "%s\n", lcd_vbyone_debug_usage_str);

	return len;
}

static ssize_t lcd_mipi_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct dsi_config_s *dsi_conf;

	dsi_conf = lcd_drv->lcd_config->lcd_control.mipi_config;

	len += sprintf(buf+len, "mipi_dsi config: lane_num=%d, ",
		dsi_conf->lane_num);
	len += sprintf(buf+len, "bit_rate_max=%dMhz, factor_numerator=%d, ",
		dsi_conf->bit_rate_max, dsi_conf->factor_numerator);
	len += sprintf(buf+len,
		"operation_mode_init=%d, operation_mode_display=%d, ",
		dsi_conf->operation_mode_init,
		dsi_conf->operation_mode_display);
	len += sprintf(buf+len,
		"video_mode_type=%d, clk_always_hs=%d, phy_switch=%d\n\n",
		dsi_conf->video_mode_type, dsi_conf->clk_always_hs,
		dsi_conf->phy_switch);
	len += sprintf(buf+len, "%s\n", lcd_mipi_debug_usage_str);

	return len;
}

static ssize_t lcd_mlvds_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct mlvds_config_s *mlvds_conf;

	mlvds_conf = lcd_drv->lcd_config->lcd_control.mlvds_config;

	len += sprintf(buf+len, "minilvds config: channel_num=%d, ",
		mlvds_conf->channel_num);
	len += sprintf(buf+len, "channel_sel0=0x%08x, channel_sel1=0x%08x, ",
		mlvds_conf->channel_sel0, mlvds_conf->channel_sel1);
	len += sprintf(buf+len, "clk_phase=0x%04x, pn_swap=%d, bit_swap=%d\n\n",
		mlvds_conf->clk_phase,
		mlvds_conf->pn_swap, mlvds_conf->bit_swap);
	len += sprintf(buf+len, "%s\n", lcd_mlvds_debug_usage_str);

	return len;
}

static ssize_t lcd_p2p_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int len = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct p2p_config_s *p2p_conf;

	p2p_conf = lcd_drv->lcd_config->lcd_control.p2p_config;

	len += sprintf(buf+len, "p2p config: p2p_type=0x%x, lane_num=%d, ",
		p2p_conf->p2p_type, p2p_conf->lane_num);
	len += sprintf(buf+len, "channel_sel0=0x%08x, channel_sel1=0x%08x, ",
		p2p_conf->channel_sel0, p2p_conf->channel_sel1);
	len += sprintf(buf+len, "pn_swap=%d, bit_swap=%d\n\n",
		p2p_conf->pn_swap, p2p_conf->bit_swap);
	len += sprintf(buf+len, "%s\n", lcd_p2p_debug_usage_str);

	return len;
}

static ssize_t lcd_tcon_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_debug_tcon_usage_str);
}

static ssize_t lcd_tcon_status_show(struct class *class,
				    struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	return sprintf(buf, "%d\n", lcd_drv->tcon_status);
}

static ssize_t lcd_ttl_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct ttl_config_s *ttl_conf;
	unsigned int temp[5];

	ttl_conf = lcd_drv->lcd_config->lcd_control.ttl_config;
	ret = sscanf(buf, "%d %d %d %d %d",
		&temp[0], &temp[1], &temp[2], &temp[3], &temp[4]);
	if (ret == 5) {
		pr_info("set ttl config:\n"
			"clk_pol=%d, de_valid=%d, hvsync_valid=%d\n"
			"rb_swap=%d, bit_swap=%d\n",
			temp[0], temp[1], temp[2], temp[3], temp[4]);
		ttl_conf->clk_pol = temp[0];
		ttl_conf->sync_valid = ((temp[1] << 1) | temp[2]);
		ttl_conf->swap_ctrl = ((temp[3] << 1) | temp[4]);
		lcd_debug_config_update();
	} else {
		pr_info("invalid data\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t lcd_lvds_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lvds_config_s *lvds_conf;

	lvds_conf = lcd_drv->lcd_config->lcd_control.lvds_config;
	ret = sscanf(buf, "%d %d %d %d %d",
		&lvds_conf->lvds_repack, &lvds_conf->dual_port,
		&lvds_conf->pn_swap, &lvds_conf->port_swap,
		&lvds_conf->lane_reverse);
	if (ret == 5 || ret == 4) {
		pr_info("set lvds config:\n"
			"repack=%d, dual_port=%d, pn_swap=%d, port_swap=%d, lane_reverse=%d\n",
			lvds_conf->lvds_repack, lvds_conf->dual_port,
			lvds_conf->pn_swap, lvds_conf->port_swap,
			lvds_conf->lane_reverse);
		lcd_debug_config_update();
	} else {
		pr_info("invalid data\n");
		return -EINVAL;
	}

	return count;
}

#ifdef CONFIG_AMLOGIC_LCD_TV
static int vx1_intr_state = 1;
#endif
static ssize_t lcd_vx1_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vx1_conf;
#ifdef CONFIG_AMLOGIC_LCD_TV
	int val[5];
#endif

	vx1_conf = lcd_drv->lcd_config->lcd_control.vbyone_config;
	if (buf[0] == 'i') { /* intr */
#ifdef CONFIG_AMLOGIC_LCD_TV
		ret = sscanf(buf, "intr %d %d", &val[0], &val[1]);
		if (ret == 1) {
			pr_info("set vbyone interrupt enable: %d\n", val[0]);
			vx1_intr_state = val[0];
			lcd_vbyone_interrupt_enable(vx1_intr_state);
		} else if (ret == 2) {
			pr_info("set vbyone interrupt enable: %d %d\n",
				val[0], val[1]);
			vx1_intr_state = val[0];
			vx1_conf->intr_en = val[1];
			lcd_vbyone_interrupt_enable(vx1_intr_state);
		} else {
			pr_info("vx1_intr_enable: %d %d\n",
				vx1_intr_state, vx1_conf->intr_en);
			return -EINVAL;
		}
#else
		return -EINVAL;
#endif
	} else if (buf[0] == 'v') { /* vintr */
#ifdef CONFIG_AMLOGIC_LCD_TV
		ret = sscanf(buf, "vintr %d", &val[0]);
		if (ret == 1) {
			pr_info("set vbyone vsync interrupt enable: %d\n",
				val[0]);
			vx1_conf->vsync_intr_en = val[0];
			lcd_vbyone_interrupt_enable(vx1_intr_state);
		} else {
			pr_info("vx1_vsync_intr_enable: %d\n",
				vx1_conf->vsync_intr_en);
			return -EINVAL;
		}
#else
		return -EINVAL;
#endif
	} else if (buf[0] == 'c') { /* ctrl */
#ifdef CONFIG_AMLOGIC_LCD_TV
		ret = sscanf(buf, "ctrl %x %d %d %d",
			&val[0], &val[1], &val[2], &val[3]);
		if (ret == 4) {
			pr_info("set vbyone ctrl_flag: 0x%x\n", val[0]);
			pr_info("power_on_reset_delay: %dms\n", val[1]);
			pr_info("hpd_data_delay: %dms\n", val[2]);
			pr_info("cdr_training_hold: %dms\n", val[3]);
			vx1_conf->ctrl_flag = val[0];
			vx1_conf->power_on_reset_delay = val[1];
			vx1_conf->hpd_data_delay = val[2];
			vx1_conf->cdr_training_hold = val[3];
			lcd_debug_config_update();
		} else {
			pr_info("vbyone ctrl_flag: 0x%x\n",
				vx1_conf->ctrl_flag);
			pr_info("power_on_reset_delay: %dms\n",
				vx1_conf->power_on_reset_delay);
			pr_info("hpd_data_delay: %dms\n",
				vx1_conf->hpd_data_delay);
			pr_info("cdr_training_hold: %dms\n",
				vx1_conf->cdr_training_hold);
			return -EINVAL;
		}
#else
		return -EINVAL;
#endif
	} else if (buf[0] == 'f') { /* filter */
		ret = sscanf(buf, "filter %x %x", &val[0], &val[1]);
		if (ret == 2) {
			pr_info("set vbyone hw_filter_time: 0x%x, hw_filter_cnt: 0x%x\n",
				val[0], val[1]);
			vx1_conf->hw_filter_time = val[0];
			vx1_conf->hw_filter_cnt = val[1];
			lcd_debug_config_update();
		} else {
			pr_info("vbyone hw_filter_time: 0x%x, hw_filter_cnt: 0x%x\n",
				vx1_conf->hw_filter_time,
				vx1_conf->hw_filter_cnt);
			return -EINVAL;
		}
	} else {
		ret = sscanf(buf, "%d %d %d", &vx1_conf->lane_count,
			&vx1_conf->region_num, &vx1_conf->byte_mode);
		if (ret == 3) {
			pr_info("set vbyone config:\n"
				"lane_count=%d, region_num=%d, byte_mode=%d\n",
				vx1_conf->lane_count, vx1_conf->region_num,
				vx1_conf->byte_mode);
			lcd_debug_config_update();
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
	}

	return count;
}

static ssize_t lcd_mipi_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct dsi_config_s *dsi_conf;
	int val[8];

	dsi_conf = lcd_drv->lcd_config->lcd_control.mipi_config;
	ret = sscanf(buf, "%d %d %d %d %d %d %d %d",
		&val[0], &val[1], &val[2], &val[3],
		&val[4], &val[5], &val[6], &val[7]);
	if (ret >= 2) {
		dsi_conf->lane_num = (unsigned char)val[0];
		dsi_conf->bit_rate_max = val[1];
		dsi_conf->factor_numerator = val[2];
		dsi_conf->operation_mode_init = (unsigned char)val[3];
		dsi_conf->operation_mode_display = (unsigned char)val[4];
		dsi_conf->video_mode_type = (unsigned char)val[5];
		dsi_conf->clk_always_hs = (unsigned char)val[6];
		dsi_conf->phy_switch = (unsigned char)val[7];
		pr_info("set mipi_dsi config:\n"
			"lane_num=%d, bit_rate_max=%dMhz, factor_numerator=%d\n"
			"operation_mode_init=%d, operation_mode_display=%d\n"
			"video_mode_type=%d\n"
			"clk_always_hs=%d, phy_switch=%d\n\n",
			dsi_conf->lane_num,
			dsi_conf->bit_rate_max,
			dsi_conf->factor_numerator,
			dsi_conf->operation_mode_init,
			dsi_conf->operation_mode_display,
			dsi_conf->video_mode_type,
			dsi_conf->clk_always_hs,
			dsi_conf->phy_switch);
		lcd_debug_config_update();
	} else {
		pr_info("invalid data\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t lcd_mlvds_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct mlvds_config_s *mlvds_conf;

	mlvds_conf = lcd_drv->lcd_config->lcd_control.mlvds_config;
	ret = sscanf(buf, "%d %x %x %x %d %d",
		&mlvds_conf->channel_num,
		&mlvds_conf->channel_sel0, &mlvds_conf->channel_sel1,
		&mlvds_conf->clk_phase,
		&mlvds_conf->pn_swap, &mlvds_conf->bit_swap);
	if (ret == 6) {
		pr_info("set minilvds config:\n"
			"channel_num=%d,\n"
			"channel_sel0=0x%08x, channel_sel1=0x%08x,\n"
			"clk_phase=0x%04x,\n"
			"pn_swap=%d, bit_swap=%d\n",
			mlvds_conf->channel_num,
			mlvds_conf->channel_sel0, mlvds_conf->channel_sel1,
			mlvds_conf->clk_phase,
			mlvds_conf->pn_swap, mlvds_conf->bit_swap);
		lcd_debug_config_update();
	} else {
		pr_info("invalid data\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t lcd_p2p_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct p2p_config_s *p2p_conf;

	p2p_conf = lcd_drv->lcd_config->lcd_control.p2p_config;
	ret = sscanf(buf, "%x %d %x %x %d %d",
		&p2p_conf->p2p_type, &p2p_conf->lane_num,
		&p2p_conf->channel_sel0, &p2p_conf->channel_sel1,
		&p2p_conf->pn_swap, &p2p_conf->bit_swap);
	if (ret == 6) {
		pr_info("set p2p config:\n"
			"p2p_type=0x%x, lane_num=%d,\n"
			"channel_sel0=0x%08x, channel_sel1=0x%08x,\n"
			"pn_swap=%d, bit_swap=%d\n",
			p2p_conf->p2p_type, p2p_conf->lane_num,
			p2p_conf->channel_sel0, p2p_conf->channel_sel1,
			p2p_conf->pn_swap, p2p_conf->bit_swap);
		lcd_debug_config_update();
	} else {
		pr_info("invalid data\n");
		return -EINVAL;
	}

	return count;
}

static void lcd_phy_config_update(unsigned int *para, int cnt)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	struct lvds_config_s *lvds_conf;

	pconf = lcd_drv->lcd_config;
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		lvds_conf = pconf->lcd_control.lvds_config;
		if (cnt == 4) {
			lvds_conf->phy_vswing = para[0];
			lvds_conf->phy_preem = para[1];
			lvds_conf->phy_clk_vswing = para[2];
			lvds_conf->phy_clk_preem = para[3];

			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_lvds_phy_set(pconf, 1);

			LCDPR("%s:\n", __func__);
			pr_info("vswing=0x%x, preemphasis=0x%x\n",
				para[0], para[1]);
			pr_info("clk_vswing=0x%x, clk_preem=0x%x\n",
				para[2], para[3]);
		} else if (cnt == 2) {
			lvds_conf->phy_vswing = para[0];
			lvds_conf->phy_preem = para[1];

			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_lvds_phy_set(pconf, 1);

			LCDPR("%s: vswing=0x%x, preemphasis=0x%x\n",
				__func__, para[0], para[1]);
		} else {
			LCDERR("%s: invalid parameters cnt: %d\n",
				__func__, cnt);
		}
		break;
	case LCD_VBYONE:
		if (cnt >= 2) {
			pconf->lcd_control.vbyone_config->phy_vswing = para[0];
			pconf->lcd_control.vbyone_config->phy_preem = para[1];

			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_vbyone_phy_set(pconf, 1);

			LCDPR("%s: vswing=0x%x, preemphasis=0x%x\n",
				__func__, para[0], para[1]);
		} else {
			LCDERR("%s: invalid parameters cnt: %d\n",
				__func__, cnt);
		}
		break;
	case LCD_MLVDS:
		if (cnt >= 2) {
			pconf->lcd_control.mlvds_config->phy_vswing = para[0];
			pconf->lcd_control.mlvds_config->phy_preem = para[1];

			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_mlvds_phy_set(pconf, 1);

			LCDPR("%s: vswing=0x%x, preemphasis=0x%x\n",
				__func__, para[0], para[1]);
		} else {
			LCDERR("%s: invalid parameters cnt: %d\n",
				__func__, cnt);
		}
		break;
	case LCD_P2P:
		if (cnt >= 2) {
			pconf->lcd_control.p2p_config->phy_vswing = para[0];
			pconf->lcd_control.p2p_config->phy_preem = para[1];

			if (lcd_drv->lcd_status & LCD_STATUS_IF_ON)
				lcd_p2p_phy_set(pconf, 1);

			LCDPR("%s: vswing=0x%x, preemphasis=0x%x\n",
				__func__, para[0], para[1]);
		} else {
			LCDERR("%s: invalid parameters cnt: %d\n",
				__func__, cnt);
		}
		break;
	default:
		LCDERR("%s: not support lcd_type: %s\n",
			__func__,
			lcd_type_type_to_str(pconf->lcd_basic.lcd_type));
		break;
	}
}

static ssize_t lcd_phy_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	unsigned int vswing = 0xff, preem = 0xff;
	unsigned int clk_vswing = 0xff, clk_preem = 0xff;
	ssize_t len = 0;

	pconf = lcd_drv->lcd_config;
	switch (pconf->lcd_basic.lcd_type) {
	case LCD_LVDS:
		vswing = pconf->lcd_control.lvds_config->phy_vswing;
		preem = pconf->lcd_control.lvds_config->phy_preem;
		clk_vswing = pconf->lcd_control.lvds_config->phy_clk_vswing;
		clk_preem = pconf->lcd_control.lvds_config->phy_clk_preem;

		len += sprintf(buf+len, "vswing=0x%x, preemphasis=0x%x\n",
			vswing, preem);
		if (lcd_drv->data->chip_type <= LCD_CHIP_TXLX) {
			len += sprintf(buf+len,
				"clk_vswing=0x%x, clk_preemphasis=0x%x\n",
				clk_vswing, clk_preem);
		}
		break;
	case LCD_VBYONE:
		vswing = pconf->lcd_control.vbyone_config->phy_vswing;
		preem = pconf->lcd_control.vbyone_config->phy_preem;
		len += sprintf(buf+len, "vswing=0x%x, preemphasis=0x%x\n",
			vswing, preem);
		break;
	case LCD_MLVDS:
		vswing = pconf->lcd_control.mlvds_config->phy_vswing;
		preem = pconf->lcd_control.mlvds_config->phy_preem;
		len += sprintf(buf+len, "vswing=0x%x, preemphasis=0x%x\n",
			vswing, preem);
		break;
	case LCD_P2P:
		vswing = pconf->lcd_control.p2p_config->phy_vswing;
		preem = pconf->lcd_control.p2p_config->phy_preem;
		len += sprintf(buf+len, "vswing=0x%x, preemphasis=0x%x\n",
			vswing, preem);
		break;
	default:
		len = sprintf(buf, "%s: invalid lcd_type: %d\n",
			__func__, pconf->lcd_basic.lcd_type);
		break;
	}
	return len;
}

static ssize_t lcd_phy_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned int para[4];

	ret = sscanf(buf, "%x %x %x %x",
		&para[0], &para[1], &para[2], &para[3]);

	if (ret == 4) {
		lcd_phy_config_update(para, 4);
	} else if (ret == 2) {
		lcd_phy_config_update(para, 2);
	} else {
		pr_info("invalid data\n");
		return -EINVAL;
	}

	return count;
}

static ssize_t lcd_vx1_status_show(struct class *class,
				   struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "vbyone status: lockn = %d hpdn = %d\n",
		       ((lcd_vcbus_read(VBO_STATUS_L) >> 7) & 0x1),
		       ((lcd_vcbus_read(VBO_STATUS_L) >> 6) & 0x1));
}

static void lcd_tcon_reg_table_save(char *path, unsigned char *reg_table,
		unsigned int size)
{
	struct file *filp = NULL;
	loff_t pos = 0;
	void *buf = NULL;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	filp = filp_open(path, O_RDWR|O_CREAT, 0666);

	if (IS_ERR(filp)) {
		LCDERR("%s: create %s error\n", __func__, path);
		set_fs(old_fs);
		return;
	}

	pos = 0;
	buf = (void *)reg_table;
	vfs_write(filp, buf, size, &pos);

	vfs_fsync(filp, 0);
	filp_close(filp, NULL);
	set_fs(old_fs);

	LCDPR("save tcon reg table to %s finished\n", path);
}

static void lcd_tcon_reg_save(char *path, unsigned int size)
{
	struct file *filp = NULL;
	loff_t pos = 0;
	unsigned char *temp;
	void *buf = NULL;
	mm_segment_t old_fs = get_fs();
	int ret;

	set_fs(KERNEL_DS);
	filp = filp_open(path, O_RDWR|O_CREAT, 0666);

	if (IS_ERR(filp)) {
		LCDERR("%s: create %s error\n", __func__, path);
		set_fs(old_fs);
		return;
	}

	temp = kcalloc(size, sizeof(unsigned char), GFP_KERNEL);
	if (!temp) {
		LCDERR("%s: Not enough memory\n", __func__);
		filp_close(filp, NULL);
		set_fs(old_fs);
		return;
	}
	ret = lcd_tcon_core_reg_get(temp, size);
	if (ret) {
		LCDPR("save tcon reg failed\n");
		filp_close(filp, NULL);
		set_fs(old_fs);
		kfree(temp);
		return;
	}

	pos = 0;
	buf = (void *)temp;
	vfs_write(filp, buf, size, &pos);

	vfs_fsync(filp, 0);
	filp_close(filp, NULL);
	set_fs(old_fs);
	kfree(temp);

	LCDPR("save tcon reg to %s success\n", path);
}

static ssize_t lcd_tcon_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	char *buf_orig;
	char **parm = NULL;
	unsigned int temp = 0, val, i, n, size;
	unsigned char data;
	unsigned char *table;
	int ret = -1;

	size = lcd_tcon_reg_table_size_get();
	if (size <= 0)
		return count;
	table = lcd_tcon_reg_table_get();
	if (table == NULL)
		return count;

	if (!buf)
		return count;
	buf_orig = kstrdup(buf, GFP_KERNEL);
	if (buf_orig == NULL) {
		LCDERR("%s: buf malloc error\n", __func__);
		return count;
	}

	parm = kcalloc(520, sizeof(char *), GFP_KERNEL);
	if (parm == NULL) {
		LCDERR("%s: parm malloc error\n", __func__);
		kfree(buf_orig);
		return count;
	}

	lcd_debug_parse_param(buf_orig, parm);

	if (strcmp(parm[0], "reg") == 0) {
		if (parm[1] == NULL) {
			lcd_tcon_reg_readback_print();
			goto lcd_tcon_debug_store_end;
		}
		if (strcmp(parm[1], "save") == 0) {
			if (parm[2] != NULL)
				lcd_tcon_reg_save(parm[2], size);
			else
				pr_info("invalid save path\n");
		} else if (strcmp(parm[1], "rb") == 0) {
			if (parm[2] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				pr_info("read tcon byte [0x%04x] = 0x%02x\n",
					temp, lcd_tcon_read_byte(temp));
			}
		} else if (strcmp(parm[1], "wb") == 0) {
			if (parm[3] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				ret = kstrtouint(parm[3], 16, &val);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				data = (unsigned char)val;
				lcd_tcon_write_byte(temp, data);
				pr_info(
			"write tcon byte [0x%04x] = 0x%02x\n", temp, data);
			}
		} else if (strcmp(parm[1], "wlb") == 0) {
			if (parm[3] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				ret = kstrtouint(parm[3], 16, &size);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}

				if (parm[3 + size] == NULL) {
					pr_info("size and data is not match\n");
					goto lcd_tcon_debug_store_err;
				}

				for (i = 0; i < size; i++) {
					ret = kstrtouint(parm[4 + i], 16, &val);
					if (ret) {
						pr_info("invalid parameters\n");
						goto lcd_tcon_debug_store_err;
					}
					data = (unsigned char)val;
					lcd_tcon_write_byte((temp + i), data);
					pr_info(
					"write tcon byte [0x%04x] = 0x%02x\n",
					(temp + i), data);
				}
			}
		} else if (strcmp(parm[1], "r") == 0) {
			if (parm[2] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				pr_info("read tcon [0x%04x] = 0x%08x\n",
					temp, lcd_tcon_read(temp));
			}
		} else if (strcmp(parm[1], "w") == 0) {
			if (parm[3] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				ret = kstrtouint(parm[3], 16, &val);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				lcd_tcon_write(temp, val);
				pr_info("write tcon [0x%04x] = 0x%08x\n",
					temp, val);
			}
		} else if (strcmp(parm[1], "wl") == 0) {
			if (parm[3] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				pr_info("temp = %d\n", temp);
				ret = kstrtouint(parm[3], 16, &size);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}

				if (parm[3 + size] == NULL) {
					pr_info("size and data is not match\n");
					goto lcd_tcon_debug_store_err;
				}

				for (i = 0; i < size; i++) {
					ret = kstrtouint(parm[4 + i], 16, &val);
					if (ret) {
						pr_info("invalid parameters\n");
						goto lcd_tcon_debug_store_err;
					}
					lcd_tcon_write(temp + i, val);
					pr_info(
					"write tcon [0x%04x] = 0x%08x\n",
					(temp + i), val);
				}
			}
		}
	} else if (strcmp(parm[0], "table") == 0) {
		if (parm[1] == NULL) {
			lcd_tcon_reg_table_print();
			goto lcd_tcon_debug_store_end;
		}
		if (strcmp(parm[1], "r") == 0) {
			if (parm[2] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				if (temp < size) {
					data = table[temp];
					pr_info("read tcon table[%d]=0x%02x\n",
						temp, data);
				} else {
					pr_info("invalid table index: %d\n",
						temp);
				}
			}
		} else if (strcmp(parm[1], "w") == 0) {
			if (parm[3] != NULL) {
				ret = kstrtouint(parm[2], 16, &temp);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				ret = kstrtouint(parm[3], 16, &val);
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				data = (unsigned char)val;
				if (temp < size) {
					table[temp] = data;
					pr_info("write tcon table[%d]=0x%02x\n",
						temp, data);
				} else {
					pr_info("invalid table index: %d\n",
						temp);
				}
			}
		} else if (strcmp(parm[1], "d") == 0) {
			if (parm[3] != NULL) {
				ret = 0;
				if (!kstrtouint(parm[2], 16, &val))
					temp = (unsigned int)val;
				else
					ret = 1;
				if (!kstrtouint(parm[3], 16, &val))
					n = (unsigned char)val;
				else
					ret = 1;
				if (ret) {
					pr_info("invalid parameters\n");
					goto lcd_tcon_debug_store_err;
				}
				pr_info("dump tcon table:\n");
				for (i = temp; i < (temp + n); i++) {
					if (i > size)
						break;
					data = table[i];
					pr_info("  [%d]=0x%02x\n", temp, data);
				}
			}
		} else if (strcmp(parm[1], "update") == 0) {
			lcd_tcon_core_reg_update();
		} else if (strcmp(parm[1], "save") == 0) {
			if (parm[2] != NULL)
				lcd_tcon_reg_table_save(parm[2], table, size);
			else
				pr_info("invalid save path\n");
		}
	} else if (strcmp(parm[0], "od") == 0) { /* over drive */
		if (parm[1] != NULL) {
			if (strcmp(parm[1], "status") == 0) {
				temp = lcd_tcon_od_get();
				if (temp) {
					LCDPR("tcon od is enabled: %d\n", temp);
				} else {
					LCDPR("tcon od is disabled: %d\n",
						temp);
				}
			} else {
				if (!kstrtouint(parm[1], 10, &temp)) {
					if (temp)
						lcd_tcon_od_set(1);
					else
						lcd_tcon_od_set(0);
				}
			}
		}
	} else {
		LCDERR("wrong command\n");
		goto lcd_tcon_debug_store_err;
	}

lcd_tcon_debug_store_end:
	kfree(parm);
	kfree(buf_orig);
	return count;

lcd_tcon_debug_store_err:
	kfree(parm);
	kfree(buf_orig);
	return count;
}

static ssize_t lcd_mipi_cmd_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_mipi_cmd_debug_usage_str);
}

static ssize_t lcd_mipi_cmd_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	int i;
	int ret = 0;
	unsigned int para[24];
	unsigned char *cmd_table = NULL;

	ret = sscanf(buf,
		"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&para[0], &para[1], &para[2], &para[3], &para[4], &para[5],
		&para[6], &para[7], &para[8], &para[9], &para[10], &para[11],
		&para[12], &para[13], &para[14], &para[15], &para[16],
		&para[17], &para[18], &para[19], &para[20], &para[21],
		&para[22], &para[23]);

	if (ret < 2) {
		pr_info("invalid mipi cmd\n");
		return count;
	}
	if (ret < (2+para[1])) {
		pr_info("invalid data num\n");
		return count;
	}

	cmd_table = kmalloc_array((2+para[1]+2),
		sizeof(unsigned char), GFP_KERNEL);
	if (cmd_table == NULL) {
		pr_err("error for mipi cmd\n");
		return -ENOMEM;
	}
	for (i = 0; i < (2+para[1]); i++)
		cmd_table[i] = (unsigned char)para[i];
	cmd_table[2+para[1]] = 0xff;
	cmd_table[2+para[1]+1] = 0xff;

#ifdef CONFIG_AMLOGIC_LCD_TABLET
	dsi_write_cmd(cmd_table);
#endif

	kfree(cmd_table);

	return count;
}

#define MIPI_RD_RET_CODE_MAX    5
static char *mipi_read_ret_code_table[] = {
	"success",
	"read null",
	"read error",
	"read back cnt is wrong",
	"timeout",
	"unknown error",
};

static struct dsi_read_s dread = {
	.flag = 0,
	.reg = 0xff,
	.cnt = 0,
	.value = NULL,
	.ret_code = 4,

	.line_start = 0x1fff,
	.line_end = 0x1fff,
};

static ssize_t lcd_mipi_read_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int i = 0, len;

	if ((lcd_drv->lcd_status & LCD_STATUS_IF_ON) == 0)
		return sprintf(buf, "error: panel is disabled\n");

	if (dread.reg == 0xff)
		return sprintf(buf, "error: reg address is invalid\n");
	if (dread.cnt == 0)
		return sprintf(buf, "error: read count is invalid\n");

	if (dread.cnt > DSI_READ_CNT_MAX)
		return sprintf(buf, "error: mipi read cnt is out of support\n");
	if (dread.value == NULL)
		return sprintf(buf, "error: mipi read return value is null\n");

	dread.line_start = 0x1fff;
	dread.line_end = 0x1fff;
	dread.ret_code = 4;

#ifdef CONFIG_AMLOGIC_LCD_TABLET
	if (lcd_drv->lcd_config->lcd_control.mipi_config->current_mode == 0) {
		dread.flag = 1;
		while (i++ < 5000) {
			if (dread.flag == 0)
				break;
			udelay(20);
		}
	} else {
		lcd_mipi_test_read(&dread);
	}
#endif

	if (dread.ret_code) {
		dread.ret_code = (dread.ret_code >= MIPI_RD_RET_CODE_MAX) ?
					MIPI_RD_RET_CODE_MAX : dread.ret_code;
		return sprintf(buf, "read error: %s(%d)\n",
			mipi_read_ret_code_table[dread.ret_code],
			dread.ret_code);
	}

	len = sprintf(buf, "read reg 0x%02x: ", dread.reg);
	for (i = 0; i < dread.cnt; i++) {
		if (i == 0)
			len += sprintf(buf+len, "0x%02x", dread.value[i]);
		else
			len += sprintf(buf+len, ",0x%02x", dread.value[i]);
	}

	len += sprintf(buf+len, "\nread line start=%d, end=%d\n",
		dread.line_start, dread.line_end);

	return len;
}

static ssize_t lcd_mipi_read_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int para[2];
	int ret = 0;

	ret = sscanf(buf, "%x %d", &para[0], &para[1]);
	if (ret < 2) {
		dread.reg = 0xff;
		dread.cnt = 0;
		pr_info("invalid data\n");
		return count;
	}
	dread.reg = (unsigned char)para[0];
	dread.cnt = (unsigned char)para[1];
	if (dread.cnt > DSI_READ_CNT_MAX) {
		LCDERR("mipi read cnt is out of support\n");
		return count;
	}
	if (dread.value == NULL) {
		LCDERR("mipi read return value is null\n");
		return count;
	}

	pr_info("set mipi read reg: 0x%02x, cnt: %d\n", dread.reg, dread.cnt);

	return count;
}

static ssize_t lcd_mipi_state_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int state_save;

	state_save = lcd_vcbus_getb(L_VCOM_VS_ADDR, 12, 1);

	return sprintf(buf, "state: %d, check_en: %d, state_save: %d\n",
		lcd_drv->lcd_config->lcd_control.mipi_config->check_state,
		lcd_drv->lcd_config->lcd_control.mipi_config->check_en,
		state_save);
}

static ssize_t lcd_mipi_mode_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned char mode;

	if ((lcd_drv->lcd_status & LCD_STATUS_IF_ON) == 0)
		return sprintf(buf, "error: panel is disabled\n");

	mode = lcd_drv->lcd_config->lcd_control.mipi_config->current_mode;
	return sprintf(buf, "current mipi-dsi operation mode: %s(%d)\n",
		(mode ? "command" : "video"), mode);
}

static ssize_t lcd_mipi_mode_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int temp;
	unsigned char mode;
	int ret = 0;

	if ((lcd_drv->lcd_status & LCD_STATUS_IF_ON) == 0) {
		LCDERR("panel is disabled\n");
		return count;
	}

	ret = kstrtouint(buf, 10, &temp);
	if (ret) {
		pr_info("invalid data\n");
		return -EINVAL;
	}
	mode = (unsigned char)temp;
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	dsi_set_operation_mode(mode);
#endif

	return count;
}

static struct class_attribute lcd_debug_class_attrs_ttl[] = {
	__ATTR(ttl,    0644,
		lcd_ttl_debug_show, lcd_ttl_debug_store),
	__ATTR(null,   0644, NULL, NULL),
};

static struct class_attribute lcd_debug_class_attrs_lvds[] = {
	__ATTR(lvds,   0644,
		lcd_lvds_debug_show, lcd_lvds_debug_store),
	__ATTR(phy,    0644,
		lcd_phy_debug_show, lcd_phy_debug_store),
	__ATTR(null,   0644, NULL, NULL),
};

static struct class_attribute lcd_debug_class_attrs_vbyone[] = {
	__ATTR(vbyone, 0644,
		lcd_vx1_debug_show, lcd_vx1_debug_store),
	__ATTR(phy,    0644,
		lcd_phy_debug_show, lcd_phy_debug_store),
	__ATTR(status,    0644,
	       lcd_vx1_status_show, NULL),
	__ATTR(null,   0644, NULL, NULL),
};

static struct class_attribute lcd_debug_class_attrs_mlvds[] = {
	__ATTR(mlvds,  0644,
		lcd_mlvds_debug_show, lcd_mlvds_debug_store),
	__ATTR(phy,    0644,
		lcd_phy_debug_show, lcd_phy_debug_store),
	__ATTR(tcon,   0644,
		lcd_tcon_debug_show, lcd_tcon_debug_store),
	__ATTR(tcon_status,   0644,
	       lcd_tcon_status_show, NULL),
	__ATTR(null,   0644, NULL, NULL),
};

static struct class_attribute lcd_debug_class_attrs_p2p[] = {
	__ATTR(p2p,    0644,
		lcd_p2p_debug_show, lcd_p2p_debug_store),
	__ATTR(phy,    0644,
		lcd_phy_debug_show, lcd_phy_debug_store),
	__ATTR(tcon,   0644,
		lcd_tcon_debug_show, lcd_tcon_debug_store),
	__ATTR(tcon_status,   0644,
	       lcd_tcon_status_show, NULL),
	__ATTR(null,   0644, NULL, NULL),
};

static struct class_attribute lcd_debug_class_attrs_mipi[] = {
	__ATTR(mipi,   0644,
		lcd_mipi_debug_show, lcd_mipi_debug_store),
	__ATTR(mpcmd,    0644,
		lcd_mipi_cmd_debug_show, lcd_mipi_cmd_debug_store),
	__ATTR(mpread,   0644,
		lcd_mipi_read_debug_show, lcd_mipi_read_debug_store),
	__ATTR(mpstate,  0644, lcd_mipi_state_debug_show, NULL),
	__ATTR(mpmode,   0644,
		lcd_mipi_mode_debug_show, lcd_mipi_mode_debug_store),
	__ATTR(null,     0644, NULL, NULL),
};

#define LCD_DEBUG_CLASS_ATTRS_IF_MAX    10
static int lcd_class_creat(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct class *lcd_class;
	struct class_attribute *lcd_attr;
	int i;

	lcd_drv->lcd_screen_restore = lcd_screen_restore;
	lcd_drv->lcd_screen_black = lcd_screen_black;

	INIT_WORK(&lcd_test_check_work, lcd_test_pattern_check);

	lcd_drv->lcd_debug_class = class_create(THIS_MODULE, "lcd");
	if (IS_ERR(lcd_drv->lcd_debug_class)) {
		LCDERR("create lcd debug class fail\n");
		return -1;
	}
	lcd_class = lcd_drv->lcd_debug_class;

	for (i = 0; i < ARRAY_SIZE(lcd_debug_class_attrs); i++) {
		if (class_create_file(lcd_class, &lcd_debug_class_attrs[i])) {
			LCDERR("create lcd debug attribute %s fail\n",
				lcd_debug_class_attrs[i].attr.name);
		}
	}

	if (lcd_debug_info_if) {
		if (lcd_debug_info_if->class_attrs) {
			lcd_attr = lcd_debug_info_if->class_attrs;
			while (lcd_attr) {
				if (strcmp(lcd_attr->attr.name, "null") == 0)
					break;
				if (class_create_file(lcd_class, lcd_attr)) {
					LCDERR(
			"create lcd_interface debug attribute %s fail\n",
						lcd_attr->attr.name);
				}
				lcd_attr++;
			}
		} else {
			LCDERR("lcd_debug_info_if class_attrs is null\n");
		}
	} else {
		LCDERR("lcd_debug_info_if is null\n");
	}

	return 0;
}

static int lcd_class_remove(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct class *lcd_class;
	struct class_attribute *lcd_attr;
	int i;

	lcd_class = lcd_drv->lcd_debug_class;

	for (i = 0; i < ARRAY_SIZE(lcd_debug_class_attrs); i++)
		class_remove_file(lcd_class, &lcd_debug_class_attrs[i]);

	if (lcd_debug_info_if) {
		if (lcd_debug_info_if->class_attrs) {
			lcd_attr = lcd_debug_info_if->class_attrs;
			while (lcd_attr) {
				if (strcmp(lcd_attr->attr.name, "null") == 0)
					break;
				class_remove_file(lcd_class, lcd_attr);
				lcd_attr++;
			}
		}
	}

	class_destroy(lcd_drv->lcd_debug_class);
	lcd_drv->lcd_debug_class = NULL;

	return 0;
}

/* **********************************
 * lcd debug match data
 * **********************************
 */
/* chip_type data */
static struct lcd_debug_info_reg_s lcd_debug_info_reg_gxl = {
	.reg_clk_table = lcd_reg_dump_clk_dft,
	.reg_encl_table = lcd_reg_dump_encl_dft,
	.reg_pinmux_table = lcd_reg_dump_pinmux_gxl,
};

static struct lcd_debug_info_reg_s lcd_debug_info_reg_txl = {
	.reg_clk_table = lcd_reg_dump_clk_dft,
	.reg_encl_table = lcd_reg_dump_encl_dft,
	.reg_pinmux_table = lcd_reg_dump_pinmux_txl,
};

static struct lcd_debug_info_reg_s lcd_debug_info_reg_txlx = {
	.reg_clk_table = lcd_reg_dump_clk_dft,
	.reg_encl_table = lcd_reg_dump_encl_dft,
	.reg_pinmux_table = lcd_reg_dump_pinmux_txlx,
};

static struct lcd_debug_info_reg_s lcd_debug_info_reg_axg = {
	.reg_clk_table = lcd_reg_dump_clk_axg,
	.reg_encl_table = lcd_reg_dump_encl_dft,
	.reg_pinmux_table = NULL,
};

static struct lcd_debug_info_reg_s lcd_debug_info_reg_g12a_clk_path0 = {
	.reg_clk_table = lcd_reg_dump_clk_hpll_g12a,
	.reg_encl_table = lcd_reg_dump_encl_dft,
	.reg_pinmux_table = NULL,
};

static struct lcd_debug_info_reg_s lcd_debug_info_reg_g12a_clk_path1 = {
	.reg_clk_table = lcd_reg_dump_clk_gp0_g12a,
	.reg_encl_table = lcd_reg_dump_encl_dft,
	.reg_pinmux_table = NULL,
};

static struct lcd_debug_info_reg_s lcd_debug_info_reg_tl1 = {
	.reg_clk_table = lcd_reg_dump_clk_tl1,
	.reg_encl_table = lcd_reg_dump_encl_tl1,
	.reg_pinmux_table = lcd_reg_dump_pinmux_tl1,
};

/* interface data */
static struct lcd_debug_info_if_s lcd_debug_info_if_ttl = {
	.interface_print = lcd_info_print_ttl,
	.reg_dump_interface = lcd_reg_print_ttl,
	.reg_dump_phy = NULL,
	.class_attrs = lcd_debug_class_attrs_ttl,
};

static struct lcd_debug_info_if_s lcd_debug_info_if_lvds = {
	.interface_print = lcd_info_print_lvds,
	.reg_dump_interface = lcd_reg_print_lvds,
	.reg_dump_phy = lcd_reg_print_phy_analog,
	.class_attrs = lcd_debug_class_attrs_lvds,
};

static struct lcd_debug_info_if_s lcd_debug_info_if_vbyone = {
	.interface_print = lcd_info_print_vbyone,
	.reg_dump_interface = lcd_reg_print_vbyone_txl,
	.reg_dump_phy = lcd_reg_print_phy_analog,
	.class_attrs = lcd_debug_class_attrs_vbyone,
};

static struct lcd_debug_info_if_s lcd_debug_info_if_mipi = {
	.interface_print = lcd_info_print_mipi,
	.reg_dump_interface = lcd_reg_print_mipi,
	.reg_dump_phy = lcd_reg_print_mipi_phy_analog,
	.class_attrs = lcd_debug_class_attrs_mipi,
};

static struct lcd_debug_info_if_s lcd_debug_info_if_mlvds = {
	.interface_print = lcd_info_print_mlvds,
	.reg_dump_interface = lcd_reg_print_mlvds,
	.reg_dump_phy = lcd_reg_print_phy_analog,
	.class_attrs = lcd_debug_class_attrs_mlvds,
};

static struct lcd_debug_info_if_s lcd_debug_info_if_p2p = {
	.interface_print = lcd_info_print_p2p,
	.reg_dump_interface = lcd_reg_print_p2p,
	.reg_dump_phy = lcd_reg_print_phy_analog,
	.class_attrs = lcd_debug_class_attrs_p2p,
};

int lcd_debug_probe(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int lcd_type, ret;

	lcd_type = lcd_drv->lcd_config->lcd_basic.lcd_type;

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		lcd_debug_info_reg = &lcd_debug_info_reg_tl1;
		lcd_debug_info_if_lvds.reg_dump_phy =
			lcd_reg_print_phy_analog_tl1;
		lcd_debug_info_if_vbyone.reg_dump_interface =
			lcd_reg_print_vbyone_tl1;
		lcd_debug_info_if_vbyone.reg_dump_phy =
			lcd_reg_print_phy_analog_tl1;
		lcd_debug_info_if_mlvds.reg_dump_phy =
			lcd_reg_print_phy_analog_tl1;
		lcd_debug_info_if_p2p.reg_dump_phy =
			lcd_reg_print_phy_analog_tl1;
		break;
	case LCD_CHIP_G12A:
	case LCD_CHIP_G12B:
	case LCD_CHIP_SM1:
		if (lcd_drv->lcd_clk_path)
			lcd_debug_info_reg = &lcd_debug_info_reg_g12a_clk_path1;
		else
			lcd_debug_info_reg = &lcd_debug_info_reg_g12a_clk_path0;
		break;
	case LCD_CHIP_AXG:
		lcd_debug_info_reg = &lcd_debug_info_reg_axg;
		break;
	case LCD_CHIP_TXLX:
		lcd_debug_info_reg = &lcd_debug_info_reg_txlx;
		break;
	case LCD_CHIP_TXL:
		lcd_debug_info_reg = &lcd_debug_info_reg_txl;
		break;
	case LCD_CHIP_GXL:
	case LCD_CHIP_GXM:
		lcd_debug_info_reg = &lcd_debug_info_reg_gxl;
		break;
	default:
		lcd_debug_info_reg = NULL;
		break;
	}

	switch (lcd_type) {
	case LCD_TTL:
		lcd_debug_info_if = &lcd_debug_info_if_ttl;
		break;
	case LCD_LVDS:
		lcd_debug_info_if = &lcd_debug_info_if_lvds;
		break;
	case LCD_VBYONE:
		lcd_debug_info_if = &lcd_debug_info_if_vbyone;
		break;
	case LCD_MIPI:
		lcd_debug_info_if = &lcd_debug_info_if_mipi;
		break;
	case LCD_MLVDS:
		lcd_debug_info_if = &lcd_debug_info_if_mlvds;
		break;
	case LCD_P2P:
		lcd_debug_info_if = &lcd_debug_info_if_p2p;
		break;
	default:
		lcd_debug_info_if = NULL;
		break;
	}

	ret = lcd_class_creat();

	return ret;
}

int lcd_debug_remove(void)
{
	int ret;

	ret = lcd_class_remove();
	return ret;
}
