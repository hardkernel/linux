/*
 * drivers/amlogic/media/vout/lcd/lcd_common.c
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
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/media/vout/lcd/lcd_notify.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#include <linux/amlogic/media/vout/vinfo.h>

#include "lcd_common.h"
#include "lcd_reg.h"

/* **********************************
 * lcd type
 * **********************************
 */
struct lcd_type_match_s {
	char *name;
	enum lcd_type_e type;
};

static struct lcd_type_match_s lcd_type_match_table[] = {
	{"ttl",      LCD_TTL},
	{"lvds",     LCD_LVDS},
	{"vbyone",   LCD_VBYONE},
	{"mipi",     LCD_MIPI},
	{"minilvds", LCD_MLVDS},
	{"p2p",      LCD_P2P},
	{"invalid",  LCD_TYPE_MAX},
};

int lcd_type_str_to_type(const char *str)
{
	int i;
	int type = LCD_TYPE_MAX;

	for (i = 0; i < LCD_TYPE_MAX; i++) {
		if (!strcmp(str, lcd_type_match_table[i].name)) {
			type = lcd_type_match_table[i].type;
			break;
		}
	}
	return type;
}

char *lcd_type_type_to_str(int type)
{
	int i;
	char *str = lcd_type_match_table[LCD_TYPE_MAX].name;

	for (i = 0; i < LCD_TYPE_MAX; i++) {
		if (type == lcd_type_match_table[i].type) {
			str = lcd_type_match_table[i].name;
			break;
		}
	}
	return str;
}

static char *lcd_mode_table[] = {
	"tv",
	"tablet",
	"invalid",
};

unsigned char lcd_mode_str_to_mode(const char *str)
{
	unsigned char mode;

	for (mode = 0; mode < ARRAY_SIZE(lcd_mode_table); mode++) {
		if (!strcmp(str, lcd_mode_table[mode]))
			break;
	}
	return mode;
}

char *lcd_mode_mode_to_str(int mode)
{
	return lcd_mode_table[mode];
}

/* **********************************
 * lcd gpio
 * **********************************
 */

void lcd_cpu_gpio_probe(unsigned int index)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_cpu_gpio_s *cpu_gpio;
	const char *str;
	int ret;

	if (index >= LCD_CPU_GPIO_NUM_MAX) {
		LCDERR("gpio index %d, exit\n", index);
		return;
	}
	cpu_gpio = &lcd_drv->lcd_config->lcd_power->cpu_gpio[index];
	if (cpu_gpio->probe_flag) {
		if (lcd_debug_print_flag) {
			LCDPR("gpio %s[%d] is already registered\n",
				cpu_gpio->name, index);
		}
		return;
	}

	/* get gpio name */
	ret = of_property_read_string_index(lcd_drv->dev->of_node,
		"lcd_cpu_gpio_names", index, &str);
	if (ret) {
		LCDERR("failed to get lcd_cpu_gpio_names: %d\n", index);
		str = "unknown";
	}
	strcpy(cpu_gpio->name, str);

	/* init gpio flag */
	cpu_gpio->probe_flag = 1;
	cpu_gpio->register_flag = 0;
}

static int lcd_cpu_gpio_register(unsigned int index, int init_value)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_cpu_gpio_s *cpu_gpio;
	int value;

	if (index >= LCD_CPU_GPIO_NUM_MAX) {
		LCDERR("%s: gpio index %d, exit\n", __func__, index);
		return -1;
	}
	cpu_gpio = &lcd_drv->lcd_config->lcd_power->cpu_gpio[index];
	if (cpu_gpio->probe_flag == 0) {
		LCDERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return -1;
	}
	if (cpu_gpio->register_flag) {
		LCDPR("%s: gpio %s[%d] is already registered\n",
			__func__, cpu_gpio->name, index);
		return 0;
	}

	switch (init_value) {
	case LCD_GPIO_OUTPUT_LOW:
		value = GPIOD_OUT_LOW;
		break;
	case LCD_GPIO_OUTPUT_HIGH:
		value = GPIOD_OUT_HIGH;
		break;
	case LCD_GPIO_INPUT:
	default:
		value = GPIOD_IN;
		break;
	}

	/* request gpio */
	cpu_gpio->gpio = devm_gpiod_get_index(lcd_drv->dev,
		"lcd_cpu", index, value);
	if (IS_ERR(cpu_gpio->gpio)) {
		LCDERR("register gpio %s[%d]: %p, err: %d\n",
			cpu_gpio->name, index, cpu_gpio->gpio,
			IS_ERR(cpu_gpio->gpio));
		return -1;
	}
	cpu_gpio->register_flag = 1;
	if (lcd_debug_print_flag) {
		LCDPR("register gpio %s[%d]: %p, init value: %d\n",
			cpu_gpio->name, index, cpu_gpio->gpio, init_value);
	}

	return 0;
}

void lcd_cpu_gpio_set(unsigned int index, int value)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_cpu_gpio_s *cpu_gpio;

	if (index >= LCD_CPU_GPIO_NUM_MAX) {
		LCDERR("gpio index %d, exit\n", index);
		return;
	}
	cpu_gpio = &lcd_drv->lcd_config->lcd_power->cpu_gpio[index];
	if (cpu_gpio->probe_flag == 0) {
		LCDERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return;
	}
	if (cpu_gpio->register_flag == 0) {
		lcd_cpu_gpio_register(index, value);
		return;
	}

	if (IS_ERR_OR_NULL(cpu_gpio->gpio)) {
		LCDERR("gpio %s[%d]: %p, err: %ld\n",
			cpu_gpio->name, index, cpu_gpio->gpio,
			PTR_ERR(cpu_gpio->gpio));
		return;
	}

	switch (value) {
	case LCD_GPIO_OUTPUT_LOW:
	case LCD_GPIO_OUTPUT_HIGH:
		gpiod_direction_output(cpu_gpio->gpio, value);
		break;
	case LCD_GPIO_INPUT:
	default:
		gpiod_direction_input(cpu_gpio->gpio);
		break;
	}
	if (lcd_debug_print_flag) {
		LCDPR("set gpio %s[%d] value: %d\n",
			cpu_gpio->name, index, value);
	}
}

unsigned int lcd_cpu_gpio_get(unsigned int index)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_cpu_gpio_s *cpu_gpio;

	cpu_gpio = &lcd_drv->lcd_config->lcd_power->cpu_gpio[index];
	if (cpu_gpio->probe_flag == 0) {
		LCDERR("%s: gpio [%d] is not probed\n", __func__, index);
		return -1;
	}
	if (cpu_gpio->register_flag == 0) {
		LCDERR("%s: gpio %s[%d] is not registered\n",
			__func__, cpu_gpio->name, index);
		return -1;
	}
	if (IS_ERR_OR_NULL(cpu_gpio->gpio)) {
		LCDERR("gpio[%d]: %p, err: %ld\n",
			index, cpu_gpio->gpio, PTR_ERR(cpu_gpio->gpio));
		return -1;
	}

	return gpiod_get_value(cpu_gpio->gpio);
}

static char *lcd_ttl_pinmux_str[] = {
	"ttl_6bit_hvsync_on",      /* 0 */
	"ttl_6bit_de_on",          /* 1 */
	"ttl_6bit_hvsync_de_on",   /* 2 */
	"ttl_6bit_hvsync_de_off",  /* 3 */
	"ttl_8bit_hvsync_on",      /* 4 */
	"ttl_8bit_de_on",          /* 5 */
	"ttl_8bit_hvsync_de_on",   /* 6 */
	"ttl_8bit_hvsync_de_off",  /* 7 */
};

void lcd_ttl_pinmux_set(int status)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	unsigned int base, index;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	pconf = lcd_drv->lcd_config;
	if (pconf->lcd_basic.lcd_bits == 6)
		base = 0;
	else
		base = 4;

	if (status) {
		switch (pconf->lcd_control.ttl_config->sync_valid) {
		case 1: /* hvsync */
			index = base + 0;
			break;
		case 2: /* DE */
			index = base + 1;
			break;
		default:
		case 3: /* DE + hvsync */
			index = base + 2;
			break;
		}
	} else {
		index = base + 3;
	}

	if (pconf->pinmux_flag == index) {
		LCDPR("pinmux %s is already selected\n",
			lcd_ttl_pinmux_str[index]);
		return;
	}

	/* request pinmux */
	pconf->pin = devm_pinctrl_get_select(lcd_drv->dev,
		lcd_ttl_pinmux_str[index]);
	if (IS_ERR(pconf->pin))
		LCDERR("set ttl pinmux %s error\n", lcd_ttl_pinmux_str[index]);
	else {
		if (lcd_debug_print_flag) {
			LCDPR("set ttl pinmux %s: %p\n",
				lcd_ttl_pinmux_str[index], pconf->pin);
		}
	}
	pconf->pinmux_flag = index;
}

static char *lcd_vbyone_pinmux_str[] = {
	"vbyone",
	"vbyone_off",
	"none",
};

/* set VX1_LOCKN && VX1_HTPDN */
void lcd_vbyone_pinmux_set(int status)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	unsigned int index;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	pconf = lcd_drv->lcd_config;
	index = (status) ? 0 : 1;

	if (pconf->pinmux_flag == index) {
		LCDPR("pinmux %s is already selected\n",
			lcd_vbyone_pinmux_str[index]);
		return;
	}

	pconf->pin = devm_pinctrl_get_select(lcd_drv->dev,
		lcd_vbyone_pinmux_str[index]);
	if (IS_ERR(pconf->pin)) {
		LCDERR("set vbyone pinmux %s error\n",
			lcd_vbyone_pinmux_str[index]);
	} else {
		if (lcd_debug_print_flag) {
			LCDPR("set vbyone pinmux %s: %p\n",
				lcd_vbyone_pinmux_str[index], pconf->pin);
		}
	}
	pconf->pinmux_flag = index;
}

static char *lcd_tcon_pinmux_str[] = {
	"tcon",
	"tcon_off",
	"none",
};

void lcd_tcon_pinmux_set(int status)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	unsigned int index;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	pconf = lcd_drv->lcd_config;
	index = (status) ? 0 : 1;

	if (pconf->pinmux_flag == index) {
		LCDPR("pinmux %s is already selected\n",
			lcd_tcon_pinmux_str[index]);
		return;
	}

	pconf->pin = devm_pinctrl_get_select(lcd_drv->dev,
		lcd_tcon_pinmux_str[index]);
	if (IS_ERR(pconf->pin)) {
		LCDERR("set tcon pinmux %s error\n",
			lcd_tcon_pinmux_str[index]);
	} else {
		if (lcd_debug_print_flag) {
			LCDPR("set tcon pinmux %s: %p\n",
				lcd_tcon_pinmux_str[index], pconf->pin);
		}
	}
	pconf->pinmux_flag = index;
}

int lcd_power_load_from_dts(struct lcd_config_s *pconf,
		struct device_node *child)
{
	int ret = 0;
	unsigned int para[5];
	unsigned int val;
	struct lcd_power_ctrl_s *lcd_power = pconf->lcd_power;
	int i, j, temp;
	unsigned int index;

	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	if (child == NULL) {
		LCDPR("error: failed to get %s\n", pconf->lcd_propname);
		return -1;
	}

	ret = of_property_read_u32_array(child, "power_on_step", &para[0], 4);
	if (ret) {
		LCDPR("failed to get power_on_step\n");
		lcd_power->power_on_step[0].type = LCD_POWER_TYPE_MAX;
	} else {
		i = 0;
		while (i < LCD_PWR_STEP_MAX) {
			lcd_power->power_on_step_max = i;
			j = 4 * i;
			ret = of_property_read_u32_index(child, "power_on_step",
				j, &val);
			lcd_power->power_on_step[i].type = (unsigned char)val;
			if (val == 0xff) /* ending */
				break;
			j = 4 * i + 1;
			ret = of_property_read_u32_index(child,
				"power_on_step", j, &val);
			lcd_power->power_on_step[i].index = val;
			j = 4 * i + 2;
			ret = of_property_read_u32_index(child,
				"power_on_step", j, &val);
			lcd_power->power_on_step[i].value = val;
			j = 4 * i + 3;
			ret = of_property_read_u32_index(child,
				"power_on_step", j, &val);
			lcd_power->power_on_step[i].delay = val;

			/* gpio/extern probe */
			index = lcd_power->power_on_step[i].index;
			switch (lcd_power->power_on_step[i].type) {
			case LCD_POWER_TYPE_CPU:
			case LCD_POWER_TYPE_WAIT_GPIO:
				if (index < LCD_CPU_GPIO_NUM_MAX)
					lcd_cpu_gpio_probe(index);
				break;
			case LCD_POWER_TYPE_EXTERN:
				pconf->extern_index = index;
				break;
			case LCD_POWER_TYPE_CLK_SS:
				temp = pconf->lcd_power->power_on_step[i].value;
				pconf->lcd_timing.ss_level |= temp << 8;
			break;
			default:
				break;
			}
			if (lcd_debug_print_flag) {
				LCDPR("power_on %d type: %d\n", i,
					lcd_power->power_on_step[i].type);
				LCDPR("power_on %d index: %d\n", i,
					lcd_power->power_on_step[i].index);
				LCDPR("power_on %d value: %d\n", i,
					lcd_power->power_on_step[i].value);
				LCDPR("power_on %d delay: %d\n", i,
					lcd_power->power_on_step[i].delay);
			}
			i++;
		}
	}

	ret = of_property_read_u32_array(child, "power_off_step", &para[0], 4);
	if (ret) {
		LCDPR("failed to get power_off_step\n");
		lcd_power->power_off_step[0].type = LCD_POWER_TYPE_MAX;
	} else {
		i = 0;
		while (i < LCD_PWR_STEP_MAX) {
			lcd_power->power_off_step_max = i;
			j = 4 * i;
			ret = of_property_read_u32_index(child,
				"power_off_step", j, &val);
			lcd_power->power_off_step[i].type = (unsigned char)val;
			if (val == 0xff) /* ending */
				break;
			j = 4 * i + 1;
			ret = of_property_read_u32_index(child,
				"power_off_step", j, &val);
			lcd_power->power_off_step[i].index = val;
			j = 4 * i + 2;
			ret = of_property_read_u32_index(child,
				"power_off_step", j, &val);
			lcd_power->power_off_step[i].value = val;
			j = 4 * i + 3;
			ret = of_property_read_u32_index(child,
				"power_off_step", j, &val);
			lcd_power->power_off_step[i].delay = val;

			/* gpio/extern probe */
			index = lcd_power->power_off_step[i].index;
			switch (lcd_power->power_off_step[i].type) {
			case LCD_POWER_TYPE_CPU:
			case LCD_POWER_TYPE_WAIT_GPIO:
				if (index < LCD_CPU_GPIO_NUM_MAX)
					lcd_cpu_gpio_probe(index);
				break;
			case LCD_POWER_TYPE_EXTERN:
				if (pconf->extern_index == 0xff)
					pconf->extern_index = index;
				break;
			default:
				break;
			}
			if (lcd_debug_print_flag) {
				LCDPR("power_off %d type: %d\n", i,
					lcd_power->power_off_step[i].type);
				LCDPR("power_off %d index: %d\n", i,
					lcd_power->power_off_step[i].index);
				LCDPR("power_off %d value: %d\n", i,
					lcd_power->power_off_step[i].value);
				LCDPR("power_off %d delay: %d\n", i,
					lcd_power->power_off_step[i].delay);
			}
			i++;
		}
	}

	ret = of_property_read_u32(child, "backlight_index", &para[0]);
	if (ret) {
		LCDPR("failed to get backlight_index\n");
		pconf->backlight_index = 0xff;
	} else {
		pconf->backlight_index = para[0];
	}

	return ret;
}

int lcd_power_load_from_unifykey(struct lcd_config_s *pconf,
		unsigned char *buf, int key_len, int len)
{
	int i, j, temp;
	unsigned char *p;
	unsigned int index;
	int ret;

	/* power: (5byte * n) */
	p = buf + len;
	i = 0;
	while (i < LCD_PWR_STEP_MAX) {
		pconf->lcd_power->power_on_step_max = i;
		len += 5;
		ret = lcd_unifykey_len_check(key_len, len);
		if (ret < 0) {
			pconf->lcd_power->power_on_step[i].type = 0xff;
			pconf->lcd_power->power_on_step[i].index = 0;
			pconf->lcd_power->power_on_step[i].value = 0;
			pconf->lcd_power->power_on_step[i].delay = 0;
			LCDERR("unifykey power_on length is incorrect\n");
			return -1;
		}
		pconf->lcd_power->power_on_step[i].type =
			*(p + LCD_UKEY_PWR_TYPE + 5*i);
		pconf->lcd_power->power_on_step[i].index =
			*(p + LCD_UKEY_PWR_INDEX + 5*i);
		pconf->lcd_power->power_on_step[i].value =
			*(p + LCD_UKEY_PWR_VAL + 5*i);
		pconf->lcd_power->power_on_step[i].delay =
			(*(p + LCD_UKEY_PWR_DELAY + 5*i) |
			((*(p + LCD_UKEY_PWR_DELAY + 5*i + 1)) << 8));

		/* gpio/extern probe */
		index = pconf->lcd_power->power_on_step[i].index;
		switch (pconf->lcd_power->power_on_step[i].type) {
		case LCD_POWER_TYPE_CPU:
		case LCD_POWER_TYPE_WAIT_GPIO:
			if (index < LCD_CPU_GPIO_NUM_MAX)
				lcd_cpu_gpio_probe(index);
			break;
		case LCD_POWER_TYPE_EXTERN:
			pconf->extern_index = index;
			break;
		case LCD_POWER_TYPE_CLK_SS:
			temp = pconf->lcd_power->power_on_step[i].value;
			pconf->lcd_timing.ss_level |= temp << 8;
			break;
		default:
			break;
		}
		if (lcd_debug_print_flag) {
			LCDPR("%d: type=%d, index=%d, value=%d, delay=%d\n",
				i, pconf->lcd_power->power_on_step[i].type,
				pconf->lcd_power->power_on_step[i].index,
				pconf->lcd_power->power_on_step[i].value,
				pconf->lcd_power->power_on_step[i].delay);
		}
		if (pconf->lcd_power->power_on_step[i].type >=
			LCD_POWER_TYPE_MAX)
			break;
		i++;
	}

	if (lcd_debug_print_flag)
		LCDPR("power_off step:\n");
	p += (5*(i + 1));
	j = 0;
	while (j < LCD_PWR_STEP_MAX) {
		pconf->lcd_power->power_off_step_max = j;
		len += 5;
		ret = lcd_unifykey_len_check(key_len, len);
		if (ret < 0) {
			pconf->lcd_power->power_off_step[j].type = 0xff;
			pconf->lcd_power->power_off_step[j].index = 0;
			pconf->lcd_power->power_off_step[j].value = 0;
			pconf->lcd_power->power_off_step[j].delay = 0;
			LCDERR("unifykey power_off length is incorrect\n");
			return -1;
		}
		pconf->lcd_power->power_off_step[j].type =
			*(p + LCD_UKEY_PWR_TYPE + 5*j);
		pconf->lcd_power->power_off_step[j].index =
			*(p + LCD_UKEY_PWR_INDEX + 5*j);
		pconf->lcd_power->power_off_step[j].value =
			*(p + LCD_UKEY_PWR_VAL + 5*j);
		pconf->lcd_power->power_off_step[j].delay =
				(*(p + LCD_UKEY_PWR_DELAY + 5*j) |
				((*(p + LCD_UKEY_PWR_DELAY + 5*j + 1)) << 8));

		/* gpio/extern probe */
		index = pconf->lcd_power->power_off_step[j].index;
		switch (pconf->lcd_power->power_off_step[j].type) {
		case LCD_POWER_TYPE_CPU:
		case LCD_POWER_TYPE_WAIT_GPIO:
			if (index < LCD_CPU_GPIO_NUM_MAX)
				lcd_cpu_gpio_probe(index);
			break;
		case LCD_POWER_TYPE_EXTERN:
			if (pconf->extern_index == 0xff)
				pconf->extern_index = index;
			break;
		default:
			break;
		}
		if (lcd_debug_print_flag) {
			LCDPR("%d: type=%d, index=%d, value=%d, delay=%d\n",
				j, pconf->lcd_power->power_off_step[j].type,
				pconf->lcd_power->power_off_step[j].index,
				pconf->lcd_power->power_off_step[j].value,
				pconf->lcd_power->power_off_step[j].delay);
		}
		if (pconf->lcd_power->power_off_step[j].type >=
			LCD_POWER_TYPE_MAX)
			break;
		j++;
	}

	return 0;
}

int lcd_vlock_param_load_from_dts(struct lcd_config_s *pconf,
		struct device_node *child)
{
	unsigned int para[4];
	int ret;

	pconf->lcd_control.vlock_param[0] = LCD_VLOCK_PARAM_BIT_UPDATE;

	ret = of_property_read_u32_array(child, "vlock_attr", &para[0], 4);
	if (ret == 0) {
		LCDPR("find vlock_attr\n");
		pconf->lcd_control.vlock_param[0] |= LCD_VLOCK_PARAM_BIT_VALID;
		pconf->lcd_control.vlock_param[1] = para[0];
		pconf->lcd_control.vlock_param[2] = para[1];
		pconf->lcd_control.vlock_param[3] = para[2];
		pconf->lcd_control.vlock_param[4] = para[3];
	}

	return 0;
}

int lcd_vlock_param_load_from_unifykey(struct lcd_config_s *pconf,
		unsigned char *buf)
{
	unsigned char *p;

	p = buf;

	pconf->lcd_control.vlock_param[0] = LCD_VLOCK_PARAM_BIT_UPDATE;
	pconf->lcd_control.vlock_param[1] = *(p + LCD_UKEY_VLOCK_VAL_0);
	pconf->lcd_control.vlock_param[2] = *(p + LCD_UKEY_VLOCK_VAL_1);
	pconf->lcd_control.vlock_param[3] = *(p + LCD_UKEY_VLOCK_VAL_2);
	pconf->lcd_control.vlock_param[4] = *(p + LCD_UKEY_VLOCK_VAL_3);
	if (pconf->lcd_control.vlock_param[1] ||
		pconf->lcd_control.vlock_param[2] ||
		pconf->lcd_control.vlock_param[3] ||
		pconf->lcd_control.vlock_param[4]) {
		LCDPR("find vlock_attr\n");
		pconf->lcd_control.vlock_param[0] |= LCD_VLOCK_PARAM_BIT_VALID;
	}

	return 0;
}

void lcd_optical_vinfo_update(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lcd_config_s *pconf;
	struct master_display_info_s *disp_vinfo;

	pconf = lcd_drv->lcd_config;
	disp_vinfo = &lcd_drv->lcd_info->master_display_info;
	disp_vinfo->present_flag = pconf->optical_info.hdr_support;
	disp_vinfo->features = pconf->optical_info.features;
	disp_vinfo->primaries[0][0] = pconf->optical_info.primaries_g_x;
	disp_vinfo->primaries[0][1] = pconf->optical_info.primaries_g_y;
	disp_vinfo->primaries[1][0] = pconf->optical_info.primaries_b_x;
	disp_vinfo->primaries[1][1] = pconf->optical_info.primaries_b_y;
	disp_vinfo->primaries[2][0] = pconf->optical_info.primaries_r_x;
	disp_vinfo->primaries[2][1] = pconf->optical_info.primaries_r_y;
	disp_vinfo->white_point[0] = pconf->optical_info.white_point_x;
	disp_vinfo->white_point[1] = pconf->optical_info.white_point_y;
	disp_vinfo->luminance[0] = pconf->optical_info.luma_max;
	disp_vinfo->luminance[1] = pconf->optical_info.luma_min;

	lcd_drv->lcd_info->hdr_info.lumi_max = pconf->optical_info.luma_max;
}

void lcd_timing_init_config(struct lcd_config_s *pconf)
{
	unsigned short h_period, v_period, h_active, v_active;
	unsigned short hsync_bp, hsync_width, vsync_bp, vsync_width;
	unsigned short de_hstart, de_vstart;
	unsigned short hstart, hend, vstart, vend;
	unsigned short h_delay;

	switch (pconf->lcd_basic.lcd_type) {
	case LCD_TTL:
		h_delay = TTL_DELAY;
		break;
	default:
		h_delay = 0;
		break;
	}
	/* use period_dft to avoid period changing offset */
	h_period = pconf->lcd_timing.h_period_dft;
	v_period = pconf->lcd_timing.v_period_dft;
	h_active = pconf->lcd_basic.h_active;
	v_active = pconf->lcd_basic.v_active;
	hsync_bp = pconf->lcd_timing.hsync_bp;
	hsync_width = pconf->lcd_timing.hsync_width;
	vsync_bp = pconf->lcd_timing.vsync_bp;
	vsync_width = pconf->lcd_timing.vsync_width;

	de_hstart = hsync_bp + hsync_width;
	de_vstart = vsync_bp + vsync_width;

	pconf->lcd_timing.video_on_pixel = de_hstart - h_delay;
	pconf->lcd_timing.video_on_line = de_vstart;

	pconf->lcd_timing.de_hs_addr = de_hstart;
	pconf->lcd_timing.de_he_addr = de_hstart + h_active;
	pconf->lcd_timing.de_vs_addr = de_vstart;
	pconf->lcd_timing.de_ve_addr = de_vstart + v_active - 1;

	hstart = (de_hstart + h_period - hsync_bp - hsync_width) % h_period;
	hend = (de_hstart + h_period - hsync_bp) % h_period;
	pconf->lcd_timing.hs_hs_addr = hstart;
	pconf->lcd_timing.hs_he_addr = hend;
	pconf->lcd_timing.hs_vs_addr = 0;
	pconf->lcd_timing.hs_ve_addr = v_period - 1;

	pconf->lcd_timing.vs_hs_addr = (hstart + h_period) % h_period;
	pconf->lcd_timing.vs_he_addr = pconf->lcd_timing.vs_hs_addr;
	vstart = (de_vstart + v_period - vsync_bp - vsync_width) % v_period;
	vend = (de_vstart + v_period - vsync_bp) % v_period;
	pconf->lcd_timing.vs_vs_addr = vstart;
	pconf->lcd_timing.vs_ve_addr = vend;

	if (lcd_debug_print_flag) {
		LCDPR("hs_hs_addr=%d, hs_he_addr=%d\n"
		"hs_vs_addr=%d, hs_ve_addr=%d\n"
		"vs_hs_addr=%d, vs_he_addr=%d\n"
		"vs_vs_addr=%d, vs_ve_addr=%d\n",
		pconf->lcd_timing.hs_hs_addr, pconf->lcd_timing.hs_he_addr,
		pconf->lcd_timing.hs_vs_addr, pconf->lcd_timing.hs_ve_addr,
		pconf->lcd_timing.vs_hs_addr, pconf->lcd_timing.vs_he_addr,
		pconf->lcd_timing.vs_vs_addr, pconf->lcd_timing.vs_ve_addr);
	}
}

#if 0
/* change frame_rate for different vmode */
int lcd_vmode_change(struct lcd_config_s *pconf)
{
	unsigned int pclk = pconf->lcd_timing.lcd_clk_dft; /* avoid offset */
	unsigned char type = pconf->lcd_timing.fr_adjust_type;
	unsigned int h_period = pconf->lcd_basic.h_period;
	unsigned int v_period = pconf->lcd_basic.v_period;
	unsigned int sync_duration_num = pconf->lcd_timing.sync_duration_num;
	unsigned int sync_duration_den = pconf->lcd_timing.sync_duration_den;

	/* frame rate adjust */
	switch (type) {
	case 1: /* htotal adjust */
		h_period = ((pclk / v_period) * sync_duration_den * 10) /
				sync_duration_num;
		h_period = (h_period + 5) / 10; /* round off */
		if (pconf->lcd_basic.h_period != h_period) {
			LCDPR("%s: adjust h_period %u -> %u\n",
				__func__, pconf->lcd_basic.h_period, h_period);
			pconf->lcd_basic.h_period = h_period;
			/* check clk frac update */
			pclk = (h_period * v_period) / sync_duration_den *
				sync_duration_num;
			if (pconf->lcd_timing.lcd_clk != pclk)
				pconf->lcd_timing.lcd_clk = pclk;
		}
		break;
	case 2: /* vtotal adjust */
		v_period = ((pclk / h_period) * sync_duration_den * 10) /
				sync_duration_num;
		v_period = (v_period + 5) / 10; /* round off */
		if (pconf->lcd_basic.v_period != v_period) {
			LCDPR("%s: adjust v_period %u -> %u\n",
				__func__, pconf->lcd_basic.v_period, v_period);
			pconf->lcd_basic.v_period = v_period;
			/* check clk frac update */
			pclk = (h_period * v_period) / sync_duration_den *
				sync_duration_num;
			if (pconf->lcd_timing.lcd_clk != pclk)
				pconf->lcd_timing.lcd_clk = pclk;
		}
		break;
	case 0: /* pixel clk adjust */
	default:
		pclk = (h_period * v_period) / sync_duration_den *
			sync_duration_num;
		if (pconf->lcd_timing.lcd_clk != pclk) {
			LCDPR("%s: adjust pclk %u.%03uMHz -> %u.%03uMHz\n",
				__func__, (pconf->lcd_timing.lcd_clk / 1000000),
				((pconf->lcd_timing.lcd_clk / 1000) % 1000),
				(pclk / 1000000), ((pclk / 1000) % 1000));
			pconf->lcd_timing.lcd_clk = pclk;
		}
		break;
	}

	return 0;
}
#else
int lcd_vmode_change(struct lcd_config_s *pconf)
{
	unsigned char type = pconf->lcd_timing.fr_adjust_type;
	 /* use default value to avoid offset */
	unsigned int pclk = pconf->lcd_timing.lcd_clk_dft;
	unsigned int h_period = pconf->lcd_timing.h_period_dft;
	unsigned int v_period = pconf->lcd_timing.v_period_dft;
	unsigned int pclk_min = pconf->lcd_basic.lcd_clk_min;
	unsigned int pclk_max = pconf->lcd_basic.lcd_clk_max;
	unsigned int duration_num = pconf->lcd_timing.sync_duration_num;
	unsigned int duration_den = pconf->lcd_timing.sync_duration_den;
	char str[100];
	int len = 0;

	pconf->lcd_timing.clk_change = 0; /* clear clk flag */
	switch (type) {
	case 0: /* pixel clk adjust */
		pclk = (h_period * v_period) / duration_den * duration_num;
		if (pconf->lcd_timing.lcd_clk != pclk)
			pconf->lcd_timing.clk_change = LCD_CLK_PLL_CHANGE;
		break;
	case 1: /* htotal adjust */
		h_period = ((pclk / v_period) * duration_den * 100) /
				duration_num;
		h_period = (h_period + 99) / 100; /* round off */
		if (pconf->lcd_basic.h_period != h_period) {
			/* check clk frac update */
			pclk = (h_period * v_period) / duration_den *
				duration_num;
			if (pconf->lcd_timing.lcd_clk != pclk) {
				pconf->lcd_timing.clk_change =
					LCD_CLK_FRAC_UPDATE;
			}
		}
		break;
	case 2: /* vtotal adjust */
		v_period = ((pclk / h_period) * duration_den * 100) /
				duration_num;
		v_period = (v_period + 99) / 100; /* round off */
		if (pconf->lcd_basic.v_period != v_period) {
			/* check clk frac update */
			pclk = (h_period * v_period) / duration_den *
				duration_num;
			if (pconf->lcd_timing.lcd_clk != pclk) {
				pconf->lcd_timing.clk_change =
					LCD_CLK_FRAC_UPDATE;
			}
		}
		break;
	case 4: /* hdmi mode */
		if ((duration_num / duration_den) == 59) {
			/* pixel clk adjust */
			pclk = (h_period * v_period) /
				duration_den * duration_num;
			if (pconf->lcd_timing.lcd_clk != pclk)
				pconf->lcd_timing.clk_change =
					LCD_CLK_PLL_CHANGE;
		} else {
			/* htotal adjust */
			h_period = ((pclk / v_period) * duration_den * 100) /
					duration_num;
			h_period = (h_period + 99) / 100; /* round off */
			if (pconf->lcd_basic.h_period != h_period) {
				/* check clk frac update */
				pclk = (h_period * v_period) / duration_den *
					duration_num;
				if (pconf->lcd_timing.lcd_clk != pclk) {
					pconf->lcd_timing.clk_change =
						LCD_CLK_FRAC_UPDATE;
				}
			}
		}
		break;
	case 3: /* free adjust, use min/max range to calculate */
	default:
		v_period = ((pclk / h_period) * duration_den * 100) /
			duration_num;
		v_period = (v_period + 99) / 100; /* round off */
		if (v_period > pconf->lcd_basic.v_period_max) {
			v_period = pconf->lcd_basic.v_period_max;
			h_period = ((pclk / v_period) * duration_den * 100) /
				duration_num;
			h_period = (h_period + 99) / 100; /* round off */
			if (h_period > pconf->lcd_basic.h_period_max) {
				h_period = pconf->lcd_basic.h_period_max;
				pclk = (h_period * v_period) / duration_den *
					duration_num;
				if (pconf->lcd_timing.lcd_clk != pclk) {
					if (pclk > pclk_max) {
						pclk = pclk_max;
						LCDERR("invalid vmode\n");
						return -1;
					}
					pconf->lcd_timing.clk_change =
						LCD_CLK_PLL_CHANGE;
				}
			}
		} else if (v_period < pconf->lcd_basic.v_period_min) {
			v_period = pconf->lcd_basic.v_period_min;
			h_period = ((pclk / v_period) * duration_den * 100) /
				duration_num;
			h_period = (h_period + 99) / 100; /* round off */
			if (h_period < pconf->lcd_basic.h_period_min) {
				h_period = pconf->lcd_basic.h_period_min;
				pclk = (h_period * v_period) / duration_den *
					duration_num;
				if (pconf->lcd_timing.lcd_clk != pclk) {
					if (pclk < pclk_min) {
						pclk = pclk_min;
						LCDERR("invalid vmode\n");
						return -1;
					}
					pconf->lcd_timing.clk_change =
						LCD_CLK_PLL_CHANGE;
				}
			}
		}
		/* check clk frac update */
		if ((pconf->lcd_timing.clk_change & LCD_CLK_PLL_CHANGE) == 0) {
			pclk = (h_period * v_period) / duration_den *
				duration_num;
			if (pconf->lcd_timing.lcd_clk != pclk) {
				pconf->lcd_timing.clk_change =
					LCD_CLK_FRAC_UPDATE;
			}
		}
		break;
	}

	if (pconf->lcd_basic.v_period != v_period) {
		len += sprintf(str+len, "v_period %u->%u",
			pconf->lcd_basic.v_period, v_period);
		/* update v_period */
		pconf->lcd_basic.v_period = v_period;
	}
	if (pconf->lcd_basic.h_period != h_period) {
		if (len > 0)
			len += sprintf(str+len, ", ");
		len += sprintf(str+len, "h_period %u->%u",
			pconf->lcd_basic.h_period, h_period);
		/* update h_period */
		pconf->lcd_basic.h_period = h_period;
	}
	if (pconf->lcd_timing.lcd_clk != pclk) {
		if (len > 0)
			len += sprintf(str+len, ", ");
		len += sprintf(str+len, "pclk %u.%03uMHz->%u.%03uMHz",
			(pconf->lcd_timing.lcd_clk / 1000000),
			((pconf->lcd_timing.lcd_clk / 1000) % 1000),
			(pclk / 1000000), ((pclk / 1000) % 1000));
		pconf->lcd_timing.lcd_clk = pclk;
	}
	if (lcd_debug_print_flag) {
		if (len > 0)
			LCDPR("%s: %s\n", __func__, str);
	}

	return 0;
}
#endif

void lcd_clk_change(struct lcd_config_s *pconf)
{
	switch (pconf->lcd_timing.clk_change) {
	case LCD_CLK_PLL_CHANGE:
		lcd_clk_generate_parameter(pconf);
		lcd_clk_set(pconf);
		break;
	case LCD_CLK_FRAC_UPDATE:
		lcd_clk_update(pconf);
		break;
	default:
		break;
	}
}

void lcd_venc_change(struct lcd_config_s *pconf)
{
	unsigned int htotal, vtotal, frame_rate;

	htotal = lcd_vcbus_read(ENCL_VIDEO_MAX_PXCNT) + 1;
	vtotal = lcd_vcbus_read(ENCL_VIDEO_MAX_LNCNT) + 1;
	if (pconf->lcd_basic.h_period != htotal) {
		lcd_vcbus_write(ENCL_VIDEO_MAX_PXCNT,
			pconf->lcd_basic.h_period - 1);
	}
	if (pconf->lcd_basic.v_period != vtotal) {
		lcd_vcbus_write(ENCL_VIDEO_MAX_LNCNT,
			pconf->lcd_basic.v_period - 1);
	}
	if (lcd_debug_print_flag) {
		LCDPR("venc changed: %d,%d\n",
			pconf->lcd_basic.h_period,
			pconf->lcd_basic.v_period);
	}

	frame_rate = (pconf->lcd_timing.sync_duration_num * 100) /
		pconf->lcd_timing.sync_duration_den;
	frame_rate = (frame_rate + 50) / 100;
	aml_lcd_notifier_call_chain(LCD_EVENT_BACKLIGHT_UPDATE, &frame_rate);
}

void lcd_if_enable_retry(struct lcd_config_s *pconf)
{
	pconf->retry_enable_cnt = 0;
	while (pconf->retry_enable_flag) {
		if (pconf->retry_enable_cnt++ >= LCD_ENABLE_RETRY_MAX)
			break;
		LCDPR("retry enable...%d\n", pconf->retry_enable_cnt);
		aml_lcd_notifier_call_chain(LCD_EVENT_IF_POWER_OFF, NULL);
		msleep(1000);
		aml_lcd_notifier_call_chain(LCD_EVENT_IF_POWER_ON, NULL);
	}
	pconf->retry_enable_cnt = 0;
}

void lcd_vout_notify_mode_change_pre(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_drv->viu_sel == 1) {
		vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE_PRE,
			&lcd_drv->lcd_info->mode);
	} else if (lcd_drv->viu_sel == 2) {
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
		vout2_notifier_call_chain(VOUT_EVENT_MODE_CHANGE_PRE,
			&lcd_drv->lcd_info->mode);
#endif
	}
}

void lcd_vout_notify_mode_change(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (lcd_drv->viu_sel == 1) {
		vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE,
			&lcd_drv->lcd_info->mode);
	} else if (lcd_drv->viu_sel == 2) {
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
		vout2_notifier_call_chain(VOUT_EVENT_MODE_CHANGE,
			&lcd_drv->lcd_info->mode);
#endif
	}
}
