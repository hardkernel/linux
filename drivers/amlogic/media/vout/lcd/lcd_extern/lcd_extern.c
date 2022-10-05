/*
 * drivers/amlogic/media/vout/lcd/lcd_extern/lcd_extern.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/amlogic/i2c-amlogic.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/amlogic/media/vout/lcd/lcd_extern.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#include <linux/amlogic/media/vout/lcd/lcd_notify.h>
#include "lcd_extern.h"

static struct device *lcd_extern_dev;
static struct aml_lcd_extern_driver_s *lcd_ext_driver;

static unsigned char lcd_ext_config_load;
static unsigned char *table_init_on_dft;
static unsigned char *table_init_off_dft;

struct lcd_ext_gpio_s {
	char name[15];
	struct gpio_desc *gpio;
	int probe_flag;
	int register_flag;
};

static struct lcd_ext_gpio_s lcd_extern_gpio[LCD_EXTERN_GPIO_NUM_MAX] = {
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
};

static struct lcd_extern_config_s lcd_extern_config = {
	.index = LCD_EXTERN_INDEX_INVALID,
	.name = "invalid",
	.type = LCD_EXTERN_MAX,
	.status = 0,
	.pinmux_valid = 0,
	.pinmux_gpio_off = 2,
	.key_valid = 0,
	.addr_sel = 0,

	.i2c_addr = LCD_EXT_I2C_ADDR_INVALID,
	.i2c_addr2 = LCD_EXT_I2C_ADDR_INVALID,
	.i2c_bus = LCD_EXT_I2C_BUS_INVALID,
	.i2c_sck_gpio = LCD_EXT_GPIO_INVALID,
	.i2c_sda_gpio = LCD_EXT_GPIO_INVALID,

	.spi_gpio_cs = LCD_EXT_GPIO_INVALID,
	.spi_gpio_clk = LCD_EXT_GPIO_INVALID,
	.spi_gpio_data = LCD_EXT_GPIO_INVALID,
	.spi_clk_freq = 0,
	.spi_delay_us = 0,
	.spi_clk_pol = 1,

	.cmd_size = 0,
	.table_init_loaded = 0, /* internal use */
	.table_init_on_cnt = 0,
	.table_init_off_cnt = 0,
	.table_init_on = NULL,
	.table_init_off = NULL,
};

struct aml_lcd_extern_driver_s *aml_lcd_extern_get_driver(int index)
{
	if (lcd_ext_driver == NULL) {
		EXTERR("invalid driver\n");
		return NULL;
	}

	if (index >= LCD_EXTERN_INDEX_INVALID) {
		EXTERR("invalid driver index: %d\n", index);
		return NULL;
	}
	if (lcd_ext_driver->config->index == index)
		return lcd_ext_driver;

	EXTERR("invalid driver index: %d\n", index);
	return NULL;
}

#ifdef CONFIG_OF
void lcd_extern_gpio_probe(unsigned char index)
{
	struct lcd_ext_gpio_s *ext_gpio;
	const char *str;
	int ret;

	if (index >= LCD_EXTERN_GPIO_NUM_MAX) {
		EXTERR("gpio index %d, exit\n", index);
		return;
	}
	ext_gpio = &lcd_extern_gpio[index];
	if (ext_gpio->probe_flag) {
		if (lcd_debug_print_flag) {
			EXTPR("gpio %s[%d] is already registered\n",
				ext_gpio->name, index);
		}
		return;
	}

	/* get gpio name */
	ret = of_property_read_string_index(lcd_extern_dev->of_node,
		"extern_gpio_names", index, &str);
	if (ret) {
		EXTERR("failed to get extern_gpio_names: %d\n", index);
		str = "unknown";
	}
	strcpy(ext_gpio->name, str);

	/* init gpio flag */
	ext_gpio->probe_flag = 1;
	ext_gpio->register_flag = 0;
}

void lcd_extern_gpio_unregister(int index)
{
	struct lcd_ext_gpio_s *ext_gpio;

	if (index >= LCD_EXTERN_GPIO_NUM_MAX) {
		EXTERR("gpio index %d, exit\n", index);
		return;
	}
	ext_gpio = &lcd_extern_gpio[index];
	if (ext_gpio->probe_flag == 0) {
		if (lcd_debug_print_flag) {
			EXTPR("gpio %s[%d] is already registered\n",
				ext_gpio->name, index);
		}
		return;
	}
	if (ext_gpio->register_flag) {
		EXTPR("%s: gpio %s[%d] is already registered\n",
			__func__, ext_gpio->name, index);
		return;
	}
	if (IS_ERR(ext_gpio->gpio)) {
		EXTERR("register gpio %s[%d]: %p, err: %d\n",
			ext_gpio->name, index, ext_gpio->gpio,
			IS_ERR(ext_gpio->gpio));
		ext_gpio->gpio = NULL;
		return;
	}

	/* release gpio */
	devm_gpiod_put(lcd_extern_dev, ext_gpio->gpio);
	ext_gpio->probe_flag = 0;
	ext_gpio->register_flag = 0;
	if (lcd_debug_print_flag)
		EXTPR("release gpio %s[%d]\n", ext_gpio->name, index);
}

static int lcd_extern_gpio_register(unsigned char index, int init_value)
{
	struct lcd_ext_gpio_s *ext_gpio;
	int value;

	if (index >= LCD_EXTERN_GPIO_NUM_MAX) {
		EXTERR("gpio index %d, exit\n", index);
		return -1;
	}
	ext_gpio = &lcd_extern_gpio[index];
	if (ext_gpio->probe_flag == 0) {
		EXTERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return -1;
	}
	if (ext_gpio->register_flag) {
		EXTPR("%s: gpio %s[%d] is already registered\n",
			__func__, ext_gpio->name, index);
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
	ext_gpio->gpio = devm_gpiod_get_index(
		lcd_extern_dev, "extern", index, value);
	if (IS_ERR(ext_gpio->gpio)) {
		EXTERR("register gpio %s[%d]: %p, err: %d\n",
			ext_gpio->name, index, ext_gpio->gpio,
			IS_ERR(ext_gpio->gpio));
		ext_gpio->gpio = NULL;
		return -1;
	}
	ext_gpio->register_flag = 1;
	if (lcd_debug_print_flag) {
		EXTPR("register gpio %s[%d]: %p, init value: %d\n",
			ext_gpio->name, index, ext_gpio->gpio, init_value);
	}

	return 0;
}
#endif

void lcd_extern_gpio_set(unsigned char index, int value)
{
	struct lcd_ext_gpio_s *ext_gpio;

	if (index >= LCD_EXTERN_GPIO_NUM_MAX) {
		EXTERR("gpio index %d, exit\n", index);
		return;
	}
	ext_gpio = &lcd_extern_gpio[index];
	if (ext_gpio->probe_flag == 0) {
		EXTERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return;
	}
	if (ext_gpio->register_flag == 0) {
		lcd_extern_gpio_register(index, value);
		return;
	}

	if (IS_ERR_OR_NULL(ext_gpio->gpio)) {
		EXTERR("gpio %s[%d]: %p, err: %ld\n",
			ext_gpio->name, index, ext_gpio->gpio,
			PTR_ERR(ext_gpio->gpio));
		return;
	}

	switch (value) {
	case LCD_GPIO_OUTPUT_LOW:
	case LCD_GPIO_OUTPUT_HIGH:
		gpiod_direction_output(ext_gpio->gpio, value);
		break;
	case LCD_GPIO_INPUT:
	default:
		gpiod_direction_input(ext_gpio->gpio);
		break;
	}
	if (lcd_debug_print_flag) {
		EXTPR("set gpio %s[%d] value: %d\n",
		ext_gpio->name, index, value);
	}
}

unsigned int lcd_extern_gpio_get(unsigned char index)
{
	struct lcd_ext_gpio_s *ext_gpio;

	if (index >= LCD_EXTERN_GPIO_NUM_MAX) {
		EXTERR("gpio index %d, exit\n", index);
		return -1;
	}
	ext_gpio = &lcd_extern_gpio[index];
	if (ext_gpio->probe_flag == 0) {
		EXTERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return -1;
	}
	if (ext_gpio->register_flag == 0) {
		EXTERR("%s: gpio %s[%d] is not registered\n",
			__func__, ext_gpio->name, index);
		return -1;
	}
	if (IS_ERR_OR_NULL(ext_gpio->gpio)) {
		EXTERR("gpio %s[%d]: %p, err: %ld\n",
			ext_gpio->name, index, ext_gpio->gpio,
			PTR_ERR(ext_gpio->gpio));
		return -1;
	}

	return gpiod_get_value(ext_gpio->gpio);
}

#define LCD_EXTERN_PINMUX_MAX    3
static char *lcd_extern_pinmux_str[LCD_EXTERN_PINMUX_MAX] = {
	"extern_on",   /* 0 */
	"extern_off",  /* 1 */
	"none",
};

void lcd_extern_pinmux_set(int status)
{
	int index = 0xff;

	if (lcd_ext_driver == NULL)
		return;

	if (lcd_ext_driver->config->pinmux_valid == 0) {
		if (lcd_debug_print_flag)
			EXTPR("%s: pinmux invalid, bypass\n", __func__);
		return;
	}

	if (lcd_debug_print_flag)
		EXTPR("%s: %d\n", __func__, status);

	index = (status) ? 0 : 1;

	if (lcd_ext_driver->pinmux_flag == index) {
		EXTPR("%s(%d) pinmux %s is already selected\n",
			lcd_ext_driver->config->name,
			lcd_ext_driver->config->index,
			lcd_extern_pinmux_str[index]);
		return;
	}

	/* request pinmux */
	lcd_ext_driver->pin = devm_pinctrl_get_select(lcd_extern_dev,
		lcd_extern_pinmux_str[index]);
	if (IS_ERR(lcd_ext_driver->pin)) {
		EXTERR("set %s(%d) pinmux %s error\n",
			lcd_ext_driver->config->name,
			lcd_ext_driver->config->index,
			lcd_extern_pinmux_str[index]);
	} else {
		if (lcd_debug_print_flag) {
			EXTPR("set %s(%d) pinmux %s ok\n",
				lcd_ext_driver->config->name,
				lcd_ext_driver->config->index,
				lcd_extern_pinmux_str[index]);
		}
	}
	lcd_ext_driver->pinmux_flag = index;
}

#ifdef CONFIG_OF
static unsigned char lcd_extern_get_i2c_bus_str(const char *str)
{
	unsigned char i2c_bus;

	if (strncmp(str, "i2c_bus_ao", 10) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_4;
	else if (strncmp(str, "i2c_bus_a", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_0;
	else if (strncmp(str, "i2c_bus_b", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_1;
	else if (strncmp(str, "i2c_bus_c", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_2;
	else if (strncmp(str, "i2c_bus_d", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_3;
	else if (strncmp(str, "i2c_bus_0", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_0;
	else if (strncmp(str, "i2c_bus_1", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_1;
	else if (strncmp(str, "i2c_bus_2", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_2;
	else if (strncmp(str, "i2c_bus_3", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_3;
	else if (strncmp(str, "i2c_bus_4", 9) == 0)
		i2c_bus = LCD_EXT_I2C_BUS_4;
	else {
		i2c_bus = LCD_EXT_I2C_BUS_INVALID;
		EXTERR("invalid i2c_bus: %s\n", str);
	}

	return i2c_bus;
}

struct device_node *aml_lcd_extern_get_dts_child(int index)
{
	char propname[30];
	struct device_node *child;

	sprintf(propname, "extern_%d", index);
	child = of_get_child_by_name(lcd_extern_dev->of_node, propname);
	return child;
}

static int lcd_extern_init_table_dynamic_size_load_dts(
		struct device_node *of_node,
		struct lcd_extern_config_s *extconf, int flag)
{
	unsigned char cmd_size, index, type;
	int i = 0, j, val, max_len, step = 0, ret = 0;
	unsigned char *table;
	char propname[20];

	if (flag) {
		table = table_init_on_dft;
		max_len = LCD_EXTERN_INIT_ON_MAX;
		sprintf(propname, "init_on");
	} else {
		table = table_init_off_dft;
		max_len = LCD_EXTERN_INIT_OFF_MAX;
		sprintf(propname, "init_off");
	}
	if (table == NULL) {
		EXTERR("%s: init_table is null\n", __func__);
		return -1;
	}

	switch (extconf->type) {
	case LCD_EXTERN_I2C:
	case LCD_EXTERN_SPI:
		while ((i + 1) < max_len) {
			/* type */
			ret = of_property_read_u32_index(
				of_node, propname, i, &val);
			if (ret) {
				EXTERR("%s: get %s type failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i] = (unsigned char)val;
			type = table[i];
			/* cmd_size */
			ret = of_property_read_u32_index(
				of_node, propname, (i+1), &val);
			if (ret) {
				EXTERR("%s: get %s cmd_size failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i+1] = (unsigned char)val;
			cmd_size = table[i+1];

			if (type == LCD_EXT_CMD_TYPE_END)
				break;
			if (cmd_size == 0)
				goto init_table_dynamic_i2c_spi_dts_next;
			if ((i + 2 + cmd_size) > max_len) {
				EXTERR(
				"%s: %s cmd_size out of support, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}

			/* data */
			for (j = 0; j < cmd_size; j++) {
				ret = of_property_read_u32_index(
					of_node, propname, (i+2+j), &val);
				if (ret) {
					EXTERR(
					"%s: get %s data failed, step %d\n",
						extconf->name, propname, step);
					table[i] = LCD_EXT_CMD_TYPE_END;
					table[i+1] = 0;
					return -1;
				}
				table[i+2+j] = (unsigned char)val;
			}
			if (type == LCD_EXT_CMD_TYPE_GPIO) {
				/* gpio probe */
				index = table[i+2];
				if (index < LCD_EXTERN_GPIO_NUM_MAX)
					lcd_extern_gpio_probe(index);
			}

init_table_dynamic_i2c_spi_dts_next:
			i += (cmd_size + 2);
			step++;
		}
		if (flag)
			extconf->table_init_on_cnt = i + 2;
		else
			extconf->table_init_off_cnt = i + 2;
		break;
	case LCD_EXTERN_MIPI:
		while ((i + 1) < max_len) {
			/* type */
			ret = of_property_read_u32_index(
				of_node, propname, i, &val);
			if (ret) {
				EXTERR("%s: get %s type failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i] = (unsigned char)val;
			type = table[i];
			/* cmd_size */
			ret = of_property_read_u32_index(
				of_node, propname, (i+1), &val);
			if (ret) {
				EXTERR("%s: get %s cmd_size failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i+1] = (unsigned char)val;
			cmd_size = table[i+1];

			if (type == LCD_EXT_CMD_TYPE_END) {
				if ((cmd_size == 0xff) || (cmd_size == 0))
					break;
				cmd_size = 0;
			}
			if (cmd_size == 0)
				goto init_table_dynamic_mipi_dts_next;
			if ((i + 2 + cmd_size) > max_len) {
				EXTERR(
				"%s: %s cmd_size out of support, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}

			for (j = 0; j < cmd_size; j++) {
				ret = of_property_read_u32_index(
					of_node, propname, (i+2+j), &val);
				if (ret) {
					EXTERR("%s: get %s failed, step %d\n",
						extconf->name, propname, step);
					table[i] = LCD_EXT_CMD_TYPE_END;
					table[i+1] = 0;
					return -1;
				}
				table[i+2+j] = (unsigned char)val;
			}

			if (type == LCD_EXT_CMD_TYPE_GPIO) {
				/* gpio probe */
				index = table[i+2];
				if (index < LCD_EXTERN_GPIO_NUM_MAX)
					lcd_extern_gpio_probe(index);
			}

init_table_dynamic_mipi_dts_next:
			i += (cmd_size + 2);
			step++;
		}
		if (flag)
			extconf->table_init_on_cnt = i + 2;
		else
			extconf->table_init_off_cnt = i + 2;
		break;
	default:
		break;
	}

	return 0;
}

static int lcd_extern_init_table_fixed_size_load_dts(
		struct device_node *of_node,
		struct lcd_extern_config_s *extconf, int flag)
{
	unsigned char cmd_size, index;
	int i = 0, j, val, max_len, step = 0, ret = 0;
	unsigned char *table;
	char propname[20];

	cmd_size = extconf->cmd_size;
	if (flag) {
		table = table_init_on_dft;
		max_len = LCD_EXTERN_INIT_ON_MAX;
		sprintf(propname, "init_on");
	} else {
		table = table_init_off_dft;
		max_len = LCD_EXTERN_INIT_OFF_MAX;
		sprintf(propname, "init_off");
	}
	if (table == NULL) {
		EXTERR("%s: init_table is null\n", __func__);
		return -1;
	}

	while (i < max_len) { /* group detect */
		if ((i + cmd_size) > max_len) {
			EXTERR("%s: %s cmd_size out of support, step %d\n",
				extconf->name, propname, step);
			table[i] = LCD_EXT_CMD_TYPE_END;
			return -1;
		}
		for (j = 0; j < cmd_size; j++) {
			ret = of_property_read_u32_index(
				of_node, propname, (i+j), &val);
			if (ret) {
				EXTERR("%s: get %s failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				return -1;
			}
			table[i+j] = (unsigned char)val;
		}
		if (table[i] == LCD_EXT_CMD_TYPE_END)
			break;
		if (table[i] == LCD_EXT_CMD_TYPE_GPIO) {
			/* gpio probe */
			index = table[i+1];
			if (index < LCD_EXTERN_GPIO_NUM_MAX)
				lcd_extern_gpio_probe(index);
		}
		i += cmd_size;
		step++;
	}

	if (flag)
		extconf->table_init_on_cnt = i + cmd_size;
	else
		extconf->table_init_off_cnt = i + cmd_size;

	return 0;
}

static int lcd_extern_get_config_dts(struct device_node *of_node,
		struct lcd_extern_config_s *extconf)
{
	unsigned int ext_index = LCD_EXTERN_INDEX_INVALID;
	char ext_propname[20];
	struct device_node *child;
	const char *str;
	int val;
	int ret;

	extconf->index = LCD_EXTERN_INDEX_INVALID;
	extconf->type = LCD_EXTERN_MAX;
	extconf->status = 0;
	extconf->table_init_loaded = 0;

	aml_lcd_notifier_call_chain(LCD_EVENT_EXTERN_SEL, &ext_index);
	if (ext_index == LCD_EXTERN_INDEX_INVALID) {
		EXTPR("%s: invalid index\n", __func__);
		return -1;
	}
	sprintf(ext_propname, "extern_%d", ext_index);
	if (lcd_debug_print_flag)
		EXTPR("%s: %s\n", __func__, ext_propname);

	child = of_get_child_by_name(of_node, ext_propname);
	if (child == NULL) {
		EXTERR("failed to get %s\n", ext_propname);
		return -1;
	}

	ret = of_property_read_u32(child, "index", &val);
	if (ret) {
		EXTERR("get index failed, exit\n");
		return -1;
	}
	extconf->index = (unsigned char)val;
	if (extconf->index == LCD_EXTERN_INDEX_INVALID) {
		if (lcd_debug_print_flag)
			EXTPR("index %d is invalid\n", extconf->index);
		return -1;
	}

	ret = of_property_read_string(child, "status", &str);
	if (ret) {
		EXTERR("get index %d status failed\n", extconf->index);
		return -1;
	}
	if (lcd_debug_print_flag)
		EXTPR("index %d status = %s\n", extconf->index, str);
	if (strncmp(str, "okay", 2) == 0)
		extconf->status = 1;
	else
		return -1;

	ret = of_property_read_string(child, "extern_name", &str);
	if (ret) {
		EXTERR("get extern_name failed\n");
		strncpy(extconf->name, "none", LCD_EXTERN_NAME_LEN_MAX);
	} else {
		strncpy(extconf->name, str, LCD_EXTERN_NAME_LEN_MAX);
	}
	/* ensure string ending */
	extconf->name[LCD_EXTERN_NAME_LEN_MAX-1] = '\0';
	EXTPR("load config: %s[%d]\n", extconf->name, extconf->index);

	ret = of_property_read_u32(child, "type", &extconf->type);
	if (ret) {
		extconf->type = LCD_EXTERN_MAX;
		EXTERR("%s: get type failed, exit\n", extconf->name);
		return -1;
	}

	switch (extconf->type) {
	case LCD_EXTERN_I2C:
		if (extconf->i2c_bus == LCD_EXT_I2C_BUS_INVALID)
			EXTERR("%s: get i2c_bus failed\n", extconf->name);
		if (lcd_debug_print_flag) {
			EXTPR("%s: i2c_bus = %d\n",
				extconf->name, extconf->i2c_bus);
		}
		ret = of_property_read_u32(child, "i2c_address", &val);
		if (ret) {
			EXTERR("%s: get i2c_address failed, exit\n",
				extconf->name);
			extconf->i2c_addr = LCD_EXT_I2C_ADDR_INVALID;
			return -1;
		}
		extconf->i2c_addr = (unsigned char)val;
		if (lcd_debug_print_flag) {
			EXTPR("%s: i2c_address = 0x%02x\n",
				extconf->name, extconf->i2c_addr);
		}
		ret = of_property_read_u32(child, "i2c_address2", &val);
		if (ret) {
			ret = of_property_read_u32(child,
				"i2c_second_address", &val);
			if (ret) {
				EXTPR("%s: no i2c_address2 exist\n",
					extconf->name);
				extconf->i2c_addr2 = LCD_EXT_I2C_ADDR_INVALID;
			} else {
				extconf->i2c_addr2 = (unsigned char)val;
			}
		} else {
			extconf->i2c_addr2 = (unsigned char)val;
		}
		if (lcd_debug_print_flag) {
			EXTPR("%s: i2c_address2 = 0x%02x\n",
				extconf->name, extconf->i2c_addr2);
		}

		ret = of_property_read_u32(child, "cmd_size", &val);
		if (ret) {
			EXTPR("%s: no cmd_size\n", extconf->name);
			extconf->cmd_size = 0;
		} else {
			extconf->cmd_size = (unsigned char)val;
		}
		if (lcd_debug_print_flag) {
			EXTPR("%s: cmd_size = %d\n",
				extconf->name, extconf->cmd_size);
		}
		if (extconf->cmd_size == 0)
			break;

		if (extconf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = lcd_extern_init_table_dynamic_size_load_dts(
				child, extconf, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_dynamic_size_load_dts(
				child, extconf, 0);
		} else {
			ret = lcd_extern_init_table_fixed_size_load_dts(
				child, extconf, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_fixed_size_load_dts(
				child, extconf, 0);
		}
		if (ret == 0)
			extconf->table_init_loaded = 1;
		break;
	case LCD_EXTERN_SPI:
		ret = of_property_read_u32(child, "gpio_spi_cs", &val);
		if (ret) {
			EXTERR("%s: get gpio_spi_cs failed, exit\n",
				extconf->name);
			extconf->spi_gpio_cs = LCD_EXT_GPIO_INVALID;
			return -1;
		}
		extconf->spi_gpio_cs = val;
		lcd_extern_gpio_probe(val);
		if (lcd_debug_print_flag) {
			EXTPR("%s: spi_gpio_cs: %d\n",
				extconf->name, extconf->spi_gpio_cs);
		}
		ret = of_property_read_u32(child, "gpio_spi_clk", &val);
		if (ret) {
			EXTERR("%s: get gpio_spi_clk failed, exit\n",
				extconf->name);
			extconf->spi_gpio_clk = LCD_EXT_GPIO_INVALID;
			return -1;
		}
		extconf->spi_gpio_clk = val;
		lcd_extern_gpio_probe(val);
		if (lcd_debug_print_flag) {
			EXTPR("%s: spi_gpio_clk: %d\n",
				extconf->name, extconf->spi_gpio_clk);
		}
		ret = of_property_read_u32(child, "gpio_spi_data", &val);
		if (ret) {
			EXTERR("%s: get gpio_spi_data failed, exit\n",
				extconf->name);
			extconf->spi_gpio_data = LCD_EXT_GPIO_INVALID;
			return -1;
		}
		extconf->spi_gpio_data = val;
		lcd_extern_gpio_probe(val);
		if (lcd_debug_print_flag) {
			EXTPR("%s: spi_gpio_data: %d\n",
				extconf->name, extconf->spi_gpio_data);
		}
		ret = of_property_read_u32(child, "spi_clk_freq", &val);
		if (ret) {
			EXTERR(
			"%s: get spi_clk_freq failed, default to %dKHz\n",
				extconf->name, LCD_EXT_SPI_CLK_FREQ_DFT);
			extconf->spi_clk_freq = LCD_EXT_SPI_CLK_FREQ_DFT;
		} else {
			extconf->spi_clk_freq = val;
		}
		ret = of_property_read_u32(child, "spi_clk_pol", &val);
		if (ret) {
			EXTERR("%s: get spi_clk_pol failed, default to 1\n",
				extconf->name);
			extconf->spi_clk_pol = 1;
		} else {
			extconf->spi_clk_pol = (unsigned char)val;
		}
		if (lcd_debug_print_flag) {
			EXTPR("%s: spi_clk_freq: %dKHz, spi_clk_pol: %d\n",
				extconf->name, extconf->spi_clk_freq,
				extconf->spi_clk_pol);
		}
		ret = of_property_read_u32(child, "cmd_size", &val);
		if (ret) {
			EXTPR("%s: no cmd_size\n", extconf->name);
			extconf->cmd_size = 0;
		} else {
			extconf->cmd_size = (unsigned char)val;
		}
		if (lcd_debug_print_flag) {
			EXTPR("%s: cmd_size: %d\n",
				extconf->name, extconf->cmd_size);
		}
		if (extconf->cmd_size == 0)
			break;

		if (extconf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = lcd_extern_init_table_dynamic_size_load_dts(
				child, extconf, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_dynamic_size_load_dts(
				child, extconf, 0);
		} else {
			ret = lcd_extern_init_table_fixed_size_load_dts(
				child, extconf, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_fixed_size_load_dts(
				child, extconf, 0);
		}
		if (ret == 0)
			extconf->table_init_loaded = 1;
		break;
	case LCD_EXTERN_MIPI:
		ret = of_property_read_u32(child, "cmd_size", &val);
		if (ret) {
			EXTPR("%s: no cmd_size\n", extconf->name);
			extconf->cmd_size = 0;
		} else {
			extconf->cmd_size = (unsigned char)val;
		}
		if (lcd_debug_print_flag) {
			EXTPR("%s: cmd_size = %d\n",
				extconf->name, extconf->cmd_size);
		}
		if (extconf->cmd_size != LCD_EXT_CMD_SIZE_DYNAMIC)
			break;

		ret = lcd_extern_init_table_dynamic_size_load_dts(
			child, extconf, 1);
		if (ret)
			break;
		ret = lcd_extern_init_table_dynamic_size_load_dts(
			child, extconf, 0);
		if (ret == 0)
			extconf->table_init_loaded = 1;
		break;
	default:
		break;
	}

	return 0;
}
#endif

static int lcd_extern_init_table_dynamic_size_load_unifykey(
		struct lcd_extern_config_s *extconf, unsigned char *p,
		int key_len, int len, int flag)
{
	unsigned char cmd_size = 0;
	unsigned char index;
	int i = 0, j, max_len, ret = 0;
	unsigned char *table, *buf;
	char propname[20];

	if (flag) {
		table = table_init_on_dft;
		max_len = LCD_EXTERN_INIT_ON_MAX;
		sprintf(propname, "init_on");
		buf = p;
	} else {
		table = table_init_off_dft;
		max_len = LCD_EXTERN_INIT_OFF_MAX;
		sprintf(propname, "init_off");
		buf = p + extconf->table_init_on_cnt;
	}
	if (table == NULL) {
		EXTERR("%s: init_table is null\n", __func__);
		return -1;
	}

	switch (extconf->type) {
	case LCD_EXTERN_I2C:
	case LCD_EXTERN_SPI:
		while ((i + 1) < max_len) {
			/* type */
			len += 1;
			ret = lcd_unifykey_len_check(key_len, len);
			if (ret) {
				EXTERR("%s: get %s failed\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i] = *(buf + LCD_UKEY_EXT_INIT + i);
			/* cmd_size */
			len += 1;
			ret = lcd_unifykey_len_check(key_len, len);
			if (ret) {
				EXTERR("%s: get %s failed\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i+1] = *(buf + LCD_UKEY_EXT_INIT + i + 1);
			cmd_size = table[i+1];

			if (table[i] == LCD_EXT_CMD_TYPE_END)
				break;
			if (cmd_size == 0)
				goto init_table_dynamic_i2c_spi_ukey_next;
			if ((i + 2 + cmd_size) > max_len) {
				EXTERR("%s: %s cmd_size out of support\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}

			/* step3: data */
			len += cmd_size;
			ret = lcd_unifykey_len_check(key_len, len);
			if (ret) {
				EXTERR("%s: get %s failed\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			for (j = 0; j < cmd_size; j++) {
				table[i+2+j] =
					*(buf + LCD_UKEY_EXT_INIT + i + 2 + j);
			}

			if (table[i] == LCD_EXT_CMD_TYPE_GPIO) {
				/* gpio probe */
				index = table[i+1];
				if (index < LCD_EXTERN_GPIO_NUM_MAX)
					lcd_extern_gpio_probe(index);
			}
init_table_dynamic_i2c_spi_ukey_next:
			i += (cmd_size + 2);
		}
		if (flag)
			extconf->table_init_on_cnt = i + 2;
		else
			extconf->table_init_off_cnt = i + 2;
		break;
	case LCD_EXTERN_MIPI:
		while ((i + 1) < max_len) {
			/* type */
			len += 1;
			ret = lcd_unifykey_len_check(key_len, len);
			if (ret) {
				EXTERR("%s: get %s failed\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i] = *(buf + LCD_UKEY_EXT_INIT + i);
			/* cmd_size */
			len += 1;
			ret = lcd_unifykey_len_check(key_len, len);
			if (ret) {
				EXTERR("%s: get %s failed\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i+1] = *(buf + LCD_UKEY_EXT_INIT + i + 1);
			cmd_size = table[i+1];

			if (table[i] == LCD_EXT_CMD_TYPE_END) {
				if ((cmd_size == 0xff) || (cmd_size == 0))
					break;
				cmd_size = 0;
			}
			if (cmd_size == 0)
				goto init_table_dynamic_mipi_ukey_next;
			if ((i + 2 + cmd_size) > max_len) {
				EXTERR("%s: %s cmd_size out of max\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}

			/* data */
			len += cmd_size;
			ret = lcd_unifykey_len_check(key_len, len);
			if (ret) {
				EXTERR("%s: get %s failed\n",
					extconf->name, propname);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			for (j = 0; j < cmd_size; j++) {
				table[i+2+j] =
					*(buf + LCD_UKEY_EXT_INIT + i + 2 + j);
			}

			if (table[i] == LCD_EXT_CMD_TYPE_GPIO) {
				/* gpio probe */
				index = table[i+2];
				if (index < LCD_EXTERN_GPIO_NUM_MAX)
					lcd_extern_gpio_probe(index);
			}
init_table_dynamic_mipi_ukey_next:
			i += (cmd_size + 2);
		}
		if (flag)
			extconf->table_init_on_cnt = i + 2;
		else
			extconf->table_init_off_cnt = i + 2;
		break;
	default:
		break;
	}

	return 0;
}

static int lcd_extern_init_table_fixed_size_load_unifykey(
		struct lcd_extern_config_s *extconf, unsigned char *p,
		int key_len, int len, int flag)
{
	unsigned char cmd_size;
	unsigned char index;
	int i = 0, j, max_len, ret = 0;
	unsigned char *table, *buf;
	char propname[20];

	cmd_size = extconf->cmd_size;
	if (flag) {
		table = table_init_on_dft;
		max_len = LCD_EXTERN_INIT_ON_MAX;
		sprintf(propname, "init_on");
		buf = p;
	} else {
		table = table_init_off_dft;
		max_len = LCD_EXTERN_INIT_OFF_MAX;
		sprintf(propname, "init_off");
		buf = p + extconf->table_init_on_cnt;
	}
	if (table == NULL) {
		EXTERR("%s: init_table is null\n", __func__);
		return -1;
	}

	while (i < max_len) {
		if ((i + cmd_size) > max_len) {
			EXTERR("%s: %s cmd_size out of max\n",
				extconf->name, propname);
			table[i] = LCD_EXT_CMD_TYPE_END;
			return -1;
		}
		len += cmd_size;
		ret = lcd_unifykey_len_check(key_len, len);
		if (ret) {
			EXTERR("%s: get %s failed\n",
				extconf->name, propname);
			table[i] = LCD_EXT_CMD_TYPE_END;
			for (j = 1; j < cmd_size; j++)
				table[i+j] = 0;
			return -1;
		}
		for (j = 0; j < cmd_size; j++)
			table[i+j] = *(buf + LCD_UKEY_EXT_INIT + i + j);
		if (table[i] == LCD_EXT_CMD_TYPE_END)
			break;
		if (table[i] == LCD_EXT_CMD_TYPE_GPIO) {
			/* gpio probe */
			index = table[i+1];
			if (index < LCD_EXTERN_GPIO_NUM_MAX)
				lcd_extern_gpio_probe(index);
		}
		i += cmd_size;
	}
	if (flag)
		extconf->table_init_on_cnt = i + cmd_size;
	else
		extconf->table_init_off_cnt = i + cmd_size;

	return 0;
}

static int lcd_extern_get_config_unifykey(struct lcd_extern_config_s *extconf)
{
	unsigned char *para, *p;
	int key_len, len;
	const char *str;
	struct aml_lcd_unifykey_header_s ext_header;
	int ret;

	extconf->index = LCD_EXTERN_INDEX_INVALID;
	extconf->type = LCD_EXTERN_MAX;
	extconf->table_init_loaded = 0;

	para = kmalloc((sizeof(unsigned char) * LCD_UKEY_LCD_EXT_SIZE),
		GFP_KERNEL);
	if (!para) {
		EXTERR("%s: Not enough memory\n", __func__);
		return -1;
	}
	key_len = LCD_UKEY_LCD_EXT_SIZE;
	memset(para, 0, (sizeof(unsigned char) * key_len));
	ret = lcd_unifykey_get("lcd_extern", para, &key_len);
	if (ret) {
		kfree(para);
		return -1;
	}

	/* check lcd_extern unifykey length */
	len = 10 + 33 + 10;
	ret = lcd_unifykey_len_check(key_len, len);
	if (ret) {
		EXTERR("unifykey length is not correct\n");
		kfree(para);
		return -1;
	}

	/* header: 10byte */
	lcd_unifykey_header_check(para, &ext_header);
	if (lcd_debug_print_flag) {
		EXTPR("unifykey header:\n");
		EXTPR("crc32             = 0x%08x\n", ext_header.crc32);
		EXTPR("data_len          = %d\n", ext_header.data_len);
		EXTPR("version           = 0x%04x\n", ext_header.version);
		EXTPR("reserved          = 0x%04x\n", ext_header.reserved);
	}

	/* basic: 33byte */
	p = para;
	str = (const char *)(p + LCD_UKEY_HEAD_SIZE);
	strncpy(extconf->name, str, LCD_EXTERN_NAME_LEN_MAX);
	/* ensure string ending */
	extconf->name[LCD_EXTERN_NAME_LEN_MAX-1] = '\0';
	extconf->index = *(p + LCD_UKEY_EXT_INDEX);
	extconf->type = *(p + LCD_UKEY_EXT_TYPE);
	extconf->status = *(p + LCD_UKEY_EXT_STATUS);

	if (extconf->index == LCD_EXTERN_INDEX_INVALID) {
		if (lcd_debug_print_flag)
			EXTPR("index %d is invalid\n", extconf->index);
		kfree(para);
		return -1;
	}

	/* type: 10byte */
	switch (extconf->type) {
	case LCD_EXTERN_I2C:
		extconf->i2c_addr = *(p + LCD_UKEY_EXT_TYPE_VAL_0);
		extconf->i2c_addr2 = *(p + LCD_UKEY_EXT_TYPE_VAL_1);
		/*extconf->i2c_bus = *(p + LCD_UKEY_EXT_TYPE_VAL_2);*/
		if (extconf->i2c_bus == LCD_EXT_I2C_BUS_MAX)
			EXTERR("%s: get i2c_bus failed\n", extconf->name);

		extconf->cmd_size = *(p + LCD_UKEY_EXT_TYPE_VAL_3);
		if (lcd_debug_print_flag) {
			EXTPR("%s: cmd_size = %d\n",
				extconf->name, extconf->cmd_size);
		}

		/* init */
		if (extconf->cmd_size == 0)
			break;
		if (extconf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = lcd_extern_init_table_dynamic_size_load_unifykey(
				extconf, p, key_len, len, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_dynamic_size_load_unifykey(
				extconf, p, key_len, len, 0);
		} else {
			ret = lcd_extern_init_table_fixed_size_load_unifykey(
				extconf, p, key_len, len, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_fixed_size_load_unifykey(
				extconf, p, key_len, len, 0);
		}
		if (ret == 0)
			extconf->table_init_loaded = 1;
		break;
	case LCD_EXTERN_SPI:
		extconf->spi_gpio_cs = *(p + LCD_UKEY_EXT_TYPE_VAL_0);
		lcd_extern_gpio_probe(extconf->spi_gpio_cs);
		extconf->spi_gpio_clk = *(p + LCD_UKEY_EXT_TYPE_VAL_1);
		lcd_extern_gpio_probe(extconf->spi_gpio_clk);
		extconf->spi_gpio_data = *(p + LCD_UKEY_EXT_TYPE_VAL_2);
		lcd_extern_gpio_probe(extconf->spi_gpio_data);
		extconf->spi_clk_freq = (*(p + LCD_UKEY_EXT_TYPE_VAL_3) |
			((*(p + LCD_UKEY_EXT_TYPE_VAL_4)) << 8));
		extconf->spi_clk_pol = *(p + LCD_UKEY_EXT_TYPE_VAL_5);
		extconf->cmd_size = *(p + LCD_UKEY_EXT_TYPE_VAL_6);
		if (lcd_debug_print_flag) {
			EXTPR("%s: cmd_size = %d\n",
				extconf->name, extconf->cmd_size);
		}

		/* init */
		if (extconf->cmd_size == 0)
			break;
		if (extconf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = lcd_extern_init_table_dynamic_size_load_unifykey(
				extconf, p, key_len, len, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_dynamic_size_load_unifykey(
				extconf, p, key_len, len, 0);
		} else {
			ret = lcd_extern_init_table_fixed_size_load_unifykey(
				extconf, p, key_len, len, 1);
			if (ret)
				break;
			ret = lcd_extern_init_table_fixed_size_load_unifykey(
				extconf, p, key_len, len, 0);
		}
		if (ret == 0)
			extconf->table_init_loaded = 1;
		break;
	case LCD_EXTERN_MIPI:
		extconf->cmd_size = *(p + LCD_UKEY_EXT_TYPE_VAL_0);
		if (lcd_debug_print_flag) {
			EXTPR("%s: cmd_size = %d\n",
				extconf->name, extconf->cmd_size);
		}
		if (extconf->cmd_size != LCD_EXT_CMD_SIZE_DYNAMIC)
			break;
		ret = lcd_extern_init_table_dynamic_size_load_unifykey(
			extconf, p, key_len, len, 1);
		if (ret)
			break;
		ret = lcd_extern_init_table_dynamic_size_load_unifykey(
			extconf, p, key_len, len, 0);
		if (ret == 0)
			extconf->table_init_loaded = 1;
		break;
	default:
		break;
	}

	kfree(para);
	return 0;
}

static int lcd_extern_table_init_dft_malloc(void)
{
	table_init_on_dft = kcalloc(LCD_EXTERN_INIT_ON_MAX,
		sizeof(unsigned char), GFP_KERNEL);
	if (table_init_on_dft == NULL) {
		EXTERR("failed to alloc init_on_dft table\n");
		return -1;
	}
	table_init_off_dft = kcalloc(LCD_EXTERN_INIT_OFF_MAX,
		sizeof(unsigned char), GFP_KERNEL);
	if (table_init_off_dft == NULL) {
		EXTERR("failed to alloc init_off_dft table\n");
		kfree(table_init_on_dft);
		return -1;
	}
	table_init_on_dft[0] = LCD_EXT_CMD_TYPE_END;
	table_init_on_dft[1] = 0;
	table_init_off_dft[0] = LCD_EXT_CMD_TYPE_END;
	table_init_off_dft[1] = 0;

	return 0;
}

static int lcd_extern_table_init_save(struct lcd_extern_config_s *extconf)
{
	if (extconf->table_init_on_cnt > 0) {
		extconf->table_init_on = kcalloc(extconf->table_init_on_cnt,
			sizeof(unsigned char), GFP_KERNEL);
		if (extconf->table_init_on == NULL) {
			EXTERR("failed to alloc init_on table\n");
			return -1;
		}
		memcpy(extconf->table_init_on, table_init_on_dft,
			extconf->table_init_on_cnt*sizeof(unsigned char));
	}
	if (extconf->table_init_off_cnt > 0) {
		extconf->table_init_off = kcalloc(extconf->table_init_off_cnt,
			sizeof(unsigned char), GFP_KERNEL);
		if (extconf->table_init_off == NULL) {
			EXTERR("failed to alloc init_off table\n");
			kfree(extconf->table_init_on);
			return -1;
		}
		memcpy(extconf->table_init_off, table_init_off_dft,
			extconf->table_init_off_cnt*sizeof(unsigned char));
	}

	return 0;
}

static int lcd_extern_get_config(void)
{
	unsigned int extern_para[5];
	const char *str;
	unsigned int load_id = 0;
	int ret;

	if (lcd_extern_dev->of_node == NULL) {
		EXTERR("no lcd_extern of_node exist\n");
		return -1;
	}

	ret = lcd_extern_table_init_dft_malloc();
	if (ret)
		return -1;

	ret = of_property_read_string(lcd_extern_dev->of_node, "i2c_bus", &str);
	if (ret)
		lcd_extern_config.i2c_bus = LCD_EXT_I2C_BUS_MAX;
	else
		lcd_extern_config.i2c_bus = lcd_extern_get_i2c_bus_str(str);

	ret = of_property_read_u32_array(lcd_extern_dev->of_node,
			"i2c_gpio", &extern_para[0], 2);
	if (ret) {
		lcd_extern_config.i2c_sck_gpio = LCD_EXT_GPIO_INVALID;
		lcd_extern_config.i2c_sda_gpio = LCD_EXT_GPIO_INVALID;
	} else {
		lcd_extern_config.i2c_sck_gpio = (unsigned char)extern_para[0];
		lcd_extern_config.i2c_sda_gpio = (unsigned char)extern_para[1];
	}

	ret = of_property_read_string(lcd_extern_dev->of_node,
			"pinctrl-names", &str);
	if (ret)
		lcd_extern_config.pinmux_valid = 0;
	else
		lcd_extern_config.pinmux_valid = 1;

	ret = of_property_read_u32(lcd_extern_dev->of_node,
			"pinctrl_gpio_off", &extern_para[0]);
	if (ret) {
		lcd_extern_config.pinmux_gpio_off = 2;
	} else {
		lcd_extern_config.pinmux_gpio_off =
			(unsigned char)extern_para[0];
	}

	ret = of_property_read_u32(lcd_extern_dev->of_node,
			"key_valid", &extern_para[0]);
	if (ret) {
		if (lcd_debug_print_flag)
			EXTPR("failed to get key_valid\n");
		lcd_extern_config.key_valid = 0;
	} else {
		lcd_extern_config.key_valid = (unsigned char)extern_para[0];
	}
	EXTPR("key_valid: %d\n", lcd_extern_config.key_valid);

	if (lcd_extern_config.key_valid) {
		ret = lcd_unifykey_check("lcd_extern");
		if (ret < 0)
			load_id = 0;
		else
			load_id = 1;
	}

	if (load_id) {
		EXTPR("%s from unifykey\n", __func__);
		lcd_ext_config_load = 1;
		ret = lcd_extern_get_config_unifykey(&lcd_extern_config);
	} else {
#ifdef CONFIG_OF
		EXTPR("%s from dts\n", __func__);
		lcd_ext_config_load = 0;
		ret = lcd_extern_get_config_dts(lcd_extern_dev->of_node,
			&lcd_extern_config);
#endif
	}
	if (ret)
		goto lcd_extern_get_config_err;

	if (lcd_extern_config.table_init_loaded > 0) {
		ret = lcd_extern_table_init_save(&lcd_extern_config);
		if (ret)
			goto lcd_extern_get_config_err;
	}

	kfree(table_init_on_dft);
	kfree(table_init_off_dft);
	return 0;

lcd_extern_get_config_err:
	kfree(table_init_on_dft);
	kfree(table_init_off_dft);
	return -1;
}

static int lcd_extern_add_i2c(struct aml_lcd_extern_driver_s *ext_drv)
{
	int ret = -1;

	if (strcmp(ext_drv->config->name, "ext_default") == 0) {
		if (ext_drv->config->cmd_size < 2) {
			EXTERR("%s: cmd_size %d is invalid\n",
				ext_drv->config->name,
				ext_drv->config->cmd_size);
			ret = -1;
		} else {
			ret = aml_lcd_extern_default_probe(ext_drv);
		}
	} else if (strcmp(ext_drv->config->name, "i2c_T5800Q") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_I2C_T5800Q
		ret = aml_lcd_extern_i2c_T5800Q_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "i2c_tc101") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_I2C_TC101
		ret = aml_lcd_extern_i2c_tc101_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "i2c_anx6345") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_I2C_ANX6345
		ret = aml_lcd_extern_i2c_anx6345_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "i2c_DLPC3439") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_I2C_DLPC3439
		ret = aml_lcd_extern_i2c_DLPC3439_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "i2c_CS602") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_I2C_CS602
		ret = aml_lcd_extern_i2c_CS602_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "i2c_ANX6862_7911") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_I2C_ANX6862_7911
		ret = aml_lcd_extern_i2c_ANX6862_7911_probe(ext_drv);
#endif
	} else {
		EXTERR("invalid driver name: %s\n", ext_drv->config->name);
		ret = -1;
	}
	return ret;
}

static int lcd_extern_add_spi(struct aml_lcd_extern_driver_s *ext_drv)
{
	int ret = 0;

	if (strcmp(ext_drv->config->name, "ext_default") == 0) {
		if (ext_drv->config->cmd_size < 2) {
			EXTERR("%s: cmd_size %d is invalid\n",
				ext_drv->config->name,
				ext_drv->config->cmd_size);
			ret = -1;
		} else {
			ret = aml_lcd_extern_default_probe(ext_drv);
		}
	} else if (strcmp(ext_drv->config->name, "spi_LD070WS2") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_SPI_LD070WS2
		ret = aml_lcd_extern_spi_LD070WS2_probe(ext_drv);
#endif
	} else {
		EXTERR("invalid driver name: %s\n", ext_drv->config->name);
		ret = -1;
	}
	return ret;
}

static int lcd_extern_add_mipi(struct aml_lcd_extern_driver_s *ext_drv)
{
	int ret = 0;

	if ((strcmp(ext_drv->config->name, "mipi_default") == 0) ||
		(strcmp(ext_drv->config->name, "ext_default") == 0)) {
		if (ext_drv->config->cmd_size != LCD_EXT_CMD_SIZE_DYNAMIC) {
			EXTERR("%s: cmd_size %d is invalid\n",
				ext_drv->config->name,
				ext_drv->config->cmd_size);
			ret = -1;
		} else {
			ret = aml_lcd_extern_mipi_default_probe(ext_drv);
		}
	} else if (strcmp(ext_drv->config->name, "mipi_N070ICN") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_MIPI_N070ICN
		ret = aml_lcd_extern_mipi_N070ICN_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "mipi_KD080D13") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_MIPI_KD080D13
		ret = aml_lcd_extern_mipi_KD080D13_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "mipi_TV070WSM") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_MIPI_TV070WSM
		ret = aml_lcd_extern_mipi_TV070WSM_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "mipi_ST7701") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_MIPI_ST7701
		ret = aml_lcd_extern_mipi_st7701_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "mipi_P070ACB") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_MIPI_P070ACB
		ret = aml_lcd_extern_mipi_p070acb_probe(ext_drv);
#endif
	} else if (strcmp(ext_drv->config->name, "mipi_TL050FHV02CT") == 0) {
#ifdef CONFIG_AMLOGIC_LCD_EXTERN_MIPI_TL050FHV02CT
		ret = aml_lcd_extern_mipi_tl050fhv02ct_probe(ext_drv);
#endif
	} else {
		EXTERR("invalid driver name: %s\n", ext_drv->config->name);
		ret = -1;
	}
	return ret;
}

static int lcd_extern_add_invalid(struct aml_lcd_extern_driver_s *ext_drv)
{
	return -1;
}

static int lcd_extern_add_driver(struct lcd_extern_config_s *extconf)
{
	struct aml_lcd_extern_driver_s *ext_drv;
	int ret = 0;

	if (extconf->status == 0) {
		EXTERR("driver %s[%d] status is disabled\n",
			extconf->name, extconf->index);
		return -1;
	}

	lcd_ext_driver =
		kzalloc(sizeof(struct aml_lcd_extern_driver_s), GFP_KERNEL);
	if (lcd_ext_driver == NULL) {
		EXTERR("failed to alloc driver %s[%d], not enough memory\n",
			extconf->name, extconf->index);
		return -1;
	}
	ext_drv = lcd_ext_driver;

	ext_drv->config = extconf;
	ext_drv->pinmux_flag = 0xff;
	ext_drv->reg_read  = NULL;
	ext_drv->reg_write = NULL;
	ext_drv->power_on  = NULL;
	ext_drv->power_off = NULL;

	switch (ext_drv->config->type) {
	case LCD_EXTERN_I2C:
		ret = lcd_extern_add_i2c(ext_drv);
		break;
	case LCD_EXTERN_SPI:
		ret = lcd_extern_add_spi(ext_drv);
		break;
	case LCD_EXTERN_MIPI:
		ret = lcd_extern_add_mipi(ext_drv);
		break;
	default:
		ret = lcd_extern_add_invalid(ext_drv);
		EXTERR("don't support type %d\n", ext_drv->config->type);
		break;
	}
	if (ret) {
		EXTERR("add driver failed\n");
		kfree(lcd_ext_driver->config->table_init_on);
		kfree(lcd_ext_driver->config->table_init_off);
		lcd_ext_driver->config->table_init_on = NULL;
		lcd_ext_driver->config->table_init_off = NULL;
		kfree(lcd_ext_driver);
		lcd_ext_driver = NULL;
		return -1;
	}

	EXTPR("add driver %s(%d)\n",
		ext_drv->config->name, ext_drv->config->index);
	return 0;
}

/* *********************************************************
 * debug function
 * *********************************************************
 */
#define EXT_LEN_MAX   500
static void lcd_extern_init_table_dynamic_size_print(
		struct lcd_extern_config_s *econf, int flag)
{
	int i, j, k, max_len;
	unsigned char cmd_size;
	char *str;
	unsigned char *table;

	str = kcalloc(EXT_LEN_MAX, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		EXTERR("%s: str malloc error\n", __func__);
		return;
	}
	if (flag) {
		pr_info("power on:\n");
		table = econf->table_init_on;
		max_len = econf->table_init_on_cnt;
	} else {
		pr_info("power off:\n");
		table = econf->table_init_off;
		max_len = econf->table_init_off_cnt;
	}
	if (table == NULL) {
		EXTERR("init_table %d is NULL\n", flag);
		kfree(str);
		return;
	}

	i = 0;
	switch (econf->type) {
	case LCD_EXTERN_I2C:
	case LCD_EXTERN_SPI:
		while ((i + 1) < max_len) {
			if (table[i] == LCD_EXT_CMD_TYPE_END) {
				pr_info("  0x%02x,%d,\n",
					table[i], table[i+1]);
				break;
			}
			cmd_size = table[i+1];

			k = snprintf(str, EXT_LEN_MAX, "  0x%02x,%d,",
				table[i], cmd_size);
			if (cmd_size == 0)
				goto init_table_dynamic_print_i2c_spi_next;
			if (i + 2 + cmd_size > max_len) {
				pr_info("cmd_size out of support\n");
				break;
			}

			if ((table[i] == LCD_EXT_CMD_TYPE_GPIO) ||
				(table[i] == LCD_EXT_CMD_TYPE_DELAY)) {
				for (j = 0; j < cmd_size; j++) {
					k += snprintf(str+k, EXT_LEN_MAX,
						"%d,", table[i+2+j]);
				}
			} else if ((table[i] == LCD_EXT_CMD_TYPE_CMD) ||
				(table[i] == LCD_EXT_CMD_TYPE_CMD2)) {
				for (j = 0; j < cmd_size; j++) {
					k += snprintf(str+k, EXT_LEN_MAX,
						"0x%02x,", table[i+2+j]);
				}
			} else if ((table[i] == LCD_EXT_CMD_TYPE_CMD_DELAY) ||
				(table[i] == LCD_EXT_CMD_TYPE_CMD2_DELAY)) {
				for (j = 0; j < (cmd_size - 1); j++) {
					k += snprintf(str+k, EXT_LEN_MAX,
						"0x%02x,", table[i+2+j]);
				}
				snprintf(str+k, EXT_LEN_MAX,
					"%d,", table[i+cmd_size+1]);
			} else {
				for (j = 0; j < cmd_size; j++) {
					k += snprintf(str+k, EXT_LEN_MAX,
						"0x%02x,", table[i+2+j]);
				}
			}

init_table_dynamic_print_i2c_spi_next:
			pr_info("%s\n", str);
			i += (cmd_size + 2);
		}
		break;
	case LCD_EXTERN_MIPI:
		while ((i + 1) < max_len) {
			cmd_size = table[i+1];
			if (table[i] == LCD_EXT_CMD_TYPE_END) {
				if (cmd_size == 0xff) {
					pr_info("  0x%02x,0x%02x,\n",
						table[i], table[i+1]);
					break;
				}
				if (cmd_size == 0) {
					pr_info("  0x%02x,%d,\n",
						table[i], table[i+1]);
					break;
				}
				cmd_size = 0;
			}

			k = snprintf(str, EXT_LEN_MAX, "  0x%02x,%d,",
				table[i], cmd_size);
			if (cmd_size == 0)
				goto init_table_dynamic_print_mipi_next;
			if (i + 2 + cmd_size > max_len) {
				pr_info("cmd_size out of support\n");
				break;
			}

			if ((table[i] == LCD_EXT_CMD_TYPE_GPIO) ||
				(table[i] == LCD_EXT_CMD_TYPE_DELAY)) {
				for (j = 0; j < cmd_size; j++) {
					k += snprintf(str+k, EXT_LEN_MAX,
						"%d,", table[i+2+j]);
				}
			} else if ((table[i] & 0xf) == 0x0) {
				pr_info("  init_%s wrong data_type: 0x%02x\n",
					flag ? "on" : "off", table[i]);
				break;
			} else {
				cmd_size = table[i+DSI_CMD_SIZE_INDEX];
				k = snprintf(str, EXT_LEN_MAX, "  0x%02x,%d,",
					table[i], cmd_size);
				for (j = 0; j < cmd_size; j++) {
					k += snprintf(str+k, EXT_LEN_MAX,
						"0x%02x,",
						table[i+2+j]);
				}
			}

init_table_dynamic_print_mipi_next:
			pr_info("%s\n", str);
			i += (cmd_size + 2);
		}
		break;
	default:
		break;
	}
	kfree(str);
}

static void lcd_extern_init_table_fixed_size_print(
		struct lcd_extern_config_s *econf, int flag)
{
	int i, j, k, max_len;
	unsigned char cmd_size;
	char *str;
	unsigned char *table;

	str = kcalloc(EXT_LEN_MAX, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		EXTERR("%s: str malloc error\n", __func__);
		return;
	}
	cmd_size = econf->cmd_size;
	if (flag) {
		pr_info("power on:\n");
		table = econf->table_init_on;
		max_len = econf->table_init_on_cnt;
	} else {
		pr_info("power off:\n");
		table = econf->table_init_off;
		max_len = econf->table_init_off_cnt;
	}
	if (table == NULL) {
		EXTERR("init_table %d is NULL\n", flag);
		kfree(str);
		return;
	}

	i = 0;
	while ((i + cmd_size) <= max_len) {
		k = snprintf(str, EXT_LEN_MAX, " ");
		for (j = 0; j < cmd_size; j++) {
			k += snprintf(str+k, EXT_LEN_MAX, " 0x%02x",
				table[i+j]);
		}
		pr_info("%s\n", str);

		if (table[i] == LCD_EXT_CMD_TYPE_END)
			break;
		i += cmd_size;
	}
	kfree(str);
}

static ssize_t lcd_extern_info_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct lcd_extern_config_s *econf;

	if (lcd_ext_driver == NULL)
		return sprintf(buf, "lcd extern  driver is NULL\n");

	econf = lcd_ext_driver->config;
	pr_info("lcd extern driver %s(%d) info:\n", econf->name, econf->index);
	pr_info("status:             %d\n", econf->status);
	switch (econf->type) {
	case LCD_EXTERN_I2C:
		pr_info("type:               i2c(%d)\n", econf->type);
		pr_info("i2c_addr:           0x%02x\n"
			"i2c_addr2:          0x%02x\n"
			"i2c_bus:            %d\n"
			"table_loaded:       %d\n"
			"cmd_size:           %d\n"
			"table_init_on_cnt:  %d\n"
			"table_init_off_cnt: %d\n",
			econf->i2c_addr, econf->i2c_addr2, econf->i2c_bus,
			econf->table_init_loaded, econf->cmd_size,
			econf->table_init_on_cnt,
			econf->table_init_off_cnt);
		if (econf->cmd_size == 0)
			break;
		if (econf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			lcd_extern_init_table_dynamic_size_print(econf, 1);
			lcd_extern_init_table_dynamic_size_print(econf, 0);
		} else {
			lcd_extern_init_table_fixed_size_print(econf, 1);
			lcd_extern_init_table_fixed_size_print(econf, 0);
		}
		break;
	case LCD_EXTERN_SPI:
		pr_info("type:               spi(%d)\n", econf->type);
		pr_info("spi_gpio_cs:        %d\n"
			"spi_gpio_clk:       %d\n"
			"spi_gpio_data:      %d\n"
			"spi_clk_freq:       %dKHz\n"
			"spi_delay_us:       %d\n"
			"spi_clk_pol:        %d\n"
			"table_loaded:       %d\n"
			"cmd_size:           %d\n"
			"table_init_on_cnt:  %d\n"
			"table_init_off_cnt: %d\n",
			econf->spi_gpio_cs, econf->spi_gpio_clk,
			econf->spi_gpio_data, econf->spi_clk_freq,
			econf->spi_delay_us, econf->spi_clk_pol,
			econf->table_init_loaded, econf->cmd_size,
			econf->table_init_on_cnt,
			econf->table_init_off_cnt);
		if (econf->cmd_size == 0)
			break;
		if (econf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			lcd_extern_init_table_dynamic_size_print(econf, 1);
			lcd_extern_init_table_dynamic_size_print(econf, 0);
		} else {
			lcd_extern_init_table_fixed_size_print(econf, 1);
			lcd_extern_init_table_fixed_size_print(econf, 0);
		}
		break;
	case LCD_EXTERN_MIPI:
		pr_info("type:            mipi(%d)\n", econf->type);
		pr_info("table_loaded:       %d\n"
			"cmd_size:        %d\n"
			"table_init_on_cnt:  %d\n"
			"table_init_off_cnt: %d\n",
			econf->table_init_loaded,
			econf->cmd_size,
			econf->table_init_on_cnt,
			econf->table_init_off_cnt);
		if (econf->cmd_size != LCD_EXT_CMD_SIZE_DYNAMIC)
			break;
		lcd_extern_init_table_dynamic_size_print(econf, 1);
		lcd_extern_init_table_dynamic_size_print(econf, 0);
		break;
	default:
		pr_info("not support extern_type\n");
		break;
	}
	if (econf->pinmux_valid) {
		pr_info("pinmux_flag:     %d\n", lcd_ext_driver->pinmux_flag);
		pr_info("pinmux_pointer:  0x%p\n", lcd_ext_driver->pin);
	}

	return sprintf(buf, "\n");
}

static ssize_t lcd_extern_key_valid_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lcd_extern_config.key_valid);
}

static ssize_t lcd_extern_config_load_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lcd_ext_config_load);
}

static const char *lcd_extern_debug_usage_str = {
"Usage:\n"
"    echo test <on/off> > debug ; test power on/off for extern device\n"
"        <on/off>: 1 for power on, 0 for power off\n"
"    echo r <addr_sel> <reg> > debug ; read reg for extern device\n"
"    echo d <addr_sel> <reg> <cnt> > debug ; dump regs for extern device\n"
"    echo w <addr_sel> <reg> <value> > debug ; write reg for extern device\n"
};

static ssize_t lcd_extern_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lcd_extern_debug_usage_str);
}

static ssize_t lcd_extern_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret, i;
	unsigned int val[3];
	unsigned char reg, value;
	unsigned char reg_buf[2];

	if (lcd_ext_driver == NULL) {
		pr_info("lcd_extern_driver is null\n");
		return count;
	}

	switch (buf[0]) {
	case 't':
		ret = sscanf(buf, "test %d", &val[0]);
		if (ret == 1) {
			if (val[0]) {
				if (lcd_ext_driver->power_on)
					lcd_ext_driver->power_on();
			} else {
				if (lcd_ext_driver->power_off)
					lcd_ext_driver->power_off();
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'r':
		ret = sscanf(buf, "r %d %x", &val[0], &val[1]);
		if (ret == 2) {
			lcd_ext_driver->config->addr_sel =
				(unsigned char)val[0];
			reg = (unsigned char)val[1];
			if (lcd_ext_driver->reg_read) {
				lcd_ext_driver->reg_read(reg, &value);
				pr_info("reg read: 0x%02x = 0x%02x\n",
					reg, value);
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'd':
		ret = sscanf(buf, "d %d %x %d", &val[0], &val[1], &val[2]);
		if (ret == 3) {
			lcd_ext_driver->config->addr_sel =
				(unsigned char)val[0];
			reg = (unsigned char)val[1];
			if (lcd_ext_driver->reg_read) {
				pr_info("reg dump:\n");
				for (i = 0; i < val[2]; i++) {
					lcd_ext_driver->reg_read(reg+i, &value);
					pr_info("  0x%02x = 0x%02x\n",
						reg, value);
				}
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'w':
		ret = sscanf(buf, "w %d %x %x", &val[0], &val[1], &val[2]);
		if (ret == 2) {
			lcd_ext_driver->config->addr_sel =
				(unsigned char)val[0];
			reg = (unsigned char)val[1];
			value = (unsigned char)val[2];
			if (lcd_ext_driver->reg_write) {
				reg_buf[0] = (unsigned char)val[1];
				reg_buf[1] = (unsigned char)val[2];
				lcd_ext_driver->reg_write(reg_buf, 2);
				if (lcd_ext_driver->reg_read) {
					lcd_ext_driver->reg_read(reg, &value);
					pr_info(
				"reg write 0x%02x = 0x%02x, readback: 0x%02x\n",
					reg, val[2], value);
				} else {
					pr_info("reg write 0x%02x = 0x%02x\n",
						reg, value);
				}
			}
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	default:
		pr_info("invalid data\n");
		break;
	}

	return count;
}

static struct class_attribute lcd_extern_class_attrs[] = {
	__ATTR(info, 0644,
		lcd_extern_info_show, NULL),
	__ATTR(key_valid,   0444,
		lcd_extern_key_valid_show, NULL),
	__ATTR(config_load, 0444,
		lcd_extern_config_load_show, NULL),
	__ATTR(debug, 0644,
		lcd_extern_debug_show, lcd_extern_debug_store),
};

static struct class *debug_class;
static int creat_lcd_extern_class(void)
{
	int i;

	debug_class = class_create(THIS_MODULE, "lcd_ext");
	if (IS_ERR(debug_class)) {
		EXTERR("create debug class failed\n");
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(lcd_extern_class_attrs); i++) {
		if (class_create_file(debug_class,
			&lcd_extern_class_attrs[i])) {
			EXTERR("create debug attribute %s failed\n",
				lcd_extern_class_attrs[i].attr.name);
		}
	}

	return 0;
}

static int remove_lcd_extern_class(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_extern_class_attrs); i++)
		class_remove_file(debug_class, &lcd_extern_class_attrs[i]);

	class_destroy(debug_class);
	debug_class = NULL;

	return 0;
}
/* ********************************************************* */

static int aml_lcd_extern_probe(struct platform_device *pdev)
{
	int ret;

	lcd_extern_dev = &pdev->dev;

	ret = lcd_extern_get_config();
	if (ret == 0)
		lcd_extern_add_driver(&lcd_extern_config);


	creat_lcd_extern_class();

	EXTPR("%s ok\n", __func__);
	return 0;
}

static int aml_lcd_extern_remove(struct platform_device *pdev)
{
	remove_lcd_extern_class();
	kfree(lcd_ext_driver->config->table_init_on);
	kfree(lcd_ext_driver->config->table_init_off);
	lcd_ext_driver->config->table_init_on = NULL;
	lcd_ext_driver->config->table_init_off = NULL;
	kfree(lcd_ext_driver);
	lcd_ext_driver = NULL;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aml_lcd_extern_dt_match[] = {
	{
		.compatible = "amlogic, lcd_extern",
	},
	{},
};
#endif

static struct platform_driver aml_lcd_extern_driver = {
	.probe  = aml_lcd_extern_probe,
	.remove = aml_lcd_extern_remove,
	.driver = {
		.name  = "lcd_extern",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aml_lcd_extern_dt_match,
#endif
	},
};

static int __init aml_lcd_extern_init(void)
{
	int ret;

	if (lcd_debug_print_flag)
		EXTPR("%s\n", __func__);

	ret = platform_driver_register(&aml_lcd_extern_driver);
	if (ret) {
		EXTERR("driver register failed\n");
		return -ENODEV;
	}
	return ret;
}

static void __exit aml_lcd_extern_exit(void)
{
	platform_driver_unregister(&aml_lcd_extern_driver);
}

late_initcall(aml_lcd_extern_init);
module_exit(aml_lcd_extern_exit);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("LCD extern driver");
MODULE_LICENSE("GPL");

