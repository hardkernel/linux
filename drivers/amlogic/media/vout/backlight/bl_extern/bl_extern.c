/*
 * drivers/amlogic/media/vout/backlight/bl_extern/bl_extern.c
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/amlogic/media/vout/lcd/aml_bl_extern.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#include <linux/amlogic/media/vout/lcd/aml_bl.h>
#include "bl_extern.h"

static struct aml_bl_extern_driver_s bl_extern_driver;

static unsigned char *table_init_on_dft;
static unsigned char *table_init_off_dft;

static int bl_extern_set_level(unsigned int level)
{
	struct aml_bl_drv_s *bl_drv = aml_bl_get_driver();
	unsigned int level_max, level_min;
	unsigned int dim_max, dim_min;
	int ret = 0;

	if (bl_drv == NULL)
		return -1;

	bl_extern_driver.brightness = level;
	if (bl_extern_driver.status == 0)
		return 0;

	level_max = bl_drv->bconf->level_max;
	level_min = bl_drv->bconf->level_min;
	dim_max = bl_extern_driver.config.dim_max;
	dim_min = bl_extern_driver.config.dim_min;
	level = dim_min - ((level - level_min) * (dim_min - dim_max)) /
			(level_max - level_min);

	if (bl_extern_driver.device_bri_update)
		ret = bl_extern_driver.device_bri_update(level);

	return ret;
}

static int bl_extern_power_on(void)
{
	int ret = 0;

	BLEX("%s\n", __func__);

	if (bl_extern_driver.device_power_on)
		ret = bl_extern_driver.device_power_on();
	bl_extern_driver.status = 1;

	/* restore bl level */
	bl_extern_set_level(bl_extern_driver.brightness);

	return ret;
}

static int bl_extern_power_off(void)
{
	int ret = 0;

	BLEX("%s\n", __func__);

	bl_extern_driver.status = 0;
	if (bl_extern_driver.device_power_off)
		ret = bl_extern_driver.device_power_off();

	return ret;
}

static struct aml_bl_extern_driver_s bl_extern_driver = {
	.status = 0,
	.brightness = 0,
	.power_on = bl_extern_power_on,
	.power_off = bl_extern_power_off,
	.set_level = bl_extern_set_level,
	.config_print = NULL,
	.device_power_on = NULL,
	.device_power_off = NULL,
	.device_bri_update = NULL,
	.config = {
		.index = BL_EXTERN_INDEX_INVALID,
		.name = "none",
		.type = BL_EXTERN_MAX,
		.i2c_addr = 0xff,
		.i2c_bus = LCD_EXT_I2C_BUS_MAX,
		.dim_min = 10,
		.dim_max = 255,

		.init_loaded = 0,
		.cmd_size = 0,
		.init_on = NULL,
		.init_off = NULL,
		.init_on_cnt = 0,
		.init_off_cnt = 0,
	},
};

struct aml_bl_extern_driver_s *aml_bl_extern_get_driver(void)
{
	return &bl_extern_driver;
}

static unsigned char bl_extern_get_i2c_bus_str(const char *str)
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
	else if (strncmp(str, "i2c_bus_0", 10) == 0)
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
		i2c_bus = LCD_EXT_I2C_BUS_MAX;
		BLEXERR("invalid i2c_bus: %s\n", str);
	}

	return i2c_bus;
}

#define EXT_LEN_MAX   500
static void bl_extern_init_table_dynamic_size_print(
		struct bl_extern_config_s *econf, int flag)
{
	int i, j, k, max_len;
	unsigned char cmd_size;
	char *str;
	unsigned char *table;

	str = kcalloc(EXT_LEN_MAX, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		BLEXERR("%s: str malloc error\n", __func__);
		return;
	}
	if (flag) {
		pr_info("power on:\n");
		table = econf->init_on;
		max_len = econf->init_off_cnt;
	} else {
		pr_info("power off:\n");
		table = econf->init_off;
		max_len = econf->init_off_cnt;
	}
	if (table == NULL) {
		BLEXERR("init_table %d is NULL\n", flag);
		kfree(str);
		return;
	}

	i = 0;
	while ((i + 1) < max_len) {
		if (table[i] == LCD_EXT_CMD_TYPE_END) {
			pr_info("  0x%02x,%d,\n", table[i], table[i+1]);
			break;
		}
		cmd_size = table[i+1];

		k = snprintf(str, EXT_LEN_MAX, "  0x%02x,%d,",
			table[i], cmd_size);
		if (cmd_size == 0)
			goto init_table_dynamic_print_next;
		if (i + 2 + cmd_size > max_len) {
			pr_info("cmd_size out of support\n");
			break;
		}

		if (table[i] == LCD_EXT_CMD_TYPE_DELAY) {
			for (j = 0; j < cmd_size; j++) {
				k += snprintf(str+k, EXT_LEN_MAX,
					"%d,", table[i+2+j]);
			}
		} else if (table[i] == LCD_EXT_CMD_TYPE_CMD) {
			for (j = 0; j < cmd_size; j++) {
				k += snprintf(str+k, EXT_LEN_MAX,
					"0x%02x,", table[i+2+j]);
			}
		} else if (table[i] == LCD_EXT_CMD_TYPE_CMD_DELAY) {
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
init_table_dynamic_print_next:
		pr_info("%s\n", str);
		i += (cmd_size + 2);
	}

	kfree(str);
}

static void bl_extern_init_table_fixed_size_print(
		struct bl_extern_config_s *econf, int flag)
{
	int i, j, k, max_len;
	unsigned char cmd_size;
	char *str;
	unsigned char *table;

	str = kcalloc(EXT_LEN_MAX, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		BLEXERR("%s: str malloc error\n", __func__);
		return;
	}
	cmd_size = econf->cmd_size;
	if (flag) {
		pr_info("power on:\n");
		table = econf->init_on;
		max_len = econf->init_on_cnt;
	} else {
		pr_info("power off:\n");
		table = econf->init_off;
		max_len = econf->init_off_cnt;
	}
	if (table == NULL) {
		BLEXERR("init_table %d is NULL\n", flag);
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

static void bl_extern_config_print(void)
{
	struct aml_bl_extern_driver_s *bl_extern = aml_bl_extern_get_driver();
	struct aml_bl_extern_i2c_dev_s *i2c_dev = aml_bl_extern_i2c_get_dev();

	BLEX("%s:\n", __func__);
	pr_info("index:          %d\n"
		"name:          %s\n",
		bl_extern->config.index,
		bl_extern->config.name);
	switch (bl_extern->config.type) {
	case BL_EXTERN_I2C:
		pr_info("type:          i2c(%d)\n"
			"i2c_addr:      0x%02x\n"
			"i2c_bus:       %d\n"
			"dim_min:       %d\n"
			"dim_max:       %d\n",
			bl_extern->config.type,
			bl_extern->config.i2c_addr,
			bl_extern->config.i2c_bus,
			bl_extern->config.dim_min,
			bl_extern->config.dim_max);
		if (i2c_dev) {
			pr_info("i2c_dev_name:      %s\n"
				"i2c_client_name:   %s\n"
				"i2c_client_addr:   0x%02x\n",
				i2c_dev->name,
				i2c_dev->client->name,
				i2c_dev->client->addr);
		} else {
			pr_info("invalid i2c device\n");
		}
		if (bl_extern->config.cmd_size == 0)
			break;
		pr_info("table_loaded:       %d\n"
			"cmd_size:           %d\n"
			"init_on_cnt:        %d\n"
			"init_off_cnt:       %d\n",
			bl_extern->config.init_loaded,
			bl_extern->config.cmd_size,
			bl_extern->config.init_on_cnt,
			bl_extern->config.init_off_cnt);
		if (bl_extern->config.cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			bl_extern_init_table_dynamic_size_print(
				&bl_extern->config, 1);
			bl_extern_init_table_dynamic_size_print(
				&bl_extern->config, 0);
		} else {
			bl_extern_init_table_fixed_size_print(
				&bl_extern->config, 1);
			bl_extern_init_table_fixed_size_print(
				&bl_extern->config, 0);
		}
		break;
	case BL_EXTERN_SPI:
		pr_info("type:          spi(%d)\n"
			"dim_min:       %d\n"
			"dim_max:       %d\n",
			bl_extern->config.type,
			bl_extern->config.dim_min,
			bl_extern->config.dim_max);
		if (bl_extern->config.cmd_size == 0)
			break;
		pr_info("table_loaded:       %d\n"
			"cmd_size:           %d\n"
			"init_on_cnt:        %d\n"
			"init_off_cnt:       %d\n",
			bl_extern->config.init_loaded,
			bl_extern->config.cmd_size,
			bl_extern->config.init_on_cnt,
			bl_extern->config.init_off_cnt);
		if (bl_extern->config.cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			bl_extern_init_table_dynamic_size_print(
				&bl_extern->config, 1);
			bl_extern_init_table_dynamic_size_print(
				&bl_extern->config, 0);
		} else {
			bl_extern_init_table_fixed_size_print(
				&bl_extern->config, 1);
			bl_extern_init_table_fixed_size_print(
				&bl_extern->config, 0);
		}
		break;
	case BL_EXTERN_MIPI:
		pr_info("type:          mipi(%d)\n"
			"dim_min:       %d\n"
			"dim_max:       %d\n",
			bl_extern->config.type,
			bl_extern->config.dim_min,
			bl_extern->config.dim_max);
		break;
	default:
		break;
	}
}

static int bl_extern_init_table_dynamic_size_load_dts(
		struct device_node *of_node,
		struct bl_extern_config_s *extconf, int flag)
{
	unsigned char cmd_size, type;
	int i = 0, j, val, max_len, step = 0, ret = 0;
	unsigned char *table;
	char propname[20];

	if (flag) {
		table = table_init_on_dft;
		max_len = BL_EXTERN_INIT_ON_MAX;
		sprintf(propname, "init_on");
	} else {
		table = table_init_off_dft;
		max_len = BL_EXTERN_INIT_OFF_MAX;
		sprintf(propname, "init_off");
	}
	if (table == NULL) {
		BLEXERR("%s: init_table is null\n", __func__);
		return -1;
	}

	while ((i + 1) < max_len) {
		/* type */
		ret = of_property_read_u32_index(of_node, propname, i, &val);
		if (ret) {
			BLEXERR("%s: get %s type failed, step %d\n",
				extconf->name, propname, step);
			table[i] = LCD_EXT_CMD_TYPE_END;
			table[i+1] = 0;
			return -1;
		}
		table[i] = (unsigned char)val;
		type = table[i];
		/* cmd_size */
		ret = of_property_read_u32_index(of_node, propname,
			(i+1), &val);
		if (ret) {
			BLEXERR("%s: get %s cmd_size failed, step %d\n",
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
			goto init_table_dynamic_dts_next;
		if ((i + 2 + cmd_size) > max_len) {
			BLEXERR("%s: %s cmd_size out of support, step %d\n",
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
				BLEXERR("%s: get %s data failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				table[i+1] = 0;
				return -1;
			}
			table[i+2+j] = (unsigned char)val;
		}

init_table_dynamic_dts_next:
		i += (cmd_size + 2);
		step++;
	}
	if (flag)
		extconf->init_on_cnt = i + 2;
	else
		extconf->init_off_cnt = i + 2;

	return 0;
}

static int bl_extern_init_table_fixed_size_load_dts(
		struct device_node *of_node,
		struct bl_extern_config_s *extconf, int flag)
{
	unsigned char cmd_size;
	int i = 0, j, val, max_len, step = 0, ret = 0;
	unsigned char *table;
	char propname[20];

	cmd_size = extconf->cmd_size;
	if (flag) {
		table = table_init_on_dft;
		max_len = BL_EXTERN_INIT_ON_MAX;
		sprintf(propname, "init_on");
	} else {
		table = table_init_off_dft;
		max_len = BL_EXTERN_INIT_OFF_MAX;
		sprintf(propname, "init_off");
	}
	if (table == NULL) {
		BLEXERR("%s: init_table is null\n", __func__);
		return -1;
	}

	while (i < max_len) { /* group detect */
		if ((i + cmd_size) > max_len) {
			BLEXERR("%s: %s cmd_size out of support, step %d\n",
				extconf->name, propname, step);
			table[i] = LCD_EXT_CMD_TYPE_END;
			return -1;
		}
		for (j = 0; j < cmd_size; j++) {
			ret = of_property_read_u32_index(
				of_node, propname, (i+j), &val);
			if (ret) {
				BLEXERR("%s: get %s failed, step %d\n",
					extconf->name, propname, step);
				table[i] = LCD_EXT_CMD_TYPE_END;
				return -1;
			}
			table[i+j] = (unsigned char)val;
		}
		if (table[i] == LCD_EXT_CMD_TYPE_END)
			break;

		i += cmd_size;
		step++;
	}

	if (flag)
		extconf->init_on_cnt = i + cmd_size;
	else
		extconf->init_off_cnt = i + cmd_size;

	return 0;
}

static int bl_extern_tablet_init_dft_malloc(void)
{
	table_init_on_dft = kcalloc(BL_EXTERN_INIT_ON_MAX,
		sizeof(unsigned char), GFP_KERNEL);
	if (table_init_on_dft == NULL) {
		BLEXERR("failed to alloc init_on table\n");
		return -1;
	}
	table_init_off_dft = kcalloc(BL_EXTERN_INIT_OFF_MAX,
		sizeof(unsigned char), GFP_KERNEL);
	if (table_init_off_dft == NULL) {
		BLEXERR("failed to alloc init_off table\n");
		kfree(table_init_on_dft);
		return -1;
	}
	table_init_on_dft[0] = LCD_EXT_CMD_TYPE_END;
	table_init_on_dft[1] = 0;
	table_init_off_dft[0] = LCD_EXT_CMD_TYPE_END;
	table_init_off_dft[1] = 0;

	return 0;
}

static int bl_extern_table_init_save(struct bl_extern_config_s *extconf)
{
	if (extconf->init_on_cnt > 0) {
		extconf->init_on = kcalloc(extconf->init_on_cnt,
			sizeof(unsigned char), GFP_KERNEL);
		if (extconf->init_on == NULL) {
			BLEXERR("failed to alloc init_on table\n");
			return -1;
		}
		memcpy(extconf->init_on, table_init_on_dft,
			extconf->init_off_cnt*sizeof(unsigned char));
	}
	if (extconf->init_off_cnt > 0) {
		extconf->init_off = kcalloc(extconf->init_off_cnt,
			sizeof(unsigned char), GFP_KERNEL);
		if (extconf->init_off == NULL) {
			BLEXERR("failed to alloc init_off table\n");
			kfree(extconf->init_on);
			return -1;
		}
		memcpy(extconf->init_off, table_init_off_dft,
			extconf->init_off_cnt*sizeof(unsigned char));
	}

	return 0;
}

static int bl_extern_config_from_dts(struct device_node *np, int index)
{
	char propname[20];
	struct device_node *child;
	const char *str;
	unsigned int temp[5], val;
	int ret = 0;
	struct aml_bl_extern_driver_s *bl_extern = aml_bl_extern_get_driver();

	ret = of_property_read_string(np, "i2c_bus", &str);
	if (ret == 0)
		bl_extern->config.i2c_bus = LCD_EXT_I2C_BUS_MAX;
	else
		bl_extern->config.i2c_bus = bl_extern_get_i2c_bus_str(str);

	/* get device config */
	sprintf(propname, "extern_%d", index);
	child = of_get_child_by_name(np, propname);
	if (child == NULL) {
		BLEXERR("failed to get %s\n", propname);
		return -1;
	}
	BLEX("load: %s\n", propname);

	ret = of_property_read_u32_array(child, "index", &temp[0], 1);
	if (ret) {
		BLEXERR("failed to get index, exit\n");
	} else {
		if (temp[0] == index)
			bl_extern->config.index = temp[0];
		else {
			BLEXERR("index %d not match, exit\n", index);
			return -1;
		}
	}
	ret = of_property_read_string(child, "extern_name", &str);
	if (ret) {
		BLEXERR("failed to get bl_extern_name\n");
		strcpy(bl_extern->config.name, "none");
	} else {
		strcpy(bl_extern->config.name, str);
	}

	ret = of_property_read_u32(child, "type", &val);
	if (ret) {
		BLEXERR("failed to get type\n");
	} else {
		bl_extern->config.type = val;
		BLEX("type: %d\n", bl_extern->config.type);
	}
	if (bl_extern->config.type >= BL_EXTERN_MAX) {
		BLEXERR("invalid type %d\n", bl_extern->config.type);
		return -1;
	}

	ret = of_property_read_u32_array(child, "dim_max_min", &temp[0], 2);
	if (ret) {
		BLEXERR("failed to get dim_max_min\n");
		bl_extern->config.dim_max = 255;
		bl_extern->config.dim_min = 10;
	} else {
		bl_extern->config.dim_max = temp[0];
		bl_extern->config.dim_min = temp[1];
	}

	ret = bl_extern_tablet_init_dft_malloc();
	if (ret)
		return -1;
	switch (bl_extern->config.type) {
	case BL_EXTERN_I2C:
		if (bl_extern->config.i2c_bus >= LCD_EXT_I2C_BUS_MAX) {
			BLEXERR("failed to get i2c_bus\n");
		} else {
			BLEX("%s: i2c_bus=%s[%d]\n",
				bl_extern->config.name,
				str, bl_extern->config.i2c_bus);
		}

		ret = of_property_read_u32(child, "i2c_address", &val);
		if (ret) {
			BLEXERR("failed to get i2c_address\n");
		} else {
			bl_extern->config.i2c_addr = (unsigned char)val;
			BLEX("%s: i2c_address=0x%02x\n",
				bl_extern->config.name,
				bl_extern->config.i2c_addr);
		}

		ret = of_property_read_u32(child, "cmd_size", &val);
		if (ret) {
			BLEX("%s: no cmd_size\n", bl_extern->config.name);
			bl_extern->config.cmd_size = 0;
		} else {
			bl_extern->config.cmd_size = (unsigned char)val;
		}
		if (bl_debug_print_flag) {
			BLEX("%s: cmd_size = %d\n",
				bl_extern->config.name,
				bl_extern->config.cmd_size);
		}
		if (bl_extern->config.cmd_size == 0)
			break;

		if (bl_extern->config.cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = bl_extern_init_table_dynamic_size_load_dts(
				child, &bl_extern->config, 1);
			if (ret)
				break;
			ret = bl_extern_init_table_dynamic_size_load_dts(
				child, &bl_extern->config, 0);
		} else {
			ret = bl_extern_init_table_fixed_size_load_dts(
				child, &bl_extern->config, 1);
			if (ret)
				break;
			ret = bl_extern_init_table_fixed_size_load_dts(
				child, &bl_extern->config, 0);
		}
		if (ret == 0)
			bl_extern->config.init_loaded = 1;
		break;
	case BL_EXTERN_SPI:
		ret = of_property_read_u32(child, "cmd_size", &val);
		if (ret) {
			BLEX("%s: no cmd_size\n", bl_extern->config.name);
			bl_extern->config.cmd_size = 0;
		} else {
			bl_extern->config.cmd_size = (unsigned char)val;
		}
		if (bl_debug_print_flag) {
			BLEX("%s: cmd_size = %d\n",
				bl_extern->config.name,
				bl_extern->config.cmd_size);
		}
		if (bl_extern->config.cmd_size == 0)
			break;

		if (bl_extern->config.cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = bl_extern_init_table_dynamic_size_load_dts(
				child, &bl_extern->config, 1);
			if (ret)
				break;
			ret = bl_extern_init_table_dynamic_size_load_dts(
				child, &bl_extern->config, 0);
		} else {
			ret = bl_extern_init_table_fixed_size_load_dts(
				child, &bl_extern->config, 1);
			if (ret)
				break;
			ret = bl_extern_init_table_fixed_size_load_dts(
				child, &bl_extern->config, 0);
		}
		if (ret == 0)
			bl_extern->config.init_loaded = 1;
		break;
	case BL_EXTERN_MIPI:
		break;
	default:
		break;
	}

	if (bl_extern->config.init_loaded > 0) {
		ret = bl_extern_table_init_save(&bl_extern->config);
		if (ret)
			goto bl_extern_get_config_err;
	}

	kfree(table_init_on_dft);
	kfree(table_init_off_dft);
	return 0;

bl_extern_get_config_err:
	kfree(table_init_on_dft);
	kfree(table_init_off_dft);
	return -1;
}

static int bl_extern_add_driver(void)
{
	int ret = -1;
	struct bl_extern_config_s *extconf = &bl_extern_driver.config;

	if (strcmp(extconf->name, "i2c_lp8556") == 0) {
#ifdef CONFIG_AMLOGIC_BL_EXTERN_I2C_LP8556
		ret = i2c_lp8556_probe();
#endif
	} else if (strcmp(extconf->name, "mipi_lt070me05") == 0) {
#ifdef CONFIG_AMLOGIC_BL_EXTERN_MIPI_LT070ME05
		ret = mipi_lt070me05_probe();
#endif
	} else {
		BLEXERR("invalid device name: %s\n", extconf->name);
	}

	if (ret) {
		BLEXERR("add device driver failed %s(%d)\n",
			extconf->name, extconf->index);
	} else {
		BLEX("add device driver %s(%d) ok\n",
			extconf->name, extconf->index);
	}

	return ret;
}

static int bl_extern_remove_driver(void)
{
	int ret = -1;
	struct bl_extern_config_s *extconf = &bl_extern_driver.config;

	if (strcmp(extconf->name, "i2c_lp8556") == 0) {
#ifdef CONFIG_AMLOGIC_BL_EXTERN_I2C_LP8556
		ret = i2c_lp8556_remove();
#endif
	} else if (strcmp(extconf->name, "mipi_lt070me05") == 0) {
#ifdef CONFIG_AMLOGIC_BL_EXTERN_MIPI_LT070ME05
		ret = mipi_lt070me05_remove();
#endif
	} else {
		BLEXERR("invalid device name: %s\n", extconf->name);
	}

	if (ret) {
		BLEXERR("remove device driver failed %s(%d)\n",
			extconf->name, extconf->index);
	} else {
		BLEX("remove device driver %s(%d) ok\n",
			extconf->name, extconf->index);
	}

	return ret;
}

int aml_bl_extern_device_load(int index)
{
	int ret = 0;

	bl_extern_config_from_dts(bl_extern_driver.dev->of_node, index);
	ret = bl_extern_add_driver();
	bl_extern_driver.config_print = bl_extern_config_print;

	BLEX("%s OK\n", __func__);

	return ret;
}

int aml_bl_extern_probe(struct platform_device *pdev)
{
	int ret = 0;

	bl_extern_driver.dev = &pdev->dev;
	BLEX("%s OK\n", __func__);

	return ret;
}

static int aml_bl_extern_remove(struct platform_device *pdev)
{
	int ret = 0;

	bl_extern_remove_driver();

	BLEX("%s OK\n", __func__);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id aml_bl_extern_dt_match[] = {
	{
		.compatible = "amlogic, bl_extern",
	},
	{},
};
#endif

static struct platform_driver aml_bl_extern_driver = {
	.probe  = aml_bl_extern_probe,
	.remove = aml_bl_extern_remove,
	.driver = {
		.name  = "bl_extern",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aml_bl_extern_dt_match,
#endif
	},
};

static int __init aml_bl_extern_init(void)
{
	int ret;

	ret = platform_driver_register(&aml_bl_extern_driver);
	if (ret) {
		BLEXERR("driver register failed\n");
		return -ENODEV;
	}
	return ret;
}

static void __exit aml_bl_extern_exit(void)
{
	platform_driver_unregister(&aml_bl_extern_driver);
}

module_init(aml_bl_extern_init);
module_exit(aml_bl_extern_exit);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("bl extern driver");
MODULE_LICENSE("GPL");

