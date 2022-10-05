/*
 * drivers/amlogic/media/vout/backlight/aml_ldim/ldim_dev_drv.c
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
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/media/vout/lcd/aml_ldim.h>
#include <linux/amlogic/media/vout/lcd/aml_bl.h>
#include "ldim_drv.h"
#include "ldim_dev_drv.h"
#include "../aml_bl_reg.h"

struct bl_gpio_s ldim_gpio[BL_GPIO_NUM_MAX] = {
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
	{.probe_flag = 0, .register_flag = 0,},
};

static struct spi_board_info ldim_spi_info = {
	.modalias = "ldim_dev",
	.mode = SPI_MODE_0,
	.max_speed_hz = 1000000, /* 1MHz */
	.bus_num = 0, /* SPI bus No. */
	.chip_select = 0, /* the device index on the spi bus */
	.controller_data = NULL,
};

static unsigned char *table_init_on_dft;
static unsigned char *table_init_off_dft;
static int ldim_dev_probe_flag;
static struct class ldim_dev_class;

struct ldim_dev_config_s ldim_dev_config = {
	.type = LDIM_DEV_TYPE_NORMAL,
	.cs_hold_delay = 0,
	.cs_clk_delay = 0,
	.en_gpio = LCD_EXT_GPIO_INVALID,
	.en_gpio_on = 1,
	.en_gpio_off = 0,
	.lamp_err_gpio = LCD_EXT_GPIO_INVALID,
	.fault_check = 0,
	.write_check = 0,
	.dim_min = 0x7f, /* min 3% duty */
	.dim_max = 0xfff,
	.init_loaded = 0,
	.cmd_size = 4,
	.init_on = NULL,
	.init_off = NULL,
	.init_on_cnt = 0,
	.init_off_cnt = 0,
	.ldim_pwm_config = {
		.pwm_method = BL_PWM_POSITIVE,
		.pwm_port = BL_PWM_MAX,
		.pwm_duty_max = 100,
		.pwm_duty_min = 0,
	},
	.analog_pwm_config = {
		.pwm_method = BL_PWM_POSITIVE,
		.pwm_port = BL_PWM_MAX,
		.pwm_freq = 1000,
		.pwm_duty_max = 100,
		.pwm_duty_min = 10,
	},

	.bl_regnum = 0,

	.dim_range_update = NULL,
	.dev_reg_write = NULL,
	.dev_reg_read = NULL,
};

static void ldim_gpio_probe(int index)
{
	struct bl_gpio_s *ld_gpio;
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();
	const char *str;
	int ret;

	if (index >= BL_GPIO_NUM_MAX) {
		LDIMERR("gpio index %d, exit\n", index);
		return;
	}
	ld_gpio = &ldim_gpio[index];
	if (ld_gpio->probe_flag) {
		if (ldim_debug_print) {
			LDIMPR("gpio %s[%d] is already registered\n",
				ld_gpio->name, index);
		}
		return;
	}

	/* get gpio name */
	ret = of_property_read_string_index(ldim_drv->dev->of_node,
		"ldim_dev_gpio_names", index, &str);
	if (ret) {
		LDIMERR("failed to get ldim_dev_gpio_names: %d\n", index);
		str = "unknown";
	}
	strcpy(ld_gpio->name, str);

	/* init gpio flag */
	ld_gpio->probe_flag = 1;
	ld_gpio->register_flag = 0;
}

static int ldim_gpio_register(int index, int init_value)
{
	struct bl_gpio_s *ld_gpio;
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();
	int value;

	if (index >= BL_GPIO_NUM_MAX) {
		LDIMERR("%s: gpio index %d, exit\n", __func__, index);
		return -1;
	}
	ld_gpio = &ldim_gpio[index];
	if (ld_gpio->probe_flag == 0) {
		LDIMERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return -1;
	}
	if (ld_gpio->register_flag) {
		if (ldim_debug_print) {
			LDIMPR("%s: gpio %s[%d] is already registered\n",
				__func__, ld_gpio->name, index);
		}
		return 0;
	}

	switch (init_value) {
	case BL_GPIO_OUTPUT_LOW:
		value = GPIOD_OUT_LOW;
		break;
	case BL_GPIO_OUTPUT_HIGH:
		value = GPIOD_OUT_HIGH;
		break;
	case BL_GPIO_INPUT:
	default:
		value = GPIOD_IN;
		break;
	}

	/* request gpio */
	ld_gpio->gpio = devm_gpiod_get_index(ldim_drv->dev,
		"ldim_dev", index, value);
	if (IS_ERR(ld_gpio->gpio)) {
		LDIMERR("register gpio %s[%d]: %p, err: %d\n",
			ld_gpio->name, index, ld_gpio->gpio,
			IS_ERR(ld_gpio->gpio));
		return -1;
	}
	ld_gpio->register_flag = 1;
	if (ldim_debug_print) {
		LDIMPR("register gpio %s[%d]: %p, init value: %d\n",
			ld_gpio->name, index, ld_gpio->gpio, init_value);
	}

	return 0;
}

void ldim_gpio_set(int index, int value)
{
	struct bl_gpio_s *ld_gpio;

	if (index >= BL_GPIO_NUM_MAX) {
		LDIMERR("gpio index %d, exit\n", index);
		return;
	}
	ld_gpio = &ldim_gpio[index];
	if (ld_gpio->probe_flag == 0) {
		BLERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return;
	}
	if (ld_gpio->register_flag == 0) {
		ldim_gpio_register(index, value);
		return;
	}
	if (IS_ERR_OR_NULL(ld_gpio->gpio)) {
		LDIMERR("gpio %s[%d]: %p, err: %ld\n",
			ld_gpio->name, index, ld_gpio->gpio,
			PTR_ERR(ld_gpio->gpio));
		return;
	}

	switch (value) {
	case BL_GPIO_OUTPUT_LOW:
	case BL_GPIO_OUTPUT_HIGH:
		gpiod_direction_output(ld_gpio->gpio, value);
		break;
	case BL_GPIO_INPUT:
	default:
		gpiod_direction_input(ld_gpio->gpio);
		break;
	}
	if (ldim_debug_print) {
		LDIMPR("set gpio %s[%d] value: %d\n",
			ld_gpio->name, index, value);
	}
}

unsigned int ldim_gpio_get(int index)
{
	struct bl_gpio_s *ld_gpio;

	if (index >= BL_GPIO_NUM_MAX) {
		LDIMERR("gpio index %d, exit\n", index);
		return -1;
	}

	ld_gpio = &ldim_gpio[index];
	if (ld_gpio->probe_flag == 0) {
		LDIMERR("%s: gpio [%d] is not probed, exit\n", __func__, index);
		return -1;
	}
	if (ld_gpio->register_flag == 0) {
		LDIMERR("%s: gpio %s[%d] is not registered\n",
			__func__, ld_gpio->name, index);
		return -1;
	}
	if (IS_ERR_OR_NULL(ld_gpio->gpio)) {
		LDIMERR("gpio %s[%d]: %p, err: %ld\n",
			ld_gpio->name, index,
			ld_gpio->gpio, PTR_ERR(ld_gpio->gpio));
		return -1;
	}

	return gpiod_get_value(ld_gpio->gpio);
}

void ldim_set_duty_pwm(struct bl_pwm_config_s *bl_pwm)
{
	unsigned long long temp;

	if (bl_pwm->pwm_port >= BL_PWM_MAX)
		return;

	temp = bl_pwm->pwm_cnt;
	bl_pwm->pwm_level = bl_do_div(((temp * bl_pwm->pwm_duty) + 50), 100);

	if (ldim_debug_print == 2) {
		LDIMPR(
	"pwm port %d: duty=%d%%, pwm_max=%d, pwm_min=%d, pwm_level=%d\n",
			bl_pwm->pwm_port, bl_pwm->pwm_duty,
			bl_pwm->pwm_max, bl_pwm->pwm_min, bl_pwm->pwm_level);
	}

	bl_pwm_ctrl(bl_pwm, 1);
}

void ldim_pwm_off(struct bl_pwm_config_s *bl_pwm)
{
	if (bl_pwm->pwm_port >= BL_PWM_MAX)
		return;

	bl_pwm_ctrl(bl_pwm, 0);
}

/* ****************************************************** */
static char *ldim_pinmux_str[] = {
	"ldim_pwm",               /* 0 */
	"ldim_pwm_vs",            /* 1 */
	"ldim_pwm_combo",         /* 2 */
	"ldim_pwm_vs_combo",      /* 3 */
	"ldim_pwm_off",           /* 4 */
	"ldim_pwm_combo_off",     /* 5 */
	"custome",
};

static int ldim_pwm_pinmux_ctrl(int status)
{
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();
	struct bl_pwm_config_s *bl_pwm;
	char *str;
	int ret = 0, index = 0xff;

	if (ldim_drv->ldev_conf->ldim_pwm_config.pwm_port >= BL_PWM_MAX)
		return 0;

	if (status) {
		bl_pwm = &ldim_drv->ldev_conf->ldim_pwm_config;
		if (bl_pwm->pwm_port == BL_PWM_VS)
			index = 1;
		else
			index = 0;
		bl_pwm = &ldim_drv->ldev_conf->analog_pwm_config;
		if (bl_pwm->pwm_port < BL_PWM_VS)
			index += 2;
	} else {
		bl_pwm = &ldim_drv->ldev_conf->analog_pwm_config;
		if (bl_pwm->pwm_port < BL_PWM_VS)
			index = 5;
		else
			index = 4;
	}

	str = ldim_pinmux_str[index];
	if (ldim_drv->pinmux_flag == index) {
		LDIMPR("pinmux %s is already selected\n", str);
		return 0;
	}

	/* request pwm pinmux */
	ldim_drv->pin = devm_pinctrl_get_select(ldim_drv->dev, str);
	if (IS_ERR(ldim_drv->pin)) {
		LDIMERR("set pinmux %s error\n", str);
		ret = -1;
	} else {
		LDIMPR("set pinmux %s: 0x%p\n", str, ldim_drv->pin);
	}
	ldim_drv->pinmux_flag = index;

	return ret;
}

static int ldim_pwm_vs_update(void)
{
	struct bl_pwm_config_s *bl_pwm = &ldim_dev_config.ldim_pwm_config;
	unsigned int cnt;
	int ret = 0;

	if (bl_pwm->pwm_port != BL_PWM_VS)
		return 0;

	if (ldim_debug_print)
		LDIMPR("%s\n", __func__);

	cnt = bl_vcbus_read(ENCL_VIDEO_MAX_LNCNT) + 1;
	if (cnt != bl_pwm->pwm_cnt) {
		bl_pwm_config_init(bl_pwm);
		ldim_set_duty_pwm(bl_pwm);
	}

	return ret;
}

#define EXT_LEN_MAX   500
static void ldim_dev_init_table_dynamic_size_print(
		struct ldim_dev_config_s *econf, int flag)
{
	int i, j, k, max_len;
	unsigned char cmd_size;
	char *str;
	unsigned char *table;

	str = kcalloc(EXT_LEN_MAX, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		LDIMERR("%s: str malloc error\n", __func__);
		return;
	}
	if (flag) {
		pr_info("power on:\n");
		table = econf->init_on;
		max_len = econf->init_on_cnt;
	} else {
		pr_info("power off:\n");
		table = econf->init_off;
		max_len = econf->init_off_cnt;
	}
	if (max_len == 0) {
		kfree(str);
		return;
	}
	if (table == NULL) {
		LDIMERR("init_table %d is NULL\n", flag);
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

static void ldim_dev_init_table_fixed_size_print(
		struct ldim_dev_config_s *econf, int flag)
{
	int i, j, k, max_len;
	unsigned char cmd_size;
	char *str;
	unsigned char *table;

	str = kcalloc(EXT_LEN_MAX, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		LDIMERR("%s: str malloc error\n", __func__);
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
	if (max_len == 0) {
		kfree(str);
		return;
	}
	if (table == NULL) {
		LDIMERR("init_table %d is NULL\n", flag);
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

static void ldim_dev_config_print(void)
{
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();
	struct aml_bl_drv_s *bl_drv = aml_bl_get_driver();
	struct bl_pwm_config_s *bl_pwm;
	struct pwm_state pstate;
	unsigned int value;
	int i, n, len = 0;
	char *str = NULL;

	LDIMPR("%s:\n", __func__);

	pr_info("valid_flag            = %d\n"
		"dev_index             = %d\n"
		"vsync_change_flag     = %d\n\n",
		ldim_drv->valid_flag,
		ldim_drv->dev_index,
		ldim_drv->vsync_change_flag);
	if (ldim_drv->ldev_conf == NULL) {
		LDIMERR("%s: device config is null\n", __func__);
		return;
	}

	pr_info("dev_name              = %s\n"
		"type                  = %d\n"
		"en_gpio               = %d\n"
		"en_gpio_on            = %d\n"
		"en_gpio_off           = %d\n"
		"dim_min               = 0x%03x\n"
		"dim_max               = 0x%03x\n"
		"region_num            = %d\n",
		ldim_drv->ldev_conf->name,
		ldim_drv->ldev_conf->type,
		ldim_drv->ldev_conf->en_gpio,
		ldim_drv->ldev_conf->en_gpio_on,
		ldim_drv->ldev_conf->en_gpio_off,
		ldim_drv->ldev_conf->dim_min,
		ldim_drv->ldev_conf->dim_max,
		ldim_drv->ldev_conf->bl_regnum);
	n = ldim_drv->ldev_conf->bl_regnum;
	len = (n * 4) + 50;
	str = kcalloc(len, sizeof(char), GFP_KERNEL);
	if (str == NULL) {
		pr_info("%s: buf malloc error\n", __func__);
	} else {
		len = sprintf(str, "region_mapping:\n  ");
		for (i = 0; i < n; i++) {
			len += sprintf(str+len, "%d,",
				ldim_drv->ldev_conf->bl_mapping[i]);
		}
		pr_info("%s\n\n", str);
		kfree(str);
	}

	switch (ldim_drv->ldev_conf->type) {
	case LDIM_DEV_TYPE_SPI:
		pr_info("spi_pointer           = 0x%p\n"
			"spi_modalias          = %s\n"
			"spi_mode              = %d\n"
			"spi_max_speed_hz      = %d\n"
			"spi_bus_num           = %d\n"
			"spi_chip_select       = %d\n"
			"cs_hold_delay         = %d\n"
			"cs_clk_delay          = %d\n"
			"lamp_err_gpio         = %d\n"
			"fault_check           = %d\n"
			"write_check           = %d\n\n",
			ldim_drv->spi_dev,
			ldim_drv->spi_info->modalias,
			ldim_drv->spi_info->mode,
			ldim_drv->spi_info->max_speed_hz,
			ldim_drv->spi_info->bus_num,
			ldim_drv->spi_info->chip_select,
			ldim_drv->ldev_conf->cs_hold_delay,
			ldim_drv->ldev_conf->cs_clk_delay,
			ldim_drv->ldev_conf->lamp_err_gpio,
			ldim_drv->ldev_conf->fault_check,
			ldim_drv->ldev_conf->write_check);
		break;
	case LDIM_DEV_TYPE_I2C:
		break;
	case LDIM_DEV_TYPE_NORMAL:
	default:
		break;
	}
	bl_pwm = &ldim_drv->ldev_conf->ldim_pwm_config;
	if (bl_pwm->pwm_port < BL_PWM_MAX) {
		pr_info("lidm_pwm_port:       %d\n"
			"lidm_pwm_pol:        %d\n"
			"lidm_pwm_freq:       %d\n"
			"lidm_pwm_cnt:        %d\n"
			"lidm_pwm_level:      %d\n"
			"lidm_pwm_duty:       %d%%\n",
			bl_pwm->pwm_port, bl_pwm->pwm_method,
			bl_pwm->pwm_freq, bl_pwm->pwm_cnt,
			bl_pwm->pwm_level, bl_pwm->pwm_duty);
		switch (bl_pwm->pwm_port) {
		case BL_PWM_A:
		case BL_PWM_B:
		case BL_PWM_C:
		case BL_PWM_D:
		case BL_PWM_E:
		case BL_PWM_F:
			if (IS_ERR_OR_NULL(bl_pwm->pwm_data.pwm)) {
				pr_info("lidm_pwm invalid\n");
				break;
			}
			pr_info("lidm_pwm_pointer:    0x%p\n",
				bl_pwm->pwm_data.pwm);
			pwm_get_state(bl_pwm->pwm_data.pwm, &pstate);
			pr_info("lidm_pwm state:\n"
				"  period:            %d\n"
				"  duty_cycle:        %d\n"
				"  polarity:          %d\n"
				"  enabled:           %d\n",
				pstate.period, pstate.duty_cycle,
				pstate.polarity, pstate.enabled);
			value = bl_cbus_read(
				bl_drv->data->pwm_reg[bl_pwm->pwm_port]);
			pr_info("lidm_pwm_reg:        0x%08x\n", value);
			break;
		case BL_PWM_VS:
			pr_info("lidm_pwm_reg0:       0x%08x\n"
				"lidm_pwm_reg1:       0x%08x\n"
				"lidm_pwm_reg2:       0x%08x\n"
				"lidm_pwm_reg3:       0x%08x\n",
				bl_vcbus_read(VPU_VPU_PWM_V0),
				bl_vcbus_read(VPU_VPU_PWM_V1),
				bl_vcbus_read(VPU_VPU_PWM_V2),
				bl_vcbus_read(VPU_VPU_PWM_V3));
			break;
		default:
			break;
		}
	}
	bl_pwm = &ldim_drv->ldev_conf->analog_pwm_config;
	if (bl_pwm->pwm_port < BL_PWM_MAX) {
		pr_info("\nanalog_pwm_port:     %d\n"
			"analog_pwm_pol:      %d\n"
			"analog_pwm_freq:     %d\n"
			"analog_pwm_cnt:      %d\n"
			"analog_pwm_level:    %d\n"
			"analog_pwm_duty:     %d%%\n"
			"analog_pwm_duty_max: %d%%\n"
			"analog_pwm_duty_min: %d%%\n",
			bl_pwm->pwm_port, bl_pwm->pwm_method,
			bl_pwm->pwm_freq, bl_pwm->pwm_cnt,
			bl_pwm->pwm_level, bl_pwm->pwm_duty,
			bl_pwm->pwm_duty_max, bl_pwm->pwm_duty_min);
		switch (bl_pwm->pwm_port) {
		case BL_PWM_A:
		case BL_PWM_B:
		case BL_PWM_C:
		case BL_PWM_D:
		case BL_PWM_E:
		case BL_PWM_F:
			if (IS_ERR_OR_NULL(bl_pwm->pwm_data.pwm)) {
				pr_info("analog_pwm invalid\n");
				break;
			}
			pr_info("analog_pwm_pointer:  0x%p\n",
				bl_pwm->pwm_data.pwm);
			pwm_get_state(bl_pwm->pwm_data.pwm, &pstate);
			pr_info("analog_pwm state:\n"
				"  period:            %d\n"
				"  duty_cycle:        %d\n"
				"  polarity:          %d\n"
				"  enabled:           %d\n",
				pstate.period, pstate.duty_cycle,
				pstate.polarity, pstate.enabled);
			value = bl_cbus_read(
				bl_drv->data->pwm_reg[bl_pwm->pwm_port]);
			pr_info("analog_pwm_reg:      0x%08x\n", value);
			break;
		default:
			break;
		}
	}
	pr_info("\npinmux_flag:         %d\n"
		"pinmux_pointer:      0x%p\n\n",
		ldim_drv->pinmux_flag,
		ldim_drv->pin);

	if (ldim_drv->ldev_conf->cmd_size > 0) {
		pr_info("table_loaded:        %d\n"
			"cmd_size:            %d\n"
			"init_on_cnt:         %d\n"
			"init_off_cnt:        %d\n",
			ldim_drv->ldev_conf->init_loaded,
			ldim_drv->ldev_conf->cmd_size,
			ldim_drv->ldev_conf->init_on_cnt,
			ldim_drv->ldev_conf->init_off_cnt);
		if (ldim_drv->ldev_conf->cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ldim_dev_init_table_dynamic_size_print(
				ldim_drv->ldev_conf, 1);
			ldim_dev_init_table_dynamic_size_print(
				ldim_drv->ldev_conf, 0);
		} else {
			ldim_dev_init_table_fixed_size_print(
				ldim_drv->ldev_conf, 1);
			ldim_dev_init_table_fixed_size_print(
				ldim_drv->ldev_conf, 0);
		}
	}
}

static int ldim_dev_pwm_channel_register(struct bl_pwm_config_s *bl_pwm,
		struct device_node *blnode)
{
	int ret = 0;
	int index0 = BL_PWM_MAX;
	int index1 = BL_PWM_MAX;
	phandle pwm_phandle;
	struct device_node *pnode = NULL;
	struct device_node *child;
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();

	if (ldim_debug_print)
		LDIMPR("%s ok\n", __func__);

	ret = of_property_read_u32(blnode, "ldim_pwm_config", &pwm_phandle);
	if (ret) {
		LDIMERR("not match ldim_pwm_config node\n");
		return -1;
	}
	pnode = of_find_node_by_phandle(pwm_phandle);
	if (!pnode) {
		LDIMERR("can't find ldim_pwm_config node\n");
		return -1;
	}

	/*request for pwm device */
	for_each_child_of_node(pnode, child) {
		ret = of_property_read_u32(child, "pwm_port_index", &index0);
		if (ret) {
			LDIMERR("failed to get pwm_port_index\n");
			return ret;
		}
		ret = of_property_read_u32_index(child, "pwms", 1, &index1);
		if (ret) {
			LDIMERR("failed to get meson_pwm_index\n");
			return ret;
		}

		if (index0 >= BL_PWM_VS)
			continue;
		if ((index0 % 2) != index1)
			continue;
		if (index0 != bl_pwm->pwm_port)
			continue;

		bl_pwm->pwm_data.port_index = index0;
		bl_pwm->pwm_data.meson_index = index1;
		bl_pwm->pwm_data.pwm = devm_of_pwm_get(
			ldim_drv->dev, child, NULL);
		if (IS_ERR_OR_NULL(bl_pwm->pwm_data.pwm)) {
			ret = PTR_ERR(bl_pwm->pwm_data.pwm);
			LDIMERR("unable to request bl_pwm\n");
			return ret;
		}
		bl_pwm->pwm_data.meson = to_meson_pwm(
			bl_pwm->pwm_data.pwm->chip);
		pwm_init_state(bl_pwm->pwm_data.pwm, &(bl_pwm->pwm_data.state));
		LDIMPR("register pwm_ch(%d) 0x%p\n",
			bl_pwm->pwm_data.port_index, bl_pwm->pwm_data.pwm);
	}

	return ret;

}

static int ldim_dev_init_table_dynamic_size_load_dts(
		struct device_node *of_node,
		struct ldim_dev_config_s *ldconf, int flag)
{
	unsigned char cmd_size, type;
	int i = 0, j, val, max_len, step = 0, ret = 0;
	unsigned char *table;
	char propname[20];

	if (flag) {
		table = table_init_on_dft;
		max_len = LDIM_INIT_ON_MAX;
		sprintf(propname, "init_on");
	} else {
		table = table_init_off_dft;
		max_len = LDIM_INIT_OFF_MAX;
		sprintf(propname, "init_off");
	}
	if (table == NULL) {
		LDIMERR("%s: init_table is null\n", __func__);
		return -1;
	}

	while ((i + 1) < max_len) {
		/* type */
		ret = of_property_read_u32_index(of_node, propname, i, &val);
		if (ret) {
			LDIMERR("%s: get %s type failed, step %d\n",
				ldconf->name, propname, step);
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
			LDIMERR("%s: get %s cmd_size failed, step %d\n",
				ldconf->name, propname, step);
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
			LDIMERR("%s: %s cmd_size out of support, step %d\n",
				ldconf->name, propname, step);
			table[i] = LCD_EXT_CMD_TYPE_END;
			table[i+1] = 0;
			return -1;
		}

		/* data */
		for (j = 0; j < cmd_size; j++) {
			ret = of_property_read_u32_index(
				of_node, propname, (i+2+j), &val);
			if (ret) {
				LDIMERR("%s: get %s data failed, step %d\n",
					ldconf->name, propname, step);
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
		ldconf->init_on_cnt = i + 2;
	else
		ldconf->init_off_cnt = i + 2;

	return 0;
}

static int ldim_dev_init_table_fixed_size_load_dts(
		struct device_node *of_node,
		struct ldim_dev_config_s *ldconf, int flag)
{
	unsigned char cmd_size;
	int i = 0, j, val, max_len, step = 0, ret = 0;
	unsigned char *table;
	char propname[20];

	cmd_size = ldconf->cmd_size;
	if (flag) {
		table = table_init_on_dft;
		max_len = LDIM_INIT_ON_MAX;
		sprintf(propname, "init_on");
	} else {
		table = table_init_off_dft;
		max_len = LDIM_INIT_OFF_MAX;
		sprintf(propname, "init_off");
	}
	if (table == NULL) {
		LDIMERR("%s: init_table is null\n", __func__);
		return -1;
	}

	while (i < max_len) { /* group detect */
		if ((i + cmd_size) > max_len) {
			LDIMERR("%s: %s cmd_size out of support, step %d\n",
				ldconf->name, propname, step);
			table[i] = LCD_EXT_CMD_TYPE_END;
			return -1;
		}
		for (j = 0; j < cmd_size; j++) {
			ret = of_property_read_u32_index(
				of_node, propname, (i+j), &val);
			if (ret) {
				LDIMERR("%s: get %s failed, step %d\n",
					ldconf->name, propname, step);
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
		ldconf->init_on_cnt = i + cmd_size;
	else
		ldconf->init_off_cnt = i + cmd_size;

	return 0;
}

static int ldim_dev_tablet_init_dft_malloc(void)
{
	table_init_on_dft = kcalloc(LDIM_INIT_ON_MAX,
		sizeof(unsigned char), GFP_KERNEL);
	if (table_init_on_dft == NULL) {
		LDIMERR("failed to alloc init_on table\n");
		return -1;
	}
	table_init_off_dft = kcalloc(LDIM_INIT_OFF_MAX,
		sizeof(unsigned char), GFP_KERNEL);
	if (table_init_off_dft == NULL) {
		LDIMERR("failed to alloc init_off table\n");
		kfree(table_init_on_dft);
		return -1;
	}
	table_init_on_dft[0] = LCD_EXT_CMD_TYPE_END;
	table_init_on_dft[1] = 0;
	table_init_off_dft[0] = LCD_EXT_CMD_TYPE_END;
	table_init_off_dft[1] = 0;

	return 0;
}

static int ldim_dev_table_init_save(struct ldim_dev_config_s *ldconf)
{
	if (ldconf->init_on_cnt > 0) {
		ldconf->init_on = kcalloc(ldconf->init_on_cnt,
			sizeof(unsigned char), GFP_KERNEL);
		if (ldconf->init_on == NULL) {
			LDIMERR("failed to alloc init_on table\n");
			return -1;
		}
		memcpy(ldconf->init_on, table_init_on_dft,
			ldconf->init_on_cnt*sizeof(unsigned char));
	}
	if (ldconf->init_off_cnt > 0) {
		ldconf->init_off = kcalloc(ldconf->init_off_cnt,
			sizeof(unsigned char), GFP_KERNEL);
		if (ldconf->init_off == NULL) {
			LDIMERR("failed to alloc init_off table\n");
			kfree(ldconf->init_on);
			return -1;
		}
		memcpy(ldconf->init_off, table_init_off_dft,
			ldconf->init_on_cnt*sizeof(unsigned char));
	}

	return 0;
}

static int ldim_dev_get_config_from_dts(struct device_node *np, int index)
{
	char ld_propname[20];
	struct device_node *child;
	const char *str;
	unsigned int *temp, val;
	int i;
	int ret = 0;
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();
	struct bl_pwm_config_s *bl_pwm;

	temp = kcalloc(LD_BLKREGNUM, sizeof(unsigned int), GFP_KERNEL);
	if (temp == NULL) {
		LDIMERR("%s: buf malloc error\n", __func__);
		return -1;
	}

	/* get device config */
	sprintf(ld_propname, "ldim_dev_%d", index);
	LDIMPR("load: %s\n", ld_propname);
	child = of_get_child_by_name(np, ld_propname);
	if (child == NULL) {
		LDIMERR("failed to get %s\n", ld_propname);
		goto ldim_get_config_err;
	}

	ret = of_property_read_string(child, "ldim_dev_name", &str);
	if (ret) {
		LDIMERR("failed to get ldim_dev_name\n");
		str = "ldim_dev";
	}
	strcpy(ldim_dev_config.name, str);

	ret = of_property_read_string(child, "ldim_pwm_pinmux_sel", &str);
	if (ret) {
		strcpy(ldim_dev_config.pinmux_name, "invalid");
	} else {
		LDIMPR("find custome ldim_pwm_pinmux_sel: %s\n", str);
		strcpy(ldim_dev_config.pinmux_name, str);
	}

	/* ldim pwm config */
	bl_pwm = &ldim_dev_config.ldim_pwm_config;
	ret = of_property_read_string(child, "ldim_pwm_port", &str);
	if (ret) {
		LDIMERR("failed to get ldim_pwm_port\n");
	} else {
		bl_pwm->pwm_port = bl_pwm_str_to_pwm(str);
		LDIMPR("ldim_pwm_port: %s(%u)\n", str, bl_pwm->pwm_port);
	}
	if (bl_pwm->pwm_port < BL_PWM_MAX) {
		ret = of_property_read_u32_array(child, "ldim_pwm_attr",
			temp, 3);
		if (ret) {
			LDIMERR("failed to get ldim_pwm_attr\n");
			bl_pwm->pwm_method = BL_PWM_POSITIVE;
			if (bl_pwm->pwm_port == BL_PWM_VS)
				bl_pwm->pwm_freq = 1;
			else
				bl_pwm->pwm_freq = 60;
			bl_pwm->pwm_duty = 50;
		} else {
			bl_pwm->pwm_method = temp[0];
			bl_pwm->pwm_freq = temp[1];
			bl_pwm->pwm_duty = temp[2];
		}
		LDIMPR(
		"get ldim_pwm pol = %d, freq = %d, default duty = %d%%\n",
			bl_pwm->pwm_method, bl_pwm->pwm_freq,
			bl_pwm->pwm_duty);

		bl_pwm_config_init(bl_pwm);

		if (bl_pwm->pwm_port < BL_PWM_VS)
			ldim_dev_pwm_channel_register(bl_pwm, np);
	}

	/* analog pwm config */
	bl_pwm = &ldim_dev_config.analog_pwm_config;
	ret = of_property_read_string(child, "analog_pwm_port", &str);
	if (ret)
		bl_pwm->pwm_port = BL_PWM_MAX;
	else
		bl_pwm->pwm_port = bl_pwm_str_to_pwm(str);
	if (bl_pwm->pwm_port < BL_PWM_VS) {
		LDIMPR("find analog_pwm_port: %s(%u)\n", str, bl_pwm->pwm_port);
		ret = of_property_read_u32_array(child, "analog_pwm_attr",
			temp, 5);
		if (ret) {
			LDIMERR("failed to get analog_pwm_attr\n");
		} else {
			bl_pwm->pwm_method = temp[0];
			bl_pwm->pwm_freq = temp[1];
			bl_pwm->pwm_duty_max = temp[2];
			bl_pwm->pwm_duty_min = temp[3];
			bl_pwm->pwm_duty = temp[4];
		}
		LDIMPR(
"get analog_pwm pol = %d, freq = %d, duty_max = %d%%, duty_min = %d%%, default duty = %d%%\n",
			bl_pwm->pwm_method, bl_pwm->pwm_freq,
			bl_pwm->pwm_duty_max, bl_pwm->pwm_duty_min,
			bl_pwm->pwm_duty);

		bl_pwm_config_init(bl_pwm);

		ldim_dev_pwm_channel_register(bl_pwm, np);
	}

	ret = of_property_read_u32_array(child, "en_gpio_on_off", temp, 3);
	if (ret) {
		LDIMERR("failed to get en_gpio_on_off\n");
		ldim_dev_config.en_gpio = BL_GPIO_MAX;
		ldim_dev_config.en_gpio_on = BL_GPIO_OUTPUT_HIGH;
		ldim_dev_config.en_gpio_off = BL_GPIO_OUTPUT_LOW;
	} else {
		if (temp[0] >= BL_GPIO_NUM_MAX) {
			ldim_dev_config.en_gpio = BL_GPIO_MAX;
		} else {
			ldim_dev_config.en_gpio = temp[0];
			ldim_gpio_probe(ldim_dev_config.en_gpio);
		}
		ldim_dev_config.en_gpio_on = temp[1];
		ldim_dev_config.en_gpio_off = temp[2];
	}

	ret = of_property_read_u32_array(child, "dim_max_min", &temp[0], 2);
	if (ret) {
		LDIMERR("failed to get dim_max_min\n");
		ldim_dev_config.dim_max = 0xfff;
		ldim_dev_config.dim_min = 0x7f;
	} else {
		ldim_dev_config.dim_max = temp[0];
		ldim_dev_config.dim_min = temp[1];
	}

	val = ldim_drv->ldim_conf->row * ldim_drv->ldim_conf->col;
	ret = of_property_read_u32_array(child, "ldim_region_mapping",
			&temp[0], val);
	if (ret) {
		LDIMERR("failed to get ldim_region_mapping\n");
		for (i = 0; i < LD_BLKREGNUM; i++)
			ldim_dev_config.bl_mapping[i] = (unsigned short)i;
	} else {
		for (i = 0; i < val; i++)
			ldim_dev_config.bl_mapping[i] = (unsigned short)temp[i];
	}
	ldim_dev_config.bl_regnum = (unsigned short)val;

	ret = of_property_read_u32(child, "type", &val);
	if (ret) {
		LDIMERR("failed to get type\n");
		ldim_dev_config.type = LDIM_DEV_TYPE_NORMAL;
	} else {
		ldim_dev_config.type = val;
		if (ldim_debug_print)
			LDIMPR("type: %d\n", ldim_dev_config.type);
	}
	if (ldim_dev_config.type >= LDIM_DEV_TYPE_MAX) {
		LDIMERR("type num is out of support\n");
		goto ldim_get_config_err;
	}

	ret = ldim_dev_tablet_init_dft_malloc();
	if (ret)
		goto ldim_get_config_err;
	switch (ldim_dev_config.type) {
	case LDIM_DEV_TYPE_SPI:
		/* get spi config */
		ldim_drv->spi_info = &ldim_spi_info;

		ret = of_property_read_u32(child, "spi_bus_num", &val);
		if (ret) {
			LDIMERR("failed to get spi_bus_num\n");
		} else {
			ldim_spi_info.bus_num = val;
			if (ldim_debug_print) {
				LDIMPR("spi bus_num: %d\n",
					ldim_spi_info.bus_num);
			}
		}

		ret = of_property_read_u32(child, "spi_chip_select", &val);
		if (ret) {
			LDIMERR("failed to get spi_chip_select\n");
		} else {
			ldim_spi_info.chip_select = val;
			if (ldim_debug_print) {
				LDIMPR("spi chip_select: %d\n",
					ldim_spi_info.chip_select);
			}
		}

		ret = of_property_read_u32(child, "spi_max_frequency", &val);
		if (ret) {
			LDIMERR("failed to get spi_chip_select\n");
		} else {
			ldim_spi_info.max_speed_hz = val;
			if (ldim_debug_print) {
				LDIMPR("spi max_speed_hz: %d\n",
					ldim_spi_info.max_speed_hz);
			}
		}

		ret = of_property_read_u32(child, "spi_mode", &val);
		if (ret) {
			LDIMERR("failed to get spi_mode\n");
		} else {
			ldim_spi_info.mode = val;
			if (ldim_debug_print)
				LDIMPR("spi mode: %d\n", ldim_spi_info.mode);
		}

		ret = of_property_read_u32_array(child, "spi_cs_delay",
			&temp[0], 2);
		if (ret) {
			ldim_dev_config.cs_hold_delay = 0;
			ldim_dev_config.cs_clk_delay = 0;
		} else {
			ldim_dev_config.cs_hold_delay = temp[0];
			ldim_dev_config.cs_clk_delay = temp[1];
		}

		ret = of_property_read_u32(child, "lamp_err_gpio", &val);
		if (ret) {
			ldim_dev_config.lamp_err_gpio = LCD_EXT_GPIO_INVALID;
			ldim_dev_config.fault_check = 0;
		} else {
			if (val >= BL_GPIO_NUM_MAX) {
				ldim_dev_config.lamp_err_gpio =
					LCD_EXT_GPIO_INVALID;
				ldim_dev_config.fault_check = 0;
			} else {
				ldim_dev_config.lamp_err_gpio = val;
				ldim_dev_config.fault_check = 1;
				ldim_gpio_probe(
					ldim_dev_config.lamp_err_gpio);
				ldim_gpio_set(ldim_dev_config.lamp_err_gpio,
					BL_GPIO_INPUT);
			}
		}

		ret = of_property_read_u32(child, "spi_write_check", &val);
		if (ret)
			ldim_dev_config.write_check = 0;
		else
			ldim_dev_config.write_check = (unsigned char)val;

		/* get init_cmd */
		ret = of_property_read_u32(child, "cmd_size", &val);
		if (ret) {
			LDIMPR("no cmd_size\n");
			ldim_dev_config.cmd_size = 0;
		} else {
			ldim_dev_config.cmd_size = (unsigned char)val;
		}
		if (ldim_debug_print) {
			LDIMPR("%s: cmd_size = %d\n",
				ldim_dev_config.name,
				ldim_dev_config.cmd_size);
		}
		if (ldim_dev_config.cmd_size == 0)
			break;

		if (ldim_dev_config.cmd_size == LCD_EXT_CMD_SIZE_DYNAMIC) {
			ret = ldim_dev_init_table_dynamic_size_load_dts(
				child, &ldim_dev_config, 1);
			if (ret)
				break;
			ldim_dev_init_table_dynamic_size_load_dts(
				child, &ldim_dev_config, 0);
		} else {
			ret = ldim_dev_init_table_fixed_size_load_dts(
				child, &ldim_dev_config, 1);
			if (ret)
				break;
			ldim_dev_init_table_fixed_size_load_dts(
				child, &ldim_dev_config, 0);
		}
		if (ret == 0)
			ldim_dev_config.init_loaded = 1;
		break;
	case LDIM_DEV_TYPE_I2C:
		break;
	case LDIM_DEV_TYPE_NORMAL:
	default:
		break;
	}

	if (ldim_dev_config.init_loaded > 0) {
		ret = ldim_dev_table_init_save(&ldim_dev_config);
		if (ret)
			goto ldim_get_config_init_table_err;
	}

	kfree(table_init_on_dft);
	kfree(table_init_off_dft);
	kfree(temp);
	return 0;

ldim_get_config_init_table_err:
	kfree(table_init_on_dft);
	kfree(table_init_off_dft);
ldim_get_config_err:
	kfree(temp);
	return -1;
}

static ssize_t ldim_dev_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int ret = 0;

	ldim_dev_config_print();

	return ret;
}

static ssize_t ldim_dev_pwm_ldim_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct bl_pwm_config_s *bl_pwm;
	ssize_t len = 0;

	bl_pwm = &ldim_dev_config.ldim_pwm_config;
	if (bl_pwm->pwm_port < BL_PWM_MAX) {
		len += sprintf(buf+len,
			"ldim_pwm: freq=%d, pol=%d, duty_max=%d, duty_min=%d,",
			bl_pwm->pwm_freq, bl_pwm->pwm_method,
			bl_pwm->pwm_duty_max, bl_pwm->pwm_duty_min);
		len += sprintf(buf+len, " duty_value=%d%%\n", bl_pwm->pwm_duty);
	}

	return len;
}

static ssize_t ldim_dev_pwm_ldim_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int val = 0;
	struct bl_pwm_config_s *bl_pwm;

	bl_pwm = &ldim_dev_config.ldim_pwm_config;
	if (bl_pwm->pwm_port >= BL_PWM_MAX)
		return count;

	switch (buf[0]) {
	case 'f': /* frequency */
		ret = sscanf(buf, "freq %d", &val);
		if (ret == 1) {
			bl_pwm->pwm_freq = val;
			bl_pwm_config_init(bl_pwm);
			ldim_set_duty_pwm(bl_pwm);
			if (ldim_debug_print) {
				LDIMPR("set ldim_pwm (port %d): freq = %dHz\n",
					bl_pwm->pwm_port, bl_pwm->pwm_freq);
			}
		} else {
			LDIMERR("invalid parameters\n");
		}
		break;
	case 'd': /* duty */
		ret = sscanf(buf, "duty %d", &val);
		if (ret == 1) {
			bl_pwm->pwm_duty = val;
			ldim_set_duty_pwm(bl_pwm);
			if (ldim_debug_print) {
				LDIMPR("set ldim_pwm (port %d): duty = %d%%\n",
					bl_pwm->pwm_port, bl_pwm->pwm_duty);
			}
		} else {
			LDIMERR("invalid parameters\n");
		}
		break;
	case 'p': /* polarity */
		ret = sscanf(buf, "pol %d", &val);
		if (ret == 1) {
			bl_pwm->pwm_method = val;
			bl_pwm_config_init(bl_pwm);
			ldim_set_duty_pwm(bl_pwm);
			if (ldim_debug_print) {
				LDIMPR("set ldim_pwm (port %d): method = %d\n",
					bl_pwm->pwm_port, bl_pwm->pwm_method);
			}
		} else {
			LDIMERR("invalid parameters\n");
		}
		break;
	case 'm':
		if (buf[1] == 'a') { /* max */
			ret = sscanf(buf, "max %d", &val);
			if (ret == 1) {
				bl_pwm->pwm_duty_max = val;
				if (ldim_dev_config.dim_range_update)
					ldim_dev_config.dim_range_update();
				bl_pwm_config_init(bl_pwm);
				ldim_set_duty_pwm(bl_pwm);
				if (ldim_debug_print) {
					LDIMPR(
				"set ldim_pwm (port %d): duty_max = %d%%\n",
						bl_pwm->pwm_port,
						bl_pwm->pwm_duty_max);
				}
			} else {
				LDIMERR("invalid parameters\n");
			}
		} else if (buf[1] == 'i') { /* min */
			ret = sscanf(buf, "min %d", &val);
			if (ret == 1) {
				bl_pwm->pwm_duty_min = val;
				if (ldim_dev_config.dim_range_update)
					ldim_dev_config.dim_range_update();
				bl_pwm_config_init(bl_pwm);
				ldim_set_duty_pwm(bl_pwm);
				if (ldim_debug_print) {
					LDIMPR(
				"set ldim_pwm (port %d): duty_min = %d%%\n",
						bl_pwm->pwm_port,
						bl_pwm->pwm_duty_min);
				}
			} else {
				LDIMERR("invalid parameters\n");
			}
		}
		break;
	default:
		LDIMERR("wrong command\n");
		break;
	}

	return count;
}

static ssize_t ldim_dev_pwm_analog_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct bl_pwm_config_s *bl_pwm;
	ssize_t len = 0;

	bl_pwm = &ldim_dev_config.analog_pwm_config;
	if (bl_pwm->pwm_port < BL_PWM_VS) {
		len += sprintf(buf+len,
			"analog_pwm: freq=%d, pol=%d, duty_max=%d, duty_min=%d,",
			bl_pwm->pwm_freq, bl_pwm->pwm_method,
			bl_pwm->pwm_duty_max, bl_pwm->pwm_duty_min);
		len += sprintf(buf+len, " duty_value=%d%%\n", bl_pwm->pwm_duty);
	}

	return len;
}

static ssize_t ldim_dev_pwm_analog_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int val = 0;
	struct bl_pwm_config_s *bl_pwm;

	bl_pwm = &ldim_dev_config.analog_pwm_config;
	if (bl_pwm->pwm_port >= BL_PWM_VS)
		return count;

	switch (buf[0]) {
	case 'f': /* frequency */
		ret = sscanf(buf, "freq %d", &val);
		if (ret == 1) {
			bl_pwm->pwm_freq = val;
			bl_pwm_config_init(bl_pwm);
			ldim_set_duty_pwm(bl_pwm);
			if (ldim_debug_print) {
				LDIMPR("set ldim_pwm (port %d): freq = %dHz\n",
					bl_pwm->pwm_port, bl_pwm->pwm_freq);
			}
		} else {
			LDIMERR("invalid parameters\n");
		}
		break;
	case 'd': /* duty */
		ret = sscanf(buf, "duty %d", &val);
		if (ret == 1) {
			bl_pwm->pwm_duty = val;
			ldim_set_duty_pwm(bl_pwm);
			if (ldim_debug_print) {
				LDIMPR("set ldim_pwm (port %d): duty = %d%%\n",
					bl_pwm->pwm_port, bl_pwm->pwm_duty);
			}
		} else {
			LDIMERR("invalid parameters\n");
		}
		break;
	case 'p': /* polarity */
		ret = sscanf(buf, "pol %d", &val);
		if (ret == 1) {
			bl_pwm->pwm_method = val;
			bl_pwm_config_init(bl_pwm);
			ldim_set_duty_pwm(bl_pwm);
			if (ldim_debug_print) {
				LDIMPR("set ldim_pwm (port %d): method = %d\n",
					bl_pwm->pwm_port, bl_pwm->pwm_method);
			}
		} else {
			LDIMERR("invalid parameters\n");
		}
		break;
	case 'm':
		if (buf[1] == 'a') { /* max */
			ret = sscanf(buf, "max %d", &val);
			if (ret == 1) {
				bl_pwm->pwm_duty_max = val;
				bl_pwm_config_init(bl_pwm);
				ldim_set_duty_pwm(bl_pwm);
				if (ldim_debug_print) {
					LDIMPR(
				"set ldim_pwm (port %d): duty_max = %d%%\n",
						bl_pwm->pwm_port,
						bl_pwm->pwm_duty_max);
				}
			} else {
				LDIMERR("invalid parameters\n");
			}
		} else if (buf[1] == 'i') { /* min */
			ret = sscanf(buf, "min %d", &val);
			if (ret == 1) {
				bl_pwm->pwm_duty_min = val;
				bl_pwm_config_init(bl_pwm);
				ldim_set_duty_pwm(bl_pwm);
				if (ldim_debug_print) {
					LDIMPR(
				"set ldim_pwm (port %d): duty_min = %d%%\n",
						bl_pwm->pwm_port,
						bl_pwm->pwm_duty_min);
				}
			} else {
				LDIMERR("invalid parameters\n");
			}
		}
		break;
	default:
		LDIMERR("wrong command\n");
		break;
	}

	return count;
}

static unsigned char ldim_dev_reg;
static unsigned char ldim_dev_reg_dump_cnt;
static ssize_t ldim_dev_reg_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	unsigned char data;
	ssize_t len = 0;
	int ret;

	if (ldim_dev_config.dev_reg_read == NULL)
		return sprintf(buf, "ldim dev_reg_read is null\n");

	data = ldim_dev_reg;
	ret = ldim_dev_config.dev_reg_read(&data, 1);
	if (ret) {
		len = sprintf(buf, "reg[0x%02x] read error\n", ldim_dev_reg);
	} else {
		len = sprintf(buf, "reg[0x%02x] = 0x%02x\n",
			ldim_dev_reg, data);
	}

	return len;
}

static ssize_t ldim_dev_reg_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int reg = 0, val = 0;
	unsigned char data[2];
	unsigned int ret;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 1) {
		if (reg > 0xff) {
			LDIMERR("invalid reg address: 0x%x\n", reg);
			return count;
		}
		ldim_dev_reg = (unsigned char)reg;
	} else if (ret == 2) {
		if (ldim_dev_config.dev_reg_write == NULL) {
			LDIMERR("ldim dev_reg_write is null\n");
			return count;
		}
		if (reg > 0xff) {
			LDIMERR("invalid reg address: 0x%x\n", reg);
			return count;
		}
		ldim_dev_reg = (unsigned char)reg;
		data[0] = ldim_dev_reg;
		data[1] = (unsigned char)val;
		ldim_dev_config.dev_reg_write(data, 2);
		LDIMPR("write reg[0x%02x] = 0x%02x\n", data[0], data[1]);
	} else {
		LDIMERR("invalid parameters\n");
	}

	return count;
}

static ssize_t ldim_dev_reg_dump_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	unsigned char *data;
	ssize_t len = 0;
	int i, ret;

	if (ldim_dev_config.dev_reg_read == NULL)
		return sprintf(buf, "ldim dev_reg_read is null\n");

	if (ldim_dev_reg_dump_cnt == 0)
		return sprintf(buf, "ldim reg_dump_cnt is zero\n");

	data = kcalloc(ldim_dev_reg_dump_cnt,
		sizeof(unsigned char), GFP_KERNEL);
	ret = ldim_dev_config.dev_reg_read(data, ldim_dev_reg_dump_cnt);
	if (ret) {
		len = sprintf(buf, "reg[0x%02x] read error\n", ldim_dev_reg);
	} else {
		len += sprintf(buf+len, "reg[0x%02x] = ", ldim_dev_reg);
		for (i = 0; i < (ldim_dev_reg_dump_cnt - 1); i++)
			len += sprintf(buf+len, "0x%02x,", data[i]);
		len += sprintf(buf+len, "0x%02x\n",
			data[ldim_dev_reg_dump_cnt - 1]);
	}
	kfree(data);

	return len;
}

static ssize_t ldim_dev_reg_dump_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int reg = 0, cnt = 0;
	unsigned int ret;

	ret = sscanf(buf, "%x %d", &reg, &cnt);
	if (ret == 2) {
		if (reg > 0xff) {
			LDIMERR("invalid reg address: 0x%x\n", reg);
			return count;
		}
		if (cnt > 0xff) {
			LDIMERR("invalid reg cnt: %d\n", cnt);
			return count;
		}
		ldim_dev_reg = (unsigned char)reg;
		ldim_dev_reg_dump_cnt = (unsigned char)cnt;
	} else {
		LDIMERR("invalid parameters\n");
	}

	return count;
}

static struct class_attribute ldim_dev_class_attrs[] = {
	__ATTR(status, 0644, ldim_dev_show, NULL),
	__ATTR(pwm_ldim, 0644, ldim_dev_pwm_ldim_show, ldim_dev_pwm_ldim_store),
	__ATTR(pwm_analog, 0644, ldim_dev_pwm_analog_show,
		ldim_dev_pwm_analog_store),
	__ATTR(reg, 0644, ldim_dev_reg_show, ldim_dev_reg_store),
	__ATTR(reg_dump, 0644, ldim_dev_reg_dump_show, ldim_dev_reg_dump_store),
	__ATTR_NULL
};

static void ldim_dev_class_create(void)
{
	int ret;

	ldim_dev_class.name = kzalloc(10, GFP_KERNEL);
	if (ldim_dev_class.name == NULL) {
		LDIMERR("%s: malloc failed\n", __func__);
		return;
	}
	sprintf((char *)ldim_dev_class.name, "ldim_dev");
	ldim_dev_class.class_attrs = ldim_dev_class_attrs;
	ret = class_register(&ldim_dev_class);
	if (ret < 0)
		LDIMERR("register ldim_dev_class failed\n");
}

static int ldim_dev_add_driver(struct aml_ldim_driver_s *ldim_drv)
{
	struct ldim_dev_config_s *ldev_conf = ldim_drv->ldev_conf;
	int index = ldim_drv->dev_index;
	int ret = 0;

	switch (ldim_dev_config.type) {
	case LDIM_DEV_TYPE_SPI:
		ret = ldim_spi_driver_add(ldim_drv);
		break;
	case LDIM_DEV_TYPE_I2C:
		break;
	case LDIM_DEV_TYPE_NORMAL:
	default:
		break;
	}
	if (ret)
		return ret;

	ret = -1;
	if (strcmp(ldev_conf->name, "iw7027") == 0)
		ret = ldim_dev_iw7027_probe(ldim_drv);
	else if (strcmp(ldev_conf->name, "ob3350") == 0)
		ret = ldim_dev_ob3350_probe(ldim_drv);
	else if (strcmp(ldev_conf->name, "global") == 0)
		ret = ldim_dev_global_probe(ldim_drv);
	else
		LDIMERR("invalid device name: %s\n", ldev_conf->name);

	if (ret) {
		LDIMERR("add device driver failed: %s(%d)\n",
			ldev_conf->name, index);
	} else {
		ldim_dev_probe_flag = 1;
		LDIMPR("add device driver: %s(%d)\n", ldev_conf->name, index);
	}

	return ret;
}

static int ldim_dev_remove_driver(struct aml_ldim_driver_s *ldim_drv)
{
	struct ldim_dev_config_s *ldev_conf = ldim_drv->ldev_conf;
	int index = ldim_drv->dev_index;
	int ret = -1;

	if (ldim_dev_probe_flag) {
		if (strcmp(ldev_conf->name, "iw7027") == 0)
			ret = ldim_dev_iw7027_remove(ldim_drv);
		else if (strcmp(ldev_conf->name, "ob3350") == 0)
			ret = ldim_dev_ob3350_remove(ldim_drv);
		else if (strcmp(ldev_conf->name, "global") == 0)
			ret = ldim_dev_global_remove(ldim_drv);
		else
			LDIMERR("invalid device name: %s\n", ldev_conf->name);

		if (ret) {
			LDIMERR("remove device driver failed: %s(%d)\n",
				ldev_conf->name, index);
		} else {
			ldim_dev_probe_flag = 0;
			LDIMPR("remove device driver: %s(%d)\n",
				ldev_conf->name, index);
		}
	}

	switch (ldim_dev_config.type) {
	case LDIM_DEV_TYPE_SPI:
		ldim_spi_driver_remove(ldim_drv);
		break;
	case LDIM_DEV_TYPE_I2C:
		break;
	case LDIM_DEV_TYPE_NORMAL:
	default:
		break;
	}

	return ret;
}

static int ldim_dev_probe(struct platform_device *pdev)
{
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();

	ldim_dev_probe_flag = 0;
	if (ldim_drv->dev_index != 0xff) {
		/* get configs */
		ldim_drv->dev = &pdev->dev;
		ldim_drv->ldev_conf = &ldim_dev_config;
		ldim_drv->pinmux_ctrl = ldim_pwm_pinmux_ctrl;
		ldim_drv->pwm_vs_update = ldim_pwm_vs_update;
		ldim_drv->config_print = ldim_dev_config_print,
		ldim_dev_get_config_from_dts(pdev->dev.of_node,
			ldim_drv->dev_index);

		ldim_dev_class_create();
		ldim_dev_add_driver(ldim_drv);

		/* init ldim function */
		if (ldim_drv->valid_flag)
			ldim_drv->init();
		LDIMPR("%s OK\n", __func__);
	}

	return 0;
}

static int __exit ldim_dev_remove(struct platform_device *pdev)
{
	struct aml_ldim_driver_s *ldim_drv = aml_ldim_get_driver();

	if (ldim_drv->dev_index != 0xff) {
		ldim_dev_remove_driver(ldim_drv);
		LDIMPR("%s OK\n", __func__);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ldim_dev_dt_match[] = {
	{
		.compatible = "amlogic, ldim_dev",
	},
	{},
};
#endif

static struct platform_driver ldim_dev_platform_driver = {
	.driver = {
		.name  = "ldim_dev",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ldim_dev_dt_match,
#endif
	},
	.probe   = ldim_dev_probe,
	.remove  = __exit_p(ldim_dev_remove),
};

static int __init ldim_dev_init(void)
{
	if (platform_driver_register(&ldim_dev_platform_driver)) {
		LDIMPR("failed to register ldim_dev driver module\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit ldim_dev_exit(void)
{
	platform_driver_unregister(&ldim_dev_platform_driver);
}

late_initcall(ldim_dev_init);
module_exit(ldim_dev_exit);


MODULE_DESCRIPTION("LDIM device Driver for LCD Backlight");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");

