// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Rockchip Electronics Co., Ltd.
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>
#include "fp9931.h"

#define FP9931_MAX_ENABLE_GPIO_NUM 4

struct fp9931_data {
	struct regmap *regmap;
	struct gpio_descs *power_gpio;
	struct gpio_desc *enable_gpio;
	/*
	 * When registering a new regulator, regulator core will try to
	 * call "set_suspend_disable" to check whether regulator device
	 * working well.
	 * Just skip this suspend to avoid reconfiguring pinctrl.
	 */
	bool initial_suspend;
	u8 vcom_reg;
	u8 vpos_vneg_reg;
};

/* in uV */
static const int fp9931_vpos_vneg_voltages[] = {
	7040000, 7040000, 7040000, 7040000, 7040000,
	7260000, 7490000, 7710000, 7930000, 8150000,
	8380000, 8600000, 8820000, 9040000, 9270000,
	9490000, 9710000, 9940000, 10160000, 10380000,
	10600000, 10830000, 11050000, 11270000, 11490000,
	11720000, 11940000, 12160000, 12380000, 12610000,
	12830000, 13050000, 13280000, 13500000, 13720000,
	13940000, 14170000, 14390000, 14610000, 14830000,
	15060000, 15060000, 15060000, 15060000, 15060000,
	15060000, 15060000, 15060000, 15060000, 15060000,
	15060000, 15060000, 15060000, 15060000, 15060000,
	15060000, 15060000, 15060000, 15060000, 15060000,
	15060000, 15060000, 15060000, 15060000,
};

static int fp9931_vcom_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, FP9931_VCOM_SETTING, &val);
	if (ret)
		return ret;

	/* 0V for Vcom is meaningless */
	if (!val)
		return -EINVAL;

	return --val;
}

static int fp9931_vcom_set_voltage_sel(struct regulator_dev *rdev,
				       unsigned selector)
{
	struct fp9931_data *data = rdev->reg_data;
	int ret;

	if (++selector > 0xFF)
		return -EINVAL;

	data->vcom_reg = selector;
	ret = regmap_write(rdev->regmap, FP9931_VCOM_SETTING, selector);

	return ret;
}

/* VCOM = 0V + [(-5 / 255) * N]V, N = 1~255 (0 is meaningless) */
static int fp9931_vcom_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int vol_uV;

	if (++selector > 0xFF)
		return -EINVAL;

	vol_uV = selector * 1000 * 1000;

	return DIV_ROUND_CLOSEST(vol_uV * 5, 255);
}

static int fp9931_vcom_enable(struct regulator_dev *rdev)
{
	struct fp9931_data *data = rdev->reg_data;
	int ret;

	ret = regmap_write_bits(rdev->regmap, FP9931_CONTROL_REG1, CONTROL_REG1_V3P3_EN,
				CONTROL_REG1_V3P3_EN);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(data->enable_gpio, 1);

	return 0;
}

static int fp9931_vcom_disable(struct regulator_dev *rdev)
{
	struct fp9931_data *data = rdev->reg_data;

	/* V3P3 is auto off when EN pin disable */
	gpiod_set_value_cansleep(data->enable_gpio, 0);

	return 0;
}

static int fp9931_vpos_vneg_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	struct fp9931_data *data = rdev->reg_data;
	int ret;

	if (unlikely(selector > VPOS_VNEG_SETTING))
		return -EINVAL;

	data->vpos_vneg_reg = selector;
	ret = regmap_write_bits(rdev->regmap, FP9931_VPOS_VNEG_SETTING, VPOS_VNEG_SETTING,
				selector);

	return ret;
}

static int fp9931_vpos_vneg_get_voltage_sel(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, FP9931_VPOS_VNEG_SETTING, &val);
	if (ret)
		return ret;

	return val & VPOS_VNEG_SETTING;
}

static int fp9931_is_enabled(struct regulator_dev *rdev)
{
	struct fp9931_data *data = rdev->reg_data;

	return !!gpiod_get_value_cansleep(data->enable_gpio);
}

static int fp9931_get_status(struct regulator_dev *rdev)
{
	return 0;
}

static int fp9931_set_suspend_disable(struct regulator_dev *rdev)
{
	DECLARE_BITMAP(values, FP9931_MAX_ENABLE_GPIO_NUM);
	struct fp9931_data *data = rdev->reg_data;
	int ret;

	bitmap_zero(values, FP9931_MAX_ENABLE_GPIO_NUM);

	/* Skip initial suspend */
	if (data->initial_suspend) {
		data->initial_suspend = false;
		return 0;
	}

	gpiod_set_value_cansleep(data->enable_gpio, 0);

	if (data->power_gpio) {
		ret = gpiod_set_array_value_cansleep(data->power_gpio->ndescs,
						     data->power_gpio->desc,
						     data->power_gpio->info, values);
		if (ret)
			return ret;
	}

	return 0;
}

static int fp9931_resume(struct regulator_dev *rdev)
{
	DECLARE_BITMAP(values, FP9931_MAX_ENABLE_GPIO_NUM);
	struct fp9931_data *data = rdev->reg_data;
	int ret;

	bitmap_fill(values, FP9931_MAX_ENABLE_GPIO_NUM);

	if (data->power_gpio) {
		ret = gpiod_set_array_value_cansleep(data->power_gpio->ndescs,
						     data->power_gpio->desc,
						     data->power_gpio->info, values);
		if (ret)
			return ret;
	}

	/* Waiting for i2c to become available after power up */
	usleep_range(1000, 1500);

	/* VCOM/VPOS_VNEG setting resume */
	ret = regmap_write(rdev->regmap, FP9931_VCOM_SETTING, data->vcom_reg);
	if (ret)
		return ret;

	ret = regmap_write_bits(rdev->regmap, FP9931_VPOS_VNEG_SETTING, VPOS_VNEG_SETTING,
				data->vpos_vneg_reg);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(data->enable_gpio, 1);

	return 0;
}

static const struct regulator_ops fp9931_vpos_vneg_volt_ops = {
	.list_voltage = regulator_list_voltage_table,
	.set_voltage_sel = fp9931_vpos_vneg_set_voltage_sel,
	.get_voltage_sel = fp9931_vpos_vneg_get_voltage_sel,
	.is_enabled = fp9931_is_enabled,
};

static const struct regulator_desc vpos_vneg_desc = {
	.name = "vpos_vneg",
	.id = 0,
	.ops = &fp9931_vpos_vneg_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.n_voltages = ARRAY_SIZE(fp9931_vpos_vneg_voltages),
	.volt_table = fp9931_vpos_vneg_voltages,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vpos_vneg"),
};

static const struct regulator_ops fp9931_vcom_volt_ops = {
	.list_voltage = fp9931_vcom_list_voltage,
	.get_voltage_sel = fp9931_vcom_get_voltage_sel,
	.set_voltage_sel = fp9931_vcom_set_voltage_sel,
	.enable = fp9931_vcom_enable,
	.disable = fp9931_vcom_disable,
	.is_enabled = fp9931_is_enabled,
	.get_status = fp9931_get_status,
	.set_suspend_disable = fp9931_set_suspend_disable,
	.resume = fp9931_resume,
};

static const struct regulator_desc vcom_desc = {
	.name = "vcom",
	.id = 0,
	.ops = &fp9931_vcom_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vcom"),
	.n_voltages = 255,
};

static int fp9931_regulator_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct fp9931_data *data;
	int ret, val;

	if (!regmap)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(struct fp9931_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = regmap;
	data->initial_suspend = true;

	data->power_gpio = devm_gpiod_get_array_optional(pdev->dev.parent, "power",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(data->power_gpio)) {
		dev_err(pdev->dev.parent, "Failed to get fp9931 power gpio %ld\n",
			PTR_ERR(data->power_gpio));
		return PTR_ERR(data->power_gpio);
	}

	/* Waiting for i2c to become available after power up */
	usleep_range(1000, 1500);

	data->enable_gpio = devm_gpiod_get(pdev->dev.parent, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(data->enable_gpio)) {
		dev_err(pdev->dev.parent, "Failed to get fp9931 enable gpio %ld\n",
			PTR_ERR(data->enable_gpio));
		return PTR_ERR(data->enable_gpio);
	}

	ret = regmap_read(regmap, FP9931_VCOM_SETTING, &val);
	if (ret)
		goto fail;

	data->vcom_reg = val & 0xFF;

	ret = regmap_read(regmap, FP9931_VPOS_VNEG_SETTING, &val);
	if (ret)
		goto fail;

	data->vpos_vneg_reg = val & VPOS_VNEG_SETTING;

	gpiod_set_value_cansleep(data->enable_gpio, 1);

	platform_set_drvdata(pdev, data);

	config.dev = &pdev->dev;
	config.dev->of_node = pdev->dev.parent->of_node;
	config.regmap = regmap;
	config.driver_data = data;

	rdev = devm_regulator_register(&pdev->dev, &vcom_desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(pdev->dev.parent, "Failed to register %s vcom\n", pdev->name);
		return PTR_ERR(rdev);
	}

	rdev = devm_regulator_register(&pdev->dev, &vpos_vneg_desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(pdev->dev.parent, "Failed to register %s vpos_vneg\n", pdev->name);
		return PTR_ERR(rdev);
	}

	return 0;

fail:
	dev_err(pdev->dev.parent, "Failed to initialize regulator: %d\n", ret);
	return ret;
}

static const struct platform_device_id fp9931_regulator_id_table[] = {
	{ "fp9931-regulator", },
	{ }
};
MODULE_DEVICE_TABLE(platform, fp9931_regulator_id_table);

static struct platform_driver fp9931_regulator_driver = {
	.driver = {
		.name = "fp9931-regulator",
	},
	.probe = fp9931_regulator_probe,
	.id_table = fp9931_regulator_id_table,
};
module_platform_driver(fp9931_regulator_driver);

MODULE_DESCRIPTION("FP9931 voltage regulator driver");
MODULE_LICENSE("GPL");
