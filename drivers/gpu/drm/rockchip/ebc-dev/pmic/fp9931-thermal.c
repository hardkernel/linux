// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Rockchip Electronics Co., Ltd.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#include "fp9931.h"

struct fp9931_data {
	struct regmap *regmap;
	struct thermal_zone_device *thermal_zone_dev;
	struct regulator *regulator;
};

static int fp9931_get_temp(struct thermal_zone_device *dev, int *res)
{
	struct fp9931_data *data = dev->devdata;
	unsigned int reg_val;
	int ret = 0;

	ret = regulator_is_enabled(data->regulator);
	if (ret <= 0)
		return -EBUSY;

	ret = regmap_read(data->regmap, FP9931_TMST_VALUE, &reg_val);
	if (!ret) {
		*res = *((signed char *)&reg_val);
		*res *= 1000;
	}

	return ret;
}

static const struct thermal_zone_device_ops ops = {
	.get_temp = fp9931_get_temp,
};

static int fp9931_thermal_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct fp9931_data *data;

	if (!regmap)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(struct fp9931_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->regulator = devm_regulator_get(&pdev->dev, "vcom");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Unable to get fp993x vcom regulator, returned %ld\n",
			PTR_ERR(data->regulator));
		return PTR_ERR(data->regulator);
	}

	data->regmap = regmap;

	data->thermal_zone_dev = devm_thermal_of_zone_register(pdev->dev.parent, 0, data, &ops);
	if (IS_ERR(data->thermal_zone_dev)) {
		dev_err(&pdev->dev, "Fail to create fp993x thermal zone\n");
		return PTR_ERR(data->thermal_zone_dev);
	}

	return 0;
}

static const struct platform_device_id fp9931_thermal_id_table[] = {
	{ "fp9931-thermal", },
	{ "fp9936-thermal", },
	{ },
};
MODULE_DEVICE_TABLE(platform, fp9931_thermal_id_table);

static struct platform_driver fp9931_thermal_driver = {
	.driver = {
		.name = "fp9931-thermal",
	},
	.probe = fp9931_thermal_probe,
	.id_table = fp9931_thermal_id_table,
};
module_platform_driver(fp9931_thermal_driver);

MODULE_DESCRIPTION("fp9931 thermal driver");
MODULE_LICENSE("GPL");
