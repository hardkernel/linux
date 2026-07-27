// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Rockchip Electronics Co., Ltd.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>

#include "g2195.h"

struct g2195_data {
	struct regmap *regmap;
	struct thermal_zone_device *thermal_zone_dev;
	struct regulator *regulator;
};

static int g2195_get_temp(struct thermal_zone_device *dev, int *res)
{
	struct g2195_data *data = dev->devdata;
	unsigned int reg_val;
	int ret = 0;

	ret = regulator_is_enabled(data->regulator);
	if (ret <= 0)
		return -EBUSY;

	ret = regmap_read(data->regmap, G2195_REG_TMST, &reg_val);
	if (!ret) {
		reg_val = (reg_val >> 8) & 0xFF;
		*res = (signed char)reg_val;
		*res *= 1000;
	}

	return ret;
}

static const struct thermal_zone_device_ops ops = {
	.get_temp = g2195_get_temp,
};

static int g2195_thermal_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct g2195_data *data;

	if (!regmap)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(struct g2195_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->regulator = devm_regulator_get(&pdev->dev, "vcom");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Unable to get vcom regulator, returned %ld\n",
			PTR_ERR(data->regulator));
		return PTR_ERR(data->regulator);
	}

	data->regmap = regmap;

	data->thermal_zone_dev = devm_thermal_of_zone_register(pdev->dev.parent, 0, data, &ops);
	if (IS_ERR(data->thermal_zone_dev)) {
		dev_err(&pdev->dev, "Fail to create thermal zone\n");
		return PTR_ERR(data->thermal_zone_dev);
	}

	return 0;
}

static const struct platform_device_id g2195_thermal_id_table[] = {
	{ "g2195-thermal", },
	{ },
};
MODULE_DEVICE_TABLE(platform, g2195_thermal_id_table);

static struct platform_driver g2195_thermal_driver = {
	.driver = {
		.name = "g2195-thermal",
	},
	.probe = g2195_thermal_probe,
	.id_table = g2195_thermal_id_table,
};
module_platform_driver(g2195_thermal_driver);

MODULE_DESCRIPTION("G2195 thermal driver");
MODULE_LICENSE("GPL");
