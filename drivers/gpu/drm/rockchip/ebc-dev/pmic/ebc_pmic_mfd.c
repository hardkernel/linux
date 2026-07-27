// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Rockchip Electronics Co., Ltd.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "g2195.h"

struct pmic_mfd_data {
	struct device *dev;
	const struct regmap_config *regmap_config;
	const struct mfd_cell *mfd_cell;
	size_t mfd_cell_size;
};

static const struct regmap_config regmap_config_8r_8v = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int pmic_mfd_i2c_probe(struct i2c_client *i2c)
{
	struct pmic_mfd_data *data;
	const struct regmap_config *regmap_config;
	struct regmap *regmap;
	int ret;

	data = (struct pmic_mfd_data *)device_get_match_data(&i2c->dev);
	if (!data)
		return -ENODEV;

	data->dev = &i2c->dev;

	/* If no regmap_config is specified, use the default 8reg and 8val bits */
	if (!data->regmap_config)
		regmap_config = &regmap_config_8r_8v;
	else
		regmap_config = data->regmap_config;

	regmap = devm_regmap_init_i2c(i2c, regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* If no MFD cells are specified, register using the DT child nodes instead */
	if (!data->mfd_cell)
		return devm_of_platform_populate(&i2c->dev);

	ret = devm_mfd_add_devices(&i2c->dev, PLATFORM_DEVID_AUTO, data->mfd_cell,
				   data->mfd_cell_size, NULL, 0, NULL);
	if (ret) {
		dev_err(&i2c->dev, "Failed to add child devices\n");
		return ret;
	}

	dev_info(&i2c->dev, "EBC PMIC MFD driver probe success\n");

	return 0;
}

static const struct mfd_cell sy7636a_cells[] = {
	{ .name = "sy7636a-regulator", },
	{ .name = "sy7636a-thermal", },
};

static struct pmic_mfd_data silergy_sy7636a = {
	.mfd_cell = sy7636a_cells,
	.mfd_cell_size = ARRAY_SIZE(sy7636a_cells),
};

static const struct mfd_cell fp9931_cells[] = {
	{ .name = "fp9931-regulator", },
	{ .name = "fp9931-thermal", },
};

static struct pmic_mfd_data fitipower_fp9931 = {
	.mfd_cell = fp9931_cells,
	.mfd_cell_size = ARRAY_SIZE(fp9931_cells),
};

static const struct mfd_cell fp9936_cells[] = {
	{ .name = "fp9936-regulator", },
	{ .name = "fp9936-thermal", },
};

static struct pmic_mfd_data fitipower_fp9936 = {
	.mfd_cell = fp9936_cells,
	.mfd_cell_size = ARRAY_SIZE(fp9936_cells),
};

#ifdef CONFIG_EPD_G2195_PMIC
static const struct mfd_cell g2195_cells[] = {
	{ .name = "g2195-regulator", },
	{ .name = "g2195-thermal", },
};

static struct pmic_mfd_data gmt_g2195 = {
	.mfd_cell = g2195_cells,
	.mfd_cell_size = ARRAY_SIZE(g2195_cells),
	.regmap_config = &regmap_config_g2195,
};
#endif

static const struct of_device_id pmic_mfd_i2c_of_match[] = {
	{ .compatible = "silergy,sy7636a-pmic", .data = &silergy_sy7636a },
	{ .compatible = "fitipower,fp9931-pmic", .data = &fitipower_fp9931 },
	{ .compatible = "fitipower,fp9936-pmic", .data = &fitipower_fp9936 },
#ifdef CONFIG_EPD_G2195_PMIC
	{ .compatible = "gmt,g2195-pmic", .data = &gmt_g2195 },
#endif
	{}
};
MODULE_DEVICE_TABLE(of, pmic_mfd_i2c_of_match);

static struct i2c_driver ebook_pmic_mfd_i2c_driver = {
	.probe_new = pmic_mfd_i2c_probe,
	.driver = {
		.name = "rockchip-ebook-pmic-mfd-i2c",
		.of_match_table = pmic_mfd_i2c_of_match,
	},
};
module_i2c_driver(ebook_pmic_mfd_i2c_driver);


MODULE_DESCRIPTION("Rockchip EBOOK PMIC MFD I2C Driver");
MODULE_LICENSE("GPL");
