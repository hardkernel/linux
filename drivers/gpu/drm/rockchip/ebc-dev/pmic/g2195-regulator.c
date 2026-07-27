// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2026 Rockchip Electronics Co., Ltd.
 */
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "g2195.h"

#define G2195_MAX_ENABLE_GPIO_NUM 4

enum g2195_regulator_id {
	G2195_VPOS1 = 0,
	G2195_VNEG1,
	G2195_VPOS2,
	G2195_VNEG2,
	G2195_VPOS3,
	G2195_VNEG3,
	G2195_DCVCOM,
	G2195_VCOMH,
	G2195_VCOML,
	G2195_VGH1,
	G2195_VGH2,
	G2195_NUM_REGULATORS,
};

struct g2195_voltage_table {
	int min_uV;
	int step_uV;
	unsigned int hw_selector_base;
	unsigned int n_voltages;
	unsigned int reg;
	u16 vsel_mask;
};

struct g2195_data {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_descs *power_gpio;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *pgood_gpio;
	int pgood_irq;
	bool initial_suspend;
};

/*
 * regulator selector:  [0, n_voltages)
 * hardware selector: [selector_base, selector_base + n_voltages)
 */
static const struct g2195_voltage_table g2195_voltage_tables[] = {
	[G2195_VPOS1] = {
		.min_uV = 1995200,
		.step_uV = 17600,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VPOS1,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_VNEG1] = {
		.min_uV = 1995200,
		.step_uV = 17600,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VNEG1,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_VPOS2] = {
		.min_uV = 1995200,
		.step_uV = 17600,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VPOS2,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_VNEG2] = {
		.min_uV = 1995200,
		.step_uV = 17600,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VNEG2,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_VPOS3] = {
		.min_uV = 1969800,
		.step_uV = 27400,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VPOS3,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_VNEG3] = {
		.min_uV = 1969800,
		.step_uV = 27400,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VNEG3,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_DCVCOM] = {
		.min_uV = 100000,
		.step_uV = 9800,
		/* It is recommended to set DCVCOM between -0.1V and -5V. */
		.hw_selector_base = 11,
		.n_voltages = 501,
		.reg = G2195_REG_DCVCOM,
		.vsel_mask = G2195_VSEL_9BIT_MASK,
	},
	[G2195_VCOMH] = {
		.min_uV = 4056800,
		.step_uV = 31200,
		.hw_selector_base = 0,
		.n_voltages = 512,
		.reg = G2195_REG_VCOMH,
		.vsel_mask = G2195_VSEL_9BIT_MASK,
	},
	[G2195_VCOML] = {
		.min_uV = 9056800,
		.step_uV = 31200,
		.hw_selector_base = 0,
		.n_voltages = 512,
		.reg = G2195_REG_VCOML,
		.vsel_mask = G2195_VSEL_9BIT_MASK,
	},
	[G2195_VGH1] = {
		.min_uV = 12046700,
		.step_uV = 37100,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VGH1,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
	[G2195_VGH2] = {
		.min_uV = 12046700,
		.step_uV = 37100,
		.hw_selector_base = 0,
		.n_voltages = 1024,
		.reg = G2195_REG_VGH2,
		.vsel_mask = G2195_VSEL_10BIT_MASK,
	},
};

static int g2195_voltage_get(struct g2195_data *data,
			     const struct g2195_voltage_table *table,
			     unsigned int *selector)
{
	unsigned int val;
	int ret;

	ret = regmap_read(data->regmap, table->reg, &val);
	if (ret)
		return ret;

	*selector = val & table->vsel_mask;

	return 0;
}

static int g2195_voltage_set(struct g2195_data *data,
			     const struct g2195_voltage_table *table,
			     unsigned int selector)
{
	return regmap_write_bits(data->regmap, table->reg, table->vsel_mask, selector);
}

static int g2195_list_voltage(struct regulator_dev *rdev, unsigned int selector)
{
	const struct g2195_voltage_table *table;

	table = &g2195_voltage_tables[rdev->desc->id];
	if (selector >= table->n_voltages)
		return -EINVAL;

	return table->min_uV + selector * table->step_uV;
}

static int g2195_get_voltage_sel(struct regulator_dev *rdev)
{
	const struct g2195_voltage_table *table;
	struct g2195_data *data = rdev->reg_data;
	unsigned int hw_selector;
	int ret;

	table = &g2195_voltage_tables[rdev->desc->id];

	ret = g2195_voltage_get(data, table, &hw_selector);
	if (ret)
		return ret;

	if (hw_selector < table->hw_selector_base ||
	    hw_selector >= table->hw_selector_base + table->n_voltages)
		return -EINVAL;

	return hw_selector - table->hw_selector_base;
}

static int g2195_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	const struct g2195_voltage_table *table;
	struct g2195_data *data = rdev->reg_data;
	unsigned int hw_selector;
	int ret;

	table = &g2195_voltage_tables[rdev->desc->id];
	if (selector >= table->n_voltages)
		return -EINVAL;

	hw_selector = selector + table->hw_selector_base;

	ret = g2195_voltage_set(data, table, hw_selector);
	return ret;
}

static int g2195_vcom_is_enabled(struct regulator_dev *rdev)
{
	struct g2195_data *data = rdev->reg_data;

	return gpiod_get_value_cansleep(data->enable_gpio);
}

static int g2195_vcom_enable(struct regulator_dev *rdev)
{
	struct g2195_data *data = rdev->reg_data;

	if (data->enable_gpio)
		gpiod_set_value_cansleep(data->enable_gpio, 1);
	if (data->pgood_irq >= 0)
		enable_irq(data->pgood_irq);

	return 0;
}

static int g2195_vcom_disable(struct regulator_dev *rdev)
{
	struct g2195_data *data = rdev->reg_data;

	if (data->pgood_irq >= 0)
		disable_irq(data->pgood_irq);

	if (data->enable_gpio)
		gpiod_set_value_cansleep(data->enable_gpio, 0);

	return 0;
}

static int g2195_set_suspend_disable(struct regulator_dev *rdev)
{
	DECLARE_BITMAP(values, G2195_MAX_ENABLE_GPIO_NUM);
	struct g2195_data *data = rdev->reg_data;
	int ret;

	bitmap_zero(values, G2195_MAX_ENABLE_GPIO_NUM);

	if (data->initial_suspend) {
		data->initial_suspend = false;
		return 0;
	}

	g2195_vcom_disable(rdev);
	if (data->power_gpio) {
		ret = gpiod_set_array_value_cansleep(data->power_gpio->ndescs,
						     data->power_gpio->desc,
						     data->power_gpio->info, values);
		if (ret)
			return ret;
	}

	regcache_cache_only(data->regmap, true);
	regcache_mark_dirty(data->regmap);

	return 0;
}

static int g2195_resume(struct regulator_dev *rdev)
{
	DECLARE_BITMAP(values, G2195_MAX_ENABLE_GPIO_NUM);
	struct g2195_data *data = rdev->reg_data;
	int ret;

	bitmap_fill(values, G2195_MAX_ENABLE_GPIO_NUM);

	if (data->power_gpio) {
		ret = gpiod_set_array_value_cansleep(data->power_gpio->ndescs,
						     data->power_gpio->desc,
						     data->power_gpio->info, values);
		if (ret)
			return ret;
	}

	/* Wait for the PMIC register interface to become available again. */
	usleep_range(3000, 4000);

	regcache_cache_only(data->regmap, false);
	ret = regcache_sync(data->regmap);
	if (ret)
		return ret;

	g2195_vcom_enable(rdev);

	return 0;
}

static const struct regulator_ops g2195_voltage_ops = {
	.list_voltage = g2195_list_voltage,
	.get_voltage_sel = g2195_get_voltage_sel,
	.set_voltage_sel = g2195_set_voltage_sel,
};

static const struct regulator_ops g2195_vcom_ops = {
	.enable = g2195_vcom_enable,
	.disable = g2195_vcom_disable,
	.is_enabled = g2195_vcom_is_enabled,
	.set_suspend_disable = g2195_set_suspend_disable,
	.resume = g2195_resume,
	.list_voltage = g2195_list_voltage,
	.get_voltage_sel = g2195_get_voltage_sel,
	.set_voltage_sel = g2195_set_voltage_sel,
};

#define G2195_DESC(_name, _id, _ops, _n_voltages)		\
	{							\
		.name = _name,					\
		.id = _id,					\
		.ops = _ops,					\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
		.regulators_node = of_match_ptr("regulators"),	\
		.of_match = of_match_ptr(_name),		\
		.n_voltages = _n_voltages,			\
	}

static const struct regulator_desc g2195_desc_list[] = {
	G2195_DESC("vpos1", G2195_VPOS1, &g2195_voltage_ops, 1024),
	G2195_DESC("vneg1", G2195_VNEG1, &g2195_voltage_ops, 1024),
	G2195_DESC("vpos2", G2195_VPOS2, &g2195_voltage_ops, 1024),
	G2195_DESC("vneg2", G2195_VNEG2, &g2195_voltage_ops, 1024),
	G2195_DESC("vpos3", G2195_VPOS3, &g2195_voltage_ops, 1024),
	G2195_DESC("vneg3", G2195_VNEG3, &g2195_voltage_ops, 1024),
	G2195_DESC("vcomh", G2195_VCOMH, &g2195_voltage_ops, 512),
	G2195_DESC("vcoml", G2195_VCOML, &g2195_voltage_ops, 512),
	G2195_DESC("vgh1", G2195_VGH1, &g2195_voltage_ops, 1024),
	G2195_DESC("vgh2", G2195_VGH2, &g2195_voltage_ops, 1024),

	G2195_DESC("vcom", G2195_DCVCOM, &g2195_vcom_ops, 501),
};

static irqreturn_t g2195_pgood_irq_handler(int irq, void *dev_id)
{
	struct g2195_data *data = dev_id;
	unsigned int val;
	int ret;

	ret = regmap_read(data->regmap, G2195_REG_FAULT_FLAGS, &val);
	if (ret)
		goto out;

	dev_err(data->dev, "g2195 fault flag1=0x%02x flag2=0x%02x\n",
		(val >> 8) & 0xff, val & 0xff);

out:
	return IRQ_HANDLED;
}

static bool g2195_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case G2195_REG_VPOS1:
	case G2195_REG_VNEG1:
	case G2195_REG_VPOS2:
	case G2195_REG_VNEG2:
	case G2195_REG_VPOS3:
	case G2195_REG_VNEG3:
	case G2195_REG_DCVCOM:
	case G2195_REG_VCOMH:
	case G2195_REG_VCOML:
	case G2195_REG_VGH1:
	case G2195_REG_VGH2:
		return false;
	}

	return true;
}

const struct regmap_config regmap_config_g2195 = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = g2195_is_volatile_reg,
	.max_register = G2195_REG_FAULT_FLAGS,
};

static int g2195_regulator_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct g2195_data *data;
	int ret;
	int i;

	if (!regmap)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = pdev->dev.parent;
	data->regmap = regmap;
	data->pgood_irq = -1;
	data->initial_suspend = true;

	data->power_gpio = devm_gpiod_get_array_optional(pdev->dev.parent, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(data->power_gpio)) {
		dev_err(pdev->dev.parent, "Failed to get g2195 power gpio %ld\n",
			PTR_ERR(data->power_gpio));
		return PTR_ERR(data->power_gpio);
	}

	data->enable_gpio = devm_gpiod_get_optional(pdev->dev.parent, "enable",
						    GPIOD_OUT_LOW);
	if (IS_ERR(data->enable_gpio)) {
		dev_err(data->dev, "Failed to get g2195 enable gpio %ld\n",
			PTR_ERR(data->enable_gpio));
		return PTR_ERR(data->enable_gpio);
	}

	data->pgood_gpio = devm_gpiod_get_optional(pdev->dev.parent, "pgood", GPIOD_IN);
	if (IS_ERR(data->pgood_gpio)) {
		dev_err(data->dev, "Failed to get g2195 pgood gpio %ld\n",
			PTR_ERR(data->pgood_gpio));
		return PTR_ERR(data->pgood_gpio);
	}

	if (data->pgood_gpio) {
		data->pgood_irq = gpiod_to_irq(data->pgood_gpio);
		if (data->pgood_irq < 0) {
			dev_err(data->dev, "Failed to get g2195 pgood irq\n");
			return data->pgood_irq;
		}

		ret = devm_request_threaded_irq(data->dev, data->pgood_irq, NULL,
						g2195_pgood_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"g2195", data);
		if (ret) {
			dev_err(data->dev, "Failed to request g2195 irq\n");
			return ret;
		}

		disable_irq(data->pgood_irq);
	}

	/* Wait for the PMIC register interface to become available again. */
	usleep_range(3000, 4000);

	platform_set_drvdata(pdev, data);

	config.dev = &pdev->dev;
	config.dev->of_node = pdev->dev.parent->of_node;
	config.regmap = regmap;
	config.driver_data = data;

	for (i = 0; i < ARRAY_SIZE(g2195_desc_list); i++) {
		rdev = devm_regulator_register(&pdev->dev, &g2195_desc_list[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(data->dev, "failed to register regulator %s\n",
				g2195_desc_list[i].name);
			return PTR_ERR(rdev);
		}
	}

	return 0;
}

static const struct platform_device_id g2195_regulator_id_table[] = {
	{ "g2195-regulator", },
	{ }
};
MODULE_DEVICE_TABLE(platform, g2195_regulator_id_table);

static struct platform_driver g2195_regulator_driver = {
	.driver = {
		.name = "g2195-regulator",
	},
	.probe = g2195_regulator_probe,
	.id_table = g2195_regulator_id_table,
};
module_platform_driver(g2195_regulator_driver);

MODULE_DESCRIPTION("G2195 voltage regulator driver");
MODULE_LICENSE("GPL");
