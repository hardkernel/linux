// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Rockchip Electronics Co., Ltd.
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

#include "fp9936.h"

enum fp9936_regulator_type {
	VPOS1 = 0,
	VNEG1,
	VPOS2,
	VNEG2,
	VPOS3,
	VNEG3,
	VGH,
	VGL,
	VCOM,
	VGHNM,
};

struct fp9936_voltage_table {
	int min_uV;
	int step;
	int max_selector;
	int reg;
	int mask;
	bool support_discharge;
};

static struct fp9936_voltage_table fp9936_voltage_tables[] = {
	/* VPOS1/VNEG1 +/-3.5V ~ +/-12V, 100mV step */
	{ 3500000, 100000, 85, FP9936_VPOS1_SETTING, VPOS_VNEG_SETTING, true },
	{ 3500000, 100000, 85, FP9936_VNEG1_SETTING, VPOS_VNEG_SETTING, true },
	/* VPOS2/VNEG2 +/-6V ~ +/-20V, 200mV step */
	{ 6000000, 200000, 70, FP9936_VPOS2_SETTING, VPOS_VNEG_SETTING, true },
	{ 6000000, 200000, 70, FP9936_VNEG2_SETTING, VPOS_VNEG_SETTING, true },
	/* VPOS3/VNEG3 +/-9V ~ +/-27.5V, 250mV step */
	{ 9000000, 250000, 74, FP9936_VPOS3_SETTING, VPOS_VNEG_SETTING, true },
	{ 9000000, 250000, 74, FP9936_VNEG3_SETTING, VPOS_VNEG_SETTING, true },
	/* VGH 10V ~ 40V, 1V step */
	{ 10000000, 1000000, 30, FP9936_VGH_SETTING, VGH_VGL_SETTING, true },
	/* VGL -15V ~ -40V, 1V step */
	{ 15000000, 1000000, 25, FP9936_VGL_SETTING, VGH_VGL_SETTING, true },
	/* VCOM 0V ~ -5V, 19.5mV step, 0V is meaningless */
	{ 19500, 19500, 255, FP9936_VCOM_SETTING, 0xFF, false },
	/* VGHNM 2V ~ 10V, 0.5V step */
	{ 2000000, 500000, 16, FP9936_VGHNM_SETTING, VGHNM_SETTING, false }
};

#define FP9936_MAX_ENABLE_GPIO_NUM 4

struct fp9936_data {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct gpio_descs *power_gpio;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *vdd_gpio;
	struct gpio_desc *pgood_gpio;
	int pgood_irq;
	/*
	 * When registering a new regulator, regulator core will try to
	 * call "set_suspend_disable" to check whether regulator device
	 * working well.
	 * Just skip this suspend to avoid reconfiguring pinctrl.
	 */
	bool initial_suspend;
	u8 regsbak[FP9936_MAX_REG_NUM];
};

static inline
int fp9936_write_reg(struct fp9936_data *data, unsigned int reg, unsigned int val)
{
	data->regsbak[reg] = val;
	return regmap_write(data->regmap, reg, val);
}

static inline
int fp9936_update_bits(struct fp9936_data *data, unsigned int reg, unsigned int mask,
		       unsigned int val)
{
	unsigned int tmp = data->regsbak[reg];

	data->regsbak[reg] = ((~mask) & tmp) | (val & mask);
	return regmap_update_bits(data->regmap, reg, mask, val);
}

static bool fp9936_is_setting_reg(int reg)
{
	switch (reg) {
	case FP9936_TMST_VALUE:
	case FP9936_FAULT_FLAG1:
	case FP9936_FAULT_FLAG2:
		return false;
	default:
		return true;
	};
}

static int fp9936_regsbak_init(struct fp9936_data *data)
{
	struct regmap *regmap = data->regmap;
	unsigned int val;
	int i, ret;

	for (i = 0; i < FP9936_MAX_REG_NUM; i++) {
		if (!fp9936_is_setting_reg(i))
			continue;
		ret = regmap_read(regmap, i, &val);
		if (ret) {
			dev_err(data->pdev->dev.parent, "Failed to read reg 0x%x\n", i);
			return ret;
		}
		data->regsbak[i] = val & 0xFF;
	}

	return 0;
}

static void fp9936_powerup_sequence(struct fp9936_data *data)
{
	gpiod_set_value_cansleep(data->vdd_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(data->enable_gpio, 1);
	enable_irq(data->pgood_irq);
}

static void fp9936_poweroff_sequence(struct fp9936_data *data)
{
	disable_irq(data->pgood_irq);
	gpiod_set_value_cansleep(data->enable_gpio, 0);
	msleep(50);
	gpiod_set_value_cansleep(data->vdd_gpio, 0);
}


static int fp9936_common_list_voltage(struct regulator_dev *rdev, unsigned int selector)
{
	int id = rdev->desc->id;
	struct fp9936_voltage_table *table = &fp9936_voltage_tables[id];
	int vol_uV;

	if (selector > table->max_selector)
		return -EINVAL;

	vol_uV = selector * table->step + table->min_uV;

	return vol_uV;
}

static int fp9936_common_get_voltage_sel(struct regulator_dev *rdev)
{
	int id = rdev->desc->id;
	struct fp9936_voltage_table *table = &fp9936_voltage_tables[id];
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, table->reg, &val);
	if (ret)
		return ret;

	return val & table->mask;
}

static int fp9936_common_set_voltage_sel(struct regulator_dev *rdev, unsigned int selector)
{
	int id = rdev->desc->id;
	struct fp9936_voltage_table *table = &fp9936_voltage_tables[id];
	struct fp9936_data *data = rdev->reg_data;
	unsigned int val;
	int ret;

	val = selector & table->mask;
	if (table->support_discharge)
		val |= DISCHARGE_ENABLE;

	ret = fp9936_write_reg(data, table->reg, val);
	if (ret)
		return ret;

	return 0;
}

static int fp9936_vghnm_enable(struct regulator_dev *rdev)
{
	struct fp9936_data *data = rdev->reg_data;
	int ret;

	ret = fp9936_update_bits(data, FP9936_PMIC_CONTROL, DISCHARGE_ENABLE, DISCHARGE_ENABLE);
	if (ret)
		return ret;

	return 0;
}

static int fp9936_vghnm_disable(struct regulator_dev *rdev)
{
	struct fp9936_data *data = rdev->reg_data;
	int ret;

	ret = fp9936_update_bits(data, FP9936_PMIC_CONTROL, DISCHARGE_ENABLE, 0);
	if (ret)
		return ret;

	return 0;
}

static int fp9936_vcom_enable(struct regulator_dev *rdev)
{
	struct fp9936_data *data = rdev->reg_data;

	fp9936_powerup_sequence(data);

	return 0;
}

static int fp9936_vcom_disable(struct regulator_dev *rdev)
{
	struct fp9936_data *data = rdev->reg_data;

	fp9936_poweroff_sequence(data);

	return 0;
}

static int fp9936_vcom_is_enabled(struct regulator_dev *rdev)
{
	struct fp9936_data *data = rdev->reg_data;

	return gpiod_get_value_cansleep(data->enable_gpio);
}

static int fp9936_set_suspend_disable(struct regulator_dev *rdev)
{
	DECLARE_BITMAP(values, FP9936_MAX_ENABLE_GPIO_NUM);
	struct fp9936_data *data = rdev->reg_data;
	int ret;

	bitmap_zero(values, FP9936_MAX_ENABLE_GPIO_NUM);

	/* Skip initial suspend */
	if (data->initial_suspend) {
		data->initial_suspend = false;
		return 0;
	}

	fp9936_poweroff_sequence(data);

	if (data->power_gpio) {
		ret = gpiod_set_array_value_cansleep(data->power_gpio->ndescs,
						     data->power_gpio->desc,
						     data->power_gpio->info, values);
		if (ret)
			return ret;
	}

	return 0;
}

static int fp9936_resume(struct regulator_dev *rdev)
{
	DECLARE_BITMAP(values, FP9936_MAX_ENABLE_GPIO_NUM);
	struct fp9936_data *data = rdev->reg_data;
	int i, ret;

	bitmap_fill(values, FP9936_MAX_ENABLE_GPIO_NUM);

	if (data->power_gpio) {
		ret = gpiod_set_array_value_cansleep(data->power_gpio->ndescs,
						     data->power_gpio->desc,
						     data->power_gpio->info, values);
		if (ret)
			return ret;
	}

	/* Waiting for i2c to become available after power up */
	usleep_range(1500, 2500);

	/* reg resume */
	for (i = 0; i < FP9936_MAX_REG_NUM; i++) {
		if (!fp9936_is_setting_reg(i))
			continue;
		ret = regmap_write(data->regmap, i, data->regsbak[i]);
		if (ret)
			dev_err(data->pdev->dev.parent, "Failed to write reg %d, ret:%d\n", i,
				ret);
	}

	fp9936_powerup_sequence(data);

	return 0;
}

static const struct regulator_ops fp9936_vcom_volt_ops = {
	.enable = fp9936_vcom_enable,
	.disable = fp9936_vcom_disable,
	.is_enabled = fp9936_vcom_is_enabled,
	.set_suspend_disable = fp9936_set_suspend_disable,
	.resume = fp9936_resume,
	.list_voltage = fp9936_common_list_voltage,
	.get_voltage_sel = fp9936_common_get_voltage_sel,
	.set_voltage_sel = fp9936_common_set_voltage_sel,
};

static const struct regulator_ops fp9936_vghnm_volt_ops = {
	.enable = fp9936_vghnm_enable,
	.disable = fp9936_vghnm_disable,
	.list_voltage = fp9936_common_list_voltage,
	.get_voltage_sel = fp9936_common_get_voltage_sel,
	.set_voltage_sel = fp9936_common_set_voltage_sel,
};

static const struct regulator_ops fp9936_common_volt_ops = {
	.list_voltage = fp9936_common_list_voltage,
	.get_voltage_sel = fp9936_common_get_voltage_sel,
	.set_voltage_sel = fp9936_common_set_voltage_sel,
};

static const struct regulator_desc vpos1_desc = {
	.name = "vpos1",
	.id = VPOS1,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vpos1"),
	.n_voltages = 86,
};

static const struct regulator_desc vneg1_desc = {
	.name = "vneg1",
	.id = VNEG1,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vneg1"),
	.n_voltages = 86,
};

static const struct regulator_desc vpos2_desc = {
	.name = "vpos2",
	.id = VPOS2,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vpos2"),
	.n_voltages = 71,
};

static const struct regulator_desc vneg2_desc = {
	.name = "vneg2",
	.id = VNEG2,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vneg2"),
	.n_voltages = 71,
};

static const struct regulator_desc vpos3_desc = {
	.name = "vpos3",
	.id = VPOS3,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vpos3"),
	.n_voltages = 75,
};

static const struct regulator_desc vneg3_desc = {
	.name = "vneg3",
	.id = VNEG3,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vneg3"),
	.n_voltages = 75,
};

static const struct regulator_desc vgl_desc = {
	.name = "vgl",
	.id = VGL,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vgl"),
	.n_voltages = 26,
};

/* day mode vgh */
static const struct regulator_desc vgh_desc = {
	.name = "vgh",
	.id = VGH,
	.ops = &fp9936_common_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vgh"),
	.n_voltages = 31,
};

/* night mode vghnm */
static const struct regulator_desc vghnm_desc = {
	.name = "vghnm",
	.id = VGHNM,
	.ops = &fp9936_vghnm_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vghnm"),
	.n_voltages = 17,
};

static const struct regulator_desc vcom_desc = {
	.name = "vcom",
	.id = VCOM,
	.ops = &fp9936_vcom_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vcom"),
	.n_voltages = 256,
};

static const struct regulator_desc *fp9936_desc_list[] = {
	&vpos1_desc, &vneg1_desc, &vpos2_desc, &vneg2_desc,
	&vpos3_desc, &vneg3_desc, &vgl_desc, &vgh_desc,
	&vghnm_desc, &vcom_desc
};

static const char *fp9936_channel_name[][4] = {
	{"VN1", "VN2", "VN3", "VGL"},
	{"VP1", "VP2", "VP3", "VGH"}
};

static irqreturn_t fp9936_power_good_irq_handler(int irq, void *dev_id)
{
	struct fp9936_data *data = dev_id;
	struct regmap *regmap = data->regmap;
	struct platform_device *pdev = data->pdev;
	u32 val;
	int ret;

	ret = regmap_read(regmap, FP9936_FAULT_FLAG1, &val);
	if (ret) {
		dev_err(pdev->dev.parent, "fp9936 failed to read fault flag\n");
		goto out;
	}

	dev_err(pdev->dev.parent, "fp9936 power fault flag:0x%x, %s at %s\n", val,
		val & FAULT_SCP_UVP ? "SCP" : "UVP",
		fp9936_channel_name[!!(val & FAULT_PORN)][val & FAULT_CHANNNEL]);

out:
	return IRQ_HANDLED;
}

static int fp9936_regulator_probe(struct platform_device *pdev)
{
	struct regmap *regmap = dev_get_regmap(pdev->dev.parent, NULL);
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct fp9936_data *data;
	int ret, i;

	if (!regmap)
		return -EPROBE_DEFER;

	data = devm_kzalloc(&pdev->dev, sizeof(struct fp9936_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->pdev = pdev;
	data->regmap = regmap;
	data->initial_suspend = true;

	data->power_gpio = devm_gpiod_get_array_optional(pdev->dev.parent, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(data->power_gpio)) {
		dev_err(pdev->dev.parent, "Failed to get fp9936 power gpio %ld\n",
			PTR_ERR(data->power_gpio));
		return PTR_ERR(data->power_gpio);
	}

	data->enable_gpio = devm_gpiod_get(pdev->dev.parent, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(data->enable_gpio)) {
		dev_err(pdev->dev.parent, "Failed to get fp9936 enable gpio %ld\n",
			PTR_ERR(data->enable_gpio));
		return PTR_ERR(data->enable_gpio);
	}

	data->vdd_gpio = devm_gpiod_get_optional(pdev->dev.parent, "vdd", GPIOD_OUT_LOW);
	if (IS_ERR(data->vdd_gpio)) {
		dev_err(pdev->dev.parent, "Failed to get fp9936 vdd gpio %ld\n",
			PTR_ERR(data->vdd_gpio));
		return PTR_ERR(data->vdd_gpio);
	}

	data->pgood_irq = -1;
	data->pgood_gpio = devm_gpiod_get(pdev->dev.parent, "pgood", GPIOD_IN);
	if (IS_ERR(data->pgood_gpio)) {
		dev_warn(pdev->dev.parent, "Failed to get fp9936 pgood gpio %ld\n",
			 PTR_ERR(data->pgood_gpio));
	} else {
		data->pgood_irq = gpiod_to_irq(data->pgood_gpio);
		if (data->pgood_irq < 0) {
			dev_err(pdev->dev.parent, "Failed to get fp9936 power good int irq\n");
		} else {
			ret = devm_request_threaded_irq(pdev->dev.parent, data->pgood_irq, NULL,
							fp9936_power_good_irq_handler,
							IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT,
							"fp9936", data);
			if (ret)
				dev_err(pdev->dev.parent, "Failed to request fp9936 irq\n");
			disable_irq(data->pgood_irq);
		}
	}

	/* Waiting for i2c to become available after power up */
	usleep_range(1500, 2500);

	ret = fp9936_regsbak_init(data);
	if (ret)
		goto fail;

	/* Enable gate control */
	fp9936_update_bits(data, FP9936_GD_CONTROL, ENABLE_GD, ENABLE_GD);

	platform_set_drvdata(pdev, data);

	config.dev = &pdev->dev;
	config.dev->of_node = pdev->dev.parent->of_node;
	config.regmap = regmap;
	config.driver_data = data;

	for (i = 0; i < ARRAY_SIZE(fp9936_desc_list); i++) {
		rdev = devm_regulator_register(&pdev->dev, fp9936_desc_list[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(pdev->dev.parent, "Failed to register regulator %s\n",
				fp9936_desc_list[i]->name);
			return PTR_ERR(rdev);
		}
	}

	return 0;

fail:
	dev_err(pdev->dev.parent, "Failed to initialize regulator: %d\n", ret);
	return ret;
}

static const struct platform_device_id fp9936_regulator_id_table[] = {
	{ "fp9936-regulator", },
	{ }
};
MODULE_DEVICE_TABLE(platform, fp9936_regulator_id_table);

static struct platform_driver fp9936_regulator_driver = {
	.driver = {
		.name = "fp9936-regulator",
	},
	.probe = fp9936_regulator_probe,
	.id_table = fp9936_regulator_id_table,
};
module_platform_driver(fp9936_regulator_driver);

MODULE_DESCRIPTION("FP9936 voltage regulator driver");
MODULE_LICENSE("GPL");
