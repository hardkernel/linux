/*
 * s2mps11.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/of_regulator.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mps11.h>

struct s2mps11_info {
	struct regulator_dev *rdev[S2MPS11_REGULATOR_MAX];
	unsigned int opmode[S2MPS11_REGULATOR_MAX];
};

/* Some LDOs supports [LPM/Normal]ON mode during suspend state */
static int s2m_set_mode(struct regulator_dev *rdev,
				     unsigned int mode)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret, id = rdev_get_id(rdev);

	switch (mode) {
	case SEC_OPMODE_OFF:			/* ON in Standby Mode */
		val = 0x0 << S2MPS11_ENABLE_SHIFT;
		break;
	case SEC_OPMODE_SUSPEND:			/* ON in Standby Mode */
		val = 0x1 << S2MPS11_ENABLE_SHIFT;
		break;
	case SEC_OPMODE_ON:			/* ON in Normal Mode */
		val = 0x3 << S2MPS11_ENABLE_SHIFT;
		break;
	default:
		pr_warn("%s: regulator_suspend_mode : 0x%x not supported\n",
			rdev->desc->name, mode);
		return -EINVAL;
	}

	ret = regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask, val);
	if (ret)
		return ret;

	s2mps11->opmode[id] = val;
	return 0;
}

static int s2m_enable(struct regulator_dev *rdev)
{
	struct s2mps11_info *s2mps11 = rdev_get_drvdata(rdev);

	return regmap_update_bits(rdev->regmap, rdev->desc->enable_reg,
				  rdev->desc->enable_mask,
				  s2mps11->opmode[rdev_get_id(rdev)]);
}

static int get_ramp_delay(int ramp_delay)
{
	unsigned char cnt = 0;

	ramp_delay /= 6;

	while (true) {
		ramp_delay = ramp_delay >> 1;
		if (ramp_delay == 0)
			break;
		cnt++;
	}
	return cnt;
}

static int s2m_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	int ramp_reg, ramp_shift, reg_id = rdev_get_id(rdev);
	int ramp_mask = 0x03;
	unsigned int ramp_value = 0;

	ramp_value = get_ramp_delay(ramp_delay / 1000);
	if (ramp_value > 4) {
		pr_warn("%s: ramp_delay: %d not supported\n",
			rdev->desc->name, ramp_delay);
	}

	switch (reg_id) {
	case S2MPS11_BUCK1:
	case S2MPS11_BUCK6:
		ramp_reg = S2MPS11_REG_RAMP_BUCK;
		ramp_shift = 4;
		break;
	case S2MPS11_BUCK2:
		ramp_reg = S2MPS11_REG_RAMP;
		ramp_shift = 6;
		break;
	case S2MPS11_BUCK3:
	case S2MPS11_BUCK4:
		ramp_reg = S2MPS11_REG_RAMP;
		ramp_shift = 4;
		break;
	case S2MPS11_BUCK7:
	case S2MPS11_BUCK8:
	case S2MPS11_BUCK10:
		ramp_reg = S2MPS11_REG_RAMP_BUCK;
		ramp_shift = 2;
		break;
	case S2MPS11_BUCK9:
		ramp_reg = S2MPS11_REG_RAMP_BUCK;
		ramp_shift = 0;
		break;
	case S2MPS11_BUCK5:
		ramp_reg = S2MPS11_REG_RAMP_BUCK;
		ramp_shift = 6;
		break;
	default:
		return -EINVAL;
	}

	return regmap_update_bits(rdev->regmap, ramp_reg,
				  ramp_mask << ramp_shift, ramp_value << ramp_shift);
}

static int s2m_set_voltage_time_sel(struct regulator_dev *rdev,
				   unsigned int old_selector,
				   unsigned int new_selector)
{
	unsigned int ramp_delay = 0;
	int old_volt, new_volt;

	if (rdev->constraints->ramp_delay)
		ramp_delay = rdev->constraints->ramp_delay;
	else if (rdev->desc->ramp_delay)
		ramp_delay = rdev->desc->ramp_delay;

	if (ramp_delay == 0) {
		pr_warn("%s: ramp_delay not set\n", rdev->desc->name);
		return -EINVAL;
	}

	/* sanity check */
	if (!rdev->desc->ops->list_voltage)
		return -EINVAL;

	old_volt = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_volt = rdev->desc->ops->list_voltage(rdev, new_selector);

	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_volt - old_volt, ramp_delay);

	return 0;
}


static struct regulator_ops s2mps11_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
	.set_mode		= s2m_set_mode,
};

static struct regulator_ops s2mps11_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
	.set_mode		= s2m_set_mode,
	.set_ramp_delay		= s2m_set_ramp_delay,
};

#define _BUCK(macro)	S2MPS11_BUCK##macro
#define _buck_ops(num)	s2mps11_buck_ops##num

#define _LDO(macro)	S2MPS11_LDO##macro
#define _REG(ctrl)	S2MPS11_REG##ctrl
#define _ldo_ops(num)	s2mps11_ldo_ops##num

#define BUCK_DESC(_name, _id, _ops, m, s, v, e)	{		\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPS11_BUCK_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPS11_BUCK_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPS11_ENABLE_MASK			\
}

#define LDO_DESC(_name, _id, _ops, m, s, v, e)	{		\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPS11_LDO_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPS11_LDO_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPS11_ENABLE_MASK			\
}

enum regulator_desc_type {
	S2MPS11_DESC_TYPE0 = 0,
};

static struct regulator_desc regulators[][S2MPS11_REGULATOR_MAX] = {
	[S2MPS11_DESC_TYPE0] = {
			/* name, id, ops, min_uv, uV_step, vsel_reg, enable_reg */
		LDO_DESC("LDO1", _LDO(1), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L1CTRL), _REG(_L1CTRL)),
		LDO_DESC("LDO2", _LDO(2), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L2CTRL), _REG(_L2CTRL)),
		LDO_DESC("LDO3", _LDO(3), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L3CTRL), _REG(_L3CTRL)),
		LDO_DESC("LDO4", _LDO(4), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L4CTRL), _REG(_L4CTRL)),
		LDO_DESC("LDO5", _LDO(5), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L5CTRL), _REG(_L5CTRL)),
		LDO_DESC("LDO6", _LDO(6), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L6CTRL), _REG(_L6CTRL)),
		LDO_DESC("LDO7", _LDO(7), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L7CTRL), _REG(_L7CTRL)),
		LDO_DESC("LDO8", _LDO(8), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L8CTRL), _REG(_L8CTRL)),
		LDO_DESC("LDO9", _LDO(9), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L9CTRL), _REG(_L9CTRL)),
		LDO_DESC("LDO10", _LDO(10), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L10CTRL), _REG(_L10CTRL)),
		LDO_DESC("LDO11", _LDO(11), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L11CTRL), _REG(_L11CTRL)),
		LDO_DESC("LDO12", _LDO(12), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L12CTRL), _REG(_L12CTRL)),
		LDO_DESC("LDO13", _LDO(13), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L13CTRL), _REG(_L13CTRL)),
		LDO_DESC("LDO14", _LDO(14), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L14CTRL), _REG(_L14CTRL)),
		LDO_DESC("LDO15", _LDO(15), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L15CTRL), _REG(_L15CTRL)),
		LDO_DESC("LDO16", _LDO(16), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L16CTRL), _REG(_L16CTRL)),
		LDO_DESC("LDO17", _LDO(17), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L17CTRL), _REG(_L17CTRL)),
		LDO_DESC("LDO18", _LDO(18), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L18CTRL), _REG(_L18CTRL)),
		LDO_DESC("LDO19", _LDO(19), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L19CTRL), _REG(_L19CTRL)),
		LDO_DESC("LDO20", _LDO(20), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L20CTRL), _REG(_L20CTRL)),
		LDO_DESC("LDO21", _LDO(21), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L21CTRL), _REG(_L21CTRL)),
		LDO_DESC("LDO22", _LDO(22), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L22CTRL), _REG(_L22CTRL)),
		LDO_DESC("LDO23", _LDO(23), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L23CTRL), _REG(_L23CTRL)),
		LDO_DESC("LDO24", _LDO(24), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L24CTRL), _REG(_L24CTRL)),
		LDO_DESC("LDO25", _LDO(25), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L25CTRL), _REG(_L25CTRL)),
		LDO_DESC("LDO26", _LDO(26), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L26CTRL), _REG(_L26CTRL)),
		LDO_DESC("LDO27", _LDO(27), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L27CTRL), _REG(_L27CTRL)),
		LDO_DESC("LDO28", _LDO(28), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L28CTRL), _REG(_L28CTRL)),
		LDO_DESC("LDO29", _LDO(29), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L29CTRL), _REG(_L29CTRL)),
		LDO_DESC("LDO30", _LDO(30), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L30CTRL), _REG(_L30CTRL)),
		LDO_DESC("LDO31", _LDO(31), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L31CTRL), _REG(_L31CTRL)),
		LDO_DESC("LDO32", _LDO(32), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L32CTRL), _REG(_L32CTRL)),
		LDO_DESC("LDO33", _LDO(33), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L33CTRL), _REG(_L33CTRL)),
		LDO_DESC("LDO34", _LDO(34), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L34CTRL), _REG(_L34CTRL)),
		LDO_DESC("LDO35", _LDO(35), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP2), _REG(_L35CTRL), _REG(_L35CTRL)),
		LDO_DESC("LDO36", _LDO(36), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L36CTRL), _REG(_L36CTRL)),
		LDO_DESC("LDO37", _LDO(37), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L37CTRL), _REG(_L37CTRL)),
		LDO_DESC("LDO38", _LDO(38), &_ldo_ops(), _LDO(_MIN1), _LDO(_STEP1), _REG(_L38CTRL), _REG(_L38CTRL)),
		BUCK_DESC("BUCK1", _BUCK(1), &_buck_ops(), _BUCK(_MIN1), _BUCK(_STEP1), _REG(_B1CTRL2), _REG(_B1CTRL1)),
		BUCK_DESC("BUCK2", _BUCK(2), &_buck_ops(), _BUCK(_MIN1), _BUCK(_STEP1), _REG(_B2CTRL2), _REG(_B2CTRL1)),
		BUCK_DESC("BUCK3", _BUCK(3), &_buck_ops(), _BUCK(_MIN1), _BUCK(_STEP1), _REG(_B3CTRL2), _REG(_B3CTRL1)),
		BUCK_DESC("BUCK4", _BUCK(4), &_buck_ops(), _BUCK(_MIN1), _BUCK(_STEP1), _REG(_B4CTRL2), _REG(_B4CTRL1)),
		BUCK_DESC("BUCK5", _BUCK(5), &_buck_ops(), _BUCK(_MIN1), _BUCK(_STEP1), _REG(_B5CTRL2), _REG(_B5CTRL1)),
		BUCK_DESC("BUCK6", _BUCK(6), &_buck_ops(), _BUCK(_MIN1), _BUCK(_STEP1), _REG(_B6CTRL2), _REG(_B6CTRL1)),
		BUCK_DESC("BUCK9", _BUCK(9), &_buck_ops(), _BUCK(_MIN3), _BUCK(_STEP3), _REG(_B9CTRL2), _REG(_B9CTRL1)),
		BUCK_DESC("BUCK10", _BUCK(10), &_buck_ops(), _BUCK(_MIN2), _BUCK(_STEP2), _REG(_B10CTRL2), _REG(_B10CTRL1)),
	},
};

#ifdef CONFIG_OF
static int s2mps11_pmic_dt_parse_pdata(struct sec_pmic_dev *iodev,
					struct sec_platform_data *pdata)
{
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct sec_regulator_data *rdata;
	unsigned int i;

	pmic_np = iodev->dev->of_node;
	if (!pmic_np) {
		dev_err(iodev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(iodev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in pmic */
	pdata->num_regulators = 0;
	for_each_child_of_node(regulators_np, reg_np) {
		pdata->num_regulators++;
	}

	rdata = devm_kzalloc(iodev->dev, sizeof(*rdata) *
				pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		dev_err(iodev->dev,
			"could not allocate memory for regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(regulators[S2MPS11_DESC_TYPE0]); i++)
			if (!of_node_cmp(reg_np->name,
					regulators[S2MPS11_DESC_TYPE0][i].name))
				break;

		if (i == ARRAY_SIZE(regulators[S2MPS11_DESC_TYPE0])) {
			dev_warn(iodev->dev,
			"don't know how to configure regulator %s\n",
			reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(
						iodev->dev, reg_np);
		rdata->reg_node = reg_np;
		rdata++;
	}

	return 0;
}
#else
static int s2mps11_pmic_dt_parse_pdata(struct sec_pmic_dev *iodev,
					struct sec_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */
//////////////////////////////////////
struct sec_pmic_dev *g_iodev = NULL;

int s2mps11_pmic_ethonoff(unsigned char status)
{
    unsigned int ldo15 = 0, ldo17 = 0;

    if(sec_reg_read(g_iodev, S2MPS11_REG_L15CTRL, &ldo15)) {
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL15 read Error!!\n", __func__);
        return  -1;
    }

    if(sec_reg_read(g_iodev, S2MPS11_REG_L17CTRL, &ldo17)) {
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL17 read Error!!\n", __func__);
        return  -1;
    }

    if(status) {
        // ETH VDD0 ON
        if(sec_reg_update(g_iodev, S2MPS11_REG_L15CTRL, 0x72, 0xFF)) {
            printk(KERN_EMERG "%s : S2MPS11 LDO CTRL15 Error!!\n", __func__);
            return  -1;
        }

        // ETH VDD1 ON
        if(sec_reg_update(g_iodev, S2MPS11_REG_L17CTRL, 0x72, 0xFF)) {
            printk(KERN_EMERG "%s : S2MPS11 LDO CTRL17 Error!!\n", __func__);
            return  -1;
        }
    } else {
        // ETH VDD0 OFF
        if(sec_reg_update(g_iodev, S2MPS11_REG_L15CTRL, 0x00, 0x3F))  {
            printk(KERN_EMERG "%s : S2MPS11 LDO CTRL15 Error!!\n", __func__);
            return  -1;
        }

        // ETH VDD1 OFF
        if(sec_reg_update(g_iodev, S2MPS11_REG_L17CTRL, 0x00, 0x3F)) {
            printk(KERN_EMERG "%s : S2MPS11 LDO CTRL17 Error!!\n", __func__);
            return  -1;
        }
    }
    return  0;
}

EXPORT_SYMBOL(s2mps11_pmic_ethonoff);
//////////////////////////////////////

static int s2mps11_pmic_probe(struct platform_device *pdev)
{
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct sec_platform_data *pdata = iodev->pdata;
	struct regulator_config config = { };
	struct s2mps11_info *s2mps11;
	int i, ret;

	g_iodev = dev_get_drvdata(pdev->dev.parent);

	ret = sec_reg_read(iodev, S2MPS11_REG_ID, &iodev->rev_num);
	if (ret < 0)
		return ret;

	if (iodev->dev->of_node) {
		ret = s2mps11_pmic_dt_parse_pdata(iodev, pdata);
		if (ret)
			return ret;
	}

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mps11 = devm_kzalloc(&pdev->dev, sizeof(struct s2mps11_info),
				GFP_KERNEL);
	if (!s2mps11)
		return -ENOMEM;

	platform_set_drvdata(pdev, s2mps11);

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;
		config.dev = &pdev->dev;
		config.regmap = iodev->regmap;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = s2mps11;
		config.of_node = pdata->regulators[i].reg_node;
		s2mps11->opmode[id] = regulators[S2MPS11_DESC_TYPE0][id].enable_mask;

		s2mps11->rdev[i] = regulator_register(
				&regulators[S2MPS11_DESC_TYPE0][id], &config);
		if (IS_ERR(s2mps11->rdev[i])) {
			ret = PTR_ERR(s2mps11->rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n",
				i);
			s2mps11->rdev[i] = NULL;
			goto err;
		}
	}

	return 0;
err:
	for (i = 0; i < S2MPS11_REGULATOR_MAX; i++)
		regulator_unregister(s2mps11->rdev[i]);

	return ret;
}

static int s2mps11_pmic_remove(struct platform_device *pdev)
{
	struct s2mps11_info *s2mps11 = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < S2MPS11_REGULATOR_MAX; i++)
		regulator_unregister(s2mps11->rdev[i]);

	return 0;
}

#if defined(CONFIG_MACH_ODROIDXU3)
static void s2mps11_pmic_shutdown(struct platform_device *pdev)
{
    struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
    unsigned int reg_ctrl1 = 0, retry = 0;

    for(retry = 0; retry < 5; retry++)  {
    	/* PWR_HOLD bit-clear when power off */
        if(sec_reg_update(iodev, S2MPS11_REG_CTRL1, 0x00, 0xff))    {
            printk(KERN_EMERG "%s : S2MPS11 REG CTRL1 Update error! retry %d", __func__, retry);
            continue;
        }

        if(sec_reg_read(iodev, S2MPS11_REG_CTRL1, &reg_ctrl1))  {
            printk(KERN_EMERG "%s : S2MPS11 REG CTRL1 read error! retry %d", __func__, retry);
            continue;
        }
        
        // PWR_HOLD not clear.
        if(reg_ctrl1 & 0x10)    {
            printk(KERN_EMERG "%s : S2MPS11 REG CTRL1 value error!, value = 0x%08X, retry %d", __func__, reg_ctrl1, retry);
            continue;
        }
        else    {
        	printk(KERN_EMERG "%s : PWR_HOLD Clear Bit. (retry %d)\n", __func__, retry);
            break;
        }
   }

    // MMC Power Control for MMC UHS Mode
    if(sec_reg_read(iodev, S2MPS11_REG_L19CTRL, &reg_ctrl1))
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL19 Error!!\n", __func__);

    // VDDQ_MMC2 OFF
    if(sec_reg_update(iodev, S2MPS11_REG_L13CTRL, 0x00, 0x3F))
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL13 Error!!\n", __func__);

    // VDD_SD_2V8 OFF
    if(sec_reg_update(iodev, S2MPS11_REG_L19CTRL, 0x00, 0x3F))
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL19 Error!!\n", __func__);

    mdelay(10);

    if(sec_reg_update(iodev, S2MPS11_REG_L19CTRL, reg_ctrl1, 0x3F))
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL19 Error!!\n", __func__);
    if(sec_reg_update(iodev, S2MPS11_REG_L13CTRL, 0xE8, 0xFF))
        printk(KERN_EMERG "%s : S2MPS11 LDO CTRL13 Error!!\n", __func__);
}
#endif

static const struct platform_device_id s2mps11_pmic_id[] = {
	{ "s2mps11-pmic", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mps11_pmic_id);

static struct platform_driver s2mps11_pmic_driver = {
	.driver = {
		.name = "s2mps11-pmic",
		.owner = THIS_MODULE,
	},
	.probe = s2mps11_pmic_probe,
	.remove = s2mps11_pmic_remove,
#if defined(CONFIG_MACH_ODROIDXU3)
	.shutdown	= s2mps11_pmic_shutdown,
#endif	
	.id_table = s2mps11_pmic_id,
};

static int __init s2mps11_pmic_init(void)
{
	return platform_driver_register(&s2mps11_pmic_driver);
}
subsys_initcall(s2mps11_pmic_init);

static void __exit s2mps11_pmic_exit(void)
{
	platform_driver_unregister(&s2mps11_pmic_driver);
}
module_exit(s2mps11_pmic_exit);

/* Module information */
MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG S2MPS11 Regulator Driver");
MODULE_LICENSE("GPL");
