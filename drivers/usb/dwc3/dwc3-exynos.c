/**
 * dwc3-exynos.c - Samsung EXYNOS DWC3 Specific Glue layer
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/platform_data/dwc3-exynos.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/usb/otg.h>
#include <linux/usb/nop-usb-xceiv.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include "../phy/phy-fsm-usb.h"

struct dwc3_exynos_rsw {
	struct otg_fsm		*fsm;

	int			id_gpio;
	int			b_sess_gpio;
};

struct dwc3_exynos {
	struct platform_device	*usb2_phy;
	struct platform_device	*usb3_phy;
	struct device		*dev;

	struct clk		*clk;

	struct dwc3_exynos_rsw	rsw;
	struct regulator        *vdd33;
	struct regulator        *vdd10;
};

void dwc3_otg_run_sm(struct otg_fsm *fsm);

#ifdef CONFIG_OF
static const struct of_device_id exynos_dwc3_match[] = {
	{ .compatible = "samsung,exynos5250-dwusb3" },
	{ .compatible = "samsung,exynos5-dwusb3" },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_dwc3_match);
#endif

static int dwc3_exynos_get_id_state(struct dwc3_exynos_rsw *rsw)
{
	if (gpio_is_valid(rsw->id_gpio))
		return gpio_get_value(rsw->id_gpio);
	else
		/* B-device by default */
		return 1;
}

static int dwc3_exynos_get_b_sess_state(struct dwc3_exynos_rsw *rsw)
{
	if (gpio_is_valid(rsw->b_sess_gpio))
		return gpio_get_value(rsw->b_sess_gpio);
	else
		return 0;
}

static irqreturn_t dwc3_exynos_rsw_thread_interrupt(int irq, void *_rsw)
{
	struct dwc3_exynos_rsw	*rsw = (struct dwc3_exynos_rsw *)_rsw;
	struct dwc3_exynos	*exynos = container_of(rsw,
						struct dwc3_exynos, rsw);

	dev_vdbg(exynos->dev, "%s\n", __func__);

	dwc3_otg_run_sm(rsw->fsm);

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_exynos_id_interrupt(int irq, void *_rsw)
{
	struct dwc3_exynos_rsw	*rsw = (struct dwc3_exynos_rsw *)_rsw;
	struct dwc3_exynos	*exynos = container_of(rsw,
						struct dwc3_exynos, rsw);
	int			state;

	state = dwc3_exynos_get_id_state(rsw);

	dev_vdbg(exynos->dev, "IRQ: ID: %d\n", state);

	if (rsw->fsm->id != state) {
		rsw->fsm->id = state;
		return IRQ_WAKE_THREAD;
	}

	return IRQ_NONE;
}

static irqreturn_t dwc3_exynos_b_sess_interrupt(int irq, void *_rsw)
{
	struct dwc3_exynos_rsw	*rsw = (struct dwc3_exynos_rsw *)_rsw;
	struct dwc3_exynos	*exynos = container_of(rsw,
						struct dwc3_exynos, rsw);
	int			state;

	state = dwc3_exynos_get_b_sess_state(rsw);

	dev_vdbg(exynos->dev, "IRQ: B_Sess: %sactive\n", state ? "" : "in");

	if (rsw->fsm->b_sess_vld != state) {
		rsw->fsm->b_sess_vld = state;
		return IRQ_WAKE_THREAD;
	}

	return IRQ_NONE;
}

/**
 * dwc3_exynos_id_event - receive ID pin state change event.
 *
 * @state : New ID pin state.
 *
 * Context: may sleep.
 */
int dwc3_exynos_id_event(struct device *dev, int state)
{
	struct dwc3_exynos	*exynos;
	struct otg_fsm		*fsm;

	dev_dbg(dev, "EVENT: ID: %d\n", state);

	exynos = dev_get_drvdata(dev);
	if (!exynos)
		return -ENOENT;

	fsm = exynos->rsw.fsm;
	if (!fsm)
		return -ENOENT;

	if (fsm->id != state) {
		fsm->id = state;
		dwc3_otg_run_sm(fsm);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_id_event);

/**
 * dwc3_exynos_vbus_event - receive VBus change event.
 *
 * vbus_active : New VBus state, true if active, false otherwise.
 *
 * Context: may sleep.
 */
int dwc3_exynos_vbus_event(struct device *dev, bool vbus_active)
{
	struct dwc3_exynos	*exynos;
	struct otg_fsm		*fsm;

	dev_dbg(dev, "EVENT: VBUS: %sactive\n", vbus_active ? "" : "in");

	exynos = dev_get_drvdata(dev);
	if (!exynos)
		return -ENOENT;

	fsm = exynos->rsw.fsm;
	if (!fsm)
		return -ENOENT;

	if (fsm->b_sess_vld != vbus_active) {
		fsm->b_sess_vld = vbus_active;
		dwc3_otg_run_sm(fsm);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_vbus_event);

int dwc3_exynos_rsw_start(struct device *dev)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;
	unsigned long		irq_flags = IRQF_TRIGGER_RISING |
					    IRQF_TRIGGER_FALLING;
	int			irq;
	int			ret;

	dev_dbg(dev, "%s\n", __func__);

	rsw->fsm->id = dwc3_exynos_get_id_state(rsw);
	rsw->fsm->b_sess_vld = dwc3_exynos_get_b_sess_state(rsw);

	if (gpio_is_valid(rsw->id_gpio)) {
		irq = gpio_to_irq(rsw->id_gpio);
		ret = devm_request_threaded_irq(exynos->dev, irq,
					dwc3_exynos_id_interrupt,
					dwc3_exynos_rsw_thread_interrupt,
					irq_flags, "dwc3_id", rsw);
		if (ret) {
			dev_err(exynos->dev, "failed to request irq #%d --> %d\n",
					irq, ret);
			return ret;
		}
	}

	if (gpio_is_valid(rsw->b_sess_gpio)) {
		irq = gpio_to_irq(rsw->b_sess_gpio);
		ret = devm_request_threaded_irq(exynos->dev, irq,
					dwc3_exynos_b_sess_interrupt,
					dwc3_exynos_rsw_thread_interrupt,
					irq_flags, "dwc3_b_sess", rsw);
		if (ret) {
			dev_err(exynos->dev, "failed to request irq #%d --> %d\n",
					irq, ret);
			return ret;
		}
	}

	return 0;
}

void dwc3_exynos_rsw_stop(struct device *dev)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;
	int			irq;

	dev_dbg(dev, "%s\n", __func__);

	if (gpio_is_valid(rsw->id_gpio)) {
		irq = gpio_to_irq(rsw->id_gpio);
		devm_free_irq(exynos->dev, irq, rsw);
	}
	if (gpio_is_valid(rsw->b_sess_gpio)) {
		irq = gpio_to_irq(rsw->b_sess_gpio);
		devm_free_irq(exynos->dev, irq, rsw);
	}
}

int dwc3_exynos_rsw_setup(struct device *dev, struct otg_fsm *fsm)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;
	int			ret;

	dev_dbg(dev, "%s\n", __func__);

	if (gpio_is_valid(rsw->id_gpio)) {
		ret = devm_gpio_request(exynos->dev, rsw->id_gpio,
						"dwc3_id_gpio");
		if (ret) {
			dev_err(exynos->dev, "failed to request dwc3 id gpio");
			return ret;
		}
	}

	if (gpio_is_valid(rsw->b_sess_gpio)) {
		ret = devm_gpio_request_one(exynos->dev, rsw->b_sess_gpio,
						GPIOF_IN, "dwc3_b_sess_gpio");
		if (ret) {
			dev_err(exynos->dev, "failed to request dwc3 b_sess gpio");
			return ret;
		}
	}

	rsw->fsm = fsm;

	return 0;
}

void dwc3_exynos_rsw_exit(struct device *dev)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;

	dev_dbg(dev, "%s\n", __func__);

	rsw->fsm = NULL;
}

static struct dwc3_exynos *dwc3_exynos_match(struct device *dev)
{
	struct dwc3_exynos		*exynos = NULL;
	const struct of_device_id	*matches = NULL;

	if (!dev)
		return NULL;

#if IS_ENABLED(CONFIG_OF)
	matches = exynos_dwc3_match;
#endif
	if (of_match_device(matches, dev))
		exynos = dev_get_drvdata(dev);

	return exynos;
}

bool dwc3_exynos_rsw_available(struct device *dev)
{
	struct dwc3_exynos	*exynos;

	exynos = dwc3_exynos_match(dev);
	if (!exynos)
		return false;

	return true;
}

static void dwc3_exynos_rsw_init(struct dwc3_exynos *exynos)
{
	struct device		*dev = exynos->dev;
	struct dwc3_exynos_rsw	*rsw = &exynos->rsw;
	struct pinctrl		*pinctrl;

	if (!dev->of_node)
		return;

	/* ID gpio */
	rsw->id_gpio = of_get_named_gpio(dev->of_node,
					"samsung,id-gpio", 0);
	if (!gpio_is_valid(rsw->id_gpio))
		dev_info(dev, "id gpio is not available\n");

	/* B-Session gpio */
	rsw->b_sess_gpio = of_get_named_gpio(dev->of_node,
					"samsung,bsess-gpio", 0);
	if (!gpio_is_valid(rsw->b_sess_gpio))
		dev_info(dev, "b_sess gpio is not available\n");

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_info(exynos->dev, "failed to configure pins\n");
}

static int dwc3_exynos_register_phys(struct dwc3_exynos *exynos)
{
	struct nop_usb_xceiv_platform_data pdata;
	struct platform_device	*pdev;
	int			ret;

	memset(&pdata, 0x00, sizeof(pdata));

	pdev = platform_device_alloc("nop_usb_xceiv", PLATFORM_DEVID_AUTO);
	if (!pdev)
		return -ENOMEM;

	exynos->usb2_phy = pdev;
	pdata.type = USB_PHY_TYPE_USB2;

	ret = platform_device_add_data(exynos->usb2_phy, &pdata, sizeof(pdata));
	if (ret)
		goto err1;

	pdev = platform_device_alloc("nop_usb_xceiv", PLATFORM_DEVID_AUTO);
	if (!pdev) {
		ret = -ENOMEM;
		goto err1;
	}

	exynos->usb3_phy = pdev;
	pdata.type = USB_PHY_TYPE_USB3;

	ret = platform_device_add_data(exynos->usb3_phy, &pdata, sizeof(pdata));
	if (ret)
		goto err2;

	ret = platform_device_add(exynos->usb2_phy);
	if (ret)
		goto err2;

	ret = platform_device_add(exynos->usb3_phy);
	if (ret)
		goto err3;

	return 0;

err3:
	platform_device_del(exynos->usb2_phy);

err2:
	platform_device_put(exynos->usb3_phy);

err1:
	platform_device_put(exynos->usb2_phy);

	return ret;
}

static int dwc3_exynos_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static int dwc3_exynos_probe(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos;
	struct clk		*clk;
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;

	int			ret = -ENOMEM;

	exynos = devm_kzalloc(dev, sizeof(*exynos), GFP_KERNEL);
	if (!exynos) {
		dev_err(dev, "not enough memory\n");
		goto err1;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we move to full device tree support this will vanish off.
	 */
	if (!dev->dma_mask)
		dev->dma_mask = &dev->coherent_dma_mask;
	if (!dev->coherent_dma_mask)
		dev->coherent_dma_mask = DMA_BIT_MASK(32);

	platform_set_drvdata(pdev, exynos);

	ret = dwc3_exynos_register_phys(exynos);
	if (ret) {
		dev_err(dev, "couldn't register PHYs\n");
		goto err1;
	}

	clk = devm_clk_get(dev, "usbdrd30");
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(dev, "couldn't get clock\n");
		ret = -EINVAL;
		goto err1;
	}

	exynos->dev	= dev;
	exynos->clk	= clk;

	clk_prepare_enable(exynos->clk);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	dwc3_exynos_rsw_init(exynos);

	exynos->vdd33 = devm_regulator_get(dev, "vdd33");
	if (IS_ERR(exynos->vdd33)) {
		ret = PTR_ERR(exynos->vdd33);
		goto err2;
	}
	ret = regulator_enable(exynos->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		goto err2;
	}

	exynos->vdd10 = devm_regulator_get(dev, "vdd10");
	if (IS_ERR(exynos->vdd10)) {
		ret = PTR_ERR(exynos->vdd10);
		goto err3;
	}
	ret = regulator_enable(exynos->vdd10);
	if (ret) {
		dev_err(dev, "Failed to enable VDD10 supply\n");
		goto err3;
	}

	if (node) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to add dwc3 core\n");
			goto err4;
		}
	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto err4;
	}

	return 0;

err4:
	regulator_disable(exynos->vdd10);
err3:
	regulator_disable(exynos->vdd33);
err2:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(clk);
	pm_runtime_set_suspended(&pdev->dev);
err1:
	return ret;
}

static int dwc3_exynos_remove(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, dwc3_exynos_remove_child);
	platform_device_unregister(exynos->usb2_phy);
	platform_device_unregister(exynos->usb3_phy);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev)) {
		clk_disable(exynos->clk);
		pm_runtime_set_suspended(&pdev->dev);
	}
	clk_unprepare(exynos->clk);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int dwc3_exynos_runtime_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	regulator_disable(exynos->vdd33);
	regulator_disable(exynos->vdd10);

	clk_disable(exynos->clk);

	return 0;
}

static int dwc3_exynos_runtime_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	clk_enable(exynos->clk);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int dwc3_exynos_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (pm_runtime_suspended(dev))
		return 0;

	clk_disable(exynos->clk);

	regulator_disable(exynos->vdd33);
	regulator_disable(exynos->vdd10);

	return 0;
}

static int dwc3_exynos_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s\n", __func__);
	ret = regulator_enable(exynos->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		return ret;
	}
	ret = regulator_enable(exynos->vdd10);
	if (ret) {
		dev_err(dev, "Failed to enable VDD10 supply\n");
		return ret;
	}

	clk_enable(exynos->clk);

	/* runtime set active to reflect active state. */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}

static const struct dev_pm_ops dwc3_exynos_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_exynos_suspend, dwc3_exynos_resume)
	SET_RUNTIME_PM_OPS(dwc3_exynos_runtime_suspend,
			dwc3_exynos_runtime_resume, NULL)
};

#define DEV_PM_OPS	(&dwc3_exynos_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver dwc3_exynos_driver = {
	.probe		= dwc3_exynos_probe,
	.remove		= dwc3_exynos_remove,
	.driver		= {
		.name	= "exynos-dwc3",
		.of_match_table = of_match_ptr(exynos_dwc3_match),
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(dwc3_exynos_driver);

MODULE_ALIAS("platform:exynos-dwc3");
MODULE_AUTHOR("Anton Tikhomirov <av.tikhomirov@samsung.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 EXYNOS Glue Layer");
