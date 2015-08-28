/*
 * Pluggable GPIO IR receiver
 *
 * Copyright (c) 2015 Dongjin Kim (tobetter@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <media/gpio-ir-recv.h>

static unsigned gpio_nr = -1;
module_param(gpio_nr, uint, 0);
MODULE_PARM_DESC(gpio_nr, "GPIO number to receive IR pulse");

static bool active_low = 0;
module_param(active_low, bool, 0);
MODULE_PARM_DESC(active_low,
		"IR pulse trigger level, (1=low active, 0=high active");

static struct platform_device *pdev;
static struct gpio_ir_recv_platform_data *pdata;

static int __init gpio_init(void)
{
	int rc = -ENOMEM;

	if (gpio_nr == -1) {
		pr_err("gpioplug-ir-recv: missing module parameter: 'gpio_nr'\n");
		return -EINVAL;
	}

	pdev = platform_device_alloc(GPIO_IR_DRIVER_NAME, -1);
	if (!pdev)
		return rc;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto err_free_platform_data;

	pdev->dev.platform_data = pdata;

	pdata->gpio_nr = gpio_nr;
	pdata->active_low = active_low;
	pdata->allowed_protos = 0;
	pdata->map_name = NULL;

	rc = platform_device_add(pdev);
	if (rc < 0)
		goto err_free_device;

	dev_info (&pdev->dev, "IR driver is initialized (gpio=%d, pulse level=%s)\n",
		  pdata->gpio_nr, pdata->active_low ? "low" : "high");

	return 0;

err_free_platform_data:
	kfree(pdata);

err_free_device:
	platform_device_put(pdev);

	return rc;
}

static void __exit gpio_exit(void)
{
	dev_info(&pdev->dev, "gpioplug-ir-recv: IR driver is removed\n");
	platform_device_unregister(pdev);
}

MODULE_DESCRIPTION("GPIO IR Receiver driver");
MODULE_LICENSE("GPL v2");

module_init(gpio_init);
module_exit(gpio_exit);
