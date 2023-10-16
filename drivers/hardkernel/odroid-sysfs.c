/*
 * ODROID sysfs support for extra feature enhancement
 *
 * Copyright (C) 2014, Hardkernel Co,.Ltd
 *   Author: Charles Park <charles.park@hardkernel.com>
 *   Author: Dongjin Kim <tobetter@gmail.com>
 *
 * This driver has been modified to support ODROID-N1.
 *   Modified by Joy Cho <joycho78@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <asm/setup.h>

bool touch_invert_x;
/*-------------------------------------------------------------------------*/
static int __init touch_invert_x_para_setup(char *s)
{
	touch_invert_x = false;
	if (!strncmp(s, "true", 4))
		touch_invert_x = true;
	else if (!strncmp(s, "false", 5))
		touch_invert_x = false;
	else {
		pr_err("%s - wrong touch_invert_x parameter", __func__);
		touch_invert_x = true;
	}

	return 0;
}
__setup("touch_invert_x=", touch_invert_x_para_setup);

bool get_touch_invert_x(void)
{
	return touch_invert_x;
}
EXPORT_SYMBOL(get_touch_invert_x);

bool touch_invert_y;
/*-------------------------------------------------------------------------*/
static int __init touch_invert_y_para_setup(char *s)
{
	touch_invert_y = false;
	if (!strncmp(s, "true", 4))
		touch_invert_y = true;
	else if (!strncmp(s, "false", 5))
		touch_invert_y = false;
	else {
		pr_err("%s - wrong touch_invert_y parameter", __func__);
		touch_invert_y = true;
	}

	return 0;
}
__setup("touch_invert_y=", touch_invert_y_para_setup);

bool get_touch_invert_y(void)
{
	return touch_invert_y;
}
EXPORT_SYMBOL(get_touch_invert_y);

bool get_disable_vu7(void)
{
	return touch_invert_y;
}
EXPORT_SYMBOL(get_disable_vu7);

static int prevent_sleep = 0;

static  int __init prevent_sleep_setup(char *s)
{
	if (!strcmp(s, "true") || !strcmp(s, "1"))
		prevent_sleep = 1;
	else
		prevent_sleep = 0;

	return 0;
}
__setup("prevent_sleep=", prevent_sleep_setup);

struct wakeup_source wake_src;

MODULE_AUTHOR("Hardkernel Co,.Ltd");
MODULE_DESCRIPTION("SYSFS driver for ODROID hardware");
MODULE_LICENSE("GPL");

static struct hrtimer input_timer;
static struct input_dev *input_dev;
static int keycode[] = { KEY_POWER, };
static int key_release_seconds;

static enum hrtimer_restart input_timer_function(struct hrtimer *timer)
{
	key_release_seconds = 0;
	input_report_key(input_dev, KEY_POWER, 0);
	input_sync(input_dev);

	return HRTIMER_NORESTART;
}

static int init_input_dev(void)
{
	int error = 0;

/***********************************************************************
 * virtual key init (Power Off Key)
 ***********************************************************************/
	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	input_dev->name = "vt-input";
	input_dev->phys = "vt-input/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x16B4;
	input_dev->id.product = 0x0701;
	input_dev->id.version = 0x0001;
	input_dev->keycode = keycode;
	input_dev->keycodesize = sizeof(keycode[0]);
	input_dev->keycodemax = ARRAY_SIZE(keycode);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_POWER & KEY_MAX, input_dev->keybit);

	error = input_register_device(input_dev);
	if (error) {
		input_free_device(input_dev);
		return error;
	}

	pr_info("%s input driver registered!!\n", "Virtual-Key");

	hrtimer_init(&input_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	input_timer.function = input_timer_function;

	return error;
}

static ssize_t odroid_sysfs_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return 0;
}

static CLASS_ATTR_RO(odroid_sysfs);

static struct attribute *odroid_sysfs_class_attrs[] = {
	&class_attr_odroid_sysfs.attr,
	NULL,
};
ATTRIBUTE_GROUPS(odroid_sysfs_class);

static struct class odroid_sysfs_class = {
	.name = "odroid",
	.owner = THIS_MODULE,
	.class_groups = odroid_sysfs_class_groups,
};

static int odroid_sysfs_probe(struct platform_device *pdev)
{
	int error = 0;
	if (prevent_sleep) {
		__pm_stay_awake(&wake_src);
	}

#ifdef CONFIG_USE_OF
	struct device_node *node;

	if (pdev->dev.of_node)
		node = pdev->dev.of_node;
#endif

	if (input_dev == NULL)
		error = init_input_dev();

	return error;
}

static int odroid_sysfs_remove(struct platform_device *pdev)
{
	if (prevent_sleep)
		__pm_relax(&wake_src);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int odroid_sysfs_suspend(struct platform_device *dev, pm_message_t state)
{
	pr_info("%s\n", __func__);

	return 0;
}

static int odroid_sysfs_resume(struct platform_device *dev)
{
	pr_info("%s\n", __func__);

	return  0;
}
#endif

static const struct of_device_id odroid_sysfs_dt[] = {
	{ .compatible = "odroid-sysfs", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, odroid_sysfs_dt);

static struct platform_driver odroid_sysfs_driver = {
	.driver = {
		.name = "odroid-sysfs",
		.owner = THIS_MODULE,
		.of_match_table = odroid_sysfs_dt,
	},
	.probe = odroid_sysfs_probe,
	.remove = odroid_sysfs_remove,
#ifdef CONFIG_PM_SLEEP
	.suspend = odroid_sysfs_suspend,
	.resume = odroid_sysfs_resume,
#endif
};

static int __init odroid_sysfs_init(void)
{
	int error = class_register(&odroid_sysfs_class);

	if (prevent_sleep) {
		wakeup_source_init(&wake_src, "odroid_sysfs");
	}

	if (error < 0)
		return error;

	return platform_driver_register(&odroid_sysfs_driver);
}

static void __exit odroid_sysfs_exit(void)
{
	platform_driver_unregister(&odroid_sysfs_driver);
	class_unregister(&odroid_sysfs_class);
}

module_init(odroid_sysfs_init);
module_exit(odroid_sysfs_exit);
