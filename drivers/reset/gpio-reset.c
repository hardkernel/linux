/*
 * Copyright 2014 Parkeon
 * Martin Fuzzey <mfuzzey@parkeon.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 *
 * Driver allowing arbitrary hardware to be reset by GPIO signals.
 * The reset may be triggered in several ways:
 *	At boot time (if configured in DT)
 *	On userspace request via sysfs
 *	By a driver using the reset controller framework
 *
 * The first two methods are supplied for devices on discoverable busses
 * needing an external reset (eg some SDIO modules, USB hub chips)
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>


struct gpio_reset_priv;

struct gpio_reset_line {
	struct kobject kobj;
	struct gpio_reset_priv *priv;
	const char *name;
	int gpio;
	uint32_t asserted_value;
	uint32_t duration_ms;
#ifdef CONFIG_RESET_CONTROLLER
	struct reset_controller_dev rcdev;
#endif
};
#define kobj_to_gpio_reset_line(x) container_of(x, struct gpio_reset_line, kobj)


struct gpio_reset_priv {
	struct kref refcount;
	struct device *dev;
	int num_lines;
	struct gpio_reset_line lines[];
};

struct gpio_reset_attribute {
	struct attribute attr;
	ssize_t (*show)(struct gpio_reset_line *line,
				struct gpio_reset_attribute *attr, char *buf);
	ssize_t (*store)(struct gpio_reset_line *line,
				struct gpio_reset_attribute *attr,
				const char *buf, size_t count);
};
#define to_gpio_reset_attr(x) container_of(x, struct gpio_reset_attribute, attr)


static void gpio_reset_assert(struct gpio_reset_line *line)
{
	gpio_set_value(line->gpio, line->asserted_value);
}

static void gpio_reset_deassert(struct gpio_reset_line *line)
{
	gpio_set_value(line->gpio, !line->asserted_value);
}

static void gpio_reset_reset(struct gpio_reset_line *line)
{
	dev_info(line->priv->dev, "Resetting '%s' (%dms)",
				line->name, line->duration_ms);
	gpio_reset_assert(line);
	msleep(line->duration_ms);
	gpio_reset_deassert(line);
}


static void gpio_reset_free_priv(struct kref *ref)
{
	struct gpio_reset_priv *priv = container_of(ref,
					struct gpio_reset_priv, refcount);

	kfree(priv);
}

static void gpio_reset_release_kobj(struct kobject *kobj)
{
	struct gpio_reset_line *line;

	line = kobj_to_gpio_reset_line(kobj);

	kref_put(&line->priv->refcount, gpio_reset_free_priv);
}


#ifdef CONFIG_SYSFS

static ssize_t gpio_reset_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct gpio_reset_attribute *attribute;
	struct gpio_reset_line *line;

	attribute = to_gpio_reset_attr(attr);
	line = kobj_to_gpio_reset_line(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(line, attribute, buf);
}

static ssize_t gpio_reset_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct gpio_reset_attribute *attribute;
	struct gpio_reset_line *line;

	attribute = to_gpio_reset_attr(attr);
	line = kobj_to_gpio_reset_line(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(line, attribute, buf, len);
}

static ssize_t control_store(struct gpio_reset_line *line,
	struct gpio_reset_attribute *attr, const char *buf, size_t count)
{
	char action[10];
	char *eol;

	strncpy(action, buf, min(count, sizeof(action)));
	action[sizeof(action) - 1] = '\0';
	eol = strrchr(action, '\n');
	if (eol)
		*eol = '\0';

	if (!strcmp("reset", action))
		gpio_reset_reset(line);
	else if (!strcmp("assert", action))
		gpio_reset_assert(line);
	else if (!strcmp("deassert", action))
		gpio_reset_deassert(line);
	else
		return -EINVAL;

	return count;
}

static ssize_t duration_ms_show(struct gpio_reset_line *line,
	struct gpio_reset_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", line->duration_ms);
}

static ssize_t duration_ms_store(struct gpio_reset_line *line,
	struct gpio_reset_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = kstrtouint(buf, 0, &line->duration_ms);
	if (ret != 1)
		return -EINVAL;

	return count;
}

static struct gpio_reset_attribute control_attribute = __ATTR_WO(control);
static struct gpio_reset_attribute duration_attribute = __ATTR_RW(duration_ms);

static struct attribute *gpio_reset_attrs[] = {
	&control_attribute.attr,
	&duration_attribute.attr,
	NULL
};


static const struct sysfs_ops gpio_reset_sysfs_ops = {
	.show = gpio_reset_attr_show,
	.store = gpio_reset_attr_store,
};


static struct kobj_type gpio_reset_ktype = {
	.release = gpio_reset_release_kobj,
	.sysfs_ops = &gpio_reset_sysfs_ops,
	.default_attrs = gpio_reset_attrs,
};

static int gpio_reset_create_sysfs(struct gpio_reset_line *line)
{
	int ret;

	ret = kobject_init_and_add(&line->kobj, &gpio_reset_ktype,
		&line->priv->dev->kobj, "reset-%s", line->name);

	kref_get(&line->priv->refcount); /* kobject part of private structure */

	if (ret) {
		kobject_put(&line->kobj);
		return ret;
	}

	kobject_uevent(&line->kobj, KOBJ_ADD);

	return 0;
}

static void gpio_reset_destroy_sysfs(struct gpio_reset_line *line)
{
	kobject_put(&line->kobj);
}


#else

static int gpio_reset_create_sysfs(struct gpio_reset_line *line)
{
	return 0;
}

static void gpio_reset_destroy_sysfs(struct gpio_reset_line *line)
{
}

#endif  /* CONFIG_SYSFS */


#ifdef CONFIG_RESET_CONTROLLER
#define rcdev_to_gpio_reset_line(x) \
		container_of(x, struct gpio_reset_line, rcdev)

static int gpio_reset_controller_assert(struct reset_controller_dev *rcdev,
		unsigned long sw_reset_idx)
{
	struct gpio_reset_line *line =  rcdev_to_gpio_reset_line(rcdev);

	gpio_reset_assert(line);

	return 0;
}

static int gpio_reset_controller_deassert(struct reset_controller_dev *rcdev,
		unsigned long sw_reset_idx)
{
	struct gpio_reset_line *line =  rcdev_to_gpio_reset_line(rcdev);

	gpio_reset_deassert(line);

	return 0;
}

static int gpio_reset_controller_reset(struct reset_controller_dev *rcdev,
		unsigned long sw_reset_idx)
{
	struct gpio_reset_line *line =  rcdev_to_gpio_reset_line(rcdev);

	gpio_reset_reset(line);

	return 0;
}

static struct reset_control_ops gpio_reset_ops = {
	.assert = gpio_reset_controller_assert,
	.deassert = gpio_reset_controller_deassert,
	.reset = gpio_reset_controller_reset,
};


int gpio_reset_nooarg_xlate(struct reset_controller_dev *rcdev,
			  const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

/* We register one controller per line rather than a single
 * global controller so that drivers my directly reference the
 * phandle of the gpio_reset subnode rather than having to know
 * the index.
 */
static int gpio_reset_register_controller(
	struct device_node *np,
	struct gpio_reset_line *line)
{
	line->rcdev.of_node = np;
	line->rcdev.nr_resets = 1;
	line->rcdev.ops = &gpio_reset_ops;
	line->rcdev.of_xlate = gpio_reset_nooarg_xlate;

	return reset_controller_register(&line->rcdev);
}

static void gpio_reset_unregister_controller(struct gpio_reset_line *line)
{
	if (line->rcdev.nr_resets)
		reset_controller_unregister(&line->rcdev);
}

#else

static int gpio_reset_register_controller(
	struct device_node *np,
	struct gpio_reset_line *line)
{
	return 0;
}

static void gpio_reset_unregister_controller(struct gpio_reset_line *line)
{
}
#endif  /* CONFIG_RESET_CONTROLLER */


static int gpio_reset_init_line(
	struct device_node *np,
	struct gpio_reset_line *line)
{
	int ret;
	struct device *dev = line->priv->dev;

	line->name = np->name;

	line->gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(line->gpio)) {
		dev_warn(dev, "Invalid reset gpio for '%s'", np->name);
		return 0;
	}

	line->duration_ms = 1;
	of_property_read_u32(np, "asserted-state", &line->asserted_value);
	of_property_read_u32(np, "duration-ms", &line->duration_ms);

	ret = devm_gpio_request_one(dev, line->gpio,
		line->asserted_value ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
		line->name);
	if (ret)
		return ret;

	ret = gpio_reset_create_sysfs(line);
	if (ret)
		return ret;

	ret = gpio_reset_register_controller(np, line);
	if (ret)
		return ret;


	if (of_property_read_bool(np, "auto"))
		gpio_reset_reset(line);

	return 0;
}

static void gpio_reset_free_line(struct gpio_reset_line *line)
{
	gpio_reset_destroy_sysfs(line);
	gpio_reset_unregister_controller(line);
}

static int gpio_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct gpio_reset_priv *priv;
	struct gpio_reset_line *line;
	int num_lines;
	int ret;

	num_lines = of_get_available_child_count(np);
	if (!num_lines)
		return -ENODEV;

	for_each_available_child_of_node(np, child) {
		ret = of_get_gpio(child, 0);
		if (ret == -EPROBE_DEFER)
			return ret;
	}

	priv = kzalloc(sizeof(*priv) + sizeof(*line) * num_lines, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	kref_init(&priv->refcount);
	priv->dev = &pdev->dev;
	priv->num_lines = num_lines;

	line = priv->lines;
	for_each_available_child_of_node(np, child) {
		line->priv = priv;
		ret = gpio_reset_init_line(child, line);
		if (ret)
			goto rollback;
		line++;
	}

	platform_set_drvdata(pdev, priv);

	return 0;

rollback:
	while (line >= priv->lines)
		gpio_reset_free_line(line--);

	kref_put(&priv->refcount, gpio_reset_free_priv);

	return ret;
}

static int gpio_reset_remove(struct platform_device *pdev)
{
	struct gpio_reset_priv *priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < priv->num_lines; i++)
		gpio_reset_free_line(&priv->lines[i]);

	kref_put(&priv->refcount, gpio_reset_free_priv);

	return 0;
}

static const struct of_device_id gpio_reset_dt_ids[] = {
	{ .compatible = "linux,gpio-reset" },
	{}
};

static struct platform_driver gpio_reset_driver = {
	.probe		= gpio_reset_probe,
	.remove		= gpio_reset_remove,
	.driver		= {
		.name	= "gpio_reset",
		.owner	= THIS_MODULE,
		.of_match_table = gpio_reset_dt_ids,
	},
};

static int __init gpio_reset_init(void)
{
	return platform_driver_register(&gpio_reset_driver);
}
subsys_initcall(gpio_reset_init);

static void __exit gpio_reset_exit(void)
{
	platform_driver_unregister(&gpio_reset_driver);
}
module_exit(gpio_reset_exit);

MODULE_AUTHOR("Martin Fuzzey <mfuzzey@parkeon.com>");
MODULE_DESCRIPTION("GPIO reset controller");
MODULE_LICENSE("GPL");
