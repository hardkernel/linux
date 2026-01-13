/*
 * linux/drivers/char/odroid-gpiomem.c
 *
 * GPIO memory device driver
 *
 * Creates a chardev /dev/gpiomem which will provide user access to
 * the Meson g12 GPIO registers when it is mmap()'d.
 * No longer need root for user GPIO access, but without relaxing permissions
 * on /dev/mem.
 *
 * Copyright (c) 2025 Hardkernel Co., Ltd.
 *
 * This driver is based on bcm2835-gpiomem.c in Raspberrypi's linux kernel 6.1:
 *
 *	raspberrypi-gpiomem.c
 *
 *	Provides MMIO access to discontiguous section of Device memory as a linear
 *	user mapping. Successor to bcm2835-gpiomem.c.
 *
 *	Copyright (c) 2023, Raspberry Pi Ltd.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/pagemap.h>
#include <linux/io.h>

#define DEVICE_NAME "rk35xx-gpiomem"
#define DRIVER_NAME "odroid-gpiomem"
#define DEVICE_MINOR 0

/*
 * Sensible max for a hypothetical "gpio" controller that splits pads,
 * IO controls, GPIO in/out/enable, and function selection into different
 * ranges. Most use only one or two.
 */
#define MAX_RANGES 12

struct io_windows {
	unsigned long phys_base;
	unsigned long len;
};

struct odroid_gpiomem_priv {
	dev_t devid;
	struct class *class;
	struct cdev odroid_gpiomem_cdev;
	struct device *dev;
	const char *name;
	unsigned int nr_wins;
	struct io_windows iowins[MAX_RANGES];
};

static int odroid_gpiomem_open(struct inode *inode, struct file *file)
{
	int dev = iminor(inode);
	int ret = 0;
	struct odroid_gpiomem_priv *priv;

	if (dev != DEVICE_MINOR)
		ret = -ENXIO;

	priv = container_of(inode->i_cdev, struct odroid_gpiomem_priv,
				odroid_gpiomem_cdev);
	if (!priv)
		return -EINVAL;
	file->private_data = priv;
	return ret;
}

static int odroid_gpiomem_release(struct inode *inode, struct file *file)
{
	int dev = iminor(inode);
	int ret = 0;

	if (dev != DEVICE_MINOR)
		ret = -ENXIO;

	return ret;
}

static const struct vm_operations_struct odroid_gpiomem_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int odroid_gpiomem_mmap(struct file *file, struct vm_area_struct *vma)
{
	int i;
	struct odroid_gpiomem_priv *priv;
	unsigned long req_phys_addr = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long req_len = vma->vm_end - vma->vm_start;
	unsigned long pfn;

	priv = file->private_data;

	/* Find the requested physical address in the allowed windows */
	for (i = 0; i < priv->nr_wins; i++)
		if (req_phys_addr >= priv->iowins[i].phys_base &&
		    (req_phys_addr + req_len) <= (priv->iowins[i].phys_base + priv->iowins[i].len))
			goto found;


	/* If not found, return error */
	dev_err(priv->dev, "mmap request for invalid address 0x%lx or length 0x%lx\n",
		req_phys_addr, req_len);
	return -EINVAL;

found:
	vma->vm_ops = &odroid_gpiomem_vm_ops;
	pfn = req_phys_addr >> PAGE_SHIFT;
	vma->vm_page_prot = phys_mem_access_prot(file, pfn, req_len, vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, pfn, req_len, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations odroid_gpiomem_fops = {
	.owner = THIS_MODULE,
	.open = odroid_gpiomem_open,
	.release = odroid_gpiomem_release,
	.mmap = odroid_gpiomem_mmap,
};

static const struct of_device_id odroid_gpiomem_of_match[];

static int odroid_gpiomem_probe(struct platform_device *pdev)
{
	int err, i;
	const struct of_device_id *id;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource *ioresource;
	struct odroid_gpiomem_priv *priv;

	/* Allocate buffers and instance data */

	priv = kzalloc(sizeof(struct odroid_gpiomem_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto failed_inst_alloc;
	}
	platform_set_drvdata(pdev, priv);

	priv->dev = dev;
	id = of_match_device(odroid_gpiomem_of_match, dev);
	if (!id)
		return -EINVAL;

	/*
	 * Device node naming - for legacy (bcm2835) DT bindings, the driver
	 * created the node based on a hardcoded name - for new bindings,
	 * take the node name from DT.
	 */
	if (id == &odroid_gpiomem_of_match[0]) {
		priv->name = "gpiomem";
	} else {
		err = of_property_read_string(node, "chardev-name", &priv->name);
		if (err)
			return -EINVAL;
	}

	/*
	 * Go find the register ranges associated with this instance
	 */
	for (i = 0; i < MAX_RANGES; i++) {
		ioresource = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!ioresource && i == 0) {
			dev_err(priv->dev, "failed to get IO resource - no ranges available\n");
			err = -ENOENT;
			goto failed_get_resource;
		}
		if (!ioresource)
			break;

		priv->iowins[i].phys_base = ioresource->start;
		priv->iowins[i].len = (ioresource->end + 1) - ioresource->start;
		dev_info(&pdev->dev, "window base 0x%08lx size 0x%08lx\n",
			 priv->iowins[i].phys_base, priv->iowins[i].len);
		priv->nr_wins++;
	}

	/* Create character device entries */

	err = alloc_chrdev_region(&priv->devid,
				  DEVICE_MINOR, 1, priv->name);
	if (err != 0) {
		dev_err(priv->dev, "unable to allocate device number");
		goto failed_alloc_chrdev;
	}
	cdev_init(&priv->odroid_gpiomem_cdev, &odroid_gpiomem_fops);
	priv->odroid_gpiomem_cdev.owner = THIS_MODULE;
	err = cdev_add(&priv->odroid_gpiomem_cdev, priv->devid, 1);
	if (err != 0) {
		dev_err(priv->dev, "unable to register device");
		goto failed_cdev_add;
	}

	/* Create sysfs entries */

	priv->class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(priv->class)) {
		err = PTR_ERR(priv->class);
		goto failed_class_create;
	}

	dev = device_create(priv->class, NULL, priv->devid, NULL, priv->name);
	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		goto failed_device_create;
	}

	dev_info(priv->dev, "initialised %u regions as /dev/%s\n",
		 priv->nr_wins, priv->name);

	return 0;

failed_device_create:
	class_destroy(priv->class);
failed_class_create:
	cdev_del(&priv->odroid_gpiomem_cdev);
failed_cdev_add:
	unregister_chrdev_region(priv->devid, 1);
failed_alloc_chrdev:
failed_get_resource:
	kfree(priv);
failed_inst_alloc:
	dev_err(&pdev->dev, "could not load odroid_gpiomem");
	return err;
}

static int odroid_gpiomem_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct odroid_gpiomem_priv *priv = platform_get_drvdata(pdev);

	device_destroy(priv->class, priv->devid);
	class_destroy(priv->class);
	cdev_del(&priv->odroid_gpiomem_cdev);
	unregister_chrdev_region(priv->devid, 1);
	kfree(priv);

	dev_info(dev, "%s driver removed - OK", priv->name);
	return 0;
}

static const struct of_device_id odroid_gpiomem_of_match[] = {
	{
		.compatible = "hardkernel,rk35xx-gpiomem",
	},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, odroid_gpiomem_of_match);

static struct platform_driver odroid_gpiomem_driver = {
	.probe = odroid_gpiomem_probe,
	.remove = odroid_gpiomem_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = odroid_gpiomem_of_match,
		   },
};

module_platform_driver(odroid_gpiomem_driver);

MODULE_ALIAS("platform:hardkernel-gpiomem");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Driver for accessing GPIOs from userspace");
MODULE_AUTHOR("Bob shin <bob.shin@hardkernel.com>");