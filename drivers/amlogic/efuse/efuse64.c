/*
 * drivers/amlogic/efuse/efuse64.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <linux/amlogic/secmon.h>
#include "efuse.h"
#if defined(CONFIG_ARM64) || defined(CONFIG_ARM64_A32)
#include <linux/amlogic/efuse.h>
#endif

#ifdef CONFIG_AMLOGIC_CPU_INFO
#include <linux/amlogic/cpu_version.h>
#endif

#define EFUSE_MODULE_NAME   "efuse"
#define EFUSE_DRIVER_NAME	"efuse"
#define EFUSE_DEVICE_NAME   "efuse"
#define EFUSE_CLASS_NAME    "efuse"
#define EFUSE_IS_OPEN           (0x01)

struct efusekey_info *efusekey_infos;
int efusekeynum =  -1;

struct efuse_dev_t {
	struct cdev         cdev;
	unsigned int        flags;
};

static struct efuse_dev_t *efuse_devp;
/* static struct class *efuse_clsp; */
static dev_t efuse_devno;

void __iomem *sharemem_input_base;
void __iomem *sharemem_output_base;
unsigned int efuse_read_cmd;
unsigned int efuse_write_cmd;
unsigned int efuse_get_max_cmd;

#if !defined(CONFIG_ARCH_MESON64_ODROID_COMMON)
#define  DEFINE_EFUEKEY_SHOW_ATTR(keyname)	\
	static ssize_t  show_##keyname(struct class *cla, \
					  struct class_attribute *attr,	\
						char *buf)	\
	{	\
		ssize_t ret;	\
		\
		ret = efuse_user_attr_show(#keyname, buf); \
		return ret; \
	}
DEFINE_EFUEKEY_SHOW_ATTR(mac)
DEFINE_EFUEKEY_SHOW_ATTR(mac_bt)
DEFINE_EFUEKEY_SHOW_ATTR(mac_wifi)
DEFINE_EFUEKEY_SHOW_ATTR(usid)


#define  DEFINE_EFUEKEY_STORE_ATTR(keyname)	\
	static ssize_t  store_##keyname(struct class *cla, \
					  struct class_attribute *attr,	\
						const char *buf,	\
						size_t count)	\
	{	\
		ssize_t ret;	\
		\
		ret = efuse_user_attr_store(#keyname, buf, count); \
		return ret; \
	}
DEFINE_EFUEKEY_STORE_ATTR(mac)
DEFINE_EFUEKEY_STORE_ATTR(mac_bt)
DEFINE_EFUEKEY_STORE_ATTR(mac_wifi)
DEFINE_EFUEKEY_STORE_ATTR(usid)
#else
#define  DEFINE_EFUEKEY_SHOW_ATTR(keyname)	\
	static ssize_t  keyname##_show(struct class *cla, \
					  struct class_attribute *attr,	\
						char *buf)	\
	{	\
		ssize_t ret;	\
		\
		ret = efuse_user_attr_show(#keyname, buf); \
		return ret; \
	}
DEFINE_EFUEKEY_SHOW_ATTR(uuid)
#endif

int efuse_getinfo(char *item, struct efusekey_info *info)
{
	int i;
	int ret = -1;

	for (i = 0; i < efusekeynum; i++) {
		if (strcmp(efusekey_infos[i].keyname, item) == 0) {
			strcpy(info->keyname, efusekey_infos[i].keyname);
			info->offset = efusekey_infos[i].offset;
			info->size = efusekey_infos[i].size;
			ret = 0;
			break;
		}
	}
	if (ret < 0)
		pr_err("%s item not found.\n", item);
	return ret;
}

static int efuse_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct efuse_dev_t *devp;

	devp = container_of(inode->i_cdev, struct efuse_dev_t, cdev);
	file->private_data = devp;

	return ret;
}

static int efuse_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct efuse_dev_t *devp;
	unsigned long efuse_status = 0;

	devp = file->private_data;
	efuse_status &= ~EFUSE_IS_OPEN;
	return ret;
}

loff_t efuse_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch (whence) {
	case 0: /* SEEK_SET */
		newpos = off;
		break;

	case 1: /* SEEK_CUR */
		newpos = filp->f_pos + off;
		break;

	case 2: /* SEEK_END */
		newpos = EFUSE_BYTES + off;
		break;

	default: /* can't happen */
		return -EINVAL;
	}

	if (newpos < 0)
		return -EINVAL;
	filp->f_pos = newpos;
		return newpos;
}

#if defined(CONFIG_ARCH_MESON64_ODROID_COMMON)
static int odroid_valid(u8 *data, u32 len, u8 expected)
{
	u8 sum = 0;

	while (len--)
		sum += *data++;

	return (sum == expected) ? 0 : -EINVAL;
}

static int odroid_provision(struct odroid_efuse_t *fuse)
{
	int ret;
	loff_t pos;

	if (memcmp((void *)&fuse->magic, EFUSE_ODROID_MAGIC,
				sizeof(((struct odroid_efuse_t *)0)->magic)))
		return -EINVAL;

	if (fuse->offset > 128 - sizeof((struct odroid_efuse_t *)0)->data)
		return -EINVAL;

	if (fuse->len > sizeof((struct odroid_efuse_t *)0)->data)
		return -EINVAL;

	ret = odroid_valid(fuse->data, fuse->len, fuse->sum);
	if (ret < 0) {
		pr_err("%s:%d, Invalid checksum\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* Write */
	pos = ((loff_t)(fuse->offset)) & 0xffffffff;
	ret = efuse_write_usr(fuse->data, fuse->len, &pos);
	if (ret < 0) {
		pr_err("failed to write efuse\n");
		return ret;
	}

	/* Read back */
	pos = ((loff_t)(fuse->offset)) & 0xffffffff;
	ret = efuse_read_usr(fuse->data, fuse->len, &pos);
	if (ret < 0) {
		pr_err("failed to read efuse\n");
		return ret;
	}

	/* Verify */
	ret = odroid_valid(fuse->data, fuse->len, fuse->sum);
	if (ret < 0) {
		pr_err("data mismatch\n");
		return -EINVAL;
	}

	return 0;
}
#endif

static long efuse_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	void __user	     *argp = (void __user *)arg;
	struct efusekey_info info;
	int		     ret;
#if defined(CONFIG_ARCH_MESON64_ODROID_COMMON)
	struct odroid_efuse_t fuse;
#endif

	switch (cmd) {
	case EFUSE_INFO_GET:
		ret = copy_from_user(&info, argp, sizeof(info));
		if (ret != 0) {
			pr_err("%s:%d,copy_from_user fail\n",
				__func__, __LINE__);
			return ret;
		}
		info.keyname[sizeof(info.keyname) - 1] = '\0';
		if (efuse_getinfo(info.keyname, &info) < 0) {
			pr_err("%s if not found\n", info.keyname);
			return -EFAULT;
		}

		ret = copy_to_user(argp, &info, sizeof(info));
		if (ret != 0) {
			pr_err("%s:%d,copy_to_user fail\n",
				__func__, __LINE__);
			return ret;
		}
		break;
#if defined(CONFIG_ARCH_MESON64_ODROID_COMMON)
	case EFUSE_INFO_PROVISION:
		ret = copy_from_user(&fuse, argp, sizeof(fuse));
		if (ret != 0) {
			pr_err("%s:%d, copy_from_user fail\n",
				__func__, __LINE__);
			return ret;
		}

		ret = odroid_provision(&fuse);
		if (ret < 0) {
			pr_err("%s:%d, Failed (%d)\n",
					__func__, __LINE__, ret);
		}
		break;
#endif

	default:
		return -ENOTTY;
	}
	return 0;
}


#ifdef CONFIG_COMPAT
static long efuse_compat_ioctl(struct file *filp,
			      unsigned int cmd, unsigned long args)
{
	unsigned long ret;

	args = (unsigned long)compat_ptr(args);
	ret = efuse_unlocked_ioctl(filp, cmd, args);

	return ret;
}
#endif

static ssize_t efuse_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	int ret;
	int local_count = 0;

	unsigned char *local_buf = kcalloc(count, sizeof(char), GFP_KERNEL);

	if (!local_buf)
		return -ENOMEM;

	local_count = efuse_read_usr(local_buf, count, ppos);
	if (local_count < 0) {
		ret =  -EFAULT;
		pr_err("%s: read user error!!--%d\n", __func__, __LINE__);
		goto error_exit;
	}

	if (copy_to_user((void *)buf, (void *)local_buf, local_count)) {
		ret =  -EFAULT;
		pr_err("%s: copy_to_user error!!--%d\n", __func__, __LINE__);
		goto error_exit;
	}
	ret = local_count;

error_exit:
	/*if (local_buf)*/
		kfree(local_buf);
	return ret;
}

static ssize_t efuse_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	int ret, size;

	unsigned int  pos = (unsigned int)*ppos;

	unsigned char *contents = NULL;

	if (pos >= EFUSE_BYTES)
		return 0;       /* Past EOF */
	if (count > EFUSE_BYTES - pos)
		count = EFUSE_BYTES - pos;
	if (count > EFUSE_BYTES)
		return -EFAULT;

	contents = kzalloc(sizeof(unsigned char)*EFUSE_BYTES, GFP_KERNEL);
	if (!contents)
		return -ENOMEM;

	size = sizeof(contents);
	memset(contents, 0, size);
	if (copy_from_user(contents, buf, count)) {
		pr_err("efuse_write copy_from_user ERROR!!\n");
			kfree(contents);
		return -EFAULT;
	}

	ret = efuse_write_usr(contents, count, ppos);
	if (ret < 0) {
		pr_err("write user area %Zd bytes data, fail!!!\n", count);
		kfree(contents);
		return -EFAULT;
	}

	pr_info("user area %d bytes data, write OK\n", ret);
	kfree(contents);
	return count;
}

static const struct file_operations efuse_fops = {
	.owner      = THIS_MODULE,
	.llseek     = efuse_llseek,
	.open       = efuse_open,
	.release    = efuse_release,
	.read       = efuse_read,
	.write      = efuse_write,
	.unlocked_ioctl      = efuse_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = efuse_compat_ioctl,
#endif
};


ssize_t efuse_user_attr_store(char *name, const char *buf, size_t count)
{
#ifndef EFUSE_READ_ONLY
	char *local_buf;
	ssize_t ret;
	int i;
	const char *c, *s;
	struct efusekey_info info;
	unsigned int uint_val;
	loff_t pos;

	if (efuse_getinfo(name, &info) < 0) {
		pr_err("%s is not found\n", name);
		return -EFAULT;
	}

	local_buf = kzalloc(sizeof(char)*(count), GFP_KERNEL);
	if (!local_buf) {
		ret = -ENOMEM;
		pr_err("efuse: failed to allocate memory!\n");
		return ret;
	}

	memcpy(local_buf, buf, count);

	c = ":";
	s = local_buf;
	if (strstr(s, c) != NULL) {
		for (i = 0; i < info.size; i++) {
			uint_val = 0;
			ret = sscanf(s, "%x", &uint_val);
			if (ret < 0) {
				pr_err("ERROR: efuse get user data fail!\n");
				goto error_exit;
			} else
				local_buf[i] = uint_val;
			s += 2;
			if (!strncmp(s, c, 1))
				s++;
			pr_debug("local_buf[%d]: 0x%x\n",
				i, local_buf[i]);
		}
	}

	pos = ((loff_t)(info.offset)) & 0xffffffff;
	ret = efuse_write_usr(local_buf, info.size, &pos);
	if (ret == -1) {
		pr_err("ERROR: efuse write user data fail!\n");
		goto error_exit;
	}
	if (ret != info.size)
		pr_err("ERROR: write %zd byte(s) not %d byte(s) data\n",
			ret, info.size);

	pr_info("efuse write %zd data OK\n", ret);

error_exit:
	kfree(local_buf);
	return count;
#else
	pr_err("no permission to write!!\n");
	return -1;
#endif
}

ssize_t efuse_user_attr_show(char *name, char *buf)
{
	char *local_buf;
	ssize_t ret;
	ssize_t len = 0;
	int i;
	struct efusekey_info info;
	loff_t pos;
#ifdef CONFIG_AMLOGIC_CPU_INFO
	unsigned char chipid[CHIPID_LEN];
	int j;
#endif

	if (efuse_getinfo(name, &info) < 0) {
		pr_err("%s is not found\n", name);
		return -EFAULT;
	}

	local_buf = kzalloc(sizeof(char)*(info.size), GFP_KERNEL);
	if (!local_buf) {
		ret = -ENOMEM;
		pr_err("efuse: failed to allocate memory!\n");
		return ret;
	}

	memset(local_buf, 0, info.size);

	pos = ((loff_t)(info.offset)) & 0xffffffff;
	ret = efuse_read_usr(local_buf, info.size, &pos);
	if (ret == -1) {
		pr_err("ERROR: efuse read user data fail!\n");
		goto error_exit;
	}
	if (ret != info.size)
		pr_err("ERROR: read %zd byte(s) not %d byte(s) data\n",
			ret, info.size);

	if (strcmp(name, "uuid")) {
		for (i = 0; i < info.size; i++) {
			if (i%16 == 0)
				len += sprintf(buf + len, "\n");
			if (i%16 == 0)
				len += sprintf(buf + len, "0x%02x: ", i);
			len += sprintf(buf + len, "%02x ", local_buf[i]);
		}
	} else {
#ifdef CONFIG_AMLOGIC_CPU_INFO
		if (strlen(local_buf) == 0) {
			cpuinfo_get_chipid(chipid, CHIPID_LEN);
			for (i = 0, j = 0; i < 16; i++, j += 2)
				sprintf(local_buf + j, "%02x", chipid[i]);
		}
#endif
		for (i = 0; i < info.size; i++) {
			if ((i == 8) || (i == 12) || (i == 16) || (i == 20))
				len += sprintf(buf + len, "-");
			len += sprintf(buf + len, "%c", local_buf[i]);
		}
	}
	len += sprintf(buf + len, "\n");
	ret = len;

error_exit:
	kfree(local_buf);
	return ret;
}

ssize_t efuse_user_attr_read(char *name, char *buf)
{
	char *local_buf;
	ssize_t ret;
	struct efusekey_info info;
	loff_t pos;

	if (efuse_getinfo(name, &info) < 0) {
		pr_err("%s is not found\n", name);
		return -EFAULT;
	}

	local_buf = kzalloc(sizeof(char)*(info.size), GFP_KERNEL);
	if (!local_buf) {
		ret = -ENOMEM;
		pr_err("efuse: failed to allocate memory!\n");
		return ret;
	}

	memset(local_buf, 0, info.size);

	pos = ((loff_t)(info.offset)) & 0xffffffff;
	ret = efuse_read_usr(local_buf, info.size, &pos);
	if (ret == -1) {
		pr_err("ERROR: efuse read user data fail!\n");
		goto error_exit;
	}
	if (ret != info.size)
		pr_err("ERROR: read %zd byte(s) not %d byte(s) data\n",
			ret, info.size);

	memcpy(buf, local_buf, info.size);

error_exit:
	kfree(local_buf);
	return ret;
}

static ssize_t userdata_show(struct class *cla,
	struct class_attribute *attr, char *buf)
{
	char *op;
	int ret;
	ssize_t len = 0;
	int i;
	loff_t offset;
	unsigned int max_size;

	ret = efuse_get_max();
	if (ret < 0) {
		pr_err("efuse: failed to get max size\n");
		return -1;
	}
	max_size = (unsigned int)ret;

	op = kmalloc_array(max_size, sizeof(char), GFP_KERNEL);
	if (!op) {
		ret = -ENOMEM;
		return ret;
	}

	memset(op, 0, max_size);
	offset = 0;

	ret = efuse_read_usr(op, max_size, &offset);
	if (ret < 0) {
		pr_err("%s: efuse read user data error!!\n", __func__);
		kfree(op);
		return -1;
	}

	for (i = 0; i < ret; i++) {
		if ((i != 0) && (i%16 == 0))
			len += sprintf(buf + len, "\n");
		if (i%16 == 0)
			len += sprintf(buf + len, "0x%02x: ", i);

		len += sprintf(buf + len, "%02x ", op[i]);
	}
	len += sprintf(buf + len, "\n");

	kfree(op);
	return len;
}

#ifndef EFUSE_READ_ONLY
static ssize_t userdata_write(struct class *cla,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int i;
	int ret;
	loff_t offset;
	char *op = NULL;
	unsigned int max_size;

	ret = efuse_get_max();
	if (ret < 0) {
		pr_err("efuse: failed to get max size\n");
		return -1;
	}
	max_size = (unsigned int)ret;

	op = kzalloc((sizeof(char)*max_size), GFP_KERNEL);
	if (!op) {
		pr_err("efuse: failed to allocate memory\n");
		ret = -ENOMEM;
		return ret;
	}

	memset(op, 0, max_size);
	for (i = 0; i < (count-1); i++)
		op[i] = buf[i];

	offset = 0;
	pr_debug("user area %d bytes data, write...\n", max_size);
	ret = efuse_write_usr(op, count, &offset);
	if (ret < 0) {
		kfree(op);
		pr_err("write user area %d bytes data, fail!!!\n", max_size);
		return -1;
	}
	pr_info("user area %d bytes data, write OK\n", ret);
	kfree(op);
	return count;
}
#endif

static ssize_t amlogic_set_store(struct class *cla,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int i;
	int ret;
	char *op = NULL;

	if (count != GXB_EFUSE_PATTERN_SIZE) {
		pr_err("efuse: bad size, only support size %d!\n",
			GXB_EFUSE_PATTERN_SIZE);
		return -EINVAL;
	}

	op = kzalloc((sizeof(char)*count), GFP_KERNEL);
	if (!op) {
		ret = -ENOMEM;
		pr_err("efuse: failed to allocate memory!\n");
		return ret;
	}

	memset(op, 0, count);
	for (i = 0; i < count; i++)
		op[i] = buf[i];

	ret = efuse_amlogic_set(op, count);
	kfree(op);

	if (ret) {
		pr_err("EFUSE pattern programming fail! ret: %d\n", ret);
		return -EINVAL;
	}

	pr_info("EFUSE pattern programming success!\n");

	return count;
}

static struct class_attribute efuse_class_attrs[] = {

	#ifndef EFUSE_READ_ONLY
	/*make the efuse can not be write through sysfs */
	__ATTR(userdata, 0700, userdata_show, userdata_write),

	#else
	__ATTR_RO(userdata),

	#endif

#if !defined(CONFIG_ARCH_MESON64_ODROID_COMMON)
	__ATTR(mac, 0700, show_mac, store_mac),

	__ATTR(mac_bt, 0700, show_mac_bt, store_mac_bt),

	__ATTR(mac_wifi, 0700, show_mac_wifi, store_mac_wifi),

	__ATTR(usid, 0700, show_usid, store_usid),
#else
	__ATTR_RO(uuid),
#endif
	__ATTR_WO(amlogic_set),

	__ATTR_NULL

};

static struct class efuse_class = {

	.name = EFUSE_CLASS_NAME,

	.class_attrs = efuse_class_attrs,

};

int get_efusekey_info(struct device_node *np)
{
	const phandle *phandle;
	struct device_node *np_efusekey, *np_key;
	int index;
	char *propname;
	const char *uname;
	int ret;
	int size;

	phandle = of_get_property(np, "key", NULL);
	if (!phandle) {
		pr_info("%s:don't find match key\n", __func__);
		return -1;
	}
	if (phandle) {
		np_efusekey = of_find_node_by_phandle(be32_to_cpup(phandle));
		if (!np_efusekey) {
			pr_err("can't find device node key\n");
			return -1;
		}
	}

	ret = of_property_read_u32(np_efusekey, "keynum", &efusekeynum);
	if (ret) {
		pr_err("please config efusekeynum item\n");
		return -1;
	}
	pr_info("efusekeynum: %d\n", efusekeynum);

	if (efusekeynum > 0) {
		efusekey_infos = kzalloc((sizeof(struct efusekey_info))
			*efusekeynum, GFP_KERNEL);
		if (!efusekey_infos)
			return -1;
	}

	for (index = 0; index < efusekeynum; index++) {
		propname = kasprintf(GFP_KERNEL, "key%d", index);

		phandle = of_get_property(np_efusekey, propname, NULL);
		if (!phandle) {
			pr_err("don't find  match %s\n", propname);
			goto err;
		}
		if (phandle) {
			np_key = of_find_node_by_phandle(be32_to_cpup(phandle));
			if (!np_key) {
				pr_err("can't find device node\n");
				goto err;
			}
		}
		ret = of_property_read_string(np_key, "keyname", &uname);
		if (ret) {
			pr_err("please config keyname item\n");
			goto err;
		}
		size = sizeof(efusekey_infos[index].keyname) - 1;
		strncpy(efusekey_infos[index].keyname, uname,
			strlen(uname) > size ? size:strlen(uname));
		ret = of_property_read_u32(np_key, "offset",
			&(efusekey_infos[index].offset));
		if (ret) {
			pr_err("please config offset item\n");
			goto err;
		}
		ret = of_property_read_u32(np_key, "size",
			&(efusekey_infos[index].size));
		if (ret) {
			pr_err("please config size item\n");
			goto err;
		}

		pr_info("efusekeyname: %15s\toffset: %5d\tsize: %5d\n",
			efusekey_infos[index].keyname,
			efusekey_infos[index].offset,
			efusekey_infos[index].size);
		kfree(propname);
	}
	return 0;
err:
	kfree(efusekey_infos);
	return -1;

}

static int efuse_probe(struct platform_device *pdev)
{
	int ret;
	struct device *devp;
	struct device_node *np = pdev->dev.of_node;
	struct clk *efuse_clk;

	/* open clk gate HHI_GCLK_MPEG0 bit62*/
	efuse_clk = devm_clk_get(&pdev->dev, "efuse_clk");
	if (IS_ERR(efuse_clk))
		dev_err(&pdev->dev, " open efuse clk gate error!!\n");
	else{
		ret = clk_prepare_enable(efuse_clk);
		if (ret)
			dev_err(&pdev->dev, "enable efuse clk gate error!!\n");
	}

	if (pdev->dev.of_node) {
		of_node_get(np);

		ret = of_property_read_u32(np, "read_cmd", &efuse_read_cmd);
		if (ret) {
			dev_err(&pdev->dev, "please config read_cmd item\n");
			return -1;
		}

		ret = of_property_read_u32(np, "write_cmd", &efuse_write_cmd);
		if (ret) {
			dev_err(&pdev->dev, "please config write_cmd item\n");
			return -1;
		}

	  ret = of_property_read_u32(np, "get_max_cmd", &efuse_get_max_cmd);
		if (ret) {
			dev_err(&pdev->dev, "please config get_max_cmd\n");
			return -1;
		}

		get_efusekey_info(np);
	}

	ret = alloc_chrdev_region(&efuse_devno, 0, 1, EFUSE_DEVICE_NAME);
	if (ret < 0) {
		ret = -ENODEV;
		goto out;
	}

	ret = class_register(&efuse_class);
	if (ret)
		goto error1;

	efuse_devp = kmalloc(sizeof(struct efuse_dev_t), GFP_KERNEL);
	if (!efuse_devp) {
		ret = -ENOMEM;
		goto error2;
	}

	/* connect the file operations with cdev */
	cdev_init(&efuse_devp->cdev, &efuse_fops);
	efuse_devp->cdev.owner = THIS_MODULE;
	/* connect the major/minor number to the cdev */
	ret = cdev_add(&efuse_devp->cdev, efuse_devno, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to add device\n");
		goto error3;
	}

	devp = device_create(&efuse_class, NULL, efuse_devno, NULL, "efuse");
	if (IS_ERR(devp)) {
		dev_err(&pdev->dev, "failed to create device node\n");
		ret = PTR_ERR(devp);
		goto error4;
	}
	dev_dbg(&pdev->dev, "device %s created\n", EFUSE_DEVICE_NAME);

	sharemem_input_base = get_secmon_sharemem_input_base();
	sharemem_output_base = get_secmon_sharemem_output_base();
	dev_info(&pdev->dev, "probe OK!\n");
	return 0;

error4:
	cdev_del(&efuse_devp->cdev);
error3:
	kfree(efuse_devp);
error2:
	/* class_destroy(efuse_clsp); */
	class_unregister(&efuse_class);
error1:
	unregister_chrdev_region(efuse_devno, 1);
out:
	return ret;
}

static int efuse_remove(struct platform_device *pdev)
{
	unregister_chrdev_region(efuse_devno, 1);
	/* device_destroy(efuse_clsp, efuse_devno); */
	device_destroy(&efuse_class, efuse_devno);
	cdev_del(&efuse_devp->cdev);
	kfree(efuse_devp);
	/* class_destroy(efuse_clsp); */
	class_unregister(&efuse_class);
	return 0;
}

static const struct of_device_id amlogic_efuse_dt_match[] = {
	{	.compatible = "amlogic, efuse",
	},
	{},
};

static struct platform_driver efuse_driver = {
	.probe = efuse_probe,
	.remove = efuse_remove,
	.driver = {
		.name = EFUSE_DEVICE_NAME,
		.of_match_table = amlogic_efuse_dt_match,
	.owner = THIS_MODULE,
	},
};

static int __init efuse_init(void)
{
	int ret = -1;

	ret = platform_driver_register(&efuse_driver);
	if (ret != 0) {
		pr_err("failed to register efuse driver, error %d\n", ret);
		return -ENODEV;
	}

	return ret;
}

static void __exit efuse_exit(void)
{
	platform_driver_unregister(&efuse_driver);
}

module_init(efuse_init);
module_exit(efuse_exit);

MODULE_DESCRIPTION("AMLOGIC eFuse driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cai Yun <yun.cai@amlogic.com>");

