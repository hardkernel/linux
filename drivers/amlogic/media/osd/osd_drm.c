/*
 * drivers/amlogic/media/osd/osd_hw.c
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

/* Linux Headers */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/irqreturn.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/ctype.h>

/* Android Headers */
#include <linux/amlogic/cpu_version.h>
#include "osd_drm.h"
#include "osd_hw.h"
#include "osd_io.h"
#include "osd.h"
#include "osd_log.h"

#define MAX_PLANE 4
static struct dentry *osd_debugfs_root;
static unsigned int osd_enable[MAX_PLANE];
static int plane_osd_id[MAX_PLANE];

static int parse_para(const char *para, int para_num, int *result)
{
	char *token = NULL;
	char *params, *params_base;
	int *out = result;
	int len = 0, count = 0;
	int res = 0;
	int ret = 0;

	if (!para)
		return 0;

	params = kstrdup(para, GFP_KERNEL);
	params_base = params;
	token = params;
	if (token) {
		len = strlen(token);
		do {
			token = strsep(&params, " ");
			if (!token)
				break;
			while (token && (isspace(*token)
					|| !isgraph(*token)) && len) {
				token++;
				len--;
			}
			if (len == 0)
				break;
			ret = kstrtoint(token, 0, &res);
			if (ret < 0)
				break;
			len = strlen(token);
			*out++ = res;
			count++;
		} while ((count < para_num) && (len > 0));
	}

	kfree(params_base);
	return count;
}

static ssize_t loglevel_read_file(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;

	len = snprintf(buf, 128, "%d\n", osd_log_level);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t loglevel_write_file(
	struct file *file, const char __user *userbuf,
	size_t count, loff_t *ppos)
{
	unsigned int log_level;
	char buf[128];
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &log_level);
	osd_log_info("log_level: %d->%d\n", osd_log_level, log_level);
	osd_log_level = log_level;
	return count;
}

static ssize_t logmodule_read_file(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;

	len = snprintf(buf, 128, "%d\n", osd_log_module);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t logmodule_write_file(
	struct file *file, const char __user *userbuf,
	size_t count, loff_t *ppos)
{
	unsigned int log_module;
	char buf[128];
	int ret = 0;

	if (count > sizeof(buf) || count <= 0)
		return -EINVAL;
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	if (buf[count - 1] == '\n')
		buf[count - 1] = '\0';
	ret = kstrtoint(buf, 0, &log_module);
	osd_log_info("log_level: %d->%d\n", osd_log_module, log_module);
	osd_log_module = log_module;
	return count;
}

static ssize_t debug_read_file(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[1024];
	char *help;
	ssize_t len;

	help = osd_get_debug_hw();
	len = snprintf(buf, strlen(help), "%s", help);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t debug_write_file(struct file *file, const char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	char buf[128];

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	osd_set_debug_hw(buf);
	return count;
}

static ssize_t osd_display_debug_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	ssize_t len;
	u32 osd_display_debug_enable;

	osd_get_display_debug(osd_id, &osd_display_debug_enable);
	len = snprintf(buf, 128, "%d\n", osd_display_debug_enable);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t osd_display_debug_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	u32 osd_display_debug_enable;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &osd_display_debug_enable);
	osd_set_display_debug(osd_id, osd_display_debug_enable);
	return count;
}

static ssize_t reset_status_read_file(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	unsigned int status;

	status = osd_get_reset_status();
	len = snprintf(buf, 128, "0x%x\n", status);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t blank_read_file(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;

	len = snprintf(buf, 128, "%d\n", osd_enable[osd_id]);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t blank_write_file(struct file *file, const char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &osd_enable[osd_id]);
	osd_enable_hw(osd_id, (osd_enable[osd_id] != 0) ? 0 : 1);
	return count;
}

static ssize_t free_scale_read_file(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned int free_scale_enable;

	osd_get_free_scale_enable_hw(osd_id, &free_scale_enable);
	len = snprintf(buf, 128, "free_scale_enable:[0x%x]\n",
			free_scale_enable);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t free_scale_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned int free_scale_enable;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &free_scale_enable);
	osd_set_free_scale_enable_hw(osd_id, free_scale_enable);
	return count;
}

static ssize_t free_scale_axis_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	int x, y, w, h;

	osd_get_free_scale_axis_hw(osd_id, &x, &y, &w, &h);
	len = snprintf(buf, 128, "%d %d %d %d\n", x, y, w, h);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t free_scale_axis_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	int parsed[4];

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	if (likely(parse_para(buf, 4, parsed) == 4))
		osd_set_free_scale_axis_hw(osd_id,
			parsed[0], parsed[1], parsed[2], parsed[3]);
	else
		osd_log_err("set free scale axis error\n");
	return count;
}

static ssize_t window_axis_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	int x0, y0, x1, y1;

	osd_get_window_axis_hw(osd_id, &x0, &y0, &x1, &y1);
	len = snprintf(buf, 128, "%d %d %d %d\n",
		x0, y0, x1, y1);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t window_axis_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	int parsed[4];

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	if (likely(parse_para(buf, 4, parsed) == 4))
		osd_set_window_axis_hw(osd_id,
			parsed[0], parsed[1], parsed[2], parsed[3]);
	else
		osd_log_err("set window axis error\n");
	return count;
}

static ssize_t osd_reverse_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char *str[4] = {"NONE", "ALL", "X_REV", "Y_REV"};
	unsigned int osd_reverse = 0;

	osd_get_reverse_hw(osd_id, &osd_reverse);
	if (osd_reverse >= REVERSE_MAX)
		osd_reverse = REVERSE_FALSE;
	len = snprintf(buf, 128, "osd_reverse:[%s]\n",
			str[osd_reverse]);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t osd_reverse_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned int osd_reverse = 0;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &osd_reverse);
	if (osd_reverse >= REVERSE_MAX)
		osd_reverse = REVERSE_FALSE;
	osd_set_reverse_hw(osd_id, osd_reverse, 1);
	return count;
}

static ssize_t osd_order_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	ssize_t len;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned int order = 0;

	osd_get_order_hw(osd_id, &order);
	len = snprintf(buf, 128, "order:[0x%x]\n", order);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t osd_order_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned int order = 0;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &order);
	osd_set_order_hw(osd_id, order);
	return count;
}

static ssize_t osd_afbcd_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	ssize_t len;
	unsigned int enable_afbcd = 0;

	enable_afbcd = osd_get_afbc(osd_id);
	len = snprintf(buf, 128, "%d\n", enable_afbcd);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t osd_afbcd_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	unsigned int enable_afbcd = 0;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &enable_afbcd);
	osd_log_info("afbc: %d\n", enable_afbcd);
	osd_set_afbc(osd_id, enable_afbcd);
	return count;
}

static ssize_t osd_clear_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned int osd_clear = 0;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &osd_clear);
	if (osd_clear)
		osd_set_clear(osd_id);
	return count;
}

static ssize_t osd_dump_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	u8 __iomem *buf = NULL;
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	unsigned long len = 0;

	len = get_vmap_addr(osd_id, &buf);
	if (buf && len)
		return simple_read_from_buffer(userbuf, count, ppos, buf, len);
	else
		return 0;
}

static ssize_t osd_dump_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
#if 1
	return 0;
#else
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;

	return dd_vmap_write(osd_id,  userbuf, count, ppos);
#endif
}

static void parse_param(char *buf_orig, char **parm)
{
	char *ps, *token;
	unsigned int n = 0;
	char delim1[3] = " ";
	char delim2[2] = "\n";

	ps = buf_orig;
	strcat(delim1, delim2);
	while (1) {
		token = strsep(&ps, delim1);
		if (token == NULL)
			break;
		if (*token == '\0')
			continue;
		parm[n++] = token;
	}
}

static ssize_t osd_reg_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	char buf[128];
	char *buf_orig, *parm[8] = {NULL};
	long val = 0;
	unsigned int reg_addr, reg_val;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	buf_orig = kstrdup(buf, GFP_KERNEL);
	parse_param(buf_orig, (char **)&parm);
	if (!strcmp(parm[0], "rv")) {
		if (kstrtoul(parm[1], 16, &val) < 0)
			return -EINVAL;
		reg_addr = val;
		reg_val = osd_reg_read(reg_addr);
		pr_info("reg[0x%04x]=0x%08x\n", reg_addr, reg_val);
	} else if (!strcmp(parm[0], "wv")) {
		if (kstrtoul(parm[1], 16, &val) < 0)
			return -EINVAL;
		reg_addr = val;
		if (kstrtoul(parm[2], 16, &val) < 0)
			return -EINVAL;
		reg_val = val;
		osd_reg_write(reg_addr, reg_val);
	}
	return count;
}

static ssize_t osd_hwc_enable_read_file(struct file *file,
				char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	ssize_t len;
	unsigned int hwc_enable = 0;

	osd_get_hwc_enable(osd_id, &hwc_enable);
	len = snprintf(buf, 128, "%d\n", hwc_enable);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t osd_hwc_enable_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	unsigned int hwc_enable = 0;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &hwc_enable);
	osd_log_info("hwc enable: %d\n", hwc_enable);
	osd_set_hwc_enable(osd_id, hwc_enable);
	return count;
}

static ssize_t osd_do_hwc_write_file(struct file *file,
				const char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	int osd_id = *(int *)s;
	char buf[128];
	unsigned int do_hwc = 0;
	int ret = 0;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = 0;
	ret = kstrtoint(buf, 0, &do_hwc);
	osd_log_info("do_hwc: %d\n", do_hwc);
	if (do_hwc)
		osd_do_hwc(osd_id);
	return count;
}

static const struct file_operations loglevel_file_ops = {
	.open		= simple_open,
	.read		= loglevel_read_file,
	.write		= loglevel_write_file,
};

static const struct file_operations logmodule_file_ops = {
	.open		= simple_open,
	.read		= logmodule_read_file,
	.write		= logmodule_write_file,
};

static const struct file_operations debug_file_ops = {
	.open		= simple_open,
	.read		= debug_read_file,
	.write		= debug_write_file,
};

static const struct file_operations osd_display_debug_file_ops = {
	.open		= simple_open,
	.read		= osd_display_debug_read_file,
	.write		= osd_display_debug_write_file,
};

static const struct file_operations reset_status_file_ops = {
	.open		= simple_open,
	.read		= reset_status_read_file,
};

static const struct file_operations blank_file_ops = {
	.open		= simple_open,
	.read		= blank_read_file,
	.write		= blank_write_file,
};

static const struct file_operations free_scale_file_ops = {
	.open		= simple_open,
	.read		= free_scale_read_file,
	.write		= free_scale_write_file,
};

static const struct file_operations free_scale_axis_file_ops = {
	.open		= simple_open,
	.read		= free_scale_axis_read_file,
	.write		= free_scale_axis_write_file,
};

static const struct file_operations window_axis_file_ops = {
	.open		= simple_open,
	.read		= window_axis_read_file,
	.write		= window_axis_write_file,
};

static const struct file_operations osd_reverse_file_ops = {
	.open		= simple_open,
	.read		= osd_reverse_read_file,
	.write		= osd_reverse_write_file,
};

static const struct file_operations osd_order_file_ops = {
	.open		= simple_open,
	.read		= osd_order_read_file,
	.write		= osd_order_write_file,
};

static const struct file_operations osd_afbcd_file_ops = {
	.open		= simple_open,
	.read		= osd_afbcd_read_file,
	.write		= osd_afbcd_write_file,
};

static const struct file_operations osd_clear_file_ops = {
	.open		= simple_open,
	.write		= osd_clear_write_file,
};

static const struct file_operations osd_dump_file_ops = {
	.open		= simple_open,
	.read		= osd_dump_read_file,
	.write		= osd_dump_write_file,
};

static const struct file_operations osd_reg_file_ops = {
	.open		= simple_open,
	.write		= osd_reg_write_file,
};

static const struct file_operations osd_hwc_enable_file_ops = {
	.open		= simple_open,
	.read		= osd_hwc_enable_read_file,
	.write		= osd_hwc_enable_write_file,
};

static const struct file_operations osd_do_hwc_file_ops = {
	.open		= simple_open,
	.write		= osd_do_hwc_write_file,
};

struct osd_drm_debugfs_files_s {
	const char *name;
	const umode_t mode;
	const struct file_operations *fops;
};

static struct osd_drm_debugfs_files_s osd_drm_debugfs_files[] = {
	{"loglevel", S_IFREG | 0640, &loglevel_file_ops},
	{"logmodule", S_IFREG | 0640, &logmodule_file_ops},
	{"debug", S_IFREG | 0640, &debug_file_ops},
	{"osd_display_debug", S_IFREG | 0640, &osd_display_debug_file_ops},
	{"reset_status", S_IFREG | 0440, &reset_status_file_ops},
	{"blank", S_IFREG | 0640, &blank_file_ops},
	{"free_scale", S_IFREG | 0640, &free_scale_file_ops},
	{"free_scale_axis", S_IFREG | 0640, &free_scale_axis_file_ops},
	{"window_axis", S_IFREG | 0640, &window_axis_file_ops},
	{"osd_reverse", S_IFREG | 0640, &osd_reverse_file_ops},
	{"order", S_IFREG | 0640, &osd_order_file_ops},
	{"osd_afbcd", S_IFREG | 0640, &osd_afbcd_file_ops},
	{"osd_clear", S_IFREG | 0220, &osd_clear_file_ops},
	{"osd_dump", S_IFREG | 0640, &osd_dump_file_ops},
	{"osd_reg", S_IFREG | 0220, &osd_reg_file_ops},
	{"osd_hwc_enable", S_IFREG | 0640, &osd_hwc_enable_file_ops},
	{"osd_do_hwc", S_IFREG | 0220, &osd_do_hwc_file_ops},

};

void osd_drm_debugfs_add(
	struct dentry **plane_debugfs_dir,
	char *name,
	int osd_id)
{
	struct dentry *ent;
	int i;

	osd_drm_debugfs_init();

	plane_osd_id[osd_id] = osd_id;
	*plane_debugfs_dir = debugfs_create_dir(name, osd_debugfs_root);
	if (!*plane_debugfs_dir) {
		osd_log_info("debugfs_create_dir failed: name=%s\n", name);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(osd_drm_debugfs_files); i++) {
		ent = debugfs_create_file(osd_drm_debugfs_files[i].name,
			osd_drm_debugfs_files[i].mode,
			*plane_debugfs_dir, &plane_osd_id[osd_id],
			osd_drm_debugfs_files[i].fops);
		if (!ent)
			osd_log_info("debugfs create failed\n");
	}

}
EXPORT_SYMBOL(osd_drm_debugfs_add);

void osd_drm_debugfs_init(void)
{
	if (osd_debugfs_root)
		return;
	osd_debugfs_root = debugfs_create_dir("graphics", NULL);
	if (!osd_debugfs_root)
		pr_err("can't create debugfs dir\n");
}
EXPORT_SYMBOL(osd_drm_debugfs_init);

void osd_drm_debugfs_exit(void)
{
	debugfs_remove(osd_debugfs_root);
}
EXPORT_SYMBOL(osd_drm_debugfs_exit);

void osd_drm_vsync_isr_handler(void)
{

	if (!osd_hw.hw_rdma_en) {
		osd_update_scan_mode();
		/* go through update list */
		walk_through_update_list();
		osd_update_3d_mode();
		osd_mali_afbc_start();
		osd_update_vsync_hit();
		osd_hw_reset();
	} else {
		if (osd_hw.osd_meson_dev.cpu_id != __MESON_CPU_MAJOR_ID_AXG)
			osd_rdma_interrupt_done_clear();
		else {
			osd_update_scan_mode();
			/* go through update list */
			walk_through_update_list();
			osd_update_3d_mode();
			osd_update_vsync_hit();
			osd_hw_reset();
		}
	}
}
EXPORT_SYMBOL(osd_drm_vsync_isr_handler);

void osd_drm_plane_page_flip(struct osd_plane_map_s *plane_map)
{
	osd_page_flip(plane_map);
}
EXPORT_SYMBOL(osd_drm_plane_page_flip);

void osd_drm_plane_enable_hw(u32 index, u32 enable)
{
	osd_enable_hw(index, enable);
}
EXPORT_SYMBOL(osd_drm_plane_enable_hw);

int osd_drm_init(struct osd_device_data_s *osd_meson_dev)
{
	int ret;

	/* osd hw init */
	ret = osd_io_remap(osd_meson_dev->osd_ver == OSD_SIMPLE);
	if (!ret) {
		osd_log_err("osd_io_remap failed\n");
		return -1;
	}
	/* init osd logo */
	ret = logo_work_init();
	if (ret == 0)
		osd_init_hw(1, 0, osd_meson_dev);
	else
		osd_init_hw(0, 0, osd_meson_dev);
	if (osd_meson_dev->osd_ver <= OSD_NORMAL) {
		/* freescale switch from osd2 to osd1*/
		osd_log_info("freescale switch from osd2 to osd1\n");
		osd_set_free_scale_mode_hw(OSD2, 1);
		osd_set_free_scale_enable_hw(OSD2, 0);
		osd_set_free_scale_mode_hw(OSD1, 1);
		osd_set_free_scale_axis_hw(OSD1, 0, 0, 1919, 1279);
		osd_set_window_axis_hw(OSD1, 0, 0, 1919, 1279);
		osd_set_free_scale_enable_hw(OSD1, 0x10001);
		osd_enable_hw(OSD1, 1);
	}
	return 0;
}
EXPORT_SYMBOL(osd_drm_init);
