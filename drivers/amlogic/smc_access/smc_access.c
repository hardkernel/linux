/*
 * drivers/amlogic/smc_access/smc_access.c
 * github.com/frederic/smc_access
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
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

/* This is a debugfs driver written by github.com/frederic/ 
 * and modified by myself to add JTAG enable function.
*/

/* Standard Linux headers */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#define JTAG_ON		0x82000040
#define JTAG_OFF	0x82000041 // Not implemented yet

/* Currently only the Cortex-M3 is available for JTAG, the A53 still has issues for me, but it may be because of my tools (OpenOCD) and their developmental ARMv8 support. Neither of the power domain taps are working yet either, this may be an Amlogic limitation. */

static struct dentry *debugfs_root = NULL;

static ssize_t smc_write_file(struct file *file, const char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	register uint64_t x0 asm("x0") = 0;
	register uint64_t x1 asm("x1") = 0;
	register uint64_t x2 asm("x2") = 0;
	register uint64_t x3 asm("x3") = 0;
	register uint64_t x4 asm("x4") = 0;
	char buf[80];
	uint64_t arg0, arg1, arg2, arg3, arg4;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	sscanf(buf, "%llx %llx %llx %llx %llx", &arg0, &arg1, &arg2, &arg3, &arg4);
	
	x0 = arg0;
	x1 = arg1;
	x2 = arg2;
	x3 = arg3;
	x4 = arg4;
	
	asm __volatile__("" : : : "memory");

	do {
		asm volatile(
		    __asmeq("%0", "x0")
		    __asmeq("%1", "x0")
		    __asmeq("%2", "x1")
		    __asmeq("%3", "x2")
		    __asmeq("%4", "x3")
		    __asmeq("%5", "x4")
		    "smc #0\n"
		    : "=r"(x0)
		    : "r"(x0), "r"(x1), "r"(x2),"r"(x3),"r"(x4));
	} while (0);
	
	arg1 = x0;
	
	printk(KERN_ALERT "smc_access: SMC call %llx returns: %llx\n", arg0, arg1);

	return count;
}

static ssize_t jtag_write_file(struct file *file, const char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	register uint64_t x0 asm("x0") = 0;
	register uint64_t x1 asm("x1") = 0;
	char buf[80];
	uint64_t ret, tap;
	
	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
			return -EFAULT;
	
	buf[count] = 0;
	
	sscanf(buf, "%llx", &tap);
	
	/* As mentioned previously, the only TAP that I've had success with so far is the Cortex-M3 (co-processor?)
		0 = M3
		1 = M3 power domain (not working, Amlogic restricted?)
		2 = A53 (semi-working, TAP is exposed but cannot halt, maybe non-invasive/non-secure debug only?
		3 = A53 power domain (not working, Amlogic restricted?)
		>3? Possibly a boundary scan tap, have not experimented yet
	*/
	if (tap > 3) {
		printk(KERN_INFO "smc_access: Max value is 3, argument invalid\n");
		return -EINVAL;
		}
	
	x0 = JTAG_ON;
	x1 = tap;
	
	asm __volatile__("" : : : "memory");

	do {
		asm volatile(
		    __asmeq("%0", "x0")
		    __asmeq("%1", "x0")
		    __asmeq("%2", "x1")
		    "smc #0\n"
		    : "=r"(x0)
		    : "r"(x0), "r"(x1));
	} while (0);
	
	ret = x0;
	
	printk(KERN_ALERT "smc_access: JTAG enable call returns: %llx\n", ret);

	return count;
}

static const struct file_operations smc_file_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= smc_write_file,
};

static const struct file_operations jtag_file_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= jtag_write_file,
};

static int __init smc_access_init(void)
{
	struct dentry *debugfs_file;
	
	debugfs_root = debugfs_create_dir("aml_smc", NULL);
	if (IS_ERR(debugfs_root) || !debugfs_root) {
		pr_warn("smc_access: failed to create smc_access debugfs directory\n");
		debugfs_root = NULL;
		return -1;
	}

	debugfs_file = debugfs_create_file("smc", S_IFREG | S_IRUGO,
			    debugfs_root, NULL, &smc_file_ops);
	if (!debugfs_file) {
		printk(KERN_ALERT "smc_access: failed to create smc_access debugfs file\n");
		return -1;
    }

	debugfs_file = debugfs_create_file("jtag", S_IFREG | S_IRUGO,
			    debugfs_root, NULL, &jtag_file_ops);
	if (!debugfs_file) {
		printk(KERN_ALERT "smc_access: failed to create smc_access debugfs file\n");
		return -1;
    }
    
	return 0;
}

static void __exit smc_access_exit(void)
{
	printk(KERN_ALERT "smc_access: exiting...\n");
	if(debugfs_root)
		debugfs_remove_recursive(debugfs_root);
}


module_init(smc_access_init);
module_exit(smc_access_exit);

MODULE_DESCRIPTION("SMC module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xing Xu <xing.xu@amlogic.com>");

