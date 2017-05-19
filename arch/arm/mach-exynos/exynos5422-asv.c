/* linux/arch/arm/mach-exynos/asv-exynos5422.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5422 - ASV(Adoptive Support Voltage) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/*----------------------------------------------------------------------------*/
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/platform_device.h>
#include <linux/irqchip.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>

#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_info.h>

#include <mach/map.h>
#include <plat/cpu.h>
/*----------------------------------------------------------------------------*/
#include <linux/cpu.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/export.h>

#include "exynos5422-asv.h"

/*----------------------------------------------------------------------------*/
enum {
	DEVICE_ID_ARM = 0,
	DEVICE_ID_KFC,
	DEVICE_ID_MAX
};

struct exynos_device {
	unsigned int	base_volt;
	unsigned int	dvfs_nr;
	unsigned int	offset_volt_h;
	unsigned int	offset_volt_l;
	unsigned int	(*asv_func)(unsigned int freq, unsigned int volt);
};

struct exynos_asv {
	unsigned char		is_odroid;
	unsigned char		is_bin2;
	unsigned char		is_special_lot;
	unsigned char		hpm;
	unsigned char		ids;
	unsigned char		group;
	unsigned char		table;

	struct exynos_device	device[DEVICE_ID_MAX];
};

static struct exynos_asv	*exynos5422_asv = NULL;

/*----------------------------------------------------------------------------*/
int is_odroid(void)
{
	if (exynos5422_asv)
		return	exynos5422_asv->is_odroid;

	return	0;
}
EXPORT_SYMBOL(is_odroid);

/*----------------------------------------------------------------------------*/
const unsigned char DEVICE_ID_ARM_STR[] = "/soc/opp_table0/";
const unsigned char DEVICE_ID_KFC_STR[] = "/soc/opp_table1/";

int find_device_id(const char *node_str)
{
	if (!strncmp(DEVICE_ID_ARM_STR, node_str, sizeof(DEVICE_ID_ARM_STR)-1))
		return	DEVICE_ID_ARM;
	if (!strncmp(DEVICE_ID_KFC_STR, node_str, sizeof(DEVICE_ID_KFC_STR)-1))
		return	DEVICE_ID_KFC;

	return	-1;
}

EXPORT_SYMBOL(find_device_id);
/*----------------------------------------------------------------------------*/
static unsigned int asv_apply_volt(struct exynos_device *device,
			const unsigned int (*asv_table)[EXYNOS5422_ASV_NR +1],
			unsigned int freq, unsigned int volt)
{
	unsigned int i, asv_volt;

	for (i = 0; i < device->dvfs_nr; i++) {
		if (freq == asv_table[i][0])	break;
	}

	if (i == device->dvfs_nr)
		return	0;

	asv_volt = asv_table[i][exynos5422_asv->group +1];

	if (volt > device->base_volt)
		asv_volt += device->offset_volt_h;
	else
		asv_volt += device->offset_volt_l;

	return	asv_volt;
}

/*----------------------------------------------------------------------------*/
static unsigned int arm_asv_func(unsigned int freq, unsigned int volt)
{
	const unsigned int (*asv_table)[EXYNOS5422_ASV_NR +1];

	if (exynos5422_asv->is_bin2)
		asv_table = ARM_INFO_BIN2;
	else {
		switch(exynos5422_asv->table) {
			default:
			case	0:
			case	1:
				asv_table = ARM_INFO_TABLE01;
				break;
			case	2:
				asv_table = ARM_INFO_TABLE2;
				break;
			case	3:
				asv_table = ARM_INFO_TABLE3;
				break;
		}
	}

	return	asv_apply_volt(&exynos5422_asv->device[DEVICE_ID_ARM],
				asv_table,
				freq,
				volt);
}

/*----------------------------------------------------------------------------*/
static unsigned int kfc_asv_func(unsigned int freq, unsigned int volt)
{
	const unsigned int (*asv_table)[EXYNOS5422_ASV_NR +1];

	if (exynos5422_asv->is_bin2)
		asv_table = KFC_INFO_BIN2;
	else {
		switch(exynos5422_asv->table) {
			default:
			case	0:
			case	1:
				asv_table = KFC_INFO_TABLE01;
				break;
			case	2:
				asv_table = KFC_INFO_TABLE2;
				break;
			case	3:
				asv_table = KFC_INFO_TABLE3;
				break;
		}
	}

	return	asv_apply_volt(&exynos5422_asv->device[DEVICE_ID_KFC],
				asv_table,
				freq,
				volt);
}

/*----------------------------------------------------------------------------*/
int opp_update_for_odroid(unsigned int opp_freq,
	unsigned int opp_volt, const char *node_name)
{
	int device_id = find_device_id(node_name);

	if (device_id < 0)
		return	-1;

	/* find freq & get apply volt */
	return	exynos5422_asv->device[device_id].asv_func(opp_freq, opp_volt);
}

EXPORT_SYMBOL(opp_update_for_odroid);
/*----------------------------------------------------------------------------*/
static int board_check(void)
{
	struct device_node *root_node;
	const char *model_str;

	root_node = of_find_node_by_path("/");

	if (root_node) {
		if(!of_property_read_string(root_node, "model", &model_str)) {
			if (!strncmp("Hardkernel Odroid",
				model_str,
				sizeof("Hardkernel Odroid")-1))
				return	true;
		}
	}
	return	false;
}

/*----------------------------------------------------------------------------*/
static int bin2_check(void)
{
	unsigned int chip_id3 = __raw_readl(CHIP_ID3_REG);
	struct device_node *root_node;
	const char *model_str;

	root_node = of_find_node_by_path("/");

	if (root_node) {
		if(!of_property_read_string(root_node, "model", &model_str)) {
			if (!strncmp("Hardkernel Odroid XU3 Lite",
				model_str,
				sizeof("Hardkernel Odroid XU3 Lite")-1))
				return	true;
		}
	}

	return	(chip_id3 >> EXYNOS5422_BIN2_OFFSET) & EXYNOS5422_BIN2_MASK;
}

/*----------------------------------------------------------------------------*/
static int asv_special_group_check(void)
{
	unsigned int chip_id3 = __raw_readl(CHIP_ID3_REG);

	if ((chip_id3 >> EXYNOS5422_USESG_OFFSET) & EXYNOS5422_USESG_MASK &&
		exynos5422_asv->is_bin2 == 0)
		return	true;

	return	false;
}

/*----------------------------------------------------------------------------*/
static int asv_group_check(unsigned int special_group)
{
	unsigned int chip_id3 = __raw_readl(CHIP_ID3_REG);
	unsigned int chip_id4 = __raw_readl(CHIP_ID4_REG);
	int asv_group, hpm, ids, i;

	if (special_group) {
		if (!((chip_id3 >> EXYNOS5422_SG_BSIGN_OFFSET) & EXYNOS5422_SG_BSIGN_MASK))
			asv_group = ((chip_id3 >> EXYNOS5422_SG_A_OFFSET) & EXYNOS5422_SG_A_MASK)
					- ((chip_id3 >> EXYNOS5422_SG_B_OFFSET) & EXYNOS5422_SG_B_MASK);
		else
			asv_group = ((chip_id3 >> EXYNOS5422_SG_A_OFFSET) & EXYNOS5422_SG_A_MASK)
					+ ((chip_id3 >> EXYNOS5422_SG_B_OFFSET) & EXYNOS5422_SG_B_MASK);
	}
	else {
		hpm = (chip_id4 >> EXYNOS5422_TMCB_OFFSET) & EXYNOS5422_TMCB_MASK;
		ids = (chip_id3 >> EXYNOS5422_IDS_OFFSET) & EXYNOS5422_IDS_MASK;

		for (i = 0; i < EXYNOS5422_ASV_NR; i++) {
			if (ids <= ASV_REFER_TABLE[0][i])
				break;
			if (hpm <= ASV_REFER_TABLE[1][i])
				break;
		}
		if (i < EXYNOS5422_ASV_NR)
			asv_group = i;
		else
			asv_group = 0;
	}

	return	asv_group;
}

/*----------------------------------------------------------------------------*/
static int asv_table_check(void)
{
	unsigned int chip_id3 = __raw_readl(CHIP_ID3_REG);

	return	(chip_id3 >> EXYNOS5422_TABLE_OFFSET) & EXYNOS5422_TABLE_MASK;
}

/*----------------------------------------------------------------------------*/
static int exynos5422_offset_volt(unsigned int value)
{
	switch(value) {
		default :
		case	0:	return 0;
		case	1:	return 12500;
		case	2:	return 50000;
		case	3:	return 25000;
	}
}

/*----------------------------------------------------------------------------*/
static void asv_offset_volt_setup(void)
{
	unsigned int chip_id4 = __raw_readl(CHIP_ID4_REG);
	unsigned int value;

	if (exynos5422_asv->is_bin2) {
		exynos5422_asv->device[DEVICE_ID_ARM].dvfs_nr = ARM_BIN2_DVFS_NR;
		exynos5422_asv->device[DEVICE_ID_KFC].dvfs_nr = KFC_BIN2_DVFS_NR;
	}
	else {
		exynos5422_asv->device[DEVICE_ID_ARM].dvfs_nr = ARM_DVFS_NR;
		exynos5422_asv->device[DEVICE_ID_KFC].dvfs_nr = KFC_DVFS_NR;
	}

	/* ARM Offset Volt setup */
	exynos5422_asv->device[DEVICE_ID_ARM].asv_func = arm_asv_func;
	exynos5422_asv->device[DEVICE_ID_ARM].base_volt = 1000000;

	value = (chip_id4 >> EXYNOS5422_ARM_UP_OFFSET) & EXYNOS5422_ARM_UP_MASK;
	exynos5422_asv->device[DEVICE_ID_ARM].offset_volt_h =
		exynos5422_offset_volt(value);

	value = (chip_id4 >> EXYNOS5422_ARM_DN_OFFSET) & EXYNOS5422_ARM_DN_MASK;
	exynos5422_asv->device[DEVICE_ID_ARM].offset_volt_l =
		exynos5422_offset_volt(value);

	/* KFC Offset Volt setup */
	exynos5422_asv->device[DEVICE_ID_KFC].asv_func = kfc_asv_func;
	exynos5422_asv->device[DEVICE_ID_KFC].base_volt = 1000000;

	value = (chip_id4 >> EXYNOS5422_KFC_UP_OFFSET) & EXYNOS5422_KFC_UP_MASK;
	exynos5422_asv->device[DEVICE_ID_KFC].offset_volt_h =
		exynos5422_offset_volt(value);

	value = (chip_id4 >> EXYNOS5422_KFC_DN_OFFSET) & EXYNOS5422_KFC_DN_MASK;
	exynos5422_asv->device[DEVICE_ID_KFC].offset_volt_l =
		exynos5422_offset_volt(value);
}

/*----------------------------------------------------------------------------*/
static int __init exynos5422_asv_init(void)
{
	exynos5422_asv = kmalloc(sizeof(struct exynos_asv), GFP_KERNEL);

	if (!exynos5422_asv) {
		pr_err("%s : memory allocation error!\n", __func__);
		return -ENOMEM;
	}

	exynos5422_asv->is_odroid = board_check();

	if (exynos5422_asv->is_odroid) {
		exynos5422_asv->is_bin2 = bin2_check();

		exynos5422_asv->is_special_lot =
			asv_special_group_check();

		exynos5422_asv->group =
			asv_group_check(exynos5422_asv->is_special_lot);

		exynos5422_asv->table =
			asv_table_check();

		asv_offset_volt_setup();
	}

#if 0	// DEBUG Message
{
	unsigned int chip_id3 = __raw_readl(CHIP_ID3_REG);
	unsigned int chip_id4 = __raw_readl(CHIP_ID4_REG);

	pr_err("======================================================================\n");
	pr_err("DEBUG FOR ASV TABLE\n");
	pr_err("======================================================================\n");
	printk("charles(%s) : CHIP_ID3_REG = 0x%08X(0x%08X)\n",__func__, CHIP_ID3_REG, chip_id3);
	printk("charles(%s) : CHIP_ID4_REG = 0x%08X(0x%08X)\n",__func__, CHIP_ID4_REG, chip_id4);

	pr_err("charles(%s) : is_odroid = %d\n", __func__, exynos5422_asv->is_odroid);
	pr_err("charles(%s) : is_bin2 = %d\n", __func__,  exynos5422_asv->is_bin2);
	pr_err("charles(%s) : is_special_lot = %d\n", __func__,  exynos5422_asv->is_special_lot);
	pr_err("charles(%s) : asv_group_no = %d\n", __func__,  exynos5422_asv->group);
	pr_err("charles(%s) : asv_table_no = %d\n", __func__,  exynos5422_asv->table);

	pr_err("charles(%s) : ARM offset Volt H = %d\n",
		exynos5422_asv->device[DEVICE_ID_ARM].offset_volt_h);

	pr_err("charles(%s) : ARM offset Volt L = %d\n",
		exynos5422_asv->device[DEVICE_ID_ARM].offset_volt_l);

	pr_err("charles(%s) : KFC offset Volt H = %d\n",
		exynos5422_asv->device[DEVICE_ID_KFC].offset_volt_h);

	pr_err("charles(%s) : KFC offset Volt L = %d\n",
		exynos5422_asv->device[DEVICE_ID_KFC].offset_volt_l);

	pr_err("======================================================================\n");
	pr_err("======================================================================\n");
}
#endif
	return	0;
}
arch_initcall_sync(exynos5422_asv_init);
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
