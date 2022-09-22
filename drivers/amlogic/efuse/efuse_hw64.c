/*
 * drivers/amlogic/efuse/efuse_hw64.c
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
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/amlogic/iomap.h>
#include "efuse.h"
#ifdef CONFIG_ARM64
#include <linux/amlogic/efuse.h>
#endif
#include <linux/amlogic/secmon.h>
#include <linux/arm-smccc.h>
#include <asm/cacheflush.h>
#ifdef CONFIG_AMLOGIC_CPU_INFO
#include <linux/amlogic/cpu_version.h>
#endif


static long meson64_efuse_fn_smc(struct efuse_hal_api_arg *arg)
{
	long ret;
	unsigned int cmd, offset, size;
	unsigned long *retcnt = (unsigned long *)(arg->retcnt);
	struct arm_smccc_res res;

	if (!sharemem_input_base || !sharemem_output_base)
		return -1;

	if (arg->cmd == EFUSE_HAL_API_READ)
		cmd = efuse_read_cmd;
	else
		cmd = efuse_write_cmd;
	offset = arg->offset;
	size = arg->size;
	sharemem_mutex_lock();
	if (arg->cmd == EFUSE_HAL_API_WRITE)
		memcpy((void *)sharemem_input_base,
			(const void *)arg->buffer, size);

	asm __volatile__("" : : : "memory");

	arm_smccc_smc(cmd, offset, size, 0, 0, 0, 0, 0, &res);
	ret = res.a0;
	*retcnt = res.a0;

	if ((arg->cmd == EFUSE_HAL_API_READ) && (ret != 0))
		memcpy((void *)arg->buffer,
			(const void *)sharemem_output_base, ret);
	sharemem_mutex_unlock();

	if (!ret)
		return -1;
	else
		return 0;
}

int meson64_trustzone_efuse(struct efuse_hal_api_arg *arg)
{
	int ret;
	struct cpumask org_cpumask;

	if (!arg)
		return -1;

	cpumask_copy(&org_cpumask, &current->cpus_allowed);
	set_cpus_allowed_ptr(current, cpumask_of(0));
	ret = meson64_efuse_fn_smc(arg);
	set_cpus_allowed_ptr(current, &org_cpumask);
	return ret;
}

unsigned long efuse_aml_sec_boot_check(unsigned long nType,
	unsigned long pBuffer,
	unsigned long nLength,
	unsigned long nOption)
{
	struct arm_smccc_res res;
	long sharemem_phy_base;

	sharemem_phy_base = get_secmon_phy_input_base();
	if ((!sharemem_input_base) || (!sharemem_phy_base))
		return -1;

	sharemem_mutex_lock();

	memcpy((void *)sharemem_input_base,
		(const void *)pBuffer, nLength);

	//__flush_dcache_area(sharemem_input_base, nLength);

	asm __volatile__("" : : : "memory");

	do {
		arm_smccc_smc((unsigned long)AML_DATA_PROCESS,
					(unsigned long)nType,
					(unsigned long)sharemem_phy_base,
					(unsigned long)nLength,
					(unsigned long)nOption,
					0, 0, 0, &res);
	} while (0);

	sharemem_mutex_unlock();

	return res.a0;
}

unsigned long efuse_amlogic_set(char *buf, size_t count)
{
	unsigned long ret;
	struct cpumask org_cpumask;

	cpumask_copy(&org_cpumask, &current->cpus_allowed);
	set_cpus_allowed_ptr(current, cpumask_of(0));

	ret = efuse_aml_sec_boot_check(AML_D_P_W_EFUSE_AMLOGIC,
		(unsigned long)buf, (unsigned long)count, 0);

	set_cpus_allowed_ptr(current, &org_cpumask);

	return ret;
}

ssize_t meson64_trustzone_efuse_get_max(struct efuse_hal_api_arg *arg)
{
	ssize_t ret;
	unsigned int cmd;
	struct arm_smccc_res res;

	if (arg->cmd == EFUSE_HAL_API_USER_MAX) {
		cmd = efuse_get_max_cmd;

		asm __volatile__("" : : : "memory");
		arm_smccc_smc(cmd, 0, 0, 0, 0, 0, 0, 0, &res);
		ret = res.a0;

		if (!ret)
			return -1;
		else
			return ret;
	} else {
		pr_err("%s: cmd error!!!\n", __func__);
		return -1;
	}
}

ssize_t efuse_get_max(void)
{
	struct efuse_hal_api_arg arg;
	int ret;
	struct cpumask org_cpumask;

	arg.cmd = EFUSE_HAL_API_USER_MAX;

	cpumask_copy(&org_cpumask, &current->cpus_allowed);
	set_cpus_allowed_ptr(current, cpumask_of(0));
	ret = meson64_trustzone_efuse_get_max(&arg);
	set_cpus_allowed_ptr(current, &org_cpumask);

	if (ret == 0) {
		pr_info("ERROR: can not get efuse user max bytes!!!\n");
		return -1;
	} else
		return ret;
}

ssize_t _efuse_read(char *buf, size_t count, loff_t *ppos)
{
	unsigned int pos = *ppos;

	struct efuse_hal_api_arg arg;
	unsigned long retcnt;
	int ret;

	arg.cmd = EFUSE_HAL_API_READ;
	arg.offset = pos;
	arg.size = count;
	arg.buffer = (unsigned long)buf;
	arg.retcnt = (unsigned long)&retcnt;
	ret = meson64_trustzone_efuse(&arg);
	if (ret == 0) {
		*ppos += retcnt;
		return retcnt;
	}
	pr_err("%s:%s:%d: read error!!!\n",
			   __FILE__, __func__, __LINE__);
	return ret;
}

ssize_t _efuse_write(const char *buf, size_t count, loff_t *ppos)
{
	unsigned int pos = *ppos;

	struct efuse_hal_api_arg arg;
	unsigned long retcnt;
	int ret;

	arg.cmd = EFUSE_HAL_API_WRITE;
	arg.offset = pos;
	arg.size = count;
	arg.buffer = (unsigned long)buf;
	arg.retcnt = (unsigned long)&retcnt;

	ret = meson64_trustzone_efuse(&arg);
	if (ret == 0) {
		*ppos = retcnt;
		return retcnt;
	}
	pr_err("%s:%s:%d: write error!!!\n",
			   __FILE__, __func__, __LINE__);
	return ret;
}

ssize_t efuse_read_usr(char *buf, size_t count, loff_t *ppos)
{
	char data[EFUSE_BYTES];
	char *pdata = NULL;
	ssize_t ret;
	loff_t pos;
#ifdef CONFIG_AMLOGIC_CPU_INFO
	unsigned char chipid[CHIPID_LEN];
	int i, j;
#endif

	if (count > EFUSE_BYTES)
		count = EFUSE_BYTES;
	memset(data, 0, count);

	pdata = data;
	pos = *ppos;
	ret = _efuse_read(pdata, count, (loff_t *)&pos);

#ifdef CONFIG_AMLOGIC_CPU_INFO
	if (strlen(pdata) == 0) {
		cpuinfo_get_chipid(chipid, CHIPID_LEN);
		for (i = 0, j = 0; i < 16; i++, j += 2)
			sprintf(data + j, "%02x", chipid[i]);
	}
#endif

	memcpy(buf, data, count);

	return ret;
}

ssize_t efuse_write_usr(char *buf, size_t count, loff_t *ppos)
{
	char data[EFUSE_BYTES];
	char *pdata = NULL;
	ssize_t ret;
	loff_t pos;

	if (count == 0) {
		pr_info("data length: 0 is error!\n");
		return -1;
	}
	if (count > EFUSE_BYTES)
		count = EFUSE_BYTES;

	memset(data, 0, EFUSE_BYTES);

	memcpy(data, buf, count);
	pdata = data;
	pos = *ppos;

	ret = _efuse_write(pdata, count, (loff_t *)&pos);

	return ret;
}
