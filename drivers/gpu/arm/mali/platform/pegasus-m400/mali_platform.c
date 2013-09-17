/*
 * Copyright (C) 2010 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_platform.c
 * Platform specific Mali driver functions for a default platform
 */
#include <linux/version.h>
#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "mali_platform.h"
//#include "mali_linux_pm.h"

#ifdef USING_MALI_PMM
#include "mali_pmm.h"
#endif

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>


#include <asm/io.h>
#include <mach/regs-pmu.h>

#define EXTXTALCLK_NAME 	"ext_xtal"
#define VPLLSRCCLK_NAME 	"vpll_src"
#define FOUTVPLLCLK_NAME	"fout_vpll"
#define SCLVPLLCLK_NAME 	"sclk_vpll"
#define GPUMOUT1CLK_NAME	"mout_g3d1"

#define MPLLCLK_NAME 		"mout_mpll"
#define GPUMOUT0CLK_NAME 	"mout_g3d0"
#define GPUCLK_NAME 		"sclk_g3d"
#define CLK_DIV_STAT_G3D 	0x1003C62C
#define CLK_DESC 		"clk-divider-status"

typedef struct mali_runtime_resumeTag{
	int clk;
	int vol;
}mali_runtime_resume_table;

mali_runtime_resume_table mali_runtime_resume = {400, 1100000};

static struct clk  *ext_xtal_clock = 0;
static struct clk  *vpll_src_clock = 0;
static struct clk  *fout_vpll_clock = 0;
static struct clk  *sclk_vpll_clock = 0;

static struct clk  *mpll_clock = 0;
static struct clk  *mali_parent_clock = 0;
static struct clk  *mali_clock = 0;


static unsigned int GPU_MHZ	= 1000000;

/* Please take special care lowering these values, specially the voltage
 * as it can cause system stability problems: random oops, usb hub resets */
int mali_gpu_clk = 533; /* 533 MHz */
int mali_gpu_vol = 1125000;/* 1.1125 V */

#ifdef CONFIG_MALI_DVFS
#define MALI_DVFS_DEFAULT_STEP 0
#endif

int  gpu_power_state;
static int bPoweroff;

#ifdef CONFIG_REGULATOR
struct regulator {
	struct device *dev;
	struct list_head list;
	unsigned int always_on:1;
	int uA_load;
	int min_uV;
	int max_uV;
	char *supply_name;
	struct device_attribute dev_attr;
	struct regulator_dev *rdev;
	struct dentry *debugfs;
};
struct regulator *g3d_regulator = NULL;
#endif


mali_io_address clk_register_map=0;

_mali_osk_lock_t *mali_dvfs_lock = 0;

void mali_set_runtime_resume_params(int clk, int volt)
{
	mali_runtime_resume.clk = clk;
	mali_runtime_resume.vol = volt;
}

#ifdef CONFIG_REGULATOR
int mali_regulator_get_usecount(void)
{
	struct regulator_dev *rdev;

	if( IS_ERR_OR_NULL(g3d_regulator) )
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_get_usecount : g3d_regulator is null\n"));
		return 0;
	}
	rdev = g3d_regulator->rdev;
	return rdev->use_count;
}

void mali_regulator_disable(void)
{
	bPoweroff = 1;
	if( IS_ERR_OR_NULL(g3d_regulator) )
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_disable : g3d_regulator is null\n"));
		return;
	}
	regulator_disable(g3d_regulator);
	MALI_DEBUG_PRINT(1, ("regulator_disable -> use cnt: %d \n",mali_regulator_get_usecount()));
}

void mali_regulator_enable(void)
{
	bPoweroff = 0;
	if( IS_ERR_OR_NULL(g3d_regulator) )
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_enable : g3d_regulator is null\n"));
		return;
	}
	regulator_enable(g3d_regulator);
	MALI_DEBUG_PRINT(1, ("regulator_enable -> use cnt: %d \n",mali_regulator_get_usecount()));
}

void mali_regulator_set_voltage(int min_uV, int max_uV)
{
	int voltage;
#ifndef CONFIG_MALI_DVFS
	min_uV = mali_gpu_vol;
	max_uV = mali_gpu_vol;
#endif

	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);

	if( IS_ERR_OR_NULL(g3d_regulator) )
	{
		MALI_DEBUG_PRINT(1, ("error on mali_regulator_set_voltage : g3d_regulator is null\n"));
		return;
	}
	MALI_DEBUG_PRINT(2, ("= regulator_set_voltage: %d, %d \n",min_uV, max_uV));
	regulator_set_voltage(g3d_regulator,min_uV,max_uV);
	voltage = regulator_get_voltage(g3d_regulator);
	mali_gpu_vol = voltage;
	MALI_DEBUG_PRINT(1, ("= regulator_get_voltage: %d \n",mali_gpu_vol));

	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);
}
#endif

unsigned long mali_clk_get_rate(void)
{
	return clk_get_rate(mali_clock);
}

mali_bool mali_clk_get(mali_bool bis_vpll)
{
	if (bis_vpll == MALI_TRUE)
	{
		if (ext_xtal_clock == NULL)
		{
			ext_xtal_clock = clk_get(NULL,EXTXTALCLK_NAME);
			if (IS_ERR(ext_xtal_clock)) {
				MALI_PRINT( ("MALI Error : failed to get source ext_xtal_clock\n"));
				return MALI_FALSE;
			}
		}

		if (vpll_src_clock == NULL)
		{
			vpll_src_clock = clk_get(NULL,VPLLSRCCLK_NAME);
			if (IS_ERR(vpll_src_clock)) {
				MALI_PRINT( ("MALI Error : failed to get source vpll_src_clock\n"));
				return MALI_FALSE;
			}
		}

		if (fout_vpll_clock == NULL)
		{
			fout_vpll_clock = clk_get(NULL,FOUTVPLLCLK_NAME);
			if (IS_ERR(fout_vpll_clock)) {
				MALI_PRINT( ("MALI Error : failed to get source fout_vpll_clock\n"));
				return MALI_FALSE;
			}
		}

		if (sclk_vpll_clock == NULL)
		{
			sclk_vpll_clock = clk_get(NULL,SCLVPLLCLK_NAME);
			if (IS_ERR(sclk_vpll_clock)) {
				MALI_PRINT( ("MALI Error : failed to get source sclk_vpll_clock\n"));
				return MALI_FALSE;
			}
		}

		if (mali_parent_clock == NULL)
		{
			mali_parent_clock = clk_get(NULL, GPUMOUT1CLK_NAME);

			if (IS_ERR(mali_parent_clock)) {
				MALI_PRINT( ( "MALI Error : failed to get source mali parent clock\n"));
				return MALI_FALSE;
			}
		}
	}
	else // mpll
	{
		if (mpll_clock == NULL)
		{
			mpll_clock = clk_get(NULL,MPLLCLK_NAME);

			if (IS_ERR(mpll_clock)) {
				MALI_PRINT( ("MALI Error : failed to get source mpll clock\n"));
				return MALI_FALSE;
			}
		}

		if (mali_parent_clock == NULL)
		{
			mali_parent_clock = clk_get(NULL, GPUMOUT0CLK_NAME);

			if (IS_ERR(mali_parent_clock)) {
				MALI_PRINT( ( "MALI Error : failed to get source mali parent clock\n"));
				return MALI_FALSE;
			}
		}
	}

	// mali clock get always.
	if (mali_clock == NULL)
	{
		mali_clock = clk_get(NULL, GPUCLK_NAME);

		if (IS_ERR(mali_clock)) {
			MALI_PRINT( ("MALI Error : failed to get source mali clock\n"));
			return MALI_FALSE;
		}
	}

	return MALI_TRUE;
}

void mali_clk_put(mali_bool binc_mali_clock)
{
	if (mali_parent_clock)
	{
		clk_put(mali_parent_clock);
		mali_parent_clock = 0;
	}

	if (mpll_clock)
	{
		clk_put(mpll_clock);
		mpll_clock = 0;
	}

	if (sclk_vpll_clock)
	{
		clk_put(sclk_vpll_clock);
		sclk_vpll_clock = 0;
	}

	if (fout_vpll_clock)
	{
		clk_put(fout_vpll_clock);
		fout_vpll_clock = 0;
	}

	if (vpll_src_clock)
	{
		clk_put(vpll_src_clock);
		vpll_src_clock = 0;
	}

	if (ext_xtal_clock)
	{
		clk_put(ext_xtal_clock);
		ext_xtal_clock = 0;
	}

	if (binc_mali_clock == MALI_TRUE && mali_clock)
	{
		clk_put(mali_clock);
		mali_clock = 0;
	}

}


mali_bool mali_clk_set_rate(unsigned int clk, unsigned int mhz)
{
	unsigned long rate = 0;
	mali_bool bis_vpll = MALI_TRUE;

#ifndef CONFIG_VPLL_USE_FOR_TVENC
	bis_vpll = MALI_TRUE;
#endif

#ifndef CONFIG_MALI_DVFS
	clk = mali_gpu_clk;
#endif

	_mali_osk_lock_wait(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);

	if (mali_clk_get(bis_vpll) == MALI_FALSE)
	{
		printk("~~~~~~~~ERROR: [%s] %d\n ",__func__,__LINE__);
		return MALI_FALSE;
	}

	rate = (unsigned long)clk * (unsigned long)mhz;
	MALI_DEBUG_PRINT(2,("= clk_set_rate : %d , %d \n",clk, mhz ));

	if (bis_vpll)
	{
		clk_set_rate(fout_vpll_clock, (unsigned int)clk * GPU_MHZ);
		//clk_set_parent(vpll_src_clock, ext_xtal_clock);
		clk_set_parent(sclk_vpll_clock, fout_vpll_clock);

		clk_set_parent(mali_parent_clock, sclk_vpll_clock);
		clk_set_parent(mali_clock, mali_parent_clock);
	}
	else
	{
		clk_set_parent(mali_parent_clock, mpll_clock);
		clk_set_parent(mali_clock, mali_parent_clock);
	}

	if (clk_enable(mali_clock) < 0)
	{
		printk("~~~~~~~~ERROR: [%s] %d\n ",__func__,__LINE__);
		return MALI_FALSE;
	}

	clk_set_rate(mali_clock, rate);
	rate = clk_get_rate(mali_clock);

	if (bis_vpll)
		mali_gpu_clk = (int)(rate / mhz);
	else
		mali_gpu_clk = (int)((rate + 500000) / mhz);

	GPU_MHZ = mhz;
	MALI_DEBUG_PRINT(2,("= clk_get_rate: %d \n",mali_gpu_clk));

	mali_clk_put(MALI_FALSE);

	_mali_osk_lock_signal(mali_dvfs_lock, _MALI_OSK_LOCKMODE_RW);

	return MALI_TRUE;
}

static mali_bool init_mali_clock(void)
{
	mali_bool ret = MALI_TRUE;

	gpu_power_state = 0;

	if (mali_clock != 0)
		return ret; // already initialized

	mali_dvfs_lock = _mali_osk_lock_init(_MALI_OSK_LOCKFLAG_NONINTERRUPTABLE
			| _MALI_OSK_LOCKFLAG_ONELOCK, 0, 0);
	if (mali_dvfs_lock == NULL)
		return _MALI_OSK_ERR_FAULT;

	if (mali_clk_set_rate(mali_gpu_clk, GPU_MHZ) == MALI_FALSE)
	{
		ret = MALI_FALSE;
		goto err_clock_get;
	}

	MALI_PRINT(("init_mali_clock mali_clock %p \n", mali_clock));


#ifdef CONFIG_REGULATOR
#ifdef USING_MALI_PMM
	g3d_regulator = regulator_get(&mali_gpu_device.dev, "vdd_g3d");
#else
	g3d_regulator = regulator_get(NULL, "vdd_g3d");
#endif

	if (IS_ERR(g3d_regulator))
	{
		MALI_PRINT( ("MALI Error : failed to get vdd_g3d\n"));
		ret = MALI_FALSE;
		goto err_regulator;
	}

	regulator_enable(g3d_regulator);
	MALI_DEBUG_PRINT(1, ("= regulator_enable -> use cnt: %d \n",mali_regulator_get_usecount()));
	mali_regulator_set_voltage(mali_gpu_vol, mali_gpu_vol);
#endif

	MALI_DEBUG_PRINT(2, ("MALI Clock is set at mali driver\n"));


	MALI_DEBUG_PRINT(3,("::clk_put:: %s mali_parent_clock - normal\n", __FUNCTION__));
	MALI_DEBUG_PRINT(3,("::clk_put:: %s mpll_clock  - normal\n", __FUNCTION__));

	mali_clk_put(MALI_FALSE);

	return MALI_TRUE;


#ifdef CONFIG_REGULATOR
err_regulator:
	regulator_put(g3d_regulator);
#endif

err_clock_get:
	mali_clk_put(MALI_TRUE);

	return ret;
}

static mali_bool deinit_mali_clock(void)
{
	if (mali_clock == 0)
		return MALI_TRUE;

#ifdef CONFIG_REGULATOR
	if (g3d_regulator)
	{
		regulator_put(g3d_regulator);
		g3d_regulator=NULL;
	}
#endif

	mali_clk_put(MALI_TRUE);

	return MALI_TRUE;
}

_mali_osk_errcode_t mali_platform_init()
{
	MALI_CHECK(init_mali_clock(), _MALI_OSK_ERR_FAULT);
#if CONFIG_MALI_DVFS
	if (!clk_register_map) clk_register_map = _mali_osk_mem_mapioregion( CLK_DIV_STAT_G3D, 0x20, CLK_DESC );
	if(!init_mali_dvfs_status(MALI_DVFS_DEFAULT_STEP))
		MALI_DEBUG_PRINT(1, ("mali_platform_init failed\n"));
#endif

	MALI_SUCCESS;
}

_mali_osk_errcode_t mali_platform_deinit()
{
	deinit_mali_clock();

#ifdef CONFIG_MALI_DVFS
	deinit_mali_dvfs_status();
	if (clk_register_map )
	{
		_mali_osk_mem_unmapioregion(CLK_DIV_STAT_G3D, 0x20, clk_register_map);
		clk_register_map=0;
	}
#endif

	MALI_SUCCESS;
}

void mali_gpu_utilization_handler(u32 utilization)
{
	if (bPoweroff==0)
	{
#ifdef CONFIG_MALI_DVFS
		if(!mali_dvfs_handler(utilization))
			MALI_DEBUG_PRINT(1,( "error on mali dvfs status in utilization\n"));
#endif
	}
}
