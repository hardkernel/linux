/* drivers/gpu/t6xx/kbase/src/platform/gpu_control.c
 *
 * Copyright 2011 by S.LSI. Samsung Electronics Inc.
 * San#24, Nongseo-Dong, Giheung-Gu, Yongin, Korea
 *
 * Samsung SoC Mali-T604 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file gpu_control.c
 * DVFS
 */

#include <mali_kbase.h>

#include <linux/pm_qos.h>
#include <mach/bts.h>

#include "mali_kbase_platform.h"
#include "gpu_dvfs_handler.h"
#include "gpu_control.h"
#include "mach/asv-exynos.h"

struct kbase_device *pkbdev;

#ifdef CONFIG_MALI_MIDGARD_DVFS
static int gpu_pm_qos_command(struct exynos_context *platform, gpu_pmqos_state state);
#endif
static int gpu_set_clk_vol(struct kbase_device *kbdev, int clock, int voltage);

int gpu_control_state_set(struct kbase_device *kbdev, gpu_control_state state, int param)
{
	int ret = 0, voltage;
#ifdef CONFIG_MALI_MIDGARD_DVFS
	unsigned long flags;
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	mutex_lock(&platform->gpu_clock_lock);
	switch (state) {
	case GPU_CONTROL_CLOCK_ON:
		ret = gpu_clock_on(platform);
#ifdef CONFIG_MALI_MIDGARD_DVFS
		if (!kbdev->pm.metrics.timer_active) {
			spin_lock_irqsave(&kbdev->pm.metrics.lock, flags);
			kbdev->pm.metrics.timer_active = true;
			spin_unlock_irqrestore(&kbdev->pm.metrics.lock, flags);
			hrtimer_start(&kbdev->pm.metrics.timer, HR_TIMER_DELAY_MSEC(platform->polling_speed), HRTIMER_MODE_REL);
		}
		gpu_dvfs_handler_control(kbdev, GPU_HANDLER_UPDATE_TIME_IN_STATE, 0);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
		break;
	case GPU_CONTROL_CLOCK_OFF:
#ifdef CONFIG_MALI_MIDGARD_DVFS
		if (platform->dvfs_status && kbdev->pm.metrics.timer_active) {
			spin_lock_irqsave(&kbdev->pm.metrics.lock, flags);
			kbdev->pm.metrics.timer_active = false;
			spin_unlock_irqrestore(&kbdev->pm.metrics.lock, flags);
			hrtimer_cancel(&kbdev->pm.metrics.timer);
		}
		gpu_pm_qos_command(platform, GPU_CONTROL_PM_QOS_RESET);
		gpu_dvfs_handler_control(kbdev, GPU_HANDLER_UPDATE_TIME_IN_STATE, platform->cur_clock);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
		ret = gpu_clock_off(platform);
		break;
	case GPU_CONTROL_CHANGE_CLK_VOL:
		ret = gpu_set_clk_vol(kbdev, param, gpu_dvfs_handler_control(kbdev, GPU_HANDLER_DVFS_GET_VOLTAGE, param));
#ifdef CONFIG_MALI_MIDGARD_DVFS
		if (ret == 0) {
			ret = gpu_dvfs_handler_control(kbdev, GPU_HANDLER_DVFS_GET_LEVEL, platform->cur_clock);
			if (ret >= 0) {
				spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);
				platform->step = ret;
				spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			} else {
				GPU_LOG(DVFS_ERROR, "Invalid dvfs level returned [%d]\n", GPU_CONTROL_CHANGE_CLK_VOL);
			}
		}
		if (gpu_pm_qos_command(platform, GPU_CONTROL_PM_QOS_SET) < -1)
			GPU_LOG(DVFS_ERROR, "failed to set the PM_QOS\n");
#endif /* CONFIG_MALI_MIDGARD_DVFS */
		break;
	case GPU_CONTROL_PREPARE_ON:
#ifdef CONFIG_MALI_MIDGARD_DVFS
		spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);
		if ((platform->dvfs_status && platform->wakeup_lock) &&
				(platform->table[platform->step].clock < MALI_DVFS_START_FREQ))
			platform->cur_clock = MALI_DVFS_START_FREQ;

		if (platform->min_lock > 0)
			platform->cur_clock = MAX(platform->min_lock, platform->cur_clock);
		else if (platform->max_lock > 0)
			platform->cur_clock = MIN(platform->max_lock, platform->cur_clock);

		platform->down_requirement = platform->table[platform->step].stay_count;
		spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
		break;
	case GPU_CONTROL_IS_POWER_ON:
		ret = gpu_is_power_on();
		break;
	case GPU_CONTROL_SET_MARGIN:
		voltage = MAX(platform->table[platform->step].voltage + platform->voltage_margin, COLD_MINIMUM_VOL);
		gpu_set_voltage(platform, voltage);
		GPU_LOG(DVFS_DEBUG, "we set the voltage: %d\n", voltage);
		break;
	default:
		return -1;
	}
	mutex_unlock(&platform->gpu_clock_lock);

	return ret;
}

#ifdef CONFIG_MALI_MIDGARD_DVFS
#ifdef CONFIG_BUS_DEVFREQ
static struct pm_qos_request exynos5_g3d_mif_qos;
static struct pm_qos_request exynos5_g3d_int_qos;
static struct pm_qos_request exynos5_g3d_cpu_kfc_min_qos;
static struct pm_qos_request exynos5_g3d_cpu_egl_max_qos;
#endif /* CONFIG_BUS_DEVFREQ */
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#ifdef CONFIG_MALI_MIDGARD_DVFS
static int gpu_pm_qos_command(struct exynos_context *platform, gpu_pmqos_state state)
{
#ifdef CONFIG_BUS_DEVFREQ
	switch (state) {
	case GPU_CONTROL_PM_QOS_INIT:
		pm_qos_add_request(&exynos5_g3d_mif_qos, PM_QOS_BUS_THROUGHPUT, 0);
		pm_qos_add_request(&exynos5_g3d_int_qos, PM_QOS_DEVICE_THROUGHPUT, 0);
		pm_qos_add_request(&exynos5_g3d_cpu_kfc_min_qos, PM_QOS_KFC_FREQ_MIN, 0);
		pm_qos_add_request(&exynos5_g3d_cpu_egl_max_qos, PM_QOS_CPU_FREQ_MAX, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
		break;
	case GPU_CONTROL_PM_QOS_DEINIT:
		pm_qos_remove_request(&exynos5_g3d_mif_qos);
		pm_qos_remove_request(&exynos5_g3d_int_qos);
		pm_qos_remove_request(&exynos5_g3d_cpu_kfc_min_qos);
		pm_qos_remove_request(&exynos5_g3d_cpu_egl_max_qos);
		break;
	case GPU_CONTROL_PM_QOS_SET:
		if (platform->step < 0)
			return -1;
		pm_qos_update_request(&exynos5_g3d_mif_qos, platform->table[platform->step].mem_freq);
		pm_qos_update_request(&exynos5_g3d_int_qos, platform->table[platform->step].int_freq);
		pm_qos_update_request(&exynos5_g3d_cpu_kfc_min_qos, platform->table[platform->step].cpu_freq);
		pm_qos_update_request(&exynos5_g3d_cpu_egl_max_qos, platform->table[platform->step].cpu_max_freq);
		break;
	case GPU_CONTROL_PM_QOS_RESET:
		pm_qos_update_request(&exynos5_g3d_mif_qos, 0);
		pm_qos_update_request(&exynos5_g3d_int_qos, 0);
		pm_qos_update_request(&exynos5_g3d_cpu_kfc_min_qos, 0);
		pm_qos_update_request(&exynos5_g3d_cpu_egl_max_qos, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
	default:
		break;
	}
#endif /* CONFIG_BUS_DEVFREQ */
	return 0;
}
#endif /* CONFIG_MALI_MIDGARD_DVFS */

static int gpu_set_clk_vol(struct kbase_device *kbdev, int clock, int voltage)
{
	static int prev_clock = -1;
	struct exynos_context *platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if ((clock > platform->table[platform->table_size-1].clock) || (clock < platform->table[0].clock)) {
		GPU_LOG(DVFS_ERROR, "Mismatch clock error (%d)\n", clock);
		return -1;
	}

	if (platform->voltage_margin)
		voltage = MAX(voltage + platform->voltage_margin, COLD_MINIMUM_VOL);

	if (clock > prev_clock) {
		gpu_set_voltage(platform, voltage);
#ifdef CONFIG_DYNIMIC_ABB
        set_match_abb(ID_G3D, platform->devfreq_g3d_asv_abb[platform->step]);
#endif
		gpu_set_clock(platform, clock);
#if defined(CONFIG_EXYNOS5422_BTS)
		bts_scen_update(TYPE_G3D_FREQ, clock);
#endif /* CONFIG_EXYNOS5422_BTS */
	} else {
#if defined(CONFIG_EXYNOS5422_BTS)
		bts_scen_update(TYPE_G3D_FREQ, clock);
#endif /* CONFIG_EXYNOS5422_BTS */
		gpu_set_clock(platform, clock);
#ifdef CONFIG_DYNIMIC_ABB
        set_match_abb(ID_G3D, platform->devfreq_g3d_asv_abb[platform->step]);
#endif
		gpu_set_voltage(platform, voltage);
	}

	GPU_LOG(DVFS_INFO, "[G3D]clk[%d -> %d], vol[%d + %d]\n", prev_clock, clock, voltage, platform->voltage_margin);

	gpu_dvfs_handler_control(kbdev, GPU_HANDLER_UPDATE_TIME_IN_STATE, prev_clock);

	prev_clock = clock;

	return 0;
}

int gpu_control_module_init(struct kbase_device *kbdev)
{
	struct exynos_context *platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return -ENODEV;

#ifdef CONFIG_PM_RUNTIME
	platform->exynos_pm_domain = gpu_get_pm_domain(kbdev);
#endif /* CONFIG_PM_RUNTIME */

	pkbdev = kbdev;

	if (gpu_power_init(kbdev) < 0) {
		GPU_LOG(DVFS_ERROR, "failed to initialize g3d power\n");
		goto out;
	}

	if (gpu_clock_init(kbdev) < 0) {
		GPU_LOG(DVFS_ERROR, "failed to initialize g3d clock\n");
		goto out;
	}

#ifdef CONFIG_REGULATOR
	if (gpu_regulator_init(platform) < 0) {
		GPU_LOG(DVFS_ERROR, "failed to initialize g3d regulator\n");
		goto regulator_init_fail;
	}
#endif /* CONFIG_REGULATOR */

#ifdef CONFIG_MALI_MIDGARD_DVFS
	gpu_pm_qos_command(platform, GPU_CONTROL_PM_QOS_INIT);
#endif /* CONFIG_MALI_MIDGARD_DVFS */

	return 0;
#ifdef CONFIG_REGULATOR
regulator_init_fail:
	gpu_regulator_disable(platform);
#endif /* CONFIG_REGULATOR */
out:
	return -EPERM;
}

void gpu_control_module_term(struct kbase_device *kbdev)
{
	struct exynos_context *platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return;

#ifdef CONFIG_PM_RUNTIME
	platform->exynos_pm_domain = NULL;
#endif /* CONFIG_PM_RUNTIME */
#ifdef CONFIG_REGULATOR
	gpu_regulator_disable(platform);
#endif /* CONFIG_REGULATOR */

#ifdef CONFIG_MALI_MIDGARD_DVFS
	gpu_pm_qos_command(platform, GPU_CONTROL_PM_QOS_DEINIT);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
}

