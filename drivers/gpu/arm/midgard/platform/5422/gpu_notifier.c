/* drivers/gpu/t6xx/kbase/src/platform/gpu_notifier.c
 *
 * Copyright 2011 by S.LSI. Samsung Electronics Inc.
 * San#24, Nongseo-Dong, Giheung-Gu, Yongin, Korea
 *
 * Samsung SoC Mali-T604 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file gpu_notifier.c
 */

#include <mali_kbase.h>

#include <linux/suspend.h>
#include <linux/pm_runtime.h>

#include "mali_kbase_platform.h"
#include "gpu_dvfs_handler.h"
#include "gpu_notifier.h"
#include "gpu_control.h"

#if defined(CONFIG_EXYNOS_THERMAL)
#include <mach/tmu.h>
#endif /* CONFIG_EXYNOS_THERMAL */

extern struct kbase_device *pkbdev;

#if defined(CONFIG_EXYNOS_THERMAL)
static int gpu_tmu_hot_check_and_work(struct kbase_device *kbdev, unsigned long event)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct exynos_context *platform;
	int lock_clock;

	KBASE_DEBUG_ASSERT(kbdev != NULL);

	platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	switch (event) {
	case GPU_THROTTLING1:
		lock_clock = GPU_THROTTLING_90_95;
		GPU_LOG(DVFS_INFO, "[G3D] GPU_THROTTLING_90_95\n");
		break;
	case GPU_THROTTLING2:
		lock_clock = GPU_THROTTLING_95_100;
		GPU_LOG(DVFS_INFO, "[G3D] GPU_THROTTLING_95_100\n");
		break;
	case GPU_THROTTLING3:
		lock_clock = GPU_THROTTLING_100_105;
		GPU_LOG(DVFS_INFO, "[G3D] GPU_THROTTLING_100_105\n");
		break;
	case GPU_THROTTLING4:
		lock_clock = GPU_THROTTLING_105_110;
		GPU_LOG(DVFS_INFO, "[G3D] GPU_THROTTLING_105_110\n");
		break;
	case GPU_TRIPPING:
		lock_clock = GPU_TRIPPING_110;
		GPU_LOG(DVFS_INFO, "[G3D] GPU_THROTTLING_110\n");
		break;
	default:
		GPU_LOG(DVFS_ERROR, "[G3D] Wrong event, %lu,  in the kbase_tmu_hot_check_and_work function\n", event);
		return 0;
	}

	platform->target_lock_type = TMU_LOCK;
	gpu_dvfs_handler_control(kbdev, GPU_HANDLER_DVFS_MAX_LOCK, lock_clock);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	return 0;
}

static void gpu_tmu_normal_work(struct kbase_device *kbdev)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct exynos_context *platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return;

	platform->target_lock_type = TMU_LOCK;
	gpu_dvfs_handler_control(kbdev, GPU_HANDLER_DVFS_MAX_UNLOCK, 0);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
}

static int gpu_tmu_notifier(struct notifier_block *notifier,
				unsigned long event, void *v)
{
	struct exynos_context *platform = (struct exynos_context *)pkbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if (!platform->tmu_status)
		return NOTIFY_OK;

	platform->voltage_margin = 0;

	if (event == GPU_COLD) {
		platform->voltage_margin = VOLTAGE_OFFSET_MARGIN;
	} else if (event == GPU_NORMAL) {
		gpu_tmu_normal_work(pkbdev);
	} else if (event >= GPU_THROTTLING1 && event <= GPU_TRIPPING) {
		if (gpu_tmu_hot_check_and_work(pkbdev, event))
			GPU_LOG(DVFS_ERROR, "failed to open device");
	}

	gpu_control_state_set(pkbdev, GPU_CONTROL_SET_MARGIN, 0);

	return NOTIFY_OK;
}

static struct notifier_block gpu_tmu_nb = {
	.notifier_call = gpu_tmu_notifier,
};
#endif /* CONFIG_EXYNOS_THERMAL */

#ifdef CONFIG_MALI_MIDGARD_RT_PM
static int gpu_pm_notifier(struct notifier_block *nb, unsigned long event, void *cmd)
{
	return NOTIFY_OK;
}

static int gpu_power_on(struct kbase_device *kbdev)
{
	int ret_val;

	if (pm_runtime_status_suspended(kbdev->dev))
		ret_val = 1;
	else
		ret_val = 0;

	pm_runtime_resume(kbdev->dev);

	return ret_val;
}

static void gpu_power_off(struct kbase_device *kbdev)
{
	pm_schedule_suspend(kbdev->dev, RUNTIME_PM_DELAY_TIME);
}

static struct notifier_block gpu_pm_nb = {
	.notifier_call = gpu_pm_notifier
};

static mali_error gpu_device_runtime_init(struct kbase_device *kbdev)
{
	pm_suspend_ignore_children(kbdev->dev, true);
	return MALI_ERROR_NONE;
}

static void gpu_device_runtime_disable(struct kbase_device *kbdev)
{
	pm_runtime_disable(kbdev->dev);
}

static int pm_callback_runtime_on(struct kbase_device *kbdev)
{
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	GPU_LOG(DVFS_INFO, "g3d turn on\n");

#ifdef CONFIG_MALI_MIDGARD_DVFS
	gpu_control_state_set(kbdev, GPU_CONTROL_PREPARE_ON, 0);
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	gpu_control_state_set(kbdev, GPU_CONTROL_CLOCK_ON, 0);
	gpu_control_state_set(kbdev, GPU_CONTROL_CHANGE_CLK_VOL, platform->cur_clock);

	return 0;
}

static void pm_callback_runtime_off(struct kbase_device *kbdev)
{
	GPU_LOG(DVFS_INFO, "g3d turn off\n");
	gpu_control_state_set(kbdev, GPU_CONTROL_CLOCK_OFF, 0);
}

kbase_pm_callback_conf pm_callbacks = {
	.power_on_callback = gpu_power_on,
	.power_off_callback = gpu_power_off,
#ifdef CONFIG_PM_RUNTIME
	.power_runtime_init_callback = gpu_device_runtime_init,
	.power_runtime_term_callback = gpu_device_runtime_disable,
	.power_runtime_on_callback = pm_callback_runtime_on,
	.power_runtime_off_callback = pm_callback_runtime_off,
#else /* CONFIG_PM_RUNTIME */
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_off_callback = NULL,
#endif /* CONFIG_PM_RUNTIME */
};
#endif /* CONFIG_MALI_MIDGARD_RT_PM */

int gpu_notifier_init(struct kbase_device *kbdev)
{
	struct exynos_context *platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	platform->voltage_margin = 0;
#if defined(CONFIG_EXYNOS_THERMAL)
	exynos_gpu_add_notifier(&gpu_tmu_nb);
	platform->tmu_status = true;
#else /* CONFIG_EXYNOS_THERMAL */
	platform->tmu_status = false;
#endif /* CONFIG_EXYNOS_THERMAL */

#ifdef CONFIG_MALI_MIDGARD_RT_PM
	if (register_pm_notifier(&gpu_pm_nb))
		return MALI_FALSE;
#endif /* CONFIG_MALI_MIDGARD_RT_PM */

	pm_runtime_enable(kbdev->dev);

	return MALI_TRUE;
}

void gpu_notifier_term(void)
{
#ifdef CONFIG_MALI_MIDGARD_RT_PM
	unregister_pm_notifier(&gpu_pm_nb);
#endif /* CONFIG_MALI_MIDGARD_RT_PM */
	return;
}
