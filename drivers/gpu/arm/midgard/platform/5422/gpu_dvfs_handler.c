/* drivers/gpu/t6xx/kbase/src/platform/gpu_dvfs_handler.c
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
 * @file gpu_dvfs_handler.c
 * DVFS
 */

#include <mali_kbase.h>

#include "mali_kbase_platform.h"
#include "gpu_control.h"
#include "gpu_dvfs_handler.h"
#include "gpu_dvfs_governor.h"
#ifdef CONFIG_CPU_THERMAL_IPA
#include "gpu_ipa.h"
#endif /* CONFIG_CPU_THERMAL_IPA */

extern struct kbase_device *pkbdev;

#ifdef CONFIG_CPU_THERMAL_IPA
int gpu_dvfs_get_clock(int level)
{
	struct kbase_device *kbdev = pkbdev;
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;

	if (!platform)
		return -ENODEV;

	return platform->table[level].clock;
}

int gpu_dvfs_get_step(void)
{
	struct kbase_device *kbdev = pkbdev;
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;

	if (!platform)
		return -ENODEV;

	return platform->table_size;
}
#endif /* CONFIG_CPU_THERMAL_IPA */

static int gpu_get_target_freq(void)
{
	int freq;
	struct kbase_device *kbdev = pkbdev;
	struct exynos_context *platform;

	platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

#ifdef CONFIG_MALI_MIDGARD_DVFS
	gpu_dvfs_decide_next_level(kbdev, platform->utilization);
#endif /* CONFIG_MALI_MIDGARD_DVFS */

	freq = platform->table[platform->step].clock;
#ifdef CONFIG_MALI_MIDGARD_DVFS
	if ((platform->max_lock > 0) && (freq > platform->max_lock))
		freq = platform->max_lock;
	else if ((platform->min_lock > 0) && (freq < platform->min_lock))
		freq = platform->min_lock;
#endif /* CONFIG_MALI_MIDGARD_DVFS */

	return freq;
}

static int gpu_set_target_freq(int freq)
{
	struct kbase_device *kbdev = pkbdev;
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	gpu_control_state_set(kbdev, GPU_CONTROL_CHANGE_CLK_VOL, freq);

	return 0;
}

static void gpu_dvfs_event_proc(struct work_struct *q)
{
	int freq = 0;
	struct kbase_device *kbdev = pkbdev;
	struct exynos_context *platform;

	platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return;

	mutex_lock(&platform->gpu_dvfs_handler_lock);
	if (gpu_control_state_set(kbdev, GPU_CONTROL_IS_POWER_ON, 0)) {
		freq = gpu_get_target_freq();
		gpu_set_target_freq(freq);
	}
	mutex_unlock(&platform->gpu_dvfs_handler_lock);
}
static DECLARE_WORK(gpu_dvfs_work, gpu_dvfs_event_proc);

int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation, u32 util_gl_share, u32 util_cl_share[2])
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	unsigned long flags;
#endif /* CONFIG_MALI_MIDGARD_DVFS */
    struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform) {
		GPU_LOG(DVFS_ERROR, "platform context is not initialized\n");
		return -ENODEV;
	}

#ifdef CONFIG_MALI_MIDGARD_DVFS
	spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);
#ifdef CONFIG_CPU_THERMAL_IPA
	if (platform->time_tick < GPU_DVFS_TIME_INTERVAL) {
		platform->time_tick++;
		platform->time_busy += kbdev->pm.metrics.time_busy;
		platform->time_idle += kbdev->pm.metrics.time_idle;
	} else {
		platform->time_busy = kbdev->pm.metrics.time_busy;
		platform->time_idle = kbdev->pm.metrics.time_idle;
		platform->time_tick = 0;
	}
#endif /* CONFIG_CPU_THERMAL_IPA */

	platform->utilization = utilisation;
#ifdef CONFIG_CPU_THERMAL_IPA
	gpu_ipa_dvfs_calc_norm_utilisation(kbdev);
#endif /* CONFIG_CPU_THERMAL_IPA */
	spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#if defined(SLSI_INTEGRATION) && defined(CL_UTILIZATION_BOOST_BY_TIME_WEIGHT)
	atomic_set(&kbdev->pm.metrics.time_compute_jobs, 0);
	atomic_set(&kbdev->pm.metrics.time_vertex_jobs, 0);
	atomic_set(&kbdev->pm.metrics.time_fragment_jobs, 0);
#endif

	if (platform->dvfs_wq)
		queue_work_on(0, platform->dvfs_wq, &gpu_dvfs_work);

	GPU_LOG(DVFS_DEBUG, "[G3D] dvfs hanlder is called\n");

	return 0;
}

int gpu_dvfs_handler_init(struct kbase_device *kbdev)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if (!platform->dvfs_wq)
		platform->dvfs_wq = create_singlethread_workqueue("g3d_dvfs");

	if (!platform->dvfs_status)
		platform->dvfs_status = true;
#ifdef CONFIG_CPU_THERMAL_IPA
	platform->time_tick = 0;
	platform->time_busy = 0;
	platform->time_idle = 0;
#endif /* CONFIG_CPU_THERMAL_IPA */

	GPU_LOG(DVFS_INFO, "g3d dvfs handler initialized\n");
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	return 0;
}

int gpu_dvfs_handler_deinit(struct kbase_device *kbdev)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct exynos_context *platform = (struct exynos_context *) kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if (platform->dvfs_wq)
		destroy_workqueue(platform->dvfs_wq);
	platform->dvfs_wq = NULL;

	if (platform->dvfs_status)
		platform->dvfs_status = false;

	GPU_LOG(DVFS_INFO, "g3d dvfs handler de-initialized\n");
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	return 0;
}

#ifdef CONFIG_MALI_MIDGARD_DVFS
static int gpu_dvfs_on_off(struct kbase_device *kbdev, bool enable)
{
	unsigned long flags;
	struct exynos_context *platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	if (enable && !platform->dvfs_status) {
		platform->dvfs_status = true;
		gpu_control_state_set(kbdev, GPU_CONTROL_CHANGE_CLK_VOL, platform->cur_clock);
		gpu_dvfs_handler_init(kbdev);
		if (!kbdev->pm.metrics.timer_active) {
			spin_lock_irqsave(&kbdev->pm.metrics.lock, flags);
			kbdev->pm.metrics.timer_active = true;
			spin_unlock_irqrestore(&kbdev->pm.metrics.lock, flags);
			hrtimer_start(&kbdev->pm.metrics.timer, HR_TIMER_DELAY_MSEC(platform->polling_speed), HRTIMER_MODE_REL);
		}
	} else if (!enable && platform->dvfs_status) {
		platform->dvfs_status = false;
		gpu_dvfs_handler_deinit(kbdev);
		gpu_control_state_set(kbdev, GPU_CONTROL_CHANGE_CLK_VOL, MALI_DVFS_BL_CONFIG_FREQ);
		if (kbdev->pm.metrics.timer_active) {
			spin_lock_irqsave(&kbdev->pm.metrics.lock, flags);
			kbdev->pm.metrics.timer_active = false;
			spin_unlock_irqrestore(&kbdev->pm.metrics.lock, flags);
			hrtimer_cancel(&kbdev->pm.metrics.timer);
		}
	} else {
		GPU_LOG(DVFS_WARNING, "impossible state to change dvfs status (current: %d, request: %d)\n",
				platform->dvfs_status, enable);
		return -1;
	}
	return 0;
}
#endif /* CONFIG_MALI_MIDGARD_DVFS */

int gpu_dvfs_handler_control(struct kbase_device *kbdev, gpu_dvfs_handler_command command, int param)
{
	int ret = 0;
#ifdef CONFIG_MALI_MIDGARD_DVFS
	int i;
	bool dirty = false;
	unsigned long flags;
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	struct exynos_context *platform;

	platform = (struct exynos_context *)kbdev->platform_context;
	if (!platform)
		return -ENODEV;

	switch (command) {
#ifdef CONFIG_MALI_MIDGARD_DVFS
	case GPU_HANDLER_DVFS_ON:
		mutex_lock(&platform->gpu_dvfs_handler_lock);
		gpu_dvfs_on_off(kbdev, true);
		mutex_unlock(&platform->gpu_dvfs_handler_lock);
		break;
	case GPU_HANDLER_DVFS_OFF:
		mutex_lock(&platform->gpu_dvfs_handler_lock);
		gpu_dvfs_on_off(kbdev, false);
		mutex_unlock(&platform->gpu_dvfs_handler_lock);
		break;
	case GPU_HANDLER_DVFS_GOVERNOR_CHANGE:
		mutex_lock(&platform->gpu_dvfs_handler_lock);
		gpu_dvfs_on_off(kbdev, false);
		gpu_dvfs_governor_init(kbdev, param);
		gpu_dvfs_on_off(kbdev, true);
		mutex_unlock(&platform->gpu_dvfs_handler_lock);
		break;
	case GPU_HANDLER_DVFS_MAX_LOCK:
		spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);
		if ((platform->min_lock >= 0) && (param < platform->min_lock)) {
			spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			GPU_LOG(DVFS_WARNING, "[G3D] max lock Error: lock is smaller than min lock\n");
			return -1;
		}

		if ((platform->target_lock_type < TMU_LOCK) || (platform->target_lock_type >= NUMBER_LOCK)) {
			spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			return -1;
		}

		platform->user_max_lock[platform->target_lock_type] = param;
		platform->max_lock = param;

		if (platform->max_lock > 0) {
			for (i = 0; i < NUMBER_LOCK; i++) {
				if (platform->user_max_lock[i] > 0)
					platform->max_lock = MIN(platform->max_lock, platform->user_max_lock[i]);
			}
		} else {
			platform->max_lock = param;
		}

		spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);

		if ((platform->max_lock > 0) && (platform->cur_clock > platform->max_lock))
			gpu_control_state_set(kbdev, GPU_CONTROL_CHANGE_CLK_VOL, platform->max_lock);

		GPU_LOG(DVFS_DEBUG, "[G3D] Lock max clk[%d], user lock[%d], current clk[%d]\n", platform->max_lock,
				platform->user_min_lock[platform->target_lock_type], platform->cur_clock);

		platform->target_lock_type = -1;
		break;
	case GPU_HANDLER_DVFS_MIN_LOCK:
		spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);
		if ((platform->max_lock > 0) && (param > platform->max_lock)) {
			spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			GPU_LOG(DVFS_WARNING, "min lock Error: the lock is larger than max lock\n");
			return -1;
		}

		if ((platform->target_lock_type < TMU_LOCK) || (platform->target_lock_type >= NUMBER_LOCK)) {
			spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			return -1;
		}

		platform->user_min_lock[platform->target_lock_type] = param;
		platform->min_lock = param;

		if (platform->min_lock > 0) {
			for (i = 0; i < NUMBER_LOCK; i++) {
				if (platform->user_min_lock[i] > 0)
					platform->min_lock = MAX(platform->min_lock, platform->user_min_lock[i]);
			}
		} else {
			platform->min_lock = param;
		}

		spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);

		if ((platform->min_lock > 0) && (platform->cur_clock < platform->min_lock))
			gpu_control_state_set(kbdev, GPU_CONTROL_CHANGE_CLK_VOL, platform->min_lock);

		GPU_LOG(DVFS_DEBUG, "[G3D] Lock min clk[%d], user lock[%d], current clk[%d]\n", platform->min_lock,
				platform->user_min_lock[platform->target_lock_type], platform->cur_clock);

		platform->target_lock_type = -1;
		break;
	case GPU_HANDLER_DVFS_MAX_UNLOCK:
		spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);

		if ((platform->target_lock_type < TMU_LOCK) || (platform->target_lock_type >= NUMBER_LOCK)) {
			spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			return -1;
		}

		platform->user_max_lock[platform->target_lock_type] = 0;
		platform->max_lock = platform->table[platform->table_size-1].clock;

		for (i = 0; i < NUMBER_LOCK; i++) {
			if (platform->user_max_lock[i] > 0) {
				dirty = true;
				platform->max_lock = MIN(platform->user_max_lock[i], platform->max_lock);
			}
		}

		if (!dirty)
			platform->max_lock = 0;

		platform->target_lock_type = -1;

		spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
		GPU_LOG(DVFS_DEBUG, "[G3D] Unlock max clk\n");
		break;
	case GPU_HANDLER_DVFS_MIN_UNLOCK:
		spin_lock_irqsave(&platform->gpu_dvfs_spinlock, flags);

		if ((platform->target_lock_type < TMU_LOCK) || (platform->target_lock_type >= NUMBER_LOCK)) {
			spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
			return -1;
		}

		platform->user_min_lock[platform->target_lock_type] = 0;
		platform->min_lock = platform->table[0].clock;

		for (i = 0; i < NUMBER_LOCK; i++) {
			if (platform->user_min_lock[i] > 0) {
				dirty = true;
				platform->min_lock = MAX(platform->user_min_lock[i], platform->min_lock);
			}
		}

		if (!dirty)
			platform->min_lock = 0;

		platform->target_lock_type = -1;

		spin_unlock_irqrestore(&platform->gpu_dvfs_spinlock, flags);
		GPU_LOG(DVFS_DEBUG, "[G3D] Unlock min clk\n");
		break;
	case GPU_HANDLER_INIT_TIME_IN_STATE:
		gpu_dvfs_init_time_in_state(platform);
		break;
	case GPU_HANDLER_UPDATE_TIME_IN_STATE:
		gpu_dvfs_update_time_in_state(platform, param);
		break;
	case GPU_HANDLER_DVFS_GET_LEVEL:
		ret = gpu_dvfs_get_level(platform, param);
		break;
#endif /* CONFIG_MALI_MIDGARD_DVFS */
	case GPU_HANDLER_DVFS_GET_VOLTAGE:
		ret = gpu_dvfs_get_voltage(platform, param);
		break;
	default:
		break;
	}
	return ret;
}

