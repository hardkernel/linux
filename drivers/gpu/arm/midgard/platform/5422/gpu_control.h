/* drivers/gpu/t6xx/kbase/src/platform/gpu_control.h
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
 * @file gpu_control.h
 * DVFS
 */

#ifndef _GPU_CONTROL_H_
#define _GPU_CONTROL_H_

typedef enum {
	GPU_CONTROL_CLOCK_ON = 0,
	GPU_CONTROL_CLOCK_OFF,
	GPU_CONTROL_CHANGE_CLK_VOL,
	GPU_CONTROL_PREPARE_ON,
	GPU_CONTROL_IS_POWER_ON,
	GPU_CONTROL_SET_MARGIN,
} gpu_control_state;

typedef enum {
	GPU_CONTROL_PM_QOS_INIT = 0,
	GPU_CONTROL_PM_QOS_DEINIT,
	GPU_CONTROL_PM_QOS_SET,
	GPU_CONTROL_PM_QOS_RESET,
} gpu_pmqos_state;

/* mali_kbase_platform */
#if SOC_NAME == 5422
#ifdef CONFIG_SOC_EXYNOS5422_REV_0
#define G3D_MAX_FREQ    600000
#else
#define G3D_MAX_FREQ    600000
#endif /* CONFIG_SOC_EXYNOS5422_REV_0 */
#define G3D_MIN_FREQ    177000
#elif SOC_NAME == 5430
#define G3D_MAX_FREQ    600000
#define G3D_MIN_FREQ    160000
#elif SOC_NAME == 5260
#define G3D_MAX_FREQ    480000
#define G3D_MIN_FREQ    160000
#else
#error SOC_NAME should be specified.
#endif /* SOC_NAME */

/* GPU NOTIFIER */
#if SOC_NAME == 5422
#define GPU_THROTTLING_90_95    480
#define GPU_THROTTLING_95_100   266
#define GPU_THROTTLING_100_105  177
#define GPU_THROTTLING_105_110  177
#define GPU_TRIPPING_110        177
#define VOLTAGE_OFFSET_MARGIN   37500
#define RUNTIME_PM_DELAY_TIME   100
#elif SOC_NAME == 5430
#define GPU_THROTTLING_90_95    420
#define GPU_THROTTLING_95_100   350
#define GPU_THROTTLING_100_105  266
#define GPU_THROTTLING_105_110  160
#define GPU_TRIPPING_110        160
#define VOLTAGE_OFFSET_MARGIN   37500
#define RUNTIME_PM_DELAY_TIME   100
#define GPU_DYNAMIC_CLK_GATING  0
#elif SOC_NAME == 5260
#define GPU_THROTTLING_90_95	266
#define GPU_THROTTLING_95_100	266
#define GPU_THROTTLING_100_105	160
#define GPU_THROTTLING_105_110	160
#define GPU_TRIPPING_110        160
#define VOLTAGE_OFFSET_MARGIN	37500
#else
#error SOC_NAME should be specified.
#endif /* SOC_NAME */

#ifdef CONFIG_MALI_MIDGARD_RT_PM
#define RUNTIME_PM_DELAY_TIME 100
#endif /* CONFIG_MALI_MIDGARD_RT_PM */

/* GPU DVFS HANDLER */
#if SOC_NAME == 5422
#define MALI_DVFS_START_FREQ		266
#define MALI_DVFS_BL_CONFIG_FREQ	266
#elif SOC_NAME == 5430
#define MALI_DVFS_START_FREQ		266
#define MALI_DVFS_BL_CONFIG_FREQ	266
#elif SOC_NAME == 5260
#define MALI_DVFS_START_FREQ		350
#define MALI_DVFS_BL_CONFIG_FREQ	350
#define ACLK_G3D_STAT				0x10
#else
#error SOC_NAME should be specified.
#endif

#ifdef CONFIG_ARM_EXYNOS5422_BUS_DEVFREQ
#define CONFIG_BUS_DEVFREQ
#endif

#ifdef CONFIG_ARM_EXYNOS5430_BUS_DEVFREQ
#define CONFIG_BUS_DEVFREQ
#endif

#ifdef CONFIG_ARM_EXYNOS5260_BUS_DEVFREQ
#define CONFIG_BUS_DEVFREQ
#endif

#define GPU_DVFS_FREQUENCY		100
#ifdef CONFIG_CPU_THERMAL_IPA
#define GPU_DVFS_TIME_INTERVAL	5
#endif /* CONFIG_CPU_THERMAL_IPA */

/* GPU DVFS GOVERNOR */
#if SOC_NAME == 5422
#define G3D_GOVERNOR_DEFAULT_CLOCK_DEFAULT  266
#define G3D_GOVERNOR_DEFAULT_CLOCK_STATIC   266
#define G3D_GOVERNOR_STATIC_PERIOD          10
#define G3D_GOVERNOR_DEFAULT_CLOCK_BOOSTER  266
#elif SOC_NAME == 5430
#define G3D_GOVERNOR_DEFAULT_CLOCK_DEFAULT  266
#define G3D_GOVERNOR_DEFAULT_CLOCK_STATIC   266
#define G3D_GOVERNOR_STATIC_PERIOD          10
#define G3D_GOVERNOR_DEFAULT_CLOCK_BOOSTER  266
#elif SOC_NAME == 5260
#define G3D_GOVERNOR_DEFAULT_CLOCK_DEFAULT  266
#define G3D_GOVERNOR_DEFAULT_CLOCK_STATIC   350
#define G3D_GOVERNOR_STATIC_PERIOD          10
#define G3D_GOVERNOR_DEFAULT_CLOCK_BOOSTER  266
#else
#error SOC_NAME should be specified.
#endif /* SOC_NAME */

/* GPU CONTROL */
#if SOC_NAME == 5422
#define COLD_MINIMUM_VOL		950000
#define GPU_DEFAULT_VOLTAGE		1037500
#elif SOC_NAME == 5430
#define COLD_MINIMUM_VOL		950000
#define GPU_DEFAULT_VOLTAGE		1025000
#elif SOC_NAME == 5260
#define COLD_MINIMUM_VOL		950000
#define GPU_DEFAULT_VOLTAGE		900000
#else
#error SOC_NAME should be specified.
#endif /* SOC_NAME */

#define MALI_T6XX_DEFAULT_CLOCK (MALI_DVFS_START_FREQ*MHZ)

struct exynos_pm_domain *gpu_get_pm_domain(struct kbase_device *kbdev);
int get_cpu_clock_speed(u32 *cpu_clock);
int gpu_is_power_on(void);
int gpu_power_init(struct kbase_device *kbdev);
int gpu_is_clock_on(struct exynos_context *platform);
int gpu_clock_on(struct exynos_context *platform);
int gpu_clock_off(struct exynos_context *platform);
int gpu_set_clock(struct exynos_context *platform, int freq);
int gpu_clock_init(struct kbase_device *kbdev);
int gpu_set_voltage(struct exynos_context *platform, int vol);
int gpu_regulator_enable(struct exynos_context *platform);
int gpu_regulator_disable(struct exynos_context *platform);
int gpu_regulator_init(struct exynos_context *platform);

int gpu_control_state_set(struct kbase_device *kbdev, gpu_control_state state, int param);
int gpu_control_module_init(struct kbase_device *kbdev);
void gpu_control_module_term(struct kbase_device *kbdev);

#endif /* _GPU_CONTROL_H_ */
