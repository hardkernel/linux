/* drivers/gpu/t6xx/kbase/src/platform/gpu_dvfs_handler.h
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
 * @file gpu_dvfs_handler.h
 * DVFS
 */

#ifndef _GPU_DVFS_HANDLER_H_
#define _GPU_DVFS_HANDLER_H_

#define DVFS_ASSERT(x) \
do { if (x) break; \
	printk(KERN_EMERG "### ASSERTION FAILED %s: %s: %d: %s\n", __FILE__, __func__, __LINE__, #x); dump_stack(); \
} while (0)

typedef enum {
	GPU_HANDLER_DVFS_ON = 0,
	GPU_HANDLER_DVFS_OFF,
	GPU_HANDLER_DVFS_GOVERNOR_CHANGE,
	GPU_HANDLER_DVFS_MAX_LOCK,
	GPU_HANDLER_DVFS_MIN_LOCK,
	GPU_HANDLER_DVFS_MAX_UNLOCK,
	GPU_HANDLER_DVFS_MIN_UNLOCK,
	GPU_HANDLER_INIT_TIME_IN_STATE,
	GPU_HANDLER_UPDATE_TIME_IN_STATE,
	GPU_HANDLER_DVFS_GET_LEVEL,
	GPU_HANDLER_DVFS_GET_VOLTAGE,
} gpu_dvfs_handler_command;

int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation, u32 util_gl_share, u32 util_cl_share[2]);
int gpu_dvfs_handler_init(struct kbase_device *kbdev);
int gpu_dvfs_handler_deinit(struct kbase_device *kbdev);
int gpu_dvfs_handler_control(struct kbase_device *kbdev, gpu_dvfs_handler_command command, int param);
#ifdef CONFIG_CPU_THERMAL_IPA
int gpu_dvfs_get_clock(int level);
int gpu_dvfs_get_step(void);
#endif /* CONFIG_CPU_THERMAL_IPA */

#endif /* _GPU_DVFS_HANDLER_H_ */
