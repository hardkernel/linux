/*
 * exynos_thermal.c - Samsung EXYNOS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ipa.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_data/exynos_thermal.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <plat/cpu.h>
#include <mach/tmu.h>
#include <mach/cpufreq.h>
#include <mach/asv-exynos.h>

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static struct cpumask mp_cluster_cpus[CA_END];
#endif

/* Exynos generic registers */
#define EXYNOS_TMU_REG_TRIMINFO			0x0
#define EXYNOS_TMU_REG_CONTROL			0x20
#define EXYNOS_TMU_REG_STATUS			0x28
#define EXYNOS_TMU_REG_CURRENT_TEMP		0x40

#if defined(CONFIG_SOC_EXYNOS5430_REV_0)
/* Exynos5430 specific registers */
#define EXYNOS_THD_TEMP_RISE			0x50
#define EXYNOS_THD_TEMP_FALL			0x60
#define EXYNOS_TMU_REG_INTEN			0xB0
#define EXYNOS_TMU_REG_INTSTAT			0xB4
#define EXYNOS_TMU_REG_INTCLEAR			0xB8
#elif defined(CONFIG_SOC_EXYNOS5430_REV_1)
#define EXYNOS_THD_TEMP_RISE			0x50
#define EXYNOS_THD_TEMP_FALL			0x60
#define EXYNOS_THD_TEMP_RISE3_0			0x50
#define EXYNOS_THD_TEMP_RISE7_4			0x54
#define EXYNOS_THD_TEMP_FALL3_0			0x60
#define EXYNOS_THD_TEMP_FALL7_4			0x64
#define EXYNOS_TMU_REG_INTEN			0xC0
#define EXYNOS_TMU_REG_INTCLEAR			0xC8
#else
#define EXYNOS_THD_TEMP_RISE            	0x50
#define EXYNOS_THD_TEMP_FALL            	0x54
#define EXYNOS_TMU_REG_INTEN			0x70
#define EXYNOS_TMU_REG_INTSTAT			0x74
#define EXYNOS_TMU_REG_INTCLEAR			0x78
#endif

#define EXYNOS_TMU_TRIM_TEMP_MASK		0xff
#define EXYNOS_TMU_GAIN_SHIFT			8
#define EXYNOS_TMU_REF_VOLTAGE_SHIFT		24
#define EXYNOS_TMU_CORE_ON			3
#define EXYNOS_TMU_CORE_OFF			2
#define EXYNOS_TMU_DEF_CODE_TO_TEMP_OFFSET	50

/* Exynos4210 specific registers */
#define EXYNOS4210_TMU_REG_THRESHOLD_TEMP	0x44
#define EXYNOS4210_TMU_REG_TRIG_LEVEL0		0x50
#define EXYNOS4210_TMU_REG_TRIG_LEVEL1		0x54
#define EXYNOS4210_TMU_REG_TRIG_LEVEL2		0x58
#define EXYNOS4210_TMU_REG_TRIG_LEVEL3		0x5C
#define EXYNOS4210_TMU_REG_PAST_TEMP0		0x60
#define EXYNOS4210_TMU_REG_PAST_TEMP1		0x64
#define EXYNOS4210_TMU_REG_PAST_TEMP2		0x68
#define EXYNOS4210_TMU_REG_PAST_TEMP3		0x6C

#define EXYNOS4210_TMU_TRIG_LEVEL0_MASK		0x1
#define EXYNOS4210_TMU_TRIG_LEVEL1_MASK		0x10
#define EXYNOS4210_TMU_TRIG_LEVEL2_MASK		0x100
#define EXYNOS4210_TMU_TRIG_LEVEL3_MASK		0x1000
#define EXYNOS4210_TMU_INTCLEAR_VAL		0x1111

/* Exynos5250 and Exynos4412 specific registers */
#define EXYNOS_TRIMINFO_RELOAD1			0x01
#define EXYNOS_TRIMINFO_RELOAD2			0x11
#define EXYNOS_TRIMINFO_CONFIG			0x10
#define EXYNOS_TRIMINFO_CONTROL			0x14
#define EXYNOS_EMUL_CON				0x80

#define EXYNOS_TRIMINFO_RELOAD			0x1
#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
#define EXYNOS_TMU_CLEAR_RISE_INT      		0xff
#define EXYNOS_TMU_CLEAR_FALL_INT      		(0xff << 16)
#else
#define EXYNOS_TMU_CLEAR_RISE_INT		0x1111
#define EXYNOS_TMU_CLEAR_FALL_INT		(0x1111 << 16)
#endif
#define EXYNOS_MUX_ADDR_VALUE			6
#define EXYNOS_MUX_ADDR_SHIFT			20
#define EXYNOS_TMU_TRIP_MODE_SHIFT		13
#define EXYNOS_THERM_TRIP_EN			(1 << 12)
#define EXYNOS_MUX_ADDR				0x600000

#define EFUSE_MIN_VALUE				40
#define EFUSE_MAX_VALUE				100

/* In-kernel thermal framework related macros & definations */
#define SENSOR_NAME_LEN				16
#define MAX_TRIP_COUNT				9
#define MAX_COOLING_DEVICE 			5
#define MAX_THRESHOLD_LEVS 			8

#define PASSIVE_INTERVAL			100
#define ACTIVE_INTERVAL				300
#define IDLE_INTERVAL 				1000
#define MCELSIUS				1000

#ifdef CONFIG_THERMAL_EMULATION
#define EXYNOS_EMUL_TIME			0x57F0
#define EXYNOS_EMUL_TIME_SHIFT			16
#define EXYNOS_EMUL_DATA_SHIFT			8
#define EXYNOS_EMUL_DATA_MASK			0xFF
#define EXYNOS_EMUL_ENABLE			0x1
#endif /* CONFIG_THERMAL_EMULATION */

/* CPU Zone information */
#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
#define PANIC_ZONE      			10
#else
#define PANIC_ZONE      			6
#endif
#define WARN_ZONE       			3
#define MONITOR_ZONE    			2
#define SAFE_ZONE       			1

/* Rising, Falling interrupt bit number*/
#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
#define RISE_LEVEL1_SHIFT      			1
#define RISE_LEVEL2_SHIFT      			2
#define RISE_LEVEL3_SHIFT			3
#define RISE_LEVEL4_SHIFT      			4
#define RISE_LEVEL5_SHIFT      			5
#define RISE_LEVEL6_SHIFT      			6
#define RISE_LEVEL7_SHIFT      			7
#define FALL_LEVEL0_SHIFT      			16
#define FALL_LEVEL1_SHIFT      			17
#define FALL_LEVEL2_SHIFT      			18
#define FALL_LEVEL3_SHIFT      			19
#define FALL_LEVEL4_SHIFT      			20
#define FALL_LEVEL5_SHIFT      			21
#define FALL_LEVEL6_SHIFT      			22
#define FALL_LEVEL7_SHIFT      			23
#else
#define RISE_LEVEL1_SHIFT			4
#define RISE_LEVEL2_SHIFT			8
#define RISE_LEVEL3_SHIFT			12
#define RISE_LEVEL4_SHIFT      			0
#define RISE_LEVEL5_SHIFT      			0
#define RISE_LEVEL6_SHIFT      			0
#define RISE_LEVEL7_SHIFT      			0
#define FALL_LEVEL0_SHIFT			16
#define FALL_LEVEL1_SHIFT			20
#define FALL_LEVEL2_SHIFT			24
#define FALL_LEVEL3_SHIFT			28
#define FALL_LEVEL4_SHIFT      			0
#define FALL_LEVEL5_SHIFT      			0
#define FALL_LEVEL6_SHIFT      			0
#define FALL_LEVEL7_SHIFT      			0
#endif

#define GET_ZONE(trip) (trip + 2)
#define GET_TRIP(zone) (zone - 2)

#define EXYNOS_ZONE_COUNT			1
#define EXYNOS_TMU_COUNT			5
#define EXYSNO_CLK_COUNT			2
#define TRIP_EN_COUNT				8
#ifdef CONFIG_SOC_EXYNOS5422
#define EXYNOS_GPU_NUMBER			4
#else
#define EXYNOS_GPU_NUMBER			2
#endif

#define MIN_TEMP				20
#define MAX_TEMP				125

#define CA7_POLICY_CORE		((exynos_boot_cluster == CA7) ? 0 : 4)
#define CA15_POLICY_CORE 	((exynos_boot_cluster == CA15) ? 0 : 4)
#define CS_POLICY_CORE		0

#if defined(CONFIG_SOC_EXYNOS5430)
#define CPU_HOTPLUG_IN_TEMP	95
#define CPU_HOTPLUG_OUT_TEMP	110
#elif defined(CONFIG_SOC_EXYNOS5422)
#define CPU_HOTPLUG_IN_TEMP	95
#define CPU_HOTPLUG_OUT_TEMP	100
#endif

static enum tmu_noti_state_t tmu_old_state = TMU_NORMAL;
static enum gpu_noti_state_t gpu_old_state = GPU_NORMAL;
static enum mif_noti_state_t mif_old_state = MIF_TH_LV1;
static bool is_suspending;
static bool is_cpu_hotplugged_out;

static unsigned long ctmu_normal, ctmu_cold, ctmu_hot, ctmu_critical, throttle_cpu;

static BLOCKING_NOTIFIER_HEAD(exynos_tmu_notifier);
static BLOCKING_NOTIFIER_HEAD(exynos_gpu_notifier);

struct exynos_tmu_data {
	struct exynos_tmu_platform_data *pdata;
	struct resource *mem[EXYNOS_TMU_COUNT];
	void __iomem *base[EXYNOS_TMU_COUNT];
	int irq[EXYNOS_TMU_COUNT];
	enum soc_type soc;
	struct work_struct irq_work;
	struct mutex lock;
	struct clk *clk[EXYSNO_CLK_COUNT];
	u8 temp_error1[EXYNOS_TMU_COUNT];
	u8 temp_error2[EXYNOS_TMU_COUNT];
};

struct	thermal_trip_point_conf {
	int trip_val[MAX_TRIP_COUNT];
	int trip_count;
	u8 trigger_falling;
};

struct	thermal_cooling_conf {
	struct freq_clip_table freq_data[MAX_TRIP_COUNT];
	int size[THERMAL_TRIP_CRITICAL + 1];
	int freq_clip_count;
};

struct thermal_sensor_conf {
	char name[SENSOR_NAME_LEN];
	int (*read_temperature)(void *data);
	int (*write_emul_temp)(void *drv_data, unsigned long temp);
	struct thermal_trip_point_conf trip_data;
	struct thermal_cooling_conf cooling_data;
	void *private_data;
};

struct exynos_thermal_zone {
	enum thermal_device_mode mode;
	struct thermal_zone_device *therm_dev;
	struct thermal_cooling_device *cool_dev[MAX_COOLING_DEVICE];
	unsigned int cool_dev_size;
	struct platform_device *exynos4_dev;
	struct thermal_sensor_conf *sensor_conf;
	bool bind;
};

static struct exynos_thermal_zone *th_zone;
static struct platform_device *exynos_tmu_pdev;
static struct exynos_tmu_data *tmudata;
static void exynos_unregister_thermal(void);
static int exynos_register_thermal(struct thermal_sensor_conf *sensor_conf);
static int exynos5_tmu_cpufreq_notifier(struct notifier_block *notifier, unsigned long event, void *v);

static struct notifier_block exynos_cpufreq_nb = {
	.notifier_call = exynos5_tmu_cpufreq_notifier,
};

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static void __init init_mp_cpumask_set(void)
{
	 unsigned int i;

	 for_each_cpu(i, cpu_possible_mask) {
		 if (exynos_boot_cluster == CA7) {
			 if (i >= NR_CA7)
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA15]);
			 else
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA7]);
		 } else {
			 if (i >= NR_CA15)
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA7]);
			 else
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA15]);
		 }
	 }
}
#endif

/* Get mode callback functions for thermal zone */
static int exynos_get_mode(struct thermal_zone_device *thermal,
			enum thermal_device_mode *mode)
{
	if (th_zone)
		*mode = th_zone->mode;
	return 0;
}

/* Set mode callback functions for thermal zone */
static int exynos_set_mode(struct thermal_zone_device *thermal,
			enum thermal_device_mode mode)
{
	if (!th_zone->therm_dev) {
		pr_notice("thermal zone not registered\n");
		return 0;
	}

	mutex_lock(&th_zone->therm_dev->lock);

	if (mode == THERMAL_DEVICE_ENABLED &&
		!th_zone->sensor_conf->trip_data.trigger_falling)
		th_zone->therm_dev->polling_delay = IDLE_INTERVAL;
	else
		th_zone->therm_dev->polling_delay = 0;

	mutex_unlock(&th_zone->therm_dev->lock);

	th_zone->mode = mode;
	thermal_zone_device_update(th_zone->therm_dev);
	pr_info("thermal polling set for duration=%d msec\n",
				th_zone->therm_dev->polling_delay);
	return 0;
}


/* Get trip type callback functions for thermal zone */
static int exynos_get_trip_type(struct thermal_zone_device *thermal, int trip,
				 enum thermal_trip_type *type)
{
	unsigned int cur_zone;
	cur_zone = GET_ZONE(trip);

	if (cur_zone >= MONITOR_ZONE && cur_zone < WARN_ZONE)
		*type = THERMAL_TRIP_ACTIVE;
	else if (cur_zone >= WARN_ZONE && cur_zone < PANIC_ZONE)
		*type = THERMAL_TRIP_PASSIVE;
	else if (cur_zone >= PANIC_ZONE)
		*type = THERMAL_TRIP_CRITICAL;
	else
		return -EINVAL;

	return 0;
}

/* Get trip temperature callback functions for thermal zone */
static int exynos_get_trip_temp(struct thermal_zone_device *thermal, int trip,
				unsigned long *temp)
{
	if (trip < GET_TRIP(MONITOR_ZONE) || trip > GET_TRIP(PANIC_ZONE))
		return -EINVAL;

	*temp = th_zone->sensor_conf->trip_data.trip_val[trip];
	/* convert the temperature into millicelsius */
	*temp = *temp * MCELSIUS;

	return 0;
}

/* Get critical temperature callback functions for thermal zone */
static int exynos_get_crit_temp(struct thermal_zone_device *thermal,
				unsigned long *temp)
{
	int ret;
	/* Panic zone */
	ret = exynos_get_trip_temp(thermal, GET_TRIP(PANIC_ZONE), temp);
	return ret;
}

/* Bind callback functions for thermal zone */
static int exynos_bind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	int ret = 0, i, tab_size, level = THERMAL_CSTATE_INVALID;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	int cluster_idx = 0;
#endif
	struct freq_clip_table *tab_ptr, *clip_data;
	struct thermal_sensor_conf *data = th_zone->sensor_conf;
	enum thermal_trip_type type = 0;

	tab_ptr = (struct freq_clip_table *)data->cooling_data.freq_data;
	tab_size = data->cooling_data.freq_clip_count;

	if (tab_ptr == NULL || tab_size == 0)
		return -EINVAL;

	/* find the cooling device registered*/
	for (i = 0; i < th_zone->cool_dev_size; i++)
		if (cdev == th_zone->cool_dev[i]) {
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
			cluster_idx = i;
#endif
			break;
		}

	/* No matching cooling device */
	if (i == th_zone->cool_dev_size)
		return 0;

	/* Bind the thermal zone to the cpufreq cooling device */
	for (i = 0; i < tab_size; i++) {
		clip_data = (struct freq_clip_table *)&(tab_ptr[i]);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		if (cluster_idx == CA7)
			level = cpufreq_cooling_get_level(CA7_POLICY_CORE, clip_data->freq_clip_max_kfc);
		else if (cluster_idx == CA15)
			level = cpufreq_cooling_get_level(CA15_POLICY_CORE, clip_data->freq_clip_max);
#else
		level = cpufreq_cooling_get_level(CS_POLICY_CORE, clip_data->freq_clip_max);
#endif
		if (level == THERMAL_CSTATE_INVALID) {
			thermal->cooling_dev_en = false;
			return 0;
		}
		exynos_get_trip_type(th_zone->therm_dev, i, &type);
		switch (type) {
		case THERMAL_TRIP_ACTIVE:
		case THERMAL_TRIP_PASSIVE:
			if (thermal_zone_bind_cooling_device(thermal, i, cdev,
								level, 0)) {
				pr_err("error binding cdev inst %d\n", i);
				thermal->cooling_dev_en = false;
				ret = -EINVAL;
			}
			th_zone->bind = true;
			break;
		default:
			ret = -EINVAL;
		}
	}

	return ret;
}

/* Unbind callback functions for thermal zone */
static int exynos_unbind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	int ret = 0, i, tab_size;
	struct thermal_sensor_conf *data = th_zone->sensor_conf;

	if (th_zone->bind == false)
		return 0;

	tab_size = data->cooling_data.freq_clip_count;

	if (tab_size == 0)
		return -EINVAL;

	/* find the cooling device registered*/
	for (i = 0; i < th_zone->cool_dev_size; i++)
		if (cdev == th_zone->cool_dev[i])
			break;

	/* No matching cooling device */
	if (i == th_zone->cool_dev_size)
		return 0;

	/* Bind the thermal zone to the cpufreq cooling device */
	for (i = 0; i < tab_size; i++) {
		switch (GET_ZONE(i)) {
		case MONITOR_ZONE:
		case WARN_ZONE:
			if (thermal_zone_unbind_cooling_device(thermal, i,
								cdev)) {
				pr_err("error unbinding cdev inst=%d\n", i);
				ret = -EINVAL;
			}
			th_zone->bind = false;
			break;
		default:
			ret = -EINVAL;
		}
	}
	return ret;
}


int exynos_tmu_add_notifier(struct notifier_block *n)
{
	return blocking_notifier_chain_register(&exynos_tmu_notifier, n);
}

void exynos_tmu_call_notifier(enum tmu_noti_state_t cur_state, int temp)
{
	if (is_suspending)
		cur_state = TMU_COLD;

	if (cur_state != tmu_old_state) {
		if ((cur_state == TMU_COLD) ||
			((cur_state == TMU_NORMAL) && (tmu_old_state == TMU_COLD)))
			blocking_notifier_call_chain(&exynos_tmu_notifier, TMU_COLD, &cur_state);
		else
			blocking_notifier_call_chain(&exynos_tmu_notifier, cur_state, &tmu_old_state);
		if (cur_state == TMU_COLD)
			pr_debug("tmu temperature state %d to %d\n", tmu_old_state, cur_state);
		else
			pr_debug("tmu temperature state %d to %d, cur_temp : %d\n", tmu_old_state, cur_state, temp);
		tmu_old_state = cur_state;
	}
}

int exynos_gpu_add_notifier(struct notifier_block *n)
{
	return blocking_notifier_chain_register(&exynos_gpu_notifier, n);
}

void exynos_gpu_call_notifier(enum gpu_noti_state_t cur_state)
{
	if (is_suspending)
		cur_state = GPU_COLD;

	if (cur_state != gpu_old_state) {
		pr_debug("gpu temperature state %d to %d\n", gpu_old_state, cur_state);
		blocking_notifier_call_chain(&exynos_gpu_notifier, cur_state, &cur_state);
		gpu_old_state = cur_state;
	}
}

static void exynos_check_tmu_noti_state(int min_temp, int max_temp)
{
	enum tmu_noti_state_t cur_state;

	/* check current temperature state */
	if (max_temp > HOT_CRITICAL_TEMP) {
		cur_state = TMU_CRITICAL;
		ctmu_critical++;
	} else if (max_temp > HOT_NORMAL_TEMP && max_temp <= HOT_CRITICAL_TEMP) {
		cur_state = TMU_HOT;
		ctmu_hot++;
	} else if (max_temp > COLD_TEMP && max_temp <= HOT_NORMAL_TEMP) {
		cur_state = TMU_NORMAL;
		ctmu_normal++;
	} else {
		cur_state = TMU_COLD;
		ctmu_cold++;
	}

	if (min_temp <= COLD_TEMP)
		cur_state = TMU_COLD;

	/* Just to check for overflow */
	if (ctmu_critical >= ULONG_MAX)
		ctmu_critical = 0;
	if (ctmu_hot >= ULONG_MAX)
		ctmu_hot = 0;
	if (ctmu_normal >= ULONG_MAX)
		ctmu_normal = 0;
	if (ctmu_cold > ULONG_MAX)
		ctmu_cold = 0;
	if (throttle_cpu > ULONG_MAX)
		throttle_cpu = 0;

	exynos_tmu_call_notifier(cur_state, max_temp);
}

static void exynos_check_mif_noti_state(int temp)
{
	enum mif_noti_state_t cur_state;

	/* check current temperature state */
	if (temp < MIF_TH_TEMP1)
		cur_state = MIF_TH_LV1;
	else if (temp >= MIF_TH_TEMP1 && temp < MIF_TH_TEMP2)
		cur_state = MIF_TH_LV2;
	else
		cur_state = MIF_TH_LV3;

	if (cur_state != mif_old_state) {
		blocking_notifier_call_chain(&exynos_tmu_notifier, cur_state, &mif_old_state);
		mif_old_state = cur_state;
	}
}

static void exynos_check_gpu_noti_state(int temp)
{
	enum gpu_noti_state_t cur_state;

	/* check current temperature state */
	if (temp >= GPU_TH_TEMP5)
		cur_state = GPU_TRIPPING;
	else if (temp >= GPU_TH_TEMP4 && temp < GPU_TH_TEMP5)
		cur_state = GPU_THROTTLING4;
	else if (temp >= GPU_TH_TEMP3 && temp < GPU_TH_TEMP4)
		cur_state = GPU_THROTTLING3;
	else if (temp >= GPU_TH_TEMP2 && temp < GPU_TH_TEMP3)
		cur_state = GPU_THROTTLING2;
	else if (temp >= GPU_TH_TEMP1 && temp < GPU_TH_TEMP2)
		cur_state = GPU_THROTTLING1;
	else if (temp > COLD_TEMP && temp < GPU_TH_TEMP1)
		cur_state = GPU_NORMAL;
	else
		cur_state = GPU_COLD;

	exynos_gpu_call_notifier(cur_state);
}

/* Get temperature callback functions for thermal zone */
static int exynos_get_temp(struct thermal_zone_device *thermal,
			unsigned long *temp)
{
	void *data;

	if (!th_zone->sensor_conf) {
		pr_info("Temperature sensor not initialised\n");
		return -EINVAL;
	}
	data = th_zone->sensor_conf->private_data;
	*temp = th_zone->sensor_conf->read_temperature(data);
	/* convert the temperature into millicelsius */
	*temp = *temp * MCELSIUS;
	return 0;
}

/* Get temperature callback functions for thermal zone */
static int exynos_set_emul_temp(struct thermal_zone_device *thermal,
						unsigned long temp)
{
	void *data;
	int ret = -EINVAL;

	if (!th_zone->sensor_conf) {
		pr_info("Temperature sensor not initialised\n");
		return -EINVAL;
	}
	data = th_zone->sensor_conf->private_data;
	if (th_zone->sensor_conf->write_emul_temp)
		ret = th_zone->sensor_conf->write_emul_temp(data, temp);
	return ret;
}

/* Get the temperature trend */
static int exynos_get_trend(struct thermal_zone_device *thermal,
			int trip, enum thermal_trend *trend)
{
	int ret;
	unsigned long trip_temp;

	ret = exynos_get_trip_temp(thermal, trip, &trip_temp);
	if (ret < 0)
		return ret;

	if (thermal->temperature >= trip_temp)
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5422)
static int __ref exynos_throttle_cpu_hotplug(struct thermal_zone_device *thermal)
{
	int ret = 0;
	int cur_temp = 0;

	if (!thermal->temperature)
		return -EINVAL;

	cur_temp = thermal->temperature / MCELSIUS;

	if (is_cpu_hotplugged_out) {
		if (cur_temp < CPU_HOTPLUG_IN_TEMP) {
			/*
			 * If current temperature is lower than low threshold,
			 * call big_cores_hotplug(false) for hotplugged out cpus.
			 */
			throttle_cpu++;
			ret = big_cores_hotplug(false);
			if (ret)
				pr_err("%s: failed big cores hotplug in\n",
							__func__);
			else
				is_cpu_hotplugged_out = false;
		}
	} else {
		if (cur_temp >= CPU_HOTPLUG_OUT_TEMP) {
			/*
			 * If current temperature is higher than high threshold,
			 * call big_cores_hotplug(true) to hold temperature down.
			 */
			throttle_cpu++;
			ret = big_cores_hotplug(true);
			if (ret)
				pr_err("%s: failed big cores hotplug out\n",
							__func__);
			else
				is_cpu_hotplugged_out = true;
		}
	}

	return ret;
}
#endif

/* Operation callback functions for thermal zone */
static struct thermal_zone_device_ops const exynos_dev_ops = {
	.bind = exynos_bind,
	.unbind = exynos_unbind,
	.get_temp = exynos_get_temp,
	.set_emul_temp = exynos_set_emul_temp,
	.get_trend = exynos_get_trend,
	.get_mode = exynos_get_mode,
	.set_mode = exynos_set_mode,
	.get_trip_type = exynos_get_trip_type,
	.get_trip_temp = exynos_get_trip_temp,
	.get_crit_temp = exynos_get_crit_temp,
#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5422)
	.throttle_cpu_hotplug = exynos_throttle_cpu_hotplug,
#endif
};

/*
 * This function may be called from interrupt based temperature sensor
 * when threshold is changed.
 */
static void exynos_report_trigger(void)
{
	unsigned int i;
	char data[10];
	char *envp[] = { data, NULL };

	if (!th_zone || !th_zone->therm_dev)
		return;
	if (th_zone->bind == false) {
		for (i = 0; i < th_zone->cool_dev_size; i++) {
			if (!th_zone->cool_dev[i])
				continue;
			exynos_bind(th_zone->therm_dev,
					th_zone->cool_dev[i]);
		}
	}

	thermal_zone_device_update(th_zone->therm_dev);

	mutex_lock(&th_zone->therm_dev->lock);
	/* Find the level for which trip happened */
	for (i = 0; i < th_zone->sensor_conf->trip_data.trip_count; i++) {
		if (th_zone->therm_dev->last_temperature <
			th_zone->sensor_conf->trip_data.trip_val[i] * MCELSIUS)
			break;
	}

	if (th_zone->mode == THERMAL_DEVICE_ENABLED) {
		if (GET_ZONE(i) > WARN_ZONE)
			th_zone->therm_dev->passive_delay = PASSIVE_INTERVAL;
		else
			th_zone->therm_dev->passive_delay = ACTIVE_INTERVAL;
	}

	snprintf(data, sizeof(data), "%u", i);
	kobject_uevent_env(&th_zone->therm_dev->device.kobj, KOBJ_CHANGE, envp);
	mutex_unlock(&th_zone->therm_dev->lock);
}

/* Register with the in-kernel thermal management */
static int exynos_register_thermal(struct thermal_sensor_conf *sensor_conf)
{
	int ret, count = 0;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	int i, j;
#endif
	struct cpumask mask_val;

	if (!sensor_conf || !sensor_conf->read_temperature) {
		pr_err("Temperature sensor not initialised\n");
		return -EINVAL;
	}

	th_zone = kzalloc(sizeof(struct exynos_thermal_zone), GFP_KERNEL);
	if (!th_zone)
		return -ENOMEM;

	th_zone->sensor_conf = sensor_conf;
	cpumask_clear(&mask_val);
	cpumask_set_cpu(0, &mask_val);

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	for (i = 0; i < EXYNOS_ZONE_COUNT; i++) {
		for (j = 0; j < CA_END; j++) {
			th_zone->cool_dev[count] = cpufreq_cooling_register(&mp_cluster_cpus[count]);
			if (IS_ERR(th_zone->cool_dev[count])) {
				pr_err("Failed to register cpufreq cooling device\n");
				ret = -EINVAL;
				th_zone->cool_dev_size = count;
				goto err_unregister;
			}
			count++;
		}
	}
#else
	for (count = 0; count < EXYNOS_ZONE_COUNT; count++) {
		th_zone->cool_dev[count] = cpufreq_cooling_register(&mask_val);
		if (IS_ERR(th_zone->cool_dev[count])) {
			 pr_err("Failed to register cpufreq cooling device\n");
			 ret = -EINVAL;
			 th_zone->cool_dev_size = count;
			 goto err_unregister;
		 }
	}
#endif
	th_zone->cool_dev_size = count;

	th_zone->therm_dev = thermal_zone_device_register(sensor_conf->name,
			th_zone->sensor_conf->trip_data.trip_count, 0, NULL, &exynos_dev_ops, NULL, PASSIVE_INTERVAL,
			IDLE_INTERVAL);

	if (IS_ERR(th_zone->therm_dev)) {
		pr_err("Failed to register thermal zone device\n");
		ret = PTR_ERR(th_zone->therm_dev);
		goto err_unregister;
	}
	th_zone->mode = THERMAL_DEVICE_ENABLED;

	pr_info("Exynos: Kernel Thermal management registered\n");

	return 0;

err_unregister:
	exynos_unregister_thermal();
	return ret;
}

/* Un-Register with the in-kernel thermal management */
static void exynos_unregister_thermal(void)
{
	int i;

	if (!th_zone)
		return;

	if (th_zone->therm_dev)
		thermal_zone_device_unregister(th_zone->therm_dev);

	for (i = 0; i < th_zone->cool_dev_size; i++) {
		if (th_zone->cool_dev[i])
			cpufreq_cooling_unregister(th_zone->cool_dev[i]);
	}

	kfree(th_zone);
	pr_info("Exynos: Kernel Thermal management unregistered\n");
}

/*
 * TMU treats temperature as a mapped temperature code.
 * The temperature is converted differently depending on the calibration type.
 */
static int temp_to_code(struct exynos_tmu_data *data, u8 temp, int id)
{
	struct exynos_tmu_platform_data *pdata = data->pdata;
	int temp_code;
	int fuse_id = 0;

	if (temp > MAX_TEMP)
		temp_code = MAX_TEMP;
	else if (temp < MIN_TEMP)
		temp_code = MIN_TEMP;

	if (soc_is_exynos5422()) {
		switch (id) {
		case 0:
			fuse_id = 0;
			break;
		case 1:
			fuse_id = 1;
			break;
		case 2:
			fuse_id = 3;
			break;
		case 3:
			fuse_id = 4;
			break;
		case 4:
			fuse_id = 2;
			break;
		default:
			pr_err("unknown sensor id on Exynos5422\n");
			break;
		}
	} else {
		fuse_id = id;
	}

	switch (pdata->cal_type) {
	case TYPE_TWO_POINT_TRIMMING:
		temp_code = (temp - 25) *
		    (data->temp_error2[fuse_id] - data->temp_error1[fuse_id]) /
		    (85 - 25) + data->temp_error1[fuse_id];
		break;
	case TYPE_ONE_POINT_TRIMMING:
		temp_code = temp + data->temp_error1[fuse_id] - 25;
		break;
	default:
		temp_code = temp + EXYNOS_TMU_DEF_CODE_TO_TEMP_OFFSET;
		break;
	}

	return temp_code;
}

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(struct exynos_tmu_data *data, u8 temp_code, int id)
{
	struct exynos_tmu_platform_data *pdata = data->pdata;
	int temp;
	int fuse_id = 0;

	if (soc_is_exynos5422()) {
		switch (id) {
		case 0:
			fuse_id = 0;
			break;
		case 1:
			fuse_id = 1;
			break;
		case 2:
			fuse_id = 3;
			break;
		case 3:
			fuse_id = 4;
			break;
		case 4:
			fuse_id = 2;
			break;
		default:
			pr_err("unknown sensor id on Exynos5422\n");
			break;
		}
	} else {
		fuse_id = id;
	}

	switch (pdata->cal_type) {
	case TYPE_TWO_POINT_TRIMMING:
		temp = (temp_code - data->temp_error1[fuse_id]) * (85 - 25) /
		    (data->temp_error2[fuse_id] - data->temp_error1[fuse_id]) + 25;
		break;
	case TYPE_ONE_POINT_TRIMMING:
		temp = temp_code - data->temp_error1[fuse_id] + 25;
		break;
	default:
		temp = temp_code - EXYNOS_TMU_DEF_CODE_TO_TEMP_OFFSET;
		break;
	}

	/* temperature should range between minimum and maximum */
	if (temp > MAX_TEMP)
		temp = MAX_TEMP;
	else if (temp < MIN_TEMP)
		temp = MIN_TEMP;

	return temp;
}

static int exynos_tmu_initialize(struct platform_device *pdev, int id)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_tmu_platform_data *pdata = data->pdata;
	unsigned int status;
	unsigned int rising_threshold = 0, falling_threshold = 0;
#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
	unsigned int rising_threshold7_4 = 0, falling_threshold7_4 = 0;
#endif
	int ret = 0, threshold_code, i, trigger_levs = 0;
	int timeout = 20000;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	while(1) {
		status = readb(data->base[id] + EXYNOS_TMU_REG_STATUS);
		if (status)
			break;

		timeout--;
		if (!timeout) {
			pr_err("%s: timeout TMU busy\n", __func__);
			ret = -EBUSY;
			goto out;
		}

		cpu_relax();
		usleep_range(1, 2);
	};

	/* Count trigger levels to be enabled */
	for (i = 0; i < MAX_THRESHOLD_LEVS; i++)
		if (pdata->trigger_levels[i])
			trigger_levs++;

	if (data->soc == SOC_ARCH_EXYNOS4210) {
		/* Write temperature code for threshold */
		threshold_code = temp_to_code(data, pdata->threshold, 0);
		if (threshold_code < 0) {
			ret = threshold_code;
			goto out;
		}
		writeb(threshold_code,
			data->base[0] + EXYNOS4210_TMU_REG_THRESHOLD_TEMP);
		for (i = 0; i < trigger_levs; i++)
			writeb(pdata->trigger_levels[i],
			data->base[0] + EXYNOS4210_TMU_REG_TRIG_LEVEL0 + i * 4);

		writel(EXYNOS4210_TMU_INTCLEAR_VAL,
			data->base[i] + EXYNOS_TMU_REG_INTCLEAR);
	} else if (data->soc == SOC_ARCH_EXYNOS) {
		/* Write temperature code for rising and falling threshold */
		for (i = 0; i < trigger_levs; i++) {
			threshold_code = temp_to_code(data,
					pdata->trigger_levels[i], id);
			if (threshold_code < 0) {
				ret = threshold_code;
				goto out;
			}
			rising_threshold |= threshold_code << 8 * i;
			if (pdata->threshold_falling) {
				threshold_code = temp_to_code(data,
						pdata->trigger_levels[i] -
						pdata->threshold_falling, id);
				if (threshold_code > 0)
					falling_threshold |=
						threshold_code << 8 * i;
			}
		}

		writel(rising_threshold, data->base[id] + EXYNOS_THD_TEMP_RISE);
		writel(falling_threshold, data->base[id] + EXYNOS_THD_TEMP_FALL);
		writel(EXYNOS_TMU_CLEAR_RISE_INT | EXYNOS_TMU_CLEAR_FALL_INT, data->base[id] + EXYNOS_TMU_REG_INTCLEAR);
	} else if (data->soc == SOC_ARCH_EXYNOS5430) {
#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
		for (i = 0; i < trigger_levs; i++) {
			threshold_code = temp_to_code(data,
					pdata->trigger_levels[i], id);
			if (threshold_code < 0) {
				ret = threshold_code;
				goto out;
			}
			if (i < 4)
				rising_threshold |= threshold_code << (8 * i);
			else
				rising_threshold7_4 |= threshold_code << (8 * (i - 4));
			if (pdata->threshold_falling) {
				threshold_code = temp_to_code(data,
						pdata->trigger_levels[i] -
						pdata->threshold_falling, id);
				if (threshold_code > 0) {
					if (i < 4)
						falling_threshold |= threshold_code << (8 * i);
					else
						falling_threshold7_4 |= threshold_code << (8 * (i - 4));
				}
			}
		}
		writel(rising_threshold,
				data->base[id] + EXYNOS_THD_TEMP_RISE3_0);
		writel(rising_threshold7_4,
				data->base[id] + EXYNOS_THD_TEMP_RISE7_4);
		writel(falling_threshold,
				data->base[id] + EXYNOS_THD_TEMP_FALL3_0);
		writel(falling_threshold7_4,
				data->base[id] + EXYNOS_THD_TEMP_FALL7_4);
		writel(EXYNOS_TMU_CLEAR_RISE_INT | EXYNOS_TMU_CLEAR_FALL_INT,
				data->base[id] + EXYNOS_TMU_REG_INTCLEAR);
#endif
	}
out:
	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);

	return ret;
}

static void exynos_tmu_get_efuse(struct platform_device *pdev, int id)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_tmu_platform_data *pdata = data->pdata;
	unsigned int trim_info;
	int timeout = 5;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	if (data->soc == SOC_ARCH_EXYNOS) {
		__raw_writel(EXYNOS_TRIMINFO_RELOAD1,
				data->base[id] + EXYNOS_TRIMINFO_CONFIG);
		__raw_writel(EXYNOS_TRIMINFO_RELOAD2,
				data->base[id] + EXYNOS_TRIMINFO_CONTROL);
		while (readl(data->base[id] + EXYNOS_TRIMINFO_CONTROL) & EXYNOS_TRIMINFO_RELOAD1) {
			if (!timeout) {
				pr_err("Thermal TRIMINFO register reload failed\n");
				break;
			}
			timeout--;
			cpu_relax();
			usleep_range(5, 10);
		}
	}

	/* Save trimming info in order to perform calibration */
	trim_info = readl(data->base[id] + EXYNOS_TMU_REG_TRIMINFO);
	data->temp_error1[id] = trim_info & EXYNOS_TMU_TRIM_TEMP_MASK;
	data->temp_error2[id] = ((trim_info >> 8) & EXYNOS_TMU_TRIM_TEMP_MASK);

#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5422)
	if (data->temp_error1[id] == 0)
		data->temp_error1[id] = pdata->efuse_value;
#else
	if ((EFUSE_MIN_VALUE > data->temp_error1[id]) || (data->temp_error1[id] > EFUSE_MAX_VALUE) ||
			(data->temp_error1[id] == 0))
		data->temp_error1[id] = pdata->efuse_value;
#endif
	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
}

static void exynos_tmu_control(struct platform_device *pdev, int id, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_tmu_platform_data *pdata = data->pdata;
	unsigned int con, interrupt_en;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	con = pdata->reference_voltage << EXYNOS_TMU_REF_VOLTAGE_SHIFT |
		pdata->gain << EXYNOS_TMU_GAIN_SHIFT;

	if (data->soc != SOC_ARCH_EXYNOS4210)
		con |= pdata->noise_cancel_mode << EXYNOS_TMU_TRIP_MODE_SHIFT;

	if (on) {
		con |= (EXYNOS_TMU_CORE_ON | EXYNOS_THERM_TRIP_EN);
		interrupt_en =
			pdata->trigger_level7_en << FALL_LEVEL7_SHIFT |
			pdata->trigger_level6_en << FALL_LEVEL6_SHIFT |
			pdata->trigger_level5_en << FALL_LEVEL5_SHIFT |
			pdata->trigger_level4_en << FALL_LEVEL4_SHIFT |
			pdata->trigger_level3_en << FALL_LEVEL3_SHIFT |
			pdata->trigger_level2_en << FALL_LEVEL2_SHIFT |
			pdata->trigger_level1_en << FALL_LEVEL1_SHIFT |
			pdata->trigger_level0_en << FALL_LEVEL0_SHIFT |
			pdata->trigger_level7_en << RISE_LEVEL7_SHIFT |
			pdata->trigger_level6_en << RISE_LEVEL6_SHIFT |
			pdata->trigger_level5_en << RISE_LEVEL5_SHIFT |
			pdata->trigger_level4_en << RISE_LEVEL4_SHIFT |
			pdata->trigger_level3_en << RISE_LEVEL3_SHIFT |
			pdata->trigger_level2_en << RISE_LEVEL2_SHIFT |
			pdata->trigger_level1_en << RISE_LEVEL1_SHIFT |
			pdata->trigger_level0_en;
	} else {
		con |= EXYNOS_TMU_CORE_OFF;
		interrupt_en = 0; /* Disable all interrupts */
	}
	con |= EXYNOS_THERM_TRIP_EN;
#if defined(CONFIG_SOC_EXYNOS5430)
	if (id == EXYNOS_GPU_NUMBER)
		con |= EXYNOS_MUX_ADDR;
#elif defined(CONFIG_SOC_EXYNOS5422)
	con |= EXYNOS_MUX_ADDR;
#endif

	writel(interrupt_en, data->base[id] + EXYNOS_TMU_REG_INTEN);
	writel(con, data->base[id] + EXYNOS_TMU_REG_CONTROL);

	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
}

static int exynos_tmu_read(struct exynos_tmu_data *data)
{
	u8 temp_code, status;
	int temp, i, max = INT_MIN, min = INT_MAX, gpu_temp = 0;
	int alltemp[EXYNOS_TMU_COUNT] = {0, };
	int timeout = 20000;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		while (1) {
			status = readb(data->base[i] + EXYNOS_TMU_REG_STATUS);
			if (status)
				break;

			timeout--;
			if (!timeout) {
				pr_err("%s: timeout TMU busy\n", __func__);
				break;
			}

			cpu_relax();
			usleep_range(1, 2);
		};

		temp_code = readb(data->base[i] + EXYNOS_TMU_REG_CURRENT_TEMP);
		temp = code_to_temp(data, temp_code, i);
		alltemp[i] = temp;

		if (i == EXYNOS_GPU_NUMBER) {
			gpu_temp = temp;
		} else {
			if (temp > max)
				max = temp;
			if (temp < min)
				min = temp;
		}

	}
	exynos_check_tmu_noti_state(min, max);
	exynos_check_mif_noti_state(max);
	exynos_check_gpu_noti_state(gpu_temp);

	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
#if defined(CONFIG_CPU_THERMAL_IPA)
	check_switch_ipa_on(max);
#endif
	pr_debug("[TMU] TMU0 = %d, TMU1 = %d, TMU2 = %d, TMU3 = %d, TMU4 = %d    MAX = %d, GPU = %d\n",
			alltemp[0], alltemp[1], alltemp[2], alltemp[3], alltemp[4], max, gpu_temp);

	return max;
}

#ifdef CONFIG_THERMAL_EMULATION
static int exynos_tmu_set_emulation(void *drv_data, unsigned long temp)
{
	struct exynos_tmu_data *data = drv_data;
	unsigned int reg;
	int ret = -EINVAL;

	if (data->soc == SOC_ARCH_EXYNOS4210)
		goto out;

	if (temp && temp < MCELSIUS)
		goto out;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	reg = readl(data->base + EXYNOS_EMUL_CON);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		if (temp) {
			temp /= MCELSIUS;

			reg = (EXYNOS_EMUL_TIME << EXYNOS_EMUL_TIME_SHIFT) |
				(temp_to_code(data, temp, i)
				 << EXYNOS_EMUL_DATA_SHIFT) | EXYNOS_EMUL_ENABLE;
		} else {
			reg &= ~EXYNOS_EMUL_ENABLE;
		}
	}

	writel(reg, data->base + EXYNOS_EMUL_CON);

	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
	return 0;
out:
	return ret;
}
#else
static int exynos_tmu_set_emulation(void *drv_data,	unsigned long temp)
	{ return -EINVAL; }
#endif/*CONFIG_THERMAL_EMULATION*/

static void exynos_tmu_work(struct work_struct *work)
{
	struct exynos_tmu_data *data = container_of(work,
			struct exynos_tmu_data, irq_work);
	int i;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);
	if (data->soc != SOC_ARCH_EXYNOS4210)
		for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		writel(EXYNOS_TMU_CLEAR_RISE_INT | EXYNOS_TMU_CLEAR_FALL_INT,
				data->base[i] + EXYNOS_TMU_REG_INTCLEAR);
		}
	else
		writel(EXYNOS4210_TMU_INTCLEAR_VAL,
				data->base[0] + EXYNOS_TMU_REG_INTCLEAR);
	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
	exynos_report_trigger();
	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		enable_irq(data->irq[i]);
}

static irqreturn_t exynos_tmu_irq(int irq, void *id)
{
	struct exynos_tmu_data *data = id;
	int i;

	pr_debug("[TMUIRQ] irq = %d\n", irq);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		disable_irq_nosync(data->irq[i]);
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}
static struct thermal_sensor_conf exynos_sensor_conf = {
	.name			= "exynos-therm",
	.read_temperature	= (int (*)(void *))exynos_tmu_read,
	.write_emul_temp	= exynos_tmu_set_emulation,
};
#if defined(CONFIG_CPU_THERMAL_IPA)
static struct ipa_sensor_conf ipa_sensor_conf = {
	.read_soc_temperature	= (int (*)(void *))exynos_tmu_read,
};
#endif
static int exynos_pm_notifier(struct notifier_block *notifier,
		unsigned long pm_event, void *v)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		is_suspending = true;
		exynos_tmu_call_notifier(TMU_COLD, 0);
		exynos_gpu_call_notifier(TMU_COLD);
		break;
	case PM_POST_SUSPEND:
		is_suspending = false;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_pm_nb = {
	.notifier_call = exynos_pm_notifier,
};

#if defined(CONFIG_CPU_EXYNOS4210)
static struct exynos_tmu_platform_data const exynos4210_default_tmu_data = {
	.threshold = 80,
	.trigger_levels[0] = 5,
	.trigger_levels[1] = 20,
	.trigger_levels[2] = 30,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 0,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 15,
	.reference_voltage = 7,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.freq_tab[0] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 85,
	},
	.freq_tab[1] = {
		.freq_clip_max = 200 * 1000,
		.temp_level = 100,
	},
	.freq_tab_count = 2,
	.type = SOC_ARCH_EXYNOS4210,
};
#define EXYNOS4210_TMU_DRV_DATA (&exynos4210_default_tmu_data)
#else
#define EXYNOS4210_TMU_DRV_DATA (NULL)
#endif

#if defined(CONFIG_SOC_EXYNOS5250) || defined(CONFIG_SOC_EXYNOS4412)
static struct exynos_tmu_platform_data const exynos_default_tmu_data = {
	.threshold_falling = 10,
	.trigger_levels[0] = 85,
	.trigger_levels[1] = 103,
	.trigger_levels[2] = 110,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 0,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 85,
	},
	.freq_tab[1] = {
		.freq_clip_max = 200 * 1000,
		.temp_level = 103,
	},
	.freq_tab_count = 2,
	.type = SOC_ARCH_EXYNOS,
};
#define EXYNOS_TMU_DRV_DATA (&exynos_default_tmu_data)
#else
#define EXYNOS_TMU_DRV_DATA (NULL)
#endif

#if defined(CONFIG_SOC_EXYNOS5430_REV_0)
static struct exynos_tmu_platform_data const exynos5430_evt0_tmu_data = {
	.threshold_falling = 2,
	.trigger_levels[0] = 55,
	.trigger_levels[1] = 60,
	.trigger_levels[2] = 65,
	.trigger_levels[3] = 110,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 7,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 1000 * 1000,	/* max frequency of Eagle is 1.0Ghz temporarily. */
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1200 * 1000,
#endif
		.temp_level = 55,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[1] = {
		.freq_clip_max = 1000 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1000 * 1000,
#endif
		.temp_level = 60,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[2] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 800 * 1000,
#endif
		.temp_level = 65,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[3] = {
		.freq_clip_max = 700 * 1000,	/* eagle need to be hotplugged-out */
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 700 * 1000,
#endif
		.temp_level = 70,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 3,
	.freq_tab_count = 4,
	.type = SOC_ARCH_EXYNOS,
	.clock_count = 2,
	.clk_name[0] = "pclk_tmu0_apbif",
	.clk_name[1] = "pclk_tmu1_apbif",
};
#define EXYNOS5430_EVT0_TMU_DRV_DATA (&exynos5430_evt0_tmu_data)
#else
#define EXYNOS5430_EVT0_TMU_DRV_DATA (NULL)
#endif

#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
static struct exynos_tmu_platform_data const exynos5430_tmu_data = {
	.threshold_falling = 2,
	.trigger_levels[0] = 80,
	.trigger_levels[1] = 90,
	.trigger_levels[2] = 100,
	.trigger_levels[3] = 110,
	.trigger_levels[4] = 110,
	.trigger_levels[5] = 110,
	.trigger_levels[6] = 110,
	.trigger_levels[7] = 115,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.trigger_level4_en = 1,
	.trigger_level5_en = 1,
	.trigger_level6_en = 1,
	.trigger_level7_en = 1,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 75,
	.freq_tab[0] = {
		.freq_clip_max = 1300 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1500 * 1000,
#endif
		.temp_level = 80,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[1] = {
		.freq_clip_max = 1100 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1500 * 1000,
#endif
		.temp_level = 90,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[2] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1500 * 1000,
#endif
		.temp_level = 100,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[3] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 800 * 1000,
#endif
		.temp_level = 110,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[4] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 800 * 1000,
#endif
		.temp_level = 110,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[5] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 800 * 1000,
#endif
		.temp_level = 110,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[6] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 800 * 1000,
#endif
		.temp_level = 110,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 6,
	.freq_tab_count = 7,
	.type = SOC_ARCH_EXYNOS5430,
	.clock_count = 2,
	.clk_name[0] = "pclk_tmu0_apbif",
	.clk_name[1] = "pclk_tmu1_apbif",
};
#define EXYNOS5430_TMU_DRV_DATA (&exynos5430_tmu_data)
#else
#define EXYNOS5430_TMU_DRV_DATA (NULL)
#endif

#if defined(CONFIG_SOC_EXYNOS5422)
static struct exynos_tmu_platform_data const exynos5_tmu_data = {
	.threshold_falling = 2,
	.trigger_levels[0] = 95,
	.trigger_levels[1] = 100,
	.trigger_levels[2] = 105,
	.trigger_levels[3] = 110,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 900 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1500 * 1000,
#endif
		.temp_level = 95,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[1] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1200 * 1000,
#endif
		.temp_level = 100,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[2] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1200 * 1000,
#endif
		.temp_level = 105,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 2,
	.freq_tab_count = 3,
	.type = SOC_ARCH_EXYNOS,
	.clock_count = 2,
	.clk_name[0] = "tmu",
	.clk_name[1] = "tmu_gpu",
};
#define EXYNOS5422_TMU_DRV_DATA (&exynos5_tmu_data)
#else
#define EXYNOS5422_TMU_DRV_DATA (NULL)
#endif

#ifdef CONFIG_OF
static const struct of_device_id exynos_tmu_match[] = {
	{
		.compatible = "samsung,exynos4210-tmu",
		.data = (void *)EXYNOS4210_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos4412-tmu",
		.data = (void *)EXYNOS_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos5250-tmu",
		.data = (void *)EXYNOS_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos5430-tmu",
#if defined(CONFIG_SOC_EXYNOS5430_REV_0)
		.data = (void *)EXYNOS5430_EVT0_TMU_DRV_DATA,
#else
		.data = (void *)EXYNOS5430_TMU_DRV_DATA,
#endif
	},
	{
		.compatible = "samsung,exynos5422-tmu",
		.data = (void *)EXYNOS5422_TMU_DRV_DATA,
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_tmu_match);
#endif

static struct platform_device_id exynos_tmu_driver_ids[] = {
	{
		.name		= "exynos4210-tmu",
		.driver_data    = (kernel_ulong_t)EXYNOS4210_TMU_DRV_DATA,
	},
	{
		.name		= "exynos5250-tmu",
		.driver_data    = (kernel_ulong_t)EXYNOS_TMU_DRV_DATA,
	},
	{
		.name		= "exynos5430-tmu",
#if defined(CONFIG_SOC_EXYNOS5430_REV_0)
		.driver_data	= (kernel_ulong_t)EXYNOS5430_EVT0_TMU_DRV_DATA,
#else
		.driver_data	= (kernel_ulong_t)EXYNOS5430_TMU_DRV_DATA,
#endif
	},
	{
		.name		= "exynos5422-tmu",
		.driver_data	= (kernel_ulong_t)EXYNOS5422_TMU_DRV_DATA,
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, exynos_tmu_driver_ids);

static inline struct  exynos_tmu_platform_data *exynos_get_driver_data(
			struct platform_device *pdev)
{
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		const struct of_device_id *match;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		init_mp_cpumask_set();
#endif
		match = of_match_node(exynos_tmu_match, pdev->dev.of_node);
		if (!match)
			return NULL;
		return (struct exynos_tmu_platform_data *) match->data;
	}
#endif
	return (struct exynos_tmu_platform_data *)
			platform_get_device_id(pdev)->driver_data;
}

/* sysfs interface : /sys/devices/10060000.tmu/temp */
static ssize_t
exynos_thermal_sensor_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_tmu_data *data = th_zone->sensor_conf->private_data;
	u8 temp_code;
	unsigned long temp[EXYNOS_TMU_COUNT] = {0,};
	int i, len = 0;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		temp_code = readb(data->base[i] + EXYNOS_TMU_REG_CURRENT_TEMP);
		if (temp_code == 0xff)
			continue;
		temp[i] = code_to_temp(data, temp_code, i) * MCELSIUS;
	}

	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		len += snprintf(&buf[len], PAGE_SIZE, "sensor%d : %ld\n", i, temp[i]);

	return len;
}

static DEVICE_ATTR(temp, S_IRUSR | S_IRGRP, exynos_thermal_sensor_temp, NULL);

static ssize_t exynos_throttle_counter(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int len = 0;

	len += snprintf(&buf[len], PAGE_SIZE, "TMU_NORMAL   :%ld\n", ctmu_normal);
	len += snprintf(&buf[len], PAGE_SIZE, "TMU_COLD     :%ld\n", ctmu_cold);
	len += snprintf(&buf[len], PAGE_SIZE, "TMU_HOT      :%ld\n", ctmu_hot);
	len += snprintf(&buf[len], PAGE_SIZE, "TMU_CRITICAL :%ld\n", ctmu_critical);
	len += snprintf(&buf[len], PAGE_SIZE, "THROTTLE_CPU :%ld\n", throttle_cpu);

	return len;
}

static DEVICE_ATTR(throttle, S_IRUGO, exynos_throttle_counter, NULL);

static struct attribute *exynos_thermal_sensor_attributes[] = {
	&dev_attr_temp.attr,
	&dev_attr_throttle.attr,
	NULL
};

static const struct attribute_group exynos_thermal_sensor_attr_group = {
	.attrs = exynos_thermal_sensor_attributes,
};

static void exynos_tmu_regdump(struct platform_device *pdev, int id)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int reg_data;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_TRIMINFO);
	pr_info("TRIMINFO[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_CONTROL);
	pr_info("TMU_CONTROL[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_CURRENT_TEMP);
	pr_info("CURRENT_TEMP[%d] = 0x%x\n", id, reg_data);
#if defined(CONFIG_SOC_EXYNOS5430_REV_1)
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_RISE3_0);
	pr_info("THRESHOLD_TEMP_RISE3_0[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_RISE7_4);
	pr_info("THRESHOLD_TEMP_RISE7_4[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_FALL3_0);
	pr_info("THRESHOLD_TEMP_FALL3_0[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_FALL7_4);
	pr_info("THRESHOLD_TEMP_FALL7_4[%d] = 0x%x\n", id, reg_data);
#else
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_RISE);
	pr_info("THRESHOLD_TEMP_RISE[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_FALL);
	pr_info("THRESHOLD_TEMP_FALL[%d] = 0x%x\n", id, reg_data);
#endif
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_INTEN);
	pr_info("INTEN[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_INTCLEAR);
	pr_info("INTCLEAR[%d] = 0x%x\n", id, reg_data);

	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
}

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static int exynos5_tmu_cpufreq_notifier(struct notifier_block *notifier, unsigned long event, void *v)
{
	int ret = 0, i;
	struct exynos_tmu_platform_data *pdata = exynos_tmu_pdev->dev.platform_data;

	switch (event) {
	case CPUFREQ_INIT_COMPLETE:
		ret = exynos_register_thermal(&exynos_sensor_conf);

		if (ret) {
			dev_err(&exynos_tmu_pdev->dev, "Failed to register thermal interface\n");
			sysfs_remove_group(&exynos_tmu_pdev->dev.kobj, &exynos_thermal_sensor_attr_group);
			unregister_pm_notifier(&exynos_pm_nb);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
			exynos_cpufreq_init_unregister_notifier(&exynos_cpufreq_nb);
#endif
			platform_set_drvdata(exynos_tmu_pdev, NULL);
			for (i = 0; i < pdata->clock_count; i++)
				clk_unprepare(tmudata->clk[i]);
			for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
				if (tmudata->irq[i])
					free_irq(tmudata->irq[i], tmudata);
			}
			kfree(tmudata);

			return ret;
		}
#if defined(CONFIG_CPU_THERMAL_IPA)
		ipa_sensor_conf.private_data = exynos_sensor_conf.private_data;
		ipa_register_thermal_sensor(&ipa_sensor_conf);
#endif
		break;
	}
	return 0;
}
#endif

unsigned long exynos_thermal_get_value(void)
{
	struct exynos_tmu_data *data = th_zone->sensor_conf->private_data;
	u8 temp_code;
	unsigned long temp[EXYNOS_TMU_COUNT] = {0,}, max_temp =0;
	int i;

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		temp_code = readb(data->base[i] + EXYNOS_TMU_REG_CURRENT_TEMP);
		if (temp_code == 0xff)
			continue;
		temp[i] = code_to_temp(data, temp_code, i);
		if(max_temp < temp[i]) max_temp = temp[i];
	}

	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);
	return max_temp;
}
EXPORT_SYMBOL(exynos_thermal_get_value);

static int exynos_tmu_probe(struct platform_device *pdev)
{
	struct exynos_tmu_data *data;
	struct exynos_tmu_platform_data *pdata = pdev->dev.platform_data;
	int ret, i, count = 0;
	int trigger_level_en[TRIP_EN_COUNT];
#ifdef CONFIG_SOC_EXYNOS5430_REV_1
	unsigned int spd_option_flag, spd_sel;
#endif

	exynos_tmu_pdev = pdev;
	is_suspending = false;

	if (!pdata)
		pdata = exynos_get_driver_data(pdev);

	if (!pdata) {
		dev_err(&pdev->dev, "No platform init data supplied.\n");
		return -ENODEV;
	}

#ifdef CONFIG_SOC_EXYNOS5430_REV_1
	exynos5430_get_egl_speed_option(&spd_option_flag, &spd_sel);
	if (spd_option_flag == EGL_DISABLE_SPD_OPTION)
		pdata->freq_tab[0].freq_clip_max = 1200 * 1000;
#endif

	data = devm_kzalloc(&pdev->dev, sizeof(struct exynos_tmu_data),
					GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate driver structure\n");
		return -ENOMEM;
	}

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	exynos_cpufreq_init_register_notifier(&exynos_cpufreq_nb);
#endif

	INIT_WORK(&data->irq_work, exynos_tmu_work);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		data->irq[i] = platform_get_irq(pdev, i);
		if (data->irq[i] < 0) {
			ret = data->irq[i];
			dev_err(&pdev->dev, "Failed to get platform irq\n");
			goto err_get_irq;
		}

		ret = request_irq(data->irq[i], exynos_tmu_irq,
				IRQF_TRIGGER_RISING, "exynos_tmu", data);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request irq: %d\n", data->irq[i]);
			goto err_request_irq;
		}

		data->mem[i] = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!data->mem[i]) {
			ret = -ENOENT;
			dev_err(&pdev->dev, "Failed to get platform resource\n");
			goto err_get_resource;
		}

		data->base[i] = devm_request_and_ioremap(&pdev->dev, data->mem[i]);
		if (IS_ERR(data->base[i])) {
			ret = PTR_ERR(data->base[i]);
			dev_err(&pdev->dev, "Failed to ioremap memory\n");
			goto err_io_remap;
		}
	}

	for (i = 0; i < pdata->clock_count; i++) {
		data->clk[i] = devm_clk_get(&pdev->dev, pdata->clk_name[i]);
		if (IS_ERR(data->clk[i])) {
			ret = PTR_ERR(data->clk);
			dev_err(&pdev->dev, "Failed to get clock\n");
			goto err_clk;
		}
	}

	for (i = 0; i < pdata->clock_count; i++) {
		ret = clk_prepare(data->clk[i]);
		if (ret) {
			dev_err(&pdev->dev, "Failed to prepare clk\n");
			goto err_prepare_clk;
		}
	}

	if (pdata->type == SOC_ARCH_EXYNOS || pdata->type == SOC_ARCH_EXYNOS4210 ||
			pdata->type == SOC_ARCH_EXYNOS5430)
		data->soc = pdata->type;
	else {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Platform not supported\n");
		goto err_soc_type;
	}

	data->pdata = pdata;
	tmudata = data;
	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	/* Save the eFuse value before initializing TMU */
	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_get_efuse(pdev, i);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		ret = exynos_tmu_initialize(pdev, i);
		if (ret) {
			dev_err(&pdev->dev, "Failed to initialize TMU\n");
			goto err_tmu;
		}

		exynos_tmu_control(pdev, i, true);
		exynos_tmu_regdump(pdev, i);
	}

	mutex_lock(&data->lock);
	clk_enable(data->clk[0]);
	clk_enable(data->clk[1]);
	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		unsigned int temp_code = readb(data->base[i] + EXYNOS_TMU_REG_CURRENT_TEMP);
		int temp = code_to_temp(data, temp_code, i);
		pr_debug("[TMU]temp[%d] : %d\n", i, temp);
	}
	clk_disable(data->clk[0]);
	clk_disable(data->clk[1]);
	mutex_unlock(&data->lock);


	/* Register the sensor with thermal management interface */
	(&exynos_sensor_conf)->private_data = data;
	exynos_sensor_conf.trip_data.trip_count = pdata->trigger_level0_en +
			pdata->trigger_level1_en + pdata->trigger_level2_en +
			pdata->trigger_level3_en + pdata->trigger_level4_en +
			pdata->trigger_level5_en + pdata->trigger_level6_en +
			pdata->trigger_level7_en;

	trigger_level_en[0] = pdata->trigger_level0_en;
	trigger_level_en[1] = pdata->trigger_level1_en;
	trigger_level_en[2] = pdata->trigger_level2_en;
	trigger_level_en[3] = pdata->trigger_level3_en;
	trigger_level_en[4] = pdata->trigger_level4_en;
	trigger_level_en[5] = pdata->trigger_level5_en;
	trigger_level_en[6] = pdata->trigger_level6_en;
	trigger_level_en[7] = pdata->trigger_level7_en;

	for (i = 0; i < TRIP_EN_COUNT; i++) {
		if (trigger_level_en[i]) {
			exynos_sensor_conf.trip_data.trip_val[count] =
				pdata->threshold + pdata->freq_tab[i].temp_level;
			count++;
		}
	}

	exynos_sensor_conf.trip_data.trigger_falling = pdata->threshold_falling;

	exynos_sensor_conf.cooling_data.freq_clip_count =
						pdata->freq_tab_count;
	for (i = 0; i < pdata->freq_tab_count; i++) {
		exynos_sensor_conf.cooling_data.freq_data[i].freq_clip_max =
					pdata->freq_tab[i].freq_clip_max;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		exynos_sensor_conf.cooling_data.freq_data[i].freq_clip_max_kfc =
					pdata->freq_tab[i].freq_clip_max_kfc;
#endif
		exynos_sensor_conf.cooling_data.freq_data[i].temp_level =
					pdata->freq_tab[i].temp_level;
		if (pdata->freq_tab[i].mask_val) {
			exynos_sensor_conf.cooling_data.freq_data[i].mask_val =
				pdata->freq_tab[i].mask_val;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
			exynos_sensor_conf.cooling_data.freq_data[i].mask_val_kfc =
				pdata->freq_tab[i].mask_val_kfc;
#endif
		} else
			exynos_sensor_conf.cooling_data.freq_data[i].mask_val =
				cpu_all_mask;
	}

	register_pm_notifier(&exynos_pm_nb);

	ret = sysfs_create_group(&pdev->dev.kobj, &exynos_thermal_sensor_attr_group);
	if (ret)
		dev_err(&exynos_tmu_pdev->dev, "cannot create thermal sensor attributes\n");

	is_cpu_hotplugged_out = false;

	return 0;

err_tmu:
	platform_set_drvdata(pdev, NULL);
err_soc_type:
	for (i = 0; i < pdata->clock_count; i++)
		clk_unprepare(data->clk[i]);
err_prepare_clk:
err_clk:
err_io_remap:
err_get_resource:
	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		if (data->irq[i])
			free_irq(data->irq[i], data);
	}
err_request_irq:
err_get_irq:
	kfree(data);

	return ret;
}

static int exynos_tmu_remove(struct platform_device *pdev)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_control(pdev, i, false);

	unregister_pm_notifier(&exynos_pm_nb);

	exynos_unregister_thermal();

	clk_unprepare(data->clk[0]);
	clk_unprepare(data->clk[1]);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_tmu_suspend(struct device *dev)
{
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_control(to_platform_device(dev), i, false);

	return 0;
}

static int exynos_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		exynos_tmu_initialize(pdev, i);
		exynos_tmu_control(pdev, i, true);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_tmu_pm,
			 exynos_tmu_suspend, exynos_tmu_resume);
#define EXYNOS_TMU_PM	(&exynos_tmu_pm)
#else
#define EXYNOS_TMU_PM	NULL
#endif

static struct platform_driver exynos_tmu_driver = {
	.driver = {
		.name   = "exynos-tmu",
		.owner  = THIS_MODULE,
		.pm     = EXYNOS_TMU_PM,
		.of_match_table = of_match_ptr(exynos_tmu_match),
	},
	.probe = exynos_tmu_probe,
	.remove	= exynos_tmu_remove,
	.id_table = exynos_tmu_driver_ids,
};

module_platform_driver(exynos_tmu_driver);

MODULE_DESCRIPTION("EXYNOS TMU Driver");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-tmu");
