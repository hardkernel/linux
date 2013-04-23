/*
 * drivers/cpufreq/exynos4x12-dvfs-hotplug.c
 *
 * DVFS cpu-hotplug driver for Samsung Exynos 4x12 SoCs
 *
 * Author: Gokturk Gezer <gokturk@apache.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/workqueue.h>

#include <plat/cpu.h>

// tunables
static unsigned int hotplug_min_cpu_count;
static unsigned int hotplug_max_cpu_count;
static unsigned int hotplug_freq_load_tolerance;
static unsigned int hotplug_tick_interval;
static unsigned int hotplug_tick_anticipation;

// cpufreq state variables
static char governor_name[CPUFREQ_NAME_LEN];
static unsigned int freq_current;
static unsigned int freq_min;
static unsigned int freq_max;

// hotplug state variables
static int freq_out_target;
static int freq_out_limit;
static int freq_in_target;
static int freq_in_limit;
static unsigned int available_cpu_count;
static unsigned int can_hotplug;
static struct delayed_work hotplug_dynamic_tick_work;
static struct delayed_work hotplug_fixed_tick_work;
void (*dynamic_tick_step)(void);
static unsigned int fixed_tick_cpu_count;
static unsigned int in_dynamic_mode;
static unsigned int dynamic_tick_suspended;
static unsigned int reached_max_cpu;
static unsigned int reached_min_cpu;
static unsigned int hotplug_effective_interval;

// function declerations
static void dynamic_hotplug_work();
static void fixed_hotplug_work();
static void start_hotplug_dynamic_tick();
static void start_hotplug_fixed_tick(unsigned int);
static void stop_hotplug_ticks();
static void set_anticipation();
static void unset_anticipation();
static void boot_fixed_cores();
static int cpu_adjust(unsigned int);
static void cpu_increase();
static void cpu_decrease();
static void hotplug_deploy(struct cpufreq_policy*);
static int hotplug_cpufreq_transition(struct notifier_block*,unsigned long, void*);


// sysfs objects and functions
struct kobject * dvfs_hotplug_kobject;

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", object);		\
}

show_one(min_cpu_count, hotplug_min_cpu_count);
show_one(max_cpu_count, hotplug_max_cpu_count);
show_one(freq_load_tolerance, hotplug_freq_load_tolerance);
show_one(tick_interval, hotplug_tick_interval);
show_one(tick_anticipation, hotplug_tick_anticipation);

static ssize_t store_min_cpu_count(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if(input > hotplug_max_cpu_count || input <= 0)
	{
		return -EINVAL;
	}

	// Adjusting cpu count for dynamic governors is deferred
	if(!in_dynamic_mode && fixed_tick_cpu_count == hotplug_min_cpu_count)
	{
		hotplug_min_cpu_count = input;
		start_hotplug_fixed_tick(input);
	}

	hotplug_min_cpu_count = input;

	return count;
}

static ssize_t store_max_cpu_count(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if(input < hotplug_min_cpu_count || input > available_cpu_count)
	{
		return -EINVAL;
	}

	// Adjusting cpu count for dynamic governors is deferred
	if(!in_dynamic_mode && fixed_tick_cpu_count == hotplug_max_cpu_count)
	{
		hotplug_max_cpu_count = input;
		start_hotplug_fixed_tick(input);
	}

	hotplug_max_cpu_count = input;

	return count;
}

static ssize_t store_freq_load_tolerance(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if(input <= 20 || input > 100)
	{
		return -EINVAL;
	}

	hotplug_freq_load_tolerance = input;

	return count;
}

static ssize_t store_tick_interval(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if(input <= 0)
	{
		return -EINVAL;
	}

	hotplug_tick_interval = input;

	return count;
}

static ssize_t store_tick_anticipation(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if(input != 0 && input != 1)
	{
		return -EINVAL;
	}

	unset_anticipation();
	hotplug_tick_anticipation = input;

	return count;
}

define_one_global_rw(min_cpu_count);
define_one_global_rw(max_cpu_count);
define_one_global_rw(freq_load_tolerance);
define_one_global_rw(tick_interval);
define_one_global_rw(tick_anticipation);

static struct attribute *hotplug_attributes[] = {
	&min_cpu_count.attr,
	&max_cpu_count.attr,
	&freq_load_tolerance.attr,
	&tick_interval.attr,
	&tick_anticipation.attr,
	NULL
};

// End of "sysfs objects and functions"

static void __hotplug_tick_step_freq_track()
{
	unsigned int tolerated_freq_in, tolerated_freq_out;

	tolerated_freq_in = freq_max / 100 * hotplug_freq_load_tolerance;
	tolerated_freq_out = freq_max / 100 * (hotplug_freq_load_tolerance - 20);
	if (tolerated_freq_out < freq_min)
		tolerated_freq_out = freq_min;

	if (freq_current >= tolerated_freq_in)
	{
		freq_out_target = 0;

		if (++freq_in_target == freq_in_limit)
		{
			cpu_increase();
			freq_in_target = 0;

			if (hotplug_tick_anticipation)
				set_anticipation();
		}
	}
	else if (freq_current <= tolerated_freq_out)
	{
		freq_in_target = 0;
		if (++freq_out_target == freq_out_limit)
		{
			cpu_decrease();
			freq_out_target = 0;

			if (hotplug_tick_anticipation)
				unset_anticipation();
		}
	}
}

static void dynamic_hotplug_work()
{
	(*dynamic_tick_step)();

	if(!dynamic_tick_suspended)
		start_hotplug_dynamic_tick();
}

static void fixed_hotplug_work()
{
	boot_fixed_cores();
}

static void start_hotplug_dynamic_tick()
{
	dynamic_tick_suspended = 0;

	schedule_delayed_work_on(0, &hotplug_dynamic_tick_work,
			msecs_to_jiffies(hotplug_effective_interval));
}

static void start_hotplug_fixed_tick(unsigned int cpu_count)
{
	fixed_tick_cpu_count = cpu_count;

	schedule_delayed_work_on(0, &hotplug_fixed_tick_work,
			msecs_to_jiffies(500));
}

static void stop_hotplug_ticks()
{
	cancel_delayed_work_sync(&hotplug_dynamic_tick_work);
	cancel_delayed_work_sync(&hotplug_fixed_tick_work);
}

static void boot_fixed_cores()
{
	int operation_count;
	unsigned int i,online_count;

	void (*fix_operation)(void) = cpu_increase;

	for(i = 0, online_count = 0; i < 4; i++)
	{
		if(cpu_online(i))
			online_count++;
	}

	operation_count = fixed_tick_cpu_count - online_count;
	if(operation_count < 0)
	{
		operation_count *= -1;
		fix_operation = cpu_decrease;
	}

	for(i = 0; i < operation_count; i++)
		(*fix_operation)();
}

static void set_anticipation()
{
	if(freq_in_limit > 1)
	{
		hotplug_effective_interval = hotplug_tick_interval * 3 / --freq_in_limit;
	}
}

static void unset_anticipation()
{
	hotplug_effective_interval = hotplug_tick_interval;
	freq_in_limit = 3;
}

static int cpu_adjust(unsigned int increase)
{
	unsigned int nr_cpus;

	nr_cpus = num_online_cpus();

	if(nr_cpus <= hotplug_min_cpu_count)
	{
		reached_min_cpu = 1;
		reached_max_cpu = 0;

		return dynamic_tick_suspended = (1 && !increase);
	}
	else if(nr_cpus > hotplug_min_cpu_count &&
			nr_cpus < hotplug_max_cpu_count)
	{
		reached_min_cpu = 0;
		reached_max_cpu = 0;
		dynamic_tick_suspended = 0;

		return 0;
	}
	else
	{
		reached_min_cpu = 0;
		reached_max_cpu = 1;

		return dynamic_tick_suspended = (1 && increase);
	}
}

static void cpu_increase()
{
	unsigned int i;

	if(cpu_adjust(1))
		return;

	for(i = 0; i < 4; i++)
	{
		if(!cpu_online(i))
		{
			cpu_up(i);
			break;
		}
	}
}

static void cpu_decrease()
{
	unsigned int i;

	if(cpu_adjust(0))
			return;

	for(i = 3; i >= 0; i--)
	{
		if(cpu_online(i))
		{
			cpu_down(i);
			break;
		}
	}
}

static void hotplug_deploy(struct cpufreq_policy * policy)
{
	unsigned int cpu;

	/*
	 * no governor, no hot-plug, all cores up
	 */
	if (!policy->governor)
	{
		in_dynamic_mode = 0;
		stop_hotplug_ticks();

		for_each_cpu_mask(cpu, policy->cpus[0])
		{
			if (!cpu_online(cpu))
				cpu_up(cpu);
		}

		return;
	}

	freq_max = policy->max;
	freq_min = policy->min;

	if( 0 != strnicmp(policy->governor->name, governor_name, CPUFREQ_NAME_LEN))
	{
		stop_hotplug_ticks();

		in_dynamic_mode = 0;

		strncpy(governor_name, policy->governor->name, CPUFREQ_NAME_LEN);

		if (0 == strnicmp(governor_name, "performance", CPUFREQ_NAME_LEN))
		{
			freq_current = freq_max;
			start_hotplug_fixed_tick(hotplug_max_cpu_count);
		}
		else if (0 == strnicmp(governor_name, "powersave", CPUFREQ_NAME_LEN))
		{
			freq_current = freq_min;
			start_hotplug_fixed_tick(hotplug_min_cpu_count);
		}
		else
		{
			dynamic_tick_step = __hotplug_tick_step_freq_track;
			in_dynamic_mode = 1;

			start_hotplug_dynamic_tick();
		}
	}
}

static int hotplug_cpufreq_transition(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = (struct cpufreq_freqs *) data;
	unsigned int tolerated_freq_in, tolerated_freq_out, rising, falling;

	if ((val == CPUFREQ_POSTCHANGE))
	{
		rising = (freqs->new > freq_current);
		falling = (freqs->new < freq_current);

		freq_current = freqs->new;

		if(in_dynamic_mode && dynamic_tick_suspended)
		{
			tolerated_freq_in = freq_max / 100 * hotplug_freq_load_tolerance;
			tolerated_freq_out = freq_max / 100 * (hotplug_freq_load_tolerance - 20);

			if((freq_current <= tolerated_freq_out && falling && !reached_min_cpu) ||
			   (freq_current >= tolerated_freq_in && rising && !reached_max_cpu))
			{
				start_hotplug_dynamic_tick();
			}
		}
	}

	return 0;
}

static int hotplug_cpufreq_policy(struct notifier_block *nb, unsigned long val,	void * data)
{
	struct cpufreq_policy * policy = (struct cpufreq_policy*) data;

	if (val != CPUFREQ_ADJUST)
		return 0;


	hotplug_deploy(policy);

	return 0;
}

static int hotplug_pm_transition(struct notifier_block *nb,	unsigned long val, void *data)
{
	switch (val) {
	case PM_SUSPEND_PREPARE:
		stop_hotplug_ticks();
		can_hotplug = 0;
		freq_out_target = 0;
		freq_in_target = 0;
		break;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		can_hotplug = 1;
		start_hotplug_dynamic_tick();
		break;
	}

	return 0;
}

static struct notifier_block dvfs_hotplug = { .notifier_call =
		hotplug_cpufreq_transition, };

static struct notifier_block dvfs_policy_change =
	{ .notifier_call = hotplug_cpufreq_policy, };

static struct notifier_block pm_hotplug =
	{ .notifier_call = hotplug_pm_transition, };

/*
 * Note : This function should be called after intialization of CPUFreq
 * driver for exynos4. The cpufreq_frequency_table for exynos4 should be
 * established before calling this function.
 */
static int __init exynos4_dvfs_hotplug_init(void)
{
	int i, register_result = 0;
	struct cpufreq_frequency_table *table;
	unsigned int freq;
	struct cpufreq_policy policy;

	hotplug_min_cpu_count = 2;
	if(soc_is_exynos4412())
		available_cpu_count = 4;
	else
		available_cpu_count = 2;
	hotplug_max_cpu_count = available_cpu_count;
	hotplug_freq_load_tolerance = 60;
	hotplug_tick_interval = 200;
	hotplug_tick_anticipation = 1;

	freq_out_target = 0;
	freq_out_limit = 3;
	freq_in_target = 0;
	freq_in_limit = 3;
	can_hotplug = 1;
	in_dynamic_mode = 0;
	dynamic_tick_suspended = 1;
	reached_max_cpu = 0;
	reached_min_cpu = 0;
	hotplug_effective_interval = hotplug_tick_interval;

	table = cpufreq_frequency_get_table(0);
	if (IS_ERR(table))
	{
		printk(KERN_ERR "%s: Check loading cpufreq before\n", __func__);
		return PTR_ERR(table);
	}

	for (i=0; table[i].frequency != CPUFREQ_TABLE_END; i++)
	{
		freq = table[i].frequency;

		if (freq != CPUFREQ_ENTRY_INVALID && freq > freq_max)
			freq_max = freq;
		else if (freq != CPUFREQ_ENTRY_INVALID && freq_min > freq)
			freq_min = freq;
	}

	freq_current = freq_max;

	INIT_DEFERRABLE_WORK(&hotplug_dynamic_tick_work, dynamic_hotplug_work);
	INIT_DEFERRABLE_WORK(&hotplug_fixed_tick_work, fixed_hotplug_work);

	printk(KERN_INFO "%s, max(%d),min(%d)\n", __func__, freq_max, freq_min);

	register_result |= register_pm_notifier(&pm_hotplug);

	register_result |= cpufreq_register_notifier(&dvfs_policy_change,
			CPUFREQ_POLICY_NOTIFIER);

	register_result |= cpufreq_register_notifier(&dvfs_hotplug,
			CPUFREQ_TRANSITION_NOTIFIER);

	dvfs_hotplug_kobject = kobject_create_and_add("dvfs-hotplug", &cpu_subsys.dev_root->kobj);
	sysfs_create_files(dvfs_hotplug_kobject,&hotplug_attributes);

	cpufreq_get_policy(&policy, 0);
	hotplug_deploy(&policy);

	return register_result;

}

late_initcall(exynos4_dvfs_hotplug_init);
