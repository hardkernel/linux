/*
 * drivers/input/touchscreen/gt801_ts.h
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DRIVERS_TOUCHSCREEN_GT801_TS_H
#define __DRIVERS_TOUCHSCREEN_GT801_TS_H

#define MAX_FINGER_NUM (5)
#define IOMUX_NAME_SIZE 48
#define GT801_TS_NAME "gt801"
#define FLAG_UP (1)
#define FLAG_DOWN (0)
#define GT801_DEBUG	0
#if GT801_DEBUG
	#define gt801printk(msg...) printk(msg);
#else
	#define gt801printk(msg...)
#endif

enum regadd {
	ptxh = 0, ptxl = 1, ptyh = 2, ptyl = 3, ptpressure = 4,
};
enum touchstate {
	TOUCH_UP = 0, TOUCH_DOWN = 1,
};	

struct gt801_ts_data {
	uint16_t		model;			/* 801. */
	uint16_t		pol;		/* swap x and y axes */
	uint16_t		x_min, x_max;
	uint16_t		y_min, y_max;
    uint16_t		addr;
    int 	use_irq;
	int 	gpio_pendown;
	int 	gpio_reset;
	int 	gpio_reset_active_low;
	int		pendown_iomux_mode;
	int		resetpin_iomux_mode;
	char	pendown_iomux_name[IOMUX_NAME_SIZE];
	char	resetpin_iomux_name[IOMUX_NAME_SIZE];
	char	phys[32];
	char	name[32];
	struct 	i2c_client *client;
    struct 	input_dev *input_dev;
    struct 	hrtimer timer;
    struct 	work_struct  work;
    struct 	early_suspend early_suspend;
};
#endif
