/*
 * drivers/amlogic/media/vout/lcd/lcd_vout.c
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

#include <linux/init.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/amlogic/iomap.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/amlogic/pm.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/media/vout/vinfo.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/media/vout/lcd/lcd_notify.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#ifdef CONFIG_AMLOGIC_LCD_EXTERN
#include <linux/amlogic/media/vout/lcd/lcd_extern.h>
#endif
#include "lcd_reg.h"
#include "lcd_common.h"
#include "lcd_clk_config.h"

#define LCD_CDEV_NAME  "lcd"

unsigned char lcd_debug_print_flag;
unsigned char lcd_resume_flag;
static struct aml_lcd_drv_s *lcd_driver;

struct mutex lcd_vout_mutex;
int lcd_vout_serve_bypass;

static char lcd_propname[20] = "null";

struct lcd_cdev_s {
	dev_t           devno;
	struct cdev     cdev;
	struct device   *dev;
};

static struct lcd_cdev_s *lcd_cdev;

static int lcd_vsync_cnt;
static int lcd_vsync_cnt_previous;

#define LCD_VSYNC_NONE_INTERVAL     msecs_to_jiffies(500)
static struct timer_list lcd_vsync_none_timer;

/* *********************************************************
 * lcd config define
 * *********************************************************
 */
static struct ttl_config_s lcd_ttl_config = {
	.clk_pol = 0,
	.sync_valid = ((1 << 1) | (1 << 0)),
	.swap_ctrl = ((0 << 1) | (0 << 0)),
};

static struct lvds_config_s lcd_lvds_config = {
	.lvds_repack = 1,
	.dual_port = 0,
	.pn_swap = 0,
	.port_swap = 0,
	.lane_reverse = 0,
	.port_sel = 0,
	.phy_vswing = LVDS_PHY_VSWING_DFT,
	.phy_preem = LVDS_PHY_PREEM_DFT,
	.phy_clk_vswing = LVDS_PHY_CLK_VSWING_DFT,
	.phy_clk_preem = LVDS_PHY_CLK_PREEM_DFT,
};

static struct vbyone_config_s lcd_vbyone_config = {
	.lane_count = 8,
	.region_num = 2,
	.byte_mode = 4,
	.color_fmt = 4,
	.phy_div = 1,
	.phy_vswing = VX1_PHY_VSWING_DFT,
	.phy_preem = VX1_PHY_PREEM_DFT,
	.intr_en = 1,
	.vsync_intr_en = 1,

	.ctrl_flag = 0,
	.power_on_reset_delay = VX1_PWR_ON_RESET_DLY_DFT,
	.hpd_data_delay = VX1_HPD_DATA_DELAY_DFT,
	.cdr_training_hold = VX1_CDR_TRAINING_HOLD_DFT,

	.hw_filter_time = 0,
	.hw_filter_cnt = 0,
};

static struct mlvds_config_s lcd_mlvds_config = {
	.channel_num = 6,
	.channel_sel0 = 0x76543210,
	.channel_sel1 = 0,
	.clk_phase = 0,
	.pn_swap = 0,
	.bit_swap = 0,
	.phy_vswing = 0,
	.phy_preem = 0,

	.pi_clk_sel = 0,
};

static struct p2p_config_s lcd_p2p_config = {
	.p2p_type = P2P_MAX,
	.lane_num = 12,
	.channel_sel0 = 0x76543210,
	.channel_sel1 = 0xba98,
	.pn_swap = 0,
	.bit_swap = 0,
	.phy_vswing = 0,
	.phy_preem = 0,
};

static unsigned char dsi_init_on_table[DSI_INIT_ON_MAX] = {0xff, 0xff};
static unsigned char dsi_init_off_table[DSI_INIT_OFF_MAX] = {0xff, 0xff};

static struct dsi_config_s lcd_mipi_config = {
	.lane_num = 4,
	.bit_rate_max = 550, /* MHz */
	.factor_numerator   = 0,
	.factor_denominator = 100,
	.operation_mode_init = 1,    /* 0=video mode, 1=command mode */
	.operation_mode_display = 0, /* 0=video mode, 1=command mode */
	.video_mode_type = 2, /* 0=sync_pulse, 1=sync_event, 2=burst */
	.clk_always_hs = 1, /* 0=disable, 1=enable */
	.phy_switch = 0,   /* 0=auto, 1=standard, 2=slow */

	.dsi_init_on  = &dsi_init_on_table[0],
	.dsi_init_off = &dsi_init_off_table[0],
	.extern_init = 0xff,
		/* ext_index if needed, 0xff for invalid */
	.check_en = 0,
	.check_reg = 0,
	.check_cnt = 0,
	.check_state = 0,
};

static struct lcd_power_ctrl_s lcd_power_config = {
	.cpu_gpio = {
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
		{.probe_flag = 0, .register_flag = 0,},
	},
	.power_on_step = {
		{
			.type = LCD_POWER_TYPE_MAX,
		},
	},
	.power_off_step = {
		{
			.type = LCD_POWER_TYPE_MAX,
		},
	},
};

static struct lcd_boot_ctrl_s lcd_boot_ctrl_config = {
	.lcd_type = LCD_TYPE_MAX,
	.lcd_bits = 0,
	.advanced_flag = 0,
	.lcd_init_level = 0,
	.debug_print_flag = 0,
	.debug_test_pattern = 0,
	.debug_para_source = 0,
	.debug_lcd_mode = 0,
};

/* index 0: valid flag */
static unsigned int vlock_param[LCD_VLOCK_PARAM_NUM] = {0};

static struct lcd_config_s lcd_config_dft = {
	.lcd_propname = lcd_propname,
	.lcd_basic = {
		.lcd_type = LCD_TYPE_MAX,
	},
	.lcd_timing = {
		.lcd_clk = 0,
		.clk_auto = 1,
		.ss_level = 0,
		.fr_adjust_type = 0,
		.bit_rate = 0,
	},
	.optical_info = {
		.hdr_support = 0,
		.features = 0,
		.primaries_r_x = 0,
		.primaries_r_y = 0,
		.primaries_g_x = 0,
		.primaries_g_y = 0,
		.primaries_b_x = 0,
		.primaries_b_y = 0,
		.white_point_x = 0,
		.white_point_y = 0,
		.luma_max = 0,
		.luma_min = 0,
		.luma_avg = 0,
	},
	.lcd_control = {
		.ttl_config = &lcd_ttl_config,
		.lvds_config = &lcd_lvds_config,
		.vbyone_config = &lcd_vbyone_config,
		.mlvds_config = &lcd_mlvds_config,
		.p2p_config = &lcd_p2p_config,
		.mipi_config = &lcd_mipi_config,
		.vlock_param = vlock_param,
	},
	.lcd_power = &lcd_power_config,
	.lcd_boot_ctrl = &lcd_boot_ctrl_config,
	.pinmux_flag = 0xff,
	.change_flag = 0,
	.retry_enable_flag = 0,
	.retry_enable_cnt = 0,

	.backlight_index = 0xff,
	.extern_index = 0xff,
};

static struct vinfo_s lcd_vinfo = {
	.name = "panel",
	.mode = VMODE_LCD,
	.viu_color_fmt = COLOR_FMT_RGB444,
	.viu_mux = VIU_MUX_ENCL,
	.vout_device = NULL,
};

struct aml_lcd_drv_s *aml_lcd_get_driver(void)
{
	return lcd_driver;
}
EXPORT_SYMBOL(aml_lcd_get_driver);
/* ********************************************************* */

static void lcd_power_ctrl(int status)
{
	struct lcd_power_ctrl_s *lcd_power = lcd_driver->lcd_config->lcd_power;
	struct lcd_power_step_s *power_step;
#ifdef CONFIG_AMLOGIC_LCD_EXTERN
	struct aml_lcd_extern_driver_s *ext_drv;
#endif
	unsigned int i, index, wait;
	int value = -1;

	LCDPR("%s: %d\n", __func__, status);
	i = 0;
	while (i < LCD_PWR_STEP_MAX) {
		if (status)
			power_step = &lcd_power->power_on_step[i];
		else
			power_step = &lcd_power->power_off_step[i];

		if (power_step->type >= LCD_POWER_TYPE_MAX)
			break;
		if (lcd_debug_print_flag) {
			LCDPR("power_ctrl: %d, step %d\n", status, i);
			LCDPR("%s: type=%d, index=%d, value=%d, delay=%d\n",
				__func__, power_step->type, power_step->index,
				power_step->value, power_step->delay);
		}
		switch (power_step->type) {
		case LCD_POWER_TYPE_CPU:
			index = power_step->index;
			lcd_cpu_gpio_set(index, power_step->value);
			break;
		case LCD_POWER_TYPE_PMU:
			LCDPR("to do\n");
			break;
		case LCD_POWER_TYPE_SIGNAL:
			if (status)
				lcd_driver->driver_init();
			else
				lcd_driver->driver_disable();
			break;
#ifdef CONFIG_AMLOGIC_LCD_EXTERN
		case LCD_POWER_TYPE_EXTERN:
			index = power_step->index;
			ext_drv = aml_lcd_extern_get_driver(index);
			if (ext_drv) {
				if (status) {
					if (ext_drv->power_on)
						ext_drv->power_on();
					else
						LCDERR("no ext power on\n");
				} else {
					if (ext_drv->power_off)
						ext_drv->power_off();
					else
						LCDERR("no ext power off\n");
				}
			}
			break;
#endif
		case LCD_POWER_TYPE_WAIT_GPIO:
			index = power_step->index;
			lcd_cpu_gpio_set(index, LCD_GPIO_INPUT);
			LCDPR("lcd_power_type_wait_gpio wait\n");
			for (wait = 0; wait < power_step->delay; wait++) {
				value = lcd_cpu_gpio_get(index);
				if (value == power_step->value) {
					LCDPR("wait_gpio %d ok\n", value);
					break;
				}
				mdelay(1);
			}
			if (wait == power_step->delay)
				LCDERR("wait_gpio %d timeout!\n", value);
			break;
		case LCD_POWER_TYPE_CLK_SS:
			break;
		default:
			break;
		}
		if ((power_step->type != LCD_POWER_TYPE_WAIT_GPIO) &&
			(power_step->delay > 0))
			mdelay(power_step->delay);
		i++;
	}
	if (lcd_debug_print_flag)
		LCDPR("%s: %d finished\n", __func__, status);
}

static void lcd_power_encl_on(void)
{
	mutex_lock(&lcd_vout_mutex);

	if (lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) {
		LCDPR("%s: on already\n", __func__);
		mutex_unlock(&lcd_vout_mutex);
		return;
	}

	lcd_driver->driver_init_pre();
	lcd_driver->lcd_status |= LCD_STATUS_ENCL_ON;

	/* vsync_none_timer conditional enabled to save cpu loading */
	if (lcd_driver->viu_sel == LCD_VIU_SEL_NONE) {
		if (lcd_driver->vsync_none_timer_flag == 0) {
			lcd_vsync_none_timer.expires =
				jiffies + LCD_VSYNC_NONE_INTERVAL;
			add_timer(&lcd_vsync_none_timer);
			lcd_driver->vsync_none_timer_flag = 1;
			LCDPR("add lcd_vsync_none_timer handler\n");
		}
	} else {
		if (lcd_driver->vsync_none_timer_flag) {
			del_timer_sync(&lcd_vsync_none_timer);
			lcd_driver->vsync_none_timer_flag = 0;
		}
	}

	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_power_encl_off(void)
{
	mutex_lock(&lcd_vout_mutex);

	if (!(lcd_driver->lcd_status & LCD_STATUS_ENCL_ON)) {
		LCDPR("%s: off already\n", __func__);
		mutex_unlock(&lcd_vout_mutex);
		return;
	}
	lcd_driver->lcd_status &= ~LCD_STATUS_ENCL_ON;
	lcd_driver->driver_disable_post();

	if (lcd_driver->vsync_none_timer_flag) {
		del_timer_sync(&lcd_vsync_none_timer);
		lcd_driver->vsync_none_timer_flag = 0;
	}

	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_power_if_on(void)
{
	mutex_lock(&lcd_vout_mutex);
	if (!(lcd_driver->lcd_status & LCD_STATUS_IF_ON)) {
		lcd_driver->power_ctrl(1);
		lcd_driver->lcd_status |= LCD_STATUS_IF_ON;
	}
	lcd_driver->lcd_config->change_flag = 0;
	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_power_if_off(void)
{
	mutex_lock(&lcd_vout_mutex);
	if (lcd_driver->lcd_status & LCD_STATUS_IF_ON) {
		lcd_driver->lcd_status &= ~LCD_STATUS_IF_ON;
		lcd_driver->power_ctrl(0);
	}
	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_power_screen_black(void)
{
	mutex_lock(&lcd_vout_mutex);

	lcd_driver->lcd_mute_flag = (unsigned char)(1 | LCD_MUTE_UPDATE);
	LCDPR("set mute\n");

	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_power_screen_restore(void)
{
	mutex_lock(&lcd_vout_mutex);

	lcd_driver->lcd_mute_flag = (unsigned char)(0 | LCD_MUTE_UPDATE);
	LCDPR("clear mute\n");

	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_module_reset(void)
{
	mutex_lock(&lcd_vout_mutex);

	lcd_driver->lcd_status &= ~LCD_STATUS_ON;
	lcd_driver->power_ctrl(0);

	msleep(500);

	lcd_driver->driver_init_pre();
	lcd_driver->power_ctrl(1);
	lcd_driver->lcd_status |= LCD_STATUS_ON;
	lcd_driver->lcd_config->change_flag = 0;

	lcd_driver->lcd_mute_flag = (unsigned char)(0 | LCD_MUTE_UPDATE);
	LCDPR("clear mute\n");

	mutex_unlock(&lcd_vout_mutex);
}

static void lcd_resume_work(struct work_struct *p_work)
{
	mutex_lock(&lcd_driver->power_mutex);
	aml_lcd_notifier_call_chain(LCD_EVENT_POWER_ON, NULL);
	lcd_if_enable_retry(lcd_driver->lcd_config);
	LCDPR("%s finished\n", __func__);
	mutex_unlock(&lcd_driver->power_mutex);
}

static struct delayed_work lcd_test_delayed_work;
static void lcd_auto_test_delayed(struct work_struct *work)
{
	LCDPR("%s\n", __func__);
	mutex_lock(&lcd_driver->power_mutex);
	aml_lcd_notifier_call_chain(LCD_EVENT_POWER_ON, NULL);
	mutex_unlock(&lcd_driver->power_mutex);
}

static void lcd_auto_test(unsigned char flag)
{
	lcd_driver->lcd_test_state = flag;
	if (lcd_driver->workqueue) {
		queue_delayed_work(lcd_driver->workqueue,
			&lcd_test_delayed_work,
			msecs_to_jiffies(20000));
	} else {
		schedule_delayed_work(&lcd_test_delayed_work,
			msecs_to_jiffies(20000));
	}
}

static int lcd_vsync_print_cnt;
static inline void lcd_vsync_handler(void)
{
	int flag;
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	struct lcd_config_s *pconf;
#endif

	if (lcd_driver == NULL)
		return;

#ifdef CONFIG_AMLOGIC_LCD_TABLET
	pconf = lcd_driver->lcd_config;
	if (pconf->lcd_control.mipi_config->dread) {
		if (pconf->lcd_control.mipi_config->dread->flag) {
			lcd_mipi_test_read(
				pconf->lcd_control.mipi_config->dread);
			pconf->lcd_control.mipi_config->dread->flag = 0;
		}
	}
#endif

	if (lcd_driver->lcd_mute_flag & LCD_MUTE_UPDATE) {
		flag = lcd_driver->lcd_mute_flag & 0x1;
		if (flag) {
			if (lcd_driver->lcd_mute_state == 0) {
				lcd_driver->lcd_mute_state = 1;
				lcd_driver->lcd_screen_black();
			}
		} else {
			if (lcd_driver->lcd_mute_state) {
				lcd_driver->lcd_mute_state = 0;
				lcd_driver->lcd_screen_restore();
			}
		}
		lcd_driver->lcd_mute_flag &= ~(LCD_MUTE_UPDATE);
	}

	if (lcd_driver->lcd_test_flag & LCD_TEST_UPDATE) {
		flag = lcd_driver->lcd_test_flag & 0xf;
		if (flag != lcd_driver->lcd_test_state) {
			lcd_driver->lcd_test_state = (unsigned char)flag;
			lcd_debug_test(flag);
		}
		lcd_driver->lcd_test_flag &= ~(LCD_TEST_UPDATE);
	}

	if (lcd_vsync_print_cnt++ >= LCD_DEBUG_VSYNC_INTERVAL) {
		lcd_vsync_print_cnt = 0;
		if (lcd_debug_print_flag & LCD_DEBUG_LEVEL_VSYNC) {
			LCDPR("%s: viu_sel: %d\n",
				__func__, lcd_driver->viu_sel);
		}
	}
}

static irqreturn_t lcd_vsync_isr(int irq, void *dev_id)
{
	if (lcd_driver == NULL)
		return IRQ_HANDLED;

	if ((lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) == 0)
		return IRQ_HANDLED;

	if (lcd_driver->viu_sel == 1) {
		lcd_vsync_handler();
		if (lcd_vsync_cnt++ >= 65536)
			lcd_vsync_cnt = 0;
	}
	return IRQ_HANDLED;
}

static irqreturn_t lcd_vsync2_isr(int irq, void *dev_id)
{
	if (lcd_driver == NULL)
		return IRQ_HANDLED;

	if ((lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) == 0)
		return IRQ_HANDLED;

	if (lcd_driver->viu_sel == 2) {
		lcd_vsync_handler();
		if (lcd_vsync_cnt++ >= 65536)
			lcd_vsync_cnt = 0;
	}
	return IRQ_HANDLED;
}

#define LCD_WAIT_VSYNC_TIMEOUT    50000
static void lcd_wait_vsync(void)
{
	int line_cnt, line_cnt_previous;
	int i = 0;

	line_cnt = 0x1fff;
	line_cnt_previous = lcd_vcbus_getb(ENCL_INFO_READ, 16, 13);
	while (i++ < LCD_WAIT_VSYNC_TIMEOUT) {
		line_cnt = lcd_vcbus_getb(ENCL_INFO_READ, 16, 13);
		if (line_cnt < line_cnt_previous)
			break;
		line_cnt_previous = line_cnt;
		udelay(2);
	}
	/*LCDPR("line_cnt=%d, line_cnt_previous=%d, i=%d\n",
	 *	line_cnt, line_cnt_previous, i);
	 */
}

static void lcd_vsync_none_timer_handler(unsigned long arg)
{
	if (lcd_driver == NULL)
		return;

	if ((lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) == 0)
		goto lcd_vsync_none_timer_handler_end;

	if (lcd_vsync_cnt == lcd_vsync_cnt_previous) {
		lcd_wait_vsync();
		lcd_vsync_handler();
	}

	lcd_vsync_cnt_previous = lcd_vsync_cnt;

lcd_vsync_none_timer_handler_end:
	if (lcd_driver->vsync_none_timer_flag) {
		lcd_vsync_none_timer.expires =
			jiffies + LCD_VSYNC_NONE_INTERVAL;
		add_timer(&lcd_vsync_none_timer);
	}
}

/* ****************************************
 * lcd notify
 * ****************************************
 */
static int lcd_power_encl_on_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if ((event & LCD_EVENT_ENCL_ON) == 0)
		return NOTIFY_DONE;

	if (lcd_debug_print_flag)
		LCDPR("%s: 0x%lx\n", __func__, event);

	if (lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) {
		LCDPR("lcd is already enabled\n");
		return NOTIFY_OK;
	}

	lcd_power_encl_on();

	return NOTIFY_OK;
}

static struct notifier_block lcd_power_encl_on_nb = {
	.notifier_call = lcd_power_encl_on_notifier,
	.priority = LCD_PRIORITY_POWER_ENCL_ON,
};

static int lcd_power_encl_off_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if ((event & LCD_EVENT_ENCL_OFF) == 0)
		return NOTIFY_DONE;

	if (lcd_debug_print_flag)
		LCDPR("%s: 0x%lx\n", __func__, event);
	lcd_power_encl_off();

	return NOTIFY_OK;
}

static struct notifier_block lcd_power_encl_off_nb = {
	.notifier_call = lcd_power_encl_off_notifier,
	.priority = LCD_PRIORITY_POWER_ENCL_OFF,
};

static int lcd_power_if_on_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if ((event & LCD_EVENT_IF_ON) == 0)
		return NOTIFY_DONE;

	if (lcd_debug_print_flag)
		LCDPR("%s: 0x%lx\n", __func__, event);

	if (lcd_driver->lcd_status & LCD_STATUS_IF_ON) {
		LCDPR("lcd interface is already enabled\n");
		return NOTIFY_OK;
	}

	if (lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) {
		lcd_power_if_on();
	} else {
		LCDERR("%s: can't power on when controller is off\n",
				__func__);
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block lcd_power_if_on_nb = {
	.notifier_call = lcd_power_if_on_notifier,
	.priority = LCD_PRIORITY_POWER_IF_ON,
};

static int lcd_power_if_off_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if ((event & LCD_EVENT_IF_OFF) == 0)
		return NOTIFY_DONE;

	if (lcd_debug_print_flag)
		LCDPR("%s: 0x%lx\n", __func__, event);
	lcd_power_if_off();

	return NOTIFY_OK;
}

static struct notifier_block lcd_power_if_off_nb = {
	.notifier_call = lcd_power_if_off_notifier,
	.priority = LCD_PRIORITY_POWER_IF_OFF,
};

static int lcd_power_screen_black_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if ((event & LCD_EVENT_SCREEN_BLACK) == 0)
		return NOTIFY_DONE;

	if (lcd_debug_print_flag)
		LCDPR("%s: 0x%lx\n", __func__, event);
	lcd_power_screen_black();

	return NOTIFY_OK;
}

static struct notifier_block lcd_power_screen_black_nb = {
	.notifier_call = lcd_power_screen_black_notifier,
	.priority = LCD_PRIORITY_SCREEN_BLACK,
};

static int lcd_power_screen_restore_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	if ((event & LCD_EVENT_SCREEN_RESTORE) == 0)
		return NOTIFY_DONE;

	if (lcd_debug_print_flag)
		LCDPR("%s: 0x%lx\n", __func__, event);
	lcd_power_screen_restore();

	return NOTIFY_OK;
}

static struct notifier_block lcd_power_screen_restore_nb = {
	.notifier_call = lcd_power_screen_restore_notifier,
	.priority = LCD_PRIORITY_SCREEN_RESTORE,
};

static int lcd_bl_select_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	unsigned int *index;

	if ((event & LCD_EVENT_BACKLIGHT_SEL) == 0)
		return NOTIFY_DONE;
	/* LCDPR("%s: 0x%lx\n", __func__, event); */

	index = (unsigned int *)data;
	*index = lcd_driver->lcd_config->backlight_index;

	return NOTIFY_OK;
}

static struct notifier_block lcd_bl_select_nb = {
	.notifier_call = lcd_bl_select_notifier,
};

static int lcd_extern_select_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	unsigned int *index;
	struct lcd_config_s *pconf = lcd_driver->lcd_config;

	if ((event & LCD_EVENT_EXTERN_SEL) == 0)
		return NOTIFY_DONE;
	/* LCDPR("%s: 0x%lx\n", __func__, event); */

	index = (unsigned int *)data;
	*index = pconf->extern_index;
	if (pconf->lcd_basic.lcd_type == LCD_MIPI) {
		if (*index == LCD_EXTERN_INDEX_INVALID)
			*index = pconf->lcd_control.mipi_config->extern_init;
	}

	return NOTIFY_OK;
}

static struct notifier_block lcd_extern_select_nb = {
	.notifier_call = lcd_extern_select_notifier,
};

static int lcd_vlock_param_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	unsigned int *param;

	if ((event & LCD_EVENT_VLOCK_PARAM) == 0)
		return NOTIFY_DONE;
	/* LCDPR("%s: 0x%lx\n", __func__, event); */

	param = (unsigned int *)data;
	memcpy(param, vlock_param,
		(LCD_VLOCK_PARAM_NUM * sizeof(unsigned int)));

	return NOTIFY_OK;
}

static struct notifier_block lcd_vlock_param_nb = {
	.notifier_call = lcd_vlock_param_notifier,
};

static int lcd_notifier_register(void)
{
	int ret = 0;

	ret = aml_lcd_notifier_register(&lcd_power_encl_on_nb);
	if (ret)
		LCDERR("register lcd_power_encl_on_nb failed\n");
	ret = aml_lcd_notifier_register(&lcd_power_encl_off_nb);
	if (ret)
		LCDERR("register lcd_power_encl_off_nb failed\n");
	ret = aml_lcd_notifier_register(&lcd_power_if_on_nb);
	if (ret)
		LCDERR("register lcd_power_if_on_nb failed\n");
	ret = aml_lcd_notifier_register(&lcd_power_if_off_nb);
	if (ret)
		LCDERR("register lcd_power_if_off_nb failed\n");
	ret = aml_lcd_notifier_register(&lcd_power_screen_black_nb);
	if (ret)
		LCDERR("register lcd_power_screen_black_nb failed\n");
	ret = aml_lcd_notifier_register(&lcd_power_screen_restore_nb);
	if (ret)
		LCDERR("register lcd_power_screen_restore_nb failed\n");

	ret = aml_lcd_notifier_register(&lcd_bl_select_nb);
	if (ret)
		LCDERR("register aml_bl_select_notifier failed\n");
	ret = aml_lcd_notifier_register(&lcd_extern_select_nb);
	if (ret)
		LCDERR("register lcd_extern_select_nb failed\n");
	ret = aml_lcd_notifier_register(&lcd_vlock_param_nb);
	if (ret)
		LCDERR("register lcd_vlock_param_nb failed\n");

	return 0;
}

static void lcd_notifier_unregister(void)
{
	aml_lcd_notifier_unregister(&lcd_power_screen_restore_nb);
	aml_lcd_notifier_unregister(&lcd_power_screen_black_nb);
	aml_lcd_notifier_unregister(&lcd_power_if_off_nb);
	aml_lcd_notifier_unregister(&lcd_power_if_on_nb);
	aml_lcd_notifier_unregister(&lcd_power_encl_off_nb);
	aml_lcd_notifier_unregister(&lcd_power_encl_on_nb);

	aml_lcd_notifier_unregister(&lcd_bl_select_nb);
	aml_lcd_notifier_unregister(&lcd_extern_select_nb);
	aml_lcd_notifier_unregister(&lcd_vlock_param_nb);
}
/* **************************************** */

/* ************************************************************* */
/* lcd ioctl                                                     */
/* ************************************************************* */
static int lcd_io_open(struct inode *inode, struct file *file)
{
	struct lcd_cdev_s *lcd_cdev;

	LCDPR("%s\n", __func__);
	lcd_cdev = container_of(inode->i_cdev, struct lcd_cdev_s, cdev);
	file->private_data = lcd_cdev;
	return 0;
}

static int lcd_io_release(struct inode *inode, struct file *file)
{
	LCDPR("%s\n", __func__);
	file->private_data = NULL;
	return 0;
}

static long lcd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *argp;
	int mcd_nr;
	struct lcd_optical_info_s *opt_info;

	opt_info = &lcd_driver->lcd_config->optical_info;
	mcd_nr = _IOC_NR(cmd);
	LCDPR("%s: cmd_dir = 0x%x, cmd_nr = 0x%x\n",
		__func__, _IOC_DIR(cmd), mcd_nr);

	argp = (void __user *)arg;
	switch (mcd_nr) {
	case LCD_IOC_NR_GET_HDR_INFO:
		if (copy_to_user(argp, opt_info,
			sizeof(struct lcd_optical_info_s))) {
			ret = -EFAULT;
		}
		break;
	case LCD_IOC_NR_SET_HDR_INFO:
		if (copy_from_user(opt_info, argp,
			sizeof(struct lcd_optical_info_s))) {
			ret = -EFAULT;
		} else {
			lcd_optical_vinfo_update();
			if (lcd_debug_print_flag) {
				LCDPR("set optical info:\n"
					"hdr_support          %d\n"
					"features             %d\n"
					"primaries_r_x        %d\n"
					"primaries_r_y        %d\n"
					"primaries_g_x        %d\n"
					"primaries_g_y        %d\n"
					"primaries_b_x        %d\n"
					"primaries_b_y        %d\n"
					"white_point_x        %d\n"
					"white_point_y        %d\n"
					"luma_max             %d\n"
					"luma_min             %d\n\n",
					opt_info->hdr_support,
					opt_info->features,
					opt_info->primaries_r_x,
					opt_info->primaries_r_y,
					opt_info->primaries_g_x,
					opt_info->primaries_g_y,
					opt_info->primaries_b_x,
					opt_info->primaries_b_y,
					opt_info->white_point_x,
					opt_info->white_point_y,
					opt_info->luma_max,
					opt_info->luma_min);
			}
		}
		break;
	default:
		LCDERR("not support ioctl cmd_nr: 0x%x\n", mcd_nr);
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long lcd_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	unsigned long ret;

	arg = (unsigned long)compat_ptr(arg);
	ret = lcd_ioctl(file, cmd, arg);
	return ret;
}
#endif

static const struct file_operations lcd_fops = {
	.owner          = THIS_MODULE,
	.open           = lcd_io_open,
	.release        = lcd_io_release,
	.unlocked_ioctl = lcd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = lcd_compat_ioctl,
#endif
};

static int lcd_fops_create(void)
{
	int ret = 0;

	lcd_cdev = kmalloc(sizeof(struct lcd_cdev_s), GFP_KERNEL);
	if (!lcd_cdev) {
		LCDERR("%s: failed to allocate lcd_cdev\n", __func__);
		return -1;
	}

	ret = alloc_chrdev_region(&lcd_cdev->devno, 0, 1, LCD_CDEV_NAME);
	if (ret < 0) {
		LCDERR("%s: failed to alloc devno\n", __func__);
		kfree(lcd_cdev);
		lcd_cdev = NULL;
		return -1;
	}

	cdev_init(&lcd_cdev->cdev, &lcd_fops);
	lcd_cdev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&lcd_cdev->cdev, lcd_cdev->devno, 1);
	if (ret) {
		LCDERR("%s: failed to add cdev\n", __func__);
		unregister_chrdev_region(lcd_cdev->devno, 1);
		kfree(lcd_cdev);
		lcd_cdev = NULL;
		return -1;
	}

	lcd_cdev->dev = device_create(lcd_driver->lcd_debug_class, NULL,
			lcd_cdev->devno, NULL, LCD_CDEV_NAME);
	if (IS_ERR(lcd_cdev->dev)) {
		LCDERR("%s: failed to add device\n", __func__);
		ret = PTR_ERR(lcd_cdev->dev);
		cdev_del(&lcd_cdev->cdev);
		unregister_chrdev_region(lcd_cdev->devno, 1);
		kfree(lcd_cdev);
		lcd_cdev = NULL;
		return -1;
	}

	if (lcd_debug_print_flag)
		LCDPR("%s OK\n", __func__);
	return 0;
}

static void lcd_fops_remove(void)
{
	cdev_del(&lcd_cdev->cdev);
	unregister_chrdev_region(lcd_cdev->devno, 1);
	kfree(lcd_cdev);
	lcd_cdev = NULL;
}
/* ************************************************************* */

static int lcd_vsync_irq_init(void)
{
	if (lcd_driver->res_vsync_irq) {
		if (request_irq(lcd_driver->res_vsync_irq->start,
			lcd_vsync_isr, IRQF_SHARED,
			"lcd_vsync", (void *)"lcd_vsync")) {
			LCDERR("can't request lcd_vsync_irq\n");
		} else {
			if (lcd_debug_print_flag)
				LCDPR("request lcd_vsync_irq successful\n");
		}
	}

	if (lcd_driver->res_vsync2_irq) {
		if (request_irq(lcd_driver->res_vsync2_irq->start,
			lcd_vsync2_isr, IRQF_SHARED,
			"lcd_vsync2", (void *)"lcd_vsync2")) {
			LCDERR("can't request lcd_vsync2_irq\n");
		} else {
			if (lcd_debug_print_flag)
				LCDPR("request lcd_vsync2_irq successful\n");
		}
	}

	/* add timer to monitor hpll frequency */
	init_timer(&lcd_vsync_none_timer);
	/* lcd_vsync_none_timer.data = NULL; */
	lcd_vsync_none_timer.function = lcd_vsync_none_timer_handler;
	lcd_vsync_none_timer.expires = jiffies + LCD_VSYNC_NONE_INTERVAL;
	/*add_timer(&lcd_vsync_none_timer);*/
	/*LCDPR("add lcd_vsync_none_timer handler\n"); */

	return 0;
}

static void lcd_vsync_irq_remove(void)
{
	if (lcd_driver->res_vsync_irq)
		free_irq(lcd_driver->res_vsync_irq->start, (void *)"lcd_vsync");

	if (lcd_driver->res_vsync2_irq) {
		free_irq(lcd_driver->res_vsync2_irq->start,
			(void *)"lcd_vsync");
	}

	if (lcd_driver->vsync_none_timer_flag) {
		del_timer_sync(&lcd_vsync_none_timer);
		lcd_driver->vsync_none_timer_flag = 0;
	}
}

static void lcd_init_vout(void)
{
	switch (lcd_driver->lcd_mode) {
#ifdef CONFIG_AMLOGIC_LCD_TV
	case LCD_MODE_TV:
		lcd_tv_vout_server_init();
		break;
#endif
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	case LCD_MODE_TABLET:
		lcd_tablet_vout_server_init();
		break;
#endif
	default:
		LCDERR("invalid lcd mode: %d\n", lcd_driver->lcd_mode);
		break;
	}
}

static int lcd_mode_probe(struct device *dev)
{
	switch (lcd_driver->lcd_mode) {
#ifdef CONFIG_AMLOGIC_LCD_TV
	case LCD_MODE_TV:
		lcd_tv_probe(dev);
		break;
#endif
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	case LCD_MODE_TABLET:
		lcd_tablet_probe(dev);
		break;
#endif
	default:
		LCDERR("invalid lcd mode: %d\n", lcd_driver->lcd_mode);
		break;
	}
	if (lcd_driver->lcd_status & LCD_STATUS_ENCL_ON)
		lcd_clk_gate_switch(1);

	lcd_tcon_probe(lcd_driver);
	lcd_debug_probe();
	lcd_fops_create();

	lcd_notifier_register();
	lcd_vsync_irq_init();

	/* add notifier for video sync_duration info refresh */
	vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE,
		&lcd_driver->lcd_info->mode);

	if (lcd_driver->lcd_auto_test)
		lcd_auto_test(lcd_driver->lcd_auto_test);

	return 0;
}

static int lcd_config_remove(struct device *dev)
{
	lcd_notifier_unregister();

	switch (lcd_driver->lcd_mode) {
#ifdef CONFIG_AMLOGIC_LCD_TV
	case LCD_MODE_TV:
		lcd_tv_vout_server_remove();
		lcd_tv_remove(dev);
		break;
#endif
#ifdef CONFIG_AMLOGIC_LCD_TABLET
	case LCD_MODE_TABLET:
		lcd_tablet_vout_server_remove();
		lcd_tablet_remove(dev);
		break;
#endif
	default:
		LCDPR("invalid lcd mode\n");
		break;
	}

	lcd_clk_config_remove();

	return 0;
}

static void lcd_config_probe_delayed(struct work_struct *work)
{
	int key_init_flag = 0;
	int i = 0;
	int ret;

	key_init_flag = key_unify_get_init_flag();
	while (key_init_flag == 0) {
		if (i++ >= LCD_UNIFYKEY_WAIT_TIMEOUT)
			break;
		msleep(LCD_UNIFYKEY_RETRY_INTERVAL);
		key_init_flag = key_unify_get_init_flag();
	}
	LCDPR("key_init_flag=%d, i=%d\n", key_init_flag, i);

	if (key_init_flag == 0) {
		kfree(lcd_driver);
		lcd_driver = NULL;
		LCDERR("key is not ready, probe exit\n");
		return;
	}

	ret = lcd_mode_probe(lcd_driver->dev);
	if (ret) {
		kfree(lcd_driver);
		lcd_driver = NULL;
		LCDERR("probe exit\n");
	}

	if ((lcd_driver->lcd_status & LCD_STATUS_VMODE_ACTIVE)
	&& !(lcd_driver->lcd_status & LCD_STATUS_ENCL_ON)) {
		LCDPR("%s: lcd_enable in kernel\n", __func__);
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_ON, NULL);
		lcd_if_enable_retry(lcd_driver->lcd_config);
	}
}

static void lcd_config_default(void)
{
	struct lcd_config_s *pconf;

	pconf = lcd_driver->lcd_config;
	pconf->lcd_basic.h_active = lcd_vcbus_read(ENCL_VIDEO_HAVON_END)
			- lcd_vcbus_read(ENCL_VIDEO_HAVON_BEGIN) + 1;
	pconf->lcd_basic.v_active = lcd_vcbus_read(ENCL_VIDEO_VAVON_ELINE)
			- lcd_vcbus_read(ENCL_VIDEO_VAVON_BLINE) + 1;
	if (lcd_vcbus_read(ENCL_VIDEO_EN)) {
		if (lcd_boot_ctrl_config.lcd_init_level)
			lcd_driver->lcd_status = LCD_STATUS_ENCL_ON;
		else
			lcd_driver->lcd_status = LCD_STATUS_ON;
		lcd_resume_flag = 1;
	} else {
		lcd_driver->lcd_status = 0;
		lcd_resume_flag = 0;
	}
	LCDPR("status: %d\n", lcd_driver->lcd_status);
}

static int lcd_config_probe(struct platform_device *pdev)
{
	const char *str;
	unsigned int val;
	int ret = 0;

	if (lcd_driver->dev->of_node == NULL) {
		LCDERR("dev of_node is null\n");
		lcd_driver->lcd_mode = LCD_MODE_MAX;
		return -1;
	}

	lcd_driver->res_vsync_irq = NULL;
	lcd_driver->res_vsync2_irq = NULL;
	lcd_driver->res_vx1_irq = NULL;
	lcd_driver->res_tcon_irq = NULL;

	/* lcd driver assign */
	switch (lcd_boot_ctrl_config.debug_lcd_mode) {
	case 1:
		LCDPR("debug_lcd_mode: 1,tv mode\n");
		lcd_driver->lcd_mode = LCD_MODE_TV;
		break;
	case 2:
		LCDPR("debug_lcd_mode: 2,tablet mode\n");
		lcd_driver->lcd_mode = LCD_MODE_TABLET;
		break;
	default:
		ret = of_property_read_string(lcd_driver->dev->of_node,
		"mode", &str);
		if (ret) {
			str = "none";
			LCDERR("failed to get mode\n");
			return -1;
		}
		lcd_driver->lcd_mode = lcd_mode_str_to_mode(str);
		break;
	}

	ret = of_property_read_u32(lcd_driver->dev->of_node,
		"fr_auto_policy", &val);
	if (ret) {
		if (lcd_debug_print_flag)
			LCDPR("failed to get fr_auto_policy\n");
		lcd_driver->fr_auto_policy = 0;
	} else {
		lcd_driver->fr_auto_policy = (unsigned char)val;
	}

	switch (lcd_boot_ctrl_config.debug_para_source) {
	case 1:
		LCDPR("debug_para_source: 1,dts\n");
		lcd_driver->lcd_key_valid = 0;
		break;
	case 2:
		LCDPR("debug_para_source: 2,unifykey\n");
		lcd_driver->lcd_key_valid = 1;
		break;
	default:
		ret = of_property_read_u32(lcd_driver->dev->of_node,
		"key_valid", &val);
		if (ret) {
			if (lcd_debug_print_flag)
				LCDPR("failed to get key_valid\n");
			lcd_driver->lcd_key_valid = 0;
		} else {
			lcd_driver->lcd_key_valid = (unsigned char)val;
		}
		break;
	}

	LCDPR("detect mode: %s, fr_auto_policy: %d, key_valid: %d\n",
		str, lcd_driver->fr_auto_policy, lcd_driver->lcd_key_valid);

	ret = of_property_read_u32(lcd_driver->dev->of_node, "clk_path", &val);
	if (ret) {
		if (lcd_debug_print_flag)
			LCDPR("failed to get clk_path\n");
		lcd_driver->lcd_clk_path = 0;
	} else {
		lcd_driver->lcd_clk_path = (unsigned char)val;
		LCDPR("detect lcd_clk_path: %d\n", lcd_driver->lcd_clk_path);
	}

	ret = of_property_read_u32(lcd_driver->dev->of_node, "auto_test", &val);
	if (ret) {
		if (lcd_debug_print_flag)
			LCDPR("failed to get auto_test\n");
		lcd_driver->lcd_auto_test = 0;
	} else {
		lcd_driver->lcd_auto_test = (unsigned char)val;
		LCDPR("detect lcd_auto_test: %d\n", lcd_driver->lcd_auto_test);
	}

	lcd_driver->res_vsync_irq = platform_get_resource_byname(pdev,
		IORESOURCE_IRQ, "vsync");
	lcd_driver->res_vsync2_irq = platform_get_resource_byname(pdev,
		IORESOURCE_IRQ, "vsync2");
	lcd_driver->res_vx1_irq = platform_get_resource_byname(pdev,
		IORESOURCE_IRQ, "vbyone");
	lcd_driver->res_tcon_irq = platform_get_resource_byname(pdev,
		IORESOURCE_IRQ, "tcon");

	lcd_driver->lcd_info = &lcd_vinfo;
	lcd_driver->lcd_config = &lcd_config_dft;
	lcd_driver->lcd_test_state = lcd_boot_ctrl_config.debug_test_pattern;
	lcd_driver->lcd_test_flag = 0;
	lcd_driver->lcd_mute_state = 0;
	lcd_driver->lcd_mute_flag = 0;
	lcd_driver->lcd_resume_type = 1; /* default workqueue */
	lcd_driver->fr_mode = 0;
	lcd_driver->viu_sel = LCD_VIU_SEL_NONE;
	lcd_driver->vsync_none_timer_flag = 0;
	lcd_driver->power_ctrl = lcd_power_ctrl;
	lcd_driver->module_reset = lcd_module_reset;
	lcd_phy_probe();
	lcd_clk_config_probe();
	lcd_config_default();
	lcd_init_vout();

	if (lcd_driver->lcd_key_valid) {
		if (lcd_driver->workqueue) {
			queue_delayed_work(lcd_driver->workqueue,
				&lcd_driver->lcd_probe_delayed_work,
				msecs_to_jiffies(2000));
		} else {
			schedule_delayed_work(
				&lcd_driver->lcd_probe_delayed_work,
				msecs_to_jiffies(2000));
		}
	} else {
		ret = lcd_mode_probe(lcd_driver->dev);
		if (ret) {
			kfree(lcd_driver);
			lcd_driver = NULL;
			LCDERR("probe exit\n");
		}
	}

	if (lcd_driver->lcd_config->lcd_basic.lcd_type == LCD_TYPE_MAX) {
		switch (lcd_boot_ctrl_config.lcd_type) {
		case LCD_TTL:
			lcd_driver->lcd_config->lcd_basic.lcd_bits =
			lcd_boot_ctrl_config.lcd_bits;
		lcd_driver->lcd_config->lcd_control.ttl_config->sync_valid =
			lcd_boot_ctrl_config.advanced_flag;
			lcd_ttl_pinmux_set(1);
			break;
		case LCD_VBYONE:
			lcd_vbyone_pinmux_set(1);
			break;
		case LCD_MLVDS:
		case LCD_P2P:
			lcd_tcon_pinmux_set(1);
			break;
		default:
			break;
		}
	}
	return 0;
}

#ifdef CONFIG_OF

static struct lcd_data_s lcd_data_gxl = {
	.chip_type = LCD_CHIP_GXL,
	.chip_name = "gxl",
	.reg_map_table = &lcd_reg_gxb[0],
};

static struct lcd_data_s lcd_data_gxm = {
	.chip_type = LCD_CHIP_GXM,
	.chip_name = "gxm",
	.reg_map_table = &lcd_reg_gxb[0],
};

static struct lcd_data_s lcd_data_txl = {
	.chip_type = LCD_CHIP_TXL,
	.chip_name = "txl",
	.reg_map_table = &lcd_reg_gxb[0],
};

static struct lcd_data_s lcd_data_txlx = {
	.chip_type = LCD_CHIP_TXLX,
	.chip_name = "txlx",
	.reg_map_table = &lcd_reg_gxb[0],
};

static struct lcd_data_s lcd_data_axg = {
	.chip_type = LCD_CHIP_AXG,
	.chip_name = "axg",
	.reg_map_table = &lcd_reg_axg[0],
};

static struct lcd_data_s lcd_data_g12a = {
	.chip_type = LCD_CHIP_G12A,
	.chip_name = "g12a",
	.reg_map_table = &lcd_reg_axg[0],
};

static struct lcd_data_s lcd_data_g12b = {
	.chip_type = LCD_CHIP_G12B,
	.chip_name = "g12b",
	.reg_map_table = &lcd_reg_axg[0],
};

static struct lcd_data_s lcd_data_tl1 = {
	.chip_type = LCD_CHIP_TL1,
	.chip_name = "tl1",
	.reg_map_table = &lcd_reg_tl1[0],
};

static struct lcd_data_s lcd_data_sm1 = {
	.chip_type = LCD_CHIP_SM1,
	.chip_name = "sm1",
	.reg_map_table = &lcd_reg_axg[0],
};

static struct lcd_data_s lcd_data_tm2 = {
	.chip_type = LCD_CHIP_TM2,
	.chip_name = "tm2",
	.reg_map_table = &lcd_reg_tl1[0],
};

static const struct of_device_id lcd_dt_match_table[] = {
	{
		.compatible = "amlogic, lcd-gxl",
		.data = &lcd_data_gxl,
	},
	{
		.compatible = "amlogic, lcd-gxm",
		.data = &lcd_data_gxm,
	},
	{
		.compatible = "amlogic, lcd-txl",
		.data = &lcd_data_txl,
	},
	{
		.compatible = "amlogic, lcd-txlx",
		.data = &lcd_data_txlx,
	},
	{
		.compatible = "amlogic, lcd-axg",
		.data = &lcd_data_axg,
	},
	{
		.compatible = "amlogic, lcd-g12a",
		.data = &lcd_data_g12a,
	},
	{
		.compatible = "amlogic, lcd-g12b",
		.data = &lcd_data_g12b,
	},
	{
		.compatible = "amlogic, lcd-tl1",
		.data = &lcd_data_tl1,
	},
	{
		.compatible = "amlogic, lcd-sm1",
		.data = &lcd_data_sm1,
	},
	{
		.compatible = "amlogic, lcd-tm2",
		.data = &lcd_data_tm2,
	},
	{},
};
#endif

static int lcd_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	int ret = 0;

	lcd_debug_print_flag = lcd_boot_ctrl_config.debug_print_flag;

	lcd_driver = kmalloc(sizeof(struct aml_lcd_drv_s), GFP_KERNEL);
	if (!lcd_driver) {
		LCDERR("%s: lcd driver no enough memory\n", __func__);
		return -ENOMEM;
	}
	lcd_driver->dev = &pdev->dev;

	match = of_match_device(lcd_dt_match_table, &pdev->dev);
	if (match == NULL) {
		LCDERR("%s: no match table\n", __func__);
		return -1;
	}
	lcd_driver->data = (struct lcd_data_s *)match->data;
	strcpy(lcd_driver->version, LCD_DRV_VERSION);
	LCDPR("driver version: %s(%d-%s)\n",
		lcd_driver->version,
		lcd_driver->data->chip_type,
		lcd_driver->data->chip_name);

	mutex_init(&lcd_vout_mutex);
	mutex_init(&lcd_driver->power_mutex);
	lcd_vout_serve_bypass = 0;

	/* init workqueue */
	INIT_DELAYED_WORK(&lcd_driver->lcd_probe_delayed_work,
		lcd_config_probe_delayed);
	INIT_DELAYED_WORK(&lcd_test_delayed_work, lcd_auto_test_delayed);
	lcd_driver->workqueue = create_singlethread_workqueue("lcd_work_queue");
	if (lcd_driver->workqueue == NULL)
		LCDERR("can't create lcd workqueue\n");

	INIT_WORK(&(lcd_driver->lcd_resume_work), lcd_resume_work);

	lcd_ioremap(pdev);
	ret = lcd_config_probe(pdev);

	LCDPR("%s %s\n", __func__, (ret ? "failed" : "ok"));

	return 0;
}

static int lcd_remove(struct platform_device *pdev)
{
	if (lcd_driver == NULL)
		return 0;

	cancel_delayed_work(&lcd_driver->lcd_probe_delayed_work);
	cancel_work_sync(&(lcd_driver->lcd_resume_work));
	if (lcd_driver->workqueue)
		destroy_workqueue(lcd_driver->workqueue);

	lcd_vsync_irq_remove();
	lcd_fops_remove();
	lcd_debug_remove();
	lcd_config_remove(lcd_driver->dev);

	kfree(lcd_driver);
	lcd_driver = NULL;

	LCDPR("%s\n", __func__);
	return 0;
}

static int lcd_resume(struct platform_device *pdev)
{
	if (lcd_debug_print_flag)
		LCDPR("resume method: %d\n", get_resume_method());

	if ((get_resume_method() == RTC_WAKEUP) ||
		(get_resume_method() == AUTO_WAKEUP) ||
		(get_resume_method() == UDEFINED_WAKEUP))
		return 0;

	if (lcd_driver == NULL)
		return 0;
	if ((lcd_driver->lcd_status & LCD_STATUS_VMODE_ACTIVE) == 0)
		return 0;

	if (lcd_driver->lcd_resume_type) {
		lcd_resume_flag = 1;
		if (lcd_driver->workqueue) {
			queue_work(lcd_driver->workqueue,
				&(lcd_driver->lcd_resume_work));
		} else {
			schedule_work(&(lcd_driver->lcd_resume_work));
		}
	} else {
		mutex_lock(&lcd_driver->power_mutex);
		LCDPR("directly lcd resume\n");
		lcd_resume_flag = 1;
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_ON, NULL);
		lcd_if_enable_retry(lcd_driver->lcd_config);
		LCDPR("%s finished\n", __func__);
		mutex_unlock(&lcd_driver->power_mutex);
	}

	return 0;
}

static int lcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (lcd_driver == NULL)
		return 0;

	mutex_lock(&lcd_driver->power_mutex);
	if (lcd_driver->lcd_status & LCD_STATUS_ENCL_ON) {
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_OFF, NULL);
		lcd_resume_flag = 0;
		LCDPR("%s finished\n", __func__);
	}
	mutex_unlock(&lcd_driver->power_mutex);
	return 0;
}

static void lcd_shutdown(struct platform_device *pdev)
{
	if (lcd_debug_print_flag)
		LCDPR("%s\n", __func__);

	if (lcd_driver == NULL)
		return;
	if (lcd_driver->lcd_status & LCD_STATUS_ENCL_ON)
		aml_lcd_notifier_call_chain(LCD_EVENT_POWER_OFF, NULL);
}

static struct platform_driver lcd_platform_driver = {
	.probe = lcd_probe,
	.remove = lcd_remove,
	.suspend = lcd_suspend,
	.resume = lcd_resume,
	.shutdown = lcd_shutdown,
	.driver = {
		.name = "mesonlcd",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lcd_dt_match_table),
#endif
	},
};

static int __init lcd_init(void)
{
	if (platform_driver_register(&lcd_platform_driver)) {
		LCDERR("failed to register lcd driver module\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcd_exit(void)
{
	platform_driver_unregister(&lcd_platform_driver);
}

subsys_initcall(lcd_init);
module_exit(lcd_exit);

static int __init lcd_panel_type_para_setup(char *str)
{
	if (str != NULL)
		sprintf(lcd_propname, "%s", str);

	LCDPR("panel_type: %s\n", lcd_propname);
	return 0;
}

static int __init lcd_boot_ctrl_setup(char *str)
{
	//struct lcd_boot_ctrl_s *lcd_boot_ctrl = lcd_config_dft.lcd_boot_ctrl;

	int ret = 0;
	unsigned int lcd_ctrl = 0;

	if (str == NULL)
		return -EINVAL;

	ret = kstrtouint(str, 16, &lcd_ctrl);
	if (ret) {
		LCDERR("%s:invalid data\n", __func__);
		return -EINVAL;
	}

	LCDPR("lcd_ctrl: 0x%08x\n", lcd_ctrl);
	lcd_boot_ctrl_config.lcd_type = 0xf & lcd_ctrl;
	lcd_boot_ctrl_config.lcd_bits = 0xf & (lcd_ctrl >> 4);
	lcd_boot_ctrl_config.advanced_flag = 0xff & (lcd_ctrl >> 8);
	lcd_boot_ctrl_config.lcd_init_level = 0x1 & (lcd_ctrl >> 19);
	lcd_boot_ctrl_config.debug_print_flag = 0xf & (lcd_ctrl >> 20);
	lcd_boot_ctrl_config.debug_test_pattern = 0xf & (lcd_ctrl >> 24);
	lcd_boot_ctrl_config.debug_para_source = 0x3 & (lcd_ctrl >> 28);
	lcd_boot_ctrl_config.debug_lcd_mode = 0x3 & (lcd_ctrl >> 30);
	return 0;
}

__setup("panel_type=", lcd_panel_type_para_setup);
__setup("lcd_ctrl=", lcd_boot_ctrl_setup);

MODULE_DESCRIPTION("Meson LCD Panel Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");
