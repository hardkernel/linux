/*
 * drivers/amlogic/media/vout/vout_serve/dummy_lcd.c
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

/* Linux Headers */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/of.h>
#include <linux/clk.h>
#ifdef CONFIG_AMLOGIC_VPU
#include <linux/amlogic/media/vpu/vpu.h>
#endif
#include <linux/amlogic/media/vout/vout_notify.h>

/* Local Headers */
#include "vout_func.h"
#include "vout_reg.h"

struct dummy_lcd_driver_s {
	struct device *dev;
	unsigned char status;
	unsigned char viu_sel;

	unsigned char clk_gate_state;

	struct clk *encp_top_gate;
	struct clk *encp_int_gate0;
	struct clk *encp_int_gate1;
};

static struct dummy_lcd_driver_s *dummy_lcd_drv;

/* **********************************************************
 * dummy_lcd support
 * **********************************************************
 */
static struct vinfo_s dummy_lcd_vinfo = {
	.name              = "dummy_panel",
	.mode              = VMODE_DUMMY_LCD,
	.width             = 1920,
	.height            = 1080,
	.field_height      = 1080,
	.aspect_ratio_num  = 16,
	.aspect_ratio_den  = 9,
	.sync_duration_num = 60,
	.sync_duration_den = 1,
	.video_clk         = 148500000,
	.htotal            = 2200,
	.vtotal            = 1125,
	.fr_adj_type       = VOUT_FR_ADJ_NONE,
	.viu_color_fmt     = COLOR_FMT_RGB444,
	.viu_mux           = VIU_MUX_ENCP,
	.vout_device       = NULL,
};

static void dummy_lcd_venc_set(void)
{
	unsigned int temp;

	VOUTPR("%s\n", __func__);

	vout_vcbus_write(ENCP_VIDEO_EN, 0);

	vout_vcbus_write(ENCP_VIDEO_MODE, 0x8000);
	vout_vcbus_write(ENCP_VIDEO_MODE_ADV, 0x0418);

	temp = vout_vcbus_read(ENCL_VIDEO_MAX_PXCNT);
	vout_vcbus_write(ENCP_VIDEO_MAX_PXCNT, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_MAX_LNCNT);
	vout_vcbus_write(ENCP_VIDEO_MAX_LNCNT, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_HAVON_BEGIN);
	vout_vcbus_write(ENCP_VIDEO_HAVON_BEGIN, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_HAVON_END);
	vout_vcbus_write(ENCP_VIDEO_HAVON_END, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_VAVON_BLINE);
	vout_vcbus_write(ENCP_VIDEO_VAVON_BLINE, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_VAVON_ELINE);
	vout_vcbus_write(ENCP_VIDEO_VAVON_ELINE, temp);

	temp = vout_vcbus_read(ENCL_VIDEO_HSO_BEGIN);
	vout_vcbus_write(ENCP_VIDEO_HSO_BEGIN, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_HSO_END);
	vout_vcbus_write(ENCP_VIDEO_HSO_END,   temp);
	temp = vout_vcbus_read(ENCL_VIDEO_VSO_BEGIN);
	vout_vcbus_write(ENCP_VIDEO_VSO_BEGIN, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_VSO_END);
	vout_vcbus_write(ENCP_VIDEO_VSO_END,   temp);
	temp = vout_vcbus_read(ENCL_VIDEO_VSO_BLINE);
	vout_vcbus_write(ENCP_VIDEO_VSO_BLINE, temp);
	temp = vout_vcbus_read(ENCL_VIDEO_VSO_ELINE);
	vout_vcbus_write(ENCP_VIDEO_VSO_ELINE, temp);

	temp = vout_vcbus_read(ENCL_VIDEO_VSO_ELINE);
	vout_vcbus_write(ENCP_VIDEO_RGBIN_CTRL, temp);

	vout_vcbus_write(ENCP_VIDEO_EN, 1);
}

static void dummy_lcd_clk_ctrl(int flag)
{
	unsigned int temp;

	if (flag) {
		temp = vout_hiu_getb(HHI_VIID_CLK_DIV, ENCL_CLK_SEL, 4);
		vout_hiu_setb(HHI_VID_CLK_DIV, temp, ENCP_CLK_SEL, 4);

		vout_hiu_setb(HHI_VID_CLK_CNTL2, 1, ENCP_GATE_VCLK, 1);
	} else {
		vout_hiu_setb(HHI_VID_CLK_CNTL2, 0, ENCP_GATE_VCLK, 1);
	}
}

static void dummy_lcd_clk_gate_switch(int flag)
{
	if (flag) {
		if (dummy_lcd_drv->clk_gate_state)
			return;
		if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_top_gate))
			VOUTERR("%s: encp_top_gate\n", __func__);
		else
			clk_prepare_enable(dummy_lcd_drv->encp_top_gate);
		if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate0))
			VOUTERR("%s: encp_int_gate0\n", __func__);
		else
			clk_prepare_enable(dummy_lcd_drv->encp_int_gate0);
		if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate1))
			VOUTERR("%s: encp_int_gate1\n", __func__);
		else
			clk_prepare_enable(dummy_lcd_drv->encp_int_gate1);
		dummy_lcd_drv->clk_gate_state = 1;
	} else {
		if (dummy_lcd_drv->clk_gate_state == 0)
			return;
		dummy_lcd_drv->clk_gate_state = 0;
		if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate0))
			VOUTERR("%s: encp_int_gate0\n", __func__);
		else
			clk_disable_unprepare(dummy_lcd_drv->encp_int_gate0);
		if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate1))
			VOUTERR("%s: encp_int_gate1\n", __func__);
		else
			clk_disable_unprepare(dummy_lcd_drv->encp_int_gate1);
		if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_top_gate))
			VOUTERR("%s: encp_top_gate\n", __func__);
		else
			clk_disable_unprepare(dummy_lcd_drv->encp_top_gate);
	}
}

static void dummy_lcd_vinfo_update(void)
{
	unsigned int lcd_viu_sel = 0;
	const struct vinfo_s *vinfo = NULL;

	if (dummy_lcd_drv->viu_sel == 1) {
		vinfo = get_current_vinfo2();
		lcd_viu_sel = 2;
	} else if (dummy_lcd_drv->viu_sel == 2) {
		vinfo = get_current_vinfo();
		lcd_viu_sel = 1;
	}

	if (vinfo) {
		if (vinfo->mode != VMODE_LCD) {
			VOUTERR("display%d is not panel\n", lcd_viu_sel);
			vinfo = NULL;
		}
	}
	if (!vinfo)
		return;

	dummy_lcd_vinfo.width = vinfo->width;
	dummy_lcd_vinfo.height = vinfo->height;
	dummy_lcd_vinfo.field_height = vinfo->field_height;
	dummy_lcd_vinfo.aspect_ratio_num = vinfo->aspect_ratio_num;
	dummy_lcd_vinfo.aspect_ratio_den = vinfo->aspect_ratio_den;
	dummy_lcd_vinfo.sync_duration_num = vinfo->sync_duration_num;
	dummy_lcd_vinfo.sync_duration_den = vinfo->sync_duration_den;
	dummy_lcd_vinfo.video_clk = vinfo->video_clk;
	dummy_lcd_vinfo.htotal = vinfo->htotal;
	dummy_lcd_vinfo.vtotal = vinfo->vtotal;
	dummy_lcd_vinfo.viu_color_fmt = vinfo->viu_color_fmt;
}

static struct vinfo_s *dummy_lcd_get_current_info(void)
{
	return &dummy_lcd_vinfo;
}

static int dummy_lcd_set_current_vmode(enum vmode_e mode)
{
	dummy_lcd_vinfo_update();

#ifdef CONFIG_AMLOGIC_VPU
	request_vpu_clk_vmod(dummy_lcd_vinfo.video_clk, VPU_VENCP);
	switch_vpu_mem_pd_vmod(VPU_VENCP, VPU_MEM_POWER_ON);
#endif
	dummy_lcd_clk_gate_switch(1);

	dummy_lcd_clk_ctrl(1);
	dummy_lcd_venc_set();

	dummy_lcd_drv->status = 1;
	VOUTPR("%s finished\n", __func__);

	return 0;
}

static enum vmode_e dummy_lcd_validate_vmode(char *name)
{
	enum vmode_e vmode = VMODE_MAX;

	if (strcmp(dummy_lcd_vinfo.name, name) == 0)
		vmode = dummy_lcd_vinfo.mode;

	return vmode;
}

static int dummy_lcd_vmode_is_supported(enum vmode_e mode)
{
	if (dummy_lcd_vinfo.mode == (mode & VMODE_MODE_BIT_MASK))
		return true;

	return false;
}

static int dummy_lcd_disable(enum vmode_e cur_vmod)
{
	dummy_lcd_drv->status = 0;

	vout_vcbus_write(ENCP_VIDEO_EN, 0); /* disable encp */
	dummy_lcd_clk_ctrl(0);
	dummy_lcd_clk_gate_switch(0);
#ifdef CONFIG_AMLOGIC_VPU
	switch_vpu_mem_pd_vmod(VPU_VENCP, VPU_MEM_POWER_DOWN);
	release_vpu_clk_vmod(VPU_VENCP);
#endif

	VOUTPR("%s finished\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int dummy_lcd_lcd_suspend(void)
{
	dummy_lcd_disable(VMODE_DUMMY_LCD);
	return 0;
}

static int dummy_lcd_lcd_resume(void)
{
	dummy_lcd_set_current_vmode(VMODE_DUMMY_LCD);
	return 0;
}

#endif

static int dummy_lcd_vout_state;
static int dummy_lcd_vout_set_state(int bit)
{
	dummy_lcd_vout_state |= (1 << bit);
	dummy_lcd_drv->viu_sel = bit;
	return 0;
}

static int dummy_lcd_vout_clr_state(int bit)
{
	dummy_lcd_vout_state &= ~(1 << bit);
	if (dummy_lcd_drv->viu_sel == bit)
		dummy_lcd_drv->viu_sel = 0;
	return 0;
}

static int dummy_lcd_vout_get_state(void)
{
	return dummy_lcd_vout_state;
}

static struct vout_server_s dummy_lcd_vout_server = {
	.name = "dummy_lcd_vout_server",
	.op = {
		.get_vinfo          = dummy_lcd_get_current_info,
		.set_vmode          = dummy_lcd_set_current_vmode,
		.validate_vmode     = dummy_lcd_validate_vmode,
		.vmode_is_supported = dummy_lcd_vmode_is_supported,
		.disable            = dummy_lcd_disable,
		.set_state          = dummy_lcd_vout_set_state,
		.clr_state          = dummy_lcd_vout_clr_state,
		.get_state          = dummy_lcd_vout_get_state,
		.set_bist           = NULL,
#ifdef CONFIG_PM
		.vout_suspend       = dummy_lcd_lcd_suspend,
		.vout_resume        = dummy_lcd_lcd_resume,
#endif
	},
};

#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
static struct vout_server_s dummy_lcd_vout2_server = {
	.name = "dummy_lcd_vout2_server",
	.op = {
		.get_vinfo          = dummy_lcd_get_current_info,
		.set_vmode          = dummy_lcd_set_current_vmode,
		.validate_vmode     = dummy_lcd_validate_vmode,
		.vmode_is_supported = dummy_lcd_vmode_is_supported,
		.disable            = dummy_lcd_disable,
		.set_state          = dummy_lcd_vout_set_state,
		.clr_state          = dummy_lcd_vout_clr_state,
		.get_state          = dummy_lcd_vout_get_state,
		.set_bist           = NULL,
#ifdef CONFIG_PM
		.vout_suspend       = dummy_lcd_lcd_suspend,
		.vout_resume        = dummy_lcd_lcd_resume,
#endif
	},
};
#endif

static void dummy_lcd_vout_server_init(void)
{
	vout_register_server(&dummy_lcd_vout_server);
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vout2_register_server(&dummy_lcd_vout2_server);
#endif
}

static void dummy_lcd_vout_server_remove(void)
{
	vout_unregister_server(&dummy_lcd_vout_server);
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vout2_unregister_server(&dummy_lcd_vout2_server);
#endif
}

/* ********************************************************* */
static int dummy_lcd_vout_mode_update(void)
{
	enum vmode_e mode = VMODE_DUMMY_LCD;

	VOUTPR("%s\n", __func__);

	if (dummy_lcd_drv->viu_sel == 1)
		vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE_PRE, &mode);
	else if (dummy_lcd_drv->viu_sel == 2)
		vout2_notifier_call_chain(VOUT_EVENT_MODE_CHANGE_PRE, &mode);
	dummy_lcd_set_current_vmode(mode);
	if (dummy_lcd_drv->viu_sel == 1)
		vout_notifier_call_chain(VOUT_EVENT_MODE_CHANGE, &mode);
	else if (dummy_lcd_drv->viu_sel == 2)
		vout2_notifier_call_chain(VOUT_EVENT_MODE_CHANGE, &mode);

	return 0;
}

static int dummy_lcd_vout_notify_callback(struct notifier_block *block,
		unsigned long cmd, void *para)
{
	const struct vinfo_s *vinfo;

	switch (cmd) {
	case VOUT_EVENT_MODE_CHANGE:
		vinfo = get_current_vinfo();
		if (!vinfo)
			break;
		if (vinfo->mode != VMODE_LCD)
			break;
		dummy_lcd_vout_mode_update();
		break;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
static int dummy_lcd_vout2_notify_callback(struct notifier_block *block,
		unsigned long cmd, void *para)
{
	const struct vinfo_s *vinfo;

	switch (cmd) {
	case VOUT_EVENT_MODE_CHANGE:
		vinfo = get_current_vinfo2();
		if (!vinfo)
			break;
		if (vinfo->mode != VMODE_LCD)
			break;
		dummy_lcd_vout_mode_update();
		break;
	default:
		break;
	}
	return 0;
}
#endif

static struct notifier_block dummy_lcd_vout_notifier = {
	.notifier_call = dummy_lcd_vout_notify_callback,
};

#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
static struct notifier_block dummy_lcd_vout2_notifier = {
	.notifier_call = dummy_lcd_vout2_notify_callback,
};
#endif

/* ********************************************************* */
static const char *dummy_lcd_debug_usage_str = {
"Usage:\n"
"    echo test <index> > /sys/class/dummy_lcd/debug ; test pattern for encp\n"
"    echo reg > /sys/class/dummy_lcd/debug ; dump regs for encp\n"
};

static ssize_t dummy_lcd_debug_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", dummy_lcd_debug_usage_str);
}

static unsigned int dummy_lcd_reg_dump_encp[] = {
	VPU_VIU_VENC_MUX_CTRL,
	ENCP_VIDEO_EN,
	ENCP_VIDEO_MODE,
	ENCP_VIDEO_MODE_ADV,
	ENCP_VIDEO_MAX_PXCNT,
	ENCP_VIDEO_MAX_LNCNT,
	ENCP_VIDEO_HAVON_BEGIN,
	ENCP_VIDEO_HAVON_END,
	ENCP_VIDEO_VAVON_BLINE,
	ENCP_VIDEO_VAVON_ELINE,
	ENCP_VIDEO_HSO_BEGIN,
	ENCP_VIDEO_HSO_END,
	ENCP_VIDEO_VSO_BEGIN,
	ENCP_VIDEO_VSO_END,
	ENCP_VIDEO_VSO_BLINE,
	ENCP_VIDEO_VSO_ELINE,
	ENCP_VIDEO_RGBIN_CTRL,
	0xffff,
};

static ssize_t dummy_lcd_debug_store(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int ret;
	unsigned int val, i;

	switch (buf[0]) {
	case 't':
		ret = sscanf(buf, "test %d", &val);
		if (ret == 1) {
			pr_info("todo bist pattern\n");
		} else {
			pr_info("invalid data\n");
			return -EINVAL;
		}
		break;
	case 'r':
		pr_info("dummy_lcd register dump:\n");
		i = 0;
		while (i < ARRAY_SIZE(dummy_lcd_reg_dump_encp)) {
			if (dummy_lcd_reg_dump_encp[i] == 0xffff)
				break;
			pr_info("vcbus   [0x%04x] = 0x%08x\n",
				dummy_lcd_reg_dump_encp[i],
				vout_vcbus_read(dummy_lcd_reg_dump_encp[i]));
			i++;
		}
		break;
	default:
		pr_info("invalid data\n");
		break;
	}

	return count;
}

static struct class_attribute dummy_lcd_class_attrs[] = {
	__ATTR(debug, 0644,
		dummy_lcd_debug_show, dummy_lcd_debug_store),
};

static struct class *debug_class;
static int dummy_lcd_creat_class(void)
{
	int i;

	debug_class = class_create(THIS_MODULE, "dummy_lcd");
	if (IS_ERR(debug_class)) {
		VOUTERR("create debug class failed\n");
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(dummy_lcd_class_attrs); i++) {
		if (class_create_file(debug_class, &dummy_lcd_class_attrs[i])) {
			VOUTERR("create debug attribute %s failed\n",
				dummy_lcd_class_attrs[i].attr.name);
		}
	}

	return 0;
}

static int dummy_lcd_remove_class(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dummy_lcd_class_attrs); i++)
		class_remove_file(debug_class, &dummy_lcd_class_attrs[i]);

	class_destroy(debug_class);
	debug_class = NULL;

	return 0;
}
/* ********************************************************* */

static void dummy_lcd_clktree_probe(void)
{
	dummy_lcd_drv->clk_gate_state = 0;

	dummy_lcd_drv->encp_top_gate = devm_clk_get(dummy_lcd_drv->dev,
			"encp_top_gate");
	if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_top_gate))
		VOUTERR("%s: get encp_top_gate error\n", __func__);

	dummy_lcd_drv->encp_int_gate0 = devm_clk_get(dummy_lcd_drv->dev,
			"encp_int_gate0");
	if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate0))
		VOUTERR("%s: get encp_int_gate0 error\n", __func__);

	dummy_lcd_drv->encp_int_gate1 = devm_clk_get(dummy_lcd_drv->dev,
			"encp_int_gate1");
	if (IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate1))
		VOUTERR("%s: get encp_int_gate1 error\n", __func__);
}

static void dummy_lcd_clktree_remove(void)
{
	if (!IS_ERR_OR_NULL(dummy_lcd_drv->encp_top_gate))
		devm_clk_put(dummy_lcd_drv->dev, dummy_lcd_drv->encp_top_gate);
	if (!IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate0))
		devm_clk_put(dummy_lcd_drv->dev, dummy_lcd_drv->encp_int_gate0);
	if (!IS_ERR_OR_NULL(dummy_lcd_drv->encp_int_gate1))
		devm_clk_put(dummy_lcd_drv->dev, dummy_lcd_drv->encp_int_gate1);
}

#ifdef CONFIG_OF
static const struct of_device_id dummy_lcd_dt_match_table[] = {
	{
		.compatible = "amlogic, dummy_lcd",
	},
	{},
};
#endif

static int dummy_lcd_probe(struct platform_device *pdev)
{
	dummy_lcd_drv = kzalloc(sizeof(struct dummy_lcd_driver_s), GFP_KERNEL);
	if (!dummy_lcd_drv) {
		VOUTERR("%s: dummy_lcd driver no enough memory\n", __func__);
		return -ENOMEM;
	}
	dummy_lcd_drv->dev = &pdev->dev;

	dummy_lcd_clktree_probe();
	dummy_lcd_vout_server_init();

	vout_register_client(&dummy_lcd_vout_notifier);
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vout2_register_client(&dummy_lcd_vout2_notifier);
#endif
	dummy_lcd_creat_class();

	VOUTPR("%s OK\n", __func__);

	return 0;
}

static int dummy_lcd_remove(struct platform_device *pdev)
{
	if (dummy_lcd_drv == NULL)
		return 0;

	dummy_lcd_remove_class();
	vout_unregister_client(&dummy_lcd_vout_notifier);
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vout2_unregister_client(&dummy_lcd_vout2_notifier);
#endif
	dummy_lcd_vout_server_remove();
	dummy_lcd_clktree_remove();

	kfree(dummy_lcd_drv);
	dummy_lcd_drv = NULL;

	VOUTPR("%s\n", __func__);
	return 0;
}

static void dummy_lcd_shutdown(struct platform_device *pdev)
{
	if (dummy_lcd_drv == NULL)
		return;
	if (dummy_lcd_drv->status)
		dummy_lcd_disable(VMODE_DUMMY_LCD);
}

static struct platform_driver dummy_lcd_platform_driver = {
	.probe = dummy_lcd_probe,
	.remove = dummy_lcd_remove,
	.shutdown = dummy_lcd_shutdown,
	.driver = {
		.name = "dummy_lcd",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = dummy_lcd_dt_match_table,
#endif
	},
};

static int __init dummy_lcd_init(void)
{
	if (platform_driver_register(&dummy_lcd_platform_driver)) {
		VOUTERR("failed to register dummy_lcd driver module\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit dummy_lcd_exit(void)
{
	platform_driver_unregister(&dummy_lcd_platform_driver);
}

subsys_initcall(dummy_lcd_init);
module_exit(dummy_lcd_exit);

