/*
 * drivers/amlogic/media/enhancement/amvecm/amcsc.c
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

/* Standard Linux headers */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/amlogic/media/vfm/vframe.h>
#include <linux/amlogic/media/vfm/video_common.h>
#include <linux/amlogic/media/amvecm/amvecm.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include "arch/vpp_regs.h"
#include "../../vin/tvin/tvin_global.h"
#include "arch/vpp_hdr_regs.h"
#include "arch/hdr_curve.h"

/* use osd rdma reg w/r */
#include "../../osd/osd_rdma.h"

#include "amcsc.h"
#include "amcsc_pip.h"
#include "set_hdr2_v0.h"
#include <linux/amlogic/media/amdolbyvision/dolby_vision.h>
#include "hdr/am_hdr10_plus.h"
#include "hdr/am_hdr10_plus_ootf.h"


uint debug_csc;
static int cur_mvc_type[2];
module_param(debug_csc, uint, 0664);
MODULE_PARM_DESC(debug_csc, "\n debug_csc\n");

signed int vd1_contrast_offset;

signed int saturation_offset;

/*hdr------------------------------------*/

void hdr_osd_off(void)
{
	enum hdr_process_sel cur_hdr_process;

	cur_hdr_process = hdr_func(OSD1_HDR, HDR_BYPASS, get_current_vinfo());
	pr_csc(8, "am_vecm: module=OSD1_HDR, process=HDR_BYPASS(%d, %d)\n",
		HDR_BYPASS, cur_hdr_process);
}

void hdr_vd1_off(void)
{
	enum hdr_process_sel cur_hdr_process;

	cur_hdr_process = hdr_func(VD1_HDR, HDR_BYPASS, get_current_vinfo());
	pr_csc(8, "am_vecm: module=VD1_HDR, process=HDR_BYPASS(%d, %d)\n",
		HDR_BYPASS, cur_hdr_process);
}

void hdr_vd2_off(void)
{
	enum hdr_process_sel cur_hdr_process;

	cur_hdr_process = hdr_func(VD2_HDR, HDR_BYPASS, get_current_vinfo());
	pr_csc(8, "am_vecm: module=VD2_HDR, process=HDR_BYPASS(%d, %d)\n",
	       HDR_BYPASS, cur_hdr_process);
}

void hdr_vd1_iptmap(void)
{
	enum hdr_process_sel cur_hdr_process;

	cur_hdr_process = hdr_func(
		VD1_HDR, IPT_MAP, get_current_vinfo());
	pr_csc(8, "am_vecm: module=VD1_HDR, process=IPT_MAP(%d, %d)\n",
		IPT_MAP, cur_hdr_process);
}

static struct hdr_data_t *phdr;

struct hdr_data_t *hdr_get_data(void)
{
	return phdr;
}

int is_hdr_cfg_osd_100(void)
{
	int ret = 0;

	if (phdr) {
		if (phdr->hdr_cfg.en_osd_lut_100)
			ret = phdr->hdr_cfg.en_osd_lut_100;
	}

	return ret;
}
void hdr_set_cfg_osd_100(int val)
{
	if (val > 0)
		phdr->hdr_cfg.en_osd_lut_100 = val;
	else
		phdr->hdr_cfg.en_osd_lut_100 = 0;
}
static ssize_t read_file_hdr_cfgosd(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	char buf[20];
	ssize_t len;

	len = snprintf(buf, 20, "%d\n", phdr->hdr_cfg.en_osd_lut_100);
	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t write_file_hdr_cfgosd(
	struct file *file, const char __user *userbuf,
	size_t count, loff_t *ppos)
{
	int val;
	char buf[20];
	int ret = 0;
	int cont2;

	cont2 = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, cont2))
		return -EFAULT;
	buf[cont2] = 0;
	ret = kstrtoint(buf, 0, &val);
	if (ret != 0) {
		pr_info("cfg_en_osd_100 do nothing!\n");
		return -EINVAL;
	}

	hdr_set_cfg_osd_100(val);

	pr_info("HDR: en_osd_lut_100: %d\n", phdr->hdr_cfg.en_osd_lut_100);

	return count;
}

static const struct file_operations file_ops_hdr_cfgosd = {
	.open		= simple_open,
	.read		= read_file_hdr_cfgosd,
	.write		= write_file_hdr_cfgosd,
};

struct hdr_debugfs_files_t {
	const char *name;
	const umode_t mode;
	const struct file_operations *fops;
};

static struct hdr_debugfs_files_t hdr_debugfs_files[] = {
	{"cfg_en_osd_100", S_IFREG | 0644, &file_ops_hdr_cfgosd},

};


static void hdr_debugfs_init(void)
{
	int i;
	struct dentry *ent;

	if (phdr == NULL)
		return;

	if (phdr->dbg_root)
		return;

	phdr->dbg_root = debugfs_create_dir("hdr", NULL);
	if (!phdr->dbg_root) {
		pr_err("can't create debugfs dir hdr\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(hdr_debugfs_files); i++) {
		ent = debugfs_create_file(hdr_debugfs_files[i].name,
			hdr_debugfs_files[i].mode,
			phdr->dbg_root, NULL,
			hdr_debugfs_files[i].fops);
		if (!ent)
			pr_err("debugfs create failed\n");
	}

}
static void hdr_debugfs_exit(void)
{
	if (phdr && phdr->dbg_root)
		debugfs_remove(phdr->dbg_root);
}

void hdr_init(struct hdr_data_t *phdr_data)
{
	if (phdr_data) {
		phdr = phdr_data;
	} else {
		phdr = NULL;
		pr_err("%s failed\n", __func__);
		return;
	}

	hdr_debugfs_init();
}
void hdr_exit(void)
{
	hdr_debugfs_exit();
}

/*-----------------------------------------*/
static void vpp_set_mtx_en_read(void);
static void vpp_set_mtx_en_write(void);

struct hdr_osd_reg_s hdr_osd_reg = {
	0x00000001, /* VIU_OSD1_MATRIX_CTRL 0x1a90 */
	0x00ba0273, /* VIU_OSD1_MATRIX_COEF00_01 0x1a91 */
	0x003f1f9a, /* VIU_OSD1_MATRIX_COEF02_10 0x1a92 */
	0x1ea801c0, /* VIU_OSD1_MATRIX_COEF11_12 0x1a93 */
	0x01c01e6a, /* VIU_OSD1_MATRIX_COEF20_21 0x1a94 */
	0x00000000, /* VIU_OSD1_MATRIX_COLMOD_COEF42 0x1a95 */
	0x00400200, /* VIU_OSD1_MATRIX_OFFSET0_1 0x1a96 */
	0x00000200, /* VIU_OSD1_MATRIX_PRE_OFFSET2 0x1a97 */
	0x00000000, /* VIU_OSD1_MATRIX_PRE_OFFSET0_1 0x1a98 */
	0x00000000, /* VIU_OSD1_MATRIX_PRE_OFFSET2 0x1a99 */
	0x1fd80000, /* VIU_OSD1_MATRIX_COEF22_30 0x1a9d */
	0x00000000, /* VIU_OSD1_MATRIX_COEF31_32 0x1a9e */
	0x00000000, /* VIU_OSD1_MATRIX_COEF40_41 0x1a9f */
	0x00000000, /* VIU_OSD1_EOTF_CTL 0x1ad4 */
	0x08000000, /* VIU_OSD1_EOTF_COEF00_01 0x1ad5 */
	0x00000000, /* VIU_OSD1_EOTF_COEF02_10 0x1ad6 */
	0x08000000, /* VIU_OSD1_EOTF_COEF11_12 0x1ad7 */
	0x00000000, /* VIU_OSD1_EOTF_COEF20_21 0x1ad8 */
	0x08000001, /* VIU_OSD1_EOTF_COEF22_RS 0x1ad9 */
	0x0,        /* VIU_OSD1_EOTF_3X3_OFST_0 0x1aa0 */
	0x0,        /* VIU_OSD1_EOTF_3X3_OFST_1 0x1aa1 */
	0x01c00000, /* VIU_OSD1_OETF_CTL 0x1adc */
	{
		/* eotf table */
		{ /* r map */
			0x0000, 0x0200, 0x0400, 0x0600, 0x0800, 0x0a00,
			0x0c00, 0x0e00, 0x1000, 0x1200, 0x1400, 0x1600,
			0x1800, 0x1a00, 0x1c00, 0x1e00, 0x2000, 0x2200,
			0x2400, 0x2600, 0x2800, 0x2a00, 0x2c00, 0x2e00,
			0x3000, 0x3200, 0x3400, 0x3600, 0x3800, 0x3a00,
			0x3c00, 0x3e00, 0x4000
		},
		{ /* g map */
			0x0000, 0x0200, 0x0400, 0x0600, 0x0800, 0x0a00,
			0x0c00, 0x0e00, 0x1000, 0x1200, 0x1400, 0x1600,
			0x1800, 0x1a00, 0x1c00, 0x1e00, 0x2000, 0x2200,
			0x2400, 0x2600, 0x2800, 0x2a00, 0x2c00, 0x2e00,
			0x3000, 0x3200, 0x3400, 0x3600, 0x3800, 0x3a00,
			0x3c00, 0x3e00, 0x4000
		},
		{ /* b map */
			0x0000, 0x0200, 0x0400, 0x0600, 0x0800, 0x0a00,
			0x0c00, 0x0e00, 0x1000, 0x1200, 0x1400, 0x1600,
			0x1800, 0x1a00, 0x1c00, 0x1e00, 0x2000, 0x2200,
			0x2400, 0x2600, 0x2800, 0x2a00, 0x2c00, 0x2e00,
			0x3000, 0x3200, 0x3400, 0x3600, 0x3800, 0x3a00,
			0x3c00, 0x3e00, 0x4000
		},
		/* oetf table */
		{ /* or map */
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000
		},
		{ /* og map */
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000
		},
		{ /* ob map */
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
			0x0000, 0x0000, 0x0000, 0x0000, 0x0000
		}
	},
	-1 /* shadow mode */
};

#define HDR_VERSION   "----gxl_20180830---g12a_20191015-----\n"

static struct vframe_s *dbg_vf;
static struct master_display_info_s dbg_hdr_send;
static struct hdr_info receiver_hdr_info;

static bool print_lut_mtx;
module_param(print_lut_mtx, bool, 0664);
MODULE_PARM_DESC(print_lut_mtx, "\n print_lut_mtx\n");

/* bit 0: enable csc */
/* bit 1: enable osd csc */
/* bit 2: enable video csc */
/* bit 4: csc delay one frame */
static uint csc_en = 0x7;
module_param(csc_en, uint, 0664);
MODULE_PARM_DESC(csc_en, "\n csc_en\n");

/* white balance adjust */
static bool cur_eye_protect_mode;

static int num_wb_val = 10;
static int wb_val[10] = {
	0, /* wb enable */
	0, /* -1024~1023, r_pre_offset */
	0, /* -1024~1023, g_pre_offset */
	0, /* -1024~1023, b_pre_offset */
	1024, /* 0~2047, r_gain */
	1024, /* 0~2047, g_gain */
	1024, /* 0~2047, b_gain */
	0, /* -1024~1023, r_post_offset */
	0, /* -1024~1023, g_post_offset */
	0  /* -1024~1023, b_post_offset */
};
module_param_array(wb_val, int, &num_wb_val, 0664);
MODULE_PARM_DESC(wb_val, "\n white balance setting\n");

static enum vframe_source_type_e pre_src_type[VD_PATH_MAX] = {
	VFRAME_SOURCE_TYPE_COMP,
	VFRAME_SOURCE_TYPE_COMP
};

static unsigned int vd_path_max = VD_PATH_MAX;

uint cur_csc_type[VD_PATH_MAX] = {0xffff, 0xffff};
module_param_array(cur_csc_type, uint, &vd_path_max, 0444);
MODULE_PARM_DESC(cur_csc_type, "\n current color space convert type\n");

static uint hdmi_csc_type = 0xffff;
module_param(hdmi_csc_type, uint, 0444);
MODULE_PARM_DESC(hdmi_csc_type, "\n current color space convert type\n");

/* 0: follow sink, 1: follow source, 2: debug, 0xff: bootup default value */
/* by default follow source to match default sdr_mode*/
static uint hdr_policy;
static uint cur_hdr_policy = 0xff;
module_param(hdr_policy, uint, 0664);
MODULE_PARM_DESC(hdr_policy, "\n current hdr_policy\n");

static int __init boot_hdr_policy(char *str)
{
	if (strncmp("1", str, 1) == 0) {
		hdr_policy = 1; //follow source
		pr_info("boot hdr_policy: 1\n");
	} else if (strncmp("0", str, 1) == 0) {
		hdr_policy = 0; //follow sink
		pr_info("boot hdr_policy: 0\n");
	}
	return 0;
}

__setup("hdr_policy=", boot_hdr_policy);

/* when get_hdr_policy() == 0 */
/* enum output_format_e */
/*	BT709 = 1,				*/
/*	BT2020 = 2,				*/
/*	BT2020_PQ = 3,			*/
/*	BT2020_PQ_DYNAMIC = 4,	*/
/*	BT2020_HLG = 5,			*/
/*	BT2100_IPT = 6			*/
/* BT2020 = 2: 2020 + gamma not support in hdmi now */
static uint force_output; /* 0: no force */
module_param(force_output, uint, 0664);
MODULE_PARM_DESC(force_output, "\n current force_output\n");

int get_hdr_policy(void)
{
	int dv_policy = 0;
	int dv_mode = 0;

	if (is_dolby_vision_enable()) {
		/* sync hdr_policy with dolby_vision_policy */
		/* get current dolby_vision_mode */
		dv_policy = get_dolby_vision_policy();
		dv_mode = get_dolby_vision_target_mode();
		if ((dv_policy != DOLBY_VISION_FORCE_OUTPUT_MODE) ||
		    (dv_mode != DOLBY_VISION_OUTPUT_MODE_BYPASS)) {
			/* use dv policy when not force bypass */
			return dv_policy;
		}
	}
	return hdr_policy;
}
EXPORT_SYMBOL(get_hdr_policy);

void set_cur_hdr_policy(uint policy)
{
	cur_hdr_policy = policy;
}
EXPORT_SYMBOL(set_cur_hdr_policy);

enum output_format_e get_force_output(void)
{
	return force_output;
}
EXPORT_SYMBOL(get_force_output);

static uint hdr_mode = 2; /* 0: hdr->hdr, 1:hdr->sdr, 2:auto */
module_param(hdr_mode, uint, 0664);
MODULE_PARM_DESC(hdr_mode, "\n set hdr_mode\n");

/* 0: hdr->hdr, 1:hdr->sdr, 2:hdr->hlg */
uint hdr_process_mode[VD_PATH_MAX];
uint cur_hdr_process_mode[VD_PATH_MAX] = {PROC_OFF, PROC_OFF};
module_param_array(hdr_process_mode, uint, &vd_path_max, 0444);
MODULE_PARM_DESC(hdr_process_mode, "\n current hdr_process_mode\n");

/* 0:bypass, 1:hdr10p->hdr, 2:hdr10p->sdr, 3:hdr10p->hlg */
uint hdr10_plus_process_mode[VD_PATH_MAX];
uint cur_hdr10_plus_process_mode[VD_PATH_MAX] = {PROC_OFF, PROC_OFF};
module_param_array(hdr10_plus_process_mode, uint, &vd_path_max, 0444);
MODULE_PARM_DESC(hdr10_plus_process_mode, "\n current hdr10_plus_process_mode\n");

/* 0: tx don't support hdr10+, 1: tx support hdr10+*/
uint tx_hdr10_plus_support;

/* 0: hlg->hlg, 1:hlg->sdr 2:hlg->hdr*/
uint hlg_process_mode[VD_PATH_MAX];
uint cur_hlg_process_mode[VD_PATH_MAX] = {PROC_OFF, PROC_OFF};
module_param_array(hlg_process_mode, uint, &vd_path_max, 0444);
MODULE_PARM_DESC(hlg_process_mode, "\n current hlg_process_mode\n");

static uint force_pure_hlg[VD_PATH_MAX];
module_param_array(force_pure_hlg, uint, &vd_path_max, 0664);
MODULE_PARM_DESC(force_pure_hlg, "\n current force_pure_hlg\n");

uint sdr_mode; /* 0: sdr->sdr, 1:sdr->hdr, 2:auto */
module_param(sdr_mode, uint, 0664);
MODULE_PARM_DESC(sdr_mode, "\n set sdr_mode\n");

static uint cur_sdr_mode;

/* 0: sdr->sdr, 1:sdr->hdr, 2:sdr->hlg */
uint sdr_process_mode[VD_PATH_MAX];
uint cur_sdr_process_mode[VD_PATH_MAX] = {PROC_OFF, PROC_OFF};
module_param_array(sdr_process_mode, uint, &vd_path_max, 0444);
MODULE_PARM_DESC(sdr_process_mode, "\n current hdr_process_mode\n");

static int sdr_saturation_offset = 20; /* 0: sdr->sdr, 1:sdr->hdr */
module_param(sdr_saturation_offset, int, 0664);
MODULE_PARM_DESC(sdr_saturation_offset, "\n add saturation\n");

static uint force_csc_type = 0xff;
module_param(force_csc_type, uint, 0664);
MODULE_PARM_DESC(force_csc_type, "\n force colour space convert type\n");

static uint cur_hdr_support;
module_param(cur_hdr_support, uint, 0664);
MODULE_PARM_DESC(cur_hdr_support, "\n cur_hdr_support\n");

static uint cur_colorimetry_support;
module_param(cur_colorimetry_support, uint, 0664);
MODULE_PARM_DESC(cur_colorimetry_support, "\n cur_colorimetry_support\n");

static uint cur_hlg_support;
module_param(cur_hlg_support, uint, 0664);
MODULE_PARM_DESC(cur_hlg_support, "\n cur_hlg_support\n");

static uint cur_color_fmt;
module_param(cur_color_fmt, uint, 0664);
MODULE_PARM_DESC(cur_color_fmt, "\n cur_color_fmt\n");

static uint range_control;
module_param(range_control, uint, 0664);
MODULE_PARM_DESC(range_control, "\n range_control 0:limit 1:full\n");

/* bit 0: use source primary,*/
/* bit 1: use display primary,*/
/* bit 2: adjust contrast according to source lumin,*/
/* bit 3: adjust saturation according to source lumin */
/* bit 4: convert HLG to HDR if sink not supprot HLG but HDR */
uint hdr_flag = (1 << 0) | (1 << 1) | (0 << 2) | (0 << 3) | (1 << 4);
module_param(hdr_flag, uint, 0664);
MODULE_PARM_DESC(hdr_flag, "\n set hdr_flag\n");

/* 0: off, 1: normal, 2: bypass */
static int video_process_status[VD_PATH_MAX] = {2, 2};
module_param_array(video_process_status, uint, &vd_path_max, 0664);
MODULE_PARM_DESC(video_process_status, "\n video_process_status\n");

void set_hdr_module_status(enum vd_path_e vd_path, int status)
{
	video_process_status[vd_path] = status;
	pr_csc(4, "video_process_status[VD%d] = %d",
	       vd_path + 1, status);
}
EXPORT_SYMBOL(set_hdr_module_status);

#define PROC_FLAG_FORCE_PROCESS 1
static uint video_process_flags[VD_PATH_MAX];
module_param_array(video_process_flags, uint, &vd_path_max, 0664);
MODULE_PARM_DESC(video_process_flags, "\n video_process_flags\n");

static uint rdma_flag =
	(1 << VPP_MATRIX_OSD) |
	(1 << VPP_MATRIX_VD1) |
	(1 << VPP_MATRIX_VD2) |
	(1 << VPP_MATRIX_POST) |
	(1 << VPP_MATRIX_XVYCC);
module_param(rdma_flag, uint, 0664);
MODULE_PARM_DESC(rdma_flag, "\n set rdma_flag\n");

#define MAX_KNEE_SETTING	35
/* recommended setting for 100 nits panel: */
/* 0,16,96,224,320,544,720,864,1000,1016,1023 */
/* knee factor = 256 */
static int num_knee_setting = MAX_KNEE_SETTING;
static int knee_setting[MAX_KNEE_SETTING] = {
	/* 0, 16, 96, 224, 320, 544, 720, 864, 1000, 1016, 1023 */
	0, 16, 36, 59, 71, 96,
	120, 145, 170, 204, 230, 258,
	288, 320, 355, 390, 428, 470,
	512, 554, 598, 650, 720, 758,
	790, 832, 864, 894, 920, 945,
	968, 980, 1000, 1016, 1023
};

static int num_knee_linear_setting = MAX_KNEE_SETTING;
static int knee_linear_setting[MAX_KNEE_SETTING] = {
	0x000,
	0x010,
	0x02f,
	0x04e,
	0x06d,
	0x08c,
	0x0ab,
	0x0ca,
	0x0e9,
	0x108,
	0x127,
	0x146,
	0x165,
	0x184,
	0x1a3,
	0x1c2,
	0x1e1,
	0x200,
	0x21f,
	0x23e,
	0x25d,
	0x27c,
	0x29b,
	0x2ba,
	0x2d9,
	0x2f8,
	0x317,
	0x336,
	0x355,
	0x374,
	0x393,
	0x3b2,
	0x3d1,
	0x3f0,
	0x3ff
};

static bool lut_289_en = 1;
module_param(lut_289_en, bool, 0664);
MODULE_PARM_DESC(lut_289_en, "\n if enable 289 lut\n");

/*for gxtvbb(968), only 289 point lut for hdr curve set */
/*default for 350nit panel*/
unsigned int lut_289_mapping[LUT_289_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0x9, 0xa, 0xb, 0xd, 0xe, 0x10, 0x11, 0x12,
	  0x14, 0x15, 0x17, 0x18, 0x1a, 0x1c, 0x1d, 0x1f,
	  0x20, 0x22, 0x24, 0x26, 0x27, 0x29, 0x2b, 0x2d,
	 0x2f,  0x31,  0x33,  0x35,  0x37,  0x39,  0x3b,  0x3d,
	 0x40,  0x42,  0x44,  0x46,  0x49,  0x4b,  0x4e,  0x50,
	 0x53,  0x55,  0x58,  0x5a,  0x5d,  0x60,  0x62,  0x65,
	 0x68,  0x6b,  0x6e,  0x71,  0x74,  0x77,  0x7a,  0x7e,
	 0x81,  0x84,  0x87,  0x8b,  0x8e,  0x92,  0x95,  0x99,
	 0x9d,  0xa1,  0xa4,  0xa8,  0xac,  0xb0,  0xb4,  0xb8,
	 0xbc,  0xc1,  0xc5,  0xc9,  0xce,  0xd2,  0xd7,  0xdc,
	 0xe0,  0xe5,  0xea,  0xef,  0xf4,  0xf9,  0xfe,  0x104,
	 0x109,  0x10e,  0x114,  0x11a,  0x11f,  0x125,  0x12b,  0x131,
	 0x137,  0x13d,  0x143,  0x14a,  0x150,  0x156,  0x15d,  0x164,
	 0x16b,  0x172,  0x179,  0x180,  0x187,  0x18e,  0x196,  0x19d,
	 0x1a5,  0x1ad,  0x1b5,  0x1bd,  0x1c5,  0x1cd,  0x1d6,  0x1de,
	 0x1e7,  0x1f0,  0x1f9,  0x202,  0x20b,  0x214,  0x21e,  0x227,
	 0x231,  0x23b,  0x245,  0x24f,  0x25a,  0x264,  0x26f,  0x27a,
	 0x285,  0x290,  0x29c,  0x2a7,  0x2b3,  0x2bf,  0x2cb,  0x2d7,
	 0x2e3,  0x2f0,  0x2fd,  0x30a,  0x317,  0x325,  0x332,  0x33e,
	 0x34a,  0x356,  0x362,  0x36d,  0x377,  0x381,  0x38b,  0x394,
	 0x39c,  0x3a4,  0x3ac,  0x3b3,  0x3ba,  0x3c1,  0x3c7,  0x3cd,
	 0x3d2,  0x3d7,  0x3db,  0x3df,  0x3e3,  0x3e6,  0x3e9,  0x3ec,
	 0x3ef,  0x3f1,  0x3f3,  0x3f5,  0x3f6,  0x3f7,  0x3f8,  0x3f9,
	 0x3fa,  0x3fa,  0x3fb,  0x3fb,  0x3fb,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,  0x3fc,
	 0x3fc, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff,
	0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff,
	0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff, 0x3ff,
	0x3ff
};

static int knee_factor; /* 0 ~ 256, 128 = 0.5 */
static int knee_interpolation_mode = 1; /* 0: linear, 1: cubic */

module_param_array(knee_setting, int, &num_knee_setting, 0664);
MODULE_PARM_DESC(knee_setting, "\n knee_setting, 256=1.0\n");

module_param_array(knee_linear_setting, int, &num_knee_linear_setting, 0444);
MODULE_PARM_DESC(knee_linear_setting, "\n reference linear knee_setting\n");

module_param(knee_factor, int, 0664);
MODULE_PARM_DESC(knee_factor, "\n knee_factor, 255=1.0\n");

module_param(knee_interpolation_mode, int, 0664);
MODULE_PARM_DESC(knee_interpolation_mode, "\n 0: linear, 1: cubic\n");

#define NUM_MATRIX_PARAM 16
static uint num_customer_matrix_param = NUM_MATRIX_PARAM;
static uint customer_matrix_param[NUM_MATRIX_PARAM] = {
	0, 0, 0,
	0x0d49, 0x1b4d, 0x1f6b,
	0x1f01, 0x0910, 0x1fef,
	0x1fdb, 0x1f32, 0x08f3,
	0, 0, 0,
	1
};

static bool customer_matrix_en = true;

#define INORM	50000
#define BL		16

static u32 bt709_primaries[3][2] = {
	{0.30 * INORM + 0.5, 0.60 * INORM + 0.5},	/* G */
	{0.15 * INORM + 0.5, 0.06 * INORM + 0.5},	/* B */
	{0.64 * INORM + 0.5, 0.33 * INORM + 0.5},	/* R */
};

static u32 bt709_white_point[2] = {
	0.3127 * INORM + 0.5, 0.3290 * INORM + 0.5
};

static u32 bt2020_primaries[3][2] = {
	{0.17 * INORM + 0.5, 0.797 * INORM + 0.5},	/* G */
	{0.131 * INORM + 0.5, 0.046 * INORM + 0.5},	/* B */
	{0.708 * INORM + 0.5, 0.292 * INORM + 0.5},	/* R */
};

static u32 bt2020_white_point[2] = {
	0.3127 * INORM + 0.5, 0.3290 * INORM + 0.5
};

/* 0: off, 1: on */
static int customer_master_display_en;
static uint num_customer_master_display_param = 12;
static uint customer_master_display_param[12] = {
	0.17 * INORM + 0.5, 0.797 * INORM + 0.5,      /* G */
	0.131 * INORM + 0.5, 0.046 * INORM + 0.5,     /* B */
	0.708 * INORM + 0.5, 0.292 * INORM + 0.5,     /* R */
	0.3127 * INORM + 0.5, 0.3290 * INORM + 0.5,   /* W */
	5000 * 10000, 50,
	/* man/min lumin */
	5000, 50
	/* content lumin and frame average */
};

static int customer_panel_lumin = 380;
module_param(customer_panel_lumin, int, 0664);
MODULE_PARM_DESC(customer_panel_lumin, "\n customer_panel_lumin\n");

int customer_hdr_clipping;
module_param(customer_hdr_clipping, int, 0664);
MODULE_PARM_DESC(customer_hdr_clipping, "\n customer_hdr_clipping\n");

module_param(customer_matrix_en, bool, 0664);
MODULE_PARM_DESC(customer_matrix_en, "\n if enable customer matrix\n");

module_param_array(customer_matrix_param, uint,
	&num_customer_matrix_param, 0664);
MODULE_PARM_DESC(customer_matrix_param,
	"\n matrix from source primary to panel primary\n");

module_param(customer_master_display_en, int, 0664);
MODULE_PARM_DESC(customer_master_display_en,
	"\n if enable customer primaries and white point\n");

module_param_array(customer_master_display_param, uint,
	&num_customer_master_display_param, 0664);
MODULE_PARM_DESC(customer_master_display_param,
	"\n matrix from source primary and white point\n");

/* 0: off, 1: on */
static int customer_hdmi_display_en;
static uint num_customer_hdmi_display_param = 14;
static uint customer_hdmi_display_param[14] = {
	9, /* color priamry = bt2020 */
	16, /* characteristic = st2084 */
	0.17 * INORM + 0.5, 0.797 * INORM + 0.5,      /* G */
	0.131 * INORM + 0.5, 0.046 * INORM + 0.5,     /* B */
	0.708 * INORM + 0.5, 0.292 * INORM + 0.5,     /* R */
	0.3127 * INORM + 0.5, 0.3290 * INORM + 0.5,   /* W */
	9997 * 10000, 0,
	/* man/min lumin */
	5000, 50
	/* content lumin and frame average */
};

module_param(customer_hdmi_display_en, int, 0664);
MODULE_PARM_DESC(customer_hdmi_display_en,
	"\n if enable customer primaries and white point\n");

module_param_array(customer_hdmi_display_param, uint,
	&num_customer_hdmi_display_param, 0664);
MODULE_PARM_DESC(customer_hdmi_display_param,
	"\n matrix from source primary and white point\n");

/* sat offset when > 1200 and <= 1200 */
static uint num_extra_sat_lut = 2;
static uint extra_sat_lut[] = {16, 32};
module_param_array(extra_sat_lut, uint,
	&num_extra_sat_lut, 0664);
MODULE_PARM_DESC(extra_sat_lut,
	 "\n lookup table for saturation match source luminance.\n");

/* norm to 128 as 1, LUT can be changed */
static uint num_extra_con_lut = 5;
static uint extra_con_lut[] = {144, 136, 132, 130, 128};
module_param_array(extra_con_lut, uint,
	&num_extra_con_lut, 0664);
MODULE_PARM_DESC(extra_con_lut,
	 "\n lookup table for contrast match source luminance.\n");

#define clip(y, ymin, ymax) ((y > ymax) ? ymax : ((y < ymin) ? ymin : y))
static const int coef[] = {
	  0,   256,     0,     0, /* phase 0  */
	 -2,   256,     2,     0, /* phase 1  */
	 -4,   256,     4,     0, /* phase 2  */
	 -5,   254,     7,     0, /* phase 3  */
	 -7,   254,    10,    -1, /* phase 4  */
	 -8,   252,    13,    -1, /* phase 5  */
	-10,   251,    16,    -1, /* phase 6  */
	-11,   249,    19,    -1, /* phase 7  */
	-12,   247,    23,    -2, /* phase 8  */
	-13,   244,    27,    -2, /* phase 9  */
	-14,   242,    31,    -3, /* phase 10 */
	-15,   239,    35,    -3, /* phase 11 */
	-16,   236,    40,    -4, /* phase 12 */
	-17,   233,    44,    -4, /* phase 13 */
	-17,   229,    49,    -5, /* phase 14 */
	-18,   226,    53,    -5, /* phase 15 */
	-18,   222,    58,    -6, /* phase 16 */
	-18,   218,    63,    -7, /* phase 17 */
	-19,   214,    68,    -7, /* phase 18 */
	-19,   210,    73,    -8, /* phase 19 */
	-19,   205,    79,    -9, /* phase 20 */
	-19,   201,    83,    -9, /* phase 21 */
	-19,   196,    89,   -10, /* phase 22 */
	-19,   191,    94,   -10, /* phase 23 */
	-19,   186,   100,   -11, /* phase 24 */
	-18,   181,   105,   -12, /* phase 25 */
	-18,   176,   111,   -13, /* phase 26 */
	-18,   171,   116,   -13, /* phase 27 */
	-18,   166,   122,   -14, /* phase 28 */
	-17,   160,   127,   -14, /* phase 28 */
	-17,   155,   133,   -15, /* phase 30 */
	-16,   149,   138,   -15, /* phase 31 */
	-16,   144,   144,   -16  /* phase 32 */
};

int cubic_interpolation(int y0, int y1, int y2, int y3, int mu)
{
	int c0, c1, c2, c3;
	int d0, d1, d2, d3;

	if (mu <= 32) {
		c0 = coef[(mu << 2) + 0];
		c1 = coef[(mu << 2) + 1];
		c2 = coef[(mu << 2) + 2];
		c3 = coef[(mu << 2) + 3];
		d0 = y0; d1 = y1; d2 = y2; d3 = y3;
	} else {
		c0 = coef[((64 - mu) << 2) + 0];
		c1 = coef[((64 - mu) << 2) + 1];
		c2 = coef[((64 - mu) << 2) + 2];
		c3 = coef[((64 - mu) << 2) + 3];
		d0 = y3; d1 = y2; d2 = y1; d3 = y0;
	}
	return (d0 * c0 + d1 * c1 + d2 * c2 + d3 * c3 + 128) >> 8;
}

static int knee_lut_on;
static int cur_knee_factor = -1;
static void load_knee_lut(int on)
{
	int i, j, k;
	int value;
	int final_knee_setting[MAX_KNEE_SETTING];

	if ((cur_knee_factor != knee_factor) && (!lut_289_en) && (on)) {
		pr_csc(1, "Knee_factor changed from %d to %d\n",
			cur_knee_factor, knee_factor);
		for (i = 0; i < MAX_KNEE_SETTING; i++) {
			final_knee_setting[i] =
				knee_linear_setting[i] + (((knee_setting[i]
				- knee_linear_setting[i]) * knee_factor) >> 8);
			if (final_knee_setting[i] > 0x3ff)
				final_knee_setting[i] = 0x3ff;
			else if (final_knee_setting[i] < 0)
				final_knee_setting[i] = 0;
		}
		VSYNC_WR_MPEG_REG(XVYCC_LUT_CTL, 0x0);
		for (j = 0; j < 3; j++) {
			for (i = 0; i < 16; i++) {
				VSYNC_WR_MPEG_REG(
					XVYCC_LUT_R_ADDR_PORT + 2 * j, i);
				value = final_knee_setting[0]
					+ (((final_knee_setting[1]
					- final_knee_setting[0]) * i) >> 4);
				value = clip(value, 0, 0x3ff);
				VSYNC_WR_MPEG_REG(XVYCC_LUT_R_DATA_PORT + 2 * j,
						value);
				if (j == 0)
					pr_csc(1,
						"xvycc_lut[%1d][%3d] = 0x%03x\n",
						j, i, value);
			}
			for (i = 16; i < 272; i++) {
				k = 1 + ((i - 16) >> 3);
				VSYNC_WR_MPEG_REG(
					XVYCC_LUT_R_ADDR_PORT + 2 * j, i);
				if (knee_interpolation_mode == 0)
					value = final_knee_setting[k]
						+ (((final_knee_setting[k+1]
						- final_knee_setting[k])
						* ((i - 16) & 0x7)) >> 3);
				else
					value = cubic_interpolation(
						final_knee_setting[k-1],
						final_knee_setting[k],
						final_knee_setting[k+1],
						final_knee_setting[k+2],
						((i - 16) & 0x7) << 3);
				value = clip(value, 0, 0x3ff);
				VSYNC_WR_MPEG_REG(XVYCC_LUT_R_DATA_PORT + 2 * j,
						value);
				if (j == 0)
					pr_csc(1,
						"xvycc_lut[%1d][%3d] = 0x%03x\n",
						j, i, value);
			}
			for (i = 272; i < 289; i++) {
				k = MAX_KNEE_SETTING - 2;
				VSYNC_WR_MPEG_REG(
					XVYCC_LUT_R_ADDR_PORT + 2 * j, i);
				value = final_knee_setting[k]
					+ (((final_knee_setting[k+1]
					- final_knee_setting[k])
					* (i - 272)) >> 4);
				value = clip(value, 0, 0x3ff);
				VSYNC_WR_MPEG_REG(XVYCC_LUT_R_DATA_PORT + 2 * j,
						value);
				if (j == 0)
					pr_csc(1,
						"xvycc_lut[%1d][%3d] = 0x%03x\n",
						j, i, value);
			}
		}
		cur_knee_factor = knee_factor;
	}

	if ((cur_knee_factor != knee_factor) && (lut_289_en) && (on)) {
		VSYNC_WR_MPEG_REG(XVYCC_LUT_CTL, 0x0);
		VSYNC_WR_MPEG_REG(XVYCC_LUT_R_ADDR_PORT, 0);
		for (i = 0; i < LUT_289_SIZE; i++)
			VSYNC_WR_MPEG_REG(XVYCC_LUT_R_DATA_PORT,
				lut_289_mapping[i]);
		VSYNC_WR_MPEG_REG(XVYCC_LUT_R_ADDR_PORT + 2, 0);
		for (i = 0; i < LUT_289_SIZE; i++)
			VSYNC_WR_MPEG_REG(XVYCC_LUT_R_DATA_PORT + 2,
				lut_289_mapping[i]);
		VSYNC_WR_MPEG_REG(XVYCC_LUT_R_ADDR_PORT + 4, 0);
		for (i = 0; i < LUT_289_SIZE; i++)
			VSYNC_WR_MPEG_REG(XVYCC_LUT_R_DATA_PORT + 4,
				lut_289_mapping[i]);
		cur_knee_factor = knee_factor;
	}

	if (on) {
		VSYNC_WR_MPEG_REG(XVYCC_LUT_CTL, 0x7f);
		knee_lut_on = 1;
	} else {
		VSYNC_WR_MPEG_REG(XVYCC_LUT_CTL, 0x0f);
		knee_lut_on = 0;
	}
}

/***************************** gxl hdr ****************************/
/* 16 for [-2048, 0), 32 for [0, 1024), 17 for [1024, 2048) */
#define EOTF_INV_LUT_NEG2048_SIZE 16
#define EOTF_INV_LUT_SIZE 32
#define EOTF_INV_LUT_1024_SIZE 17

#define INVLUT_SDR2HDR 0x1
#define INVLUT_HLG 0x2
static unsigned int int_lut_sel[] = {0};

static unsigned int num_invlut_neg_mapping = EOTF_INV_LUT_NEG2048_SIZE;
static int invlut_y_neg[EOTF_INV_LUT_NEG2048_SIZE] = {
	-2048, -1920, -1792, -1664,
	-1536, -1408, -1280, -1152,
	-1024, -896, -768, -640,
	-512, -384, -256, -128
};

static unsigned int num_invlut_mapping = EOTF_INV_LUT_SIZE;
static unsigned int invlut_y[EOTF_INV_LUT_SIZE] = {
	0, 32, 64, 96, 128, 160, 192, 224,
	256, 288, 320, 352, 384, 416, 448, 480,
	512, 544, 576, 608, 640, 672, 704, 736,
	768, 800, 832, 864, 896, 928, 960, 992
};

static unsigned int num_invlut_1024_mapping = EOTF_INV_LUT_1024_SIZE;
static unsigned int invlut_y_1024[EOTF_INV_LUT_1024_SIZE] = {
	1024, 1088, 1152, 1216,
	1280, 1344, 1408, 1472,
	1536, 1600, 1664, 1728,
	1792, 1856, 1920, 1984,
	2047
};

static unsigned int num_invlut_hlg_mapping = EOTF_INV_LUT_SIZE;
static unsigned int invlut_hlg_y[EOTF_INV_LUT_SIZE] = {
	    0, 14, 28, 48, 69, 91, 114, 138,
	  163, 188, 215, 241, 269, 297, 325, 354,
	  383, 415, 450, 490, 535, 579, 622, 663,
	  705, 745, 786, 826, 866, 905, 945, 985
};

#define EOTF_LUT_SIZE 33
static unsigned int num_osd_eotf_r_mapping = EOTF_LUT_SIZE;
static unsigned int osd_eotf_r_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

static unsigned int num_osd_eotf_g_mapping = EOTF_LUT_SIZE;
static unsigned int osd_eotf_g_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

static unsigned int num_osd_eotf_b_mapping = EOTF_LUT_SIZE;
static unsigned int osd_eotf_b_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

static unsigned int num_video_eotf_r_mapping = EOTF_LUT_SIZE;
static unsigned int video_eotf_r_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

static unsigned int num_video_eotf_g_mapping = EOTF_LUT_SIZE;
static unsigned int video_eotf_g_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

static unsigned int num_video_eotf_b_mapping = EOTF_LUT_SIZE;
static unsigned int video_eotf_b_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

#define EOTF_COEFF_NORM(a) ((int)((((a) * 4096.0) + 1) / 2))
#define EOTF_COEFF_SIZE 10
#define EOTF_COEFF_RIGHTSHIFT 1
static unsigned int num_osd_eotf_coeff = EOTF_COEFF_SIZE;
static int osd_eotf_coeff[EOTF_COEFF_SIZE] = {
	EOTF_COEFF_NORM(1.0), EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(1.0), EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(1.0),
	EOTF_COEFF_RIGHTSHIFT /* right shift */
};

static unsigned int num_video_eotf_coeff = EOTF_COEFF_SIZE;
static int video_eotf_coeff[EOTF_COEFF_SIZE] = {
	EOTF_COEFF_NORM(1.0), EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(1.0), EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(0.0), EOTF_COEFF_NORM(1.0),
	EOTF_COEFF_RIGHTSHIFT /* right shift */
};

static unsigned int reload_mtx;
static unsigned int reload_lut;

/******************** osd oetf **************/

static unsigned int num_osd_oetf_r_mapping = OSD_OETF_LUT_SIZE;
static unsigned int osd_oetf_r_mapping[OSD_OETF_LUT_SIZE] = {
		0, 4, 8, 12,
		16, 20, 24, 28,
		31, 62, 93, 124,
		155, 186, 217, 248,
		279, 310, 341, 372,
		403, 434, 465, 496,
		527, 558, 589, 620,
		651, 682, 713, 744,
		775, 806, 837, 868,
		899, 930, 961, 992,
		1023
};

static unsigned int num_osd_oetf_g_mapping = OSD_OETF_LUT_SIZE;
static unsigned int osd_oetf_g_mapping[OSD_OETF_LUT_SIZE] = {
		0, 4, 8, 12,
		16, 20, 24, 28,
		31, 62, 93, 124,
		155, 186, 217, 248,
		279, 310, 341, 372,
		403, 434, 465, 496,
		527, 558, 589, 620,
		651, 682, 713, 744,
		775, 806, 837, 868,
		899, 930, 961, 992,
		1023
};

static unsigned int num_osd_oetf_b_mapping = OSD_OETF_LUT_SIZE;
static unsigned int osd_oetf_b_mapping[OSD_OETF_LUT_SIZE] = {
		0, 4, 8, 12,
		16, 20, 24, 28,
		31, 62, 93, 124,
		155, 186, 217, 248,
		279, 310, 341, 372,
		403, 434, 465, 496,
		527, 558, 589, 620,
		651, 682, 713, 744,
		775, 806, 837, 868,
		899, 930, 961, 992,
		1023
};

/************ video oetf ***************/

#define VIDEO_OETF_LUT_SIZE 289
static unsigned int num_video_oetf_r_mapping = VIDEO_OETF_LUT_SIZE;
static unsigned int video_oetf_r_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   4,    8,   12,   16,   20,   24,   28,   32,
	  36,   40,   44,   48,   52,   56,   60,   64,
	  68,   72,   76,   80,   84,   88,   92,   96,
	 100,  104,  108,  112,  116,  120,  124,  128,
	 132,  136,  140,  144,  148,  152,  156,  160,
	 164,  168,  172,  176,  180,  184,  188,  192,
	 196,  200,  204,  208,  212,  216,  220,  224,
	 228,  232,  236,  240,  244,  248,  252,  256,
	 260,  264,  268,  272,  276,  280,  284,  288,
	 292,  296,  300,  304,  308,  312,  316,  320,
	 324,  328,  332,  336,  340,  344,  348,  352,
	 356,  360,  364,  368,  372,  376,  380,  384,
	 388,  392,  396,  400,  404,  408,  412,  416,
	 420,  424,  428,  432,  436,  440,  444,  448,
	 452,  456,  460,  464,  468,  472,  476,  480,
	 484,  488,  492,  496,  500,  504,  508,  512,
	 516,  520,  524,  528,  532,  536,  540,  544,
	 548,  552,  556,  560,  564,  568,  572,  576,
	 580,  584,  588,  592,  596,  600,  604,  608,
	 612,  616,  620,  624,  628,  632,  636,  640,
	 644,  648,  652,  656,  660,  664,  668,  672,
	 676,  680,  684,  688,  692,  696,  700,  704,
	 708,  712,  716,  720,  724,  728,  732,  736,
	 740,  744,  748,  752,  756,  760,  764,  768,
	 772,  776,  780,  784,  788,  792,  796,  800,
	 804,  808,  812,  816,  820,  824,  828,  832,
	 836,  840,  844,  848,  852,  856,  860,  864,
	 868,  872,  876,  880,  884,  888,  892,  896,
	 900,  904,  908,  912,  916,  920,  924,  928,
	 932,  936,  940,  944,  948,  952,  956,  960,
	 964,  968,  972,  976,  980,  984,  988,  992,
	 996, 1000, 1004, 1008, 1012, 1016, 1020, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

static unsigned int num_video_oetf_g_mapping = VIDEO_OETF_LUT_SIZE;
static unsigned int video_oetf_g_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   4,    8,   12,   16,   20,   24,   28,   32,
	  36,   40,   44,   48,   52,   56,   60,   64,
	  68,   72,   76,   80,   84,   88,   92,   96,
	 100,  104,  108,  112,  116,  120,  124,  128,
	 132,  136,  140,  144,  148,  152,  156,  160,
	 164,  168,  172,  176,  180,  184,  188,  192,
	 196,  200,  204,  208,  212,  216,  220,  224,
	 228,  232,  236,  240,  244,  248,  252,  256,
	 260,  264,  268,  272,  276,  280,  284,  288,
	 292,  296,  300,  304,  308,  312,  316,  320,
	 324,  328,  332,  336,  340,  344,  348,  352,
	 356,  360,  364,  368,  372,  376,  380,  384,
	 388,  392,  396,  400,  404,  408,  412,  416,
	 420,  424,  428,  432,  436,  440,  444,  448,
	 452,  456,  460,  464,  468,  472,  476,  480,
	 484,  488,  492,  496,  500,  504,  508,  512,
	 516,  520,  524,  528,  532,  536,  540,  544,
	 548,  552,  556,  560,  564,  568,  572,  576,
	 580,  584,  588,  592,  596,  600,  604,  608,
	 612,  616,  620,  624,  628,  632,  636,  640,
	 644,  648,  652,  656,  660,  664,  668,  672,
	 676,  680,  684,  688,  692,  696,  700,  704,
	 708,  712,  716,  720,  724,  728,  732,  736,
	 740,  744,  748,  752,  756,  760,  764,  768,
	 772,  776,  780,  784,  788,  792,  796,  800,
	 804,  808,  812,  816,  820,  824,  828,  832,
	 836,  840,  844,  848,  852,  856,  860,  864,
	 868,  872,  876,  880,  884,  888,  892,  896,
	 900,  904,  908,  912,  916,  920,  924,  928,
	 932,  936,  940,  944,  948,  952,  956,  960,
	 964,  968,  972,  976,  980,  984,  988,  992,
	 996, 1000, 1004, 1008, 1012, 1016, 1020, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

static unsigned int num_video_oetf_b_mapping = VIDEO_OETF_LUT_SIZE;
static unsigned int video_oetf_b_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   4,    8,   12,   16,   20,   24,   28,   32,
	  36,   40,   44,   48,   52,   56,   60,   64,
	  68,   72,   76,   80,   84,   88,   92,   96,
	 100,  104,  108,  112,  116,  120,  124,  128,
	 132,  136,  140,  144,  148,  152,  156,  160,
	 164,  168,  172,  176,  180,  184,  188,  192,
	 196,  200,  204,  208,  212,  216,  220,  224,
	 228,  232,  236,  240,  244,  248,  252,  256,
	 260,  264,  268,  272,  276,  280,  284,  288,
	 292,  296,  300,  304,  308,  312,  316,  320,
	 324,  328,  332,  336,  340,  344,  348,  352,
	 356,  360,  364,  368,  372,  376,  380,  384,
	 388,  392,  396,  400,  404,  408,  412,  416,
	 420,  424,  428,  432,  436,  440,  444,  448,
	 452,  456,  460,  464,  468,  472,  476,  480,
	 484,  488,  492,  496,  500,  504,  508,  512,
	 516,  520,  524,  528,  532,  536,  540,  544,
	 548,  552,  556,  560,  564,  568,  572,  576,
	 580,  584,  588,  592,  596,  600,  604,  608,
	 612,  616,  620,  624,  628,  632,  636,  640,
	 644,  648,  652,  656,  660,  664,  668,  672,
	 676,  680,  684,  688,  692,  696,  700,  704,
	 708,  712,  716,  720,  724,  728,  732,  736,
	 740,  744,  748,  752,  756,  760,  764,  768,
	 772,  776,  780,  784,  788,  792,  796,  800,
	 804,  808,  812,  816,  820,  824,  828,  832,
	 836,  840,  844,  848,  852,  856,  860,  864,
	 868,  872,  876,  880,  884,  888,  892,  896,
	 900,  904,  908,  912,  916,  920,  924,  928,
	 932,  936,  940,  944,  948,  952,  956,  960,
	 964,  968,  972,  976,  980,  984,  988,  992,
	 996, 1000, 1004, 1008, 1012, 1016, 1020, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

#define COEFF_NORM(a) ((int)((((a) * 2048.0) + 1) / 2))
#define MATRIX_5x3_COEF_SIZE 24
/******* osd1 matrix0 *******/
/* default rgb to yuv_limit */
static unsigned int num_osd_matrix_coeff = MATRIX_5x3_COEF_SIZE;
static int osd_matrix_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.2126),	COEFF_NORM(0.7152),	COEFF_NORM(0.0722),
	COEFF_NORM(-0.11457),	COEFF_NORM(-0.38543),	COEFF_NORM(0.5),
	COEFF_NORM(0.5),	COEFF_NORM(-0.45415),	COEFF_NORM(-0.045847),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static unsigned int num_vd1_matrix_coeff = MATRIX_5x3_COEF_SIZE;
static int vd1_matrix_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(1.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(0.0),	COEFF_NORM(1.0),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static unsigned int num_vd2_matrix_coeff = MATRIX_5x3_COEF_SIZE;
static int vd2_matrix_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(1.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(0.0),	COEFF_NORM(1.0),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static unsigned int num_post_matrix_coeff = MATRIX_5x3_COEF_SIZE;
static int post_matrix_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(1.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(0.0),	COEFF_NORM(1.0),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static unsigned int num_xvycc_matrix_coeff = MATRIX_5x3_COEF_SIZE;
static int xvycc_matrix_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(1.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(0.0),	COEFF_NORM(1.0),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

/****************** osd eotf ********************/
module_param_array(osd_eotf_coeff, int,
	&num_osd_eotf_coeff, 0664);
MODULE_PARM_DESC(osd_eotf_coeff, "\n matrix for osd eotf\n");

module_param_array(osd_eotf_r_mapping, uint,
	&num_osd_eotf_r_mapping, 0664);
MODULE_PARM_DESC(osd_eotf_r_mapping, "\n lut for osd r eotf\n");

module_param_array(osd_eotf_g_mapping, uint,
	&num_osd_eotf_g_mapping, 0664);
MODULE_PARM_DESC(osd_eotf_g_mapping, "\n lut for osd g eotf\n");

module_param_array(osd_eotf_b_mapping, uint,
	&num_osd_eotf_b_mapping, 0664);
MODULE_PARM_DESC(osd_eotf_b_mapping, "\n lut for osd b eotf\n");

module_param_array(osd_matrix_coeff, int,
	&num_osd_matrix_coeff, 0664);
MODULE_PARM_DESC(osd_matrix_coeff, "\n coef for osd matrix\n");

/****************** video eotf ********************/
module_param_array(invlut_y_neg, int,
	&num_invlut_neg_mapping, 0664);
MODULE_PARM_DESC(invlut_y_neg, "\n lut for inv y -2048..0 eotf\n");

module_param_array(invlut_y, uint,
	&num_invlut_mapping, 0664);
MODULE_PARM_DESC(invlut_y, "\n lut for inv y 0..1024 eotf\n");

module_param_array(invlut_y_1024, uint,
	&num_invlut_1024_mapping, 0664);
MODULE_PARM_DESC(invlut_y_1024, "\n lut for inv y 1024..2048 eotf\n");

module_param_array(invlut_hlg_y, int,
	&num_invlut_hlg_mapping, 0664);
MODULE_PARM_DESC(invlut_hlg_y, "\n lut for hlg 0..1024 eotf\n");

module_param_array(video_eotf_coeff, int,
	&num_video_eotf_coeff, 0664);
MODULE_PARM_DESC(video_eotf_coeff, "\n matrix for video eotf\n");

module_param_array(video_eotf_r_mapping, uint,
	&num_video_eotf_r_mapping, 0664);
MODULE_PARM_DESC(video_eotf_r_mapping, "\n lut for video r eotf\n");

module_param_array(video_eotf_g_mapping, uint,
	&num_video_eotf_g_mapping, 0664);
MODULE_PARM_DESC(video_eotf_g_mapping, "\n lut for video g eotf\n");

module_param_array(video_eotf_b_mapping, uint,
	&num_video_eotf_b_mapping, 0664);
MODULE_PARM_DESC(video_eotf_b_mapping, "\n lut for video b eotf\n");

/****************** osd oetf ********************/
module_param_array(osd_oetf_r_mapping, uint,
	&num_osd_oetf_r_mapping, 0664);
MODULE_PARM_DESC(osd_oetf_r_mapping, "\n lut for osd r oetf\n");

module_param_array(osd_oetf_g_mapping, uint,
	&num_osd_oetf_g_mapping, 0664);
MODULE_PARM_DESC(osd_oetf_g_mapping, "\n lut for osd g oetf\n");

module_param_array(osd_oetf_b_mapping, uint,
	&num_osd_oetf_b_mapping, 0664);
MODULE_PARM_DESC(osd_oetf_b_mapping, "\n lut for osd b oetf\n");

/****************** video oetf ********************/
module_param_array(video_oetf_r_mapping, uint,
	&num_video_oetf_r_mapping, 0664);
MODULE_PARM_DESC(video_oetf_r_mapping, "\n lut for video r oetf\n");

module_param_array(video_oetf_g_mapping, uint,
	&num_video_oetf_g_mapping, 0664);
MODULE_PARM_DESC(video_oetf_g_mapping, "\n lut for video g oetf\n");

module_param_array(video_oetf_b_mapping, uint,
	&num_video_oetf_b_mapping, 0664);
MODULE_PARM_DESC(video_oetf_b_mapping, "\n lut for video b oetf\n");

/****************** vpp matrix ********************/

module_param_array(vd1_matrix_coeff, int,
	&num_vd1_matrix_coeff, 0664);
MODULE_PARM_DESC(vd1_matrix_coeff, "\n vd1 matrix\n");

module_param_array(vd2_matrix_coeff, int,
	&num_vd2_matrix_coeff, 0664);
MODULE_PARM_DESC(vd2_matrix_coeff, "\n vd2 matrix\n");

module_param_array(post_matrix_coeff, int,
	&num_post_matrix_coeff, 0664);
MODULE_PARM_DESC(post_matrix_coeff, "\n post matrix\n");

module_param_array(xvycc_matrix_coeff, int,
	&num_xvycc_matrix_coeff, 0664);
MODULE_PARM_DESC(xvycc_matrix_coeff, "\n xvycc matrix\n");

static unsigned int mtx_en_mux;
module_param(mtx_en_mux, uint, 0664);
MODULE_PARM_DESC(mtx_en_mux, "\n mtx enable mux\n");

/****************** matrix/lut reload********************/

module_param(reload_mtx, uint, 0664);
MODULE_PARM_DESC(reload_mtx, "\n reload matrix coeff\n");

module_param(reload_lut, uint, 0664);
MODULE_PARM_DESC(reload_lut, "\n reload lut settings\n");

static int RGB709_to_YUV709_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.2126),	COEFF_NORM(0.7152),	COEFF_NORM(0.0722),
	COEFF_NORM(-0.114572),	COEFF_NORM(-0.385428),	COEFF_NORM(0.5),
	COEFF_NORM(0.5),	COEFF_NORM(-0.454153),	COEFF_NORM(-0.045847),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	0, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int RGB709_to_YUV709l_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.181873),	COEFF_NORM(0.611831),	COEFF_NORM(0.061765),
	COEFF_NORM(-0.100251),	COEFF_NORM(-0.337249),	COEFF_NORM(0.437500),
	COEFF_NORM(0.437500),	COEFF_NORM(-0.397384),	COEFF_NORM(-0.040116),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int RGB2020_to_YUV2020l_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.224732),	COEFF_NORM(0.580008),	COEFF_NORM(0.050729),
	COEFF_NORM(-0.122176),	COEFF_NORM(-0.315324),	COEFF_NORM(0.437500),
	COEFF_NORM(0.437500),	COEFF_NORM(-0.402312),	COEFF_NORM(-0.035188),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV709f_to_YUV709l_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, -512, -512, /* pre offset */
	COEFF_NORM(0.859),	COEFF_NORM(0),	COEFF_NORM(0),
	COEFF_NORM(0),	COEFF_NORM(0.878),	COEFF_NORM(0),
	COEFF_NORM(0),	COEFF_NORM(0),	COEFF_NORM(0.878),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV709l_to_YUV709f_coeff[MATRIX_5x3_COEF_SIZE] = {
	64, -512, -512, /* pre offset */
	COEFF_NORM(1.16895),	COEFF_NORM(0),	COEFF_NORM(0),
	COEFF_NORM(0),	COEFF_NORM(1.14286),	COEFF_NORM(0),
	COEFF_NORM(0),	COEFF_NORM(0),	COEFF_NORM(1.14286),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	0, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV709l_to_RGB709_coeff[MATRIX_5x3_COEF_SIZE] = {
	-64, -512, -512, /* pre offset */
	COEFF_NORM(1.16895),	COEFF_NORM(0.00000),	COEFF_NORM(1.79977),
	COEFF_NORM(1.16895),	COEFF_NORM(-0.21408),	COEFF_NORM(-0.53500),
	COEFF_NORM(1.16895),	COEFF_NORM(2.12069),	COEFF_NORM(0.00000),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV709l_to_YUV2020_coeff[MATRIX_5x3_COEF_SIZE] = {
	-64, -512, -512, /* pre offset */
	0x400,	0x1fe4,	0x2b,
	0,	0x39f,	0x1fe5,
	0x0,	0xc,	0x321,
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV709f_to_RGB709_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, -512, -512, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.00000),	COEFF_NORM(1.575),
	COEFF_NORM(1.0),	COEFF_NORM(-0.187),	COEFF_NORM(-0.468),
	COEFF_NORM(1.0),	COEFF_NORM(1.856),	COEFF_NORM(0.00000),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV601l_to_YUV709l_coeff[MATRIX_5x3_COEF_SIZE] = {
	-64, -512, -512, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(-0.115),	COEFF_NORM(-0.207),
	COEFF_NORM(0),	COEFF_NORM(1.018),	COEFF_NORM(0.114),
	COEFF_NORM(0),	COEFF_NORM(0.075),	COEFF_NORM(1.025),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV601f_to_YUV709f_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, -512, -512, /* pre offset */
	COEFF_NORM(1.00000),	COEFF_NORM(-0.118),	COEFF_NORM(-0.212),
	COEFF_NORM(0.00000),	COEFF_NORM(1.018),	COEFF_NORM(0.114),
	COEFF_NORM(0.00000),	COEFF_NORM(0.075),	COEFF_NORM(1.025),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV709l_to_YUV601l_coeff[MATRIX_5x3_COEF_SIZE] = {
	-64, -512, -512, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.1),	COEFF_NORM(0.192),
	COEFF_NORM(0),	COEFF_NORM(0.990),	COEFF_NORM(-0.110),
	COEFF_NORM(0),	COEFF_NORM(-0.072),	COEFF_NORM(0.984),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV601l_to_YUV709f_coeff[MATRIX_5x3_COEF_SIZE] = {
	-64, -512, -512, /* pre offset */
	COEFF_NORM(1.164),	COEFF_NORM(-0.134),	COEFF_NORM(-0.241),
	COEFF_NORM(0),	COEFF_NORM(1.160),	COEFF_NORM(-0.129),
	COEFF_NORM(0),	COEFF_NORM(-0.085),	COEFF_NORM(1.167),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int YUV601f_to_YUV709l_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, -512, -512, /* pre offset */
	COEFF_NORM(0.859),	COEFF_NORM(-0.101),	COEFF_NORM(-0.182),
	COEFF_NORM(0),	COEFF_NORM(0.894),	COEFF_NORM(-0.100),
	COEFF_NORM(0),	COEFF_NORM(-0.066),	COEFF_NORM(0.900),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

#if 0
static int YUV709f_to_YUV601f_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, -512, -512, /* pre offset */
	COEFF_NORM(1.00000),	COEFF_NORM(0.08835),	COEFF_NORM(0.37521),
	COEFF_NORM(0.00000),	COEFF_NORM(1.03570),	COEFF_NORM(-0.20445),
	COEFF_NORM(0.00000),	COEFF_NORM(-0.11088),	COEFF_NORM(1.57010),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};
#endif
#if 0
/*  eotf matrix: RGB2020 to RGB709 */
static int eotf_RGB2020_to_RGB709_coeff[EOTF_COEFF_SIZE] = {
	EOTF_COEFF_NORM(1.6607056/2), EOTF_COEFF_NORM(-0.5877533/2),
	EOTF_COEFF_NORM(-0.0729065/2),
	EOTF_COEFF_NORM(-0.1245575/2), EOTF_COEFF_NORM(1.1329346/2),
	EOTF_COEFF_NORM(-0.0083771/2),
	EOTF_COEFF_NORM(-0.0181122/2), EOTF_COEFF_NORM(-0.1005249/2),
	EOTF_COEFF_NORM(1.1186371/2),
	EOTF_COEFF_RIGHTSHIFT
};
#endif

static int bypass_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(1.0),	COEFF_NORM(0.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(1.0),	COEFF_NORM(0.0),
	COEFF_NORM(0.0),	COEFF_NORM(0.0),	COEFF_NORM(1.0),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

/*  eotf matrix: bypass */
static int eotf_bypass_coeff[EOTF_COEFF_SIZE] = {
	EOTF_COEFF_NORM(1.0),	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(1.0),	EOTF_COEFF_NORM(0.0),
	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(0.0),	EOTF_COEFF_NORM(1.0),
	EOTF_COEFF_RIGHTSHIFT /* right shift */
};

static int eotf_RGB709_to_RGB2020_coeff[EOTF_COEFF_SIZE] = {
	EOTF_COEFF_NORM(0.627441),	EOTF_COEFF_NORM(0.329285),
	EOTF_COEFF_NORM(0.043274),
	EOTF_COEFF_NORM(0.069092),	EOTF_COEFF_NORM(0.919556),
	EOTF_COEFF_NORM(0.011322),
	EOTF_COEFF_NORM(0.016418),	EOTF_COEFF_NORM(0.088058),
	EOTF_COEFF_NORM(0.895554),
	EOTF_COEFF_RIGHTSHIFT /* right shift */
};

/* post matrix: YUV2020 limit to RGB2020 */
static int YUV2020l_to_RGB2020_coeff[MATRIX_5x3_COEF_SIZE] = {
	-64, -512, -512, /* pre offset */
	COEFF_NORM(1.16895),	COEFF_NORM(0.00000),	COEFF_NORM(1.68526),
	COEFF_NORM(1.16895),	COEFF_NORM(-0.18806),	COEFF_NORM(-0.65298),
	COEFF_NORM(1.16895),	COEFF_NORM(2.15017),	COEFF_NORM(0.00000),
	0, 0, 0, /* 30/31/32 */
	0, 0, 0, /* 40/41/42 */
	0, 0, 0, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

/* eotf lut: linear */
static unsigned int eotf_33_linear_mapping[EOTF_LUT_SIZE] = {
	0x0000,	0x0200,	0x0400, 0x0600,
	0x0800, 0x0a00, 0x0c00, 0x0e00,
	0x1000, 0x1200, 0x1400, 0x1600,
	0x1800, 0x1a00, 0x1c00, 0x1e00,
	0x2000, 0x2200, 0x2400, 0x2600,
	0x2800, 0x2a00, 0x2c00, 0x2e00,
	0x3000, 0x3200, 0x3400, 0x3600,
	0x3800, 0x3a00, 0x3c00, 0x3e00,
	0x4000
};

/* osd oetf lut: linear */
static unsigned int oetf_41_linear_mapping[OSD_OETF_LUT_SIZE] = {
		0, 4, 8, 12,
		16, 20, 24, 28,
		31, 62, 93, 124,
		155, 186, 217, 248,
		279, 310, 341, 372,
		403, 434, 465, 496,
		527, 558, 589, 620,
		651, 682, 713, 744,
		775, 806, 837, 868,
		899, 930, 961, 992,
		1023
};

/* following array generated from model, do not edit */
static int video_lut_swtich;
module_param(video_lut_swtich, int, 0664);
MODULE_PARM_DESC(video_lut_swtich, "\n video_lut_swtich\n");

/* gamma=2.200000 lumin=500 boost=0.075000 */
static unsigned int display_scale_factor =
	(unsigned int)((((1.000000) * 4096.0) + 1) / 2);

static unsigned int eotf_33_2084_mapping[EOTF_LUT_SIZE] = {
	    0,     3,     6,    11,    18,    27,    40,    58,
	   84,   119,   169,   237,   329,   455,   636,   881,
	 1209,  1648,  2234,  3014,  4050,  5425,  7251,  9673,
	12889, 13773, 14513, 15117, 15594, 15951, 16196, 16338,
	16383
};

static unsigned int oetf_289_gamma22_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,  125,  171,  206,  235,  259,  281,  302,
	 321,  339,  356,  371,  386,  401,  414,  427,
	 440,  453,  465,  476,  488,  498,  509,  519,
	 529,  539,  549,  558,  568,  577,  586,  595,
	 603,  612,  620,  628,  637,  645,  652,  660,
	 668,  675,  683,  690,  697,  705,  712,  719,
	 726,  733,  740,  748,  755,  763,  771,  778,
	 786,  793,  801,  808,  814,  821,  827,  832,
	 837,  842,  846,  849,  852,  854,  856,  858,
	 860,  862,  864,  866,  867,  869,  871,  873,
	 875,  877,  879,  880,  882,  884,  886,  887,
	 889,  891,  893,  894,  896,  898,  899,  901,
	 902,  904,  906,  907,  909,  910,  912,  914,
	 915,  917,  918,  920,  921,  923,  924,  925,
	 927,  928,  930,  931,  933,  934,  935,  937,
	 938,  939,  941,  942,  943,  945,  946,  947,
	 948,  950,  951,  952,  953,  954,  956,  957,
	 958,  959,  960,  961,  962,  963,  965,  966,
	 967,  968,  969,  970,  971,  972,  973,  974,
	 975,  976,  977,  978,  979,  980,  981,  981,
	 982,  983,  984,  985,  986,  987,  987,  988,
	 989,  990,  991,  991,  992,  993,  994,  995,
	 995,  996,  997,  997,  998,  999,  999, 1000,
	1001, 1001, 1002, 1003, 1003, 1004, 1004, 1005,
	1006, 1006, 1007, 1007, 1008, 1008, 1009, 1009,
	1010, 1010, 1011, 1011, 1012, 1012, 1012, 1013,
	1013, 1014, 1014, 1015, 1015, 1015, 1016, 1016,
	1016, 1017, 1017, 1017, 1018, 1018, 1018, 1019,
	1019, 1019, 1019, 1020, 1020, 1020, 1020, 1020,
	1021, 1021, 1021, 1021, 1021, 1022, 1022, 1022,
	1022, 1022, 1022, 1022, 1022, 1023, 1023, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

/* osd eotf lut: 709 */
static unsigned int osd_eotf_33_709_mapping[EOTF_LUT_SIZE] = {
	    0,   512,  1024,  1536,  2048,  2560,  3072,  3584,
	 4096,  4608,  5120,  5632,  6144,  6656,  7168,  7680,
	 8192,  8704,  9216,  9728, 10240, 10752, 11264, 11776,
	12288, 12800, 13312, 13824, 14336, 14848, 15360, 15872,
	16383
};

/* osd oetf lut: 2084 */
static unsigned int osd_oetf_41_2084_mapping[OSD_OETF_LUT_SIZE] = {
	   0,   21,   43,   64,   79,   90,  101,  111,
	 120,  174,  208,  233,  277,  313,  344,  372,
	 398,  420,  440,  459,  476,  492,  507,  522,
	 536,  549,  561,  574,  585,  596,  606,  616,
	 624,  632,  642,  661,  684,  706,  727,  749,
	1023
};

/* osd eotf lut: 709 from baozheng */
static unsigned int osd_eotf_33_709_mapping_100[EOTF_LUT_SIZE] = {
	    0,     8,    37,    90,   169,   276,   412,   579,
	  776,  1006,  1268,  1564,  1894,  2258,  2658,  3094,
	 3566,  4075,  4621,  5204,  5826,  6486,  7185,  7923,
	 8701,  9518, 10376, 11274, 12213, 13194, 14215, 15279,
	16384
};

/* osd oetf lut: 2084 from baozheng */
static unsigned int osd_oetf_41_2084_mapping_100[OSD_OETF_LUT_SIZE] = {
	   0,  110,  141,  162,  178,  191,  203,  212,
	 221,  270,  302,  325,  344,  360,  374,  386,
	 396,  406,  415,  423,  431,  438,  445,  451,
	 457,  462,  468,  473,  478,  482,  487,  491,
	 495,  499,  503,  507,  510,  514,  517,  520,
	 523
};

static unsigned int osd_eotf_33_709_mapping_290[EOTF_LUT_SIZE] = {
	    0,     8,    37,    90,   169,   276,   412,   579,
	  776,  1006,  1268,  1564,  1894,  2258,  2658,  3094,
	 3566,  4075,  4621,  5204,  5826,  6486,  7185,  7923,
	 8701,  9518, 10376, 11274, 12213, 13194, 14215, 15279,
	16384
};

/* osd oetf lut: 2084 from baozheng */
static unsigned int osd_oetf_41_2084_mapping_290[OSD_OETF_LUT_SIZE] = {
	   0,  141,  178,  203,  221,  236,  249,  260,
	 270,  325,  360,  386,  406,  423,  438,  451,
	 462,  473,  482,  491,  499,  507,  514,  520,
	 527,  532,  538,  543,  548,  553,  558,  562,
	 567,  571,  575,  579,  583,  586,  590,  593,
	 596
};

static unsigned int osd_eotf_33_709_mapping_330[EOTF_LUT_SIZE] = {
	    0,     8,    37,    90,   169,   276,   412,   579,
	  776,  1006,  1268,  1564,  1894,  2258,  2658,  3094,
	 3566,  4075,  4621,  5204,  5826,  6486,  7185,  7923,
	 8701,  9518, 10376, 11274, 12213, 13194, 14215, 15279,
	16384
};

/* osd oetf lut: 2084 from baozheng */
static unsigned int osd_oetf_41_2084_mapping_330[OSD_OETF_LUT_SIZE] = {
	   0,  166,  207,  233,  254,  270,  284,  296,
	 307,  366,  402,  429,  451,  469,  484,  498,
	 509,  520,  530,  539,  547,  555,  562,  569,
	 576,  582,  588,  593,  598,  603,  608,  613,
	 617,  621,  625,  629,  633,  637,  640,  644,
	 647
};

/* osd eotf lut: sdr->hlg */
static unsigned int osd_eotf_33_sdr2hlg_mapping[EOTF_LUT_SIZE] = {
	    0,   512,  1024,  1536,  2048,  2560,  3072,  3584,
	 4096,  4608,  5120,  5632,  6144,  6656,  7168,  7680,
	 8192,  8704,  9216,  9728, 10240, 10752, 11264, 11776,
	12288, 12800, 13312, 13824, 14336, 14848, 15360, 15872,
	16383
};

/* osd oetf lut:  sdr->hlg */
static unsigned int osd_oetf_41_sdr2hlg_mapping[OSD_OETF_LUT_SIZE] = {
	   0,   23,   36,   43,   49,   54,   59,   64,
	  69,   96,  127,  172,  218,  267,  316,  365,
	 415,  468,  519,  567,  607,  643,  676,  706,
	 733,  758,  782,  805,  826,  847,  866,  885,
	 903,  920,  936,  951,  965,  980,  994, 1009,
	1023
};

/* sdr eotf lut: 709 */
static unsigned int eotf_33_sdr_709_mapping[EOTF_LUT_SIZE] = {
	    0,   512,  1024,  1536,  2048,  2560,  3072,  3584,
	 4096,  4608,  5120,  5632,  6144,  6656,  7168,  7680,
	 8192,  8704,  9216,  9728, 10240, 10752, 11264, 11776,
	12288, 12800, 13312, 13824, 14336, 14848, 15360, 15872,
	16383
};

/* sdr oetf lut: 2084 */
static unsigned int oetf_sdr_2084_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,   32,   55,   72,   85,   96,  106,  115,
	 124,  132,  140,  147,  154,  160,  166,  171,
	 176,  181,  186,  190,  194,  198,  204,  208,
	 211,  215,  218,  221,  224,  227,  230,  233,
	 239,  245,  252,  257,  263,  268,  274,  279,
	 283,  288,  292,  296,  302,  306,  310,  315,
	 319,  323,  326,  331,  334,  339,  342,  346,
	 350,  354,  357,  361,  365,  368,  372,  376,
	 379,  382,  386,  389,  392,  396,  399,  402,
	 405,  408,  411,  413,  416,  419,  422,  424,
	 427,  429,  432,  434,  437,  439,  442,  444,
	 446,  449,  451,  454,  456,  459,  461,  463,
	 465,  468,  470,  472,  474,  476,  478,  481,
	 483,  485,  487,  489,  491,  493,  495,  497,
	 499,  501,  503,  505,  507,  509,  511,  513,
	 515,  516,  518,  520,  522,  524,  526,  527,
	 529,  531,  533,  535,  536,  538,  540,  541,
	 543,  545,  546,  548,  550,  551,  553,  555,
	 556,  558,  559,  561,  562,  564,  566,  567,
	 569,  570,  572,  573,  575,  576,  578,  579,
	 580,  582,  584,  585,  586,  588,  589,  591,
	 592,  594,  595,  596,  598,  599,  600,  602,
	 603,  604,  606,  607,  608,  609,  611,  612,
	 613,  615,  616,  617,  618,  619,  620,  621,
	 622,  623,  624,  625,  626,  627,  628,  629,
	 630,  631,  632,  634,  635,  636,  637,  638,
	 640,  641,  642,  644,  646,  648,  651,  654,
	 657,  661,  664,  666,  670,  672,  676,  678,
	 681,  684,  687,  690,  692,  695,  698,  701,
	 704,  706,  709,  712,  715,  718,  720,  723,
	 726,  729,  731,  734,  737,  740,  743,  746,
	 748,  751,  754,  758,  763,  772,  797,  833,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

/* end of array generated from model */

module_param(display_scale_factor, uint, 0664);
MODULE_PARM_DESC(display_scale_factor, "\n display scale factor\n");

/*HLG eotf and oetf curve*/
static unsigned int eotf_33_hlg_mapping[EOTF_LUT_SIZE] = {
	    0,     5,    21,    48,    85,   133,   192,   261,
	  341,   432,   533,   645,   768,   901,  1045,  1200,
	 1365,  1552,  1774,  2038,  2353,  2729,  3175,  3707,
	 4341,  5096,  5995,  7065,  8340,  9858, 11666, 13819,
	16383
};

static unsigned int oetf_289_2084_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	0, 249, 301, 335, 360, 379, 396, 410,
	423, 434, 444, 453, 462, 470, 477, 484,
	491, 497, 502, 508, 513, 518, 523, 528,
	532, 536, 540, 544, 548, 552, 555, 559,
	562, 565, 568, 571, 574, 577, 580, 583,
	586, 588, 591, 593, 596, 598, 601, 603,
	605, 607, 609, 612, 614, 616, 618, 620,
	622, 624, 625, 627, 629, 631, 633, 634,
	636, 638, 640, 641, 643, 644, 646, 647,
	649, 651, 652, 653, 655, 656, 658, 659,
	661, 662, 663, 665, 666, 667, 668, 670,
	671, 672, 673, 675, 676, 677, 678, 679,
	681, 682, 683, 684, 685, 686, 687, 688,
	689, 690, 691, 692, 694, 695, 696, 697,
	698, 699, 699, 700, 701, 702, 703, 704,
	705, 706, 707, 708, 709, 710, 711, 711,
	712, 713, 714, 715, 716, 717, 717, 718,
	719, 720, 721, 721, 722, 723, 724, 725,
	725, 726, 727, 728, 728, 729, 730, 731,
	731, 732, 733, 734, 734, 735, 736, 736,
	737, 738, 738, 739, 740, 741, 741, 742,
	743, 743, 744, 744, 745, 746, 746, 747,
	748, 748, 749, 750, 750, 751, 751, 752,
	753, 753, 754, 754, 755, 756, 756, 757,
	757, 758, 759, 759, 760, 760, 761, 761,
	762, 762, 763, 764, 764, 765, 765, 766,
	766, 767, 767, 768, 768, 769, 769, 770,
	771, 771, 772, 772, 773, 773, 774, 774,
	775, 775, 776, 776, 777, 777, 778, 778,
	778, 779, 779, 780, 780, 781, 781, 782,
	782, 783, 783, 784, 784, 785, 785, 785,
	786, 786, 787, 787, 788, 788, 789, 789,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

static int dvll_RGB_to_YUV709l_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.181873), COEFF_NORM(0.611831), COEFF_NORM(0.061765),
	COEFF_NORM(-0.100251), COEFF_NORM(-0.337249), COEFF_NORM(0.437500),
	COEFF_NORM(0.437500), COEFF_NORM(-0.397384), COEFF_NORM(-0.040116),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};
/*static unsigned int oetf_289_709_mapping[VIDEO_OETF_LUT_SIZE] = {*/
/*	   0,    0,    0,    0,    0,    0,    0,    0,*/
/*	   0,    0,    0,    0,    0,    0,    0,    0,*/
/*	   0,   17,   35,   52,   70,   84,  100,  114,*/
/*	 127,  140,  151,  163,  173,  183,  193,  202,*/
/*	 211,  220,  228,  236,  244,  252,  259,  266,*/
/*	 274,  280,  287,  294,  300,  307,  313,  319,*/
/*	 325,  331,  337,  343,  349,  354,  360,  365,*/
/*	 371,  376,  381,  386,  391,  396,  401,  406,*/
/*	 411,  416,  421,  425,  430,  435,  439,  444,*/
/*	 448,  453,  457,  461,  466,  470,  474,  478,*/
/*	 482,  486,  491,  495,  499,  503,  506,  510,*/
/*	 514,  518,  522,  526,  530,  533,  537,  541,*/
/*	 544,  548,  552,  555,  559,  562,  566,  569,*/
/*	 573,  576,  580,  583,  586,  590,  593,  597,*/
/*	 600,  603,  606,  610,  613,  616,  619,  623,*/
/*	 626,  629,  632,  635,  638,  641,  644,  647,*/
/*	 651,  654,  657,  660,  663,  666,  668,  671,*/
/*	 674,  677,  680,  683,  686,  689,  692,  695,*/
/*	 697,  700,  703,  706,  709,  711,  714,  717,*/
/*	 720,  722,  725,  728,  730,  733,  736,  738,*/
/*	 741,  744,  746,  749,  752,  754,  757,  759,*/
/*	 762,  765,  767,  770,  772,  775,  777,  780,*/
/*	 782,  785,  787,  790,  792,  795,  797,  800,*/
/*	 802,  804,  807,  809,  812,  814,  817,  819,*/
/*	 821,  824,  826,  828,  831,  833,  835,  838,*/
/*	 840,  842,  845,  847,  849,  852,  854,  856,*/
/*	 858,  861,  863,  865,  867,  870,  872,  874,*/
/*	 876,  878,  881,  883,  885,  887,  889,  892,*/
/*	 894,  896,  898,  900,  902,  905,  907,  909,*/
/*	 911,  913,  915,  917,  919,  922,  924,  926,*/
/*	 928,  930,  932,  934,  936,  938,  940,  942,*/
/*	 944,  946,  948,  950,  952,  954,  956,  958,*/
/*	 960,  962,  964,  966,  968,  970,  972,  974,*/
/*	 976,  978,  980,  982,  984,  986,  988,  990,*/
/*	 992,  994,  996,  998, 1000, 1002, 1004, 1006,*/
/*	1008, 1010, 1012, 1014, 1016, 1018, 1020, 1022,*/
/*	1023*/
/*};*/
/*end HLG eotf and oetf curve*/

/* video oetf: linear */
#if 0
static unsigned int oetf_289_linear_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,    0,    0,    0,    0,    0,    0,    0,
	   0,    0,    0,    0,    0,    0,    0,    0,
	   4,    8,   12,   16,   20,   24,   28,   32,
	  36,   40,   44,   48,   52,   56,   60,   64,
	  68,   72,   76,   80,   84,   88,   92,   96,
	 100,  104,  108,  112,  116,  120,  124,  128,
	 132,  136,  140,  144,  148,  152,  156,  160,
	 164,  168,  172,  176,  180,  184,  188,  192,
	 196,  200,  204,  208,  212,  216,  220,  224,
	 228,  232,  236,  240,  244,  248,  252,  256,
	 260,  264,  268,  272,  276,  280,  284,  288,
	 292,  296,  300,  304,  308,  312,  316,  320,
	 324,  328,  332,  336,  340,  344,  348,  352,
	 356,  360,  364,  368,  372,  376,  380,  384,
	 388,  392,  396,  400,  404,  408,  412,  416,
	 420,  424,  428,  432,  436,  440,  444,  448,
	 452,  456,  460,  464,  468,  472,  476,  480,
	 484,  488,  492,  496,  500,  504,  508,  512,
	 516,  520,  524,  528,  532,  536,  540,  544,
	 548,  552,  556,  560,  564,  568,  572,  576,
	 580,  584,  588,  592,  596,  600,  604,  608,
	 612,  616,  620,  624,  628,  632,  636,  640,
	 644,  648,  652,  656,  660,  664,  668,  672,
	 676,  680,  684,  688,  692,  696,  700,  704,
	 708,  712,  716,  720,  724,  728,  732,  736,
	 740,  744,  748,  752,  756,  760,  764,  768,
	 772,  776,  780,  784,  788,  792,  796,  800,
	 804,  808,  812,  816,  820,  824,  828,  832,
	 836,  840,  844,  848,  852,  856,  860,  864,
	 868,  872,  876,  880,  884,  888,  892,  896,
	 900,  904,  908,  912,  916,  920,  924,  928,
	 932,  936,  940,  944,  948,  952,  956,  960,
	 964,  968,  972,  976,  980,  984,  988,  992,
	 996, 1000, 1004, 1008, 1012, 1016, 1020, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};

/* video oetf: 709 */
static unsigned int oetf_289_709_mapping[VIDEO_OETF_LUT_SIZE] = {
	   0,   2,   4,   6,   8,  10,  12,  14,
	  16,  18,  20,  22,  24,  26,  28,  30,
	  32,  34,  36,  54,  72,  90, 106, 121,
	 135, 148, 160, 172, 183, 193, 203, 213,
	 222, 231, 239, 248, 256, 264, 272, 279,
	 286, 294, 301, 308, 314, 321, 327, 334,
	 340, 346, 352, 358, 364, 370, 376, 381,
	 387, 392, 398, 403, 408, 413, 418, 423,
	 428, 433, 438, 443, 448, 453, 457, 462,
	 467, 471, 476, 480, 484, 489, 493, 497,
	 502, 506, 510, 514, 518, 522, 527, 531,
	 535, 538, 542, 546, 550, 554, 558, 562,
	 565, 569, 573, 577, 580, 584, 587, 591,
	 595, 598, 602, 605, 609, 612, 616, 619,
	 622, 626, 629, 633, 636, 639, 642, 646,
	 649, 652, 655, 659, 662, 665, 668, 671,
	 674, 678, 681, 684, 687, 690, 693, 696,
	 699, 702, 705, 708, 711, 714, 717, 720,
	 722, 725, 728, 731, 734, 737, 740, 742,
	 745, 748, 751, 754, 756, 759, 762, 765,
	 767, 770, 773, 775, 778, 781, 783, 786,
	 789, 791, 794, 797, 799, 802, 804, 807,
	 809, 812, 815, 817, 820, 822, 825, 827,
	 830, 832, 835, 837, 840, 842, 845, 847,
	 849, 852, 854, 857, 859, 861, 864, 866,
	 869, 871, 873, 876, 878, 880, 883, 885,
	 887, 890, 892, 894, 897, 899, 901, 903,
	 906, 908, 910, 912, 915, 917, 919, 921,
	 924, 926, 928, 930, 932, 935, 937, 939,
	 941, 943, 945, 948, 950, 952, 954, 956,
	 958, 960, 963, 965, 967, 969, 971, 973,
	 975, 977, 979, 981, 984, 986, 988, 990,
	 992,  994,  996,  998, 1000, 1002, 1004, 1006,
	1008, 1010, 1012, 1014, 1016, 1018, 1020, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
	1023
};
#endif

#define SIGN(a) ((a < 0) ? "-" : "+")
#define DECI(a) ((a) / 1024)
#define FRAC(a) ((((a) >= 0) ? \
	((a) & 0x3ff) : ((~(a) + 1) & 0x3ff)) * 10000 / 1024)

const char matrix_name[7][16] = {
	"OSD",
	"VD1",
	"POST",
	"XVYCC",
	"EOTF",
	"OSD_EOTF",
	"VD2"
};
static void print_vpp_matrix(int m_select, int *s, int on)
{
	unsigned int size;

	if (s == NULL)
		return;
	if (m_select == VPP_MATRIX_OSD)
		size = MATRIX_5x3_COEF_SIZE;
	else if (m_select == VPP_MATRIX_POST)
		size = MATRIX_5x3_COEF_SIZE;
	else if (m_select == VPP_MATRIX_VD1)
		size = MATRIX_5x3_COEF_SIZE;
	else if (m_select == VPP_MATRIX_VD2)
		size = MATRIX_5x3_COEF_SIZE;
	else if (m_select == VPP_MATRIX_XVYCC)
		size = MATRIX_5x3_COEF_SIZE;
	else if (m_select == VPP_MATRIX_EOTF)
		size = EOTF_COEFF_SIZE;
	else if (m_select == VPP_MATRIX_OSD_EOTF)
		size = EOTF_COEFF_SIZE;
	else
		return;

	pr_csc(1, "%s matrix %s:\n", matrix_name[m_select],
		on ? "on" : "off");

	if (size == MATRIX_5x3_COEF_SIZE) {
		pr_csc(1,
		"\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
				SIGN(s[0]), DECI(s[0]), FRAC(s[0]),
				SIGN(s[3]), DECI(s[3]), FRAC(s[3]),
				SIGN(s[4]), DECI(s[4]), FRAC(s[4]),
				SIGN(s[5]), DECI(s[5]), FRAC(s[5]),
				SIGN(s[18]), DECI(s[18]), FRAC(s[18]));
		pr_csc(1,
		"\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
				SIGN(s[1]), DECI(s[1]), FRAC(s[1]),
				SIGN(s[6]), DECI(s[6]), FRAC(s[6]),
				SIGN(s[7]), DECI(s[7]), FRAC(s[7]),
				SIGN(s[8]), DECI(s[8]), FRAC(s[8]),
				SIGN(s[19]), DECI(s[19]), FRAC(s[19]));
		pr_csc(1,
		"\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
				SIGN(s[2]), DECI(s[2]), FRAC(s[2]),
				SIGN(s[9]), DECI(s[9]), FRAC(s[9]),
				SIGN(s[10]), DECI(s[10]), FRAC(s[10]),
				SIGN(s[11]), DECI(s[11]), FRAC(s[11]),
				SIGN(s[20]), DECI(s[20]), FRAC(s[20]));
		if (s[21]) {
			pr_csc(1, "\t\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
				SIGN(s[12]), DECI(s[12]), FRAC(s[12]),
				SIGN(s[13]), DECI(s[13]), FRAC(s[13]),
				SIGN(s[14]), DECI(s[14]), FRAC(s[14]));
			pr_csc(1, "\t\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
				SIGN(s[15]), DECI(s[15]), FRAC(s[15]),
				SIGN(s[16]), DECI(s[16]), FRAC(s[16]),
				SIGN(s[17]), DECI(s[17]), FRAC(s[17]));
		}
		if (s[22])
			pr_csc(1, "\tright shift=%d\n", s[22]);
	} else {
		pr_csc(1, "\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
			SIGN(s[0]), DECI(s[0]), FRAC(s[0]),
			SIGN(s[1]), DECI(s[1]), FRAC(s[1]),
			SIGN(s[2]), DECI(s[2]), FRAC(s[2]));
		pr_csc(1, "\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
			SIGN(s[3]), DECI(s[3]), FRAC(s[3]),
			SIGN(s[4]), DECI(s[4]), FRAC(s[4]),
			SIGN(s[5]), DECI(s[5]), FRAC(s[5]));
		pr_csc(1, "\t%s%1d.%04d\t%s%1d.%04d\t%s%1d.%04d\n",
			SIGN(s[6]), DECI(s[6]), FRAC(s[6]),
			SIGN(s[7]), DECI(s[7]), FRAC(s[7]),
			SIGN(s[8]), DECI(s[8]), FRAC(s[8]));
		if (s[9])
			pr_csc(1, "\tright shift=%d\n", s[9]);
	}
	pr_csc(1, "\n");
}

static int *cur_osd_mtx = RGB709_to_YUV709l_coeff;
static int *cur_post_mtx = bypass_coeff;
static int *cur_vd1_mtx = bypass_coeff;
static int cur_vd1_on = CSC_OFF;
static int cur_post_on = CSC_OFF;
void set_vpp_matrix(int m_select, int *s, int on)
{
	int *m = NULL;
	int size = 0;
	int i, reg_value;

	if (debug_csc && print_lut_mtx)
		print_vpp_matrix(m_select, s, on);
	if (m_select == VPP_MATRIX_OSD) {
		m = osd_matrix_coeff;
		size = MATRIX_5x3_COEF_SIZE;
		cur_osd_mtx = s;
	} else if (m_select == VPP_MATRIX_POST)	{
		m = post_matrix_coeff;
		size = MATRIX_5x3_COEF_SIZE;
		cur_post_mtx = s;
		cur_post_on = on;
	} else if (m_select == VPP_MATRIX_VD1) {
		m = vd1_matrix_coeff;
		size = MATRIX_5x3_COEF_SIZE;
		cur_vd1_mtx = s;
		cur_vd1_on = on;
	} else if (m_select == VPP_MATRIX_VD2) {
		m = vd2_matrix_coeff;
		size = MATRIX_5x3_COEF_SIZE;
	} else if (m_select == VPP_MATRIX_XVYCC) {
		m = xvycc_matrix_coeff;
		size = MATRIX_5x3_COEF_SIZE;
	} else if (m_select == VPP_MATRIX_EOTF) {
		m = video_eotf_coeff;
		size = EOTF_COEFF_SIZE;
	} else if (m_select == VPP_MATRIX_OSD_EOTF) {
		m = osd_eotf_coeff;
		size = EOTF_COEFF_SIZE;
	} else
		return;

	if (s)
		for (i = 0; i < size; i++)
			m[i] = s[i];
	else
		reload_mtx &= ~(1 << m_select);

	reg_value = READ_VPP_REG(VPP_MATRIX_CTRL);

	if (m_select == VPP_MATRIX_OSD) {
		/* osd matrix, VPP_MATRIX_0 */
		if (!is_meson_txlx_cpu() &&
			!is_meson_gxlx_cpu() &&
			!is_meson_txhd_cpu()) {
			/* not enable latched */
			hdr_osd_reg.viu_osd1_matrix_pre_offset0_1 =
				((m[0] & 0xfff) << 16) | (m[1] & 0xfff);
			hdr_osd_reg.viu_osd1_matrix_pre_offset2 =
				m[2] & 0xfff;
			hdr_osd_reg.viu_osd1_matrix_coef00_01 =
				((m[3] & 0x1fff) << 16) | (m[4] & 0x1fff);
			hdr_osd_reg.viu_osd1_matrix_coef02_10 =
				((m[5] & 0x1fff) << 16) | (m[6] & 0x1fff);
			hdr_osd_reg.viu_osd1_matrix_coef11_12 =
				((m[7] & 0x1fff) << 16) | (m[8] & 0x1fff);
			hdr_osd_reg.viu_osd1_matrix_coef20_21 =
				((m[9] & 0x1fff) << 16) | (m[10] & 0x1fff);
			if (m[21]) {
				hdr_osd_reg.viu_osd1_matrix_coef22_30 =
					((m[11] & 0x1fff) << 16) |
					(m[12] & 0x1fff);
				hdr_osd_reg.viu_osd1_matrix_coef31_32 =
					((m[13] & 0x1fff) << 16) |
					(m[14] & 0x1fff);
				hdr_osd_reg.viu_osd1_matrix_coef40_41 =
					((m[15] & 0x1fff) << 16) |
					(m[16] & 0x1fff);
				hdr_osd_reg.viu_osd1_matrix_colmod_coef42 =
					m[17] & 0x1fff;
			} else {
				hdr_osd_reg.viu_osd1_matrix_coef22_30 =
					(m[11] & 0x1fff) << 16;
			}
			hdr_osd_reg.viu_osd1_matrix_offset0_1 =
				((m[18] & 0xfff) << 16) | (m[19] & 0xfff);
			hdr_osd_reg.viu_osd1_matrix_offset2 =
				m[20] & 0xfff;

			hdr_osd_reg.viu_osd1_matrix_colmod_coef42 &= 0x3ff8ffff;
			hdr_osd_reg.viu_osd1_matrix_colmod_coef42 |=
				(m[21] << 30) | (m[22] << 16);

			/* 23 reserved for clipping control */
			hdr_osd_reg.viu_osd1_matrix_ctrl &= 0xfffffffc;
			hdr_osd_reg.viu_osd1_matrix_ctrl |= on;
		} else {
			/* move the matrix from osd to vpp in txlx */
			m = osd_matrix_coeff;
			if (is_meson_gxlx_cpu()) {
				if (on) {
					/*VSYNC_WR_MPEG_REG_BITS(*/
					/*VPP_MATRIX_CTRL,*/
					/*4, 8, 3);*/
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_PRE_OFFSET0_1,
						((m[0] & 0xfff) << 16)
						| (m[1] & 0xfff));
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_PRE_OFFSET2,
						m[2] & 0xfff);
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_COEF00_01,
						((m[3] & 0x1fff) << 16)
						| (m[4] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_COEF02_10,
						((m[5]	& 0x1fff) << 16)
						| (m[6] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_COEF11_12,
						((m[7] & 0x1fff) << 16)
						| (m[8] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_COEF20_21,
						((m[9] & 0x1fff) << 16)
						| (m[10] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_COLMOD_COEF42,
						m[11] & 0x1fff);
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_OFFSET0_1,
						((m[18] & 0xfff) << 16)
						| (m[19] & 0xfff));
					VSYNC_WR_MPEG_REG(
						VIU_OSD1_MATRIX_OFFSET2,
						m[20] & 0xfff);
				}
				/*VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CTRL,*/
				/*on, 7, 1);*/
				VSYNC_WR_MPEG_REG_BITS(
					VIU_OSD1_MATRIX_CTRL,
					on, 0, 1);
			} else {
				if (on) {
					/*VSYNC_WR_MPEG_REG_BITS(*/
					/*VPP_MATRIX_CTRL,*/
					/*4, 8, 3);*/
					reg_value = (reg_value &
								(~(7 << 8)))
								| (4 << 8);
					VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL,
						reg_value);
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_PRE_OFFSET0_1,
						((m[0] & 0xfff) << 16)
						| (m[1] & 0xfff));
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_PRE_OFFSET2,
						m[2] & 0xfff);
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_COEF00_01,
						((m[3] & 0x1fff) << 16)
						| (m[4] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_COEF02_10,
						((m[5]  & 0x1fff) << 16)
						| (m[6] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_COEF11_12,
						((m[7] & 0x1fff) << 16)
						| (m[8] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_COEF20_21,
						((m[9] & 0x1fff) << 16)
						| (m[10] & 0x1fff));
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_COEF22,
						m[11] & 0x1fff);
					if (m[21]) {
						VSYNC_WR_MPEG_REG(
							VPP_MATRIX_COEF13_14,
							((m[12] & 0x1fff) << 16)
							| (m[13] & 0x1fff));
						VSYNC_WR_MPEG_REG(
							VPP_MATRIX_COEF15_25,
							((m[14] & 0x1fff) << 16)
							| (m[17] & 0x1fff));
						VSYNC_WR_MPEG_REG(
							VPP_MATRIX_COEF23_24,
							((m[15] & 0x1fff) << 16)
							| (m[16] & 0x1fff));
					}
					if (is_meson_txhd_cpu()) {
						VSYNC_WR_MPEG_REG(
						VPP_MATRIX_OFFSET0_1,
							(((m[18]>>2)&0xfff)<<16)
							| ((m[19]>>2)&0xfff));
						VSYNC_WR_MPEG_REG(
							VPP_MATRIX_OFFSET2,
							(m[20] >> 2) & 0xfff);
					} else {
						VSYNC_WR_MPEG_REG(
							VPP_MATRIX_OFFSET0_1,
							((m[18] & 0xfff) << 16)
							| (m[19] & 0xfff));
						VSYNC_WR_MPEG_REG(
							VPP_MATRIX_OFFSET2,
							m[20] & 0xfff);
					}

					VSYNC_WR_MPEG_REG_BITS(
						VPP_MATRIX_CLIP,
						m[21], 3, 2);
					VSYNC_WR_MPEG_REG_BITS(
						VPP_MATRIX_CLIP,
						m[22], 5, 3);
				}
				/*VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CTRL,*/
				/*on, 7, 1);*/
				if (on)
					mtx_en_mux |= OSD1_MTX_EN_MASK;
				else
					mtx_en_mux &= ~OSD1_MTX_EN_MASK;
			}
		}
	} else if (m_select == VPP_MATRIX_EOTF) {
		/* eotf matrix, VPP_MATRIX_EOTF */
		/* enable latched */
		for (i = 0; i < 5; i++)
			VSYNC_WR_MPEG_REG(VIU_EOTF_CTL + i + 1,
				((m[i * 2] & 0x1fff) << 16)
				| (m[i * 2 + 1] & 0x1fff));
		if (is_meson_txlx_cpu() ||
			is_meson_gxlx_cpu() ||
			is_meson_txhd_cpu()) {
			VSYNC_WR_MPEG_REG(VIU_EOTF_CTL + 8, 0);
			VSYNC_WR_MPEG_REG(VIU_EOTF_CTL + 9, 0);
		}
		WRITE_VPP_REG_BITS(VIU_EOTF_CTL, on, 30, 1);
		WRITE_VPP_REG_BITS(VIU_EOTF_CTL, on, 31, 1);
	} else if (m_select == VPP_MATRIX_OSD_EOTF) {
		/* osd eotf matrix, VPP_MATRIX_OSD_EOTF */
		if (!is_meson_txlx_cpu() &&
			!is_meson_gxlx_cpu() &&
			!is_meson_txhd_cpu()) {
			/* enable latched */
			hdr_osd_reg.viu_osd1_eotf_coef00_01 =
				((m[0 * 2] & 0x1fff) << 16)
				| (m[0 * 2 + 1] & 0x1fff);

			hdr_osd_reg.viu_osd1_eotf_coef02_10 =
				((m[1 * 2] & 0x1fff) << 16)
				| (m[1 * 2 + 1] & 0x1fff);

			hdr_osd_reg.viu_osd1_eotf_coef11_12 =
				((m[2 * 2] & 0x1fff) << 16)
				| (m[2 * 2 + 1] & 0x1fff);

			hdr_osd_reg.viu_osd1_eotf_coef20_21 =
				((m[3 * 2] & 0x1fff) << 16)
				| (m[3 * 2 + 1] & 0x1fff);
			hdr_osd_reg.viu_osd1_eotf_coef22_rs =
				((m[4 * 2] & 0x1fff) << 16)
				| (m[4 * 2 + 1] & 0x1fff);

			hdr_osd_reg.viu_osd1_eotf_ctl &= 0x3fffffff;
			hdr_osd_reg.viu_osd1_eotf_ctl |=
				(on << 30) | (on << 31);
		} else {
			/* latch enable */
			WRITE_VPP_REG_BITS(VIU_OSD1_EOTF_CTL, 1, 26, 1);
			if (on) {
				VSYNC_WR_MPEG_REG(VIU_OSD1_EOTF_COEF00_01,
					((m[0 * 2] & 0x1fff) << 16) |
					(m[0 * 2 + 1] & 0x1fff));
				VSYNC_WR_MPEG_REG(VIU_OSD1_EOTF_COEF02_10,
					((m[1 * 2] & 0x1fff) << 16) |
					(m[1 * 2 + 1] & 0x1fff));
				VSYNC_WR_MPEG_REG(VIU_OSD1_EOTF_COEF11_12,
					((m[2 * 2] & 0x1fff) << 16) |
					(m[2 * 2 + 1] & 0x1fff));
				VSYNC_WR_MPEG_REG(VIU_OSD1_EOTF_COEF20_21,
					((m[3 * 2] & 0x1fff) << 16) |
					(m[3 * 2 + 1] & 0x1fff));
				VSYNC_WR_MPEG_REG(VIU_OSD1_EOTF_COEF22_RS,
					((m[4 * 2] & 0x1fff) << 16) |
					(m[4 * 2 + 1] & 0x1fff));
			}
			WRITE_VPP_REG_BITS(VIU_OSD1_EOTF_CTL,
				(on | (on << 1)), 30, 2);
		}
	} else {
		/* vd1 matrix, VPP_MATRIX_1 */
		/* post matrix, VPP_MATRIX_2 */
		/* xvycc matrix, VPP_MATRIX_3 */
		/* vd2 matrix, VPP_MATRIX_6 */
		if (m_select == VPP_MATRIX_POST) {
			/* post matrix */
			m = post_matrix_coeff;
			if (rdma_flag & (1 << m_select)) {
				/* set bit for disable latched */
				WRITE_VPP_REG_BITS(VPP_XVYCC_MISC, 0, 14, 1);
				if (on)
					mtx_en_mux |= POST_MTX_EN_MASK;
				else
					mtx_en_mux &= ~POST_MTX_EN_MASK;
			} else {
				WRITE_VPP_REG_BITS(VPP_XVYCC_MISC, 1, 14, 1);
				WRITE_VPP_REG_BITS(VPP_MATRIX_CTRL, on, 0, 1);
			}

			if (on) {
				if (rdma_flag & (1 << m_select)) {
					/*VSYNC_WR_MPEG_REG_BITS(*/
					/*VPP_MATRIX_CTRL, 0, 8, 3);*/
					VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL,
						(reg_value & (~(7 << 8)))
						| (0 << 8));
				} else
					WRITE_VPP_REG_BITS(
						VPP_MATRIX_CTRL, 0, 8, 3);
			}
		} else if (m_select == VPP_MATRIX_VD1) {
			/* vd1 matrix, latched */
			m = vd1_matrix_coeff;
			if (rdma_flag & (1 << m_select)) {
				/* set bit for disable latched */
				WRITE_VPP_REG_BITS(VPP_XVYCC_MISC, 0, 9, 1);
				/*WRITE_VPP_REG_BITS(VPP_MATRIX_CTRL,*/
				/*on, 5, 1);*/
				if (on)
					mtx_en_mux |= VD1_MTX_EN_MASK;
				else
					mtx_en_mux &= ~VD1_MTX_EN_MASK;
			} else {
				/* set bit for enable latched */
				WRITE_VPP_REG_BITS(VPP_XVYCC_MISC, 1, 9, 1);
				WRITE_VPP_REG_BITS(VPP_MATRIX_CTRL, on, 5, 1);
			}
			if (on) {
				if (rdma_flag & (1 << m_select)) {
					/*VSYNC_WR_MPEG_REG_BITS(*/
					/*VPP_MATRIX_CTRL, 1, 8, 3);*/
					VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL,
						(reg_value & (~(7 << 8)))
						| (1 << 8));
				} else
					WRITE_VPP_REG_BITS(
						VPP_MATRIX_CTRL, 1, 8, 3);
			}
		} else if (m_select == VPP_MATRIX_VD2) {
			/* vd2 matrix, not latched */
			m = vd2_matrix_coeff;
			if (rdma_flag & (1 << m_select)) {
				if (on)
					mtx_en_mux |= VD2_MTX_EN_MASK;
				else
					mtx_en_mux &= ~VD2_MTX_EN_MASK;
			} else {
				WRITE_VPP_REG_BITS(VPP_MATRIX_CTRL, on, 4, 1);
			}
			if (on) {
				if (rdma_flag & (1 << m_select)) {
					/*VSYNC_WR_MPEG_REG_BITS(*/
					/*VPP_MATRIX_CTRL, 2, 8, 3);*/
					VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL,
						(reg_value & (~(7 << 8)))
						| (2 << 8));
				} else
					WRITE_VPP_REG_BITS(
						VPP_MATRIX_CTRL, 2, 8, 3);
			}
		} else if (m_select == VPP_MATRIX_XVYCC) {
			/* xvycc matrix, not latched */
			m = xvycc_matrix_coeff;

			if (on) {
				if (rdma_flag & (1 << m_select)) {
					mtx_en_mux |= XVY_MTX_EN_MASK;
					reg_value = (reg_value & (~(7 << 8)))
							| (3 << 8);
					VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL,
						reg_value);
				} else {
					WRITE_VPP_REG_BITS(
						VPP_MATRIX_CTRL, on, 6, 1);
					WRITE_VPP_REG_BITS(
						VPP_MATRIX_CTRL, 3, 8, 3);
				}
			} else {
				if (rdma_flag & (1 << m_select)) {
					mtx_en_mux &= ~XVY_MTX_EN_MASK;
				} else
					WRITE_VPP_REG_BITS(
						VPP_MATRIX_CTRL, on, 6, 1);
			}
		}
		if (on) {
			if (rdma_flag & (1 << m_select)) {
				VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET0_1,
					((m[0] & 0xfff) << 16)
					| (m[1] & 0xfff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET2,
					m[2] & 0xfff);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01,
					((m[3] & 0x1fff) << 16)
					| (m[4] & 0x1fff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10,
					((m[5]  & 0x1fff) << 16)
					| (m[6] & 0x1fff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12,
					((m[7] & 0x1fff) << 16)
					| (m[8] & 0x1fff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21,
					((m[9] & 0x1fff) << 16)
					| (m[10] & 0x1fff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22,
					m[11] & 0x1fff);
				if (m[21]) {
					VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF13_14,
						((m[12] & 0x1fff) << 16)
						| (m[13] & 0x1fff));
					VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF15_25,
						((m[14] & 0x1fff) << 16)
						| (m[17] & 0x1fff));
					VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF23_24,
						((m[15] & 0x1fff) << 16)
						| (m[16] & 0x1fff));
				}
				if (is_meson_txhd_cpu() &&
					(m_select == VPP_MATRIX_XVYCC)) {
					VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1,
						(((m[18] >> 2) & 0xfff) << 16)
						| ((m[19] >> 2) & 0xfff));
					VSYNC_WR_MPEG_REG(
						VPP_MATRIX_OFFSET2,
						(m[20] >> 2) & 0xfff);
				} else {
					VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1,
						((m[18] & 0xfff) << 16)
						| (m[19] & 0xfff));
					VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2,
						m[20] & 0xfff);
				}
				VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP,
					m[21], 3, 2);
				VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP,
					m[22], 5, 3);
			} else {
				WRITE_VPP_REG(VPP_MATRIX_PRE_OFFSET0_1,
					((m[0] & 0xfff) << 16)
					| (m[1] & 0xfff));
				WRITE_VPP_REG(VPP_MATRIX_PRE_OFFSET2,
					m[2] & 0xfff);
				WRITE_VPP_REG(VPP_MATRIX_COEF00_01,
					((m[3] & 0x1fff) << 16)
					| (m[4] & 0x1fff));
				WRITE_VPP_REG(VPP_MATRIX_COEF02_10,
					((m[5]  & 0x1fff) << 16)
					| (m[6] & 0x1fff));
				WRITE_VPP_REG(VPP_MATRIX_COEF11_12,
					((m[7] & 0x1fff) << 16)
					| (m[8] & 0x1fff));
				WRITE_VPP_REG(VPP_MATRIX_COEF20_21,
					((m[9] & 0x1fff) << 16)
					| (m[10] & 0x1fff));
				WRITE_VPP_REG(VPP_MATRIX_COEF22,
					m[11] & 0x1fff);
				if (m[21]) {
					WRITE_VPP_REG(VPP_MATRIX_COEF13_14,
						((m[12] & 0x1fff) << 16)
						| (m[13] & 0x1fff));
					WRITE_VPP_REG(VPP_MATRIX_COEF15_25,
						((m[14] & 0x1fff) << 16)
						| (m[17] & 0x1fff));
					WRITE_VPP_REG(VPP_MATRIX_COEF23_24,
						((m[15] & 0x1fff) << 16)
						| (m[16] & 0x1fff));
				}
				if (is_meson_txhd_cpu()	&&
					(m_select == VPP_MATRIX_XVYCC)) {
					WRITE_VPP_REG(VPP_MATRIX_OFFSET0_1,
						(((m[18] >> 2) & 0xfff) << 16)
						| ((m[19] >> 2) & 0xfff));
					WRITE_VPP_REG(VPP_MATRIX_OFFSET2,
						(m[20] >> 2) & 0xfff);
				} else {
					WRITE_VPP_REG(VPP_MATRIX_OFFSET0_1,
						((m[18] & 0xfff) << 16)
						| (m[19] & 0xfff));
					WRITE_VPP_REG(VPP_MATRIX_OFFSET2,
						m[20] & 0xfff);
				}
				WRITE_VPP_REG_BITS(VPP_MATRIX_CLIP,
					m[21], 3, 2);
				WRITE_VPP_REG_BITS(VPP_MATRIX_CLIP,
					m[22], 5, 3);
			}
		}
	}
}

void enable_osd_path(int on, int shadow_mode)
{
	static int *osd1_mtx_backup;
	static uint32_t osd1_eotf_ctl_backup;
	static uint32_t osd1_oetf_ctl_backup;

	if (!on) {
		osd1_mtx_backup = cur_osd_mtx;
		osd1_eotf_ctl_backup = hdr_osd_reg.viu_osd1_eotf_ctl;
		osd1_oetf_ctl_backup = hdr_osd_reg.viu_osd1_oetf_ctl;

		set_vpp_matrix(VPP_MATRIX_OSD, bypass_coeff, CSC_ON);
		hdr_osd_reg.viu_osd1_eotf_ctl &= 0x07ffffff;
		hdr_osd_reg.viu_osd1_oetf_ctl &= 0x1fffffff;
		hdr_osd_reg.shadow_mode = shadow_mode;
	} else {
		set_vpp_matrix(VPP_MATRIX_OSD, osd1_mtx_backup, CSC_ON);
		hdr_osd_reg.viu_osd1_eotf_ctl = osd1_eotf_ctl_backup;
		hdr_osd_reg.viu_osd1_oetf_ctl = osd1_oetf_ctl_backup;
		hdr_osd_reg.shadow_mode = shadow_mode;
	}
}
EXPORT_SYMBOL(enable_osd_path);

/* coeff: pointer to target coeff array */
/* bits: how many bits for target coeff, could be 10 ~ 12, default 10 */
static int32_t *post_mtx_backup;
static int32_t post_on_backup;
static bool restore_post_table;
static int32_t *vd1_mtx_backup;
static int32_t vd1_on_backup;
int enable_rgb_to_yuv_matrix_for_dvll(
	int32_t on, uint32_t *coeff_orig, uint32_t bits)
{
	int32_t i;
	uint32_t coeff01, coeff23, coeff45, coeff67, coeff89;
	uint32_t scale, shift, offset[3];
	int32_t *coeff = dvll_RGB_to_YUV709l_coeff;

	if ((bits < 10) || (bits > 12))
		return -1;
	if (on && !coeff_orig)
		return -2;
	if (on) {
		/* only store the start one */
		if (cur_post_mtx !=
			dvll_RGB_to_YUV709l_coeff) {
			post_mtx_backup = cur_post_mtx;
			post_on_backup = cur_post_on;
			vd1_mtx_backup = cur_vd1_mtx;
			vd1_on_backup = cur_vd1_on;
		}
		coeff01 = coeff_orig[0];
		coeff23 = coeff_orig[1];
		coeff45 = coeff_orig[2];
		coeff67 = coeff_orig[3];
		coeff89 = coeff_orig[4] & 0xffff;
		scale = (coeff_orig[4] >> 16) & 0x0f;
		offset[0] = coeff_orig[5];
		offset[1] = 0; /* coeff_orig[6]; */
		offset[2] = 0; /* coeff_orig[7]; */

		coeff[0] = coeff[1] = coeff[2] = 0; /* pre offset */

		coeff[5] = (int32_t)((coeff01 & 0xffff) << 16) >> 16;
		coeff[3] = (int32_t)coeff01 >> 16;
		coeff[4] = (int32_t)((coeff23 & 0xffff) << 16) >> 16;
		coeff[8] = (int32_t)coeff23 >> 16;
		coeff[6] = (int32_t)((coeff45 & 0xffff) << 16) >> 16;
		coeff[7] = (int32_t)coeff45 >> 16;
		coeff[11] = (int32_t)((coeff67 & 0xffff) << 16) >> 16;
		coeff[9] = (int32_t)coeff67 >> 16;
		coeff[10] = (int32_t)((coeff89 & 0xffff) << 16) >> 16;

		if (scale > 12) {
			shift = scale - 12;
			for (i = 3; i < 12; i++)
				coeff[i] =
					(coeff[i] + (1 << (shift - 1)))
					>> shift;
		} else if (scale < 12) {
			shift = 12 - scale;
			for (i = 3; i < 12; i++)
				coeff[i] <<= shift;
		}

		/* post offset */
		coeff[18] = offset[0];
		coeff[19] = offset[1];
		coeff[20] = offset[2];

		coeff[5] =
			((coeff[3] + coeff[4] + coeff[5]) & 0xfffe)
			- coeff[3] - coeff[4];
		coeff[8] = 0 - coeff[6] - coeff[7];
		coeff[11] = 0 - coeff[9] - coeff[10];
		coeff[18] -= (0x1000 - coeff[3] - coeff[4] - coeff[5]) >> 1;

		coeff[22] = 2;
		vpp_set_mtx_en_read();
		VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL, 0);
		set_vpp_matrix(VPP_MATRIX_OSD,
			cur_osd_mtx, CSC_OFF);
		set_vpp_matrix(VPP_MATRIX_VD1,
			cur_vd1_mtx, CSC_OFF);
		set_vpp_matrix(VPP_MATRIX_POST,
			coeff, CSC_ON);
		vpp_set_mtx_en_write();
		restore_post_table = true;
	} else if (restore_post_table) {
		vpp_set_mtx_en_read();
		VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL, 0);
		set_vpp_matrix(VPP_MATRIX_OSD,
			cur_osd_mtx, CSC_OFF);
		set_vpp_matrix(VPP_MATRIX_VD1,
			vd1_mtx_backup, cur_vd1_on);
		set_vpp_matrix(VPP_MATRIX_POST,
			post_mtx_backup, post_on_backup);
		vpp_set_mtx_en_write();
		restore_post_table = false;
	}
	return 0;
}
EXPORT_SYMBOL(enable_rgb_to_yuv_matrix_for_dvll);

const char lut_name[NUM_LUT][16] = {
	"OSD_EOTF",
	"OSD_OETF",
	"EOTF",
	"OETF",
	"INV_EOTF"
};
/* VIDEO_OETF_LUT_SIZE 289 >*/
/*	OSD_OETF_LUT_SIZE 41 >*/
/*	OSD_OETF_LUT_SIZE 33 */
static void print_vpp_lut(
	enum vpp_lut_sel_e lut_sel,
	int on)
{
	unsigned short *r_map;
	unsigned short *g_map;
	unsigned short *b_map;
	unsigned int addr_port;
	unsigned int data_port;
	unsigned int ctrl_port;
	unsigned int data;
	int i;

	r_map = kmalloc(sizeof(unsigned short) * VIDEO_OETF_LUT_SIZE * 3,
			GFP_ATOMIC);
	if (!r_map)
		return;
	g_map = &r_map[sizeof(unsigned short) * VIDEO_OETF_LUT_SIZE * 1];
	b_map = &r_map[sizeof(unsigned short) * VIDEO_OETF_LUT_SIZE * 2];

	if (lut_sel == VPP_LUT_OSD_EOTF) {
		addr_port = VIU_OSD1_EOTF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_EOTF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_EOTF_CTL;
	} else if (lut_sel == VPP_LUT_EOTF) {
		addr_port = VIU_EOTF_LUT_ADDR_PORT;
		data_port = VIU_EOTF_LUT_DATA_PORT;
		ctrl_port = VIU_EOTF_CTL;
	} else if (lut_sel == VPP_LUT_OSD_OETF) {
		addr_port = VIU_OSD1_OETF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_OETF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_OETF_CTL;
	} else if (lut_sel == VPP_LUT_OETF) {
		addr_port = XVYCC_LUT_R_ADDR_PORT;
		data_port = XVYCC_LUT_R_DATA_PORT;
		ctrl_port = XVYCC_LUT_CTL;
	} else if (lut_sel == VPP_LUT_INV_EOTF) {
		addr_port = XVYCC_INV_LUT_Y_ADDR_PORT;
		data_port = XVYCC_INV_LUT_Y_DATA_PORT;
		ctrl_port = XVYCC_INV_LUT_CTL;
	} else {
		kfree(r_map);
		return;
	}
	if (lut_sel == VPP_LUT_OSD_OETF) {
		for (i = 0; i < 20; i++) {
			WRITE_VPP_REG(addr_port, i);
			data = READ_VPP_REG(data_port);
			r_map[i * 2] = data & 0xffff;
			r_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		WRITE_VPP_REG(addr_port, 20);
		data = READ_VPP_REG(data_port);
		r_map[OSD_OETF_LUT_SIZE - 1] = data & 0xffff;
		g_map[0] = (data >> 16) & 0xffff;
		for (i = 0; i < 20; i++) {
			WRITE_VPP_REG(addr_port, 21 + i);
			data = READ_VPP_REG(data_port);
			g_map[i * 2 + 1] = data & 0xffff;
			g_map[i * 2 + 2] = (data >> 16) & 0xffff;
		}
		for (i = 0; i < 20; i++) {
			WRITE_VPP_REG(addr_port, 41 + i);
			data = READ_VPP_REG(data_port);
			b_map[i * 2] = data & 0xffff;
			b_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		WRITE_VPP_REG(addr_port, 61);
		data = READ_VPP_REG(data_port);
		b_map[OSD_OETF_LUT_SIZE - 1] = data & 0xffff;
		pr_csc(1, "%s lut %s:\n", lut_name[lut_sel], on ? "on" : "off");
		for (i = 0; i < OSD_OETF_LUT_SIZE; i++) {
			pr_csc(1, "\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i, r_map[i], g_map[i], b_map[i]);
		}
		pr_csc(1, "\n");
	} else if (lut_sel == VPP_LUT_OSD_EOTF) {
		WRITE_VPP_REG(addr_port, 0);
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			r_map[i * 2] = data & 0xffff;
			r_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		data = READ_VPP_REG(data_port);
		r_map[EOTF_LUT_SIZE - 1] = data & 0xffff;
		g_map[0] = (data >> 16) & 0xffff;
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			g_map[i * 2 + 1] = data & 0xffff;
			g_map[i * 2 + 2] = (data >> 16) & 0xffff;
		}
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			b_map[i * 2] = data & 0xffff;
			b_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		data = READ_VPP_REG(data_port);
		b_map[EOTF_LUT_SIZE - 1] = data & 0xffff;
		pr_csc(1, "%s lut %s:\n", lut_name[lut_sel], on ? "on" : "off");
		for (i = 0; i < EOTF_LUT_SIZE; i++) {
			pr_csc(1, "\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i, r_map[i], g_map[i], b_map[i]);
		}
		pr_csc(1, "\n");
	} else if (lut_sel == VPP_LUT_EOTF) {
		WRITE_VPP_REG(addr_port, 0);
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			r_map[i * 2] = data & 0xffff;
			r_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		data = READ_VPP_REG(data_port);
		r_map[EOTF_LUT_SIZE - 1] = data & 0xffff;
		g_map[0] = (data >> 16) & 0xffff;
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			g_map[i * 2 + 1] = data & 0xffff;
			g_map[i * 2 + 2] = (data >> 16) & 0xffff;
		}
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			b_map[i * 2] = data & 0xffff;
			b_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		data = READ_VPP_REG(data_port);
		b_map[EOTF_LUT_SIZE - 1] = data & 0xffff;
		pr_csc(1, "%s lut %s:\n", lut_name[lut_sel], on ? "on" : "off");
		for (i = 0; i < EOTF_LUT_SIZE; i++) {
			pr_csc(1, "\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i, r_map[i], g_map[i], b_map[i]);
		}
		pr_csc(1, "\n");
	} else if (lut_sel == VPP_LUT_OETF) {
		WRITE_VPP_REG(ctrl_port, 0x0);
		WRITE_VPP_REG(addr_port, 0);
		for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++)
			r_map[i] = READ_VPP_REG(data_port) & 0x3ff;
		WRITE_VPP_REG(addr_port + 2, 0);
		for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++)
			g_map[i] = READ_VPP_REG(data_port + 2) & 0x3ff;
		WRITE_VPP_REG(addr_port + 4, 0);
		for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++)
			b_map[i] = READ_VPP_REG(data_port + 4) & 0x3ff;
		pr_csc(1, "%s lut %s:\n", lut_name[lut_sel], on ? "on" : "off");
		for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++) {
			pr_csc(1, "\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i, r_map[i], g_map[i], b_map[i]);
		}
		pr_csc(1, "\n");
		if (on)
			WRITE_VPP_REG(ctrl_port, 0x7f);
	} else if (lut_sel == VPP_LUT_INV_EOTF) {
		WRITE_VPP_REG_BITS(ctrl_port, 0, 12, 3);
		pr_csc(1, "%s lut %s:\n", lut_name[lut_sel], on ? "on" : "off");
		for (i = 0;
			i < EOTF_INV_LUT_NEG2048_SIZE +
			EOTF_INV_LUT_SIZE + EOTF_INV_LUT_1024_SIZE;
			i++) {
			WRITE_VPP_REG(addr_port, i);
			data = READ_VPP_REG(data_port) & 0xfff;
			if (data & 0x800)
				pr_csc(1, "\t[%d] = %d\n",
					i, -(~(data|0xfffff000) + 1));
			else
				pr_csc(1, "\t[%d] = %d\n", i, data);
		}
		pr_csc(1, "\n");
		if (on)
			WRITE_VPP_REG_BITS(ctrl_port, 1<<2, 12, 3);
	}
	kfree(r_map);
}

void set_vpp_lut(
	enum vpp_lut_sel_e lut_sel,
	unsigned int *r,
	unsigned int *g,
	unsigned int *b,
	int on)
{
	unsigned int *r_map = NULL;
	unsigned int *g_map = NULL;
	unsigned int *b_map = NULL;
	unsigned int addr_port;
	unsigned int data_port;
	unsigned int ctrl_port;
	int i;

	if (reload_lut & (1 << lut_sel))
		reload_lut &= ~(1 << lut_sel);
	if (lut_sel == VPP_LUT_OSD_EOTF) {
		r_map = osd_eotf_r_mapping;
		g_map = osd_eotf_g_mapping;
		b_map = osd_eotf_b_mapping;
		addr_port = VIU_OSD1_EOTF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_EOTF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_EOTF_CTL;
	} else if (lut_sel == VPP_LUT_EOTF) {
		r_map = video_eotf_r_mapping;
		g_map = video_eotf_g_mapping;
		b_map = video_eotf_b_mapping;
		addr_port = VIU_EOTF_LUT_ADDR_PORT;
		data_port = VIU_EOTF_LUT_DATA_PORT;
		ctrl_port = VIU_EOTF_CTL;
	} else if (lut_sel == VPP_LUT_OSD_OETF) {
		r_map = osd_oetf_r_mapping;
		g_map = osd_oetf_g_mapping;
		b_map = osd_oetf_b_mapping;
		addr_port = VIU_OSD1_OETF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_OETF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_OETF_CTL;
	} else if (lut_sel == VPP_LUT_OETF) {
#if 0
		load_knee_lut(on);
		return;
#else
		r_map = video_oetf_r_mapping;
		g_map = video_oetf_g_mapping;
		b_map = video_oetf_b_mapping;
		addr_port = XVYCC_LUT_R_ADDR_PORT;
		data_port = XVYCC_LUT_R_DATA_PORT;
		ctrl_port = XVYCC_LUT_CTL;
#endif
	} else if (lut_sel == VPP_LUT_INV_EOTF) {
		addr_port = XVYCC_INV_LUT_Y_ADDR_PORT;
		data_port = XVYCC_INV_LUT_Y_DATA_PORT;
		ctrl_port = XVYCC_INV_LUT_CTL;
	} else
		return;

	if (lut_sel == VPP_LUT_OSD_OETF) {
		/* enable latched */
		if (r && r_map)
			for (i = 0; i < OSD_OETF_LUT_SIZE; i++)
				r_map[i] = r[i];
		if (g && g_map)
			for (i = 0; i < OSD_OETF_LUT_SIZE; i++)
				g_map[i] = g[i];
		if (r && r_map)
			for (i = 0; i < OSD_OETF_LUT_SIZE; i++)
				b_map[i] = b[i];

		if (!is_meson_txlx_cpu()) {
			for (i = 0; i < OSD_OETF_LUT_SIZE; i++) {
				hdr_osd_reg.lut_val.or_map[i] = r_map[i];
				hdr_osd_reg.lut_val.og_map[i] = g_map[i];
				hdr_osd_reg.lut_val.ob_map[i] = b_map[i];
			}
			hdr_osd_reg.viu_osd1_oetf_ctl &= 0x1fffffff;
			hdr_osd_reg.viu_osd1_oetf_ctl |= 7 << 22;
			if (on)
				hdr_osd_reg.viu_osd1_oetf_ctl |= 7 << 29;
		} else {
			/* latch enable */
			WRITE_VPP_REG_BITS(VIU_OSD1_OETF_CTL, 1, 28, 1);
			if (on) {
				/* change to 12bit from txlx */
				if (is_meson_txlx_cpu())
					for (i = 0;
						i < OSD_OETF_LUT_SIZE;
						i++) {
						r_map[i] = 4 * r_map[i];
						g_map[i] = 4 * g_map[i];
						b_map[i] = 4 * b_map[i];
					}
				for (i = 0; i < 20; i++) {
					VSYNC_WR_MPEG_REG(addr_port, i);
					VSYNC_WR_MPEG_REG(data_port,
						r_map[i * 2]
						| (r_map[i * 2 + 1] << 16));
				}
				VSYNC_WR_MPEG_REG(addr_port, 20);
				VSYNC_WR_MPEG_REG(data_port,
					r_map[41 - 1]
					| (g_map[0] << 16));
				for (i = 0; i < 20; i++) {
					VSYNC_WR_MPEG_REG(addr_port, 21 + i);
					VSYNC_WR_MPEG_REG(data_port,
						g_map[i * 2 + 1]
						| (g_map[i * 2 + 2] << 16));
				}
				for (i = 0; i < 20; i++) {
					VSYNC_WR_MPEG_REG(addr_port, 41 + i);
					VSYNC_WR_MPEG_REG(data_port,
						b_map[i * 2]
						| (b_map[i * 2 + 1] << 16));
				}
				VSYNC_WR_MPEG_REG(addr_port, 61);
				VSYNC_WR_MPEG_REG(data_port,
					b_map[41 - 1]);
			}

			WRITE_VPP_REG_BITS(VIU_OSD1_OETF_CTL, 7, 22, 3);
			WRITE_VPP_REG_BITS(VIU_OSD1_OETF_CTL,
				(on | (on << 1) | (on << 2)), 29, 3);
		}
	} else if (lut_sel == VPP_LUT_OSD_EOTF) {
		/* enable latched */
		if (r && r_map)
			for (i = 0; i < EOTF_LUT_SIZE; i++)
				r_map[i] = r[i];
		if (g && g_map)
			for (i = 0; i < EOTF_LUT_SIZE; i++)
				g_map[i] = g[i];
		if (r && r_map)
			for (i = 0; i < EOTF_LUT_SIZE; i++)
				b_map[i] = b[i];

		if (!is_meson_txlx_cpu() &&
			!is_meson_gxlx_cpu() &&
			!is_meson_txhd_cpu()) {
			for (i = 0; i < EOTF_LUT_SIZE; i++) {
				hdr_osd_reg.lut_val.r_map[i] = r_map[i];
				hdr_osd_reg.lut_val.g_map[i] = g_map[i];
				hdr_osd_reg.lut_val.b_map[i] = b_map[i];
			}
			hdr_osd_reg.viu_osd1_eotf_ctl &= 0xc7ffffff;
			if (on)
				hdr_osd_reg.viu_osd1_eotf_ctl |= 7 << 27;
			hdr_osd_reg.viu_osd1_eotf_ctl |= 1 << 31;
		} else {
			/* latch enable */
			WRITE_VPP_REG_BITS(VIU_OSD1_EOTF_CTL, 1, 26, 1);
			if (on) {
				VSYNC_WR_MPEG_REG(
					addr_port, 0);
				for (i = 0; i < 16; i++)
					VSYNC_WR_MPEG_REG(
						data_port,
						r_map[i * 2]
						| (r_map[i * 2 + 1] << 16));
				VSYNC_WR_MPEG_REG(
					data_port,
					r_map[EOTF_LUT_SIZE - 1]
					| (g_map[0] << 16));
				for (i = 0; i < 16; i++)
					VSYNC_WR_MPEG_REG(
						data_port,
						g_map[i * 2 + 1]
						| (b_map[i * 2 + 2] << 16));
				for (i = 0; i < 16; i++)
					VSYNC_WR_MPEG_REG(
						data_port,
						b_map[i * 2]
						| (b_map[i * 2 + 1] << 16));
				VSYNC_WR_MPEG_REG(
					data_port, b_map[EOTF_LUT_SIZE - 1]);
			}

			WRITE_VPP_REG_BITS(VIU_OSD1_EOTF_CTL,
				1, 31, 1);
			WRITE_VPP_REG_BITS(VIU_OSD1_EOTF_CTL,
				(on | (on << 1) | (on << 2)), 27, 3);
		}
	} else if (lut_sel == VPP_LUT_EOTF) {
		/* enable latched */
		if (r && r_map)
			for (i = 0; i < EOTF_LUT_SIZE; i++)
				r_map[i] = r[i];
		if (g && g_map)
			for (i = 0; i < EOTF_LUT_SIZE; i++)
				g_map[i] = g[i];
		if (r && r_map)
			for (i = 0; i < EOTF_LUT_SIZE; i++)
				b_map[i] = b[i];
		/*txlx add eotf latch ctl bit 26*/
		if (is_meson_txlx_cpu() ||
			is_meson_gxlx_cpu() ||
			is_meson_txhd_cpu())
			WRITE_VPP_REG_BITS(ctrl_port, 1, 26, 1);

		if (on) {
			for (i = 0; i < 16; i++) {
				VSYNC_WR_MPEG_REG(addr_port, i);
				VSYNC_WR_MPEG_REG(data_port,
					r_map[i * 2]
					| (r_map[i * 2 + 1] << 16));
			}
			VSYNC_WR_MPEG_REG(addr_port, 16);
			VSYNC_WR_MPEG_REG(data_port,
				r_map[EOTF_LUT_SIZE - 1]
				| (g_map[0] << 16));
			for (i = 0; i < 16; i++) {
				VSYNC_WR_MPEG_REG(addr_port, i + 17);
				VSYNC_WR_MPEG_REG(data_port,
					g_map[i * 2 + 1]
					| (g_map[i * 2 + 2] << 16));
			}
			for (i = 0; i < 16; i++) {
				VSYNC_WR_MPEG_REG(addr_port, i + 33);
				VSYNC_WR_MPEG_REG(data_port,
					b_map[i * 2]
					| (b_map[i * 2 + 1] << 16));
			}
			VSYNC_WR_MPEG_REG(addr_port, 49);
			VSYNC_WR_MPEG_REG(data_port, b_map[EOTF_LUT_SIZE - 1]);
			WRITE_VPP_REG_BITS(ctrl_port, 7, 27, 3);
		} else
			WRITE_VPP_REG_BITS(ctrl_port, 0, 27, 3);
		WRITE_VPP_REG_BITS(ctrl_port, 1, 31, 1);
	} else if (lut_sel == VPP_LUT_OETF) {
		/* set bit to disable latched */
		WRITE_VPP_REG_BITS(VPP_XVYCC_MISC, 0, 18, 3);
		if (r && r_map)
			for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++)
				r_map[i] = r[i];
		if (g && g_map)
			for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++)
				g_map[i] = g[i];
		if (r && r_map)
			for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++)
				b_map[i] = b[i];
		if (on) {
			VSYNC_WR_MPEG_REG(ctrl_port, 0x0f);
			for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++) {
				VSYNC_WR_MPEG_REG(addr_port, i);
				VSYNC_WR_MPEG_REG(data_port, r_map[i]);
			}
			for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++) {
				VSYNC_WR_MPEG_REG(addr_port + 2, i);
				VSYNC_WR_MPEG_REG(data_port + 2, g_map[i]);
			}
			for (i = 0; i < VIDEO_OETF_LUT_SIZE; i++) {
				VSYNC_WR_MPEG_REG(addr_port + 4, i);
				VSYNC_WR_MPEG_REG(data_port + 4, b_map[i]);
			}
			VSYNC_WR_MPEG_REG(ctrl_port, 0x7f);
			knee_lut_on = 1;
		} else {
			VSYNC_WR_MPEG_REG(ctrl_port, 0x0f);
			knee_lut_on = 0;
		}
		cur_knee_factor = knee_factor;
	} else if (lut_sel == VPP_LUT_INV_EOTF) {
		/* set bit to enable latched */
		WRITE_VPP_REG_BITS(VPP_XVYCC_MISC, 0x7, 4, 3);
		if (on) {
			if (r[0] & INVLUT_HLG) {
				for (i = 0; i < EOTF_INV_LUT_SIZE; i++)
					invlut_y[i] = invlut_hlg_y[i];
				r[0] = 0;
			}
			VSYNC_WR_MPEG_REG(addr_port, 0);
			for (i = 0; i < EOTF_INV_LUT_NEG2048_SIZE; i++) {
				VSYNC_WR_MPEG_REG(addr_port, i);
				VSYNC_WR_MPEG_REG(data_port, invlut_y_neg[i]);
			}
			for (i = 0; i < EOTF_INV_LUT_SIZE; i++) {
				VSYNC_WR_MPEG_REG(addr_port,
					EOTF_INV_LUT_NEG2048_SIZE + i);
				VSYNC_WR_MPEG_REG(data_port, invlut_y[i]);
			}
			for (i = 0; i < EOTF_INV_LUT_1024_SIZE; i++) {
				VSYNC_WR_MPEG_REG(addr_port,
					EOTF_INV_LUT_NEG2048_SIZE +
					EOTF_INV_LUT_SIZE + i);
				VSYNC_WR_MPEG_REG(data_port, invlut_y_1024[i]);
			}
			WRITE_VPP_REG_BITS(ctrl_port, 1<<2, 12, 3);
		} else
			WRITE_VPP_REG_BITS(ctrl_port, 0, 12, 3);
	}
	if (debug_csc && print_lut_mtx)
		print_vpp_lut(lut_sel, on);
}

/***************************** end of gxl hdr **************************/

/* extern unsigned int cm_size; */
/* extern unsigned int ve_size; */
/* extern unsigned int cm2_patch_flag; */
/* extern struct ve_dnlp_s am_ve_dnlp; */
/* extern struct ve_dnlp_table_s am_ve_new_dnlp; */
/* extern int cm_en; //0:disabel;1:enable */
/* extern struct tcon_gamma_table_s video_gamma_table_r; */
/* extern struct tcon_gamma_table_s video_gamma_table_g; */
/* extern struct tcon_gamma_table_s video_gamma_table_b; */
/* extern struct tcon_gamma_table_s video_gamma_table_r_adj; */
/* extern struct tcon_gamma_table_s video_gamma_table_g_adj; */
/* extern struct tcon_gamma_table_s video_gamma_table_b_adj; */
/* extern struct tcon_rgb_ogo_s     video_rgb_ogo; */

/* csc mode:*/
/*	0: 601 limit to RGB*/
/*		vd1 for ycbcr to rgb*/
/*	1: 601 full to RGB*/
/*		vd1 for ycbcr to rgb*/
/*	2: 709 limit to RGB*/
/*		vd1 for ycbcr to rgb*/
/*	3: 709 full to RGB*/
/*		vd1 for ycbcr to rgb*/
/*	4: 2020 limit to RGB*/
/*		vd1 for ycbcr to rgb*/
/*		post for rgb to r'g'b'*/
/*	5: 2020(G:33c2,86c4 B:1d4c,0bb8 R:84d0,3e80) limit to RGB*/
/*		vd1 for ycbcr to rgb*/
/*		post for rgb to r'g'b'*/
/*	6: customer matrix calculation according to src and dest primary*/
/*		vd1 for ycbcr to rgb*/
/*		post for rgb to r'g'b' */
static void vpp_set_mtx_en_write(void)
{
	int reg_val;

	reg_val = READ_VPP_REG(VPP_MATRIX_CTRL);
	VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL, (reg_val &
			(~(POST_MTX_EN_MASK |
			VD2_MTX_EN_MASK |
			VD1_MTX_EN_MASK |
			XVY_MTX_EN_MASK |
			OSD1_MTX_EN_MASK))) |
			mtx_en_mux);
}

static void vpp_set_mtx_en_read(void)
{
	int reg_value;

	reg_value = READ_VPP_REG(VPP_MATRIX_CTRL);
	mtx_en_mux = reg_value &
				(POST_MTX_EN_MASK |
				VD2_MTX_EN_MASK |
				VD1_MTX_EN_MASK |
				XVY_MTX_EN_MASK |
				OSD1_MTX_EN_MASK);
}

static void vpp_set_matrix(
		enum vpp_matrix_sel_e vd1_or_vd2_or_post,
		unsigned int on,
		enum vpp_matrix_csc_e csc_mode,
		struct matrix_s *m)
{
	int reg_value;

	if (force_csc_type != 0xff)
		csc_mode = force_csc_type;

	reg_value = READ_VPP_REG(VPP_MATRIX_CTRL);

	if (vd1_or_vd2_or_post == VPP_MATRIX_VD1) {
		/* vd1 matrix */
		reg_value = (reg_value & (~(7 << 8))) |
			(1 << 8);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL, reg_value);

		if (on)
			mtx_en_mux |= VD1_MTX_EN_MASK;
		else
			mtx_en_mux &= ~VD1_MTX_EN_MASK;
	} else if (vd1_or_vd2_or_post == VPP_MATRIX_VD2) {
		/* vd2 matrix */
		reg_value = (reg_value & (~(7 << 8))) |
			(2 << 8);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL, reg_value);

		if (on)
			mtx_en_mux |= VD2_MTX_EN_MASK;
		else
			mtx_en_mux &= ~VD2_MTX_EN_MASK;
	} else {
		/* post matrix */
		reg_value = (reg_value & (~(7 << 8))) |
			(0 << 8);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL, reg_value);

		if (on)
			mtx_en_mux |= POST_MTX_EN_MASK;
		else
			mtx_en_mux &= ~POST_MTX_EN_MASK;
	}
	if (!on)
		return;

	if (csc_mode == VPP_MATRIX_YUV601_RGB) {
		/* ycbcr limit, 601 to RGB */
		/*  -16  1.164   0       1.596   0*/
		/*    -128 1.164   -0.392  -0.813  0*/
		/*    -128 1.164   2.017   0       0 */
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0x04A80000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0x066204A8);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x1e701cbf);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x04A80812);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x00000000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x00000000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x00000000);
		VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP, 0, 5, 3);
	} else if (csc_mode == VPP_MATRIX_YUV601F_RGB) {
		/* ycbcr full range, 601F to RGB */
		/*  0    1    0           1.402    0*/
		/*   -128  1   -0.34414    -0.71414  0*/
		/*   -128  1    1.772       0        0 */
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, (0x400 << 16) | 0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, (0x59c << 16) | 0x400);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12,
			(0x1ea0 << 16) | 0x1d24);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, (0x400 << 16) | 0x718);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x0);
		VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP, 0, 5, 3);
	} else if (csc_mode == VPP_MATRIX_YUV709_RGB) {
		/* ycbcr limit range, 709 to RGB */
		/* -16      1.164  0      1.793  0 */
		/* -128     1.164 -0.213 -0.534  0 */
		/* -128     1.164  2.115  0      0 */
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0x04A80000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0x072C04A8);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x1F261DDD);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x04A80876);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x0);

		VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP, 0, 5, 3);
	} else if (csc_mode == VPP_MATRIX_YUV709F_RGB) {
		/* ycbcr full range, 709F to RGB */
		/*  0    1      0       1.575   0*/
		/*  -128  1     -0.187  -0.468   0*/
		/*  -128  1      1.856   0       0 */
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0x04000000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0x064D0400);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x1F411E21);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x0400076D);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x0);
		VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP, 0, 5, 3);
	} else if (csc_mode == VPP_MATRIX_NULL) {
		/* bypass matrix */
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0x04000000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x04000000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x00000000);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x00000400);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x0);
		VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP, 0, 5, 3);
	} else if (csc_mode >= VPP_MATRIX_BT2020YUV_BT2020RGB) {
		if (vd1_or_vd2_or_post == VPP_MATRIX_VD1) {
			/* bt2020 limit to bt2020 RGB  */
			VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0x4ad0000);
			VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0x6e50492);
			VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x1f3f1d63);
			VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x492089a);
			VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x0);
			VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x0);
			VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x0);
			VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP, 0, 5, 3);
		}
		if (vd1_or_vd2_or_post == VPP_MATRIX_POST) {
			if (csc_mode == VPP_MATRIX_BT2020YUV_BT2020RGB) {
				/* 2020 RGB to R'G'B */
				VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET0_1,
					0x0);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET2, 0x0);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01,
					0xd491b4d);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10,
					0x1f6b1f01);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12,
					0x9101fef);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21,
					0x1fdb1f32);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x108f3);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x0);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x0);
				VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP,
					1, 5, 3);
#if 0 /* disable this case after calculate mtx on the fly*/
			} else if (csc_mode == VPP_MATRIX_BT2020RGB_709RGB) {
				/*to R'G'B' */
				WRITE_VPP_REG(VPP_MATRIX_PRE_OFFSET0_1, 0x0);
				WRITE_VPP_REG(VPP_MATRIX_PRE_OFFSET2, 0x0);
				/* from Jason */
				WRITE_VPP_REG(VPP_MATRIX_COEF00_01, 0x9cd1e33);
				WRITE_VPP_REG(VPP_MATRIX_COEF02_10, 0x00001faa);
				WRITE_VPP_REG(VPP_MATRIX_COEF11_12, 0x8560000);
				WRITE_VPP_REG(VPP_MATRIX_COEF20_21, 0x1fd81f5f);
				WRITE_VPP_REG(VPP_MATRIX_COEF22, 0x108c9);
				WRITE_VPP_REG(VPP_MATRIX_OFFSET0_1, 0x0);
				WRITE_VPP_REG(VPP_MATRIX_OFFSET2, 0x0);
				WRITE_VPP_REG_BITS(VPP_MATRIX_CLIP, 1, 5, 3);
#endif
			} else if (csc_mode == VPP_MATRIX_BT2020RGB_CUSRGB) {
				/* customer matrix 2020 RGB to R'G'B' */
				VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET0_1,
					(m->pre_offset[0] << 16)
					| (m->pre_offset[1] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET2,
					m->pre_offset[2] & 0xffff);
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01,
					(m->matrix[0][0] << 16)
					| (m->matrix[0][1] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10,
					(m->matrix[0][2] << 16)
					| (m->matrix[1][0] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12,
					(m->matrix[1][1] << 16)
					| (m->matrix[1][2] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21,
					(m->matrix[2][0] << 16)
					| (m->matrix[2][1] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22,
					(m->right_shift << 16)
					| (m->matrix[2][2] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1,
					(m->offset[0] << 16)
					| (m->offset[1] & 0xffff));
				VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2,
					m->offset[2] & 0xffff);
				VSYNC_WR_MPEG_REG_BITS(VPP_MATRIX_CLIP,
					m->right_shift, 5, 3);
			}
		}
	}
}

/* matrix betreen xvycclut and linebuffer*/
static uint cur_csc_mode = 0xff;
static void vpp_set_matrix3(
		unsigned int on,
		enum vpp_matrix_csc_e csc_mode)
{
	int reg_value;

	reg_value = READ_VPP_REG(VPP_MATRIX_CTRL);

	if (on)
		mtx_en_mux |= XVY_MTX_EN_MASK;
	else
		mtx_en_mux &= ~XVY_MTX_EN_MASK;

	if (!on)
		return;

	if (cur_csc_mode == csc_mode)
		return;

	reg_value = READ_VPP_REG(VPP_MATRIX_CTRL);

	VSYNC_WR_MPEG_REG(VPP_MATRIX_CTRL,
		(reg_value & (~(7 << 8))) | (3 << 8));

	if (csc_mode == VPP_MATRIX_RGB_YUV709F) {
		/* RGB -> 709F*/
		/*WRITE_VPP_REG(VPP_MATRIX_CTRL, 0x7360);*/

		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0xda02dc);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0x4a1f8a);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x1e760200);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x2001e2f);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x1fd1);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x200);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x200);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET0_1, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET2, 0x0);
	} else if (csc_mode == VPP_MATRIX_RGB_YUV709) {
		/* RGB -> 709 limit */
		/*WRITE_VPP_REG(VPP_MATRIX_CTRL, 0x7360);*/

		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF00_01, 0x00bb0275);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF02_10, 0x003f1f99);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF11_12, 0x1ea601c2);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF20_21, 0x01c21e67);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_COEF22, 0x00001fd7);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET0_1, 0x00400200);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_OFFSET2, 0x00000200);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET0_1, 0x0);
		VSYNC_WR_MPEG_REG(VPP_MATRIX_PRE_OFFSET2, 0x0);
	}
	cur_csc_mode = csc_mode;
}

static uint cur_signal_type[VD_PATH_MAX] = {
	0xffffffff,
	0xffffffff
};
module_param_array(cur_signal_type, uint, &vd_path_max, 0664);
MODULE_PARM_DESC(cur_signal_type, "\n cur_signal_type\n");

static struct vframe_master_display_colour_s
cur_master_display_colour[VD_PATH_MAX] = {
	{
		0,
		{
			{0, 0},
			{0, 0},
			{0, 0},
		},
		{0, 0},
		{0, 0}
	},
	{
		0,
		{
			{0, 0},
			{0, 0},
			{0, 0},
		},
		{0, 0},
		{0, 0}
	}
};

unsigned int pre_vd1_mtx_sel[VD_PATH_MAX] = {
	VPP_MATRIX_NULL,
	VPP_MATRIX_NULL,
};
module_param_array(pre_vd1_mtx_sel, uint, &vd_path_max, 0664);
MODULE_PARM_DESC(pre_vd1_mtx_sel, "\n pre_vd1_mtx_sel\n");
unsigned int vd1_mtx_sel = VPP_MATRIX_NULL;
static int src_timing_outputmode_changed(struct vframe_s *vf,
	struct vinfo_s *vinfo, enum vd_path_e vd_path)
{
	unsigned int width, height;

	if (vinfo->viu_color_fmt == COLOR_FMT_RGB444)
		return 0;

	width = (vf->type & VIDTYPE_COMPRESS) ?
		vf->compWidth : vf->width;
	height = (vf->type & VIDTYPE_COMPRESS) ?
		vf->compHeight : vf->height;

	if ((height < 720) && (vinfo->height < 720))
		vd1_mtx_sel = VPP_MATRIX_NULL;
	else if ((height < 720) && (vinfo->height >= 720))
		vd1_mtx_sel = VPP_MATRIX_YUV601L_YUV709L;
	else if ((height >= 720) && (vinfo->height < 720))
		vd1_mtx_sel = VPP_MATRIX_YUV709L_YUV601L;
	else if ((height >= 720) && (vinfo->height >= 720))
		vd1_mtx_sel = VPP_MATRIX_NULL;
	else
		vd1_mtx_sel = VPP_MATRIX_NULL;

	if (pre_vd1_mtx_sel[vd_path] != vd1_mtx_sel) {
		pre_vd1_mtx_sel[vd_path] = vd1_mtx_sel;
		return 1;
	}

	return 0;
}

static void print_primaries_info(struct vframe_master_display_colour_s *p)
{
	int i, j;

	if (!p->present_flag & 1) {
		pr_csc(1, "\tmaster display color not available");
		return;
	}
	if (p->present_flag & 0x20)
		pr_csc(1, "\tcolor primaries in GBR\n");
	else if (p->present_flag & 0x10)
		pr_csc(1, "\tcolor primaries in RGB\n");
	else
		pr_csc(1, "\tcolor primaries not available use default 2020\n");
	for (i = 0; i < 3; i++)
		for (j = 0; j < 2; j++)
			pr_csc(1,
				"\t\tprimaries[%1d][%1d] = %04x\n",
				i, j,
				p->primaries[i][j]);
	pr_csc(1, "\t\twhite_point = (%04x, %04x)\n",
		p->white_point[0], p->white_point[1]);
	pr_csc(1, "\t\tmax, min luminance = %08x, %08x\n",
		p->luminance[0], p->luminance[1]);
}

static uint cur_vd_signal_type = 0xffffffff;
void get_cur_vd_signal_type(enum vd_path_e vd_path)
{
	cur_vd_signal_type = cur_signal_type[vd_path];
}

int get_primaries_type(struct vframe_master_display_colour_s *p_mdc)
{
	if (!p_mdc->present_flag)
		return 0;

#if 0
	/* revisit if needed */
	/* check endian by white point */
	/* need to fix by container parser */
	if (((p_mdc->white_point[0] & 0x00f0) == 0x30)
	&& ((p_mdc->white_point[1] & 0x00f0) == 0x40)
	&& (((p_mdc->white_point[0] & 0xf000) != 0x3000)
	|| ((p_mdc->white_point[1] & 0xf000) != 0x4000))) {
		for (i = 0; i < 2; i++)
			p_mdc->white_point[i] =
				((p_mdc->white_point[i] & 0xff) << 8) |
				((p_mdc->white_point[i] & 0xff00) >> 8);

		for (i = 0; i < 3; i++)
			for (j = 0; j < 2; j++)
				p_mdc->primaries[i][j] =
				((p_mdc->primaries[i][j] & 0xff) << 8) |
				((p_mdc->primaries[i][j] & 0xff00) >> 8);
		if (p_mdc->luminance[0] < 1000 * 10000)
			p_mdc->luminance[0] = 1000 * 10000;
		else if (p_mdc->luminance[0] > 10000 * 10000)
			p_mdc->luminance[0] = 10000 * 10000;
		if (p_mdc->luminance[1] > 100)
			p_mdc->luminance[1] = 100;
		else if (p_mdc->luminance[1] < 50)
			p_mdc->luminance[1] = 50;
		//pr_csc(1, "master_display_colour endian wrong!\n");
	}
#endif

	if ((p_mdc->primaries[0][1] > p_mdc->primaries[1][1])
	&& (p_mdc->primaries[0][1] > p_mdc->primaries[2][1])
	&& (p_mdc->primaries[2][0] > p_mdc->primaries[0][0])
	&& (p_mdc->primaries[2][0] > p_mdc->primaries[1][0])) {
		/* reasonable g,b,r */
		return 2;
	} else if ((p_mdc->primaries[0][0] > p_mdc->primaries[1][0])
	&& (p_mdc->primaries[0][0] > p_mdc->primaries[2][0])
	&& (p_mdc->primaries[1][1] > p_mdc->primaries[0][1])
	&& (p_mdc->primaries[1][1] > p_mdc->primaries[2][1])) {
		/* reasonable r,g,b */
		return 1;
	}
	/* source not usable, use standard bt2020 */
	return 0;
}
EXPORT_SYMBOL(get_primaries_type);

int hdr10_primaries_changed(
	struct vframe_master_display_colour_s *p_mdc,
	struct vframe_master_display_colour_s *p_hdr10_param)
{
	struct vframe_content_light_level_s *p_cll =
		&p_mdc->content_light_level;
	uint8_t flag = 0;
	uint32_t max_lum = 1000 * 10000;
	uint32_t min_lum = 50;
	int primaries_type = 0;
	int i, j;

	primaries_type = get_primaries_type(p_mdc);
	if (primaries_type == 0) {
		if ((p_hdr10_param->luminance[0]
		!= max_lum)
		|| (p_hdr10_param->luminance[1]
		!= min_lum)) {
			flag |= 1;
			p_hdr10_param->luminance[0]
				= max_lum;
			p_hdr10_param->luminance[1]
				= min_lum;
		}
		for (i = 0; i < 2; i++) {
			if (p_hdr10_param->white_point[i]
			!= bt2020_white_point[i]) {
				flag |= 4;
				p_hdr10_param->
				white_point[i] =
					bt2020_white_point[i];
			}
		}
		for (i = 0; i < 3; i++) {
			/* GBR -> GBR */
			for (j = 0; j < 2; j++) {
				if (p_hdr10_param->
				primaries[i][j]
				!= p_mdc->
				primaries[i][j]) {
					p_hdr10_param->
					primaries[i][j]
						= p_mdc->
						primaries[i][j];
					flag |= 2;
				}
			}
		}
		p_hdr10_param->present_flag = 1;
	} else {
		for (i = 0; i < 2; i++) {
			if (p_hdr10_param->luminance[i]
			!= p_mdc->luminance[i]) {
				flag |= 1;
				p_hdr10_param->
				luminance[i]
					= p_mdc->luminance[i];
			}
			if (p_hdr10_param->white_point[i]
			!= p_mdc->white_point[i]) {
				flag |= 4;
				p_hdr10_param->
				white_point[i] =
					p_mdc->white_point[i];
			}
		}
		if (primaries_type == 1) {
			/* RGB -> GBR */
			for (i = 0; i < 3; i++) {
				for (j = 0; j < 2; j++) {
					if (p_hdr10_param->
					primaries[i][j]
					!= p_mdc->
					primaries[(i + 2) % 3][j]) {
						p_hdr10_param->
						primaries[i][j] = p_mdc->
						primaries[(i + 2) % 3][j];
						flag |= 2;
					}
				}
			}
			p_hdr10_param->present_flag = 0x21;
		} else if (primaries_type == 2) {
			/* GBR -> GBR */
			for (i = 0; i < 3; i++) {
				for (j = 0; j < 2; j++) {
					if (p_hdr10_param->
					primaries[i][j]
					!= p_mdc->
					primaries[i][j]) {
						p_hdr10_param->
						primaries[i][j]
							= p_mdc->
							primaries[i][j];
						flag |= 2;
					}
				}
			}
			p_hdr10_param->present_flag = 0x11;
		}
	}

	if (p_cll->present_flag) {
		if ((p_hdr10_param->content_light_level.max_content
			!= p_cll->max_content)
		|| (p_hdr10_param->content_light_level.max_pic_average
			!= p_cll->max_pic_average))
			flag |= 8;
		if (flag & 8) {
			p_hdr10_param->content_light_level.max_content
				= p_cll->max_content;
			p_hdr10_param->content_light_level.max_pic_average
				= p_cll->max_pic_average;
		}
		p_hdr10_param->content_light_level.present_flag = 1;
	} else {
		if ((p_hdr10_param->content_light_level.max_content != 0)
		|| (p_hdr10_param->content_light_level.max_pic_average != 0)) {
			p_hdr10_param->content_light_level.max_content = 0;
			p_hdr10_param->content_light_level.max_pic_average = 0;
		}
		if (p_hdr10_param->content_light_level.present_flag)
			flag |= 8;
		p_hdr10_param->content_light_level.present_flag = 0;
	}
	return flag;
}

uint32_t sink_dv_support(const struct vinfo_s *vinfo)
{
	if (!vinfo || !vinfo->vout_device || !vinfo->vout_device->dv_info)
		return 0;
	if (vinfo->vout_device->dv_info->ieeeoui != 0x00d046)
		return 0;
	if (vinfo->vout_device->dv_info->block_flag != CORRECT)
		return 0;
	/* for sink not support 60 dovi */
	if ((strstr(vinfo->name, "2160p60hz") != NULL) ||
		(strstr(vinfo->name, "2160p50hz") != NULL)) {
		if (!vinfo->vout_device->dv_info->sup_2160p60hz)
			return 0;
	}
	/* for interlace output */
	if (vinfo->height != vinfo->field_height)
		return 0;
	if (vinfo->vout_device->dv_info->ver == 2) {
		if (vinfo->vout_device->dv_info->Interface <= 1)
			return 2; /* LL only */
		else
			return 3; /* STD & LL */
	} else if (vinfo->vout_device->dv_info->low_latency)
		return 3; /* STD & LL */
	return 1; /* STD only */
}
EXPORT_SYMBOL(sink_dv_support);

uint32_t sink_hdr_support(const struct vinfo_s *vinfo)
{
	uint32_t hdr_cap = 0;
	uint32_t dv_cap = 0;

	/* when policy == follow sink(0) or force output (2) */
	/* use force_output */
	if ((get_force_output() != 0)
	&& (get_hdr_policy() != 1)) {
		switch (get_force_output()) {
		case BT709:
			break;
		case BT2020:
			hdr_cap |= BT2020_SUPPORT;
			break;
		case BT2020_PQ:
			hdr_cap |= HDR_SUPPORT;
			break;
		case BT2020_PQ_DYNAMIC:
			hdr_cap |= HDRP_SUPPORT;
			break;
		case BT2020_HLG:
			hdr_cap |= HLG_SUPPORT;
			break;
		case BT2100_IPT:
			hdr_cap |= DV_SUPPORT;
		default:
			break;
		}
	} else if (vinfo) {
		if (vinfo->hdr_info.hdr_support & HDR_SUPPORT)
			hdr_cap |= HDR_SUPPORT;
		if (vinfo->hdr_info.hdr_support & HLG_SUPPORT)
			hdr_cap |= HLG_SUPPORT;
		if ((vinfo->hdr_info.hdr10plus_info.ieeeoui
		== HDR_PLUS_IEEE_OUI) &&
		(vinfo->hdr_info.hdr10plus_info.application_version
		== 1))
			hdr_cap |= HDRP_SUPPORT;
		if (vinfo->hdr_info.colorimetry_support & 0xe0)
			hdr_cap |= BT2020_SUPPORT;
		dv_cap = sink_dv_support(vinfo);
		if (dv_cap)
			hdr_cap |= (dv_cap << DV_SUPPORT_SHF) & DV_SUPPORT;
	}
	return hdr_cap;
}
EXPORT_SYMBOL(sink_hdr_support);

static char force_colorrange;
static int __init amcsc_colorrange_setup(char *s)
{
	if (!(strcmp(s, "true")))
		force_colorrange = 1;
	else
		force_colorrange = 0;

	return 0;
}
__setup("fullcolorrange=", amcsc_colorrange_setup);

int signal_type_changed(struct vframe_s *vf,
	struct vinfo_s *vinfo, enum vd_path_e vd_path)
{
	u32 signal_type = 0;
	u32 default_signal_type;
	int change_flag = 0;
	int i, j;
	struct vframe_master_display_colour_s *p_cur = NULL;
	struct vframe_master_display_colour_s *p_new = NULL;
	struct vframe_master_display_colour_s cus;
	int ret;
	u32 limit_full = 0;

	if (!vf)
		return 0;
	if (vf->type & VIDTYPE_MVC) {
		if (cur_mvc_type[vd_path] != (vf->type & VIDTYPE_MVC)) {
			change_flag |= SIG_SRC_CHG;
			cur_mvc_type[vd_path] = vf->type & VIDTYPE_MVC;
			pr_csc(1, "VIDTYPE MVC changed.\n");
			/*return change_flag;*/
		}
	} else
		cur_mvc_type[vd_path] = 0;
	limit_full = (vf->signal_type >> 25) & 0x01;
	if ((vf->source_type == VFRAME_SOURCE_TYPE_TUNER) ||
		(vf->source_type == VFRAME_SOURCE_TYPE_CVBS) ||
		(vf->source_type == VFRAME_SOURCE_TYPE_COMP) ||
		(vf->source_type == VFRAME_SOURCE_TYPE_HDMI)) {
		if (get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB)
			default_signal_type =
				/* default 709 full */
				  (1 << 29)	/* video available */
				| (5 << 26)	/* unspecified */
				| (1 << 25)	/* limit */
				| (1 << 24)	/* color available */
				| (1 << 16)	/* bt709 */
				| (1 << 8)	/* bt709 */
				| (1 << 0);	/* bt709 */
		else
			default_signal_type =
				/* default 709 limit */
				  (1 << 29)	/* video available */
				| (5 << 26)	/* unspecified */
				| (limit_full << 25)	/* limit */
				| (1 << 24)	/* color available */
				| (1 << 16)	/* bt709 */
				| (1 << 8)	/* bt709 */
				| (1 << 0);	/* bt709 */
	} else { /* for local play */
		if (vf->height >= 720)
			default_signal_type =
				/* HD default 709 limit */
				  (1 << 29)	/* video available */
				| (5 << 26)	/* unspecified */
				| (force_colorrange << 25)	/* limit */
				| (1 << 24)	/* color available */
				| (1 << 16)	/* bt709 */
				| (1 << 8)	/* bt709 */
				| (1 << 0);	/* bt709 */
		else
			default_signal_type =
				/* SD default 601 limited */
				  (1 << 29)	/* video available */
				| (5 << 26)	/* unspecified */
				| (0 << 25)	/* limited */
				| (1 << 24)	/* color available */
				| (3 << 16)	/* bt601 */
				| (3 << 8)	/* bt601 */
				| (3 << 0);	/* bt601 */
	}
	if (vf->signal_type & (1 << 29))
		signal_type = vf->signal_type;
	else
		signal_type = default_signal_type;

	p_new = &vf->prop.master_display_colour;
	p_cur = &cur_master_display_colour[vd_path];
	/* customer overwrite */
	if (customer_master_display_en
	&& ((p_new->present_flag & 0x80000000) == 0)) {
		signal_type =	  (1 << 29)	/* video available */
				| (5 << 26)	/* unspecified */
				| (0 << 25)	/* limit */
				| (1 << 24)	/* color available */
				| (9 << 16)	/* 2020 */
				| (16 << 8)	/* 2084 */
				| (9 << 0);	/* 2020 */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 2; j++)
				cus.primaries[i][j] =
					customer_master_display_param[i*2+j];
		for (i = 0; i < 2; i++)
			cus.white_point[i] =
				customer_master_display_param[6+i];
		for (i = 0; i < 2; i++)
			cus.luminance[i] =
				customer_master_display_param[8+i];
		cus.present_flag = 1;
		p_new = &cus;
	}

	/* For txhd, must skip HDR process */
	if (is_meson_txhd_cpu()) {
		if (((signal_type & 0xff) != 1)	&&
			((signal_type & 0xff) != 3)) {
			signal_type &= 0xffffff00;
			signal_type |= 0x00000001;
		}
		if (((signal_type & 0xff00) >> 8 != 1) &&
			((signal_type & 0xff00) >> 8 != 3)) {
			signal_type &= 0xffff00ff;
			signal_type |= 0x00000100;
		}
		if (((signal_type & 0xff0000) >> 16 != 1) &&
			((signal_type & 0xff0000) >> 16 != 3)) {
			signal_type &= 0xff00ffff;
			signal_type |= 0x00010000;
		}
		/*pr_err("Signal type changed from 0x%x to 0x%x.\n",*/
		/*vf->signal_type, signal_type);*/
	}

	if (p_new && p_cur) {
		ret = hdr10_primaries_changed(p_new, p_cur);
		if (ret)
			change_flag |= SIG_PRI_INFO;
	}
	if (change_flag & SIG_PRI_INFO) {
		pr_csc(1, "vd%d Master_display_colour changed %x.\n",
			vd_path + 1, ret);
		print_primaries_info(p_cur);
	}
	if (signal_type != cur_signal_type[vd_path]) {
		pr_csc(1, "vd%d Signal type changed from 0x%x to 0x%x.\n",
			vd_path + 1,
			cur_signal_type[vd_path],
			signal_type);
		change_flag |= SIG_CS_CHG;
		cur_signal_type[vd_path] = signal_type;
	}
	if (pre_src_type[vd_path] != vf->source_type) {
		pr_csc(1, "vd%d Signal source changed from 0x%x to 0x%x.\n",
			vd_path + 1, pre_src_type[vd_path], vf->source_type);
		change_flag |= SIG_SRC_CHG;
		pre_src_type[vd_path] = vf->source_type;
	}
	if (cur_knee_factor != knee_factor) {
		pr_csc(1, "vd%d Knee factor changed.\n", vd_path + 1);
		change_flag |= SIG_KNEE_FACTOR;
		cur_knee_factor = knee_factor;
	}
	if (!is_dolby_vision_on() || vd_path != VD1_PATH) {
		if (cur_hdr_process_mode[vd_path]
		!= hdr_process_mode[vd_path]) {
			pr_csc(1, "vd%d HDR mode changed %d %d.\n", vd_path + 1,
				cur_hdr_process_mode[vd_path],
				hdr_process_mode[vd_path]);
			change_flag |= SIG_HDR_MODE;
		}
		if (cur_hlg_process_mode[vd_path]
		!= hlg_process_mode[vd_path]) {
			pr_csc(1, "vd%d HLG mode changed %d %d.\n", vd_path + 1,
				cur_hlg_process_mode[vd_path],
				hlg_process_mode[vd_path]);
			change_flag |= SIG_HLG_MODE;
		}
		if (cur_sdr_process_mode[vd_path]
		!= sdr_process_mode[vd_path]) {
			pr_csc(1, "vd%d SDR mode changed %d %d.\n", vd_path + 1,
				cur_sdr_process_mode[vd_path],
				sdr_process_mode[vd_path]);
			change_flag |= SIG_HDR_MODE;
		}
		if (cur_hdr10_plus_process_mode[vd_path]
		!= hdr10_plus_process_mode[vd_path]) {
			pr_csc(1, "vd%d HDR10+ mode changed %d %d.\n",
				vd_path + 1,
				cur_hdr10_plus_process_mode[vd_path],
				hdr10_plus_process_mode[vd_path]);
			change_flag |= SIG_HDR10_PLUS_MODE;
		}
	}

	if (cur_hdr_support
	!= (vinfo->hdr_info.hdr_support & 0x4)) {
		pr_csc(1, "Tx HDR support changed.\n");
		change_flag |= SIG_HDR_SUPPORT;
		cur_hdr_support =
			vinfo->hdr_info.hdr_support & 0x4;
	}
	if (cur_colorimetry_support != (vinfo->hdr_info.colorimetry_support)) {
		pr_csc(1, "Tx colorimetry support changed.\n");
		change_flag |= SIG_COLORIMETRY_SUPPORT;
		cur_colorimetry_support = vinfo->hdr_info.colorimetry_support;
	}
	if (cur_color_fmt != vinfo->viu_color_fmt) {
		pr_csc(1, "color format changed.\n");
		change_flag |= SIG_OP_CHG;
		cur_color_fmt = vinfo->viu_color_fmt;
	}
	if (cur_hlg_support != (vinfo->hdr_info.hdr_support & 0x8)) {
		pr_csc(1, "Tx HLG support changed.\n");
		change_flag |= SIG_HLG_SUPPORT;
		cur_hlg_support = vinfo->hdr_info.hdr_support & 0x8;
	}

	if ((cur_eye_protect_mode != wb_val[0]) ||
		(cur_eye_protect_mode == 1)) {
		pr_csc(1, "eye protect mode changed.\n");
		change_flag |= SIG_WB_CHG;
	}

	if (src_timing_outputmode_changed(vf, vinfo, vd_path)) {
		pr_csc(1, "vd%d source timing output mode changed.\n",
				vd_path + 1);
		change_flag |= SIG_SRC_OUTPUT_CHG;
	}

	if (vecm_latch_flag & FLAG_HDR_OOTF_LATCH) {
		change_flag |= SIG_HDR_OOTF_CHG;
		vecm_latch_flag &= ~FLAG_HDR_OOTF_LATCH;
		pr_csc(1, "ootf curve changed.\n");
	}

	return change_flag;
}

#define signal_format ((cur_vd_signal_type >> 26) & 7)
#define signal_range ((cur_vd_signal_type >> 25) & 1)
#define signal_color_primaries ((cur_vd_signal_type >> 16) & 0xff)
#define signal_transfer_characteristic ((cur_vd_signal_type >> 8) & 0xff)
enum vpp_matrix_csc_e get_csc_type(void)
{
	enum vpp_matrix_csc_e csc_type = VPP_MATRIX_NULL;

	if ((signal_color_primaries == 1) &&
		(signal_transfer_characteristic < 14)) {
		if (signal_range == 0)
			csc_type = VPP_MATRIX_YUV709_RGB;
		else
			csc_type = VPP_MATRIX_YUV709F_RGB;
	} else if ((signal_color_primaries == 3) &&
			(signal_transfer_characteristic < 14)) {
		if (signal_range == 0)
			csc_type = VPP_MATRIX_YUV601_RGB;
		else
			csc_type = VPP_MATRIX_YUV601F_RGB;
	} else if ((signal_color_primaries == 9) ||
			(signal_transfer_characteristic >= 14)) {
		if (signal_transfer_characteristic == 16) {
			/* smpte st-2084 */
			if (signal_color_primaries != 9)
				pr_csc(1, "\tWARNING: non-standard HDR!!!\n");
			csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
		} else if (signal_transfer_characteristic == 14) {
			/* bt2020-10 */
			pr_csc(1, "\tWARNING: bt2020-10 HDR!!!\n");
			csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
		} else if (signal_transfer_characteristic == 15) {
			/* bt2020-12 */
			pr_csc(1, "\tWARNING: bt2020-12 HDR!!!\n");
			if (signal_range == 0)
				csc_type = VPP_MATRIX_YUV709_RGB;
			else
				csc_type = VPP_MATRIX_YUV709F_RGB;
		} else if (signal_transfer_characteristic == 18) {
			/* bt2020-12 */
			/* pr_csc(1, "\tHLG!!!\n"); */
			csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
		} else if (signal_transfer_characteristic == 0x30) {
			if (signal_color_primaries == 9) {
				/* pr_csc(1, "\tHDR10+!!!\n"); */
				csc_type =
					VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC;
			} else {
				pr_csc(1,
					"\tWARNING: non-standard HDR10+!!!\n");
				csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
			}
		} else {
			/* unknown transfer characteristic */
			pr_csc(1, "\tWARNING: unknown HDR!!!\n");
			if (signal_range == 0)
				csc_type = VPP_MATRIX_YUV709_RGB;
			else
				csc_type = VPP_MATRIX_YUV709F_RGB;
		}
	} else {
		if (signal_format == 0)
			pr_csc(1, "\tWARNING: unknown colour space!!!\n");
		if (signal_range == 0)
			csc_type = VPP_MATRIX_YUV601_RGB;
		else
			csc_type = VPP_MATRIX_YUV601F_RGB;
	}
	return csc_type;
}
/*hdr10: return 0;  hlg: return 1*/
#define HLG_FLAG 0x1
static int get_hdr_type(void)
{
	int change_flag = 0;

	if ((signal_transfer_characteristic == 18) ||
		(signal_transfer_characteristic == 14))
		change_flag |= HLG_FLAG;

	return change_flag;
}

void get_hdr_source_type(void)
{
	if (((signal_transfer_characteristic == 14) ||
		(signal_transfer_characteristic == 18)) &&
		(signal_color_primaries == 9))
		hdr_source_type = HLG_SOURCE;
	else if ((signal_transfer_characteristic == 0x30)
		&& (signal_color_primaries == 9))
		hdr_source_type = HDR10PLUS_SOURCE;
	else if (signal_transfer_characteristic == 16)
		hdr_source_type = HDR10_SOURCE;
	else
		hdr_source_type = SDR_SOURCE;
}

enum color_primary_e get_color_primary(void)
{
	enum color_primary_e color_pri;

	if (signal_color_primaries == 1)
		color_pri = VPP_COLOR_PRI_BT709;
	else if (signal_color_primaries == 3)
		color_pri = VPP_COLOR_PRI_BT601;
	else if (signal_color_primaries == 9)
		color_pri = VPP_COLOR_PRI_BT2020;
	else
		color_pri = VPP_COLOR_PRI_NULL;
	return color_pri;
}

static void cal_out_curve(uint panel_luma)
{
	int index;

	if (panel_luma == 0)
		return;

	if (panel_luma <= 500) {
		if (panel_luma < 250)
			panel_luma = 250;
		index = (panel_luma - 250) / 20;
	} else {
		if (panel_luma > 1000)
			panel_luma = 1000;
		index = ((500 - 260) / 20) + (panel_luma - 500) / 100;
	}
	memcpy(eotf_33_2084_mapping,
		eotf_33_2084_table[index], sizeof(int) * EOTF_LUT_SIZE);
	memcpy(oetf_289_gamma22_mapping,
		oetf_289_gamma22_table[index],
		sizeof(int) * VIDEO_OETF_LUT_SIZE);
}
static void mtx_dot_mul(
	int64_t (*a)[3], int64_t (*b)[3],
	int64_t (*out)[3], int32_t norm)
{
	int i, j;
	int64_t tmp;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			tmp = a[i][j] * b[i][j] + (norm >> 1);
			out[i][j] = div64_s64(tmp, norm);
		}
}

static void mtx_mul(int64_t (*a)[3], int64_t *b, int64_t *out, int32_t norm)
{
	int j, k;
	int64_t tmp;

	for (j = 0; j < 3; j++) {
		out[j] = 0;
		for (k = 0; k < 3; k++)
			out[j] += a[k][j] * b[k];
		tmp = out[j] + (norm >> 1);
		out[j] = div64_s64(tmp, norm);
	}
}

static void mtx_mul_mtx(
	int64_t (*a)[3], int64_t (*b)[3],
	int64_t (*out)[3], int32_t norm)
{
	int i, j, k;
	int64_t tmp;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			out[i][j] = 0;
			for (k = 0; k < 3; k++)
				out[i][j] += a[k][j] * b[i][k];
			tmp = out[i][j] + (norm >> 1);
			out[i][j] = div64_s64(tmp, norm);
		}
}

static void inverse_3x3(
	int64_t (*in)[3], int64_t (*out)[3],
	int32_t norm, int32_t obl)
{
	int i, j;
	int64_t determinant = 0;
	int64_t tmp;

	for (i = 0; i < 3; i++)
		determinant +=
			in[0][i] * (in[1][(i + 1) % 3] * in[2][(i + 2) % 3]
			- in[1][(i + 2) % 3] * in[2][(i + 1) % 3]);
	determinant = (determinant + 1) >> 1;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			out[j][i] = (in[(i + 1) % 3][(j + 1) % 3]
				* in[(i + 2) % 3][(j + 2) % 3]);
			out[j][i] -= (in[(i + 1) % 3][(j + 2) % 3]
				* in[(i + 2) % 3][(j + 1) % 3]);
			out[j][i] = (out[j][i] * norm) << (obl - 1);
			tmp = out[j][i] + (determinant >> 1);
			out[j][i] = div64_s64(tmp, determinant);
		}
	}
}

static void calc_T(
	int64_t (*prmy)[2], int64_t (*Tout)[3],
	int32_t norm, int32_t obl)
{
	int i, j;
	int64_t z[4];
	int64_t A[3][3];
	int64_t B[3];
	int64_t C[3];
	int64_t D[3][3];
	int64_t E[3][3];
	int64_t tmp;

	for (i = 0; i < 4; i++)
		z[i] = norm - prmy[i][0] - prmy[i][1];

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 2; j++)
			A[i][j] = prmy[i][j];
		A[i][2] = z[i];
	}
	tmp = norm * prmy[3][0] * 2;
	tmp = div64_s64(tmp, prmy[3][1]);
	B[0] = (tmp + 1) >> 1;
	B[1] = norm;
	tmp = norm * z[3] * 2;
	tmp = div64_s64(tmp, prmy[3][1]);
	B[2] = (tmp + 1) >> 1;
	inverse_3x3(A, D, norm, obl);
	mtx_mul(D, B, C, norm);
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			E[i][j] = C[i];
	mtx_dot_mul(A, E, Tout, norm);
}

static void gamut_mtx(
	int64_t (*src_prmy)[2], int64_t (*dst_prmy)[2],
	int64_t (*Tout)[3], int32_t norm, int32_t obl)
{
	int64_t Tsrc[3][3];
	int64_t Tdst[3][3];
	int64_t out[3][3];

	calc_T(src_prmy, Tsrc, norm, obl);
	calc_T(dst_prmy, Tdst, norm, obl);
	inverse_3x3(Tdst, out, 1 << obl, obl);
	mtx_mul_mtx(out, Tsrc, Tout, 1 << obl);
}

static void apply_scale_factor(int64_t (*in)[3], int32_t *rs)
{
	int i, j;
	int32_t right_shift;

	if (display_scale_factor > 2 * 2048)
		right_shift = -2;
	else if (display_scale_factor > 2048)
		right_shift = -1;
	else
		right_shift = 0;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			in[i][j] *= display_scale_factor;
			in[i][j] >>= 11 - right_shift;
		}
	right_shift += 1;
	if (right_shift < 0)
		*rs = 8 + right_shift;
	else
		*rs = right_shift;
}

static void N2C(int64_t (*in)[3], int32_t ibl, int32_t obl)
{
	int i, j;

	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			in[i][j] =
				(in[i][j] + (1 << (ibl - 12))) >> (ibl - 11);
			if (in[i][j] < 0)
				in[i][j] += 1 << obl;
		}
}

static void cal_mtx_seting(
	int64_t (*in)[3],
	int32_t ibl, int32_t obl,
	struct matrix_s *m)
{
	int i, j;
	int32_t right_shift;

	if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB) {
		apply_scale_factor(in, &right_shift);
		m->right_shift = right_shift;
	}
	N2C(in, ibl, obl);
	pr_csc(1, "\tHDR color convert matrix:\n");
	for (i = 0; i < 3; i++) {
		m->pre_offset[i] = 0;
		for (j = 0; j < 3; j++)
			m->matrix[i][j] = in[j][i];
		m->offset[i] = 0;
		pr_csc(1, "\t\t%04x %04x %04x\n",
			(int)(m->matrix[i][0] & 0xffff),
			(int)(m->matrix[i][1] & 0xffff),
			(int)(m->matrix[i][2] & 0xffff));
	}
}

static int check_primaries(
	/* src primaries and white point */
	uint (*p)[3][2],
	uint (*w)[2],
	/* dst display info from vinfo */
	const struct vinfo_s *v,
	/* prepare src and dst color info array */
	int64_t (*si)[4][2], int64_t (*di)[4][2])
{
	int i, j;
	/* always calculate to apply scale factor */
	int need_calculate_mtx = 1;
	const struct master_display_info_s *d;

	/* check and copy primaries */
	if (hdr_flag & 1) {
		if (((*p)[0][1] > (*p)[1][1])
			&& ((*p)[0][1] > (*p)[2][1])
			&& ((*p)[2][0] > (*p)[0][0])
			&& ((*p)[2][0] > (*p)[1][0])) {
			/* reasonable g,b,r */
			for (i = 0; i < 3; i++)
				for (j = 0; j < 2; j++) {
					(*si)[i][j] = (*p)[(i + 2) % 3][j];
				if ((*si)[i][j] !=
				bt2020_primaries[(i + 2) % 3][j])
					need_calculate_mtx = 1;
				}
		} else if (((*p)[0][0] > (*p)[1][0])
			&& ((*p)[0][0] > (*p)[2][0])
			&& ((*p)[1][1] > (*p)[0][1])
			&& ((*p)[1][1] > (*p)[2][1])) {
			/* reasonable r,g,b */
			for (i = 0; i < 3; i++)
				for (j = 0; j < 2; j++) {
					(*si)[i][j] = (*p)[i][j];
					if ((*si)[i][j] !=
					bt2020_primaries[(i + 2) % 3][j])
						need_calculate_mtx = 1;
				}
		} else {
			/* source not usable, use standard bt2020 */
			for (i = 0; i < 3; i++)
				for (j = 0; j < 2; j++)
					(*si)[i][j] =
						bt2020_primaries
						[(i + 2) % 3][j];
		}
		/* check white point */
		if (need_calculate_mtx == 1) {
			if (((*w)[0] > (*si)[2][0]) &&
				((*w)[0] < (*si)[0][0]) &&
				((*w)[1] > (*si)[2][1]) &&
				((*w)[1] < (*si)[1][1])) {
				for (i = 0; i < 2; i++)
					(*si)[3][i] = (*w)[i];
			} else {
				for (i = 0; i < 3; i++)
					for (j = 0; j < 2; j++)
						(*si)[i][j] =
					bt2020_primaries[(i + 2) % 3][j];

				for (i = 0; i < 2; i++)
					(*si)[3][i] = bt2020_white_point[i];
				/* need_calculate_mtx = 0; */
			}
		} else {
			if (((*w)[0] > (*si)[2][0]) &&
				((*w)[0] < (*si)[0][0]) &&
				((*w)[1] > (*si)[2][1]) &&
				((*w)[1] < (*si)[1][1])) {
				for (i = 0; i < 2; i++) {
					(*si)[3][i] = (*w)[i];
					if ((*si)[3][i] !=
					bt2020_white_point[i])
						need_calculate_mtx = 1;
				}
			} else {
				for (i = 0; i < 2; i++)
					(*si)[3][i] = bt2020_white_point[i];
			}
		}
	} else {
		/* use standard bt2020 */
		for (i = 0; i < 3; i++)
			for (j = 0; j < 2; j++)
				(*si)[i][j] = bt2020_primaries[(i + 2) % 3][j];
		for (i = 0; i < 2; i++)
			(*si)[3][i] = bt2020_white_point[i];
	}

	/* check display */
	if ((v->master_display_info.present_flag) && (hdr_flag & 2)) {
		d = &v->master_display_info;
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 2; j++) {
				(*di)[i][j] = d->primaries[(i + 2) % 3][j];
				if ((*di)[i][j] !=
				bt709_primaries[(i + 2) % 3][j])
					need_calculate_mtx = 1;
			}
		}
		for (i = 0; i < 2; i++) {
			(*di)[3][i] = d->white_point[i];
			if ((*di)[3][i] != bt709_white_point[i])
				need_calculate_mtx = 1;
		}
		if (v->mode == VMODE_LCD)
			cal_out_curve(v->hdr_info.lumi_max);
	} else {
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 2; j++)
				(*di)[i][j] = bt709_primaries[(i + 2) % 3][j];
		}
		for (i = 0; i < 2; i++)
			(*di)[3][i] = bt709_white_point[i];
	}
	return need_calculate_mtx;
}

enum vpp_matrix_csc_e prepare_customer_matrix(
	u32 (*s)[3][2],	/* source primaries */
	u32 (*w)[2],	/* source white point */
	const struct vinfo_s *v, /* vinfo carry display primaries */
	struct matrix_s *m,
	bool inverse_flag)
{
	int64_t prmy_src[4][2];
	int64_t prmy_dst[4][2];
	int64_t out[3][3];
	int i, j;

	if ((customer_matrix_en &&
		get_cpu_type() <= MESON_CPU_MAJOR_ID_GXTVBB)) {
		for (i = 0; i < 3; i++) {
			m->pre_offset[i] =
				customer_matrix_param[i];
			for (j = 0; j < 3; j++)
				m->matrix[i][j] =
					customer_matrix_param[3 + i * 3 + j];
			m->offset[i] =
				customer_matrix_param[12 + i];
		}
		m->right_shift =
			customer_matrix_param[15];
		return VPP_MATRIX_BT2020RGB_CUSRGB;
	} else {
		if (inverse_flag) {
			if (check_primaries(s, w, v, &prmy_src, &prmy_dst)) {
				gamut_mtx(prmy_dst, prmy_src, out, INORM, BL);
				cal_mtx_seting(out, BL, 13, m);
			}
		} else {
			if (check_primaries(s, w, v, &prmy_src, &prmy_dst)) {
				gamut_mtx(prmy_src, prmy_dst, out, INORM, BL);
				cal_mtx_seting(out, BL, 13, m);
			}
		}
		return VPP_MATRIX_BT2020RGB_CUSRGB;
	}
	return VPP_MATRIX_BT2020YUV_BT2020RGB;
}

/* Max luminance lookup table for contrast */
static const int maxLuma_thrd[5] = {512, 1024, 2048, 4096, 8192};
static int calculate_contrast_adj(int max_lumin)
{
	int k;
	int left, right, norm, alph;
	int ratio, target_contrast;

	if (max_lumin < maxLuma_thrd[0])
		k = 0;
	else if (max_lumin < maxLuma_thrd[1])
		k = 1;
	else if (max_lumin < maxLuma_thrd[2])
		k = 2;
	else if (max_lumin < maxLuma_thrd[3])
		k = 3;
	else if (max_lumin < maxLuma_thrd[4])
		k = 4;
	else
		k = 5;

	if (k == 0)
		ratio = extra_con_lut[0];
	else if (k == 5)
		ratio = extra_con_lut[4];
	else {
		left = extra_con_lut[k - 1];
		right = extra_con_lut[k];
		norm = maxLuma_thrd[k] - maxLuma_thrd[k - 1];
		alph = max_lumin - maxLuma_thrd[k - 1];
		ratio = left + (alph * (right - left) + (norm >> 1)) / norm;
	}
	target_contrast = ((vd1_contrast + 1024) * ratio + 64) >> 7;
	target_contrast = clip(target_contrast, 0, 2047);
	target_contrast -= 1024;
	return target_contrast - vd1_contrast;
}

static void amvecm_cp_hdr_info(struct master_display_info_s *hdr_data,
		struct vframe_master_display_colour_s *p)
{
	int i, j;

	if (customer_hdmi_display_en) {
		hdr_data->features =
			  (1 << 29)	/* video available */
			| (5 << 26)	/* unspecified */
			| (0 << 25)	/* limit */
			| (1 << 24)	/* color available */
			| (customer_hdmi_display_param[0] << 16) /* bt2020 */
			| (customer_hdmi_display_param[1] << 8)	/* 2084 */
			| (10 << 0);	/* bt2020c */
		memcpy(hdr_data->primaries,
			&customer_hdmi_display_param[2],
			sizeof(u32)*6);
		memcpy(hdr_data->white_point,
			&customer_hdmi_display_param[8],
			sizeof(u32)*2);
		hdr_data->luminance[0] =
				customer_hdmi_display_param[10];
		hdr_data->luminance[1] =
				customer_hdmi_display_param[11];
		hdr_data->max_content =
				customer_hdmi_display_param[12];
		hdr_data->max_frame_average =
				customer_hdmi_display_param[13];
	} else if ((((hdr_data->features >> 16) & 0xff) == 9) ||
		((((hdr_data->features >> 8) & 0xff) >= 14) &&
		(((hdr_data->features >> 8) & 0xff) <= 18))) {
		if (p->present_flag & 1) {
			memcpy(hdr_data->primaries,
				p->primaries,
				sizeof(u32)*6);
			memcpy(hdr_data->white_point,
				p->white_point,
				sizeof(u32)*2);
			hdr_data->luminance[0] =
				p->luminance[0];
			hdr_data->luminance[1] =
				p->luminance[1];
			if (p->content_light_level.present_flag == 1) {
				hdr_data->max_content =
					p->content_light_level.max_content;
				hdr_data->max_frame_average =
					p->content_light_level.max_pic_average;
			} else {
				hdr_data->max_content = 0;
				hdr_data->max_frame_average = 0;
			}
		} else {
			for (i = 0; i < 3; i++)
				for (j = 0; j < 2; j++)
					hdr_data->primaries[i][j] =
							bt2020_primaries[i][j];
			hdr_data->white_point[0] = bt709_white_point[0];
			hdr_data->white_point[1] = bt709_white_point[1];
			/* default luminance */
			hdr_data->luminance[0] = 5000 * 10000;
			hdr_data->luminance[1] = 50;

			/* content_light_level */
			hdr_data->max_content = 0;
			hdr_data->max_frame_average = 0;
		}
		hdr_data->luminance[0] = hdr_data->luminance[0] / 10000;
		hdr_data->present_flag = 1;
	} else
		memset(hdr_data->primaries, 0, sizeof(hdr_data->primaries));
	/* hdr send information debug */
	memcpy(&dbg_hdr_send, hdr_data,
			sizeof(struct master_display_info_s));
}

static void hdr_process_pq_enable(int enable)
{
	dnlp_en = enable;
	/*cm_en = enable;*/
}

static void vpp_lut_curve_set(enum vpp_lut_sel_e lut_sel,
	struct vinfo_s *vinfo)
{
	if (lut_sel == VPP_LUT_EOTF) {
		/* eotf lut 2048 */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
			if (video_lut_swtich == 1)
				/*350nit alpha_low = 0.12; */
				set_vpp_lut(VPP_LUT_EOTF,
					eotf_33_2084_mapping_level1_box, /* R */
					eotf_33_2084_mapping_level1_box, /* G */
					eotf_33_2084_mapping_level1_box, /* B */
					CSC_ON);
			else if (video_lut_swtich == 2)
				/*800nit alpha_low = 0.12; */
				set_vpp_lut(VPP_LUT_EOTF,
					eotf_33_2084_mapping_level2_box, /* R */
					eotf_33_2084_mapping_level2_box, /* G */
					eotf_33_2084_mapping_level2_box, /* B */
					CSC_ON);
			else if (video_lut_swtich == 3)
				/*400nit alpha_low = 0.20; */
				set_vpp_lut(VPP_LUT_EOTF,
					eotf_33_2084_mapping_level3_box, /* R */
					eotf_33_2084_mapping_level3_box, /* G */
					eotf_33_2084_mapping_level3_box, /* B */
					CSC_ON);
			else if (video_lut_swtich == 4)
				/*450nit alpha_low = 0.12; */
				set_vpp_lut(VPP_LUT_EOTF,
					eotf_33_2084_mapping_level4_box, /* R */
					eotf_33_2084_mapping_level4_box, /* G */
					eotf_33_2084_mapping_level4_box, /* B */
					CSC_ON);
			else
				/* eotf lut 2048 */
				/*600nit  alpha_low = 0.12;*/
				set_vpp_lut(VPP_LUT_EOTF,
					eotf_33_2084_mapping_box, /* R */
					eotf_33_2084_mapping_box, /* G */
					eotf_33_2084_mapping_box, /* B */
					CSC_ON);
		} else
			set_vpp_lut(VPP_LUT_EOTF,
				eotf_33_2084_mapping, /* R */
				eotf_33_2084_mapping, /* G */
				eotf_33_2084_mapping, /* B */
				CSC_ON);
	} else if (lut_sel == VPP_LUT_OETF) {
		/* oetf lut bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
			if (video_lut_swtich == 1)
				set_vpp_lut(VPP_LUT_OETF,
					oetf_289_gamma22_mapping_level1_box,
					oetf_289_gamma22_mapping_level1_box,
					oetf_289_gamma22_mapping_level1_box,
					CSC_ON);
			else if (video_lut_swtich == 2)
				set_vpp_lut(VPP_LUT_OETF,
					oetf_289_gamma22_mapping_level2_box,
					oetf_289_gamma22_mapping_level2_box,
					oetf_289_gamma22_mapping_level2_box,
					CSC_ON);
			else if (video_lut_swtich == 3)
				set_vpp_lut(VPP_LUT_OETF,
					oetf_289_gamma22_mapping_level3_box,
					oetf_289_gamma22_mapping_level3_box,
					oetf_289_gamma22_mapping_level3_box,
					CSC_ON);
			else if (video_lut_swtich == 4)
				set_vpp_lut(VPP_LUT_OETF,
					oetf_289_gamma22_mapping_level4_box,
					oetf_289_gamma22_mapping_level4_box,
					oetf_289_gamma22_mapping_level4_box,
					CSC_ON);
			else
				/* oetf lut bypass */
				set_vpp_lut(VPP_LUT_OETF,
					oetf_289_gamma22_mapping_box,
					oetf_289_gamma22_mapping_box,
					oetf_289_gamma22_mapping_box,
					CSC_ON);
		} else
			set_vpp_lut(VPP_LUT_OETF,
				oetf_289_gamma22_mapping,
				oetf_289_gamma22_mapping,
				oetf_289_gamma22_mapping,
				CSC_ON);
	}
}

static int hdr10p_process(
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
		hdr_func(OSD1_HDR, HDR_BYPASS, vinfo);
		if (vd_path == VD1_PATH)
			hdr10p_func(VD1_HDR, HDR10P_SDR, vinfo);
		else
			hdr10p_func(VD2_HDR, HDR10P_SDR, vinfo);
	}

	return 0;
}

static int hdr_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	int need_adjust_contrast_saturation = 0;
	int max_lumin = 10000;
	struct matrix_s m = {
		{0, 0, 0},
		{
			{0x0d49, 0x1b4d, 0x1f6b},
			{0x1f01, 0x0910, 0x1fef},
			{0x1fdb, 0x1f32, 0x08f3},
		},
		{0, 0, 0},
		1
	};
	struct matrix_s osd_m = {
		{0, 0, 0},
		{
			{0x505, 0x2A2, 0x059},
			{0x08E, 0x75B, 0x017},
			{0x022, 0x0B4, 0x72A},
		},
		{0, 0, 0},
		1
	};
	int mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(1.6607056/2), EOTF_COEFF_NORM(-0.5877533/2),
		EOTF_COEFF_NORM(-0.0729065/2),
		EOTF_COEFF_NORM(-0.1245575/2), EOTF_COEFF_NORM(1.1329346/2),
		EOTF_COEFF_NORM(-0.0083771/2),
		EOTF_COEFF_NORM(-0.0181122/2), EOTF_COEFF_NORM(-0.1005249/2),
		EOTF_COEFF_NORM(1.1186371/2),
		EOTF_COEFF_RIGHTSHIFT,
	};
	int osd_mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(0.627441),	EOTF_COEFF_NORM(0.329285),
		EOTF_COEFF_NORM(0.043274),
		EOTF_COEFF_NORM(0.069092),	EOTF_COEFF_NORM(0.919556),
		EOTF_COEFF_NORM(0.011322),
		EOTF_COEFF_NORM(0.016418),	EOTF_COEFF_NORM(0.088058),
		EOTF_COEFF_NORM(0.895554),
		EOTF_COEFF_RIGHTSHIFT
	};
	int i, j;

	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
		hdr_highclip_by_luma(master_info);
		hdr_func(OSD1_HDR, HDR_BYPASS, vinfo);
		if (vd_path == VD1_PATH)
			hdr_func(VD1_HDR, HDR_SDR, vinfo);
		else
			hdr_func(VD2_HDR, HDR_SDR, vinfo);
		return need_adjust_contrast_saturation;
	}

	if (master_info->present_flag & 1) {
		pr_csc(1, "\tMaster_display_colour available.\n");
		print_primaries_info(master_info);
		/* for VIDEO */
		csc_type =
			prepare_customer_matrix(
				&master_info->primaries,
				&master_info->white_point,
				vinfo, &m, 0);
		/* for OSD */
		if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
			prepare_customer_matrix(
				&bt2020_primaries,
				&bt2020_white_point,
				vinfo, &osd_m, 1);
		need_adjust_contrast_saturation |= 1;
	} else {
		/* use bt2020 primaries */
		pr_csc(1, "\tNo master_display_colour.\n");
		/* for VIDEO */
		csc_type =
			prepare_customer_matrix(
			&bt2020_primaries,
			&bt2020_white_point,
			vinfo, &m, 0);
		/* for OSD */
		if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
			prepare_customer_matrix(
				&bt2020_primaries,
				&bt2020_white_point,
				vinfo, &osd_m, 1);
	}

	/************** OSD ***************/
	/*vpp matrix mux read*/
	vpp_set_mtx_en_read();
	if ((get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
		&& (csc_en & 0x2)) {
		/* RGB to YUV */
		/* not using old RGB2YUV convert HW */
		/* use new 10bit OSD convert matrix */
		/* WRITE_VPP_REG_BITS */
		/*(VIU_OSD1_BLK0_CFG_W0,0, 7, 1); */

		/* eotf lut 709 */
		if (is_hdr_cfg_osd_100() == 1) {
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				osd_eotf_33_709_mapping_100, /* R */
				osd_eotf_33_709_mapping_100, /* G */
				osd_eotf_33_709_mapping_100, /* B */
				CSC_ON);
		} else if (is_hdr_cfg_osd_100() == 2) {
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				osd_eotf_33_709_mapping_290, /* R */
				osd_eotf_33_709_mapping_290, /* G */
				osd_eotf_33_709_mapping_290, /* B */
				CSC_ON);
		} else if (is_hdr_cfg_osd_100() == 3) {
			set_vpp_lut(
				VPP_LUT_OSD_EOTF,
				osd_eotf_33_709_mapping_330, /* R */
				osd_eotf_33_709_mapping_330, /* G */
				osd_eotf_33_709_mapping_330, /* B */
				CSC_ON);
		} else {
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				osd_eotf_33_709_mapping, /* R */
				osd_eotf_33_709_mapping, /* G */
				osd_eotf_33_709_mapping, /* B */
				CSC_ON);
		}
		/* eotf matrix 709->2020 */
		osd_mtx[EOTF_COEFF_SIZE - 1] = osd_m.right_shift;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (osd_m.matrix[i][j] & 0x1000)
					osd_mtx[i * 3 + j] =
					-(((~osd_m.matrix[i][j]) & 0xfff) + 1);
				else
					osd_mtx[i * 3 + j] = osd_m.matrix[i][j];
			}
		set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
			osd_mtx,
			CSC_ON);

		/* oetf lut 2084 */
		if (is_hdr_cfg_osd_100() == 1) {
			set_vpp_lut(VPP_LUT_OSD_OETF,
				osd_oetf_41_2084_mapping_100, /* R */
				osd_oetf_41_2084_mapping_100, /* G */
				osd_oetf_41_2084_mapping_100, /* B */
				CSC_ON);
		} else if (is_hdr_cfg_osd_100() == 2) {
			set_vpp_lut(VPP_LUT_OSD_OETF,
				osd_oetf_41_2084_mapping_290, /* R */
				osd_oetf_41_2084_mapping_290, /* G */
				osd_oetf_41_2084_mapping_290, /* B */
				CSC_ON);
		} else if (is_hdr_cfg_osd_100() == 3) {
			set_vpp_lut(
				VPP_LUT_OSD_OETF,
				osd_oetf_41_2084_mapping_330, /* R */
				osd_oetf_41_2084_mapping_330, /* G */
				osd_oetf_41_2084_mapping_330, /* B */
				CSC_ON);
		} else {
			set_vpp_lut(VPP_LUT_OSD_OETF,
				osd_oetf_41_2084_mapping, /* R */
				osd_oetf_41_2084_mapping, /* G */
				osd_oetf_41_2084_mapping, /* B */
				CSC_ON);
		}
		/* osd matrix RGB2020 to YUV2020 limit */
		set_vpp_matrix(VPP_MATRIX_OSD,
			RGB2020_to_YUV2020l_coeff,
			CSC_ON);
	}
	/************** VIDEO **************/
	if ((get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
		&& (csc_en & 0x4)) {
		/* vd1 matrix bypass */
		set_vpp_matrix(VPP_MATRIX_VD1,
			bypass_coeff,
			CSC_OFF);

		/*INV LUT*/
		set_vpp_lut(VPP_LUT_INV_EOTF,
			NULL,
			NULL,
			NULL,
			CSC_OFF);

		/* post matrix YUV2020 to RGB2020 */
		set_vpp_matrix(VPP_MATRIX_POST,
			YUV2020l_to_RGB2020_coeff,
			CSC_ON);

		/* eotf lut 2048 */
		vpp_lut_curve_set(VPP_LUT_EOTF, vinfo);

		need_adjust_contrast_saturation = 0;
		saturation_offset =	0;
		if (hdr_flag & 8) {
			need_adjust_contrast_saturation |= 2;
			saturation_offset =	extra_sat_lut[0];
		}
		if (master_info->present_flag & 1) {
			max_lumin = master_info->luminance[0]
				/ 10000;
			if ((max_lumin <= 1200) && (max_lumin > 0)) {
				if (hdr_flag & 4)
					need_adjust_contrast_saturation |= 1;
				if (hdr_flag & 8)
					saturation_offset =
						extra_sat_lut[1];
			}
		}
		/* eotf matrix RGB2020 to RGB709 */
		mtx[EOTF_COEFF_SIZE - 1] = m.right_shift;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (m.matrix[i][j] & 0x1000)
					mtx[i * 3 + j] =
					-(((~m.matrix[i][j]) & 0xfff) + 1);
				else
					mtx[i * 3 + j] = m.matrix[i][j];
			}

		set_vpp_matrix(VPP_MATRIX_EOTF,
			mtx,
			CSC_ON);

		vpp_lut_curve_set(VPP_LUT_OETF, vinfo);

		/* xvyccc matrix3: bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444)
			set_vpp_matrix(VPP_MATRIX_XVYCC,
				RGB709_to_YUV709l_coeff,
				CSC_ON);
	}
	if (get_cpu_type() <= MESON_CPU_MAJOR_ID_GXTVBB) {
		/* turn vd1 matrix on */
		vpp_set_matrix(VPP_MATRIX_VD1, CSC_ON,
			csc_type, NULL);
		/* turn post matrix on */
		vpp_set_matrix(VPP_MATRIX_POST, CSC_ON,
			csc_type, &m);
		/* xvycc lut on */
		load_knee_lut(CSC_ON);

		vecm_latch_flag |= FLAG_VADJ1_BRI;
		hdr_process_pq_enable(0);
		/* if GXTVBB HDMI output(YUV) case */
		/* xvyccc matrix3: RGB to YUV */
		/* other cases */
		/* xvyccc matrix3: bypass */
		if ((vinfo->viu_color_fmt != COLOR_FMT_RGB444) &&
			(get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB))
			vpp_set_matrix3(CSC_ON, VPP_MATRIX_RGB_YUV709);
		else
			vpp_set_matrix3(CSC_OFF, VPP_MATRIX_NULL);
	}
	/*vpp matrix mux write*/
	vpp_set_mtx_en_write();
	return need_adjust_contrast_saturation;
}

static int hlg_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	int need_adjust_contrast_saturation = 0;
	int max_lumin = 10000;
	struct matrix_s m = {
		{0, 0, 0},
		{
			{0x0d49, 0x1b4d, 0x1f6b},
			{0x1f01, 0x0910, 0x1fef},
			{0x1fdb, 0x1f32, 0x08f3},
		},
		{0, 0, 0},
		1
	};
	struct matrix_s osd_m = {
		{0, 0, 0},
		{
			{0x505, 0x2A2, 0x059},
			{0x08E, 0x75B, 0x017},
			{0x022, 0x0B4, 0x72A},
		},
		{0, 0, 0},
		1
	};
	int mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(1.6607056/2), EOTF_COEFF_NORM(-0.5877533/2),
		EOTF_COEFF_NORM(-0.0729065/2),
		EOTF_COEFF_NORM(-0.1245575/2), EOTF_COEFF_NORM(1.1329346/2),
		EOTF_COEFF_NORM(-0.0083771/2),
		EOTF_COEFF_NORM(-0.0181122/2), EOTF_COEFF_NORM(-0.1005249/2),
		EOTF_COEFF_NORM(1.1186371/2),
		EOTF_COEFF_RIGHTSHIFT,
	};
	int osd_mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(0.627441),	EOTF_COEFF_NORM(0.329285),
		EOTF_COEFF_NORM(0.043274),
		EOTF_COEFF_NORM(0.069092),	EOTF_COEFF_NORM(0.919556),
		EOTF_COEFF_NORM(0.011322),
		EOTF_COEFF_NORM(0.016418),	EOTF_COEFF_NORM(0.088058),
		EOTF_COEFF_NORM(0.895554),
		EOTF_COEFF_RIGHTSHIFT
	};
	int i, j;

	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
		if (vd_path == VD1_PATH)
			hdr_func(VD1_HDR, HLG_SDR, vinfo);
		else
			hdr_func(VD2_HDR, HLG_SDR, vinfo);
		hdr_func(OSD1_HDR, HDR_BYPASS, vinfo);
		return need_adjust_contrast_saturation;
	}

	if (master_info->present_flag & 1) {
		pr_csc(1, "\tMaster_display_colour available.\n");
		print_primaries_info(master_info);
		/* for VIDEO */
		csc_type =
			prepare_customer_matrix(
				&master_info->primaries,
				&master_info->white_point,
				vinfo, &m, 0);
		/* for OSD */
		if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
			prepare_customer_matrix(
				&master_info->primaries,
				&master_info->white_point,
				vinfo, &osd_m, 1);
		need_adjust_contrast_saturation |= 1;
	} else {
		/* use bt2020 primaries */
		pr_csc(1, "\tNo master_display_colour.\n");
		/* for VIDEO */
		csc_type =
			prepare_customer_matrix(
			&bt2020_primaries,
			&bt2020_white_point,
			vinfo, &m, 0);
		/* for OSD */
		if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
			prepare_customer_matrix(
				&bt2020_primaries,
				&bt2020_white_point,
				vinfo, &osd_m, 1);
	}
	/*vpp matrix mux read*/
	vpp_set_mtx_en_read();
	if ((get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
	&& (csc_en & 0x2)) {
		/************** OSD ***************/
		/* RGB to YUV */
		/* not using old RGB2YUV convert HW */
		/* use new 10bit OSD convert matrix */
		/* WRITE_VPP_REG_BITS(VIU_OSD1_BLK0_CFG_W0, */
		/* 0, 7, 1); */
		/* eotf lut 709 */
		if (get_hdr_type() & HLG_FLAG)
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				osd_eotf_33_sdr2hlg_mapping, /* R */
				osd_eotf_33_sdr2hlg_mapping, /* G */
				osd_eotf_33_sdr2hlg_mapping, /* B */
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				osd_eotf_33_709_mapping, /* R */
				osd_eotf_33_709_mapping, /* G */
				osd_eotf_33_709_mapping, /* B */
				CSC_ON);

		/* eotf matrix 709->2020 */
		osd_mtx[EOTF_COEFF_SIZE - 1] = osd_m.right_shift;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (osd_m.matrix[i][j] & 0x1000)
					osd_mtx[i * 3 + j] =
					-(((~osd_m.matrix[i][j]) & 0xfff) + 1);
				else
					osd_mtx[i * 3 + j] = osd_m.matrix[i][j];
			}
		set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
			osd_mtx,
			CSC_ON);

		/* oetf lut 2084 */
		if (get_hdr_type() & HLG_FLAG)
			set_vpp_lut(VPP_LUT_OSD_OETF,
				osd_oetf_41_sdr2hlg_mapping, /* R */
				osd_oetf_41_sdr2hlg_mapping, /* G */
				osd_oetf_41_sdr2hlg_mapping, /* B */
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_OSD_OETF,
				osd_oetf_41_2084_mapping, /* R */
				osd_oetf_41_2084_mapping, /* G */
				osd_oetf_41_2084_mapping, /* B */
				CSC_ON);

		/* osd matrix RGB2020 to YUV2020 limit */
		set_vpp_matrix(VPP_MATRIX_OSD,
			RGB2020_to_YUV2020l_coeff,
			CSC_ON);
	}
	/************** VIDEO **************/
	if ((get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
		&& (csc_en & 0x4)) {
		/* vd1 matrix bypass */
		set_vpp_matrix(VPP_MATRIX_VD1,
			bypass_coeff,
			CSC_OFF);

		/*eo oo oe*/
		set_vpp_lut(VPP_LUT_INV_EOTF,
			NULL,
			NULL,
			NULL,
			CSC_OFF);

		/* post matrix YUV2020 to RGB2020 */
		set_vpp_matrix(VPP_MATRIX_POST,
			YUV2020l_to_RGB2020_coeff,
			CSC_ON);

		/* eotf lut 2048 */
		set_vpp_lut(VPP_LUT_EOTF,
			eotf_33_hlg_mapping, /* R */
			eotf_33_hlg_mapping, /* G */
			eotf_33_hlg_mapping, /* B */
			CSC_ON);

		need_adjust_contrast_saturation = 0;
		saturation_offset =	0;
		if (hdr_flag & 8) {
			need_adjust_contrast_saturation |= 2;
			saturation_offset =	extra_sat_lut[0];
		}
		if (master_info->present_flag & 1) {
			max_lumin = master_info->luminance[0]
				/ 10000;
			if ((max_lumin <= 1200) && (max_lumin > 0)) {
				if (hdr_flag & 4)
					need_adjust_contrast_saturation |= 1;
				if (hdr_flag & 8)
					saturation_offset =
						extra_sat_lut[1];
			}
		}
		/* eotf matrix RGB2020 to RGB709 */
		mtx[EOTF_COEFF_SIZE - 1] = m.right_shift;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (m.matrix[i][j] & 0x1000)
					mtx[i * 3 + j] =
					-(((~m.matrix[i][j]) & 0xfff) + 1);
				else
					mtx[i * 3 + j] = m.matrix[i][j];
			}

		set_vpp_matrix(VPP_MATRIX_EOTF,
			mtx,
			CSC_ON);

		set_vpp_lut(VPP_LUT_OETF,
			oetf_289_gamma22_mapping,
			oetf_289_gamma22_mapping,
			oetf_289_gamma22_mapping,
			CSC_ON);

		/* xvyccc matrix3: bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444)
			set_vpp_matrix(VPP_MATRIX_XVYCC,
				RGB709_to_YUV709l_coeff,
				CSC_ON);
	}
	if (get_cpu_type() <= MESON_CPU_MAJOR_ID_GXTVBB) {
		/* turn vd1 matrix on */
		vpp_set_matrix(VPP_MATRIX_VD1, CSC_ON,
			csc_type, NULL);
		/* turn post matrix on */
		vpp_set_matrix(VPP_MATRIX_POST, CSC_ON,
			csc_type, &m);
		/* xvycc lut on */
		load_knee_lut(CSC_ON);

		vecm_latch_flag |= FLAG_VADJ1_BRI;
		hdr_process_pq_enable(0);
		/* if GXTVBB HDMI output(YUV) case */
		/* xvyccc matrix3: RGB to YUV */
		/* other cases */
		/* xvyccc matrix3: bypass */
		if ((vinfo->viu_color_fmt != COLOR_FMT_RGB444) &&
			(get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB))
			vpp_set_matrix3(CSC_ON, VPP_MATRIX_RGB_YUV709);
		else
			vpp_set_matrix3(CSC_OFF, VPP_MATRIX_NULL);
	}
	/*vpp matrix mux write*/
	vpp_set_mtx_en_write();
	return need_adjust_contrast_saturation;
}

static void bypass_hdr_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	struct matrix_s osd_m = {
		{0, 0, 0},
		{
			{0x505, 0x2A2, 0x059},
			{0x08E, 0x75B, 0x017},
			{0x022, 0x0B4, 0x72A},
		},
		{0, 0, 0},
		1
	};
	int osd_mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(0.627441),	EOTF_COEFF_NORM(0.329285),
		EOTF_COEFF_NORM(0.043274),
		EOTF_COEFF_NORM(0.069092),	EOTF_COEFF_NORM(0.919556),
		EOTF_COEFF_NORM(0.011322),
		EOTF_COEFF_NORM(0.016418),	EOTF_COEFF_NORM(0.088058),
		EOTF_COEFF_NORM(0.895554),
		EOTF_COEFF_RIGHTSHIFT
	};
	int i, j;

	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
		if (vd_path == VD1_PATH)
			hdr_func(VD1_HDR, HDR_BYPASS, vinfo);
		else
			hdr_func(VD2_HDR, HDR_BYPASS, vinfo);
		if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			((sink_hdr_support(vinfo) &
			(HDR_SUPPORT | HLG_SUPPORT)) &&
			(vinfo->viu_color_fmt != COLOR_FMT_RGB444))) {
			if (get_hdr_type() & HLG_FLAG)
				hdr_func(OSD1_HDR, SDR_HLG, vinfo);
			else
				hdr_func(OSD1_HDR, SDR_HDR, vinfo);
			pr_csc(1, "\t osd sdr->hdr/hlg\n");
		} else
			hdr_func(OSD1_HDR, HDR_BYPASS, vinfo);
		return;
	}
	/*vpp matrix mux read*/
	vpp_set_mtx_en_read();
	if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB) {
		/************** OSD ***************/
		/* RGB to YUV */
		/* not using old RGB2YUV convert HW */
		/* use new 10bit OSD convert matrix */
		/* WRITE_VPP_REG_BITS*/
		/*(VIU_OSD1_BLK0_CFG_W0,0, 7, 1);*/
		if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
			(vinfo->viu_color_fmt != COLOR_FMT_RGB444))) {
			/* OSD convert to HDR to match HDR video */
			/* osd eotf lut 709 */
			if (get_hdr_type() & HLG_FLAG) {
				set_vpp_lut(VPP_LUT_OSD_EOTF,
					osd_eotf_33_sdr2hlg_mapping, /* R */
					osd_eotf_33_sdr2hlg_mapping, /* G */
					osd_eotf_33_sdr2hlg_mapping, /* B */
					CSC_ON);
			} else {
				if (is_hdr_cfg_osd_100() == 1) {
					set_vpp_lut(VPP_LUT_OSD_EOTF,
						osd_eotf_33_709_mapping_100,
						osd_eotf_33_709_mapping_100,
						osd_eotf_33_709_mapping_100,
						CSC_ON);
				} else if (is_hdr_cfg_osd_100() == 2) {
					set_vpp_lut(VPP_LUT_OSD_EOTF,
						osd_eotf_33_709_mapping_290,
						osd_eotf_33_709_mapping_290,
						osd_eotf_33_709_mapping_290,
						CSC_ON);
				} else if (is_hdr_cfg_osd_100() == 3) {
					set_vpp_lut(
						VPP_LUT_OSD_EOTF,
						osd_eotf_33_709_mapping_330,
						osd_eotf_33_709_mapping_330,
						osd_eotf_33_709_mapping_330,
						CSC_ON);
				} else {
					set_vpp_lut(
						VPP_LUT_OSD_EOTF,
						osd_eotf_33_709_mapping, /*R*/
						osd_eotf_33_709_mapping, /*G*/
						osd_eotf_33_709_mapping, /*B*/
						CSC_ON);
				}
			}
			/* osd eotf matrix 709->2020 */
			if (master_info->present_flag & 1) {
				pr_csc(1,
					"\tMaster_display_colour available.\n");
				print_primaries_info(master_info);
				prepare_customer_matrix(
					&bt2020_primaries,
					&bt2020_white_point,
					vinfo, &osd_m, 1);
			} else {
				pr_csc(1, "\tNo master_display_colour.\n");
				prepare_customer_matrix(
					&bt2020_primaries,
					&bt2020_white_point,
					vinfo, &osd_m, 1);
			}
			osd_mtx[EOTF_COEFF_SIZE - 1] = osd_m.right_shift;
			for (i = 0; i < 3; i++)
				for (j = 0; j < 3; j++) {
					if (osd_m.matrix[i][j] & 0x1000) {
						osd_mtx[i * 3 + j] =
						(~osd_m.matrix[i][j]) & 0xfff;
						osd_mtx[i * 3 + j] =
						-(1 + osd_mtx[i * 3 + j]);
					} else
						osd_mtx[i * 3 + j] =
							osd_m.matrix[i][j];
				}
			set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
				osd_mtx,
				CSC_ON);

			/* osd oetf lut 2084 */
			if (get_hdr_type() & HLG_FLAG) {
				set_vpp_lut(VPP_LUT_OSD_OETF,
					osd_oetf_41_sdr2hlg_mapping, /* R */
					osd_oetf_41_sdr2hlg_mapping, /* G */
					osd_oetf_41_sdr2hlg_mapping, /* B */
					CSC_ON);
			} else {
				if (is_hdr_cfg_osd_100() == 1) {
					set_vpp_lut(VPP_LUT_OSD_OETF,
						osd_oetf_41_2084_mapping_100,
						osd_oetf_41_2084_mapping_100,
						osd_oetf_41_2084_mapping_100,
						CSC_ON);
				} else if (is_hdr_cfg_osd_100() == 2) {
					set_vpp_lut(VPP_LUT_OSD_OETF,
						osd_oetf_41_2084_mapping_290,
						osd_oetf_41_2084_mapping_290,
						osd_oetf_41_2084_mapping_290,
						CSC_ON);
				} else if (is_hdr_cfg_osd_100() == 3) {
					set_vpp_lut(
						VPP_LUT_OSD_OETF,
						osd_oetf_41_2084_mapping_330,
						osd_oetf_41_2084_mapping_330,
						osd_oetf_41_2084_mapping_330,
						CSC_ON);
				} else {
					set_vpp_lut(VPP_LUT_OSD_OETF,
						osd_oetf_41_2084_mapping,
						osd_oetf_41_2084_mapping,
						osd_oetf_41_2084_mapping,
						CSC_ON);
				}
			}
			/* osd matrix RGB2020 to YUV2020 limit */
			set_vpp_matrix(VPP_MATRIX_OSD,
				RGB2020_to_YUV2020l_coeff,
				CSC_ON);
		} else {
			/* OSD convert to 709 limited to match SDR video */
			/* eotf lut bypass */
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				eotf_33_linear_mapping, /* R */
				eotf_33_linear_mapping, /* G */
				eotf_33_linear_mapping, /* B */
				CSC_OFF);

			/* eotf matrix bypass */
			set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
				eotf_bypass_coeff,
				CSC_OFF);

			/* oetf lut bypass */
			set_vpp_lut(VPP_LUT_OSD_OETF,
				oetf_41_linear_mapping, /* R */
				oetf_41_linear_mapping, /* G */
				oetf_41_linear_mapping, /* B */
				CSC_OFF);

			/* osd matrix RGB709 to YUV709 limit/full */
			if (range_control)
				set_vpp_matrix(VPP_MATRIX_OSD,
					RGB709_to_YUV709_coeff,
					CSC_ON);	/* use full range */
			else
				set_vpp_matrix(VPP_MATRIX_OSD,
					RGB709_to_YUV709l_coeff,
					CSC_ON);	/* use limit range */
		}

		/************** VIDEO **************/
		/* vd1 matrix: bypass */
		if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)
			set_vpp_matrix(VPP_MATRIX_VD1,
				bypass_coeff,
				CSC_OFF);	/* limit->limit range */
		else {
			if (vinfo->viu_color_fmt == COLOR_FMT_RGB444) {
				if (signal_range == 0) {/* limit range */
					if (csc_type == VPP_MATRIX_YUV601_RGB)
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						YUV601l_to_YUV709f_coeff :
						YUV601l_to_YUV709l_coeff,
						CSC_ON);
					else
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						YUV709l_to_YUV709f_coeff :
						bypass_coeff,
						CSC_ON);
				} else {
					if (csc_type == VPP_MATRIX_YUV601_RGB)
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						YUV601f_to_YUV709f_coeff :
						YUV601f_to_YUV709l_coeff,
						CSC_ON);
					else
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						bypass_coeff :
						YUV709f_to_YUV709l_coeff,
						CSC_ON);
				}
			} else {
				/*default limit range */
				if (vd1_mtx_sel == VPP_MATRIX_NULL)
					set_vpp_matrix(VPP_MATRIX_VD1,
						bypass_coeff,
						CSC_OFF);
				else if (vd1_mtx_sel ==
					VPP_MATRIX_YUV601L_YUV709L)
					set_vpp_matrix(VPP_MATRIX_VD1,
					YUV601l_to_YUV709l_coeff,
					CSC_ON);
				else if (vd1_mtx_sel ==
					VPP_MATRIX_YUV709L_YUV601L)
					set_vpp_matrix(VPP_MATRIX_VD1,
						YUV709l_to_YUV601l_coeff,
						CSC_ON);
			}
		}

		/* post matrix bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444)
			/* yuv2rgb for eye protect mode */
			set_vpp_matrix(VPP_MATRIX_POST,
				bypass_coeff,
				CSC_OFF);
		else {/* matrix yuv2rgb for LCD */
			if (range_control)
				set_vpp_matrix(VPP_MATRIX_POST,
					YUV709f_to_RGB709_coeff,
					CSC_ON);
			else
				set_vpp_matrix(VPP_MATRIX_POST,
					YUV709l_to_RGB709_coeff,
					CSC_ON);
		}
		/* xvycc inv lut */
		if (sdr_process_mode[vd_path] &&
		(csc_type < VPP_MATRIX_BT2020YUV_BT2020RGB) &&
		((get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB) ||
		 (get_cpu_type() == MESON_CPU_MAJOR_ID_TXL)))
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_OFF);

		/* eotf lut bypass */
		set_vpp_lut(VPP_LUT_EOTF,
			NULL, /* R */
			NULL, /* G */
			NULL, /* B */
			CSC_OFF);

		/* eotf matrix bypass */
		set_vpp_matrix(VPP_MATRIX_EOTF,
			eotf_bypass_coeff,
			CSC_OFF);

		/* oetf lut bypass */
		set_vpp_lut(VPP_LUT_OETF,
			NULL,
			NULL,
			NULL,
			CSC_OFF);

		/* xvycc matrix full2limit or bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
			if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)
				set_vpp_matrix(VPP_MATRIX_XVYCC,
					bypass_coeff,
					CSC_OFF);
			else {
				if (range_control)
					set_vpp_matrix(VPP_MATRIX_XVYCC,
						YUV709f_to_YUV709l_coeff,
						CSC_ON);
				else
					set_vpp_matrix(VPP_MATRIX_XVYCC,
						bypass_coeff,
						CSC_OFF);
			}
		}
	} else {
		/* OSD */
		/* keep RGB */

		/* VIDEO */
		if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) {
			/* vd1 matrix: convert YUV to RGB */
			csc_type = VPP_MATRIX_YUV709_RGB;
		}
		/* vd1 matrix on to convert YUV to RGB */
		vpp_set_matrix(VPP_MATRIX_VD1, CSC_ON,
			csc_type, NULL);
		/* post matrix off */
		vpp_set_matrix(VPP_MATRIX_POST, CSC_OFF,
			csc_type, NULL);
		/* xvycc lut off */
		load_knee_lut(CSC_OFF);
		/* xvycc inv lut */

		if (sdr_process_mode[vd_path])
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_OFF);

		vecm_latch_flag |= FLAG_VADJ1_BRI;
		hdr_process_pq_enable(1);
		/* if GXTVBB HDMI output(YUV) case */
		/* xvyccc matrix3: RGB to YUV */
		/* other cases */
		/* xvyccc matrix3: bypass */
		if ((vinfo->viu_color_fmt != COLOR_FMT_RGB444) &&
			(get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB))
			vpp_set_matrix3(CSC_ON, VPP_MATRIX_RGB_YUV709);
		else
			vpp_set_matrix3(CSC_OFF, VPP_MATRIX_NULL);
	}
	/*vpp matrix mux write*/
	vpp_set_mtx_en_write();
}

static void set_bt2020csc_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	struct matrix_s osd_m = {
		{0, 0, 0},
		{
			{0x505, 0x2A2, 0x059},
			{0x08E, 0x75B, 0x017},
			{0x022, 0x0B4, 0x72A},
		},
		{0, 0, 0},
		1
	};
	int osd_mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(0.627441),	EOTF_COEFF_NORM(0.329285),
		EOTF_COEFF_NORM(0.043274),
		EOTF_COEFF_NORM(0.069092),	EOTF_COEFF_NORM(0.919556),
		EOTF_COEFF_NORM(0.011322),
		EOTF_COEFF_NORM(0.016418),	EOTF_COEFF_NORM(0.088058),
		EOTF_COEFF_NORM(0.895554),
		EOTF_COEFF_RIGHTSHIFT
	};
	int i, j;

	/*vpp matrix mux read*/
	vpp_set_mtx_en_read();
	if (get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB) {
		/************** OSD ***************/
		/* RGB to YUV */
		/* not using old RGB2YUV convert HW */
		/* use new 10bit OSD convert matrix */
		/* WRITE_VPP_REG_BITS*/
		/*(VIU_OSD1_BLK0_CFG_W0,0, 7, 1);*/
		if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
			(vinfo->viu_color_fmt != COLOR_FMT_RGB444))) {
			/* OSD convert to HDR to match HDR video */
			/* osd eotf lut 709 */
			if (get_hdr_type() & HLG_FLAG)
				set_vpp_lut(VPP_LUT_OSD_EOTF,
					osd_eotf_33_sdr2hlg_mapping, /* R */
					osd_eotf_33_sdr2hlg_mapping, /* G */
					osd_eotf_33_sdr2hlg_mapping, /* B */
					CSC_ON);
			else
				set_vpp_lut(VPP_LUT_OSD_EOTF,
					osd_eotf_33_709_mapping, /* R */
					osd_eotf_33_709_mapping, /* G */
					osd_eotf_33_709_mapping, /* B */
					CSC_ON);

			/* osd eotf matrix 709->2020 */
			if (master_info->present_flag & 1) {
				pr_csc(1,
					"\tMaster_display_colour available.\n");
				print_primaries_info(master_info);
				prepare_customer_matrix(
					&master_info->primaries,
					&master_info->white_point,
					vinfo, &osd_m, 1);
			} else {
				pr_csc(1, "\tNo master_display_colour.\n");
				prepare_customer_matrix(
					&bt2020_primaries,
					&bt2020_white_point,
					vinfo, &osd_m, 1);
			}
			osd_mtx[EOTF_COEFF_SIZE - 1] = osd_m.right_shift;
			for (i = 0; i < 3; i++)
				for (j = 0; j < 3; j++) {
					if (osd_m.matrix[i][j] & 0x1000) {
						osd_mtx[i * 3 + j] =
						(~osd_m.matrix[i][j]) & 0xfff;
						osd_mtx[i * 3 + j] =
						-(1 + osd_mtx[i * 3 + j]);
					} else
						osd_mtx[i * 3 + j] =
							osd_m.matrix[i][j];
				}
			set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
				osd_mtx,
				CSC_ON);

			/* osd oetf lut 2084 */
			if (get_hdr_type() & HLG_FLAG)
				set_vpp_lut(VPP_LUT_OSD_OETF,
					osd_oetf_41_sdr2hlg_mapping, /* R */
					osd_oetf_41_sdr2hlg_mapping, /* G */
					osd_oetf_41_sdr2hlg_mapping, /* B */
					CSC_ON);
			else
				set_vpp_lut(VPP_LUT_OSD_OETF,
					osd_oetf_41_2084_mapping, /* R */
					osd_oetf_41_2084_mapping, /* G */
					osd_oetf_41_2084_mapping, /* B */
					CSC_ON);

			/* osd matrix RGB2020 to YUV2020 limit */
			set_vpp_matrix(VPP_MATRIX_OSD,
				RGB2020_to_YUV2020l_coeff,
				CSC_ON);
		} else {
			/* OSD convert to 709 limited to match SDR video */
			/* eotf lut bypass */
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				eotf_33_linear_mapping, /* R */
				eotf_33_linear_mapping, /* G */
				eotf_33_linear_mapping, /* B */
				CSC_OFF);

			/* eotf matrix bypass */
			set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
				eotf_bypass_coeff,
				CSC_OFF);

			/* oetf lut bypass */
			set_vpp_lut(VPP_LUT_OSD_OETF,
				oetf_41_linear_mapping, /* R */
				oetf_41_linear_mapping, /* G */
				oetf_41_linear_mapping, /* B */
				CSC_OFF);

			/* osd matrix RGB709 to YUV709 limit/full */
			if (range_control)
				set_vpp_matrix(VPP_MATRIX_OSD,
					RGB709_to_YUV709_coeff,
					CSC_ON);	/* use full range */
			else
				set_vpp_matrix(VPP_MATRIX_OSD,
					RGB709_to_YUV709l_coeff,
					CSC_ON);	/* use limit range */
		}

		/************** VIDEO **************/
		/* vd1 matrix: bypass */
		if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)
			set_vpp_matrix(VPP_MATRIX_VD1,
				bypass_coeff,
				CSC_OFF);	/* limit->limit range */
		else {
			if (vinfo->viu_color_fmt == COLOR_FMT_RGB444) {
				if (signal_range == 0) {/* limit range */
					if (csc_type == VPP_MATRIX_YUV601_RGB)
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						YUV601l_to_YUV709f_coeff :
						YUV601l_to_YUV709l_coeff,
						CSC_ON);
					else
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						YUV709l_to_YUV709f_coeff :
						bypass_coeff,
						CSC_ON);
				} else {
					if (csc_type == VPP_MATRIX_YUV601_RGB)
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						YUV601f_to_YUV709f_coeff :
						YUV601f_to_YUV709l_coeff,
						CSC_ON);
					else
						set_vpp_matrix(VPP_MATRIX_VD1,
						range_control ?
						bypass_coeff :
						YUV709f_to_YUV709l_coeff,
						CSC_ON);
				}
			} else {
				/*default limit range */
				if (vd1_mtx_sel == VPP_MATRIX_NULL)
					set_vpp_matrix(VPP_MATRIX_VD1,
						bypass_coeff,
						CSC_OFF);
				else if (vd1_mtx_sel ==
					VPP_MATRIX_YUV601L_YUV709L)
					set_vpp_matrix(VPP_MATRIX_VD1,
					YUV601l_to_YUV709l_coeff,
					CSC_ON);
				else if (vd1_mtx_sel ==
					VPP_MATRIX_YUV709L_YUV601L)
					set_vpp_matrix(VPP_MATRIX_VD1,
						YUV709l_to_YUV601l_coeff,
						CSC_ON);
			}
		}

		/* post matrix bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444)
			/* yuv2rgb for eye protect mode */
			set_vpp_matrix(VPP_MATRIX_POST,
				YUV709l_to_YUV2020_coeff,
				CSC_ON);
		else {/* matrix yuv2rgb for LCD */
			if (range_control)
				set_vpp_matrix(VPP_MATRIX_POST,
					YUV709f_to_RGB709_coeff,
					CSC_ON);
			else
				set_vpp_matrix(VPP_MATRIX_POST,
					YUV709l_to_RGB709_coeff,
					CSC_ON);
		}
		/* xvycc inv lut */
		if (sdr_process_mode[vd_path] &&
		(csc_type < VPP_MATRIX_BT2020YUV_BT2020RGB) &&
		((get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB) ||
		 (get_cpu_type() == MESON_CPU_MAJOR_ID_TXL)))
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_OFF);

		/* eotf lut bypass */
		set_vpp_lut(VPP_LUT_EOTF,
			NULL, /* R */
			NULL, /* G */
			NULL, /* B */
			CSC_OFF);

		/* eotf matrix bypass */
		set_vpp_matrix(VPP_MATRIX_EOTF,
			bypass_coeff,
			CSC_OFF);

		/* oetf lut bypass */
		set_vpp_lut(VPP_LUT_OETF,
			NULL,
			NULL,
			NULL,
			CSC_OFF);

		/* xvycc matrix full2limit or bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
			if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)
				set_vpp_matrix(VPP_MATRIX_XVYCC,
					bypass_coeff,
					CSC_ON);
			else
				set_vpp_matrix(VPP_MATRIX_XVYCC,
					bypass_coeff,
					CSC_OFF);
		}
	} else {
		/* OSD */
		/* keep RGB */

		/* VIDEO */
		if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) {
			/* vd1 matrix: convert YUV to RGB */
			csc_type = VPP_MATRIX_YUV709_RGB;
		}
		/* vd1 matrix on to convert YUV to RGB */
		vpp_set_matrix(VPP_MATRIX_VD1, CSC_ON,
			csc_type, NULL);
		/* post matrix off */
		vpp_set_matrix(VPP_MATRIX_POST, CSC_OFF,
			csc_type, NULL);
		/* xvycc lut off */
		load_knee_lut(CSC_OFF);
		/* xvycc inv lut */

		if (sdr_process_mode[vd_path])
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_OFF);

		vecm_latch_flag |= FLAG_VADJ1_BRI;
		hdr_process_pq_enable(1);
		/* if GXTVBB HDMI output(YUV) case */
		/* xvyccc matrix3: RGB to YUV */
		/* other cases */
		/* xvyccc matrix3: bypass */
		if ((vinfo->viu_color_fmt != COLOR_FMT_RGB444) &&
			(get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB))
			vpp_set_matrix3(CSC_ON, VPP_MATRIX_RGB_YUV709);
		else
			vpp_set_matrix3(CSC_OFF, VPP_MATRIX_NULL);
	}
	/*vpp matrix mux write*/
	vpp_set_mtx_en_write();
}

static void hlg_hdr_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	struct matrix_s osd_m = {
		{0, 0, 0},
		{
			{0x505, 0x2A2, 0x059},
			{0x08E, 0x75B, 0x017},
			{0x022, 0x0B4, 0x72A},
		},
		{0, 0, 0},
		1
	};
	int osd_mtx[EOTF_COEFF_SIZE] = {
		EOTF_COEFF_NORM(0.627441),	EOTF_COEFF_NORM(0.329285),
		EOTF_COEFF_NORM(0.043274),
		EOTF_COEFF_NORM(0.069092),	EOTF_COEFF_NORM(0.919556),
		EOTF_COEFF_NORM(0.011322),
		EOTF_COEFF_NORM(0.016418),	EOTF_COEFF_NORM(0.088058),
		EOTF_COEFF_NORM(0.895554),
		EOTF_COEFF_RIGHTSHIFT
	};
	int i, j;

	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
		if (vd_path == VD1_PATH)
			hdr_func(VD1_HDR, HLG_HDR, vinfo);
		else if (vd_path == VD2_PATH)
			hdr_func(VD2_HDR, HLG_HDR, vinfo);
		hdr_func(OSD1_HDR, SDR_HDR, vinfo);
		return;
	}
	/*vpp matrix mux read*/
	vpp_set_mtx_en_read();
	if ((get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
	&& (csc_en & 0x2)) {
		/************** OSD ***************/
		/* RGB to YUV */
		/* not using old RGB2YUV convert HW */
		/* use new 10bit OSD convert matrix */
		/* WRITE_VPP_REG_BITS(VIU_OSD1_BLK0_CFG_W0, */
		/* 0, 7, 1); */
		if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
			(vinfo->viu_color_fmt != COLOR_FMT_RGB444))) {
			/* OSD convert to HDR to match HDR video */
			/* osd eotf lut 709 */
			if (get_hdr_type() & HLG_FLAG)
				set_vpp_lut(VPP_LUT_OSD_EOTF,
					osd_eotf_33_sdr2hlg_mapping, /* R */
					osd_eotf_33_sdr2hlg_mapping, /* G */
					osd_eotf_33_sdr2hlg_mapping, /* B */
					CSC_ON);
			else
				set_vpp_lut(VPP_LUT_OSD_EOTF,
					osd_eotf_33_709_mapping, /* R */
					osd_eotf_33_709_mapping, /* G */
					osd_eotf_33_709_mapping, /* B */
					CSC_ON);

			/* osd eotf matrix 709->2020 */
			if (master_info->present_flag & 1) {
				pr_csc(1,
					"\tMaster_display_colour available.\n");
				print_primaries_info(master_info);
				prepare_customer_matrix(
					&master_info->primaries,
					&master_info->white_point,
					vinfo, &osd_m, 1);
			} else {
				pr_csc(1, "\tNo master_display_colour.\n");
				prepare_customer_matrix(
					&bt2020_primaries,
					&bt2020_white_point,
					vinfo, &osd_m, 1);
			}
			osd_mtx[EOTF_COEFF_SIZE - 1] = osd_m.right_shift;
			for (i = 0; i < 3; i++)
				for (j = 0; j < 3; j++) {
					if (osd_m.matrix[i][j] & 0x1000) {
						osd_mtx[i * 3 + j] =
						(~osd_m.matrix[i][j]) & 0xfff;
						osd_mtx[i * 3 + j] =
						-(1 + osd_mtx[i * 3 + j]);
					} else
						osd_mtx[i * 3 + j] =
							osd_m.matrix[i][j];
				}
			set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
				osd_mtx,
				CSC_ON);

			/* osd oetf lut 2084 */
			if (get_hdr_type() & HLG_FLAG)
				set_vpp_lut(VPP_LUT_OSD_OETF,
					osd_oetf_41_sdr2hlg_mapping, /* R */
					osd_oetf_41_sdr2hlg_mapping, /* G */
					osd_oetf_41_sdr2hlg_mapping, /* B */
					CSC_ON);
			else
				set_vpp_lut(VPP_LUT_OSD_OETF,
					osd_oetf_41_2084_mapping, /* R */
					osd_oetf_41_2084_mapping, /* G */
					osd_oetf_41_2084_mapping, /* B */
					CSC_ON);

			/* osd matrix RGB2020 to YUV2020 limit */
			set_vpp_matrix(VPP_MATRIX_OSD,
				RGB2020_to_YUV2020l_coeff,
				CSC_ON);
		} else {
			/* OSD convert to 709 limited to match SDR video */
			/* eotf lut bypass */
			set_vpp_lut(VPP_LUT_OSD_EOTF,
				eotf_33_linear_mapping, /* R */
				eotf_33_linear_mapping, /* G */
				eotf_33_linear_mapping, /* B */
				CSC_OFF);

			/* eotf matrix bypass */
			set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
				eotf_bypass_coeff,
				CSC_OFF);

			/* oetf lut bypass */
			set_vpp_lut(VPP_LUT_OSD_OETF,
				oetf_41_linear_mapping, /* R */
				oetf_41_linear_mapping, /* G */
				oetf_41_linear_mapping, /* B */
				CSC_OFF);

			/* osd matrix RGB709 to YUV709 limit/full */
			if (range_control)
				set_vpp_matrix(VPP_MATRIX_OSD,
					RGB709_to_YUV709_coeff,
					CSC_ON);	/* use full range */
			else
				set_vpp_matrix(VPP_MATRIX_OSD,
					RGB709_to_YUV709l_coeff,
					CSC_ON);	/* use limit range */
		}
	}
	/************** VIDEO **************/
	if ((get_cpu_type() > MESON_CPU_MAJOR_ID_GXTVBB)
	&& (csc_en & 0x4)) {
		/* vd1 matrix: bypass */
		if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)
			set_vpp_matrix(VPP_MATRIX_VD1,
				bypass_coeff,
				CSC_OFF);	/* limit->limit range */
		else {
			if (signal_range == 0) /* limit range */
				set_vpp_matrix(VPP_MATRIX_VD1,
					bypass_coeff,
					CSC_OFF);
				/* limit->limit range */
			else
				set_vpp_matrix(VPP_MATRIX_VD1,
					YUV709f_to_YUV709l_coeff,
					CSC_ON);
		}
		/*eo oo oe*/
			int_lut_sel[0] |= INVLUT_HLG;
		set_vpp_lut(VPP_LUT_INV_EOTF,
			int_lut_sel,
			int_lut_sel,
			int_lut_sel,
			CSC_ON);

		/* post matrix bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444)
			/* yuv2rgb for eye protect mode */
			set_vpp_matrix(VPP_MATRIX_POST,
				YUV2020l_to_RGB2020_coeff,
				CSC_ON);
		else /* matrix yuv2rgb for LCD */
			set_vpp_matrix(VPP_MATRIX_POST,
				YUV709l_to_RGB709_coeff,
				CSC_ON);

		/* eotf lut bypass */
		set_vpp_lut(VPP_LUT_EOTF,
			eotf_33_hlg_mapping, /* R */
			eotf_33_hlg_mapping, /* G */
			eotf_33_hlg_mapping, /* B */
			CSC_ON);

		/* eotf matrix bypass */
		set_vpp_matrix(VPP_MATRIX_EOTF,
			eotf_bypass_coeff,
			CSC_ON);

		/* oetf lut bypass */
		set_vpp_lut(VPP_LUT_OETF,
			oetf_289_2084_mapping,
			oetf_289_2084_mapping,
			oetf_289_2084_mapping,
			CSC_ON);

		/* xvycc matrix full2limit or bypass */
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
			if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)
				set_vpp_matrix(VPP_MATRIX_XVYCC,
					RGB2020_to_YUV2020l_coeff,
					CSC_ON);
			else {
				if (range_control)
					set_vpp_matrix(VPP_MATRIX_XVYCC,
						YUV709f_to_YUV709l_coeff,
						CSC_ON);
				else
					set_vpp_matrix(VPP_MATRIX_XVYCC,
						bypass_coeff,
						CSC_OFF);
			}
		}
	} else {
		/* OSD */
		/* keep RGB */

		/* VIDEO */
		if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) {
			/* vd1 matrix: convert YUV to RGB */
			csc_type = VPP_MATRIX_YUV709_RGB;
		}
		/* vd1 matrix on to convert YUV to RGB */
		vpp_set_matrix(VPP_MATRIX_VD1, CSC_ON,
			csc_type, NULL);
		/* post matrix off */
		vpp_set_matrix(VPP_MATRIX_POST, CSC_OFF,
			csc_type, NULL);
		/* xvycc lut off */
		load_knee_lut(CSC_OFF);
		/* xvycc inv lut */

		if (sdr_process_mode[vd_path])
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_ON);
		else
			set_vpp_lut(VPP_LUT_INV_EOTF,
				NULL,
				NULL,
				NULL,
				CSC_OFF);

		vecm_latch_flag |= FLAG_VADJ1_BRI;
		hdr_process_pq_enable(1);
		/* if GXTVBB HDMI output(YUV) case */
		/* xvyccc matrix3: RGB to YUV */
		/* other cases */
		/* xvyccc matrix3: bypass */
		if ((vinfo->viu_color_fmt != COLOR_FMT_RGB444) &&
			(get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB))
			vpp_set_matrix3(CSC_ON, VPP_MATRIX_RGB_YUV709);
		else
			vpp_set_matrix3(CSC_OFF, VPP_MATRIX_NULL);
	}
	/*vpp matrix mux write*/
	vpp_set_mtx_en_write();
}

static void sdr_hdr_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *master_info,
	enum vd_path_e vd_path)
{
	if (vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
		if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
			if (vd_path == VD1_PATH)
				hdr_func(VD1_HDR, SDR_HDR, vinfo);
			else
				hdr_func(VD2_HDR, SDR_HDR, vinfo);
			hdr_func(OSD1_HDR, SDR_HDR, vinfo);
			return;
		}
		/*vpp matrix mux read*/
		vpp_set_mtx_en_read();
		/* OSD convert to 709 limited to match SDR video */
		/* eotf lut bypass */
		set_vpp_lut(VPP_LUT_OSD_EOTF,
			eotf_33_linear_mapping, /* R */
			eotf_33_linear_mapping, /* G */
			eotf_33_linear_mapping, /* B */
			CSC_OFF);

		/* eotf matrix bypass */
		set_vpp_matrix(VPP_MATRIX_OSD_EOTF,
			eotf_bypass_coeff,
			CSC_OFF);

		/* oetf lut bypass */
		set_vpp_lut(VPP_LUT_OSD_OETF,
			oetf_41_linear_mapping, /* R */
			oetf_41_linear_mapping, /* G */
			oetf_41_linear_mapping, /* B */
			CSC_OFF);

		/* osd matrix RGB709 to YUV709 limit/full */
		if (range_control)
			set_vpp_matrix(VPP_MATRIX_OSD,
				RGB709_to_YUV709_coeff,
				CSC_ON);	/* use full range */
		else
			set_vpp_matrix(VPP_MATRIX_OSD,
				RGB709_to_YUV709l_coeff,
				CSC_ON);	/* use limit range */

		/************** VIDEO **************/
		/* convert SDR Video to HDR */
		if (range_control) {
			if (signal_range == 0) /* limit range */
				set_vpp_matrix(VPP_MATRIX_VD1,
					YUV709l_to_YUV709f_coeff,
					CSC_ON);	/* limit->full range */
			else
				set_vpp_matrix(VPP_MATRIX_VD1,
					bypass_coeff,
					CSC_OFF);	/* full->full range */
		} else {
			if (signal_range == 0) /* limit range */
				set_vpp_matrix(VPP_MATRIX_VD1,
					bypass_coeff,
					CSC_OFF);	/* limit->limit range */
			else
				set_vpp_matrix(VPP_MATRIX_VD1,
					YUV709f_to_YUV709l_coeff,
					CSC_ON);	/* full->limit range */
		}

		set_vpp_matrix(VPP_MATRIX_POST,
			YUV709l_to_RGB709_coeff,
			CSC_ON);

		/* eotf lut bypass */
		set_vpp_lut(VPP_LUT_EOTF,
			eotf_33_sdr_709_mapping, /* R */
			eotf_33_sdr_709_mapping, /* G */
			eotf_33_sdr_709_mapping, /* B */
			CSC_ON);

		/* eotf matrix bypass */
		set_vpp_matrix(VPP_MATRIX_EOTF,
			eotf_RGB709_to_RGB2020_coeff,
			CSC_ON);

		/* oetf lut bypass */
		set_vpp_lut(VPP_LUT_OETF,
			oetf_sdr_2084_mapping,
			oetf_sdr_2084_mapping,
			oetf_sdr_2084_mapping,
			CSC_ON);

		/* xvycc matrix bypass */
		set_vpp_matrix(VPP_MATRIX_XVYCC,
			RGB2020_to_YUV2020l_coeff,
			CSC_ON);
		/*vpp matrix mux write*/
		vpp_set_mtx_en_write();
	} else
		bypass_hdr_process(csc_type, vinfo, master_info, vd_path);
}

static int vpp_eye_protection_process(
	enum vpp_matrix_csc_e csc_type,
	struct vinfo_s *vinfo,
	enum vd_path_e vd_path)
{
	cur_eye_protect_mode = wb_val[0];
	memcpy(&video_rgb_ogo, wb_val,
		sizeof(struct tcon_rgb_ogo_s));
	ve_ogo_param_update();
	vpp_set_mtx_en_read();
	/* only SDR need switch csc */
	if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			hdr_process_mode[vd_path])
		return 0;
	if ((csc_type < VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			sdr_process_mode[vd_path])
		return 0;

	if (vinfo->viu_color_fmt == COLOR_FMT_RGB444)
		return 0;
	/* post matrix bypass */

	if (cur_eye_protect_mode == 0) {
	/* yuv2rgb for eye protect mode */
		if (get_cpu_type() == MESON_CPU_MAJOR_ID_G12A)
			mtx_setting(POST2_MTX, MATRIX_YUV709_RGB,
				MTX_OFF);
		else
			set_vpp_matrix(VPP_MATRIX_POST,
				bypass_coeff,
				CSC_ON);
	} else {/* matrix yuv2rgb for LCD */
		if (get_cpu_type() == MESON_CPU_MAJOR_ID_G12A)
			mtx_setting(POST2_MTX, MATRIX_YUV709_RGB,
				MTX_ON);
		else
			set_vpp_matrix(VPP_MATRIX_POST,
				YUV709l_to_RGB709_coeff,
				CSC_ON);
	}

	/* xvycc matrix bypass */
	if (cur_eye_protect_mode == 1) {
		/*  for eye protect mode */
		if (get_cpu_type() == MESON_CPU_MAJOR_ID_G12A) {
			if (video_rgb_ogo_xvy_mtx)
				video_rgb_ogo_xvy_mtx_latch |=
					MTX_RGB2YUVL_RGB_OGO;
		} else {
			if (video_rgb_ogo_xvy_mtx) {
				video_rgb_ogo_xvy_mtx_latch |=
					MTX_RGB2YUVL_RGB_OGO;
				mtx_en_mux |= XVY_MTX_EN_MASK;
			} else
				set_vpp_matrix(VPP_MATRIX_XVYCC,
					RGB709_to_YUV709l_coeff,
					CSC_ON);
		}
	} else /* matrix yuv2rgb for LCD */
		if (get_cpu_type() == MESON_CPU_MAJOR_ID_G12A)
			mtx_setting(POST_MTX, MATRIX_RGB_YUV709,
				MTX_OFF);
		else
			set_vpp_matrix(VPP_MATRIX_XVYCC,
				bypass_coeff,
				CSC_ON);

	vpp_set_mtx_en_write();
	return 0;
}

static void hdr_support_process(struct vinfo_s *vinfo, enum vd_path_e vd_path)
{
	/*check if hdmitx support hdr10+*/
	tx_hdr10_plus_support = sink_hdr_support(vinfo) & HDRP_SUPPORT;

	/* check hdr support info from Tx or Panel */
	if (hdr_mode == 2) { /* auto */
		/*hdr_support & bit2 is hdr10*/
		/*hdr_support & bit3 is hlg*/
		if ((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
			(sink_hdr_support(vinfo) & HLG_SUPPORT)) {
			hdr_process_mode[vd_path] = PROC_BYPASS; /* hdr->hdr*/
			hlg_process_mode[vd_path] = PROC_BYPASS; /* hlg->hlg*/
		} else if ((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
			!(sink_hdr_support(vinfo) & HLG_SUPPORT)) {
			hdr_process_mode[vd_path] = PROC_BYPASS; /* hdr->hdr*/
			if (force_pure_hlg[vd_path])
				hlg_process_mode[vd_path] = PROC_BYPASS;
			else  /* hlg->hdr10*/
				hlg_process_mode[vd_path] = PROC_MATCH;
		} else if (!(sink_hdr_support(vinfo) & HDR_SUPPORT) &&
			(sink_hdr_support(vinfo) & HLG_SUPPORT)) {
			hdr_process_mode[vd_path] = PROC_MATCH;
			hlg_process_mode[vd_path] = PROC_BYPASS;
		} else {
			hdr_process_mode[vd_path] = PROC_MATCH;
			hlg_process_mode[vd_path] = PROC_MATCH;
		}

		if (tx_hdr10_plus_support)
			hdr10_plus_process_mode[vd_path] = PROC_BYPASS;
		else
			hdr10_plus_process_mode[vd_path] = PROC_MATCH;
	} else if (hdr_mode == 1) {
		hdr_process_mode[vd_path] = PROC_MATCH;
		hlg_process_mode[vd_path] = PROC_MATCH;
		hdr10_plus_process_mode[vd_path] = PROC_MATCH;
	} else {
		if (sink_hdr_support(vinfo) & HDR_SUPPORT)
			hdr_process_mode[vd_path] = PROC_BYPASS;
		else
			hdr_process_mode[vd_path] = PROC_MATCH;
		if ((sink_hdr_support(vinfo) & HLG_SUPPORT) ||
		(force_pure_hlg[vd_path]))
			hlg_process_mode[vd_path] = PROC_BYPASS;
		else
			hlg_process_mode[vd_path] = PROC_MATCH;
		if (tx_hdr10_plus_support)
			hdr10_plus_process_mode[vd_path] = PROC_BYPASS;
		else
			hdr10_plus_process_mode[vd_path] = PROC_MATCH;
	}

	if (sdr_mode == 2) { /* auto */
		if ((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
		((cpu_after_eq(MESON_CPU_MAJOR_ID_GXL)) &&
		 (vinfo->viu_color_fmt != COLOR_FMT_RGB444)))
			/*box sdr->hdr*/
			sdr_process_mode[vd_path] = PROC_SDR_TO_HDR;
		else if ((vinfo->viu_color_fmt == COLOR_FMT_RGB444) &&
			((get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB) ||
			(get_cpu_type() == MESON_CPU_MAJOR_ID_TXL)))
			/*tv sdr->hdr*/
			sdr_process_mode[vd_path] = PROC_SDR_TO_HDR;
		else
			/* sdr->sdr*/
			sdr_process_mode[vd_path] = PROC_BYPASS;
	} else
		/* force sdr->hdr */
		sdr_process_mode[vd_path] = sdr_mode;
}

static int sink_support_dolby_vision(const struct vinfo_s *vinfo)
{
	if (sink_hdr_support(vinfo) & DV_SUPPORT)
		return 1;
	return 0;
}

static int sink_support_hdr(const struct vinfo_s *vinfo)
{
	if (sink_hdr_support(vinfo) & HDR_SUPPORT)
		return 1;
	return 0;
}

static int sink_support_hlg(const struct vinfo_s *vinfo)
{
	if (sink_hdr_support(vinfo) & HLG_SUPPORT)
		return 1;
	return 0;
}

static int sink_support_hdr10_plus(const struct vinfo_s *vinfo)
{
	/* panel output and TL1 and TM2 */
	if ((vinfo->viu_color_fmt == COLOR_FMT_RGB444) &&
	    ((get_cpu_type() == MESON_CPU_MAJOR_ID_TL1) ||
	    (get_cpu_type() == MESON_CPU_MAJOR_ID_TM2)))
		return 1;

	/* hdmi */
	if (sink_hdr_support(vinfo) & HDRP_SUPPORT)
		return 1;

	return 0;
}

bool is_vinfo_available(const struct vinfo_s *vinfo)
{
	if (!vinfo)
		return false;
	else
		return strcmp(vinfo->name, "invalid") &&
			strcmp(vinfo->name, "null") &&
			strcmp(vinfo->name, "576cvbs") &&
			strcmp(vinfo->name, "470cvbs");
}
EXPORT_SYMBOL(is_vinfo_available);

static enum hdr_type_e get_source_type(enum vd_path_e vd_path)
{
	get_cur_vd_signal_type(vd_path);
	if (cur_mvc_type[vd_path] == VIDTYPE_MVC)
		return HDRTYPE_MVC;
	if (vd_path == VD1_PATH &&
	    is_dolby_vision_enable() &&
	    get_dolby_vision_src_format()
	    == HDRTYPE_DOVI)
		return HDRTYPE_DOVI;
	if (((signal_transfer_characteristic == 14) ||
	     (signal_transfer_characteristic == 18)) &&
	    (signal_color_primaries == 9))
		return HDRTYPE_HLG;
	else if ((signal_transfer_characteristic == 0x30) &&
		 (signal_color_primaries == 9)) {
		if (sink_support_hdr10_plus(get_current_vinfo()) &&
		    (vd_path == VD1_PATH))
			return HDRTYPE_HDR10PLUS;
		else
			return HDRTYPE_HDR10;
	} else if (signal_transfer_characteristic == 16)
		return HDRTYPE_HDR10;
	else
		return HDRTYPE_SDR;
}

int get_hdr_module_status(enum vd_path_e vd_path)
{
	if (vd_path == VD1_PATH &&
	    is_dolby_vision_enable() &&
	    get_dolby_vision_policy()
	    == DOLBY_VISION_FOLLOW_SOURCE &&
	    get_source_type(VD1_PATH)
	    == HDRTYPE_SDR &&
	    sdr_process_mode[VD1_PATH]
	    == PROC_BYPASS)
		return HDR_MODULE_BYPASS;
	return video_process_status[vd_path];
}
EXPORT_SYMBOL(get_hdr_module_status);

static bool video_layer_wait_on[VD_PATH_MAX];
bool is_video_layer_on(enum vd_path_e vd_path)
{
	bool video_on =
		(vd_path == VD1_PATH) ?
		get_video_enabled() :
		get_videopip_enabled();

	if (video_on)
		video_layer_wait_on[vd_path] = false;
	return video_on ||
		video_layer_wait_on[vd_path];
}

static bool hdr10_plus_metadata_update(
	struct vframe_s *vf,
	enum vpp_matrix_csc_e csc_type,
	struct hdr10plus_para *p)
{
	if (!vf)
		return false;
	if (csc_type != VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC)
		return false;

	hdr10_plus_parser_metadata(vf);

	if (tx_hdr10_plus_support) {
		hdr10_plus_hdmitx_vsif_parser(p, vf);
		pr_csc(0x10,
			"hdr10+ vsif: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			((p->application_version & 0x3) << 6) |
				((p->targeted_max_lum & 0x1f) << 1),
			p->average_maxrgb,
			p->distribution_values[0],
			p->distribution_values[1],
			p->distribution_values[2],
			p->distribution_values[3],
			p->distribution_values[4],
			p->distribution_values[5],
			p->distribution_values[6],
			p->distribution_values[7],
			p->distribution_values[8],
			((p->num_bezier_curve_anchors & 0xf) << 4) |
				((p->knee_point_x >> 6) & 0xf),
			((p->knee_point_x & 0x3f) << 2) |
				((p->knee_point_y >> 8) & 0x3),
			p->knee_point_y & 0xff,
			p->bezier_curve_anchors[0],
			p->bezier_curve_anchors[1],
			p->bezier_curve_anchors[2],
			p->bezier_curve_anchors[3],
			p->bezier_curve_anchors[4],
			p->bezier_curve_anchors[5],
			p->bezier_curve_anchors[6],
			p->bezier_curve_anchors[7],
			p->bezier_curve_anchors[8],
			((p->graphics_overlay_flag & 0x1) << 7) |
				((p->no_delay_flag & 0x1) << 6));
	}
	//TODO: return false if meta not changed
	return true;
}

static struct hdr10pgen_param_s hdr10pgen_param;
void hdr10_plus_process_update(int force_source_lumin)
{
	hdr10_plus_ootf_gen(
		customer_panel_lumin,
		force_source_lumin,
		&hdr10pgen_param);
	hdr10p_ebzcurve_update(VD1_HDR, HDR10P_SDR, &hdr10pgen_param);
}
EXPORT_SYMBOL(hdr10_plus_process_update);

static struct hdr10plus_para hdmitx_hdr10plus_params[VD_PATH_MAX];
static int hdr10_plus_pkt_update;
static bool hdr10_plus_pkt_on;
/* 0: no delay, 1:delay one frame, >1:delay one frame and repeat */
static uint hdr10_plus_pkt_delay = 1;
static struct hdr10plus_para cur_hdr10plus_params;
static struct master_display_info_s cur_send_info;
module_param(hdr10_plus_pkt_delay, uint, 0664);

uint get_hdr10_plus_pkt_delay(void)
{
	return hdr10_plus_pkt_delay;
}
EXPORT_SYMBOL(get_hdr10_plus_pkt_delay);

void update_hdr10_plus_pkt(bool enable,
	void *hdr10plus_params,
	void *send_info)
{
	struct vinfo_s *vinfo = get_current_vinfo();
	struct vout_device_s *vdev = NULL;

	if (vinfo->vout_device) {
		vdev = vinfo->vout_device;
		if (!vdev)
			return;
		if (!vdev->fresh_tx_hdr_pkt)
			return;
		if (!vdev->fresh_tx_hdr10plus_pkt)
			return;
	}

	hdr10_plus_pkt_on = enable;
	if (hdr10_plus_pkt_on) {
		memcpy((void *)&cur_hdr10plus_params, hdr10plus_params,
			sizeof(struct hdr10plus_para));
		memcpy((void *)&cur_send_info, send_info,
			sizeof(struct master_display_info_s));
		hdr10_plus_pkt_update = HDRPLUS_PKT_UPDATE;
		pr_csc(2, "update_hdr10_plus_pkt on\n");
	} else {
		if (!vdev)
			return;
		if ((get_dolby_vision_policy() ==
		     DOLBY_VISION_FOLLOW_SINK) &&
		    sink_support_hdr(vinfo) &&
		    (!sink_support_dolby_vision(vinfo))) {
			pr_csc(2, "update_hdr10_plus_pkt: DISABLE_VSIF\n");
			vdev->fresh_tx_hdr10plus_pkt(
				HDR10_PLUS_DISABLE_VSIF,
				&cur_hdr10plus_params);
		} else {
			pr_csc(2, "update_hdr10_plus_pkt: ZERO_VSIF\n");
			vdev->fresh_tx_hdr_pkt(send_info);
			vdev->fresh_tx_hdr10plus_pkt(
				HDR10_PLUS_ZERO_VSIF,
				&cur_hdr10plus_params);
		}
		hdr10_plus_pkt_update = HDRPLUS_PKT_IDLE;
		pr_csc(2, "update_hdr10_plus_pkt off\n");
	}
}
EXPORT_SYMBOL(update_hdr10_plus_pkt);

void send_hdr10_plus_pkt(enum vd_path_e vd_path)
{
	struct vinfo_s *vinfo = get_current_vinfo();
	struct vout_device_s *vdev = NULL;

	if (vinfo->vout_device) {
		vdev = vinfo->vout_device;
		if (!vdev)
			return;
		if (!vdev->fresh_tx_hdr_pkt)
			return;
		if (!vdev->fresh_tx_hdr10plus_pkt)
			return;
	}
	if (hdr10_plus_pkt_update == HDRPLUS_PKT_UPDATE) {
		if (!vdev)
			return;
		vdev->fresh_tx_hdr_pkt(
			&cur_send_info);
		vdev->fresh_tx_hdr10plus_pkt(
			hdr10_plus_pkt_on,
			&cur_hdr10plus_params);
		if (get_hdr10_plus_pkt_delay() > 1)
			hdr10_plus_pkt_update = HDRPLUS_PKT_REPEAT;
		else
			hdr10_plus_pkt_update = HDRPLUS_PKT_IDLE;
		pr_csc(2, "send_hdr10_plus_pkt update\n");
	} else if ((hdr10_plus_pkt_update == HDRPLUS_PKT_REPEAT) &&
		(get_hdr10_plus_pkt_delay() > 1)) {
		if (!vdev)
			return;
		vdev->fresh_tx_hdr_pkt(
			&cur_send_info);
		vdev->fresh_tx_hdr10plus_pkt(
			hdr10_plus_pkt_on,
			&cur_hdr10plus_params);
		pr_csc(2, "send_hdr10_plus_pkt repeat\n");
	}
}
EXPORT_SYMBOL(send_hdr10_plus_pkt);

static void hdr_tx_pkt_cb(
	struct vinfo_s *vinfo,
	int signal_change_flag,
	enum vpp_matrix_csc_e csc_type,
	struct vframe_master_display_colour_s *p,
	int *hdmi_scs_type_changed,
	struct hdr10plus_para *hdmitx_hdr10plus_param,
	enum vd_path_e vd_path)
{
	struct vout_device_s *vdev = NULL;
	struct master_display_info_s send_info;

	if (vinfo->vout_device)
		vdev = vinfo->vout_device;

	if ((vinfo->viu_color_fmt != COLOR_FMT_RGB444) &&
		((sink_hdr_support(vinfo) &
		(HDR_SUPPORT | HLG_SUPPORT)) ||
		(signal_change_flag & SIG_HDR_SUPPORT) ||
		(signal_change_flag & SIG_HLG_SUPPORT) ||
		/*hdr10 plus*/
		(tx_hdr10_plus_support) ||
		(signal_change_flag & SIG_HDR10_PLUS_MODE))) {
		if (sdr_process_mode[vd_path] &&
			(csc_type < VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			/* sdr source convert to hdr */
			/* send hdr info */
			/* use the features to discribe source info */
			send_info.features =
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/* color available */
					| (9 << 16)	/* bt2020 */
					| (16 << 8)	/* bt2020-10 */
					| (10 << 0);	/* bt2020c */
			amvecm_cp_hdr_info(&send_info, p);
			if (vdev) {
				if (vdev->fresh_tx_hdr_pkt)
					vdev->fresh_tx_hdr_pkt(&send_info);
			}

			if (cur_csc_type[vd_path] ==
				VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC) {
				if (vdev) {
					if (vdev->fresh_tx_hdr10plus_pkt)
						vdev->fresh_tx_hdr10plus_pkt(0,
							hdmitx_hdr10plus_param);
				}
			}

			if (hdmi_csc_type != VPP_MATRIX_BT2020YUV_BT2020RGB) {
				hdmi_csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
				*hdmi_scs_type_changed = 1;
			}
		} else if ((hdr_process_mode[vd_path] == PROC_BYPASS) &&
			(hlg_process_mode[vd_path] == PROC_BYPASS) &&
			(csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			/* source is hdr, send hdr info */
			/* use the features to discribe source info */
			send_info.features =
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					/* bt2020 */
					| (signal_color_primaries << 16)
					/* bt2020-10 */
					| (signal_transfer_characteristic << 8)
					| (10 << 0);	/* bt2020c */
			amvecm_cp_hdr_info(&send_info, p);
			if (vdev) {
				if (vdev->fresh_tx_hdr_pkt)
					vdev->fresh_tx_hdr_pkt(&send_info);
			}

			if (hdmi_csc_type != VPP_MATRIX_BT2020YUV_BT2020RGB) {
				hdmi_csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
				*hdmi_scs_type_changed = 1;
			}
		} else if ((hdr_process_mode[vd_path] == PROC_BYPASS) &&
			(hlg_process_mode[vd_path] == PROC_MATCH) &&
			(csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			/* source is hdr, send hdr info */
			/* use the features to discribe source info */
			if (get_hdr_type() & HLG_FLAG)
				send_info.features =
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					/* bt2020 */
					| (9 << 16)
					/* bt2020-10 */
					| (16 << 8)
					| (10 << 0);	/* bt2020c */
			else
				send_info.features =
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					/* bt2020 */
					| (signal_color_primaries << 16)
					/* bt2020-10 */
					| (signal_transfer_characteristic << 8)
					| (10 << 0);	/* bt2020c */
			amvecm_cp_hdr_info(&send_info, p);
			if (vdev) {
				if (vdev->fresh_tx_hdr_pkt)
					vdev->fresh_tx_hdr_pkt(&send_info);
			}
			if (hdmi_csc_type != VPP_MATRIX_BT2020YUV_BT2020RGB) {
				hdmi_csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
				*hdmi_scs_type_changed = 1;
			}
		} else if ((hdr_process_mode[vd_path] == PROC_MATCH) &&
			(hlg_process_mode[vd_path] == PROC_BYPASS) &&
			(csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			/* source is hdr, send hdr info */
			/* use the features to discribe source info */
			if (get_hdr_type() & HLG_FLAG)
				send_info.features =
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					/* bt2020 */
					| (signal_color_primaries << 16)
					/* bt2020-10 */
					| (signal_transfer_characteristic << 8)
					| (10 << 0);	/* bt2020c */
			else
				send_info.features =
					/* default 709 full */
					(0 << 30) /*sdr output 709*/
					| (1 << 29) /*video available*/
					| (5 << 26) /* unspecified */
					| (0 << 25) /* limit */
					| (1 << 24) /*color available*/
					| (1 << 16) /* bt709 */
					| (1 << 8)	/* bt709 */
					| (1 << 0); /* bt709 */
			amvecm_cp_hdr_info(&send_info, p);
			if (vdev) {
				if (vdev->fresh_tx_hdr_pkt)
					vdev->fresh_tx_hdr_pkt(&send_info);
			}
			if (hdmi_csc_type != VPP_MATRIX_BT2020YUV_BT2020RGB) {
				hdmi_csc_type = VPP_MATRIX_BT2020YUV_BT2020RGB;
				*hdmi_scs_type_changed = 1;
			}
		} else if ((hdr10_plus_process_mode[vd_path] == 0) &&
			(csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC)) {
			/* source is hdr10+, send hdr10 drm packet as well*/
			/* use the features to discribe source info */
			send_info.features =
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					/* bt2020 */
					| (signal_color_primaries << 16)
					| (16 << 8)     /* Always HDR10 */
					| (10 << 0);	/* bt2020c */
			amvecm_cp_hdr_info(&send_info, p);
			if (vdev) {
				/* source is hdr10 plus, send hdr10 plus info */
				if (get_hdr10_plus_pkt_delay()) {
					update_hdr10_plus_pkt(true,
						hdmitx_hdr10plus_param,
						&send_info);
				} else {
					/* send HDR10 DRM packet */
					if (vdev->fresh_tx_hdr_pkt)
						vdev->fresh_tx_hdr_pkt(
							&send_info);
					/* send hdr10 plus info */
					if (vdev->fresh_tx_hdr10plus_pkt)
						vdev->fresh_tx_hdr10plus_pkt(1,
							hdmitx_hdr10plus_param);
				}
			}
			if (hdmi_csc_type !=
				VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC) {
				hdmi_csc_type =
					VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC;
				*hdmi_scs_type_changed = 1;
			}

		} else {
			/* sdr source send normal info*/
			/* use the features to discribe source info */
			if (((sink_hdr_support(vinfo) & HDR_SUPPORT) ||
				(sink_hdr_support(vinfo) & HLG_SUPPORT)) &&
				(csc_type < VPP_MATRIX_BT2020YUV_BT2020RGB) &&
				tx_op_color_primary)
				send_info.features =
					/* default 709 limit */
					(1 << 30) /*sdr output 2020*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					| (1 << 16)	/* bt709 */
					| (1 << 8)	/* bt709 */
					| (1 << 0);	/* bt709 */
			else
				send_info.features =
					/* default 709 limit */
					(0 << 30) /*sdr output 709*/
					| (1 << 29)	/*video available*/
					| (5 << 26)	/* unspecified */
					| (0 << 25)	/* limit */
					| (1 << 24)	/*color available*/
					| (1 << 16)	/* bt709 */
					| (1 << 8)	/* bt709 */
					| (1 << 0);	/* bt709 */
			amvecm_cp_hdr_info(&send_info, p);
			if (cur_csc_type[vd_path] <=
				VPP_MATRIX_BT2020YUV_BT2020RGB) {
				if (vdev) {
					if (vdev->fresh_tx_hdr_pkt)
						vdev->fresh_tx_hdr_pkt(
							&send_info);
				}
			} else if (cur_csc_type[vd_path] ==
				VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC) {
				if (vdev) {
					if (vdev->fresh_tx_hdr_pkt)
						vdev->fresh_tx_hdr_pkt(
							&send_info);
					if (vdev->fresh_tx_hdr10plus_pkt)
						vdev->fresh_tx_hdr10plus_pkt(0,
							hdmitx_hdr10plus_param);
				}
			}
			if (hdmi_csc_type != VPP_MATRIX_YUV709_RGB) {
				hdmi_csc_type = VPP_MATRIX_YUV709_RGB;
				*hdmi_scs_type_changed = 1;
			}
		}
	}
}

static void video_process(
	struct vframe_s *vf,
	enum vpp_matrix_csc_e csc_type,
	int signal_change_flag,
	struct vinfo_s *vinfo,
	struct vframe_master_display_colour_s *p,
	enum vd_path_e vd_path)
{
	int need_adjust_contrast_saturation = 0;

	/* decided by edid or panel info or user setting */
	if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
		(hdr_process_mode[vd_path] == PROC_MATCH) &&
		(hlg_process_mode[vd_path] == PROC_MATCH)) {
		/* hdr->sdr hlg->sdr */
		if ((signal_change_flag &
				(SIG_CS_CHG |
				SIG_PRI_INFO |
				SIG_KNEE_FACTOR |
				SIG_HDR_MODE |
				SIG_HDR_SUPPORT |
				SIG_HLG_MODE |
				SIG_HDR_OOTF_CHG)
			) ||
			(cur_csc_type[vd_path] <
				VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			if (get_hdr_type() & HLG_FLAG)
				need_adjust_contrast_saturation =
					hlg_process(csc_type, vinfo, p,
						vd_path);
			else
				need_adjust_contrast_saturation
				= hdr_process(
					csc_type, vinfo, p, vd_path);
			pr_csc(1, "vd_path = %d\n"
				"hdr_process_mode = 0x%x\n"
				"hlg_process_mode = 0x%x.\n",
				vd_path + 1,
				hdr_process_mode[vd_path],
				hlg_process_mode[vd_path]);
		}
	} else if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
		(hdr_process_mode[vd_path] == PROC_BYPASS) &&
		(hlg_process_mode[vd_path] == PROC_MATCH)) {
		/* hdr->hdr hlg->hdr*/
		if ((signal_change_flag &
				(SIG_CS_CHG |
				SIG_PRI_INFO |
				SIG_KNEE_FACTOR |
				SIG_HDR_MODE |
				SIG_HDR_SUPPORT |
				SIG_HLG_MODE |
				SIG_HDR_OOTF_CHG)
			) ||
			(cur_csc_type[vd_path] <
				VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			if (get_hdr_type() & HLG_FLAG)
				hlg_hdr_process(csc_type, vinfo, p, vd_path);
			else
				bypass_hdr_process(csc_type, vinfo, p, vd_path);
			pr_csc(1, "vd_path = %d\n"
				"\thdr_process_mode = 0x%x\n"
				"\thlg_process_mode = 0x%x.\n",
				vd_path + 1,
				hdr_process_mode[vd_path],
				hlg_process_mode[vd_path]);
		}
	} else if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
		(hdr_process_mode[vd_path] == PROC_MATCH) &&
		(hlg_process_mode[vd_path] == PROC_BYPASS)) {
		/* hdr->sdr hlg->hlg*/
		if ((signal_change_flag &
				(SIG_CS_CHG |
				SIG_PRI_INFO |
				SIG_KNEE_FACTOR |
				SIG_HDR_MODE |
				SIG_HDR_SUPPORT |
				SIG_HLG_MODE |
				SIG_HDR_OOTF_CHG)
			) ||
			(cur_csc_type[vd_path] <
				VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			if (get_hdr_type() & HLG_FLAG)
				bypass_hdr_process(csc_type, vinfo, p,
					vd_path);
			else
				need_adjust_contrast_saturation =
					hdr_process(csc_type, vinfo, p,
						vd_path);
			pr_csc(1, "vd_path = %d\n"
				"\thdr_process_mode = 0x%x\n"
				"\thlg_process_mode = 0x%x.\n",
				vd_path + 1,
				hdr_process_mode[vd_path],
				hlg_process_mode[vd_path]);
		}
	} else if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB) &&
		(hdr_process_mode[vd_path] == PROC_BYPASS) &&
		(hlg_process_mode[vd_path] == PROC_BYPASS)) {
		/* hdr->hdr hlg->hlg*/
		if ((signal_change_flag &
			(SIG_CS_CHG |
			SIG_PRI_INFO |
			SIG_KNEE_FACTOR |
			SIG_HDR_MODE |
			SIG_HDR_SUPPORT |
			SIG_HLG_MODE |
			SIG_HDR_OOTF_CHG)) ||
			(cur_csc_type[vd_path] <
				VPP_MATRIX_BT2020YUV_BT2020RGB)) {
			bypass_hdr_process(csc_type, vinfo, p,
				vd_path);
			pr_csc(1, "vd_path = %d\n"
				"bypass_hdr_process: 0x%x, 0x%x.\n",
				vd_path + 1,
				hdr_process_mode[vd_path],
				hlg_process_mode[vd_path]);
		}
	} else if (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC) {
		if ((signal_change_flag & SIG_HDR10_PLUS_MODE) ||
			(cur_csc_type[vd_path] !=
			VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC)) {
			// TODO: hdr10_plus_process(vf, mode, vd_path);
			if (hdr10_plus_process_mode[vd_path] == PROC_MATCH) {
				hdr10p_process(vinfo, p, vd_path);
				/* hdr10_plus_process(vf); */
			} else {
				bypass_hdr_process(
					csc_type, vinfo, p, vd_path);
			}
		}
		pr_csc(1, "vd_path = %d\nhdr10_plus_process.\n",
			vd_path + 1);
	} else {
		if ((csc_type < VPP_MATRIX_BT2020YUV_BT2020RGB) &&
			sdr_process_mode[vd_path])
			/* for gxl and gxm SDR to HDR process */
			sdr_hdr_process(csc_type, vinfo, p, vd_path);
		else {
			/* for gxtvbb and gxl HDR bypass process */
			if (((sink_hdr_support(vinfo) &
				HDR_SUPPORT) ||
				(sink_hdr_support(vinfo) &
				HLG_SUPPORT)) &&
				(csc_type <
				VPP_MATRIX_BT2020YUV_BT2020RGB)
				&& tx_op_color_primary)
				set_bt2020csc_process(csc_type,
				vinfo, p, vd_path);
			else
				bypass_hdr_process(csc_type,
				vinfo, p, vd_path);
			pr_csc(1, "vd_path = %d\n"
				"\tcsc_type = 0x%x\n"
				"\thdr_process_mode = %d.\n"
				"\thlg_process_mode = %d.\n"
				"\thdr10_plus_process_mode = %d.\n",
				vd_path + 1,
				csc_type,
				hdr_process_mode[vd_path],
				hlg_process_mode[vd_path],
				hdr10_plus_process_mode[vd_path]);
		}
	}

	if (get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A) {
		if (vinfo->viu_color_fmt != COLOR_FMT_RGB444)
			mtx_setting(POST2_MTX, MATRIX_NULL, MTX_OFF);
		else
			mtx_setting(POST2_MTX,
				csc_type/*MATRIX_YUV709_RGB*/, MTX_ON);
	}

	if (cur_hdr_process_mode[vd_path] !=
		hdr_process_mode[vd_path]) {
		cur_hdr_process_mode[vd_path] =
			hdr_process_mode[vd_path];
		pr_csc(1, "vd_path = %d\n"
			"hdr_process_mode changed to %d\n",
			vd_path + 1, hdr_process_mode[vd_path]);
	}
	if (cur_sdr_process_mode[vd_path] !=
		sdr_process_mode[vd_path]) {
		cur_sdr_process_mode[vd_path] =
			sdr_process_mode[vd_path];
		pr_csc(1, "vd_path = %d\nsdr_process_mode changed to %d\n",
			vd_path + 1, sdr_process_mode[vd_path]);
	}
	if (cur_hlg_process_mode[vd_path] !=
		hlg_process_mode[vd_path]) {
		cur_hlg_process_mode[vd_path] =
			hlg_process_mode[vd_path];
		pr_csc(1, "vd_path = %d\nhlg_process_mode changed to %d\n",
			vd_path + 1, hlg_process_mode[vd_path]);
	}
	if (cur_hdr10_plus_process_mode[vd_path] !=
		hdr10_plus_process_mode[vd_path]) {
		cur_hdr10_plus_process_mode[vd_path] =
			hdr10_plus_process_mode[vd_path];
		pr_csc(1, "vd_path = %d\n"
			"hdr10_plus_process_mode changed to %d\n",
			vd_path + 1, hdr10_plus_process_mode[vd_path]);
	}
	if (need_adjust_contrast_saturation & 1) {
		if (lut_289_en &&
			(get_cpu_type() <= MESON_CPU_MAJOR_ID_GXTVBB))
			vd1_contrast_offset = 0;
		else
			vd1_contrast_offset =
			calculate_contrast_adj(p->luminance[0] / 10000);
		vecm_latch_flag |= FLAG_VADJ1_CON;
	} else {
		vd1_contrast_offset = 0;
		vecm_latch_flag |= FLAG_VADJ1_CON;
	}
	if (need_adjust_contrast_saturation & 2) {
		vecm_latch_flag |= FLAG_VADJ1_COLOR;
	} else {
		if (((get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB) ||
			(get_cpu_type() == MESON_CPU_MAJOR_ID_TXL)) &&
			(sdr_process_mode[vd_path] == PROC_SDR_TO_HDR))
			saturation_offset = sdr_saturation_offset;
		else
			saturation_offset = 0;
		vecm_latch_flag |= FLAG_VADJ1_COLOR;
	}
	if (cur_csc_type[vd_path] != csc_type) {
		pr_csc(1, "vd_path = %d\n"
			"\tcsc from 0x%x to 0x%x.\n"
			"contrast offset = %d.\n"
			"saturation offset = %d.\n",
			vd_path + 1, cur_csc_type[vd_path], csc_type,
			vd1_contrast_offset, saturation_offset);
		cur_csc_type[vd_path] = csc_type;

	}
}

static int current_hdr_cap[VD_PATH_MAX] = {-1, -1}; /* should set when probe */
static int current_sink_available[VD_PATH_MAX] = {-1, -1};

int is_sink_cap_changed(
	const struct vinfo_s *vinfo,
	int *p_current_hdr_cap,
	int *p_current_sink_available)
{
	int hdr_cap;
	int sink_available;
	int ret = 0;

	if (vinfo && is_vinfo_available(vinfo)) {
		hdr_cap = (1 << 0) |
			(sink_support_dolby_vision(vinfo) << 1) |
			(sink_support_hdr10_plus(vinfo) << 2) |
			(sink_support_hdr(vinfo) << 3) |
			(sink_support_hlg(vinfo) << 3);
		sink_available = 1;
	} else {
		hdr_cap = 0;
		*p_current_hdr_cap = 0;
		sink_available = 0;
	}

	if (*p_current_hdr_cap == -1) {
		*p_current_hdr_cap = hdr_cap;
		ret |= 4;
	} else if (*p_current_hdr_cap != hdr_cap) {
		*p_current_hdr_cap = hdr_cap;
		ret |= 2;
	}
	if (*p_current_sink_available != sink_available) {
		*p_current_sink_available = sink_available;
		ret |= 1;
	}
	return ret;
}
EXPORT_SYMBOL(is_sink_cap_changed);

static bool video_on[VD_PATH_MAX];
int is_video_turn_on(bool *vd_on, enum vd_path_e vd_path)
{
	int ret = 0;

	if (vd_on[vd_path] != is_video_layer_on(vd_path)) {
		vd_on[vd_path] = is_video_layer_on(vd_path);
		ret = vd_on[vd_path] ? 1 : -1;
	}
	return ret;
}
EXPORT_SYMBOL(is_video_turn_on);

static int vpp_matrix_update(
	struct vframe_s *vf, struct vinfo_s *vinfo, int flags,
	enum vd_path_e vd_path)
{
	enum vpp_matrix_csc_e csc_type = VPP_MATRIX_NULL;
	int signal_change_flag = 0;
	struct vframe_master_display_colour_s *p =
		&cur_master_display_colour[vd_path];
	int hdmi_scs_type_changed = 0;
	bool hdr10p_meta_updated = false;

	if (!vinfo || vinfo->mode == VMODE_NULL ||
	    vinfo->mode == VMODE_INVALID)
		return 0;

	/* Tx hdr information */
	memcpy(&receiver_hdr_info, &vinfo->hdr_info,
			sizeof(struct hdr_info));

	if (!cpu_after_eq(MESON_CPU_MAJOR_ID_G12A) ||
	(get_cpu_type() == MESON_CPU_MAJOR_ID_TL1))
		hdr_support_process(vinfo, vd_path);

	if (vf && vinfo)
		signal_change_flag =
			signal_type_changed(vf, vinfo, vd_path);

	get_cur_vd_signal_type(vd_path);
	if (force_csc_type != 0xff)
		csc_type = force_csc_type;
	else
		csc_type = get_csc_type();

	if ((video_process_status[vd_path] == HDR_MODULE_BYPASS) &&
	    !(video_process_flags[vd_path] & PROC_FLAG_FORCE_PROCESS)) {
		if ((is_video_layer_on(vd_path) ||
		     video_layer_wait_on[vd_path]) &&
		    (!is_dolby_vision_on() ||
		     (vd_path == VD2_PATH))) {
			video_process_status[vd_path] = HDR_MODULE_ON;
			pr_csc(4,
			       "video_process_status[%s] = HDR_MODULE_ON\n",
			       vd_path == VD1_PATH ? "VD1" : "VD2");
		} else {
			return 2;
		}
	}

	if (is_dolby_vision_on() &&
	    ((vd_path == VD1_PATH) ||
	    !cpu_after_eq(MESON_CPU_MAJOR_ID_G12A)))
		return 0;

	if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A) &&
	(get_cpu_type() != MESON_CPU_MAJOR_ID_TL1)) {
		enum hdr_type_e source_format[VD_PATH_MAX];
		enum vd_path_e oth_path =
			(vd_path == VD1_PATH) ? VD2_PATH : VD1_PATH;

		if (get_hdr_policy() != cur_hdr_policy) {
			pr_csc(4, "policy changed from %d to %d.\n",
			       cur_hdr_policy,
			       get_hdr_policy());
			signal_change_flag |= SIG_HDR_MODE;
		}

		source_format[VD1_PATH] = get_source_type(VD1_PATH);
		source_format[VD2_PATH] = get_source_type(VD2_PATH);
		get_cur_vd_signal_type(vd_path);

		signal_change_flag |=
			hdr_policy_process(vinfo, source_format, vd_path);
		if (signal_change_flag & SIG_OUTPUT_MODE_CHG) {
			if (!is_video_layer_on(vd_path))
				video_layer_wait_on[vd_path] = true;
			video_process_flags[oth_path] |=
				PROC_FLAG_FORCE_PROCESS;
			return 1;
		}
	}

	if ((vf != NULL) && (flags & CSC_FLAG_TOGGLE_FRAME)) {
		hdr10p_meta_updated = hdr10_plus_metadata_update(
			vf, csc_type,
			&hdmitx_hdr10plus_params[vd_path]);

		if ((csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC) ||
		    (csc_type == VPP_MATRIX_BT2020YUV_BT2020RGB))
			get_hist(VD1_HDR, HIST_O_BEFORE);

	}

	if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A)) {
		if ((vd_path == VD1_PATH)
		|| ((vd_path == VD2_PATH)
		&& !is_video_layer_on(VD1_PATH)
		&& is_video_layer_on(VD2_PATH)))
			hdmi_packet_process(signal_change_flag, vinfo, p,
				hdr10p_meta_updated ?
				&hdmitx_hdr10plus_params[vd_path] : NULL,
				vd_path);
	} else {
		if ((vd_path == VD1_PATH)
		|| ((vd_path == VD2_PATH) && !is_video_layer_on(VD1_PATH)))
			hdr_tx_pkt_cb(vinfo,
				signal_change_flag,
				csc_type,
				p,
				&hdmi_scs_type_changed,
				&hdmitx_hdr10plus_params[vd_path],
				vd_path);
	}

	if (hdmi_scs_type_changed &&
		(flags & CSC_FLAG_CHECK_OUTPUT) &&
		csc_en & 0x10)
		return 1;

	if (((!signal_change_flag) && (force_csc_type == 0xff))
		&& ((flags & CSC_FLAG_TOGGLE_FRAME) == 0))
		return 0;

	if ((cur_csc_type[vd_path] != csc_type)
	|| (signal_change_flag
	& (SIG_CS_CHG | SIG_PRI_INFO | SIG_KNEE_FACTOR | SIG_HDR_MODE |
		SIG_HDR_SUPPORT | SIG_HLG_MODE | SIG_OP_CHG |
		SIG_SRC_OUTPUT_CHG | SIG_HDR10_PLUS_MODE |
		SIG_SRC_CHG | SIG_HDR_OOTF_CHG))) {
		if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A) &&
		(get_cpu_type() != MESON_CPU_MAJOR_ID_TL1))
			video_post_process(csc_type, vinfo, vd_path);
		else
			video_process(vf, csc_type, signal_change_flag,
				vinfo, p, vd_path);
		cur_hdr_policy = get_hdr_policy();
	}

	if (hdr10p_meta_updated &&
	    hdr10_plus_process_mode[vd_path] == PROC_MATCH)
		hdr10_plus_process_update(0);

	/* eye protection mode */
	if (signal_change_flag & SIG_WB_CHG)
		vpp_eye_protection_process(csc_type, vinfo, vd_path);

	return 0;
}

static struct vframe_s last_vf_backup[VD_PATH_MAX];
static struct vframe_s *last_vf[VD_PATH_MAX];
static int last_vf_signal_type[VD_PATH_MAX];
static int null_vf_cnt[VD_PATH_MAX] = {2, 2};
static int prev_color_fmt;
static bool dovi_on;

static unsigned int fg_vf_sw_dbg;
unsigned int null_vf_max = 2;
module_param(null_vf_max, uint, 0664);
MODULE_PARM_DESC(null_vf_max, "\n null_vf_max\n");

int amvecm_matrix_process(
	struct vframe_s *vf, struct vframe_s *vf_rpt, int flags,
	enum vd_path_e vd_path)
{
	struct vframe_s fake_vframe;
	struct vinfo_s *vinfo = get_current_vinfo();
	int toggle_frame = 0;
	int i;
	int ret;
	int sink_changed = 0;
	bool cap_changed = false;
	bool send_fake_frame = false;

	if ((get_cpu_type() < MESON_CPU_MAJOR_ID_GXTVBB) ||
	    is_meson_gxl_package_905M2() || (csc_en == 0) || !vinfo)
		return 0;

	if (vinfo->mode == VMODE_NULL ||
	    vinfo->mode == VMODE_INVALID) {
		current_hdr_cap[vd_path] = 0;
		current_sink_available[vd_path] = 0;
		return 0;
	}

	if (reload_mtx) {
		for (i = 0; i < NUM_MATRIX; i++)
			if (reload_mtx & (1 << i))
				set_vpp_matrix(i, NULL, CSC_ON);
	}

	if (reload_lut) {
		for (i = 0; i < NUM_LUT; i++)
			if (reload_lut & (1 << i))
				set_vpp_lut(i,
					NULL, /* R */
					NULL, /* G */
					NULL, /* B */
					CSC_ON);
	}

	if (vd_path == VD1_PATH) {
		if (is_dolby_vision_on()) {
			dovi_on = true;
			if ((video_process_status[vd_path]
				== HDR_MODULE_BYPASS) &&
				(vf || vf_rpt))
				return vpp_matrix_update(
					vf ? vf : vf_rpt,
					vinfo, flags, vd_path);
			else
				return 0;
		} else {
			/* dv from on to off */
			if (dovi_on) {
				cap_changed = true;
				dovi_on = false;
			}
		}
	}

	sink_changed = is_sink_cap_changed(vinfo,
					   &current_hdr_cap[vd_path],
					   &current_sink_available[vd_path]);
	if (sink_changed) {
		cap_changed = sink_changed & 0x02;
		pr_csc(4, "sink %s, cap%s 0x%x, vd%d %s %p %p\n",
		       current_sink_available[vd_path] ? "on" : "off",
		       cap_changed ? " changed" : "",
		       current_hdr_cap[vd_path],
		       vd_path + 1,
		       is_video_layer_on(vd_path) ? "on" : "off",
		       vf, vf_rpt);
	}

	if (is_video_turn_on(video_on, vd_path) == 1)
		cap_changed = true;

	if ((vf != NULL) && (flags & CSC_FLAG_CHECK_OUTPUT)) {
		ret = vpp_matrix_update(vf, vinfo, flags, vd_path);
		if (ret == 1) {
			pr_csc(2, "vd%d: hold frame when output changing\n",
				vd_path + 1);
			return 1;
		} else if (ret == 2) {
			pr_csc(2, "vd%d: hold frame when video off\n",
				vd_path + 1);
			return 2;
		}
	} else if ((vf != NULL) && (flags & CSC_FLAG_TOGGLE_FRAME)) {
		if (is_video_layer_on(vd_path) ||
		video_layer_wait_on[vd_path] ||
		video_process_flags[vd_path] & PROC_FLAG_FORCE_PROCESS) {
			if (video_process_status[vd_path] != HDR_MODULE_ON) {
				video_process_status[vd_path] = HDR_MODULE_ON;
				pr_csc(4,
				       "video_process_status[%s] = HDR_MODULE_ON\n",
				       vd_path == VD1_PATH ? "VD1" : "VD2");
			}
			vpp_matrix_update(vf, vinfo, flags, vd_path);
			video_process_flags[vd_path] &=
				~PROC_FLAG_FORCE_PROCESS;
			memcpy(&last_vf_backup[vd_path], vf,
			       sizeof(struct vframe_s));
			last_vf[vd_path] = &last_vf_backup[vd_path];
			last_vf_signal_type[vd_path] = vf->signal_type;
			null_vf_cnt[vd_path] = 0;
			pr_csc(2, "vd%d: process toggle frame(%p) %x\n",
				vd_path + 1,
				vf, vf->signal_type);
		} else
			pr_csc(2, "vd%d: skip process frame(%p) %x\n",
				vd_path + 1,
				vf, vf->signal_type);
		fg_vf_sw_dbg = 1;
		/* debug vframe info backup */
		dbg_vf = vf;
	} else if (vf_rpt &&
		(is_video_layer_on(vd_path) ||
		video_layer_wait_on[vd_path])) {
		if ((video_process_flags[vd_path] & PROC_FLAG_FORCE_PROCESS) ||
		    cap_changed) {
			if (video_process_status[vd_path] != HDR_MODULE_ON) {
				video_process_status[vd_path] = HDR_MODULE_ON;
				pr_csc(4,
				       "video_process_status[%s] = HDR_MODULE_ON\n",
				       vd_path == VD1_PATH ? "VD1" : "VD2");
			}
			vpp_matrix_update(vf_rpt, vinfo, flags, vd_path);
			video_process_flags[vd_path] &=
				~PROC_FLAG_FORCE_PROCESS;
			pr_csc(4, "vd%d: rpt and process frame(%p)\n",
			       vd_path + 1, vf_rpt);
		} else {
			pr_csc(2, "vd%d: rpt frame(%p)\n", vd_path + 1, vf_rpt);
		}
		null_vf_cnt[vd_path] = 0;
		fg_vf_sw_dbg = 2;
	} else if (last_vf[vd_path] &&
		(is_video_layer_on(vd_path) ||
		video_layer_wait_on[vd_path])) {
		if (video_process_flags[vd_path] & PROC_FLAG_FORCE_PROCESS ||
		    cap_changed) {
			vpp_matrix_update(
				last_vf[vd_path], vinfo, flags, vd_path);
			video_process_flags[vd_path] &=
				~PROC_FLAG_FORCE_PROCESS;
			pr_csc(4, "vd%d: rpt and process frame local(%p)\n",
			       vd_path + 1, last_vf[vd_path]);
		} else {
			pr_csc(2, "vd%d: rpt frame local\n",
			       vd_path + 1);
		}
		null_vf_cnt[vd_path] = 0;
		fg_vf_sw_dbg = 3;
	} else {
		bool force_fake = false;

		last_vf[vd_path] = NULL;

		if (null_vf_cnt[vd_path] <= null_vf_max)
			null_vf_cnt[vd_path]++;

		/* handle change between TV support/not support HDR */
		if (cap_changed) {
			if (is_video_layer_on(vd_path))
				null_vf_cnt[vd_path] = 0;
			else
				force_fake = true;
			pr_csc(4, "vd%d: sink cap changed when idle\n",
			       vd_path + 1);
		}

		/* handle change between output mode*/
		if (prev_color_fmt != vinfo->viu_color_fmt) {
			if (is_video_layer_on(vd_path))
				null_vf_cnt[vd_path] = 0;
			else
				force_fake = true;
			prev_color_fmt = vinfo->viu_color_fmt;
			pr_csc(4, "vd%d: output color format changed\n",
			       vd_path + 1);
		}

		/* handle eye protect mode */
		if ((cur_eye_protect_mode != wb_val[0]) &&
		    (vd_path == VD1_PATH)) {
			if (is_video_layer_on(vd_path))
				null_vf_cnt[vd_path] = 0;
			else
				force_fake = true;
			pr_csc(4, "vd%d: eye_protect_mode changed\n",
			       vd_path + 1);
		}

		/* gxl handle sdr_mode change bug fix. */
		if ((sink_hdr_support(vinfo) & HDR_SUPPORT) &&
		    !cpu_after_eq(MESON_CPU_MAJOR_ID_G12A) &&
		    vinfo->viu_color_fmt != COLOR_FMT_RGB444) {
			if (sdr_mode != cur_sdr_mode) {
				force_fake = true;
				cur_sdr_mode = sdr_mode;
				pr_csc(4, "vd%d: sdr_mode changed\n",
				       vd_path + 1);
			}
		}

		/* handle sdr_mode change */
		if (is_video_layer_on(vd_path) &&
		    (sink_hdr_support(vinfo) & HDR_SUPPORT) &&
		    ((cpu_after_eq(MESON_CPU_MAJOR_ID_GXL)) &&
		     (vinfo->viu_color_fmt != COLOR_FMT_RGB444))) {
			if (cpu_after_eq(MESON_CPU_MAJOR_ID_G12A)) {
				if (sdr_mode != cur_sdr_mode) {
					null_vf_cnt[vd_path] = 0;
					cur_sdr_mode = sdr_mode;
					pr_csc(4, "vd%d: sdr_mode changed\n",
					       vd_path + 1);
				}
			} else {
				if (((sdr_process_mode[vd_path] !=
					PROC_SDR_TO_HDR) &&
					(sdr_mode > 0)) ||
					((sdr_process_mode[vd_path]
					> PROC_BYPASS) &&
					(sdr_process_mode[vd_path]
					!= PROC_OFF) &&
					(sdr_mode == 0))) {
					null_vf_cnt[vd_path] = 0;
					pr_csc(4, "vd%d: sdr_mode changed\n",
					       vd_path + 1);
				}
			}
		}
		if (!is_dolby_vision_enable() &&
		    get_hdr_policy() != cur_hdr_policy) {
			null_vf_cnt[vd_path] = 1;
			toggle_frame = 1;
		} else if (!is_video_layer_on(vd_path) &&
			(video_process_status[vd_path]
			== HDR_MODULE_ON)) {
			null_vf_cnt[vd_path] = 1;
			toggle_frame = 1;
		} else if (force_fake) {
			null_vf_cnt[vd_path] = 1;
			toggle_frame = 1;
		} else if (csc_en & 0x10) {
			toggle_frame = null_vf_max;
		} else {
			toggle_frame = 0;
		}
		if (null_vf_cnt[vd_path] == toggle_frame) {
			if (null_vf_cnt[vd_path] == 0) {
				/* video on */
				video_process_flags[vd_path] |=
					PROC_FLAG_FORCE_PROCESS;
				return 0;
			}
			/* video off */
			if (is_dolby_vision_enable()) {
				/* dolby enable */
				pr_csc(8,
				       "vd%d: %d %d Fake SDR frame%s, dolby on=%d, dolby policy =%d\n",
				       vd_path + 1,
				       null_vf_cnt[vd_path],
				       toggle_frame,
				       is_video_layer_on(vd_path) ?
				       " " : ", video off",
				       is_dolby_vision_on(),
				       get_dolby_vision_policy());
				if (vd_path == VD2_PATH ||
				    (vd_path == VD1_PATH &&
				     (get_dolby_vision_policy() !=
				      DOLBY_VISION_FOLLOW_SINK ||
				      get_source_type(VD1_PATH) ==
				      HDRTYPE_HDR10PLUS ||
				      get_source_type(VD1_PATH)
					== HDRTYPE_HLG ||
					get_source_type(VD1_PATH)
					== HDRTYPE_MVC))) {
					/* and VD1 adaptive or VD2 */
					/* or always hdr hdr+/hlg bypass */
					/* faked vframe to switch matrix */
					/* 2020 to 601 when video disabled */
					/* and bypass hdr module for dv core*/
					send_fake_frame = true;
					video_process_status[vd_path] =
						HDR_MODULE_OFF;
				}
			} else {
				/* dolby disable */
				pr_csc(8,
				       "vd%d: %d %d Fake SDR frame%s, policy =%d, cur_policy =%d\n",
				       vd_path + 1,
				       null_vf_cnt[vd_path],
				       toggle_frame,
				       is_video_layer_on(vd_path) ?
				       " " : ", video off",
				       get_hdr_policy(),
				       cur_hdr_policy);
				send_fake_frame = true;
				if (get_hdr_policy() == 1) {
				/* adaptive hdr */
				/* faked vframe to switch matrix */
				/* turn off hdr module */
					video_process_status[vd_path] =
						HDR_MODULE_OFF;
				} else {
				/* always hdr */
				/* faked vframe to switch matrix */
				/* hdr module to handle osd */
					video_process_status[vd_path] =
						HDR_MODULE_ON;
				}
			}

			if (send_fake_frame) {
				memset(&fake_vframe, 0x0,
					sizeof(struct vframe_s));
				fake_vframe.source_type =
					VFRAME_SOURCE_TYPE_OTHERS;
				fake_vframe.signal_type = 0;
				fake_vframe.width = 1920;
				fake_vframe.height = 1080;
				fake_vframe.prop.
				master_display_colour.present_flag
					= 0x80000000;
				memset(
				&fake_vframe.prop.master_display_colour,
				0,
				sizeof(struct vframe_master_display_colour_s));
				vpp_matrix_update(
					&fake_vframe, vinfo,
					CSC_FLAG_TOGGLE_FRAME, vd_path);
				last_vf[vd_path] = NULL;
				null_vf_cnt[vd_path] = null_vf_max + 1;
				fg_vf_sw_dbg = 4;
				dbg_vf = NULL;
				video_process_status[vd_path] =
					HDR_MODULE_BYPASS;
				pr_csc(4,
				       "video_process_status[%s] = HDR_MODULE_BYPASS\n",
				       vd_path == VD1_PATH ? "VD1" : "VD2");
				if ((vd_path == VD2_PATH) &&
				    (is_video_layer_on(VD1_PATH) ||
				     video_layer_wait_on[VD1_PATH]))
					video_process_flags[VD1_PATH] |=
						PROC_FLAG_FORCE_PROCESS;
			}
		}
	}
	return 0;
}

int amvecm_hdr_dbg(u32 sel)
{
	int i, j;
	struct vframe_content_light_level_s *content_light_level;
	/* select debug information */
	if (sel == 1) /* dump reg */
		goto reg_dump;

	if (dbg_vf == NULL)
		goto hdr_dump;

	/*pr_err("----vframe info----\n");*/
	/*pr_err("index:%d, type:0x%x, type_backup:0x%x, blend_mode:%d\n",*/
	/*	dbg_vf->index, dbg_vf->type,*/
	/*	dbg_vf->type_backup, dbg_vf->blend_mode);*/
	/*pr_err("duration:%d, duration_pulldown:%d, pts:%d, flag:0x%x\n",*/
	/*	dbg_vf->duration, dbg_vf->duration_pulldown,*/
	/*	dbg_vf->pts, dbg_vf->flag);*/
	/*pr_err("canvas0Addr:0x%x, canvas1Addr:0x%x, bufWidth:%d\n",*/
	/*	dbg_vf->canvas0Addr, dbg_vf->canvas1Addr,*/
	/*	dbg_vf->bufWidth);*/
	/*pr_err("width:%d, height:%d, ratio_control:0x%x, bitdepth:%d\n",*/
	/*	dbg_vf->width, dbg_vf->height,*/
	/*	dbg_vf->ratio_control, dbg_vf->bitdepth);*/
	/*pr_err("signal_type:%x, orientation:%d, video_angle:0x%x\n",*/
	/*	dbg_vf->signal_type, dbg_vf->orientation,*/
	/*	dbg_vf->video_angle);*/
	/*pr_err("source_type:%d, phase:%d, soruce_mode:%d, sig_fmt:0x%x\n",*/
	/*	dbg_vf->source_type, dbg_vf->phase,*/
	/*	dbg_vf->source_mode, dbg_vf->sig_fmt);*/
	/*pr_err(*/
	/*	"trans_fmt 0x%x, lefteye(%d %d %d %d),*/
	/*righteye(%d %d %d %d)\n",*/
	/*	vf->trans_fmt, vf->left_eye.start_x, vf->left_eye.start_y,*/
	/*	vf->left_eye.width, vf->left_eye.height,*/
	/*	vf->right_eye.start_x, vf->right_eye.start_y,*/
	/*	vf->right_eye.width, vf->right_eye.height);*/
	/*pr_err("mode_3d_enable %d",*/
	/*	vf->mode_3d_enable);*/
	/*pr_err("early_process_fun 0x%p, process_fun 0x%p,*/
	/*private_data %p\n",*/
	/*	vf->early_process_fun, vf->process_fun, vf->private_data);*/

	/*pr_err("hist_pow %d, luma_sum %d, chroma_sum %d, pixel_sum %d\n",*/
	/*	vf->prop.hist.hist_pow, vf->prop.hist.luma_sum,*/
	/*	vf->prop.hist.chroma_sum, vf->prop.hist.pixel_sum);*/

	/*pr_err("height %d, width %d, luma_max %d, luma_min %d\n",*/
	/*	vf->prop.hist.hist_pow, vf->prop.hist.hist_pow,*/
	/*	vf->prop.hist.hist_pow, vf->prop.hist.hist_pow);*/

	/*pr_err("vpp_luma_sum %d, vpp_chroma_sum %d, vpp_pixel_sum %d\n",*/
	/*	vf->prop.hist.vpp_luma_sum, vf->prop.hist.vpp_chroma_sum,*/
	/*	vf->prop.hist.vpp_pixel_sum);*/

	/*pr_err("vpp_height %d, vpp_width %d, vpp_luma_max %d,*/
	/* vpp_luma_min %d\n",*/
	/*	vf->prop.hist.vpp_height, vf->prop.hist.vpp_width,*/
	/*	vf->prop.hist.vpp_luma_max, vf->prop.hist.vpp_luma_min);*/

	/*pr_err("vs_span_cnt %d, vs_cnt %d, hs_cnt0 %d, hs_cnt1 %d\n",*/
	/*	vf->prop.meas.vs_span_cnt, vf->prop.meas.vs_cnt,*/
	/*	vf->prop.meas.hs_cnt0, vf->prop.meas.hs_cnt1);*/

	/*pr_err("hs_cnt2 %d, vs_cnt %d, hs_cnt3 %d,*/
	/* vs_cycle %d, vs_stamp %d\n",*/
	/*	vf->prop.meas.hs_cnt2, vf->prop.meas.hs_cnt3,*/
	/*	vf->prop.meas.vs_cycle, vf->prop.meas.vs_stamp);*/

	/*pr_err("pixel_ratio:%d list:%p ready_jiffies64:%lld,*/
	/*frame_dirty %d\n",*/
	/*	dbg_vf->pixel_ratio, &dbg_vf->list,*/
	/*	dbg_vf->ready_jiffies64, dbg_vf->frame_dirty);*/

	pr_err("----Video frame info----\n");
	pr_err("bitdepth:0x%x, signal_type:0x%x, present_flag:0x%x\n",
		dbg_vf->bitdepth,
		dbg_vf->signal_type,
		dbg_vf->prop.master_display_colour.present_flag);

	if (((dbg_vf->signal_type >> 16) & 0xff) == 9) {
		pr_err("HDR color primaries:0x%x\n",
			((dbg_vf->signal_type >> 16) & 0xff));
		pr_err("HDR transfer_characteristic:0x%x\n",
			((dbg_vf->signal_type >> 8) & 0xff));
	} else
		pr_err("SDR color primaries:0x%x\n", signal_color_primaries);

	if (dbg_vf->prop.master_display_colour.present_flag == 1) {
		pr_err("----SEI info----\n");
		for (i = 0; i < 3; i++)
			for (j = 0; j < 2; j++)
				pr_err(
					"\tprimaries[%1d][%1d] = %04x\n",
				i, j,
			dbg_vf->prop.master_display_colour.primaries[i][j]);
		pr_err("\twhite_point = (%04x, %04x)\n",
			dbg_vf->prop.master_display_colour.white_point[0],
			dbg_vf->prop.master_display_colour.white_point[1]);
		pr_err("\tmax,min luminance = %08x, %08x\n",
			dbg_vf->prop.master_display_colour.luminance[0],
			dbg_vf->prop.master_display_colour.luminance[1]);
		content_light_level =
			&dbg_vf->prop.master_display_colour.content_light_level;
		pr_err("\tcontent_light_level.present_flag = %08x\n",
			content_light_level->present_flag);
		pr_err("\tmax_content,min max_pic_average = %08x, %08x\n",
			content_light_level->max_content,
			content_light_level->max_pic_average);
	}

	pr_err(HDR_VERSION);
hdr_dump:
	pr_err("----HDR process info----\n");
	pr_err("customer_master_display_en:0x%x\n", customer_master_display_en);

	pr_err("hdr_mode:0x%x, hdr_process_mode:0x%x, cur_hdr_process_mode:0x%x\n",
		hdr_mode, hdr_process_mode[0], cur_hdr_process_mode[0]);

	pr_err("hlg_process_mode: 0x%x :0->hlg->hlg,1->hlg->sdr,2->hlg->hdr10\n",
		hlg_process_mode[0]);

	pr_err("sdr_mode:0x%x, sdr_process_mode:0x%x, cur_sdr_process_mode:0x%x\n",
		sdr_mode, sdr_process_mode[0], cur_sdr_process_mode[0]);

	pr_err("hdr_flag:0x%x,     fg_vf_sw_dbg:0x%x\n",
		hdr_flag, fg_vf_sw_dbg);
	pr_err("cur_signal_type:0x%x, cur_csc_mode:0x%x, cur_csc_type:0x%x\n",
		cur_signal_type[0], cur_csc_mode, cur_csc_type[0]);

	pr_err("knee_lut_on:0x%x,knee_interpolation_mode:0x%x,cur_knee_factor:0x%x\n",
		knee_lut_on, knee_interpolation_mode, cur_knee_factor);

	pr_err("tx_hdr10_plus_support = 0x%x\n", tx_hdr10_plus_support);
	pr_err("hdr10_plus_process_mode = 0x%x\n", hdr10_plus_process_mode[0]);

	//if (signal_transfer_characteristic == 0x30)
	if (cur_csc_type[0] == VPP_MATRIX_BT2020YUV_BT2020RGB_DYNAMIC)
		hdr10_plus_debug();

	if ((receiver_hdr_info.hdr_support & 0xc) == 0)
		goto dbg_end;
	pr_err("----TV EDID info----\n");
	pr_err("hdr_support:0x%x, lumi_max:%d, lumi_avg:%d, lumi_min:%d\n",
		receiver_hdr_info.hdr_support,
		receiver_hdr_info.lumi_max,
		receiver_hdr_info.lumi_avg,
		receiver_hdr_info.lumi_min);

	pr_err("----Tx HDR package info----\n");
	pr_err("\tfeatures = 0x%08x\n", dbg_hdr_send.features);
	for (i = 0; i < 3; i++)
		for (j = 0; j < 2; j++)
			pr_err(
				"\tprimaries[%1d][%1d] = %04x\n",
				i, j,
				dbg_hdr_send.primaries[i][j]);
	pr_err("\twhite_point = (%04x, %04x)\n",
		dbg_hdr_send.white_point[0],
		dbg_hdr_send.white_point[1]);
	pr_err("\tmax,min luminance = %08x, %08x\n",
		dbg_hdr_send.luminance[0], dbg_hdr_send.luminance[1]);

	goto dbg_end;

	/************************dump reg start***************************/
reg_dump:

	/* osd matrix, VPP_MATRIX_0 */
	pr_err("----dump regs VPP_MATRIX_OSD----\n");
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_PRE_OFFSET0_1,
		READ_VPP_REG(VIU_OSD1_MATRIX_PRE_OFFSET0_1));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_PRE_OFFSET2,
		READ_VPP_REG(VIU_OSD1_MATRIX_PRE_OFFSET2));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF00_01,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF00_01));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF02_10,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF02_10));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF11_12,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF11_12));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF20_21,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF20_21));

	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF22_30,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF22_30));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF31_32,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF31_32));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF40_41,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF40_41));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COLMOD_COEF42,
		READ_VPP_REG(VIU_OSD1_MATRIX_COLMOD_COEF42));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COEF22_30,
		READ_VPP_REG(VIU_OSD1_MATRIX_COEF22_30));

	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_OFFSET0_1,
		READ_VPP_REG(VIU_OSD1_MATRIX_OFFSET0_1));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_OFFSET2,
		READ_VPP_REG(VIU_OSD1_MATRIX_OFFSET2));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COLMOD_COEF42,
		READ_VPP_REG(VIU_OSD1_MATRIX_COLMOD_COEF42));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_COLMOD_COEF42,
		READ_VPP_REG(VIU_OSD1_MATRIX_COLMOD_COEF42));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_PRE_OFFSET0_1,
		READ_VPP_REG(VIU_OSD1_MATRIX_PRE_OFFSET0_1));
	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_MATRIX_CTRL,
		READ_VPP_REG(VIU_OSD1_MATRIX_CTRL));

	/* osd eotf matrix, VPP_MATRIX_OSD_EOTF */
	pr_err("----dump regs VPP_MATRIX_OSD_EOTF----\n");

	for (i = 0; i < 5; i++)
		pr_err("\taddr = %08x, val = %08x\n",
			(VIU_OSD1_EOTF_CTL + i + 1),
			READ_VPP_REG(VIU_OSD1_EOTF_CTL + i + 1));

	pr_err("\taddr = %08x, val = %08x\n",
		VIU_OSD1_EOTF_CTL,
		READ_VPP_REG(VIU_OSD1_EOTF_CTL));

	{
		unsigned short *r_map;
		unsigned short *g_map;
		unsigned short *b_map;
		unsigned int addr_port;
		unsigned int data_port;
		unsigned int ctrl_port;
		unsigned int data;
		int i;

		r_map = kmalloc(sizeof(unsigned short) *
			VIDEO_OETF_LUT_SIZE * 3, GFP_ATOMIC);
		if (!r_map)
			return 0;
		g_map = &r_map[sizeof(unsigned short) *
			VIDEO_OETF_LUT_SIZE * 1];
		b_map = &r_map[sizeof(unsigned short) *
			VIDEO_OETF_LUT_SIZE * 2];

		pr_err("----dump regs VPP_LUT_OSD_OETF----\n");

		addr_port = VIU_OSD1_OETF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_OETF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_OETF_CTL;

		pr_err("\taddr = %08x, val = %08x\n",
			ctrl_port, READ_VPP_REG(ctrl_port));

		for (i = 0; i < 20; i++) {
			WRITE_VPP_REG(addr_port, i);
			data = READ_VPP_REG(data_port);
			r_map[i * 2] = data & 0xffff;
			r_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		WRITE_VPP_REG(addr_port, 20);
		data = READ_VPP_REG(data_port);
		r_map[OSD_OETF_LUT_SIZE - 1] = data & 0xffff;
		g_map[0] = (data >> 16) & 0xffff;
		for (i = 0; i < 20; i++) {
			WRITE_VPP_REG(addr_port, 21 + i);
			data = READ_VPP_REG(data_port);
			g_map[i * 2 + 1] = data & 0xffff;
			g_map[i * 2 + 2] = (data >> 16) & 0xffff;
		}
		for (i = 0; i < 20; i++) {
			WRITE_VPP_REG(addr_port, 41 + i);
			data = READ_VPP_REG(data_port);
			b_map[i * 2] = data & 0xffff;
			b_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		WRITE_VPP_REG(addr_port, 61);
		data = READ_VPP_REG(data_port);
		b_map[OSD_OETF_LUT_SIZE - 1] = data & 0xffff;

		for (i = 0; i < OSD_OETF_LUT_SIZE; i++) {
			pr_err("\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i, r_map[i], g_map[i], b_map[i]);
		}

		addr_port = VIU_OSD1_EOTF_LUT_ADDR_PORT;
		data_port = VIU_OSD1_EOTF_LUT_DATA_PORT;
		ctrl_port = VIU_OSD1_EOTF_CTL;
		pr_err("----dump regs VPP_LUT_OSD_EOTF----\n");
		WRITE_VPP_REG(addr_port, 0);
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			r_map[i * 2] = data & 0xffff;
			r_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		data = READ_VPP_REG(data_port);
		r_map[EOTF_LUT_SIZE - 1] = data & 0xffff;
		g_map[0] = (data >> 16) & 0xffff;
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			g_map[i * 2 + 1] = data & 0xffff;
			g_map[i * 2 + 2] = (data >> 16) & 0xffff;
		}
		for (i = 0; i < 16; i++) {
			data = READ_VPP_REG(data_port);
			b_map[i * 2] = data & 0xffff;
			b_map[i * 2 + 1] = (data >> 16) & 0xffff;
		}
		data = READ_VPP_REG(data_port);
		b_map[EOTF_LUT_SIZE - 1] = data & 0xffff;

		for (i = 0; i < EOTF_LUT_SIZE; i++) {
			pr_err("\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i, r_map[i], g_map[i], b_map[i]);
		}

		pr_err("----dump hdr_osd_reg structure ----\n");

		pr_err("\tviu_osd1_matrix_ctrl = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_ctrl);
		pr_err("\tviu_osd1_matrix_coef00_01 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef00_01);
		pr_err("\tviu_osd1_matrix_coef02_10 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef02_10);
		pr_err("\tviu_osd1_matrix_coef11_12 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef11_12);
		pr_err("\tviu_osd1_matrix_coef20_21 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef20_21);
		pr_err("\tviu_osd1_matrix_colmod_coef42 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_colmod_coef42);
		pr_err("\tviu_osd1_matrix_offset0_1 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_offset0_1);
		pr_err("\tviu_osd1_matrix_offset2 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_offset2);
		pr_err("\tviu_osd1_matrix_pre_offset0_1 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_pre_offset0_1);
		pr_err("\tviu_osd1_matrix_pre_offset2 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_pre_offset2);
		pr_err("\tviu_osd1_matrix_coef22_30 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef22_30);
		pr_err("\tviu_osd1_matrix_coef31_32 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef31_32);
		pr_err("\tviu_osd1_matrix_coef40_41 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_matrix_coef40_41);
		pr_err("\tviu_osd1_eotf_ctl = 0x%04x\n",
				hdr_osd_reg.viu_osd1_eotf_ctl);
		pr_err("\tviu_osd1_eotf_coef00_01 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_eotf_coef00_01);
		pr_err("\tviu_osd1_eotf_coef02_10 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_eotf_coef02_10);
		pr_err("\tviu_osd1_eotf_coef11_12 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_eotf_coef11_12);
		pr_err("\tviu_osd1_eotf_coef20_21 = 0x%04x\n",
				hdr_osd_reg.viu_osd1_eotf_coef20_21);
		pr_err("\tviu_osd1_eotf_coef22_rs = 0x%04x\n",
				hdr_osd_reg.viu_osd1_eotf_coef22_rs);
		pr_err("\tviu_osd1_oetf_ctl = 0x%04x\n",
				hdr_osd_reg.viu_osd1_oetf_ctl);

		for (i = 0; i < EOTF_LUT_SIZE; i++) {
			pr_err("\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i,
				hdr_osd_reg.lut_val.r_map[i],
				hdr_osd_reg.lut_val.g_map[i],
				hdr_osd_reg.lut_val.b_map[i]);
		}
		for (i = 0; i < OSD_OETF_LUT_SIZE; i++) {
			pr_err("\t[%d] = 0x%04x 0x%04x 0x%04x\n",
				i,
				hdr_osd_reg.lut_val.or_map[i],
				hdr_osd_reg.lut_val.og_map[i],
				hdr_osd_reg.lut_val.ob_map[i]);
		}
		pr_err("\n");
		kfree(r_map);
	}
	pr_err("----dump regs VPP_LUT_EOTF----\n");
	print_vpp_lut(VPP_LUT_EOTF, READ_VPP_REG(VIU_EOTF_CTL) & (7 << 27));
	pr_err("----dump regs VPP_LUT_OETF----\n");
	print_vpp_lut(VPP_LUT_OETF, READ_VPP_REG(XVYCC_LUT_CTL) & 0x7f);
	/*********************dump reg end*********************/
dbg_end:

	return 0;
}
