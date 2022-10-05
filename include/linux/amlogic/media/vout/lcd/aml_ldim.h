/*
 * include/linux/amlogic/media/vout/lcd/aml_ldim.h
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

#ifndef _INC_AML_LDIM_H_
#define _INC_AML_LDIM_H_
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/media/vout/lcd/aml_bl.h>
#include <linux/amlogic/media/vout/lcd/aml_lcd.h>
#include <linux/amlogic/media/vout/lcd/ldim_alg.h>
#include <linux/spi/spi.h>

/*#define LDIM_DEBUG_INFO*/
#define LDIMPR(fmt, args...)     pr_info("ldim: "fmt"", ## args)
#define LDIMERR(fmt, args...)    pr_err("ldim: error: "fmt"", ## args)

#define LD_DATA_DEPTH   12
#define LD_DATA_MIN     10
#define LD_DATA_MAX     0xfff

#define _VE_LDIM  'C'

/* VPP.ldim IOCTL command list */
#define LDIM_IOC_PARA  _IOW(_VE_LDIM, 0x50, struct ldim_param_s)

enum ldim_dev_type_e {
	LDIM_DEV_TYPE_NORMAL = 0,
	LDIM_DEV_TYPE_SPI,
	LDIM_DEV_TYPE_I2C,
	LDIM_DEV_TYPE_MAX,
};

struct ldim_config_s {
	unsigned short hsize;
	unsigned short vsize;
	unsigned char row;
	unsigned char col;
	unsigned char bl_mode;
	unsigned char bl_en;
	unsigned char hvcnt_bypass;
};

#define LDIM_INIT_ON_MAX     300
#define LDIM_INIT_OFF_MAX    20
struct ldim_dev_config_s {
	char name[20];
	char pinmux_name[20];
	unsigned char type;
	int cs_hold_delay;
	int cs_clk_delay;
	int en_gpio;
	int en_gpio_on;
	int en_gpio_off;
	int lamp_err_gpio;
	unsigned char fault_check;
	unsigned char write_check;

	unsigned int dim_min;
	unsigned int dim_max;

	unsigned char init_loaded;
	unsigned char cmd_size;
	unsigned char *init_on;
	unsigned char *init_off;
	unsigned int init_on_cnt;
	unsigned int init_off_cnt;

	struct bl_pwm_config_s ldim_pwm_config;
	struct bl_pwm_config_s analog_pwm_config;

	unsigned short bl_regnum;
	unsigned short bl_mapping[LD_BLKREGNUM];

	void (*dim_range_update)(void);
	int (*dev_reg_write)(unsigned char *buf, unsigned int len);
	int (*dev_reg_read)(unsigned char *buf, unsigned int len);
};

/*******global API******/
struct aml_ldim_driver_s {
	unsigned char valid_flag;
	unsigned char dev_index;
	unsigned char static_pic_flag;
	unsigned char vsync_change_flag;
	unsigned char pinmux_flag;

	struct ldim_config_s *ldim_conf;
	struct ldim_dev_config_s *ldev_conf;
	unsigned int *hist_matrix;
	unsigned int *max_rgb;
	unsigned short *ldim_test_matrix;
	unsigned short *local_ldim_matrix;
	unsigned short *ldim_matrix_buf;
	int (*init)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int (*set_level)(unsigned int level);
	int (*pinmux_ctrl)(int status);
	int (*pwm_vs_update)(void);
	int (*device_power_on)(void);
	int (*device_power_off)(void);
	int (*device_bri_update)(unsigned short *buf, unsigned char len);
	int (*device_bri_check)(void);
	void (*config_print)(void);
	void (*test_ctrl)(int flag);
	struct pinctrl *pin;
	struct device *dev;
	struct spi_device *spi_dev;
	struct spi_board_info *spi_info;
};

struct ldim_param_s {
	/* beam model */
	int rgb_base;
	int boost_gain;
	int lpf_res;
	int fw_ld_th_sf; /* spatial filter threshold */
	/* beam curve */
	int ld_vgain;
	int ld_hgain;
	int ld_litgain;
	int ld_lut_vdg_lext;
	int ld_lut_hdg_lext;
	int ld_lut_vhk_lext;
	int ld_lut_hdg[32];
	int ld_lut_vdg[32];
	int ld_lut_vhk[32];
	/* beam shape minor adjustment */
	int ld_lut_vhk_pos[32];
	int ld_lut_vhk_neg[32];
	int ld_lut_hhk[32];
	int ld_lut_vho_pos[32];
	int ld_lut_vho_neg[32];
	/* remapping */
	int lit_idx_th;
	int comp_gain;
};

extern struct aml_ldim_driver_s *aml_ldim_get_driver(void);

extern int aml_ldim_get_config_dts(struct device_node *child);
extern int aml_ldim_get_config_unifykey(unsigned char *buf);
extern int aml_ldim_probe(struct platform_device *pdev);
extern int aml_ldim_remove(void);

#endif

