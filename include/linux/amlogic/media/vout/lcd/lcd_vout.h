/*
 * include/linux/amlogic/media/vout/lcd/lcd_vout.h
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

#ifndef _INC_LCD_VOUT_H
#define _INC_LCD_VOUT_H
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#include <linux/amlogic/iomap.h>

extern void lcd_vlock_m_update(unsigned int vlock_m);
extern void lcd_vlock_frac_update(unsigned int vlock_farc);

/* **********************************
 * debug print define
 * **********************************
 */
#define LCD_DEBUG_LEVEL_NORMAL    (1 << 0)
#define LCD_DEBUG_LEVEL_CLK       (1 << 1)
#define LCD_DEBUG_LEVEL_VSYNC     (1 << 3)

#define LCD_DEBUG_VSYNC_INTERVAL  60

/* #define LCD_DEBUG_INFO */
extern unsigned char lcd_debug_print_flag;
#define LCDPR(fmt, args...)     pr_info("lcd: "fmt"", ## args)
#define LCDERR(fmt, args...)    pr_err("lcd: error: "fmt"", ## args)

/* **********************************
 * clk parameter bit define
 * pll_ctrl, div_ctrl, clk_ctrl
 * **********************************
 */
/* ******** pll_ctrl ******** */
#define PLL_CTRL_OD3                20 /* [21:20] */
#define PLL_CTRL_OD2                18 /* [19:18] */
#define PLL_CTRL_OD1                16 /* [17:16] */
#define PLL_CTRL_N                  9  /* [13:9] */
#define PLL_CTRL_M                  0  /* [8:0] */

/* ******** div_ctrl ******** */
#define DIV_CTRL_EDP_DIV1           24 /* [26:24] */
#define DIV_CTRL_EDP_DIV0           20 /* [23:20] */
#define DIV_CTRL_DIV_SEL            8 /* [15:8] */
#define DIV_CTRL_XD                 0 /* [7:0] */

/* ******** clk_ctrl ******** */
#define CLK_CTRL_LEVEL              28 /* [30:28] */
#define CLK_CTRL_FRAC               0  /* [18:0] */

/* **********************************
 * VENC to TCON sync delay
 * **********************************
 */
#define TTL_DELAY                   13
#define PRE_DE_DELAY                8


/* **********************************
 * global control define
 * **********************************
 */
enum lcd_mode_e {
	LCD_MODE_TV = 0,
	LCD_MODE_TABLET,
	LCD_MODE_MAX,
};


enum lcd_chip_e {
	LCD_CHIP_GXL = 0,
	LCD_CHIP_GXM,   /* 1 */
	LCD_CHIP_TXL,   /* 2 */
	LCD_CHIP_TXLX,  /* 3 */
	LCD_CHIP_AXG,   /* 4 */
	LCD_CHIP_G12A,  /* 5 */
	LCD_CHIP_G12B,  /* 6 */
	LCD_CHIP_TL1,   /* 7 */
	LCD_CHIP_SM1,	/* 8 */
	LCD_CHIP_MAX,
};

struct lcd_data_s {
	enum lcd_chip_e chip_type;
	const char *chip_name;
	int *reg_map_table;
};

enum lcd_type_e {
	LCD_TTL = 0,
	LCD_LVDS,
	LCD_VBYONE,
	LCD_MIPI,
	LCD_MLVDS,
	LCD_P2P,
	LCD_TYPE_MAX,
};

#define MOD_LEN_MAX        30
struct lcd_basic_s {
	char model_name[MOD_LEN_MAX];
	enum lcd_type_e lcd_type;
	unsigned short lcd_bits;

	unsigned short h_active;    /* Horizontal display area */
	unsigned short v_active;    /* Vertical display area */
	unsigned short h_period;    /* Horizontal total period time */
	unsigned short v_period;    /* Vertical total period time */
	unsigned short h_period_min;
	unsigned short h_period_max;
	unsigned short v_period_min;
	unsigned short v_period_max;
	unsigned int lcd_clk_min;
	unsigned int lcd_clk_max;

	unsigned int screen_width;  /* screen physical width(unit: mm) */
	unsigned int screen_height; /* screen physical height(unit: mm) */
};

#define LCD_CLK_FRAC_UPDATE     (1 << 0)
#define LCD_CLK_PLL_CHANGE      (1 << 1)
struct lcd_timing_s {
	unsigned char clk_auto; /* clk parameters auto generation */
	unsigned char fr_adjust_type; /* 0=clock, 1=htotal, 2=vtotal */
	unsigned char clk_change; /* internal used */
	unsigned int lcd_clk;   /* pixel clock(unit: Hz) */
	unsigned int lcd_clk_dft; /* internal used */
	unsigned int h_period_dft; /* internal used */
	unsigned int v_period_dft; /* internal used */
	unsigned int pll_ctrl;  /* pll settings */
	unsigned int div_ctrl;  /* divider settings */
	unsigned int clk_ctrl;  /* clock settings */
	unsigned int ss_level; /* [15:12]: ss_freq, [11:8]: ss_mode,
				* [7:0]: ss_level
				*/

	unsigned int sync_duration_num;
	unsigned int sync_duration_den;

	unsigned short video_on_pixel;
	unsigned short video_on_line;

	unsigned short hsync_width;
	unsigned short hsync_bp;
	unsigned short hsync_pol;
	unsigned short vsync_width;
	unsigned short vsync_bp;
	unsigned short vsync_pol;
	/* unsigned int vsync_h_phase; // [31]sign, [15:0]value */
	unsigned int h_offset;
	unsigned int v_offset;

	unsigned short de_hs_addr;
	unsigned short de_he_addr;
	unsigned short de_vs_addr;
	unsigned short de_ve_addr;

	unsigned short hs_hs_addr;
	unsigned short hs_he_addr;
	unsigned short hs_vs_addr;
	unsigned short hs_ve_addr;

	unsigned short vs_hs_addr;
	unsigned short vs_he_addr;
	unsigned short vs_vs_addr;
	unsigned short vs_ve_addr;
};

/* **********************************
 * HDR info define
 * **********************************
 */
struct lcd_optical_info_s {
	unsigned int hdr_support;
	unsigned int features;
	unsigned int primaries_r_x;
	unsigned int primaries_r_y;
	unsigned int primaries_g_x;
	unsigned int primaries_g_y;
	unsigned int primaries_b_x;
	unsigned int primaries_b_y;
	unsigned int white_point_x;
	unsigned int white_point_y;
	unsigned int luma_max;
	unsigned int luma_min;
	unsigned int luma_avg;
};

struct ttl_config_s {
	unsigned int clk_pol;
	unsigned int sync_valid; /* [1]DE, [0]hvsync */
	unsigned int swap_ctrl; /* [1]rb swap, [0]bit swap */
};

#define LVDS_PHY_VSWING_DFT        3
#define LVDS_PHY_PREEM_DFT         0
#define LVDS_PHY_CLK_VSWING_DFT    0
#define LVDS_PHY_CLK_PREEM_DFT     0
struct lvds_config_s {
	unsigned int lvds_vswing;
	unsigned int lvds_repack;
	unsigned int dual_port;
	unsigned int pn_swap;
	unsigned int port_swap;
	unsigned int lane_reverse;
	unsigned int port_sel;
	unsigned int phy_vswing;
	unsigned int phy_preem;
	unsigned int phy_clk_vswing;
	unsigned int phy_clk_preem;
};

#define VX1_PHY_VSWING_DFT        3
#define VX1_PHY_PREEM_DFT         0

#define VX1_PWR_ON_RESET_DLY_DFT  500 /* 500ms */
#define VX1_HPD_DATA_DELAY_DFT    10 /* 10ms */
#define VX1_CDR_TRAINING_HOLD_DFT 200 /* 200ms */

struct vbyone_config_s {
	unsigned int lane_count;
	unsigned int region_num;
	unsigned int byte_mode;
	unsigned int color_fmt;
	unsigned int phy_div;
	unsigned int phy_vswing; /*[4]:ext_pullup, [3:0]vswing*/
	unsigned int phy_preem;
	unsigned int intr_en;
	unsigned int vsync_intr_en;

	unsigned int ctrl_flag;
		/*  bit[0]:power_on_reset_en
		 *  bit[1]:hpd_data_delay_en
		 *  bit[2]:cdr_training_hold_en
		 *  bit[3]:hw_filter_en
		 *  bit[5:4]:sw_filter
		 */

	/* ctrl timing */
	unsigned int power_on_reset_delay; /* ms */
	unsigned int hpd_data_delay; /* ms */
	unsigned int cdr_training_hold; /* ms */
	/* hw filter */
	unsigned int hw_filter_time;
	unsigned int hw_filter_cnt;
};

/* mipi-dsi config */
/* Operation mode parameters */
#define OPERATION_VIDEO_MODE     0
#define OPERATION_COMMAND_MODE   1

#define SYNC_PULSE               0x0
#define SYNC_EVENT               0x1
#define BURST_MODE               0x2

/* command config */
#define DSI_CMD_SIZE_INDEX       1  /* byte[1] */
#define DSI_GPIO_INDEX           2  /* byte[2] */

#define DSI_INIT_ON_MAX          100
#define DSI_INIT_OFF_MAX         30

#define DSI_READ_CNT_MAX         30
struct dsi_read_s {
	unsigned char flag;
	unsigned char reg;
	unsigned char cnt;
	unsigned char *value;
	unsigned char ret_code;

	unsigned int line_start;
	unsigned int line_end;
};

struct dsi_config_s {
	unsigned char lane_num;
	unsigned int bit_rate_max; /* MHz */
	unsigned int bit_rate_min; /* MHz*/
	unsigned int clk_factor; /* bit_rate/pclk */
	unsigned int factor_numerator;
	unsigned int factor_denominator; /* 100 */
	unsigned char operation_mode_init; /* 0=video mode, 1=command mode */
	unsigned char operation_mode_display; /* 0=video mode, 1=command mode */
	unsigned char video_mode_type; /* 0=sync_pulse, 1=sync_event, 2=burst */
	unsigned char clk_always_hs; /* 0=disable, 1=enable */
	unsigned char phy_switch; /* 0=auto, 1=standard, 2=slow */

	unsigned int venc_data_width;
	unsigned int dpi_data_format;

	unsigned char *dsi_init_on;
	unsigned char *dsi_init_off;
	unsigned char extern_init;

	unsigned char check_en;
	unsigned char check_reg;
	unsigned char check_cnt;
	unsigned char check_state;

	unsigned char current_mode;

	struct dsi_read_s *dread;
};

struct mlvds_config_s {
	unsigned int channel_num;
	unsigned int channel_sel0;
	unsigned int channel_sel1;
	unsigned int clk_phase; /* [13:12]=clk01_pi_sel,
				 * [11:8]=pi2, [7:4]=pi1, [3:0]=pi0
				 */
	unsigned int pn_swap;
	unsigned int bit_swap; /* MSB/LSB reverse */
	unsigned int phy_vswing;
	unsigned int phy_preem;

	/* internal used */
	unsigned int pi_clk_sel; /* bit[9:0] */
};

enum p2p_type_e {
	P2P_CEDS = 0,
	P2P_CMPI,
	P2P_ISP,
	P2P_EPI,
	P2P_CHPI = 0x10, /* low common mode */
	P2P_CSPI,
	P2P_USIT,
	P2P_MAX,
};

struct p2p_config_s {
	unsigned int p2p_type;
	unsigned int lane_num;
	unsigned int channel_sel0;
	unsigned int channel_sel1;
	unsigned int pn_swap;
	unsigned int bit_swap; /* MSB/LSB reverse */
	unsigned int phy_vswing;
	unsigned int phy_preem;
};

struct lcd_control_config_s {
	struct ttl_config_s *ttl_config;
	struct lvds_config_s *lvds_config;
	struct vbyone_config_s *vbyone_config;
	struct dsi_config_s *mipi_config;
	struct mlvds_config_s *mlvds_config;
	struct p2p_config_s *p2p_config;
	unsigned int *vlock_param;
};

/* **********************************
 * power control define
 * **********************************
 */
enum lcd_power_type_e {
	LCD_POWER_TYPE_CPU = 0,
	LCD_POWER_TYPE_PMU,
	LCD_POWER_TYPE_SIGNAL,
	LCD_POWER_TYPE_EXTERN,
	LCD_POWER_TYPE_WAIT_GPIO,
	LCD_POWER_TYPE_CLK_SS,
	LCD_POWER_TYPE_MAX,
};

enum lcd_pmu_gpio_e {
	LCD_PMU_GPIO0 = 0,
	LCD_PMU_GPIO1,
	LCD_PMU_GPIO2,
	LCD_PMU_GPIO3,
	LCD_PMU_GPIO4,
	LCD_PMU_GPIO_MAX,
};

#define LCD_CLK_SS_BIT_FREQ             0
#define LCD_CLK_SS_BIT_MODE             4

#define LCD_GPIO_MAX                    0xff
#define LCD_GPIO_OUTPUT_LOW             0
#define LCD_GPIO_OUTPUT_HIGH            1
#define LCD_GPIO_INPUT                  2

/* Power Control */
#define LCD_CPU_GPIO_NUM_MAX         10
struct lcd_cpu_gpio_s {
	char name[15];
	struct gpio_desc *gpio;
	int probe_flag;
	int register_flag;
};

#define LCD_PMU_GPIO_NUM_MAX         3
struct lcd_pmu_gpio_s {
	char name[15];
	int gpio;
};

#define LCD_PWR_STEP_MAX         15
struct lcd_power_step_s {
	unsigned char type;
	unsigned int index; /* point to lcd_cpu/pmu_gpio_s or lcd_extern */
	unsigned int value;
	unsigned int delay;
};

struct lcd_power_ctrl_s {
	struct lcd_cpu_gpio_s cpu_gpio[LCD_CPU_GPIO_NUM_MAX];
	struct lcd_pmu_gpio_s pmu_gpio[LCD_PMU_GPIO_NUM_MAX];
	struct lcd_power_step_s power_on_step[LCD_PWR_STEP_MAX];
	struct lcd_power_step_s power_off_step[LCD_PWR_STEP_MAX];
	int power_on_step_max; /*  internal use for debug */
	int power_off_step_max; /* internal use for debug */
};

struct lcd_boot_ctrl_s {
	unsigned char lcd_type;	//bit[3:0]
	unsigned char lcd_bits; //bit[7:4] bits:6 or 8
	unsigned char advanced_flag;	//bit[15:8]
	unsigned char lcd_init_level;	//bit[19]
	unsigned char debug_print_flag;	//bit[23:20]
	unsigned char debug_test_pattern;	//bit[27:24]
	unsigned char debug_para_source;//bit[29:28]
					//0:normal, 1:dts, 2:unifykey, 3:TBD
	unsigned char debug_lcd_mode;	//bit[31:30]
					//0:normal, 1:tv, 2:tablet, 3:TBD
};

#define LCD_ENABLE_RETRY_MAX    3
struct lcd_config_s {
	char *lcd_propname;
	unsigned int backlight_index;
	unsigned int extern_index;
	struct lcd_basic_s lcd_basic;
	struct lcd_timing_s lcd_timing;
	struct lcd_optical_info_s optical_info;
	struct lcd_control_config_s lcd_control;
	struct lcd_power_ctrl_s *lcd_power;
	struct lcd_boot_ctrl_s *lcd_boot_ctrl;
	struct pinctrl *pin;
	unsigned char pinmux_flag;
	unsigned char change_flag;
	unsigned char retry_enable_flag;
	unsigned char retry_enable_cnt;
};

struct lcd_duration_s {
	unsigned int duration_num;
	unsigned int duration_den;
};

#define LCD_STATUS_IF_ON      (1 << 0)
#define LCD_STATUS_ENCL_ON    (1 << 1)
#define LCD_STATUS_VMODE_ACTIVE  (1 << 2)
#define LCD_STATUS_ON         (LCD_STATUS_IF_ON | LCD_STATUS_ENCL_ON)

#define LCD_MUTE_UPDATE       (1 << 4)
#define LCD_TEST_UPDATE       (1 << 4)

#define LCD_VIU_SEL_NONE      0
struct aml_lcd_drv_s {
	char version[20];
	struct lcd_data_s *data;
	unsigned char lcd_mode;
	unsigned char lcd_status;
	unsigned char lcd_key_valid;
	unsigned char lcd_clk_path; /* 0=hpll, 1=gp0_pll */
	unsigned char lcd_config_load;
	unsigned char lcd_resume_type; /* 0=directly, 1=workqueue */
	unsigned char lcd_auto_test;
	unsigned char lcd_test_state;
	unsigned char lcd_test_flag;
	unsigned char lcd_mute_state;
	unsigned char lcd_mute_flag;
	unsigned char viu_sel;
	unsigned char vsync_none_timer_flag;

	struct device *dev;
	struct lcd_config_s *lcd_config;
	struct vinfo_s *lcd_info;
	struct class *lcd_debug_class;

	int fr_auto_policy;
	int fr_mode;
	struct lcd_duration_s std_duration;

	int tcon_status;

	void (*driver_init_pre)(void);
	void (*driver_disable_post)(void);
	int (*driver_init)(void);
	void (*driver_disable)(void);
	int (*driver_change)(void);
	void (*module_reset)(void);
	void (*module_tiny_reset)(void);
	void (*lcd_screen_black)(void);
	void (*lcd_screen_restore)(void);
	void (*power_ctrl)(int status);

	struct workqueue_struct *workqueue;
	struct delayed_work lcd_probe_delayed_work;
	struct work_struct  lcd_resume_work;
	struct resource *res_vsync_irq;
	struct resource *res_vsync2_irq;
	struct resource *res_vx1_irq;
	struct resource *res_tcon_irq;

	struct mutex power_mutex;
};

extern struct aml_lcd_drv_s *aml_lcd_get_driver(void);


/* **********************************
 * IOCTL define
 * **********************************
 */
#define LCD_IOC_TYPE               'C'
#define LCD_IOC_NR_GET_HDR_INFO    0x0
#define LCD_IOC_NR_SET_HDR_INFO    0x1

#define LCD_IOC_CMD_GET_HDR_INFO   \
	_IOR(LCD_IOC_TYPE, LCD_IOC_NR_GET_HDR_INFO, struct lcd_optical_info_s)
#define LCD_IOC_CMD_SET_HDR_INFO   \
	_IOW(LCD_IOC_TYPE, LCD_IOC_NR_SET_HDR_INFO, struct lcd_optical_info_s)

#endif
