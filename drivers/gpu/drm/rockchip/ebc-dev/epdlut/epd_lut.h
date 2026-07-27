// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#ifndef EPD_LUT_H
#define EPD_LUT_H

#define MAXFRAME    255

#define DEFAULT_BW_GHOST_RM_LEVEL 1
#define DEFAULT_BW_BALANCE_CTL 1
#define DEFAULT_BW_GHOST_RM_TYPE 2

#define WF_4BIT	16
#define WF_5BIT	32

// same to pvi_wf_mode
enum epd_lut_type {
	WF_TYPE_RESET = 0,
	WF_TYPE_GRAY2,	// like DU
	WF_TYPE_GRAY4,  // like DU4
	WF_TYPE_GC16,
	WF_TYPE_GL16,
	WF_TYPE_GLR16,
	WF_TYPE_GLD16,
	WF_TYPE_A2,
	WF_TYPE_GCC16,
	PVI_WF_MAX,

	WF_TYPE_OVERLAY,  // for overlay mode
	WF_TYPE_RK_GLR16, // for part regal mode
	WF_TYPE_RK_GLD16, // for part regal mode
	WF_TYPE_RK_GL16, // for part gl16 mode
	WF_TYPE_RK_GC16, // for part gc16 mode
	WF_TYPE_RK_FULL, // for full mode
	WF_TYPE_MAX,
	WF_TYPE_GRAY16,
};

struct epd_lut_data {
	int display_frame_num;
	int overlay_frame_num;
	int bw_ghost_rm_num;
	unsigned int *data;
	u8 *wf_table[2];
};

struct epd_lut_info {
	int pic;
	int wf_fix;
	int normal_repair;
	int regal_repair;
	int a2_enter;
	int swap_2bit;
	int bw_ghost_rm_level;
};

/*
 * EPD LUT module export symbols
 */
int epd_lut_from_mem_init(void *waveform);
int epd_lut_from_file_init(struct device *dev, void *waveform, int size);
const char *epd_lut_get_wf_version(void);
int epd_lut_get_wf_bit(void);
int epd_lut_get(struct epd_lut_data *output, enum epd_lut_type lut_type, u16 temperature, struct epd_lut_info lut_info);
int epd_lut_get_original(struct epd_lut_data *output, enum epd_lut_type lut_type, int temperature, int pic);

//you can change overlay lut mode here
int epd_overlay_lut(void);
int epd_gray2_last_repair(u8 *wf_table, int frame_num, int *bw_ghost_rm_num, int bw_ghost_rm_level);
int epd_overlay_gray2_repair(u8 *wf_table, int frame_num);
int epd_normal_repair(u8 *wf_table, int frame_num);
int epd_regal_repair(u8 *wf_table, int frame_num, int regal_repair);

/*
 * PVI Waveform Interfaces
 */
int pvi_wf_input(void *waveform_file);
int pvi_wf_add_custom_mode_table(u8 *table, int size);
const char *pvi_wf_get_version(void);
int pvi_wf_get_wf_bit(void);
int pvi_wf_get_lut(struct epd_lut_data *output, enum epd_lut_type lut_type, u16 temperature, struct epd_lut_info lut_info);
int pvi_wf_get_original_lut(struct epd_lut_data *output, enum epd_lut_type lut_type, int temperature, int pic);

/*
 * RKF Waveform Interfaces
 */
int rkf_wf_input(void *waveform_file);
const char *rkf_wf_get_version(void);
int rkf_wf_get_wf_bit(void);
int rkf_wf_get_lut(struct epd_lut_data *output, enum epd_lut_type lut_type, u16 temperature, struct epd_lut_info lut_info);

#if IS_ENABLED(CONFIG_EPD_EXTEND_WAVEFORM)
/* Extend waveform */
int extend_wf_input(void *waveform_file);
int extend_wf_get_lut(struct epd_lut_data *output, enum epd_lut_type lut_type, u16 temperature,
		      struct epd_lut_info lut_info);
int extend_wf_get_wf_bit(void);
const char *extend_wf_get_version(void);
#endif

#endif
