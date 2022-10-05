/*
 * include/linux/amlogic/media/vout/vinfo.h
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

#ifndef _VINFO_H_
#define _VINFO_H_
#include <linux/amlogic/media/vfm/video_common.h>


/* the MSB is represent vmode set by vmode_init */
#define	VMODE_INIT_BIT_MASK	0x8000
#define	VMODE_MODE_BIT_MASK	0xff
#define VMODE_NULL_DISP_MAX	2

enum vmode_e {
	VMODE_HDMI = 0,
	VMODE_CVBS,
	VMODE_LCD,
	VMODE_NULL, /* null mode is used as temporary witch mode state */
	VMODE_INVALID,
	VMODE_DUMMY_LCD,
	VMODE_MAX,
	VMODE_INIT_NULL,
	VMODE_MASK = 0xFF,
};

enum viu_mux_e {
	VIU_MUX_ENCL = 0,
	VIU_MUX_ENCI,
	VIU_MUX_ENCP,
	VIU_MUX_MAX,
};

enum vout_fr_adj_type_e {
	VOUT_FR_ADJ_CLK = 0,
	VOUT_FR_ADJ_HTOTAL,
	VOUT_FR_ADJ_VTOTAL,
	VOUT_FR_ADJ_COMBO, /* vtotal + htotal + clk */
	VOUT_FR_ADJ_HDMI,  /* 50<->60: htotal; 60<->59.94: clk */
	VOUT_FR_ADJ_NONE,  /* disable fr_adj */
	VOUT_FR_ADJ_MAX,
};

enum vinfo_3d_e {
	NON_3D = 0,
	SS_3D,
	FP_3D,
	TB_3D,
};

/*emp : extended metadata type*/
#define VENDOR_SPECIFIC_EM_DATA 0x0
#define COMPRESS_VIDEO_TRAMSPORT 0x1
#define HDR_DYNAMIC_METADATA 0x2
#define VIDEO_TIMING_EXTENDED 0x3

#define SUPPORT_2020	0x01

/* master_display_info for display device */
struct master_display_info_s {
	u32 present_flag;
	u32 features;			/* feature bits bt2020/2084 */
	u32 primaries[3][2];		/* normalized 50000 in G,B,R order */
	u32 white_point[2];		/* normalized 50000 */
	u32 luminance[2];		/* max/min lumin, normalized 10000 */
	u32 max_content;		/* Maximum Content Light Level */
	u32 max_frame_average;	/* Maximum Frame-average Light Level */
};
/*
 *hdr_dynamic_type
 * 0x0001: type_1_hdr_metadata_version
 * 0x0002: ts_103_433_spec_version
 * 0x0003: itu_t_h265_spec_version
 * 0x0004: type_4_hdr_metadata_version
 */
struct hdr_dynamic {
	unsigned int type;
	unsigned char support_flags;
	unsigned int of_len;   /*optional_fields length*/
	unsigned char optional_fields[28];
};

struct hdr10_plus_info {
	uint32_t ieeeoui;
	uint8_t length;
	uint8_t application_version;
};

struct hdr_info {
/* RX EDID hdr support types */
	u32 hdr_support;
	unsigned char rawdata[7];
/*
 *dynamic_info[0] expresses type1's parameters certainly
 *dynamic_info[1] expresses type2's parameters certainly
 *dynamic_info[2] expresses type3's parameters certainly
 *dynamic_info[3] expresses type4's parameters certainly
 *if some types don't exist, the corresponding dynamic_info
 *is zero instead of inexistence
 */
	struct hdr_dynamic dynamic_info[4];
	struct hdr10_plus_info hdr10plus_info;
/*bit7:BT2020RGB    bit6:BT2020YCC bit5:BT2020cYCC bit4:adobeRGB*/
/*bit3:adobeYCC601 bit2:sYCC601     bit1:xvYCC709    bit0:xvYCC601*/
	u8 colorimetry_support; /* RX EDID colorimetry support types */
	u32 lumi_max; /* RX EDID Lumi Max value */
	u32 lumi_avg; /* RX EDID Lumi Avg value */
	u32 lumi_min; /* RX EDID Lumi Min value */
};
struct hdr10plus_para {
	uint8_t application_version;
	uint8_t targeted_max_lum;
	uint8_t average_maxrgb;
	uint8_t distribution_values[9];
	uint8_t num_bezier_curve_anchors;
	uint32_t knee_point_x;
	uint32_t knee_point_y;
	uint8_t bezier_curve_anchors[9];
	uint8_t graphics_overlay_flag;
	uint8_t no_delay_flag;
};

enum eotf_type {
	EOTF_T_NULL = 0,
	EOTF_T_DOLBYVISION,
	EOTF_T_HDR10,
	EOTF_T_SDR,
	EOTF_T_LL_MODE,
	EOTF_T_MAX,
};

enum mode_type {
	YUV422_BIT12 = 0,
	RGB_8BIT,
	RGB_10_12BIT,
	YUV444_10_12BIT,
};

#define DV_IEEE_OUI	0x00D046
#define HDR10_PLUS_IEEE_OUI	0x90848B
#define HDR10_PLUS_DISABLE_VSIF 0
#define HDR10_PLUS_ENABLE_VSIF  1
#define HDR10_PLUS_ZERO_VSIF    2


/* Dolby Version VSIF  parameter*/
struct dv_vsif_para {
	uint8_t ver; /* 0 or 1 or 2*/
	uint8_t length;/*ver1: 15 or 12*/
	union {
		struct {
			uint8_t low_latency:1;
			uint8_t dobly_vision_signal:1;
			uint8_t backlt_ctrl_MD_present:1;
			uint8_t auxiliary_MD_present:1;
			uint8_t eff_tmax_PQ_hi;
			uint8_t eff_tmax_PQ_low;
			uint8_t auxiliary_runmode;
			uint8_t auxiliary_runversion;
			uint8_t auxiliary_debug0;
		} ver2;
	} vers;
};

/* Dolby Version support information from EDID*/
/* Refer to DV Spec version2.9 page26 to page39*/
enum block_type {
	ERROR_NULL = 0,
	ERROR_LENGTH,
	ERROR_OUI,
	ERROR_VER,
	CORRECT,
};

struct dv_info {
	unsigned char rawdata[27];
	enum block_type block_flag;
	uint32_t ieeeoui;
	uint8_t ver; /* 0 or 1 or 2*/
	uint8_t length;/*ver1: 15 or 12*/

	uint8_t sup_yuv422_12bit:1;
	/* if as 0, then support RGB tunnel mode */
	uint8_t sup_2160p60hz:1;
	/* if as 0, then support 2160p30hz */
	uint8_t sup_global_dimming:1;
	uint16_t Rx;
	uint16_t Ry;
	uint16_t Gx;
	uint16_t Gy;
	uint16_t Bx;
	uint16_t By;
	uint16_t Wx;
	uint16_t Wy;
	uint16_t tminPQ;
	uint16_t tmaxPQ;
	uint8_t dm_major_ver;
	uint8_t dm_minor_ver;
	uint8_t dm_version;
	uint8_t tmaxLUM;
	uint8_t colorimetry:1;/* ver1*/
	uint8_t tminLUM;
	uint8_t low_latency;/* ver1_12 and 2*/
	uint8_t sup_backlight_control:1;/*only ver2*/
	uint8_t backlt_min_luma;/*only ver2*/
	uint8_t Interface;/*only ver2*/
	uint8_t sup_10b_12b_444;/*only ver2*/
};

struct vout_device_s {
	const struct dv_info *dv_info;
	void (*fresh_tx_hdr_pkt)(struct master_display_info_s *data);
	void (*fresh_tx_vsif_pkt)(enum eotf_type type,
		enum mode_type tunnel_mode, struct dv_vsif_para *data,
		bool signal_sdr);
	void (*fresh_tx_hdr10plus_pkt)(unsigned int flag,
		struct hdr10plus_para *data);
	void  (*fresh_tx_emp_pkt)(unsigned char *data, unsigned int type,
	unsigned int size);
};

struct vinfo_base_s {
	enum vmode_e mode;
	u32 width;
	u32 height;
	u32 field_height;
	u32 aspect_ratio_num;
	u32 aspect_ratio_den;
	u32 sync_duration_num;
	u32 sync_duration_den;
	u32 screen_real_width;
	u32 screen_real_height;
	u32 video_clk;
	enum color_fmt_e viu_color_fmt;
};

#define LATENCY_INVALID_UNKNOWN	0
#define LATENCY_NOT_SUPPORT		0xffff
struct rx_av_latency {
	unsigned int vLatency;
	unsigned int aLatency;
	unsigned int i_vLatency;
	unsigned int i_aLatency;
};

struct vinfo_s {
	char *name;
	enum vmode_e mode;
	char ext_name[32];
	u32 width;
	u32 height;
	u32 field_height;
	u32 aspect_ratio_num;
	u32 aspect_ratio_den;
	u32 screen_real_width;
	u32 screen_real_height;
	u32 sync_duration_num;
	u32 sync_duration_den;
	u32 video_clk;
	u32 htotal;
	u32 vtotal;
	unsigned char hdmichecksum[10];
	enum vinfo_3d_e info_3d;
	enum vout_fr_adj_type_e fr_adj_type;
	enum color_fmt_e viu_color_fmt;
	enum viu_mux_e viu_mux;
	struct master_display_info_s master_display_info;
	struct hdr_info hdr_info;
	struct rx_av_latency rx_latency;
	struct vout_device_s *vout_device;
};

#ifdef CONFIG_AMLOGIC_MEDIA_FB
struct disp_rect_s {
	int x;
	int y;
	int w;
	int h;
};
#endif
#endif /* _VINFO_H_ */
