/*
 * include/linux/amlogic/hdmi_tx/hdmi_info_global.h
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
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

#ifndef _HDMI_INFO_GLOBAL_H
#define _HDMI_INFO_GLOBAL_H

#include "hdmi_common.h"
#include <linux/wait.h>

/* old definitions move to hdmi_common.h */

enum hdmi_rx_video_state {
	STATE_VIDEO__POWERDOWN = 0,
	STATE_VIDEO__MUTED = 1,
	STATE_VIDEO__UNMUTE = 2,
	STATE_VIDEO__ON = 3,
};

struct pixel_num {
	short H; /* Number of horizontal pixels */
	short V; /* Number of vertical pixels */
};

enum hdmi_pixel_repeat {
	NO_REPEAT = 0,
	HDMI_2_TIMES_REPEAT,
	HDMI_3_TIMES_REPEAT,
	HDMI_4_TIMES_REPEAT,
	HDMI_5_TIMES_REPEAT,
	HDMI_6_TIMES_REPEAT,
	HDMI_7_TIMES_REPEAT,
	HDMI_8_TIMES_REPEAT,
	HDMI_9_TIMES_REPEAT,
	HDMI_10_TIMES_REPEAT,
	MAX_TIMES_REPEAT,
};

enum hdmi_scan {
	SS_NO_DATA = 0,
	/* where some active pixelsand lines at the edges are not displayed. */
	SS_SCAN_OVER,
	/* where all active pixels&lines are displayed,
	 * with or withouta border.
	 */
	SS_SCAN_UNDER,
	SS_RSV
};

enum hdmi_barinfo {
	B_UNVALID = 0, B_BAR_VERT, /* Vert. Bar Infovalid */
	B_BAR_HORIZ, /* Horiz. Bar Infovalid */
	B_BAR_VERT_HORIZ,
/* Vert.and Horiz. Bar Info valid */
};

enum hdmi_colorimetry {
	CC_NO_DATA = 0, CC_ITU601, CC_ITU709, CC_XVYCC601, CC_XVYCC709,
};

enum hdmi_slacing {
	SC_NO_UINFORM = 0,
	/* Picture has been scaled horizontally */
	SC_SCALE_HORIZ,
	SC_SCALE_VERT, /* Picture has been scaled verticallv */
	SC_SCALE_HORIZ_VERT,
/* Picture has been scaled horizontally & SC_SCALE_H_V */
};

struct hdmi_videoinfo {
	enum hdmi_vic VIC;
	enum hdmi_color_space_type color;
	enum hdmi_color_depth color_depth;
	enum hdmi_barinfo bar_info;
	enum hdmi_pixel_repeat repeat_time;
	enum hdmi_aspect_ratio aspect_ratio;
	enum hdmi_colorimetry cc;
	enum hdmi_scan ss;
	enum hdmi_slacing sc;
};
/* -------------------HDMI VIDEO END---------------------------- */

/* -------------------HDMI AUDIO-------------------------------- */
#define TYPE_AUDIO_INFOFRAMES       0x84
#define AUDIO_INFOFRAMES_VERSION    0x01
#define AUDIO_INFOFRAMES_LENGTH     0x0A

#define HDMI_E_NONE         0x0
/* HPD Event & Status */
#define E_HPD_PULG_IN       0x1
#define E_HPD_PLUG_OUT      0x2
#define S_HPD_PLUG_IN       0x1
#define S_HPD_PLUG_OUT      0x0

#define E_HDCP_CHK_BKSV      0x1

/* -------------------HDMI AUDIO END---------------------- */

/* -------------------HDCP-------------------------------- */
/* HDCP keys from Efuse are encrypted by default, in this test HDCP keys
 * are written by CPU with encryption manually added
 */
#define ENCRYPT_KEY                                 0xbe

enum hdcp_authstate {
	HDCP_NO_AUTH = 0,
	HDCP_NO_DEVICE_WITH_SLAVE_ADDR,
	HDCP_BCAP_ERROR,
	HDCP_BKSV_ERROR,
	HDCP_R0S_ARE_MISSMATCH,
	HDCP_RIS_ARE_MISSMATCH,
	HDCP_REAUTHENTATION_REQ,
	HDCP_REQ_AUTHENTICATION,
	HDCP_NO_ACK_FROM_DEV,
	HDCP_NO_RSEN,
	HDCP_AUTHENTICATED,
	HDCP_REPEATER_AUTH_REQ,
	HDCP_REQ_SHA_CALC,
	HDCP_REQ_SHA_HW_CALC,
	HDCP_FAILED_ViERROR,
	HDCP_MAX
};

/* -----------------------HDCP END---------------------------------------- */

/* -----------------------HDMI TX---------------------------------- */
enum hdmitx_disptype {
	CABLE_UNPLUG = 0,
	CABLE_PLUGIN_CHECK_EDID_I2C_ERROR,
	CABLE_PLUGIN_CHECK_EDID_HEAD_ERROR,
	CABLE_PLUGIN_CHECK_EDID_CHECKSUM_ERROR,
	CABLE_PLUGIN_DVI_OUT,
	CABLE_PLUGIN_HDMI_OUT,
	CABLE_MAX
};

struct hdmitx_supstatus {
	int hpd_state:1;
	int support_480i:1;
	int support_576i:1;
	int support_480p:1;
	int support_576p:1;
	int support_720p_60hz:1;
	int support_720p_50hz:1;
	int support_1080i_60hz:1;
	int support_1080i_50hz:1;
	int support_1080p_60hz:1;
	int support_1080p_50hz:1;
	int support_1080p_24hz:1;
	int support_1080p_25hz:1;
	int support_1080p_30hz:1;
};

struct hdmitx_suplpcminfo {
	int support_flag:1;
	int max_channel_num:3;
	int _192k:1;
	int _176k:1;
	int _96k:1;
	int _88k:1;
	int _48k:1;
	int _44k:1;
	int _32k:1;
	int _24bit:1;
	int _20bit:1;
	int _16bit:1;
};

struct hdmitx_supcompressedinfo {
	int support_flag:1;
	int max_channel_num:3;
	int _192k:1;
	int _176k:1;
	int _96k:1;
	int _88k:1;
	int _48k:1;
	int _44k:1;
	int _32k:1;
	int _max_bit:10;
};

struct hdmitx_supspeakerformat {
	int rlc_rrc:1;
	int flc_frc:1;
	int rc:1;
	int rl_rr:1;
	int fc:1;
	int lfe:1;
	int fl_fr:1;
};

struct hdmitx_vidpara {
	unsigned int VIC;
	enum hdmi_color_space_type color_prefer;
	enum hdmi_color_space_type color;
	enum hdmi_color_depth color_depth;
	enum hdmi_barinfo bar_info;
	enum hdmi_pixel_repeat repeat_time;
	enum hdmi_aspect_ratio aspect_ratio;
	enum hdmi_colorimetry cc;
	enum hdmi_scan ss;
	enum hdmi_slacing sc;
};

struct hdmitx_audpara {
	enum hdmi_audio_type type;
	enum hdmi_audio_chnnum channel_num;
	enum hdmi_audio_fs sample_rate;
	enum hdmi_audio_sampsize sample_size;
};

struct hdmitx_supaudinfo {
	struct hdmitx_suplpcminfo	_60958_PCM;
	struct hdmitx_supcompressedinfo	_AC3;
	struct hdmitx_supcompressedinfo	_MPEG1;
	struct hdmitx_supcompressedinfo	_MP3;
	struct hdmitx_supcompressedinfo	_MPEG2;
	struct hdmitx_supcompressedinfo	_AAC;
	struct hdmitx_supcompressedinfo	_DTS;
	struct hdmitx_supcompressedinfo	_ATRAC;
	struct hdmitx_supcompressedinfo	_One_Bit_Audio;
	struct hdmitx_supcompressedinfo	_Dolby;
	struct hdmitx_supcompressedinfo	_DTS_HD;
	struct hdmitx_supcompressedinfo	_MAT;
	struct hdmitx_supcompressedinfo	_DST;
	struct hdmitx_supcompressedinfo	_WMA;
	struct hdmitx_supspeakerformat		speaker_allocation;
};

/* ACR packet CTS parameters have 3 types: */
/* 1. HW auto calculated */
/* 2. Fixed values defined by Spec */
/* 3. Calculated by clock meter */
enum hdmitx_audcts {
	AUD_CTS_AUTO = 0, AUD_CTS_FIXED, AUD_CTS_CALC,
};

struct dispmode_vic {
	const char *disp_mode;
	enum hdmi_vic VIC;
};

struct hdmitx_audinfo {
	/* !< Signal decoding type -- TvAudioType */
	enum hdmi_audio_type type;
	enum hdmi_audio_format format;
	/* !< active audio channels bit mask. */
	enum hdmi_audio_chnnum channels;
	enum hdmi_audio_fs fs; /* !< Signal sample rate in Hz */
	enum hdmi_audio_sampsize ss;
};

/* -----------------Source Physical Address--------------- */
struct vsdb_phyaddr {
	unsigned char a:4;
	unsigned char b:4;
	unsigned char c:4;
	unsigned char d:4;
	unsigned char valid;
#ifdef CONFIG_AML_AO_CEC
	wait_queue_head_t waitq;
#endif
};

struct hdmitx_clk {
	enum hdmi_vic vic;
	uint64_t clk_sys;
	uint64_t clk_phy;
	uint64_t clk_vid;
	uint64_t clk_encp;
	uint64_t clk_enci;
	uint64_t clk_pixel;
};

#define Y420CMDB_MAX	32
struct hdmitx_info {
	struct hdmi_rx_audioinfo audio_info;
	struct hdmitx_supaudinfo tv_audio_info;
	/* Hdmi_tx_video_info_t            video_info; */
	enum hdcp_authstate auth_state;
	enum hdmitx_disptype output_state;
	/* -----------------Source Physical Address--------------- */
	struct vsdb_phyaddr vsdb_phy_addr;
	/* ------------------------------------------------------- */
	unsigned video_out_changing_flag:1;
	unsigned support_underscan_flag:1;
	unsigned support_ycbcr444_flag:1;
	unsigned support_ycbcr422_flag:1;
	unsigned tx_video_input_stable_flag:1;
	unsigned auto_hdcp_ri_flag:1;
	unsigned hw_sha_calculator_flag:1;
	unsigned need_sup_cec:1;

	/* ------------------------------------------------------- */
	unsigned audio_out_changing_flag:1;
	unsigned audio_flag:1;
	unsigned support_basic_audio_flag:1;
	unsigned audio_fifo_overflow:1;
	unsigned audio_fifo_underflow:1;
	unsigned audio_cts_status_err_flag:1;
	unsigned support_ai_flag:1;
	unsigned hdmi_sup_480i:1;

	/* ------------------------------------------------------- */
	unsigned hdmi_sup_576i:1;
	unsigned hdmi_sup_480p:1;
	unsigned hdmi_sup_576p:1;
	unsigned hdmi_sup_720p_60hz:1;
	unsigned hdmi_sup_720p_50hz:1;
	unsigned hdmi_sup_1080i_60hz:1;
	unsigned hdmi_sup_1080i_50hz:1;
	unsigned hdmi_sup_1080p_60hz:1;

	/* ------------------------------------------------------- */
	unsigned hdmi_sup_1080p_50hz:1;
	unsigned hdmi_sup_1080p_24hz:1;
	unsigned hdmi_sup_1080p_25hz:1;
	unsigned hdmi_sup_1080p_30hz:1;

	/* ------------------------------------------------------- */
	/* for total = 32*8 = 256 VICs */
	/* for Y420CMDB bitmap */
	unsigned char bitmap_valid;
	unsigned char bitmap_length;
	unsigned char y420_all_vic;
	unsigned char y420cmdb_bitmap[Y420CMDB_MAX];
	/* ------------------------------------------------------- */
};

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
#define VOUTMODE_HDMI           0x00
#define VOUTMODE_DVI            0x01
#define VOUTMODE_VGA            0x02
#define VOUTMODE_NONHDMI        (VOUTMODE_DVI | VOUTMODE_VGA)

int odroidc_voutmode(void);
#endif

#endif  /* _HDMI_RX_GLOBAL_H */
