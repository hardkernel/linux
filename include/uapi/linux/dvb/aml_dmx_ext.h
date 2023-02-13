/* SPDX-License-Identifier: LGPL-2.1+ WITH Linux-syscall-note */
/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#ifndef _UAPI_AML_DVBDMX_EXT_H_
#define _UAPI_AML_DVBDMX_EXT_H_

#ifdef __KERNEL__
#include <linux/dvb/dmx.h>
#else
#include "dmx.h"
#endif

/* amlogic define */
#define DMX_USE_SWFILTER    0x100

/*bit 8~15 for mem sec_level*/
#define DMX_MEM_SEC_LEVEL1   (1 << 10)
#define DMX_MEM_SEC_LEVEL2   (2 << 10)
#define DMX_MEM_SEC_LEVEL3   (3 << 10)
#define DMX_MEM_SEC_LEVEL4   (4 << 10)
#define DMX_MEM_SEC_LEVEL5   (5 << 10)
#define DMX_MEM_SEC_LEVEL6   (6 << 10)
#define DMX_MEM_SEC_LEVEL7   (7 << 10)
/* amlogic define end */

/*amlogic define*/
enum dmx_input_source {
	INPUT_DEMOD,
	INPUT_LOCAL,
	INPUT_LOCAL_SEC
};

/**
 * struct dmx_non_sec_es_header - non-sec Elementary Stream (ES) Header
 *
 * @pts_dts_flag:[1:0], 10:pts valid, 01:dts valid
 * @pts_dts_flag:[3:2], 10:scb is scrambled, 01:pscp invalid
 * @pts:	pts value
 * @dts:	dts value
 * @len:	data len
 */
struct dmx_non_sec_es_header {
	__u8 pts_dts_flag;
	__u64 pts;
	__u64 dts;
	__u32 len;
};

/**
 * struct dmx_sec_es_data - sec Elementary Stream (ES)
 *
 * @pts_dts_flag:[1:0], 10:pts valid, 01:dts valid
 * @pts_dts_flag:[3:2], 10:scb is scrambled, 01:pscp invalid
 * @pts:	pts value
 * @dts:	dts value
 * @buf_start:	buf start addr
 * @buf_end:	buf end addr
 * @data_start: data start addr
 * @data_end: data end addr
 */
struct dmx_sec_es_data {
	__u8 pts_dts_flag;
	__u64 pts;
	__u64 dts;
	__u32 buf_start;
	__u32 buf_end;
	__u32 data_start;
	__u32 data_end;
};

struct dmx_sec_ts_data {
	__u32 buf_start;
	__u32 buf_end;
	__u32 data_start;
	__u32 data_end;
};

struct dmx_temi_data {
	__u8 pts_dts_flag;
	__u64 pts;
	__u64 dts;
	__u8 temi[188];
};

enum dmx_audio_format {
	AUDIO_UNKNOWN = 0,	/* unknown media */
	AUDIO_MPX = 1,		/* mpeg audio MP2/MP3 */
	AUDIO_AC3 = 2,		/* Dolby AC3/EAC3 */
	AUDIO_AAC_ADTS = 3,	/* AAC-ADTS */
	AUDIO_AAC_LOAS = 4,	/* AAC-LOAS */
	AUDIO_DTS = 5,		/* DTS */
	AUDIO_MAX
};

struct dmx_mem_info {
	__u32 dmx_total_size;
	__u32 dmx_buf_phy_start;
	__u32 dmx_free_size;
	__u32 dvb_core_total_size;
	__u32 dvb_core_free_size;
	__u32 wp_offset;
	__u64 newest_pts;
};

struct dmx_sec_mem {
	__u32 buff;
	__u32 size;
};

/* amlogic define end */

/*amlogic define*/
/*bit 8~15 for mem sec_level*/
#define DMX_MEM_SEC_LEVEL1   (1 << 10)
#define DMX_MEM_SEC_LEVEL2   (2 << 10)
#define DMX_MEM_SEC_LEVEL3   (3 << 10)
#define DMX_MEM_SEC_LEVEL4   (4 << 10)
#define DMX_MEM_SEC_LEVEL5   (5 << 10)
#define DMX_MEM_SEC_LEVEL6   (6 << 10)
#define DMX_MEM_SEC_LEVEL7   (7 << 10)

/*bit 16~23 for output */
#define DMX_ES_OUTPUT        (1 << 16)
/*set raw mode, it will send the struct dmx_sec_es_data, not es data*/
#define DMX_OUTPUT_RAW_MODE	 (1 << 17)

#define DMX_TEMI_FLAGS       (1 << 18)

/*24~31 one byte for audio type, dmx_audio_format_t*/
#define DMX_AUDIO_FORMAT_BIT 24

/* amlogic define end */

/* amlogic define */
enum {
	DMA_0 = 0,
	DMA_1,
	DMA_2,
	DMA_3,
	DMA_4,
	DMA_5,
	DMA_6,
	DMA_7,
	FRONTEND_TS0 = 32,
	FRONTEND_TS1,
	FRONTEND_TS2,
	FRONTEND_TS3,
	FRONTEND_TS4,
	FRONTEND_TS5,
	FRONTEND_TS6,
	FRONTEND_TS7,
	DMA_0_1 = 64,
	DMA_1_1,
	DMA_2_1,
	DMA_3_1,
	DMA_4_1,
	DMA_5_1,
	DMA_6_1,
	DMA_7_1,
	FRONTEND_TS0_1 = 96,
	FRONTEND_TS1_1,
	FRONTEND_TS2_1,
	FRONTEND_TS3_1,
	FRONTEND_TS4_1,
	FRONTEND_TS5_1,
	FRONTEND_TS6_1,
	FRONTEND_TS7_1,
};

/*define filter mem_info type*/
enum {
	DMX_VIDEO_TYPE = 0,
	DMX_AUDIO_TYPE,
	DMX_SUBTITLE_TYPE,
	DMX_TELETEXT_TYPE,
	DMX_SECTION_TYPE,
};

struct filter_mem_info {
	__u32 type;
	__u32 pid;
	struct dmx_mem_info	filter_info;
};

struct dmx_filter_mem_info {
	__u32 filter_num;
	struct filter_mem_info info[40];
};

struct dvr_mem_info {
	__u32 wp_offset;
};

struct decoder_mem_info {
	__u32 rp_phy;
};

/* amlogic define end */

/* amlogic define */
#define DMX_SET_INPUT           _IO('o', 80)
#define DMX_GET_MEM_INFO        _IOR('o', 81, struct dmx_mem_info)
#define DMX_SET_HW_SOURCE       _IO('o', 82)
#define DMX_GET_HW_SOURCE       _IOR('o', 83, int)
#define DMX_GET_FILTER_MEM_INFO _IOR('o', 84, struct dmx_filter_mem_info)
/*just for dvr sec mem, please call before DMX_SET_PES_FILTER*/
#define DMX_SET_SEC_MEM			_IOW('o', 85, struct dmx_sec_mem)
#define DMX_GET_DVR_MEM			_IOR('o', 86, struct dvr_mem_info)
#define DMX_REMAP_PID			_IOR('o', 87, __u16[2])
#define DMX_SET_DECODE_INFO     _IOW('o', 88, struct decoder_mem_info)
/* amlogic define end */

#endif
