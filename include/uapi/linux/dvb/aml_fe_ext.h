/* SPDX-License-Identifier: LGPL-2.1+ WITH Linux-syscall-note */
/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#ifndef _AML_FE_EXT_H_
#define _AML_FE_EXT_H_

#include <linux/types.h>

#include <linux/videodev2.h>

#ifdef __KERNEL__
#include <linux/dvb/frontend.h>
#else
#include "frontend.h"
#endif

#define FE_ANALOG		(FE_ATSC + 1)
#define FE_DTMB			(FE_ANALOG + 1)
#define FE_ISDBT		(FE_DTMB + 1)

struct fe_blind_scan_parameters {
	/* minimum tuner frequency in kHz */
	__u32 min_frequency;
	/* maximum tuner frequency in kHz */
	__u32 max_frequency;
	/* minimum symbol rate in sym/sec */
	__u32 min_symbol_rate;
	/* maximum symbol rate in sym/sec */
	__u32 max_symbol_rate;
	/* search range in kHz. freq -/+freqRange will be searched */
	__u32 frequency_range;
	/* tuner step frequency in kHz */
	__u32 frequency_step;
	/* blindscan event timeout */
	__s32 timeout;
};

#define DTV_DVBT2_PLP_ID    DTV_DVBT2_PLP_ID_LEGACY

#define BLINDSCAN_NONEDO		0x80  /* not blind scan	*/
#define BLINDSCAN_UPDATESTARTFREQ	0x100 /* blind scan update start freq	*/
#define  BLINDSCAN_UPDATEPROCESS	0x20  /* blind scan update process  */
#define BLINDSCAN_UPDATERESULTFREQ	0x400 /* blind scan update result  */

/* Get tne TS input of the frontend */
#define DTV_TS_INPUT                    100
/* Blind scan */
#define DTV_START_BLIND_SCAN            101
#define DTV_CANCEL_BLIND_SCAN           102
#define DTV_BLIND_SCAN_MIN_FRE          103
#define DTV_BLIND_SCAN_MAX_FRE          104
#define DTV_BLIND_SCAN_MIN_SRATE        105
#define DTV_BLIND_SCAN_MAX_SRATE        106
#define DTV_BLIND_SCAN_FRE_RANGE        107
#define DTV_BLIND_SCAN_FRE_STEP         108
#define DTV_BLIND_SCAN_TIMEOUT          109
#define DTV_SINGLE_CABLE_VER            110
#define DTV_SINGLE_CABLE_USER_BAND      111
#define DTV_SINGLE_CABLE_BAND_FRE       112
#define DTV_SINGLE_CABLE_BANK           113
#define DTV_SINGLE_CABLE_UNCOMMITTED    114
#define DTV_SINGLE_CABLE_COMMITTED      115
/* Blind scan end*/
#define DTV_DELIVERY_SUB_SYSTEM			116
#define AML_DTV_MAX_COMMAND		DTV_DELIVERY_SUB_SYSTEM

#define SYS_ANALOG		(SYS_DVBC_ANNEX_C + 1)

enum fe_ofdm_mode {
	OFDM_DVBT,
	OFDM_DVBT2,
};

/*#define ANALOG_FLAG_ENABLE_AFC                 0X00000001*/
#define  ANALOG_FLAG_MANUL_SCAN                0x00000011
struct dvb_analog_parameters {
	/*V4L2_TUNER_MODE_MONO,V4L2_TUNER_MODE_STEREO,*/
	/*V4L2_TUNER_MODE_LANG2,V4L2_TUNER_MODE_SAP,*/
	/*V4L2_TUNER_MODE_LANG1,V4L2_TUNER_MODE_LANG1_LANG2 */
	unsigned int audmode;
	unsigned int soundsys;	/*A2,BTSC,EIAJ,NICAM */
	v4l2_std_id std;
	unsigned int flag;
	unsigned int afc_range;
};

/* Satellite blind scan event */

/*struct dvb_frontend_parameters; */
/*struct dvbsx_blindscanevent { */
/*	enum fe_status status; */
/*	union {*/
		/* The percentage completion of the*/
		/*blind scan procedure. A value of*/
		/*100 indicates that the blind scan*/
		/*is finished. */
		/*__u16 m_uiprogress;*/
		/*The start scan frequency in units of kHz.*/
		/*The minimum value depends on the tuner*/
		/*specification.*/
		/*__u32 m_uistartfreq_khz;*/
		/* Blind scan channel info. */
		/*struct dvb_frontend_parameters parameters;*/
	/*} u;*/
/*};*/

/*for atv*/
struct tuner_status_s {
	unsigned int frequency;
	unsigned int rssi;
	unsigned char mode;/*dtv:0 or atv:1*/
	unsigned char tuner_locked;/*notlocked:0,locked:1*/
	union {
		void *ressrved;
		__u64 reserved1;
	};
};

struct atv_status_s {
	unsigned char atv_lock;/*notlocked:0,locked 1*/
	v4l2_std_id	  std;
	unsigned int  audmode;
	int  snr;
	int  afc;
	union {
		void *resrvred;
		__u64 reserved1;
	};
};

struct sound_status_s {
	unsigned short sound_sys;/*A2DK/A2BG/NICAM BG/NICAM DK/BTSC/EIAJ*/
	unsigned short sound_mode;/*SETERO/DUAL/MONO/SAP*/
	union {
		void *resrvred;
		__u64 reserved1;
	};
};

enum tuner_param_cmd_e {
	TUNER_CMD_AUDIO_MUTE = 0x0000,
	TUNER_CMD_AUDIO_ON,
	TUNER_CMD_TUNER_POWER_ON,
	TUNER_CMD_TUNER_POWER_DOWN,
	TUNER_CMD_SET_VOLUME,
	TUNER_CMD_SET_LEAP_SETP_SIZE,
	TUNER_CMD_GET_MONO_MODE,
	TUNER_CMD_SET_BEST_LOCK_RANGE,
	TUNER_CMD_GET_BEST_LOCK_RANGE,
	TUNER_CMD_SET_CVBS_AMP_OUT,
	TUNER_CMD_GET_CVBS_AMP_OUT,
	TUNER_CMD_NULL,
};

/*parameter for set param box*/
struct tuner_param_s {
	enum tuner_param_cmd_e cmd;
	unsigned int      parm;
	unsigned int	resvred;
};

/* typedef struct dvb_analog_parameters dvb_analog_parameters_t; */
/* typedef struct tuner_status_s tuner_status_t; */
/* typedef struct atv_status_s atv_status_t; */
/* typedef struct sound_status_s sound_status_t; */
/* typedef enum tuner_param_cmd_e tuner_param_cmd_t; */
/* typedef struct tuner_param_s tuner_param_t; */
/* typedef enum fe_layer fe_layer_t; */
/* typedef enum fe_ofdm_mode fe_ofdm_mode_t; */

/* Satellite blind scan settings */
struct dvbsx_blindscanpara {
	__u32 minfrequency;/* minimum tuner frequency in kHz */
	__u32 maxfrequency;/* maximum tuner frequency in kHz */
	__u32 minSymbolRate;/* minimum symbol rate in sym/sec */
	__u32 maxSymbolRate;/* maximum symbol rate in sym/sec */
	/*search range in kHz. freq -/+freqRange will be searched */
	__u32 frequencyRange;
	__u32 frequencyStep;/* tuner step frequency in kHz */
	__s32 timeout;/* blindscan event timeout*/
};

struct dvbsx_singlecable_parameters {
	/* not singlecable: 0, 1.0X - 1(EN50494), 2.0X - 2(EN50607) */
	__u32 version;
	__u32 userband;  /* 1.0X: 0 - 7, 2.0X: 0 - 31 */
	__u32 frequency; /* KHz */
	__u32 bank;
	/*
	 * Uncommitted switches setting for SCD2 (Similar to DiSEqC WriteN1 command,
	 * but lower 4 bits only)
	 *  Bit[0] : Switch 1 Position A or B
	 *  Bit[1] : Switch 2 Position A or B
	 *  Bit[2] : Switch 3 Position A or B
	 *  Bit[3] : Switch 4 Position A or B
	 */
	__u32 uncommitted;
	/*
	 * Committed switches setting for SCD2 (Similar to DiSEqC WriteN0 command,
	 * but lower 4 bits only)
	 *  Bit[0] : Low or High Band
	 *  Bit[1] : Vertical or Horizontal Polarization
	 *  Bit[2] : Satellite Position A or B
	 *  Bit[3] : Option Switch Position A or B
	 */
	__u32 committed;
};

#endif /*_AML_FE_EXT_H_*/
