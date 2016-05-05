/*
 * include/linux/amlogic/hdmi_tx/hdmi_tx_cec.h
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


#ifndef _TX_CEC_H_
#define _TX_CEC_H_

#include <linux/irq.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

#define CEC0_LOG_ADDR 4 /* MBX logical address */
#define TV_CEC_INTERVAL     (HZ*3)

#define CEC_VERSION     "v1.3"
#define _RX_DATA_BUF_SIZE_ 16


/* #define G9_CEC  //for G9 CEC debug. */
#define AO_CEC  /* for switch between aocec and hdmi cec2.0 */

extern unsigned int cec_tx_irq_flag;
extern unsigned int cec_tx_irq_syn_flag;
void fiq_gpio_test(unsigned int cmd);


#define MSG_P0(init, follow, opcode)    { \
	gbl_msg[0] = (init)<<4 | (follow);      \
	gbl_msg[1] = (opcode);                  \
}

#define MSG_P1(i, f, o, p1) { \
	gbl_msg[2] = (p1); MSG_P0((i), (f), (o)); }
#define MSG_P2(i, f, o, p1, p2) { \
	gbl_msg[3] = (p2); MSG_P1((i), (f), (o), (p1)); }
#define MSG_P3(i, f, o, p1, p2, p3) { \
	gbl_msg[4] = (p3); MSG_P2((i), (f), (o), (p1), (p2)); }
#define MSG_P4(i, f, o, p1, p2, p3, p4) { \
	gbl_msg[5] = (p4); MSG_P3((i), (f), (o), (p1), (p2), (p3)); }
#define MSG_P5(i, f, o, p1, p2, p3, p4, p5) { \
	gbl_msg[6] = (p5); MSG_P4((i), (f), (o), (p1), (p2), (p3), (p4)); }
#define MSG_P6(i, f, o, p1, p2, p3, p4, p5, p6) { \
	gbl_msg[7] = (p6); MSG_P5((i), (f), (o), \
		(p1), (p2), (p3), (p4), (p5)); }
#define MSG_P7(i, f, o, p1, p2, p3, p4, p5, p6, p7) { \
	gbl_msg[8] = (p7); MSG_P6((i), (f), (o), \
		(p1), (p2), (p3), (p4), (p5), (p6)); }
#define MSG_P8(i, f, o, p1, p2, p3, p4, p5, p6, p7, p8) { \
	gbl_msg[9] = (p8); MSG_P7((i), (f), (o), (p1), (p2), \
		(p3), (p4), (p5), (p6), (p7)); }
#define MSG_P9(i, f, o, p1, p2, p3, p4, p5, p6, p7, p8, p9) { \
	gbl_msg[10] = (p9); MSG_P8((i), (f), (o), (p1), (p2), \
		(p3), (p4), (p5), (p6), (p7), (p8)); }
#define MSG_P10(i, f, o, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10) { \
	gbl_msg[11] = (p10); MSG_P9((i), (f), (o), (p1), (p2), \
		(p3), (p4), (p5), (p6), (p7), (p8), (p9)); }
#define MSG_P11(i, f, o, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11) { \
	gbl_msg[12] = (p11); MSG_P10((i), (f), (o), (p1), (p2), \
		(p3), (p4), (p5), (p6), (p7), (p8), (p9), (p10)); }
#define MSG_P12(i, f, o, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12) \
	{ gbl_msg[13] = (p12); MSG_P11((i), (f), (o), (p1), (p2), \
		(p3), (p4), (p5), (p6), (p7), (p8), (p9), (p10), (p11)); }
#define MSG_P13(i, f, o, p1, p2, p3, p4, p5, p6, p7,  \
	p8, p9, p10, p11, p12, p13) { \
	gbl_msg[14] = (p13); MSG_P12((i), (f), (o), (p1), (p2), (p3), \
		(p4), (p5), (p6), (p7), (p8), (p9), (p10), (p11), (p12)); }
#define MSG_P14(i, f, o, p1, p2, p3, p4, p5, p6, p7, \
	p8, p9, p10, p11, p12, p13, p14) { \
	gbl_msg[15] = (p14); MSG_P13((i), (f), (o), (p1), (p2), \
		(p3), (p4), (p5), (p6), (p7), (p8), (p9), (p10), (p11), \
		(p12), (p13)); }

#define MAX_MSG           16
#define MAX_NUM_OF_DEV    16

#ifdef CONFIG_AML_AO_CEC
#define CEC_IOC_MAGIC                   'C'
#define CEC_IOC_GET_PHYSICAL_ADDR       _IOR(CEC_IOC_MAGIC, 0x00, uint16_t)
#define CEC_IOC_GET_VERSION             _IOR(CEC_IOC_MAGIC, 0x01, int)
#define CEC_IOC_GET_VENDOR_ID           _IOR(CEC_IOC_MAGIC, 0x02, uint32_t)
#define CEC_IOC_GET_PORT_INFO           _IOR(CEC_IOC_MAGIC, 0x03, int)
#define CEC_IOC_GET_PORT_NUM            _IOR(CEC_IOC_MAGIC, 0x04, int)
#define CEC_IOC_GET_SEND_FAIL_REASON    _IOR(CEC_IOC_MAGIC, 0x05, uint32_t)
#define CEC_IOC_SET_OPTION_WAKEUP       _IOW(CEC_IOC_MAGIC, 0x06, uint32_t)
#define CEC_IOC_SET_OPTION_ENALBE_CEC   _IOW(CEC_IOC_MAGIC, 0x07, uint32_t)
#define CEC_IOC_SET_OPTION_SYS_CTRL     _IOW(CEC_IOC_MAGIC, 0x08, uint32_t)
#define CEC_IOC_SET_OPTION_SET_LANG     _IOW(CEC_IOC_MAGIC, 0x09, uint32_t)
#define CEC_IOC_GET_CONNECT_STATUS      _IOR(CEC_IOC_MAGIC, 0x0A, uint32_t)
#define CEC_IOC_ADD_LOGICAL_ADDR        _IOW(CEC_IOC_MAGIC, 0x0B, uint32_t)
#define CEC_IOC_CLR_LOGICAL_ADDR        _IOW(CEC_IOC_MAGIC, 0x0C, uint32_t)
#define CEC_IOC_SET_DEV_TYPE            _IOW(CEC_IOC_MAGIC, 0x0D, uint32_t)
#define CEC_IOC_SET_ARC_ENABLE          _IOW(CEC_IOC_MAGIC, 0x0E, uint32_t)

#define CEC_FAIL_NONE                   0
#define CEC_FAIL_NACK                   1
#define CEC_FAIL_BUSY                   2
#define CEC_FAIL_OTHER                  3

struct hdmi_port_info {
	int type;
	/* Port ID should start from 1 which corresponds to HDMI "port 1". */
	int port_id;
	int cec_supported;
	int arc_supported;
	uint16_t physical_address;
};

void cec_pinmux_set(void);
void cec_arbit_bit_time_set(unsigned , unsigned , unsigned);
void cec_clear_buf(unsigned int flag);
void cec_keep_reset(void);
void ao_cec_init(void);
void tx_irq_handle(void);
int  cec_node_init(struct hdmitx_dev *hdmitx_device);
extern struct hrtimer cec_key_timer;
extern enum hrtimer_restart cec_key_up(struct hrtimer *timer);
unsigned int cec_intr_stat(void);
extern struct cec_global_info_t cec_info;
extern int get_cec_tx_fail(void);

enum hdmi_port_type {
	HDMI_INPUT = 0,
	HDMI_OUTPUT = 1
};
#endif
enum _cec_log_dev_addr_e {
	CEC_TV_ADDR					= 0x00,
	CEC_RECORDING_DEVICE_1_ADDR,
	CEC_RECORDING_DEVICE_2_ADDR,
	CEC_TUNER_1_ADDR,
	CEC_PLAYBACK_DEVICE_1_ADDR,
	CEC_AUDIO_SYSTEM_ADDR,
	CEC_TUNER_2_ADDR,
	CEC_TUNER_3_ADDR,
	CEC_PLAYBACK_DEVICE_2_ADDR,
	CEC_RECORDING_DEVICE_3_ADDR,
	CEC_TUNER_4_ADDR,
	CEC_PLAYBACK_DEVICE_3_ADDR,
	CEC_RESERVED_1_ADDR,
	CEC_RESERVED_2_ADDR,
	CEC_FREE_USE_ADDR,
	CEC_UNREGISTERED_ADDR
};

#define CEC_BROADCAST_ADDR CEC_UNREGISTERED_ADDR

#define CEC_TV                   (1 << CEC_TV_ADDR)
#define CEC_RECORDING_DEVICE_1   (1 << CEC_RECORDING_DEVICE_1_ADDR)
#define CEC_RECORDING_DEVICE_2   (1 << CEC_RECORDING_DEVICE_2_ADDR)
#define CEC_TUNER_1              (1 << CEC_TUNER_1_ADDR)
#define CEC_PLAYBACK_DEVICE_1    (1 << CEC_PLAYBACK_DEVICE_1_ADDR)
#define CEC_AUDIO_SYSTEM         (1 << CEC_AUDIO_SYSTEM_ADDR)
#define CEC_TUNER_2              (1 << CEC_TUNER_2_ADDR)
#define CEC_TUNER_3              (1 << CEC_TUNER_3_ADDR)
#define CEC_PLAYBACK_DEVICE_2    (1 << CEC_PLAYBACK_DEVICE_2_ADDR)
#define CEC_RECORDING_DEVICE_3   (1 << CEC_RECORDING_DEVICE_3_ADDR)
#define CEC_TUNER_4              (1 << CEC_TUNER_4_ADDR)
#define CEC_PLAYBACK_DEVICE_3    (1 << CEC_PLAYBACK_DEVICE_3_ADDR)
#define CEC_RESERVED_1           (1 << CEC_RESERVED_1_ADDR)
#define CEC_RESERVED_2           (1 << CEC_RESERVED_2_ADDR)
#define CEC_FREE_USE             (1 << CEC_FREE_USE_ADDR)
#define CEC_UNREGISTERED         (1 << CEC_UNREGISTERED_ADDR)

#define CEC_DISPLAY_DEVICE       (CEC_TV | CEC_FREE_USE)
#define CEC_RECORDING_DEVICE     (CEC_RECORDING_DEVICE_1 \
	    | CEC_RECORDING_DEVICE_2 | CEC_RECORDING_DEVICE_3)
#define CEC_PLAYBACK_DEVICE      (CEC_PLAYBACK_DEVICE_1 \
	    | CEC_PLAYBACK_DEVICE_2 | CEC_PLAYBACK_DEVICE_3)
#define CEC_TUNER_DEVICE         (CEC_TUNER_1 | CEC_TUNER_2 \
	    | CEC_TUNER_3 | CEC_TUNER_4)
#define CEC_AUDIO_SYSTEM_DEVICE  (CEC_AUDIO_SYSTEM)

enum cec_dev_type_addr {
	CEC_DISPLAY_DEVICE_TYPE = 0x0,
	CEC_RECORDING_DEVICE_TYPE,
	CEC_RESERVED_DEVICE_TYPE,
	CEC_TUNER_DEVICE_TYPE,
	CEC_PLAYBACK_DEVICE_TYPE,
	CEC_AUDIO_SYSTEM_DEVICE_TYPE,
	CEC_UNREGISTERED_DEVICE_TYPE,
};

enum cec_feature_abort_e {
	CEC_UNRECONIZED_OPCODE = 0x0,
	CEC_NOT_CORRECT_MODE_TO_RESPOND,
	CEC_CANNOT_PROVIDE_SOURCE,
	CEC_INVALID_OPERAND,
	CEC_REFUSED,
	CEC_UNABLE_TO_DETERMINE,
};

/* #ifdef AO_CEC */
#if 1

void cec_hw_init(void);
void cec_logic_addr_set(enum _cec_log_dev_addr_e phyaddr);

#endif
/*
 * CEC OPCODES
 */
#define CEC_OC_ABORT_MESSAGE                     0xFF
#define CEC_OC_ACTIVE_SOURCE                     0x82
#define CEC_OC_CEC_VERSION                       0x9E
#define CEC_OC_CLEAR_ANALOGUE_TIMER              0x33
#define CEC_OC_CLEAR_DIGITAL_TIMER               0x99
#define CEC_OC_CLEAR_EXTERNAL_TIMER              0xA1
#define CEC_OC_DECK_CONTROL                      0x42
#define CEC_OC_DECK_STATUS                       0x1B
#define CEC_OC_DEVICE_VENDOR_ID                  0x87
#define CEC_OC_FEATURE_ABORT                     0x00
#define CEC_OC_GET_CEC_VERSION                   0x9F
#define CEC_OC_GET_MENU_LANGUAGE                 0x91
#define CEC_OC_GIVE_AUDIO_STATUS                 0x71
#define CEC_OC_GIVE_DECK_STATUS                  0x1A
#define CEC_OC_GIVE_DEVICE_POWER_STATUS          0x8F
#define CEC_OC_GIVE_DEVICE_VENDOR_ID             0x8C
#define CEC_OC_GIVE_OSD_NAME                     0x46
#define CEC_OC_GIVE_PHYSICAL_ADDRESS             0x83
#define CEC_OC_GIVE_SYSTEM_AUDIO_MODE_STATUS     0x7D
#define CEC_OC_GIVE_TUNER_DEVICE_STATUS          0x08
#define CEC_OC_IMAGE_VIEW_ON                     0x04
#define CEC_OC_INACTIVE_SOURCE                   0x9D
#define CEC_OC_MENU_REQUEST                      0x8D
#define CEC_OC_MENU_STATUS                       0x8E
#define CEC_OC_PLAY                              0x41
#define CEC_OC_POLLING_MESSAGE                   0xFC
#define CEC_OC_RECORD_OFF                        0x0B
#define CEC_OC_RECORD_ON                         0x09
#define CEC_OC_RECORD_STATUS                     0x0A
#define CEC_OC_RECORD_TV_SCREEN                  0x0F
#define CEC_OC_REPORT_AUDIO_STATUS               0x7A
#define CEC_OC_REPORT_PHYSICAL_ADDRESS           0x84
#define CEC_OC_REPORT_POWER_STATUS               0x90
#define CEC_OC_REQUEST_ACTIVE_SOURCE             0x85
#define CEC_OC_ROUTING_CHANGE                    0x80
#define CEC_OC_ROUTING_INFORMATION               0x81
#define CEC_OC_SELECT_ANALOGUE_SERVICE           0x92
#define CEC_OC_SELECT_DIGITAL_SERVICE            0x93
#define CEC_OC_SET_ANALOGUE_TIMER                0x34
#define CEC_OC_SET_AUDIO_RATE                    0x9A
#define CEC_OC_SET_DIGITAL_TIMER                 0x97
#define CEC_OC_SET_EXTERNAL_TIMER                0xA2
#define CEC_OC_SET_MENU_LANGUAGE                 0x32
#define CEC_OC_SET_OSD_NAME                      0x47
#define CEC_OC_SET_OSD_STRING                    0x64
#define CEC_OC_SET_STREAM_PATH                   0x86
#define CEC_OC_SET_SYSTEM_AUDIO_MODE             0x72
#define CEC_OC_SET_TIMER_PROGRAM_TITLE           0x67
#define CEC_OC_STANDBY                           0x36
#define CEC_OC_SYSTEM_AUDIO_MODE_REQUEST         0x70
#define CEC_OC_SYSTEM_AUDIO_MODE_STATUS          0x7E
#define CEC_OC_TEXT_VIEW_ON                      0x0D
#define CEC_OC_TIMER_CLEARED_STATUS              0x43
#define CEC_OC_TIMER_STATUS                      0x35
#define CEC_OC_TUNER_DEVICE_STATUS               0x07
#define CEC_OC_TUNER_STEP_DECREMENT              0x06
#define CEC_OC_TUNER_STEP_INCREMENT              0x05
#define CEC_OC_USER_CONTROL_PRESSED              0x44
#define CEC_OC_USER_CONTROL_RELEASED             0x45
#define CEC_OC_VENDOR_COMMAND                    0x89
#define CEC_OC_VENDOR_COMMAND_WITH_ID            0xA0
#define CEC_OC_VENDOR_REMOTE_BUTTON_DOWN         0x8A
#define CEC_OC_VENDOR_REMOTE_BUTTON_UP           0x8B

/* cec message structure */
struct cec_rx_message_t {
	union {
		unsigned char buffer[16];	/* message memory */
		struct {
/* 4bit Initiator logical address+4bit Destination logical address */
			unsigned char header;
			unsigned char opcode;   /* message opcode */
/* the maximun operand is 14 */
			unsigned char operands[14];
		} msg;			/* message struct */
	} content;			/* message content */
	unsigned char operand_num;	/* number of operand */
	unsigned char msg_length;
};

struct cec_rx_message_list_t {
	struct cec_rx_message_t cec_message;
	struct list_head list;
};

struct cec_usr_message_list_t {
	unsigned char msg[MAX_MSG];
	unsigned char length;
	struct list_head list;
};

/* cec global struct */

enum cec_node_status_e {
	STATE_UNKNOWN = 0x00,
	STATE_START,
	STATE_STOP
};

enum cec_power_status_e {
	POWER_ON = 0x00,
	POWER_STANDBY,
	TRANS_STANDBY_TO_ON,
	TRANS_ON_TO_STANDBY,
};

enum status_req_mode_e {
	STATUS_REQ_ON = 1,
	STATUS_REQ_OFF,
	STATUS_REQ_ONCE,
};

enum deck_info_e {
	DECK_UNKNOWN_STATUS = 0,
	DECK_PLAY = 0X11,
	DECK_RECORD,
	DECK_PLAY_REVERSE,
	DECK_STILL,
	DECK_SLOW,
	DECK_SLOW_REVERSE,
	DECK_FAST_FORWARD,
	DECK_FAST_REVERSE,
	DECK_NO_MEDIA,
	DECK_STOP,
	DECK_SKIP_FORWARD_WIND,
	DECK_SKIP_REVERSE_REWIND,
	DECK_INDEX_SEARCH_FORWARD,
	DECK_INDEX_SEARCH_REVERSE,
	DECK_OTHER_STATUS,
};

enum deck_cnt_mode_e {
	DECK_CNT_SKIP_FORWARD_WIND = 1,
	DECK_CNT_SKIP_REVERSE_REWIND,
	DECK_CNT_STOP,
	DECK_CNT_EJECT,
};

enum play_mode_e {
	PLAY_FORWARD = 0X24,
	PLAY_REVERSE = 0X20,
	PLAY_STILL = 0X25,
	FAST_FORWARD_MIN_SPEED = 0X05,
	FAST_FORWARD_MEDIUM_SPEED = 0X06,
	FAST_FORWARD_MAX_SPEED = 0X07,
	FAST_REVERSE_MIN_SPEED = 0X09,
	FAST_REVERSE_MEDIUM_SPEED = 0X0A,
	FAST_REVERSE_MAX_SPEED = 0X0B,
	SLOW_FORWARD_MIN_SPEED = 0X15,
	SLOW_FORWARD_MEDIUM_SPEED = 0X16,
	SLOW_FORWARD_MAX_SPEED = 0X17,
	SLOW_REVERSE_MIN_SPEED = 0X19,
	SLOW_REVERSE_MEDIUM_SPEED = 0X1A,
	SLOW_REVERSE_MAX_SPEED = 0X1B,
};

enum menu_req_type_e {
	MENU_REQ_ACTIVATE = 0,
	MENU_REQ_DEACTIVATE,
	MENU_REQ_QUERY,
};

enum menu_state_e {
	MENU_STATE_ACTIVATED = 0,
	MENU_STATE_DEACTIVATED,
};

enum cec_version_e {
	CEC_VERSION_11 = 0,
	CEC_VERSION_12,
	CEC_VERSION_12A,
	CEC_VERSION_13,
	CEC_VERSION_13A,
	CEC_VERSION_14A,
};

enum cec_hdmi_port_e {
	CEC_HDMI_PORT_UKNOWN = 0,
	CEC_HDMI_PORT_1,
	CEC_HDMI_PORT_2,
	CEC_HDMI_PORT_3,
};

enum system_audio_status_e {
	OFF = 0,
	ON,
};

#define INFO_MASK_CEC_VERSION                (1<<0)
#define INFO_MASK_VENDOR_ID                  (1<<1)
#define INFO_MASK_DEVICE_TYPE                (1<<2)
#define INFO_MASK_POWER_STATUS               (1<<3)
#define INFO_MASK_PHYSICAL_ADDRESS           (1<<4)
#define INFO_MASK_LOGIC_ADDRESS              (1<<5)
#define INFO_MASK_OSD_NAME                   (1<<6)
#define INFO_MASK_MENU_STATE                 (1<<7)
#define INFO_MASK_MENU_LANGUAGE              (1<<8)
#define INFO_MASK_DECK_INfO                  (1<<9)
#define INFO_MASK_PLAY_MODE                  (1<<10)

/*CEC UI MASK*/
#define CEC_FUNC_MSAK                        0
#define ONE_TOUCH_PLAY_MASK                  1
#define ONE_TOUCH_STANDBY_MASK               2
#define AUTO_POWER_ON_MASK                   3


struct cec_node_info_t {
	enum cec_hdmi_port_e hdmi_port;
	unsigned long real_info_mask;
	unsigned char cec_version;
	unsigned int vendor_id:24;
	unsigned char dev_type;
	unsigned char menu_status;
	enum cec_power_status_e power_status;
	union {
	unsigned short phy_addr_4;
	struct {
		unsigned char cd;
		unsigned char ab;
	} phy_addr_2;
	struct {
		unsigned char d:4;
		unsigned char c:4;
		unsigned char b:4;
		unsigned char a:4;
	} phy_addr_1;
	} phy_addr;
	unsigned char log_addr;
	unsigned char osd_name[16];
	unsigned char osd_name_def[16];
	enum menu_state_e menu_state;
	int menu_lang;
	union {
		struct {
		} display;
		struct {
		} recording;
		struct {
			enum deck_cnt_mode_e deck_cnt_mode;
			enum deck_info_e deck_info;
			enum play_mode_e play_mode;
		} playback;
		struct {
		} tuner;
		struct {
			enum system_audio_status_e sys_audio_mode;
			struct {
				unsigned char audio_mute_status:1;
				unsigned char audio_volume_status:7;
			} audio_status;
		} audio;
	} specific_info;
};

struct cec_rx_msg_buf_t {
	struct cec_rx_message_t cec_rx_message[_RX_DATA_BUF_SIZE_];
	unsigned char rx_write_pos;
	unsigned char rx_read_pos;
	unsigned char rx_buf_size;
};

struct cec_flag_t {
	unsigned int cec_key_flag;
	unsigned int cec_init_flag;
	int cec_fiq_flag;
	int cec_repeat_flag;
};

#ifndef CONFIG_AML_AO_CEC
struct cec_global_info_t {
	unsigned short dev_mask;
	unsigned char active_log_dev;
	unsigned char my_node_index;
	struct cec_flag_t cec_flag;
	struct input_dev *remote_cec_dev; /* cec input device */
	struct cec_node_info_t cec_node_info[MAX_NUM_OF_DEV];
	struct cec_rx_msg_buf_t cec_rx_msg_buf;
	struct hdmitx_dev *hdmitx_device;
	enum cec_power_status_e tv_power_status;
};
#else
/*
 * only for 1 tx device
 */
struct cec_global_info_t {
	dev_t dev_no;
	unsigned int open_count;
	unsigned int hal_ctl;			/* message controled by hal */
	unsigned int vendor_id:24;
	unsigned int power_status;
	unsigned int menu_lang;
	unsigned int cec_version;
	//struct {
	//    unsigned int log_addr1;
	//    unsigned int log_addr2;
	//    unsigned int log_addr3;
	//    unsigned int log_addr4;
	//    unsigned int log_addr5;
	//} log_addr;
	unsigned int log_addr[5];
	unsigned int menu_status;
	unsigned char osd_name[16];
	struct input_dev *remote_cec_dev;	/* cec input device */
	struct hdmitx_dev *hdmitx_device;
};
#endif

struct cec_global_info_to_usr_t {
	unsigned char dev_number;
	unsigned char active_log_dev;
	struct cec_node_info_t cec_node_info_online[];
};

enum usr_cmd_type_e {
	GET_CEC_VERSION = 0,
	GET_POWER_STATUS,
	GET_DEV_POWER_STATUS,
	GET_DEV_VENDOR_ID,
	GET_OSD_NAME,
	GET_OSD_STRING,
	GET_PHYSICAL_ADDR,
	GET_SYSTEM_AUDIO_MODE,
	GET_TUNER_DEV_STATUS,       /* 0x8 */
	GIVE_DECK_STATUS,
	SET_DECK_CONTROL_MODE,
	SET_PLAY_MODE,
	GET_AUDIO_STATUS,
	SET_STANDBY,
	SET_IMAGEVIEW_ON,
	GET_MENU_STATE,
	SET_MENU_STATE,             /* 0x10 */
	SET_MENU_LANGAGE,
	GET_MENU_LANGUAGE,
	GET_ACTIVE_SOURCE,
	SET_ACTIVE_SOURCE,
	SET_DEACTIVE_SOURCE,
	CLR_NODE_DEV_REAL_INFO_MASK,
	REPORT_PHYSICAL_ADDRESS,    /* 0x17 */
	SET_TEXT_VIEW_ON,
	POLLING_ONLINE_DEV, /* 0x19 */
	PING_TV,
	USR_CMD_MAX,
};

enum tv_cec_pending_e {
	TV_CEC_PENDING_OFF = 0,
	TV_CEC_PENDING_ON,
};

enum tv_cec_polling_state_e {
	TV_CEC_POLLING_OFF = 0,
	TV_CEC_POLLING_ON,
};

enum cec_device_menu_state_e {
	DEVICE_MENU_ACTIVE = 0,
	DEVICE_MENU_INACTIVE,
};

void cec_enable_irq(void);
void cec_disable_irq(void);

int cec_ll_tx_polling(const unsigned char *msg, unsigned char len);

int cec_ll_tx(const unsigned char *msg, unsigned char len);
int cec_ll_rx(unsigned char *msg, unsigned char *len);
int cec_rx_irq_handle(unsigned char *msg, unsigned char *len);
unsigned int cec_intr_stat(void);

void cec_test_function(unsigned char *arg, unsigned char arg_cnt);
int  cec_node_init(struct hdmitx_dev *hdmitx_device);
void cec_node_uninit(struct hdmitx_dev *hdmitx_device);
void dumpaocecreg(void);
void raocec(unsigned int addr);
void waocec(unsigned int addr, unsigned int value);
int cec_rx_buf_check(void);
void cec_hw_reset(void);

unsigned int dispatch_buffer_parse(const char *buf,
	const char *string,  char *param, size_t count);
void register_cec_rx_msg(unsigned char *msg, unsigned char len);
void cec_handle_message(struct cec_rx_message_t *pcec_message);
void unregister_cec_tx_msg(struct cec_usr_message_list_t *tx_list);
void unregister_cec_rx_msg(struct cec_usr_message_list_t *rx_list);
void register_cec_tx_msg(unsigned char *msg, unsigned char len);
void cec_tx_cmd_post_process(void);
void cec_set_pending(enum tv_cec_pending_e on_off);
void cec_polling_online_dev(int log_addr, int *bool);
unsigned short cec_log_addr_to_dev_type(unsigned char log_addr);

void cec_routing_information(struct cec_rx_message_t *pcec_message);
void cec_routing_change(struct cec_rx_message_t *pcec_message);
void cec_usrcmd_set_osd_name(struct cec_rx_message_t *pcec_message);
void cec_usrcmd_set_device_vendor_id(void);
void cec_usrcmd_get_cec_version(unsigned char log_addr);
void cec_usrcmd_get_audio_status(unsigned char log_addr);
void cec_usrcmd_get_deck_status(unsigned char log_addr);
void cec_usrcmd_get_device_power_status(unsigned char log_addr);
void cec_usrcmd_get_device_vendor_id(unsigned char log_addr);
void cec_usrcmd_get_osd_name(unsigned char log_addr);
void cec_usrcmd_get_physical_address(unsigned char log_addr);
void cec_usrcmd_get_system_audio_mode_status(unsigned char log_addr);
void cec_usrcmd_get_tuner_device_status(unsigned char log_addr);
void cec_usrcmd_parse_all_dev_online(void);
void cec_usrcmd_set_deck_cnt_mode(unsigned char log_addr,
	enum deck_cnt_mode_e deck_cnt_mode);
void cec_usrcmd_set_standby(unsigned char log_addr);
void cec_usrcmd_set_imageview_on(unsigned char log_addr);
void cec_usrcmd_set_play_mode(unsigned char log_addr,
	enum play_mode_e play_mode);
void cec_usrcmd_get_menu_state(unsigned char log_addr);
void cec_usrcmd_set_menu_state(unsigned char log_addr,
	enum menu_req_type_e menu_req_type);
void cec_usrcmd_get_menu_language(unsigned char log_addr);
void cec_usrcmd_get_active_source(void);
void cec_usrcmd_set_active_source(void);
void cec_usrcmd_set_deactive_source(unsigned char log_addr);
void cec_usrcmd_clear_node_dev_real_info_mask(unsigned char log_addr,
	unsigned long mask);
void cec_usrcmd_set_report_physical_address(void);
void cec_usrcmd_text_view_on(unsigned char log_addr);
void cec_polling_online_dev(int log_addr, int *bool);
void cec_active_source(struct cec_rx_message_t *pcec_message);
#ifndef CONFIG_AML_AO_CEC 
void cec_inactive_source(void);
void cec_report_power_status(struct cec_rx_message_t *pcec_message);
void cec_set_stream_path(struct cec_rx_message_t *pcec_message);
void cec_set_osd_name(struct cec_rx_message_t *pcec_message);
void cec_menu_status_smp(enum cec_device_menu_state_e status);
void cec_device_vendor_id(struct cec_rx_message_t *pcec_message);
void cec_user_control_released_irq(void);
void cec_user_control_pressed_irq(void);
#else
void cec_inactive_source(int dest);
void cec_report_power_status(int dest, int status);
void cec_set_stream_path(unsigned char *msg);
void cec_set_osd_name(int dest);
void cec_menu_status_smp(int dest, int status);
void cec_device_vendor_id(void);
void cec_user_control_pressed_irq(unsigned char message_irq);
void cec_user_control_released_irq(void);
#endif
void cec_set_osd_name_init(void);
void cec_inactive_source_rx(struct cec_rx_message_t *pcec_message);
void cec_set_system_audio_mode(void);
void cec_system_audio_mode_request(void);
void cec_report_audio_status(void);
void cec_get_menu_language_smp(void);
void cec_device_vendor_id_smp(void);
void cec_set_imageview_on_irq(void);
void cec_active_source_irq(void);

void cec_report_physical_address_smp(void);
void cec_imageview_on_smp(void);
void cec_active_source_smp(void);
void cec_active_source_rx(struct cec_rx_message_t *pcec_message);

size_t cec_usrcmd_get_global_info(char *buf);
void cec_usrcmd_set_dispatch(const char *buf, size_t count);
void cec_usrcmd_set_config(const char *buf, size_t count);
void cec_usrcmd_set_lang_config(const char *buf, size_t count);
void cec_input_handle_message(void);
void cec_send_event_irq(void);
void cec_standby_irq(void);
void cec_set_standby(void);
void cec_isr_post_process(void);
void cec_clear_buf(unsigned int flag);
void cec_keep_reset(void);
void cec_wake_lock(void);
void cec_wake_unlock(void);

void cec_tx_irq_handle(void);
unsigned int cec_config(unsigned int value, bool wr_flag);
unsigned int cec_phyaddr_config(unsigned int value, bool wr_flag);
unsigned int cec_logicaddr_config(unsigned int value, bool wr_flag);
unsigned int cec_intr_stat(void);
void cec_pinmux_set(void);
irqreturn_t cec_isr_handler(int irq, void *dev_instance);
void hdmitx_setup_cecirq(struct hdmitx_dev *phdev);
#ifndef CONFIG_AML_AO_CEC
void cec_logicaddr_set(int logicaddr);
#else
void cec_logicaddr_set(int logicaddr, int logreg);
#endif
void cec_arbit_bit_time_set(unsigned bit_set, unsigned time_set,
	unsigned flag);
void tx_irq_handle(void);
void cec_arbit_bit_time_read(void);
void tx_irq_handle(void);
void ao_cec_init(void);

unsigned char check_cec_msg_valid(const struct cec_rx_message_t *pcec_message);
void cec_send_event(struct cec_rx_message_t *pcec_message);
void cec_user_control_pressed(struct cec_rx_message_t *pcec_message);
void cec_user_control_released(struct cec_rx_message_t *pcec_message);
void cec_standby(struct cec_rx_message_t *pcec_message);
void cec_send_simplink_alive(struct cec_rx_message_t *pcec_message);
void cec_send_simplink_ack(struct cec_rx_message_t *pcec_message);

extern void cec_key_init(void);
extern __u16 cec_key_map[];
extern struct cec_global_info_t cec_global_info;

#ifndef CONFIG_AML_AO_CEC
extern __u16 cec_key_map[128];
#else
extern __u16 cec_key_map[160];
#endif
extern bool cec_msg_dbg_en;
extern void cec_rx_buf_clear(void);

extern struct hrtimer cec_key_timer;
extern enum hrtimer_restart cec_key_up(struct hrtimer *timer);
#endif

