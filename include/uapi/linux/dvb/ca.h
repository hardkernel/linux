/* SPDX-License-Identifier: LGPL-2.1+ WITH Linux-syscall-note */
/*
 * ca.h
 *
 * Copyright (C) 2000 Ralph  Metzler <ralph@convergence.de>
 *                  & Marcus Metzler <marcus@convergence.de>
 *                    for convergence integrated media GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Lesser Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#ifndef _DVBCA_H_
#define _DVBCA_H_

/**
 * struct ca_slot_info - CA slot interface types and info.
 *
 * @num:	slot number.
 * @type:	slot type.
 * @flags:	flags applicable to the slot.
 *
 * This struct stores the CA slot information.
 *
 * @type can be:
 *
 *	- %CA_CI - CI high level interface;
 *	- %CA_CI_LINK - CI link layer level interface;
 *	- %CA_CI_PHYS - CI physical layer level interface;
 *	- %CA_DESCR - built-in descrambler;
 *	- %CA_SC -simple smart card interface.
 *
 * @flags can be:
 *
 *	- %CA_CI_MODULE_PRESENT - module (or card) inserted;
 *	- %CA_CI_MODULE_READY - module is ready for usage.
 */

struct ca_slot_info {
	int num;
	int type;
#define CA_CI            1
#define CA_CI_LINK       2
#define CA_CI_PHYS       4
#define CA_DESCR         8
#define CA_SC          128

	unsigned int flags;
#define CA_CI_MODULE_PRESENT 1
#define CA_CI_MODULE_READY   2
};


/**
 * struct ca_descr_info - descrambler types and info.
 *
 * @num:	number of available descramblers (keys).
 * @type:	type of supported scrambling system.
 *
 * Identifies the number of descramblers and their type.
 *
 * @type can be:
 *
 *	- %CA_ECD - European Common Descrambler (ECD) hardware;
 *	- %CA_NDS - Videoguard (NDS) hardware;
 *	- %CA_DSS - Distributed Sample Scrambling (DSS) hardware.
 */
struct ca_descr_info {
	unsigned int num;
	unsigned int type;
#define CA_ECD           1
#define CA_NDS           2
#define CA_DSS           4
};

/**
 * struct ca_caps - CA slot interface capabilities.
 *
 * @slot_num:	total number of CA card and module slots.
 * @slot_type:	bitmap with all supported types as defined at
 *		&struct ca_slot_info (e. g. %CA_CI, %CA_CI_LINK, etc).
 * @descr_num:	total number of descrambler slots (keys)
 * @descr_type:	bitmap with all supported types as defined at
 *		&struct ca_descr_info (e. g. %CA_ECD, %CA_NDS, etc).
 */
struct ca_caps {
	unsigned int slot_num;
	unsigned int slot_type;
	unsigned int descr_num;
	unsigned int descr_type;
};

/**
 * struct ca_msg - a message to/from a CI-CAM
 *
 * @index:	unused
 * @type:	unused
 * @length:	length of the message
 * @msg:	message
 *
 * This struct carries a message to be send/received from a CI CA module.
 */
struct ca_msg {
	unsigned int index;
	unsigned int type;
	unsigned int length;
	unsigned char msg[256];
};

/**
 * struct ca_descr - CA descrambler control words info
 *
 * @index: CA Descrambler slot
 * @parity: control words parity, where 0 means even and 1 means odd
 * @cw: CA Descrambler control words
 */
struct ca_descr {
	unsigned int index;
	unsigned int parity;
	unsigned char cw[8];
};

/* amlogic define */
/* CW type. */
enum ca_cw_type {
	CA_CW_DVB_CSA_EVEN,
	CA_CW_DVB_CSA_ODD,
	CA_CW_AES_EVEN,
	CA_CW_AES_ODD,
	CA_CW_AES_EVEN_IV,
	CA_CW_AES_ODD_IV,
	CA_CW_DES_EVEN,
	CA_CW_DES_ODD,
	CA_CW_SM4_EVEN,
	CA_CW_SM4_ODD,
	CA_CW_SM4_EVEN_IV,
	CA_CW_SM4_ODD_IV,
	CA_CW_TYPE_MAX
};

enum ca_dsc_mode {
	CA_DSC_CBC = 1,
	CA_DSC_ECB,
	CA_DSC_IDSA
};

struct ca_descr_ex {
	unsigned int index;
	enum ca_cw_type type;
	enum ca_dsc_mode mode;
	int          flags;
#define CA_CW_FROM_KL 1
	unsigned char cw[16];
};

/* add for support sc2 ca*/
enum ca_sc2_cmd_type {
	CA_ALLOC,
	CA_FREE,
	CA_KEY,
	CA_GET_STATUS,
	CA_SET_SCB,
	CA_SET_ALGO
};

enum ca_sc2_algo_type {
	CA_ALGO_AES_ECB_CLR_END,
	CA_ALGO_AES_ECB_CLR_FRONT,
	CA_ALGO_AES_CBC_CLR_END,
	CA_ALGO_AES_CBC_IDSA,
	CA_ALGO_CSA2,
	CA_ALGO_DES_SCTE41,
	CA_ALGO_DES_SCTE52,
	CA_ALGO_TDES_ECB_CLR_END,
	CA_ALGO_CPCM_LSA_MDI_CBC,
	CA_ALGO_CPCM_LSA_MDD_CBC,
	CA_ALGO_CSA3,
	CA_ALGO_ASA,
	CA_ALGO_ASA_LIGHT,
	CA_ALGO_S17_ECB_CLR_END,
	CA_ALGO_S17_ECB_CTS,
	CA_ALGO_UNKNOWN
};

enum ca_sc2_dsc_type {
	CA_DSC_COMMON_TYPE,
	CA_DSC_TSD_TYPE,	/*just support AES descramble.*/
	CA_DSC_TSE_TYPE		/*just support AES enscramble.*/
};

/**
 * struct ca_alloc - malloc ca slot index by params
 *
 * @pid:	slot use pid.
 * @algo:	use the algorithm
 * @dsc_type:	CA_DSC_COMMON_TYPE:support all ca_algo_type
 *		CA_DSC_TSD_TYPE & CA_DSC_TSE_TYPE just support AES
 * @ca_index:	return slot index.
 * @loop:	0: just descramble once.
 *			1: descramble twice.
 */
struct ca_sc2_alloc {
	unsigned int pid;
	enum ca_sc2_algo_type algo;
	enum ca_sc2_dsc_type dsc_type;
	unsigned int ca_index;
	unsigned char loop;
};

/**
 * struct ca_sc2_free - free slot index
 *
 * @ca_index:	need free slot index.
 */
struct ca_sc2_free {
	unsigned int ca_index;
};

enum ca_sc2_key_type {
	CA_KEY_EVEN_TYPE,
	CA_KEY_EVEN_IV_TYPE,
	CA_KEY_ODD_TYPE,
	CA_KEY_ODD_IV_TYPE,
	CA_KEY_00_TYPE,
	CA_KEY_00_IV_TYPE
};

/**
 * struct ca_sc2_key - set key slot index
 *
 * @ca_index:	use slot index.
 * @parity:	key type (odd/even/key00)
 * @key_index: key store index.
 */
struct ca_sc2_key {
	unsigned int ca_index;
	enum ca_sc2_key_type parity;
	unsigned int key_index;
};

/**
 * struct ca_sc2_scb - set scb
 *
 * @ca_index:	use slot index.
 * @ca_scb:	ca_scb (2bit)
 * @ca_scb_as_is:if 1, scb use original
 *				 if 0, use ca_scb
 */
struct ca_sc2_scb {
	unsigned int ca_index;
	unsigned char ca_scb;
	unsigned char ca_scb_as_is;
};

/**
 * struct ca_sc2_algo - set algo
 *
 * @ca_index:	use slot index.
 * @algo:	algo
 */
struct ca_sc2_algo {
	unsigned int ca_index;
	enum ca_sc2_algo_type algo;
};

/**
 * struct ca_sc2_descr_ex - ca externd descriptor
 *
 * @params:	command resource params
 */
struct ca_sc2_descr_ex {
	enum ca_sc2_cmd_type cmd;
	union {
		struct ca_sc2_alloc alloc_params;
		struct ca_sc2_free free_params;
		struct ca_sc2_key key_params;
		struct ca_sc2_scb scb_params;
		struct ca_sc2_algo algo_params;
	} params;
};

struct ca_pid {
	unsigned int pid;
	int index;   /* -1 == disable*/
};

/* amlogic define end */

#define CA_RESET          _IO('o', 128)
#define CA_GET_CAP        _IOR('o', 129, struct ca_caps)
#define CA_GET_SLOT_INFO  _IOR('o', 130, struct ca_slot_info)
#define CA_GET_DESCR_INFO _IOR('o', 131, struct ca_descr_info)
#define CA_GET_MSG        _IOR('o', 132, struct ca_msg)
#define CA_SEND_MSG       _IOW('o', 133, struct ca_msg)
#define CA_SET_DESCR      _IOW('o', 134, struct ca_descr)
/* amlogic define */
#define CA_SET_PID        _IOW('o', 135, struct ca_pid)
#define CA_SET_DESCR_EX   _IOW('o', 200, struct ca_descr_ex)
#define CA_SC2_SET_DESCR_EX   _IOWR('o', 201, struct ca_sc2_descr_ex)
/* amlogic define end */
#if !defined(__KERNEL__)

/* This is needed for legacy userspace support */
typedef struct ca_slot_info ca_slot_info_t;
typedef struct ca_descr_info  ca_descr_info_t;
typedef struct ca_caps  ca_caps_t;
typedef struct ca_msg ca_msg_t;
typedef struct ca_descr ca_descr_t;

#endif
#endif
