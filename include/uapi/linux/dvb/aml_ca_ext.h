/* SPDX-License-Identifier: LGPL-2.1+ WITH Linux-syscall-note */

/*
 * Copyright (c) 2019 Amlogic, Inc. All rights reserved.
 */

#ifndef _AML_CA_EXIT_H_
#define _AML_CA_EXIT_H_

#ifdef __KERNEL__
#include <linux/dvb/ca.h>
#else
#include "ca.h"
#endif

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
 * struct ca_sc2_descr_ex - ca extend descriptor
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

/* amlogic define */
#define CA_SET_PID        _IOW('o', 135, struct ca_pid)
#define CA_SET_DESCR_EX   _IOW('o', 200, struct ca_descr_ex)
#define CA_SC2_SET_DESCR_EX   _IOWR('o', 201, struct ca_sc2_descr_ex)
/* amlogic define end */

#endif
