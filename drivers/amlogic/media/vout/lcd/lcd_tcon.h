/*
 * drivers/amlogic/media/vout/lcd/lcd_tcon.h
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

#ifndef __AML_LCD_TCON_H__
#define __AML_LCD_TCON_H__
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>

#define REG_LCD_TCON_MAX    0xffff

struct lcd_tcon_data_s {
	unsigned char tcon_valid;

	unsigned int core_reg_width;
	unsigned int reg_table_len;

	unsigned int reg_top_ctrl;
	unsigned int bit_en;

	unsigned int reg_core_od;
	unsigned int bit_od_en;

	unsigned int reg_core_ctrl_timing_base;
	unsigned int ctrl_timing_offset;
	unsigned int ctrl_timing_cnt;

	unsigned int axi_mem_size;
	unsigned char *reg_table;

	int (*tcon_enable)(struct lcd_config_s *pconf);
};

struct tcon_rmem_s {
	unsigned char flag;
	void *mem_vaddr;
	phys_addr_t mem_paddr;
	unsigned int mem_size;
};

/* **********************************
 * tcon config
 * **********************************
 */
/* TL1 */
#define LCD_TCON_CORE_REG_WIDTH_TL1      8
#define LCD_TCON_TABLE_LEN_TL1           24000
#define LCD_TCON_AXI_BANK_TL1            3

#define BIT_TOP_EN_TL1                   4

#define REG_CORE_OD_TL1                  0x247
#define BIT_OD_EN_TL1                    0
#define REG_CORE_CTRL_TIMING_BASE_TL1    0x1b
#define CTRL_TIMING_OFFSET_TL1           12
#define CTRL_TIMING_CNT_TL1              0

#endif
