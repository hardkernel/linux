/*
 * drivers/amlogic/media/vout/lcd/lcd_tcon.c
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

#include <linux/init.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/of_reserved_mem.h>
#include <linux/cma.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-mapping.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/media/vout/lcd/lcd_unifykey.h>
#include "lcd_common.h"
#include "lcd_reg.h"
#include "lcd_tcon.h"
/*#include "tcon_ceds.h"*/

#define TCON_INTR_MASKN_VAL    0x0  /* default mask all */

static struct tcon_rmem_s tcon_rmem = {
	.flag = 0,
	.mem_vaddr = NULL,
	.mem_paddr = 0,
	.mem_size = 0,
};

static struct lcd_tcon_data_s *lcd_tcon_data;
static struct delayed_work lcd_tcon_delayed_work;

static int lcd_tcon_valid_check(void)
{
	if (lcd_tcon_data == NULL) {
		LCDERR("invalid tcon data\n");
		return -1;
	}
	if (lcd_tcon_data->tcon_valid == 0) {
		LCDERR("invalid tcon\n");
		return -1;
	}

	return 0;
}

unsigned int lcd_tcon_reg_read(unsigned int addr)
{
	unsigned int val;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return 0;

	if (addr < TCON_TOP_BASE) {
		if (lcd_tcon_data->core_reg_width == 8)
			val = lcd_tcon_read_byte(addr);
		else
			val = lcd_tcon_read(addr);
	} else {
		val = lcd_tcon_read(addr);
	}

	return val;
}

void lcd_tcon_reg_write(unsigned int addr, unsigned int val)
{
	unsigned char temp;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return;

	if (addr < TCON_TOP_BASE) {
		if (lcd_tcon_data->core_reg_width == 8) {
			temp = (unsigned char)val;
			lcd_tcon_write_byte(addr, temp);
		} else {
			lcd_tcon_write(addr, val);
		}
	} else {
		lcd_tcon_write(addr, val);
	}
}

static void lcd_tcon_od_init(unsigned char *table)
{
	unsigned int reg, bit, flag;

	if (lcd_tcon_data->reg_core_od == REG_LCD_TCON_MAX)
		return;

	reg = lcd_tcon_data->reg_core_od;
	bit = lcd_tcon_data->bit_od_en;
	flag = (table[reg] >> bit) & 1;
	lcd_tcon_od_set(flag);
}

static void lcd_tcon_od_check(unsigned char *table)
{
	unsigned int reg, bit;

	if (lcd_tcon_data->reg_core_od == REG_LCD_TCON_MAX)
		return;

	reg = lcd_tcon_data->reg_core_od;
	bit = lcd_tcon_data->bit_od_en;
	if (((table[reg] >> bit) & 1) == 0)
		return;

	if (tcon_rmem.flag == 0) {
		table[reg] &= ~(1 << bit);
		LCDPR("%s: invalid fb, disable od function\n", __func__);
	}
}

void lcd_tcon_core_reg_update(void)
{
	unsigned char *table;
	unsigned int len, temp;
	int i, ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return;

	len = lcd_tcon_data->reg_table_len;
	table = lcd_tcon_data->reg_table;
	if (table == NULL) {
		LCDERR("%s: table is NULL\n", __func__);
		return;
	}
	lcd_tcon_od_check(table);
	if (lcd_tcon_data->core_reg_width == 8) {
		for (i = 0; i < len; i++) {
			lcd_tcon_write_byte((i + TCON_CORE_REG_START),
				table[i]);
		}
	} else {
		for (i = 0; i < len; i++)
			lcd_tcon_write((i + TCON_CORE_REG_START), table[i]);
	}
	LCDPR("tcon core regs update\n");

	if (lcd_tcon_data->reg_core_od != REG_LCD_TCON_MAX) {
		i = lcd_tcon_data->reg_core_od;
		if (lcd_tcon_data->core_reg_width == 8)
			temp = lcd_tcon_read_byte(i + TCON_CORE_REG_START);
		else
			temp = lcd_tcon_read(i + TCON_CORE_REG_START);
		LCDPR("%s: tcon od reg readback: 0x%04x = 0x%04x\n",
			__func__, i, temp);
	}
}

static int lcd_tcon_top_set_tl1(struct lcd_config_s *pconf)
{
	unsigned int axi_reg[3] = {
		TCON_AXI_OFST0, TCON_AXI_OFST1, TCON_AXI_OFST2
	};
	unsigned int addr[3] = {0, 0, 0};
	unsigned int size[3] = {4162560, 4162560, 1960440};
	int i;

	LCDPR("lcd tcon top set\n");

	if (tcon_rmem.flag) {
		addr[0] = tcon_rmem.mem_paddr;
		addr[1] = addr[0] + size[0];
		addr[2] = addr[1] + size[1];
		for (i = 0; i < 3; i++) {
			lcd_tcon_write(axi_reg[i], addr[i]);
			LCDPR("set tcon axi_mem_paddr[%d]: 0x%08x\n",
				i, addr[i]);
		}
	} else {
		LCDERR("%s: invalid axi_mem\n", __func__);
	}

	lcd_tcon_write(TCON_CLK_CTRL, 0x001f);
	if (pconf->lcd_basic.lcd_type == LCD_P2P) {
		switch (pconf->lcd_control.p2p_config->p2p_type) {
		case P2P_CHPI:
			lcd_tcon_write(TCON_TOP_CTRL, 0x8199);
			break;
		default:
			lcd_tcon_write(TCON_TOP_CTRL, 0x8999);
			break;
		}
	} else {
		lcd_tcon_write(TCON_TOP_CTRL, 0x8999);
	}
	lcd_tcon_write(TCON_PLLLOCK_CNTL, 0x0037);
	lcd_tcon_write(TCON_RST_CTRL, 0x003f);
	lcd_tcon_write(TCON_RST_CTRL, 0x0000);
	lcd_tcon_write(TCON_DDRIF_CTRL0, 0x33fff000);
	lcd_tcon_write(TCON_DDRIF_CTRL1, 0x300300);

	return 0;
}

static int lcd_tcon_enable_tl1(struct lcd_config_s *pconf)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int n = 10;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return -1;

	/* step 1: tcon top */
	lcd_tcon_top_set_tl1(pconf);

	/* step 2: tcon_core_reg_update */
	lcd_tcon_core_reg_update();
	if (pconf->lcd_basic.lcd_type == LCD_P2P) {
		switch (pconf->lcd_control.p2p_config->p2p_type) {
		case P2P_CHPI:
			lcd_phy_tcon_chpi_bbc_init_tl1(n);
			break;
		default:
			break;
		}
	}

	if (lcd_tcon_getb_byte(0x23d, 0, 1))
		lcd_drv->tcon_status = 3;
	else
		lcd_drv->tcon_status = 0;

	/* step 3: tcon_top_output_set */
	lcd_tcon_write(TCON_OUT_CH_SEL0, 0x76543210);
	lcd_tcon_write(TCON_OUT_CH_SEL1, 0xba98);

	/* step 4: tcon_intr_mask */
	lcd_tcon_write(TCON_INTR_MASKN, TCON_INTR_MASKN_VAL);

	return 0;
}

static irqreturn_t lcd_tcon_isr(int irq, void *dev_id)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int temp;

	if ((lcd_drv->lcd_status & LCD_STATUS_IF_ON) == 0)
		return IRQ_HANDLED;

	temp = lcd_tcon_read(TCON_INTR_RO);
	if (temp & 0x2) {
		LCDPR("%s: tcon sw_reset triggered\n", __func__);
		lcd_tcon_core_reg_update();
	}
	if (temp & 0x40)
		LCDPR("%s: tcon ddr interface error triggered\n", __func__);

	return IRQ_HANDLED;
}

static void lcd_tcon_intr_init(struct aml_lcd_drv_s *lcd_drv)
{
	unsigned int tcon_irq = 0;

	if (!lcd_drv->res_tcon_irq) {
		LCDERR("res_tcon_irq is null\n");
		return;
	}
	tcon_irq = lcd_drv->res_tcon_irq->start;
	if (lcd_debug_print_flag)
		LCDPR("tcon_irq: %d\n", tcon_irq);

	if (request_irq(tcon_irq, lcd_tcon_isr, IRQF_SHARED,
		"lcd_tcon", (void *)"lcd_tcon"))
		LCDERR("can't request lcd_tcon irq\n");
	else {
		if (lcd_debug_print_flag)
			LCDPR("request lcd_tcon irq successful\n");
	}

	lcd_tcon_write(TCON_INTR_MASKN, TCON_INTR_MASKN_VAL);
}

static int lcd_tcon_config(struct aml_lcd_drv_s *lcd_drv)
{
	int key_len, reg_len, ret;

#if 1
	/* get reg table from unifykey */
	reg_len = lcd_tcon_data->reg_table_len;
	if (lcd_tcon_data->reg_table == NULL) {
		lcd_tcon_data->reg_table =
			kcalloc(reg_len, sizeof(unsigned char), GFP_KERNEL);
		if (!lcd_tcon_data->reg_table) {
			LCDERR("%s: Not enough memory\n", __func__);
			return -1;
		}
	}
	key_len = reg_len;
	ret = lcd_unifykey_get_no_header("lcd_tcon",
		lcd_tcon_data->reg_table, &key_len);
	if (ret) {
		kfree(lcd_tcon_data->reg_table);
		lcd_tcon_data->reg_table = NULL;
		LCDERR("%s: !!!!!!!!tcon unifykey load error!!!!!!!!\n",
			__func__);
		return -1;
	}
	if (key_len != reg_len) {
		kfree(lcd_tcon_data->reg_table);
		lcd_tcon_data->reg_table = NULL;
		LCDERR("%s: !!!!!!!!tcon unifykey load length error!!!!!!!!\n",
			__func__);
		return -1;
	}
	LCDPR("tcon: load unifykey len: %d\n", key_len);
#else
	reg_len = lcd_tcon_data->reg_table_len;
	lcd_tcon_data->reg_table = uhd_tcon_setting_ceds_h10;
	key_len = sizeof(uhd_tcon_setting_ceds_h10)/sizeof(unsigned char);
	if (key_len != reg_len) {
		lcd_tcon_data->reg_table = NULL;
		LCDERR("%s: !!!!!!!!tcon unifykey load length error!!!!!!!!\n",
			__func__);
		return -1;
	}
	LCDPR("tcon: load default table len: %d\n", key_len);
#endif

	lcd_tcon_intr_init(lcd_drv);
	lcd_tcon_od_init(lcd_tcon_data->reg_table);

	return 0;
}

static void lcd_tcon_config_delayed(struct work_struct *work)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	int key_init_flag = 0;
	int i = 0;

	key_init_flag = key_unify_get_init_flag();
	while (key_init_flag == 0) {
		if (i++ >= LCD_UNIFYKEY_WAIT_TIMEOUT)
			break;
		msleep(LCD_UNIFYKEY_RETRY_INTERVAL);
		key_init_flag = key_unify_get_init_flag();
	}
	LCDPR("tcon: key_init_flag=%d, i=%d\n", key_init_flag, i);
	if (key_init_flag)
		lcd_tcon_config(lcd_drv);
}

/* **********************************
 * tcon function api
 * **********************************
 */
#define PR_BUF_MAX    200
void lcd_tcon_reg_table_print(void)
{
	int i, j, n, cnt;
	char *buf;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return;

	if (lcd_tcon_data->reg_table == NULL) {
		LCDERR("%s: reg_table is null\n", __func__);
		return;
	}

	buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
	if (buf == NULL) {
		LCDERR("%s: buf malloc error\n", __func__);
		return;
	}

	LCDPR("%s:\n", __func__);
	cnt = lcd_tcon_data->reg_table_len;
	for (i = 0; i < cnt; i += 16) {
		n = snprintf(buf, PR_BUF_MAX, "0x%04x: ", i);
		for (j = 0; j < 16; j++) {
			if ((i + j) >= cnt)
				break;
			n += snprintf(buf+n, PR_BUF_MAX, " 0x%02x",
				lcd_tcon_data->reg_table[i+j]);
		}
		buf[n] = '\0';
		pr_info("%s\n", buf);
	}
	kfree(buf);
}

void lcd_tcon_reg_readback_print(void)
{
	int i, j, n, cnt;
	char *buf;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return;

	buf = kcalloc(PR_BUF_MAX, sizeof(char), GFP_KERNEL);
	if (buf == NULL) {
		LCDERR("%s: buf malloc error\n", __func__);
		return;
	}

	LCDPR("%s:\n", __func__);
	cnt = lcd_tcon_data->reg_table_len;
	for (i = 0; i < cnt; i += 16) {
		n = snprintf(buf, PR_BUF_MAX, "0x%04x: ", i);
		for (j = 0; j < 16; j++) {
			if ((i + j) >= cnt)
				break;
			if (lcd_tcon_data->core_reg_width == 8) {
				n += snprintf(buf+n, PR_BUF_MAX, " 0x%02x",
					lcd_tcon_read_byte(i+j));
			} else {
				n += snprintf(buf+n, PR_BUF_MAX, " 0x%02x",
					lcd_tcon_read(i+j));
			}
		}
		buf[n] = '\0';
		pr_info("%s\n", buf);
	}
	kfree(buf);
}

int lcd_tcon_info_print(char *buf, int offset)
{
	int len = 0, n, ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return len;

	n = lcd_debug_info_len(len + offset);
	len += snprintf((buf+len), n,
		"tcon info:\n"
		"core_reg_width:    %d\n"
		"reg_table_len:     %d\n"
		"axi_mem paddr:     0x%lx\n"
		"axi_mem size:      0x%x\n\n",
		lcd_tcon_data->core_reg_width,
		lcd_tcon_data->reg_table_len,
		(unsigned long)tcon_rmem.mem_paddr,
		tcon_rmem.mem_size);

	return len;
}

int lcd_tcon_reg_table_size_get(void)
{
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return -1;

	return lcd_tcon_data->reg_table_len;
}

unsigned char *lcd_tcon_reg_table_get(void)
{
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return NULL;

	return lcd_tcon_data->reg_table;
}

int lcd_tcon_core_reg_get(unsigned char *buf, unsigned int size)
{
	int i, ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return -1;

	if (size > lcd_tcon_data->reg_table_len) {
		LCDERR("%s: size is not enough\n", __func__);
		return -1;
	}

	if (lcd_tcon_data->core_reg_width == 8) {
		for (i = 0; i < size; i++)
			buf[i] = lcd_tcon_read_byte(i + TCON_CORE_REG_START);
	} else {
		for (i = 0; i < size; i++)
			buf[i] = lcd_tcon_read(i + TCON_CORE_REG_START);
	}

	return 0;
}

int lcd_tcon_od_set(int flag)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	unsigned int reg, bit, temp;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return -1;

	if (lcd_tcon_data->reg_core_od == REG_LCD_TCON_MAX) {
		LCDERR("%s: invalid od reg\n", __func__);
		return -1;
	}

	if (flag) {
		if (tcon_rmem.flag == 0) {
			LCDERR("%s: invalid memory, disable od function\n",
				__func__);
			return -1;
		}
	}

	if (!(lcd_drv->lcd_status & LCD_STATUS_IF_ON))
		return -1;

	reg = lcd_tcon_data->reg_core_od;
	bit = lcd_tcon_data->bit_od_en;
	if (lcd_tcon_data->core_reg_width == 8)
		temp = lcd_tcon_read_byte(reg + TCON_CORE_REG_START);
	else
		temp = lcd_tcon_read(reg + TCON_CORE_REG_START);
	if (flag)
		temp |= (1 << bit);
	else
		temp &= ~(1 << bit);
	temp &= 0xff;
	if (lcd_tcon_data->core_reg_width == 8)
		lcd_tcon_write_byte((reg + TCON_CORE_REG_START), temp);
	else
		lcd_tcon_write((reg + TCON_CORE_REG_START), temp);

	msleep(100);
	LCDPR("%s: %d\n", __func__, flag);

	return 0;
}

int lcd_tcon_od_get(void)
{
	unsigned int reg, bit, temp;
	int ret = 0;

	ret = lcd_tcon_valid_check();
	if (ret)
		return 0;

	if (lcd_tcon_data->reg_core_od == REG_LCD_TCON_MAX) {
		LCDERR("%s: invalid od reg\n", __func__);
		return 0;
	}

	reg = lcd_tcon_data->reg_core_od;
	bit = lcd_tcon_data->bit_od_en;
	if (lcd_tcon_data->core_reg_width == 8)
		temp = lcd_tcon_read_byte(reg + TCON_CORE_REG_START);
	else
		temp = lcd_tcon_read(reg + TCON_CORE_REG_START);
	ret = ((temp >> bit) & 1);

	return ret;
}

int lcd_tcon_enable(struct lcd_config_s *pconf)
{
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return -1;

	if (lcd_tcon_data->tcon_enable)
		lcd_tcon_data->tcon_enable(pconf);

	return 0;
}

#define TCON_CTRL_TIMING_OFFSET    12
void lcd_tcon_disable(void)
{
	unsigned int reg, i, cnt, offset, bit;
	int ret;

	ret = lcd_tcon_valid_check();
	if (ret)
		return;

	LCDPR("%s\n", __func__);

	/* disable tcon intr */
	lcd_tcon_write(TCON_INTR_MASKN, 0);

	/* disable over_drive */
	if (lcd_tcon_data->reg_core_od != REG_LCD_TCON_MAX) {
		reg = lcd_tcon_data->reg_core_od + TCON_CORE_REG_START;
		bit = lcd_tcon_data->bit_od_en;
		if (lcd_tcon_data->core_reg_width == 8)
			lcd_tcon_setb_byte(reg, 0, bit, 1);
		else
			lcd_tcon_setb(reg, 0, bit, 1);
		msleep(100);
	}

	/* disable all ctrl signal */
	if (lcd_tcon_data->reg_core_ctrl_timing_base == REG_LCD_TCON_MAX)
		goto lcd_tcon_disable_next;
	reg = lcd_tcon_data->reg_core_ctrl_timing_base + TCON_CORE_REG_START;
	offset = lcd_tcon_data->ctrl_timing_offset;
	cnt = lcd_tcon_data->ctrl_timing_cnt;
	for (i = 0; i < cnt; i++) {
		if (lcd_tcon_data->core_reg_width == 8)
			lcd_tcon_setb_byte((reg + (i * offset)), 1, 3, 1);
		else
			lcd_tcon_setb((reg + (i * offset)), 1, 3, 1);
	}

	/* disable top */
lcd_tcon_disable_next:
	if (lcd_tcon_data->reg_top_ctrl != REG_LCD_TCON_MAX) {
		reg = lcd_tcon_data->reg_top_ctrl;
		bit = lcd_tcon_data->bit_en;
		lcd_tcon_setb(reg, 0, bit, 1);
	}
}

/* **********************************
 * tcon match data
 * **********************************
 */
static struct lcd_tcon_data_s tcon_data_tl1 = {
	.tcon_valid = 0,

	.core_reg_width = LCD_TCON_CORE_REG_WIDTH_TL1,
	.reg_table_len = LCD_TCON_TABLE_LEN_TL1,

	.reg_top_ctrl = TCON_TOP_CTRL,
	.bit_en = BIT_TOP_EN_TL1,

	.reg_core_od = REG_CORE_OD_TL1,
	.bit_od_en = BIT_OD_EN_TL1,

	.reg_core_ctrl_timing_base = REG_LCD_TCON_MAX,
	.ctrl_timing_offset = CTRL_TIMING_OFFSET_TL1,
	.ctrl_timing_cnt = CTRL_TIMING_CNT_TL1,

	.axi_mem_size = 0xc00000,
	.reg_table = NULL,

	.tcon_enable = lcd_tcon_enable_tl1,
};

int lcd_tcon_probe(struct aml_lcd_drv_s *lcd_drv)
{
	struct cma *cma;
	unsigned int mem_size;
	int key_init_flag = 0;
	int ret = 0;

	lcd_tcon_data = NULL;
	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
	case LCD_CHIP_TM2:
		switch (lcd_drv->lcd_config->lcd_basic.lcd_type) {
		case LCD_MLVDS:
		case LCD_P2P:
			lcd_tcon_data = &tcon_data_tl1;
			lcd_tcon_data->tcon_valid = 1;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
	if (lcd_tcon_data == NULL)
		return 0;
	if (lcd_tcon_data->tcon_valid == 0)
		return 0;

	if (lcd_tcon_getb_byte(0x23d, 0, 1))
		lcd_drv->tcon_status = 3;
	else
		lcd_drv->tcon_status = 0;
	/* init reserved memory */
	ret = of_reserved_mem_device_init(lcd_drv->dev);
	if (ret) {
		LCDERR("tcon: init reserved memory failed\n");
	} else {
		if ((void *)tcon_rmem.mem_paddr == NULL) {
#ifdef CONFIG_CMA
			cma = dev_get_cma_area(lcd_drv->dev);
			if (cma) {
				tcon_rmem.mem_paddr = cma_get_base(cma);
				LCDPR("tcon axi_mem base:0x%lx, size:0x%lx\n",
					(unsigned long)tcon_rmem.mem_paddr,
					cma_get_size(cma));
				mem_size = lcd_tcon_data->axi_mem_size;
				if (cma_get_size(cma) < mem_size)
					tcon_rmem.flag = 0;
				else {
				tcon_rmem.mem_vaddr = dma_alloc_from_contiguous(
					lcd_drv->dev,
					(mem_size >> PAGE_SHIFT),
					0);
					if (tcon_rmem.mem_vaddr == NULL) {
					LCDERR("tcon axi_mem alloc failed\n");
					} else {
					LCDPR("tcon axi_mem dma_alloc=0x%x\n",
						mem_size);
						tcon_rmem.mem_size = mem_size;
					tcon_rmem.flag = 2; /* cma memory */
					}
				}
			} else {
				LCDERR("tcon: NO CMA\n");
			}
#else
			LCDERR("tcon axi_mem alloc failed\n");
#endif
		} else {
			mem_size = tcon_rmem.mem_size;
			if (mem_size < lcd_tcon_data->axi_mem_size)
				tcon_rmem.flag = 0;
			else {
				tcon_rmem.flag = 1; /* reserved memory */
				LCDPR("tcon axi_mem base:0x%lx, size:0x%x\n",
				(unsigned long)tcon_rmem.mem_paddr, mem_size);
			}
		}
	}

	INIT_DELAYED_WORK(&lcd_tcon_delayed_work, lcd_tcon_config_delayed);

	key_init_flag = key_unify_get_init_flag();
	if (key_init_flag) {
		ret = lcd_tcon_config(lcd_drv);
	} else {
		if (lcd_drv->workqueue) {
			queue_delayed_work(lcd_drv->workqueue,
				&lcd_tcon_delayed_work,
				msecs_to_jiffies(2000));
		} else {
			schedule_delayed_work(&lcd_tcon_delayed_work,
				msecs_to_jiffies(2000));
		}
	}

	return ret;
}

int lcd_tcon_remove(struct aml_lcd_drv_s *lcd_drv)
{
	if (tcon_rmem.flag == 2) {
		LCDPR("tcon free memory: base:0x%lx, size:0x%x\n",
			(unsigned long)tcon_rmem.mem_paddr,
			tcon_rmem.mem_size);
#ifdef CONFIG_CMA
		dma_release_from_contiguous(lcd_drv->dev,
			tcon_rmem.mem_vaddr,
			tcon_rmem.mem_size >> PAGE_SHIFT);
#endif
	}

	if (lcd_tcon_data) {
		/* lcd_tcon_data == NULL; */
		lcd_tcon_data->tcon_valid = 0;
	}

	return 0;
}

static int __init tcon_fb_device_init(struct reserved_mem *rmem,
		struct device *dev)
{
	return 0;
}

static const struct reserved_mem_ops tcon_fb_ops = {
	.device_init = tcon_fb_device_init,
};

static int __init tcon_fb_setup(struct reserved_mem *rmem)
{
	tcon_rmem.mem_paddr = rmem->base;
	tcon_rmem.mem_size = rmem->size;
	rmem->ops = &tcon_fb_ops;
	LCDPR("tcon: Reserved memory: created fb at 0x%lx, size %ld MiB\n",
		(unsigned long)rmem->base, (unsigned long)rmem->size / SZ_1M);
	return 0;
}
RESERVEDMEM_OF_DECLARE(fb, "amlogic, lcd_tcon-memory", tcon_fb_setup);

