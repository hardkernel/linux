/*
 * drivers/amlogic/media/vout/lcd/lcd_reg.c
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
#include <linux/io.h>
#include <linux/amlogic/cpu_version.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/iomap.h>
#include "lcd_common.h"
#include "lcd_reg.h"

#define LCD_MAP_PERIPHS   0
#define LCD_MAP_DSI_HOST  1
#define LCD_MAP_DSI_PHY   2
#define LCD_MAP_TCON      3
#define LCD_MAP_MAX       4

int lcd_reg_gxb[] = {
	LCD_MAP_PERIPHS,
	LCD_MAP_MAX,
};

int lcd_reg_axg[] = {
	LCD_MAP_DSI_HOST,
	LCD_MAP_DSI_PHY,
	LCD_MAP_MAX,
};

int lcd_reg_tl1[] = {
	LCD_MAP_TCON,
	LCD_MAP_PERIPHS,
	LCD_MAP_MAX,
};

struct lcd_reg_map_s {
	unsigned int base_addr;
	unsigned int size;
	void __iomem *p;
	char flag;
};

static struct lcd_reg_map_s *lcd_reg_map;

int lcd_ioremap(struct platform_device *pdev)
{
	int i = 0;
	int *table;
	struct resource *res;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	lcd_reg_map = kcalloc(LCD_MAP_MAX,
		sizeof(struct lcd_reg_map_s), GFP_KERNEL);
	if (lcd_reg_map == NULL) {
		LCDERR("%s: lcd_reg_map buf malloc error\n", __func__);
		return -1;
	}
	table = lcd_drv->data->reg_map_table;
	while (i < LCD_MAP_MAX) {
		if (table[i] == LCD_MAP_MAX)
			break;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (res == NULL) {
			LCDERR("%s: lcd_reg resource get error\n", __func__);
			kfree(lcd_reg_map);
			lcd_reg_map = NULL;
			return -1;
		}
		lcd_reg_map[table[i]].base_addr = res->start;
		lcd_reg_map[table[i]].size = resource_size(res);
		lcd_reg_map[table[i]].p = devm_ioremap_nocache(&pdev->dev,
			res->start, lcd_reg_map[table[i]].size);
		if (lcd_reg_map[table[i]].p == NULL) {
			lcd_reg_map[table[i]].flag = 0;
			LCDERR("%s: reg map failed: 0x%x\n",
				__func__,
				lcd_reg_map[table[i]].base_addr);
			kfree(lcd_reg_map);
			lcd_reg_map = NULL;
			return -1;
		} else {
			lcd_reg_map[table[i]].flag = 1;
			if (lcd_debug_print_flag) {
				LCDPR("%s: reg mapped: 0x%x -> %p\n",
					__func__,
					lcd_reg_map[table[i]].base_addr,
					lcd_reg_map[table[i]].p);
			}
		}

		i++;
	}

	return 0;
}

static int check_lcd_ioremap(int n)
{
	if (lcd_reg_map == NULL)
		return -1;
	if (n >= LCD_MAP_MAX)
		return -1;
	if (lcd_reg_map[n].flag == 0) {
		LCDERR("reg 0x%x mapped error\n", lcd_reg_map[n].base_addr);
		return -1;
	}
	return 0;
}

static inline void __iomem *check_lcd_periphs_reg(unsigned int _reg)
{
	void __iomem *p;
	int reg_bus;
	unsigned int reg_offset;

	reg_bus = LCD_MAP_PERIPHS;
	if (check_lcd_ioremap(reg_bus))
		return NULL;

	reg_offset = LCD_REG_OFFSET(_reg);

	if (reg_offset >= lcd_reg_map[reg_bus].size) {
		LCDERR("invalid periphs reg offset: 0x%04x\n", _reg);
		return NULL;
	}
	p = lcd_reg_map[reg_bus].p + reg_offset;
	return p;
}

static inline void __iomem *check_lcd_dsi_host_reg(unsigned int _reg)
{
	void __iomem *p;
	int reg_bus;
	unsigned int reg_offset;

	reg_bus = LCD_MAP_DSI_HOST;
	if (check_lcd_ioremap(reg_bus))
		return NULL;

	reg_offset = LCD_REG_OFFSET_MIPI_HOST(_reg);
	if (reg_offset >= lcd_reg_map[reg_bus].size) {
		LCDERR("invalid dsi_host reg offset: 0x%04x\n", _reg);
		return NULL;
	}
	p = lcd_reg_map[reg_bus].p + reg_offset;
	return p;
}

static inline void __iomem *check_lcd_dsi_phy_reg(unsigned int _reg)
{
	void __iomem *p;
	int reg_bus;
	unsigned int reg_offset;

	reg_bus = LCD_MAP_DSI_PHY;
	if (check_lcd_ioremap(reg_bus))
		return NULL;

	reg_offset = LCD_REG_OFFSET(_reg);
	if (reg_offset >= lcd_reg_map[reg_bus].size) {
		LCDERR("invalid dsi_phy reg offset: 0x%04x\n", _reg);
		return NULL;
	}
	p = lcd_reg_map[reg_bus].p + reg_offset;
	return p;
}

static inline void __iomem *check_lcd_tcon_reg(unsigned int _reg)
{
	void __iomem *p;
	int reg_bus;
	unsigned int reg_offset;

	reg_bus = LCD_MAP_TCON;
	if (check_lcd_ioremap(reg_bus))
		return NULL;

	reg_offset = LCD_REG_OFFSET(_reg);
	if (reg_offset >= lcd_reg_map[reg_bus].size) {
		LCDERR("invalid tcon reg offset: 0x%04x\n", _reg);
		return NULL;
	}
	p = lcd_reg_map[reg_bus].p + reg_offset;
	return p;
}

static inline void __iomem *check_lcd_tcon_reg_byte(unsigned int _reg)
{
	void __iomem *p;
	int reg_bus;
	unsigned int reg_offset;

	reg_bus = LCD_MAP_TCON;
	if (check_lcd_ioremap(reg_bus))
		return NULL;

	reg_offset = LCD_REG_OFFSET_BYTE(_reg);
	if (reg_offset >= lcd_reg_map[reg_bus].size) {
		LCDERR("invalid tcon reg offset: 0x%04x\n", _reg);
		return NULL;
	}
	p = lcd_reg_map[reg_bus].p + reg_offset;
	return p;
}

unsigned int lcd_vcbus_read(unsigned int reg)
{
	return aml_read_vcbus(reg);
};

void lcd_vcbus_write(unsigned int reg, unsigned int value)
{
	aml_write_vcbus(reg, value);
};

void lcd_vcbus_setb(unsigned int reg, unsigned int value,
		unsigned int _start, unsigned int _len)
{
	lcd_vcbus_write(reg, ((lcd_vcbus_read(reg) &
		(~(((1L << _len)-1) << _start))) |
		((value & ((1L << _len)-1)) << _start)));
}

unsigned int lcd_vcbus_getb(unsigned int reg,
		unsigned int _start, unsigned int _len)
{
	return (lcd_vcbus_read(reg) >> _start) & ((1L << _len)-1);
}

void lcd_vcbus_set_mask(unsigned int reg, unsigned int _mask)
{
	lcd_vcbus_write(reg, (lcd_vcbus_read(reg) | (_mask)));
}

void lcd_vcbus_clr_mask(unsigned int reg, unsigned int _mask)
{
	lcd_vcbus_write(reg, (lcd_vcbus_read(reg) & (~(_mask))));
}

unsigned int lcd_hiu_read(unsigned int _reg)
{
	return aml_read_hiubus(_reg);
};

void lcd_hiu_write(unsigned int _reg, unsigned int _value)
{
	aml_write_hiubus(_reg, _value);
};

void lcd_hiu_setb(unsigned int _reg, unsigned int _value,
		unsigned int _start, unsigned int _len)
{
	lcd_hiu_write(_reg, ((lcd_hiu_read(_reg) &
			~(((1L << (_len))-1) << (_start))) |
			(((_value)&((1L<<(_len))-1)) << (_start))));
}

unsigned int lcd_hiu_getb(unsigned int _reg,
		unsigned int _start, unsigned int _len)
{
	return (lcd_hiu_read(_reg) >> (_start)) & ((1L << (_len)) - 1);
}

void lcd_hiu_set_mask(unsigned int _reg, unsigned int _mask)
{
	lcd_hiu_write(_reg, (lcd_hiu_read(_reg) | (_mask)));
}

void lcd_hiu_clr_mask(unsigned int _reg, unsigned int _mask)
{
	lcd_hiu_write(_reg, (lcd_hiu_read(_reg) & (~(_mask))));
}

unsigned int lcd_cbus_read(unsigned int _reg)
{
	return aml_read_cbus(_reg);
};

void lcd_cbus_write(unsigned int _reg, unsigned int _value)
{
	aml_write_cbus(_reg, _value);
};

void lcd_cbus_setb(unsigned int _reg, unsigned int _value,
		unsigned int _start, unsigned int _len)
{
	lcd_cbus_write(_reg, ((lcd_cbus_read(_reg) &
			~(((1L << (_len))-1) << (_start))) |
			(((_value)&((1L<<(_len))-1)) << (_start))));
}

unsigned int lcd_periphs_read(unsigned int _reg)
{
	void __iomem *p;

	p = check_lcd_periphs_reg(_reg);
	if (p)
		return readl(p);
	else
		return -1;
};

void lcd_periphs_write(unsigned int _reg, unsigned int _value)
{
	void __iomem *p;

	p = check_lcd_periphs_reg(_reg);
	if (p)
		writel(_value, p);
};

unsigned int dsi_host_read(unsigned int _reg)
{
	void __iomem *p;

	p = check_lcd_dsi_host_reg(_reg);
	if (p)
		return readl(p);
	else
		return -1;
};
void dsi_host_write(unsigned int _reg, unsigned int _value)
{
	void __iomem *p;

	p = check_lcd_dsi_host_reg(_reg);
	if (p)
		writel(_value, p);
};
void dsi_host_setb(unsigned int reg, unsigned int value,
		unsigned int _start, unsigned int _len)
{
	dsi_host_write(reg, ((dsi_host_read(reg) &
		(~(((1L << _len)-1) << _start))) |
		((value & ((1L << _len)-1)) << _start)));
}

unsigned int dsi_host_getb(unsigned int reg,
		unsigned int _start, unsigned int _len)
{
	return (dsi_host_read(reg) >> _start) & ((1L << _len)-1);
}

void dsi_host_set_mask(unsigned int reg, unsigned int _mask)
{
	dsi_host_write(reg, (dsi_host_read(reg) | (_mask)));
}

void dsi_host_clr_mask(unsigned int reg, unsigned int _mask)
{
	dsi_host_write(reg, (dsi_host_read(reg) & (~(_mask))));
}

unsigned int dsi_phy_read(unsigned int _reg)
{
	void __iomem *p;

	p = check_lcd_dsi_phy_reg(_reg);
	if (p)
		return readl(p);
	else
		return -1;
};
void dsi_phy_write(unsigned int _reg, unsigned int _value)
{
	void __iomem *p;

	p = check_lcd_dsi_phy_reg(_reg);
	if (p)
		writel(_value, p);
};
void dsi_phy_setb(unsigned int reg, unsigned int value,
		unsigned int _start, unsigned int _len)
{
	dsi_phy_write(reg, ((dsi_phy_read(reg) &
		(~(((1L << _len)-1) << _start))) |
		((value & ((1L << _len)-1)) << _start)));
}

unsigned int dsi_phy_getb(unsigned int reg,
		unsigned int _start, unsigned int _len)
{
	return (dsi_phy_read(reg) >> _start) & ((1L << _len)-1);
}

void dsi_phy_set_mask(unsigned int reg, unsigned int _mask)
{
	dsi_phy_write(reg, (dsi_phy_read(reg) | (_mask)));
}

void dsi_phy_clr_mask(unsigned int reg, unsigned int _mask)
{
	dsi_phy_write(reg, (dsi_phy_read(reg) & (~(_mask))));
}

unsigned int lcd_tcon_read(unsigned int _reg)
{
	void __iomem *p;

	p = check_lcd_tcon_reg(_reg);
	if (p)
		return readl(p);
	else
		return -1;
};
void lcd_tcon_write(unsigned int _reg, unsigned int _value)
{
	void __iomem *p;

	p = check_lcd_tcon_reg(_reg);
	if (p)
		writel(_value, p);
};

void lcd_tcon_setb(unsigned int reg, unsigned int value,
		unsigned int _start, unsigned int _len)
{
	lcd_tcon_write(reg, ((lcd_tcon_read(reg) &
		(~(((1L << _len)-1) << _start))) |
		((value & ((1L << _len)-1)) << _start)));
}
unsigned int lcd_tcon_getb(unsigned int reg,
		unsigned int _start, unsigned int _len)
{
	return (lcd_tcon_read(reg) >> _start) & ((1L << _len)-1);
}

void lcd_tcon_set_mask(unsigned int reg, unsigned int _mask)
{
	lcd_tcon_write(reg, (lcd_tcon_read(reg) | (_mask)));
}
void lcd_tcon_clr_mask(unsigned int reg, unsigned int _mask)
{
	lcd_tcon_write(reg, (lcd_tcon_read(reg) & (~(_mask))));
}

unsigned char lcd_tcon_read_byte(unsigned int _reg)
{
	void __iomem *p;

	p = check_lcd_tcon_reg_byte(_reg);
	if (p)
		return readb(p);
	else
		return -1;
};
void lcd_tcon_write_byte(unsigned int _reg, unsigned char _value)
{
	void __iomem *p;

	p = check_lcd_tcon_reg_byte(_reg);
	if (p)
		writeb(_value, p);
};

void lcd_tcon_setb_byte(unsigned int reg, unsigned char value,
		unsigned int _start, unsigned int _len)
{
	lcd_tcon_write_byte(reg, ((lcd_tcon_read_byte(reg) &
		(~(((1L << _len)-1) << _start))) |
		((value & ((1L << _len)-1)) << _start)));
}
unsigned char lcd_tcon_getb_byte(unsigned int reg,
		unsigned int _start, unsigned int _len)
{
	return (lcd_tcon_read_byte(reg) >> _start) & ((1L << _len)-1);
}
