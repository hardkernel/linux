/*
 * drivers/amlogic/media/vout/lcd/lcd_phy_config.c
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/amlogic/media/vout/lcd/lcd_vout.h>
#include <linux/amlogic/cpu_version.h>
#include "lcd_reg.h"
#include "lcd_phy_config.h"
#include "lcd_common.h"

static int phy_ctrl_bit_on;

static unsigned int lcd_lvds_channel_on_value(struct lcd_config_s *pconf)
{
	unsigned int channel_on = 0;

	if (pconf->lcd_control.lvds_config->dual_port == 0) {
		if (pconf->lcd_control.lvds_config->lane_reverse == 0) {
			switch (pconf->lcd_basic.lcd_bits) {
			case 6:
				channel_on = 0xf;
				break;
			case 8:
				channel_on = 0x1f;
				break;
			case 10:
			default:
				channel_on = 0x3f;
				break;
			}
		} else {
			switch (pconf->lcd_basic.lcd_bits) {
			case 6:
				channel_on = 0x3c;
				break;
			case 8:
				channel_on = 0x3e;
				break;
			case 10:
			default:
				channel_on = 0x3f;
				break;
			}
		}
		if (pconf->lcd_control.lvds_config->port_swap == 1)
			channel_on = (channel_on << 6); /* use channel B */
	} else {
		if (pconf->lcd_control.lvds_config->lane_reverse == 0) {
			switch (pconf->lcd_basic.lcd_bits) {
			case 6:
				channel_on = 0x3cf;
				break;
			case 8:
				channel_on = 0x7df;
				break;
			case 10:
			default:
				channel_on = 0xfff;
				break;
			}
		} else {
			switch (pconf->lcd_basic.lcd_bits) {
			case 6:
				channel_on = 0xf3c;
				break;
			case 8:
				channel_on = 0xfbe;
				break;
			case 10:
			default:
				channel_on = 0xfff;
				break;
			}
		}
	}
	return channel_on;
}

void lcd_phy_cntl_set_tl1(int status, unsigned int data32, int flag)
{
	unsigned int cntl16 = 0x80000000;
	unsigned int data = 0;
	unsigned int tmp = 0;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	if (status) {
		data32 |= ((phy_ctrl_bit_on << 16) | (phy_ctrl_bit_on << 0));
		if (flag)
			tmp |= ((1 << 18) | (1 << 2));
	} else {
		if (phy_ctrl_bit_on)
			data = 0;
		else
			data = 1;
		data32 |= ((data << 16) | (data << 0));
		cntl16 = 0;
		lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14, 0);
	}

	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL15, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL16, cntl16);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL8, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL1, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL9, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL2, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL10, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL3, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL11, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL4, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL12, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL6, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL13, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL7, data32);
}

void lcd_lvds_phy_set(struct lcd_config_s *pconf, int status)
{
	unsigned int vswing, preem, clk_vswing, clk_preem, channel_on;
	unsigned int data32 = 0, size;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct lvds_config_s *lvds_conf;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	lvds_conf = pconf->lcd_control.lvds_config;
	if (status) {
		vswing = lvds_conf->phy_vswing & 0xf;
		preem = lvds_conf->phy_preem & 0xf;
		clk_vswing = lvds_conf->phy_clk_vswing & 0xf;
		clk_preem = lvds_conf->phy_clk_preem & 0xf;
		if (lcd_debug_print_flag)
			LCDPR("vswing=0x%x, prrem=0x%x\n", vswing, preem);

		switch (lcd_drv->data->chip_type) {
		case LCD_CHIP_TL1:
		case LCD_CHIP_TM2:
			size = sizeof(lvds_vx1_p2p_phy_preem_tl1) /
				sizeof(unsigned int);
			if (preem >= size) {
				LCDERR("%s: invalid preem=0x%x, use default\n",
					__func__, preem);
				preem = 0;
			}
			data32 = lvds_vx1_p2p_phy_preem_tl1[preem];
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14,
				0xff2027e0 | vswing);
			lcd_phy_cntl_set_tl1(status, data32, 0);
			break;
		default:
			if (vswing > 7) {
				LCDERR("%s: invalid vswing=0x%x, use default\n",
					__func__, vswing);
				vswing = LVDS_PHY_VSWING_DFT;
			}
			if (preem > 7) {
				LCDERR("%s: invalid preem=0x%x, use default\n",
					__func__, preem);
				preem = LVDS_PHY_PREEM_DFT;
			}
			if (clk_vswing > 3) {
				LCDERR(
				"%s: invalid clk_vswing=0x%x, use default\n",
					__func__, clk_vswing);
				clk_vswing = LVDS_PHY_CLK_VSWING_DFT;
			}
			if (clk_preem > 7) {
				LCDERR(
				"%s: invalid clk_preem=0x%x, use default\n",
					__func__, clk_preem);
				clk_preem = LVDS_PHY_CLK_PREEM_DFT;
			}
			channel_on = lcd_lvds_channel_on_value(pconf);

			data32 = LVDS_PHY_CNTL1_G9TV |
				(vswing << 26) | (preem << 0);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL1, data32);
			data32 = LVDS_PHY_CNTL2_G9TV;
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL2, data32);
			data32 = LVDS_PHY_CNTL3_G9TV |
				(channel_on << 16) |
				(clk_vswing << 8) |
				(clk_preem << 5);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL3, data32);
			break;
		}
	} else {
		switch (lcd_drv->data->chip_type) {
		case LCD_CHIP_TL1:
		case LCD_CHIP_TM2:
			lcd_phy_cntl_set_tl1(status, data32, 0);
			break;
		default:
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL1, 0);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL2, 0);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL3, 0);
			break;
		}
	}
}

void lcd_vbyone_phy_set(struct lcd_config_s *pconf, int status)
{
	unsigned int vswing, preem, ext_pullup;
	unsigned int data32 = 0, size;
	unsigned int rinner_table[] = {0xa, 0xa, 0x6, 0x4};
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();
	struct vbyone_config_s *vbyone_conf;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	vbyone_conf = pconf->lcd_control.vbyone_config;
	if (status) {
		ext_pullup = (vbyone_conf->phy_vswing >> 4) & 0x3;
		vswing = vbyone_conf->phy_vswing & 0xf;
		preem = vbyone_conf->phy_preem & 0xf;
		if (lcd_debug_print_flag) {
			LCDPR("vswing=0x%x, prrem=0x%x\n",
				vbyone_conf->phy_vswing, preem);
		}

		switch (lcd_drv->data->chip_type) {
		case LCD_CHIP_TL1:
		case LCD_CHIP_TM2:
			size = sizeof(lvds_vx1_p2p_phy_preem_tl1) /
				sizeof(unsigned int);
			if (preem >= size) {
				LCDERR("%s: invalid preem=0x%x, use default\n",
					__func__, preem);
				preem = 0x1;
			}
			data32 = lvds_vx1_p2p_phy_preem_tl1[preem];
			if (ext_pullup) {
				lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14,
					0xff2027e0 | vswing);
			} else {
				lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14,
					0xf02027a0 | vswing);
			}
			lcd_phy_cntl_set_tl1(status, data32, 0);
			break;
		default:
			if (vswing > 7) {
				LCDERR("%s: invalid vswing=0x%x, use default\n",
					__func__, vswing);
				vswing = VX1_PHY_VSWING_DFT;
			}
			if (preem > 7) {
				LCDERR("%s: invalid preem=0x%x, use default\n",
					__func__, preem);
				preem = VX1_PHY_PREEM_DFT;
			}
			if (ext_pullup) {
				data32 = VX1_PHY_CNTL1_G9TV_PULLUP |
					(vswing << 3);
			} else {
				data32 = VX1_PHY_CNTL1_G9TV | (vswing << 3);
			}
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL1, data32);
			data32 = VX1_PHY_CNTL2_G9TV | (preem << 20) |
				(rinner_table[ext_pullup] << 8);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL2, data32);
			data32 = VX1_PHY_CNTL3_G9TV;
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL3, data32);
			break;
		}
	} else {
		switch (lcd_drv->data->chip_type) {
		case LCD_CHIP_TL1:
		case LCD_CHIP_TM2:
			lcd_phy_cntl_set_tl1(status, data32, 0);
			break;
		default:
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL1, 0);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL2, 0);
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL3, 0);
			break;
		}
	}
}

void lcd_mlvds_phy_set(struct lcd_config_s *pconf, int status)
{
	unsigned int vswing, preem;
	unsigned int data32 = 0, size, cntl16;
	struct mlvds_config_s *mlvds_conf;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	mlvds_conf = pconf->lcd_control.mlvds_config;
	if (status) {
		vswing = mlvds_conf->phy_vswing & 0xf;
		preem = mlvds_conf->phy_preem & 0xf;
		if (lcd_debug_print_flag)
			LCDPR("vswing=0x%x, prrem=0x%x\n", vswing, preem);

		size = sizeof(lvds_vx1_p2p_phy_preem_tl1) /
			sizeof(unsigned int);
		if (preem >= size) {
			LCDERR("%s: invalid preem=0x%x, use default\n",
				__func__, preem);
			preem = 0;
		}
		data32 = lvds_vx1_p2p_phy_preem_tl1[preem];
		if (is_meson_rev_c())
			data32 |= ((1 << 16) | (1 << 0));
		lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14,
			0xff2027e0 | vswing);
		lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL15, 0);
		cntl16 = (mlvds_conf->pi_clk_sel << 12);
		cntl16 |= 0x80000000;
		lcd_phy_cntl_set_tl1(status, data32, cntl16);
	} else
		lcd_phy_cntl_set_tl1(status, data32, 0);
}

void lcd_p2p_phy_set(struct lcd_config_s *pconf, int status)
{
	unsigned int vswing, preem;
	unsigned int data32 = 0, size;
	struct p2p_config_s *p2p_conf;

	if (lcd_debug_print_flag)
		LCDPR("%s: %d\n", __func__, status);

	p2p_conf = pconf->lcd_control.p2p_config;
	if (status) {
		vswing = p2p_conf->phy_vswing & 0xf;
		preem = p2p_conf->phy_preem & 0xf;
		if (lcd_debug_print_flag)
			LCDPR("vswing=0x%x, prrem=0x%x\n", vswing, preem);

		switch (p2p_conf->p2p_type) {
		case P2P_CEDS:
		case P2P_CMPI:
		case P2P_ISP:
		case P2P_EPI:
			size = sizeof(lvds_vx1_p2p_phy_preem_tl1) /
				sizeof(unsigned int);
			if (preem >= size) {
				LCDERR("%s: invalid preem=0x%x, use default\n",
					__func__, preem);
				preem = 0x1;
			}
			data32 = lvds_vx1_p2p_phy_preem_tl1[preem];
			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14,
				0xff2027a0 | vswing);
			lcd_phy_cntl_set_tl1(status, data32, 0);
			break;
		case P2P_CHPI: /* low common mode */
		case P2P_CSPI:
		case P2P_USIT:
			size = sizeof(p2p_low_common_phy_preem_tl1) /
				sizeof(unsigned int);
			if (preem >= size) {
				LCDERR("%s: invalid preem=0x%x, use default\n",
					__func__, preem);
				preem = 0x1;
			}
			data32 = p2p_low_common_phy_preem_tl1[preem];
			if (p2p_conf->p2p_type == P2P_CHPI) {
				/* weakly pull down */
				data32 &= ~((1 << 19) | (1 << 3));
			}

			lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14, 0xfe60027f);
			lcd_phy_cntl_set_tl1(status, data32, 0);
			break;
		default:
			LCDERR("%s: invalid p2p_type %d\n",
				__func__, p2p_conf->p2p_type);
			break;
		}
	} else
		lcd_phy_cntl_set_tl1(status, data32, 0);
}

void lcd_mipi_phy_set(struct lcd_config_s *pconf, int status)
{
	unsigned int phy_reg, phy_bit, phy_width;
	unsigned int lane_cnt;
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	if (status) {
		switch (lcd_drv->data->chip_type) {
		case LCD_CHIP_G12A:
		case LCD_CHIP_G12B:
		case LCD_CHIP_SM1:
			/* HHI_MIPI_CNTL0 */
			/* DIF_REF_CTL1:31-16bit, DIF_REF_CTL0:15-0bit */
			lcd_hiu_write(HHI_MIPI_CNTL0,
				(0xa487 << 16) | (0x8 << 0));

			/* HHI_MIPI_CNTL1 */
			/* DIF_REF_CTL2:15-0bit; bandgap bit16 */
			lcd_hiu_write(HHI_MIPI_CNTL1,
				(0x1 << 16) | (0x002e << 0));

			/* HHI_MIPI_CNTL2 */
			/* DIF_TX_CTL1:31-16bit, DIF_TX_CTL0:15-0bit */
			lcd_hiu_write(HHI_MIPI_CNTL2,
				(0x2680 << 16) | (0x45a << 0));
			break;
		default: /* LCD_CHIP_AXG */
			/* HHI_MIPI_CNTL0 */
			/* DIF_REF_CTL1:31-16bit, DIF_REF_CTL0:15-0bit */
			lcd_hiu_setb(HHI_MIPI_CNTL0, 0x1b8, 16, 10);
			lcd_hiu_setb(HHI_MIPI_CNTL0, 1, 26, 1); /* bandgap */
			lcd_hiu_setb(HHI_MIPI_CNTL0, 1, 29, 1); /* current */
			lcd_hiu_setb(HHI_MIPI_CNTL0, 1, 31, 1);
			lcd_hiu_setb(HHI_MIPI_CNTL0, 0x8, 0, 16);

			/* HHI_MIPI_CNTL1 */
			/* DIF_REF_CTL2:15-0bit */
			lcd_hiu_write(HHI_MIPI_CNTL1, (0x001e << 0));

			/* HHI_MIPI_CNTL2 */
			/* DIF_TX_CTL1:31-16bit, DIF_TX_CTL0:15-0bit */
			lcd_hiu_write(HHI_MIPI_CNTL2,
				(0x26e0 << 16) | (0x459 << 0));
			break;
		}

		phy_reg = HHI_MIPI_CNTL2;
		phy_bit = MIPI_PHY_LANE_BIT;
		phy_width = MIPI_PHY_LANE_WIDTH;
		switch (pconf->lcd_control.mipi_config->lane_num) {
		case 1:
			lane_cnt = DSI_LANE_COUNT_1;
			break;
		case 2:
			lane_cnt = DSI_LANE_COUNT_2;
			break;
		case 3:
			lane_cnt = DSI_LANE_COUNT_3;
			break;
		case 4:
			lane_cnt = DSI_LANE_COUNT_4;
			break;
		default:
			lane_cnt = 0;
			break;
		}
		lcd_hiu_setb(phy_reg, lane_cnt, phy_bit, phy_width);
	} else {
		switch (lcd_drv->data->chip_type) {
		case LCD_CHIP_G12A:
		case LCD_CHIP_G12B:
		case LCD_CHIP_SM1:
			lcd_hiu_write(HHI_MIPI_CNTL0, 0);
			lcd_hiu_write(HHI_MIPI_CNTL1, 0);
			lcd_hiu_write(HHI_MIPI_CNTL2, 0);
			break;
		default:/* LCD_CHIP_AXG */
			lcd_hiu_setb(HHI_MIPI_CNTL0, 0, 16, 10);
			lcd_hiu_setb(HHI_MIPI_CNTL0, 0, 31, 1);
			lcd_hiu_setb(HHI_MIPI_CNTL0, 0, 0, 16);
			lcd_hiu_write(HHI_MIPI_CNTL1, 0x6);
			lcd_hiu_write(HHI_MIPI_CNTL2, 0x00200000);
			break;
		}
	}
}

void lcd_phy_tcon_chpi_bbc_init_tl1(int delay)
{
	unsigned int data32 = 0x06020602;
	unsigned int tmp = 0;

	udelay(delay);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL1, 1, 3, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL1, 1, 19, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL2, 1, 3, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL2, 1, 19, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL3, 1, 3, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL3, 1, 19, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL4, 1, 3, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL4, 1, 19, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL6, 1, 3, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL6, 1, 19, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL7, 1, 3, 1);
	lcd_hiu_setb(HHI_DIF_CSI_PHY_CNTL7, 1, 19, 1);
	LCDPR("%s: delay: %dus\n", __func__, delay);

	data32 |= ((phy_ctrl_bit_on << 16) |
		   (phy_ctrl_bit_on << 0));
	tmp |= ((1 << 18) | (1 << 2));

	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL14, 0xff2027ef);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL15, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL16, 0x80000000);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL8, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL1, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL9, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL2, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL10, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL3, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL11, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL4, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL12, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL6, data32);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL13, tmp);
	lcd_hiu_write(HHI_DIF_CSI_PHY_CNTL7, data32);
}

int lcd_phy_probe(void)
{
	struct aml_lcd_drv_s *lcd_drv = aml_lcd_get_driver();

	switch (lcd_drv->data->chip_type) {
	case LCD_CHIP_TL1:
		if (is_meson_rev_c())
			phy_ctrl_bit_on = 1;
		else
			phy_ctrl_bit_on = 0;
		break;
	case LCD_CHIP_G12B:
		if (is_meson_rev_b())
			phy_ctrl_bit_on = 1;
		else
			phy_ctrl_bit_on = 0;
		break;
	default:
		break;
	}

	return 0;
}
