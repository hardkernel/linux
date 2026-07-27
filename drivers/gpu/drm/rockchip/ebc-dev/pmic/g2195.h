/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026 Rockchip Electronics Co., Ltd.
 */
#ifndef _G2195_H_
#define _G2195_H_

#include <linux/regmap.h>

#define G2195_REG_TMST			0x00
#define G2195_REG_VPOS1			0x01
#define G2195_REG_VNEG1			0x03
#define G2195_REG_VPOS2			0x05
#define G2195_REG_VNEG2			0x07
#define G2195_REG_VPOS3			0x09
#define G2195_REG_VNEG3			0x0b
#define G2195_REG_DCVCOM		0x0d
#define G2195_REG_VCOMH			0x0f
#define G2195_REG_VCOML			0x11
#define G2195_REG_VGH1			0x16
#define G2195_REG_VGH2			0x18
#define G2195_REG_FAULT_FLAGS		0x1d

#define G2195_VSEL_10BIT_MASK		GENMASK(9, 0)
#define G2195_VSEL_9BIT_MASK		GENMASK(8, 0)

#ifdef CONFIG_EPD_G2195_PMIC
extern const struct regmap_config regmap_config_g2195;
#endif

#endif

