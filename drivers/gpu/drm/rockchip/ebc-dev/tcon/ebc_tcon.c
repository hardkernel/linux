// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/clk-conf.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "ebc_tcon.h"

#define HIWORD_UPDATE(x, l, h)	(((x) << (l)) | (GENMASK(h, l) << 16))
#define UPDATE(x, h, l)		(((x) << (l)) & GENMASK((h), (l)))

#define REG_LOAD_GLOBAL_EN	0x1

/* ebc register define */
#define EBC_DSP_START		0x0000 //Frame statrt register
#define EBC_EPD_CTRL			0x0004 //EPD control register
#define EBC_DSP_CTRL			0x0008 //Display control register
#define EBC_DSP_HTIMING0		0x000c //H-Timing setting register0
#define EBC_DSP_HTIMING1		0x0010 //H-Timing setting register1
#define EBC_DSP_VTIMING0		0x0014 //V-Timing setting register0
#define EBC_DSP_VTIMING1		0x0018 //V-Timing setting register1
#define EBC_DSP_ACT_INFO		0x001c //ACTIVE width/height
#define EBC_WIN_CTRL			0x0020 //Window ctrl
#define EBC_WIN_MST0			0x0024 //Current win memory start
#define EBC_WIN_MST1			0x0028 //Next win memory start
#define EBC_WIN_VIR			0x002c //Window vir width/height
#define EBC_WIN_ACT			0x0030 //Window act width/height
#define EBC_WIN_DSP			0x0034 //Window dsp width/height
#define EBC_WIN_DSP_ST		0x0038 //Window display start piont
#define EBC_INT_STATUS		0x003c //Interrupt register
#define EBC_VCOM0			0x0040 //VCOM setting register0
#define EBC_VCOM1			0x0044 //VCOM setting register1
#define EBC_VCOM2			0x0048 //VCOM setting register2
#define EBC_VCOM3			0x004c //VCOM setting register3
#define EBC_CONFIG_DONE		0x0050 //Config done register
#define EBC_VNUM				0x0054 //Line flag num
#define EBC_WIN_MST2			0x0058 //Framecount memory start
#define EBC_LUT_DATA_ADDR	0x1000 //lut data address

#define DSP_HTOTAL(x)			UPDATE(x, 27, 16)
#define DSP_HS_END(x)			UPDATE(x, 7, 0)
#define DSP_HACT_END(x)		UPDATE(x, 26, 16)
#define DSP_HACT_ST(x)		UPDATE(x, 7, 0)
#define DSP_VTOTAL(x)			UPDATE(x, 26, 16)
#define DSP_VS_END(x)			UPDATE(x, 7, 0)
#define DSP_VACT_END(x)		UPDATE(x, 26, 16)
#define DSP_VACT_ST(x)		UPDATE(x, 7, 0)
#define DSP_HEIGHT(x)			UPDATE(x, 26, 16)
#define DSP_WIDTH(x)			UPDATE(x, 11, 0)

#define WIN2_FIFO_ALMOST_FULL_LEVEL(x)	UPDATE(x, 27, 19)
#define WIN_EN(x)				UPDATE(x, 18, 18)
#define BURST_REG(x)			UPDATE(x, 12, 10)
#define WIN_FIFO_ALMOST_FULL_LEVEL(x)	UPDATE(x, 9, 2)
#define WIN_FMT(x)			UPDATE(x, 1, 0)

#define WIN_VIR_HEIGHT(x)		UPDATE(x, 31, 16)
#define WIN_VIR_WIDTH(x)		UPDATE(x, 15, 0)
#define WIN_ACT_HEIGHT(x)	UPDATE(x, 26, 16)
#define WIN_ACT_WIDTH(x)		UPDATE(x, 11, 0)
#define WIN_DSP_HEIGHT(x)	UPDATE(x, 26, 16)
#define WIN_DSP_WIDTH(x)		UPDATE(x, 11, 0)
#define WIN_DSP_YST(x)		UPDATE(x, 26, 16)
#define WIN_DSP_XST(x)		UPDATE(x, 11, 0)

#define DSP_OUT_LOW			BIT(31)
#define DSP_EINK_MODE(x)		UPDATE(x, 13, 13)
#define DSP_EINK_MODE_MASK	BIT(13)
#define DSP_SDCE_WIDTH(x)	UPDATE(x, 25, 16)
#define DSP_FRM_TOTAL(x)		UPDATE(x, 9, 2)
#define DSP_FRM_TOTAL_MASK	GENMASK(9, 2)
#define DSP_FRM_START		BIT(0)
#define DSP_FRM_START_MASK	BIT(0)
#define SW_BURST_CTRL		BIT(12)

#define EINK_MODE_SWAP(x)	UPDATE(x, 31, 31)
#define EINK_MODE_FRM_SEL(x)	UPDATE(x, 30, 30)
#define DSP_GD_END(x)			UPDATE(x, 26, 16)
#define DSP_GD_ST(x)			UPDATE(x, 15, 8)
#define DSP_THREE_WIN_MODE(x)		UPDATE(x, 7, 7)
#define THREE_WIN_MODE_MASK		BIT(7)
#define DSP_SDDW_MODE(x)	UPDATE(x, 6, 6)
#define EPD_AUO(x)			UPDATE(x, 5, 5)
#define EPD_PWR(x)			UPDATE(x, 4, 2)
#define EPD_GDRL(x)			UPDATE(x, 1, 1)
#define EPD_SDSHR(x)			UPDATE(x, 0, 0)

#define DSP_SWAP_MODE(x)	UPDATE(x, 31, 30)
#define DSP_SDCLK_DIV(x)		UPDATE(x, 19, 16)
#define DSP_SDCLK_DIV_MASK	GENMASK(19, 16)
#define DSP_VCOM_MODE(x)		UPDATE(x, 27, 27)

#define DSP_UPDATE_MODE(x)	UPDATE(x, 29, 29)
#define DSP_DISPLAY_MODE(x)	UPDATE(x, 28, 28)
#define UPDATE_MODE_MASK	BIT(29)
#define DISPLAY_MODE_MASK	BIT(28)

#define DSP_FRM_INT_NUM(x)	UPDATE(x, 19, 12)
#define FRM_END_INT			BIT(0)
#define DSP_END_INT			BIT(1)
#define DSP_FRM_INT			BIT(2)
#define LINE_FLAG_INT			BIT(3)
#define FRM_END_INT_MASK	BIT(4)
#define DSP_END_INT_MASK	BIT(5)
#define DSP_FRM_INT_MASK	BIT(6)
#define LINE_FLAG_INT_MASK	BIT(7)
#define FRM_END_INT_CLR		BIT(8)
#define DSP_END_INT_CLR		BIT(9)
#define DSP_FRM_INT_CLR		BIT(10)
#define LINE_FLAG_INT_CLR	BIT(11)

#define RK3576_EBC_DSP_START		0x0000
#define RK3576_EBC_EPD_CTRL		0x0004
#define RK3576_EBC_DSP_CTRL		0x0008
#define RK3576_EBC_DSP_HTIMING0		0x000C
#define RK3576_EBC_DSP_HTIMING1		0x0010
#define RK3576_EBC_DSP_VTIMING0		0x0014
#define RK3576_EBC_DSP_VTIMING1		0x0018
#define RK3576_EBC_DSP_ACT_INFO		0x001C
#define RK3576_EBC_WIN_CTRL		0x0020
#define RK3576_EBC_WIN_MST0		0x0024
#define RK3576_EBC_WIN_MST1		0x0028
#define RK3576_EBC_WIN_VIR		0x002C
#define RK3576_EBC_WIN_ACT		0x0030
#define RK3576_EBC_WIN_DSP		0x0034
#define RK3576_EBC_WIN_DSP_ST		0x0038
#define RK3576_EBC_INT_STATUS		0x003C
#define RK3576_EBC_VCOM0		0x0040
#define RK3576_EBC_VCOM1		0x0044
#define RK3576_EBC_VCOM2		0x0048
#define RK3576_EBC_VCOM3		0x004C
#define RK3576_EBC_CONFIG_DONE		0x0050
#define RK3576_EBC_VNUM			0x0054
#define RK3576_EBC_WIN_MST2		0x0058
#define RK3576_EBC_DSP_CTRL2		0x005C
#define RK3576_EBC_DSP_CTRL3		0x0060
#define RK3576_EBC_WIN0_CTRL		0x0064
#define RK3576_EBC_WIN1_CTRL		0x0068
#define RK3576_EBC_WIN2_CTRL		0x006C
#define RK3576_EBC_SYS_CTRL		0x0070
#define RK3576_EBC_LUT_ADDRESS_MAP_0	0x1000

#define RK3576_DSP_FRM_START		BIT(0)
#define RK3576_DSP_FRM_START_MASK	BIT(0)

#define RK3576_EINK_MODE_SWAP(x)	UPDATE(x, 31, 31)
#define RK3576_EINK_MODE_FRM_SEL(x)	UPDATE(x, 30, 30)
#define RK3576_DSP_THREE_WIN_MODE(x)	UPDATE(x, 29, 29)
#define RK3576_THREE_WIN_MODE_MASK	BIT(29)
#define RK3576_DSP_GD_END(x)		UPDATE(x, 26, 16)
#define RK3576_DSP_GD_ST(x)		UPDATE(x, 15, 8)
#define RK3576_DSP_SDDW_MODE(x)		UPDATE(x, 7, 6)
#define RK3576_EPD_AUO(x)		UPDATE(x, 5, 5)
#define RK3576_EPD_PWR(x)		UPDATE(x, 4, 2)
#define RK3576_EPD_GDRL(x)		UPDATE(x, 1, 1)
#define RK3576_EPD_SDSHR(x)		UPDATE(x, 0, 0)

#define RK3576_DSP_SWAP_MODE(x)		UPDATE(x, 31, 30)
#define RK3576_DSP_UPDATE_MODE(x)	UPDATE(x, 29, 29)
#define RK3576_DSP_DISPLAY_MODE(x)	UPDATE(x, 28, 28)
#define RK3576_UPDATE_MODE_MASK		BIT(29)
#define RK3576_DISPLAY_MODE_MASK	BIT(28)
#define RK3576_DSP_VCOM_MODE(x)		UPDATE(x, 27, 27)
#define RK3576_DSP_SDCLK_POL(x)		UPDATE(x, 20, 20)
#define RK3576_DSP_SDCLK_DIV(x)		UPDATE(x, 19, 16)
#define RK3576_DSP_SDCLK_DIV_MASK	GENMASK(19, 16)
#define RK3576_DSP_SDOE_MODE(x)		UPDATE(x, 0, 0)

#define RK3576_DSP_HTOTAL(x)		UPDATE(x, 31, 16)
#define RK3576_DSP_HS_END(x)		UPDATE(x, 15, 0)
#define RK3576_DSP_HACT_END(x)		UPDATE(x, 31, 16)
#define RK3576_DSP_HACT_ST(x)		UPDATE(x, 15, 0)
#define RK3576_DSP_VTOTAL(x)		UPDATE(x, 31, 16)
#define RK3576_DSP_VS_END(x)		UPDATE(x, 15, 0)
#define RK3576_DSP_VACT_END(x)		UPDATE(x, 31, 16)
#define RK3576_DSP_VACT_ST(x)		UPDATE(x, 15, 0)
#define RK3576_DSP_HEIGHT(x)		UPDATE(x, 31, 16)
#define RK3576_DSP_WIDTH(x)		UPDATE(x, 15, 0)

#define RK3576_WIN_VIR_HEIGHT(x)	UPDATE(x, 31, 16)
#define RK3576_WIN_VIR_WIDTH(x)		UPDATE(x, 15, 0)
#define RK3576_WIN_ACT_HEIGHT(x)	UPDATE(x, 31, 16)
#define RK3576_WIN_ACT_WIDTH(x)		UPDATE(x, 15, 0)
#define RK3576_WIN_DSP_HEIGHT(x)	UPDATE(x, 31, 16)
#define RK3576_WIN_DSP_WIDTH(x)		UPDATE(x, 15, 0)
#define RK3576_WIN_DSP_YST(x)		UPDATE(x, 31, 16)
#define RK3576_WIN_DSP_XST(x)		UPDATE(x, 15, 0)

#define RK3576_DMA_BURST_LENGTH(x)		UPDATE(x, 25, 24)
#define RK3576_WIN2_FIFO_ALMOST_FULL_LEVEL(x)	UPDATE(x, 20, 12)
#define RK3576_WIN_FIFO_ALMOST_FULL_LEVEL(x)	UPDATE(x, 11, 4)
#define RK3576_WIN_FMT(x)			UPDATE(x, 2, 0)
#define RK3576_WIN_FMT_MASK			GENMASK(2, 0)

#define RK3572_WIN2_FIFO_LEVEL(x)	UPDATE(x, 25, 16)
#define RK3576_WIN_RID(x)		UPDATE(x, 7, 4)
#define RK3576_WIN_AXI_GATHER_NUM(x)	UPDATE(x, 11, 8)
#define RK3576_WIN_AXI_GATHER_EN	BIT(1)
#define RK3576_WIN_EN			BIT(0)

#define RK3572_RDATA_FIFO_EMPTY_INT		BIT(31)
#define RK3572_RDATA_FIFO_EMPTY_INT_MASK	BIT(27)
#define RK3576_WIN2_EMPTY_INT_MASK		BIT(26)
#define RK3576_WIN1_EMPTY_INT_MASK		BIT(25)
#define RK3576_WIN0_EMPTY_INT_MASK		BIT(24)
#define RK3572_RDATA_FIFO_EMPTY_INT_CLR		BIT(23)
#define RK3576_FRM_END_INT_MASK			BIT(4)
#define RK3576_DSP_END_INT_MASK			BIT(5)
#define RK3576_DSP_FRM_INT_MASK			BIT(6)
#define RK3576_LINE_FLAG_INT_MASK		BIT(7)

#define RK3572_LUT_MEM_AXI_BUS_SEL(x)	UPDATE(x, 28, 28)
#define RK3572_WIN2_MUX(x)		UPDATE(x, 27, 27)
#define RK3572_OUTPUT_MODE(x)		UPDATE(x, 26, 26)
#define RK3572_DISPLAY_ALIGN(x)		UPDATE(x, 25, 25)
#define RK3572_DISPLAY_MODE(x)		UPDATE(x, 24, 24)
#define RK3576_DSP_SDCE_WIDTH(x)	UPDATE(x, 23, 12)
#define RK3576_DSP_SDCE_WIDTH_MASK(x)	GENMASK(x, 23, 12)
#define RK3576_DSP_FRM_TOTAL(x)		UPDATE(x, 11, 4)
#define RK3576_DSP_FRM_TOTAL_MASK	GENMASK(11, 4)
#define RK3576_DSP_EINK_MODE(x)		UPDATE(x, 2, 2)
#define RK3576_DSP_EINK_MODE_MASK	BIT(2)
#define RK3576_SW_BURST_CTRL		BIT(1)
#define RK3576_DSP_OUT_LOW		BIT(0)

#define RK3576_SW_AXI_RD_URGENCY_EN		BIT(24)
#define RK3576_SW_AXI_MAX_OUTSTAND_NUM(x)	UPDATE(x, 20, 16)
#define RK3576_SW_AXI_MAX_OUTSTAND_EN		BIT(12)
#define RK3576_SW_NOC_HURRY_THRESHOLD(x)	UPDATE(x, 11, 8)
#define RK3576_SW_NOC_HURRY_VALUE(x)		UPDATE(x, 6, 5)
#define RK3576_SW_NOC_HURRY_EN			BIT(4)
#define RK3576_SW_NOC_QOS_VALUE(x)		UPDATE(x, 2, 1)
#define RK3576_SW_NOC_QOS_EN			BIT(0)

#define RK3576_REG_CONFIG_DONE		BIT(0)

#define RK3572_LUT_DATA_ADDR		0x2000

#define EBC_PLL_LIMIT_FREQ		594000000
#define EBC_PLL_MIN_FREQ		40000000

enum ebc_win_data_format {
	Y_DATA_4BPP = 0,
	Y_DATA_8BPP = 1,
	RGB888 = 2,
	RGB565 = 3,
	Y_DATA_5BPP = 4,
};

struct rockchip_ebc_tcon_data {
	bool (*volatile_reg)(struct device *dev, unsigned int reg);
	struct ebc_tcon tcon;
};

static inline void tcon_write(struct ebc_tcon *tcon, unsigned int reg,
			      unsigned int value)
{
	regmap_write(tcon->regmap_base, reg, value);
}

static inline unsigned int tcon_read(struct ebc_tcon *tcon, unsigned int reg)
{
	unsigned int value;

	regmap_read(tcon->regmap_base, reg, &value);

	return value;
}

static inline void tcon_update_bits(struct ebc_tcon *tcon, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(tcon->regmap_base, reg, mask, val);
}

static inline void tcon_cfg_done(struct ebc_tcon *tcon)
{
	regmap_write(tcon->regmap_base, EBC_CONFIG_DONE, REG_LOAD_GLOBAL_EN);
}

static inline void rk3576_tcon_cfg_done(struct ebc_tcon *tcon)
{
	regmap_write(tcon->regmap_base, RK3576_EBC_CONFIG_DONE, RK3576_REG_CONFIG_DONE);
}

static int rk3576_tcon_enable(struct ebc_tcon *tcon, struct ebc_panel *panel)
{
	u32 width, height, vir_width, vir_height, val;
	u16 lsl, lbl, ldl, lel, gdck_sta, lgonl;
	u8 sddw_mode, swap_mode, display_mode, output_mode, sdclk_pol;

	clk_prepare_enable(tcon->hclk);
	clk_prepare_enable(tcon->aclk);
	clk_prepare_enable(tcon->dclk);
	pm_runtime_get_sync(tcon->dev);

	if (panel->rearrange) {
		width = panel->width * 2;
		height = panel->height / 2;
		vir_width = panel->vir_width * 2;
		vir_height = panel->vir_height / 2;
	} else if (panel->gate_dummy_lenth > 0) {
		width = panel->width;
		height = panel->height + panel->gate_dummy_lenth;
		vir_width = panel->vir_width;
		vir_height = panel->vir_height + panel->gate_dummy_lenth;
	} else {
		width = panel->width;
		height = panel->height;
		vir_width = panel->vir_width;
		vir_height = panel->vir_height;
	}

	lsl = panel->lsl;
	lbl = panel->lbl;
	ldl = panel->ldl;
	lel = panel->lel;
	gdck_sta = panel->gdck_sta;
	lgonl = panel->lgonl;

	if (panel->data_rate == 2) {
		lsl *= 2;
		lbl *= 2;
		ldl *= 2;
		lel *= 2;
		gdck_sta *= 2;
		lgonl *= 2;
	}

	/* panel timing and win info config */
	tcon_write(tcon, RK3576_EBC_DSP_HTIMING0,
		   RK3576_DSP_HTOTAL(lsl + lbl + ldl + lel) |
		   RK3576_DSP_HS_END(lsl));
	val = lsl + lbl + ldl + (panel->lel_keep_clk ? lel : 0);
	tcon_write(tcon, RK3576_EBC_DSP_HTIMING1,
		   RK3576_DSP_HACT_END(val) | RK3576_DSP_HACT_ST(lsl + lbl - 1));
	tcon_write(tcon, RK3576_EBC_DSP_VTIMING0,
		   RK3576_DSP_VTOTAL(panel->fsl + panel->fbl + panel->fdl + panel->fel) |
		   RK3576_DSP_VS_END(panel->fsl));
	tcon_write(tcon, RK3576_EBC_DSP_VTIMING1,
		   RK3576_DSP_VACT_END(panel->fsl + panel->fbl + panel->fdl) |
		   RK3576_DSP_VACT_ST(panel->fsl + panel->fbl));
	tcon_write(tcon, RK3576_EBC_DSP_ACT_INFO, RK3576_DSP_HEIGHT(vir_height) |
		   RK3576_DSP_WIDTH(vir_width));
	tcon_write(tcon, RK3576_EBC_WIN_VIR, RK3576_WIN_VIR_HEIGHT(height) |
		   RK3576_WIN_VIR_WIDTH(width));
	tcon_write(tcon, RK3576_EBC_WIN_ACT, RK3576_WIN_ACT_HEIGHT(height) |
		   RK3576_WIN_ACT_WIDTH(width));
	tcon_write(tcon, RK3576_EBC_WIN_DSP, RK3576_WIN_DSP_HEIGHT(height) |
		   RK3576_WIN_DSP_WIDTH(width));
	if (!PANEL_IS_24BIT(panel) && (width != vir_width || height != vir_height)) {
		if (((vir_width - width) / 2) % 2)
			dev_warn(tcon->dev,
				 "Margin left/right between width/vir_width must be same\n");
		if (((vir_height - height) / 2) % 2)
			dev_warn(tcon->dev,
				 "Margin top/bottom between height/vir_height must be same\n");

		val = PANEL_IS_16BIT(panel) ? 8 : 4;
		if (((vir_width - width) / 2) % val)
			dev_warn(tcon->dev,
				 "Margin left/right between width/vir_width must align with %d\n",
				 val);

		val = RK3576_WIN_DSP_YST(panel->fsl + panel->fbl + (vir_height - height) / 2);
		if (PANEL_IS_16BIT(panel))
			val |= RK3576_WIN_DSP_XST(lsl + lbl +
						  (vir_width - width) / 2 / 8);
		else
			val |= RK3576_WIN_DSP_XST(lsl + lbl +
						  (vir_width - width) / 2 / 4);
		tcon_write(tcon, RK3576_EBC_WIN_DSP_ST, val);
	} else {
		tcon_write(tcon, RK3576_EBC_WIN_DSP_ST,
			   RK3576_WIN_DSP_YST(panel->fsl + panel->fbl) |
			   RK3576_WIN_DSP_XST(lsl + lbl));
	}

	/*
	 * win2 fifo is 512x64, win fifo is 256x64,
	 * we set fifo almost value (fifo_size - 16(brust_length))
	 *
	 * dma_burst_length mean axi burst length = 16
	 */
	tcon_write(tcon, RK3576_EBC_WIN_CTRL, RK3576_WIN2_FIFO_ALMOST_FULL_LEVEL(496) |
		   RK3576_DMA_BURST_LENGTH(0) | RK3576_WIN_FIFO_ALMOST_FULL_LEVEL(240) |
		   RK3576_WIN_FMT(Y_DATA_4BPP));

	/* win0 always enable */
	tcon_write(tcon, RK3576_EBC_WIN0_CTRL, RK3576_WIN_AXI_GATHER_NUM(8) |
		   RK3576_WIN_AXI_GATHER_EN | RK3576_WIN_RID(1) | RK3576_WIN_EN);
	tcon_write(tcon, RK3576_EBC_WIN1_CTRL, RK3576_WIN_RID(2));
	if (tcon->version == EBC_VERSION_RK3576)
		tcon_write(tcon, RK3576_EBC_WIN2_CTRL, RK3576_WIN_RID(3));
	else
		tcon_write(tcon, RK3576_EBC_WIN2_CTRL, RK3576_WIN_RID(3) |
			   RK3572_WIN2_FIFO_LEVEL(0xa));

	tcon_write(tcon, RK3576_EBC_SYS_CTRL,
		   RK3576_SW_AXI_RD_URGENCY_EN | RK3576_SW_NOC_HURRY_THRESHOLD(12) |
		   RK3576_SW_NOC_HURRY_VALUE(3) | RK3576_SW_NOC_HURRY_EN |
		   RK3576_SW_NOC_QOS_VALUE(3) | RK3576_SW_NOC_QOS_EN);

	if (PANEL_IS_24BIT(panel))
		sddw_mode = 2;
	else if (PANEL_IS_16BIT(panel))
		sddw_mode = 1;
	else
		sddw_mode = 0;

	/*
	 * RK3576_EBC_EPD_CTRL info:
	 * DSP_GD_END : GCLK falling edge point(SCLK), which count from the rising edge of hsync
	 * DSP_GD_ST: GCLK rising edge point(SCLK), which count from the rising edge of hsync
	 * DSP_THREE_WIN_MODE: 0: lut mode or direct mode; 1: three win mode
	 * DSP_SDDW_MODE: 0: 8 bit data output; 1: 16 bit data output
	 * EPD_AUO: 0: EINK; 1:AUO
	 * EPD_GDRL: gate scanning direction: 1: button to top 0: top to button
	 * EPD_SDSHR: source scanning direction 1: right to left 0: left to right
	 */
	tcon_write(tcon, RK3576_EBC_EPD_CTRL, RK3576_EINK_MODE_SWAP(1) |
		   RK3576_DSP_GD_ST(lsl + gdck_sta) |
		   RK3576_DSP_GD_END(lsl + gdck_sta + lgonl) |
		   RK3576_DSP_THREE_WIN_MODE(0) | RK3576_DSP_SDDW_MODE(sddw_mode) |
		   RK3576_EPD_AUO(0) | RK3576_EPD_GDRL(1) | RK3576_EPD_SDSHR(1));

	tcon_write(tcon, RK3576_EBC_DSP_START, 0);

	if (panel->sdce_width == 0)
		val = RK3576_DSP_SDCE_WIDTH(ldl);
	else
		val = RK3576_DSP_SDCE_WIDTH(panel->sdce_width);

	/* WIN2 should be enable for ebc mode */
	if (tcon->version == EBC_VERSION_RK3572)
		val |= RK3572_WIN2_MUX(1);

	if (panel->data_rate == 2) {
		output_mode = 1;
		sdclk_pol = 1;
	} else {
		output_mode = 0;
		sdclk_pol = 0;
	}

	/*
	 * display_mode: 1 for 3bit/4bit per pixel, 0 for 2bit per pixel
	 * swap_mode: { P0, P1, P2, P3, ... }
	 */
	if (PANEL_IS_24BIT(panel)) {
		display_mode = 1;
		swap_mode = 2;
	} else if (PANEL_IS_16BIT(panel)) {
		if ((panel->data_rate == 1 && panel->ldl * 4 == panel->vir_width) ||
		    (panel->data_rate == 2 && panel->ldl * 8 == panel->vir_width)) {
			/* 4bit per pixel */
			display_mode = 1;
			swap_mode = 3;
		} else {
			/* 2bit per pixel */
			display_mode = 0;
			swap_mode = 2;
		}
	} else {
		display_mode = 0;
		swap_mode = 3;
	}

	if (tcon->version == EBC_VERSION_RK3576)
		tcon_write(tcon, RK3576_EBC_DSP_CTRL2, RK3576_SW_BURST_CTRL | val);
	else
		tcon_write(tcon, RK3576_EBC_DSP_CTRL2, RK3576_SW_BURST_CTRL |
			   RK3572_DISPLAY_MODE(display_mode) | RK3572_OUTPUT_MODE(output_mode) |
			   val);

	/**
	 *  SDOE_MODE 1 : sdce signal act as vden
	 *  SDOE_MODE 0 : sdce signal act as hden
	 */
	if (panel->sdoe_mode == 1)
		val = RK3576_DSP_SDOE_MODE(1);
	else
		val = RK3576_DSP_SDOE_MODE(0);
	tcon_write(tcon, RK3576_EBC_DSP_CTRL,
		   RK3576_DSP_SWAP_MODE(swap_mode) | RK3576_DSP_VCOM_MODE(1) |
		   RK3576_DSP_SDCLK_POL(sdclk_pol) | val);
	/* unmask RK3576_DSP_END_INT_MASK */
	tcon_update_bits(tcon, RK3576_EBC_INT_STATUS, RK3576_DSP_END_INT_MASK |
			 RK3576_DSP_FRM_INT_MASK | RK3576_FRM_END_INT_MASK |
			 RK3576_WIN2_EMPTY_INT_MASK | RK3576_WIN1_EMPTY_INT_MASK |
			 RK3576_WIN0_EMPTY_INT_MASK | RK3572_RDATA_FIFO_EMPTY_INT_MASK, 0 |
			 RK3576_DSP_FRM_INT_MASK | RK3576_FRM_END_INT_MASK |
			 RK3576_WIN2_EMPTY_INT_MASK | RK3576_WIN1_EMPTY_INT_MASK |
			 RK3576_WIN0_EMPTY_INT_MASK);

	rk3576_tcon_cfg_done(tcon);

	enable_irq(tcon->irq);
	tcon->panel = panel;

	pinctrl_pm_select_default_state(tcon->dev);

	return 0;
}

static void rk3576_tcon_disable(struct ebc_tcon *tcon)
{
	disable_irq(tcon->irq);
	pinctrl_pm_select_sleep_state(tcon->dev);

	pm_runtime_put_sync(tcon->dev);
	clk_disable_unprepare(tcon->dclk);
	clk_disable_unprepare(tcon->aclk);
	clk_disable_unprepare(tcon->hclk);
}

static void rk3576_tcon_dsp_mode_set(struct ebc_tcon *tcon, int update_mode,
				     int display_mode, int three_win_mode,
				     int eink_mode)
{
	struct ebc_panel *panel = tcon->panel;
	u32 val;
	int ret;

	if (panel && display_mode != tcon->display_mode) {
		if (display_mode == DIRECT_MODE && PANEL_IS_16BIT(panel))
			ret = tcon->clk_set_rate(tcon->dclk, panel->sdck);
		else
			ret = tcon->clk_set_rate(tcon->dclk, panel->sdck *
						 ((PANEL_IS_16BIT(panel) ? 7 : 3) + 1));
		if (ret)
			dev_err(tcon->dev, "Failed to set dclk:%d\n", ret);
	}

	if (display_mode == DIRECT_MODE && panel && PANEL_IS_16BIT(panel))
		val = RK3576_DSP_SDCLK_DIV(0);
	else
		val = RK3576_DSP_SDCLK_DIV((panel && PANEL_IS_16BIT(panel)) ? 7 : 3);

	tcon->display_mode = display_mode;

	tcon_write(tcon, RK3576_EBC_WIN1_CTRL, RK3576_WIN_AXI_GATHER_NUM(8) |
		   RK3576_WIN_AXI_GATHER_EN | RK3576_WIN_RID(2) | ((!!display_mode) |
		   (!!three_win_mode)));
	tcon_write(tcon, RK3576_EBC_WIN2_CTRL, RK3576_WIN_AXI_GATHER_NUM(8) |
		   RK3576_WIN_AXI_GATHER_EN | RK3576_WIN_RID(3) | (!!three_win_mode));

	tcon_update_bits(tcon, RK3576_EBC_DSP_CTRL, RK3576_UPDATE_MODE_MASK |
			 RK3576_DISPLAY_MODE_MASK | RK3576_DSP_SDCLK_DIV_MASK,
			 RK3576_DSP_UPDATE_MODE(!!update_mode) |
			 RK3576_DSP_DISPLAY_MODE(!!display_mode) | val);
	tcon_update_bits(tcon, RK3576_EBC_EPD_CTRL, RK3576_THREE_WIN_MODE_MASK,
			 RK3576_DSP_THREE_WIN_MODE(!!three_win_mode));
	/* always set frm start bit 0 before real frame start */
	tcon_update_bits(tcon, RK3576_EBC_DSP_CTRL2, RK3576_DSP_EINK_MODE_MASK,
			 RK3576_DSP_EINK_MODE(!!eink_mode));
	rk3576_tcon_cfg_done(tcon);
}

static void rk3572_tcon_dsp_mode_set(struct ebc_tcon *tcon, int update_mode,
				     int display_mode, int three_win_mode,
				     int eink_mode)
{
	struct ebc_panel *panel = tcon->panel;
	u32 rate;
	u8 sdclk_div;
	int ret;

	/* 4 pixel per clk */
	sdclk_div = 1;
	if (panel->data_rate == 2)
		rate = panel->sdck * 4;
	else
		rate = panel->sdck * 2;

	ret = tcon->clk_set_rate(tcon->dclk, rate);
	if (ret)
		dev_err(tcon->dev, "Failed to set dclk:%d\n", ret);

	tcon->display_mode = display_mode;

	tcon_write(tcon, RK3576_EBC_WIN1_CTRL, RK3576_WIN_AXI_GATHER_NUM(8) |
		   RK3576_WIN_AXI_GATHER_EN | RK3576_WIN_RID(2) | ((!!display_mode)));

	tcon_update_bits(tcon, RK3576_EBC_DSP_CTRL, RK3576_UPDATE_MODE_MASK |
			 RK3576_DISPLAY_MODE_MASK | RK3576_DSP_SDCLK_DIV_MASK,
			 RK3576_DSP_UPDATE_MODE(!!update_mode) |
			 RK3576_DSP_DISPLAY_MODE(!!display_mode) |
			 RK3576_DSP_SDCLK_DIV(sdclk_div));
	rk3576_tcon_cfg_done(tcon);
}

static void rk3576_tcon_image_addr_set(struct ebc_tcon *tcon, u32 pre_image_addr,
				       u32 cur_image_addr)
{
	tcon_write(tcon, RK3576_EBC_WIN_MST0, pre_image_addr);
	tcon_write(tcon, RK3576_EBC_WIN_MST1, cur_image_addr);
	rk3576_tcon_cfg_done(tcon);
}

static void rk3576_tcon_frame_addr_set(struct ebc_tcon *tcon, u32 frame_addr)
{
	tcon_write(tcon, RK3576_EBC_WIN_MST2, frame_addr);
	rk3576_tcon_cfg_done(tcon);
}

static void rk3576_tcon_data_format_set(struct ebc_tcon *tcon, enum ebc_tcon_data_format format)
{
	tcon_update_bits(tcon, RK3576_EBC_WIN_CTRL, RK3576_WIN_FMT_MASK, RK3576_WIN_FMT(format));
	rk3576_tcon_cfg_done(tcon);
}

static void rk3576_tcon_frame_start(struct ebc_tcon *tcon, int frame_total)
{
	/* always set frm start bit 0 before real frame start */
	tcon_update_bits(tcon, RK3576_EBC_DSP_CTRL2, RK3576_DSP_FRM_TOTAL_MASK,
			 RK3576_DSP_FRM_TOTAL(frame_total - 1));
	tcon_cfg_done(tcon);
	tcon_write(tcon, RK3576_EBC_DSP_START, 1);
}

static int tcon_enable(struct ebc_tcon *tcon, struct ebc_panel *panel)
{
	u32 width, height, vir_width, vir_height, val, div = 0;
	unsigned long dclk_rate;

	clk_prepare_enable(tcon->hclk);
	clk_prepare_enable(tcon->dclk);
	pm_runtime_get_sync(tcon->dev);

	if (panel->rearrange) {
		width = panel->width * 2;
		height = panel->height / 2;
		vir_width = panel->vir_width * 2;
		vir_height = panel->vir_height / 2;
	} else if (panel->gate_dummy_lenth > 0) {
		width = panel->width;
		height = panel->height + panel->gate_dummy_lenth;
		vir_width = panel->vir_width;
		vir_height = panel->vir_height + panel->gate_dummy_lenth;
	} else {
		width = panel->width;
		height = panel->height;
		vir_width = panel->vir_width;
		vir_height = panel->vir_height;
	}

	/* panel timing and win info config */
	tcon_write(tcon, EBC_DSP_HTIMING0,
				DSP_HTOTAL(panel->lsl + panel->lbl + panel->ldl + panel->lel) | DSP_HS_END(panel->lsl));
	val = panel->lsl + panel->lbl + panel->ldl + (panel->lel_keep_clk ? panel->lel : 0);
	tcon_write(tcon, EBC_DSP_HTIMING1,
				DSP_HACT_END(val) | DSP_HACT_ST(panel->lsl + panel->lbl - 1));
	tcon_write(tcon, EBC_DSP_VTIMING0,
				DSP_VTOTAL(panel->fsl + panel->fbl + panel->fdl + panel->fel) | DSP_VS_END(panel->fsl));
	tcon_write(tcon, EBC_DSP_VTIMING1,
				DSP_VACT_END(panel->fsl + panel->fbl + panel->fdl) | DSP_VACT_ST(panel->fsl + panel->fbl));
	tcon_write(tcon, EBC_DSP_ACT_INFO, DSP_HEIGHT(vir_height) | DSP_WIDTH(vir_width));
	tcon_write(tcon, EBC_WIN_VIR, WIN_VIR_HEIGHT(height) | WIN_VIR_WIDTH(width));
	tcon_write(tcon, EBC_WIN_ACT, WIN_ACT_HEIGHT(height) | WIN_ACT_WIDTH(width));
	tcon_write(tcon, EBC_WIN_DSP, WIN_DSP_HEIGHT(height) | WIN_DSP_WIDTH(width));
	tcon_write(tcon, EBC_WIN_DSP_ST, WIN_DSP_YST(panel->fsl + panel->fbl) | WIN_DSP_XST(panel->lsl + panel->lbl));

	/* win2 fifo is 512x128, win fifo is 256x128, we set fifo almost value (fifo_size - 16)
	*  burst_reg = 7 mean ahb burst is incr16
	*/
	tcon_write(tcon, EBC_WIN_CTRL, WIN2_FIFO_ALMOST_FULL_LEVEL(496) | WIN_EN(1)
							| BURST_REG(7) | WIN_FIFO_ALMOST_FULL_LEVEL(240) | WIN_FMT(Y_DATA_4BPP));

	/*
	 * EBC_EPD_CTRL info:
	 * DSP_GD_ST: GCLK rising edge point(SCLK), which count from the rasing edge of hsync(spec is wrong, count
	 * from rasing edge of hsync, not falling edge of hsync)
	 * DSP_GD_END : GCLK falling edge point(SCLK), which count from the rasing edge of hsync
	 * DSP_THREE_WIN_MODE: 0: lut mode or direct mode; 1: three win mode
	 * DSP_SDDW_MODE: 0: 8 bit data output; 1: 16 bit data output; 2: 24 bit data output
	 * EPD_AUO: 0: EINK; 1:AUO
	 * EPD_GDRL: gate scanning direction: 1: button to top 0: top to button
	 * EPD_SDSHR: source scanning direction 1: right to left 0: left to right
	 */
	tcon_write(tcon, EBC_EPD_CTRL, EINK_MODE_SWAP(1)
				| DSP_GD_ST(panel->lsl + panel->gdck_sta)
				| DSP_GD_END(panel->lsl + panel->gdck_sta + panel->lgonl)
				| DSP_THREE_WIN_MODE(0)
				| DSP_SDDW_MODE(!!PANEL_IS_16BIT(panel))
				| EPD_AUO(0)
				| EPD_GDRL(1)
				| EPD_SDSHR(1));
	tcon_write(tcon, EBC_DSP_START, DSP_SDCE_WIDTH(panel->ldl) | SW_BURST_CTRL);
	dclk_rate = clk_get_rate(clk_get_parent(tcon->dclk));
	div = DIV_ROUND_CLOSEST(dclk_rate, panel->sdck);
	tcon_write(tcon, EBC_DSP_CTRL,
				DSP_SWAP_MODE(PANEL_IS_16BIT(panel) ? 2 : 3) | DSP_VCOM_MODE(1) |
				DSP_SDCLK_DIV(div ? div - 1 : div));
	/* unmask DSP_END_INT_MASK */
	tcon_update_bits(tcon, EBC_INT_STATUS, FRM_END_INT_MASK | DSP_END_INT_MASK |
			 DSP_FRM_INT_MASK, FRM_END_INT_MASK | DSP_FRM_INT_MASK);
	tcon_cfg_done(tcon);

	enable_irq(tcon->irq);
	tcon->panel = panel;

	return 0;
}

static void tcon_disable(struct ebc_tcon *tcon)
{
	disable_irq(tcon->irq);
	/* output low */
	tcon_update_bits(tcon, EBC_DSP_START, DSP_OUT_LOW | DSP_FRM_START_MASK, DSP_OUT_LOW);

	pm_runtime_put_sync(tcon->dev);
	clk_disable_unprepare(tcon->dclk);
	clk_disable_unprepare(tcon->hclk);
}

static void tcon_dsp_mode_set(struct ebc_tcon *tcon, int update_mode, int display_mode, int three_win_mode, int eink_mode)
{
	tcon_update_bits(tcon, EBC_DSP_CTRL, UPDATE_MODE_MASK | DISPLAY_MODE_MASK,
						DSP_UPDATE_MODE(!!update_mode) | DSP_DISPLAY_MODE(!!display_mode));
	tcon_update_bits(tcon, EBC_EPD_CTRL, THREE_WIN_MODE_MASK,
						DSP_THREE_WIN_MODE(!!three_win_mode));
	/* always set frm start bit 0 before real frame start */
	tcon_update_bits(tcon, EBC_DSP_START, DSP_EINK_MODE_MASK | DSP_FRM_START_MASK, DSP_EINK_MODE(!!eink_mode));
	tcon_cfg_done(tcon);
}

static void tcon_image_addr_set(struct ebc_tcon *tcon, u32 pre_image_addr, u32 cur_image_addr)
{
	tcon_write(tcon, EBC_WIN_MST0, pre_image_addr);
	tcon_write(tcon, EBC_WIN_MST1, cur_image_addr);
	tcon_cfg_done(tcon);
}

static void tcon_frame_addr_set(struct ebc_tcon *tcon, u32 frame_addr)
{
	tcon_write(tcon, EBC_WIN_MST2, frame_addr);
	tcon_cfg_done(tcon);
}

static int tcon_lut_data_set(struct ebc_tcon *tcon, unsigned int *lut_data, int frame_count,
			     int lut_32)
{
	const u32 lut_base = tcon->lut_offset;
	int i;
	int lut_size;

	if ((!lut_32 && frame_count > 256) || (lut_32 && frame_count > 64)) {
		dev_err(tcon->dev, "frame count: %d over flow, lut_32:%d\n", frame_count, lut_32);
		return -EINVAL;
	}

	if (lut_32)
		lut_size = frame_count * 64;
	else
		lut_size = frame_count * 16;

	for (i = 0; i < lut_size; i++)
		tcon_write(tcon, lut_base + (i * 4), lut_data[i]);

	tcon_cfg_done(tcon);

	return 0;
}

static void tcon_frame_start(struct ebc_tcon *tcon, int frame_total)
{
	/* always set frm start bit 0 before real frame start */
	tcon_update_bits(tcon, EBC_DSP_START, DSP_FRM_TOTAL_MASK | DSP_FRM_START_MASK, DSP_FRM_TOTAL(frame_total - 1));
	tcon_cfg_done(tcon);

	tcon_update_bits(tcon, EBC_DSP_START, DSP_FRM_START_MASK, DSP_FRM_START);
}

static irqreturn_t tcon_irq_hanlder(int irq, void *dev_id)
{
	struct ebc_tcon *tcon = (struct ebc_tcon *)dev_id;
	u32 intr_status;

	intr_status = tcon_read(tcon, EBC_INT_STATUS);

	if (intr_status & RK3572_RDATA_FIFO_EMPTY_INT) {
		tcon_update_bits(tcon, EBC_INT_STATUS,
				 RK3572_RDATA_FIFO_EMPTY_INT_CLR, RK3572_RDATA_FIFO_EMPTY_INT_CLR);
		dev_err_ratelimited(tcon->dev, "EBC RDATA_FIFO_EMPTY irq\n");
	}

	if (intr_status & DSP_END_INT) {
		tcon_update_bits(tcon, EBC_INT_STATUS, DSP_END_INT_CLR, DSP_END_INT_CLR);
		if (tcon->dsp_end_callback)
			tcon->dsp_end_callback();
	}

	if (intr_status & LINE_FLAG_INT) {
		tcon_update_bits(tcon, EBC_INT_STATUS, LINE_FLAG_INT_CLR, LINE_FLAG_INT_CLR);
		if (tcon->line_flag_callback)
			tcon->line_flag_callback();
	}

	return IRQ_HANDLED;
}

static void tcon_set_line_flag_event(struct ebc_tcon *tcon, u32 line, bool enable)
{
	tcon_write(tcon, EBC_VNUM, line);

	if (enable)
		tcon_update_bits(tcon, EBC_INT_STATUS, LINE_FLAG_INT_MASK, 0);
	else
		tcon_update_bits(tcon, EBC_INT_STATUS, LINE_FLAG_INT_MASK, LINE_FLAG_INT_MASK);
}

/*
 * The rk3572 ebc setting clk rule.
 * The dclk_ebc can select dclk_ebc_frac_src, use digital decimal divider,
 * the recommended frequency is less than 60M.
 * The dclk_ebc can select dclk_ebc_int_src.
 * The dclk_ebc_int_src can select gpll or cpll, can only choose the nearest
 * frequency division(gpll:1188M,cpll:1000M),
 * and can't support accurate frequency setting.
 * The dclk_ebc_int_src can select vpll, the vpll is ebc exclusive.
 *
 */
static int rk3572_ebc_dclk_set_rate(struct clk *dclk, unsigned long rate)
{
	struct clk_hw *hw;
	struct clk_hw *p_hw;
	unsigned long pll_rate;
	const char *name;
	int div = 0;

	hw = __clk_get_hw(dclk);
	if (!hw)
		return -EINVAL;

	p_hw = clk_hw_get_parent(hw);
	if (!p_hw)
		return -EINVAL;

	name = clk_hw_get_name(p_hw);
	if (!strcmp(name, "dclk_ebc_frac_src")) {
		clk_set_rate(p_hw->clk, rate);
		clk_set_rate(dclk, rate);
	} else {
		p_hw = clk_hw_get_parent(p_hw);
		if (!p_hw)
			return -EINVAL;
		p_hw = clk_hw_get_parent(p_hw);
		if (!p_hw)
			return -EINVAL;

		name = clk_hw_get_name(p_hw);
		if (!strcmp(name, "vpll")) {
			pll_rate = clk_hw_get_rate(p_hw);
			if (pll_rate >= EBC_PLL_LIMIT_FREQ && pll_rate % rate == 0) {
				clk_set_rate(dclk, rate);
			} else {
				div = DIV_ROUND_UP(EBC_PLL_LIMIT_FREQ, rate);
				if (div % 2)
					div += 1;
				clk_set_rate(p_hw->clk, rate * div);
				clk_set_rate(dclk, rate);
			}
		} else {
			clk_set_rate(dclk, rate);
		}
	}

	return 0;
}

static int tcon_get_version(struct ebc_tcon *tcon)
{
	return tcon->version;
}

static bool tcon_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case EBC_DSP_START:
	case EBC_EPD_CTRL:
	case EBC_DSP_CTRL:
		return false;
	}

	return true;
}

static bool rk3576_tcon_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RK3576_EBC_DSP_START:
	case RK3576_EBC_EPD_CTRL:
	case RK3576_EBC_DSP_CTRL:
	case RK3576_EBC_DSP_CTRL2:
	case RK3576_EBC_DSP_CTRL3:
	case RK3576_EBC_WIN_CTRL:
		return false;
	}

	return true;
}

static struct regmap_config ebc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = tcon_is_volatile_reg,
};

static int ebc_tcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rockchip_ebc_tcon_data *data;
	struct ebc_tcon *tcon;
	struct resource *res;
	int ret;

	data = (struct rockchip_ebc_tcon_data *)of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	tcon = &data->tcon;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tcon->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(tcon->regs))
		return PTR_ERR(tcon->regs);

	tcon->len = resource_size(res);
	ebc_regmap_config.max_register = resource_size(res) - 4;
	ebc_regmap_config.name = "rockchip,ebc_tcon";
	ebc_regmap_config.volatile_reg = data->volatile_reg;
	tcon->regmap_base = devm_regmap_init_mmio(dev, tcon->regs, &ebc_regmap_config);
	if (IS_ERR(tcon->regmap_base))
		return PTR_ERR(tcon->regmap_base);

	tcon->hclk = devm_clk_get(dev, "hclk");
	if (IS_ERR(tcon->hclk)) {
		ret = PTR_ERR(tcon->hclk);
		dev_err(dev, "failed to get hclk: %d\n", ret);
		return ret;
	}

	tcon->dclk = devm_clk_get(dev, "dclk");
	if (IS_ERR(tcon->dclk)) {
		ret = PTR_ERR(tcon->dclk);
		dev_err(dev, "failed to get dclk: %d\n", ret);
		return ret;
	}

	tcon->aclk = devm_clk_get_optional(dev, "aclk");
	if (IS_ERR(tcon->aclk)) {
		ret = PTR_ERR(tcon->aclk);
		dev_err(dev, "failed to get aclk: %d\n", ret);
		return ret;
	}

	tcon->irq = platform_get_irq(pdev, 0);
	if (tcon->irq < 0) {
		dev_err(dev, "No IRQ resource!\n");
		return tcon->irq;
	}

	irq_set_status_flags(tcon->irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, tcon->irq, tcon_irq_hanlder,
			       0, dev_name(dev), tcon);
	if (ret < 0) {
		dev_err(dev, "failed to requeset irq: %d\n", ret);
		return ret;
	}

	tcon->dev = dev;
	/* set lut mode as default display mode */
	tcon->display_mode = LUT_MODE;
	platform_set_drvdata(pdev, tcon);

	pm_runtime_enable(dev);

	return 0;
}

static int ebc_tcon_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static struct rockchip_ebc_tcon_data rk3568_ebc_data = {
	.volatile_reg = tcon_is_volatile_reg,
	.tcon = {
		.version = EBC_VERSION_RK3568,
		.lut_offset = EBC_LUT_DATA_ADDR,
		.enable = tcon_enable,
		.disable = tcon_disable,
		.dsp_mode_set = tcon_dsp_mode_set,
		.image_addr_set = tcon_image_addr_set,
		.frame_addr_set = tcon_frame_addr_set,
		.lut_data_set = tcon_lut_data_set,
		.frame_start = tcon_frame_start,
		.set_line_flag_event = tcon_set_line_flag_event,
		.get_version = tcon_get_version,
		.clk_set_rate = clk_set_rate,
	},
};

static struct rockchip_ebc_tcon_data rk3576_ebc_data = {
	.volatile_reg = rk3576_tcon_is_volatile_reg,
	.tcon = {
		.version = EBC_VERSION_RK3576,
		.lut_offset = EBC_LUT_DATA_ADDR,
		.enable = rk3576_tcon_enable,
		.disable = rk3576_tcon_disable,
		.dsp_mode_set = rk3576_tcon_dsp_mode_set,
		.image_addr_set = rk3576_tcon_image_addr_set,
		.frame_addr_set = rk3576_tcon_frame_addr_set,
		.lut_data_set = tcon_lut_data_set,
		.frame_start = rk3576_tcon_frame_start,
		.data_format_set = rk3576_tcon_data_format_set,
		.set_line_flag_event = tcon_set_line_flag_event,
		.get_version = tcon_get_version,
		.clk_set_rate = clk_set_rate,
	},
};

static struct rockchip_ebc_tcon_data rk3572_ebc_data = {
	.volatile_reg = rk3576_tcon_is_volatile_reg,
	.tcon = {
		.version = EBC_VERSION_RK3572,
		.lut_offset = RK3572_LUT_DATA_ADDR,
		.enable = rk3576_tcon_enable,
		.disable = rk3576_tcon_disable,
		.dsp_mode_set = rk3572_tcon_dsp_mode_set,
		.image_addr_set = rk3576_tcon_image_addr_set,
		.frame_addr_set = rk3576_tcon_frame_addr_set,
		.lut_data_set = tcon_lut_data_set,
		.frame_start = rk3576_tcon_frame_start,
		.data_format_set = rk3576_tcon_data_format_set,
		.set_line_flag_event = tcon_set_line_flag_event,
		.get_version = tcon_get_version,
		.clk_set_rate = rk3572_ebc_dclk_set_rate,
	},
};

static const struct of_device_id ebc_tcon_of_match[] = {
	{ .compatible = "rockchip,rk3568-ebc-tcon", .data = &rk3568_ebc_data },
	{ .compatible = "rockchip,rk3576-ebc-tcon", .data = &rk3576_ebc_data },
	{ .compatible = "rockchip,rk3572-ebc-tcon", .data = &rk3572_ebc_data },
	{}
};
MODULE_DEVICE_TABLE(of, ebc_tcon_of_match);

static struct platform_driver ebc_tcon_driver = {
	.driver = {
		.name = "rk-ebc-tcon",
		.of_match_table = ebc_tcon_of_match,
	},
	.probe = ebc_tcon_probe,
	.remove = ebc_tcon_remove,
};
module_platform_driver(ebc_tcon_driver);

MODULE_AUTHOR("Zorro Liu <zorro.liu@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP EBC tcon driver");
MODULE_LICENSE("GPL v2");
