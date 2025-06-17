/*
 * LT8619C HDMI to LVDS converter + TFT-LCD 080-QC4011
 *
 * Copyright (C) 2020 Hardkernel Co., Ltd.
 *
 * Author : Joy Cho <joy.cho@hardkernel.com>
 *
 * The base initial code of this driver is from Lotium semiconductor
 * and output signal of LT8619C is fixed as following.
 *    - resolution 1024x768
 *    - 1 port LVDS
 *    - 8bit, DE timing mode
 *
 * FIXME for copyright notification and driver license level
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/module.h>

#include <drm/drm_panel.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_bridge.h>

#include "lt8619c.h"

static struct _LT8619C_RXStatus RXStat, *pRXStat;

static uint16_t h_active, v_active;
static uint16_t h_syncwidth, v_syncwidth;
static uint16_t h_bkporch, v_bkporch;
static uint16_t h_total, v_total;
static uint8_t h_syncpol, v_syncpol;
static uint32_t frame_counter;

/* onchip EDID */
uint8_t onchip_edid[256] = {
	/* 1024*768 */
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
	0x21, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x20, 0x01, 0x03, 0x80, 0x10, 0x09, 0x78,
	0xea, 0x5e, 0xc0, 0xa4, 0x59, 0x4a, 0x98, 0x25,
	0x20, 0x50, 0x54, 0x00, 0x08, 0x00, 0x61, 0x40,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x64, 0x19,
	0x00, 0x40, 0x41, 0x00, 0x26, 0x30, 0x08, 0x90,
	0x36, 0x00, 0x63, 0x0a, 0x11, 0x00, 0x00, 0x18,
	0x00, 0x00, 0x00, 0xff, 0x00, 0x4c, 0x69, 0x6e,
	0x75, 0x78, 0x20, 0x23, 0x30, 0x0a, 0x20, 0x20,
	0x20, 0x20, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x3b,
	0x3d, 0x2f, 0x31, 0x07, 0x00, 0x0a, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc,
	0x00, 0x48, 0x4b, 0x5f, 0x56, 0x55, 0x37, 0x43,
	0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x6a,

	0x02, 0x03, 0x12, 0xf1, 0x23, 0x09, 0x07, 0x07,
	0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c, 0x00,
	0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb6
};

/* lcd timing */
struct video_timing lcd_timing = {
	/* pix_clk/kHz, hfp, hs, hbp,hact,htotal,vfp, vs, vbp,vact, vtotal */
	/* 1024x600p60 */
	51200, 8, 144, 168, 1024, 1344, 3, 6, 26, 600, 635
};

struct lt8619c {
	struct device *dev;
	struct regmap *regmap;

	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_panel *panel;

	struct i2c_client *client;
	struct gpio_desc *gpiod_reset;
};

static struct lt8619c *lt8619c;

static struct regmap_config lt8619c_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.name		= "lt8619c",
};

#define LT8619C_ADDR	0x32 /* 0x64 >> 1 */

static uint8_t lt8619c_reg_read(char reg)
{
	uint8_t out_buf[2];
	uint8_t in_buf[2];
	uint8_t val;

	struct i2c_msg msgs[] = {
		{
			.addr = LT8619C_ADDR,
			.flags = 0,
			.len = 1,
			.buf = out_buf,
		},
		{
			.addr = LT8619C_ADDR,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = in_buf,
		}
	};

	out_buf[0] = reg;
	out_buf[1] = 0;

	if (i2c_transfer(lt8619c->client->adapter, msgs, 2) == 2) {
		val = in_buf[0];
	} else {
		dev_info(lt8619c->dev, "i2c 0x%02x 0x%02x read failed\n",
			  reg, val);
	}

	return val;
}

static int lt8619c_reg_write(uint8_t reg, uint8_t data)
{
	uint8_t outbuf[2];
	struct i2c_msg msg = {
		.addr = LT8619C_ADDR,
		.flags = 0,
		.len = 2,
		.buf = outbuf,
	};

	outbuf[0] = reg;
	outbuf[1] = data;

	if (i2c_transfer(lt8619c->client->adapter, &msg, 1) != 1)
		dev_info(lt8619c->dev, "i2c 0x%02x 0x%02x write failed\n",
			  reg, data);

	return 0;
}

static int lt8619c_reg_write_bytes(uint8_t reg, uint8_t *data, uint16_t length)
{
	//- uint8_t outbuf[length + 1];
	uint8_t outbuf[256 + 1];
	struct i2c_msg msg = {
		.addr = LT8619C_ADDR,
		.flags = 0,
		.len = length + 1,
		.buf = outbuf,
	};

	outbuf[0] = reg;
	memcpy(outbuf + 1, data, length);

	if (i2c_transfer(lt8619c->client->adapter, &msg, 1) != 1)
		dev_info(lt8619c->dev, "i2c write failed\n");

	return 0;
}

static bool lt8619c_check_chipid(void)
{
	unsigned int chip_id[3];

	lt8619c_reg_write(0xff, 0x60);

	chip_id[0] = lt8619c_reg_read(0x00);
	chip_id[1] = lt8619c_reg_read(0x01);
	chip_id[2] = lt8619c_reg_read(0x02);
	dev_dbg(lt8619c->dev, "Read Chip : 0x%x, 0x%x, 0x%x\n",
			chip_id[0], chip_id[1], chip_id[2]);

	if ((chip_id[0] == 0x16) && chip_id[1] == 0x04)
		return true;
	else
		return false;
}

static void lt8619c_setHPD(uint8_t level)
{
	lt8619c_reg_write(0xFF, 0x80);

	if (level)
		lt8619c_reg_write(0x06, (lt8619c_reg_read(0x06)|0x08));
	else
		lt8619c_reg_write(0x06, (lt8619c_reg_read(0x06)&0xF7));
}

static void lt8619c_edid_calc(uint8_t *pbuf, struct video_timing *timing)
{
	uint16_t hblanking = 0;
	uint16_t vblanking = 0;
	uint16_t pixel_clk;

	pixel_clk = timing->pixel_clk / 10;

	if (pbuf == NULL || timing == NULL)
		return;

	hblanking = timing->hfp + timing->hs + timing->hbp; /* H blanking */
	vblanking = timing->vfp + timing->vs + timing->vbp; /* V blanking */

	pbuf[0] = pixel_clk % 256;
	pbuf[1] = pixel_clk / 256;
	pbuf[2] = timing->hact % 256;
	pbuf[3] = hblanking % 256;
	pbuf[4] = ((timing->hact / 256) << 4) + hblanking / 256;
	pbuf[5] = timing->vact % 256;
	pbuf[6] = vblanking % 256;
	pbuf[7] = ((timing->vact / 256) << 4) + vblanking / 256;
	pbuf[8] = timing->hfp % 256;
	pbuf[9] = timing->hs % 256;
	pbuf[10] = ((timing->vfp % 256) << 4) + timing->vs % 256;
	pbuf[11] = ((timing->hfp/256) << 6) + ((timing->hs / 256) << 4)+
		((timing->vfp / 256) << 2) + (timing->vs / 256);
	pbuf[17] = 0x1c; /* progress negative vsync, negative hsync */
}

static uint8_t lt8619c_edid_checksum(uint8_t block, uint8_t *buf)
{
	uint8_t i = 0;
	uint8_t checksum = 0;
	uint8_t *pbuf = buf + 128 * block;

	//- if (pbuf == NULL || (pbuf + 127) == NULL)
		//- return 0;

	for (i = 0, checksum = 0; i < 127 ; i++) {
		checksum += pbuf[i];
		checksum %= 256;
	}
	checksum = 256 - checksum;

	dev_info(lt8619c->dev, "EDID checksum 0x%x\n", checksum);

	return checksum;
}

static void lt8619c_set_edid(uint8_t *edid_buf)
{
	int ret = 0;

	lt8619c_reg_write(0xFF, 0x80);
	lt8619c_reg_write(0x8E, 0x07);
	lt8619c_reg_write(0x8F, 0x00);

	if (edid_buf == NULL)
		ret = lt8619c_reg_write_bytes(0x90, &onchip_edid[0], 256);
	else
		ret = lt8619c_reg_write_bytes(0x90, edid_buf, 256);

	if (ret)
		dev_info(lt8619c->dev, "EDID Set write error\n");

	lt8619c_reg_write(0x8E, 0x02);
}

static void lt8619c_rx_init(void)
{
	lt8619c_reg_write(0xFF, 0x80);
	/* RGD_CLK_STABLE_OPT[1:0] */
	lt8619c_reg_write(0x2c, (lt8619c_reg_read(0x2C)|0x30));

	lt8619c_reg_write(0xFF, 0x60);
	lt8619c_reg_write(0x04, 0xF2);
	lt8619c_reg_write(0x83, 0x3F);
	lt8619c_reg_write(0x80, 0x08); /* use xtal_clk as sys_clk */
#ifdef DDR_CLK
	lt8619c_reg_write(0xa4, 0x14); /* 0x10:SDR clk,0x14: DDR clk */
#else
	lt8619c_reg_write(0xa4, 0x10); /* 0x10:SDR clk,0x14: DDR clk */
#endif
	dev_dbg(lt8619c->dev, "LT8619C output mode : %d\n", LT8619C_OUTPUTMODE);

	dev_info(lt8619c->dev, "\nLT8619C set to OUTPUT_RGB888");
	lt8619c_reg_write(0xFF, 0x60);
	lt8619c_reg_write(0x07, 0xff);
	lt8619c_reg_write(0xa8, 0x0f);
	lt8619c_reg_write(0x60, 0x00);
	lt8619c_reg_write(0x96, 0x71);
	lt8619c_reg_write(0xa0, 0x50);
	// 0x60A3=0x44:Phase adjust enable;0x60A3=0x30:PIN68
	// switch to output PCLK(only use for U5);
	lt8619c_reg_write(0xa3, 0x74);
	//Phase code value: 0x20,0x28,0x21,0x29,0x22,0x2a,0x23,0x2b,0x24,0x2c
	lt8619c_reg_write(0xa2, 0x29);
	//RGB mapping control
	lt8619c_reg_write(0x6d, 0x00);//0x07//00
	//RGB high/low bit swap control
	lt8619c_reg_write(0x6e, 0x00);
	/* LT8619C_OUTPUTMODE == OUTPUT_RGB888 */

	lt8619c_reg_write(0xff, 0x60);
	lt8619c_reg_write(0x0e, 0xfd);
	lt8619c_reg_write(0x0e, 0xff);
	lt8619c_reg_write(0x0d, 0xfc);
	lt8619c_reg_write(0x0d, 0xff);
}

void lt8619c_audio_init(uint8_t audio_input)
{
	if (audio_input == I2S_2CH) {
		dev_dbg(lt8619c->dev, "Audio Init : Audio set to I2S_2CH\n");
		lt8619c_reg_write(0xff, 0x60);
		lt8619c_reg_write(0x4c, 0x00);
	} else if (audio_input == SPDIF) {
		dev_dbg(lt8619c->dev, "Audio Init : Audio set to SPDIF\n");
		lt8619c_reg_write(0xff, 0x60);
		lt8619c_reg_write(0x4c, 0x80);
	} else {
		dev_dbg(lt8619c->dev, "Audio Init : Error! Check Audio Mode\n");
	}

	lt8619c_reg_write(0xff, 0x80);
	lt8619c_reg_write(0x5d, 0xc9);
	lt8619c_reg_write(0x07, 0x16);
	lt8619c_reg_write(0x08, 0x80);
}

static void lt8619c_poweron(void)
{
	/* set HPD as low */
	lt8619c_setHPD(0);

	/* calculate timing & set edid */
	lt8619c_edid_calc(onchip_edid + 0x36, &lcd_timing);
	onchip_edid[127] = lt8619c_edid_checksum(0, &onchip_edid[0]);
	onchip_edid[255] = lt8619c_edid_checksum(1, &onchip_edid[0]);
	lt8619c_set_edid(onchip_edid);

	mdelay(100);

	/* set HPD as low */
	lt8619c_setHPD(1);

	/* rx init */
	lt8619c_rx_init();

	/* audio init */
	lt8619c_audio_init(I2S_2CH);

	mdelay(1);
}

static void lt8619c_rx_reset(void)
{
	lt8619c_reg_write(0xFF, 0x60);
	lt8619c_reg_write(0x0E, 0xBF); /* reset RXPLL */
	lt8619c_reg_write(0x09, 0xFD); /* reset RXPLL Lock det */
	mdelay(5);

	lt8619c_reg_write(0x0E, 0xFF); /* release RXPLL */
	lt8619c_reg_write(0x09, 0xFF);

	lt8619c_reg_write(0xFF, 0x60);
	lt8619c_reg_write(0x0e, 0xC7); /* reset PI */
	lt8619c_reg_write(0x09, 0x0F); /* reset RX,CDR */
	mdelay(10);

	lt8619c_reg_write(0x0e, 0xFF); /* release PI */
	mdelay(10);
	lt8619c_reg_write(0x09, 0x8F); /* release RX */
	mdelay(10);
	lt8619c_reg_write(0x09, 0xFF); /* release CDR */
	mdelay(50);
}

static bool lt8619c_detect_clk(void)
{
	uint8_t read_data;
	bool ret;

	ret = true;
	lt8619c_reg_write(0xFF, 0x80);
	if (lt8619c_reg_read(0x44) & 0x08) {
		if (!pRXStat->flag_RXClkStable) {
			pRXStat->flag_RXClkStable =
					!pRXStat->flag_RXClkStable;
			dev_dbg(lt8619c->dev, "[detect_clk] rx clock stable %d\n",
					pRXStat->flag_RXClkStable);
			lt8619c_reg_write(0xFF, 0x60);
			read_data = lt8619c_reg_read(0x97);
			lt8619c_reg_write(0x97, (read_data & 0x3f));
			lt8619c_reg_write(0xFF, 0x80);
			lt8619c_reg_write(0x1b, 0x00);

			lt8619c_rx_reset();
			mdelay(5);

			lt8619c_reg_write(0xFF, 0x80);
			read_data = (lt8619c_reg_read(0x87) & 0x10);
			if (read_data) {
				pRXStat->flag_RXPLLLocked = true;
				dev_dbg(lt8619c->dev, "[detect_clk] clock detected & pll lock ok\n");
				ret = true;
			} else {
				pRXStat->flag_RXPLLLocked = false;
				memset(pRXStat, 0, sizeof(RXStat));
				dev_dbg(lt8619c->dev, "[detect_clk] clock detected & pll unlock\n");
				ret = false;
			}
		} else {
			lt8619c_reg_write(0xFF, 0x80);
			read_data = lt8619c_reg_read(0x87) & 0x10;

			if (read_data) {
				pRXStat->flag_RXPLLLocked = true;
				dev_dbg(lt8619c->dev, "[detect_clk] pll lock ok\n");
				ret = true;
			} else {
				pRXStat->flag_RXPLLLocked = false;
				memset(pRXStat, 0, sizeof(RXStat));
				dev_dbg(lt8619c->dev, "[detect_clk] pll unlock\n");
				ret = false;
			}
		}
	} else {
		if (pRXStat->flag_RXClkStable)
			dev_dbg(lt8619c->dev, "[detect_clk] lt8619c clock disappeared\n");
		memset(pRXStat, 0, sizeof(RXStat));
		ret = false;
	}
	return ret;
}

static void lt8619c_input_info(void)
{
	uint8_t loop_num, read_data;

	lt8619c_reg_write(0xFF, 0x80);

	if (pRXStat->flag_RXClkStable && pRXStat->flag_RXPLLLocked) {
		dev_dbg(lt8619c->dev, "[input info] check hsync start:\n");

		if (lt8619c_reg_read(0x13) & 0x01) {
			if (!pRXStat->Flag_HsyncStable) {
				pRXStat->Flag_HsyncStable = true;
				for (loop_num = 0; loop_num < 8; loop_num++) {
					dev_dbg(lt8619c->dev,
						"[input info] check hsync : %d\n",
						loop_num);
					mdelay(20);

					if (!(lt8619c_reg_read(0x13) & 0x01)) {
						dev_dbg(lt8619c->dev, "[input info] lt8619c 8013[0]=0\n");
						pRXStat->Flag_HsyncStable
									= false;
						dev_dbg(lt8619c->dev, "[input info] hsync stable FAIL\n");
						break;
					}
				}

				if (pRXStat->Flag_HsyncStable) {
					lt8619c_reg_write(0xFF, 0x60);
					read_data = lt8619c_reg_read(0x0D);
					 /* reset LVDS/BT fifo */
					lt8619c_reg_write(0x0D,
							read_data & 0xf8);
					lt8619c_reg_write(0x0D,
							read_data | 0x06);
					lt8619c_reg_write(0x0D,
							read_data | 0x01);
					dev_dbg(lt8619c->dev, "[input info] hsync stable ok\n");
				}
			}
		} else {
			if (pRXStat->Flag_HsyncStable)
				dev_dbg(lt8619c->dev, "[input info] hsync stable -> unstable\n");

			pRXStat->Flag_HsyncStable = false;
			dev_dbg(lt8619c->dev, "[input info] hsync keep unstable\n");
		//memset(pRXStat, 0, sizeof(_RXStat));
		}
	}

	if (pRXStat->Flag_HsyncStable) {
		read_data = lt8619c_reg_read(0x13);
		pRXStat->input_hdmimode =
				(read_data & 0x02)?(true):(false);
		if (pRXStat->input_hdmimode) {
			pRXStat->input_vic = lt8619c_reg_read(0x74) & 0x7f;
			pRXStat->input_colorspace =
					lt8619c_reg_read(0x71) & 0x60;
			pRXStat->input_colordepth =
					lt8619c_reg_read(0x16) & 0xf0;
			pRXStat->input_colorimetry =
					lt8619c_reg_read(0x72) & 0xc0;
			pRXStat->input_ex_colorimetry =
					lt8619c_reg_read(0x73) & 0x70;
			pRXStat->input_QuantRange =
					lt8619c_reg_read(0x73) & 0x0c;
			pRXStat->input_PRfactor =
					lt8619c_reg_read(0x75) & 0x0f;

			if (pRXStat->input_PRfactor == 1) {
				lt8619c_reg_write(0xFF, 0x60);
				read_data = lt8619c_reg_read(0x97);
				lt8619c_reg_write(0x97, read_data | 0x40);
				lt8619c_reg_write(0xFF, 0x80);
				lt8619c_reg_write(0x1b, 0x20);
			} else if (pRXStat->input_PRfactor == 3) {
				lt8619c_reg_write(0xFF, 0x60);
				read_data = lt8619c_reg_read(0x97);
				lt8619c_reg_write(0x97, read_data | 0x80);
				lt8619c_reg_write(0xFF, 0x80);
				lt8619c_reg_write(0x1b, 0x60);
			} else {
				lt8619c_reg_write(0xFF, 0x60);
				read_data = lt8619c_reg_read(0x97);
				lt8619c_reg_write(0x97, read_data & 0x3f);
				lt8619c_reg_write(0xFF, 0x80);
				lt8619c_reg_write(0x1b, 0x00);
			}
		} else {
			pRXStat->input_vic = 0;
			pRXStat->input_colorspace = COLOR_RGB;
			pRXStat->input_colordepth = 0;
			pRXStat->input_colorimetry = ITU_709;
			pRXStat->input_ex_colorimetry = 0;
			pRXStat->input_QuantRange = FULL_RANGE;
			pRXStat->input_PRfactor = 0;
			lt8619c_reg_write(0xFF, 0x60);
			read_data = lt8619c_reg_read(0x97);
			lt8619c_reg_write(0x97, read_data & 0x3f);
			lt8619c_reg_write(0xFF, 0x80);
			lt8619c_reg_write(0x1b, 0x00);
		}
	}
}

static void lt8619c_csc_conversion(void)
{
	dev_dbg(lt8619c->dev, "[csc conv] output color 0x%x\n",
						LT8619C_OUTPUTCOLOR);

	lt8619c_reg_write(0xFF, 0x60);
	lt8619c_reg_write(0x07, 0xfe);

	if (LT8619C_OUTPUTCOLOR == COLOR_RGB) {
		if (pRXStat->input_colorspace == COLOR_RGB) {
			lt8619c_reg_write(0x52, 0x00);
			if (pRXStat->input_QuantRange == LIMIT_RANGE)
				lt8619c_reg_write(0x53, 0x08);
			else
				lt8619c_reg_write(0x53, 0x00);
		} else {
			if (pRXStat->input_colorspace == COLOR_YCBCR422)
				lt8619c_reg_write(0x52, 0x01);
			else
				lt8619c_reg_write(0x52, 0x00);

			if (pRXStat->input_QuantRange == LIMIT_RANGE) {
				if (pRXStat->input_colorimetry == ITU_601)
					lt8619c_reg_write(0x53, 0x50);
				else if (pRXStat->input_colorimetry == ITU_709)
					lt8619c_reg_write(0x53, 0x70);
				else  /* NO_DATA or EXTENDED_COLORIETRY */
					lt8619c_reg_write(0x53, 0x70);
			} else if (pRXStat->input_QuantRange == FULL_RANGE) {
				if (pRXStat->input_colorimetry == ITU_601)
					lt8619c_reg_write(0x53, 0x40);
				else if (pRXStat->input_colorimetry == ITU_709)
					lt8619c_reg_write(0x53, 0x60);
				else /* NO_DATA or EXTENDED_COLORIETRY */
					lt8619c_reg_write(0x53, 0x60);
			} else {  /* DEFAULT_RANGE or RESERVED_VAL */
				lt8619c_reg_write(0x53, 0x60);
			}
		}
	} else if (LT8619C_OUTPUTCOLOR == COLOR_YCBCR444) {
		if (pRXStat->input_colorspace == COLOR_RGB) {
			lt8619c_reg_write(0x53, 0x00);
			if (pRXStat->input_QuantRange == LIMIT_RANGE) {
				if (pRXStat->input_colorimetry == ITU_601)
					lt8619c_reg_write(0x52, 0x08);
				else if (pRXStat->input_colorimetry == ITU_709)
					lt8619c_reg_write(0x52, 0x28);
				else  /* NO_DATA or EXTENDED_COLORIETRY */
					lt8619c_reg_write(0x52, 0x28);
			} else if (pRXStat->input_QuantRange == FULL_RANGE) {
				if (pRXStat->input_colorimetry == ITU_601)
					lt8619c_reg_write(0x52, 0x18);
				else if (pRXStat->input_colorimetry == ITU_709)
					lt8619c_reg_write(0x52, 0x38);
				else  /* NO_DATA or EXTENDED_COLORIETRY */
					lt8619c_reg_write(0x52, 0x38);
			} else {  /* DEFAULT_RANGE or RESERVED_VAL */
				lt8619c_reg_write(0x52, 0x38);
			}
		} else if (pRXStat->input_colorspace == COLOR_YCBCR444) {
			lt8619c_reg_write(0x52, 0x00);
			lt8619c_reg_write(0x53, 0x00);
		} else if (pRXStat->input_colorspace == COLOR_YCBCR422) {
			lt8619c_reg_write(0x52, 0x01);
			lt8619c_reg_write(0x53, 0x00);
		}
	} else if (LT8619C_OUTPUTCOLOR == COLOR_YCBCR422) {
		if (pRXStat->input_colorspace == COLOR_RGB) {
			lt8619c_reg_write(0x53, 0x00);

			if (pRXStat->input_QuantRange == LIMIT_RANGE) {
				if (pRXStat->input_colorimetry == ITU_601)
					lt8619c_reg_write(0x52, 0x0a);
				else if (pRXStat->input_colorimetry == ITU_709)
					lt8619c_reg_write(0x52, 0x2a);
				else  /* NO_DATA or EXTENDED_COLORIETRY */
					lt8619c_reg_write(0x52, 0x2a);
			} else if (pRXStat->input_QuantRange ==	FULL_RANGE) {
				if (pRXStat->input_colorimetry == ITU_601)
					lt8619c_reg_write(0x52, 0x1a);
				else if (pRXStat->input_colorimetry == ITU_709)
					lt8619c_reg_write(0x52, 0x3a);
				else  /* NO_DATA or EXTENDED_COLORIETRY */
					lt8619c_reg_write(0x52, 0x3a);
			} else {  /* DEFAULT_RANGE or RESERVED_VAL */
				lt8619c_reg_write(0x52, 0x3a);
			}
		} else if (pRXStat->input_colorspace ==	COLOR_YCBCR444) {
			lt8619c_reg_write(0x52, 0x02);
			lt8619c_reg_write(0x53, 0x00);
		} else if (pRXStat->input_colorspace == COLOR_YCBCR422) {
			lt8619c_reg_write(0x52, 0x00);
			lt8619c_reg_write(0x53, 0x00);
		}
	}
}

static void lt8619c_check_video(void)
{
	uint8_t tmp_read;

	if (!pRXStat->Flag_HsyncStable) {
		h_total = 0;
		v_total = 0;
		return;
	}

	lt8619c_reg_write(0xFF, 0x60);

	h_active = ((uint16_t)lt8619c_reg_read(0x22)) << 8;
	h_active += lt8619c_reg_read(0x23);
	v_active = ((uint16_t)(lt8619c_reg_read(0x20) & 0x0f)) << 8;
	v_active += lt8619c_reg_read(0x21);

	frame_counter = ((uint32_t)lt8619c_reg_read(0x10)) << 16;
	frame_counter += ((uint32_t)lt8619c_reg_read(0x11)) << 8;
	frame_counter += lt8619c_reg_read(0x12);

	h_syncwidth = ((uint16_t)(lt8619c_reg_read(0x14) & 0x0f)) << 8;
	h_syncwidth += lt8619c_reg_read(0x15);
	v_syncwidth = lt8619c_reg_read(0x13);

	h_bkporch = ((uint16_t)(lt8619c_reg_read(0x18) & 0x0f)) << 8;
	h_bkporch += lt8619c_reg_read(0x19);
	v_bkporch = lt8619c_reg_read(0x16);

	h_total = ((uint16_t)lt8619c_reg_read(0x1e)) << 8;
	h_total += lt8619c_reg_read(0x1f);
	v_total = ((uint16_t)(lt8619c_reg_read(0x1c) & 0x0f)) << 8;
	v_total += lt8619c_reg_read(0x1d);

	tmp_read = lt8619c_reg_read(0x24);
	h_syncpol = tmp_read & 0x01;
	v_syncpol = (tmp_read & 0x02) >> 1;
}

static void detect_lvdspll_lock(void)
{
	uint8_t read_data_1;
	uint8_t check_num = 0;

	lt8619c_reg_write(0xFF, 0x60);

	if ((lt8619c_reg_read(0xA3) & 0x40) == 0x40) {
		lt8619c_reg_write(0xFF, 0x80);
		while ((lt8619c_reg_read(0x87) & 0x20) == 0x00) {
			lt8619c_reg_write(0xFF, 0x60);
			read_data_1 = lt8619c_reg_read(0x0e);
			lt8619c_reg_write(0x0e, read_data_1 & 0xFD);

			mdelay(5);

			lt8619c_reg_write(0x0e, 0xFF);
			lt8619c_reg_write(0xFF, 0x80);

			check_num++;
			if (check_num > 10)
				break;
		}
	}
}

static void lt8619c_bt_setting(void)
{
	uint8_t val_6060;
	uint16_t tmp_data;

	if (!pRXStat->Flag_HsyncStable)
		return;

	detect_lvdspll_lock();

	lt8619c_reg_write(0xFF, 0x60);
	val_6060 = lt8619c_reg_read(0x60) & 0xc7;

	/* set BT TX h/vsync polarity */
	if (h_syncpol)
		val_6060 |= 0x20;

	if (v_syncpol)
		val_6060 |= 0x10;

	/*
	 * double the value of v_active&v_total
	 * when input is interlace resolution.
	 * if user needs to support interlace format not listed here,
	 * please add that interlace format info here.
	 */
	if (pRXStat->input_hdmimode) {
		switch (pRXStat->input_vic) {
		case 5: /* 1080i */
		case 6: /* 480i */
		case 7: /* 480iH */
		case 10: /* 480i4x */
		case 11: /* 480i4xH */
		case 20: /* 1080i25 */
			dev_dbg(lt8619c->dev, "[bt setting] VIC20\n");
			val_6060 |= 0x08;
			v_active <<= 1;

			if (v_total % 2 == 1)
				v_total = (v_total << 1) - 1;
			else
				v_total = (v_total << 1) + 1;

			lt8619c_reg_write(0x68, 23);
			break;
		case 21: /* 576i */
		case 22: /* 576iH */
		case 25: /* 576i4x */
		case 26: /* 576i4xH */
			dev_dbg(lt8619c->dev, "[bt setting] VIC26\n");
			val_6060 |= 0x08;
			v_active <<= 1;

			if (v_total % 2 == 1)
				v_total = (v_total << 1) - 1;
			else
				v_total = (v_total << 1) + 1;

			lt8619c_reg_write(0x68, 25);
			break;
		default:
			lt8619c_reg_write(0x68, 0x00);
			break;
		}
	/* DVI Input */
	} else {
		if ((h_active == 1920) && (v_active == 540)) {
			val_6060 |= 0x08;
			v_active <<= 1;
			if (v_total % 2 == 1)
				v_total = (v_total << 1) - 1;
			else
				v_total = (v_total << 1) + 1;
			lt8619c_reg_write(0x68, 23);
		} else if ((h_active == 1440) && (v_active == 240)) {
			val_6060 |= 0x08;
			v_active <<= 1;
			if (v_total % 2 == 1)
				v_total = (v_total << 1) - 1;
			else
				v_total = (v_total << 1) + 1;
			lt8619c_reg_write(0x68, 23);
		} else if ((h_active == 1440) && (v_active == 288))	{
			val_6060 |= 0x08;
			v_active <<= 1;
			if (v_total % 2 == 1)
				v_total = (v_total << 1) - 1;
			else
				v_total = (v_total << 1) + 1;
			lt8619c_reg_write(0x68, 25);
		}
	}

	lt8619c_reg_write(0x60, val_6060);
	tmp_data = h_syncwidth + h_bkporch;
	lt8619c_reg_write(0x61, (uint8_t)(tmp_data >> 8));
	lt8619c_reg_write(0x62, (uint8_t)tmp_data);
	tmp_data = h_active;
	lt8619c_reg_write(0x63, (uint8_t)(tmp_data >> 8));
	lt8619c_reg_write(0x64, (uint8_t)tmp_data);
	tmp_data = h_total;
	lt8619c_reg_write(0x65, (uint8_t)(tmp_data >> 8));
	lt8619c_reg_write(0x66, (uint8_t)tmp_data);
	tmp_data = v_syncwidth + v_bkporch;
	lt8619c_reg_write(0x67, (uint8_t)tmp_data);
	tmp_data = v_active;
	lt8619c_reg_write(0x69, (uint8_t)(tmp_data >> 8));
	lt8619c_reg_write(0x6a, (uint8_t)tmp_data);
	tmp_data = v_total;
	lt8619c_reg_write(0x6b, (uint8_t)(tmp_data >> 8));
	lt8619c_reg_write(0x6c, (uint8_t)tmp_data);
}

static void lt8619c_print_rxinfo(void)
{
	uint32_t clk = 0;
	static uint16_t print_en;
	uint32_t audio_rate = 0;

	print_en = 0;

	if (!pRXStat->Flag_HsyncStable) {
		print_en = 0;
		return;
	}
	if (print_en != 3)
		print_en++;

	if (print_en == 3) {
		print_en = 0;

		if (pRXStat->input_hdmimode) {
			dev_dbg(lt8619c->dev, "[rxinfo] input is HDMI signal\n");
			dev_dbg(lt8619c->dev, "[rxinfo] input vid %d\n",
				pRXStat->input_vic);

			if (pRXStat->input_colorspace == COLOR_RGB)
				dev_dbg(lt8619c->dev,
					"[rxinfo] input colorspace : COLOR_RGB\n");
			else if (pRXStat->input_colorspace == COLOR_YCBCR444)
				dev_dbg(lt8619c->dev, "[rxinfo] input colorspace : COLOR_YCBCR444\n");
			else if (pRXStat->input_colorspace == COLOR_YCBCR422)
				dev_dbg(lt8619c->dev, "[rxinfo] input colorspace : COLOR_YCBCR422\n");
			else
				dev_dbg(lt8619c->dev, "[rxinfo] input colorspace UNKNOWN\n");

			dev_info(lt8619c->dev, "[rxinfo] input colordepth %d\n",
				pRXStat->input_colordepth);

			if (pRXStat->input_colorimetry == NO_DATA)
				dev_dbg(lt8619c->dev, "[rxinfo] input colorimetry : NO DATA\n");
			else if (pRXStat->input_colorimetry == ITU_601)
				dev_dbg(lt8619c->dev, "[rxinfo] input colorimetry : ITU 601\n");
			else if (pRXStat->input_colorimetry == ITU_709)
				dev_dbg(lt8619c->dev, "[rxinfo] input colorimetry : ITU 709\n");
			else
				dev_dbg(lt8619c->dev, "[rxinfo] input colorimetry : extended colorimetry\n");

			if (pRXStat->input_colorimetry == EXTENDED_COLORIETRY) {
				if (pRXStat->input_ex_colorimetry == xvYCC601)
					dev_dbg(lt8619c->dev, "[rxinfo] input ex colorimetry : xvYCC601\n");
				else if
					(pRXStat->input_ex_colorimetry ==
								xvYCC709)
					dev_dbg(lt8619c->dev,
						"[rxinfo] input ex colorimetry : xvYCC709\n");
				else
					dev_dbg(lt8619c->dev,
						"[rxinfo] input ex colorimetry : FUTURE COLORIMETRY\n");
			}

			if (pRXStat->input_QuantRange == DEFAULT_RANGE)
				dev_dbg(lt8619c->dev,
					"[rxinfo] input quant range : DEFAULT\n");
			else if (pRXStat->input_QuantRange == LIMIT_RANGE)
				dev_dbg(lt8619c->dev,
					"[rxinfo] input quant range : LIMIT\n");
			else if (pRXStat->input_QuantRange == FULL_RANGE)
				dev_dbg(lt8619c->dev,
					"[rxinfo] input quant range : FULL\n");
			else
				dev_dbg(lt8619c->dev,
					"[rxinfo] input quant range : RESERVED\n");

			dev_info(lt8619c->dev, "[rxinfo] input PRfactor %d\n",
				pRXStat->input_PRfactor);
		} else {
			dev_dbg(lt8619c->dev, "[rxinfo] input : DVI signal\n");
		}

		dev_dbg(lt8619c->dev, "[rxinfo] input timing info\n");

		lt8619c_reg_write(0xFF, 0x80);
		clk = lt8619c_reg_read(0x44)&0x07;
		clk <<= 8;
		clk += lt8619c_reg_read(0x45);
		clk <<= 8;
		clk += lt8619c_reg_read(0x46);

		dev_dbg(lt8619c->dev,
			"[rxinfo] TMDS clock freq : %d kHz\n", clk);

		dev_dbg(lt8619c->dev, "[rxinfo] h_active %d\n",
							h_active);
		dev_dbg(lt8619c->dev, "[rxinfo] v_active %d\n",
							v_active);
		dev_dbg(lt8619c->dev, "[rxinfo] h_syncwidth %d\n",
							h_syncwidth);
		dev_dbg(lt8619c->dev, "[rxinfo] v_syncwidth %d\n",
							v_syncwidth);
		dev_dbg(lt8619c->dev, "[rxinfo] h_bkporch %d\n",
							h_bkporch);
		dev_dbg(lt8619c->dev, "[rxinfo] v_bkporch %d\n",
							v_bkporch);
		dev_dbg(lt8619c->dev, "[rxinfo] h_total %d\n", h_total);
		dev_dbg(lt8619c->dev, "[rxinfo] v_total %d\n", v_total);
		dev_dbg(lt8619c->dev, "[rxinfo] frame_counter %d\n",
							frame_counter);

		if (h_syncpol)
			dev_dbg(lt8619c->dev, "[rxinfo] h_syncpol is positive\n");
		else
			dev_dbg(lt8619c->dev, "[rxinfo] h_syncpol is negative\n");

		if (v_syncpol)
			dev_dbg(lt8619c->dev, "[rxinfo] v_syncpol is positive\n");
		else
			dev_dbg(lt8619c->dev, "[rxinfo] v_syncpol is negative\n");

		lt8619c_reg_write(0xFF, 0x80);
		audio_rate = ((lt8619c_reg_read(0x11)&0x03) * 0x100) +
						lt8619c_reg_read(0x12);
		dev_dbg(lt8619c->dev, "[rxinfo] audio rate %d kHz\n",
						audio_rate);
	}
}

static int lt8619c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;

	lt8619c = devm_kzalloc(dev, sizeof(*lt8619c), GFP_KERNEL);
	if (!lt8619c)
		return -ENOMEM;

	lt8619c->dev = dev;
	lt8619c->client = client;

	lt8619c->regmap = devm_regmap_init_i2c(client, &lt8619c_regmap_config);
	if (IS_ERR(lt8619c->regmap))
		return PTR_ERR(lt8619c->regmap);

	/* Reset GPIO */
	lt8619c->gpiod_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lt8619c->gpiod_reset))
		return PTR_ERR(lt8619c->gpiod_reset);

	/* init variable */
	pRXStat = &RXStat;
	memset(pRXStat, 0, sizeof(RXStat));

	/* reset */
	if (lt8619c->gpiod_reset) {
		gpiod_set_value_cansleep(lt8619c->gpiod_reset, 0);
		usleep_range(5000, 10000);
	}

	/* check chip ID */
	if (!lt8619c_check_chipid()) {
		dev_err(dev, "failed to read LT8619C chip id\n");
		return 0;
	}

	lt8619c_poweron();

	if (lt8619c_detect_clk()) {
		lt8619c_input_info();
		lt8619c_csc_conversion();
		lt8619c_check_video();
		lt8619c_bt_setting();
		lt8619c_print_rxinfo();
	}

	dev_info(lt8619c->dev, "LT8619C init done\n");

	return 0;
}

static int lt8619c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id lt8619c_ids[] = {
	{ "lt8619c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt8619c_ids);

static struct i2c_driver lt8619c_driver = {
	.driver = {
		.name = "lt8619c",
	},
	.probe = lt8619c_probe,
	.remove = lt8619c_remove,
	.id_table = lt8619c_ids,
};

static int __init lt8619c_init(void)
{
	return i2c_add_driver(&lt8619c_driver);
}
/* must be loaded after rk817 mfd driver */
late_initcall(lt8619c_init);

static void __exit lt8619c_exit(void)
{
	i2c_del_driver(&lt8619c_driver);
}
module_exit(lt8619c_exit);

MODULE_DESCRIPTION("LT8619C HDMI to LVDS Driver for TFT LCD QC4011");
MODULE_AUTHOR("Joy Cho <joy.cho@hardkernel.com>");
MODULE_LICENSE("GPL");
