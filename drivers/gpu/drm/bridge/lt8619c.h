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
 */

#ifndef __LT8619C_H
#define __LT8619C_H

#undef USE_EXTERNAL_HDCPKEY
#undef DDR_CLK

struct video_timing {
	uint32_t pixel_clk;
	uint16_t hfp;
	uint16_t hs;
	uint16_t hbp;
	uint16_t hact;
	uint16_t htotal;
	uint16_t vfp;
	uint16_t vs;
	uint16_t vbp;
	uint16_t vact;
	uint16_t vtotal;
};

enum LT8619C_OUTPUTMODE_ENUM {
	OUTPUT_RGB888 = 0,
	OUTPUT_RGB666,
	OUTPUT_RGB565,
	OUTPUT_YCBCR444,
	OUTPUT_YCBCR422_16BIT,
	OUTPUT_YCBCR422_20BIT,
	OUTPUT_YCBCR422_24BIT,
	OUTPUT_BT656_8BIT,
	OUTPUT_BT656_10BIT,
	OUTPUT_BT656_12BIT,
	OUTPUT_BT1120_16BIT,
	OUTPUT_BT1120_20BIT,
	OUTPUT_BT1120_24BIT,
	OUTPUT_LVDS_2_PORT,
	OUTPUT_LVDS_1_PORT
};

#define LT8619C_OUTPUTMODE  OUTPUT_RGB888

enum LT8619C_AUDIOINPUT_MODE {
	I2S_2CH = 0,
	SPDIF
};

#define Audio_Input_Mode I2S_2CH

enum {
	COLOR_RGB = 0x00,
	COLOR_YCBCR444 = 0x40,
	COLOR_YCBCR422 = 0x20
};

#define LT8619C_OUTPUTCOLOR COLOR_RGB

enum {
	NO_DATA = 0x00,
	ITU_601 = 0x40,
	ITU_709 = 0x80,
	EXTENDED_COLORIETRY = 0xc0
};

enum {
	xvYCC601 = 0x00,
	xvYCC709 = 0x10
	//FUTURE_COLORIETRY
};

enum {
	DEFAULT_RANGE = 0x00,
	LIMIT_RANGE = 0x04,
	FULL_RANGE  = 0x08,
	RESERVED_VAL = 0xc0
};

struct _LT8619C_RXStatus {
	bool flag_RXClkStable;
	bool flag_RXClkDetected;
	bool flag_RXPLLLocked;
	bool Flag_HsyncStable;
	bool input_hdmimode;
	uint8_t input_vic;
	uint8_t input_colorspace;
	uint8_t input_colordepth;
	uint8_t input_colorimetry;
	uint8_t input_ex_colorimetry;
	uint8_t input_QuantRange;
	uint8_t input_PRfactor;
	uint8_t input_videoindex;
	uint32_t ClkFreqValCurrent;
	uint32_t ClkFreqValPrevious;
};
#endif /* __LT8619C_H */
