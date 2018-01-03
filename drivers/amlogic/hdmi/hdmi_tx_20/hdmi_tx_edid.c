/*
 * drivers/amlogic/hdmi/hdmi_tx_20/hdmi_tx_edid.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
/* #include <mach/register.h> */
/* #include <plat/io.h> */
/* #include "hw/hdmi_tx_reg.h" */
#include <crypto/hash.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
/* #include <mach/am_regs.h> */

#include <linux/amlogic/hdmi_tx/hdmi_info_global.h>
#include <linux/amlogic/hdmi_tx/hdmi_tx_module.h>

#define CEA_DATA_BLOCK_COLLECTION_ADDR_1StP 0x04
#define VIDEO_TAG 0x40
#define AUDIO_TAG 0x20
#define VENDOR_TAG 0x60
#define SPEAKER_TAG 0x80

#define HDMI_EDID_BLOCK_TYPE_RESERVED	        0
#define HDMI_EDID_BLOCK_TYPE_AUDIO		1
#define HDMI_EDID_BLOCK_TYPE_VIDEO		2
#define HDMI_EDID_BLOCK_TYPE_VENDER	        3
#define HDMI_EDID_BLOCK_TYPE_SPEAKER	        4
#define HDMI_EDID_BLOCK_TYPE_VESA		5
#define HDMI_EDID_BLOCK_TYPE_RESERVED2	        6
#define HDMI_EDID_BLOCK_TYPE_EXTENDED_TAG       7

#define EXTENSION_Y420_VDB_TAG	0xe
#define EXTENSION_Y420_CMDB_TAG	0xf

#define EDID_DETAILED_TIMING_DES_BLOCK0_POS 0x36
#define EDID_DETAILED_TIMING_DES_BLOCK1_POS 0x48
#define EDID_DETAILED_TIMING_DES_BLOCK2_POS 0x5A
#define EDID_DETAILED_TIMING_DES_BLOCK3_POS 0x6C

static unsigned char edid_checkvalue[4] = {0};

static void edid_save_checkvalue(unsigned char *buf, unsigned int block_cnt)
{
	unsigned int i, length, max;

	if (buf == NULL)
		return;

	length = sizeof(edid_checkvalue);
	memset(edid_checkvalue, 0x00, length);

	max = (block_cnt > length)?length:block_cnt;

	for (i = 0; i < max; i++)
		edid_checkvalue[i] = *(buf+(i+1)*128-1);

	return;
}

static int Edid_DecodeHeader(struct hdmitx_info *info, unsigned char *buff)
{
	int i, ret = 0;

	if (!(buff[0] | buff[7])) {
		for (i = 1; i < 7; i++) {
			if (buff[i] != 0xFF) {
				info->output_state =
					CABLE_PLUGIN_DVI_OUT;
				ret = -1;
			}
		}
	} else {
		info->output_state = CABLE_PLUGIN_DVI_OUT;
		ret = -1;
	}
	return ret;
}

static void Edid_ReceiverBrandNameParse(struct rx_cap *pRxCap,
		unsigned char *data)
{
	int i;
	unsigned char uppercase[26] = { 0 };
	unsigned char brand[3];

	/* Fill array uppercase with 'A' to 'Z' */
	for (i = 0; i < 26; i++)
		uppercase[i] = 'A' + i;

	brand[0] = data[0] >> 2;
	brand[1] = ((data[0] & 0x3) << 3) + (data[1] >> 5);
	brand[2] = data[1] & 0x1f;

	for (i = 0; i < 3; i++)
		pRxCap->ReceiverBrandName[i] = uppercase[brand[i] - 1];
}

static int Edid_find_name_block(unsigned char *data)
{
	int ret = 0;
	int i;
	for (i = 0; i < 3; i++) {
		if (data[i])
			return ret;
	}
	if (data[3] == 0xfc)
		ret = 1;
	return ret;
}

static void Edid_ReceiverProductNameParse(struct rx_cap *pRxCap,
	unsigned char *data)
{
	int i = 0;
	/* some Display Product name end with 0x20, not 0x0a
	 */
	while ((data[i] != 0x0a) && (data[i] != 0x20) && (i < 13)) {
		pRxCap->ReceiverProductName[i] = data[i];
		i++;
	}
	pRxCap->ReceiverProductName[i] = '\0';
}

void Edid_DecodeStandardTiming(struct hdmitx_info *info,
	unsigned char *Data, unsigned char length)
{
	 unsigned char  i, TmpVal;
	 int hor_pixel, frame_rate;

	 for (i = 0; i < length; i++) {
		if ((Data[i*2] != 0x01) && (Data[i*2 + 1] != 0x01)) {
			hor_pixel = (int)((Data[i*2]+31)*8);
			TmpVal = Data[i*2 + 1] & 0xC0;

			frame_rate = (int)((Data[i*2 + 1]) & 0x3F) + 60;

			if ((hor_pixel == 720) && (frame_rate == 30))
				info->hdmi_sup_480i  = 1;
			else if ((hor_pixel == 720) && (frame_rate == 25))
				info->hdmi_sup_576i  = 1;
			else if ((hor_pixel == 720) && (frame_rate == 60))
				info->hdmi_sup_480p  = 1;
			else if ((hor_pixel == 720) && (frame_rate == 50))
				info->hdmi_sup_576p  = 1;
			else if ((hor_pixel == 1280) && (frame_rate == 60))
				info->hdmi_sup_720p_60hz  = 1;
			else if ((hor_pixel == 1280) && (frame_rate == 50))
				info->hdmi_sup_720p_50hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 30))
				info->hdmi_sup_1080i_60hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 25))
				info->hdmi_sup_1080i_50hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 60))
				info->hdmi_sup_1080p_60hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 50))
				info->hdmi_sup_1080p_50hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 24))
				info->hdmi_sup_1080p_24hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 25))
				info->hdmi_sup_1080p_25hz  = 1;
			else if ((hor_pixel == 1920) && (frame_rate == 30))
				info->hdmi_sup_1080p_30hz  = 1;
		}
	}
}

/* 12x17 */
static unsigned char Edid_TimingDescriptors[204] = {
	/* pixel clk --hsync active & blank  -- vsync  active
	 * & blank-- hsync/vsync off & wid -- Image size
	 */
	0x8C, 0x0A, 0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26, 0x7c, 0x43, 0x00,
	0x8C, 0x0A, 0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26, 0x7c, 0x43, 0x00,
	0x8C, 0x0A, 0xA0, 0x20, 0x51, 0x20, 0x18, 0x10, 0x18, 0x7e, 0x23, 0x00,
	0x8C, 0x0A, 0xA0, 0x20, 0x51, 0x20, 0x18, 0x10, 0x18, 0x7e, 0x23, 0x00,
	0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3e, 0x96, 0x00,
	0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0, 0x2D, 0x10, 0x10, 0x3e, 0x96, 0x00,
	0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0c, 0x40, 0x55, 0x00,
	0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0c, 0x40, 0x55, 0x00,
	0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20, 0x6e, 0x28, 0x55, 0x00,
	0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E, 0x20, 0xb8, 0x28, 0x55, 0x40,
	0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C, 0x16, 0x20, 0x58, 0x2c, 0x25, 0x00,
	0x01, 0x1D, 0x80, 0xD0, 0x72, 0x1C, 0x16, 0x20, 0x10, 0x2c, 0x25, 0x80,
	0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00,
	0x02, 0x3a, 0x80, 0xD0, 0x72, 0x38, 0x2d, 0x40, 0x10, 0x2c, 0x45, 0x80,
	0xfa, 0x1c, 0x80, 0x3e, 0x73, 0x38, 0x2d, 0x40, 0x7e, 0x2c, 0x45, 0x80,
	0x01, 0x1D, 0x80, 0xD0, 0x72, 0x38, 0x2d, 0x40, 0x10, 0x2c, 0x45, 0x80,
	0x01, 0x1D, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00,
};

/* ----------------------------------------------------------- */
void Edid_CompareTimingDescriptors(struct hdmitx_info *info,
	unsigned char *Data)
{
	int index1, index2;
	return;
	for (index1 = 0; index1 < 17; index1++) {
		for (index2 = 0; index2 < 12; index2++) {
			if (Data[index2] !=
				Edid_TimingDescriptors[index1*12+index2]
				)
				break;
		}
		if (index2 == 12) {
			switch (index1) {
			case 0:
			case 1:
				info->hdmi_sup_480i  = 1;
				break;
			case 2:
			case 3:
				info->hdmi_sup_576i  = 1;
			   break;
			case 4:
			case 5:
				info->hdmi_sup_480p  = 1;
				break;
			case 6:
			case 7:
				info->hdmi_sup_576p  = 1;
				break;
			case 8:
				info->hdmi_sup_720p_60hz  = 1;
				break;
			case 9:
				info->hdmi_sup_720p_50hz  = 1;
				break;
			case 10:
				info->hdmi_sup_1080i_60hz  = 1;
				break;
			case 11:
				info->hdmi_sup_1080i_50hz  = 1;
				break;
			case 12:
				info->hdmi_sup_1080p_60hz  = 1;
				break;
			case 13:
				info->hdmi_sup_1080p_50hz  = 1;
				break;
			case 14:
				info->hdmi_sup_1080p_24hz  = 1;
				break;
			case 15:
				info->hdmi_sup_1080p_25hz  = 1;
				break;
			case 16:
				info->hdmi_sup_1080p_30hz  = 1;
				break;
			default:
				break;
			}
			break;
		}
	}
}


/* ----------------------------------------------------------- */
void Edid_ParseCEADetailedTimingDescriptors(struct hdmitx_info *info,
	unsigned char blk_mun, unsigned char BaseAddr,
	unsigned char *buff)
{
	unsigned char index_edid;

	for (index_edid = 0; index_edid < blk_mun; index_edid++) {
		Edid_CompareTimingDescriptors(info, &buff[BaseAddr]);
		BaseAddr += 18;
		/* there is not the TimingDescriptors */
		if ((BaseAddr + 18) > 0x7d)
			break;
	}
}

static struct vsdb_phyaddr vsdb_local = {0};
int get_vsdb_phy_addr(struct vsdb_phyaddr *vsdb)
{
	vsdb = &vsdb_local;
	return vsdb->valid;
}

void set_vsdb_phy_addr(struct vsdb_phyaddr *vsdb,
	unsigned char *edid_offset)
{
	vsdb->a = (edid_offset[4] >> 4) & 0xf;
	vsdb->b = (edid_offset[4] >> 0) & 0xf;
	vsdb->c = (edid_offset[5] >> 4) & 0xf;
	vsdb->d = (edid_offset[5] >> 0) & 0xf;
	vsdb_local = *vsdb;
	vsdb->valid = 1;
}

int Edid_Parse_check_HDMI_VSDB(struct hdmitx_info *info,
	unsigned char *buff)
{
	unsigned char  VSpecificBoundary, BlockAddr,  len;
	int temp_addr = 0;
	VSpecificBoundary = buff[2];

	if (VSpecificBoundary < 4) {
		info->output_state = CABLE_PLUGIN_DVI_OUT;
		return -1;
	}
	BlockAddr = CEA_DATA_BLOCK_COLLECTION_ADDR_1StP;
	while (BlockAddr < VSpecificBoundary) {
		len = buff[BlockAddr] & 0x1F;
		if ((buff[BlockAddr] & 0xE0) == VENDOR_TAG)
			break;
		temp_addr = BlockAddr + len + 1;
		if (temp_addr >= VSpecificBoundary)
			break;
		BlockAddr = BlockAddr + len + 1;
	}

	set_vsdb_phy_addr(&info->vsdb_phy_addr, &buff[BlockAddr]);

	if (temp_addr >= VSpecificBoundary) {
		info->output_state = CABLE_PLUGIN_DVI_OUT;
		return -1;
	} else {
		if ((buff[BlockAddr + 1] != 0x03) ||
			(buff[BlockAddr + 2] != 0x0C) ||
			(buff[BlockAddr + 3] != 0x0)) {
			info->output_state = CABLE_PLUGIN_DVI_OUT;
			return -1;
		}
	}
	return 0;
}

/* ----------------------------------------------------------- */
void Edid_MonitorCapable861(struct hdmitx_info *info,
	unsigned char edid_flag)
{
	 if (edid_flag & 0x80)
		info->support_underscan_flag = 1;
	 if (edid_flag & 0x40)
		info->support_basic_audio_flag = 1;
	 if (edid_flag & 0x20)
		info->support_ycbcr444_flag = 1;
	 if (edid_flag & 0x10)
		info->support_ycbcr422_flag = 1;
}


/* ----------------------------------------------------------- */
static void Edid_ParsingVideoDATABlock(struct hdmitx_info *info,
	unsigned char *buff, unsigned char BaseAddr,
	unsigned char NBytes)
{
	unsigned char i;
	NBytes &= 0x1F;
	for (i = 0; i < NBytes; i++) {
		switch (buff[i + BaseAddr]&0x7F) {
		case 6:
		case 7:
			info->hdmi_sup_480i  = 1;
			break;
		case 21:
		case 22:
			info->hdmi_sup_576i  = 1;
			break;
		case 2:
		case 3:
			info->hdmi_sup_480p  = 1;
			break;
		case 17:
		case 18:
			info->hdmi_sup_576p  = 1;
			break;
		case 4:
			info->hdmi_sup_720p_60hz  = 1;
			break;
		case 19:
			info->hdmi_sup_720p_50hz  = 1;
			break;
		case 5:
			info->hdmi_sup_1080i_60hz  = 1;
			break;
		case 20:
			info->hdmi_sup_1080i_50hz  = 1;
			break;
		case 16:
			info->hdmi_sup_1080p_60hz  = 1;
			break;
		case 31:
			info->hdmi_sup_1080p_50hz  = 1;
			break;
		case 32:
			info->hdmi_sup_1080p_24hz  = 1;
			break;
		case 33:
			info->hdmi_sup_1080p_25hz  = 1;
			break;
		case 34:
			info->hdmi_sup_1080p_30hz  = 1;
			break;
		default:
			break;
		}
	}
}

/* ----------------------------------------------------------- */
static void Edid_ParsingAudioDATABlock(struct hdmitx_info *info,
	unsigned char *Data, unsigned char BaseAddr,
	unsigned char NBytes)
{
	 unsigned char AudioFormatCode;
	 int i = BaseAddr;
	NBytes &= 0x1F;
	do {
		AudioFormatCode = (Data[i]&0xF8)>>3;
		switch (AudioFormatCode) {
		case 1:
			info->tv_audio_info._60958_PCM.support_flag = 1;
			info->tv_audio_info._60958_PCM.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._60958_PCM._192k =
					1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._60958_PCM._176k =
					1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._60958_PCM._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._60958_PCM._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._60958_PCM._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._60958_PCM._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._60958_PCM._32k = 1;
			if ((Data[i+2]&0x04))
				info->tv_audio_info._60958_PCM._24bit = 1;
			if ((Data[i+2]&0x02))
				info->tv_audio_info._60958_PCM._20bit = 1;
			if ((Data[i+2]&0x01))
				info->tv_audio_info._60958_PCM._16bit = 1;
			break;
		case 2:
			info->tv_audio_info._AC3.support_flag = 1;
			info->tv_audio_info._AC3.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._AC3._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._AC3._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._AC3._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._AC3._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._AC3._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._AC3._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._AC3._32k = 1;
				info->tv_audio_info._AC3._max_bit =
					Data[i+2];
			break;
		case 3:
			info->tv_audio_info._MPEG1.support_flag = 1;
			info->tv_audio_info._MPEG1.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._MPEG1._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._MPEG1._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._MPEG1._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._MPEG1._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._MPEG1._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._MPEG1._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._MPEG1._32k = 1;
				info->tv_audio_info._MPEG1._max_bit =
					Data[i+2];
			break;
		case 4:
			info->tv_audio_info._MP3.support_flag = 1;
			info->tv_audio_info._MP3.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._MP3._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._MP3._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._MP3._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._MP3._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._MP3._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._MP3._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._MP3._32k = 1;
				info->tv_audio_info._MP3._max_bit = Data[i+2];
			break;
		case 5:
			info->tv_audio_info._MPEG2.support_flag = 1;
			info->tv_audio_info._MPEG2.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._MPEG2._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._MPEG2._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._MPEG2._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._MPEG2._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._MPEG2._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._MPEG2._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._MPEG2._32k = 1;
				info->tv_audio_info._MPEG2._max_bit = Data[i+2];
			break;
		case 6:
			info->tv_audio_info._AAC.support_flag = 1;
			info->tv_audio_info._AAC.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._AAC._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._AAC._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._AAC._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._AAC._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._AAC._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._AAC._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._AAC._32k = 1;
				info->tv_audio_info._AAC._max_bit = Data[i+2];
			break;
		case 7:
			info->tv_audio_info._DTS.support_flag = 1;
			info->tv_audio_info._DTS.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._DTS._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._DTS._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._DTS._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._DTS._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._DTS._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._DTS._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._DTS._32k = 1;
				info->tv_audio_info._DTS._max_bit = Data[i+2];
			break;
		case 8:
			info->tv_audio_info._ATRAC.support_flag = 1;
			info->tv_audio_info._ATRAC.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._ATRAC._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._ATRAC._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._ATRAC._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._ATRAC._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._ATRAC._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._ATRAC._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._ATRAC._32k = 1;
				info->tv_audio_info._ATRAC._max_bit = Data[i+2];
			break;
		case 9:
			info->tv_audio_info._One_Bit_Audio.support_flag = 1;
			info->tv_audio_info._One_Bit_Audio.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._One_Bit_Audio._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._One_Bit_Audio._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._One_Bit_Audio._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._One_Bit_Audio._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._One_Bit_Audio._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._One_Bit_Audio._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._One_Bit_Audio._32k = 1;
				info->tv_audio_info._One_Bit_Audio._max_bit =
					Data[i+2];
			break;
		case 10:
			info->tv_audio_info._Dolby.support_flag = 1;
			info->tv_audio_info._Dolby.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._Dolby._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._Dolby._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._Dolby._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._Dolby._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._Dolby._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._Dolby._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._Dolby._32k = 1;
				info->tv_audio_info._Dolby._max_bit = Data[i+2];
			break;

		case 11:
			info->tv_audio_info._DTS_HD.support_flag = 1;
			info->tv_audio_info._DTS_HD.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._DTS_HD._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._DTS_HD._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._DTS_HD._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._DTS_HD._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._DTS_HD._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._DTS_HD._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._DTS_HD._32k = 1;
				info->tv_audio_info._DTS_HD._max_bit =
					Data[i+2];
			break;
		case 12:
			info->tv_audio_info._MAT.support_flag = 1;
			info->tv_audio_info._MAT.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._MAT._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._MAT._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._MAT._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._MAT._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._MAT._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._MAT._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._MAT._32k = 1;
				info->tv_audio_info._MAT._max_bit = Data[i+2];
			break;

		case 13:
			info->tv_audio_info._ATRAC.support_flag = 1;
			info->tv_audio_info._ATRAC.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._DST._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._DST._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._DST._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._DST._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._DST._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._DST._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._DST._32k = 1;
				info->tv_audio_info._DST._max_bit = Data[i+2];
			break;

		case 14:
			info->tv_audio_info._WMA.support_flag = 1;
			info->tv_audio_info._WMA.max_channel_num =
				(Data[i]&0x07);
			if ((Data[i+1]&0x40))
				info->tv_audio_info._WMA._192k = 1;
			if ((Data[i+1]&0x20))
				info->tv_audio_info._WMA._176k = 1;
			if ((Data[i+1]&0x10))
				info->tv_audio_info._WMA._96k = 1;
			if ((Data[i+1]&0x08))
				info->tv_audio_info._WMA._88k = 1;
			if ((Data[i+1]&0x04))
				info->tv_audio_info._WMA._48k = 1;
			if ((Data[i+1]&0x02))
				info->tv_audio_info._WMA._44k = 1;
			if ((Data[i+1]&0x01))
				info->tv_audio_info._WMA._32k = 1;
				info->tv_audio_info._WMA._max_bit = Data[i+2];
			break;

		default:
		break;
		}
	i += 3;
	} while (i < (NBytes + BaseAddr));
}

/* ----------------------------------------------------------- */
static void Edid_ParsingSpeakerDATABlock(struct hdmitx_info *info,
	unsigned char *buff, unsigned char BaseAddr)
{
	int ii;
	for (ii = 1; ii < 0x80; ) {
		switch (buff[BaseAddr] & ii) {
		case 0x40:
			info->tv_audio_info.speaker_allocation.rlc_rrc = 1;
			break;

		case 0x20:
			info->tv_audio_info.speaker_allocation.flc_frc = 1;
			break;

		case 0x10:
			info->tv_audio_info.speaker_allocation.rc = 1;
			break;

		case 0x08:
			info->tv_audio_info.speaker_allocation.rl_rr = 1;
			break;

		case 0x04:
			info->tv_audio_info.speaker_allocation.fc = 1;
			break;

		case 0x02:
			info->tv_audio_info.speaker_allocation.lfe = 1;
			break;

		case 0x01:
			info->tv_audio_info.speaker_allocation.fl_fr = 1;
			break;

		default:
		break;
		}
		ii = ii << 1;
	}
}


/* ----------------------------------------------------------- */
static int Edid_ParsingY420VDBBlock(struct rx_cap *pRXCap,
	unsigned char *buf)
{
	unsigned char tag = 0, ext_tag = 0, data_end = 0;
	unsigned int pos = 0;

	tag = (buf[pos] >> 5) & 0x7;
	data_end = (buf[pos] & 0x1f)+1;
	pos++;
	ext_tag = buf[pos];

	if ((tag != 0x7) || (ext_tag != 0xe))
		goto INVALID_Y420VDB;

	pos++;
	while (pos < data_end) {
		if (pRXCap->VIC_count < VIC_MAX_NUM)
			pRXCap->VIC[pRXCap->VIC_count] =
				HDMITX_VIC420_OFFSET + buf[pos];
		pos++;
		pRXCap->VIC_count++;
	}

	return 0;

INVALID_Y420VDB:
	pr_info("[%s] it's not a valid y420vdb!\n", __func__);
	return -1;
}

/* ----------------------------------------------------------- */
static int Edid_ParsingY420CMDBBlock(struct hdmitx_info *info,
	unsigned char *buf)
{
	unsigned char tag = 0, ext_tag = 0, length = 0, data_end = 0;
	unsigned int pos = 0, i = 0;

	tag = (buf[pos] >> 5) & 0x7;
	length = buf[pos] & 0x1f;
	data_end = length + 1;
	pos++;
	ext_tag = buf[pos];

	if ((tag != 0x7) || (ext_tag != 0xf))
		goto INVALID_Y420CMDB;

	if (length == 1) {
		info->y420_all_vic = 1;
		return 0;
	}

	info->bitmap_length = 0;
	info->bitmap_valid = 0;
	memset(info->y420cmdb_bitmap, 0x00, Y420CMDB_MAX);

	pos++;
	if (pos < data_end) {
		info->bitmap_length = data_end - pos;
		info->bitmap_valid = 1;
	}
	while (pos < data_end) {
		if (i < VIC_MAX_NUM)
			info->y420cmdb_bitmap[i] = buf[pos];
		pos++;
		i++;
	}

	return 0;

INVALID_Y420CMDB:
	pr_info("[%s] it's not a valid y420cmdb!\n", __func__);
	return -1;

}

static int Edid_Y420CMDB_fill_all_vic(struct hdmitx_dev *hdmitx_device)
{
	struct rx_cap *rxcap = &(hdmitx_device->RXCap);
	struct hdmitx_info *info = &(hdmitx_device->hdmi_info);
	unsigned int count = rxcap->VIC_count;
	unsigned int a, b;

	if (info->y420_all_vic != 1)
		return 1;

	a = count/8;
	a = (a >= Y420CMDB_MAX)?Y420CMDB_MAX:a;
	b = count%8;

	if (a > 0)
		memset(&(info->y420cmdb_bitmap[0]), 0xff, a);

	if ((b != 0) && (a < Y420CMDB_MAX))
		info->y420cmdb_bitmap[a] = (((1 << b) - 1) << (8-b));

	info->bitmap_length = (b == 0) ? a : (a + 1);
	info->bitmap_valid = (info->bitmap_length != 0)?1:0;

	return 0;
}

static int Edid_Y420CMDB_PostProcess(struct hdmitx_dev *hdmitx_device)
{
	unsigned int i = 0, j = 0, valid = 0;
	struct rx_cap *rxcap = &(hdmitx_device->RXCap);
	struct hdmitx_info *info = &(hdmitx_device->hdmi_info);
	unsigned char *p = NULL;

	if (info->y420_all_vic == 1)
		Edid_Y420CMDB_fill_all_vic(hdmitx_device);

	if (info->bitmap_valid == 0)
		goto PROCESS_END;

	for (i = 0; i < info->bitmap_length; i++) {
		p = &(info->y420cmdb_bitmap[i]);
		for (j = 0; j < 8; j++) {
			valid = ((*p >> j) & 0x1);
			if (valid != 0) {
				rxcap->VIC[rxcap->VIC_count] =
				HDMITX_VIC420_OFFSET + rxcap->VIC[i*8+j];
				rxcap->VIC_count++;
			}
		}
	}

PROCESS_END:
	return 0;
}

static void Edid_Y420CMDB_Reset(struct hdmitx_info *info)
{
	info->bitmap_valid = 0;
	info->bitmap_length = 0;
	info->y420_all_vic = 0;
	memset(info->y420cmdb_bitmap, 0x00, Y420CMDB_MAX);

	return;
}
/* ----------------------------------------------------------- */
int Edid_ParsingCEADataBlockCollection(struct hdmitx_info *info,
	unsigned char *buff)
{
	unsigned char AddrTag, D, Addr, Data;
	int temp_addr;

	/* Byte number offset d where Detailed Timing data begins */
	D = buff[2];
	Addr = 4;

	AddrTag = Addr;
	do {
		Data = buff[AddrTag];
		switch (Data&0xE0) {
		case VIDEO_TAG:
			if ((Addr + (Data&0x1f)) < D)
				Edid_ParsingVideoDATABlock(info, buff,
					Addr + 1, (Data & 0x1F));
				break;

		case AUDIO_TAG:
			if ((Addr + (Data&0x1f)) < D)
				Edid_ParsingAudioDATABlock(info, buff,
					Addr + 1, (Data & 0x1F));
			break;

		case SPEAKER_TAG:
			if ((Addr + (Data&0x1f)) < D)
				Edid_ParsingSpeakerDATABlock(info, buff,
					Addr + 1);
			break;

		case VENDOR_TAG:
			if ((Addr + (Data&0x1f)) < D) {
				if ((buff[Addr + 1] != 0x03) ||
					(buff[Addr + 2] != 0x0c) ||
					(buff[Addr + 3] != 0x00)) {
					info->auth_state = HDCP_NO_AUTH;
					info->output_state =
						CABLE_PLUGIN_DVI_OUT;
				}
				if ((Data&0x1f) > 5) {
					if (buff[Addr + 6] & 0x80)
						info->support_ai_flag = 1;
				}
			}
			break;

		default:
		break;
		}
		Addr += (Data & 0x1F);   /* next Tag Address */
		AddrTag = ++Addr;
		Data = buff[Addr];
		temp_addr =   Addr + (Data & 0x1F);
		if (temp_addr >= D)	/* force to break; */
			break;
	} while (Addr < D);

	return 0;
}

/* ----------------------------------------------------------- */

/* parse Sink 3D information */
static int hdmitx_edid_3d_parse(struct rx_cap *pRXCap, unsigned char *dat,
	unsigned size)
{
	int j = 0;
	int bit = 16;
	unsigned int base = 0;
	unsigned int pos = base + 1;

	if (dat[base] & (1<<7))
		pos += 2;
	if (dat[base] & (1<<6))
		pos += 2;
	if (dat[base] & (1<<5)) {
		pRXCap->threeD_present = dat[pos] >> 7;
		pRXCap->threeD_Multi_present = (dat[pos] >> 5) & 0x3;
		pos += 1;
		pRXCap->hdmi_vic_LEN = (dat[pos]) >> 5;
		pRXCap->HDMI_3D_LEN = (dat[pos]) & 0x1f;
		pos += pRXCap->hdmi_vic_LEN + 1;
		if ((pRXCap->threeD_Multi_present == 0x01) ||
			(pRXCap->threeD_Multi_present == 0x2)) {
			pRXCap->threeD_Structure_ALL_15_0 =
				(dat[pos] << 8) + dat[pos+1];
			pos += 2;
		}
		if (pRXCap->threeD_Multi_present == 0x2) {
			pRXCap->threeD_MASK_15_0 = (dat[pos] << 8)
				+ dat[pos+1];
			pos += 2;
		}
	}
	while (pos < size) {
		if ((dat[pos] & 0xf) == 0x0) { /* frame packing */
			pRXCap->support_3d_format[pRXCap->VIC[((dat[pos]
				& 0xf0) >> 4)]].frame_packing = 1;
			pRXCap->support_3d_format[pRXCap->VIC[((
				dat[pos+1] & 0xf0) >> 4)]].frame_packing = 1;
		}
		if ((dat[pos] & 0xf) == 0x6) { /* top and bottom */
			pRXCap->support_3d_format[pRXCap->VIC[((dat[pos]
				& 0xf0) >> 4)]].top_and_bottom = 1;
			pRXCap->support_3d_format[pRXCap->VIC[((dat[pos+1]
				& 0xf0) >> 4)]].top_and_bottom = 1;
		}
		if ((dat[pos] & 0xf) == 0x8) { /* top and bottom */
			pRXCap->support_3d_format[pRXCap->VIC[((dat[pos]
				& 0xf0) >> 4)]].side_by_side = 1;
			pRXCap->support_3d_format[pRXCap->VIC[((dat[pos+1]
				& 0xf0) >> 4)]].side_by_side = 1;
		}
		pos += 2;
	}

	while (bit--) {
		if (((pRXCap->threeD_MASK_15_0) >> j) & 0x1) {
			/* frame packing */
			if (pRXCap->threeD_Structure_ALL_15_0 & (1 << 0))
				pRXCap->support_3d_format[pRXCap->VIC[j]].
					frame_packing = 1;
			/* top and bottom */
			if (pRXCap->threeD_Structure_ALL_15_0 & (1 << 6))
				pRXCap->support_3d_format[pRXCap->VIC[j]].
					top_and_bottom = 1;
			/* top and bottom */
			if (pRXCap->threeD_Structure_ALL_15_0 & (1 << 8))
				pRXCap->support_3d_format[pRXCap->VIC[j]].
					side_by_side = 1;
		j++;
		}
	}
	return 1;
}

/* parse Sink 4k2k information */
static void hdmitx_edid_4k2k_parse(struct rx_cap *pRXCap, unsigned char *dat,
	unsigned size)
{
	if ((size > 4) || (size == 0)) {
		hdmi_print(ERR, EDID
			"HDMI: 4k2k in edid out of range, SIZE = %d\n",
			size);
		return;
	}
	while (size--) {
		if (*dat == 1)
			pRXCap->VIC[pRXCap->VIC_count] = HDMI_4k2k_30;
		else if (*dat == 2)
			pRXCap->VIC[pRXCap->VIC_count] = HDMI_4k2k_25;
		else if (*dat == 3)
			pRXCap->VIC[pRXCap->VIC_count] = HDMI_4k2k_24;
		else if (*dat == 4)
			pRXCap->VIC[pRXCap->VIC_count] = HDMI_4k2k_smpte_24;
		else
			;
		dat++;
		pRXCap->VIC_count++;
	}
}

static int hdmitx_edid_block_parse(struct hdmitx_dev *hdmitx_device,
	unsigned char *BlockBuf)
{
	unsigned char offset, End;
	unsigned char count;
	unsigned char tag;
	int i, tmp, idx;
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);

	if (BlockBuf[0] != 0x02)
		return -1; /* not a CEA BLOCK. */
	End = BlockBuf[2]; /* CEA description. */
	pRXCap->native_Mode = BlockBuf[3];

	pRXCap->VIC_count = 0;
	pRXCap->native_VIC = 0xff;

	Edid_Y420CMDB_Reset(&(hdmitx_device->hdmi_info));

	for (offset = 4 ; offset < End ; ) {
		tag = BlockBuf[offset] >> 5;
		count = BlockBuf[offset] & 0x1f;
		switch (tag) {
		case HDMI_EDID_BLOCK_TYPE_AUDIO:
			pRXCap->AUD_count = count/3;
			offset++;
			for (i = 0 ; i < pRXCap->AUD_count ; i++) {
				pRXCap->RxAudioCap[i].audio_format_code =
					(BlockBuf[offset + i * 3]>>3)&0xf;
				pRXCap->RxAudioCap[i].channel_num_max =
					BlockBuf[offset + i * 3]&0x7;
				pRXCap->RxAudioCap[i].freq_cc =
					BlockBuf[offset + i * 3 + 1]&0x7f;
				pRXCap->RxAudioCap[i].cc3 =
					BlockBuf[offset + i * 3 + 2]&0x7;
			}
			offset += count;
			break;

		case HDMI_EDID_BLOCK_TYPE_VIDEO:
			offset++;
			for (i = 0 ; i < count ; i++) {
				unsigned char VIC;
				VIC = BlockBuf[offset + i] & (~0x80);
				pRXCap->VIC[pRXCap->VIC_count] = VIC;
				if (BlockBuf[offset + i] & 0x80)
					pRXCap->native_VIC = VIC;
				pRXCap->VIC_count++;
			}
			offset += count;
			break;

		case HDMI_EDID_BLOCK_TYPE_VENDER:
			offset++;
			if ((BlockBuf[offset] == 0x03) &&
				(BlockBuf[offset+1] == 0x0c) &&
				(BlockBuf[offset+2] == 0x00))
				pRXCap->IEEEOUI = 0x000c03;
			else
				goto case_hf;
			pRXCap->ColorDeepSupport =
				(unsigned long)BlockBuf[offset+5];
			pRXCap->Max_TMDS_Clock =
				(unsigned long)BlockBuf[offset+6];
			if (count > 7) {
				tmp = BlockBuf[offset+7];
				idx = offset + 8;
				if (tmp & (1<<6))
					idx += 2;
				if (tmp & (1<<7))
					idx += 2;
				if (tmp & (1<<5)) {
					idx += 1;
					/* valid 4k */
					if (BlockBuf[idx] & 0xe0) {
						hdmitx_edid_4k2k_parse(
							pRXCap,
							&BlockBuf[idx + 1],
							BlockBuf[idx] >> 5);
					}
					/* valid 3D */
					if (BlockBuf[idx-1] & 0xe0) {
						hdmitx_edid_3d_parse(
							pRXCap,
							&BlockBuf[offset+7],
							count - 7);
					}
				}
			}
			goto case_next;
case_hf:
			if ((BlockBuf[offset] == 0xd8) &&
				(BlockBuf[offset+1] == 0x5d) &&
				(BlockBuf[offset+2] == 0xc4))
				pRXCap->HF_IEEEOUI = 0xd85dc4;
			pRXCap->scdc_present =
				!!(BlockBuf[offset+5] & (1 << 7));
			pRXCap->scdc_rr_capable =
				!!(BlockBuf[offset+5] & (1 << 6));
			pRXCap->lte_340mcsc_scramble =
				!!(BlockBuf[offset+5] & (1 << 3));
case_next:
			offset += count; /* ignore the remaind. */
			break;

		case HDMI_EDID_BLOCK_TYPE_SPEAKER:
			offset++;
			pRXCap->RxSpeakerAllocation = BlockBuf[offset];
			offset += count;
			break;

		case HDMI_EDID_BLOCK_TYPE_VESA:
			offset++;
			offset += count;
			break;

		case HDMI_EDID_BLOCK_TYPE_EXTENDED_TAG:
			{
				unsigned char ext_tag = 0;

				ext_tag = BlockBuf[offset+1];
				switch (ext_tag) {

				case EXTENSION_Y420_VDB_TAG:
					Edid_ParsingY420VDBBlock(pRXCap,
						&BlockBuf[offset]);
					break;

				case EXTENSION_Y420_CMDB_TAG:
					Edid_ParsingY420CMDBBlock(
						&(hdmitx_device->hdmi_info),
						&BlockBuf[offset]);
					break;

				default:
					break;
				}
			}
			offset += count+1;
			break;

		case HDMI_EDID_BLOCK_TYPE_RESERVED:
			offset++;
			offset += count;
			break;

		case HDMI_EDID_BLOCK_TYPE_RESERVED2:
			offset++;
			offset += count;
			break;

		default:
			break;
		}
	}

	Edid_Y420CMDB_PostProcess(hdmitx_device);
	hdmitx_device->vic_count = pRXCap->VIC_count;
	return 0;
}

/* add default VICs for DVI case */
static void hdmitx_edid_set_default_vic(struct hdmitx_dev *hdmitx_device)
{
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);
	pRXCap->VIC_count = 0x3;
	pRXCap->VIC[0] = 2;
	pRXCap->VIC[1] = 4;
	pRXCap->VIC[2] = 16;
	pRXCap->native_VIC = 4;
	hdmitx_device->vic_count = pRXCap->VIC_count;
	hdmi_print(IMP, EDID "HDMI: set default vic\n");
}

#if 0
#define PRINT_HASH(hash)	\
	{			\
		pr_info("%s:%d ", __func__, __LINE__); int __i;	\
		for (__i = 0; __i < 20; __i++)	\
			pr_info("%02x,", hash[__i]);	\
			pr_info("\n");\
	}
#else
#define PRINT_HASH(hash)
#endif

static int edid_hash_calc(unsigned char *hash, const char *data,
	unsigned int len)
{
	struct scatterlist sg;

	struct crypto_hash *tfm;
	struct hash_desc desc;

	tfm = crypto_alloc_hash("sha1", 0, CRYPTO_ALG_ASYNC);
	PRINT_HASH(hash);
	if (IS_ERR(tfm))
		return -EINVAL;

	PRINT_HASH(hash);
	/* ... set up the scatterlists ... */
	sg_init_one(&sg, (u8 *) data, len);
	desc.tfm = tfm;
	desc.flags = 0;

	if (crypto_hash_digest(&desc, &sg, len, hash))
		return -EINVAL;
	PRINT_HASH(hash);
	crypto_free_hash(tfm);
	return 1;
}

static int hdmitx_edid_search_IEEEOUI(char *buf)
{
	int i;

	for (i = 0; i < 125; i++) {
		if ((buf[i] == 0x03) && (buf[i+1] == 0x0c) &&
			(buf[i+2] == 0x00))
			return 1;
	}
	return 0;
}

/* check EDID strictly */
static int edid_check_valid(unsigned char *buf)
{
	unsigned int chksum = 0;
	unsigned int i = 0;

	/* check block 0 first 8 bytes */
	if ((buf[0] != 0) && (buf[7] != 0))
		return 0;
	for (i = 1; i < 7; i++) {
		if (buf[i] != 0xff)
			return 0;
	}

	/* check block 0 checksum */
	for (chksum = 0, i = 0; i < 0x80; i++)
		chksum += buf[i];

	if ((chksum & 0xff) != 0)
		return 0;

	/* check Extension flag at block 0 */
	if (buf[0x7e] == 0)
		return 0;

	/* check block 1 extension tag */
	if (buf[0x80] != 0x2)
		return 0;

	/* check block 1 checksum */
	for (chksum = 0, i = 0x80; i < 0x100; i++)
		chksum += buf[i];

	if ((chksum & 0xff) != 0)
		return 0;

	return 1;
}

static int check_dvi_hdmi_edid_valid(unsigned char *buf)
{
	unsigned int chksum = 0;
	unsigned int i = 0;

	/* check block 0 first 8 bytes */
	if ((buf[0] != 0) && (buf[7] != 0))
		return 0;
	for (i = 1; i < 7; i++) {
		if (buf[i] != 0xff)
			return 0;
	}

	/* check block 0 checksum */
	for (chksum = 0, i = 0; i < 0x80; i++)
		chksum += buf[i];
	if ((chksum & 0xff) != 0)
		return 0;

	if (buf[0x7e] == 0)/* check Extension flag at block 0 */
		return 1;
	else if (buf[0x80] != 0x2)/* check block 1 extension tag */
		return 0;

	/* check block 1 checksum */
	for (chksum = 0, i = 0x80; i < 0x100; i++)
		chksum += buf[i];
	if ((chksum & 0xff) != 0)
		return 0;

	return 1;
}

int hdmitx_edid_parse(struct hdmitx_dev *hdmitx_device)
{
	unsigned char CheckSum;
	unsigned char zero_numbers;
	unsigned char BlockCount;
	unsigned char *EDID_buf;
	int i, j, ret_val;
	int idx[4];
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);
	if (check_dvi_hdmi_edid_valid(hdmitx_device->EDID_buf))
		EDID_buf = hdmitx_device->EDID_buf;
	else
		EDID_buf = hdmitx_device->EDID_buf1;
	hdmi_print(0, "EDID Parser:\n");

	/* Calculate the EDID hash for special use */
	memset(hdmitx_device->EDID_hash, 0,
		ARRAY_SIZE(hdmitx_device->EDID_hash));
	edid_hash_calc(hdmitx_device->EDID_hash, hdmitx_device->EDID_buf, 256);

	ret_val = Edid_DecodeHeader(&hdmitx_device->hdmi_info, &EDID_buf[0]);

/* if(ret_val == -1) */
/* return -1; */

	for (i = 0, CheckSum = 0 ; i < 128 ; i++) {
		CheckSum += EDID_buf[i];
		CheckSum &= 0xFF;
	}

	if (CheckSum != 0) {
		hdmitx_device->hdmi_info.output_state = CABLE_PLUGIN_DVI_OUT;
		hdmi_print(0, "PLUGIN_DVI_OUT\n");
		/* return -1 ; */
	}

	Edid_ReceiverBrandNameParse(&hdmitx_device->RXCap, &EDID_buf[8]);

	idx[0] = EDID_DETAILED_TIMING_DES_BLOCK0_POS;
	idx[1] = EDID_DETAILED_TIMING_DES_BLOCK1_POS;
	idx[2] = EDID_DETAILED_TIMING_DES_BLOCK2_POS;
	idx[3] = EDID_DETAILED_TIMING_DES_BLOCK3_POS;
	for (i = 0; i < 4; i++) {
		if (Edid_find_name_block(&EDID_buf[idx[i]]))
			Edid_ReceiverProductNameParse(&hdmitx_device->RXCap,
				&EDID_buf[idx[i]+5]);
	}
	Edid_DecodeStandardTiming(&hdmitx_device->hdmi_info, &EDID_buf[26], 8);
	Edid_ParseCEADetailedTimingDescriptors(&hdmitx_device->hdmi_info,
		4, 0x36, &EDID_buf[0]);

	BlockCount = EDID_buf[0x7E];
	hdmitx_device->RXCap.blk0_chksum = EDID_buf[0x7F];

	if (BlockCount == 0) {
		hdmitx_device->hdmi_info.output_state = CABLE_PLUGIN_DVI_OUT;
		hdmi_print(0, "EDID BlockCount=0\n");
		hdmitx_edid_set_default_vic(hdmitx_device);

		/* DVI case judgement: only contains one block and
		 * checksum valid */
		CheckSum = 0;
		zero_numbers = 0;
		for (i = 0; i < 128; i++) {
			CheckSum += EDID_buf[i];
			if (EDID_buf[i] == 0)
				zero_numbers++;
		}
		hdmi_print(INF, EDID "edid blk0 checksum:%d ext_flag:%d\n",
			CheckSum, EDID_buf[0x7e]);
		if ((CheckSum & 0xff) == 0)
			hdmitx_device->RXCap.IEEEOUI = 0;
		else
			hdmitx_device->RXCap.IEEEOUI = 0x0c03;
		if (zero_numbers > 120)
			hdmitx_device->RXCap.IEEEOUI = 0x0c03;

		return 0; /* do nothing. */
	}

	/* Note: some DVI monitor have more than 1 block */
	if ((BlockCount == 1) && (EDID_buf[0x81] == 1)) {
		hdmitx_device->RXCap.IEEEOUI = 0;
		return 0;
	} else if (BlockCount > EDID_MAX_BLOCK) {
		BlockCount = EDID_MAX_BLOCK;
	}

	for (i = 1 ; i <= BlockCount ; i++) {
		if ((BlockCount > 1) && (i == 1))
			CheckSum = 0;	   /* ignore the block1 data */
		else {
			if (((BlockCount == 1) && (i == 1)) ||
				((BlockCount > 1) && (i == 2)))
				Edid_Parse_check_HDMI_VSDB(
					&hdmitx_device->hdmi_info,
					&EDID_buf[i * 128]);

			for (j = 0, CheckSum = 0 ; j < 128 ; j++) {
				CheckSum += EDID_buf[i*128 + j];
				CheckSum &= 0xFF;
			}
			if (CheckSum != 0)
				hdmitx_device->hdmi_info.output_state =
					CABLE_PLUGIN_DVI_OUT;
			else {
				Edid_MonitorCapable861(
					&hdmitx_device->hdmi_info,
					EDID_buf[i * 128 + 3]);
				ret_val = Edid_ParsingCEADataBlockCollection(
					&hdmitx_device->hdmi_info,
					&EDID_buf[i * 128]);
				Edid_ParseCEADetailedTimingDescriptors(
					&hdmitx_device->hdmi_info, 5,
					EDID_buf[i * 128 + 2],
					&EDID_buf[i * 128]);
				if (hdmitx_device->hdmi_info.output_state !=
					CABLE_PLUGIN_DVI_OUT)
					hdmitx_device->hdmi_info.output_state =
						CABLE_PLUGIN_HDMI_OUT;
			}
		}

		if (EDID_buf[i*128+0] == 0x2) {
			if (hdmitx_edid_block_parse(hdmitx_device,
				&(EDID_buf[i*128])) >= 0) {
				if (hdmitx_device->RXCap.IEEEOUI == 0x0c03)
					break;
			}
		}
	}

	if (hdmitx_edid_search_IEEEOUI(&EDID_buf[128])) {
		pRXCap->IEEEOUI = 0x0c03;
		pr_info("hdmitx: edid: find IEEEOUT\n");
	} else {
		pRXCap->IEEEOUI = 0x0;
		pr_info("hdmitx: edid: not find IEEEOUT\n");
	}

	if ((pRXCap->IEEEOUI != 0x0c03) || (pRXCap->IEEEOUI == 0x0) ||
		(pRXCap->VIC_count == 0))
		hdmitx_edid_set_default_vic(hdmitx_device);

	/* strictly DVI device judgement */
	/* valid EDID & no audio tag & no IEEEOUI */
	if (edid_check_valid(&EDID_buf[0]) &&
		!hdmitx_edid_search_IEEEOUI(&EDID_buf[128])) {
		pRXCap->IEEEOUI = 0x0;
		pr_info("hdmitx: edid: sink is DVI device\n");
	} else {
		pRXCap->IEEEOUI = 0x0c03;
	}

	edid_save_checkvalue(EDID_buf, BlockCount+1);

#if 1
	i = hdmitx_edid_dump(hdmitx_device, (char *)(hdmitx_device->tmp_buf),
		HDMI_TMP_BUF_SIZE);
	hdmitx_device->tmp_buf[i] = 0;
	hdmi_print(0, "\n");
#endif
	return 0;

}

static struct dispmode_vic dispmode_vic_tab[] = {
	{"480i_4_3",  HDMI_480i60},
	{"480i_rpt",  HDMI_480i60_16x9_rpt},
	{"480i60hz",	  HDMI_480i60_16x9},
	{"480p_4_3",  HDMI_480p60},
	{"480p_rpt",  HDMI_480p60_16x9_rpt},
	{"480p60hz",	  HDMI_480p60_16x9},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"480p59hz",  HDMI_480p60_16x9},
#endif
	{"576i_4_3",  HDMI_576i50},
	{"576i_rpt",  HDMI_576i50_16x9_rpt},
	{"576i50hz",	  HDMI_576i50_16x9},
	{"576p_4_3",  HDMI_576p50},
	{"576p_rpt",  HDMI_576p50_16x9_rpt},
	{"576p50hz",	  HDMI_576p50_16x9},
	{"720p50hz",  HDMI_720p50},
	{"720p60hz",	  HDMI_720p60},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"720p59hz",  HDMI_720p60},
#endif
	{"1080i50hz", HDMI_1080i50},
	{"1080i60hz",	 HDMI_1080i60},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"1080i59hz", HDMI_1080i60},
#endif
	{"1080p50hz", HDMI_1080p50},
	{"1080p30hz", HDMI_1080p30},
	{"1080p24hz", HDMI_1080p24},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"1080p23hz", HDMI_1080p24},
#endif
	{"1080p60hz",	 HDMI_1080p60},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"1080p59hz", HDMI_1080p60},
#endif
	{"2160p30hz",  HDMI_4k2k_30},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"4k2k29hz",  HDMI_4k2k_30},
#endif
	{"2160p25hz",  HDMI_4k2k_25},
	{"2160p24hz",  HDMI_4k2k_24},
#ifdef CONFIG_AML_VOUT_FRAMERATE_AUTOMATION
	{"4k2k23hz",  HDMI_4k2k_24},
#endif
	{"smpte24hz", HDMI_4k2k_smpte_24},
	{"2160p60hz", HDMI_4k2k_60},
	{"2160p50hz", HDMI_4k2k_50},
	{"2160p60hz420", HDMI_4k2k_60_y420},
	{"2160p50hz420", HDMI_4k2k_50_y420},
	{"640x480p60hz", HDMIV_640x480p60hz},
	{"800x480p60hz", HDMIV_800x480p60hz},
	{"480x800p60hz", HDMIV_480x800p60hz},
	{"800x600p60hz", HDMIV_800x600p60hz},
	{"1024x600p60hz", HDMIV_1024x600p60hz},
	{"1024x768p60hz", HDMIV_1024x768p60hz},
	{"1280x800p60hz", HDMIV_1280x800p60hz},
	{"1280x1024p60hz", HDMIV_1280x1024p60hz},
/*
/sys/class/display # echo 1280x1024p60hz > mode
vout_serve: vmode set to 1280x1024p60hz

vout_serve: disable HDMI PHY as soon as possible
tv_vout: tv_set_current_vmode[671]fps_target_mode=43
tv_vout: mode is 43,sync_duration_den=1,sync_duration_num=60
tv_vout: TV mode 1280x1024p60hz selected.
tv_vout: new mode =1280x1024 set ok
vout_serve: vinfo mode is: 1280x1024
vout_serve: new mode 1280x1024p60hz
 set ok
hdmitx: video: get current mode: 1280x1024
 */
	{"1280x1024", HDMIV_1280x1024p60hz}, /* alias of "1280x1024p60hz" */
	{"1360x768p60hz", HDMIV_1360x768p60hz},
	{"1366x768p60hz", HDMIV_1366x768p60hz},
	{"1440x900p60hz", HDMIV_1440x900p60hz},
	{"1600x900p60hz", HDMIV_1600x900p60hz},
	{"1600x1200p60hz", HDMIV_1600x1200p60hz},
	{"1680x1050p60hz", HDMIV_1680x1050p60hz},
	{"1920x1200p60hz", HDMIV_1920x1200p60hz},
	{"2560x1440p60hz", HDMIV_2560x1440p60hz},
	{"2560x1600p60hz", HDMIV_2560x1600p60hz},
	{"2560x1080p60hz", HDMIV_2560x1080p60hz},
	{"3440x1440p60hz", HDMIV_3440x1440p60hz},
	{"480x320p60hz", HDMIV_480x320p60hz},
	{"custombuilt", HDMIV_CUSTOMBUILT},
};

int hdmitx_edid_VIC_support(enum hdmi_vic vic)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dispmode_vic_tab); i++) {
		if (vic == dispmode_vic_tab[i].VIC)
			return 1;
	}

	return 0;
}

enum hdmi_vic hdmitx_edid_vic_tab_map_vic(const char *disp_mode)
{
	enum hdmi_vic vic = HDMI_Unkown;
	int i;

	for (i = 0; i < ARRAY_SIZE(dispmode_vic_tab); i++) {
		if (strcmp(disp_mode, dispmode_vic_tab[i].disp_mode) == 0) {
			vic = dispmode_vic_tab[i].VIC;
			break;
		}
	}

	if (vic == HDMI_Unkown)
		hdmi_print(INF, EDID "not find mapped vic\n");

	return vic;
}

const char *hdmitx_edid_vic_tab_map_string(enum hdmi_vic vic)
{
	int i;
	const char *disp_str = NULL;

	for (i = 0; i < ARRAY_SIZE(dispmode_vic_tab); i++) {
		if (vic == dispmode_vic_tab[i].VIC) {
			disp_str = dispmode_vic_tab[i].disp_mode;
			break;
		}
	}

	return disp_str;
}

/* force_flag: 0 means check with RX's edid */
/* 1 means no check wich RX's edid */
enum hdmi_vic hdmitx_edid_get_VIC(struct hdmitx_dev *hdmitx_device,
	const char *disp_mode, char force_flag)
{
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);
	int  j;
	enum hdmi_vic vic = hdmitx_edid_vic_tab_map_vic(disp_mode);

	if (vic != HDMI_Unkown) {
		if (force_flag == 0) {
			for (j = 0 ; j < pRXCap->VIC_count ; j++) {
				if (pRXCap->VIC[j] == vic)
					break;
			}
			if (j >= pRXCap->VIC_count)
				vic = HDMI_Unkown;
		}
	}
	return vic;
}

const char *hdmitx_edid_get_native_VIC(struct hdmitx_dev *hdmitx_device)
{
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);

	return hdmitx_edid_vic_tab_map_string(pRXCap->native_VIC);
}

/* Clear HDMI Hardware Module EDID RAM and EDID Buffer */
void hdmitx_edid_ram_buffer_clear(struct hdmitx_dev *hdmitx_device)
{
	unsigned int i = 0;

	/* Clear HDMI Hardware Module EDID RAM */
	hdmitx_device->HWOp.CntlDDC(hdmitx_device, DDC_EDID_CLEAR_RAM, 0);

	/* Clear EDID Buffer */
	for (i = 0; i < EDID_MAX_BLOCK*128; i++)
		hdmitx_device->EDID_buf[i] = 0;
	for (i = 0; i < EDID_MAX_BLOCK*128; i++)
		hdmitx_device->EDID_buf1[i] = 0;
}

/* Clear the Parse result of HDMI Sink's EDID. */
void hdmitx_edid_clear(struct hdmitx_dev *hdmitx_device)
{
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);
	hdmitx_device->vic_count = 0;
	pRXCap->VIC_count = 0;
	pRXCap->AUD_count = 0;
	/* Note: in most cases, we think that rx is tv and the default
	 * IEEEOUI is HDMI Identifier */
	pRXCap->IEEEOUI = 0x000c03;
	pRXCap->HF_IEEEOUI = 0x0;
	pRXCap->scdc_present = 0x0;
	pRXCap->scdc_rr_capable = 0x0;
	pRXCap->lte_340mcsc_scramble = 0x0;
	pRXCap->lte_340mcsc_scramble = 0x0;
	pRXCap->native_Mode = 0;
	pRXCap->native_VIC = 0xff;
	pRXCap->RxSpeakerAllocation = 0;
	hdmitx_device->hdmi_info.vsdb_phy_addr.a = 0;
	hdmitx_device->hdmi_info.vsdb_phy_addr.b = 0;
	hdmitx_device->hdmi_info.vsdb_phy_addr.c = 0;
	hdmitx_device->hdmi_info.vsdb_phy_addr.d = 0;
	hdmitx_device->hdmi_info.vsdb_phy_addr.valid = 0;
	memset(&vsdb_local, 0, sizeof(struct vsdb_phyaddr));
	pRXCap->Video_Latency = 0;
	pRXCap->Audio_Latency = 0;
	pRXCap->Interlaced_Video_Latency = 0;
	pRXCap->Interlaced_Audio_Latency = 0;
	pRXCap->threeD_present = 0;
	pRXCap->threeD_Multi_present = 0;
	pRXCap->hdmi_vic_LEN = 0;
	pRXCap->HDMI_3D_LEN = 0;
	pRXCap->threeD_Structure_ALL_15_0 = 0;
	pRXCap->threeD_MASK_15_0 = 0;
	memset(pRXCap->ReceiverBrandName, 0,
		sizeof(pRXCap->ReceiverBrandName));
	memset(pRXCap->ReceiverProductName, 0,
		sizeof(pRXCap->ReceiverProductName));
	memset(&hdmitx_device->RXCap.support_3d_format[0], 0,
		sizeof(hdmitx_device->RXCap.support_3d_format));
	memset(&hdmitx_device->EDID_hash[0], 0,
		sizeof(hdmitx_device->EDID_hash));
}

/*
 * print one block data of edid
 */
#define TMP_EDID_BUF_SIZE	(256+8)
static void hdmitx_edid_blk_print(unsigned char *blk, unsigned int blk_idx)
{
	unsigned int i, pos;
	unsigned char *tmp_buf = NULL;

	tmp_buf = kmalloc(TMP_EDID_BUF_SIZE, GFP_KERNEL);
	if (!tmp_buf) {
		pr_info("hdmitx: no mem\n");
		return;
	}

	memset(tmp_buf, 0, sizeof(TMP_EDID_BUF_SIZE));
	hdmi_print(INF, EDID "blk%d raw data\n", blk_idx);
	for (i = 0, pos = 0; i < 128; i++) {
		pos += sprintf(tmp_buf + pos, "%02x", blk[i]);
		if (((i+1) & 0x1f) == 0)    /* print 32bytes a line */
			pos += sprintf(tmp_buf + pos, "\n");
	}
	pos += sprintf(tmp_buf + pos, "\n");
	pr_info("%s\n", tmp_buf);
	kfree(tmp_buf);
}

/*
 * check EDID buf contains valid block numbers
 */
static unsigned int hdmitx_edid_check_valid_blocks(unsigned char *buf)
{
	unsigned int valid_blk_no = 0;
	unsigned int i = 0, j = 0;
	unsigned int tmp_chksum = 0;
	for (j = 0; j < EDID_MAX_BLOCK; j++) {
		for (i = 0; i < 128; i++)
			tmp_chksum += buf[i + j*128];
		if (tmp_chksum != 0) {
			valid_blk_no++;
			if ((tmp_chksum & 0xff) == 0)
				hdmi_print(INF, EDID "check sum valid\n");
			else
				hdmi_print(INF, EDID "check sum invalid\n");
		}
		tmp_chksum = 0;
	}
	return valid_blk_no;
}

/*
 * suppose DDC read EDID two times successfully,
 * then compare EDID_buf and EDID_buf1.
 * if same, just print out EDID_buf raw data, else print out 2 buffers
 */
void hdmitx_edid_buf_compare_print(struct hdmitx_dev *hdmitx_device)
{
	unsigned int i = 0;
	unsigned int err_no = 0;
	unsigned char *buf0 = hdmitx_device->EDID_buf;
	unsigned char *buf1 = hdmitx_device->EDID_buf1;
	unsigned int valid_blk_no = 0;
	unsigned int blk_idx = 0;

	for (i = 0; i < EDID_MAX_BLOCK*128; i++) {
		if (buf0[i] != buf1[i])
			err_no++;
	}

	if (err_no == 0) {
		/* calculate valid edid block numbers */
		valid_blk_no = hdmitx_edid_check_valid_blocks(buf0);

		if (valid_blk_no == 0)
			hdmi_print(ERR, EDID "raw data are all zeroes\n");
		else {
			for (blk_idx = 0; blk_idx < valid_blk_no; blk_idx++)
				hdmitx_edid_blk_print(&buf0[blk_idx*128],
					blk_idx);
		}
	} else {
		hdmi_print(ERR, EDID "%d errors between two reading\n", err_no);
		valid_blk_no = hdmitx_edid_check_valid_blocks(buf0);
		for (blk_idx = 0; blk_idx < valid_blk_no; blk_idx++)
			hdmitx_edid_blk_print(&buf0[blk_idx*128], blk_idx);

		valid_blk_no = hdmitx_edid_check_valid_blocks(buf1);
		for (blk_idx = 0; blk_idx < valid_blk_no; blk_idx++)
			hdmitx_edid_blk_print(&buf1[blk_idx*128], blk_idx);
	}
}

int hdmitx_edid_dump(struct hdmitx_dev *hdmitx_device, char *buffer,
	int buffer_len)
{
	int i, pos = 0;
	struct rx_cap *pRXCap = &(hdmitx_device->RXCap);

	pos += snprintf(buffer+pos, buffer_len-pos,
		"Rx Brand Name: %s\n", pRXCap->ReceiverBrandName);
	pos += snprintf(buffer+pos, buffer_len-pos,
		"Rx Product Name: %s\n", pRXCap->ReceiverProductName);

	pos += snprintf(buffer+pos, buffer_len-pos,
		"EDID block number: 0x%x\n", hdmitx_device->EDID_buf[0x7e]);
	pos += snprintf(buffer+pos, buffer_len-pos,
		"blk0 chksum: 0x%02x\n", pRXCap->blk0_chksum);

	pos += snprintf(buffer+pos, buffer_len-pos,
		"Source Physical Address[a.b.c.d]: %x.%x.%x.%x\n",
		hdmitx_device->hdmi_info.vsdb_phy_addr.a,
		hdmitx_device->hdmi_info.vsdb_phy_addr.b,
		hdmitx_device->hdmi_info.vsdb_phy_addr.c,
		hdmitx_device->hdmi_info.vsdb_phy_addr.d);

	pos += snprintf(buffer+pos, buffer_len-pos,
		"native Mode %x, VIC (native %d):\n",
		pRXCap->native_Mode, pRXCap->native_VIC);

	pos += snprintf(buffer+pos, buffer_len-pos,
		"ColorDeepSupport %x, MaxTMDSClock %d\n",
		pRXCap->ColorDeepSupport, pRXCap->Max_TMDS_Clock);

	for (i = 0 ; i < pRXCap->VIC_count ; i++) {
		pos += snprintf(buffer+pos, buffer_len-pos, "%d ",
		pRXCap->VIC[i]);
	}
	pos += snprintf(buffer+pos, buffer_len-pos, "\n");
	pos += snprintf(buffer+pos, buffer_len-pos,
		"Audio {format, channel, freq, cce}\n");
	for (i = 0; i < pRXCap->AUD_count; i++) {
		pos += snprintf(buffer+pos, buffer_len-pos,
			"{%d, %d, %x, %x}\n",
			pRXCap->RxAudioCap[i].audio_format_code,
			pRXCap->RxAudioCap[i].channel_num_max,
			pRXCap->RxAudioCap[i].freq_cc,
			pRXCap->RxAudioCap[i].cc3);
	}
	pos += snprintf(buffer+pos, buffer_len-pos,
		"Speaker Allocation: %x\n", pRXCap->RxSpeakerAllocation);
	pos += snprintf(buffer+pos, buffer_len-pos, "Vendor: 0x%x\n",
		pRXCap->IEEEOUI);
	pos += snprintf(buffer+pos, buffer_len-pos, "Vendor2: 0x%x\n",
		pRXCap->HF_IEEEOUI);
	pos += snprintf(buffer+pos, buffer_len-pos, "SCDC: %x\n",
		pRXCap->scdc_present);
	pos += snprintf(buffer+pos, buffer_len-pos, "RR_Cap: %x\n",
		pRXCap->scdc_rr_capable);
	pos += snprintf(buffer+pos, buffer_len-pos, "LTE_340M_Scramble: %x\n",
		pRXCap->lte_340mcsc_scramble);
#if 1
	pos += snprintf(buffer+pos, buffer_len-pos,
		"Rx 3D Format Support List:\n"
		"{VIC FramePacking TopBottom SidebySide}\n");
	for (i = 0 ; i < pRXCap->VIC_count; i++) {
		pos += snprintf(buffer+pos, buffer_len-pos,
			"{ %d\t%d\t%d\t%d }\n", pRXCap->VIC[i],
			pRXCap->support_3d_format[pRXCap->VIC[i]].frame_packing,
			pRXCap->support_3d_format[pRXCap->VIC[i]].
				top_and_bottom,
			pRXCap->support_3d_format[pRXCap->VIC[i]].side_by_side);
	}
#endif

	/* for checkvalue which maybe used by application to adjust
		whether edid is changed */
	pos += snprintf(buffer+pos, buffer_len-pos,
		"checkvalue: 0x%02x%02x%02x%02x\n",
			edid_checkvalue[0],
			edid_checkvalue[1],
			edid_checkvalue[2],
			edid_checkvalue[3]);

	return pos;
}

