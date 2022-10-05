/*
 * include/linux/amlogic/media/vout/lcd/lcd_unifykey.h
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

#ifndef _INC_AML_LCD_UNIFYKEY_H__
#define _INC_AML_LCD_UNIFYKEY_H__

#define LCD_UNIFYKEY_WAIT_TIMEOUT      500
#define LCD_UNIFYKEY_RETRY_INTERVAL    20   /* ms */

/* declare external unifykey function */
extern void *get_ukdev(void);
extern int key_unify_read(void *ukdev, char *keyname, unsigned char *keydata,
	unsigned int datalen, unsigned int *reallen);
extern int key_unify_size(void *ukdev, char *keyname, unsigned int *reallen);
extern int key_unify_query(void *ukdev, char *keyname,
	unsigned int *keystate, unsigned int *keypermit);
extern int key_unify_get_init_flag(void);


#define LCD_UKEY_RETRY_CNT_MAX   5
/*
 *  lcd unifykey data struct: little-endian, for example:
 *    4byte: d[0]=0x01, d[1]=0x02, d[2] = 0x03, d[3]= 0x04,
 *	   data = 0x04030201
 */

/* define lcd unifykey length */

#define LCD_UKEY_HEAD_SIZE        10
#define LCD_UKEY_HEAD_CRC32       4
#define LCD_UKEY_HEAD_DATA_LEN    2
#define LCD_UKEY_HEAD_VERSION     2
#define LCD_UKEY_HEAD_RESERVED    2

struct aml_lcd_unifykey_header_s {
	unsigned int crc32;
	unsigned short data_len;
	unsigned short version;
	unsigned short reserved;
};

/* ********************************
 * lcd
 * *********************************
 */
/* V1: 265 */
/* V2: 319 */
#define LCD_UKEY_LCD_SIZE          319

/* header (10Byte) */
/* LCD_UKEY_HEAD_SIZE */
/* basic (36Byte) */
#define LCD_UKEY_MODEL_NAME      (LCD_UKEY_HEAD_SIZE + 0)
#define LCD_UKEY_INTERFACE       (LCD_UKEY_MODEL_NAME + 30)
#define LCD_UKEY_LCD_BITS        (LCD_UKEY_MODEL_NAME + 31)
#define LCD_UKEY_SCREEN_WIDTH    (LCD_UKEY_MODEL_NAME + 32)
#define LCD_UKEY_SCREEN_HEIGHT   (LCD_UKEY_MODEL_NAME + 34)
/* timing (18Byte) */
#define LCD_UKEY_H_ACTIVE        (LCD_UKEY_MODEL_NAME + 36)/* +36 byte */
#define LCD_UKEY_V_ACTIVE        (LCD_UKEY_MODEL_NAME + 38)
#define LCD_UKEY_H_PERIOD        (LCD_UKEY_MODEL_NAME + 40)
#define LCD_UKEY_V_PERIOD        (LCD_UKEY_MODEL_NAME + 42)
#define LCD_UKEY_HS_WIDTH        (LCD_UKEY_MODEL_NAME + 44)
#define LCD_UKEY_HS_BP           (LCD_UKEY_MODEL_NAME + 46)
#define LCD_UKEY_HS_POL          (LCD_UKEY_MODEL_NAME + 48)
#define LCD_UKEY_VS_WIDTH        (LCD_UKEY_MODEL_NAME + 49)
#define LCD_UKEY_VS_BP           (LCD_UKEY_MODEL_NAME + 51)
#define LCD_UKEY_VS_POL          (LCD_UKEY_MODEL_NAME + 53)
/* customer (31Byte) */
#define LCD_UKEY_FR_ADJ_TYPE     (LCD_UKEY_MODEL_NAME + 54)/* +36+18 byte */
#define LCD_UKEY_SS_LEVEL        (LCD_UKEY_MODEL_NAME + 55)
#define LCD_UKEY_CLK_AUTO_GEN    (LCD_UKEY_MODEL_NAME + 56)
#define LCD_UKEY_PCLK            (LCD_UKEY_MODEL_NAME + 57)
#define LCD_UKEY_H_PERIOD_MIN    (LCD_UKEY_MODEL_NAME + 61)
#define LCD_UKEY_H_PERIOD_MAX    (LCD_UKEY_MODEL_NAME + 63)
#define LCD_UKEY_V_PERIOD_MIN    (LCD_UKEY_MODEL_NAME + 65)
#define LCD_UKEY_V_PERIOD_MAX    (LCD_UKEY_MODEL_NAME + 67)
#define LCD_UKEY_PCLK_MIN        (LCD_UKEY_MODEL_NAME + 69)
#define LCD_UKEY_PCLK_MAX        (LCD_UKEY_MODEL_NAME + 73)
#define LCD_UKEY_VLOCK_VAL_0     (LCD_UKEY_MODEL_NAME + 77)
#define LCD_UKEY_VLOCK_VAL_1     (LCD_UKEY_MODEL_NAME + 78)
#define LCD_UKEY_VLOCK_VAL_2     (LCD_UKEY_MODEL_NAME + 79)
#define LCD_UKEY_VLOCK_VAL_3     (LCD_UKEY_MODEL_NAME + 80)
#define LCD_UKEY_CUST_VAL_9      (LCD_UKEY_MODEL_NAME + 81)
/* interface (20Byte) */
#define LCD_UKEY_IF_ATTR_0       (LCD_UKEY_MODEL_NAME + 85)/* +36+18+31 byte */
#define LCD_UKEY_IF_ATTR_1       (LCD_UKEY_MODEL_NAME + 87)
#define LCD_UKEY_IF_ATTR_2       (LCD_UKEY_MODEL_NAME + 89)
#define LCD_UKEY_IF_ATTR_3       (LCD_UKEY_MODEL_NAME + 91)
#define LCD_UKEY_IF_ATTR_4       (LCD_UKEY_MODEL_NAME + 93)
#define LCD_UKEY_IF_ATTR_5       (LCD_UKEY_MODEL_NAME + 95)
#define LCD_UKEY_IF_ATTR_6       (LCD_UKEY_MODEL_NAME + 97)
#define LCD_UKEY_IF_ATTR_7       (LCD_UKEY_MODEL_NAME + 99)
#define LCD_UKEY_IF_ATTR_8       (LCD_UKEY_MODEL_NAME + 101)
#define LCD_UKEY_IF_ATTR_9       (LCD_UKEY_MODEL_NAME + 103)
/* ctrl (44Byte) */ /* V2 */
#define LCD_UKEY_CTRL_FLAG      (LCD_UKEY_MODEL_NAME + 105)
#define LCD_UKEY_CTRL_ATTR_0    (LCD_UKEY_MODEL_NAME + 109)
#define LCD_UKEY_CTRL_ATTR_1    (LCD_UKEY_MODEL_NAME + 111)
#define LCD_UKEY_CTRL_ATTR_2    (LCD_UKEY_MODEL_NAME + 113)
#define LCD_UKEY_CTRL_ATTR_3    (LCD_UKEY_MODEL_NAME + 115)
#define LCD_UKEY_CTRL_ATTR_4    (LCD_UKEY_MODEL_NAME + 117)
#define LCD_UKEY_CTRL_ATTR_5    (LCD_UKEY_MODEL_NAME + 119)
#define LCD_UKEY_CTRL_ATTR_6    (LCD_UKEY_MODEL_NAME + 121)
#define LCD_UKEY_CTRL_ATTR_7    (LCD_UKEY_MODEL_NAME + 123)
#define LCD_UKEY_CTRL_ATTR_8    (LCD_UKEY_MODEL_NAME + 125)
#define LCD_UKEY_CTRL_ATTR_9    (LCD_UKEY_MODEL_NAME + 127)
#define LCD_UKEY_CTRL_ATTR_10   (LCD_UKEY_MODEL_NAME + 129)
#define LCD_UKEY_CTRL_ATTR_11   (LCD_UKEY_MODEL_NAME + 131)
#define LCD_UKEY_CTRL_ATTR_12   (LCD_UKEY_MODEL_NAME + 133)
#define LCD_UKEY_CTRL_ATTR_13   (LCD_UKEY_MODEL_NAME + 135)
#define LCD_UKEY_CTRL_ATTR_14   (LCD_UKEY_MODEL_NAME + 137)
#define LCD_UKEY_CTRL_ATTR_15   (LCD_UKEY_MODEL_NAME + 139)
#define LCD_UKEY_CTRL_ATTR_16   (LCD_UKEY_MODEL_NAME + 141)
#define LCD_UKEY_CTRL_ATTR_17   (LCD_UKEY_MODEL_NAME + 143)
#define LCD_UKEY_CTRL_ATTR_18   (LCD_UKEY_MODEL_NAME + 145)
#define LCD_UKEY_CTRL_ATTR_19   (LCD_UKEY_MODEL_NAME + 147)
/* phy (10Byte) */ /* V2 */
#define LCD_UKEY_PHY_ATTR_0     (LCD_UKEY_MODEL_NAME + 149)
#define LCD_UKEY_PHY_ATTR_1     (LCD_UKEY_MODEL_NAME + 150)
#define LCD_UKEY_PHY_ATTR_2     (LCD_UKEY_MODEL_NAME + 151)
#define LCD_UKEY_PHY_ATTR_3     (LCD_UKEY_MODEL_NAME + 152)
#define LCD_UKEY_PHY_ATTR_4     (LCD_UKEY_MODEL_NAME + 153)
#define LCD_UKEY_PHY_ATTR_5     (LCD_UKEY_MODEL_NAME + 154)
#define LCD_UKEY_PHY_ATTR_6     (LCD_UKEY_MODEL_NAME + 155)
#define LCD_UKEY_PHY_ATTR_7     (LCD_UKEY_MODEL_NAME + 156)
#define LCD_UKEY_PHY_ATTR_8     (LCD_UKEY_MODEL_NAME + 157)
#define LCD_UKEY_PHY_ATTR_9     (LCD_UKEY_MODEL_NAME + 158)

#define LCD_UKEY_DATA_LEN_V1        (LCD_UKEY_MODEL_NAME + 105)
#define LCD_UKEY_DATA_LEN_V2        (LCD_UKEY_MODEL_NAME + 159)
/* power (5Byte * n) */
#define LCD_UKEY_PWR_TYPE          (0)
#define LCD_UKEY_PWR_INDEX         (1)
#define LCD_UKEY_PWR_VAL           (2)
#define LCD_UKEY_PWR_DELAY         (3)

/* ********************************
 * lcd extern
 * *********************************
 */
#define LCD_UKEY_LCD_EXT_SIZE       550

/* header (10Byte) */
/* LCD_UKEY_HEAD_SIZE */
/* basic (33Byte) */
#define LCD_UKEY_EXT_NAME           (LCD_UKEY_HEAD_SIZE + 0)
#define LCD_UKEY_EXT_INDEX          (LCD_UKEY_EXT_NAME + 30)
#define LCD_UKEY_EXT_TYPE           (LCD_UKEY_EXT_NAME + 31)
#define LCD_UKEY_EXT_STATUS         (LCD_UKEY_EXT_NAME + 32)
/* type (10Byte) */
#define LCD_UKEY_EXT_TYPE_VAL_0     (LCD_UKEY_EXT_NAME + 33)/* +33 byte */
#define LCD_UKEY_EXT_TYPE_VAL_1     (LCD_UKEY_EXT_NAME + 34)
#define LCD_UKEY_EXT_TYPE_VAL_2     (LCD_UKEY_EXT_NAME + 35)
#define LCD_UKEY_EXT_TYPE_VAL_3     (LCD_UKEY_EXT_NAME + 36)
#define LCD_UKEY_EXT_TYPE_VAL_4     (LCD_UKEY_EXT_NAME + 37)
#define LCD_UKEY_EXT_TYPE_VAL_5     (LCD_UKEY_EXT_NAME + 38)
#define LCD_UKEY_EXT_TYPE_VAL_6     (LCD_UKEY_EXT_NAME + 39)
#define LCD_UKEY_EXT_TYPE_VAL_7     (LCD_UKEY_EXT_NAME + 40)
#define LCD_UKEY_EXT_TYPE_VAL_8     (LCD_UKEY_EXT_NAME + 41)
#define LCD_UKEY_EXT_TYPE_VAL_9     (LCD_UKEY_EXT_NAME + 42)
/* init (cmd_size) */
#define LCD_UKEY_EXT_INIT           (LCD_UKEY_EXT_NAME + 43)/* +33+10 byte */
/*#define LCD_UKEY_EXT_INIT_TYPE      (0)*/
/*#define LCD_UKEY_EXT_INIT_VAL        1   //not defined */
/*#define LCD_UKEY_EXT_INIT_DELAY     (n)*/

/* ********************************
 * backlight
 * *********************************
 */
/* V1: 92 */
/* V2: 118 */
#define LCD_UKEY_BL_SIZE            118

/* header (10Byte) */
/* LCD_UKEY_HEAD_SIZE */
/* basic (30Byte) */
#define LCD_UKEY_BL_NAME            (LCD_UKEY_HEAD_SIZE + 0)
/* level (12Byte) */
#define LCD_UKEY_BL_LEVEL_UBOOT     (LCD_UKEY_BL_NAME + 30)/* +30 byte */
#define LCD_UKEY_BL_LEVEL_KERNEL    (LCD_UKEY_BL_NAME + 32)
#define LCD_UKEY_BL_LEVEL_MAX       (LCD_UKEY_BL_NAME + 34)
#define LCD_UKEY_BL_LEVEL_MIN       (LCD_UKEY_BL_NAME + 36)
#define LCD_UKEY_BL_LEVEL_MID       (LCD_UKEY_BL_NAME + 38)
#define LCD_UKEY_BL_LEVEL_MID_MAP   (LCD_UKEY_BL_NAME + 40)
/* method (8Byte) */
#define LCD_UKEY_BL_METHOD          (LCD_UKEY_BL_NAME + 42)/* +30+12 byte */
#define LCD_UKEY_BL_EN_GPIO         (LCD_UKEY_BL_NAME + 43)
#define LCD_UKEY_BL_EN_GPIO_ON      (LCD_UKEY_BL_NAME + 44)
#define LCD_UKEY_BL_EN_GPIO_OFF     (LCD_UKEY_BL_NAME + 45)
#define LCD_UKEY_BL_ON_DELAY        (LCD_UKEY_BL_NAME + 46)
#define LCD_UKEY_BL_OFF_DELAY       (LCD_UKEY_BL_NAME + 48)
/* pwm (32Byte) */
#define LCD_UKEY_BL_PWM_ON_DELAY    (LCD_UKEY_BL_NAME + 50)/* +30+12+8 byte */
#define LCD_UKEY_BL_PWM_OFF_DELAY   (LCD_UKEY_BL_NAME + 52)
#define LCD_UKEY_BL_PWM_METHOD      (LCD_UKEY_BL_NAME + 54)
#define LCD_UKEY_BL_PWM_PORT        (LCD_UKEY_BL_NAME + 55)
#define LCD_UKEY_BL_PWM_FREQ        (LCD_UKEY_BL_NAME + 56)
#define LCD_UKEY_BL_PWM_DUTY_MAX    (LCD_UKEY_BL_NAME + 60)
#define LCD_UKEY_BL_PWM_DUTY_MIN    (LCD_UKEY_BL_NAME + 61)
#define LCD_UKEY_BL_PWM_GPIO        (LCD_UKEY_BL_NAME + 62)
#define LCD_UKEY_BL_PWM_GPIO_OFF    (LCD_UKEY_BL_NAME + 63)
#define LCD_UKEY_BL_PWM2_METHOD     (LCD_UKEY_BL_NAME + 64)
#define LCD_UKEY_BL_PWM2_PORT       (LCD_UKEY_BL_NAME + 65)
#define LCD_UKEY_BL_PWM2_FREQ       (LCD_UKEY_BL_NAME + 66)
#define LCD_UKEY_BL_PWM2_DUTY_MAX   (LCD_UKEY_BL_NAME + 70)
#define LCD_UKEY_BL_PWM2_DUTY_MIN   (LCD_UKEY_BL_NAME + 71)
#define LCD_UKEY_BL_PWM2_GPIO       (LCD_UKEY_BL_NAME + 72)
#define LCD_UKEY_BL_PWM2_GPIO_OFF   (LCD_UKEY_BL_NAME + 73)
#define LCD_UKEY_BL_PWM_LEVEL_MAX   (LCD_UKEY_BL_NAME + 74)
#define LCD_UKEY_BL_PWM_LEVEL_MIN   (LCD_UKEY_BL_NAME + 76)
#define LCD_UKEY_BL_PWM2_LEVEL_MAX  (LCD_UKEY_BL_NAME + 78)
#define LCD_UKEY_BL_PWM2_LEVEL_MIN  (LCD_UKEY_BL_NAME + 80)
/* local dimming (16Byte) */ /* V2 */
#define LCD_UKEY_BL_LDIM_ROW        (LCD_UKEY_BL_NAME + 82)
#define LCD_UKEY_BL_LDIM_COL        (LCD_UKEY_BL_NAME + 83)
#define LCD_UKEY_BL_LDIM_MODE       (LCD_UKEY_BL_NAME + 84)
#define LCD_UKEY_BL_LDIM_DEV_INDEX  (LCD_UKEY_BL_NAME + 85)
#define LCD_UKEY_BL_LDIM_ATTR_4     (LCD_UKEY_BL_NAME + 86)
#define LCD_UKEY_BL_LDIM_ATTR_5     (LCD_UKEY_BL_NAME + 88)
#define LCD_UKEY_BL_LDIM_ATTR_6     (LCD_UKEY_BL_NAME + 90)
#define LCD_UKEY_BL_LDIM_ATTR_7     (LCD_UKEY_BL_NAME + 92)
#define LCD_UKEY_BL_LDIM_ATTR_8     (LCD_UKEY_BL_NAME + 94)
#define LCD_UKEY_BL_LDIM_ATTR_9     (LCD_UKEY_BL_NAME + 96)
/* customer(10Byte) */ /* V2 */
#define LCD_UKEY_BL_CUST_VAL_0      (LCD_UKEY_BL_NAME + 98)
#define LCD_UKEY_BL_CUST_VAL_1      (LCD_UKEY_BL_NAME + 100)
#define LCD_UKEY_BL_CUST_VAL_2      (LCD_UKEY_BL_NAME + 102)
#define LCD_UKEY_BL_CUST_VAL_3      (LCD_UKEY_BL_NAME + 104)
#define LCD_UKEY_BL_CUST_VAL_4      (LCD_UKEY_BL_NAME + 106)

/* ********************************
 * API
 * *********************************
 */
extern int lcd_unifykey_len_check(int key_len, int len);
extern int lcd_unifykey_check(char *key_name);
extern int lcd_unifykey_header_check(unsigned char *buf,
		struct aml_lcd_unifykey_header_s *header);
extern int lcd_unifykey_get(char *key_name,
		unsigned char *buf, int *len);
extern int lcd_unifykey_check_no_header(char *key_name);
extern int lcd_unifykey_get_no_header(char *key_name,
		unsigned char *buf, int *len);
extern void lcd_unifykey_print(void);

#endif
