/* linux/drivers/media/video/samsung/tv20/s5pc100/regs/regs-cec.h
 *
 * Copyright (c) 2009 Samsung Electronics
 *      http://www.samsungsemi.com/
 *
 * CEC register header file for Samsung TVOut driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/*
 * Source:
 * 	http://code.google.com/p/openwrt-for-embedded/source/browse/trunk/target/linux/s5pv210/files-2.6.35/arch/arm/plat-s5p/include/plat/regs-cec.h?r=56
 */

#ifndef __REGS_CEC_H
#define __REGS_CEC_H

#define HDMIDP_CECREG(x) (x)

#define S5P_CEC_STATUS_0                    HDMIDP_CECREG(0x0000)
#define S5P_CEC_STATUS_1                    HDMIDP_CECREG(0x0004)
#define S5P_CEC_STATUS_2                    HDMIDP_CECREG(0x0008)
#define S5P_CEC_STATUS_3                    HDMIDP_CECREG(0x000C)
#define S5P_CEC_IRQ_MASK                    HDMIDP_CECREG(0x0010)
#define S5P_CEC_IRQ_CLEAR                   HDMIDP_CECREG(0x0014)
#define S5P_CEC_LOGIC_ADDR                  HDMIDP_CECREG(0x0020)
#define S5P_CEC_DIVISOR_0                   HDMIDP_CECREG(0x0030)
#define S5P_CEC_DIVISOR_1                   HDMIDP_CECREG(0x0034)
#define S5P_CEC_DIVISOR_2                   HDMIDP_CECREG(0x0038)
#define S5P_CEC_DIVISOR_3                   HDMIDP_CECREG(0x003C)

#define S5P_CEC_TX_CTRL                     HDMIDP_CECREG(0x0040)
#define S5P_CEC_TX_BYTES                    HDMIDP_CECREG(0x0044)
#define S5P_CEC_TX_STAT0                    HDMIDP_CECREG(0x0060)
#define S5P_CEC_TX_STAT1                    HDMIDP_CECREG(0x0064)
#define S5P_CEC_TX_BUFF0                    HDMIDP_CECREG(0x0080)
#define S5P_CEC_TX_BUFF1                    HDMIDP_CECREG(0x0084)
#define S5P_CEC_TX_BUFF2                    HDMIDP_CECREG(0x0088)
#define S5P_CEC_TX_BUFF3                    HDMIDP_CECREG(0x008C)
#define S5P_CEC_TX_BUFF4                    HDMIDP_CECREG(0x0090)
#define S5P_CEC_TX_BUFF5                    HDMIDP_CECREG(0x0094)
#define S5P_CEC_TX_BUFF6                    HDMIDP_CECREG(0x0098)
#define S5P_CEC_TX_BUFF7                    HDMIDP_CECREG(0x009C)
#define S5P_CEC_TX_BUFF8                    HDMIDP_CECREG(0x00A0)
#define S5P_CEC_TX_BUFF9                    HDMIDP_CECREG(0x00A4)
#define S5P_CEC_TX_BUFF10                   HDMIDP_CECREG(0x00A8)
#define S5P_CEC_TX_BUFF11                   HDMIDP_CECREG(0x00AC)
#define S5P_CEC_TX_BUFF12                   HDMIDP_CECREG(0x00B0)
#define S5P_CEC_TX_BUFF13                   HDMIDP_CECREG(0x00B4)
#define S5P_CEC_TX_BUFF14                   HDMIDP_CECREG(0x00B8)
#define S5P_CEC_TX_BUFF15                   HDMIDP_CECREG(0x00BC)

#define S5P_CEC_RX_CTRL                     HDMIDP_CECREG(0x00C0)
#define S5P_CEC_RX_STAT0                    HDMIDP_CECREG(0x00E0)
#define S5P_CEC_RX_STAT1                    HDMIDP_CECREG(0x00E4)
#define S5P_CEC_RX_BUFF0                    HDMIDP_CECREG(0x0100)
#define S5P_CEC_RX_BUFF1                    HDMIDP_CECREG(0x0104)
#define S5P_CEC_RX_BUFF2                    HDMIDP_CECREG(0x0108)
#define S5P_CEC_RX_BUFF3                    HDMIDP_CECREG(0x010C)
#define S5P_CEC_RX_BUFF4                    HDMIDP_CECREG(0x0110)
#define S5P_CEC_RX_BUFF5                    HDMIDP_CECREG(0x0114)
#define S5P_CEC_RX_BUFF6                    HDMIDP_CECREG(0x0118)
#define S5P_CEC_RX_BUFF7                    HDMIDP_CECREG(0x011C)
#define S5P_CEC_RX_BUFF8                    HDMIDP_CECREG(0x0120)
#define S5P_CEC_RX_BUFF9                    HDMIDP_CECREG(0x0124)
#define S5P_CEC_RX_BUFF10                   HDMIDP_CECREG(0x0128)
#define S5P_CEC_RX_BUFF11                   HDMIDP_CECREG(0x012C)
#define S5P_CEC_RX_BUFF12                   HDMIDP_CECREG(0x0130)
#define S5P_CEC_RX_BUFF13                   HDMIDP_CECREG(0x0134)
#define S5P_CEC_RX_BUFF14                   HDMIDP_CECREG(0x0138)
#define S5P_CEC_RX_BUFF15                   HDMIDP_CECREG(0x013C)

#define S5P_CEC_RX_FILTER_CTRL              HDMIDP_CECREG(0x0180)
#define S5P_CEC_RX_FILTER_TH                HDMIDP_CECREG(0x0184)

#define S5P_CEC_IRQ_TX_DONE                 (1<<0)
#define S5P_CEC_IRQ_TX_ERROR                (1<<1)
#define S5P_CEC_IRQ_RX_DONE                 (1<<4)
#define S5P_CEC_IRQ_RX_ERROR                (1<<5)

#define S5P_CEC_TX_CTRL_START               (1<<0)
#define S5P_CEC_TX_CTRL_BCAST               (1<<1)
#define S5P_CEC_TX_CTRL_RETRY               (0x04<<4)
#define S5P_CEC_TX_CTRL_RESET               (1<<7)

#define S5P_CEC_RX_CTRL_ENABLE              (1<<0)
#define S5P_CEC_RX_CTRL_RESET               (1<<7)

#define S5P_CEC_LOGIC_ADDR_MASK             0x0F

#endif  /* __REGS_CEC_H */
 
