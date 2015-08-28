/* linux/arch/arm/mach-exynos/pmu-exynos5422.c
 *
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS5422 - EXYNOS PMU(Power Management Unit) support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/bug.h>

#include <mach/regs-clock.h>
#include <mach/regs-pmu.h>
#include <mach/pmu.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/topology.h>
#include "common.h"

#define REG_CPU_STATE_ADDR     (S5P_VA_SYSRAM_NS + 0x28)

static struct exynos_pmu_conf *exynos_pmu_config;

#ifdef CONFIG_SOC_EXYNOS5422_REV_0
/* PMU Table v0.6 */
static struct exynos_pmu_conf exynos5422_pmu_config[] = {
	/* .reg = address, .val = ( AFTR, LPA, DSTOP, SLEEP ) */
	{ EXYNOS5422_ARM_CORE0_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1000 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE0_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1004 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1008 */
	{ EXYNOS5422_ARM_CORE1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1010 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE1_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1014 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1018 */
	{ EXYNOS5422_ARM_CORE2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1020 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE2_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1024 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE2_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1028 */
	{ EXYNOS5422_ARM_CORE3_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1030 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE3_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1034 */
	{ EXYNOS5422_DIS_IRQ_ARM_CORE3_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1038 */
	{ EXYNOS5422_KFC_CORE0_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1040 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE0_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1044 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1048 */
	{ EXYNOS5422_KFC_CORE1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1050 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE1_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1054 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1058 */
	{ EXYNOS5422_KFC_CORE2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1060 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE2_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1064 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE2_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1068 */
	{ EXYNOS5422_KFC_CORE3_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1070 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE3_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1074 */
	{ EXYNOS5422_DIS_IRQ_KFC_CORE3_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_1078 */
	{ EXYNOS5422_ISP_ARM_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1090 */
	{ EXYNOS5422_DIS_IRQ_ISP_ARM_LOCAL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1094 */
	{ EXYNOS5422_DIS_IRQ_ISP_ARM_CENTRAL_SYS_PWR_REG,	{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1098 */
	{ EXYNOS5422_ARM_COMMON_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x0 } }, /* 0x1004_10A0 */
	{ EXYNOS5422_KFC_COMMON_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x0 } }, /* 0x1004_10B0 */
	{ EXYNOS5422_ARM_L2_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x0 } }, /* 0x1004_10C0 */
	{ EXYNOS5422_KFC_L2_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x0 } }, /* 0x1004_10D0 */
	{ EXYNOS5422_CMU_CPU_ACLKSTOP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_10E0 */
	{ EXYNOS5422_CMU_CPU_SCLKSTOP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_10E4 */
	{ EXYNOS5422_CMU_KFC_ACLKSTOP_SYS_PWR_REG,		{ 0x0, 0x0, 0x0, 0x0 } }, /* 0x1004_10F0 */
	{ EXYNOS5422_CMU_ACLKSTOP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1100 */
	{ EXYNOS5422_CMU_SCLKSTOP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x1 } }, /* 0x1004_1104 */
	{ EXYNOS5422_CMU_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_110C */
	{ EXYNOS5422_CMU_ACLKSTOP_COREBLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1120 */
	{ EXYNOS5422_CMU_SCLKSTOP_COREBLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1124 */
	{ EXYNOS5422_CMU_RESET_COREBLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_112C */
	{ EXYNOS5422_DRAM_FREQ_DOWN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x1 } }, /* 0x1004_1130 */
	{ EXYNOS5422_DDRPHY_DLLOFF_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x1 } }, /* 0x1004_1134 */
	{ EXYNOS5422_DDRPHY_DLLLOCK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x1 } }, /* 0x1004_1138 */
	{ EXYNOS5422_APLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1140 */
	{ EXYNOS5422_MPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1144 */
	{ EXYNOS5422_VPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1148 */
	{ EXYNOS5422_EPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_114C */
	{ EXYNOS5422_BPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1150 */
	{ EXYNOS5422_CPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1154 */
	{ EXYNOS5422_DPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1158 */
	{ EXYNOS5422_IPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_115C */
	{ EXYNOS5422_KPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1160 */
	{ EXYNOS5422_MPLLUSER_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1164 */
	{ EXYNOS5422_BPLLUSER_SYSCLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1170 */
	{ EXYNOS5422_RPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1174 */
	{ EXYNOS5422_SPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1178 */
	{ EXYNOS5422_TOP_BUS_SYS_PWR_REG,			{ 0x3, 0x0, 0x0, 0x0 } }, /* 0x1004_1180 */
	{ EXYNOS5422_TOP_RETENTION_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x1 } }, /* 0x1004_1184 */
	{ EXYNOS5422_TOP_PWR_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x0 } }, /* 0x1004_1188 */
	{ EXYNOS5422_TOP_BUS_COREBLK_SYS_PWR_REG,		{ 0x3, 0x0, 0x0, 0x0 } }, /* 0x1004_1190 */
	{ EXYNOS5422_TOP_RETENTION_COREBLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x1 } }, /* 0x1004_1194 */
	{ EXYNOS5422_TOP_PWR_COREBLK_SYS_PWR_REG,		{ 0x3, 0x3, 0x3, 0x0 } }, /* 0x1004_1198 */
	{ EXYNOS5422_LOGIC_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_11A0 */
	{ EXYNOS5422_OSCCLK_GATE_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x1 } }, /* 0x1004_11A4 */
	{ EXYNOS5422_LOGIC_RESET_COREBLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_11B0 */
	{ EXYNOS5422_OSCCLK_GATE_COREBLK_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_11B4 */
	{ EXYNOS5422_INTRAM_MEM_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x3 } }, /* 0x1004_11B8 */
	{ EXYNOS5422_INTROM_MEM_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x3 } }, /* 0x1004_11BC */
	{ EXYNOS5422_PAD_RETENTION_DRAM_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1200 */
	{ EXYNOS5422_PAD_RETENTION_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1204 */
	{ EXYNOS5422_PAD_RETENTION_JTAG_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1208 */
	{ EXYNOS5422_PAD_RETENTION_GPIO_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1210 */
	{ EXYNOS5422_PAD_RETENTION_UART_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1214 */
	{ EXYNOS5422_PAD_RETENTION_MMCA_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1218 */
	{ EXYNOS5422_PAD_RETENTION_MMCB_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_121C */
	{ EXYNOS5422_PAD_RETENTION_MMCC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1220 */
	{ EXYNOS5422_PAD_RETENTION_HSI_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1224 */
	{ EXYNOS5422_PAD_RETENTION_EBIA_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1228 */
	{ EXYNOS5422_PAD_RETENTION_EBIB_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_122C */
	{ EXYNOS5422_PAD_RETENTION_SPI_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1230 */
	{ EXYNOS5422_PAD_RETENTION_DRAM_COREBLK_SYS_PWR_REG,	{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1234 */
	{ EXYNOS5422_PAD_ISOLATION_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1240 */
	{ EXYNOS5422_PAD_ISOLATION_COREBLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1250 */
	{ EXYNOS5422_PAD_ALV_SEL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1260 */
	{ EXYNOS5422_XUSBXTI_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1280 */
	{ EXYNOS5422_XXTI_SYS_PWR_REG,				{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1284 */
	{ EXYNOS5422_EXT_REGULATOR_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_12C0 */
	{ EXYNOS5422_GPIO_MODE_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1300 */
	{ EXYNOS5422_GPIO_MODE_COREBLK_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1320 */
	{ EXYNOS5422_GPIO_MODE_MAU_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1340 */
	{ EXYNOS5422_TOP_ASB_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0 } }, /* 0x1004_1344 */
	{ EXYNOS5422_TOP_ASB_ISOLATION_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1348 */
	{ EXYNOS5422_SCALER_SYS_PWR_REG,			{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_1400 */
	{ EXYNOS5422_ISP_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_1404 */
	{ EXYNOS5422_MFC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_1408 */
	{ EXYNOS5422_G3D_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_140C */
	{ EXYNOS5422_DISP1_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_1410 */
	{ EXYNOS5422_MAU_SYS_PWR_REG,				{ 0x7, 0x7, 0x0, 0x0 } }, /* 0x1004_1414 */
	{ EXYNOS5422_G2D_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_1418 */
	{ EXYNOS5422_MSC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_141C */
	{ EXYNOS5422_FSYS_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0 } }, /* 0x1004_1420 */
	{ EXYNOS5422_FSYS2_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0 } }, /* 0x1004_1424 */
	{ EXYNOS5422_PSGEN_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0 } }, /* 0x1004_1428 */
	{ EXYNOS5422_PERIC_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0 } }, /* 0x1004_142C */
	{ EXYNOS5422_WCORE_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_1430 */
	{ EXYNOS5422_CMU_CLKSTOP_SCALER_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1480 */
	{ EXYNOS5422_CMU_CLKSTOP_ISP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1484 */
	{ EXYNOS5422_CMU_CLKSTOP_MFC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1488 */
	{ EXYNOS5422_CMU_CLKSTOP_G3D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_148C */
	{ EXYNOS5422_CMU_CLKSTOP_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1490 */
	{ EXYNOS5422_CMU_CLKSTOP_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1494 */
	{ EXYNOS5422_CMU_CLKSTOP_G2D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1498 */
	{ EXYNOS5422_CMU_CLKSTOP_MSC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_149C */
	{ EXYNOS5422_CMU_CLKSTOP_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14A0 */
	{ EXYNOS5422_CMU_CLKSTOP_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14A4 */
	{ EXYNOS5422_CMU_CLKSTOP_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14A8 */
	{ EXYNOS5422_CMU_CLKSTOP_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14AC */
	{ EXYNOS5422_CMU_CLKSTOP_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14B0 */
	{ EXYNOS5422_CMU_SYSCLK_TOPPWR_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14BC */
	{ EXYNOS5422_CMU_SYSCLK_SCALER_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14C0 */
	{ EXYNOS5422_CMU_SYSCLK_ISP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14C4 */
	{ EXYNOS5422_CMU_SYSCLK_MFC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14C8 */
	{ EXYNOS5422_CMU_SYSCLK_G3D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14CC */
	{ EXYNOS5422_CMU_SYSCLK_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14D0 */
	{ EXYNOS5422_CMU_SYSCLK_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_14D4 */
	{ EXYNOS5422_CMU_SYSCLK_G2D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14D8 */
	{ EXYNOS5422_CMU_SYSCLK_MSC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14DC */
	{ EXYNOS5422_CMU_SYSCLK_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14E0 */
	{ EXYNOS5422_CMU_SYSCLK_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14E4 */
	{ EXYNOS5422_CMU_SYSCLK_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14E8 */
	{ EXYNOS5422_CMU_SYSCLK_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14EC */
	{ EXYNOS5422_CMU_SYSCLK_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14F0 */
	{ EXYNOS5422_CMU_SYSCLK_COREBLK_TOPPWR_SYS_PWR_REG,	{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_14F4 */
	{ EXYNOS5422_CMU_RESET_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1570 */
	{ EXYNOS5422_CMU_RESET_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1574 */
	{ EXYNOS5422_CMU_RESET_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1578 */
	{ EXYNOS5422_CMU_RESET_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_157C */
	{ EXYNOS5422_CMU_RESET_SCALER_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1580 */
	{ EXYNOS5422_CMU_RESET_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1584 */
	{ EXYNOS5422_CMU_RESET_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1588 */
	{ EXYNOS5422_CMU_RESET_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_158C */
	{ EXYNOS5422_CMU_RESET_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1590 */
	{ EXYNOS5422_CMU_RESET_MAU_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0 } }, /* 0x1004_1594 */
	{ EXYNOS5422_CMU_RESET_G2D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_1598 */
	{ EXYNOS5422_CMU_RESET_MSC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_159C */
	{ EXYNOS5422_CMU_RESET_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_15A0 */
	{ EXYNOS5422_CAM_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0 } }, /* 0x1004_5000 */
	{ EXYNOS5422_CMU_CLKSTOP_CAM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_5010 */
	{ EXYNOS5422_CMU_SYSCLK_CAM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_5020 */
	{ EXYNOS5422_CMU_RESET_CAM_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0 } }, /* 0x1004_5030 */
	{ PMU_TABLE_END,},
};
#else
static struct exynos_pmu_conf exynos5422_pmu_config[] = {
	/* .reg = address, .val = ( AFTR, LPA, SLEEP ) */
	{ EXYNOS5_ARM_CORE0_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1000 */
	{ EXYNOS5_DIS_IRQ_ARM_CORE0_LOCAL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1004 */
	{ EXYNOS5_DIS_IRQ_ARM_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1008 */
	{ EXYNOS5_ARM_CORE1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1010 */
	{ EXYNOS5_DIS_IRQ_ARM_CORE1_LOCAL_SYS_PWR_REG,		{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1014 */
	{ EXYNOS5_DIS_IRQ_ARM_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1018 */
	{ EXYNOS54XX_ARM_CORE2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1020 */
	{ EXYNOS54XX_DIS_IRQ_ARM_CORE2_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1024 */
	{ EXYNOS54XX_DIS_IRQ_ARM_CORE2_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1028 */
	{ EXYNOS54XX_ARM_CORE3_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1030 */
	{ EXYNOS54XX_DIS_IRQ_ARM_CORE3_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1034 */
	{ EXYNOS54XX_DIS_IRQ_ARM_CORE3_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1038 */
	{ EXYNOS54XX_KFC_CORE0_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1040 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE0_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1044 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE0_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1048 */
	{ EXYNOS54XX_KFC_CORE1_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1050 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE1_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1054 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE1_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1058 */
	{ EXYNOS54XX_KFC_CORE2_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1060 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE2_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1064 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE2_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1068 */
	{ EXYNOS54XX_KFC_CORE3_SYS_PWR_REG,			{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1070 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE3_LOCAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1074 */
	{ EXYNOS54XX_DIS_IRQ_KFC_CORE3_CENTRAL_SYS_PWR_REG,	{ 0x0, 0x0, 0x0, 0x0} }, /* 0x1004_1078 */
	{ EXYNOS54XX_ISP_ARM_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1090 */
	{ EXYNOS54XX_DIS_IRQ_ISP_ARM_LOCAL_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1094 */
	{ EXYNOS54XX_DIS_IRQ_ISP_ARM_CENTRAL_SYS_PWR_REG,	{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1098 */
	{ EXYNOS54XX_ARM_COMMON_SYS_PWR_REG,			{ 0x0, 0x3, 0x3, 0x0} }, /* 0x1004_10A0 */
	{ EXYNOS54XX_KFC_COMMON_SYS_PWR_REG,			{ 0x0, 0x3, 0x3, 0x0} }, /* 0x1004_10B0 */
	{ EXYNOS5_ARM_L2_SYS_PWR_REG,				{ 0x0, 0x3, 0x3, 0x0} }, /* 0x1004_10C0 */
	{ EXYNOS54XX_KFC_L2_SYS_PWR_REG,			{ 0x0, 0x3, 0x3, 0x0} }, /* 0x1004_10D0 */
	{ EXYNOS5422_CMU_CPU_ACLKSTOP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_10E0 */
	{ EXYNOS5422_CMU_CPU_SCLKSTOP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_10E4 */
	{ EXYNOS5422_CMU_KFC_ACLKSTOP_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_10F0 */
	{ EXYNOS5_CMU_ACLKSTOP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1100 */
	{ EXYNOS5_CMU_SCLKSTOP_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x1} }, /* 0x1004_1104 */
	{ EXYNOS5_CMU_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_110C */
	{ EXYNOS5_CMU_ACLKSTOP_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1120 */
	{ EXYNOS5_CMU_SCLKSTOP_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x1} }, /* 0x1004_1124 */
	{ EXYNOS5_CMU_RESET_SYSMEM_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_112C */
	{ EXYNOS5_DRAM_FREQ_DOWN_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x1} }, /* 0x1004_1130 */
	{ EXYNOS5_DDRPHY_DLLOFF_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x1} }, /* 0x1004_1134 */
	{ EXYNOS5_DDRPHY_DLLLOCK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x1} }, /* 0x1004_1138 */
	{ EXYNOS5_APLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1140 */
	{ EXYNOS5_MPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1144 */
	{ EXYNOS5_VPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1148 */
	{ EXYNOS5_EPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_114C */
	{ EXYNOS5_BPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1150 */
	{ EXYNOS5_CPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1154 */
	{ EXYNOS54XX_DPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1158 */
	{ EXYNOS54XX_IPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_115C */
	{ EXYNOS54XX_KPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1160 */
	{ EXYNOS5_MPLLUSER_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1164 */
	{ EXYNOS5_BPLLUSER_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1170 */
	{ EXYNOS5422_RPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1174 */
	{ EXYNOS5422_SPLL_SYSCLK_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1178 */
	{ EXYNOS5_TOP_BUS_SYS_PWR_REG,				{ 0x3, 0x0, 0x0, 0x0} }, /* 0x1004_1180 */
	{ EXYNOS5_TOP_RETENTION_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x1} }, /* 0x1004_1184 */
	{ EXYNOS5_TOP_PWR_SYS_PWR_REG,				{ 0x3, 0x3, 0x3, 0x0} }, /* 0x1004_1188 */
	{ EXYNOS5_TOP_BUS_SYSMEM_SYS_PWR_REG,			{ 0x3, 0x0, 0x0, 0x0} }, /* 0x1004_1190 */
	{ EXYNOS5_TOP_RETENTION_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x1} }, /* 0x1004_1194 EVT1 */
	{ EXYNOS5_TOP_PWR_SYSMEM_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x0} }, /* 0x1004_1198 EVT1 */
	{ EXYNOS5_LOGIC_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_11A0 */
	{ EXYNOS5_OSCCLK_GATE_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x1} }, /* 0x1004_11A4 */
	{ EXYNOS5_LOGIC_RESET_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_11B0 EVT1 */
	{ EXYNOS5_OSCCLK_GATE_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_11B4 */
	{ EXYNOS5422_INTRAM_MEM_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x3} }, /* 0x1004_11B8 EVT1 */
	{ EXYNOS5422_INTROM_MEM_SYS_PWR_REG,			{ 0x3, 0x3, 0x3, 0x3} }, /* 0x1004_11BC EVT1 */
	{ EXYNOS5_PAD_RETENTION_DRAM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1200 */
	{ EXYNOS5_PAD_RETENTION_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_1204 */
	{ EXYNOS5422_PAD_RETENTION_JTAG_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_1208 */
	{ EXYNOS5422_PAD_RETENTION_DRAM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1210 */
	{ EXYNOS54XX_PAD_RETENTION_UART_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1214 */
	{ EXYNOS54XX_PAD_RETENTION_MMC0_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1218 */
	{ EXYNOS54XX_PAD_RETENTION_MMC1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_121C */
	{ EXYNOS54XX_PAD_RETENTION_MMC2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1220 */
	{ EXYNOS54XX_PAD_RETENTION_HSI_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1224 */
	{ EXYNOS54XX_PAD_RETENTION_EBIA_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1228 */
	{ EXYNOS54XX_PAD_RETENTION_EBIB_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_122C */
	{ EXYNOS54XX_PAD_RETENTION_SPI_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1230 */
	{ EXYNOS54XX_PAD_RETENTION_DRAM_SYSMEM_SYS_PWR_REG,	{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1234 EVT1 */
	{ EXYNOS5_PAD_ISOLATION_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_1240 */
	{ EXYNOS5_PAD_ISOLATION_SYSMEM_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1250 EVT1 */
	{ EXYNOS5_PAD_ALV_SEL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1260 */
	{ EXYNOS5_XUSBXTI_SYS_PWR_REG,				{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_1280 */
	{ EXYNOS5_XXTI_SYS_PWR_REG,				{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_1284 */
	{ EXYNOS5_EXT_REGULATOR_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_12C0 */
	{ EXYNOS5_GPIO_MODE_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1300 */
	{ EXYNOS5_GPIO_MODE_SYSMEM_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_1320 */
	{ EXYNOS5_GPIO_MODE_MAU_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_1340 */
	{ EXYNOS5_TOP_ASB_RESET_SYS_PWR_REG,			{ 0x1, 0x1, 0x1, 0x0} }, /* 0x1004_1344 */
	{ EXYNOS5_TOP_ASB_ISOLATION_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1348 */
	{ EXYNOS5_GSCL_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0} }, /* 0x1004_1400 */
	{ EXYNOS5_ISP_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0} }, /* 0x1004_1404 */
	{ EXYNOS5_MFC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0} }, /* 0x1004_1408 */
	{ EXYNOS5_G3D_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0} }, /* 0x1004_140C */
	{ EXYNOS5422_DISP1_SYS_PWR_REG,				{ 0x7, 0x0, 0x7, 0x0} }, /* 0x1004_1410 */
	{ EXYNOS5422_MAU_SYS_PWR_REG,				{ 0x7, 0x7, 0x0, 0x0} }, /* 0x1004_1414 */
	{ EXYNOS5422_G2D_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0} }, /* 0x1004_1418 */
	{ EXYNOS5422_MSC_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0} }, /* 0x1004_141C */
	{ EXYNOS5422_FSYS_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0} }, /* 0x1004_1420 */
	{ EXYNOS5422_FSYS2_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0} }, /* 0x1004_1424 */
	{ EXYNOS5422_PSGEN_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0} }, /* 0x1004_1428 */
	{ EXYNOS5422_PERIC_SYS_PWR_REG,				{ 0x7, 0x7, 0x7, 0x0} }, /* 0x1004_142C */
	{ EXYNOS5422_WCORE_SYS_PWR_REG,				{ 0x7, 0x0, 0x0, 0x0} }, /* 0x1004_1430 */
	{ EXYNOS5_CMU_CLKSTOP_GSCL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1480 */
	{ EXYNOS5_CMU_CLKSTOP_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1484 */
	{ EXYNOS5_CMU_CLKSTOP_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1488 */
	{ EXYNOS5_CMU_CLKSTOP_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_148C */
	{ EXYNOS5422_CMU_CLKSTOP_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1490 */
	{ EXYNOS5422_CMU_CLKSTOP_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_1494 */
	{ EXYNOS5422_CMU_CLKSTOP_G2D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1498 */
	{ EXYNOS5422_CMU_CLKSTOP_MSC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_149C */
	{ EXYNOS5422_CMU_CLKSTOP_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14A0 */
	{ EXYNOS5422_CMU_CLKSTOP_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14A4 */
	{ EXYNOS5422_CMU_CLKSTOP_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14A8 */
	{ EXYNOS5422_CMU_CLKSTOP_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14AC */
	{ EXYNOS5422_CMU_CLKSTOP_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14B0 */
	{ EXYNOS5_CMU_SYSCLK_GSCL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14C0 */
	{ EXYNOS5_CMU_SYSCLK_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14C4 */
	{ EXYNOS5_CMU_SYSCLK_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14C8 */
	{ EXYNOS5_CMU_SYSCLK_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14CC */
	{ EXYNOS5422_CMU_SYSCLK_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14D0 */
	{ EXYNOS5422_CMU_SYSCLK_MAU_SYS_PWR_REG,		{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_14D4 */
	{ EXYNOS5422_CMU_SYSCLK_G2D_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14D8 */
	{ EXYNOS5422_CMU_SYSCLK_MSC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14DC */
	{ EXYNOS5422_CMU_SYSCLK_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14E0 */
	{ EXYNOS5422_CMU_SYSCLK_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14E4 */
	{ EXYNOS5422_CMU_SYSCLK_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14E8 */
	{ EXYNOS5422_CMU_SYSCLK_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14EC */
	{ EXYNOS5422_CMU_SYSCLK_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_14F0 */
	{ EXYNOS5422_CMU_RESET_FSYS2_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1570 */
	{ EXYNOS5422_CMU_RESET_PSGEN_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1574 */
	{ EXYNOS5422_CMU_RESET_PERIC_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1578 */
	{ EXYNOS5422_CMU_RESET_WCORE_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_157C */
	{ EXYNOS5_CMU_RESET_GSCL_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1580 */
	{ EXYNOS5_CMU_RESET_ISP_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1584 */
	{ EXYNOS5_CMU_RESET_MFC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1588 */
	{ EXYNOS5_CMU_RESET_G3D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_158C */
	{ EXYNOS5422_CMU_RESET_DISP1_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1590 */
	{ EXYNOS5422_CMU_RESET_MAU_SYS_PWR_REG,			{ 0x1, 0x1, 0x0, 0x0} }, /* 0x1004_1594 */
	{ EXYNOS5422_CMU_RESET_G2D_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_1598 */
	{ EXYNOS5422_CMU_RESET_MSC_SYS_PWR_REG,			{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_159C */
	{ EXYNOS5422_CMU_RESET_FSYS_SYS_PWR_REG,		{ 0x1, 0x0, 0x0, 0x0} }, /* 0x1004_15A0 */
	{ PMU_TABLE_END,},
};
#endif

#ifdef CONFIG_SOC_EXYNOS5422_REV_0
void __iomem *exynos5422_list_feed[] = {
	EXYNOS_ARM_CORE_OPTION(0),
	EXYNOS_ARM_CORE_OPTION(1),
	EXYNOS_ARM_CORE_OPTION(2),
	EXYNOS_ARM_CORE_OPTION(3),
	EXYNOS_ARM_CORE_OPTION(4),
	EXYNOS_ARM_CORE_OPTION(5),
	EXYNOS_ARM_CORE_OPTION(6),
	EXYNOS_ARM_CORE_OPTION(7),
	EXYNOS5422_ARM_COMMON_OPTION,
	EXYNOS5422_KFC_COMMON_OPTION,
	EXYNOS5422_SCALER_OPTION,
	EXYNOS5422_ISP_OPTION,
	EXYNOS5422_MFC_OPTION,
	EXYNOS5422_G3D_OPTION,
	EXYNOS5422_DISP1_OPTION,
	EXYNOS5422_MAU_OPTION,
	EXYNOS5422_G2D_OPTION,
	EXYNOS5422_MSC_OPTION,
	EXYNOS5422_TOP_PWR_OPTION,
	EXYNOS5422_TOP_PWR_COREBLK_OPTION,
	EXYNOS5422_CAM_OPTION,
};
#else
void __iomem *exynos5422_list_feed[] = {
	EXYNOS_ARM_CORE_OPTION(0),
	EXYNOS_ARM_CORE_OPTION(1),
	EXYNOS_ARM_CORE_OPTION(2),
	EXYNOS_ARM_CORE_OPTION(3),
	EXYNOS_ARM_CORE_OPTION(4),
	EXYNOS_ARM_CORE_OPTION(5),
	EXYNOS_ARM_CORE_OPTION(6),
	EXYNOS_ARM_CORE_OPTION(7),
	EXYNOS54XX_ARM_COMMON_OPTION,
	EXYNOS54XX_KFC_COMMON_OPTION,
	EXYNOS5_GSCL_OPTION,
	EXYNOS5_ISP_OPTION,
	EXYNOS5410_MFC_OPTION,
	EXYNOS5410_G3D_OPTION,
	EXYNOS5410_DISP1_OPTION,
	EXYNOS5410_MAU_OPTION,
	EXYNOS5422_G2D_OPTION,
	EXYNOS5422_MSC_OPTION,
	EXYNOS5_TOP_PWR_OPTION,
	EXYNOS5_TOP_PWR_SYSMEM_OPTION,
};
#endif

void set_boot_flag(unsigned int cpu, unsigned int mode)
{
	unsigned int phys_cpu = cpu_logical_map(cpu);
	unsigned int tmp, core, cluster;
	void __iomem *addr;

	core = MPIDR_AFFINITY_LEVEL(phys_cpu, 0);
	cluster = MPIDR_AFFINITY_LEVEL(phys_cpu, 1);
	addr = REG_CPU_STATE_ADDR + 4 * (core + cluster * 4);

	tmp = __raw_readl(addr);

	if (mode & BOOT_MODE_MASK)
		tmp &= ~BOOT_MODE_MASK;
	else
		BUG_ON(mode == 0);

	tmp |= mode;
	__raw_writel(tmp, addr);
}

void clear_boot_flag(unsigned int cpu, unsigned int mode)
{
	unsigned int phys_cpu = cpu_logical_map(cpu);
	unsigned int tmp, core, cluster;
	void __iomem *addr;

	BUG_ON(mode == 0);

	core = MPIDR_AFFINITY_LEVEL(phys_cpu, 0);
	cluster = MPIDR_AFFINITY_LEVEL(phys_cpu, 1);
	addr = REG_CPU_STATE_ADDR + 4 * (core + cluster * 4);

	tmp = __raw_readl(addr);
	tmp &= ~mode;
	__raw_writel(tmp, addr);
}

void exynos5422_secondary_up(unsigned int cpu_id)
{
	unsigned int phys_cpu = cpu_logical_map(cpu_id);
	unsigned int tmp, core, cluster;
	void __iomem *addr;

	core = MPIDR_AFFINITY_LEVEL(phys_cpu, 0);
	cluster = MPIDR_AFFINITY_LEVEL(phys_cpu, 1);

	addr = EXYNOS_ARM_CORE_CONFIGURATION(core + (4 * cluster));

	tmp = __raw_readl(addr);
	tmp |= EXYNOS_CORE_LOCAL_PWR_EN;

	__raw_writel(tmp, addr);
}

void exynos5422_cpu_up(unsigned int cpu_id)
{
	unsigned int phys_cpu = cpu_logical_map(cpu_id);
	unsigned int tmp, core, cluster;
	void __iomem *addr;

	core = MPIDR_AFFINITY_LEVEL(phys_cpu, 0);
	cluster = MPIDR_AFFINITY_LEVEL(phys_cpu, 1);

	addr = EXYNOS_ARM_CORE_CONFIGURATION(core + (4 * cluster));

	tmp = __raw_readl(addr);
	tmp |= EXYNOS_CORE_LOCAL_PWR_EN;
	__raw_writel(tmp, addr);
}

void exynos5422_cpu_down(unsigned int cpu_id)
{
	unsigned int phys_cpu = cpu_logical_map(cpu_id);
	unsigned int tmp, core, cluster;
	void __iomem *addr;

	core = MPIDR_AFFINITY_LEVEL(phys_cpu, 0);
	cluster = MPIDR_AFFINITY_LEVEL(phys_cpu, 1);

	addr = EXYNOS_ARM_CORE_CONFIGURATION(core + (4 * cluster));

	tmp = __raw_readl(addr);
	tmp &= ~(EXYNOS_CORE_LOCAL_PWR_EN);
	__raw_writel(tmp, addr);
}

unsigned int exynos5422_cpu_state(unsigned int cpu_id)
{
	unsigned int phys_cpu = cpu_logical_map(cpu_id);
	unsigned int core, cluster, val;

	core = MPIDR_AFFINITY_LEVEL(phys_cpu, 0);
	cluster = MPIDR_AFFINITY_LEVEL(phys_cpu, 1);

	val = __raw_readl(EXYNOS_ARM_CORE_STATUS(core + (4 * cluster)))
						& EXYNOS_CORE_LOCAL_PWR_EN;

	return val == EXYNOS_CORE_LOCAL_PWR_EN;
}

extern struct cpumask hmp_slow_cpu_mask;
extern struct cpumask hmp_fast_cpu_mask;

#define cpu_online_hmp(cpu, mask)      cpumask_test_cpu((cpu), mask)

bool exynos5422_is_last_core(unsigned int cpu)
{
	unsigned int cluster = MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
	unsigned int cpu_id;
	struct cpumask mask, mask_and_online;

	if (cluster)
		cpumask_copy(&mask, &hmp_slow_cpu_mask);
	else
		cpumask_copy(&mask, &hmp_fast_cpu_mask);

	cpumask_and(&mask_and_online, &mask, cpu_online_mask);

	for_each_cpu(cpu_id, &mask) {
		if (cpu_id == cpu)
			continue;
		if (cpu_online_hmp(cpu_id, &mask_and_online))
			return false;
	}

	return true;
}

void exynos_pmu_wdt_control(bool on, unsigned int pmu_wdt_reset_type)
{
	unsigned int value;
	unsigned int wdt_auto_reset_dis;
	unsigned int wdt_reset_mask;

	/*
	 * When SYS_WDTRESET is set, watchdog timer reset request is ignored
	 * by power management unit.
	 */
	if (pmu_wdt_reset_type == PMU_WDT_RESET_TYPE0) {
		wdt_auto_reset_dis = EXYNOS_SYS_WDTRESET;
		wdt_reset_mask = EXYNOS_SYS_WDTRESET;
	} else if (pmu_wdt_reset_type == PMU_WDT_RESET_TYPE1) {
		wdt_auto_reset_dis = EXYNOS5422_SYS_WDTRESET;
		wdt_reset_mask = EXYNOS5422_SYS_WDTRESET;
	} else if (pmu_wdt_reset_type == PMU_WDT_RESET_TYPE2) {
		wdt_auto_reset_dis = EXYNOS5430_SYS_WDTRESET_EGL |
			EXYNOS5430_SYS_WDTRESET_KFC;
		wdt_reset_mask = EXYNOS5430_SYS_WDTRESET_EGL;
	} else if (pmu_wdt_reset_type == PMU_WDT_RESET_TYPE3) {
		wdt_auto_reset_dis = EXYNOS5430_SYS_WDTRESET_EGL |
			EXYNOS5430_SYS_WDTRESET_KFC;
		wdt_reset_mask = EXYNOS5430_SYS_WDTRESET_KFC;
	} else {
		pr_err("Failed to %s pmu wdt reset\n",
				on ? "enable" : "disable");
		return;
	}

	value = __raw_readl(EXYNOS5422_AUTOMATIC_WDT_RESET_DISABLE);
	if (on)
		value &= ~wdt_auto_reset_dis;
	else
		value |= wdt_auto_reset_dis;
	__raw_writel(value, EXYNOS5422_AUTOMATIC_WDT_RESET_DISABLE);
	value = __raw_readl(EXYNOS5422_MASK_WDT_RESET_REQUEST);
	if (on)
		value &= ~wdt_reset_mask;
	else
		value |= wdt_reset_mask;
	__raw_writel(value, EXYNOS5422_MASK_WDT_RESET_REQUEST);

	return;
}

void exynos_sys_powerdown_conf(enum sys_powerdown mode)
{
	unsigned int i;

	/* Setting SEQ_OPTION register */
	exynos_set_core_flag();

	for (i = 0; (exynos_pmu_config[i].reg != PMU_TABLE_END) ; i++)
		__raw_writel(exynos_pmu_config[i].val[mode],
				exynos_pmu_config[i].reg);
}

void exynos_reset_assert_ctrl(bool on)
{
	unsigned int i;
	unsigned int option;

	for (i = 0; i < num_possible_cpus(); i++) {
		option = __raw_readl(EXYNOS_ARM_CORE_OPTION(i));
		option = on ? (option | EXYNOS_USE_DELAYED_RESET_ASSERTION) :
				   (option & ~EXYNOS_USE_DELAYED_RESET_ASSERTION);
		__raw_writel(option, EXYNOS_ARM_CORE_OPTION(i));
	}

	on = true;

	option = __raw_readl(EXYNOS5422_ARM_COMMON_OPTION);
	option = on ? (option | EXYNOS_USE_DELAYED_RESET_ASSERTION) :
			(option & ~EXYNOS_USE_DELAYED_RESET_ASSERTION);
	__raw_writel(option, EXYNOS5422_ARM_COMMON_OPTION);

	option = __raw_readl(EXYNOS5422_KFC_COMMON_OPTION);
	option = on ? (option | EXYNOS_USE_DELAYED_RESET_ASSERTION) :
			(option & ~EXYNOS_USE_DELAYED_RESET_ASSERTION);
	__raw_writel(option, EXYNOS5422_KFC_COMMON_OPTION);
}

void exynos_set_core_flag(void)
{
	__raw_writel(KFC, EXYNOS5422_IROM_DATA_REG2);
}

static void exynos_use_feedback(void)
{
	unsigned int i;
	unsigned int tmp;

	/*
	 * Enable only SC_FEEDBACK
	 */
	for (i = 0; i < ARRAY_SIZE(exynos5422_list_feed); i++) {
		tmp = __raw_readl(exynos5422_list_feed[i]);
		tmp &= ~EXYNOS5_USE_SC_COUNTER;
		tmp |= EXYNOS5_USE_SC_FEEDBACK;
		__raw_writel(tmp, exynos5422_list_feed[i]);
	}
}

#define EXYNOS5422_PRINT_PMU(name) \
	pr_info("  - %s  CONFIG : 0x%x  STATUS : 0x%x  OPTION : 0x%x\n", \
			#name, \
			__raw_readl(EXYNOS5422_##name##_CONFIGURATION), \
			__raw_readl(EXYNOS5422_##name##_STATUS), \
			__raw_readl(EXYNOS5422_##name##_OPTION))

void show_exynos_pmu(void)
{
	int i;
	pr_info("\n");
	pr_info(" -----------------------------------------------------------------------------------\n");
	pr_info(" **** CPU PMU register dump ****\n");
	for (i=0; i < NR_CPUS; i++) {
		printk("[%d]   CONFIG : 0x%x  STATUS : 0x%x  OPTION : 0x%x\n", i,
			__raw_readl(EXYNOS5422_ARM_CORE0_CONFIGURATION + i * 0x80),
			__raw_readl(EXYNOS5422_ARM_CORE0_STATUS + i * 0x80),
			__raw_readl(EXYNOS5422_ARM_CORE0_OPTION + i * 0x80));
	}
	pr_info(" **** EGL NONCPU CONFIG : 0x%x  STATUS : 0x%x  OPTION : 0x%x\n",
			__raw_readl(EXYNOS5422_ARM_COMMON_CONFIGURATION),
			__raw_readl(EXYNOS5422_ARM_COMMON_STATUS),
			__raw_readl(EXYNOS5422_ARM_COMMON_OPTION));
	pr_info("      EGL L2 CONFIG : 0x%x  STATUS : 0x%x  OPTION : 0x%x\n",
			__raw_readl(EXYNOS5422_ARM_L2_CONFIGURATION),
			__raw_readl(EXYNOS5422_ARM_L2_STATUS),
			__raw_readl(EXYNOS5422_ARM_L2_OPTION));
	pr_info(" **** KFC NONCPU CONFIG : 0x%x  STATUS : 0x%x  OPTION : 0x%x\n",
			__raw_readl(EXYNOS5422_KFC_COMMON_CONFIGURATION),
			__raw_readl(EXYNOS5422_KFC_COMMON_STATUS),
			__raw_readl(EXYNOS5422_KFC_COMMON_OPTION));
	pr_info("      KFC L2 CONFIG : 0x%x  STATUS : 0x%x  OPTION : 0x%x\n",
			__raw_readl(EXYNOS5422_KFC_L2_CONFIGURATION),
			__raw_readl(EXYNOS5422_KFC_L2_STATUS),
			__raw_readl(EXYNOS5422_KFC_L2_OPTION));
	pr_info(" **** LOCAL BLOCK POWER ****\n");
	EXYNOS5422_PRINT_PMU(SCALER);
	EXYNOS5422_PRINT_PMU(ISP);
	EXYNOS5422_PRINT_PMU(MFC);
	EXYNOS5422_PRINT_PMU(G3D);
	EXYNOS5422_PRINT_PMU(DISP1);
	EXYNOS5422_PRINT_PMU(MAU);
	EXYNOS5422_PRINT_PMU(G2D);
	EXYNOS5422_PRINT_PMU(MSC);
	EXYNOS5422_PRINT_PMU(FSYS);
	EXYNOS5422_PRINT_PMU(FSYS2);
	EXYNOS5422_PRINT_PMU(PSGEN);
	EXYNOS5422_PRINT_PMU(PERIC);
	EXYNOS5422_PRINT_PMU(WCORE);
	pr_info(" -----------------------------------------------------------------------------------\n");
}

#if defined(CONFIG_MACH_ODROIDXU3)
#include <linux/delay.h>
#include <linux/pm.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/system.h>

static void odroid_power_off(void)
{
	unsigned int value, off_retry = 0;
	
	printk(KERN_EMERG "%s : Set PS_HOLD_CONTROL (PS_HOLD_LOW)\n", __func__);

	local_irq_disable();

	while (1) {
        /*
        * Set PSHOLD port for ouput low
        */
    	value = __raw_readl(EXYNOS5422_PS_HOLD_CONTROL);
    	value &= ~EXYNOS_PS_HOLD_OUTPUT_HIGH;
    	__raw_writel(value, EXYNOS5422_PS_HOLD_CONTROL);
    
    	/*
    	* Enable signal for PSHOLD port
    	*/
    	value = __raw_readl(EXYNOS5422_PS_HOLD_CONTROL);
    	value |= EXYNOS_PS_HOLD_EN;
    	__raw_writel(value, EXYNOS5422_PS_HOLD_CONTROL);
    
    	mdelay(3000);

    	if(off_retry++ > 5) {
			flush_cache_all();
			outer_flush_all();
			exynos5_restart(0, 0);

			printk(KERN_EMERG "%s: waiting for reboot\n", __func__);
			while(1);
    	}
    	printk(KERN_EMERG "%s : Power OFF Error! PS_HOLD = 0x%08X, error retry = %d !!!\n", __func__, value, off_retry);
    }
}
#endif

int __init exynos5422_pmu_init(void)
{
	unsigned int value, i;

#if defined(CONFIG_MACH_ODROIDXU3)
    pm_power_off = odroid_power_off;
#endif    

	exynos_cpu.power_up = exynos5422_secondary_up;
	exynos_cpu.power_state = exynos5422_cpu_state;
	exynos_cpu.power_down = exynos5422_cpu_down;
	exynos_cpu.is_last_core = exynos5422_is_last_core;

	/* Enable USE_STANDBY_WFI for all CORE */
	__raw_writel(EXYNOS5422_USE_STANDBY_WFI_ALL,
			EXYNOS5422_CENTRAL_SEQ_OPTION);

	value = __raw_readl(EXYNOS5422_ARM_L2_OPTION);
	value &= ~EXYNOS5_USE_RETENTION;
	__raw_writel(value, EXYNOS5422_ARM_L2_OPTION);

	value = __raw_readl(EXYNOS5422_KFC_L2_OPTION);
	value &= ~EXYNOS5_USE_RETENTION;
	__raw_writel(value, EXYNOS5422_KFC_L2_OPTION);

	/*
	 * To skip to control L2 commont at resume and DFT logic,
	 * set the #0 and #1 bit of PMU_SPARE3.
	 */
	__raw_writel(EXYNOS5422_SWRESET_KFC_SEL, EXYNOS5422_PMU_SPARE3);

	/*
	 * If turn L2_COMMON off, clocks relating ATB async bridge is gated.
	 * So when ISP power is gated, LPI is stucked.
	 */
	value = __raw_readl(EXYNOS5422_LPI_MASK0);
	value |= EXYNOS5422_ATB_ISP_ARM | EXYNOS5422_DIS;
	__raw_writel(value, EXYNOS5422_LPI_MASK0);

	value = __raw_readl(EXYNOS5422_LPI_MASK1);
	value |= EXYNOS5422_ATB_KFC;
	__raw_writel(value, EXYNOS5422_LPI_MASK1);

	/*
	 * To prevent form issuing new bus request form L2 memory system
	 * If core status is power down, should be set '1' to L2  power down
	 */
	value = __raw_readl(EXYNOS5422_ARM_COMMON_OPTION);
	value |= EXYNOS5_SKIP_DEACTIVATE_ACEACP_IN_PWDN;
	__raw_writel(value, EXYNOS5422_ARM_COMMON_OPTION);

	/*
	* Set PSHOLD port for ouput high
	*/
	value = __raw_readl(EXYNOS5422_PS_HOLD_CONTROL);
	value |= EXYNOS_PS_HOLD_OUTPUT_HIGH;
	__raw_writel(value, EXYNOS5422_PS_HOLD_CONTROL);

	/*
	* Enable signal for PSHOLD port
	*/
	value = __raw_readl(EXYNOS5422_PS_HOLD_CONTROL);
	value |= EXYNOS_PS_HOLD_EN;
	__raw_writel(value, EXYNOS_PS_HOLD_CONTROL);

	/*
	 * DUR_WAIT_RESET : 0xF
	 * This setting is to reduce suspend/resume time.
	 */
	__raw_writel(DUR_WAIT_RESET, EXYNOS5422_LOGIC_RESET_DURATION3);

	/* Serialized CPU wakeup of Eagle */
	__raw_writel(SPREAD_ENABLE, EXYNOS5422_ARM_INTR_SPREAD_ENABLE);
	__raw_writel(SPREAD_USE_STANDWFI, EXYNOS5422_ARM_INTR_SPREAD_USE_STANDBYWFI);
	__raw_writel(0x1, EXYNOS5422_UP_SCHEDULER);

	/* PMU setting to use L2 auto-power gating */
	value = __raw_readl(EXYNOS5422_ARM_COMMON_OPTION);
	value |= (1 << 30) | (1 << 29) | (1 << 9);
	__raw_writel(value, EXYNOS5422_ARM_COMMON_OPTION);

	exynos_reset_assert_ctrl(false);

	for (i = 0; i < 8; i++) {
		value = __raw_readl(EXYNOS_ARM_CORE_OPTION(i));
#if defined CONFIG_EXYNOS_CPUIDLE_C2
		if (i > 3)
			value &= ~EXYNOS_ENABLE_AUTOMATIC_WAKEUP;
		else
			value |= EXYNOS_ENABLE_AUTOMATIC_WAKEUP;
#else
			value &= ~EXYNOS_ENABLE_AUTOMATIC_WAKEUP;
#endif
		__raw_writel(value, EXYNOS_ARM_CORE_OPTION(i));
	}

	/*
	 * Set measure power on/off duration
	 * Use SC_USE_FEEDBACK
	 */
	exynos_use_feedback();

	exynos_pmu_config = exynos5422_pmu_config;

	if (exynos_pmu_config != NULL)
		pr_info("EXYNOS5422 PMU Initialize\n");
	else
		pr_info("EXYNOS: PMU not supported\n");

	return 0;
}
