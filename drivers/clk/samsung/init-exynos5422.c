/*
 * Copyright (c) 2013 Samsung Electronics Co., Ltd.
 * Author: Hyosang Jung
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common Clock Framework support for Exynos5430 SoC.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mach/regs-clock-exynos5422.h>
#include <mach/regs-pmu.h>
#include <mach/regs-audss.h>

#include "clk.h"
#include "clk-pll.h"
#define CMU_PRINT_PLL

struct clk_enabler {
	struct clk		*clk;
	struct list_head	node;
};

static LIST_HEAD(clk_enabler_list);

static inline void add_enabler(const char *name)
{
	struct clk_enabler *ce;
	struct clk *clock;
	clock = __clk_lookup(name);

	if (IS_ERR(clock))
		return;

	ce = kzalloc(sizeof(struct clk_enabler), GFP_KERNEL);
	if (!ce)
		return;

	ce->clk = clock;
	list_add(&ce->node, &clk_enabler_list);
}

static void top_clk_enable(void)
{
	struct clk_enabler *ce;
	struct clk *clk;
#if defined(CMU_PRINT_PLL)
	unsigned long tmp;
#endif

	/* CAM CMU */
	add_enabler("clk_camif_top_3aa");
	add_enabler("clk_camif_top_fimcl1");
	add_enabler("clk_camif_top_3aa0");
	add_enabler("clk_gscl_wrap_b");
	add_enabler("clk_gscl_wrap_a");
	add_enabler("clk_camif_top_fimcl0");
	add_enabler("clk_camif_top_fimcl3");
	add_enabler("mout_aclk_432_cam_sw");
	add_enabler("clk_noc_p_rstop_fimcl");
	add_enabler("clk_xiu_si_gscl_cam");
	add_enabler("clk_xiu_mi_gscl_cam");
	add_enabler("gscl_fimc_lite3");
	add_enabler("clk_3aa");
	add_enabler("clk_camif_top_csis0");

	/* SPI  */
	add_enabler("sclk_spi2");
	add_enabler("sclk_spi1");
	add_enabler("sclk_spi0");
	add_enabler("fout_apll");
	add_enabler("fout_bpll");
	add_enabler("fout_cpll");
	add_enabler("fout_dpll");
	add_enabler("fout_ipll");
	add_enabler("fout_mpll");
	add_enabler("fout_spll");
	add_enabler("fout_vpll");
	add_enabler("fout_epll");
	add_enabler("fout_rpll");
	add_enabler("fout_kpll");
#ifdef CONFIG_SOC_EXYNOS5422_REV_0
	add_enabler("dout_spll_ctrl_div2");
#endif
#ifdef CONFIG_SOC_EXYNOS5422_REV_0
	/* top bus clock enable */
	add_enabler("mout_aclk_200_fsys_user");
	add_enabler("mout_pclk_200_fsys_user");
	add_enabler("mout_aclk_100_noc_user");
	add_enabler("mout_aclk_400_wcore_user");
	add_enabler("mout_aclk_200_fsys2_user");
	add_enabler("aclk_200_disp1");
	add_enabler("aclk_400_mscl");
	add_enabler("aclk_400_isp");
	add_enabler("mout_aclk_333_user");
	add_enabler("mout_aclk_266_user");
	add_enabler("mout_aclk_166_user");
	add_enabler("aclk_66_peric");
	add_enabler("aclk_66_psgen");
	add_enabler("pclk_66_gpio");
	add_enabler("aclk_333_432_isp0");
	add_enabler("aclk_333_432_isp");
	add_enabler("aclk_333_432_gscl");
	add_enabler("aclk_300_gscl");
	add_enabler("mout_aclk_300_disp1_user");
	add_enabler("aclk_300_jpeg");
	add_enabler("mout_aclk_g3d_user");
	add_enabler("aclk_266_g2d");
	add_enabler("aclk_333_g2d");
	add_enabler("mout_aclk_400_disp1_user");
	add_enabler("aclk_266_isp");
	add_enabler("aclk_432_scaler");
	add_enabler("aclk_432_cam");
	add_enabler("aclk_f1_550_cam");
	add_enabler("aclk_550_cam");
	add_enabler("mau_epll_clk");
	add_enabler("mx_mspll_ccore_phy");
	add_enabler("sclk_gscl_wrap_b");
	add_enabler("sclk_gscl_wrap_a");
	add_enabler("sclk_isp_sensor2");
	add_enabler("sclk_isp_sensor1");
	add_enabler("sclk_isp_sensor0");
	add_enabler("sclk_uart_isp");
	add_enabler("sclk_isp0_isp");
	add_enabler("sclk_isp1_isp");
	add_enabler("sclk_pwm_isp");
	add_enabler("aclk_333");
#else
	add_enabler("pclk_66_gpio");
	add_enabler("aclk_66_peric");
	add_enabler("aclk_66_psgen");
	add_enabler("aclk_300_jpeg");
	add_enabler("aclk_200_disp1");
#endif
	add_enabler("aclk_noc_fsys");
	add_enabler("aclk_noc_fsys2");
	add_enabler("dout_unipro");
	add_enabler("mout_aclk_g3d_user");
	add_enabler("clk_ahb2apb_g3dp");
	add_enabler("clk_sdmmc0");
	add_enabler("clk_sdmmc1");
	add_enabler("clk_sdmmc2");

	add_enabler("clk_smmufimd1x_m1");
	add_enabler("clk_smmufimd1x_m0");

	/* enable list */
	add_enabler("aclk_aclk_333_g2d");

	/* enable list to enter suspend to ram */
	add_enabler("sclk_usbphy300");
	add_enabler("dout_usbphy300");
	add_enabler("sclk_usbdrd300");
	add_enabler("dout_usbdrd300");
	add_enabler("mout_usbdrd300");
	add_enabler("sclk_usbphy301");
	add_enabler("dout_usbphy301");
	add_enabler("sclk_usbdrd301");
	add_enabler("dout_usbdrd301");
	add_enabler("mout_usbdrd301");

	/* enable list to wake up */
	add_enabler("pclk_seckey_apbif");
	add_enabler("pclk_tzpc9");
	add_enabler("pclk_tzpc8");
	add_enabler("pclk_tzpc7");
	add_enabler("pclk_tzpc6");
	add_enabler("pclk_tzpc5");
	add_enabler("pclk_tzpc4");
	add_enabler("pclk_tzpc3");
	add_enabler("pclk_tzpc2");
	add_enabler("pclk_tzpc1");
	add_enabler("pclk_tzpc0");

	list_for_each_entry(ce, &clk_enabler_list, node) {
		clk_prepare(ce->clk);
		clk_enable(ce->clk);
	}

#ifdef CONFIG_SOC_EXYNOS5422_REV_0
	exynos_set_rate("fout_spll", 800000000);
	exynos_set_rate("dout_spll_ctrl", 400000000);
	exynos_set_rate("dout_spll_ctrl_div2", 400000000);
#endif
	/* Enable unipro mux to support LPA Mode */
	clk = __clk_lookup("mout_unipro");
	clk_prepare_enable(clk);
#if defined(CMU_PRINT_PLL)
	clk = __clk_lookup("fout_apll");
	tmp = clk_get_rate(clk);
	pr_info("apll %ld\n", tmp);

	clk = __clk_lookup("fout_bpll");
	tmp = clk_get_rate(clk);
	pr_info("bpll %ld\n", tmp);

	clk = __clk_lookup("fout_cpll");
	tmp = clk_get_rate(clk);
	pr_info("cpll %ld\n", tmp);

	clk = __clk_lookup("fout_dpll");
	tmp = clk_get_rate(clk);
	pr_info("dpll %ld\n", tmp);

	clk = __clk_lookup("fout_epll");
	clk_set_rate(clk, 180633600);
	tmp = clk_get_rate(clk);
	pr_info("epll %ld\n", tmp);

	clk = __clk_lookup("fout_rpll");
	tmp = clk_get_rate(clk);
	pr_info("rpll %ld\n", tmp);

	clk = __clk_lookup("fout_ipll");
	tmp = clk_get_rate(clk);
	pr_info("ipll %ld\n", tmp);

	clk = __clk_lookup("fout_spll");
	tmp = clk_get_rate(clk);
	pr_info("spll %ld\n", tmp);
#ifdef CONFIG_SOC_EXYNOS5422_REV_0
	clk = __clk_lookup("dout_spll_ctrl_div2");
	tmp = clk_get_rate(clk);
	pr_info("dout_spll_ctrl_div2 %ld\n", tmp);
#endif
	clk = __clk_lookup("fout_vpll");
	tmp = clk_get_rate(clk);
	pr_info("vpll %ld\n", tmp);

	clk = __clk_lookup("fout_mpll");
	tmp = clk_get_rate(clk);
	pr_info("mpll %ld\n", tmp);
#endif
	pr_info("Clock enables : TOP, MIF\n");
}

static void clkout_init_clock(void)
{
       writel(0x1000, EXYNOS_PMU_DEBUG);
}

static void aud_init_clock(void)
{
	/* To avoid over-clock */
	writel(0xF48, EXYNOS_CLKDIV_AUDSS);

#ifdef CONFIG_SOC_EXYNOS5422_REV_0
	exynos_set_parent("mout_epll_ctrl", "fout_epll");
	exynos_set_parent("mout_mau_epll_clk", "mout_epll_ctrl");
	exynos_set_parent("mout_mau_epll_clk_user", "mout_mau_epll_clk");
	exynos_set_parent("mout_ass_clk", "mout_mau_epll_clk_user");
	exynos_set_parent("mout_ass_i2s", "mout_ass_clk");
#else
	exynos_set_parent("mout_ass_clk", "fin_pll");
	exynos_set_parent("mout_ass_i2s", "mout_ass_clk");
#endif
	exynos_set_rate("dout_ass_srp", 100 * 1000000);
	exynos_set_rate("dout_ass_bus", 50 * 1000000);
	exynos_set_rate("dout_ass_i2s", 12 * 1000000);
}

static void uart_clock_init(void)
{
	exynos_set_rate("dout_uart0", 150000000);
	exynos_set_rate("dout_uart1", 150000000);
	exynos_set_rate("dout_uart2", 150000000);
	exynos_set_rate("dout_uart3", 150000000);
}

static void spi_clock_init(void)
{
	exynos_set_parent("mout_spi1", "mout_dpll_ctrl");
	exynos_set_rate("dout_spi1", 80 * 1000000);
}

static void mscl_init_clock(void)
{

	exynos_set_parent("mout_aclk_400_mscl", "fout_cpll");
	exynos_set_parent("mout_aclk_400_mscl_sw", "dout_aclk_400_mscl");
	exynos_set_parent("mout_aclk_400_mscl_user", "mout_aclk_400_mscl_sw");
	exynos_set_parent("aclk_400_mscl", "mout_aclk_400_mscl_user");

	exynos_set_rate("dout_aclk_400_mscl", 333 * 1000000);

	pr_info("scaler: dout_aclk_400_mscl %d aclk_400_mscl %d\n",
			exynos_get_rate("dout_aclk_400_mscl"),
			exynos_get_rate("aclk_400_mscl"));
}

void g2d_init_clock(void)
{
	int clk_rate1;

	if (exynos_set_parent("mout_aclk_333_g2d", "mout_cpll_ctrl"))
		pr_err("Unable to set clock %s's parent %s\n"
				, "mout_aclk_333_g2d", "mout_cpll_ctrl");

	if (exynos_set_parent("mout_aclk_333_g2d_sw", "dout_aclk_333_g2d"))
		pr_err("Unable to set clock %s's parent %s\n"
				, "mout_aclk_333_g2d_sw", "dout_aclk_333_g2d");

	if (exynos_set_parent("mout_aclk_333_g2d_user", "mout_aclk_333_g2d_sw"))
		pr_err("Unable to set clock %s's parent %s\n"
				, "mout_aclk_333_g2d_user", "mout_aclk_333_g2d_sw");

	if(exynos_set_rate("dout_aclk_333_g2d", 333 * 1000000))
		pr_err("Can't set %s clock rate\n", "dout_aclk_g2d_400");

	clk_rate1 = exynos_get_rate("dout_aclk_333_g2d");

	pr_info("[%s:%d] aclk_333_g2d:%d\n"
			, __func__, __LINE__, clk_rate1);
}

void gsc_clock_init(void)
{
	if (exynos_set_parent("mout_aclk_300_gscl", "mout_dpll_ctrl"))
		pr_err("failed clock mout_aclk_300_gscl to \
				mout_dpll_ctrl\n");
	if (exynos_set_parent("mout_aclk_300_gscl_sw",
				"dout_aclk_300_gscl"))
		pr_err("failed clock mout_aclk_300_gscl_sw to \
				dout_aclk_300_gscl\n");
	if (exynos_set_parent("mout_aclk_300_gscl_user",
				"mout_aclk_300_gscl_sw"))
		pr_err("failed clock mout_aclk_300_gscl_user to \
				mout_aclk_300_gscl_sw\n");
	if (exynos_set_parent("aclk_300_gscl", "mout_aclk_300_gscl_user"))
		pr_err("failed clock aclk_300_gscl to \
				mout_aclk_300_gscl_user\n");
	exynos_set_rate("dout_aclk_300_gscl", 300 * MHZ);
	pr_info("gscaler: dout_aclk_300_gscl %d aclk_300_gscl %d\n",
			exynos_get_rate("dout_aclk_300_gscl"),
			exynos_get_rate("aclk_300_gscl"));
}

void pwm_init_clock(void)
{
	clk_register_fixed_factor(NULL, "pwm-clock",
			"sclk_pwm",CLK_SET_RATE_PARENT, 1, 1);
	if (exynos_set_parent("mout_pwm", "mout_cpll_ctrl"))
		pr_err("failed clock mout_pwm to mout_cpll_ctrl\n");
	exynos_set_rate("dout_pwm", 66600000);
}

void jpeg_clock_init(void)
{
	if (exynos_set_parent("mout_aclk_300_jpeg", "mout_dpll_ctrl"))
		pr_err("failed clock mout_aclk_300_jpeg to \
				mout_dpll_ctrl\n");
	if (exynos_set_parent("mout_aclk_300_jpeg_sw",
				"dout_aclk_300_jpeg"))
		pr_err("failed clock mout_aclk_300_jpeg_sw to \
				dout_aclk_300_jpeg\n");
	if (exynos_set_parent("mout_aclk_300_jpeg_user",
				"mout_aclk_300_jpeg_sw"))
		pr_err("failed clock mout_aclk_300_jpeg_user to \
				mout_aclk_300_jpeg_sw\n");
	if (exynos_set_parent("aclk_300_jpeg", "mout_aclk_300_jpeg_user"))
		pr_err("failed clock aclk_300_jpeg to \
				mout_aclk_300_jpeg_user\n");
	exynos_set_rate("dout_aclk_300_jpeg", 300 * MHZ);

	pr_info("[%s:%d]jpeg: dout_aclk_300_jpeg %d aclk_300_jpeg %d\n",
			__func__, __LINE__,
			exynos_get_rate("dout_aclk_300_jpeg"),
			exynos_get_rate("aclk_300_jpeg"));
}

void mfc_clock_init(void)
{
	exynos_set_parent("mout_aclk_333", "dout_spll_ctrl_div2");
	exynos_set_parent("mout_aclk_333_sw", "mout_aclk_333");
	exynos_set_parent("mout_aclk_333_user", "mout_aclk_333_sw");
	exynos_set_parent("aclk_333", "mout_aclk_333_user");

	exynos_set_rate("dout_aclk_333", 400*1000000);

	pr_info("mfc: aclk_333 %d\n", exynos_get_rate("aclk_333"));
}

void crypto_init_clock(void)
{
	if (exynos_set_parent("mout_aclk_266_g2d", "mout_mpll_ctrl"))
		pr_err("failed to set parent %s\n", "mout_aclk_266_g2d");

	if (exynos_set_parent("mout_aclk_266_g2d_sw", "dout_aclk_266_g2d"))
		pr_err("failed to set parent %s\n", "mout_aclk_266_g2d_sw");

	if (exynos_set_parent("mout_aclk_266_g2d_user", "mout_aclk_266_g2d_sw"))
		pr_err("failed to set parent %s\n", "mout_aclk_266_g2d_user");

	if (exynos_set_rate("dout_aclk_266_g2d", 300 * 1000000))
		pr_err("failed to set rate %s\n", "dout_aclk_266_g2d");

	if (exynos_set_rate("dout_acp_pclk", 150 * 1000000))
		pr_err("failed to set rate %s\n", "dout_acp_pclk");
}

void mmc_clock_init(void)
{
	exynos_set_parent("mout_mmc0", "mout_cpll_ctrl");
	exynos_set_parent("mout_mmc1", "mout_spll_ctrl");
	exynos_set_parent("mout_mmc2", "mout_cpll_ctrl");

	exynos_set_rate("dout_mmc0", 666 * 1000000);
	exynos_set_rate("dout_mmc1", 800 * 1000000);
	exynos_set_rate("dout_mmc2", 666 * 1000000);

	pr_info("mmc0: dout_mmc0 %d\n", exynos_get_rate("dout_mmc0"));
	pr_info("mmc1: dout_mmc1 %d\n", exynos_get_rate("dout_mmc1"));
	pr_info("mmc2: dout_mmc2 %d\n", exynos_get_rate("dout_mmc2"));
}

void fimd1_clock_init(void)
{
	unsigned int reg = 0;

	/*
	 * Set DISP1BLK_CFG register for Display path selection
	 *
	 * FIMD of DISP1_BLK Bypass selection : DISP1BLK_CFG[15]
	 * ---------------------
	 *  1 | FIMD : selected
	 */
	reg = __raw_readl(S3C_VA_SYS + 0x0214);
	reg &= ~(1 << 15);	/* To save other reset values */
	reg |= (1 << 15);
	__raw_writel(reg, S3C_VA_SYS + 0x0214);

	/* Reference clcok selection for DPTX_PHY: PAD_OSC_IN */
	reg = __raw_readl(S3C_VA_SYS + 0x04d4);
	reg &= ~(1 << 0);
	__raw_writel(reg, S3C_VA_SYS + 0x04d4);

	/* DPTX_PHY: XXTI */
	reg = __raw_readl(S3C_VA_SYS + 0x04d8);
	reg &= ~(1 << 3);
	__raw_writel(reg, S3C_VA_SYS + 0x04d8);
    
	if (exynos_set_parent("mout_fimd1", "mout_mpll_ctrl"))
		pr_err("failed to set parent %s\n", "mout_fimd1");
	if (exynos_set_rate("dout_fimd1", 266 * 1000000))
		pr_err("failed to set rate %s\n", "dout_fimd1");
}

void __init exynos5422_clock_init(void)
{
#ifndef CONFIG_L2_AUTO_CLOCK_DISABLE
/* EXYNOS5422 C2 enable support */
	__raw_writel(__raw_readl(EXYNOS5422_CMU_CPU_SPARE1) | (1<<1 | 1<<3),
			EXYNOS5422_CMU_CPU_SPARE1);
#endif

	top_clk_enable();
	clkout_init_clock();
	aud_init_clock();
	uart_clock_init();
	spi_clock_init();
	mmc_clock_init();
	mscl_init_clock();
	g2d_init_clock();
	pwm_init_clock();
	gsc_clock_init();
	jpeg_clock_init();
	mfc_clock_init();
	crypto_init_clock();
	fimd1_clock_init();
}
