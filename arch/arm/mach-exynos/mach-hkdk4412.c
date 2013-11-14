/*
 * linux/arch/arm/mach-exynos4/mach-hkdk4412.c
 *
 * Copyright (c) 2012 AgreeYa Mobility Co., Ltd.
 *		http://www.agreeyamobility.net
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/mfd/max77686.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/serial_core.h>
#include <linux/platform_data/s3c-hsotg.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/usb-ehci-s5p.h>
#include <linux/platform_data/usb-exynos.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/clk.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>

#include <plat/backlight.h>
#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <plat/keypad.h>
#include <plat/mfc.h>
#include <plat/regs-serial.h>
#include <plat/sdhci.h>
#include <plat/fb.h>
#include <plat/hdmi.h>

#include <video/platform_lcd.h>
#include <video/samsung_fimd.h>

#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/dwmci.h>
#include <drm/exynos_drm.h>

#include "common.h"
#include "pmic-77686.h"

extern void exynos4_setup_dwmci_cfg_gpio(struct platform_device *dev, int width);

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define HKDK4412_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define HKDK4412_ULCON_DEFAULT	S3C2410_LCON_CS8

#define HKDK4412_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg hkdk4412_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= HKDK4412_UCON_DEFAULT,
		.ulcon		= HKDK4412_ULCON_DEFAULT,
		.ufcon		= HKDK4412_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= HKDK4412_UCON_DEFAULT,
		.ulcon		= HKDK4412_ULCON_DEFAULT,
		.ufcon		= HKDK4412_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= HKDK4412_UCON_DEFAULT,
		.ulcon		= HKDK4412_ULCON_DEFAULT,
		.ufcon		= HKDK4412_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= HKDK4412_UCON_DEFAULT,
		.ulcon		= HKDK4412_ULCON_DEFAULT,
		.ufcon		= HKDK4412_UFCON_DEFAULT,
	},
};


#if defined(CONFIG_USB_HSIC_USB3503)
#include <linux/platform_data/usb3503.h>

static struct usb3503_platform_data usb3503_pdata = {
	.initial_mode	= USB3503_MODE_HUB,
	.gpio_intn	= EXYNOS4_GPX3(0),
	.gpio_connect	= EXYNOS4_GPX3(4),
	.gpio_reset	= EXYNOS4_GPX3(5),
};
#endif

static struct i2c_board_info hkdk4412_i2c_devs0[] __initdata = {
	{
		I2C_BOARD_INFO("max77686", (0x12 >> 1)),
		.platform_data	= &exynos4_max77686_info,
	},
#if defined(CONFIG_USB_HSIC_USB3503)
	{
		I2C_BOARD_INFO("usb3503", (0x08)),
		.platform_data  = &usb3503_pdata,
	},
#endif
};

static struct i2c_board_info hkdk4412_i2c_devs1[] __initdata = {
#if defined(CONFIG_SND_SOC_MAX98090)
	{
		I2C_BOARD_INFO("max98090", (0x20>>1)),
	},
#endif
};

/* Odroid-O2 schematics show the DDC of the remote HDMI device connected to
 * I2C2. HDMI specs state that DDC always sits at bus address 0x50. */
static struct i2c_board_info hkdk4412_i2c_devs2[] __initdata = {
	{
		I2C_BOARD_INFO("s5p_ddc", 0x50),
	},
};

static struct i2c_board_info hkdk4412_i2c_devs3[] __initdata = {
	/* nothing here yet */
};

static struct i2c_board_info hkdk4412_i2c_devs7[] __initdata = {
	/* nothing here yet */
};

#if defined(CONFIG_ODROID_U2)
static struct gpio_led hkdk4412_gpio_leds[] = {
        {
                .name			= "led1",
                .default_trigger	= "heartbeat",
                .gpio			= EXYNOS4_GPC1(0),
                .active_low		= 1,
        },
};
#else
static struct gpio_led hkdk4412_gpio_leds[] = {
	{
		.name		= "led1",	/* D5 on ODROID-X */
		.default_trigger	= "oneshot",
		.gpio		= EXYNOS4_GPC1(0),
		.active_low	= 1,
	},
	{
		.name		= "led2",	/* D6 on ODROID-X */
		.default_trigger	= "heartbeat",
		.gpio		= EXYNOS4_GPC1(2),
		.active_low	= 1,
	},
};
#endif

static struct gpio_led_platform_data hkdk4412_gpio_led_info = {
	.leds		= hkdk4412_gpio_leds,
	.num_leds	= ARRAY_SIZE(hkdk4412_gpio_leds),
};

static struct platform_device hkdk4412_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &hkdk4412_gpio_led_info,
	},
};

/* LCD Backlight data */
static struct samsung_bl_gpio_info hkdk4412_bl_gpio_info = {
	.no	= EXYNOS4_GPD0(1),
	.func	= S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data hkdk4412_bl_data = {
	.pwm_id		= 1,
	.pwm_period_ns	= 1000,
};

#if defined(CONFIG_LCD_LP101WH1) && defined(CONFIG_DRM_EXYNOS_FIMD)
static struct exynos_drm_fimd_pdata drm_fimd_pdata = {
	.panel = {
		.timing = {
			.left_margin 	= 80,
			.right_margin 	= 48,
			.upper_margin 	= 14,
			.lower_margin 	= 3,
			.hsync_len 	= 32,
			.vsync_len 	= 5,
			.xres 		= 1366,
			.yres 		= 768,
		},
	},
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC | VIDCON1_INV_VCLK,
	.default_win 	= 0,
	.bpp 		= 32,
};
	
static void lcd_lp101wh1_set_power(struct plat_lcd_data *pd,
				   unsigned int power)
{
	gpio_request(EXYNOS4_GPA1(3), "bl_enable");
	gpio_direction_output(EXYNOS4_GPA1(3), power);
	gpio_free(EXYNOS4_GPA1(3));
}

static struct plat_lcd_data hkdk4412_lcd_lp101wh1_data = {
	.set_power	= lcd_lp101wh1_set_power,
};

static struct platform_device hkdk4412_lcd_lp101wh1 = {
	.name	= "platform-lcd",
	.dev	= {
		.parent		= &s5p_device_fimd0.dev,
		.platform_data	= &hkdk4412_lcd_lp101wh1_data,
	},
};
#endif // LCD

/* GPIO KEYS */
static struct gpio_keys_button hkdk4412_gpio_keys_tables[] = {
	{
		.code			= KEY_POWER,
		.gpio			= EXYNOS4_GPX1(3),	/* XEINT11 */
		.desc			= "KEY_POWER",
		.type			= EV_KEY,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 1,
	},
	{
		.code			= KEY_PROG1,
		.gpio			= EXYNOS4_GPX2(2),
		.desc			= "KEY_USER",
		.type			= EV_KEY,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 1,
	},
};

static struct gpio_keys_platform_data hkdk4412_gpio_keys_data = {
	.buttons	= hkdk4412_gpio_keys_tables,
	.nbuttons	= ARRAY_SIZE(hkdk4412_gpio_keys_tables),
};

static struct platform_device hkdk4412_gpio_keys = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data	= &hkdk4412_gpio_keys_data,
	},
};

#if defined(CONFIG_SND_SOC_HKDK_MAX98090)
static struct platform_device hardkernel_audio_device = {
	.name	= "hkdk-snd-max89090",
	.id	= -1,
};
#endif

/* USB EHCI */
static struct s5p_ehci_platdata hkdk4412_ehci_pdata;

static void __init hkdk4412_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &hkdk4412_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}

/* USB OHCI */
static struct exynos4_ohci_platdata hkdk4412_ohci_pdata;

static void __init hkdk4412_ohci_init(void)
{
	struct exynos4_ohci_platdata *pdata = &hkdk4412_ohci_pdata;

	exynos4_ohci_set_platdata(pdata);
}

/* USB OTG */
static struct s3c_hsotg_plat hkdk4412_hsotg_pdata;

/* SDCARD */
static struct s3c_sdhci_platdata hkdk4412_hsmmc2_pdata __initdata = {
	.max_width	= 4,
	.host_caps	= MMC_CAP_4_BIT_DATA |
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.cd_type	= S3C_SDHCI_CD_INTERNAL,
};

/* DWMMC */
static int hkdk4412_dwmci_get_bus_wd(u32 slot_id)
{
       return 8;
}

static int hkdk4412_dwmci_init(u32 slot_id, irq_handler_t handler, void *data)
{
       return 0;
}

static struct dw_mci_board hkdk4412_dwmci_pdata = {
       .num_slots              = 1,
       .quirks                 = DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
       .caps		       = MMC_CAP_UHS_DDR50 | MMC_CAP_1_8V_DDR | MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
       .fifo_depth	       = 0x80,
       .bus_hz                 = 100 * 1000 * 1000,
       .detect_delay_ms        = 200,
       .init                   = hkdk4412_dwmci_init,
       .get_bus_wd             = hkdk4412_dwmci_get_bus_wd,
       .cfg_gpio	       = exynos4_setup_dwmci_cfg_gpio,
};

static struct resource tmu_resource[] = {
	[0] = {
		.start = EXYNOS4_PA_TMU,
		.end = EXYNOS4_PA_TMU + 0x0100,
		.flags = IORESOURCE_MEM,
	},
	[1] = { 
		.start = EXYNOS4_IRQ_TMU_TRIG0,
		.end = EXYNOS4_IRQ_TMU_TRIG0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device hkdk4412_tmu = {
	.id = -1,
	.name = "exynos5250-tmu",
	.num_resources = ARRAY_SIZE(tmu_resource),
	.resource = tmu_resource,
};

#if defined(CONFIG_ODROID_U2_FAN)
#include	<linux/platform_data/odroidu2_fan.h>
struct odroid_fan_platform_data odroid_fan_pdata = {
        .pwm_gpio = EXYNOS4_GPD0(0),
        .pwm_func = S3C_GPIO_SFN(2),

        .pwm_id = 0,
        .pwm_periode_ns = 20972,        // Freq 22KHz,
        .pwm_duty = 255,                                // max=255, 
};

static struct platform_device odroid_fan = {
        .name   = "odroidu2-fan",
        .id     = -1,  
        .dev.platform_data = &odroid_fan_pdata,
};
#endif

static struct platform_device *hkdk4412_devices[] __initdata = {
	&s3c_device_hsmmc2,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&s3c_device_i2c3,
	&s3c_device_i2c7,
	&s3c_device_rtc,
	&s3c_device_usb_hsotg,
	&s3c_device_wdt,
	&s5p_device_ehci,
#ifdef CONFIG_SND_SAMSUNG_I2S
	&exynos4_device_i2s0,
#endif
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc3,
	&s5p_device_fimc_md,
	&s5p_device_fimd0,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5p_device_g2d,
	&mali_gpu_device,
#if defined(CONFIG_S5P_DEV_TV)
	&s5p_device_hdmi,
	&s5p_device_cec,
	&s5p_device_i2c_hdmiphy,
	&s5p_device_mixer,
	&hdmi_fixed_voltage,
#endif
	&exynos4_device_ohci,
	&exynos_device_dwmci,
	&hkdk4412_leds_gpio,
#if defined(CONFIG_LCD_LP101WH1)
	&hkdk4412_lcd_lp101wh1,
#endif
	&hkdk4412_gpio_keys,
	&samsung_asoc_idma,
#if defined(CONFIG_SND_SOC_HKDK_MAX98090)
	&hardkernel_audio_device,
#endif
#if defined(CONFIG_EXYNOS_THERMAL)
	&hkdk4412_tmu,
#endif
#if defined(CONFIG_ODROID_U2_FAN)
	&s3c_device_timer[0],
	&odroid_fan,
#endif
};

#if defined(CONFIG_S5P_DEV_TV)
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

static void __init hkdk4412_map_io(void)
{
	clk_xusbxti.rate = 24000000;

	exynos_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(hkdk4412_uartcfgs, ARRAY_SIZE(hkdk4412_uartcfgs));
}

static void __init hkdk4412_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 64 << 20, 0x51000000, 64 << 20);
}

#if defined(CONFIG_S5P_DEV_TV)
/* I2C module and id for HDMIPHY */
static struct i2c_board_info hdmiphy_info = {
	I2C_BOARD_INFO("s5p_hdmiphy", 0x38),
};
#endif



static void __init hkdk4412_gpio_init(void)
{
	/* Peripheral power enable (P3V3) */
	gpio_request_one(EXYNOS4_GPA1(1), GPIOF_OUT_INIT_HIGH, "p3v3_en");

	/* Power on/off button */
	s3c_gpio_cfgpin(EXYNOS4_GPX1(3), S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(EXYNOS4_GPX1(3), S3C_GPIO_PULL_NONE);
}

static void hkdk4412_power_off(void)
{
	pr_emerg("Bye...\n");

	writel(0x5200, S5P_PS_HOLD_CONTROL);
	while (1) {
		pr_emerg("%s : should not reach here!\n", __func__);
		msleep(1000);
	}
}

static int hkdk4412_reboot_notifier(struct notifier_block *this, unsigned long code, void *_cmd) {
	pr_emerg("exynos4-reboot: Notifier called\n");

	__raw_writel(0, S5P_INFORM4);

        // eMMC HW_RST  
        gpio_request(EXYNOS4_GPK1(2), "GPK1");
        gpio_direction_output(EXYNOS4_GPK1(2), 0);
        msleep(150);
        gpio_direction_output(EXYNOS4_GPK1(2), 1);
        gpio_free(EXYNOS4_GPK1(2));
	msleep(150);
        return NOTIFY_DONE;
}	


static struct notifier_block hkdk4412_reboot_notifier_nb = {
	.notifier_call = hkdk4412_reboot_notifier,
};

static void __init hkdk4412_machine_init(void)
{
	hkdk4412_gpio_init();

	/* Register power off function */
	pm_power_off = hkdk4412_power_off;

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, hkdk4412_i2c_devs0,
				ARRAY_SIZE(hkdk4412_i2c_devs0));

	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, hkdk4412_i2c_devs1,
				ARRAY_SIZE(hkdk4412_i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, hkdk4412_i2c_devs2,
				ARRAY_SIZE(hkdk4412_i2c_devs2));

	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, hkdk4412_i2c_devs3,
				ARRAY_SIZE(hkdk4412_i2c_devs3));

	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, hkdk4412_i2c_devs7,
				ARRAY_SIZE(hkdk4412_i2c_devs7));

	s3c_sdhci2_set_platdata(&hkdk4412_hsmmc2_pdata);

	exynos4_setup_dwmci_cfg_gpio(NULL, MMC_BUS_WIDTH_8);
	exynos_dwmci_set_platdata(&hkdk4412_dwmci_pdata);

	hkdk4412_ehci_init();
	hkdk4412_ohci_init();
	s3c_hsotg_set_platdata(&hkdk4412_hsotg_pdata);

#ifdef CONFIG_LCD_LP101WH1
//        s5p_fimd0_set_platdata(&hkdk4412_fb_pdata);
#endif
        

#if defined(CONFIG_S5P_DEV_TV)
	s5p_i2c_hdmiphy_set_platdata(NULL);
	s5p_hdmi_set_platdata(&hdmiphy_info, NULL, 0, EXYNOS4_GPX3(7));
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
	/* FIXME: hdmiphy i2c adapter has dynamic ID, and setting it to 8 causes
	 * a failure to initialize (can't find clock?). so for now we are relying
	 * on the hdmiphy i2c adapter being dynamically assigned address 8. */
	i2c_register_board_info(8, &hdmiphy_info, 1);
#endif

#ifdef CONFIG_LCD_LP101WH1
	s5p_device_fimd0.dev.platform_data = &drm_fimd_pdata;
	exynos4_fimd0_gpio_setup_24bpp();
#endif
	platform_add_devices(hkdk4412_devices, ARRAY_SIZE(hkdk4412_devices));

	samsung_bl_set(&hkdk4412_bl_gpio_info, &hkdk4412_bl_data);

	register_reboot_notifier(&hkdk4412_reboot_notifier_nb);
}

#if defined(CONFIG_ODROID_X)
MACHINE_START(ODROIDX, "ODROIDX")
#elif defined(CONFIG_ODROID_X2)
MACHINE_START(ODROIDX, "ODROIDX2")
#elif defined(CONFIG_ODROID_U2)
MACHINE_START(ODROIDX, "ODROIDU2")
#endif
	/* Maintainer: Dongjin Kim <dongjin.kim@agreeyamobiity.net> */
	.atag_offset	= 0x100,
	.smp		= smp_ops(exynos_smp_ops),
	.init_irq	= exynos4_init_irq,
	.init_early	= exynos_firmware_init,
	.map_io		= hkdk4412_map_io,
	.handle_irq	= gic_handle_irq,
	.init_machine	= hkdk4412_machine_init,
	.init_late	= exynos_init_late,
	.timer		= &exynos4_timer,
	.restart	= exynos4_restart,
	.reserve	= &hkdk4412_reserve,
MACHINE_END
