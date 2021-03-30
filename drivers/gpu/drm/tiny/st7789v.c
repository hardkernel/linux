// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix ST7789V
 * display controller in SPI mode.
 *
 * Copyright 2017 David Lechner <david@lechnology.com>
 * Copyright (C) 2019 Glider bvba
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>

#define ST7789V_PORCTRL     0xb2
#define ST7789V_GCTRL       0xb7
#define ST7789V_VCOMS       0xbb
#define ST7789V_LCMCTRL     0xc0
#define ST7789V_VDVVRHEN    0xc2
#define ST7789V_VRHS        0xc3
#define ST7789V_VDVS        0xc4
#define ST7789V_FRCTRL2     0xc6
#define ST7789V_PWCTRL1     0xd0
#define ST7789V_PVGAMCTRL   0xe0
#define ST7789V_NVGAMCTRL   0xe1

#define ST7789V_MY	BIT(7)
#define ST7789V_MX	BIT(6)
#define ST7789V_MV	BIT(5)
#define ST7789V_RGB	BIT(3)

struct st7789v_cfg {
	const struct drm_display_mode mode;
	unsigned int left_offset;
	unsigned int top_offset;
	unsigned int write_only:1;
	unsigned int rgb:1;		/* RGB (vs. BGR) */
};

struct st7789v_priv {
	struct mipi_dbi_dev dbidev;	/* Must be first for .release() */
	const struct st7789v_cfg *cfg;
};

static void st7789v_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct st7789v_priv *priv = container_of(dbidev, struct st7789v_priv,
						 dbidev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;
	u8 addr_mode;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_reset(dbidev);
	if (ret)
		goto out_exit;

	msleep(150);

	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(100);

	switch (dbidev->rotation) {
	default:
		addr_mode = 0;
		break;
	case 90:
		addr_mode = ST7789V_MY | ST7789V_MV;
		break;
	case 180:
		addr_mode = ST7789V_MX | ST7789V_MY;
		break;
	case 270:
		addr_mode = ST7789V_MX | ST7789V_MV;
		break;
	}

	if (priv->cfg->rgb)
		addr_mode |= ST7789V_RGB;

	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT,
			 MIPI_DCS_PIXEL_FMT_16BIT);
	mipi_dbi_command(dbi, MIPI_DCS_ENTER_INVERT_MODE);
	mipi_dbi_command(dbi, ST7789V_PORCTRL, 0x0c, 0x0c, 0x00, 0x33, 0x33);
	mipi_dbi_command(dbi, ST7789V_GCTRL, 0x35);
	mipi_dbi_command(dbi, ST7789V_VCOMS, 0x1f);
	mipi_dbi_command(dbi, ST7789V_LCMCTRL, 0x2c);
	mipi_dbi_command(dbi, ST7789V_VDVVRHEN, 0x01);
	mipi_dbi_command(dbi, ST7789V_VRHS, 0x12);
	mipi_dbi_command(dbi, ST7789V_VDVS, 0x20);
	mipi_dbi_command(dbi, ST7789V_FRCTRL2, 0x0f);
	mipi_dbi_command(dbi, ST7789V_PWCTRL1, 0xa4, 0xa1);
	mipi_dbi_command(dbi, ST7789V_PVGAMCTRL, 0xd0, 0x08, 0x11, 0x08, 0x0c,
				0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2d);
	mipi_dbi_command(dbi, ST7789V_NVGAMCTRL, 0xd0, 0x08, 0x10, 0x08, 0x06,
				0x06, 0x39, 0x44, 0x51, 0x0b, 0x16, 0x14, 0x2f, 0x31);
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(100);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);

out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7789v_pipe_funcs = {
	.enable		= st7789v_pipe_enable,
	.disable	= mipi_dbi_pipe_disable,
	.update		= mipi_dbi_pipe_update,
	.prepare_fb	= drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct st7789v_cfg ws_2inch_lcd_cfg = {
	.mode		= { DRM_SIMPLE_MODE(240, 320, 34, 43) },
	/* Cannot read from Waveshare 2inch lcd module" display via SPI */
	.write_only	= true,
	.rgb		= false,
};


DEFINE_DRM_GEM_CMA_FOPS(st7789v_fops);

static struct drm_driver st7789v_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7789v_fops,
	DRM_GEM_CMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7789v",
	.desc			= "Sitronix ST7789R",
	.date			= "20210310",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id st7789v_of_match[] = {
	{ .compatible = "waveshare,ws2inch", .data = &ws_2inch_lcd_cfg },
	{ },
};
MODULE_DEVICE_TABLE(of, st7789v_of_match);

static const struct spi_device_id st7789v_id[] = {
	{ "ws2inch", (uintptr_t)&ws_2inch_lcd_cfg },
	{ },
};
MODULE_DEVICE_TABLE(spi, st7789v_id);

static int st7789v_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	const struct st7789v_cfg *cfg;
	struct mipi_dbi_dev *dbidev;
	struct st7789v_priv *priv;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	cfg = device_get_match_data(&spi->dev);
	if (!cfg)
		cfg = (void *)spi_get_device_id(spi)->driver_data;

	priv = devm_drm_dev_alloc(dev, &st7789v_driver,
				  struct st7789v_priv, dbidev.drm);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	dbidev = &priv->dbidev;
	priv->cfg = cfg;

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	if (cfg->write_only)
		dbi->read_commands = NULL;

	dbidev->left_offset = cfg->left_offset;
	dbidev->top_offset = cfg->top_offset;

	ret = mipi_dbi_dev_init(dbidev, &st7789v_pipe_funcs, &cfg->mode,
				rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int st7789v_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void st7789v_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7789v_spi_driver = {
	.driver = {
		.name = "st7789v-dbi",
		.of_match_table = st7789v_of_match,
	},
	.id_table = st7789v_id,
	.probe = st7789v_probe,
	.remove = st7789v_remove,
	.shutdown = st7789v_shutdown,
};
module_spi_driver(st7789v_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7789V DRM driver");
MODULE_AUTHOR("Carlis <zhangxuezhi1@yulong.com>");
MODULE_LICENSE("GPL");
