/*
 * Driver for the Sino Wealth SSD1307 OLED controller
 *
 * Copyright 2020 Dongjin Kim <tobetter@gmail.com>
 *
 * This version is based on :
 *     linux/drivers/video/fbdevssd1307fb.c -- Solomon SSD1307 OLED controller
 *
 *     Copyright 2012 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>

#define SH1106_DATA			0x40
#define SH1106_COMMAND			0x80

#define SH1106_SET_ADDRESS_MODE		0x20
#define SH1106_SET_ADDRESS_MODE_HORIZONTAL	(0x00)
#define SH1106_SET_ADDRESS_MODE_VERTICAL	(0x01)
#define SH1106_SET_ADDRESS_MODE_PAGE		(0x02)
#define SH1106_SET_COL_RANGE		0x21
#define SH1106_SET_PAGE_RANGE		0x22
#define SH1106_CONTRAST			0x81
#define SH1106_CHARGE_PUMP		0x8d
#define SH1106_SEG_REMAP_ON		0xa1
#define SH1106_DISPLAY_OFF		0xae
#define SH1106_SET_MULTIPLEX_RATIO	0xa8
#define SH1106_DISPLAY_ON		0xaf
#define SH1106_START_PAGE_ADDRESS	0xb0
#define SH1106_SET_DISPLAY_OFFSET	0xd3
#define SH1106_SET_CLOCK_FREQ		0xd5
#define SH1106_SET_PRECHARGE_PERIOD	0xd9
#define SH1106_SET_COM_PINS_CONFIG	0xda
#define SH1106_SET_VCOMH		0xdb

#define MAX_CONTRAST			255

#define REFRESHRATE			20

static u_int refreshrate = REFRESHRATE;
module_param(refreshrate, uint, 0);

static u_int rotate;
module_param(rotate, uint, 0);

struct sh1106fb_par;

struct sh1106fb_deviceinfo {
	u32 default_vcomh;
	u32 default_dclk_div;
	u32 default_dclk_frq;
	int need_chargepump;
};

struct sh1106fb_par {
	u32 com_invdir;
	u32 com_lrremap;
	u32 com_offset;
	u32 com_seq;
	u32 contrast;
	u32 dclk_div;
	u32 dclk_frq;
	const struct sh1106fb_deviceinfo *device_info;
	struct i2c_client *client;
	u32 height;
	struct fb_info *info;
	u32 page_offset;
	u32 prechargep1;
	u32 prechargep2;
	u32 seg_remap;
	u32 vcomh;
	u32 width;
	u32 rotate;
};

struct sh1106fb_array {
	u8	type;
	u8	data[0];
};

static const struct fb_fix_screeninfo sh1106fb_fix = {
	.id		= "SH1106 OLED",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo sh1106fb_var = {
	.bits_per_pixel	= 1,
};

static struct sh1106fb_array *sh1106fb_alloc_array(u32 len, u8 type)
{
	struct sh1106fb_array *array;

	array = kzalloc(sizeof(struct sh1106fb_array) + len, GFP_KERNEL);
	if (!array)
		return NULL;

	array->type = type;

	return array;
}

static int sh1106fb_write_array(struct i2c_client *client,
				 struct sh1106fb_array *array, u32 len)
{
	int ret;

	len += sizeof(struct sh1106fb_array);

	ret = i2c_master_send(client, (u8 *)array, len);
	if (ret != len) {
		dev_err(&client->dev, "Couldn't send I2C command.\n");
		return ret;
	}

	return 0;
}

static inline int sh1106fb_write_cmd(struct i2c_client *client, u8 cmd)
{
	struct sh1106fb_array *array;
	int ret;

	array = sh1106fb_alloc_array(1, SH1106_COMMAND);
	if (!array)
		return -ENOMEM;

	array->data[0] = cmd;

	ret = sh1106fb_write_array(client, array, 1);
	kfree(array);

	return ret;
}

static void sh1106fb_update_display(struct sh1106fb_par *par)
{
	struct sh1106fb_array *array;
	u8 *vmem = par->info->screen_base;
	int i, j, k;

	array = sh1106fb_alloc_array(par->width * par->height / 8,
				      SH1106_DATA);
	if (!array)
		return;

	/*
	 * The screen is divided in pages, each having a height of 8
	 * pixels, and the width of the screen. When sending a byte of
	 * data to the controller, it gives the 8 bits for the current
	 * column. I.e, the first byte are the 8 bits of the first
	 * column, then the 8 bits for the second column, etc.
	 *
	 *
	 * Representation of the screen, assuming it is 5 bits
	 * wide. Each letter-number combination is a bit that controls
	 * one pixel.
	 *
	 * A0 A1 A2 A3 A4
	 * B0 B1 B2 B3 B4
	 * C0 C1 C2 C3 C4
	 * D0 D1 D2 D3 D4
	 * E0 E1 E2 E3 E4
	 * F0 F1 F2 F3 F4
	 * G0 G1 G2 G3 G4
	 * H0 H1 H2 H3 H4
	 *
	 * If you want to update this screen, you need to send 5 bytes:
	 *  (1) A0 B0 C0 D0 E0 F0 G0 H0
	 *  (2) A1 B1 C1 D1 E1 F1 G1 H1
	 *  (3) A2 B2 C2 D2 E2 F2 G2 H2
	 *  (4) A3 B3 C3 D3 E3 F3 G3 H3
	 *  (5) A4 B4 C4 D4 E4 F4 G4 H4
	 */

	if (par->rotate == 0) {
		for (i = 0; i < (par->height / 8); i++) {
			for (j = 0; j < par->width; j++) {
				u32 array_idx = i * par->width + j;

				array->data[array_idx] = 0;
				for (k = 0; k < 8; k++) {
					u32 page_length = par->width * i;
					u32 index = page_length + (par->width * k + j) / 8;
					u8 byte = *(vmem + index);
					u8 bit = byte & (1 << (j % 8));
					bit = bit >> (j % 8);
					array->data[array_idx] |= bit << k;
				}
			}
		}
	} else if (par->rotate == 180) {
		for (i = 0; i < (par->height / 8); i++) {
			for (j = 0; j < par->width; j++) {
				u32 array_idx = i * par->width + j;

				array->data[array_idx] = 0;
				for (k = 0; k < 8; k++) {
					u32 page_length = par->width * (7 - i);
					u32 index = page_length + (par->width * (7 - k) + j) / 8;
					u8 byte = *(vmem + index);
					u8 bit = byte & (1 << (j % 8));
					bit = bit >> (j % 8);
					array->data[array_idx] |= bit << k;
				}
			}
		}
	}

	sh1106fb_write_array(par->client, array, par->width * par->height / 8);
	kfree(array);
}

static ssize_t sh1106fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct sh1106fb_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		return -EFAULT;

	sh1106fb_update_display(par);

	*ppos += count;

	return count;
}

static int sh1106fb_blank(int blank_mode, struct fb_info *info)
{
	struct sh1106fb_par *par = info->par;

	if (blank_mode != FB_BLANK_UNBLANK)
		return sh1106fb_write_cmd(par->client, SH1106_DISPLAY_OFF);
	else
		return sh1106fb_write_cmd(par->client, SH1106_DISPLAY_ON);
}

static void sh1106fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct sh1106fb_par *par = info->par;
	sys_fillrect(info, rect);
	sh1106fb_update_display(par);
}

static void sh1106fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct sh1106fb_par *par = info->par;
	sys_copyarea(info, area);
	sh1106fb_update_display(par);
}

static void sh1106fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct sh1106fb_par *par = info->par;
	sys_imageblit(info, image);
	sh1106fb_update_display(par);
}

static struct fb_ops sh1106fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= sh1106fb_write,
	.fb_blank	= sh1106fb_blank,
	.fb_fillrect	= sh1106fb_fillrect,
	.fb_copyarea	= sh1106fb_copyarea,
	.fb_imageblit	= sh1106fb_imageblit,
};

static void sh1106fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	sh1106fb_update_display(info->par);
}

static int sh1106fb_init(struct sh1106fb_par *par)
{
	int ret;
	u32 precharge, dclk, com_invdir, compins;

	/* Set initial contrast */
	ret = sh1106fb_write_cmd(par->client, SH1106_CONTRAST);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, par->contrast);
	if (ret < 0)
		return ret;

	/* Set segment re-map */
	if (par->seg_remap) {
		ret = sh1106fb_write_cmd(par->client, SH1106_SEG_REMAP_ON);
		if (ret < 0)
			return ret;
	};

	/* Set COM direction */
	com_invdir = 0xc0 | (par->com_invdir & 0x1) << 3;
	ret = sh1106fb_write_cmd(par->client,  com_invdir);
	if (ret < 0)
		return ret;

	/* Set multiplex ratio value */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_MULTIPLEX_RATIO);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, par->height - 1);
	if (ret < 0)
		return ret;

	/* set display offset value */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_DISPLAY_OFFSET);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, par->com_offset);
	if (ret < 0)
		return ret;

	/* Set clock frequency */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_CLOCK_FREQ);
	if (ret < 0)
		return ret;

	dclk = ((par->dclk_div - 1) & 0xf) | (par->dclk_frq & 0xf) << 4;
	ret = sh1106fb_write_cmd(par->client, dclk);
	if (ret < 0)
		return ret;

	/* Set precharge period in number of ticks from the internal clock */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_PRECHARGE_PERIOD);
	if (ret < 0)
		return ret;

	precharge = (par->prechargep1 & 0xf) | (par->prechargep2 & 0xf) << 4;
	ret = sh1106fb_write_cmd(par->client, precharge);
	if (ret < 0)
		return ret;

	/* Set COM pins configuration */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_COM_PINS_CONFIG);
	if (ret < 0)
		return ret;

	compins = 0x02 | !(par->com_seq & 0x1) << 4
				   | (par->com_lrremap & 0x1) << 5;
	ret = sh1106fb_write_cmd(par->client, compins);
	if (ret < 0)
		return ret;

	/* Set VCOMH */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_VCOMH);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, par->vcomh);
	if (ret < 0)
		return ret;

	/* Turn on the DC-DC Charge Pump */
	ret = sh1106fb_write_cmd(par->client, SH1106_CHARGE_PUMP);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client,
		BIT(4) | (par->device_info->need_chargepump ? BIT(2) : 0));
	if (ret < 0)
		return ret;

	/* Switch to horizontal addressing mode */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_ADDRESS_MODE);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client,
				  SH1106_SET_ADDRESS_MODE_HORIZONTAL);
	if (ret < 0)
		return ret;

	/* Set column range */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_COL_RANGE);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, 0x0);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, par->width - 1);
	if (ret < 0)
		return ret;

	/* Set page range */
	ret = sh1106fb_write_cmd(par->client, SH1106_SET_PAGE_RANGE);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client, par->page_offset);
	if (ret < 0)
		return ret;

	ret = sh1106fb_write_cmd(par->client,
				  par->page_offset + (par->height / 8) - 1);
	if (ret < 0)
		return ret;

	/* Turn on the display */
	ret = sh1106fb_write_cmd(par->client, SH1106_DISPLAY_ON);
	if (ret < 0)
		return ret;

	return 0;
}

static struct sh1106fb_deviceinfo sh1106fb_sh1106_deviceinfo = {
	.default_vcomh = 0x40,
	.default_dclk_div = 2,
	.default_dclk_frq = 12,
	.need_chargepump = 1,
};

static const struct of_device_id sh1106fb_of_match[] = {
	{
		.compatible = "sinowealth,sh1106-i2c",
		.data = (void *)&sh1106fb_sh1106_deviceinfo,
	},
	{},
};
MODULE_DEVICE_TABLE(of, sh1106fb_of_match);

static int sh1106fb_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct fb_info *info;
	struct device_node *node = client->dev.of_node;
	struct fb_deferred_io *sh1106fb_defio;
	u32 vmem_size;
	struct sh1106fb_par *par;
	u8 *vmem;
	int ret;

	if (!node) {
		dev_err(&client->dev, "No device tree data found!\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct sh1106fb_par), &client->dev);
	if (!info) {
		dev_err(&client->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}

	par = info->par;
	par->info = info;
	par->client = client;

	par->device_info = of_device_get_match_data(&client->dev);

	if (of_property_read_u32(node, "width", &par->width))
		par->width = 96;

	if (of_property_read_u32(node, "height", &par->height))
		par->height = 16;

	if (of_property_read_u32(node, "page-offset", &par->page_offset))
		par->page_offset = 0;

	if (of_property_read_u32(node, "com-offset", &par->com_offset))
		par->com_offset = 0;

	if (of_property_read_u32(node, "prechargep1", &par->prechargep1))
		par->prechargep1 = 2;

	if (of_property_read_u32(node, "prechargep2", &par->prechargep2))
		par->prechargep2 = 2;

	if (of_property_read_u32(node, "rotate", &par->rotate))
		par->rotate = rotate;

	par->seg_remap = !of_property_read_bool(node, "segment-no-remap");
	par->com_seq = of_property_read_bool(node, "com-seq");
	par->com_lrremap = of_property_read_bool(node, "com-lrremap");
	par->com_invdir = of_property_read_bool(node, "com-invdir");

	par->contrast = 127;
	par->vcomh = par->device_info->default_vcomh;

	/* Setup display timing */
	par->dclk_div = par->device_info->default_dclk_div;
	par->dclk_frq = par->device_info->default_dclk_frq;

	vmem_size = par->width * par->height / 8;

	vmem = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
					get_order(vmem_size));
	if (!vmem) {
		dev_err(&client->dev, "Couldn't allocate graphical memory.\n");
		ret = -ENOMEM;
		goto fb_alloc_error;
	}

	sh1106fb_defio = devm_kzalloc(&client->dev, sizeof(struct fb_deferred_io), GFP_KERNEL);
	if (!sh1106fb_defio) {
		dev_err(&client->dev, "Couldn't allocate deferred io.\n");
		ret = -ENOMEM;
		goto fb_alloc_error;
	}

	sh1106fb_defio->delay = HZ / refreshrate;
	sh1106fb_defio->deferred_io = sh1106fb_deferred_io;

	info->fbops = &sh1106fb_ops;
	info->fix = sh1106fb_fix;
	info->fix.line_length = par->width / 8;
	info->fbdefio = sh1106fb_defio;

	info->var = sh1106fb_var;
	info->var.xres = par->width;
	info->var.xres_virtual = par->width;
	info->var.yres = par->height;
	info->var.yres_virtual = par->height;

	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = __pa(vmem);
	info->fix.smem_len = vmem_size;

	fb_deferred_io_init(info);

	i2c_set_clientdata(client, info);

	ret = sh1106fb_init(par);
	if (ret)
		goto reset_oled_error;

	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&client->dev, "Couldn't register the framebuffer\n");
		goto panel_init_error;
	}

	dev_info(&client->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n", info->node, info->fix.id, vmem_size);

	return 0;

panel_init_error:
reset_oled_error:
	fb_deferred_io_cleanup(info);
fb_alloc_error:
	framebuffer_release(info);
	return ret;
}

static int sh1106fb_remove(struct i2c_client *client)
{
	struct fb_info *info = i2c_get_clientdata(client);
	struct sh1106fb_par *par = info->par;

	sh1106fb_write_cmd(par->client, SH1106_DISPLAY_OFF);

	unregister_framebuffer(info);
	fb_deferred_io_cleanup(info);
	__free_pages(__va(info->fix.smem_start), get_order(info->fix.smem_len));
	framebuffer_release(info);

	return 0;
}

static const struct i2c_device_id sh1106fb_i2c_id[] = {
	{ "sh1106fb", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sh1106fb_i2c_id);

static struct i2c_driver sh1106fb_driver = {
	.probe = sh1106fb_probe,
	.remove = sh1106fb_remove,
	.id_table = sh1106fb_i2c_id,
	.driver = {
		.name = "sh1106fb",
		.of_match_table = sh1106fb_of_match,
	},
};

module_i2c_driver(sh1106fb_driver);

MODULE_DESCRIPTION("FB driver for the Sino Wealth SH1106 OLED controller");
MODULE_AUTHOR("Dongjin Kim <tobetter@gmail.com>");
MODULE_LICENSE("GPL");
