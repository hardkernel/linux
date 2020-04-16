/*
 * drivers/amlogic/drm/am_meson_fbdev.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>

#include "am_meson_drv.h"
#include "am_meson_gem.h"
#include "am_meson_fb.h"
#include "am_meson_fbdev.h"

#define PREFERRED_BPP		32
#define MESON_DRM_MAX_CONNECTOR	2

static int am_meson_fbdev_mmap(struct fb_info *info,
	struct vm_area_struct *vma)
{
	struct drm_fb_helper *helper = info->par;
	struct meson_drm *private;
	struct am_meson_gem_object *meson_gem;

	private = helper->dev->dev_private;
	meson_gem = container_of(private->fbdev_bo,
		struct am_meson_gem_object, base);

	return am_meson_gem_object_mmap(meson_gem, vma);
}

static int am_meson_drm_fbdev_sync(struct fb_info *info)
{
	return 0;
}

static int am_meson_drm_fbdev_ioctl(struct fb_info *info,
	unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct fb_ops meson_drm_fbdev_ops = {
	.owner		= THIS_MODULE,
	.fb_mmap	= am_meson_fbdev_mmap,
	.fb_fillrect	= drm_fb_helper_cfb_fillrect,
	.fb_copyarea	= drm_fb_helper_cfb_copyarea,
	.fb_imageblit	= drm_fb_helper_cfb_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
	.fb_sync	= am_meson_drm_fbdev_sync,
	.fb_ioctl       = am_meson_drm_fbdev_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = am_meson_drm_fbdev_ioctl,
#endif
};

static int am_meson_drm_fbdev_create(struct drm_fb_helper *helper,
	struct drm_fb_helper_surface_size *sizes)
{
	struct meson_drm *private = helper->dev->dev_private;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	struct am_meson_gem_object *meson_obj;
	struct drm_framebuffer *fb;
	struct ion_client *client;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;
	void* vaddr;

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = ALIGN(sizes->surface_width * bytes_per_pixel, 64);
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

	size = mode_cmd.pitches[0] * mode_cmd.height;

	client = (struct ion_client *)private->gem_client;
	meson_obj = am_meson_gem_object_create(dev, 0, size, client);
	if (IS_ERR(meson_obj))
		return -ENOMEM;

	private->fbdev_bo = &meson_obj->base;

	vaddr = ion_map_kernel(client, meson_obj->handle);
	if (!vaddr) {
		ret = -ENOMEM;
		goto err_meson_gem_free_object;
	}

	fbi = drm_fb_helper_alloc_fbi(helper);
	if (IS_ERR(fbi)) {
		dev_err(dev->dev, "Failed to create framebuffer info.\n");
		ret = PTR_ERR(fbi);
		goto err_meson_gem_free_object;
	}

	helper->fb = am_meson_drm_framebuffer_init(dev, &mode_cmd,
						   private->fbdev_bo);
	if (IS_ERR(helper->fb)) {
		dev_err(dev->dev, "Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(helper->fb);
		goto err_release_fbi;
	}

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &meson_drm_fbdev_ops;
	fbi->screen_size = size;
	fbi->fix.smem_len = fbi->screen_size;
	fbi->screen_buffer = vaddr;
	/* Shamelessly leak the physical address to user-space */
	fbi->fix.smem_start =
		page_to_phys(virt_to_page(fbi->screen_buffer));

	fb = helper->fb;
	fb->bits_per_pixel = sizes->surface_bpp;
	fb->depth = sizes->surface_depth;

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, sizes->fb_width, sizes->fb_height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = 0;
	fbi->screen_size = size;
	fbi->fix.smem_len = size;

	DRM_DEBUG_KMS("FB [%dx%d]-%d offset=%ld size=%zu\n",
		      fb->width, fb->height, fb->depth, offset, size);

	fbi->skip_vt_switch = true;

	return 0;

err_release_fbi:
	drm_fb_helper_release_fbi(helper);
err_meson_gem_free_object:
	am_meson_gem_object_free(&meson_obj->base);
	return ret;
}

static const struct drm_fb_helper_funcs meson_drm_fb_helper_funcs = {
	.fb_probe = am_meson_drm_fbdev_create,
};

int am_meson_drm_fbdev_init(struct drm_device *dev)
{
	struct meson_drm *private = dev->dev_private;
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return -EINVAL;

	num_crtc = dev->mode_config.num_crtc;

	helper = devm_kzalloc(dev->dev, sizeof(*helper), GFP_KERNEL);
	if (!helper)
		return -ENOMEM;

	drm_fb_helper_prepare(dev, helper, &meson_drm_fb_helper_funcs);

	ret = drm_fb_helper_init(dev, helper, num_crtc,
		MESON_DRM_MAX_CONNECTOR);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize drm fb helper - %d.\n",
			ret);
		goto err_free;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to add connectors - %d.\n", ret);
		goto err_drm_fb_helper_fini;
	}

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to set initial hw config - %d.\n",
			ret);
		goto err_drm_fb_helper_fini;
	}

	private->fbdev_helper = helper;

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(helper);
err_free:
	kfree(helper);
	return ret;
}

void am_meson_drm_fbdev_fini(struct drm_device *dev)
{
	struct meson_drm *private = dev->dev_private;
	struct drm_fb_helper *helper = private->fbdev_helper;

	if (!helper)
		return;

	drm_fb_helper_unregister_fbi(helper);
	drm_fb_helper_release_fbi(helper);

	if (helper->fb)
		drm_framebuffer_unreference(helper->fb);

	drm_fb_helper_fini(helper);
}
