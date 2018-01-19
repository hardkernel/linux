/*
 * drivers/amlogic/amports/amvideocap.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/bug.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/major.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/amlogic/amports/amvideocap.h>
#include <linux/amlogic/amports/vframe.h>
#include <linux/amlogic/canvas/canvas.h>
#include <linux/amlogic/canvas/canvas_mgr.h>
#include <linux/amlogic/ge2d/ge2d.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/dma-mapping.h>

#include "amports_priv.h"
#include "amvideocap_priv.h"
/*
#if MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON6
#include <mach/mod_gate.h>
#endif
*/

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#define DRIVER_NAME "amvideocap"
#define MODULE_NAME "amvideocap"
#define DEVICE_NAME "amvideocap"

#define CAP_WIDTH_MAX      1920
#define CAP_HEIGHT_MAX     1080

#define BUF_SIZE_MAX      (0x600000) /* 1920 * 1088 * 4 */

MODULE_DESCRIPTION("Video Frame capture");
MODULE_AUTHOR("amlogic-bj");
MODULE_LICENSE("GPL");

#define AMCAP_MAX_OPEND 16
struct amvideocap_global_data {
	struct class *class;
	struct device *dev;
	struct device *micro_dev;
	int major;
	unsigned long phyaddr;
	unsigned long vaddr;
	unsigned long size;
	int opened_cnt;
	int flags;
	struct mutex lock;
	struct video_frame_info want;
	u64 wait_max_ms;
};
static struct amvideocap_global_data amvideocap_gdata;
static inline struct amvideocap_global_data *getgctrl(void)
{
	return &amvideocap_gdata;
}

#define gLOCK() mutex_lock(&(getgctrl()->lock))
#define gUNLOCK() mutex_unlock(&(getgctrl()->lock))
#define gLOCKINIT() mutex_init(&(getgctrl()->lock))

/*********************************************************
 * /dev/amvideo APIs
 *********************************************************/
static int amvideocap_open(struct inode *inode, struct file *file)
{
	struct amvideocap_private *priv;
	dma_addr_t dma_handle;
	gLOCK();
	getgctrl()->size = BUF_SIZE_MAX;
	getgctrl()->vaddr =
		(unsigned long)dma_alloc_coherent(amports_get_dma_device(),
						  getgctrl()->size, &dma_handle,
						  GFP_KERNEL);
	if (!getgctrl()->vaddr) {
		gUNLOCK();
		pr_err("%s: failed to remap y addr\n", __func__);
		return -ENOMEM;
	}
	getgctrl()->phyaddr = (unsigned long)(dma_handle);

	if (!getgctrl()->phyaddr) {
		gUNLOCK();
		pr_err("Error,no memory have register for amvideocap\n");
		return -ENOMEM;
	}

	if (getgctrl()->opened_cnt > AMCAP_MAX_OPEND) {
		gUNLOCK();
		pr_err("Too Many opend video cap files\n");
		return -EMFILE;
	}
	priv = kmalloc(sizeof(struct amvideocap_private), GFP_KERNEL);
	if (!priv) {
		gUNLOCK();
		pr_err("alloc memory failed for amvideo cap\n");
		return -ENOMEM;
	}
	memset(priv, 0, sizeof(struct amvideocap_private));
	getgctrl()->opened_cnt++;
	gUNLOCK();
	file->private_data = priv;
	priv->phyaddr = getgctrl()->phyaddr;
	priv->vaddr = (u8 *) getgctrl()->vaddr;
	priv->want = getgctrl()->want;
	priv->src_rect.x = -1;
	priv->src_rect.y = -1;
	priv->src_rect.width = -1;
	priv->src_rect.height = -1;
	return 0;
}

static int amvideocap_release(struct inode *inode, struct file *file)
{
	struct amvideocap_private *priv = file->private_data;
	kfree(priv);
	if (getgctrl()->vaddr != 0) {
		dma_free_coherent(amports_get_dma_device(),
				  getgctrl()->size, (void *)getgctrl()->vaddr,
				  (dma_addr_t) getgctrl()->phyaddr);
		getgctrl()->vaddr = 0;
	}
	gLOCK();
	getgctrl()->opened_cnt--;

	gUNLOCK();
	return 0;
}

static int amvideocap_format_to_byte4pix(int fmt)
{
	switch (fmt) {
	case GE2D_FORMAT_S16_RGB_565:
		return 2;
	case GE2D_FORMAT_S24_BGR:
		return 3;
	case GE2D_FORMAT_S24_RGB:
		return 3;
	case GE2D_FORMAT_S32_ABGR:
		return 4;
	case GE2D_FORMAT_S32_RGBA:
		return 4;
	case GE2D_FORMAT_S32_BGRA:
		return 4;
	case GE2D_FORMAT_S32_ARGB:
		return 4;
	default:
		return 4;
	}
};

static int amvideocap_capture_get_frame
(struct amvideocap_private *priv, struct vframe_s **vf, int *cur_index)
{
	int ret;
	ret = ext_get_cur_video_frame(vf, cur_index);
	return ret;
}

static int amvideocap_capture_put_frame
(struct amvideocap_private *priv, struct vframe_s *vf)
{
	return ext_put_video_frame(vf);
}

static int amvideocap_get_input_format(struct vframe_s *vf)
{
	int format = GE2D_FORMAT_M24_NV21;
	/* pr_info("vf->type:0x%x\n", vf->type); */

	if ((vf->type & VIDTYPE_VIU_422) == VIDTYPE_VIU_422) {
		pr_info
		("*****************Into VIDTYPE_VIU_422******************\n");
		format = GE2D_FORMAT_S16_YUV422;
	} else if ((vf->type & VIDTYPE_VIU_444) == VIDTYPE_VIU_444) {
		pr_info
		("*****************Into VIDTYPE_VIU_444******************\n");
		format = GE2D_FORMAT_S24_YUV444;
	} else if ((vf->type & VIDTYPE_VIU_NV21) == VIDTYPE_VIU_NV21) {
		pr_info
		("****************Into VIDTYPE_VIU_NV21******************\n");
		format = GE2D_FORMAT_M24_NV21;
	}
	return format;
}

static ssize_t amvideocap_YUV_to_RGB(
	struct amvideocap_private *priv,
	u32 cur_index, int w, int h,
	struct vframe_s *vf, int outfmt)
{

	struct config_para_ex_s ge2d_config;
	struct canvas_s cs0, cs1, cs2, cd;
	const char *amvideocap_owner = "amvideocapframe";
	int canvas_idx = canvas_pool_map_alloc_canvas(amvideocap_owner);
	int y_index = cur_index & 0xff;
	int u_index = (cur_index >> 8) & 0xff;
	int v_index = (cur_index >> 16) & 0xff;
	int input_x, input_y, input_width, input_height, intfmt;
	unsigned long RGB_addr;
	struct ge2d_context_s *context = create_ge2d_work_queue();
	memset(&ge2d_config, 0, sizeof(struct config_para_ex_s));
	intfmt = amvideocap_get_input_format(vf);

	/* /unsigned long RGB_phy_addr=getgctrl()->phyaddr; */

	if (!priv->phyaddr) {
		pr_err("%s: failed to alloc y addr\n", __func__);
		return -1;
	}
	pr_info("RGB_phy_addr:%x\n", (unsigned int)priv->phyaddr);
	RGB_addr = (unsigned long)priv->vaddr;
	if (!RGB_addr) {
		pr_err("%s: failed to remap y addr\n", __func__);
		return -1;
	}
	pr_info("RGB_addr:%lx\n", RGB_addr);

	if (vf == NULL) {
		pr_err("%s: vf is NULL\n", __func__);
		return -1;
	}

	canvas_config(canvas_idx, priv->phyaddr,
				  w * amvideocap_format_to_byte4pix(outfmt), h,
				  CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
	if (priv->src_rect.x < 0 || priv->src_rect.x > vf->width)
		input_x = 0;
	else
		input_x = priv->src_rect.x;

	if (priv->src_rect.y < 0 || priv->src_rect.y > vf->height)
		input_y = 0;
	else
		input_y = priv->src_rect.y;

	if (priv->src_rect.width < 0)
		input_width = vf->width;
	else if ((priv->src_rect.x + priv->src_rect.width) > vf->width) {
		input_width =
			priv->src_rect.x + priv->src_rect.width - vf->width;
	} else
		input_width = priv->src_rect.width;

	if (priv->src_rect.height < 0)
		input_height = vf->height;
	else if ((priv->src_rect.y + priv->src_rect.height) > vf->height) {
		input_height =
			priv->src_rect.y + priv->src_rect.height - vf->height;
	} else
		input_height = priv->src_rect.height;

	if (intfmt == GE2D_FORMAT_S16_YUV422)
		input_height = input_height / 2;

	ge2d_config.alu_const_color = 0;
	ge2d_config.bitmask_en = 0;
	ge2d_config.src1_gb_alpha = 0;
	ge2d_config.dst_xy_swap = 0;

	canvas_read(y_index, &cs0);
	canvas_read(u_index, &cs1);
	canvas_read(v_index, &cs2);
	pr_info("y_index=[0x%x]  u_index=[0x%x] cur_index:%x\n", y_index,
			u_index, cur_index);
	ge2d_config.src_planes[0].addr = cs0.addr;
	ge2d_config.src_planes[0].w = cs0.width;
	ge2d_config.src_planes[0].h = cs0.height;
	ge2d_config.src_planes[1].addr = cs1.addr;
	ge2d_config.src_planes[1].w = cs1.width;
	ge2d_config.src_planes[1].h = cs1.height;
	ge2d_config.src_planes[2].addr = cs2.addr;
	ge2d_config.src_planes[2].w = cs2.width;
	ge2d_config.src_planes[2].h = cs2.height;
	pr_info("w=%d-height=%d cur_index:%x\n", cs0.width, cs0.height,
			cur_index);

	ge2d_config.src_key.key_enable = 0;
	ge2d_config.src_key.key_mask = 0;
	ge2d_config.src_key.key_mode = 0;
	ge2d_config.src_key.key_color = 0;

	ge2d_config.src_para.canvas_index = cur_index;
	ge2d_config.src_para.mem_type = CANVAS_TYPE_INVALID;
	ge2d_config.src_para.format = intfmt;
	ge2d_config.src_para.fill_color_en = 0;
	ge2d_config.src_para.fill_mode = 0;
	ge2d_config.src_para.x_rev = 0;
	ge2d_config.src_para.y_rev = 0;
	ge2d_config.src_para.color = 0;
	ge2d_config.src_para.top = input_x;
	ge2d_config.src_para.left = input_y;
	ge2d_config.src_para.width = input_width;
	ge2d_config.src_para.height = input_height;

	canvas_read(canvas_idx, &cd);
	pr_info("cd.addr:%x\n", (unsigned int)cd.addr);
	ge2d_config.dst_planes[0].addr = cd.addr;
	ge2d_config.dst_planes[0].w = cd.width;
	ge2d_config.dst_planes[0].h = cd.height;

	ge2d_config.dst_para.canvas_index = canvas_idx;
	ge2d_config.dst_para.mem_type = CANVAS_TYPE_INVALID;
	ge2d_config.dst_para.format = outfmt;
	ge2d_config.dst_para.fill_color_en = 0;
	ge2d_config.dst_para.fill_mode = 0;
	ge2d_config.dst_para.x_rev = 0;
	ge2d_config.dst_para.y_rev = 0;
	ge2d_config.dst_xy_swap = 0;
	ge2d_config.dst_para.color = 0;
	ge2d_config.dst_para.top = 0;
	ge2d_config.dst_para.left = 0;
	ge2d_config.dst_para.width = w;
	ge2d_config.dst_para.height = h;

	if (ge2d_context_config_ex(context, &ge2d_config) < 0) {
		pr_err("++ge2d configing error.\n");
		return -1;
	}

	stretchblt_noalpha(context,
					   0,
					   0,
					   ge2d_config.src_para.width,
					   ge2d_config.src_para.height,
					   0,
					   0,
					   ge2d_config.dst_para.width,
					   ge2d_config.dst_para.height);
	if (context) {
		destroy_ge2d_work_queue(context);
		context = NULL;
	}
	if (canvas_idx)
		canvas_pool_map_free_canvas(canvas_idx);
	return 0;
	/* vfs_write(video_rgb_filp,RGB_addr,size, &video_yuv_pos); */
}

static int amvideocap_capture_one_frame_l(
		struct amvideocap_private *priv,
		int curindex, int w, int h,
		struct vframe_s *vf, int outge2dfmt)
{
	int ret;
	/* TODO: mod gate */
	/* switch_mod_gate_by_name("ge2d", 1); */
	ret = amvideocap_YUV_to_RGB(priv, curindex, w, h, vf, outge2dfmt);
	/* TODO: mod gate */
	/* switch_mod_gate_by_name("ge2d", 0); */
	return ret;
}

static int amvideocap_capture_one_frame(
	struct amvideocap_private *priv,
	struct vframe_s *vfput, int index)
{
	int w, h, ge2dfmt;
	int curindex;
	struct vframe_s *vf = vfput;
	int ret = 0;
	pr_info("%s:start vf=%p,index=%x\n", __func__, vf, index);
	if (!vf)
		ret = amvideocap_capture_get_frame(priv, &vf, &curindex);
	else
		curindex = index;
	if (ret < 0 || !vf)
		return -EAGAIN;
	pr_info("%s: get vf type=%x\n", __func__, vf->type);

#define CHECK_AND_SETVAL(want, def) ((want) > 0 ? (want) : (def))
	ge2dfmt = CHECK_AND_SETVAL(priv->want.fmt, vf->type);
	w = CHECK_AND_SETVAL(priv->want.width, vf->width);
	h = CHECK_AND_SETVAL(priv->want.height, vf->height);
#undef	CHECK_AND_SETVAL

	w = (w < CAP_WIDTH_MAX) ? w : CAP_WIDTH_MAX;
	h = (h < CAP_HEIGHT_MAX) ? h : CAP_HEIGHT_MAX;

	ret = amvideocap_capture_one_frame_l(priv, curindex, w, h, vf,
						priv->want.fmt);
	amvideocap_capture_put_frame(priv, vf);

	if (!ret) {
		pr_info("%s: capture ok priv->want.fmt=%d\n", __func__,
				priv->want.fmt);
		priv->state = AMVIDEOCAP_STATE_FINISHED_CAPTURE;
		priv->src.width = vf->width;
		priv->src.height = vf->height;
		priv->out.timestamp_ms = vf->pts * 1000 / 90;
		priv->out.width = w;
		priv->out.height = h;
		priv->out.fmt = priv->want.fmt;
		priv->out.width_aligned = priv->out.width;
		priv->out.byte_per_pix = amvideocap_format_to_byte4pix(
			priv->out.fmt);	/* RGBn */
	} else
		priv->state = AMVIDEOCAP_STATE_ERROR;
	pr_info("amvideocap_capture_one_frame priv->state=%d\n", priv->state);
	return ret;
}

static int amvideocap_capture_one_frame_callback(
		unsigned long data,
		struct vframe_s *vfput,
		int index)
{
	struct amvideocap_req_data *reqdata =
		(struct amvideocap_req_data *)data;
	amvideocap_capture_one_frame(reqdata->privdata, vfput, index);
	return 0;
}

static int amvideocap_capture_one_frame_wait(
		struct amvideocap_private *priv,
		int waitms)
{
	unsigned long timeout = jiffies + waitms * HZ / 1000;
	int ret = 0;
	struct amvideocap_req_data reqdata;
	struct amvideocap_req req;
	priv->sended_end_frame_cap_req = 0;
	priv->state = AMVIDEOCAP_STATE_ON_CAPTURE;
	do {
		if (ret == -EAGAIN)
			msleep(100);
		if (priv->want.at_flags == CAP_FLAG_AT_END) {
			if (!priv->sended_end_frame_cap_req) {
				reqdata.privdata = priv;
				req.callback =
					amvideocap_capture_one_frame_callback;
				req.data = (unsigned long)&reqdata;
				req.at_flags = priv->want.at_flags;
				req.timestamp_ms = priv->want.timestamp_ms;
				priv->sended_end_frame_cap_req =
					!ext_register_end_frame_callback(&req);
				ret = -EAGAIN;
			} else {
				if (priv->state ==
					AMVIDEOCAP_STATE_FINISHED_CAPTURE)
					ret = 0;
				else if (priv->state ==
						 AMVIDEOCAP_STATE_ON_CAPTURE)
					ret = -EAGAIN;
			}
		} else {
			ret = amvideocap_capture_one_frame(priv, NULL, 0);
			pr_info("amvideocap_capture_one_frame_wait ret=%d\n",
					ret);
		}
	} while (ret == -EAGAIN && time_before(jiffies, timeout));
	ext_register_end_frame_callback(NULL);	/*del req */
	return ret;
}

static long amvideocap_ioctl(struct file *file,
	unsigned int cmd, ulong arg)
{
	int ret = 0;
	struct amvideocap_private *priv = file->private_data;
	switch (cmd) {
	case AMVIDEOCAP_IOW_SET_WANTFRAME_FORMAT: {
		priv->want.fmt = arg;
		break;
	}
	case AMVIDEOCAP_IOW_SET_WANTFRAME_WIDTH: {
		priv->want.width = arg;
		break;
	}
	case AMVIDEOCAP_IOW_SET_WANTFRAME_HEIGHT: {
		priv->want.height = arg;
		break;
	}
	case AMVIDEOCAP_IOW_SET_WANTFRAME_TIMESTAMP_MS: {
		priv->want.timestamp_ms = arg;
		break;
	}
	case AMVIDEOCAP_IOW_SET_WANTFRAME_AT_FLAGS: {
		priv->want.at_flags = arg;
		break;
	}
	case AMVIDEOCAP_IOR_GET_FRAME_FORMAT: {
		if (copy_to_user
			((void *)arg, (void *)&priv->out.fmt,
			 sizeof(priv->out.fmt))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_FRAME_WIDTH: {
		if (copy_to_user
			((void *)arg, (void *)&priv->out.width,
			 sizeof(priv->out.width))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_FRAME_HEIGHT: {
		if (copy_to_user
			((void *)arg, (void *)&priv->out.height,
			 sizeof(priv->out.height))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_FRAME_TIMESTAMP_MS: {
		if (copy_to_user
			((void *)arg, (void *)&priv->out.timestamp_ms,
			 sizeof(priv->out.timestamp_ms))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_SRCFRAME_FORMAT: {
		if (copy_to_user
			((void *)arg, (void *)&priv->src.fmt,
			 sizeof(priv->src.fmt))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_SRCFRAME_WIDTH: {
		if (copy_to_user
			((void *)arg, (void *)&priv->src.width,
			 sizeof(priv->src.width))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_SRCFRAME_HEIGHT: {
		if (copy_to_user
			((void *)arg, (void *)&priv->src.height,
			 sizeof(priv->src.height))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOR_GET_STATE: {
		if (copy_to_user
			((void *)arg, (void *)&priv->state,
			 sizeof(priv->state))) {
			ret = -EFAULT;
			break;
		}
		break;
	}
	case AMVIDEOCAP_IOW_SET_WANTFRAME_WAIT_MAX_MS: {
		priv->wait_max_ms = arg;
		break;
	}
	case AMVIDEOCAP_IOW_SET_START_CAPTURE: {
		ret = amvideocap_capture_one_frame_wait(priv, arg);
		break;
	}
	case AMVIDEOCAP_IOW_SET_CANCEL_CAPTURE: {
		if (priv->sended_end_frame_cap_req) {
			ext_register_end_frame_callback(NULL);	/*del req */
			priv->sended_end_frame_cap_req = 0;
			priv->state = AMVIDEOCAP_STATE_INIT;
		}
		break;
	}
	case AMVIDEOCAP_IOR_SET_SRC_X: {
		priv->src_rect.x = arg;
		break;
	}
	case AMVIDEOCAP_IOR_SET_SRC_Y: {
		priv->src_rect.y = arg;
		break;
	}
	case AMVIDEOCAP_IOR_SET_SRC_WIDTH: {
		priv->src_rect.width = arg;
		break;
	}
	case AMVIDEOCAP_IOR_SET_SRC_HEIGHT: {
		priv->src_rect.height = arg;
		break;
	}
	default:
		pr_err("unkonw cmd=%x\n", cmd);
		ret = -1;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long amvideocap_compat_ioctl(struct file *file,
				unsigned int cmd, ulong arg)
{
	return amvideocap_ioctl(file, cmd, (ulong)compat_ptr(arg));
}
#endif
static int amvideocap_mmap(struct file *file,
	struct vm_area_struct *vma)
{
	struct amvideocap_private *priv = file->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned vm_size = vma->vm_end - vma->vm_start;
	if (!priv->phyaddr)
		return -EIO;

	if (vm_size == 0)
		return -EAGAIN;
	/* pr_info("mmap:%x\n",vm_size); */
	off += priv->phyaddr;

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP | VM_IO;
	if (remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		pr_err("set_cached: failed remap_pfn_range\n");
		return -EAGAIN;
	}
	pr_info("amvideocap_mmap ok\n");
	return 0;
}

static ssize_t amvideocap_read(struct file *file, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct amvideocap_private *priv = file->private_data;
	int waitdelay;
	int ret = 0;
	int copied;
	loff_t pos;
	pos = *ppos;
	if (priv->wait_max_ms > 0)
		waitdelay = priv->wait_max_ms;
	else {
		if (priv->want.at_flags == CAP_FLAG_AT_END)	/*need end */
			waitdelay = file->f_flags & O_NONBLOCK ? HZ : HZ * 100;
		else
			waitdelay =
				file->f_flags & O_NONBLOCK ?
				HZ / 100 : HZ * 10;
	}
	if (!pos) {		/*trigger a new capture, */
		pr_info("start amvideocap_read waitdelay=%d\n", waitdelay);
		ret = amvideocap_capture_one_frame_wait(priv, waitdelay);
		pr_info("amvideocap_read=%d,priv->state=%d,priv->vaddr=%p\n",
				ret, priv->state, priv->vaddr);
		if ((ret == 0)
			&& (priv->state == AMVIDEOCAP_STATE_FINISHED_CAPTURE)
			&& (priv->vaddr != NULL)) {
			int size = min((int)count, (priv->out.byte_per_pix *
						priv->out.width_aligned *
						priv->out.height));
			pr_info
			("priv->out_width=%d priv->out_height=%d",
			 priv->out.width, priv->out.height);
			pr_info
			(" priv->outfmt_byteppix=%d, size=%d\n",
			 priv->out.byte_per_pix, size);
			copied = copy_to_user(buf, priv->vaddr, size);
			if (copied) {
				pr_err
				("amvideocap_read %d copy_to_user failed\n",
				 size);
			}
			ret = size;
		}
	} else {
		/*read from old capture. */
		if (priv->state != AMVIDEOCAP_STATE_FINISHED_CAPTURE
			|| priv->vaddr == NULL) {
			ret = 0;	/*end. */
		} else {
			int maxsize =
				priv->out.byte_per_pix *
				priv->out.width_aligned * priv->out.height;
			if (pos < maxsize) {
				int rsize =
					min((int)count, (maxsize - (int)pos));
				copied =
					copy_to_user(buf, priv->vaddr + pos,
					rsize);
				if (copied) {
					pr_err
					("%s %d copy_to_user failed\n",
					 __func__, rsize);
				}
				ret = rsize;
			} else {
				ret = 0;	/*end. */
			}
		}
	}
	if (ret > 0) {
		pos += ret;
		*ppos = pos;
	}
	return ret;
}

static const struct file_operations amvideocap_fops = {
	.owner = THIS_MODULE,
	.open = amvideocap_open,
	.read = amvideocap_read,
	.mmap = amvideocap_mmap,
	.release = amvideocap_release,
	.unlocked_ioctl = amvideocap_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = amvideocap_compat_ioctl,
#endif
	/* /  .poll     = amvideocap_poll, */
};

static ssize_t show_amvideocap_config(struct class *class,
					  struct class_attribute *attr,
					  char *buf)
{
	char *pbuf = buf;
	ssize_t size = 0;
	pbuf += sprintf(pbuf, "at_flags:%d\n", getgctrl()->want.at_flags);
	pbuf += sprintf(pbuf, "timestampms:%lld\n",
					getgctrl()->want.timestamp_ms);
	pbuf += sprintf(pbuf, "width:%d\n", getgctrl()->want.width);
	pbuf += sprintf(pbuf, "height:%d\n", getgctrl()->want.height);
	pbuf += sprintf(pbuf, "format:%d\n", getgctrl()->want.fmt);
	pbuf += sprintf(pbuf, "waitmaxms:%lld\n", getgctrl()->wait_max_ms);
	size = pbuf - buf;
	return size;
}

static ssize_t store_amvideocap_config(struct class *class,
						struct class_attribute *attr,
						const char *buf, size_t size)
{
	int ret, val;
	const char *pbuf = buf;
	for (; pbuf && pbuf[0] != '\0';) {
#ifdef GETVAL
#undef GETVAL
#endif
#define GETVAL(tag, v) do { \
		val = 0;\
		ret = sscanf(pbuf, tag ":%d", &val); \
		if (ret == 1) { \
			v = val; \
			pbuf += strlen(tag); \
			goto tonext; \
		}; \
	} while (0)
		GETVAL("timestamp", getgctrl()->want.timestamp_ms);
		GETVAL("width", getgctrl()->want.width);
		GETVAL("height", getgctrl()->want.height);
		GETVAL("format", getgctrl()->want.fmt);
		GETVAL("waitmaxms", getgctrl()->wait_max_ms);
		GETVAL("at_flags", getgctrl()->want.at_flags);
#undef GETVAL
		pbuf++;
tonext:
		while (pbuf[0] != ';' && pbuf[0] != '\0')
			pbuf++;
		if (pbuf[0] == ';')
			pbuf++;
	}
	return size;
}

static struct class_attribute amvideocap_class_attrs[] = {
	__ATTR(config, S_IRUGO | S_IWUSR | S_IWGRP, show_amvideocap_config,
	store_amvideocap_config),
	__ATTR_NULL
};

static struct class amvideocap_class = {
		.name = MODULE_NAME,
		.class_attrs = amvideocap_class_attrs,
	};

s32 amvideocap_register_memory(unsigned char *phybufaddr,
					int phybufsize)
{
	pr_info("amvideocap_register_memory %p %d\n", phybufaddr, phybufsize);
	getgctrl()->phyaddr = (unsigned long)phybufaddr;
	getgctrl()->size = (unsigned long)phybufsize;
	getgctrl()->vaddr = 0;
	return 0;
}

s32 amvideocap_dev_register(void)
{
	s32 r = 0;
	pr_info("amvideocap_dev_register\n");

	gLOCKINIT();
	r = register_chrdev(0, DEVICE_NAME, &amvideocap_fops);
	if (r < 0) {
		pr_err("Can't register major for amvideocap device\n");
		return r;
	}
	getgctrl()->major = r;
	r = class_register(&amvideocap_class);
	if (r) {
		pr_err("amvideocap class create fail.\n");
		goto err1;
	}
	getgctrl()->class = &amvideocap_class;
	getgctrl()->dev = device_create(getgctrl()->class,
					NULL, MKDEV(getgctrl()->major, 0),
					NULL, DEVICE_NAME "0");
	if (getgctrl()->dev == NULL) {
		pr_err("amvideocap device_create fail.\n");
		r = -EEXIST;
		goto err2;
	}

	getgctrl()->wait_max_ms = 0;
	getgctrl()->want.fmt = GE2D_FORMAT_S24_RGB;
	getgctrl()->want.width = 0;
	getgctrl()->want.height = 0;
	getgctrl()->want.timestamp_ms = 0;
	/*get last frame */
	getgctrl()->want.at_flags = CAP_FLAG_AT_CURRENT;
	return 0;
err2:
	class_unregister(&amvideocap_class);
err1:
	unregister_chrdev(getgctrl()->major, DEVICE_NAME);
	return r;
}

s32 amvideocap_dev_unregister(void)
{
	device_destroy(getgctrl()->class, MKDEV(getgctrl()->major, 0));
	class_unregister(getgctrl()->class);
	unregister_chrdev(getgctrl()->major, DEVICE_NAME);

	return 0;
}

/*******************************************************************
 *
 * interface for Linux driver
 *
 * ******************************************************************/

/* for driver. */
static int amvideocap_probe(struct platform_device *pdev)
{
	amvideocap_dev_register();
	return 0;
}

static int amvideocap_remove(struct platform_device *plat_dev)
{
	/* struct rtc_device *rtc = platform_get_drvdata(plat_dev); */
	/* rtc_device_unregister(rtc); */
	/* device_remove_file(&plat_dev->dev, &dev_attr_irq); */
	amvideocap_dev_unregister();
	return 0;
}


static const struct of_device_id amlogic_amvideocap_dt_match[] = {
	{
		.compatible = "amlogic, amvideocap",
	},
	{},
};

/* general interface for a linux driver .*/
struct platform_driver amvideocap_drv = {
	.probe = amvideocap_probe,
	.remove = amvideocap_remove,
	.driver = {
		.name = "amvideocap",
		.of_match_table = amlogic_amvideocap_dt_match,
	}
};

static int __init amvideocap_init_module(void)
{

	pr_info("amvideocap_init_module\n");
	if ((platform_driver_register(&amvideocap_drv))) {
		pr_err("failed to register amstream module\n");
		return -ENODEV;
	}
	return 0;

}

static void __exit amvideocap_remove_module(void)
{
	platform_driver_unregister(&amvideocap_drv);
	pr_info("amvideocap module removed.\n");
}

module_init(amvideocap_init_module);
module_exit(amvideocap_remove_module);

MODULE_DESCRIPTION("AMLOGIC  amvideocap driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Wang Jian <jian.wang@amlogic.com>");
