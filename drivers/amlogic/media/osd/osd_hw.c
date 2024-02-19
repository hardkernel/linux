/*
 * drivers/amlogic/media/osd/osd_hw.c
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

/* Linux Headers */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/irqreturn.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <linux/amlogic/cpu_version.h>
/* Android Headers */

/* Amlogic Headers */
#include <linux/amlogic/media/vout/vinfo.h>
#include <linux/amlogic/media/vout/vout_notify.h>
#ifdef CONFIG_AMLOGIC_MEDIA_CANVAS
#include <linux/amlogic/media/canvas/canvas.h>
#include <linux/amlogic/media/canvas/canvas_mgr.h>
#endif
#ifdef CONFIG_AMLOGIC_VPU
#include <linux/amlogic/media/vpu/vpu.h>
#endif
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_VECM
#include <linux/amlogic/media/amvecm/ve.h>
#endif

#ifdef CONFIG_AMLOGIC_MEDIA_VIDEO
#include <linux/amlogic/media/vfm/vframe.h>
#include <linux/amlogic/media/video_sink/video.h>
#endif
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
#include <linux/amlogic/media/amdolbyvision/dolby_vision.h>
#endif

/* Local Headers */
#include "osd_canvas.h"
#include "osd_prot.h"
#include "osd_antiflicker.h"
#include "osd_clone.h"
#include "osd_log.h"
#include "osd_reg.h"
#include "osd_io.h"
#include "osd_backup.h"

#include "osd_hw.h"
#include "osd_hw_def.h"
#include "osd_fb.h"
#include "osd_sw_sync.h"

#ifdef CONFIG_AMLOGIC_VSYNC_FIQ_ENABLE
#define FIQ_VSYNC
#endif
#define VOUT_ENCI	1
#define VOUT_ENCP	2
#define VOUT_ENCT	3
#define OSD_TYPE_TOP_FIELD 0
#define OSD_TYPE_BOT_FIELD 1

#define OSD_DISP_DEBUG    1
#define OSD_OLD_HWC         (0x01 << 0)
#define OSD_OTHER_NEW_HWC   (0x01 << 1)
#define OSD_G12A_NEW_HWC    (0x01 << 2)

#define WAIT_AFBC_READY_COUNT 100
#define NEW_PPS_PHASE

#define osd_tprintk(...)

#define FREE_SCALE_MAX_WIDTH    1920
#define WAIT_CNT_MAX            20

struct hw_para_s osd_hw;
static DEFINE_MUTEX(osd_mutex);
static DECLARE_WAIT_QUEUE_HEAD(osd_vsync_wq);
static DECLARE_WAIT_QUEUE_HEAD(osd_vsync2_wq);

static bool vsync_hit[VIU_COUNT];
static bool user_vsync_hit[VIU_COUNT];
static bool osd_update_window_axis;
static int osd_afbc_dec_enable;
static int ext_canvas_id[HW_OSD_COUNT];
static int osd_extra_idx[HW_OSD_COUNT][2];
static bool suspend_flag;
static u32 rdma_dt_cnt;
static void osd_clone_pan(u32 index, u32 yoffset, int debug_flag);
static void osd_set_dummy_data(u32 index, u32 alpha);
static void osd_wait_vsync_hw_viu1(void);

struct hw_osd_reg_s hw_osd_reg_array[HW_OSD_COUNT];

struct hw_osd_reg_s hw_osd_reg_array_g12a[HW_OSD_COUNT] = {
	{
		VIU_OSD1_CTRL_STAT,
		VIU_OSD1_CTRL_STAT2,
		VIU_OSD1_COLOR_ADDR,
		VIU_OSD1_COLOR,
		VIU_OSD1_TCOLOR_AG0,
		VIU_OSD1_TCOLOR_AG1,
		VIU_OSD1_TCOLOR_AG2,
		VIU_OSD1_TCOLOR_AG3,
		VIU_OSD1_BLK0_CFG_W0,
		VIU_OSD1_BLK0_CFG_W1,
		VIU_OSD1_BLK0_CFG_W2,
		VIU_OSD1_BLK0_CFG_W3,
		VIU_OSD1_BLK0_CFG_W4,
		VIU_OSD1_BLK1_CFG_W4,
		VIU_OSD1_BLK2_CFG_W4,
		VIU_OSD1_FIFO_CTRL_STAT,
		VIU_OSD1_TEST_RDDATA,
		VIU_OSD1_PROT_CTRL,
		VIU_OSD1_MALI_UNPACK_CTRL,
		VIU_OSD1_DIMM_CTRL,

		VPP_OSD_SCALE_COEF_IDX,
		VPP_OSD_SCALE_COEF,
		VPP_OSD_VSC_PHASE_STEP,
		VPP_OSD_VSC_INI_PHASE,
		VPP_OSD_VSC_CTRL0,
		VPP_OSD_HSC_PHASE_STEP,
		VPP_OSD_HSC_INI_PHASE,
		VPP_OSD_HSC_CTRL0,
		VPP_OSD_SC_DUMMY_DATA,
		VPP_OSD_SC_CTRL0,
		VPP_OSD_SCI_WH_M1,
		VPP_OSD_SCO_H_START_END,
		VPP_OSD_SCO_V_START_END,
		VPU_MAFBC_HEADER_BUF_ADDR_LOW_S0,
		VPU_MAFBC_HEADER_BUF_ADDR_HIGH_S0,
		VPU_MAFBC_FORMAT_SPECIFIER_S0,
		VPU_MAFBC_BUFFER_WIDTH_S0,
		VPU_MAFBC_BUFFER_HEIGHT_S0,
		VPU_MAFBC_BOUNDING_BOX_X_START_S0,
		VPU_MAFBC_BOUNDING_BOX_X_END_S0,
		VPU_MAFBC_BOUNDING_BOX_Y_START_S0,
		VPU_MAFBC_BOUNDING_BOX_Y_END_S0,
		VPU_MAFBC_OUTPUT_BUF_ADDR_LOW_S0,
		VPU_MAFBC_OUTPUT_BUF_ADDR_HIGH_S0,
		VPU_MAFBC_OUTPUT_BUF_STRIDE_S0,
		VPU_MAFBC_PREFETCH_CFG_S0,


	},
	{
		VIU_OSD2_CTRL_STAT,
		VIU_OSD2_CTRL_STAT2,
		VIU_OSD2_COLOR_ADDR,
		VIU_OSD2_COLOR,
		VIU_OSD2_TCOLOR_AG0,
		VIU_OSD2_TCOLOR_AG1,
		VIU_OSD2_TCOLOR_AG2,
		VIU_OSD2_TCOLOR_AG3,
		VIU_OSD2_BLK0_CFG_W0,
		VIU_OSD2_BLK0_CFG_W1,
		VIU_OSD2_BLK0_CFG_W2,
		VIU_OSD2_BLK0_CFG_W3,
		VIU_OSD2_BLK0_CFG_W4,
		VIU_OSD2_BLK1_CFG_W4,
		VIU_OSD2_BLK2_CFG_W4,
		VIU_OSD2_FIFO_CTRL_STAT,
		VIU_OSD2_TEST_RDDATA,
		VIU_OSD2_PROT_CTRL,
		VIU_OSD2_MALI_UNPACK_CTRL,
		VIU_OSD2_DIMM_CTRL,

		OSD2_SCALE_COEF_IDX,
		OSD2_SCALE_COEF,
		OSD2_VSC_PHASE_STEP,
		OSD2_VSC_INI_PHASE,
		OSD2_VSC_CTRL0,
		OSD2_HSC_PHASE_STEP,
		OSD2_HSC_INI_PHASE,
		OSD2_HSC_CTRL0,
		OSD2_SC_DUMMY_DATA,
		OSD2_SC_CTRL0,
		OSD2_SCI_WH_M1,
		OSD2_SCO_H_START_END,
		OSD2_SCO_V_START_END,
		VPU_MAFBC_HEADER_BUF_ADDR_LOW_S1,
		VPU_MAFBC_HEADER_BUF_ADDR_HIGH_S1,
		VPU_MAFBC_FORMAT_SPECIFIER_S1,
		VPU_MAFBC_BUFFER_WIDTH_S1,
		VPU_MAFBC_BUFFER_HEIGHT_S1,
		VPU_MAFBC_BOUNDING_BOX_X_START_S1,
		VPU_MAFBC_BOUNDING_BOX_X_END_S1,
		VPU_MAFBC_BOUNDING_BOX_Y_START_S1,
		VPU_MAFBC_BOUNDING_BOX_Y_END_S1,
		VPU_MAFBC_OUTPUT_BUF_ADDR_LOW_S1,
		VPU_MAFBC_OUTPUT_BUF_ADDR_HIGH_S1,
		VPU_MAFBC_OUTPUT_BUF_STRIDE_S1,
		VPU_MAFBC_PREFETCH_CFG_S1,

	},
	{
		VIU_OSD3_CTRL_STAT,
		VIU_OSD3_CTRL_STAT2,
		VIU_OSD3_COLOR_ADDR,
		VIU_OSD3_COLOR,
		VIU_OSD3_TCOLOR_AG0,
		VIU_OSD3_TCOLOR_AG1,
		VIU_OSD3_TCOLOR_AG2,
		VIU_OSD3_TCOLOR_AG3,
		VIU_OSD3_BLK0_CFG_W0,
		VIU_OSD3_BLK0_CFG_W1,
		VIU_OSD3_BLK0_CFG_W2,
		VIU_OSD3_BLK0_CFG_W3,
		VIU_OSD3_BLK0_CFG_W4,
		VIU_OSD3_BLK1_CFG_W4,
		VIU_OSD3_BLK2_CFG_W4,
		VIU_OSD3_FIFO_CTRL_STAT,
		VIU_OSD3_TEST_RDDATA,
		VIU_OSD3_PROT_CTRL,
		VIU_OSD3_MALI_UNPACK_CTRL,
		VIU_OSD3_DIMM_CTRL,

		OSD34_SCALE_COEF_IDX,
		OSD34_SCALE_COEF,
		OSD34_VSC_PHASE_STEP,
		OSD34_VSC_INI_PHASE,
		OSD34_VSC_CTRL0,
		OSD34_HSC_PHASE_STEP,
		OSD34_HSC_INI_PHASE,
		OSD34_HSC_CTRL0,
		OSD34_SC_DUMMY_DATA,
		OSD34_SC_CTRL0,
		OSD34_SCI_WH_M1,
		OSD34_SCO_H_START_END,
		OSD34_SCO_V_START_END,
		VPU_MAFBC_HEADER_BUF_ADDR_LOW_S2,
		VPU_MAFBC_HEADER_BUF_ADDR_HIGH_S2,
		VPU_MAFBC_FORMAT_SPECIFIER_S2,
		VPU_MAFBC_BUFFER_WIDTH_S2,
		VPU_MAFBC_BUFFER_HEIGHT_S2,
		VPU_MAFBC_BOUNDING_BOX_X_START_S2,
		VPU_MAFBC_BOUNDING_BOX_X_END_S2,
		VPU_MAFBC_BOUNDING_BOX_Y_START_S2,
		VPU_MAFBC_BOUNDING_BOX_Y_END_S2,
		VPU_MAFBC_OUTPUT_BUF_ADDR_LOW_S2,
		VPU_MAFBC_OUTPUT_BUF_ADDR_HIGH_S2,
		VPU_MAFBC_OUTPUT_BUF_STRIDE_S2,
		VPU_MAFBC_PREFETCH_CFG_S2,
	},
	{
		VIU2_OSD1_CTRL_STAT,
		VIU2_OSD1_CTRL_STAT2,
		VIU2_OSD1_COLOR_ADDR,
		VIU2_OSD1_COLOR,
		VIU2_OSD1_TCOLOR_AG0,
		VIU2_OSD1_TCOLOR_AG1,
		VIU2_OSD1_TCOLOR_AG2,
		VIU2_OSD1_TCOLOR_AG3,
		VIU2_OSD1_BLK0_CFG_W0,
		VIU2_OSD1_BLK0_CFG_W1,
		VIU2_OSD1_BLK0_CFG_W2,
		VIU2_OSD1_BLK0_CFG_W3,
		VIU2_OSD1_BLK0_CFG_W4,
		VIU2_OSD1_BLK1_CFG_W4,
		VIU2_OSD1_BLK2_CFG_W4,
		VIU2_OSD1_FIFO_CTRL_STAT,
		VIU2_OSD1_TEST_RDDATA,
		VIU2_OSD1_PROT_CTRL,
		VIU2_OSD1_MALI_UNPACK_CTRL,
		VIU2_OSD1_DIMM_CTRL,

		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		}
};

struct hw_osd_reg_s hw_osd_reg_array_tl1[HW_OSD_COUNT] = {
	{
		VIU_OSD1_CTRL_STAT,
		VIU_OSD1_CTRL_STAT2,
		VIU_OSD1_COLOR_ADDR,
		VIU_OSD1_COLOR,
		VIU_OSD1_TCOLOR_AG0,
		VIU_OSD1_TCOLOR_AG1,
		VIU_OSD1_TCOLOR_AG2,
		VIU_OSD1_TCOLOR_AG3,
		VIU_OSD1_BLK0_CFG_W0,
		VIU_OSD1_BLK0_CFG_W1,
		VIU_OSD1_BLK0_CFG_W2,
		VIU_OSD1_BLK0_CFG_W3,
		VIU_OSD1_BLK0_CFG_W4,
		VIU_OSD1_BLK1_CFG_W4,
		VIU_OSD1_BLK2_CFG_W4,
		VIU_OSD1_FIFO_CTRL_STAT,
		VIU_OSD1_TEST_RDDATA,
		VIU_OSD1_PROT_CTRL,
		VIU_OSD1_MALI_UNPACK_CTRL,
		VIU_OSD1_DIMM_CTRL,

		VPP_OSD_SCALE_COEF_IDX,
		VPP_OSD_SCALE_COEF,
		VPP_OSD_VSC_PHASE_STEP,
		VPP_OSD_VSC_INI_PHASE,
		VPP_OSD_VSC_CTRL0,
		VPP_OSD_HSC_PHASE_STEP,
		VPP_OSD_HSC_INI_PHASE,
		VPP_OSD_HSC_CTRL0,
		VPP_OSD_SC_DUMMY_DATA,
		VPP_OSD_SC_CTRL0,
		VPP_OSD_SCI_WH_M1,
		VPP_OSD_SCO_H_START_END,
		VPP_OSD_SCO_V_START_END,
		VPU_MAFBC_HEADER_BUF_ADDR_LOW_S0,
		VPU_MAFBC_HEADER_BUF_ADDR_HIGH_S0,
		VPU_MAFBC_FORMAT_SPECIFIER_S0,
		VPU_MAFBC_BUFFER_WIDTH_S0,
		VPU_MAFBC_BUFFER_HEIGHT_S0,
		VPU_MAFBC_BOUNDING_BOX_X_START_S0,
		VPU_MAFBC_BOUNDING_BOX_X_END_S0,
		VPU_MAFBC_BOUNDING_BOX_Y_START_S0,
		VPU_MAFBC_BOUNDING_BOX_Y_END_S0,
		VPU_MAFBC_OUTPUT_BUF_ADDR_LOW_S0,
		VPU_MAFBC_OUTPUT_BUF_ADDR_HIGH_S0,
		VPU_MAFBC_OUTPUT_BUF_STRIDE_S0,
		VPU_MAFBC_PREFETCH_CFG_S0,


	},
	{
		VIU_OSD2_CTRL_STAT,
		VIU_OSD2_CTRL_STAT2,
		VIU_OSD2_COLOR_ADDR,
		VIU_OSD2_COLOR,
		VIU_OSD2_TCOLOR_AG0,
		VIU_OSD2_TCOLOR_AG1,
		VIU_OSD2_TCOLOR_AG2,
		VIU_OSD2_TCOLOR_AG3,
		VIU_OSD2_BLK0_CFG_W0,
		VIU_OSD2_BLK0_CFG_W1,
		VIU_OSD2_BLK0_CFG_W2,
		VIU_OSD2_BLK0_CFG_W3,
		VIU_OSD2_BLK0_CFG_W4,
		VIU_OSD2_BLK1_CFG_W4,
		VIU_OSD2_BLK2_CFG_W4,
		VIU_OSD2_FIFO_CTRL_STAT,
		VIU_OSD2_TEST_RDDATA,
		VIU_OSD2_PROT_CTRL,
		VIU_OSD2_MALI_UNPACK_CTRL,
		VIU_OSD2_DIMM_CTRL,

		OSD2_SCALE_COEF_IDX,
		OSD2_SCALE_COEF,
		OSD2_VSC_PHASE_STEP,
		OSD2_VSC_INI_PHASE,
		OSD2_VSC_CTRL0,
		OSD2_HSC_PHASE_STEP,
		OSD2_HSC_INI_PHASE,
		OSD2_HSC_CTRL0,
		OSD2_SC_DUMMY_DATA,
		OSD2_SC_CTRL0,
		OSD2_SCI_WH_M1,
		OSD2_SCO_H_START_END,
		OSD2_SCO_V_START_END,
		VPU_MAFBC_HEADER_BUF_ADDR_LOW_S1,
		VPU_MAFBC_HEADER_BUF_ADDR_HIGH_S1,
		VPU_MAFBC_FORMAT_SPECIFIER_S1,
		VPU_MAFBC_BUFFER_WIDTH_S1,
		VPU_MAFBC_BUFFER_HEIGHT_S1,
		VPU_MAFBC_BOUNDING_BOX_X_START_S1,
		VPU_MAFBC_BOUNDING_BOX_X_END_S1,
		VPU_MAFBC_BOUNDING_BOX_Y_START_S1,
		VPU_MAFBC_BOUNDING_BOX_Y_END_S1,
		VPU_MAFBC_OUTPUT_BUF_ADDR_LOW_S1,
		VPU_MAFBC_OUTPUT_BUF_ADDR_HIGH_S1,
		VPU_MAFBC_OUTPUT_BUF_STRIDE_S1,
		VPU_MAFBC_PREFETCH_CFG_S1,

	},
	{
		VIU2_OSD1_CTRL_STAT,
		VIU2_OSD1_CTRL_STAT2,
		VIU2_OSD1_COLOR_ADDR,
		VIU2_OSD1_COLOR,
		VIU2_OSD1_TCOLOR_AG0,
		VIU2_OSD1_TCOLOR_AG1,
		VIU2_OSD1_TCOLOR_AG2,
		VIU2_OSD1_TCOLOR_AG3,
		VIU2_OSD1_BLK0_CFG_W0,
		VIU2_OSD1_BLK0_CFG_W1,
		VIU2_OSD1_BLK0_CFG_W2,
		VIU2_OSD1_BLK0_CFG_W3,
		VIU2_OSD1_BLK0_CFG_W4,
		VIU2_OSD1_BLK1_CFG_W4,
		VIU2_OSD1_BLK2_CFG_W4,
		VIU2_OSD1_FIFO_CTRL_STAT,
		VIU2_OSD1_TEST_RDDATA,
		VIU2_OSD1_PROT_CTRL,
		VIU2_OSD1_MALI_UNPACK_CTRL,
		VIU2_OSD1_DIMM_CTRL,

		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		VIU2_OSD1_UNSUPPORT,
		}
};

#define VIU2_OSD_REG_NUM    13
static u32 viu2_osd_table[VIU2_OSD_REG_NUM];
static struct viu2_osd_reg_item viu2_osd_reg_table[VIU2_OSD_REG_NUM] = {
		{VIU2_OSD1_CTRL_STAT, 0, 0xc01ff9f7},
		{VIU2_OSD1_CTRL_STAT2, 0x0, 0x00007fff},
		{VIU2_OSD1_BLK0_CFG_W0, 0x0, 0x70ffff7f},
		{VIU2_OSD1_BLK0_CFG_W1, 0x0, 0x1fff1fff},
		{VIU2_OSD1_BLK0_CFG_W2, 0x0, 0x1fff1fff},
		{VIU2_OSD1_BLK0_CFG_W3, 0x0, 0x0fff0fff},
		{VIU2_OSD1_BLK0_CFG_W4, 0x0, 0x0fff0fff},
		{VIU2_OSD1_BLK1_CFG_W4, 0x0, 0xffffffff},
		{VIU2_OSD1_BLK2_CFG_W4, 0x0, 0xffffffff},
		{VIU2_OSD1_FIFO_CTRL_STAT, 0x0, 0xffc7ffff},
		{VIU2_OSD1_PROT_CTRL, 0x0, 0xffff0000},
		{VIU2_OSD1_MALI_UNPACK_CTRL, 0x0, 0x9f01ffff},
		{VIU2_OSD1_DIMM_CTRL, 0x0, 0x7fffffff},
};

static int osd_setting_blending_scope(u32 index);
static int vpp_blend_setting_default(u32 index);

#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
/* sync fence relative varible. */
static int timeline_created[VIU_COUNT];
static void *osd_timeline[VIU_COUNT];
static u32 cur_streamline_val[VIU_COUNT];
/* thread control part */
struct kthread_worker buffer_toggle_worker[VIU_COUNT];
struct task_struct *buffer_toggle_thread[VIU_COUNT];
struct kthread_work buffer_toggle_work[VIU_COUNT];
struct list_head post_fence_list[VIU_COUNT];
struct mutex post_fence_list_lock[VIU_COUNT]; /*post fence mutex*/
struct osd_layers_fence_map_s map_layers[VIU_COUNT];
struct file *displayed_bufs[HW_OSD_COUNT];
static void osd_pan_display_single_fence(
	struct osd_fence_map_s *fence_map);
static void osd_pan_display_layers_fence(
	struct osd_layers_fence_map_s *fence_map);
static void osd_pan_display_single_fence_viu2(
	struct osd_fence_map_s *fence_map);
static void osd_pan_display_layers_fence_viu2(
	struct osd_layers_fence_map_s *fence_map);


static void *osd_timeline_create(u32 output_index)
{
	char tlname[32] = {};

	sprintf(tlname, "osd_timeline_%d", output_index);
	if (!osd_timeline[output_index]) {
		if (osd_hw.hwc_enable[output_index])
			/* present fence */
			cur_streamline_val[output_index] = 0;
		else
			cur_streamline_val[output_index] = 1;
		osd_timeline[output_index] = aml_sync_create_timeline(tlname);
		osd_tprintk("osd timeline create\n");
	}
	return osd_timeline[output_index];
}

static int osd_timeline_create_fence(u32 output_index)
{
	int out_fence_fd = -1;
	u32 pt_val = 0;

	pt_val = cur_streamline_val[output_index] + 1;
	out_fence_fd = aml_sync_create_fence
		(osd_timeline[output_index], pt_val);
	osd_tprintk("osd created out pt:%d, fence_fd:%d\n",
			pt_val, out_fence_fd);

	if (out_fence_fd >= 0)
		cur_streamline_val[output_index]++;
	else
		pr_info("create fence returned %d", out_fence_fd);
	return out_fence_fd;
}

static void  osd_timeline_increase(u32 output_index)
{
	aml_sync_inc_timeline(osd_timeline[output_index], 1);
	osd_tprintk("osd out timeline %d inc\n", output_index);

}

static struct fence *osd_get_fenceobj(int fencefd)
{
	struct fence *syncobj = NULL;

	if (fencefd < 0)
		return NULL;

	syncobj = aml_sync_get_fence(fencefd);
	osd_tprintk("osd get in fence%p, fd:%d\n", syncobj, fencefd);

	return syncobj;
}

static int osd_wait_fenceobj(struct fence *fence, long timeout)
{
	osd_tprintk("osd wait in fence%p\n", fence);
	return aml_sync_wait_fence(fence, timeout);
}

static void osd_put_fenceobj(struct fence *fence)
{
	osd_tprintk("osd put fence\n");
	aml_sync_put_fence(fence);
}

#endif

static int pxp_mode;
s64 timestamp[VIU_COUNT];

static unsigned int osd_h_filter_mode = 1;
#define BYTE_32_ALIGNED(x)	(((x) + 31) & ~31)
#define BYTE_16_ALIGNED(x)	(((x) + 15) & ~15)
#define BYTE_8_ALIGNED(x)	(((x) + 7) & ~7)
module_param(osd_h_filter_mode, uint, 0664);
MODULE_PARM_DESC(osd_h_filter_mode, "osd_h_filter_mode");

static unsigned int osd_v_filter_mode = 1;
module_param(osd_v_filter_mode, uint, 0664);
MODULE_PARM_DESC(osd_v_filter_mode, "osd_v_filter_mode");

static unsigned int osd_auto_adjust_filter = 1;
module_param(osd_auto_adjust_filter, uint, 0664);
MODULE_PARM_DESC(osd_auto_adjust_filter, "osd_auto_adjust_filter");

static int osd_logo_index = 1;
module_param(osd_logo_index, int, 0664);
MODULE_PARM_DESC(osd_logo_index, "osd_logo_index");

module_param(osd_afbc_dec_enable, int, 0664);
MODULE_PARM_DESC(osd_afbc_dec_enable, "osd_afbc_dec_enable");

static u32 osd_vpp_misc;
static u32 osd_vpp_misc_mask = OSD_RELATIVE_BITS;
module_param(osd_vpp_misc, uint, 0444);
MODULE_PARM_DESC(osd_vpp_misc, "osd_vpp_misc");

static unsigned int rdarb_reqen_slv = 0xff7f;
module_param(rdarb_reqen_slv, uint, 0664);
MODULE_PARM_DESC(rdarb_reqen_slv, "rdarb_reqen_slv");

static unsigned int supsend_delay;
module_param(supsend_delay, uint, 0664);
MODULE_PARM_DESC(supsend_delay, "supsend_delay");

int enable_vd_zorder = 1;
MODULE_PARM_DESC(enable_vd_zorder, "\n enable_vd_zorder\n");
module_param(enable_vd_zorder, uint, 0664);

static int vsync_enter_line_max;
static int vsync_exit_line_max;
static int line_threshold = 90;
MODULE_PARM_DESC(vsync_enter_line_max, "\n vsync_enter_line_max\n");
module_param(vsync_enter_line_max, uint, 0664);
MODULE_PARM_DESC(vsync_exit_line_max, "\n vsync_exit_line_max\n");
module_param(vsync_exit_line_max, uint, 0664);
MODULE_PARM_DESC(line_threshold, "\n line_threshold\n");
module_param(line_threshold, uint, 0664);

static unsigned int osd_filter_coefs_bicubic_sharp[] = {
	0x01fa008c, 0x01fa0100, 0xff7f0200, 0xfe7f0300,
	0xfd7e0500, 0xfc7e0600, 0xfb7d0800, 0xfb7c0900,
	0xfa7b0b00, 0xfa7a0dff, 0xf9790fff, 0xf97711ff,
	0xf87613ff, 0xf87416fe, 0xf87218fe, 0xf8701afe,
	0xf76f1dfd, 0xf76d1ffd, 0xf76b21fd, 0xf76824fd,
	0xf76627fc, 0xf76429fc, 0xf7612cfc, 0xf75f2ffb,
	0xf75d31fb, 0xf75a34fb, 0xf75837fa, 0xf7553afa,
	0xf8523cfa, 0xf8503ff9, 0xf84d42f9, 0xf84a45f9,
	0xf84848f8
};

static unsigned int osd_filter_coefs_bicubic[] = { /* bicubic	coef0 */
	0x00800000, 0x007f0100, 0xff7f0200, 0xfe7f0300, 0xfd7e0500, 0xfc7e0600,
	0xfb7d0800, 0xfb7c0900, 0xfa7b0b00, 0xfa7a0dff, 0xf9790fff, 0xf97711ff,
	0xf87613ff, 0xf87416fe, 0xf87218fe, 0xf8701afe, 0xf76f1dfd, 0xf76d1ffd,
	0xf76b21fd, 0xf76824fd, 0xf76627fc, 0xf76429fc, 0xf7612cfc, 0xf75f2ffb,
	0xf75d31fb, 0xf75a34fb, 0xf75837fa, 0xf7553afa, 0xf8523cfa, 0xf8503ff9,
	0xf84d42f9, 0xf84a45f9, 0xf84848f8
};

static unsigned int osd_filter_coefs_bilinear[] = { /* 2 point bilinear	coef1 */
	0x00800000, 0x007e0200, 0x007c0400, 0x007a0600, 0x00780800, 0x00760a00,
	0x00740c00, 0x00720e00, 0x00701000, 0x006e1200, 0x006c1400, 0x006a1600,
	0x00681800, 0x00661a00, 0x00641c00, 0x00621e00, 0x00602000, 0x005e2200,
	0x005c2400, 0x005a2600, 0x00582800, 0x00562a00, 0x00542c00, 0x00522e00,
	0x00503000, 0x004e3200, 0x004c3400, 0x004a3600, 0x00483800, 0x00463a00,
	0x00443c00, 0x00423e00, 0x00404000
};

static unsigned int osd_filter_coefs_2point_binilear[] = {
	/* 2 point bilinear, bank_length == 2	coef2 */
	0x80000000, 0x7e020000, 0x7c040000, 0x7a060000, 0x78080000, 0x760a0000,
	0x740c0000, 0x720e0000, 0x70100000, 0x6e120000, 0x6c140000, 0x6a160000,
	0x68180000, 0x661a0000, 0x641c0000, 0x621e0000, 0x60200000, 0x5e220000,
	0x5c240000, 0x5a260000, 0x58280000, 0x562a0000, 0x542c0000, 0x522e0000,
	0x50300000, 0x4e320000, 0x4c340000, 0x4a360000, 0x48380000, 0x463a0000,
	0x443c0000, 0x423e0000, 0x40400000
};

/* filt_triangle, point_num =3, filt_len =2.6, group_num = 64 */
static unsigned int osd_filter_coefs_3point_triangle_sharp[] = {
	0x40400000, 0x3e420000, 0x3d430000, 0x3b450000,
	0x3a460000, 0x38480000, 0x37490000, 0x354b0000,
	0x344c0000, 0x324e0000, 0x314f0000, 0x2f510000,
	0x2e520000, 0x2c540000, 0x2b550000, 0x29570000,
	0x28580000, 0x265a0000, 0x245c0000, 0x235d0000,
	0x215f0000, 0x20600000, 0x1e620000, 0x1d620100,
	0x1b620300, 0x19630400, 0x17630600, 0x15640700,
	0x14640800, 0x12640a00, 0x11640b00, 0x0f650c00,
	0x0d660d00
};

static unsigned int osd_filter_coefs_3point_triangle[] = {
	0x40400000, 0x3f400100, 0x3d410200, 0x3c410300,
	0x3a420400, 0x39420500, 0x37430600, 0x36430700,
	0x35430800, 0x33450800, 0x32450900, 0x31450a00,
	0x30450b00, 0x2e460c00, 0x2d460d00, 0x2c470d00,
	0x2b470e00, 0x29480f00, 0x28481000, 0x27481100,
	0x26491100, 0x25491200, 0x24491300, 0x234a1300,
	0x224a1400, 0x214a1500, 0x204a1600, 0x1f4b1600,
	0x1e4b1700, 0x1d4b1800, 0x1c4c1800, 0x1b4c1900,
	0x1a4c1a00
};

static unsigned int osd_filter_coefs_4point_triangle[] = {
	0x20402000, 0x20402000, 0x1f3f2101, 0x1f3f2101,
	0x1e3e2202, 0x1e3e2202, 0x1d3d2303, 0x1d3d2303,
	0x1c3c2404, 0x1c3c2404, 0x1b3b2505, 0x1b3b2505,
	0x1a3a2606, 0x1a3a2606, 0x19392707, 0x19392707,
	0x18382808, 0x18382808, 0x17372909, 0x17372909,
	0x16362a0a, 0x16362a0a, 0x15352b0b, 0x15352b0b,
	0x14342c0c, 0x14342c0c, 0x13332d0d, 0x13332d0d,
	0x12322e0e, 0x12322e0e, 0x11312f0f, 0x11312f0f,
	0x10303010
};

/* 4th order (cubic) b-spline */
/* filt_cubic point_num =4, filt_len =4, group_num = 64 */
static unsigned int vpp_filter_coefs_4point_bspline[] = {
	0x15561500, 0x14561600, 0x13561700, 0x12561800,
	0x11551a00, 0x11541b00, 0x10541c00, 0x0f541d00,
	0x0f531e00, 0x0e531f00, 0x0d522100, 0x0c522200,
	0x0b522300, 0x0b512400, 0x0a502600, 0x0a4f2700,
	0x094e2900, 0x084e2a00, 0x084d2b00, 0x074c2c01,
	0x074b2d01, 0x064a2f01, 0x06493001, 0x05483201,
	0x05473301, 0x05463401, 0x04453601, 0x04433702,
	0x04423802, 0x03413a02, 0x03403b02, 0x033f3c02,
	0x033d3d03
};

/* filt_quadratic, point_num =3, filt_len =3, group_num = 64 */
static unsigned int osd_filter_coefs_3point_bspline[] = {
	0x40400000, 0x3e420000, 0x3c440000, 0x3a460000,
	0x38480000, 0x364a0000, 0x344b0100, 0x334c0100,
	0x314e0100, 0x304f0100, 0x2e500200, 0x2c520200,
	0x2a540200, 0x29540300, 0x27560300, 0x26570300,
	0x24580400, 0x23590400, 0x215a0500, 0x205b0500,
	0x1e5c0600, 0x1d5c0700, 0x1c5d0700, 0x1a5e0800,
	0x195e0900, 0x185e0a00, 0x175f0a00, 0x15600b00,
	0x14600c00, 0x13600d00, 0x12600e00, 0x11600f00,
	0x10601000
};

static unsigned int osd_filter_coefs_repeat[] = { /* repeat coef0 */
	0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000,
	0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000,
	0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000,
	0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000,
	0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000,
	0x00800000, 0x00800000, 0x00800000
};

static unsigned int *filter_table[] = {
	osd_filter_coefs_bicubic_sharp,
	osd_filter_coefs_bicubic,
	osd_filter_coefs_bilinear,
	osd_filter_coefs_2point_binilear,
	osd_filter_coefs_3point_triangle_sharp,
	osd_filter_coefs_3point_triangle,
	osd_filter_coefs_4point_triangle,
	vpp_filter_coefs_4point_bspline,
	osd_filter_coefs_3point_bspline,
	osd_filter_coefs_repeat
};

#ifdef NEW_PPS_PHASE
#define OSD_ZOOM_BITS 20
#define OSD_PHASE_BITS 16

enum osd_f2v_vphase_type_e {
	OSD_F2V_IT2IT = 0,
	OSD_F2V_IB2IB,
	OSD_F2V_IT2IB,
	OSD_F2V_IB2IT,
	OSD_F2V_P2IT,
	OSD_F2V_P2IB,
	OSD_F2V_IT2P,
	OSD_F2V_IB2P,
	OSD_F2V_P2P,
	OSD_F2V_TYPE_MAX
};

struct osd_f2v_vphase_s {
	u8 rcv_num;
	u8 rpt_num;
	u16 phase;
};

#define COEFF_NORM(a) ((int)((((a) * 2048.0) + 1) / 2))
#define MATRIX_5x3_COEF_SIZE 24

static int RGB709_to_YUV709l_coeff[MATRIX_5x3_COEF_SIZE] = {
	0, 0, 0, /* pre offset */
	COEFF_NORM(0.181873),	COEFF_NORM(0.611831),	COEFF_NORM(0.061765),
	COEFF_NORM(-0.100251),	COEFF_NORM(-0.337249),	COEFF_NORM(0.437500),
	COEFF_NORM(0.437500),	COEFF_NORM(-0.397384),	COEFF_NORM(-0.040116),
	0, 0, 0, /* 10'/11'/12' */
	0, 0, 0, /* 20'/21'/22' */
	64, 512, 512, /* offset */
	0, 0, 0 /* mode, right_shift, clip_en */
};

static int is_yuv_format(u32 format)
{
	int b_yuv = 0;

	switch (format) {
	case COLOR_FMT_YUV422:
	case COLOR_FMT_YUV444:
	case COLOR_FMT_YUYV422:
	case COLOR_FMT_YVYU422:
	case COLOR_FMT_UYVY422:
	case COLOR_FMT_VYUY422:
	case COLOR_FMT_NV12:
	case COLOR_FMT_NV21:
		b_yuv = 1;
		break;
	default:
		break;
	}
	return b_yuv;
}

/*for G12A, set osd3 matrix(10bit) RGB2YUV*/
static void set_viu2_rgb2yuv(bool on)
{
	/* RGB -> 709 limit */
	int *m = RGB709_to_YUV709l_coeff;

	/* VPP WRAP OSD3 matrix */
	osd_reg_write(VIU2_OSD1_MATRIX_PRE_OFFSET0_1,
		((m[0] & 0xfff) << 16) | (m[1] & 0xfff));
	osd_reg_write(VIU2_OSD1_MATRIX_PRE_OFFSET2,
		m[2] & 0xfff);
	osd_reg_write(VIU2_OSD1_MATRIX_COEF00_01,
		((m[3] & 0x1fff) << 16) | (m[4] & 0x1fff));
	osd_reg_write(VIU2_OSD1_MATRIX_COEF02_10,
		((m[5]  & 0x1fff) << 16) | (m[6] & 0x1fff));
	osd_reg_write(VIU2_OSD1_MATRIX_COEF11_12,
		((m[7] & 0x1fff) << 16) | (m[8] & 0x1fff));
	osd_reg_write(VIU2_OSD1_MATRIX_COEF20_21,
		((m[9] & 0x1fff) << 16) | (m[10] & 0x1fff));
	osd_reg_write(VIU2_OSD1_MATRIX_COEF22,
		m[11] & 0x1fff);

	osd_reg_write(VIU2_OSD1_MATRIX_OFFSET0_1,
		((m[18] & 0xfff) << 16) | (m[19] & 0xfff));
	osd_reg_write(VIU2_OSD1_MATRIX_OFFSET2,
		m[20] & 0xfff);

	osd_reg_set_bits(VIU2_OSD1_MATRIX_EN_CTRL, on, 0, 1);
}

static void f2v_get_vertical_phase(
	u32 zoom_ratio, enum osd_f2v_vphase_type_e type,
	u8 bank_length, struct osd_f2v_vphase_s *vphase)
{
	u8 f2v_420_in_pos_luma[OSD_F2V_TYPE_MAX] = {
		0, 2, 0, 2, 0, 0, 0, 2, 0};
	u8 f2v_420_out_pos[OSD_F2V_TYPE_MAX] = {
		0, 2, 2, 0, 0, 2, 0, 0, 0};
	s32 offset_in, offset_out;

	/* luma */
	offset_in = f2v_420_in_pos_luma[type]
		<< OSD_PHASE_BITS;
	offset_out = (f2v_420_out_pos[type] * zoom_ratio)
		>> (OSD_ZOOM_BITS - OSD_PHASE_BITS);

	vphase->rcv_num = bank_length;
	if (bank_length == 4 || bank_length == 3)
		vphase->rpt_num = 1;
	else
		vphase->rpt_num = 0;

	if (offset_in > offset_out) {
		vphase->rpt_num = vphase->rpt_num + 1;
		vphase->phase =
			((4 << OSD_PHASE_BITS) + offset_out - offset_in)
			>> 2;
	} else {
		while ((offset_in + (4 << OSD_PHASE_BITS))
			<= offset_out) {
			if (vphase->rpt_num == 1)
				vphase->rpt_num = 0;
			else
				vphase->rcv_num++;
			offset_in += 4 << OSD_PHASE_BITS;
		}
		vphase->phase = (offset_out - offset_in) >> 2;
	}
}
#endif

#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_VECM
static bool osd_hdr_on;
#endif

static int get_active_begin_line(u32 viu_type)
{
	int active_line_begin = 0;
	u32 viu = VIU1;

	if (viu_type == VIU1)
		viu = osd_reg_read(VPU_VIU_VENC_MUX_CTRL) & 0x3;
	else if (viu_type == VIU2)
		viu = (osd_reg_read(VPU_VIU_VENC_MUX_CTRL) >> 2) & 0x3;
	switch (viu) {
	case 0:
		active_line_begin =
			osd_reg_read(ENCL_VIDEO_VAVON_BLINE);
		break;
	case 1:
		active_line_begin =
			osd_reg_read(ENCI_VFIFO2VD_LINE_TOP_START);
		break;
	case 2:
		active_line_begin =
			osd_reg_read(ENCP_VIDEO_VAVON_BLINE);
		break;
	case 3:
		active_line_begin =
			osd_reg_read(ENCT_VIDEO_VAVON_BLINE);
		break;
	}

	return active_line_begin;
}

static int get_encp_line(u32 viu_type)
{
	int enc_line = 0;
	unsigned int reg = 0;
	u32 viu = VIU1;

	if (viu_type == VIU1)
		viu = osd_reg_read(VPU_VIU_VENC_MUX_CTRL) & 0x3;
	else if (viu_type == VIU2)
		viu = (osd_reg_read(VPU_VIU_VENC_MUX_CTRL) >> 2) & 0x3;
	switch (viu) {
	case 0:
		reg = osd_reg_read(ENCL_INFO_READ);
		break;
	case 1:
		reg = osd_reg_read(ENCI_INFO_READ);
		break;
	case 2:
		reg = osd_reg_read(ENCP_INFO_READ);
		break;
	case 3:
		reg = osd_reg_read(ENCT_INFO_READ);
		break;
	}
	enc_line = (reg >> 16) & 0x1fff;

	return enc_line;
}

static int get_enter_encp_line(u32 viu_type)
{
	int enc_line = 0;

	enc_line = get_encp_line(viu_type);
	if (enc_line > vsync_enter_line_max)
		vsync_enter_line_max = enc_line;
	return enc_line;
}

static int get_exit_encp_line(u32 viu_type)
{
	int enc_line = 0;

	enc_line = get_encp_line(viu_type);
	if (enc_line > vsync_exit_line_max)
		vsync_exit_line_max = enc_line;
	return enc_line;
}

static void osd_vpu_power_on(void)
{
#ifdef CONFIG_AMLOGIC_VPU
	switch_vpu_mem_pd_vmod(VPU_VIU_OSD1, VPU_MEM_POWER_ON);
	switch_vpu_mem_pd_vmod(VPU_VIU_OSD2, VPU_MEM_POWER_ON);
	switch_vpu_mem_pd_vmod(VPU_VIU_OSD_SCALE, VPU_MEM_POWER_ON);
	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {

		switch_vpu_mem_pd_vmod(
			VPU_VD2_OSD2_SCALE,
			VPU_MEM_POWER_ON);
		switch_vpu_mem_pd_vmod(VPU_VIU_OSD3,
			VPU_MEM_POWER_ON);
		switch_vpu_mem_pd_vmod(VPU_OSD_BLD34,
			VPU_MEM_POWER_ON);
	}
	if (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) {
		switch_vpu_mem_pd_vmod(
			VPU_AFBC_DEC,
			VPU_MEM_POWER_ON);
	} else if (osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) {
		switch_vpu_mem_pd_vmod(
			VPU_MAIL_AFBCD,
			VPU_MEM_POWER_ON);
	}
#endif
}

static void osd_vpu_power_on_viu2(void)
{
#ifdef CONFIG_AMLOGIC_VPU
	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		u32 val;

		switch_vpu_mem_pd_vmod(VPU_VIU2_OSD1,
			VPU_MEM_POWER_ON);
		switch_vpu_mem_pd_vmod(VPU_VIU2_OFIFO,
			VPU_MEM_POWER_ON);
		switch_vpu_mem_pd_vmod(VPU_VIU2_OSD_ROT,
			VPU_MEM_POWER_ON);
		val = osd_reg_read(VPU_CLK_GATE);
		val =  val | 0x30000;
		osd_reg_write(VPU_CLK_GATE, val);
	}
#endif
}

u32 get_output_device_id(u32 index)
{
	u32 output_index = VIU1;

	if (osd_hw.osd_meson_dev.has_viu2) {
		if (index == osd_hw.osd_meson_dev.viu2_index)
			output_index = VIU2;
		else
			output_index = VIU1;
	}
	return output_index;
}

static int get_osd_hwc_type(u32 index)
{
	int ret = 0;
	u32 output_index;

	output_index = get_output_device_id(index);
	/* new hwcomposer enable */
	if (osd_hw.hwc_enable[output_index]) {
		if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE)
			ret = OSD_G12A_NEW_HWC;
		else
			ret = OSD_OTHER_NEW_HWC;
	} else
		ret = OSD_OLD_HWC;
	return ret;
}

#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
static inline  int  find_buf_num(u32 yres, u32 yoffset)
{
	int n = yres;
	int i;

	for (i = 0; i < OSD_MAX_BUF_NUM; i++) {
		/* find current addr position. */
		if (yoffset  < (n))
			break;
		n += yres;
	}
	return i;
}

static void osd_toggle_buffer_single(struct kthread_work *work)
{
	struct osd_fence_map_s *data, *next;
	struct list_head saved_list;

	mutex_lock(&post_fence_list_lock[VIU1]);
	saved_list = post_fence_list[VIU1];
	list_replace_init(&post_fence_list[VIU1], &saved_list);
	mutex_unlock(&post_fence_list_lock[VIU1]);
	list_for_each_entry_safe(data, next, &saved_list, list) {
		osd_pan_display_single_fence(data);
		list_del(&data->list);
		kfree(data);
	}
}

static void osd_toggle_buffer_layers(struct kthread_work *work)
{
	struct osd_layers_fence_map_s *data, *next;
	struct list_head saved_list;

	mutex_lock(&post_fence_list_lock[VIU1]);
	saved_list = post_fence_list[VIU1];
	list_replace_init(&post_fence_list[VIU1], &saved_list);
	mutex_unlock(&post_fence_list_lock[VIU1]);
	list_for_each_entry_safe(data, next, &saved_list, list) {
		osd_pan_display_layers_fence(data);
		list_del(&data->list);
		kfree(data);
	}
}


static void osd_toggle_buffer(struct kthread_work *work)
{
	u32 hwc_enable;

	hwc_enable = osd_hw.hwc_enable[VIU1];
	if (osd_hw.osd_fence[VIU1][hwc_enable].toggle_buffer_handler)
		osd_hw.osd_fence[VIU1][hwc_enable]
			.toggle_buffer_handler(work);
}

static void osd_toggle_buffer_single_viu2(struct kthread_work *work)
{
	struct osd_fence_map_s *data, *next;
	struct list_head saved_list;

	mutex_lock(&post_fence_list_lock[VIU2]);
	saved_list = post_fence_list[VIU2];
	list_replace_init(&post_fence_list[VIU2], &saved_list);
	mutex_unlock(&post_fence_list_lock[VIU2]);
	list_for_each_entry_safe(data, next, &saved_list, list) {
		osd_pan_display_single_fence_viu2(data);
		list_del(&data->list);
		kfree(data);
	}
}

static void osd_toggle_buffer_layers_viu2(struct kthread_work *work)
{
	struct osd_layers_fence_map_s *data, *next;
	struct list_head saved_list;

	mutex_lock(&post_fence_list_lock[VIU2]);
	saved_list = post_fence_list[VIU2];
	list_replace_init(&post_fence_list[VIU2], &saved_list);
	mutex_unlock(&post_fence_list_lock[VIU2]);
	list_for_each_entry_safe(data, next, &saved_list, list) {
		osd_pan_display_layers_fence_viu2(data);
		list_del(&data->list);
		kfree(data);
	}
}

static void osd_toggle_buffer_viu2(struct kthread_work *work)
{
	u32 hwc_enable;

	hwc_enable = osd_hw.hwc_enable[VIU2];
	if (osd_hw.osd_fence[VIU2][hwc_enable].toggle_buffer_handler)
		osd_hw.osd_fence[VIU2][hwc_enable]
			.toggle_buffer_handler(work);
}

static int out_fence_create(u32 output_index, int *release_fence_fd)
{
	int out_fence_fd = -1;
	struct sched_param param = {.sched_priority = 2};
	char toggle_thread_name[32] = {};

	if (!timeline_created[output_index]) {
		/* timeline has not been created */
		if (osd_timeline_create(output_index)) {
			kthread_init_worker
				(&buffer_toggle_worker[output_index]);
			sprintf(toggle_thread_name,
				"aml_buf_toggle_%d", output_index);
			buffer_toggle_thread[output_index] = kthread_run(
					kthread_worker_fn,
					&buffer_toggle_worker[output_index],
					toggle_thread_name);
			if (IS_ERR(buffer_toggle_thread[output_index])) {
				osd_log_err("create osd toggle kthread failed");
				return -1;
			}
			sched_setscheduler(buffer_toggle_thread[output_index],
					   SCHED_FIFO, &param);
			if (output_index == VIU1)
				kthread_init_work(
					&buffer_toggle_work[output_index],
					osd_toggle_buffer);
			else if (output_index == VIU2)
				kthread_init_work(
					&buffer_toggle_work[output_index],
					osd_toggle_buffer_viu2);
			timeline_created[output_index] = 1;
		}
	}

	/* hwc_enable disable create fence every time */
	if (!osd_hw.hwc_enable[output_index]) {
		out_fence_fd = osd_timeline_create_fence(output_index);
		if (out_fence_fd < 0) {
			osd_log_err("fence obj create fail\n");
			out_fence_fd = -1;
		}
	} else {
		/* hwc_enable enable create fence
		 * when first sync request called
		 */
		if (osd_hw.out_fence_fd[output_index] == -1) {
			out_fence_fd = osd_timeline_create_fence(output_index);
			if (out_fence_fd < 0) {
				osd_log_err("fence obj create fail\n");
				out_fence_fd = -1;
			}
			osd_hw.out_fence_fd[output_index] = out_fence_fd;
		} else {
			out_fence_fd = osd_hw.out_fence_fd[output_index];
		}
	}
	if (release_fence_fd)
		*release_fence_fd = out_fence_fd;
	return out_fence_fd;
}

int osd_sync_request(u32 index, u32 yres, struct fb_sync_request_s *request)
{
	int out_fence_fd = -1;
	int buf_num = 0;
	int in_fence_fd = -1;
	u32 output_index = 0;
	struct osd_fence_map_s *fence_map =
		kzalloc(sizeof(struct osd_fence_map_s), GFP_KERNEL);

	output_index = get_output_device_id(index);
	osd_hw.hwc_enable[output_index] = 0;
	if (request->sync_req.magic == FB_SYNC_REQUEST_MAGIC) {
		buf_num = find_buf_num(yres, request->sync_req.yoffset);
		if (!fence_map) {
			osd_log_err("could not allocate osd_fence_map\n");
			return -ENOMEM;
		}
		mutex_lock(&post_fence_list_lock[output_index]);
		fence_map->op = 0xffffffff;
		fence_map->fb_index = index;
		fence_map->buf_num = buf_num;
		fence_map->yoffset = request->sync_req.yoffset;
		fence_map->xoffset = request->sync_req.xoffset;
		fence_map->yres = yres;
		fence_map->format = request->sync_req.format;
		fence_map->in_fd = request->sync_req.in_fen_fd;
		in_fence_fd = request->sync_req.in_fen_fd;
		osd_tprintk("request fence fd:%d\n", fence_map->in_fd);
		fence_map->in_fence = osd_get_fenceobj(
			request->sync_req.in_fen_fd);
		fence_map->out_fd =
			out_fence_create(output_index, &out_fence_fd);
	} else {
		buf_num = find_buf_num(yres, request->sync_req_old.yoffset);
		if (!fence_map) {
			osd_log_err("could not allocate osd_fence_map\n");
			return -ENOMEM;
		}
		mutex_lock(&post_fence_list_lock[output_index]);
		fence_map->op = 0xffffffff;
		fence_map->fb_index = index;
		fence_map->buf_num = buf_num;
		fence_map->yoffset = request->sync_req_old.yoffset;
		fence_map->xoffset = request->sync_req_old.xoffset;
		fence_map->yres = yres;
		fence_map->in_fd = request->sync_req_old.in_fen_fd;
		in_fence_fd = request->sync_req_old.in_fen_fd;
		osd_tprintk("request fence fd:%d\n", fence_map->in_fd);
		fence_map->in_fence = osd_get_fenceobj(
				request->sync_req_old.in_fen_fd);
		fence_map->out_fd =
			out_fence_create(output_index, &out_fence_fd);
	}
	list_add_tail(&fence_map->list, &post_fence_list[output_index]);
	mutex_unlock(&post_fence_list_lock[output_index]);
	kthread_queue_work(&buffer_toggle_worker[output_index],
			   &buffer_toggle_work[output_index]);
	if (in_fence_fd >= 0)
		__close_fd(current->files, in_fence_fd);
	return  out_fence_fd;
}


static int sync_render_single_fence(u32 index, u32 yres,
	struct sync_req_render_s *request,
	u32 phys_addr,
	size_t len)
{
	int out_fence_fd = -1;
	int buf_num = 0;
	u32 xoffset, yoffset;
	u32 output_index = 0;
	struct osd_fence_map_s *fence_map =  NULL;

	if (index > OSD1)
		return -1;
	xoffset = request->xoffset;
	yoffset = request->yoffset;
	buf_num = find_buf_num(yres, yoffset);
	fence_map = kzalloc(sizeof(struct osd_fence_map_s), GFP_KERNEL);
	if (!fence_map) {
		osd_log_err("could not allocate osd_fence_map\n");
		return -ENOMEM;
	}
	output_index = get_output_device_id(index);
	mutex_lock(&post_fence_list_lock[output_index]);
	fence_map->op = 0xffffffff;
	fence_map->fb_index = index;
	fence_map->buf_num = buf_num;
	fence_map->yoffset = yoffset;
	fence_map->xoffset = xoffset;
	fence_map->yres = yres;
	fence_map->in_fd = request->in_fen_fd;
	fence_map->ext_addr = phys_addr;
	if (fence_map->ext_addr) {
		fence_map->width = request->width;
		fence_map->height = request->height;
		fence_map->dst_x = request->dst_x;
		fence_map->dst_y = request->dst_y;
		fence_map->dst_w = request->dst_w;
		fence_map->dst_h = request->dst_h;
		fence_map->byte_stride = request->byte_stride;
		fence_map->pxiel_stride = request->pxiel_stride;
		fence_map->afbc_inter_format = request->afbc_inter_format;
		fence_map->afbc_len = len;
	}
	fence_map->format = request->format;
	fence_map->compose_type = request->type;
	fence_map->op = request->op;

	osd_tprintk("direct render fence fd:%d\n", fence_map->in_fd);
	fence_map->in_fence = osd_get_fenceobj(fence_map->in_fd);
	fence_map->out_fd =
		out_fence_create(output_index, &out_fence_fd);
	/* Todo: */
	list_add_tail(&fence_map->list, &post_fence_list[output_index]);
	mutex_unlock(&post_fence_list_lock[output_index]);
	kthread_queue_work(&buffer_toggle_worker[output_index],
			   &buffer_toggle_work[output_index]);
	request->out_fen_fd = out_fence_fd;
	if (request->in_fen_fd >= 0)
		__close_fd(current->files, request->in_fen_fd);
	return out_fence_fd;
}

static int sync_render_layers_fence(u32 index, u32 yres,
	struct sync_req_render_s *request,
	u32 phys_addr,
	size_t len)
{
	int out_fence_fd = -1;
	s32 in_fence_fd;
	u32 output_index = 0;
	struct osd_layers_fence_map_s *fence_map = NULL;

	if (index > OSD_MAX)
		return -1;
	output_index = get_output_device_id(index);
	in_fence_fd = request->in_fen_fd;
	mutex_lock(&post_fence_list_lock[output_index]);
	fence_map = &map_layers[output_index];
	fence_map->cmd = LAYER_SYNC;
	fence_map->layer_map[index].fb_index = index;
	/* layer_map[index].enable will update if have blank ioctl */
	fence_map->layer_map[index].enable = request->op;
	fence_map->layer_map[index].in_fd = request->in_fen_fd;
	if (request->shared_fd < 0)
		fence_map->layer_map[index].buf_file = NULL;
	else
		fence_map->layer_map[index].buf_file = fget(request->shared_fd);
	fence_map->layer_map[index].ext_addr = phys_addr;
	fence_map->layer_map[index].format = request->format;
	fence_map->layer_map[index].compose_type = request->type;
	fence_map->layer_map[index].fb_width = request->fb_width;
	fence_map->layer_map[index].fb_height = request->fb_height;
	fence_map->layer_map[index].src_x = request->xoffset;
	fence_map->layer_map[index].src_y = request->yoffset;
	fence_map->layer_map[index].src_w = request->width;
	fence_map->layer_map[index].src_h = request->height;
	fence_map->layer_map[index].dst_x = request->dst_x;
	fence_map->layer_map[index].dst_y = request->dst_y;
	fence_map->layer_map[index].dst_w = request->dst_w;
	fence_map->layer_map[index].dst_h = request->dst_h;
	fence_map->layer_map[index].byte_stride =
		request->byte_stride;
	fence_map->layer_map[index].pxiel_stride =
		request->pxiel_stride;
	fence_map->layer_map[index].afbc_inter_format =
		request->afbc_inter_format;
	fence_map->layer_map[index].zorder = request->zorder;
	fence_map->layer_map[index].blend_mode =
		request->blend_mode;
	fence_map->layer_map[index].plane_alpha =
		request->plane_alpha;
	fence_map->layer_map[index].dim_layer = request->dim_layer;
	fence_map->layer_map[index].dim_color = request->dim_color;
	fence_map->layer_map[index].afbc_len = len;
	/* just return out_fd,but not signal */
	/* no longer put list, will put them via do_hwc */
	fence_map->layer_map[index].in_fence = osd_get_fenceobj(in_fence_fd);
	fence_map->layer_map[index].out_fd =
		out_fence_create(output_index, &out_fence_fd);
	mutex_unlock(&post_fence_list_lock[output_index]);
	osd_log_dbg(MODULE_FENCE, "sync_render_layers_fence:osd%d: ind_fd=%d,out_fd=%d\n",
		fence_map->layer_map[index].fb_index,
		fence_map->layer_map[index].in_fd,
		fence_map->layer_map[index].out_fd);
	request->out_fen_fd = out_fence_fd;
	if (in_fence_fd >= 0)
		__close_fd(current->files, in_fence_fd);
	if (request->shared_fd >= 0)
		__close_fd(current->files, request->shared_fd);
	return out_fence_fd;
}

int osd_sync_request_render(u32 index, u32 yres,
	struct sync_req_render_s *request,
	u32 phys_addr,
	size_t len)
{
	int line, hwc_enable;
	u32 output_index = 0;

	line = get_encp_line(output_index);
	output_index = get_output_device_id(index);
	osd_log_dbg2(MODULE_RENDER,
			"enter osd_sync_request_render:encp line=%d\n",
			line);
	if (request->magic == FB_SYNC_REQUEST_RENDER_MAGIC_V1)
		osd_hw.hwc_enable[output_index] = 0;
	else if (request->magic == FB_SYNC_REQUEST_RENDER_MAGIC_V2)
		osd_hw.hwc_enable[output_index] = 1;
	output_index = get_output_device_id(index);
	hwc_enable = osd_hw.hwc_enable[output_index];

	if (osd_hw.osd_fence[output_index][hwc_enable].sync_fence_handler)
		osd_hw.osd_fence[output_index][hwc_enable]
			.sync_fence_handler(index, yres,
			request, phys_addr, len);
	return request->out_fen_fd;
}

int osd_sync_do_hwc(u32 index, struct do_hwc_cmd_s *hwc_cmd)
{
	int out_fence_fd = -1;
	struct osd_layers_fence_map_s *fence_map = NULL;
	int line;
	u32 output_index = 0;

	output_index = get_output_device_id(index);
	line = get_encp_line(output_index);
	osd_log_dbg2(MODULE_RENDER,
		"enter osd_sync_do_hwc:encp line=%d\n",
		line);
	fence_map = kzalloc(
		sizeof(struct osd_layers_fence_map_s), GFP_KERNEL);
	if (!fence_map)
		return -ENOMEM;
	osd_hw.hwc_enable[output_index] = 1;
	mutex_lock(&post_fence_list_lock[output_index]);
	memcpy(fence_map, &map_layers[output_index],
	       sizeof(struct osd_layers_fence_map_s));
	/* clear map_layers, need alloc next add_sync ioctl */
	memset(&map_layers[output_index], 0,
	       sizeof(struct osd_layers_fence_map_s));
	fence_map->out_fd =
		out_fence_create(output_index, &out_fence_fd);
	fence_map->disp_info.background_w =
		hwc_cmd->disp_info.background_w;
	fence_map->disp_info.background_h =
		hwc_cmd->disp_info.background_h;
	fence_map->disp_info.fullscreen_w =
		hwc_cmd->disp_info.fullscreen_w;
	fence_map->disp_info.fullscreen_h =
		hwc_cmd->disp_info.fullscreen_h;
	fence_map->disp_info.position_x =
		hwc_cmd->disp_info.position_x;
	fence_map->disp_info.position_y =
		hwc_cmd->disp_info.position_y;
	fence_map->disp_info.position_w =
		hwc_cmd->disp_info.position_w;
	fence_map->disp_info.position_h =
		hwc_cmd->disp_info.position_h;
	fence_map->hdr_mode = hwc_cmd->hdr_mode;
	/* other info set via add_sync and blank ioctl */
	list_add_tail(&fence_map->list, &post_fence_list[output_index]);
	/* after do_hwc, clear osd_hw.out_fence_fd */
	if (timeline_created[output_index] &&
	    osd_hw.out_fence_fd[output_index])
		osd_hw.out_fence_fd[output_index] = -1;
	mutex_unlock(&post_fence_list_lock[output_index]);
	kthread_queue_work(&buffer_toggle_worker[output_index],
			   &buffer_toggle_work[output_index]);
	if (get_logo_loaded()) {
		int logo_index;

		logo_index = osd_get_logo_index();
		if (logo_index < 0) {
			osd_log_info("set logo loaded\n");
			set_logo_loaded();
		}
	}
	osd_log_dbg(MODULE_FENCE, "osd_sync_do_hwc :out_fence_fd=%d\n",
		out_fence_fd);
	return out_fence_fd;
}

static int osd_wait_buf_ready(struct osd_fence_map_s *fence_map)
{
	s32 ret = -1;
	struct fence *buf_ready_fence = NULL;

	if (fence_map->in_fd <= 0)
		return -1;
	buf_ready_fence = fence_map->in_fence;
	if (buf_ready_fence == NULL)
		return -1;/* no fence ,output directly. */
	ret = osd_wait_fenceobj(buf_ready_fence, 4000);
	if (ret < 0) {
		osd_log_err("Sync Fence wait error:%d\n", ret);
		osd_log_err("-----wait buf idx:[%d] ERROR\n"
			"-----on screen buf idx:[%d]\n",
			fence_map->buf_num, find_buf_num(fence_map->yres,
			osd_hw.pandata[fence_map->fb_index].y_start));
	} else
		ret = 1;
	return ret;
}

static int osd_wait_buf_ready_combine(struct layer_fence_map_s *layer_map)
{
	s32 ret = -1;
	struct fence *buf_ready_fence = NULL;

	if (layer_map->in_fd <= 0)
		return -1;
	buf_ready_fence = layer_map->in_fence;
	if (buf_ready_fence == NULL) {
		return -1;/* no fence ,output directly. */
	}
	ret = osd_wait_fenceobj(buf_ready_fence, 4000);
	if (ret < 0)
		osd_log_err("osd%d: Sync Fence wait error:%d\n",
		layer_map->fb_index, ret);
	else
		ret = 1;

	osd_log_dbg(MODULE_FENCE, "osd_wait_buf_ready_combine:osd%d,in_fd=%d\n",
		layer_map->fb_index, layer_map->in_fd);
	return ret;
}

#else
int osd_sync_request(u32 index, u32 yres, struct fb_sync_request_s *request)
{
	osd_log_err("osd_sync_request not supported\n");
	return -5566;
}


int osd_sync_request_render(u32 index, u32 yres,
	struct sync_req_render_s *request,
	u32 phys_addr,
	size_t len)
{
	osd_log_err("osd_sync_request_render not supported\n");
	return -5566;
}

int osd_sync_do_hwc(u32 output_index, struct do_hwc_cmd_s *hwc_cmd)
{
	osd_log_err("osd_do_hwc not supported\n");
	return -5566;
}
#endif


void osd_set_enable_hw(u32 index, u32 enable)
{
	u32 output_index = 0;

	output_index = get_output_device_id(index);
	if (osd_hw.hwc_enable[output_index]) {
		if (index > OSD_MAX)
			return;
		if ((osd_hw.osd_meson_dev.osd_ver < OSD_HIGH_ONE)
			&& (index == OSD2))
			osd_enable_hw(index, enable);
		else {
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
			mutex_lock(&post_fence_list_lock[output_index]);
			map_layers[output_index].layer_map[index]
				.fb_index = index;
			map_layers[output_index].layer_map[index]
				.enable = enable;
			map_layers[output_index].cmd = BLANK_CMD;
			mutex_unlock(&post_fence_list_lock[output_index]);
			osd_log_dbg(MODULE_BASE, "osd_set_enable_hw: osd%d,enable=%d\n",
				index, enable);
#endif
		}
	} else
		osd_enable_hw(index, enable);
}

void osd_update_3d_mode(void)
{
	int i = 0;

	/* only called by vsync irq or rdma irq */
	for (i = 0; i <= OSD2; i++)
		if (osd_hw.mode_3d[i].enable)
			osd_update_disp_3d_mode(i);
}

static inline void wait_vsync_wakeup(void)
{
	user_vsync_hit[VIU1] = vsync_hit[VIU1] = true;
	wake_up_interruptible_all(&osd_vsync_wq);
}

static inline void wait_vsync_wakeup_viu2(void)
{
	user_vsync_hit[VIU2] = vsync_hit[VIU2] = true;
	wake_up_interruptible_all(&osd_vsync2_wq);
}

void osd_update_vsync_hit(void)
{
	ktime_t stime;

	stime = ktime_get();
	timestamp[VIU1] = stime.tv64;
#ifdef FIQ_VSYNC
	fiq_bridge_pulse_trigger(&osd_hw.fiq_handle_item);
#else
	wait_vsync_wakeup();
#endif
}

void osd_update_vsync_hit_viu2(void)
{
	ktime_t stime;

	stime = ktime_get();
	timestamp[VIU2] = stime.tv64;
#ifdef FIQ_VSYNC
	fiq_bridge_pulse_trigger(&osd_hw.fiq_handle_item);
#else
	wait_vsync_wakeup_viu2();
#endif
}

/* the return stride unit is 128bit(16bytes) */
static u32 line_stride_calc(
		u32 fmt_mode,
		u32 hsize,
		u32 stride_align_32bytes)
{
	u32 line_stride = 0;

	switch (fmt_mode) {
	/* 2-bit LUT */
	case COLOR_INDEX_02_PAL4:
		line_stride = ((hsize<<1)+127)>>7;
		break;
	/* 4-bit LUT */
	case COLOR_INDEX_04_PAL16:
		line_stride = ((hsize<<2)+127)>>7;
		break;
	/* 8-bit LUT */
	case COLOR_INDEX_08_PAL256:
		line_stride = ((hsize<<3)+127)>>7;
		break;
	/* 4:2:2, 32-bit per 2 pixels */
	case COLOR_INDEX_YUV_422:
		line_stride = ((((hsize+1)>>1)<<5)+127)>>7;
		break;
	/* 16-bit LUT */
	case COLOR_INDEX_16_655:
	case COLOR_INDEX_16_844:
	case COLOR_INDEX_16_6442:
	case COLOR_INDEX_16_4444_R:
	case COLOR_INDEX_16_4642_R:
	case COLOR_INDEX_16_1555_A:
	case COLOR_INDEX_16_4444_A:
	case COLOR_INDEX_16_565:
		line_stride = ((hsize<<4)+127)>>7;
		break;
	/* 32-bit LUT */
	case COLOR_INDEX_32_BGRX:
	case COLOR_INDEX_32_XBGR:
	case COLOR_INDEX_32_RGBX:
	case COLOR_INDEX_32_XRGB:
	case COLOR_INDEX_32_BGRA:
	case COLOR_INDEX_32_ABGR:
	case COLOR_INDEX_32_RGBA:
	case COLOR_INDEX_32_ARGB:
		line_stride = ((hsize<<5)+127)>>7;
		break;
	/* 24-bit LUT */
	case COLOR_INDEX_24_6666_A:
	case COLOR_INDEX_24_6666_R:
	case COLOR_INDEX_24_8565:
	case COLOR_INDEX_24_5658:
	case COLOR_INDEX_24_888_B:
	case COLOR_INDEX_24_RGB:
		line_stride = ((hsize<<4)+(hsize<<3)+127)>>7;
		break;
	}
	/* need wr ddr is 32bytes aligned */
	if (stride_align_32bytes)
		line_stride = ((line_stride+1)>>1)<<1;
	else
		line_stride = line_stride;
	return line_stride;
}

static u32 line_stride_calc_afbc(
		u32 fmt_mode,
		u32 hsize,
		u32 stride_align_32bytes)
{
	u32 line_stride = 0;

	switch (fmt_mode) {
	case R8:
		line_stride = ((hsize << 3) + 127) >> 7;
		break;
	case YUV422_8B:
	case RGB565:
	case RGBA5551:
	case RGBA4444:
		line_stride = ((hsize << 4) + 127) >> 7;
		break;
	case RGBA8888:
	case RGB888:
	case YUV422_10B:
	case RGBA1010102:
		line_stride = ((hsize << 5) + 127) >> 7;
		break;
	}
	/* need wr ddr is 32bytes aligned */
	if (stride_align_32bytes)
		line_stride = ((line_stride+1) >> 1) << 1;
	else
		line_stride = line_stride;
	return line_stride;
}


static void osd_update_phy_addr(u32 index)
{
	u32 line_stride, bpp;
	u32 fmt_mode = COLOR_INDEX_32_BGRX;

	if (osd_hw.color_info[index])
		fmt_mode =
			osd_hw.color_info[index]->color_index;
	bpp = osd_hw.color_info[index]->bpp / 8;
	line_stride = line_stride_calc(fmt_mode,
		osd_hw.fb_gem[index].width / bpp, 1);
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[index].osd_blk1_cfg_w4,
		osd_hw.fb_gem[index].addr);
	VSYNCOSD_WR_MPEG_REG_BITS(
		hw_osd_reg_array[index].osd_blk2_cfg_w4,
		line_stride,
		0, 12);
}

/* only need judge osd1 and osd2 index, osd3 same with osd1 */
static void osd_update_interlace_mode(int index)
{
	/* only called by vsync irq or rdma irq */
	unsigned int fb0_cfg_w0 = 0, fb1_cfg_w0 = 0;
	unsigned int scan_line_number = 0;
	unsigned int odd_even;

	spin_lock_irqsave(&osd_lock, lock_flags);
	if ((index & (1 << OSD1)) == (1 << OSD1))
		fb0_cfg_w0 = VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[OSD1].osd_blk0_cfg_w0);
	if ((index & (1 << OSD2)) == (1 << OSD2))
		fb1_cfg_w0 = VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[OSD2].osd_blk0_cfg_w0);

	if (osd_reg_read(ENCP_VIDEO_MODE) & (1 << 12)) {
		/* 1080I */
		scan_line_number = ((osd_reg_read(ENCP_INFO_READ))
				& 0x1fff0000) >> 16;
		if ((osd_hw.pandata[OSD1].y_start % 2) == 0) {
			if (scan_line_number >= 562) {
				/* bottom field, odd lines*/
				odd_even = OSD_TYPE_BOT_FIELD;
			} else {
				/* top field, even lines*/
				odd_even = OSD_TYPE_TOP_FIELD;
			}
		} else {
			if (scan_line_number >= 562) {
				/* top field, even lines*/
				odd_even = OSD_TYPE_TOP_FIELD;
			} else {
				/* bottom field, odd lines*/
				odd_even = OSD_TYPE_BOT_FIELD;
			}
		}
	} else {
		if ((osd_hw.pandata[OSD1].y_start % 2) == 1) {
			odd_even = (osd_reg_read(ENCI_INFO_READ) & (1 << 29)) ?
				OSD_TYPE_TOP_FIELD : OSD_TYPE_BOT_FIELD;
		} else {
			odd_even = (osd_reg_read(ENCI_INFO_READ) & (1 << 29)) ?
				OSD_TYPE_BOT_FIELD : OSD_TYPE_TOP_FIELD;
		}
	}
	if (osd_hw.hw_rdma_en)
		/* when RDMA enabled, top/bottom fields changed in next vsync */
		odd_even = (odd_even == OSD_TYPE_TOP_FIELD) ?
			OSD_TYPE_BOT_FIELD : OSD_TYPE_TOP_FIELD;
	fb0_cfg_w0 &= ~1;
	fb1_cfg_w0 &= ~1;
	fb0_cfg_w0 |= odd_even;
	fb1_cfg_w0 |= odd_even;
	if ((index & (1 << OSD1)) == (1 << OSD1)) {
		VSYNCOSD_IRQ_WR_MPEG_REG(
		hw_osd_reg_array[OSD1].osd_blk0_cfg_w0, fb0_cfg_w0);
		if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
			if ((osd_hw.osd_meson_dev.viu1_osd_count - 1) == OSD3)
				VSYNCOSD_IRQ_WR_MPEG_REG(
				hw_osd_reg_array[OSD3].osd_blk0_cfg_w0,
				fb0_cfg_w0);
		}
	}
	if ((index & (1 << OSD2)) == (1 << OSD2))
		VSYNCOSD_IRQ_WR_MPEG_REG(
		hw_osd_reg_array[OSD2].osd_blk0_cfg_w0, fb1_cfg_w0);

	spin_unlock_irqrestore(&osd_lock, lock_flags);
}

void osd_update_scan_mode(void)
{
	/* only called by vsync irq or rdma irq */
	unsigned int output_type = 0;
	int index = 0;
	int viu2_index = osd_hw.osd_meson_dev.viu2_index;
	int osd_count = osd_hw.osd_meson_dev.viu1_osd_count;
	int has_osd3 = 0;

	if ((osd_count - 1) == OSD3)
		has_osd3 = 1;
	//todo: Karry
	output_type = osd_reg_read(VPU_VIU_VENC_MUX_CTRL) & 0x3;
	osd_hw.scan_mode[OSD1] = SCAN_MODE_PROGRESSIVE;
	osd_hw.scan_mode[OSD2] = SCAN_MODE_PROGRESSIVE;
	if (has_osd3)
		osd_hw.scan_mode[OSD3] = SCAN_MODE_PROGRESSIVE;
	osd_hw.scan_mode[viu2_index] = SCAN_MODE_PROGRESSIVE;
	switch (output_type) {
	case VOUT_ENCP:
		if (osd_reg_read(ENCP_VIDEO_MODE) & (1 << 12)) {
			/* 1080i */
			osd_hw.scan_mode[OSD1] = SCAN_MODE_INTERLACE;
			osd_hw.scan_mode[OSD2] = SCAN_MODE_INTERLACE;
			if (has_osd3)
			osd_hw.scan_mode[OSD3] = SCAN_MODE_INTERLACE;
		}
		break;
	case VOUT_ENCI:
		if (osd_reg_read(ENCI_VIDEO_EN) & 1) {
			osd_hw.scan_mode[OSD1] = SCAN_MODE_INTERLACE;
			osd_hw.scan_mode[OSD2] = SCAN_MODE_INTERLACE;
			if (has_osd3)
			osd_hw.scan_mode[OSD3] = SCAN_MODE_INTERLACE;
		}
		break;
	}
	if (osd_hw.hw_cursor_en) {
		/* 3 layers osd don't support osd2 cursor */
		if (osd_hw.free_scale_enable[OSD1])
			osd_hw.scan_mode[OSD1] = SCAN_MODE_PROGRESSIVE;
		if (osd_hw.osd_afbcd[OSD1].enable)
			osd_hw.scan_mode[OSD1] = SCAN_MODE_PROGRESSIVE;

		if (osd_hw.scan_mode[OSD1] == SCAN_MODE_INTERLACE)
			index |= 1 << OSD1;
		if (osd_hw.scan_mode[OSD2] == SCAN_MODE_INTERLACE)
			index |= 1 << OSD2;
	} else {
		int i;

		for (i = 0; i < osd_hw.osd_meson_dev.viu1_osd_count; i++) {
			if (osd_hw.free_scale_enable[i])
				osd_hw.scan_mode[i] = SCAN_MODE_PROGRESSIVE;
			if (osd_hw.osd_afbcd[i].enable)
				osd_hw.scan_mode[i] = SCAN_MODE_PROGRESSIVE;
			if (osd_hw.scan_mode[i] == SCAN_MODE_INTERLACE)
				index |= 1 << i;
		}
	}
	if ((osd_hw.scan_mode[OSD1] == SCAN_MODE_INTERLACE)
		|| (osd_hw.scan_mode[OSD2] == SCAN_MODE_INTERLACE))
		osd_update_interlace_mode(index);
}

static void osd_update_interlace_mode_viu2(void)
{
	/* only called by vsync irq or rdma irq */
	unsigned int fb3_cfg_w0 = 0;
	unsigned int odd_even = 0;
	u32 viu2_index;

	spin_lock_irqsave(&osd_lock, lock_flags);
	viu2_index = osd_hw.osd_meson_dev.viu2_index;
	if (osd_hw.powered[viu2_index]) {
		fb3_cfg_w0 = VSYNCOSD_RD_MPEG_REG(
			hw_osd_reg_array[viu2_index].osd_blk0_cfg_w0);
		if (osd_hw.scan_mode[viu2_index] == SCAN_MODE_INTERLACE) {
			if ((osd_hw.pandata[viu2_index].y_start % 2) == 1) {
				odd_even = (osd_reg_read(ENCI_INFO_READ) &
					(1 << 29)) ? OSD_TYPE_TOP_FIELD :
					OSD_TYPE_BOT_FIELD;
			} else {
				odd_even = (osd_reg_read(ENCI_INFO_READ)
				& (1 << 29)) ? OSD_TYPE_BOT_FIELD :
				OSD_TYPE_TOP_FIELD;
			}
		}
		fb3_cfg_w0 &= ~1;
		fb3_cfg_w0 |= odd_even;
		VSYNCOSD_IRQ_WR_MPEG_REG(
			hw_osd_reg_array[viu2_index].osd_blk0_cfg_w0,
			fb3_cfg_w0);
	}
	spin_unlock_irqrestore(&osd_lock, lock_flags);
}

void osd_update_scan_mode_viu2(void)
{
	/* only called by vsync irq or rdma irq */
	unsigned int output_type = 0;
	int viu2_index = osd_hw.osd_meson_dev.viu2_index;

	output_type = osd_reg_read(VPU_VIU_VENC_MUX_CTRL) & 0x3;
	osd_hw.scan_mode[viu2_index] = SCAN_MODE_PROGRESSIVE;
	if (osd_hw.powered[viu2_index]) {
		output_type = (osd_reg_read(VPU_VIU_VENC_MUX_CTRL)
			>> 2) & 0x3;
		switch (output_type) {
		case VOUT_ENCP:
			if (osd_reg_read(ENCP_VIDEO_MODE) & (1 << 12))
				/* 1080i */
				osd_hw.scan_mode[viu2_index] =
				SCAN_MODE_INTERLACE;
			break;
		case VOUT_ENCI:
			if (osd_reg_read(ENCI_VIDEO_EN) & 1)
				osd_hw.scan_mode[viu2_index] =
				SCAN_MODE_INTERLACE;
			break;
		}

	}
	if (osd_hw.scan_mode[viu2_index] == SCAN_MODE_INTERLACE)
		osd_update_interlace_mode_viu2();
}

//not rdma will call update func;
void walk_through_update_list(void)
{
	u32  i, j;

	for (i = 0; i < HW_OSD_COUNT; i++) {
		j = 0;
		while (osd_hw.updated[i] && j < HW_REG_INDEX_MAX) {
			if (osd_hw.updated[i] & (1 << j)) {
				if (osd_hw.reg[j].update_func)
					osd_hw.reg[j].update_func(i);
				remove_from_update_list(i, j);
			}
			j++;
		}
	}
}

/*************** for GXL/GXM hardware alpha bug workaround ***************/
static bool mali_afbc_get_error(void)
{
	u32 status;
	bool error = false;

	status = VSYNCOSD_RD_MPEG_REG(VPU_MAFBC_IRQ_RAW_STATUS);
	if (status & 0x3c) {
		osd_log_dbg(MODULE_BASE,
			"afbc error happened,status=0x%x\n", status);
		osd_hw.afbc_err_cnt++;
		error = true;
	}
	status = VSYNCOSD_WR_MPEG_REG(VPU_MAFBC_IRQ_CLEAR, 0x3f);
	return error;
}
static u32 osd_get_hw_reset_flag(void)
{
	u32 hw_reset_flag = HW_RESET_NONE;

	/* check hw version */
	switch (osd_hw.osd_meson_dev.cpu_id) {
	case __MESON_CPU_MAJOR_ID_GXTVBB:
		if (osd_hw.osd_afbcd[OSD1].enable)
			hw_reset_flag |= HW_RESET_AFBCD_REGS;
		break;
	case __MESON_CPU_MAJOR_ID_GXM:
		/* same bit, but gxm only reset hardware, not top reg*/
		if (osd_hw.osd_afbcd[OSD1].enable)
			hw_reset_flag |= HW_RESET_AFBCD_HARDWARE;
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_VECM
		if (((hdr_osd_reg.viu_osd1_matrix_ctrl & 0x00000001)
			!= 0x0) ||
			((hdr_osd_reg.viu_osd1_eotf_ctl & 0x80000000)
			!= 0) ||
			((hdr_osd_reg.viu_osd1_oetf_ctl & 0xe0000000)
			!= 0)) {
			hw_reset_flag |= HW_RESET_OSD1_REGS;
			osd_hdr_on = true;
		} else if (osd_hdr_on) {
			hw_reset_flag |= HW_RESET_OSD1_REGS;
			osd_hdr_on = false;
		}
#endif
		break;
	case __MESON_CPU_MAJOR_ID_GXL:
	case __MESON_CPU_MAJOR_ID_TXL:
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_VECM

		if (((hdr_osd_reg.viu_osd1_matrix_ctrl & 0x00000001)
			!= 0x0) ||
			((hdr_osd_reg.viu_osd1_eotf_ctl & 0x80000000)
			!= 0) ||
			((hdr_osd_reg.viu_osd1_oetf_ctl & 0xe0000000)
			!= 0)) {
			hw_reset_flag |= HW_RESET_OSD1_REGS;
			osd_hdr_on = true;
		} else if (osd_hdr_on) {
			hw_reset_flag |= HW_RESET_OSD1_REGS;
			osd_hdr_on = false;
		}
#endif
		break;
	case __MESON_CPU_MAJOR_ID_G12A:
	case __MESON_CPU_MAJOR_ID_G12B:
	case __MESON_CPU_MAJOR_ID_TL1:
	case __MESON_CPU_MAJOR_ID_SM1:
	case __MESON_CPU_MAJOR_ID_TM2:
		{
		int i, afbc_enable = 0;

		for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++)
			afbc_enable |= osd_hw.osd_afbcd[i].enable;
		if (afbc_enable &&
			osd_hw.afbc_force_reset)
			hw_reset_flag |= HW_RESET_MALI_AFBCD_REGS;
		if (afbc_enable && mali_afbc_get_error() &&
			osd_hw.afbc_status_err_reset)
			hw_reset_flag |= HW_RESET_MALI_AFBCD_REGS;
		}
		break;
	default:
		hw_reset_flag = HW_RESET_NONE;
		break;
	}
	return hw_reset_flag;
}

void osd_hw_reset(void)
{
	/* only called by vsync irq or rdma irq */
	u32 backup_mask;
	u32 reset_bit =
		osd_get_hw_reset_flag();

	backup_mask = is_backup();
	osd_hw.hw_reset_flag = reset_bit;
	if (reset_bit == HW_RESET_NONE)
		return;
	spin_lock_irqsave(&osd_lock, lock_flags);
	if ((reset_bit & HW_RESET_OSD1_REGS)
		&& !(backup_mask & HW_RESET_OSD1_REGS))
		reset_bit &= ~HW_RESET_OSD1_REGS;

	if (((get_cpu_type() == MESON_CPU_MAJOR_ID_GXTVBB) &&
	     (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC))
		&& (reset_bit & HW_RESET_AFBCD_REGS)
		&& !(backup_mask & HW_RESET_AFBCD_REGS))
		reset_bit &= ~HW_RESET_AFBCD_REGS;

	if ((osd_hw.osd_meson_dev.afbc_type == MALI_AFBC)
		&& (reset_bit & HW_RESET_MALI_AFBCD_REGS)
		&& !(backup_mask & HW_RESET_MALI_AFBCD_REGS))
		reset_bit &= ~HW_RESET_MALI_AFBCD_REGS;

	if (!osd_hw.hw_rdma_en) {
		/* if not osd rdma, don't reset osd1 for hdr */
		reset_bit &= ~HW_RESET_OSD1_REGS;
		backup_mask &= ~HW_RESET_OSD1_REGS;

		VSYNCOSD_IRQ_WR_MPEG_REG(
			VIU_SW_RESET, reset_bit);
		VSYNCOSD_IRQ_WR_MPEG_REG(
			VIU_SW_RESET, 0);
		if (reset_bit & HW_RESET_OSD1_REGS) {
			/* restore osd regs */
			int i;
			u32 addr;
			u32 base = hw_osd_reg_array[OSD1].osd_ctrl_stat;

			for (i = 0; i < OSD_REG_BACKUP_COUNT; i++) {
				addr = osd_reg_backup[i];
				VSYNCOSD_IRQ_WR_MPEG_REG(
					addr, osd_backup[addr - base]);
			}
		}

		if ((osd_hw.osd_meson_dev.afbc_type == MESON_AFBC)
			&& (reset_bit & HW_RESET_AFBCD_REGS)) {
			/* restore osd afbcd regs */
			int i;
			u32 addr;
			u32 value;
			u32 base = OSD1_AFBCD_ENABLE;

			for (i = 0; i < OSD_AFBC_REG_BACKUP_COUNT; i++) {
				addr = osd_afbc_reg_backup[i];
				value = osd_afbc_backup[addr - base];
				if (addr == OSD1_AFBCD_ENABLE)
					value |= 0x100;
				VSYNCOSD_IRQ_WR_MPEG_REG(
					addr, value);
			}
		}
		if ((osd_hw.osd_meson_dev.afbc_type == MALI_AFBC)
			&& (reset_bit & HW_RESET_MALI_AFBCD_REGS)) {
			/* restore mali afbcd regs */
			if (osd_hw.afbc_regs_backup) {
				int i;
				u32 addr;
				u32 value;
				u32 base = VPU_MAFBC_IRQ_MASK;

				for (i = 0; i < MALI_AFBC_REG_BACKUP_COUNT;
					i++) {
					addr = mali_afbc_reg_backup[i];
					value = mali_afbc_backup[addr - base];
					VSYNCOSD_IRQ_WR_MPEG_REG(
						addr, value);
				}
			}
			VSYNCOSD_IRQ_WR_MPEG_REG(VPU_MAFBC_COMMAND, 1);
		}
	} else
		osd_rdma_reset_and_flush(reset_bit);
	spin_unlock_irqrestore(&osd_lock, lock_flags);
	/* maybe change reset bit */
	osd_hw.hw_reset_flag = reset_bit;
}

static int notify_to_amvideo(void)
{
	u32 para[2];

	para[0] = osd_vpp_misc;
	para[1] = osd_vpp_misc_mask;
	pr_debug(
		"osd notify_to_amvideo vpp misc:0x%08x, mask:0x%08x\n",
		para[0], para[1]);
	if (osd_hw.hw_rdma_en) {
#ifdef CONFIG_AMLOGIC_MEDIA_VIDEO
		amvideo_notifier_call_chain(
			AMVIDEO_UPDATE_OSD_MODE,
			(void *)&para[0]);
#endif
	}
	return 0;
}
/*************** end of GXL/GXM hardware alpha bug workaround ***************/
static void viu2_osd_reg_table_init(void)
{
	int i = 0;

	for (i = 0; i < VIU2_OSD_REG_NUM; i++) {
		viu2_osd_reg_table[i].val =
			osd_reg_read(viu2_osd_reg_table[i].addr)
			& viu2_osd_reg_table[i].mask;
		viu2_osd_table[i] = viu2_osd_reg_table[i].val;
		osd_log_dbg(MODULE_VIU2, "init: reg:%x = %x\n",
			viu2_osd_reg_table[i].addr,
			viu2_osd_reg_table[i].val);
	}
}

static void viu2_osd_reg_table_write(u32 index)
{

	if ((viu2_osd_table[index] & viu2_osd_reg_table[index].mask) !=
		(viu2_osd_reg_table[index].val &
		viu2_osd_reg_table[index].mask)) {
		/* not same, need write to hw regs*/
		osd_reg_write(viu2_osd_reg_table[index].addr,
			viu2_osd_table[index] & viu2_osd_reg_table[index].mask);
		viu2_osd_reg_table[index].val =
			viu2_osd_table[index] & viu2_osd_reg_table[index].mask;
		osd_log_dbg(MODULE_VIU2, "write: reg:%x = %x, update table:%x\n",
			viu2_osd_reg_table[index].addr,
			viu2_osd_table[index],
			viu2_osd_reg_table[index].val);
	}
}

u32 viu2_osd_reg_read(u32 addr)
{
	int i = 0;

	for (i = 0; i < VIU2_OSD_REG_NUM; i++) {
		if (addr == viu2_osd_reg_table[i].addr)
			return viu2_osd_table[i];
	}
	return 0;
}

void viu2_osd_reg_set(u32 addr, u32 val)
{
	int i = 0;

	osd_log_dbg2(MODULE_VIU2, "%s: reg:%x, val=%x\n",
		__func__, addr, val);
	for (i = 0; i < VIU2_OSD_REG_NUM; i++) {
		if (addr == viu2_osd_reg_table[i].addr) {
			viu2_osd_table[i] = val;
			osd_log_dbg2(MODULE_VIU2, "%s: set table:%x\n",
				__func__,
				viu2_osd_table[i]);
			viu2_osd_reg_table_write(i);
			break;
		}
	}
}

void viu2_osd_reg_set_bits(u32 addr, u32 val, u32 start, u32 len)
{
	int i = 0;

	osd_log_dbg2(MODULE_VIU2, "%s: reg:%x,val=%x,%x,%x\n",
		__func__, addr, val, start, len);
	for (i = 0; i < VIU2_OSD_REG_NUM; i++) {
		if (addr == viu2_osd_reg_table[i].addr) {
			viu2_osd_table[i] = ((viu2_osd_table[i] &
					 ~(((1L << (len)) - 1) << (start))) |
					(((val) & ((1L << (len)) - 1))
					<< (start)));
			osd_log_dbg2(MODULE_VIU2, "%s: set table:%x\n",
				__func__,
				viu2_osd_table[i]);
			viu2_osd_reg_table_write(i);
			break;
		}
	}
}

void viu2_osd_reg_set_mask(u32 addr, u32 _mask)
{
	int i = 0;

	osd_log_dbg2(MODULE_VIU2, "%s: reg:%x, mask:%x\n",
		__func__, addr, _mask);
	for (i = 0; i < VIU2_OSD_REG_NUM; i++) {
		if (addr == viu2_osd_reg_table[i].addr) {
			viu2_osd_table[i] = (viu2_osd_table[i] | (_mask));
			osd_log_dbg2(MODULE_VIU2, "%s: set table:%x\n",
				__func__,
				viu2_osd_table[i]);
			viu2_osd_reg_table_write(i);
			break;
		}
	}
}

void viu2_osd_reg_clr_mask(u32 addr, u32 _mask)
{
	int i = 0;

	osd_log_dbg2(MODULE_VIU2, "%s: reg:%x, mask:%x\n",
		__func__, addr, _mask);
	for (i = 0; i < VIU2_OSD_REG_NUM; i++) {
		if (addr == viu2_osd_reg_table[i].addr) {
			viu2_osd_table[i] = (viu2_osd_table[i] & (~(_mask)));
			osd_log_dbg2(MODULE_VIU2, "%s: set table:%x\n",
				__func__,
				viu2_osd_table[i]);
			viu2_osd_reg_table_write(i);
			break;
		}
	}
}

#ifdef FIQ_VSYNC
static irqreturn_t vsync_isr(int irq, void *dev_id)
{
	wait_vsync_wakeup();
	return IRQ_HANDLED;
}

static void osd_fiq_isr(void)
#else
static irqreturn_t vsync_isr(int irq, void *dev_id)
#endif
{
	if (!osd_hw.hw_rdma_en) {
		osd_update_scan_mode();
		/* go through update list */
		walk_through_update_list();
		osd_update_3d_mode();
		osd_mali_afbc_start();
		osd_update_vsync_hit();
		osd_hw_reset();
	} else
		osd_rdma_interrupt_done_clear();

#ifndef FIQ_VSYNC
	return IRQ_HANDLED;
#endif
}

#ifdef FIQ_VSYNC
static irqreturn_t vsync_viu2_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static void osd_viu2_fiq_isr(void)
#else
static irqreturn_t vsync_viu2_isr(int irq, void *dev_id)
#endif
{
	/* osd_update_scan_mode_viu2(); */
	osd_update_vsync_hit_viu2();
#ifndef FIQ_VSYNC
	return IRQ_HANDLED;
#endif
}

void osd_set_pxp_mode(u32 mode)
{
	pxp_mode = mode;
}
void osd_set_afbc(u32 index, u32 enable)
{

	if (osd_hw.osd_meson_dev.afbc_type)
		osd_hw.osd_afbcd[index].enable = enable;
	osd_log_info("afbc_type=%d,enable=%d\n",
		osd_hw.osd_meson_dev.afbc_type,
		osd_hw.osd_afbcd[index].enable);
}

u32 osd_get_afbc(u32 index)
{
	u32 afbc_type = 0;

	if (osd_hw.osd_meson_dev.cpu_id ==
		__MESON_CPU_MAJOR_ID_GXM)
		afbc_type = 1;
	else if (osd_hw.osd_meson_dev.cpu_id >=
		__MESON_CPU_MAJOR_ID_G12A)
		afbc_type = 2;
	else
		afbc_type = 0;
	return afbc_type;
}

u32 osd_get_reset_status(void)
{
	return osd_hw.hw_reset_flag;
}

static void osd_wait_vsync_hw_viu1(void)
{
	unsigned long timeout;

	if (osd_hw.fb_drvier_probe) {
		vsync_hit[VIU1] = false;

		if (pxp_mode)
			timeout = msecs_to_jiffies(50);
		else {
			struct vinfo_s *vinfo;

			vinfo = get_current_vinfo();
			if (vinfo && (!strcmp(vinfo->name, "invalid") ||
				!strcmp(vinfo->name, "null"))) {
				timeout = msecs_to_jiffies(1);
			} else
				timeout = msecs_to_jiffies(1000);
		}
		wait_event_interruptible_timeout(
				osd_vsync_wq, vsync_hit[VIU1], timeout);
	}
}

static void osd_wait_vsync_hw_viu2(void)
{
	unsigned long timeout;

	if (osd_hw.fb_drvier_probe) {
		vsync_hit[VIU2] = false;

		if (pxp_mode)
			timeout = msecs_to_jiffies(50);
		else {
			struct vinfo_s *vinfo;

			vinfo = get_current_vinfo2();
			if (vinfo && (!strcmp(vinfo->name, "invalid") ||
				!strcmp(vinfo->name, "null"))) {
				timeout = msecs_to_jiffies(1);
			} else
				timeout = msecs_to_jiffies(1000);
		}
		wait_event_interruptible_timeout(
				osd_vsync2_wq, vsync_hit[VIU2], timeout);
	}
}

void osd_wait_vsync_hw(u32 index)
{
	u32 output_index = get_output_device_id(index);

	if (output_index == VIU1)
		osd_wait_vsync_hw_viu1();
	else if (output_index == VIU2)
		osd_wait_vsync_hw_viu2();
}

s64 osd_wait_vsync_event(void)
{
	unsigned long timeout;

	user_vsync_hit[VIU1] = false;

	if (pxp_mode)
		timeout = msecs_to_jiffies(50);
	else
		timeout = msecs_to_jiffies(1000);

	/* waiting for 10ms. */
	wait_event_interruptible_timeout(osd_vsync_wq,
		user_vsync_hit[VIU1], timeout);

	return timestamp[VIU1];
}

s64 osd_wait_vsync_event_viu2(void)
{
	unsigned long timeout;

	user_vsync_hit[VIU2] = false;

	if (pxp_mode)
		timeout = msecs_to_jiffies(50);
	else
		timeout = msecs_to_jiffies(1000);

	/* waiting for 10ms. */
	wait_event_interruptible_timeout(osd_vsync2_wq,
		user_vsync_hit[VIU2], timeout);

	return timestamp[VIU2];
}

int is_interlaced(struct vinfo_s *vinfo)
{
	if (!vinfo)
		return 0;
	if (vinfo->mode == VMODE_CVBS)
		return 1;
	if (vinfo->height != vinfo->field_height)
		return 1;
	else
		return 0;
}

int osd_set_scan_mode(u32 index)
{
	struct vinfo_s *vinfo = NULL;
	u32 data32 = 0x0;
	s32 y_end = 0;
	u32 output_index;

	output_index = get_output_device_id(index);
	osd_hw.scan_mode[index] = SCAN_MODE_PROGRESSIVE;
	if (output_index == VIU1)
		vinfo = get_current_vinfo();
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
		else if (output_index == VIU2)
			vinfo = get_current_vinfo2();
#endif
	if (vinfo && (strcmp(vinfo->name, "invalid") &&
		strcmp(vinfo->name, "null"))) {
		osd_hw.scale_workaround = 0;
		if (osd_auto_adjust_filter) {
			osd_h_filter_mode = 1;
			osd_v_filter_mode = 1;
		}
		if ((osd_hw.fb_for_4k2k) &&
		    (osd_hw.free_scale_enable[index]))
			osd_hw.scale_workaround = 1;

		if (is_interlaced(vinfo)) {
			osd_hw.scan_mode[index] = SCAN_MODE_INTERLACE;
			if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL)
				osd_hw.scan_mode[OSD2] = SCAN_MODE_INTERLACE;
			y_end = osd_hw.free_src_data[index].y_end;
			if ((vinfo->width == 720)
				&& (vinfo->height == 480)) {
				if (osd_hw.free_scale_mode[index]) {
					osd_hw.field_out_en[output_index] = 1;
					switch (y_end) {
					case 719:
						osd_hw.bot_type = 2;
						break;
					case 1079:
						osd_hw.bot_type = 3;
						break;
					default:
						osd_hw.bot_type = 2;
						break;
					}
				}
				if (osd_auto_adjust_filter) {
					osd_h_filter_mode = 6;
					osd_v_filter_mode = 6;
				}
			} else if ((vinfo->width == 720)
				&& (vinfo->height == 576)) {
				if (osd_hw.free_scale_mode[index]) {
					osd_hw.field_out_en[output_index] = 1;
					switch (y_end) {
					case 719:
						osd_hw.bot_type = 2;
						break;
					case 1079:
						osd_hw.bot_type = 2;
						break;
					default:
						osd_hw.bot_type = 2;
						break;
					}
				}
				if (osd_auto_adjust_filter) {
					osd_h_filter_mode = 6;
					osd_v_filter_mode = 6;
				}

			} else if ((vinfo->width == 1920)
				&& (vinfo->height == 1080)) {
				if (osd_hw.free_scale_mode[index]) {
					osd_hw.field_out_en[output_index] = 1;
					switch (y_end) {
					case 719:
						osd_hw.bot_type = 1;
						break;
					case 1079:
						osd_hw.bot_type = 2;
						break;
					default:
						osd_hw.bot_type = 1;
						break;
					}
				}
			}
		} else {
			if (((vinfo->width == 3840)
				&& (vinfo->height == 2160))
				|| ((vinfo->width == 4096)
				&& (vinfo->height == 2160))) {
				osd_hw.field_out_en[output_index] = 0;
			} else if (((vinfo->width == 720)
				&& (vinfo->height == 480))
				|| ((vinfo->width == 720)
				&& (vinfo->height == 576))) {
				if (osd_auto_adjust_filter) {
					osd_h_filter_mode = 6;
					osd_v_filter_mode = 6;
				}
				if (osd_hw.free_scale_mode[index])
					osd_hw.field_out_en[output_index] = 0;
			} else {
				if (osd_hw.free_scale_mode[index])
					osd_hw.field_out_en[output_index] = 0;
			}
		}
	}
	if (osd_hw.free_scale_enable[index])
		osd_hw.scan_mode[index] = SCAN_MODE_PROGRESSIVE;
	if (osd_hw.osd_afbcd[index].enable)
		osd_hw.scan_mode[index] = SCAN_MODE_PROGRESSIVE;
	if (index == OSD2) {
		if (osd_hw.scan_mode[OSD2] == SCAN_MODE_INTERLACE)
			return 1;
		data32 = (VSYNCOSD_RD_MPEG_REG(
			hw_osd_reg_array[index].osd_blk0_cfg_w0) & 3) >> 1;
	} else
		data32 = (VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[index].osd_blk0_cfg_w0) & 3) >> 1;
	if (data32 == osd_hw.scan_mode[index])
		return 1;
	else
		return 0;
}

void  osd_set_gbl_alpha_hw(u32 index, u32 gbl_alpha)
{
	/* normalized */
	if (gbl_alpha == 0xff)
		gbl_alpha = 0x100;
	if (osd_hw.gbl_alpha[index] != gbl_alpha) {
		osd_hw.gbl_alpha[index] = gbl_alpha;
		add_to_update_list(index, OSD_GBL_ALPHA);
		osd_wait_vsync_hw(index);
	}
}

u32 osd_get_gbl_alpha_hw(u32  index)
{
	return osd_hw.gbl_alpha[index];
}

void  osd_set_line_n_rdma(u32 line_n_rdma)
{
	if (osd_hw.line_n_rdma != line_n_rdma) {
		osd_hw.line_n_rdma = line_n_rdma;
		if (osd_hw.line_n_rdma)
			enable_line_n_rdma();
		else
			enable_vsync_rdma();
	}
}

u32 osd_get_line_n_rdma(void)
{
	return osd_hw.line_n_rdma;
}

void osd_set_blend_bypass(int index, u32 blend_bypass)
{
	if (blend_bypass != osd_hw.blend_bypass) {
		osd_hw.blend_bypass = blend_bypass;
		if (blend_bypass) {
			VSYNCOSD_WR_MPEG_REG_BITS
				(hw_osd_reg_array[index].osd_mali_unpack_ctrl,
				0x0, 28, 1);
			VSYNCOSD_WR_MPEG_REG_BITS
				(VIU_OSD_BLEND_CTRL,
				0x1, 26, 1);
		}
	}
}

u32 osd_get_blend_bypass(void)
{
	return osd_hw.blend_bypass;
}

void osd_set_color_key_hw(u32 index, u32 color_index, u32 colorkey)
{
	u8 r = 0, g = 0, b = 0, a = (colorkey & 0xff000000) >> 24;
	u32 data32;

	colorkey &= 0x00ffffff;
	switch (color_index) {
	case COLOR_INDEX_16_655:
		r = (colorkey >> 10 & 0x3f) << 2;
		g = (colorkey >> 5 & 0x1f) << 3;
		b = (colorkey & 0x1f) << 3;
		break;
	case COLOR_INDEX_16_844:
		r = colorkey >> 8 & 0xff;
		g = (colorkey >> 4 & 0xf) << 4;
		b = (colorkey & 0xf) << 4;
		break;
	case COLOR_INDEX_16_565:
		r = (colorkey >> 11 & 0x1f) << 3;
		g = (colorkey >> 5 & 0x3f) << 2;
		b = (colorkey & 0x1f) << 3;
		break;
	case COLOR_INDEX_24_888_B:
		b = colorkey >> 16 & 0xff;
		g = colorkey >> 8 & 0xff;
		r = colorkey & 0xff;
		break;
	case COLOR_INDEX_24_RGB:
	case COLOR_INDEX_YUV_422:
		r = colorkey >> 16 & 0xff;
		g = colorkey >> 8 & 0xff;
		b = colorkey & 0xff;
		break;
	}
	data32 = r << 24 | g << 16 | b << 8 | a;
	if (osd_hw.color_key[index] != data32) {
		osd_hw.color_key[index] = data32;
		osd_log_dbg2(MODULE_BASE,
			"bpp:%d--r:0x%x g:0x%x b:0x%x ,a:0x%x\n",
			color_index, r, g, b, a);
		add_to_update_list(index, OSD_COLOR_KEY);
		osd_wait_vsync_hw(index);
	}
}
void  osd_srckey_enable_hw(u32  index, u8 enable)
{
	if (enable != osd_hw.color_key_enable[index]) {
		osd_hw.color_key_enable[index] = enable;
		add_to_update_list(index, OSD_COLOR_KEY_ENABLE);
		osd_wait_vsync_hw(index);
	}
}

void osd_set_color_mode(u32 index, const struct color_bit_define_s *color)
{
	if (color != osd_hw.color_info[index]) {
		osd_hw.color_info[index] = color;
		osd_hw.color_backup[index] = color;
		add_to_update_list(index, OSD_COLOR_MODE);
	}
}

void osd_update_disp_axis_hw(
	u32 index,
	u32 display_h_start,
	u32 display_h_end,
	u32 display_v_start,
	u32 display_v_end,
	u32 xoffset,
	u32 yoffset,
	u32 mode_change)
{
	struct pandata_s disp_data;
	struct pandata_s pan_data;

	if (osd_hw.color_info[index] == NULL)
		return;
	disp_data.x_start = display_h_start;
	disp_data.y_start = display_v_start;
	disp_data.x_end = display_h_end;
	disp_data.y_end = display_v_end;
	pan_data.x_start = xoffset;
	pan_data.x_end = xoffset +
		osd_hw.pandata[index].x_end -
		osd_hw.pandata[index].x_start;
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
	pan_data.y_start = osd_hw.pandata[index].y_start;
	pan_data.y_end = osd_hw.pandata[index].y_start +
		osd_hw.pandata[index].y_end - osd_hw.pandata[index].y_start;
#else
	pan_data.y_start = yoffset;
	pan_data.y_end = yoffset +
		osd_hw.pandata[index].y_end - osd_hw.pandata[index].y_start;
#endif
	/* if output mode change then reset pan ofFfset. */
	memcpy(&osd_hw.pandata[index], &pan_data, sizeof(struct pandata_s));
	memcpy(&osd_hw.dispdata[index], &disp_data, sizeof(struct pandata_s));
	memcpy(&osd_hw.dispdata_backup[index],
		&disp_data, sizeof(struct pandata_s));
	osd_log_info("osd_update_disp_axis_hw:pan_data(%d,%d,%d,%d)\n",
		pan_data.x_start,
		pan_data.y_start,
		pan_data.x_end,
		pan_data.y_end);
	osd_log_info("osd_update_disp_axis_hw:dispdata(%d,%d,%d,%d)\n",
		disp_data.x_start,
		disp_data.y_start,
		disp_data.x_end,
		disp_data.y_end);
	spin_lock_irqsave(&osd_lock, lock_flags);
	if (mode_change) /* modify pandata . */
		osd_hw.reg[OSD_COLOR_MODE].update_func(index);
	osd_hw.reg[DISP_GEOMETRY].update_func(index);
	osd_update_window_axis = true;
	spin_unlock_irqrestore(&osd_lock, lock_flags);
	osd_wait_vsync_hw(index);
}

void osd_setup_hw(u32 index,
		  struct osd_ctl_s *osd_ctl,
		  u32 xoffset,
		  u32 yoffset,
		  u32 xres,
		  u32 yres,
		  u32 xres_virtual,
		  u32 yres_virtual,
		  u32 disp_start_x,
		  u32 disp_start_y,
		  u32 disp_end_x,
		  u32 disp_end_y,
		  u32 fbmem,
		  phys_addr_t *afbc_fbmem,
		  const struct color_bit_define_s *color)
{
	struct pandata_s disp_data;
	struct pandata_s pan_data;
	int update_color_mode = 0;
	int update_geometry = 0;
	u32 w = (color->bpp * xres_virtual + 7) >> 3;
	u32 i;

	osd_hw.buffer_alloc[index] = 1;
	pan_data.x_start = xoffset;
	pan_data.y_start = yoffset;
	disp_data.x_start = disp_start_x;
	disp_data.y_start = disp_start_y;

	pan_data.x_end = xoffset + xres - 1;
	pan_data.y_end = yoffset + yres - 1;
	disp_data.x_end = disp_end_x;
	disp_data.y_end = disp_end_y;

	/* need always set color mode for osd2 */
	if ((color != osd_hw.color_info[index]) ||
		(index == (osd_hw.osd_meson_dev.osd_count - 1))) {
		update_color_mode = 1;
		osd_hw.color_info[index] = color;
		osd_hw.color_backup[index] = color;
	}

	if (osd_hw.fb_gem[index].addr != fbmem
		|| osd_hw.fb_gem[index].width != w
		|| osd_hw.fb_gem[index].height != yres_virtual) {
		osd_hw.fb_gem[index].addr = fbmem;
		osd_hw.fb_gem[index].width = w;
		osd_hw.fb_gem[index].height = yres_virtual;
		if (osd_hw.osd_afbcd[index].enable == ENABLE &&
				afbc_fbmem != NULL) {
			osd_hw.osd_afbcd[index].frame_width = xres;
			/* osd_hw.osd_afbcd[index].frame_height =
			 *	ALIGN_CALC(yres, 16) +
			 *	ALIGN_CALC(yres, 64) / 64;
			 */
			osd_hw.osd_afbcd[index].frame_height = yres;
			for (i = 0; i < OSD_MAX_BUF_NUM; i++)
				osd_hw.osd_afbcd[index].addr[i] =
					(u32)afbc_fbmem[i];
			osd_hw.osd_afbcd[index].phy_addr =
				osd_hw.osd_afbcd[index].addr[0];
			/* we need update geometry
			 * and color mode for afbc mode
			 * update_geometry = 1;
			 * update_color_mode = 1;
			 */
			if (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) {
				if (xres <= 128)
					osd_hw.osd_afbcd[index]
					    .conv_lbuf_len = 32;
				else if (xres <= 256)
					osd_hw.osd_afbcd[index]
					    .conv_lbuf_len = 64;
				else if (xres <= 512)
					osd_hw.osd_afbcd[index]
					    .conv_lbuf_len = 128;
				else if (xres <= 1024)
					osd_hw.osd_afbcd[index]
					    .conv_lbuf_len = 256;
				else if (xres <= 2048)
					osd_hw.osd_afbcd[index]
					    .conv_lbuf_len = 512;
				else
					osd_hw.osd_afbcd[index]
					    .conv_lbuf_len = 1024;
			} else if (osd_hw.osd_meson_dev.afbc_type
			    == MALI_AFBC)
				osd_hw.osd_afbcd[index].out_addr_id = index + 1;
		}
		osd_hw.fb_gem[index].xres = xres;
		osd_hw.fb_gem[index].yres = yres;
		osd_log_info("osd[%d] canvas.idx =0x%x\n",
			index, osd_hw.fb_gem[index].canvas_idx);
		osd_log_info("osd[%d] canvas.addr=0x%x\n",
			index, osd_hw.fb_gem[index].addr);
		osd_log_info("osd[%d] canvas.width=%d\n",
			index, osd_hw.fb_gem[index].width);
		osd_log_info("osd[%d] canvas.height=%d\n",
			index, osd_hw.fb_gem[index].height);
		osd_log_info("osd[%d] frame.width=%d\n",
			index, osd_hw.fb_gem[index].xres);
		osd_log_info("osd[%d] frame.height=%d\n",
			index, osd_hw.fb_gem[index].yres);
		osd_log_info("osd[%d] out_addr_id =0x%x\n",
			index, osd_hw.osd_afbcd[index].out_addr_id);

		if (osd_hw.osd_meson_dev.osd_ver == OSD_SIMPLE)
			osd_update_phy_addr(0);
#ifdef CONFIG_AMLOGIC_MEDIA_CANVAS
		else {
			canvas_config(osd_hw.fb_gem[index].canvas_idx,
				osd_hw.fb_gem[index].addr,
				CANVAS_ALIGNED(osd_hw.fb_gem[index].width),
				osd_hw.fb_gem[index].height,
				CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
		}
#endif
	}

	/* osd blank only control by /sys/class/graphcis/fbx/blank */
#if 0
	if (osd_hw.enable[index] == DISABLE) {
		osd_hw.enable[index] = ENABLE;
		add_to_update_list(index, OSD_ENABLE);
	}
#endif

	if (memcmp(&pan_data, &osd_hw.pandata[index],
				sizeof(struct pandata_s)) != 0 ||
	    memcmp(&disp_data, &osd_hw.dispdata[index],
		    sizeof(struct pandata_s)) != 0) {
		update_geometry = 1;
		memcpy(&osd_hw.pandata[index], &pan_data,
				sizeof(struct pandata_s));
		memcpy(&osd_hw.dispdata[index], &disp_data,
				sizeof(struct pandata_s));
		memcpy(&osd_hw.dispdata_backup[index], &disp_data,
				sizeof(struct pandata_s));
		osd_hw.src_data[index].x = osd_hw.pandata[index].x_start;
		osd_hw.src_data[index].y = osd_hw.pandata[index].y_start;
		osd_hw.src_data[index].w = osd_hw.pandata[index].x_end
			- osd_hw.pandata[index].x_start + 1;
		osd_hw.src_data[index].h = osd_hw.pandata[index].y_end
			- osd_hw.pandata[index].y_start + 1;
	}
	spin_lock_irqsave(&osd_lock, lock_flags);
	if (update_color_mode)
		osd_hw.reg[OSD_COLOR_MODE].update_func(index);
	if (update_geometry)
		osd_hw.reg[DISP_GEOMETRY].update_func(index);
	spin_unlock_irqrestore(&osd_lock, lock_flags);

	if (osd_hw.antiflicker_mode)
		osd_antiflicker_update_pan(yoffset, yres);
	if (osd_hw.clone[index])
		osd_clone_pan(index, yoffset, 0);
#ifdef CONFIG_AMLOGIC_MEDIA_FB_EXT
	osd_ext_clone_pan(index);
#endif
	osd_wait_vsync_hw(index);
}

void osd_setpal_hw(u32 index,
		   unsigned int regno,
		   unsigned int red,
		   unsigned int green,
		   unsigned int blue,
		   unsigned int transp
		  )
{
	if ((osd_hw.osd_meson_dev.has_lut)
		/*&& (index == OSD2)*/) {
		if (regno < 256) {
			u32 pal;

			pal = ((red   & 0xff) << 24) |
			      ((green & 0xff) << 16) |
			      ((blue  & 0xff) <<  8) |
			      (transp & 0xff);
			spin_lock_irqsave(&osd_lock, lock_flags);
			VSYNCOSD_WR_MPEG_REG(
				hw_osd_reg_array[index].osd_color_addr,
				regno);
			VSYNCOSD_WR_MPEG_REG(
				hw_osd_reg_array[index].osd_color, pal);
			spin_unlock_irqrestore(&osd_lock, lock_flags);
		}
	}
}

void osd_get_order_hw(u32 index, u32 *order)
{
	*order = osd_hw.order[index] & 0x3;
}

void osd_set_order_hw(u32 index, u32 order)
{
	#if 0
	if ((order != OSD_ORDER_01) && (order != OSD_ORDER_10))
		return;
	#endif
	osd_hw.order[index] = order;
	//add_to_update_list(index, OSD_CHANGE_ORDER);
	//osd_wait_vsync_hw();
}

/* osd free scale mode */
static void osd_set_free_scale_enable_mode1(u32 index, u32 enable)
{
	unsigned int h_enable = 0;
	unsigned int v_enable = 0;
	int ret = 0;

	if (osd_hw.osd_meson_dev.osd_ver == OSD_SIMPLE)
		return;
	h_enable = (enable & 0xffff0000 ? 1 : 0);
	v_enable = (enable & 0xffff ? 1 : 0);
	osd_hw.free_scale[index].h_enable = h_enable;
	osd_hw.free_scale[index].v_enable = v_enable;
	osd_hw.free_scale_backup[index].h_enable = h_enable;
	osd_hw.free_scale_backup[index].v_enable = v_enable;
	osd_hw.free_scale_enable[index] = enable;
	osd_hw.free_scale_enable_backup[index] = enable;
	if (osd_hw.free_scale_enable[index]) {
		ret = osd_set_scan_mode(index);
		spin_lock_irqsave(&osd_lock, lock_flags);
		if (ret)
			osd_hw.reg[OSD_COLOR_MODE].update_func(index);
		osd_hw.reg[OSD_FREESCALE_COEF].update_func(index);
		osd_hw.reg[DISP_GEOMETRY].update_func(index);
		osd_hw.reg[DISP_FREESCALE_ENABLE].update_func(index);
		osd_hw.reg[OSD_ENABLE].update_func(index);
		spin_unlock_irqrestore(&osd_lock, lock_flags);
	} else {
		ret = osd_set_scan_mode(index);
		spin_lock_irqsave(&osd_lock, lock_flags);
		if (ret)
			osd_hw.reg[OSD_COLOR_MODE].update_func(index);
		osd_hw.reg[DISP_GEOMETRY].update_func(index);
		osd_hw.reg[DISP_FREESCALE_ENABLE].update_func(index);
		osd_hw.reg[OSD_ENABLE].update_func(index);
		spin_unlock_irqrestore(&osd_lock, lock_flags);
	}
	osd_wait_vsync_hw(index);
}

void osd_set_free_scale_enable_hw(u32 index, u32 enable)
{
	if (osd_hw.free_scale_mode[index]
		&& (osd_hw.osd_meson_dev.has_viu2 ?
			((index != osd_hw.osd_meson_dev.viu2_index) ?
				1 : 0) : 1)) {
		osd_set_free_scale_enable_mode1(index, enable);
		if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL) {
			u32 height_dst, height_src;

			height_dst = osd_hw.free_dst_data[index].y_end -
				osd_hw.free_dst_data[index].y_start + 1;
			height_src = osd_hw.free_src_data[index].y_end -
				osd_hw.free_src_data[index].y_start + 1;
			if (height_dst != height_src &&
				osd_hw.free_dst_data[index].y_end <
						osd_hw.vinfo_height[index] - 1)
				osd_set_dummy_data(index, 0);
			else
				osd_set_dummy_data(index, 0xff);
		}
	} else if (enable)
		osd_log_info(
			"osd[%d] free_scale_enable_hw mode is error %d\n",
			index, osd_hw.free_scale_mode[index]);
}

void osd_get_free_scale_enable_hw(u32 index, u32 *free_scale_enable)
{
	*free_scale_enable = osd_hw.free_scale_enable[index];
}

void osd_set_free_scale_mode_hw(u32 index, u32 freescale_mode)
{
	osd_hw.free_scale_mode[index] = freescale_mode;
	osd_hw.free_scale_mode_backup[index] = freescale_mode;
}

void osd_get_free_scale_mode_hw(u32 index, u32 *freescale_mode)
{
	*freescale_mode = osd_hw.free_scale_mode[index];
}

void osd_set_4k2k_fb_mode_hw(u32 fb_for_4k2k)
{
	osd_hw.fb_for_4k2k = fb_for_4k2k;
}

void osd_get_free_scale_width_hw(u32 index, u32 *free_scale_width)
{
	*free_scale_width = osd_hw.free_src_data_backup[index].x_end -
		osd_hw.free_src_data_backup[index].x_start + 1;
}

void osd_get_free_scale_height_hw(u32 index, u32 *free_scale_height)
{
	*free_scale_height = osd_hw.free_src_data_backup[index].y_end -
		osd_hw.free_src_data_backup[index].y_start + 1;
}

void osd_get_free_scale_axis_hw(u32 index, s32 *x0, s32 *y0, s32 *x1, s32 *y1)
{
	*x0 = osd_hw.free_src_data_backup[index].x_start;
	*y0 = osd_hw.free_src_data_backup[index].y_start;
	*x1 = osd_hw.free_src_data_backup[index].x_end;
	*y1 = osd_hw.free_src_data_backup[index].y_end;
}

void osd_set_free_scale_axis_hw(u32 index, s32 x0, s32 y0, s32 x1, s32 y1)
{
	osd_hw.free_src_data[index].x_start = x0;
	osd_hw.free_src_data[index].y_start = y0;
	osd_hw.free_src_data[index].x_end = x1;
	osd_hw.free_src_data[index].y_end = y1;
	osd_hw.free_src_data_backup[index].x_start = x0;
	osd_hw.free_src_data_backup[index].y_start = y0;
	osd_hw.free_src_data_backup[index].x_end = x1;
	osd_hw.free_src_data_backup[index].y_end = y1;
	osd_hw.src_data[index].x = x0;
	osd_hw.src_data[index].y = y0;
	osd_hw.src_data[index].w = x1 - x0 + 1;
	osd_hw.src_data[index].h = y1 - y0 + 1;
}

void osd_get_scale_axis_hw(u32 index, s32 *x0, s32 *y0, s32 *x1, s32 *y1)
{
	*x0 = osd_hw.scaledata[index].x_start;
	*x1 = osd_hw.scaledata[index].x_end;
	*y0 = osd_hw.scaledata[index].y_start;
	*y1 = osd_hw.scaledata[index].y_end;
}

void osd_set_scale_axis_hw(u32 index, s32 x0, s32 y0, s32 x1, s32 y1)
{
	osd_hw.scaledata[index].x_start = x0;
	osd_hw.scaledata[index].x_end = x1;
	osd_hw.scaledata[index].y_start = y0;
	osd_hw.scaledata[index].y_end = y1;
}

void osd_get_window_axis_hw(u32 index, s32 *x0, s32 *y0, s32 *x1, s32 *y1)
{
	struct vinfo_s *vinfo = NULL;
	s32 height;
	u32 output_index;

	output_index = get_output_device_id(index);
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		if (output_index == VIU1)
			vinfo = get_current_vinfo();
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
		else if (output_index == VIU2)
			vinfo = get_current_vinfo2();
#endif
		if (vinfo && (strcmp(vinfo->name, "invalid") &&
			strcmp(vinfo->name, "null"))) {
			if (is_interlaced(vinfo)) {
				height = osd_hw.free_dst_data_backup[index]
					.y_end -
					osd_hw.free_dst_data_backup[index]
					.y_start + 1;
				height *= 2;
				*y0 = osd_hw.free_dst_data_backup[index]
					.y_start * 2;
				*y1 = height + *y0 - 1;
			} else {
				*y0 = osd_hw.free_dst_data_backup[index]
					.y_start;
				*y1 = osd_hw.free_dst_data_backup[index]
					.y_end;
			}
		} else {
			*y0 = osd_hw.free_dst_data_backup[index].y_start;
			*y1 = osd_hw.free_dst_data_backup[index].y_end;
		}
		*x0 = osd_hw.free_dst_data_backup[index].x_start;
		*x1 = osd_hw.free_dst_data_backup[index].x_end;
	} else {
		*x0 = osd_hw.dst_data[index].x;
		*y0 = osd_hw.dst_data[index].y;
		*x1 = osd_hw.dst_data[index].x +
			osd_hw.dst_data[index].w - 1;
		*y1 = osd_hw.dst_data[index].y +
			osd_hw.dst_data[index].h - 1;
	}
}

void osd_set_window_axis_hw(u32 index, s32 x0, s32 y0, s32 x1, s32 y1)
{
	struct vinfo_s *vinfo = NULL;
	s32 temp_y0, temp_y1;
	u32 output_index;
	u32 height_dst, height_src;

	output_index = get_output_device_id(index);
	if (output_index == VIU1)
		vinfo = get_current_vinfo();
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	else if (output_index == VIU2)
		vinfo = get_current_vinfo2();
#endif
	mutex_lock(&osd_mutex);
	if (vinfo && (strcmp(vinfo->name, "invalid") &&
		strcmp(vinfo->name, "null"))) {
		if (is_interlaced(vinfo)) {
			temp_y0 = y0 / 2;
			temp_y1 = y1 / 2;
		} else {
			temp_y0 = y0;
			temp_y1 = y1;
		}
	} else {
		temp_y0 = y0;
		temp_y1 = y1;
	}
	osd_hw.free_dst_data[index].y_start = temp_y0;
	osd_hw.free_dst_data[index].y_end = temp_y1;
	osd_hw.free_dst_data[index].x_start = x0;
	osd_hw.free_dst_data[index].x_end = x1;
	osd_hw.free_dst_data_backup[index].y_start = temp_y0;
	osd_hw.free_dst_data_backup[index].y_end = temp_y1;
	osd_hw.free_dst_data_backup[index].x_start = x0;
	osd_hw.free_dst_data_backup[index].x_end = x1;
	if (osd_hw.hw_cursor_en) {
		osd_hw.cursor_dispdata[index].x_start = x0;
		osd_hw.cursor_dispdata[index].x_end = x1;
		osd_hw.cursor_dispdata[index].y_start = temp_y0;
		osd_hw.cursor_dispdata[index].y_end = temp_y1;
	}
	osd_hw.dst_data[index].x = x0;
	osd_hw.dst_data[index].y = y0;
	osd_hw.dst_data[index].w = x1 - x0 + 1;
	osd_hw.dst_data[index].h = y1 - y0 + 1;

	height_dst = osd_hw.free_dst_data[index].y_end -
		osd_hw.free_dst_data[index].y_start + 1;
	height_src = osd_hw.free_src_data[index].y_end -
		osd_hw.free_src_data[index].y_start + 1;

	if (height_dst != height_src)
		osd_set_dummy_data(index, 0);
	else
		osd_set_dummy_data(index, 0xff);

	if (osd_hw.free_dst_data[index].y_end >= osd_hw.vinfo_height[index] - 1)
		osd_set_dummy_data(index, 0xff);
	osd_update_window_axis = true;
	if (osd_hw.hwc_enable[output_index] &&
	    (osd_hw.osd_display_debug[output_index] == OSD_DISP_DEBUG))
		osd_setting_blend(output_index);
	mutex_unlock(&osd_mutex);
}

s32 osd_get_position_from_reg(
	u32 index,
	s32 *src_x_start, s32 *src_x_end,
	s32 *src_y_start, s32 *src_y_end,
	s32 *dst_x_start, s32 *dst_x_end,
	s32 *dst_y_start, s32 *dst_y_end)
{
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];
	u32 data32 = 0x0;

	if (index >= OSD_MAX)
		return -1;

	if (!src_x_start || !src_x_end || !src_y_start || !src_y_end
		|| !dst_y_start || !dst_x_end || !dst_y_start || !dst_y_end)
		return -1;

	data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w1);
	*src_x_start = data32 & 0x1fff;
	*src_x_end = (data32 >> 16) & 0x1fff;

	data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w2);
	*src_y_start = data32 & 0x1fff;
	*src_y_end = (data32 >> 16) & 0x1fff;

	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		data32 = osd_reg_read(osd_reg->osd_sc_ctrl0);
		if ((data32 & 0xc) == 0xc) {
			data32 = osd_reg_read(osd_reg->osd_sco_h_start_end);
			*dst_x_end = data32 & 0x1fff;
			*dst_x_start = (data32 >> 16) & 0x1fff;

			data32 = osd_reg_read(osd_reg->osd_sco_v_start_end);
			*dst_y_end = data32 & 0x1fff;
			*dst_y_start = (data32 >> 16) & 0x1fff;
		} else {
			data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w3);
			*dst_x_start = data32 & 0x1fff;
			*dst_x_end = (data32 >> 16) & 0x1fff;

			data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w4);
			*dst_y_start = data32 & 0x1fff;
			*dst_y_end = (data32 >> 16) & 0x1fff;
		}
	} else if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL) {
		s32 scaler_index = -1;

		osd_reg = &hw_osd_reg_array[0];
		data32 = osd_reg_read(osd_reg->osd_sc_ctrl0);

		if ((data32 & 3) == 0)
			scaler_index = 0;
		else if ((data32 & 3) == 1)
			scaler_index = 1;

		if (((data32 & 0xc) == 0xc) && ((u32)scaler_index == index)) {
			data32 = osd_reg_read(osd_reg->osd_sco_h_start_end);
			*dst_x_end = data32 & 0x1fff;
			*dst_x_start = (data32 >> 16) & 0x1fff;

			data32 = osd_reg_read(osd_reg->osd_sco_v_start_end);
			*dst_y_end = data32 & 0x1fff;
			*dst_y_start = (data32 >> 16) & 0x1fff;
		} else {
			osd_reg = &hw_osd_reg_array[index];
			data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w3);
			*dst_x_start = data32 & 0x1fff;
			*dst_x_end = (data32 >> 16) & 0x1fff;

			data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w4);
			*dst_y_start = data32 & 0x1fff;
			*dst_y_end = (data32 >> 16) & 0x1fff;
		}
	} else if (osd_hw.osd_meson_dev.osd_ver == OSD_SIMPLE) {
		osd_reg = &hw_osd_reg_array[0];
		data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w3);
		*dst_x_start = data32 & 0x1fff;
		*dst_x_end = (data32 >> 16) & 0x1fff;

		data32 = osd_reg_read(osd_reg->osd_blk0_cfg_w4);
		*dst_y_start = data32 & 0x1fff;
		*dst_y_end = (data32 >> 16) & 0x1fff;
	}
	return 0;
}

void osd_get_block_windows_hw(u32 index, u32 *windows)
{
	/*
	 * memcpy(windows, osd_hw.block_windows[index],
	 *      sizeof(osd_hw.block_windows[index]));
	 */
}

void osd_set_block_windows_hw(u32 index, u32 *windows)
{
	/*
	 * memcpy(osd_hw.block_windows[index], windows,
	 *      sizeof(osd_hw.block_windows[index]));
	 */
	add_to_update_list(index, DISP_GEOMETRY);
	osd_wait_vsync_hw(index);
}

void osd_get_block_mode_hw(u32 index, u32 *mode)
{
	*mode = osd_hw.block_mode[index];
}

void osd_set_block_mode_hw(u32 index, u32 mode)
{
	/* osd_hw.block_mode[index] = mode; */
	add_to_update_list(index, DISP_GEOMETRY);
	osd_wait_vsync_hw(index);
}

void osd_enable_3d_mode_hw(u32 index, u32 enable)
{
	spin_lock_irqsave(&osd_lock, lock_flags);
	osd_hw.mode_3d[index].enable = enable;
	spin_unlock_irqrestore(&osd_lock, lock_flags);
	if (enable) {
		/* when disable 3d mode ,we should return to stardard state. */
		osd_hw.mode_3d[index].left_right = OSD_LEFT;
		osd_hw.mode_3d[index].l_start = osd_hw.pandata[index].x_start;
		osd_hw.mode_3d[index].l_end = (osd_hw.pandata[index].x_end +
					osd_hw.pandata[index].x_start) >> 1;
		osd_hw.mode_3d[index].r_start = osd_hw.mode_3d[index].l_end + 1;
		osd_hw.mode_3d[index].r_end = osd_hw.pandata[index].x_end;
		osd_hw.mode_3d[index].origin_scale.h_enable =
					osd_hw.scale[index].h_enable;
		osd_hw.mode_3d[index].origin_scale.v_enable =
					osd_hw.scale[index].v_enable;
		osd_set_2x_scale_hw(index, 1, 0);
	} else {
		osd_set_2x_scale_hw(index,
				osd_hw.mode_3d[index].origin_scale.h_enable,
				osd_hw.mode_3d[index].origin_scale.v_enable);
	}
}

void osd_enable_hw(u32 index, u32 enable)
{
	int i = 0;
	int count = (pxp_mode == 1)?3:WAIT_AFBC_READY_COUNT;
	u32 output_index;

	if (index == 0) {
		osd_log_info("osd[%d] enable: %d (%s)\n",
				index, enable, current->comm);
	} else {
		osd_log_info("osd[%d] enable: %d (%s)\n",
				index, enable, current->comm);
	}

	/* reset viu 31bit ?? */
	if (!osd_hw.enable[index] &&
			osd_hw.osd_afbcd[index].enable && enable &&
			(get_osd_hwc_type(index) != OSD_G12A_NEW_HWC)) {
		spin_lock_irqsave(&osd_lock, lock_flags);
		if (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) {
			osd_reg_write(VIU_SW_RESET, 0x80000000);
			osd_reg_write(VIU_SW_RESET, 0);
		}
		//else
		//	osd_reg_write(VIU_SW_RESET, 0x200000);

		spin_unlock_irqrestore(&osd_lock, lock_flags);
		osd_afbc_dec_enable = 0;
		add_to_update_list(index, OSD_COLOR_MODE);
		add_to_update_list(index, OSD_GBL_ALPHA);
		add_to_update_list(index, DISP_GEOMETRY);
		osd_wait_vsync_hw(index);

		while ((index == 0) && osd_hw.osd_afbcd[index].enable &&
			(osd_hw.osd_afbcd[index].phy_addr == 0) &&
			enable && (i < count)) {
			osd_wait_vsync_hw(index);
			i++;
		}
		if (i > 0)
			osd_log_info("osd[%d]: wait %d vsync first buffer ready.\n",
				index, i);
	}

	osd_hw.enable[index] = enable;
	output_index = get_output_device_id(index);
	if (get_osd_hwc_type(index) != OSD_G12A_NEW_HWC) {
		add_to_update_list(index, OSD_ENABLE);
		osd_wait_vsync_hw(index);
	} else if (osd_hw.hwc_enable[output_index] &&
		osd_hw.osd_display_debug[output_index])
		osd_setting_blend(output_index);
}

void osd_set_2x_scale_hw(u32 index, u16 h_scale_enable, u16 v_scale_enable)
{
	osd_log_info("osd[%d] set scale, h_scale: %s, v_scale: %s\n",
		     index, h_scale_enable ? "ENABLE" : "DISABLE",
		     v_scale_enable ? "ENABLE" : "DISABLE");
	osd_log_info("osd[%d].scaledata: %d %d %d %d\n",
		     index,
		     osd_hw.scaledata[index].x_start,
		     osd_hw.scaledata[index].x_end,
		     osd_hw.scaledata[index].y_start,
		     osd_hw.scaledata[index].y_end);
	osd_log_info("osd[%d].pandata: %d %d %d %d\n",
		     index,
		     osd_hw.pandata[index].x_start,
		     osd_hw.pandata[index].x_end,
		     osd_hw.pandata[index].y_start,
		     osd_hw.pandata[index].y_end);
	osd_hw.scale[index].h_enable = h_scale_enable;
	osd_hw.scale[index].v_enable = v_scale_enable;
	spin_lock_irqsave(&osd_lock, lock_flags);
	osd_hw.reg[DISP_SCALE_ENABLE].update_func(index);
	osd_hw.reg[DISP_GEOMETRY].update_func(index);
	spin_unlock_irqrestore(&osd_lock, lock_flags);
	osd_wait_vsync_hw(index);
}

void osd_get_flush_rate_hw(u32 index, u32 *break_rate)
{
	const struct vinfo_s *vinfo = NULL;
	u32 output_index;

	output_index = get_output_device_id(index);
	if (output_index == VIU1)
		vinfo = get_current_vinfo();
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	else if (output_index == VIU2)
		vinfo = get_current_vinfo2();
#endif
	if (!vinfo)
		return;
	*break_rate = vinfo->sync_duration_num / vinfo->sync_duration_den;
}

void osd_set_antiflicker_hw(u32 index, struct vinfo_s *vinfo, u32 yres)
{
#ifdef NEED_ANTIFLICKER
	bool osd_need_antiflicker = false;

	if (is_interlaced(vinfo))
		osd_need_antiflicker = false;
	else
		osd_need_antiflicker = true;
	if (osd_need_antiflicker) {
		osd_hw.antiflicker_mode = 1;
		osd_antiflicker_task_start();
		osd_antiflicker_enable(1);
		osd_antiflicker_update_pan(osd_hw.pandata[index].y_start, yres);
	}
#else
	if (osd_hw.antiflicker_mode)
		osd_antiflicker_task_stop();
	osd_hw.antiflicker_mode = 0;
#endif
}

void osd_get_antiflicker_hw(u32 index, u32 *on_off)
{
	*on_off = osd_hw.antiflicker_mode;
}

/* Todo: how to expand to 3 osd */
void osd_clone_pan(u32 index, u32 yoffset, int debug_flag)
{
	s32 offset = 0;
	u32 index_buffer = 0;
	s32 osd0_buffer_number = 0;
	s32 height_osd1 = 0;

	if (yoffset != 0) {
		index_buffer = osd_hw.fb_gem[index].height / yoffset;
		if (index_buffer == 3)
			osd0_buffer_number = 1;
		else if (index_buffer == 1)
			osd0_buffer_number = 2;
	} else
		osd0_buffer_number = 0;
	osd_clone_get_virtual_yres(&height_osd1);
	if (osd_hw.clone[index]) {
		offset = osd0_buffer_number * height_osd1;
		osd_hw.pandata[OSD2].y_start = offset;
		osd_hw.pandata[OSD2].y_end = offset + height_osd1 - 1;
		if (osd_hw.angle[OSD2]) {
			if (debug_flag)
				osd_log_dbg(MODULE_BASE, "++ osd_clone_pan start when enable clone\n");
			osd_clone_update_pan(osd0_buffer_number);
		}
		add_to_update_list(OSD2, DISP_GEOMETRY);
	}
}

/* Todo: how to expand to 3 osd */
void osd_set_angle_hw(u32 index, u32 angle, u32  virtual_osd1_yres,
		      u32 virtual_osd2_yres)
{
#ifndef OSD_GE2D_CLONE_SUPPORT
	osd_log_err("++ osd_clone depends on GE2D module!\n");
	return;
#endif
	if (angle > 4) {
		osd_log_err("++ invalid angle: %d\n", angle);
		return;
	}
	osd_log_info("++ virtual_osd1_yres is %d, virtual_osd2_yres is %d!\n",
		     virtual_osd1_yres, virtual_osd2_yres);
	osd_clone_set_virtual_yres(virtual_osd1_yres, virtual_osd2_yres);
	if (osd_hw.clone[index] == 0) {
		osd_log_info("++ set osd[%d]->angle: %d->%d\n",
				index, osd_hw.angle[index], angle);
		osd_clone_set_angle(angle);
		osd_hw.angle[index] = angle;
	} else if (!((osd_hw.angle[index] == 0) || (angle == 0))) {
		osd_log_info("++ set osd[%d]->angle: %d->%d\n",
				index, osd_hw.angle[index], angle);
		osd_clone_set_angle(angle);
		osd_hw.angle[index] = angle;
		osd_clone_pan(index, osd_hw.pandata[OSD1].y_start, 1);
	}
}

void osd_get_angle_hw(u32 index, u32 *angle)
{
	*angle = osd_hw.angle[index];
}

void osd_set_clone_hw(u32 index, u32 clone)
{
	int ret = -1;

	osd_log_info("++ set osd[%d]->clone: %d->%d\n",
			index, osd_hw.clone[index], clone);
	osd_hw.clone[index] = clone;
	if (osd_hw.clone[index]) {
		if (osd_hw.angle[index]) {
			osd_hw.color_info[index] = osd_hw.color_info[OSD1];
			osd_hw.color_backup[index] = osd_hw.color_info[OSD1];
			ret = osd_clone_task_start();
			if (ret)
				osd_clone_pan(index,
					osd_hw.pandata[OSD1].y_start, 1);
			else
				osd_log_err("++ start clone error\n");
		}
	} else {
		if (osd_hw.angle[index])
			osd_clone_task_stop();
	}
	add_to_update_list(index, OSD_COLOR_MODE);
}

void osd_set_update_pan_hw(u32 index)
{
	osd_clone_pan(index, osd_hw.pandata[OSD1].y_start, 1);
}

void osd_get_clone_hw(u32 index, u32 *clone)
{
	*clone = osd_hw.clone[index];
}

void osd_set_reverse_hw(u32 index, u32 reverse, u32 update)
{
	char *str[4] = {"NONE", "ALL", "X_REV", "Y_REV"};

	osd_hw.osd_reverse[index] = reverse;
	pr_info("set osd%d reverse as %s\n", index, str[reverse]);
	if (update) {
		add_to_update_list(index, DISP_OSD_REVERSE);
		osd_wait_vsync_hw(index);
	}
}

void osd_get_reverse_hw(u32 index, u32 *reverse)
{
	*reverse = osd_hw.osd_reverse[index];
}

/* Todo: how to do with uboot logo */
void osd_switch_free_scale(
	u32 pre_index, u32 pre_enable, u32 pre_scale,
	u32 next_index, u32 next_enable, u32 next_scale)
{
	unsigned int h_enable = 0;
	unsigned int v_enable = 0;
	int i = 0;
	int count = (pxp_mode == 1)?3:WAIT_AFBC_READY_COUNT;

	osd_log_info("osd[%d] enable: %d scale:0x%x (%s)\n",
		pre_index, pre_enable, pre_scale, current->comm);
	osd_log_info("osd[%d] enable: %d scale:0x%x (%s)\n",
		next_index, next_enable, next_scale, current->comm);
	if (osd_hw.free_scale_mode[pre_index]
		|| osd_hw.free_scale_mode[next_index]) {
		if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL) {
			while ((next_index == OSD1)
				&& osd_hw.osd_afbcd[next_index].enable
				&& (osd_hw.osd_afbcd[next_index].phy_addr == 0)
				&& next_enable && (i < count)) {
				osd_wait_vsync_hw(OSD1);
				i++;
			}
			if (i > 0)
				osd_log_info("osd[%d]: wait %d vsync first buffer ready.\n",
					next_index, i);
		}
		if (pre_index != next_index) {
			h_enable = (pre_scale & 0xffff0000 ? 1 : 0);
			v_enable = (pre_scale & 0xffff ? 1 : 0);
			osd_hw.free_scale[pre_index].h_enable = h_enable;
			osd_hw.free_scale[pre_index].v_enable = v_enable;
			osd_hw.free_scale_enable[pre_index] = pre_scale;
			osd_hw.free_scale_backup[pre_index].h_enable = h_enable;
			osd_hw.free_scale_backup[pre_index].v_enable = v_enable;
			osd_hw.free_scale_enable_backup[pre_index] = pre_scale;
			osd_hw.enable[pre_index] = pre_enable;
		}
		h_enable = (next_scale & 0xffff0000 ? 1 : 0);
		v_enable = (next_scale & 0xffff ? 1 : 0);
		osd_hw.free_scale[next_index].h_enable = h_enable;
		osd_hw.free_scale[next_index].v_enable = v_enable;
		osd_hw.free_scale_enable[next_index] = next_scale;
		osd_hw.free_scale_backup[next_index].h_enable = h_enable;
		osd_hw.free_scale_backup[next_index].v_enable = v_enable;
		osd_hw.free_scale_enable_backup[next_index] = next_scale;
		osd_hw.enable[next_index] = next_enable;

		osd_set_scan_mode(next_index);
		spin_lock_irqsave(&osd_lock, lock_flags);
		if (next_index == OSD1
			&& osd_hw.osd_afbcd[next_index].enable
			&& next_enable) {
			if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL) {
				osd_reg_write(VIU_SW_RESET, 0x80000000);
				osd_reg_write(VIU_SW_RESET, 0);
			}
			osd_afbc_dec_enable = 0;
			osd_hw.reg[OSD_GBL_ALPHA].update_func(next_index);
		}
		if (pre_index != next_index) {
			osd_hw.reg[OSD_COLOR_MODE].update_func(pre_index);
			if (pre_scale)
				osd_hw.reg[OSD_FREESCALE_COEF].
						update_func(pre_index);
			osd_hw.reg[DISP_GEOMETRY].update_func(pre_index);
			osd_hw.reg[DISP_FREESCALE_ENABLE].
						update_func(pre_index);
			osd_hw.reg[OSD_ENABLE].update_func(pre_index);

			osd_hw.reg[OSD_COLOR_MODE].update_func(next_index);
			if (next_scale)
				osd_hw.reg[OSD_FREESCALE_COEF].
				update_func(next_index);
			osd_hw.reg[DISP_GEOMETRY].update_func(next_index);
			osd_hw.reg[DISP_FREESCALE_ENABLE].
				update_func(next_index);
			osd_hw.reg[OSD_ENABLE].update_func(next_index);
		}
		spin_unlock_irqrestore(&osd_lock, lock_flags);
		osd_wait_vsync_hw(next_index);
	} else {
		if (pre_index != next_index)
			osd_enable_hw(pre_index, pre_enable);
		osd_enable_hw(next_index, next_enable);
	}
}

void osd_get_urgent(u32 index, u32 *urgent)
{
	*urgent = osd_hw.urgent[index];
}

void osd_set_urgent(u32 index, u32 urgent)
{
	osd_hw.urgent[index] = urgent;
	add_to_update_list(index, OSD_FIFO);
	osd_wait_vsync_hw(index);
}

void osd_get_deband(u32 *osd_deband_enable)
{
	*osd_deband_enable = osd_hw.osd_deband_enable;
}

void osd_set_deband(u32 osd_deband_enable)
{
	u32 data32;

	if (osd_hw.osd_meson_dev.has_deband) {
		if (osd_hw.free_scale_enable[OSD1]
			|| osd_hw.free_scale_enable[OSD2]
			|| osd_hw.free_scale_enable[OSD3]) {
			osd_hw.osd_deband_enable = osd_deband_enable;

			spin_lock_irqsave(&osd_lock, lock_flags);

			data32 = VSYNCOSD_RD_MPEG_REG(OSD_DB_FLT_CTRL);
			if (osd_deband_enable) {
				/* Bit 23	 debanding registers of side
				 * lines, [0] for luma
				 * Bit 22	 debanding registers of side
				 * lines, [1] for chroma
				 * Bit 5debanding registers,for luma
				 * Bit 4debanding registers,for chroma
				 */
				data32 |= 3 << 4;
				data32 |= 3 << 22;
			} else {
				data32 |= 0 << 4;
				data32 |= 0 << 22;
			}
			VSYNCOSD_WR_MPEG_REG(OSD_DB_FLT_CTRL, data32);
			spin_unlock_irqrestore(&osd_lock, lock_flags);
			osd_wait_vsync_hw_viu1();
		}
	}
}


void osd_get_fps(u32 index, u32 *osd_fps)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	*osd_fps = osd_hw.osd_fps[output_index];

}

void osd_set_fps(u32 index, u32 osd_fps_start)
{
	static int stime, etime;
	u32 output_index;

	output_index = get_output_device_id(index);
	osd_hw.osd_fps_start[output_index] = osd_fps_start;
	if (osd_fps_start) {
		/* start to calc fps */
		stime = ktime_to_us(ktime_get());
		osd_hw.osd_fps[output_index] = 0;
	} else {
		/* stop to calc fps */
		etime = ktime_to_us(ktime_get());
		osd_hw.osd_fps[output_index] =
			(osd_hw.osd_fps[output_index] * 1000000)
			/ (etime - stime);
		osd_log_info("osd fps:=%d\n", osd_hw.osd_fps[output_index]);
	}
}

void osd_get_display_debug(u32 index, u32 *osd_display_debug_enable)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	*osd_display_debug_enable = osd_hw.osd_display_debug[output_index];
}

void osd_set_display_debug(u32 index, u32 osd_display_debug_enable)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	osd_hw.osd_display_debug[output_index] = osd_display_debug_enable;
}

void osd_get_background_size(u32 index, struct display_flip_info_s *disp_info)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	memcpy(disp_info, &osd_hw.disp_info[output_index],
	       sizeof(struct display_flip_info_s));
}

void osd_set_background_size(u32 index, struct display_flip_info_s *disp_info)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	memcpy(&osd_hw.disp_info[output_index], disp_info,
	       sizeof(struct display_flip_info_s));
}

void osd_get_hdr_used(u32 *val)
{
	*val = osd_hw.hdr_used;
}

void osd_set_hdr_used(u32 val)
{
	osd_hw.hdr_used = val;
}

void osd_get_afbc_format(u32 index, u32 *format, u32 *inter_format)
{
	*format = osd_hw.osd_afbcd[index].format;
	*inter_format = osd_hw.osd_afbcd[index].inter_format;
}

void osd_set_afbc_format(u32 index, u32 format, u32 inter_format)
{
	osd_hw.osd_afbcd[index].format = format;
	osd_hw.osd_afbcd[index].inter_format = inter_format;
}

void osd_get_hwc_enable(u32 index, u32 *hwc_enable)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	*hwc_enable = osd_hw.hwc_enable[output_index];
}

void osd_set_hwc_enable(u32 index, u32 hwc_enable)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	osd_hw.hwc_enable[output_index] = hwc_enable;
	/* setting default hwc path */
	if (!hwc_enable)
		osd_setting_blend(OSD1);
}

void osd_do_hwc(u32 index)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	osd_setting_blend(output_index);
}

static void osd_set_two_ports(bool set)
{
	static u32 data32[2];

	if (set) {
		data32[0] = osd_reg_read(VPP_RDARB_MODE);
		data32[1] = osd_reg_read(VPU_RDARB_MODE_L2C1);
		osd_reg_set_bits(VPP_RDARB_MODE, 2, 20, 8);
		osd_reg_set_bits(VPU_RDARB_MODE_L2C1, 2, 16, 8);
	} else {
		osd_reg_write(VPP_RDARB_MODE, data32[0]);
		osd_reg_write(VPU_RDARB_MODE_L2C1, data32[1]);
	}
}

static void osd_set_basic_urgent(bool set)
{
	if (set)
		osd_reg_write(0x27c2, 0xffff);
	else
		osd_reg_write(0x27c2, 0x0);
}

void osd_get_urgent_info(u32 *ports, u32 *basic_urgent)
{
	*basic_urgent = osd_hw.basic_urgent;
	*ports = osd_hw.two_ports;
}

void osd_set_urgent_info(u32 ports, u32 basic_urgent)
{
	osd_hw.basic_urgent = basic_urgent;
	osd_hw.two_ports = ports;
	osd_set_basic_urgent(osd_hw.basic_urgent);
	osd_set_two_ports(osd_hw.two_ports);
}

void osd_set_single_step_mode(u32 index, u32 osd_single_step_mode)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	if (output_index != VIU1)
		return;
	osd_hw.osd_debug.osd_single_step_mode = osd_single_step_mode;
	if ((osd_hw.osd_debug.wait_fence_release) &&
		(osd_hw.osd_debug.osd_single_step_mode == 0)) {
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
		osd_timeline_increase(output_index);
#endif
		osd_hw.osd_debug.wait_fence_release = false;
	}

}

void osd_set_single_step(u32 index, u32 osd_single_step)
{
	u32 output_index;

	output_index = get_output_device_id(index);
	if (output_index != VIU1)
		return;
	osd_hw.osd_debug.osd_single_step = osd_single_step;
	if ((osd_hw.osd_debug.wait_fence_release) &&
		(osd_hw.osd_debug.osd_single_step > 0)) {
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
		osd_timeline_increase(output_index);
#endif
		osd_hw.osd_debug.wait_fence_release = false;
	}
}

void osd_get_rotate(u32 index, u32 *osd_rotate)
{
	*osd_rotate = osd_hw.osd_rotate[index];
}

void osd_set_rotate(u32 index, u32 osd_rotate)
{
	if (index != osd_hw.osd_meson_dev.viu2_index)
		osd_log_err("osd%d not support rotate\n", index);
	osd_hw.osd_rotate[index] = osd_rotate;
	add_to_update_list(index, DISP_OSD_ROTATE);
	osd_wait_vsync_hw(index);
}

void osd_get_afbc_err_cnt(u32 *err_cnt)
{
	*err_cnt = osd_hw.afbc_err_cnt;
}

void osd_get_dimm_info(u32 index, u32 *osd_dimm_layer, u32 *osd_dimm_color)
{
	*osd_dimm_layer = osd_hw.dim_layer[index];
	*osd_dimm_color = osd_hw.dim_color[index];
}

void osd_set_dimm_info(u32 index, u32 osd_dimm_layer, u32 osd_dimm_color)
{
	osd_hw.dim_layer[index] = osd_dimm_layer;
	osd_hw.dim_color[index] = osd_dimm_color;
}

u32 osd_get_hold_line(u32 index)
{
	unsigned int data32 = 0, val = 0;

	if (osd_hw.powered[index]) {
		data32 = VSYNCOSD_RD_MPEG_REG
			(hw_osd_reg_array[index].osd_fifo_ctrl_stat);
		val = (data32 >> 5) & 0x1f;
	}
	return val;
}

void osd_set_hold_line(u32 index, int hold_line)
{
	unsigned int data32 = 0, val = 0;

	if (osd_hw.powered[index]) {
		data32 = VSYNCOSD_RD_MPEG_REG
			(hw_osd_reg_array[index].osd_fifo_ctrl_stat);
		val = (data32 >> 5) & 0x1f;
		if (val != hold_line) {
			VSYNCOSD_WR_MPEG_REG_BITS
				(hw_osd_reg_array[index].osd_fifo_ctrl_stat,
				hold_line & 0x1f, 5, 5);
		}
	}
}

int osd_get_capbility(u32 index)
{
	u32 capbility = 0;
	u32 afbc = osd_hw.osd_meson_dev.afbc_type;

	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		if (index == OSD1)
			capbility |= OSD_LAYER_ENABLE | OSD_FREESCALE
				| OSD_UBOOT_LOGO | OSD_ZORDER | OSD_VIU1
				| OSD_PRIMARY | (afbc ? OSD_AFBC : 0);
		else if (index < osd_hw.osd_meson_dev.viu1_osd_count)
			capbility |= OSD_LAYER_ENABLE | OSD_FREESCALE |
				OSD_ZORDER | OSD_VIU1 |
				(afbc ? OSD_AFBC : 0);
		else if (index == osd_hw.osd_meson_dev.viu2_index)
			capbility |= OSD_LAYER_ENABLE | OSD_VIU2;
	} else if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL) {
		if (index == OSD1)
			capbility |= OSD_LAYER_ENABLE | OSD_FREESCALE
				| OSD_VIU1 | (afbc ? OSD_AFBC : 0);
		else if (index == OSD2)
			capbility |= OSD_LAYER_ENABLE |
				OSD_HW_CURSOR | OSD_FREESCALE
				| OSD_UBOOT_LOGO | OSD_VIU1 |
				(afbc ? OSD_AFBC : 0);
	}

	return capbility;
}

static void save_blend_reg(struct layer_blend_reg_s *blend_reg)
{
	struct osd_debug_backup_s *osd_backup;
	int count = osd_hw.osd_debug.backup_count;

	osd_backup = &osd_hw.osd_debug.osd_backup[count];
	memcpy(&(osd_backup->blend_reg), blend_reg,
		sizeof(struct layer_blend_reg_s));
	osd_hw.osd_debug.backup_count++;
	if (osd_hw.osd_debug.backup_count >= OSD_BACKUP_COUNT)
		osd_hw.osd_debug.backup_count = 0;
}

static void osd_info_output(int count)
{
	struct osd_debug_backup_s *osd_backup;
	struct layer_blend_reg_s *blend_reg;
	int index;
	u32 value;

	osd_backup = &osd_hw.osd_debug.osd_backup[count];
	osd_log_info("|index|enable|ext_addr|order|blend_mode|plane_alpha|dim_layer|dim_color|src axis|dst axis\n");
	for (index = 0; index < HW_OSD_COUNT; index++) {
		if (osd_backup->layer[index].enable) {
		osd_log_info("%d\t%4d\t  0x%8x %2d\t%2d\t%2d\t%2d\t0x%x\t(%4d,%4d,%4d,%4d)\t(%4d,%4d,%4d,%4d)\n",
			index,
			osd_backup->layer[index].enable,
			osd_backup->layer[index].ext_addr,
			osd_backup->layer[index].zorder,
			osd_backup->layer[index].blend_mode,
			osd_backup->layer[index].plane_alpha,
			osd_backup->layer[index].dim_layer,
			osd_backup->layer[index].dim_color,
			osd_backup->layer[index].src_x,
			osd_backup->layer[index].src_y,
			osd_backup->layer[index].src_w,
			osd_backup->layer[index].src_h,
			osd_backup->layer[index].dst_x,
			osd_backup->layer[index].dst_h,
			osd_backup->layer[index].dst_w,
			osd_backup->layer[index].dst_h);
		}
	}

	blend_reg = &osd_backup->blend_reg;
	value = 4				  << 29|
		blend_reg->blend2_premult_en << 27|
		blend_reg->din0_byp_blend	  << 26|
		blend_reg->din2_osd_sel	  << 25|
		blend_reg->din3_osd_sel	  << 24|
		blend_reg->blend_din_en	  << 20|
		blend_reg->din_premult_en	  << 16|
		blend_reg->din_reoder_sel;
	osd_log_info("(0x39b0)=0x%x\n", value);
	for (index = 0; index < HW_OSD_COUNT; index++) {
		if (osd_hw.enable[index]) {
			osd_log_info("(0x%x)=0x%x, (0x%x)=0x%x\n",
			VIU_OSD_BLEND_DIN0_SCOPE_H + 2 * index,
			blend_reg->osd_blend_din_scope_h[index],
			VIU_OSD_BLEND_DIN0_SCOPE_V + 2 * index,
			blend_reg->osd_blend_din_scope_v[index]);
		}
	}
	/* vpp osd1 blend ctrl */
	value = (0 & 0xf) << 0 |
		(0 & 0x1) << 4 |
		(blend_reg->postbld_src3_sel & 0xf) << 8 |
		(blend_reg->postbld_osd1_premult & 0x1) << 16|
		(1 & 0x1) << 20;
	osd_log_info("(0x1dfd)=0x%x\n", value);
	osd_log_info("(0x39bb)=0x%x\n",
		blend_reg->osd_blend_blend0_size);

	/* vpp osd2 blend ctrl */
	value = (0 & 0xf) << 0 |
		(0 & 0x1) << 4 |
		(blend_reg->postbld_src4_sel & 0xf) << 8 |
		(blend_reg->postbld_osd2_premult & 0x1) << 16 |
		(1 & 0x1) << 20;
	osd_log_info("(0x1dfe)=0x%x\n", value);
	osd_log_info("(0x39bc)=0x%x\n",
		blend_reg->osd_blend_blend0_size);
	osd_log_info("(0x1df5)=0x%x, (0x1df6)=0x%x\n",
		blend_reg->vpp_osd1_blend_h_scope,
		blend_reg->vpp_osd1_blend_v_scope);
	osd_log_info("(0x1df7)=0x%x, (0x1df8)=0x%x\n",
		blend_reg->vpp_osd2_blend_h_scope,
		blend_reg->vpp_osd2_blend_v_scope);
}

void output_save_info(void)
{
	struct osd_debug_backup_s *osd_backup;
	int count = osd_hw.osd_debug.backup_count - 1;
	int i;

	osd_backup = &osd_hw.osd_debug.osd_backup[count];
	for (i = count; i >= 0; i--) {
		osd_log_info("save list %d\n", i);
		osd_info_output(i);
	}
	for (i = OSD_BACKUP_COUNT - 1; i > count; i--) {
		osd_log_info("save list %d\n", i);
		osd_info_output(i);
	}
}

#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
enum {
	HAL_PIXEL_FORMAT_RGBA_8888 = 1,
	HAL_PIXEL_FORMAT_RGBX_8888 = 2,
	HAL_PIXEL_FORMAT_RGB_888 = 3,
	HAL_PIXEL_FORMAT_RGB_565 = 4,
	HAL_PIXEL_FORMAT_BGRA_8888 = 5,
	HAL_PIXEL_FORMAT_RGBA_1010102 = 43,
	/* 0x2B */
};

static bool use_ext;
const struct color_bit_define_s extern_color_format_array[] = {
	/*32 bit color RGBA */
	{
		COLOR_INDEX_32_ABGR, 2, 5,
		0, 8, 0, 8, 8, 0, 16, 8, 0, 24, 8, 0,
		0, 32
	},
	/*32 bit color RGBX */
	{
		COLOR_INDEX_32_XBGR, 2, 5,
		0, 8, 0, 8, 8, 0, 16, 8, 0, 24, 0, 0,
		0, 32
	},
	/*24 bit color RGB */
	{
		COLOR_INDEX_24_RGB, 5, 7,
		16, 8, 0, 8, 8, 0, 0, 8, 0, 0, 0, 0,
		0, 24
	},
	/*16 bit color BGR */
	{
		COLOR_INDEX_16_565, 4, 4,
		11, 5, 0, 5, 6, 0, 0, 5, 0, 0, 0, 0,
		0, 16
	},
	/*32 bit color BGRA */
	{
		COLOR_INDEX_32_ARGB, 1, 5,
		16, 8, 0, 8, 8, 0, 0, 8, 0, 24, 8, 0,
		0, 32
	},
	/*32 bit color RGBA 1010102 for mali afbc*/
	{
		COLOR_INDEX_RGBA_1010102, 2, 9,
		0, 10, 0, 10, 10, 0, 20, 10, 0, 30, 2, 0,
		0, 32
	},
};

static void clear_backup_info(void)
{
	struct osd_debug_backup_s *osd_backup;
	int count = osd_hw.osd_debug.backup_count;
	int i;

	osd_backup = &osd_hw.osd_debug.osd_backup[count];
	for (i = 0; i < HW_OSD_COUNT; i++)
		memset(&(osd_backup->layer[i]), 0x0,
			sizeof(struct layer_info_s));
}

static void save_layer_info(struct layer_fence_map_s *layer_map)
{
	struct osd_debug_backup_s *osd_backup;
	int count = osd_hw.osd_debug.backup_count;
	u32 index = layer_map->fb_index;

	osd_backup = &osd_hw.osd_debug.osd_backup[count];
	osd_backup->layer[index].enable = layer_map->enable;
	osd_backup->layer[index].ext_addr = layer_map->ext_addr;
	osd_backup->layer[index].src_x = layer_map->src_x;
	osd_backup->layer[index].src_y = layer_map->src_y;
	osd_backup->layer[index].src_w = layer_map->src_w;
	osd_backup->layer[index].src_h = layer_map->src_h;
	osd_backup->layer[index].dst_x = layer_map->dst_x;
	osd_backup->layer[index].dst_y = layer_map->dst_y;
	osd_backup->layer[index].dst_w = layer_map->dst_w;
	osd_backup->layer[index].dst_h = layer_map->dst_h;
	osd_backup->layer[index].zorder = layer_map->zorder;
	osd_backup->layer[index].blend_mode = layer_map->blend_mode;
	osd_backup->layer[index].plane_alpha = layer_map->plane_alpha;
	osd_backup->layer[index].dim_layer = layer_map->dim_layer;
	osd_backup->layer[index].dim_color = layer_map->dim_color;
}

static const struct color_bit_define_s *convert_hal_format(u32 format)
{
	const struct color_bit_define_s *color = NULL;
	int b_mali_afbc = 0;

	b_mali_afbc = (format & AFBC_EN) >> 31;
	format &= ~AFBC_EN;
	switch (format) {
	case HAL_PIXEL_FORMAT_RGBA_8888:
	case HAL_PIXEL_FORMAT_RGBX_8888:
	case HAL_PIXEL_FORMAT_RGB_888:
	case HAL_PIXEL_FORMAT_RGB_565:
	case HAL_PIXEL_FORMAT_BGRA_8888:
		color = &extern_color_format_array[format - 1];
		break;
	case HAL_PIXEL_FORMAT_RGBA_1010102:
		if (b_mali_afbc)
			color = &extern_color_format_array[format - 38];
		break;
	}
	return color;
}

static bool osd_ge2d_compose_pan_display(struct osd_fence_map_s *fence_map)
{
	u32 index = fence_map->fb_index;
	bool free_scale_set = false;

	canvas_config(osd_hw.fb_gem[index].canvas_idx,
		fence_map->ext_addr,
		CANVAS_ALIGNED(fence_map->width *
		(osd_hw.color_info[index]->bpp >> 3)),
		fence_map->height,
		CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
	osd_hw.screen_base[index] = fence_map->ext_addr;
	osd_hw.screen_size[index] =
		CANVAS_ALIGNED(fence_map->width *
		osd_hw.color_info[index]->bpp) * fence_map->height;
	osd_hw.pandata[index].x_start = 0;
	osd_hw.pandata[index].x_end = fence_map->width - 1;
	osd_hw.pandata[index].y_start = 0;
	osd_hw.pandata[index].y_end = fence_map->height - 1;
	#if 0
	if ((osd_hw.screen_base[index] !=
		osd_hw.screen_base_backup[index])
		|| (osd_hw.screen_size[index] !=
		osd_hw.screen_size_backup[index])) {
		osd_hw.screen_base[index] = osd_hw.screen_base_backup[index];
		osd_hw.screen_size[index] = osd_hw.screen_size_backup[index];
		free_scale_set = true;
	}
	#endif
	if ((memcmp(&osd_hw.dispdata[index],
		&osd_hw.dispdata_backup[index],
		sizeof(struct pandata_s)) != 0) ||
		(memcmp(&osd_hw.free_src_data[index],
		&osd_hw.free_src_data_backup[index],
		sizeof(struct pandata_s)) != 0) ||
		(memcmp(&osd_hw.free_dst_data[index],
		&osd_hw.free_dst_data_backup[index],
		sizeof(struct pandata_s)) != 0)) {
		memcpy(&osd_hw.dispdata[index],
			&osd_hw.dispdata_backup[index],
			sizeof(struct pandata_s));
		memcpy(&osd_hw.free_src_data[index],
			&osd_hw.free_src_data_backup[index],
			sizeof(struct pandata_s));
		memcpy(&osd_hw.free_dst_data[index],
			&osd_hw.free_dst_data_backup[index],
			sizeof(struct pandata_s));
		free_scale_set = true;
	}

	if ((osd_hw.free_scale[index].h_enable !=
		osd_hw.free_scale_backup[index].h_enable) ||
		(osd_hw.free_scale[index].v_enable !=
		osd_hw.free_scale_backup[index].v_enable) ||
		(osd_hw.free_scale_enable[index] !=
		osd_hw.free_scale_enable_backup[index]) ||
		(osd_hw.free_scale_mode[index] !=
		osd_hw.free_scale_mode_backup[index])) {
		osd_hw.free_scale[index].h_enable =
			osd_hw.free_scale_backup[index].h_enable;
		osd_hw.free_scale[index].v_enable =
			osd_hw.free_scale_backup[index].v_enable;
		osd_hw.free_scale_enable[index] =
			osd_hw.free_scale_enable_backup[index];
		osd_hw.free_scale_mode[index] =
			osd_hw.free_scale_mode_backup[index];
		free_scale_set = true;
	}
	return free_scale_set;
}

static bool osd_direct_compose_pan_display(struct osd_fence_map_s *fence_map)
{
	u32 index = fence_map->fb_index;
	u32 ext_addr = fence_map->ext_addr;
	u32 width_src = 0, width_dst = 0, height_src = 0, height_dst = 0;
	u32 x_start, x_end, y_start, y_end;
	bool freescale_update = false;
	struct pandata_s freescale_dst[HW_OSD_COUNT];

	ext_addr = ext_addr + fence_map->byte_stride * fence_map->yoffset;

	if (!osd_hw.osd_afbcd[index].enable) {
		canvas_config(osd_hw.fb_gem[index].canvas_idx,
			ext_addr,
			CANVAS_ALIGNED(fence_map->byte_stride),
			fence_map->height,
			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
		osd_hw.screen_base[index] = ext_addr;
		osd_hw.screen_size[index] =
			fence_map->byte_stride * fence_map->height;
	} else {
		osd_hw.osd_afbcd[index].phy_addr = ext_addr;
		osd_hw.osd_afbcd[index].frame_width =
			fence_map->width;
		osd_hw.osd_afbcd[index].frame_height =
			fence_map->height;
		if (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) {
			if (fence_map->width <= 128)
				osd_hw.osd_afbcd[index].conv_lbuf_len = 32;
			else if (fence_map->width <= 256)
				osd_hw.osd_afbcd[index].conv_lbuf_len = 64;
			else if (fence_map->width <= 512)
				osd_hw.osd_afbcd[index].conv_lbuf_len = 128;
			else if (fence_map->width <= 1024)
				osd_hw.osd_afbcd[index].conv_lbuf_len = 256;
			else if (fence_map->width <= 2048)
				osd_hw.osd_afbcd[index].conv_lbuf_len = 512;
			else
				osd_hw.osd_afbcd[index].conv_lbuf_len = 1024;
		}
		osd_hw.screen_base[index] = ext_addr;
		osd_hw.screen_size[index] = fence_map->afbc_len;
	}
	width_dst = osd_hw.free_dst_data_backup[index].x_end -
		osd_hw.free_dst_data_backup[index].x_start + 1;
	width_src = osd_hw.free_src_data_backup[index].x_end -
		osd_hw.free_src_data_backup[index].x_start + 1;

	height_dst = osd_hw.free_dst_data_backup[index].y_end -
		osd_hw.free_dst_data_backup[index].y_start + 1;
	height_src = osd_hw.free_src_data_backup[index].y_end -
		osd_hw.free_src_data_backup[index].y_start + 1;
	if (osd_hw.free_scale_enable[index] ||
		(width_src != width_dst) ||
		(height_src != height_dst) ||
		(fence_map->width != fence_map->dst_w) ||
		(fence_map->height != fence_map->dst_h)) {
		osd_hw.free_scale[index].h_enable = 1;
		osd_hw.free_scale[index].v_enable = 1;
		osd_hw.free_scale_enable[index] = 0x10001;
		osd_hw.free_scale_mode[index] = 1;
		if (osd_hw.free_scale_enable[index] !=
			osd_hw.free_scale_enable_backup[index]) {
			/* Todo: */
			osd_set_scan_mode(index);
			freescale_update = true;
		}

		osd_hw.pandata[index].x_start = fence_map->xoffset;
		osd_hw.pandata[index].x_end =
			fence_map->xoffset + fence_map->width - 1;
		osd_hw.pandata[index].y_start = 0;
		osd_hw.pandata[index].y_end = fence_map->height - 1;


		freescale_dst[index].x_start =
			osd_hw.free_dst_data_backup[index].x_start +
			(fence_map->dst_x * width_dst) / width_src;
		freescale_dst[index].x_end =
			osd_hw.free_dst_data_backup[index].x_start +
			((fence_map->dst_x + fence_map->dst_w) *
			width_dst) / width_src - 1;

		freescale_dst[index].y_start =
			osd_hw.free_dst_data_backup[index].y_start +
			(fence_map->dst_y * height_dst) / height_src;
		freescale_dst[index].y_end =
			osd_hw.free_dst_data_backup[index].y_start +
			((fence_map->dst_y + fence_map->dst_h) *
			height_dst) / height_src - 1;

		if (osd_hw.osd_reverse[index] == REVERSE_TRUE) {
			x_start = osd_hw.vinfo_width[VIU1]
				- freescale_dst[index].x_end - 1;
			y_start = osd_hw.vinfo_height[VIU1]
				- freescale_dst[index].y_end - 1;
			x_end = osd_hw.vinfo_width[VIU1]
				- freescale_dst[index].x_start - 1;
			y_end = osd_hw.vinfo_height[VIU1]
				- freescale_dst[index].y_start - 1;
			freescale_dst[index].x_start = x_start;
			freescale_dst[index].y_start = y_start;
			freescale_dst[index].x_end = x_end;
			freescale_dst[index].y_end = y_end;
		} else if (osd_hw.osd_reverse[index] == REVERSE_X) {
			x_start = osd_hw.vinfo_width[VIU1]
				- freescale_dst[index].x_end - 1;
			x_end = osd_hw.vinfo_width[VIU1]
				- freescale_dst[index].x_start - 1;
			freescale_dst[index].x_start = x_start;
			freescale_dst[index].x_end = x_end;

		} else if (osd_hw.osd_reverse[index] == REVERSE_Y) {
			y_start = osd_hw.vinfo_height[VIU1]
				- freescale_dst[index].y_end - 1;
			y_end = osd_hw.vinfo_height[VIU1]
				- freescale_dst[index].y_start - 1;
			freescale_dst[index].y_start = y_start;
			freescale_dst[index].y_end = y_end;
		}
		if (memcmp(&(osd_hw.free_src_data[index]),
			&osd_hw.pandata[index],
			sizeof(struct pandata_s)) != 0 ||
			memcmp(&(osd_hw.free_dst_data[index]),
			&freescale_dst[index],
			sizeof(struct pandata_s)) != 0) {
			memcpy(&(osd_hw.free_src_data[index]),
				&osd_hw.pandata[index],
				sizeof(struct pandata_s));
			memcpy(&(osd_hw.free_dst_data[index]),
				&freescale_dst[index],
				sizeof(struct pandata_s));
			freescale_update = true;

			if (((height_dst != height_src) ||
				(width_dst != width_src)) &&
				osd_hw.free_dst_data[index].y_end <
						osd_hw.vinfo_height[index] - 1)
				osd_set_dummy_data(index, 0);
			else
				osd_set_dummy_data(index, 0xff);
			osd_log_dbg(MODULE_BASE,
				"direct pandata x=%d,x_end=%d,y=%d,y_end=%d,width=%d,height=%d\n",
				osd_hw.pandata[index].x_start,
				osd_hw.pandata[index].x_end,
				osd_hw.pandata[index].y_start,
				osd_hw.pandata[index].y_end,
				fence_map->width,
				fence_map->height);
			osd_log_dbg(MODULE_BASE,
				"fence_map:xoffset=%d,yoffset=%d\n",
				fence_map->xoffset,
				fence_map->yoffset);
			osd_log_dbg(MODULE_BASE,
				"fence_map:dst_x=%d,dst_y=%d,dst_w=%d,dst_h=%d,byte_stride=%d\n",
				fence_map->dst_x,
				fence_map->dst_y,
				fence_map->dst_w,
				fence_map->dst_h,
				fence_map->byte_stride);
		}
	} else {
		osd_hw.pandata[index].x_start = 0;
		osd_hw.pandata[index].x_end = fence_map->width - 1;
		osd_hw.pandata[index].y_start = 0;
		osd_hw.pandata[index].y_end = fence_map->height - 1;

		osd_hw.dispdata[index].x_start = fence_map->dst_x;
		osd_hw.dispdata[index].x_end =
			fence_map->dst_x + fence_map->dst_w - 1;
		osd_hw.dispdata[index].y_start = fence_map->dst_y;
		osd_hw.dispdata[index].y_end =
			fence_map->dst_y + fence_map->dst_h - 1;
		if (osd_hw.osd_reverse[index] == REVERSE_TRUE) {
			x_start = osd_hw.vinfo_width[VIU1]
				- osd_hw.dispdata[index].x_end - 1;
			y_start = osd_hw.vinfo_height[VIU1]
				- osd_hw.dispdata[index].y_end - 1;
			x_end = osd_hw.vinfo_width[VIU1]
				- osd_hw.dispdata[index].x_start - 1;
			y_end = osd_hw.vinfo_height[VIU1]
				- osd_hw.dispdata[index].y_start - 1;
			osd_hw.dispdata[index].x_start = x_start;
			osd_hw.dispdata[index].y_start = y_start;
			osd_hw.dispdata[index].x_end = x_end;
			osd_hw.dispdata[index].y_end = y_end;
		} else if (osd_hw.osd_reverse[index] == REVERSE_X) {
			x_start = osd_hw.vinfo_width[VIU1]
				- osd_hw.dispdata[index].x_end - 1;
			x_end = osd_hw.vinfo_width[VIU1]
				- osd_hw.dispdata[index].x_start - 1;
			osd_hw.dispdata[index].x_start = x_start;
			osd_hw.dispdata[index].x_end = x_end;

		} else if (osd_hw.osd_reverse[index] == REVERSE_Y) {
			y_start = osd_hw.vinfo_height[VIU1]
				- osd_hw.dispdata[index].y_end - 1;
			y_end = osd_hw.vinfo_height[VIU1]
				- osd_hw.dispdata[index].y_start - 1;
			osd_hw.dispdata[index].y_start = y_start;
			osd_hw.dispdata[index].y_end = y_end;
		}
	}
	fence_map->ext_addr = ext_addr;
	return freescale_update;
}

static void osd_pan_display_single_fence(struct osd_fence_map_s *fence_map)
{
	s32 ret = 1;
	long diff_x, diff_y;
	u32 index = fence_map->fb_index;
	u32 xoffset = fence_map->xoffset;
	u32 yoffset = fence_map->yoffset;
	const struct color_bit_define_s *color = NULL;
	bool color_mode = false;
	bool freescale_update = false;
	u32 osd_enable = 0;
	bool skip = false;
	const struct vinfo_s *vinfo;
	u32 output_index = VIU1;

	if (index >= OSD2)
		goto out;
	vinfo = get_current_vinfo();
	if (!vinfo || (!strcmp(vinfo->name, "invalid") ||
			!strcmp(vinfo->name, "null")))
		goto out;

	osd_hw.vinfo_width[output_index] = vinfo->width;
	osd_hw.vinfo_height[output_index] = vinfo->height;

	if (timeline_created[output_index]) { /* out fence created success. */
		ret = osd_wait_buf_ready(fence_map);
		if (ret < 0)
			osd_log_dbg(MODULE_BASE, "fence wait ret %d\n", ret);
	}
	if (ret) {
		osd_hw.buffer_alloc[index] = 1;
		if (osd_hw.osd_fps_start[output_index])
			osd_hw.osd_fps[output_index]++;
		if (fence_map->op == 0xffffffff)
			skip = true;
		else
			osd_enable = (fence_map->op & 1) ? DISABLE : ENABLE;
		/* need update via direct render interface later */
		//fence_map->afbc_en = osd_hw.osd_afbcd[index].enable;
		if ((osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE)
			&& (osd_hw.osd_afbcd[index].enable)) {
			osd_hw.osd_afbcd[index].inter_format =
				 AFBC_EN | BLOCK_SPLIT |
				 YUV_TRANSFORM | SUPER_BLOCK_ASPECT;
		}
		/* Todo: */
		if (fence_map->ext_addr && fence_map->width
			&& fence_map->height) {
			spin_lock_irqsave(&osd_lock, lock_flags);
			use_ext = true;
			if (!osd_hw.osd_afbcd[index].enable) {
				osd_hw.fb_gem[index].canvas_idx =
				osd_extra_idx[index][ext_canvas_id[index]];
				ext_canvas_id[index] ^= 1;
				osd_hw.osd_afbcd[index].enable = DISABLE;
			} else
				osd_hw.osd_afbcd[index].enable = ENABLE;

			color = convert_hal_format(fence_map->format);
			if (color) {
				osd_hw.color_info[index] = color;
			} else
				osd_log_err("fence color format error %d\n",
					fence_map->format);

			if (DIRECT_COMPOSE_MODE ==
				fence_map->compose_type)
				freescale_update =
				osd_direct_compose_pan_display(fence_map);
			else if (GE2D_COMPOSE_MODE ==
				fence_map->compose_type) {
				freescale_update =
				osd_ge2d_compose_pan_display(fence_map);
				if (freescale_update)
					osd_set_dummy_data(index, 0xff);
			}

			osd_hw.reg[OSD_COLOR_MODE].update_func(index);

			/* geometry and freescale need update with ioctl */
			osd_hw.reg[DISP_GEOMETRY].update_func(index);
			osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
			if ((osd_hw.free_scale_enable[index]
					&& osd_update_window_axis)
					|| freescale_update) {
				if (!osd_hw.osd_display_debug[output_index])
				osd_hw.reg[DISP_FREESCALE_ENABLE]
					.update_func(index);
				osd_update_window_axis = false;
			}
			if ((osd_enable != osd_hw.enable[index] ||
				(osd_hw.osd_meson_dev.afbc_type
					== MALI_AFBC &&
				osd_hw.osd_afbcd[index].enable))
				&& (skip == false)
				&& (suspend_flag == false)) {
				osd_hw.enable[index] = osd_enable;
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[OSD_ENABLE]
					.update_func(index);
			}
			if (osd_hw.hw_rdma_en)
				osd_mali_afbc_start();
			spin_unlock_irqrestore(&osd_lock, lock_flags);
			osd_wait_vsync_hw(index);
		} else if (xoffset != osd_hw.pandata[index].x_start
			|| yoffset != osd_hw.pandata[index].y_start
			|| (use_ext)) {
			spin_lock_irqsave(&osd_lock, lock_flags);
			if (use_ext) {
				osd_hw.fb_gem[index].canvas_idx =
					OSD1_CANVAS_INDEX;

				osd_hw.pandata[index].x_start = xoffset;
				osd_hw.pandata[index].x_end   = xoffset +
					osd_hw.fb_gem[index].xres - 1;
				osd_hw.pandata[index].y_start = yoffset;
				osd_hw.pandata[index].y_end   = yoffset +
					osd_hw.fb_gem[index].yres - 1;
				osd_hw.screen_base[index] =
					osd_hw.screen_base_backup[index];
				osd_hw.screen_size[index] =
					osd_hw.screen_size_backup[index];

				osd_hw.dispdata[index].x_start =
					osd_hw.dispdata_backup[index].x_start;
				osd_hw.dispdata[index].x_end =
					osd_hw.dispdata_backup[index].x_end;
				osd_hw.dispdata[index].y_start =
					osd_hw.dispdata_backup[index].y_start;
				osd_hw.dispdata[index].y_end =
					osd_hw.dispdata_backup[index].y_end;

				/* restore free_scale info */
				osd_hw.free_scale[index].h_enable =
				osd_hw.free_scale_backup[index].h_enable;
				osd_hw.free_scale[index].v_enable =
				osd_hw.free_scale_backup[index].v_enable;
				osd_hw.free_scale_enable[index] =
					osd_hw.free_scale_enable_backup[index];
				osd_hw.free_scale_mode[index] =
					osd_hw.free_scale_mode_backup[index];

				memcpy(&osd_hw.free_src_data[index],
					&osd_hw.free_src_data_backup[index],
					sizeof(struct pandata_s));
				memcpy(&osd_hw.free_dst_data[index],
					&osd_hw.free_dst_data_backup[index],
					sizeof(struct pandata_s));
				osd_set_dummy_data(index, 0xff);
				osd_log_dbg(MODULE_BASE,
					"switch back dispdata_backup x=%d,x_end=%d,y=%d,y_end=%d\n",
					osd_hw.dispdata_backup[index].x_start,
					osd_hw.dispdata_backup[index].x_end,
					osd_hw.dispdata_backup[index].y_start,
					osd_hw.dispdata_backup[index].y_end);

				canvas_config(osd_hw.fb_gem[index].canvas_idx,
					osd_hw.fb_gem[index].addr,
					CANVAS_ALIGNED(
						osd_hw.fb_gem[index].width),
					osd_hw.fb_gem[index].height,
					CANVAS_ADDR_NOWRAP,
					CANVAS_BLKMODE_LINEAR);
				use_ext = false;
				color_mode = true;
				freescale_update = true;
			} else {
				diff_x = xoffset -
					osd_hw.pandata[index].x_start;
				diff_y = yoffset -
					osd_hw.pandata[index].y_start;
				osd_hw.pandata[index].x_start += diff_x;
				osd_hw.pandata[index].x_end   += diff_x;
				osd_hw.pandata[index].y_start += diff_y;
				osd_hw.pandata[index].y_end   += diff_y;
			}
			if (osd_hw.osd_afbcd[index].enable == ENABLE) {
				/* osd_hw.osd_afbcd[index].phy_addr =
				 *	(osd_hw.pandata[index].y_start /
				 *	osd_hw.osd_afbcd[index].frame_height) *
				 *	osd_hw.osd_afbcd[index].frame_height *
				 *  osd_hw.fb_gem[index].width;
				 *  osd_hw.osd_afbcd[index].phy_addr +=
				 *  osd_hw.fb_gem[index].addr;
				 */
				osd_hw.osd_afbcd[index].phy_addr =
					osd_hw.osd_afbcd[index].addr
					[osd_hw.pandata[index].y_start /
					osd_hw.osd_afbcd[index].frame_height];
			}
			color = convert_hal_format(fence_map->format);
			if (color) {
				if (color != osd_hw.color_backup[index]) {
					color_mode = true;
					osd_hw.color_backup[index] = color;
				}
				osd_hw.color_info[index] = color;
			} else {
				osd_log_err("fence color format error %d\n",
					fence_map->format);
			}
			if (color_mode)
				osd_hw.reg[OSD_COLOR_MODE].update_func(index);
			osd_hw.reg[DISP_GEOMETRY].update_func(index);
			osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
			if ((osd_hw.free_scale_enable[index]
					&& osd_update_window_axis)
					|| (osd_hw.free_scale_enable[index]
					&& freescale_update)) {
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[DISP_FREESCALE_ENABLE]
						.update_func(index);
				osd_update_window_axis = false;
			}
			if ((osd_enable != osd_hw.enable[index] ||
				(osd_hw.osd_meson_dev.afbc_type
					== MALI_AFBC &&
				osd_hw.osd_afbcd[index].enable))
				&& (skip == false)
				&& (suspend_flag == false)) {
				osd_hw.enable[index] = osd_enable;
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[OSD_ENABLE]
					.update_func(index);
			}
			if (osd_hw.hw_rdma_en)
				osd_mali_afbc_start();
			spin_unlock_irqrestore(&osd_lock, lock_flags);
			osd_wait_vsync_hw(index);
		} else if ((osd_enable != osd_hw.enable[index] ||
			(osd_hw.osd_meson_dev.afbc_type == MALI_AFBC &&
			osd_hw.osd_afbcd[index].enable))
			&& (skip == false)) {
			spin_lock_irqsave(&osd_lock, lock_flags);
			if (suspend_flag == false) {
				osd_hw.enable[index] = osd_enable;
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[OSD_ENABLE]
					.update_func(index);
			}
			if (osd_hw.hw_rdma_en)
				osd_mali_afbc_start();
			spin_unlock_irqrestore(&osd_lock, lock_flags);
			osd_wait_vsync_hw(index);
		}
	}
#ifdef CONFIG_AMLOGIC_MEDIA_FB_EXT
		if (ret)
			osd_ext_clone_pan(index);
#endif
out:
	if (timeline_created[output_index]) {
		if (ret)
			osd_timeline_increase(output_index);
		else
			osd_log_err("------NOT signal out_fence ERROR\n");
	}
	if (fence_map->in_fence)
		osd_put_fenceobj(fence_map->in_fence);
}

static void osd_pan_display_single_fence_viu2(struct osd_fence_map_s *fence_map)
{
	osd_log_err("osd hwc version not support viu2\n");
}

static void osd_pan_display_update_info(struct layer_fence_map_s *layer_map)
{
	u32 index = layer_map->fb_index;
	const struct color_bit_define_s *color = NULL;
	u32 ext_addr = 0;
	u32 format = 0;
	u32 output_index = 0;

	if (index > OSD_MAX)
		return;
	/* get in fence_fd */
	osd_hw.in_fd[index] = layer_map->in_fd;
	osd_hw.buffer_alloc[index] = 1;
	osd_hw.enable[index] = layer_map->enable;
	if (osd_hw.enable[index] == 0) {
		osd_log_dbg(MODULE_BASE, "osd%d: blanked\n", index);
		return;
	}
	osd_hw.osd_afbcd[index].enable =
		(layer_map->afbc_inter_format & AFBC_EN) >> 31;
	if (layer_map->plane_alpha == 0xff)
		layer_map->plane_alpha = 0x100;
	osd_hw.gbl_alpha[index] = layer_map->plane_alpha;
	osd_hw.dim_layer[index] = layer_map->dim_layer;

	/* Todo: */
	if (layer_map->dim_layer) {
		/* osd dim layer */
		osd_hw.dim_layer[index] = layer_map->dim_layer;
		osd_hw.dim_color[index] = layer_map->dim_color;
		osd_hw.order[index] = layer_map->zorder;
		switch (layer_map->blend_mode) {
		case BLEND_MODE_PREMULTIPLIED:
			osd_hw.premult_en[index] = 1;
			break;
		case BLEND_MODE_COVERAGE:
		case BLEND_MODE_NONE:
		case BLEND_MODE_INVALID:
			osd_hw.premult_en[index] = 0;
			break;
		}
		osd_hw.src_data[index].x = 0;
		osd_hw.src_data[index].y = 0;
		osd_hw.src_data[index].w = 64;
		osd_hw.src_data[index].h = 64;
		osd_hw.dst_data[index].x = layer_map->dst_x;
		osd_hw.dst_data[index].y = layer_map->dst_y;
		osd_hw.dst_data[index].w = layer_map->dst_w;
		osd_hw.dst_data[index].h = layer_map->dst_h;
	} else if (layer_map->ext_addr && layer_map->src_w
		&& layer_map->src_h) {
		if (!osd_hw.osd_afbcd[index].enable) {
			osd_hw.fb_gem[index].canvas_idx =
				osd_extra_idx[index][ext_canvas_id[index]];
			ext_canvas_id[index] ^= 1;
			osd_hw.osd_afbcd[index].enable = DISABLE;
		} else
			osd_hw.osd_afbcd[index].enable = ENABLE;
		if (osd_hw.osd_afbcd[index].enable)
			format = layer_map->format | AFBC_EN;
		else
			format = layer_map->format;
		color = convert_hal_format(format);
		if (color) {
			osd_hw.color_info[index] = color;
		} else {
			osd_log_err("fence color format error %d\n",
				layer_map->format);
			return;
		}
		if (DIRECT_COMPOSE_MODE !=
			layer_map->compose_type)
			return;
		ext_addr = layer_map->ext_addr;
		#if 0
		ext_addr = ext_addr +
			layer_map->byte_stride *
			layer_map->src_y;
		#endif
		if (!osd_hw.osd_afbcd[index].enable) {
			/*ext_addr is no crop, so height =
			 * layer_map->src_h + layer_map->src_y
			 */
			osd_hw.fb_gem[index].addr = ext_addr;
			osd_hw.fb_gem[index].width = layer_map->byte_stride;
			osd_hw.fb_gem[index].height =
				layer_map->src_h + layer_map->src_y;
			osd_hw.screen_base[index] = ext_addr;
			osd_hw.screen_size[index] =
				layer_map->byte_stride * layer_map->src_h;
		} else {
			osd_hw.osd_afbcd[index].phy_addr = ext_addr;
			if (osd_hw.osd_meson_dev.afbc_type ==
				MESON_AFBC) {
				osd_hw.osd_afbcd[index].frame_width =
					layer_map->src_w;
				osd_hw.osd_afbcd[index].frame_height =
					layer_map->src_h;
				if (layer_map->src_w <= 128)
					osd_hw.osd_afbcd[index]
						.conv_lbuf_len = 32;
				else if (layer_map->src_w <= 256)
					osd_hw.osd_afbcd[index]
						.conv_lbuf_len = 64;
				else if (layer_map->src_w <= 512)
					osd_hw.osd_afbcd[index]
						.conv_lbuf_len = 128;
				else if (layer_map->src_w <= 1024)
					osd_hw.osd_afbcd[index]
						.conv_lbuf_len = 256;
				else if (layer_map->src_w <= 2048)
					osd_hw.osd_afbcd[index]
						.conv_lbuf_len = 512;
				else
					osd_hw.osd_afbcd[index]
						.conv_lbuf_len = 1024;
				osd_hw.screen_base[index] = ext_addr;
				osd_hw.screen_size[index] =
					layer_map->afbc_len;
			} else if (osd_hw.osd_meson_dev
				.afbc_type == MALI_AFBC) {
				osd_hw.osd_afbcd[index].frame_width =
					layer_map->byte_stride / 4;
					//BYTE_32_ALIGNED(layer_map->src_w);
				osd_hw.osd_afbcd[index].frame_height =
					BYTE_8_ALIGNED(layer_map->fb_height);
					/*BYTE_8_ALIGNED(layer_map->src_h);*/
				osd_hw.screen_base[index] = ext_addr;
				osd_hw.screen_size[index] =
					layer_map->afbc_len;
			}
		}
		/* just get para, need update via do_hwc */
		osd_hw.order[index] = layer_map->zorder;
		switch (layer_map->blend_mode) {
		case BLEND_MODE_PREMULTIPLIED:
			osd_hw.premult_en[index] = 1;
			break;
		case BLEND_MODE_COVERAGE:
		case BLEND_MODE_NONE:
		case BLEND_MODE_INVALID:
			osd_hw.premult_en[index] = 0;
			break;
		}
		//Todo: fence_map.plane_alpha
		osd_hw.osd_afbcd[index].inter_format =
			layer_map->afbc_inter_format & 0x7fffffff;
		osd_hw.osd_afbcd[index].format =
			color->color_index;
		osd_hw.src_data[index].x = layer_map->src_x;
		osd_hw.src_data[index].y = layer_map->src_y;
		osd_hw.src_data[index].w = layer_map->src_w;
		osd_hw.src_data[index].h = layer_map->src_h;
		osd_hw.dst_data[index].x = layer_map->dst_x;
		osd_hw.dst_data[index].y = layer_map->dst_y;
		osd_hw.dst_data[index].w = layer_map->dst_w;
		osd_hw.dst_data[index].h = layer_map->dst_h;
		if ((layer_map->fb_height != layer_map->src_h) ||
		    (layer_map->fb_width != layer_map->src_w))
			osd_hw.src_crop[index] = 1;
		else
			osd_hw.src_crop[index] = 0;
		if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
			osd_hw.free_src_data[index].x_start = layer_map->src_x;
			osd_hw.free_src_data[index].y_start = layer_map->src_y;
			osd_hw.free_src_data[index].x_end =
				layer_map->src_x + layer_map->src_w - 1;
			osd_hw.free_src_data[index].y_end =
				layer_map->src_y + layer_map->src_h - 1;
			osd_hw.free_dst_data[index].x_start = layer_map->dst_x;
			osd_hw.free_dst_data[index].y_start = layer_map->dst_y;
			osd_hw.free_dst_data[index].x_end =
				layer_map->dst_x + layer_map->dst_w - 1;
			osd_hw.free_dst_data[index].y_end =
				layer_map->dst_y + layer_map->dst_h - 1;
			output_index = get_output_device_id(index);
			if (osd_hw.field_out_en[output_index]) {
				osd_hw.free_dst_data[index].y_start /= 2;
				osd_hw.free_dst_data[index].y_end /= 2;
			}
		}
	}
	#if 0
	osd_hw.dst_data[index].x -= osd_hw.disp_info.position_x;
	osd_hw.dst_data[index].y -= osd_hw.disp_info.position_y;
	#endif
}

static void _osd_pan_display_layers_fence(
	u32 output_index,
	struct vinfo_s *vinfo,
	struct osd_layers_fence_map_s *fence_map)
{
	int i = 0;
	int ret;
	int start_index = 0;
	int backup_en = 0;
	int osd_count = 0;
	/* osd_count need -1 when VIU2 enable */
	struct layer_fence_map_s *layer_map = NULL;

	if (output_index == VIU1) {
		osd_count = osd_hw.osd_meson_dev.viu1_osd_count;
		if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL)
			osd_count = 1;
		start_index = 0;
		backup_en = 1;
	} else if (output_index == VIU2) {
		start_index = osd_hw.osd_meson_dev.viu2_index;
		osd_count = start_index + 1;
		backup_en = 0;
	} else {
		osd_log_err("invald output_index=%d\n", output_index);
		return;
	}

	if (!vinfo || (!strcmp(vinfo->name, "invalid") ||
				!strcmp(vinfo->name, "null")))
		/* vout is null, release fence & buf file. */
		goto out;

	osd_hw.vinfo_width[output_index] = vinfo->width;
	osd_hw.vinfo_height[output_index] = vinfo->field_height;
	memcpy(&osd_hw.disp_info[output_index], &fence_map->disp_info,
	       sizeof(struct display_flip_info_s));

	if (osd_hw.osd_fps_start[output_index])
		osd_hw.osd_fps[output_index]++;
	if (backup_en)
		clear_backup_info();
	osd_hw.hdr_used = fence_map->hdr_mode;
	for (i = start_index; i < osd_count; i++) {
		layer_map = &fence_map->layer_map[i];
		if (i != layer_map->fb_index) {
			osd_hw.screen_base[i] = 0;
			osd_hw.screen_size[i] = 0;
			osd_hw.enable[i] = 0;
			continue;
		}
		/* wait in fence */
		if (timeline_created[output_index] && layer_map->enable
			&& (fence_map->cmd == LAYER_SYNC)) {
			ret = osd_wait_buf_ready_combine(layer_map);
			if (ret < 0)
				osd_log_dbg(MODULE_FENCE,
					"fence wait ret %d\n", ret);
		}
		osd_pan_display_update_info(layer_map);
		if (backup_en)
			save_layer_info(layer_map);
	}
	/* set hw regs */
	if (osd_hw.osd_display_debug[output_index] != OSD_DISP_DEBUG)
		osd_setting_blend(output_index);
out:
	/* signal out fence */
	if (timeline_created[output_index]) {
		if (osd_hw.osd_debug.osd_single_step_mode) {
			/* single step mode */
			if (osd_hw.osd_debug.osd_single_step > 0) {
				osd_timeline_increase(output_index);
				osd_log_dbg(MODULE_FENCE, "signal out fence\n");
				osd_hw.osd_debug.osd_single_step--;
			} else
				osd_hw.osd_debug.wait_fence_release = true;
		} else
			osd_timeline_increase(output_index);
	}
	/* clear osd layer's order */
	for (i = start_index; i < osd_count; i++) {
		layer_map = &fence_map->layer_map[i];
		if (displayed_bufs[i]) {
			fput(displayed_bufs[i]);
			displayed_bufs[i] = NULL;
		}
		if (layer_map->buf_file)
			displayed_bufs[i] = layer_map->buf_file;
		if (layer_map->in_fence)
			osd_put_fenceobj(layer_map->in_fence);
		osd_hw.order[i] = 0;
	}
}

static void osd_pan_display_layers_fence(
	struct osd_layers_fence_map_s *fence_map)
{
	u32 output_index = VIU1;
	struct vinfo_s *vinfo = NULL;

	vinfo = get_current_vinfo();

	_osd_pan_display_layers_fence(output_index,
				      vinfo, fence_map);
}

static void osd_pan_display_layers_fence_viu2(
	struct osd_layers_fence_map_s *fence_map)
{
	u32 output_index = VIU2;
	struct vinfo_s *vinfo = NULL;

#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vinfo = get_current_vinfo2();
#endif

	_osd_pan_display_layers_fence(output_index,
				      vinfo, fence_map);
}
#endif

void osd_pan_display_hw(u32 index, unsigned int xoffset, unsigned int yoffset)
{
	long diff_x, diff_y;
	u32 output_index;

	if (index >= HW_OSD_COUNT)
		return;
	output_index = get_output_device_id(index);
	if (xoffset != osd_hw.pandata[index].x_start
	    || yoffset != osd_hw.pandata[index].y_start) {
		diff_x = xoffset - osd_hw.pandata[index].x_start;
		diff_y = yoffset - osd_hw.pandata[index].y_start;
		osd_hw.pandata[index].x_start += diff_x;
		osd_hw.pandata[index].x_end   += diff_x;
		osd_hw.pandata[index].y_start += diff_y;
		osd_hw.pandata[index].y_end   += diff_y;
		osd_hw.src_data[index].x = osd_hw.pandata[index].x_start;
		osd_hw.src_data[index].y = osd_hw.pandata[index].y_start;
		osd_hw.src_data[index].w = osd_hw.pandata[index].x_end
			- osd_hw.pandata[index].x_start + 1;
		osd_hw.src_data[index].h = osd_hw.pandata[index].y_end
			- osd_hw.pandata[index].y_start + 1;
		add_to_update_list(index, DISP_GEOMETRY);
		if (osd_hw.osd_fps_start[output_index])
			osd_hw.osd_fps[output_index]++;
		/* osd_wait_vsync_hw(); */
	}
#ifdef CONFIG_AMLOGIC_MEDIA_FB_EXT
	osd_ext_clone_pan(index);
#endif
	osd_log_dbg2(MODULE_BASE, "offset[%d-%d]x[%d-%d]y[%d-%d]\n",
		    xoffset, yoffset,
		    osd_hw.pandata[index].x_start,
		    osd_hw.pandata[index].x_end,
		    osd_hw.pandata[index].y_start,
		    osd_hw.pandata[index].y_end);
}

void osd_get_info(u32 index, u32 *addr, u32 *width, u32 *height)
{
	*addr = osd_hw.fb_gem[index].addr;
	*width = osd_hw.fb_gem[index].width;
	*height = osd_hw.fb_gem[index].yres;
}

static  void  osd_update_disp_scale_enable(u32 index)
{
	u32 osd2_cursor = 0;
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];

	if (osd_hw.hw_cursor_en)
		osd2_cursor = 1;

	if (osd_hw.scale[index].h_enable) {
		VSYNCOSD_SET_MPEG_REG_MASK(
			osd_reg->osd_blk0_cfg_w0, 3 << 12);
		if ((index == OSD2) && osd2_cursor) {
			VSYNCOSD_CLR_MPEG_REG_MASK(
				osd_reg->osd_blk0_cfg_w0, 3 << 12);
		}
	} else
		VSYNCOSD_CLR_MPEG_REG_MASK(
			osd_reg->osd_blk0_cfg_w0, 3 << 12);

	if (osd_hw.scan_mode[index] != SCAN_MODE_INTERLACE) {
		if (osd_hw.scale[index].v_enable) {
			VSYNCOSD_SET_MPEG_REG_MASK(
				osd_reg->osd_blk0_cfg_w0, 1 << 14);
			if ((index == OSD2) && osd2_cursor) {
				VSYNCOSD_CLR_MPEG_REG_MASK(
					osd_reg->osd_blk0_cfg_w0, 1 << 14);
			}
		} else
			VSYNCOSD_CLR_MPEG_REG_MASK(
				osd_reg->osd_blk0_cfg_w0, 1 << 14);
	}
	remove_from_update_list(index, DISP_SCALE_ENABLE);
}

static void osd_set_dummy_data(u32 index, u32 alpha)
{
	VSYNCOSD_WR_MPEG_REG(hw_osd_reg_array[index].osd_sc_dummy_data,
		osd_hw.osd_meson_dev.dummy_data | alpha);
}

static void osd_update_disp_freescale_enable(u32 index)
{
	u64 hf_phase_step, vf_phase_step;
	int src_w, src_h, dst_w, dst_h;
	int bot_ini_phase, top_ini_phase;
	int vsc_ini_rcv_num, vsc_ini_rpt_p0_num;
	int vsc_bot_rcv_num = 0, vsc_bot_rpt_p0_num = 0;
	int hsc_ini_rcv_num, hsc_ini_rpt_p0_num;
	int hf_bank_len = 4;
	int vf_bank_len = 4;
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];
	u32 data32 = 0x0;
	u32 shift_workaround = 0;
	u32 output_index = 0;

	if (osd_hw.osd_meson_dev.osd_ver != OSD_HIGH_ONE)
		osd_reg = &hw_osd_reg_array[0];

	if (osd_hw.scale_workaround)
		vf_bank_len = 2;
	else
		vf_bank_len = 4;

	output_index = get_output_device_id(index);
	if (osd_hw.hwc_enable[output_index] && (index == OSD1))
		shift_workaround = osd_hw.workaround_line;

#ifndef NEW_PPS_PHASE
	if (osd_hw.bot_type == 1) {
		vsc_bot_rcv_num = 4;
		vsc_bot_rpt_p0_num = 1;
	} else if (osd_hw.bot_type == 2) {
		vsc_bot_rcv_num = 6;
		vsc_bot_rpt_p0_num = 2;
	} else if (osd_hw.bot_type == 3) {
		vsc_bot_rcv_num = 8;
		vsc_bot_rpt_p0_num = 3;
	}
	vsc_ini_rcv_num = vf_bank_len;
	vsc_ini_rpt_p0_num =
		(vf_bank_len / 2 - 1) > 0 ? (vf_bank_len / 2 - 1) : 0;
#endif

	hsc_ini_rcv_num = hf_bank_len;
	hsc_ini_rpt_p0_num = hf_bank_len / 2 - 1;

	src_w = osd_hw.free_src_data[index].x_end -
		osd_hw.free_src_data[index].x_start + 1;
	src_h = osd_hw.free_src_data[index].y_end -
		osd_hw.free_src_data[index].y_start + 1;
	dst_w = osd_hw.free_dst_data[index].x_end -
		osd_hw.free_dst_data[index].x_start + 1;
	dst_h = osd_hw.free_dst_data[index].y_end -
		osd_hw.free_dst_data[index].y_start + 1;

	/* config osd sc control reg */
	data32 = 0x0;
	if (osd_hw.free_scale_enable[index]) {
		/* enable osd scaler */
		if (osd_hw.free_scale_mode[index] & 0x1) {
			/* Todo: */
			if (osd_hw.osd_meson_dev.osd_ver == OSD_NORMAL)
				data32 |= index;  /* select osd1/2 input */
			/* Todo: vd alpha */
			data32 |= 1 << 2; /* enable osd scaler */
			data32 |= 1 << 3; /* enable osd scaler path */
			VSYNCOSD_WR_MPEG_REG(
				osd_reg->osd_sc_ctrl0, data32);
		}
	} else {
		/* disable osd scaler path */
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_sc_ctrl0, 0);
	}

	hf_phase_step = (u64)src_w << 24;
	do_div(hf_phase_step, dst_w);
	if (shift_workaround) {
		vf_phase_step = (u64)(src_h - 1) << 20;
		do_div(vf_phase_step, dst_h);
	} else {
		vf_phase_step = (u64)src_h << 20;
		do_div(vf_phase_step, dst_h);
	}

#ifdef NEW_PPS_PHASE
	if (osd_hw.field_out_en[output_index]) {
		struct osd_f2v_vphase_s vphase;

		f2v_get_vertical_phase(
			vf_phase_step, OSD_F2V_P2IT,
			vf_bank_len, &vphase);
		vsc_ini_rcv_num = vphase.rcv_num;
		vsc_ini_rpt_p0_num = vphase.rpt_num;
		top_ini_phase = vphase.phase;

		f2v_get_vertical_phase(
			vf_phase_step, OSD_F2V_P2IB,
			vf_bank_len, &vphase);
		vsc_bot_rcv_num = vphase.rcv_num;
		vsc_bot_rpt_p0_num = vphase.rpt_num;
		bot_ini_phase = vphase.phase;
	} else {
		struct osd_f2v_vphase_s vphase;

		f2v_get_vertical_phase(
			vf_phase_step, OSD_F2V_P2P,
			vf_bank_len, &vphase);
		vsc_ini_rcv_num = vphase.rcv_num;
		vsc_ini_rpt_p0_num = vphase.rpt_num;
		top_ini_phase = vphase.phase;

		vsc_bot_rcv_num = 0;
		vsc_bot_rpt_p0_num = 0;
		bot_ini_phase = 0;
	}
#else
	if (osd_hw.field_out_en[output_index])   /* interface output */
		bot_ini_phase = ((vf_phase_step / 2) >> 4);
	else
		bot_ini_phase = 0;
	top_ini_phase = 0;
#endif

	vf_phase_step = (vf_phase_step << 4);
	/* config osd scaler in/out hv size */
	data32 = 0x0;
	if (shift_workaround) {
		vsc_ini_rcv_num++;
		if (osd_hw.field_out_en[output_index])
			vsc_bot_rcv_num++;
	}


	if (osd_hw.free_scale_enable[index]) {
		data32 = (((src_h - 1) & 0x1fff)
				| ((src_w - 1) & 0x1fff) << 16);
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_sci_wh_m1, data32);
		data32 = ((osd_hw.free_dst_data[index].x_end & 0xfff) |
			  (osd_hw.free_dst_data[index].x_start & 0xfff) << 16);
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_sco_h_start_end, data32);
		data32 = ((osd_hw.free_dst_data[index].y_end & 0xfff) |
			  (osd_hw.free_dst_data[index].y_start & 0xfff) << 16);
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_sco_v_start_end, data32);
	}

	data32 = 0x0;
	if (osd_hw.free_scale[index].v_enable) {
		data32 |= (vf_bank_len & 0x7)
			| ((vsc_ini_rcv_num & 0xf) << 3)
			| ((vsc_ini_rpt_p0_num & 0x3) << 8);
		if (osd_hw.field_out_en[output_index])
			data32 |= ((vsc_bot_rcv_num & 0xf) << 11)
				| ((vsc_bot_rpt_p0_num & 0x3) << 16)
				| (1 << 23);
		if (osd_hw.scale_workaround)
			data32 |= 1 << 21;
		data32 |= 1 << 24;
		if (osd_hw.osd_meson_dev.cpu_id >=
			__MESON_CPU_MAJOR_ID_G12B)
			data32 |= 1 << 25;
	}
	VSYNCOSD_WR_MPEG_REG(osd_reg->osd_vsc_ctrl0, data32);
	data32 = 0x0;
	if (osd_hw.free_scale[index].h_enable) {
		data32 |= (hf_bank_len & 0x7)
			| ((hsc_ini_rcv_num & 0xf) << 3)
			| ((hsc_ini_rpt_p0_num & 0x3) << 8);
		data32 |= 1 << 22;
	}
	VSYNCOSD_WR_MPEG_REG(osd_reg->osd_hsc_ctrl0, data32);

	data32 = top_ini_phase;
	if (osd_hw.free_scale_enable[index]) {
		data32 |= (bot_ini_phase & 0xffff) << 16;
		VSYNCOSD_WR_MPEG_REG_BITS(
			osd_reg->osd_hsc_phase_step,
			hf_phase_step, 0, 28);
		VSYNCOSD_WR_MPEG_REG_BITS(
			osd_reg->osd_hsc_init_phase, 0, 0, 16);
		VSYNCOSD_WR_MPEG_REG_BITS(
			osd_reg->osd_vsc_phase_step,
			vf_phase_step, 0, 28);
		VSYNCOSD_WR_MPEG_REG(
			osd_reg->osd_vsc_init_phase, data32);
	}
	if ((osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) &&
	    (!osd_hw.hwc_enable[output_index])) {
		osd_setting_blending_scope(index);
		vpp_blend_setting_default(index);
	}

	remove_from_update_list(index, DISP_FREESCALE_ENABLE);
}


static void osd_update_coef(u32 index)
{
	int i;
	bool need_update_coef = false;
	int *hf_coef, *vf_coef;
	int use_v_filter_mode, use_h_filter_mode;
	int OSD_SCALE_COEF_IDX, OSD_SCALE_COEF;

	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		OSD_SCALE_COEF_IDX =
			hw_osd_reg_array[index].osd_scale_coef_idx;
		OSD_SCALE_COEF =
			hw_osd_reg_array[index].osd_scale_coef;
	} else {
		OSD_SCALE_COEF_IDX =
			hw_osd_reg_array[0].osd_scale_coef_idx;
		OSD_SCALE_COEF =
			hw_osd_reg_array[0].osd_scale_coef;
	}
	use_v_filter_mode = osd_hw.use_v_filter_mode[index];
	use_h_filter_mode = osd_hw.use_h_filter_mode[index];
	if (osd_hw.scale_workaround) {
		if (use_v_filter_mode != 3) {
			use_v_filter_mode = 3;
			need_update_coef = true;
		} else
			need_update_coef = false;
	} else {
		if (use_v_filter_mode != osd_v_filter_mode) {
			use_v_filter_mode = osd_v_filter_mode;
			need_update_coef = true;
		} else
			need_update_coef = false;
	}
	if (need_update_coef) {
		vf_coef = filter_table[use_v_filter_mode];
		osd_reg_set_bits(OSD_SCALE_COEF_IDX, 0x0000, 0, 9);
		for (i = 0; i < 33; i++)
			osd_reg_write(OSD_SCALE_COEF, vf_coef[i]);
	}
	need_update_coef = false;
	if (use_h_filter_mode != osd_h_filter_mode) {
		use_h_filter_mode = osd_h_filter_mode;
		need_update_coef = true;
	}
	hf_coef = filter_table[use_h_filter_mode];
	if (need_update_coef) {
		osd_reg_set_bits(OSD_SCALE_COEF_IDX, 0x0100, 0, 9);
		for (i = 0; i < 33; i++)
			osd_reg_write(OSD_SCALE_COEF, hf_coef[i]);
	}
	osd_hw.use_v_filter_mode[index] = use_v_filter_mode;
	osd_hw.use_h_filter_mode[index] = use_h_filter_mode;

	remove_from_update_list(index, OSD_FREESCALE_COEF);
}

static int afbc_pix_format(u32 fmt_mode)
{
	u32 pix_format = RGBA8888;

	switch (fmt_mode) {
	case COLOR_INDEX_YUV_422:
		pix_format = YUV422_8B;
		break;
	case COLOR_INDEX_16_565:
		pix_format = RGB565;
		break;
	case COLOR_INDEX_16_1555_A:
		pix_format = RGBA5551;
		break;
	case COLOR_INDEX_16_4444_R:
	case COLOR_INDEX_16_4444_A:
		pix_format = RGBA4444;
		break;
	case COLOR_INDEX_32_BGRX:
	case COLOR_INDEX_32_XBGR:
	case COLOR_INDEX_32_RGBX:
	case COLOR_INDEX_32_XRGB:
	case COLOR_INDEX_32_BGRA:
	case COLOR_INDEX_32_ABGR:
	case COLOR_INDEX_32_RGBA:
	case COLOR_INDEX_32_ARGB:
		pix_format = RGBA8888;
		break;
	case COLOR_INDEX_24_888_B:
	case COLOR_INDEX_24_RGB:
		pix_format = RGB888;
		break;
	case COLOR_INDEX_RGBA_1010102:
		pix_format = RGBA1010102;
		break;
	default:
		osd_log_err("unsupport fmt:%x\n", fmt_mode);
		break;
	}
	return pix_format;
}

static char force_colorrange;
static int __init osd_colorrange_setup(char *s)
{
	if (!(strcmp(s, "true")))
		force_colorrange = 1;
	else
		force_colorrange = 0;

	return 0;
}
__setup("fullcolorrange=", osd_colorrange_setup);

static void osd_update_color_mode(u32 index)
{
	u32  data32 = 0;
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];

	if (osd_hw.color_info[index] != NULL) {
		enum color_index_e idx =
			osd_hw.color_info[index]->color_index;

		data32 = (osd_hw.scan_mode[index] ==
			SCAN_MODE_INTERLACE) ? 2 : 0;
		data32 |= VSYNCOSD_RD_MPEG_REG(
			osd_reg->osd_blk0_cfg_w0)
			& 0x70007040;
		data32 |= osd_hw.fb_gem[index].canvas_idx << 16;
		if ((osd_hw.osd_afbcd[index].enable) &&
			(osd_hw.osd_meson_dev.afbc_type == MALI_AFBC))
			data32 |= OSD_DATA_BIG_ENDIAN << 15;
		else
			data32 |= OSD_DATA_LITTLE_ENDIAN << 15;
		data32 |= osd_hw.color_info[index]->hw_colormat << 2;

		/* Todo: what about osd3 */
		if ((osd_hw.osd_meson_dev.cpu_id < __MESON_CPU_MAJOR_ID_GXTVBB)
			&& (index == OSD1)) {
			if (osd_hw.color_info[index]->color_index <
				COLOR_INDEX_YUV_422)
				data32 |= 1 << 7; /* yuv enable */
		}
		if ((osd_hw.osd_meson_dev.cpu_id != __MESON_CPU_MAJOR_ID_GXTVBB)
			&& (index == OSD2)) {
			if (osd_hw.color_info[index]->color_index <
				COLOR_INDEX_YUV_422)
				data32 |= 1 << 7; /* yuv enable */
		}
		/* osd_blk_mode */
		data32 |=  osd_hw.color_info[index]->hw_blkmode << 8;
		VSYNCOSD_WR_MPEG_REG(
			osd_reg->osd_blk0_cfg_w0, data32);
		/*
		 * VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK1_CFG_W0, data32);
		 * VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK2_CFG_W0, data32);
		 * VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK3_CFG_W0, data32);
		 */
		if (osd_hw.osd_afbcd[index].enable) {
			if (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) {
				/* only the RGBA32 mode */
				VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_MODE,
					(3 << 24) |
					(4 << 16) | /* hold_line_num */
					(0xe4 << 8) |
					(1 << 6) |
					(1 << 5) |
					(0x15 << 0)); /* pixel_packing_fmt */
				/* TODO: add pixel_packing_fmt setting */
				VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_CONV_CTRL,
					(osd_hw.osd_afbcd[index].conv_lbuf_len
					& 0xffff));
				/* afbc mode RGBA32 -> RGB24
				 * data32 = (0x1b << 24) |
				 *	(osd_hw.osd_afbcd[OSD1].phy_addr
				 *   & 0xffffff);
				 * VSYNCOSD_WR_MPEG_REG(
				 *	OSD1_AFBCD_CHROMA_PTR,
				 *	data32);
				 */
				VSYNCOSD_WR_MPEG_REG_BITS(
					OSD1_AFBCD_CHROMA_PTR, 0xe4, 24, 8);
				/*
				 * data32 = (0xe4 << 24) |
				 *	(osd_hw.osd_afbcd[OSD1].phy_addr
				 *   & 0xffffff);
				 * VSYNCOSD_WR_MPEG_REG(
				 *	OSD1_AFBCD_CHROMA_PTR,
				 *	data32);
				 */
			} else if (
			osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) {
				u32 bpp, pixel_format;
				u32 line_stride, output_stride;
				u32 aligned_32 = 1; /* need get from hwc line */
				#if 0
				u32 super_block_aspect = MALI_AFBC_16X16_PIXEL;
				/* 00:16x16 pixels
				 * 01:32x8 pixels
				 */
				u32 block_split = MALI_AFBC_SPLIT_ON;
				/*  0  Block split mode off.
				 *  1  Block split mode on.
				 */
				u32 yuv_transform = 1;
				/* 0  Internal YUV transform off.
				 * 1  Internal YUV transform on.
				 */
				#endif
				u32 afbc_color_reorder = 0x1234;
				/* 0x4321 = ABGR
				 * 0x1234 = RGBA
				 */
				#if 0
				if (osd_hw.color_info[index])
					fmt_mode = osd_hw.color_info
						[index]->color_index;
				#endif
				pixel_format = afbc_pix_format(
				osd_hw.osd_afbcd[index].format);
				bpp = osd_hw.color_info[index]->bpp >> 3;
				line_stride = line_stride_calc_afbc(
					pixel_format,
					osd_hw.osd_afbcd[index].frame_width,
					aligned_32);
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->afbc_format_specifier_s,
					(osd_hw.osd_afbcd[index].inter_format |
					(pixel_format & 0xf)));
				VSYNCOSD_WR_MPEG_REG_BITS(
					osd_reg->osd_mali_unpack_ctrl,
					afbc_color_reorder,
					0, 16);
				VSYNCOSD_WR_MPEG_REG_BITS(
					osd_reg->osd_blk2_cfg_w4,
					line_stride, 0, 12);
				output_stride =
					osd_hw.osd_afbcd[index].frame_width
					* bpp;
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->afbc_output_buf_stride_s,
					output_stride);
			}
		} else {
			if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
				/* line stride */
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->osd_blk2_cfg_w4, 0);
			}
		}
		if (idx >= COLOR_INDEX_32_BGRX
		    && idx <= COLOR_INDEX_32_XRGB) {
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat2,
				0x1ff, 6, 9);
			if (force_colorrange)
				VSYNCOSD_WR_MPEG_REG_BITS(
					osd_reg->osd_ctrl_stat2, 1, 3, 1);
			else
				VSYNCOSD_WR_MPEG_REG_BITS(
					osd_reg->osd_ctrl_stat2, 0, 3, 1);
		} else
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat2,
				0, 6, 9);
	}
	remove_from_update_list(index, OSD_COLOR_MODE);
}


static void osd_update_enable(u32 index)
{
	u32 temp_val = 0;
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];

	/*
	if (!osd_hw.buffer_alloc[index])
		return;
	*/
	if ((osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) &&
		(osd_hw.enable[index] == ENABLE)) {
		/* only for osd1 */
		if ((VSYNCOSD_RD_MPEG_REG(VPU_RDARB_MODE_L1C2) &
			(1 << 16)) == 0) {
			VSYNCOSD_WR_MPEG_REG_BITS(
				VPU_RDARB_MODE_L1C2, 1, 16, 1);
		}
	}
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		if (index == OSD1)
			temp_val = VPP_OSD1_POSTBLEND;
		else if (index == OSD2)
			temp_val = VPP_OSD2_POSTBLEND;
		if (osd_hw.enable[index] == ENABLE)
			osd_vpp_misc |= temp_val;
		else
			osd_vpp_misc &= ~temp_val;

	}
	if (osd_hw.enable[index] == ENABLE) {
		VSYNCOSD_SET_MPEG_REG_MASK(
			osd_reg->osd_ctrl_stat, 1 << 0);
		if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
			VSYNCOSD_SET_MPEG_REG_MASK(VPP_MISC,
				temp_val | VPP_POSTBLEND_EN);
			notify_to_amvideo();
		}
	} else {
		if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
			notify_to_amvideo();
			VSYNCOSD_CLR_MPEG_REG_MASK(VPP_MISC, temp_val);
		}
		if (!(osd_hw.osd_meson_dev.cpu_id ==
			__MESON_CPU_MAJOR_ID_G12B))
			VSYNCOSD_CLR_MPEG_REG_MASK(
				osd_reg->osd_ctrl_stat, 1 << 0);
	}
	if (osd_hw.osd_meson_dev.afbc_type == MESON_AFBC) {
		if ((osd_hw.osd_afbcd[index].enable == ENABLE)
			&& (osd_hw.enable[index] == ENABLE)) {
			if (!osd_afbc_dec_enable &&
				osd_hw.osd_afbcd[index].phy_addr != 0) {
				VSYNCOSD_WR_MPEG_REG(
					OSD1_AFBCD_ENABLE, 0x8100);
				osd_afbc_dec_enable = 1;
			}
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat2, 1, 15, 1);
			if ((VSYNCOSD_RD_MPEG_REG(VIU_MISC_CTRL1) &
				(0xff << 8)) != 0x9000) {
				VSYNCOSD_WR_MPEG_REG_BITS(
					VIU_MISC_CTRL1, 0x90, 8, 8);
			}
		} else {
			if (osd_afbc_dec_enable) {
				VSYNCOSD_WR_MPEG_REG(
					OSD1_AFBCD_ENABLE, 0x8000);
				osd_afbc_dec_enable = 0;
			}
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat2, 0, 15, 1);
		}
	} else if (osd_hw.osd_meson_dev.afbc_type
		== MALI_AFBC) {
		if ((osd_hw.osd_afbcd[index].enable == ENABLE)
			&& (osd_hw.enable[index] == ENABLE)
			&& !osd_hw.dim_layer[index]) {
			if (!osd_hw.osd_afbcd[index].afbc_start &&
					osd_hw.osd_afbcd[index].phy_addr != 0) {
				/* enable mali afbc */
				VSYNCOSD_WR_MPEG_REG(
					VPU_MAFBC_IRQ_MASK, 0xf);
				VSYNCOSD_WR_MPEG_REG_BITS(
					VPU_MAFBC_SURFACE_CFG,
					1, index, 1);
				osd_hw.osd_afbcd[index].afbc_start = 1;
			}
		} else {
			/* disable mali afbc */
			VSYNCOSD_WR_MPEG_REG_BITS(
				VPU_MAFBC_SURFACE_CFG,
				0, index, 1);
			osd_hw.osd_afbcd[index].afbc_start = 0;
		}
		VSYNCOSD_WR_MPEG_REG_BITS(
			osd_reg->osd_ctrl_stat2, 1, 1, 1);
	}
	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		u8 postbld_src_sel = 0;

		if (osd_hw.enable[index] == ENABLE)
			postbld_src_sel = (index == 0) ? 3 : 4;
		if (index == 0)
			VSYNCOSD_WR_MPEG_REG(
				OSD1_BLEND_SRC_CTRL,
				(0 & 0xf) << 0 |
				(0 & 0x1) << 4 |
				(postbld_src_sel & 0xf) << 8 |
				(0 & 0x1) << 16|
				(1 & 0x1) << 20);
		else if ((index == 1) && (!enable_vd_zorder))
			VSYNCOSD_WR_MPEG_REG(
				OSD2_BLEND_SRC_CTRL,
				(0 & 0xf) << 0 |
				(0 & 0x1) << 4 |
				(postbld_src_sel & 0xf) << 8 |
				(0 & 0x1) << 16 |
				(1 & 0x1) << 20);
	}
	remove_from_update_list(index, OSD_ENABLE);
}

static void osd_update_disp_osd_reverse(u32 index)
{
	if (osd_hw.osd_reverse[index] == REVERSE_TRUE) {
		VSYNCOSD_WR_MPEG_REG_BITS(
		hw_osd_reg_array[index].osd_blk0_cfg_w0, 3, 28, 2);
		if ((osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) &&
			(osd_hw.osd_afbcd[index].enable == ENABLE))
			VSYNCOSD_WR_MPEG_REG_BITS(
			hw_osd_reg_array[index].afbc_prefetch_cfg_s, 3, 0, 2);
	} else if (osd_hw.osd_reverse[index] == REVERSE_X) {
		VSYNCOSD_WR_MPEG_REG_BITS(
		hw_osd_reg_array[index].osd_blk0_cfg_w0, 1, 28, 2);
		if ((osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) &&
			(osd_hw.osd_afbcd[index].enable == ENABLE))
			VSYNCOSD_WR_MPEG_REG_BITS(
			hw_osd_reg_array[index].afbc_prefetch_cfg_s, 1, 0, 2);
	} else if (osd_hw.osd_reverse[index] == REVERSE_Y) {
		VSYNCOSD_WR_MPEG_REG_BITS(
		hw_osd_reg_array[index].osd_blk0_cfg_w0, 2, 28, 2);
		if ((osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) &&
			(osd_hw.osd_afbcd[index].enable == ENABLE))
			VSYNCOSD_WR_MPEG_REG_BITS(
			hw_osd_reg_array[index].afbc_prefetch_cfg_s, 2, 0, 2);
	} else {
		VSYNCOSD_WR_MPEG_REG_BITS(
		hw_osd_reg_array[index].osd_blk0_cfg_w0, 0, 28, 2);
		if ((osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) &&
			(osd_hw.osd_afbcd[index].enable == ENABLE))
			VSYNCOSD_WR_MPEG_REG_BITS(
			hw_osd_reg_array[index].afbc_prefetch_cfg_s, 0, 0, 2);
	}
	remove_from_update_list(index, DISP_OSD_REVERSE);
}

static int get_viu2_src_format(void)
{
	return RGBA;
}

static void osd_update_disp_osd_rotate(u32 index)
{
	u32 rotate_en = osd_hw.osd_rotate[index];
	u32 src_fmt;
	u32 x_start, x_end, y_start, y_end;
	u32 src_width, src_height;
	u32 rot_hsize, blk_vsize, rd_blk_hsize;
	u32 reg_fmt_buf_en, reg_fmt_444_mode, line5_mode;
	u32 y_reverse = 0, x_reverse = 0;
	u32 data32;
	enum color_index_e idx;
	struct dispdata_s src_data;
	const struct vinfo_s *vinfo = NULL;
	int out_y_crop_start, out_y_crop_end;

	if (osd_hw.osd_meson_dev.cpu_id != __MESON_CPU_MAJOR_ID_G12B)
		return;
	src_fmt = get_viu2_src_format();
	src_data.x = 0;
	src_data.y = 0;
	src_data.w = osd_hw.fb_gem[index].xres;
	src_data.h = osd_hw.fb_gem[index].yres;
		src_data.h = osd_hw.fb_gem[index].yres;
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vinfo = get_current_vinfo2();
	if (!vinfo) {
		osd_log_err("current vinfo NULL\n");
		return;
	}
#endif
	out_y_crop_start = 0;
	out_y_crop_end = vinfo->height;
	src_width = src_data.w;
	src_height = src_data.h;

	x_start = src_data.x;
	x_end = src_data.x + src_data.w - 1;
	y_start = src_data.y;
	y_end = src_data.y + src_data.h - 1;
	/* x/y start end can be crop axis */
	switch (src_fmt) {
	case YUV422:
		blk_vsize		  = 30;
		rd_blk_hsize	  = 32;
		reg_fmt_buf_en	  = 1;
		reg_fmt_444_mode  = 0;
		line5_mode		  = 1;
		break;
	case RGB:
		blk_vsize		  = 20;
		rd_blk_hsize	  = 22;
		reg_fmt_buf_en	  = 0;
		reg_fmt_444_mode  = 1;
		line5_mode		  = 0;
		break;
	case RGBA:
		blk_vsize		  = 15;
		rd_blk_hsize	  = 16;
		reg_fmt_buf_en	  = 0;
		reg_fmt_444_mode  = 1;
		line5_mode		  = 0;
		break;
	}
	osd_reg_set_bits(VPP2_MISC, rotate_en, 16, 1);
	rotate_en = 0;
	osd_reg_set_bits(VPU_VIU_VENC_MUX_CTRL,
		rotate_en, 20, 1);
	idx = osd_hw.color_info[index]->color_index;
	data32 = (osd_hw.fb_gem[index].canvas_idx << 16) |
		(2 << 8) | (1 << 7) | (0 << 6) |
		(y_reverse << 5) |
		(x_reverse << 4) | src_fmt;
	osd_reg_set_bits(VIU2_RMIF_CTRL1, data32, 0, 32);
	osd_reg_set_bits(VIU2_RMIF_SCOPE_X,
		x_end << 16 | x_start, 0, 32);
	osd_reg_set_bits(VIU2_RMIF_SCOPE_Y,
		y_end << 16 | y_start, 0, 32);

	rot_hsize = (src_height + blk_vsize - 1) / blk_vsize;
	osd_reg_set_bits(
		VIU2_ROT_BLK_SIZE,
		rd_blk_hsize | (blk_vsize << 8), 0, 32);
	osd_reg_set_bits(
		VIU2_ROT_LBUF_SIZE,
		rot_hsize * rd_blk_hsize |
		(rot_hsize << 16), 0, 32);
	osd_reg_set_bits(
		VIU2_ROT_FMT_CTRL,
		reg_fmt_buf_en |
		(line5_mode << 1) |
		(reg_fmt_444_mode << 2) |
		(180	<< 8) | (0xe4 << 24), 0, 32);
	osd_reg_set_bits(VIU2_ROT_OUT_VCROP,
		out_y_crop_end |
		out_y_crop_start << 16, 0, 32);
	osd_reg_set_bits(
		VPP2_OFIFO_SIZE,
		src_height - 1, 20, 12);
	remove_from_update_list(index, DISP_OSD_ROTATE);
}

static void osd_update_color_key(u32 index)
{
	VSYNCOSD_WR_MPEG_REG(hw_osd_reg_array[index].osd_tcolor_ag0,
		osd_hw.color_key[index]);
	remove_from_update_list(index, OSD_COLOR_KEY);
}

static void osd_update_color_key_enable(u32 index)
{
	u32  data32;

	data32 = VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[index].osd_blk0_cfg_w0);
	data32 &= ~(1 << 6);
	data32 |= (osd_hw.color_key_enable[index] << 6);
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[index].osd_blk0_cfg_w0, data32);
	/*
	 * VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK1_CFG_W0, data32);
	 * VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK2_CFG_W0, data32);
	 * VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK3_CFG_W0, data32);
	 */
	remove_from_update_list(index, OSD_COLOR_KEY_ENABLE);
}

static void osd_update_gbl_alpha(u32 index)
{
	u32  data32;

	data32 = VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[index].osd_ctrl_stat);
	data32 &= ~(0x1ff << 12);
	data32 |= osd_hw.gbl_alpha[index] << 12;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[index].osd_ctrl_stat, data32);
	remove_from_update_list(index, OSD_GBL_ALPHA);
}

static void osd_update_order(u32 index)
{
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		switch (osd_hw.order[index]) {
		case  OSD_ORDER_01:
			if (!osd_hw.hw_rdma_en)
			osd_reg_clr_mask(VPP_MISC,
				VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			osd_vpp_misc &= ~(VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			break;
		case  OSD_ORDER_10:
			/* OSD2 is top */
			if (!osd_hw.hw_rdma_en)
				osd_reg_set_mask(VPP_MISC,
				VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			osd_vpp_misc |= (VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			break;
		default:
			break;
		}
		notify_to_amvideo();
	}

	remove_from_update_list(index, OSD_CHANGE_ORDER);
}


static void osd1_2x_scale_update_geometry(void)
{
	u32 data32;

	data32 = (osd_hw.scaledata[OSD1].x_start & 0x1fff) |
		 (osd_hw.scaledata[OSD1].x_end & 0x1fff) << 16;
	VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK0_CFG_W1, data32);

	if (osd_hw.osd_afbcd[OSD1].enable) {
		data32 = (osd_hw.scaledata[OSD1].x_end & 0x1fff) |
			(osd_hw.scaledata[OSD1].x_start & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_PIXEL_HSCOPE, data32);
		data32 = (osd_hw.scaledata[OSD1].y_end & 0x1fff) |
			(osd_hw.scaledata[OSD1].y_start & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_PIXEL_VSCOPE, data32);
		VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_HDR_PTR,
			osd_hw.osd_afbcd[OSD1].phy_addr >> 4);
		VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_FRAME_PTR,
			osd_hw.osd_afbcd[OSD1].phy_addr >> 4);
		data32 = (0xe4 << 24) |
			(osd_hw.osd_afbcd[OSD1].phy_addr & 0xffffff);
		VSYNCOSD_WR_MPEG_REG(
			OSD1_AFBCD_CHROMA_PTR,
			data32);
	}

	data32 = ((osd_hw.scaledata[OSD1].y_start
			+ osd_hw.pandata[OSD1].y_start) & 0x1fff)
		 | ((osd_hw.scaledata[OSD1].y_end
			+ osd_hw.pandata[OSD1].y_start) & 0x1fff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[OSD1].osd_blk0_cfg_w2, data32);
	/* adjust display x-axis */
	if (osd_hw.scale[OSD1].h_enable) {
		data32 = (osd_hw.dispdata[OSD1].x_start & 0xfff)
			 | ((osd_hw.dispdata[OSD1].x_start
				+ (osd_hw.scaledata[OSD1].x_end
				- osd_hw.scaledata[OSD1].x_start) * 2 + 1)
					 & 0xfff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w3, data32);
		if (osd_hw.scan_mode[OSD1] == SCAN_MODE_INTERLACE) {
			data32 = ((osd_hw.dispdata[OSD1].y_start >> 1) & 0xfff)
				 | (((((osd_hw.dispdata[OSD1].y_start
					+ (osd_hw.scaledata[OSD1].y_end
					- osd_hw.scaledata[OSD1].y_start) * 2)
					+ 1) >> 1) - 1) & 0xfff) << 16;
		} else {
			data32 = (osd_hw.dispdata[OSD1].y_start & 0xfff)
				 | (((osd_hw.dispdata[OSD1].y_start
					+ (osd_hw.scaledata[OSD1].y_end
					- osd_hw.scaledata[OSD1].y_start) * 2))
						 & 0xfff) << 16;
		}
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w4, data32);
	}
	/* adjust display y-axis */
	if (osd_hw.scale[OSD1].v_enable) {
		data32 = (osd_hw.dispdata[OSD1].x_start & 0xfff)
			 | ((osd_hw.dispdata[OSD1].x_start
				+ (osd_hw.scaledata[OSD1].x_end
				- osd_hw.scaledata[OSD1].x_start) * 2 + 1)
					 & 0xfff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w3, data32);
		if (osd_hw.scan_mode[OSD1] == SCAN_MODE_INTERLACE) {
			data32 = ((osd_hw.dispdata[OSD1].y_start >> 1) & 0xfff)
				 | (((((osd_hw.dispdata[OSD1].y_start
					+ (osd_hw.scaledata[OSD1].y_end
					- osd_hw.scaledata[OSD1].y_start) * 2)
					+ 1) >> 1) - 1) & 0xfff) << 16;
		} else {
			data32 = (osd_hw.dispdata[OSD1].y_start & 0xfff)
				 | (((osd_hw.dispdata[OSD1].y_start
					+ (osd_hw.scaledata[OSD1].y_end
					- osd_hw.scaledata[OSD1].y_start) * 2))
						 & 0xfff) << 16;
		}
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w4, data32);
	}
}

static u32 _set_bits(u32 data32, u32 bits, u32 value)
{
	data32 &= ~(1 << bits);
	data32 |= (value << bits);
	return data32;
}

void insert_sort(u32 *array, int length)
{
	int i = 0, j = 0;
	u32 temp;

	/* decrease */
	for (j = 0; j < length; j++) {
		for (i = j + 1; i <= length; i++) {
			if (array[j] < array[i]) {
				temp = array[j];
				array[j] = array[i];
				array[i] = temp;
			}
		}
	}
}

static int get_available_layers(void)
{
	int i;
	int available_layer = 0;

	for (i = 0 ; i < osd_hw.osd_meson_dev.viu1_osd_count; i++) {
		if (osd_hw.enable[i])
			available_layer++;
	}
	return available_layer;
}

static u32 blend_din_to_osd(
	u32 blend_din_index, struct hw_osd_blending_s *blending)
{
	u32 osd_index = 0;

	osd_index =
		blending->osd_to_bdin_table[blend_din_index];
	if (osd_index >= osd_hw.osd_meson_dev.viu1_osd_count)
		return OSD_MAX;
	else
		return osd_index;
}

static u32 get_max_order(u32 order1, u32 order2)
{
	u32 max_order = 0;

	if (order1 > order2)
		max_order = order1;
	else
		max_order = order2;
	return max_order;
}

static u32 get_min_order(u32 order1, u32 order2)
{
	u32 min_order = 0;

	if (order1 > order2)
		min_order = order2;
	else
		min_order = order1;
	return min_order;
}

static u32 get_max_order_to_osd(struct hw_osd_blending_s *blending)
{
	u32 index = 0, max_order = 0;

	max_order = get_max_order(blending->reorder[OSD1],
		blending->reorder[OSD2]);
	max_order = get_max_order(max_order,
		blending->reorder[OSD3]);

	if (max_order == blending->reorder[OSD1])
		index = OSD1;
	else if (max_order == blending->reorder[OSD2])
		index = OSD2;
	else if (max_order == blending->reorder[OSD3])
		index = OSD3;
	return index;
}

static u32 get_min_order_to_osd(struct hw_osd_blending_s *blending)
{
	u32 index = 0, min_order = 0;

	min_order = get_min_order(blending->reorder[OSD1],
		blending->reorder[OSD2]);
	min_order = get_min_order(min_order,
		blending->reorder[OSD3]);

	if (min_order == blending->reorder[OSD1])
		index = OSD1;
	else if (min_order == blending->reorder[OSD2])
		index = OSD2;
	else if (min_order == blending->reorder[OSD3])
		index = OSD3;
	return index;
}

static u32 get_middle_order_to_osd(struct hw_osd_blending_s *blending)
{
	u32 index = 0;

	if (blending->reorder[OSD1] == LAYER_2)
		index = OSD1;
	else if (blending->reorder[OSD2] == LAYER_2)
		index = OSD2;
	else if (blending->reorder[OSD3] == LAYER_2)
		index = OSD3;
	return index;
}

static void exchange_din0_din2(struct hw_osd_blending_s *blending)
{
	int temp1 = 0, temp2 = 0;

	osd_log_dbg(MODULE_BLEND, "need exchange osd din0 and din2 order\n");
	temp1 = blending->din_reoder_sel & 0x000f;
	temp2 = blending->din_reoder_sel & 0x0f00;
	blending->din_reoder_sel &= ~0x0f0f;
	blending->din_reoder_sel |= temp1 << 8;
	blending->din_reoder_sel |= temp2 >> 8;
	temp1 = blending->osd_to_bdin_table[0];
	temp2 = blending->osd_to_bdin_table[2];
	blending->osd_to_bdin_table[2] = temp1;
	blending->osd_to_bdin_table[0] = temp2;
	osd_log_dbg(MODULE_BLEND, "din_reoder_sel%x\n",
		blending->din_reoder_sel);
}

static void exchange_din2_din3(struct hw_osd_blending_s *blending)
{
	int temp1 = 0, temp2 = 0;

	osd_log_dbg(MODULE_BLEND, "need exchange osd din2 and din3 order\n");
	temp1 = blending->din_reoder_sel & 0x0f00;
	temp2 = blending->din_reoder_sel & 0xf000;
	blending->din_reoder_sel &= ~0xff00;
	blending->din_reoder_sel |= temp1 << 4;
	blending->din_reoder_sel |= temp2 >> 4;
	temp1 = blending->osd_to_bdin_table[2];
	temp2 = blending->osd_to_bdin_table[3];
	blending->osd_to_bdin_table[3] = temp1;
	blending->osd_to_bdin_table[2] = temp2;
	osd_log_dbg(MODULE_BLEND, "din_reoder_sel%x\n",
		blending->din_reoder_sel);
}

static void exchange_vpp_order(struct hw_osd_blending_s *blending)
{
	blending->b_exchange_blend_in = true;
	osd_log_dbg(MODULE_BLEND, "need exchange vpp order\n");
}

static void generate_blend_din_table(struct hw_osd_blending_s *blending)
{
	int i = 0;
	int temp1 = 0, temp2 = 0;
	int osd_count = osd_hw.osd_meson_dev.viu1_osd_count;

	/* reorder[i] = osd[i]'s display layer */
	for (i = 0; i < OSD_BLEND_LAYERS; i++)
		blending->osd_to_bdin_table[i] = -1;
	blending->din_reoder_sel = 0;
	switch (blending->layer_cnt) {
	case 0:
		break;
	case 1:
		for (i = 0; i < osd_count; i++) {
			if (blending->reorder[i] != LAYER_UNSUPPORT) {
				/* blend_din1 */
				blending->din_reoder_sel |= (i + 1) << 0;
				/* blend_din1 -- osdx */
				blending->osd_to_bdin_table[0] = i;
				break;
			}
		}
		break;
	case 2:
	{
		int temp_index[2] = {0};
		int j = 0;

		for (i = 0; i < osd_count; i++) {
			if (blending->reorder[i] != LAYER_UNSUPPORT) {
				/* save the osd index */
				temp_index[j] = i;
				j++;
			}
		}
		osd_log_dbg(MODULE_BLEND, "blend_din4==%d\n",
			blending->reorder[temp_index[0]]);
		osd_log_dbg(MODULE_BLEND, "blend_din1==%d\n",
			blending->reorder[temp_index[1]]);
		/* mode A_C */
		if (blending->osd_blend_mode == OSD_BLEND_A_C) {
			/* blend_din1 */
			blending->din_reoder_sel |= (temp_index[0] + 1) << 0;
			/* blend_din1 -- osdx */
			blending->osd_to_bdin_table[0] = temp_index[0];
			/* blend_din3 */
			blending->din_reoder_sel |= (temp_index[1] + 1) << 12;
			/* blend_din3 -- osdx */
			blending->osd_to_bdin_table[3] = temp_index[1];
			/* exchane vpp osd blend in order */
			if (blending->reorder[temp_index[0]] <
				blending->reorder[temp_index[1]]) {
				blending->b_exchange_blend_in = true;
				osd_log_dbg(MODULE_BLEND, "need exchange vpp order\n");
			}
		} else {
			if (blending->reorder[temp_index[0]] <
				blending->reorder[temp_index[1]]) {
				/* blend_din4 */
				blending->din_reoder_sel |=
					(temp_index[0] + 1) << 12;
				/* blend_din3 -- osdx */
				blending->osd_to_bdin_table[3] = temp_index[0];
				/* blend_din1 */
				blending->din_reoder_sel |=
					(temp_index[1] + 1) << 0;
				/* blend_din1 -- osdx */
				blending->osd_to_bdin_table[0] = temp_index[1];
				blending->b_exchange_din = true;
				osd_log_dbg(MODULE_BLEND, "need exchange osd din order\n");
			} else {
				/* blend_din1 */
				blending->din_reoder_sel |=
					(temp_index[0] + 1) << 0;
				/* blend_din1 -- osdx */
				blending->osd_to_bdin_table[0] = temp_index[0];
				/* blend_din3 */
				blending->din_reoder_sel |=
					(temp_index[1] + 1) << 12;
				/* blend_din3 -- osdx */
				blending->osd_to_bdin_table[3] = temp_index[1];
			}
		}
		break;
	}
	case 3:
		/* blend_din1 is bottom, blend_din4 is top layer */
		osd_log_dbg(MODULE_BLEND, "reorder:%d,%d,%d\n",
			blending->reorder[OSD1],
			blending->reorder[OSD2],
			blending->reorder[OSD3]);
		if (blending->osd_blend_mode == OSD_BLEND_AB_C) {
			/* suppose osd0 used blend_din1,
			 * osd1 used din2, osd3 used din4
			 */
			/* blend_din1 */
			blending->din_reoder_sel |= 1 << 0;
			/* blend_din1 -- osd1 */
			blending->osd_to_bdin_table[0] = OSD1;
			/* blend_din3 */
			blending->din_reoder_sel |= (OSD2 + 1) << 8;
			/* blend_din3 -- osd2 */
			blending->osd_to_bdin_table[2] = OSD2;
			/* blend_din4 */
			blending->din_reoder_sel |= (OSD3 + 1) << 12;
			/* blend_din4 -- osd3 */
			blending->osd_to_bdin_table[3] = OSD3;
			if ((blending->reorder[OSD1] == LAYER_3)
				&& (blending->reorder[OSD2] == LAYER_2)
				&& (blending->reorder[OSD3] == LAYER_1)) {
				osd_log_dbg2(MODULE_BLEND,
					"use default order\n");
			} else if ((blending->reorder[OSD1] == LAYER_2)
				&& (blending->reorder[OSD2] == LAYER_3)
				&& (blending->reorder[OSD3] == LAYER_1)) {
				exchange_din0_din2(blending);
			} else if ((blending->reorder[OSD1] == LAYER_2)
				&& (blending->reorder[OSD2] == LAYER_1)
				&& (blending->reorder[OSD3] == LAYER_3)) {
				exchange_vpp_order(blending);
			} else if ((blending->reorder[OSD1] == LAYER_1)
				&& (blending->reorder[OSD2] == LAYER_2)
				&& (blending->reorder[OSD3] == LAYER_3)) {
				exchange_din0_din2(blending);
				exchange_vpp_order(blending);
			} else if ((blending->reorder[OSD1] == LAYER_3)
				&& (blending->reorder[OSD2] == LAYER_1)
				&& (blending->reorder[OSD3] == LAYER_2)) {
				exchange_din2_din3(blending);
			} else if ((blending->reorder[OSD1] == LAYER_1)
				&& (blending->reorder[OSD2] == LAYER_3)
				&& (blending->reorder[OSD3] == LAYER_2)) {
				exchange_din2_din3(blending);
				exchange_din0_din2(blending);
				exchange_vpp_order(blending);
			}
		} else if (blending->osd_blend_mode == OSD_BLEND_ABC) {
			u32 osd_index = 0;

			osd_index = get_max_order_to_osd(blending);
			/* blend_din1 is max_order*/
			blending->din_reoder_sel |= (osd_index + 1) << 0;
			blending->osd_to_bdin_table[0] = osd_index;

			osd_index = get_min_order_to_osd(blending);
			/* blend_din4 is min_order*/
			blending->din_reoder_sel |= (osd_index + 1) << 12;
			blending->osd_to_bdin_table[3] = osd_index;

			osd_index = get_middle_order_to_osd(blending);
			/* blend_din3 is middle_order*/
			blending->din_reoder_sel |= (osd_index + 1) << 8;
			blending->osd_to_bdin_table[2] = osd_index;
		}
		break;
	}

	/* workaround for shift issue */
	/* not enable layers used osd4 */
	temp2 = 0;
	for (i = 0; i < OSD_BLEND_LAYERS; i++) {
		temp1 = (blending->din_reoder_sel >> (i*4)) & 0xf;
		if (!(temp1 & 0x0f))
			temp2 |= (0x04 << (i*4));
	}
	blending->din_reoder_sel |= temp2;
	osd_log_dbg(MODULE_BLEND, "osd_to_bdin_table[i]=[%x,%x,%x,%x]\n",
		blending->osd_to_bdin_table[0],
		blending->osd_to_bdin_table[1],
		blending->osd_to_bdin_table[2],
		blending->osd_to_bdin_table[3]);
	blending->blend_reg.din_reoder_sel =
		blending->din_reoder_sel;
}

#ifdef FREESCAL_CHECK
static bool is_freescale_para_changed(u32 index)
{
	static int first[HW_OSD_COUNT - 1] = {1};
	bool freescale_update = false;

	if (memcmp(&(osd_hw.free_src_data[index]),
		&osd_hw.free_src_data_backup[index],
		sizeof(struct pandata_s)) != 0 ||
		memcmp(&(osd_hw.free_dst_data[index]),
		&osd_hw.free_dst_data_backup[index],
		sizeof(struct pandata_s)) != 0) {
		memcpy(&(osd_hw.free_src_data_backup[index]),
			&osd_hw.free_src_data[index],
			sizeof(struct pandata_s));
		memcpy(&(osd_hw.free_dst_data_backup[index]),
			&osd_hw.free_dst_data[index],
			sizeof(struct pandata_s));
		freescale_update = true;
	}
	if (first[index])
		freescale_update = true;
	first[index] = 0;
	return freescale_update;
}
#endif

static int osd_setting_blending_scope(u32 index)
{
	u32 bld_osd_h_start, bld_osd_h_end;
	u32 bld_osd_v_start, bld_osd_v_end;
	u32 reg_offset = 2;

	if (index >= osd_hw.osd_meson_dev.viu1_osd_count) {
		osd_log_err("error osd index=%d\n", index);
		return -1;
	}
	bld_osd_h_start =
		osd_hw.dst_data[index].x;
	bld_osd_h_end =
		osd_hw.dst_data[index].x +
		osd_hw.dst_data[index].w - 1;
	bld_osd_v_start =
		osd_hw.dst_data[index].y;
	bld_osd_v_end =
		osd_hw.dst_data[index].y +
		osd_hw.dst_data[index].h - 1;

	osd_log_dbg(MODULE_BLEND, "osd%d_hw.dst_data:%d,%d,%d,%d\n",
		index,
		osd_hw.dst_data[index].x,
		osd_hw.dst_data[index].y,
		osd_hw.dst_data[index].w,
		osd_hw.dst_data[index].h);
	osd_log_dbg(MODULE_BLEND, "h,v axis:%d,%d,%d,%d\n",
		bld_osd_h_start,
		bld_osd_h_end,
		bld_osd_v_start,
		bld_osd_v_end);

	/* it is setting for osdx */
	VSYNCOSD_WR_MPEG_REG(
		VIU_OSD_BLEND_DIN0_SCOPE_H + reg_offset * index,
		bld_osd_h_end << 16 |
		bld_osd_h_start);
	VSYNCOSD_WR_MPEG_REG(
		VIU_OSD_BLEND_DIN0_SCOPE_V + reg_offset * index,
		bld_osd_v_end << 16 |
		bld_osd_v_start);
	return 0;
}

static int vpp_blend_setting_default(u32 index)
{
	u32 osd1_dst_h = 0, osd1_dst_v = 0;
	u32 osd1_h_start = 0, osd1_h_end = 0;
	u32 osd1_v_start = 0, osd1_v_end = 0;

	osd_log_dbg(MODULE_BASE, "vpp_blend_setting_default\n");

	if (index >= osd_hw.osd_meson_dev.viu1_osd_count) {
		osd_log_err("error osd index=%d\n", index);
		return -1;
	}
	if (osd_hw.free_scale_enable[index]) {
		osd1_dst_h = osd_hw.free_dst_data[index].x_end
			- osd_hw.free_dst_data[index].x_start + 1;
		osd1_dst_v = osd_hw.free_dst_data[index].y_end
			- osd_hw.free_dst_data[index].y_start + 1;
		osd1_h_start = osd_hw.free_dst_data[index].x_start;
		osd1_h_end = osd_hw.free_dst_data[index].x_end;
		osd1_v_start = osd_hw.free_dst_data[index].y_start;
		osd1_v_end = osd_hw.free_dst_data[index].y_end;
	} else {
		osd1_dst_h = osd_hw.dispdata[index].x_end
			- osd_hw.dispdata[index].x_start + 1;
		osd1_dst_v = osd_hw.dispdata[index].y_end
			- osd_hw.dispdata[index].y_start + 1;
		osd1_h_start = osd_hw.dispdata[index].x_start;
		osd1_h_end = osd_hw.dispdata[index].x_end;
		osd1_v_start = osd_hw.dispdata[index].y_start;
		osd1_v_end = osd_hw.dispdata[index].y_end;
	}

	/* setting blend scope */
	VSYNCOSD_WR_MPEG_REG(VPP_OSD1_BLD_H_SCOPE,
		osd1_h_start << 16 | osd1_h_end);
	VSYNCOSD_WR_MPEG_REG(VPP_OSD1_BLD_V_SCOPE,
		osd1_v_start << 16 | osd1_v_end);

	return 0;
}


static u32 order[HW_OSD_COUNT];
static struct hw_osd_blending_s osd_blending;

static void set_blend_order(struct hw_osd_blending_s *blending)
{
	u32 org_order[HW_OSD_COUNT];
	int i = 0, j = 0;
	u32 osd_count = osd_hw.osd_meson_dev.viu1_osd_count;

	if (!blending)
		return;
	/* number largest is top layer */
	for (i = 0; i < osd_count; i++) {
		org_order[i] = osd_hw.order[i];
		if (!osd_hw.enable[i])
			org_order[i] = 0;
		order[i] = org_order[i];
	}
	insert_sort(order, osd_count);
	//check_order_continuous(order);
	osd_log_dbg2(MODULE_BLEND, "after sort:zorder:%d,%d,%d\n",
		order[0], order[1], order[2]);

	/* reorder[i] = osd[i]'s display layer */
	for (i = 0; i < osd_count; i++) {
		for (j = 0; j < osd_count; j++) {
			if (order[i] == org_order[j]) {
				if (osd_hw.enable[j])
					blending->reorder[j] = LAYER_1 + i;
				else
					blending->reorder[j] =
					LAYER_UNSUPPORT;
			}
		}
	}
	osd_log_dbg2(MODULE_BLEND, "after reorder:zorder:%d,%d,%d\n",
		blending->reorder[0],
		blending->reorder[1],
		blending->reorder[2]);
}

static void set_blend_din(struct hw_osd_blending_s *blending)
{
	int i = 0, osd_index;
	/* workaround for shift issue */
	/* blend_din_en must equal 5 */
	u32 blend_din_en = 0x5;

	if (!blending)
		return;
	for (i = 0; i < OSD_BLEND_LAYERS; i++) {
		/* find osd index */
		osd_index = blend_din_to_osd(i, blending);
		if ((osd_index >= OSD1) && (osd_index <= OSD3)) {
			/* depend on osd enable */
			blend_din_en = _set_bits(
				blend_din_en,
				i,
			osd_hw.enable[osd_index]);
		}
	}
	blending->blend_reg.blend_din_en = blend_din_en;
}

static void set_blend_mode(struct hw_osd_blending_s *blending)
{
	u8 osd_blend_mode = OSD_BLEND_NONE;

	if (!blending)
		return;
	switch (blending->layer_cnt) {
	case 0:
		osd_blend_mode = OSD_BLEND_NONE;
		break;
	case 1:
		/* select blend_din1 always */
		osd_blend_mode = OSD_BLEND_A;
		break;
	case 2:
		/* select blend_din1, blend_din4 always */
		if (osd_hw.hdr_used)
			osd_blend_mode = OSD_BLEND_AC;
		else
			osd_blend_mode = OSD_BLEND_A_C;
		break;
	case 3:
		/* select blend_din1, blend_din3 blend_din4 always */
		if (osd_hw.hdr_used)
			osd_blend_mode = OSD_BLEND_ABC;
		else
			osd_blend_mode = OSD_BLEND_AB_C;
		break;
	}
	blending->osd_blend_mode = osd_blend_mode;
	osd_log_dbg2(MODULE_BLEND, "osd_blend_mode=%d\n",
		blending->osd_blend_mode);
}

static void calc_max_output(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	u32 input1_end_x, input1_end_y;
	u32 input2_end_x, input2_end_y;
	u32 output_end_x, output_end_y;

	if (!blending)
		return;
	layer_blend = &(blending->layer_blend);
	osd_log_dbg2(MODULE_BLEND, "calc_max_output input1_data:%d,%d,%d,%d\n",
		layer_blend->input1_data.x,
		layer_blend->input1_data.y,
		layer_blend->input1_data.w,
		layer_blend->input1_data.h);
	osd_log_dbg2(MODULE_BLEND, "calc_max_output input2_data:%d,%d,%d,%d\n",
		layer_blend->input2_data.x,
		layer_blend->input2_data.y,
		layer_blend->input2_data.w,
		layer_blend->input2_data.h);

	input1_end_x = layer_blend->input1_data.x +
		layer_blend->input1_data.w - 1;
	input1_end_y = layer_blend->input1_data.y +
		layer_blend->input1_data.h - 1;
	input2_end_x = layer_blend->input2_data.x +
		layer_blend->input2_data.w - 1;
	input2_end_y = layer_blend->input2_data.y +
		layer_blend->input2_data.h - 1;
	if (layer_blend->input1_data.x < layer_blend->input2_data.x)
		layer_blend->output_data.x = layer_blend->input1_data.x;
	else
		layer_blend->output_data.x = layer_blend->input2_data.x;
	if (layer_blend->input1_data.y < layer_blend->input2_data.y)
		layer_blend->output_data.y = layer_blend->input1_data.y;
	else
		layer_blend->output_data.y = layer_blend->input2_data.y;

	if (input1_end_x < input2_end_x)
		output_end_x = input2_end_x;
	else
		output_end_x = input1_end_x;
	if (input1_end_y < input2_end_y)
		output_end_y = input2_end_y;
	else
		output_end_y = input1_end_y;
	layer_blend->output_data.x = 0;
	layer_blend->output_data.y = 0;
	layer_blend->output_data.w = output_end_x -
		layer_blend->output_data.x + 1;
	layer_blend->output_data.h = output_end_y -
		layer_blend->output_data.y + 1;
}

static void osd_setting_blend0(struct hw_osd_blending_s *blending)
{
	struct layer_blend_reg_s *blend_reg;
	struct layer_blend_s *layer_blend;
	u32 index = 0;
	u32 bld_osd_h_start = 0, bld_osd_h_end = 0;
	u32 bld_osd_v_start = 0, bld_osd_v_end = 0;

	if (!blending)
		return;
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	/* blend0 only accept input1 */
	if (layer_blend->input1 & BYPASS_DIN) {
		blend_reg->din0_byp_blend = 1;
		layer_blend->input1 &= ~BYPASS_DIN;
	} else
		blend_reg->din0_byp_blend = 0;
	if (layer_blend->input1 != BLEND_NO_DIN) {
		/* calculate osd blend din scope */
		index = blend_din_to_osd(layer_blend->input1, blending);
		if (index >= OSD_MAX)
			return;
		bld_osd_h_start =
			layer_blend->input1_data.x;
		bld_osd_h_end =
			layer_blend->input1_data.x +
			layer_blend->input1_data.w - 1;
		bld_osd_v_start =
			layer_blend->input1_data.y;
		bld_osd_v_end =
			layer_blend->input1_data.y +
			layer_blend->input1_data.h - 1;
		blend_reg->osd_blend_din_scope_h[index] =
			bld_osd_h_end << 16 | bld_osd_h_start;
		blend_reg->osd_blend_din_scope_v[index] =
			bld_osd_v_end << 16 | bld_osd_v_start;
	}
	layer_blend->output_data.x = 0;
	layer_blend->output_data.y = 0;
	layer_blend->output_data.w = layer_blend->input1_data.x
		+ layer_blend->input1_data.w;
	layer_blend->output_data.h = layer_blend->input1_data.y
		+ layer_blend->input1_data.h;
	osd_log_dbg2(MODULE_BLEND,
		"blend0:input1_data[osd%d]:%d,%d,%d,%d\n",
		index,
		layer_blend->input1_data.x,
		layer_blend->input1_data.y,
		layer_blend->input1_data.w,
		layer_blend->input1_data.h);
	osd_log_dbg2(MODULE_BLEND,
		"blend0:layer_blend->output_data:%d,%d,%d,%d\n",
		layer_blend->output_data.x,
		layer_blend->output_data.y,
		layer_blend->output_data.w,
		layer_blend->output_data.h);
}

static void osd_setting_blend1(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	u32 index = 0;
	u32 blend_hsize, blend_vsize;
	u32 bld_osd_h_start = 0, bld_osd_h_end = 0;
	u32 bld_osd_v_start = 0, bld_osd_v_end = 0;
	u32 workaround_line = 0;
	/* for g12a blend shift issue */

	if (!blending)
		return;
	if (osd_hw.hdr_used)
		workaround_line = osd_hw.workaround_line;
	else {
		if (blending->layer_cnt == 2)
			workaround_line = 0;
		else
			workaround_line = osd_hw.workaround_line;
	}
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	/* blend1_dout to blend2 */
	blend_reg->din2_osd_sel = 0;

	if (layer_blend->input2 & BYPASS_DIN) {
		/* blend1_din3 bypass to dout1 */
		blend_reg->din3_osd_sel = 1;
		layer_blend->input2 &= ~BYPASS_DIN;
	} else
		/* blend1_din3 input to blend1 */
		blend_reg->din3_osd_sel = 0;

	if (layer_blend->input1 != BLEND_NO_DIN) {
		index = blend_din_to_osd(layer_blend->input1, blending);
		if (index >= OSD_MAX)
			return;
		/* calculate osd blend din scope */
		bld_osd_h_start =
			layer_blend->input1_data.x;
		bld_osd_h_end =
			layer_blend->input1_data.x +
			layer_blend->input1_data.w - 1;
		bld_osd_v_start =
			layer_blend->input1_data.y;
		bld_osd_v_end =
			layer_blend->input1_data.y +
			layer_blend->input1_data.h - 1;
		blend_reg->osd_blend_din_scope_h[index] =
			bld_osd_h_end << 16 | bld_osd_h_start;
		blend_reg->osd_blend_din_scope_v[index] =
			bld_osd_v_end << 16 | bld_osd_v_start;
		osd_log_dbg2(MODULE_BLEND,
			"blend1:input1_data(osd%d):%d,%d,%d,%d\n",
			index,
			layer_blend->input1_data.x,
			layer_blend->input1_data.y,
			layer_blend->input1_data.w,
			layer_blend->input1_data.h);
	}
	if (layer_blend->input2 != BLEND_NO_DIN) {
		index = blend_din_to_osd(layer_blend->input2, blending);
		if (index >= OSD_MAX)
			return;
		/* calculate osd blend din scope */
		bld_osd_h_start =
			layer_blend->input2_data.x;
		bld_osd_h_end =
			layer_blend->input2_data.x +
			layer_blend->input2_data.w - 1;
		bld_osd_v_start =
			layer_blend->input2_data.y;
		bld_osd_v_end =
			layer_blend->input2_data.y +
			layer_blend->input2_data.h - 1;
		blend_reg->osd_blend_din_scope_h[index] =
			bld_osd_h_end << 16 | bld_osd_h_start;
		blend_reg->osd_blend_din_scope_v[index] =
			bld_osd_v_end << 16 | bld_osd_v_start;
		osd_log_dbg2(MODULE_BLEND,
			"layer_blend->input2_data:%d,%d,%d,%d\n",
			layer_blend->input2_data.x,
			layer_blend->input2_data.y,
			layer_blend->input2_data.w,
			layer_blend->input2_data.h);
	}
	if (blend_reg->din3_osd_sel || layer_blend->input1 == BLEND_NO_DIN) {
		/* blend din3 bypass,output == input */
	if (layer_blend->input2 == BLEND_NO_DIN) {
		memcpy(&layer_blend->output_data,
			&layer_blend->input1_data,
			sizeof(struct dispdata_s));
	} else {
		memcpy(&layer_blend->output_data,
			&layer_blend->input2_data,
			sizeof(struct dispdata_s));
	}
	layer_blend->output_data.h += workaround_line;

	} else
		calc_max_output(blending);
	blend_hsize = layer_blend->output_data.w;
	blend_vsize = layer_blend->output_data.h;
	blend_reg->osd_blend_blend1_size =
		blend_vsize  << 16 | blend_hsize;

	osd_log_dbg2(MODULE_BLEND, "layer_blend1->output_data:%d,%d,%d,%d\n",
		layer_blend->output_data.x,
		layer_blend->output_data.y,
		layer_blend->output_data.w,
		layer_blend->output_data.h);
}

static void osd_setting_blend2(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	u32 blend_hsize, blend_vsize;
	int blend1_input = 0;

	if (!blending)
		return;
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);

	blend_hsize = layer_blend->input1_data.w;
	blend_vsize = layer_blend->input1_data.h;
	/* always used input1_data */
	/* osd_blend_blend0_size share with blend2_size*/
	blend_reg->osd_blend_blend0_size =
		blend_vsize  << 16 | blend_hsize;
	switch (layer_blend->input2) {
	case BLEND1_DIN:
		blend1_input = 0;
		break;
	default:
		/* blend1_dout to dout1 */
		blend1_input = 1;
		break;
	}

	/* premult set */
	blend_reg->blend2_premult_en = 3;
	if (blend1_input)
		memcpy(&(layer_blend->output_data), &(layer_blend->input1_data),
			sizeof(struct dispdata_s));
	else {
		calc_max_output(blending);
		blend_hsize = layer_blend->output_data.w;
		blend_vsize = layer_blend->output_data.h;
		blend_reg->osd_blend_blend0_size =
			blend_vsize  << 16 | blend_hsize;
		/* blend 0 and blend1 size need same */
		blend_reg->osd_blend_blend1_size =
			blend_reg->osd_blend_blend0_size;
	}
	osd_log_dbg2(MODULE_BLEND, "layer_blend2->output_data:%d,%d,%d,%d\n",
		layer_blend->output_data.x,
		layer_blend->output_data.y,
		layer_blend->output_data.w,
		layer_blend->output_data.h);
}

static void vpp_setting_blend(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	u32 osd1_h_start = 0, osd1_h_end = 0;
	u32 osd1_v_start = 0, osd1_v_end = 0;
	u32 osd2_h_start = 0, osd2_h_end = 0;
	u32 osd2_v_start = 0, osd2_v_end = 0;

	if (!blending)
		return;
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);

	osd1_h_start = layer_blend->input1_data.x;
	osd1_h_end = layer_blend->input1_data.x +
		layer_blend->input1_data.w - 1;
	osd1_v_start = layer_blend->input1_data.y;
	osd1_v_end = layer_blend->input1_data.y +
		layer_blend->input1_data.h - 1;
	osd2_h_start = layer_blend->input2_data.x;
	osd2_h_end = layer_blend->input2_data.x +
		layer_blend->input2_data.w - 1;
	osd2_v_start = layer_blend->input2_data.y;
	osd2_v_end = layer_blend->input2_data.y +
		layer_blend->input2_data.h - 1;
	/* vpp osd1 postblend scope */
	switch (layer_blend->input1) {
	case BLEND1_DIN:
		blend_reg->postbld_src3_sel = POSTBLD_OSD2;
		if (layer_blend->blend_core1_bypass)
			blend_reg->postbld_osd1_premult = 0;
		else
			blend_reg->postbld_osd1_premult = 1;
		blend_reg->vpp_osd1_blend_h_scope =
			(osd2_h_start << 16 | osd2_h_end);
		blend_reg->vpp_osd1_blend_v_scope =
			(osd2_v_start << 16 | osd2_v_end);
		break;
	case BLEND2_DIN:
		blend_reg->postbld_src3_sel = POSTBLD_OSD1;
		if (blend_reg->din0_byp_blend)
			blend_reg->postbld_osd1_premult = 0;
		else
			blend_reg->postbld_osd1_premult = 1;
		blend_reg->vpp_osd1_blend_h_scope =
			(osd1_h_start << 16 | osd1_h_end);
		blend_reg->vpp_osd1_blend_v_scope =
			(osd1_v_start << 16 | osd1_v_end);
		break;
	default:
		blend_reg->postbld_src3_sel = POSTBLD_CLOSE;
		blend_reg->postbld_osd1_premult = 0;
		break;
	}
	/* vpp osd2 postblend scope */
	switch (layer_blend->input2) {
	case BLEND1_DIN:
		blend_reg->postbld_src4_sel = POSTBLD_OSD2;
		if (layer_blend->blend_core1_bypass)
			blend_reg->postbld_osd2_premult = 0;
		else
			blend_reg->postbld_osd2_premult = 1;
		blend_reg->vpp_osd2_blend_h_scope =
			(osd2_h_start << 16 | osd2_h_end);
		blend_reg->vpp_osd2_blend_v_scope =
			(osd2_v_start << 16 | osd2_v_end);
		break;
	case BLEND2_DIN:
		blend_reg->postbld_src4_sel = POSTBLD_OSD1;
		if (blend_reg->din0_byp_blend)
			blend_reg->postbld_osd2_premult = 0;
		else
			blend_reg->postbld_osd2_premult = 1;
		blend_reg->vpp_osd2_blend_h_scope =
			(osd1_h_start << 16 | osd1_h_end);
		blend_reg->vpp_osd2_blend_v_scope =
			(osd1_v_start << 16 | osd1_v_end);
		break;
	default:
		blend_reg->postbld_src4_sel = POSTBLD_CLOSE;
		blend_reg->postbld_osd2_premult = 0;
		break;
	}
	osd_log_dbg2(MODULE_BLEND,
		"vpp_osd1_blend_h_scope=%x, vpp_osd1_blend_v_scope=%x\n",
		blend_reg->vpp_osd1_blend_h_scope,
		blend_reg->vpp_osd1_blend_v_scope);
	osd_log_dbg2(MODULE_BLEND,
		"vpp_osd2_blend_h_scope=%x, vpp_osd2_blend_v_scope=%x\n",
		blend_reg->vpp_osd2_blend_h_scope,
		blend_reg->vpp_osd2_blend_v_scope);
}

static void osd_set_freescale(u32 index,
	struct hw_osd_blending_s *blending)

{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	u32 width, height;
	u32 src_height;
#ifdef CONFIG_ARCH_MESON64_ODROID_COMMON
	u32 dst_width, dst_height;
#endif
	u32 workaround_line = osd_hw.workaround_line;
	u32 output_index = 0;

	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	if (index >= osd_hw.osd_meson_dev.viu1_osd_count) {
		osd_log_err("error osd index=%d\n", index);
		return;
	}
	if (!(osd_hw.osd_display_debug[output_index] &&
		!osd_hw.free_scale_enable[index])) {
		osd_hw.free_scale_enable[index] = 0x10001;
		osd_hw.free_scale[index].h_enable = 1;
		osd_hw.free_scale[index].v_enable = 1;
		osd_hw.free_scale_mode[index] = 1;
	}
	output_index = get_output_device_id(index);

	if (index == OSD1) {
		osd_hw.free_src_data[index].x_start =
			layer_blend->output_data.x;
		osd_hw.free_src_data[index].x_end =
			layer_blend->output_data.x +
			layer_blend->output_data.w - 1;
		osd_hw.free_src_data[index].y_start =
			layer_blend->output_data.y;
		osd_hw.free_src_data[index].y_end =
			layer_blend->output_data.y +
			layer_blend->output_data.h - 1;

		osd_hw.free_dst_data[index].x_start = 0;
		osd_hw.free_dst_data[index].y_start = 0;
		width = layer_blend->output_data.w *
			blending->screen_ratio_w_num /
			blending->screen_ratio_w_den;
		height = (layer_blend->output_data.h - workaround_line) *
			blending->screen_ratio_h_num /
			blending->screen_ratio_h_den;
		if (osd_hw.field_out_en[output_index])
			height = height >> 1;
	} else {
		osd_hw.free_src_data[index].x_start =
			osd_hw.src_data[index].x;
		osd_hw.free_src_data[index].x_end =
			osd_hw.src_data[index].x +
			osd_hw.src_data[index].w - 1;
		osd_hw.free_src_data[index].y_start =
			osd_hw.src_data[index].y;
		osd_hw.free_src_data[index].y_end =
			osd_hw.src_data[index].y +
			osd_hw.src_data[index].h - 1;

		if ((blending->osd_blend_mode == OSD_BLEND_AC) ||
			(blending->osd_blend_mode == OSD_BLEND_ABC)) {
			/* combine mode, need uniformization */
			osd_hw.free_dst_data[index].x_start =
				osd_hw.dst_data[index].x *
				blending->screen_ratio_w_den /
				blending->screen_ratio_w_num;
			osd_hw.free_dst_data[index].y_start =
				osd_hw.dst_data[index].y *
				blending->screen_ratio_h_den /
				blending->screen_ratio_h_num;
			width = osd_hw.dst_data[index].w *
				blending->screen_ratio_w_den /
				blending->screen_ratio_w_num;
			height = osd_hw.dst_data[index].h *
				blending->screen_ratio_h_den /
				blending->screen_ratio_h_num;
			if (width > FREE_SCALE_MAX_WIDTH)
				width = FREE_SCALE_MAX_WIDTH;
		} else if (blending->osd_blend_mode == OSD_BLEND_AB_C) {
			osd_log_dbg(MODULE_BLEND, "blending->blend_din=%d\n",
				blending->blend_din);
			if (blending->blend_din != BLEND_DIN4) {
				/* combine mode, need uniformization */
				osd_hw.free_dst_data[index].x_start =
					osd_hw.dst_data[index].x *
					blending->screen_ratio_w_den /
					blending->screen_ratio_w_num;
				osd_hw.free_dst_data[index].y_start =
					osd_hw.dst_data[index].y *
					blending->screen_ratio_h_den /
					blending->screen_ratio_h_num;
				width = osd_hw.dst_data[index].w *
					blending->screen_ratio_w_den /
					blending->screen_ratio_w_num;
				height = osd_hw.dst_data[index].h *
					blending->screen_ratio_h_den /
					blending->screen_ratio_h_num;
			} else {
				/* direct used dst as freescale dst */
				osd_hw.free_dst_data[index].x_start =
					osd_hw.dst_data[index].x;
				osd_hw.free_dst_data[index].y_start =
					osd_hw.dst_data[index].y;
				width = osd_hw.dst_data[index].w;
				height = osd_hw.dst_data[index].h;
				/* interleaced case */
				if (osd_hw.field_out_en[output_index]) {
					height = height >> 1;
					osd_hw.free_dst_data[index].y_start
						>>= 1;
				}
			}
		} else {
			/* direct used dst as freescale dst */
			osd_hw.free_dst_data[index].x_start =
				osd_hw.dst_data[index].x;
			osd_hw.free_dst_data[index].y_start =
				osd_hw.dst_data[index].y;
			width = osd_hw.dst_data[index].w;
			height = osd_hw.dst_data[index].h;
			if (osd_hw.field_out_en[output_index]) {
				height = height >> 1;
				osd_hw.free_dst_data[index].y_start >>= 1;
			}
		}
	}
	osd_hw.free_dst_data[index].x_end =
		osd_hw.free_dst_data[index].x_start +
		width - 1;
	osd_hw.free_dst_data[index].y_end =
		osd_hw.free_dst_data[index].y_start +
		height - 1;

	src_height = osd_hw.free_src_data[index].x_end -
		osd_hw.free_src_data[index].x_start + 1;
	if ((osd_hw.osd_meson_dev.cpu_id ==
		__MESON_CPU_MAJOR_ID_G12A) &&
		(height != src_height) &&
		osd_hw.free_dst_data[index].y_end <
					osd_hw.vinfo_height[index] - 1)
		osd_set_dummy_data(index, 0);
	else
		osd_set_dummy_data(index, 0xff);

#ifdef CONFIG_ARCH_MESON64_ODROID_COMMON
	/* Adjust free_scale option based on dst axis */
	dst_width = osd_hw.free_dst_data[index].x_end -
		osd_hw.free_dst_data[index].x_start + 1;
	dst_height = osd_hw.free_dst_data[index].y_end -
		osd_hw.free_dst_data[index].y_start + 1;

	/*
	 * On Android, higher framebuffer size over 1920x1080
	 * will be limited to 1920x1080
	 * and osd driver needs to scale the size
	 * to fit in actual hdmi output resolution.
	 */
	if ((dst_width <= 1920) && (dst_height <= 1200)) {
		osd_hw.free_scale[index].h_enable = 1;
		osd_hw.free_scale[index].v_enable = 1;
	} else {
		if (dst_width > 1920)
			osd_hw.free_scale[index].h_enable = 1;
		else
			osd_hw.free_scale[index].h_enable = 0;

		/*
		 * On amlogic hwcomposer, fb size is limited to 1920x1080
		 * in case of higher resolution over 2560x1080.
		 * Other cases, native vertical size is used.
		 */
#ifdef CONFIG_ARCH_MESON64_ODROIDN2
		if ((dst_height >= 1080) && (dst_width >= 2560))
#else
		if ((dst_height > 1080) && (dst_width >= 2560))
#endif
			osd_hw.free_scale[index].v_enable = 1;
		else
			osd_hw.free_scale[index].v_enable = 0;
	}

	osd_hw.free_scale_enable[index] =
		(((osd_hw.free_scale[index].h_enable << 16) & 0xffff0000)
		| (osd_hw.free_scale[index].v_enable & 0xffff));
	osd_hw.free_scale_mode[index] = 1;

	osd_log_dbg2(MODULE_BLEND, "h_enable %d v_enable %d, free_scale 0x%x\n",
		osd_hw.free_scale[index].h_enable,
		osd_hw.free_scale[index].v_enable,
		osd_hw.free_scale_enable[index]);
#endif /* CONFIG_ARCH_MESON64_ODROID_COMMON */

	osd_log_dbg2(MODULE_BLEND, "osd%d:free_src_data:%d,%d,%d,%d\n",
		index,
		osd_hw.free_src_data[index].x_start,
		osd_hw.free_src_data[index].y_start,
		osd_hw.free_src_data[index].x_end,
		osd_hw.free_src_data[index].y_end);
	osd_log_dbg2(MODULE_BLEND, "osd%d:free_dst_data:%d,%d,%d,%d\n",
		index,
		osd_hw.free_dst_data[index].x_start,
		osd_hw.free_dst_data[index].y_start,
		osd_hw.free_dst_data[index].x_end,
		osd_hw.free_dst_data[index].y_end);
}

static void osd_setting_blend0_input(u32 index,
	struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	u32 workaround_line = 0;
	/* for g12a blend shift issue */

	workaround_line = osd_hw.workaround_line;
	layer_blend = &(blending->layer_blend);
	if (index == OSD1) {

			layer_blend->input1_data.x =
				blending->dst_data.x;
			layer_blend->input1_data.y =
				blending->dst_data.y
				+ workaround_line;
			layer_blend->input1_data.w =
				blending->dst_data.w;
			layer_blend->input1_data.h =
				blending->dst_data.h;
	} else {
		layer_blend->input1_data.x =
			osd_hw.free_dst_data[index].x_start;
		layer_blend->input1_data.y =
			osd_hw.free_dst_data[index].y_start
			+ workaround_line;
		layer_blend->input1_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input1_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;
	}
	osd_log_dbg2(MODULE_BLEND,
		"blend0_input:input0_data[osd%d]:%d,%d,%d,%d\n",
		index,
		layer_blend->input1_data.x,
		layer_blend->input1_data.y,
		layer_blend->input1_data.w,
		layer_blend->input1_data.h);
}

static void osd_setting_blend1_input(u32 index,
	struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	u32 workaround_line = 0;
	/* for g12a blend shift issue */

	if (osd_hw.hdr_used)
		workaround_line = osd_hw.workaround_line;
	else {
		if (blending->layer_cnt == 2)
			workaround_line = 0;
		else
			workaround_line = osd_hw.workaround_line;
	}

	layer_blend = &(blending->layer_blend);
	if (index == OSD1) {
		if ((blending->osd_blend_mode == OSD_BLEND_AC)
			|| (blending->osd_blend_mode == OSD_BLEND_ABC)
			|| (blending->osd_blend_mode == OSD_BLEND_AB_C)) {
			layer_blend->output_data.x =
				blending->dst_data.x;
			layer_blend->output_data.y =
				blending->dst_data.y
				+ workaround_line;
			layer_blend->output_data.w =
				blending->dst_data.w;
			layer_blend->output_data.h =
				blending->dst_data.h;
		}
	} else {
		layer_blend->output_data.x =
			osd_hw.free_dst_data[index].x_start;
		layer_blend->output_data.y =
			osd_hw.free_dst_data[index].y_start
			+ workaround_line;
		layer_blend->output_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->output_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;
	}
	osd_log_dbg2(MODULE_BLEND,
		"blend1_input:input_data[osd%d]:%d,%d,%d,%d\n",
		index,
		layer_blend->output_data.x,
		layer_blend->output_data.y,
		layer_blend->output_data.w,
		layer_blend->output_data.h);
}

static void set_blend_path(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	struct dispdata_s output1_data;
	u32 index = 0;
	u8 input1 = 0, input2 = 0;
	u32 output_index;

	if (!blending)
		return;
	output_index = get_output_device_id(index);
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	layer_blend->blend_core1_bypass = 0;
	switch (blending->osd_blend_mode) {
	case OSD_BLEND_NONE:
		blend_reg->postbld_osd1_premult = 0;
		blend_reg->postbld_src4_sel = POSTBLD_CLOSE;
		blend_reg->postbld_src3_sel = POSTBLD_CLOSE;
		blend_reg->postbld_osd2_premult = 0;
		break;
	case OSD_BLEND_A:
		/* blend0-->blend2-->sc-->vpp_osd1 or */
		/* sc-->blend0-->blend2-->vpp_osd1 */
		layer_blend->input1 = BLEND_DIN1;
		if (osd_hw.blend_bypass)
			layer_blend->input1 |= BYPASS_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index >= OSD_MAX)
			return;
		if (index == OSD1) {
			osd_setting_blend0_input(index, blending);
			osd_setting_blend0(blending);

			if (!blend_reg->din0_byp_blend) {
				layer_blend->input1 = BLEND0_DIN;
				layer_blend->input2 = BLEND_NO_DIN;
				memcpy(&layer_blend->input1_data,
					&layer_blend->output_data,
					sizeof(struct dispdata_s));
				/* same with blend0's background */
				osd_setting_blend2(blending);
			}
			/* osd1 freescale,output is vinfo.w/h */
			osd_log_dbg2(MODULE_BLEND,
				"after blend: set osd%d freescale\n", index);
			osd_set_freescale(index, blending);

			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND_NO_DIN;

			layer_blend->input1_data.x =
				osd_hw.free_dst_data[index].x_start +
				osd_hw.disp_info[output_index].position_x;
			layer_blend->input1_data.w =
				osd_hw.free_dst_data[index].x_end -
				osd_hw.free_dst_data[index].x_start + 1;
			layer_blend->input1_data.y =
				osd_hw.free_dst_data[index].y_start +
				osd_hw.disp_info[output_index].position_y;
			layer_blend->input1_data.h =
				osd_hw.free_dst_data[index].y_end -
				osd_hw.free_dst_data[index].y_start + 1;
		} else {
			osd_log_dbg2(MODULE_BLEND,
				"first: set osd%d freescale\n", index);
			osd_set_freescale(index, blending);
			osd_hw.free_dst_data[index].x_start +=
				osd_hw.disp_info[output_index].position_x;
			osd_hw.free_dst_data[index].x_end +=
				osd_hw.disp_info[output_index].position_x;
			osd_hw.free_dst_data[index].y_start +=
				osd_hw.disp_info[output_index].position_y;
			osd_hw.free_dst_data[index].y_end +=
				osd_hw.disp_info[output_index].position_y;

			osd_setting_blend0_input(index, blending);
			osd_setting_blend0(blending);
			if (!blend_reg->din0_byp_blend) {
				layer_blend->input1 = BLEND0_DIN;
				layer_blend->input2 = BLEND_NO_DIN;
				memcpy(&layer_blend->input1_data,
					&layer_blend->output_data,
					sizeof(struct dispdata_s));
				/* same with blend0's background */
				osd_setting_blend2(blending);
			}

			if (index != OSD1) {
				/* if not osd1, need disable osd1 freescale */
				osd_hw.free_scale_enable[OSD1] = 0x0;
				osd_hw.free_scale[OSD1].h_enable = 0;
				osd_hw.free_scale[OSD1].v_enable = 0;
				blending->osd1_freescale_disable = true;
			}
			/* freescale dst is != vpp input,need adjust */
			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND_NO_DIN;

			memcpy(&(layer_blend->input1_data),
				&(layer_blend->output_data),
				sizeof(struct dispdata_s));
		}
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_AC:
		/* blend0 & sc-->blend1-->blend2-->sc-->vpp_osd1 */
		/* sc-->blend0 & blend1-->blend2-->sc-->vpp_osd1 */
		if (!blending->b_exchange_din) {
			input1 = BLEND_DIN1;
			input2 = BLEND_DIN4;
		} else {
			input1 = BLEND_DIN4;
			input2 = BLEND_DIN1;
		}
		layer_blend->input1 = input1;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(input1, blending);
		if (index >= OSD_MAX)
			return;
		if (index != OSD1) {
			/* here used freescale osd1/osd2 */
			osd_log_dbg2(MODULE_BLEND,
				"before blend0: set osd%d freescale\n",
				index);
			osd_set_freescale(index, blending);
		}
		osd_setting_blend0_input(index, blending);
		osd_setting_blend0(blending);
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));

		index = blend_din_to_osd(input2, blending);
		if (index >= OSD_MAX)
			return;
		if (index != OSD1) {
			osd_log_dbg2(MODULE_BLEND,
				"before blend1: set osd%d freescale\n",
				index);
			osd_set_freescale(index, blending);
		}
		layer_blend->input1 = BLEND_NO_DIN;
		layer_blend->input2 = input2;

		osd_setting_blend1_input(index, blending);
		memcpy(&layer_blend->input2_data,
			&layer_blend->output_data,
			sizeof(struct dispdata_s));

		osd_setting_blend1(blending);

		layer_blend->input1 = BLEND0_DIN;
		layer_blend->input2 = BLEND1_DIN;
		/* always used input1_data */
		memcpy(&layer_blend->input1_data, &output1_data,
			sizeof(struct dispdata_s));
		osd_setting_blend2(blending);

		/* used osd0 freescale */
		osd_set_freescale(OSD1, blending);

		layer_blend->input1 = BLEND2_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		layer_blend->input1_data.x =
			osd_hw.free_dst_data[OSD1].x_start +
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input1_data.w =
			osd_hw.free_dst_data[OSD1].x_end -
			osd_hw.free_dst_data[OSD1].x_start + 1;
		layer_blend->input1_data.y =
			osd_hw.free_dst_data[OSD1].y_start +
			osd_hw.disp_info[output_index].position_y;
		layer_blend->input1_data.h =
			osd_hw.free_dst_data[OSD1].y_end -
			osd_hw.free_dst_data[OSD1].y_start + 1;
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_A_C:
		/* blend0 -->blend2-->sc->vpp_osd1 */
		/* sc-->blend1 -->vpp_osd2 */
		/* or */
		/* sc-->blend0 -->blend2-->vpp_osd1 */
		/* sc-->blend1 -->vpp_osd2 */
		layer_blend->input1 = BLEND_DIN1;
		if (osd_hw.blend_bypass)
			layer_blend->input1 |= BYPASS_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index >= OSD_MAX)
			return;
		osd_setting_blend0_input(index, blending);
		osd_setting_blend0(blending);

		if (!blend_reg->din0_byp_blend) {
			layer_blend->input1 = BLEND0_DIN;
			layer_blend->input2 = BLEND_NO_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			/* background is same with blend0's background */
			osd_setting_blend2(blending);
		}
		/* here freescale osd0 used */
		osd_log_dbg2(MODULE_BLEND,
			"after blend: set osd%d freescale\n", index);
		osd_set_freescale(index, blending);
		/* save freescale output */
		output1_data.x =
			osd_hw.free_dst_data[index].x_start +
			osd_hw.disp_info[output_index].position_x;
		output1_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		output1_data.y =
			osd_hw.free_dst_data[index].y_start +
			osd_hw.disp_info[output_index].position_y;
		output1_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		osd_log_dbg2(MODULE_BLEND, "position_x=%d, y=%d\n",
			osd_hw.disp_info[output_index].position_x,
			osd_hw.disp_info[output_index].position_y);

		index = blend_din_to_osd(BLEND_DIN4, blending);
		if (index >= OSD_MAX)
			return;
		/* here freescale osd1/osd2 used */
		osd_log_dbg2(MODULE_BLEND,
		"before blend1: set osd%d freescale\n", index);
		osd_set_freescale(index, blending);
		layer_blend->input1 = BLEND_NO_DIN;
		/* must bypass for shift workaround */
		layer_blend->input2 = BLEND_DIN4 | BYPASS_DIN;
		layer_blend->blend_core1_bypass = 1;
		osd_setting_blend1_input(index, blending);
		memcpy(&layer_blend->input2_data,
			&layer_blend->output_data,
			sizeof(struct dispdata_s));
		/* adjust offset*/
		layer_blend->input2_data.x +=
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input2_data.y +=
			osd_hw.disp_info[output_index].position_y;

		osd_setting_blend1(blending);

		if (!blending->b_exchange_blend_in) {
			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND1_DIN;
			memcpy(&layer_blend->input1_data, &output1_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
		} else {
			layer_blend->input1 = BLEND1_DIN;
			layer_blend->input2 = BLEND2_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data, &output1_data,
				sizeof(struct dispdata_s));
		}
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_ABC:
		/* blend0 -->blend2-->sc-->vpp_osd1 */
		/* sc-->blend1 -->blend2 */
		input1 = BLEND_DIN1;
		input2 = BLEND_DIN4;
		layer_blend->input1 = input1;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(input1, blending);
		if (index >= OSD_MAX)
			return;
		if (index != OSD1) {
			osd_log_dbg2(MODULE_BLEND,
				"index=%d, need set freescale\n", index);
			osd_set_freescale(index, blending);
		}
		osd_setting_blend0_input(index, blending);
		osd_setting_blend0(blending);
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));

		layer_blend->input1 = BLEND_DIN3;
		layer_blend->input2 = input2;
		index = blend_din_to_osd(layer_blend->input1, blending);
		if (index != OSD1) {
			osd_log_dbg2(MODULE_BLEND,
				"blend1 input1: set osd%d freescale\n",
				index);
			osd_set_freescale(index, blending);
		}
		osd_setting_blend1_input(index, blending);
		memcpy(&layer_blend->input1_data,
			&layer_blend->output_data,
			sizeof(struct dispdata_s));

		index = blend_din_to_osd(layer_blend->input2, blending);
		if (index >= OSD_MAX)
			return;
		if (index != OSD1) {
			osd_log_dbg2(MODULE_BLEND,
				"blend1 input2: set osd%d freescale\n",
				index);
			osd_set_freescale(index, blending);
		}
		osd_setting_blend1_input(index, blending);
		memcpy(&layer_blend->input2_data,
			&layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend1(blending);

		layer_blend->input1 = BLEND0_DIN;
		layer_blend->input2 = BLEND1_DIN;
		memcpy(&layer_blend->input1_data, &output1_data,
			sizeof(struct dispdata_s));
		memcpy(&layer_blend->input2_data, &layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend2(blending);
		/* used osd0 freescale */
		osd_set_freescale(OSD1, blending);

		layer_blend->input1 = BLEND2_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		layer_blend->input1_data.x =
			osd_hw.free_dst_data[OSD1].x_start +
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input1_data.w =
			osd_hw.free_dst_data[OSD1].x_end -
			osd_hw.free_dst_data[OSD1].x_start + 1;
		layer_blend->input1_data.y =
			osd_hw.free_dst_data[OSD1].y_start +
			osd_hw.disp_info[output_index].position_y;
		layer_blend->input1_data.h =
			osd_hw.free_dst_data[OSD1].y_end -
			osd_hw.free_dst_data[OSD1].y_start + 1;
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_A_BC:
		/* blend0 -->blend2-->sc->vpp_osd1 */
		/* 2input-->sc-->blend1 -->vpp_osd2 */
		layer_blend->input1 = BLEND_DIN1;
		if (osd_hw.blend_bypass)
			layer_blend->input1 |= BYPASS_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index >= OSD_MAX)
			return;
		osd_setting_blend0_input(index, blending);
		osd_setting_blend0(blending);

		if (!blend_reg->din0_byp_blend) {
			layer_blend->input1 = BLEND0_DIN;
			layer_blend->input2 = BLEND_NO_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			/* background is same with blend0's background */
			osd_setting_blend2(blending);
		}

		osd_log_dbg(MODULE_BLEND,
			"after blend2: set osd%d freescale\n", index);
		osd_set_freescale(index, blending);
		/* save freescale output */
		output1_data.x =
			osd_hw.free_dst_data[index].x_start +
			osd_hw.disp_info[output_index].position_x;
		output1_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		output1_data.y =
			osd_hw.free_dst_data[index].y_start +
			osd_hw.disp_info[output_index].position_y;
		output1_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;
		index = blend_din_to_osd(BLEND_DIN3, blending);
		if (index >= OSD_MAX)
			return;
		osd_log_dbg2(MODULE_BLEND,
			"before blend1: set osd%d freescale\n", index);
		osd_set_freescale(index, blending);
		layer_blend->input1_data.x =
			osd_hw.free_dst_data[index].x_start +
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input1_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input1_data.y =
			osd_hw.free_dst_data[index].y_start +
			osd_hw.disp_info[output_index].position_y;
		layer_blend->input1_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		index = blend_din_to_osd(BLEND_DIN4, blending);
		if (index >= OSD_MAX)
			return;
		osd_log_dbg2(MODULE_BLEND,
			"before blend1: set osd%d freescale\n", index);
		osd_set_freescale(index, blending);
		layer_blend->input2_data.x =
			osd_hw.free_dst_data[index].x_start +
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input2_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input2_data.y =
			osd_hw.free_dst_data[index].y_start +
			osd_hw.disp_info[output_index].position_y;
		layer_blend->input2_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		/* always route(bypass) to dout1 */
		layer_blend->input1 = BLEND_DIN3 | BYPASS_DIN;
		layer_blend->input2 = BLEND_DIN4;

		osd_setting_blend1(blending);

		if (!blending->b_exchange_blend_in) {
			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND1_DIN;
			memcpy(&layer_blend->input1_data, &output1_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
		} else {
			layer_blend->input1 = BLEND1_DIN;
			layer_blend->input2 = BLEND2_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data,
				&output1_data,
				sizeof(struct dispdata_s));
		}
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_AB_C:
		/* blend0 -->blend2-->sc->vpp_osd1 */
		/* sc-->blend1-->blend2-->sc-->vpp_osd1 */
		/* sc -->vpp_osd2 */
		layer_blend->input1 = BLEND_DIN1;
		layer_blend->input2 = BLEND_NO_DIN;
		blending->blend_din = BLEND_DIN1;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index != OSD1) {
			osd_log_info("index=%d, need set freescale\n", index);
			osd_set_freescale(index, blending);
		}
		osd_setting_blend0_input(index, blending);
		osd_setting_blend0(blending);
		/* save blend0 output */
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));

		/* din3 input to blend1 */
		layer_blend->input1 = BLEND_DIN3;
		layer_blend->input2 = BLEND_NO_DIN | BYPASS_DIN;
		layer_blend->blend_core1_bypass = 1;
		blending->blend_din = BLEND_DIN3;
		index = blend_din_to_osd(BLEND_DIN3, blending);
		if (index != OSD1) {
			osd_log_dbg2(MODULE_BLEND,
				"before blend1: set osd%d freescale\n",
				index);
			osd_set_freescale(index, blending);
		}
		osd_setting_blend1_input(index, blending);
		memcpy(&layer_blend->input1_data,
			&layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend1(blending);

		/* din1=>blend0 & din3-> blend1 ==> blend2 */
		layer_blend->input1 = BLEND0_DIN;
		layer_blend->input2 = BLEND1_DIN;
		memcpy(&layer_blend->input1_data, &output1_data,
			sizeof(struct dispdata_s));
		memcpy(&layer_blend->input2_data, &layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend2(blending);

		/* blend2 ==> osd0 freescale */
		osd_set_freescale(OSD1, blending);
		output1_data.x =
			osd_hw.free_dst_data[OSD1].x_start +
			osd_hw.disp_info[output_index].position_x;
		output1_data.w =
			osd_hw.free_dst_data[OSD1].x_end -
			osd_hw.free_dst_data[OSD1].x_start + 1;
		output1_data.y =
			osd_hw.free_dst_data[OSD1].y_start +
			osd_hw.disp_info[output_index].position_y;
		output1_data.h =
			osd_hw.free_dst_data[OSD1].y_end -
			osd_hw.free_dst_data[OSD1].y_start + 1;
		osd_log_dbg2(MODULE_BLEND, "output1_data:%d,%d,%d,%d\n",
			output1_data.x,
			output1_data.w,
			output1_data.y,
			output1_data.h);


		/* din4 ==> vpp */
		index = blend_din_to_osd(BLEND_DIN4, blending);
		blending->blend_din = BLEND_DIN4;
		osd_log_dbg2(MODULE_BLEND,
			"bypass blend1: set osd%d freescale\n", index);
		osd_set_freescale(index, blending);

		layer_blend->input2_data.x =
			osd_hw.free_dst_data[index].x_start +
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input2_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input2_data.y =
			osd_hw.free_dst_data[index].y_start +
			osd_hw.disp_info[output_index].position_y;
		layer_blend->input2_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		/* 2vpp input */
		if (!blending->b_exchange_blend_in) {
			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND1_DIN;
			memcpy(&layer_blend->input1_data, &output1_data,
				sizeof(struct dispdata_s));
		} else {
			layer_blend->input1 = BLEND1_DIN;
			layer_blend->input2 = BLEND2_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->input2_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data,
				&output1_data,
				sizeof(struct dispdata_s));
		}
		vpp_setting_blend(blending);
		break;
	}
}

static void osd_setting_blend0_new(struct hw_osd_blending_s *blending)
{
	struct layer_blend_reg_s *blend_reg;
	struct layer_blend_s *layer_blend;
	u32 index = 0;
	u32 bld_osd_h_start = 0, bld_osd_h_end = 0;
	u32 bld_osd_v_start = 0, bld_osd_v_end = 0;

	if (!blending)
		return;
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	/* blend0 only accept input1 */
	if (layer_blend->input1 & BYPASS_DIN) {
		blend_reg->din0_byp_blend = 1;
		layer_blend->input1 &= ~BYPASS_DIN;
	} else
		blend_reg->din0_byp_blend = 0;
	if (layer_blend->input1 != BLEND_NO_DIN) {
		/* calculate osd blend din scope */
		index = blend_din_to_osd(layer_blend->input1, blending);
		if (index >= OSD_MAX)
			return;
		layer_blend->input1_data.x =
			osd_hw.free_dst_data[index].x_start;
		layer_blend->input1_data.y =
			osd_hw.free_dst_data[index].y_start;
		layer_blend->input1_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input1_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		bld_osd_h_start =
			layer_blend->input1_data.x;
		bld_osd_h_end =
			layer_blend->input1_data.x +
			layer_blend->input1_data.w - 1;
		bld_osd_v_start =
			layer_blend->input1_data.y;
		bld_osd_v_end =
			layer_blend->input1_data.y +
			layer_blend->input1_data.h - 1;
		blend_reg->osd_blend_din_scope_h[index] =
			bld_osd_h_end << 16 | bld_osd_h_start;
		blend_reg->osd_blend_din_scope_v[index] =
			bld_osd_v_end << 16 | bld_osd_v_start;
	}
	layer_blend->output_data.x = 0;
	layer_blend->output_data.y = 0;
	layer_blend->output_data.w = layer_blend->input1_data.x
		+ layer_blend->input1_data.w;
	layer_blend->output_data.h = layer_blend->input1_data.y
		+ layer_blend->input1_data.h;
	osd_log_dbg2(MODULE_BLEND,
		"blend0:input1_data[osd%d]:%d,%d,%d,%d\n",
		index,
		layer_blend->input1_data.x,
		layer_blend->input1_data.y,
		layer_blend->input1_data.w,
		layer_blend->input1_data.h);
	osd_log_dbg2(MODULE_BLEND,
		"blend0:layer_blend->output_data:%d,%d,%d,%d\n",
		layer_blend->output_data.x,
		layer_blend->output_data.y,
		layer_blend->output_data.w,
		layer_blend->output_data.h);
}

static void osd_setting_blend1_new(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	u32 index = 0;
	u32 blend_hsize, blend_vsize;
	u32 bld_osd_h_start = 0, bld_osd_h_end = 0;
	u32 bld_osd_v_start = 0, bld_osd_v_end = 0;
	/* for g12a blend shift issue */

	if (!blending)
		return;
	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	/* blend1_dout to blend2 */
	blend_reg->din2_osd_sel = 0;

	if (layer_blend->input2 & BYPASS_DIN) {
		/* blend1_din3 bypass to dout1 */
		blend_reg->din3_osd_sel = 1;
		layer_blend->input2 &= ~BYPASS_DIN;
	} else
		/* blend1_din3 input to blend1 */
		blend_reg->din3_osd_sel = 0;

	if (layer_blend->input1 != BLEND_NO_DIN) {
		index = blend_din_to_osd(layer_blend->input1, blending);
		if (index >= OSD_MAX)
			return;
		layer_blend->input1_data.x =
			osd_hw.free_dst_data[index].x_start;
		layer_blend->input1_data.y =
			osd_hw.free_dst_data[index].y_start;
		layer_blend->input1_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input1_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		/* calculate osd blend din scope */
		bld_osd_h_start =
			layer_blend->input1_data.x;
		bld_osd_h_end =
			layer_blend->input1_data.x +
			layer_blend->input1_data.w - 1;
		bld_osd_v_start =
			layer_blend->input1_data.y;
		bld_osd_v_end =
			layer_blend->input1_data.y +
			layer_blend->input1_data.h - 1;
		blend_reg->osd_blend_din_scope_h[index] =
			bld_osd_h_end << 16 | bld_osd_h_start;
		blend_reg->osd_blend_din_scope_v[index] =
			bld_osd_v_end << 16 | bld_osd_v_start;
		osd_log_dbg2(MODULE_BLEND,
			"blend1:input1_data(osd%d):%d,%d,%d,%d\n",
			index,
			layer_blend->input1_data.x,
			layer_blend->input1_data.y,
			layer_blend->input1_data.w,
			layer_blend->input1_data.h);
	}
	if (layer_blend->input2 != BLEND_NO_DIN) {
		index = blend_din_to_osd(layer_blend->input2, blending);
		if (index >= OSD_MAX)
			return;
		layer_blend->input2_data.x =
			osd_hw.free_dst_data[index].x_start;
		layer_blend->input2_data.y =
			osd_hw.free_dst_data[index].y_start;
		layer_blend->input2_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input2_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;
		/* calculate osd blend din scope */
		bld_osd_h_start =
			layer_blend->input2_data.x;
		bld_osd_h_end =
			layer_blend->input2_data.x +
			layer_blend->input2_data.w - 1;
		bld_osd_v_start =
			layer_blend->input2_data.y;
		bld_osd_v_end =
			layer_blend->input2_data.y +
			layer_blend->input2_data.h - 1;
		blend_reg->osd_blend_din_scope_h[index] =
			bld_osd_h_end << 16 | bld_osd_h_start;
		blend_reg->osd_blend_din_scope_v[index] =
			bld_osd_v_end << 16 | bld_osd_v_start;
		osd_log_dbg2(MODULE_BLEND,
			"layer_blend->input2_data:%d,%d,%d,%d\n",
			layer_blend->input2_data.x,
			layer_blend->input2_data.y,
			layer_blend->input2_data.w,
			layer_blend->input2_data.h);
	}
	if (blend_reg->din3_osd_sel || layer_blend->input1 == BLEND_NO_DIN) {
		/* blend din3 bypass,output == input */
		if (layer_blend->input2 == BLEND_NO_DIN) {
			memcpy(&layer_blend->output_data,
				&layer_blend->input1_data,
				sizeof(struct dispdata_s));
		} else {
			memcpy(&layer_blend->output_data,
				&layer_blend->input2_data,
				sizeof(struct dispdata_s));
		}
	} else
		calc_max_output(blending);
	blend_hsize = layer_blend->output_data.w;
	blend_vsize = layer_blend->output_data.h;
	blend_reg->osd_blend_blend1_size =
		blend_vsize  << 16 | blend_hsize;

	osd_log_dbg2(MODULE_BLEND, "layer_blend1->output_data:%d,%d,%d,%d\n",
		layer_blend->output_data.x,
		layer_blend->output_data.y,
		layer_blend->output_data.w,
		layer_blend->output_data.h);
}

/* input w, h is background */
static void osd_set_freescale_new(u32 index,
	struct hw_osd_blending_s *blending)

{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	u32 width, height;
	u32 src_height;
	u32 output_index;

	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	if (index >= osd_hw.osd_meson_dev.viu1_osd_count) {
		osd_log_err("error osd index=%d\n", index);
		return;
	}
	output_index = get_output_device_id(index);

	if (!(osd_hw.osd_display_debug[output_index] &&
		!osd_hw.free_scale_enable[index])) {
		osd_hw.free_scale_enable[index] = 0x10001;
		osd_hw.free_scale[index].h_enable = 1;
		osd_hw.free_scale[index].v_enable = 1;
		osd_hw.free_scale_mode[index] = 1;
	}

	osd_hw.free_src_data[index].x_start =
		osd_hw.src_data[index].x;
	osd_hw.free_src_data[index].x_end =
		osd_hw.src_data[index].x +
		osd_hw.src_data[index].w - 1;
	osd_hw.free_src_data[index].y_start =
		osd_hw.src_data[index].y;
	osd_hw.free_src_data[index].y_end =
		osd_hw.src_data[index].y +
		osd_hw.src_data[index].h - 1;

	/* direct used dst as freescale dst */
	osd_hw.free_dst_data[index].x_start =
		osd_hw.dst_data[index].x;
	osd_hw.free_dst_data[index].y_start =
		osd_hw.dst_data[index].y;
	width = osd_hw.dst_data[index].w;
	height = osd_hw.dst_data[index].h;
	if (osd_hw.field_out_en[output_index]) {
		height = height >> 1;
		osd_hw.free_dst_data[index].y_start >>= 1;
	}
	osd_hw.free_dst_data[index].x_end =
		osd_hw.free_dst_data[index].x_start +
		width - 1;
	osd_hw.free_dst_data[index].y_end =
		osd_hw.free_dst_data[index].y_start +
		height - 1;

	src_height = osd_hw.free_src_data[index].x_end -
		osd_hw.free_src_data[index].x_start + 1;
	osd_set_dummy_data(index, 0xff);
	osd_log_dbg2(MODULE_BLEND, "osd%d:free_src_data:%d,%d,%d,%d\n",
		index,
		osd_hw.free_src_data[index].x_start,
		osd_hw.free_src_data[index].y_start,
		osd_hw.free_src_data[index].x_end,
		osd_hw.free_src_data[index].y_end);
	osd_log_dbg2(MODULE_BLEND, "osd%d:free_dst_data:%d,%d,%d,%d\n",
		index,
		osd_hw.free_dst_data[index].x_start,
		osd_hw.free_dst_data[index].y_start,
		osd_hw.free_dst_data[index].x_end,
		osd_hw.free_dst_data[index].y_end);
}

/* every output is next path input */
static void set_blend_path_new(struct hw_osd_blending_s *blending)
{
	struct layer_blend_s *layer_blend;
	struct layer_blend_reg_s *blend_reg;
	struct dispdata_s output1_data;
	u32 index = 0;
	u8 input1 = 0, input2 = 0;
	u32 output_index;

	if (!blending)
		return;
	output_index = get_output_device_id(index);

	layer_blend = &(blending->layer_blend);
	blend_reg = &(blending->blend_reg);
	layer_blend->blend_core1_bypass = 0;
	switch (blending->osd_blend_mode) {
	case OSD_BLEND_NONE:
		blend_reg->postbld_osd1_premult = 0;
		blend_reg->postbld_src4_sel = POSTBLD_CLOSE;
		blend_reg->postbld_src3_sel = POSTBLD_CLOSE;
		blend_reg->postbld_osd2_premult = 0;
		break;
	case OSD_BLEND_A:
		/* sc-->blend0-->blend2-->vpp_osd1 */
		layer_blend->input1 = BLEND_DIN1;
		if (osd_hw.blend_bypass)
			layer_blend->input1 |= BYPASS_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		osd_setting_blend0_new(blending);
		if (!blend_reg->din0_byp_blend) {
			layer_blend->input1 = BLEND0_DIN;
			layer_blend->input2 = BLEND_NO_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			/* same with blend0's background */
			osd_setting_blend2(blending);
		}
		layer_blend->input1 = BLEND2_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		memcpy(&(layer_blend->input1_data),
				&(layer_blend->output_data),
				sizeof(struct dispdata_s));
		layer_blend->input1_data.x +=
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input1_data.y +=
			osd_hw.disp_info[output_index].position_y;
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_AC:
		/* sc-->blend0 -->blend1-->blend2-->vpp_osd1 */
		/* sc-->blend0 & blend1-->blend2-->vpp_osd1 */
		if (!blending->b_exchange_din) {
			input1 = BLEND_DIN1;
			input2 = BLEND_DIN4;
		} else {
			input1 = BLEND_DIN4;
			input2 = BLEND_DIN1;
		}
		layer_blend->input1 = input1;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(input1, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		osd_setting_blend0_new(blending);
		/* save blend0 output */
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));

		index = blend_din_to_osd(input2, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		layer_blend->input1 = BLEND_NO_DIN;
		layer_blend->input2 = input2;
		osd_setting_blend1_new(blending);

		layer_blend->input1 = BLEND0_DIN;
		layer_blend->input2 = BLEND1_DIN;

		/* blend0 output-> blend2  input1 */
		memcpy(&layer_blend->input1_data, &output1_data,
			sizeof(struct dispdata_s));
		/* blend1 output-> blend2  input2 */
		memcpy(&layer_blend->input2_data, &layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend2(blending);

		layer_blend->input1 = BLEND2_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		memcpy(&(layer_blend->input1_data),
				&(layer_blend->output_data),
				sizeof(struct dispdata_s));
		layer_blend->input1_data.x +=
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input1_data.y +=
			osd_hw.disp_info[output_index].position_y;
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_A_C:
		/* sc-->blend0 -->blend2->vpp_osd1 */
		/* sc-->blend1 -->vpp_osd2 */
		layer_blend->input1 = BLEND_DIN1;
		if (osd_hw.blend_bypass)
			layer_blend->input1 |= BYPASS_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		osd_setting_blend0_new(blending);
		if (!blend_reg->din0_byp_blend) {
			layer_blend->input1 = BLEND0_DIN;
			layer_blend->input2 = BLEND_NO_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			/* background is same with blend0's background */
			osd_setting_blend2(blending);
		}
		/* save blend0/blend2 output */
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));
		/* adjust input 1 offset*/
		output1_data.x +=
			osd_hw.disp_info[output_index].position_x;
		output1_data.y +=
			osd_hw.disp_info[output_index].position_y;

		index = blend_din_to_osd(BLEND_DIN4, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		layer_blend->input1 = BLEND_NO_DIN;
		layer_blend->input2 = BLEND_DIN4 | BYPASS_DIN;
		layer_blend->blend_core1_bypass = 1;
		osd_setting_blend1_new(blending);
		/* adjust input 2 offset*/
		layer_blend->output_data.x +=
			osd_hw.disp_info[output_index].position_x;
		layer_blend->output_data.y +=
			osd_hw.disp_info[output_index].position_y;

		if (!blending->b_exchange_blend_in) {
			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND1_DIN;
			memcpy(&layer_blend->input1_data, &output1_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
		} else {
			layer_blend->input1 = BLEND1_DIN;
			layer_blend->input2 = BLEND2_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->output_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data, &output1_data,
				sizeof(struct dispdata_s));
		}
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_ABC:
		/* sc->blend0 -->blend2-->vpp_osd1 */
		/* sc-->blend1 -->blend2 */
		input1 = BLEND_DIN1;
		input2 = BLEND_DIN4;
		layer_blend->input1 = input1;
		layer_blend->input2 = BLEND_NO_DIN;
		index = blend_din_to_osd(input1, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		osd_setting_blend0_new(blending);
		/* save blend0 output */
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));

		layer_blend->input1 = BLEND_DIN3;
		layer_blend->input2 = input2;
		index = blend_din_to_osd(layer_blend->input1, blending);
		osd_set_freescale_new(index, blending);
		index = blend_din_to_osd(layer_blend->input2, blending);
		osd_set_freescale_new(index, blending);
		osd_setting_blend1_new(blending);

		layer_blend->input1 = BLEND0_DIN;
		layer_blend->input2 = BLEND1_DIN;
		memcpy(&layer_blend->input1_data, &output1_data,
			sizeof(struct dispdata_s));
		memcpy(&layer_blend->input2_data, &layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend2(blending);

		layer_blend->input1 = BLEND2_DIN;
		layer_blend->input2 = BLEND_NO_DIN;
		memcpy(&(layer_blend->input1_data),
				&(layer_blend->output_data),
				sizeof(struct dispdata_s));
		layer_blend->input1_data.x +=
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input1_data.y +=
			osd_hw.disp_info[output_index].position_y;
		vpp_setting_blend(blending);
		break;
	case OSD_BLEND_AB_C:
		/* sc-->blend0 -->blend2->vpp_osd1 */
		/* sc-->blend1-->blend2-->vpp_osd1 */
		/* sc -->vpp_osd2 */
		layer_blend->input1 = BLEND_DIN1;
		layer_blend->input2 = BLEND_NO_DIN;
		blending->blend_din = BLEND_DIN1;
		index = blend_din_to_osd(BLEND_DIN1, blending);
		if (index >= OSD_MAX)
			return;
		osd_set_freescale_new(index, blending);
		osd_setting_blend0_new(blending);
		/* save blend0 output */
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));

		/* din3 input to blend1 */
		layer_blend->input1 = BLEND_DIN3;
		layer_blend->input2 = BLEND_NO_DIN | BYPASS_DIN;
		layer_blend->blend_core1_bypass = 1;
		blending->blend_din = BLEND_DIN3;
		index = blend_din_to_osd(BLEND_DIN3, blending);
		osd_set_freescale_new(index, blending);
		osd_setting_blend1_new(blending);

		/* din1=>blend0 & din3-> blend1 ==> blend2 */
		layer_blend->input1 = BLEND0_DIN;
		layer_blend->input2 = BLEND1_DIN;
		memcpy(&layer_blend->input1_data, &output1_data,
			sizeof(struct dispdata_s));
		memcpy(&layer_blend->input2_data, &layer_blend->output_data,
			sizeof(struct dispdata_s));
		osd_setting_blend2(blending);
		memcpy(&output1_data, &(layer_blend->output_data),
			sizeof(struct dispdata_s));
		output1_data.x +=
			osd_hw.disp_info[output_index].position_x;
		output1_data.y +=
			osd_hw.disp_info[output_index].position_y;
		osd_log_dbg2(MODULE_BLEND, "output1_data:%d,%d,%d,%d\n",
			output1_data.x,
			output1_data.w,
			output1_data.y,
			output1_data.h);


		/* din4 ==> vpp */
		index = blend_din_to_osd(BLEND_DIN4, blending);
		blending->blend_din = BLEND_DIN4;
		osd_set_freescale_new(index, blending);
		layer_blend->input2_data.x =
			osd_hw.free_dst_data[index].x_start +
			osd_hw.disp_info[output_index].position_x;
		layer_blend->input2_data.w =
			osd_hw.free_dst_data[index].x_end -
			osd_hw.free_dst_data[index].x_start + 1;
		layer_blend->input2_data.y =
			osd_hw.free_dst_data[index].y_start +
			osd_hw.disp_info[output_index].position_y;
		layer_blend->input2_data.h =
			osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;

		/* 2vpp input */
		if (!blending->b_exchange_blend_in) {
			layer_blend->input1 = BLEND2_DIN;
			layer_blend->input2 = BLEND1_DIN;
			memcpy(&layer_blend->input1_data, &output1_data,
				sizeof(struct dispdata_s));
		} else {
			layer_blend->input1 = BLEND1_DIN;
			layer_blend->input2 = BLEND2_DIN;
			memcpy(&layer_blend->input1_data,
				&layer_blend->input2_data,
				sizeof(struct dispdata_s));
			memcpy(&layer_blend->input2_data,
				&output1_data,
				sizeof(struct dispdata_s));
		}
		vpp_setting_blend(blending);
		break;
	}
}

static void set_blend_reg(struct layer_blend_reg_s *blend_reg)
{
	int i;
	u32 reg_offset = 2;
	u32 osd1_alpha_div = 0, osd2_alpha_div = 0;
	u32 osd_count = OSD_BLEND_LAYERS;
	u32 dv_core2_hsize;
	u32 dv_core2_vsize;

	if (!blend_reg)
		return;
	/* osd0 scale position before osd blend */
	if (osd_hw.osd_meson_dev.osd0_sc_independ)
		VSYNCOSD_WR_MPEG_REG(VPP_OSD_SCALE_CTRL, 0x01);
	if (osd_hw.blend_bypass)
		blend_reg->din0_byp_blend = 1;
	/* osd blend ctrl */
	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_CTRL,
		4				  << 29|
		blend_reg->blend2_premult_en << 27|
		blend_reg->din0_byp_blend	  << 26|
		blend_reg->din2_osd_sel	  << 25|
		blend_reg->din3_osd_sel	  << 24|
		blend_reg->blend_din_en	  << 20|
		blend_reg->din_premult_en	  << 16|
		blend_reg->din_reoder_sel);
	if (blend_reg->postbld_osd1_premult)
		osd1_alpha_div = 1;
	if (blend_reg->postbld_osd2_premult)
		osd2_alpha_div = 1;
	/* VIU_OSD_BLEND_CTRL1 */
	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_CTRL1,
		 (osd1_alpha_div & 0x1) |
		(3 << 4) |
		((osd2_alpha_div & 0x1) << 12) |
		(3 << 16));
	/* vpp osd1 blend ctrl */
	VSYNCOSD_WR_MPEG_REG(OSD1_BLEND_SRC_CTRL,
		(0 & 0xf) << 0 |
		(0 & 0x1) << 4 |
		(blend_reg->postbld_src3_sel & 0xf) << 8 |
		(0 << 16) |
		(1 & 0x1) << 20);
	/* vpp osd2 blend ctrl */
	if (!enable_vd_zorder)
		VSYNCOSD_WR_MPEG_REG(
			OSD2_BLEND_SRC_CTRL,
			(0 & 0xf) << 0 |
			(0 & 0x1) << 4 |
			(blend_reg->postbld_src4_sel & 0xf)
			<< 8 |
			(0 << 16) |
			(1 & 0x1) << 20);

	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_BLEND0_SIZE,
		blend_reg->osd_blend_blend0_size);
	/* hdr input size should set to osd blend0 output size */
	VSYNCOSD_WR_MPEG_REG(VPP_OSD1_IN_SIZE,
		blend_reg->osd_blend_blend0_size);

	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_BLEND1_SIZE,
		blend_reg->osd_blend_blend1_size);

	VSYNCOSD_WR_MPEG_REG(VPP_OSD1_BLD_H_SCOPE,
		blend_reg->vpp_osd1_blend_h_scope);
	VSYNCOSD_WR_MPEG_REG(VPP_OSD1_BLD_V_SCOPE,
		blend_reg->vpp_osd1_blend_v_scope);

	VSYNCOSD_WR_MPEG_REG(VPP_OSD2_BLD_H_SCOPE,
		blend_reg->vpp_osd2_blend_h_scope);
	VSYNCOSD_WR_MPEG_REG(VPP_OSD2_BLD_V_SCOPE,
		blend_reg->vpp_osd2_blend_v_scope);
	for (i = 0; i < osd_count; i++) {
		if (osd_hw.enable[i]) {
		VSYNCOSD_WR_MPEG_REG(
			VIU_OSD_BLEND_DIN0_SCOPE_H + reg_offset * i,
			blend_reg->osd_blend_din_scope_h[i]);
		}
		if ((blend_reg->osd_blend_din_scope_v[i] & 0xffff0000) == 0)
			blend_reg->osd_blend_din_scope_v[i] =
				0xffffffff;
		VSYNCOSD_WR_MPEG_REG(
			VIU_OSD_BLEND_DIN0_SCOPE_V + reg_offset * i,
			blend_reg->osd_blend_din_scope_v[i]);
	}

	dv_core2_vsize = (blend_reg->osd_blend_blend0_size >> 16) & 0xfff;
	dv_core2_hsize = blend_reg->osd_blend_blend0_size & 0xfff;

	if (osd_hw.osd_meson_dev.has_dolby_vision) {
		VSYNCOSD_WR_MPEG_REG(
			DOLBY_CORE2A_SWAP_CTRL1,
			((dv_core2_hsize + 0x40) << 16)
			| (dv_core2_vsize + 0x80 + 0));
		VSYNCOSD_WR_MPEG_REG(
			DOLBY_CORE2A_SWAP_CTRL2,
			(dv_core2_hsize << 16) | (dv_core2_vsize + 0));
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
		update_graphic_width_height(dv_core2_hsize, dv_core2_vsize);
#endif
	}

}

static void uniformization_fb(u32 index,
	struct hw_osd_blending_s *blending)
{
	if ((index == OSD1) && osd_hw.src_crop[index]) {
		blending->screen_ratio_w_den =
			osd_hw.src_data[index].w;
		blending->screen_ratio_h_den =
			osd_hw.src_data[index].h;
	}
	blending->dst_data.x = osd_hw.dst_data[index].x *
		blending->screen_ratio_w_den /
		blending->screen_ratio_w_num;
	blending->dst_data.y = osd_hw.dst_data[index].y *
		blending->screen_ratio_h_den /
		blending->screen_ratio_h_num;
	blending->dst_data.w = osd_hw.dst_data[index].w *
		blending->screen_ratio_w_den /
		blending->screen_ratio_w_num;
	blending->dst_data.h = osd_hw.dst_data[index].h *
		blending->screen_ratio_h_den /
		blending->screen_ratio_h_num;
	osd_log_dbg2(MODULE_BLEND,
		"uniformization:osd%d:dst_data:%d,%d,%d,%d\n",
		index,
		blending->dst_data.x,
		blending->dst_data.y,
		blending->dst_data.w,
		blending->dst_data.h);
}

static void adjust_dst_position(u32 output_index)
{
	int i = 0;
	int osd_count = 0;
	int start_index = 0;

	if (output_index == VIU1) {
		osd_count = osd_hw.osd_meson_dev.viu1_osd_count;
		start_index = 0;
	} else if (output_index == VIU2) {
		start_index = osd_hw.osd_meson_dev.viu2_index;
		osd_count = start_index + 1;
	} else {
		osd_log_err("invald output_index=%d\n", output_index);
		return;
	}

	for (i = 0; i < osd_count; i++) {
		if (osd_hw.enable[i]) {
			osd_hw.dst_data[i].x -=
				osd_hw.disp_info[output_index].position_x;
			osd_hw.dst_data[i].y -=
				osd_hw.disp_info[output_index].position_y;
			if (osd_hw.dst_data[i].x < 0)
				osd_hw.dst_data[i].x = 0;
			if (osd_hw.dst_data[i].y < 0)
				osd_hw.dst_data[i].y = 0;
			osd_log_dbg2(MODULE_BLEND,
				"adjust dst_data:osd%d:%d,%d,%d,%d\n",
				i,
				osd_hw.dst_data[i].x,
				osd_hw.dst_data[i].y,
				osd_hw.dst_data[i].w,
				osd_hw.dst_data[i].h);
		}
	}
	if (osd_hw.field_out_en[output_index])
		osd_hw.disp_info[output_index].position_y /= 2;
}

static int osd_setting_order(u32 output_index)
{
#define RDMA_DETECT_REG VIU_OSD2_TCOLOR_AG2
	int i;
	struct layer_blend_reg_s *blend_reg;
	struct hw_osd_blending_s *blending;
	u32 osd_count = osd_hw.osd_meson_dev.viu1_osd_count;
	int line1;
	int line2;
	int active_begin_line;
	int vinfo_height;
	u32 val, wait_cnt = 0;

	blending = &osd_blending;
	blend_reg = &(blending->blend_reg);

	blending->vinfo_width = osd_hw.vinfo_width[output_index];
	blending->vinfo_height = osd_hw.vinfo_height[output_index];
	blending->screen_ratio_w_num =
		osd_hw.disp_info[output_index].position_w;
	blending->screen_ratio_w_den =
		osd_hw.disp_info[output_index].background_w;
	blending->screen_ratio_h_num =
		osd_hw.disp_info[output_index].position_h;
	blending->screen_ratio_h_den =
		osd_hw.disp_info[output_index].background_h;
	blending->layer_cnt = get_available_layers();
	set_blend_order(blending);

	osd_log_dbg(MODULE_BLEND, "layer_cnt:%d\n",
		blending->layer_cnt);

	blending->b_exchange_din = false;
	blending->b_exchange_blend_in = false;
	blending->osd1_freescale_disable = false;
	adjust_dst_position(output_index);
	if (!osd_hw.osd_meson_dev.osd0_sc_independ)
		uniformization_fb(OSD1, blending);

	/* set blend mode */
	set_blend_mode(blending);
	generate_blend_din_table(blending);

	set_blend_din(blending);

	/* set blend path */
	if (osd_hw.osd_meson_dev.osd0_sc_independ)
		set_blend_path_new(blending);
	else
		set_blend_path(blending);
	active_begin_line = get_active_begin_line(VIU1);
	line1 = get_enter_encp_line(VIU1);
	/* if nearly vsync signal, wait vsync here */
	vinfo_height = osd_hw.field_out_en[output_index] ?
		(osd_hw.vinfo_height[output_index] * 2) :
		osd_hw.vinfo_height[output_index];
	while (line1 >= vinfo_height + active_begin_line *
			(100 - line_threshold) / 100 ||
			line1 <= active_begin_line * line_threshold / 100) {
		osd_log_dbg(MODULE_RENDER,
			"enter osd_setting_order:encp line=%d\n",
			line1);
		/* 0.5ms */
		usleep_range(500, 600);
		wait_cnt++;
		if (wait_cnt >= WAIT_CNT_MAX)
			break;
		line1 = get_enter_encp_line(VIU1);
	}
	spin_lock_irqsave(&osd_lock, lock_flags);
	if (blending->osd1_freescale_disable)
		osd_hw.reg[DISP_FREESCALE_ENABLE].update_func(OSD1);
	for (i = 0; i < osd_count; i++) {
		if (osd_hw.enable[i]) {
			struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[i];
			enum color_index_e idx =
				osd_hw.color_info[i]->color_index;
			bool rgbx = false;

			/* update = is_freescale_para_changed(i); */
			if (!osd_hw.osd_afbcd[i].enable)
				canvas_config(osd_hw.fb_gem[i].canvas_idx,
					osd_hw.fb_gem[i].addr,
					CANVAS_ALIGNED(osd_hw.fb_gem[i].width),
					osd_hw.fb_gem[i].height,
					CANVAS_ADDR_NOWRAP,
					CANVAS_BLKMODE_LINEAR);
			osd_hw.reg[OSD_COLOR_MODE].update_func(i);
			if (!osd_hw.dim_layer[i]) {
				VSYNCOSD_WR_MPEG_REG(osd_reg->osd_dimm_ctrl,
					0x00000000);
			} else {
				u32 dimm_rgb = 0;

				dimm_rgb =
					((osd_hw.dim_color[i] & 0xff000000)
					>> 24) << 22;
				dimm_rgb |=
					((osd_hw.dim_color[i] & 0xff0000)
					>> 16) << 12;
				dimm_rgb |=
					((osd_hw.dim_color[i] & 0xff00)
					>> 8) << 2;
				VSYNCOSD_WR_MPEG_REG(osd_reg->osd_dimm_ctrl,
					0x40000000 | dimm_rgb);
				VSYNCOSD_WR_MPEG_REG_BITS(
					osd_reg->osd_ctrl_stat2, 0x1, 14, 1);
				VSYNCOSD_WR_MPEG_REG_BITS(
					osd_reg->osd_ctrl_stat2,
					osd_hw.dim_color[i] & 0xff, 6, 8);
			}
			osd_hw.reg[DISP_GEOMETRY].update_func(i);
			osd_hw.reg[OSD_GBL_ALPHA].update_func(i);
			osd_hw.reg[DISP_OSD_REVERSE].update_func(i);
			osd_set_scan_mode(i);
			osd_hw.reg[OSD_FREESCALE_COEF].update_func(i);
			osd_hw.reg[DISP_FREESCALE_ENABLE]
				.update_func(i);
			if (osd_update_window_axis)
				osd_update_window_axis = false;
			if (idx >= COLOR_INDEX_32_BGRX &&
			    idx <= COLOR_INDEX_32_XRGB)
				rgbx = true;
			if (osd_hw.premult_en[i] && !osd_hw.blend_bypass &&
			    !rgbx)
				VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_mali_unpack_ctrl, 0x1, 28, 1);
			else
				VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_mali_unpack_ctrl, 0x0, 28, 1);
		}
		osd_hw.reg[OSD_ENABLE].update_func(i);
	}

	//for (i = 0; i < osd_count; i++)
	//	osd_hw.reg[OSD_ENABLE].update_func(i);

	if (osd_hw.hw_rdma_en)
		osd_mali_afbc_start();

	set_blend_reg(blend_reg);
	save_blend_reg(blend_reg);
	/* append RDMA_DETECT_REG at last and detect if rdma missed some regs */
	rdma_dt_cnt++;
	VSYNCOSD_WR_MPEG_REG(RDMA_DETECT_REG, rdma_dt_cnt);
	spin_unlock_irqrestore(&osd_lock, lock_flags);
	line2 = get_exit_encp_line(VIU1);
	osd_log_dbg2(MODULE_RENDER,
		"enter osd_setting_order:encp line=%d\n",
		line2);
	osd_wait_vsync_hw_viu1();
	val = osd_reg_read(RDMA_DETECT_REG);
	/* if missed, need wait vsync */
	if (/*(line2 < line1) || */(val != rdma_dt_cnt)) {
		osd_wait_vsync_hw_viu1();
		osd_log_dbg(MODULE_RENDER, "osd line %d,%d\n", line1, line2);
	}
	return 0;
}

/* only one layer */
static void osd_setting_default_hwc(void)
{
	u32 blend_hsize, blend_vsize;
	u32 blend2_premult_en = 1, din_premult_en = 0;
	u32 blend_din_en = 0x1;
	/* blend_din0 input to blend0 */
	u32 din0_byp_blend = 1;
	/* blend1_dout to blend2 */
	u32 din2_osd_sel = 1;
	/* blend1_din3 input to blend1 */
	u32 din3_osd_sel = 1;
	u32 din_reoder_sel = 0x1;
	u32 postbld_src3_sel = 3, postbld_src4_sel = 0;
	u32 postbld_osd1_premult = 0, postbld_osd2_premult = 0;

	osd_log_dbg(MODULE_BASE, "osd_setting_default_hwc\n");
	/* depend on din0_premult_en */
	postbld_osd1_premult = 1;
	/* depend on din_premult_en bit 4 */
	postbld_osd2_premult = 0;
	/* osd blend ctrl */
	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_CTRL,
		4				  << 29|
		blend2_premult_en << 27|
		din0_byp_blend	  << 26|
		din2_osd_sel	  << 25|
		din3_osd_sel	  << 24|
		blend_din_en	  << 20|
		din_premult_en	  << 16|
		din_reoder_sel);
	/* vpp osd1 blend ctrl */
	VSYNCOSD_WR_MPEG_REG(OSD1_BLEND_SRC_CTRL,
		(0 & 0xf) << 0 |
		(0 & 0x1) << 4 |
		(postbld_src3_sel & 0xf) << 8 |
		(postbld_osd1_premult & 0x1) << 16|
		(1 & 0x1) << 20);
	/* vpp osd2 blend ctrl */
	if (!enable_vd_zorder)
		VSYNCOSD_WR_MPEG_REG(
			OSD2_BLEND_SRC_CTRL,
			(0 & 0xf) << 0 |
			(0 & 0x1) << 4 |
			(postbld_src4_sel & 0xf)
			<< 8 |
			(postbld_osd2_premult & 0x1)
			<< 16 |
			(1 & 0x1) << 20);

	/* Do later: different format select different dummy_data */
	/* used default dummy data */
	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_DUMMY_DATA0,
		0x80 << 16 |
		0x80 << 8 |
		0x80);
	/* used default dummy alpha data */
	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_DUMMY_ALPHA,
		0x0  << 20 |
		0x0  << 11 |
		0x0);

	blend_hsize = osd_hw.disp_info[VIU1].background_w;
	blend_vsize = osd_hw.disp_info[VIU1].background_h;

	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_BLEND0_SIZE,
		blend_vsize  << 16 |
		blend_hsize);
	VSYNCOSD_WR_MPEG_REG(VIU_OSD_BLEND_BLEND1_SIZE,
		blend_vsize  << 16 |
		blend_hsize);
	/* hdr input size should set to osd blend0 output size */
	VSYNCOSD_WR_MPEG_REG(VPP_OSD1_IN_SIZE,
		blend_vsize  << 16 |
		blend_hsize);
}

static bool set_old_hwc_freescale(u32 index)
{
	u32 x_start, x_end, y_start, y_end, height_dst, height_src;

	if (osd_hw.osd_reverse[index] == REVERSE_TRUE) {
		x_start = osd_hw.vinfo_width[VIU1]
			- osd_hw.free_dst_data[index].x_end - 1;
		y_start = osd_hw.vinfo_height[VIU1]
			- osd_hw.free_dst_data[index].y_end - 1;
		x_end = osd_hw.vinfo_width[VIU1]
			- osd_hw.free_dst_data[index].x_start - 1;
		y_end = osd_hw.vinfo_height[VIU1]
			- osd_hw.free_dst_data[index].y_start - 1;
		osd_hw.free_dst_data[index].x_start = x_start;
		osd_hw.free_dst_data[index].y_start = y_start;
		osd_hw.free_dst_data[index].x_end = x_end;
		osd_hw.free_dst_data[index].y_end = y_end;
	} else if (osd_hw.osd_reverse[index] == REVERSE_X) {
		x_start = osd_hw.vinfo_width[VIU1]
			- osd_hw.free_dst_data[index].x_end - 1;
		x_end = osd_hw.vinfo_width[VIU1]
			- osd_hw.free_dst_data[index].x_start - 1;
		osd_hw.free_dst_data[index].x_start = x_start;
		osd_hw.free_dst_data[index].x_end = x_end;
	} else if (osd_hw.osd_reverse[index] == REVERSE_Y) {
		y_start = osd_hw.vinfo_height[VIU1]
			- osd_hw.free_dst_data[index].y_end - 1;
		y_end = osd_hw.vinfo_height[VIU1]
			- osd_hw.free_dst_data[index].y_start - 1;
		osd_hw.free_dst_data[index].y_start = y_start;
		osd_hw.free_dst_data[index].y_end = y_end;
	}
	/* set dummy_data alpha */
	height_dst = osd_hw.free_dst_data[index].y_end -
			osd_hw.free_dst_data[index].y_start + 1;
	height_src = osd_hw.free_src_data[index].y_end -
			osd_hw.free_src_data[index].y_start + 1;
	if (height_dst != height_src &&
		osd_hw.free_dst_data[index].y_end <
					osd_hw.vinfo_height[index] - 1)
		osd_set_dummy_data(index, 0);
	else
		osd_set_dummy_data(index, 0xff);

	if ((memcmp(&(osd_hw.free_src_data[index]),
		&osd_hw.free_src_data_backup[index],
		sizeof(struct pandata_s)) != 0) ||
		(memcmp(&(osd_hw.free_dst_data[index]),
		&osd_hw.free_dst_data_backup[index],
		sizeof(struct pandata_s)) != 0)) {
		memcpy(&osd_hw.free_src_data_backup[index],
			&osd_hw.free_src_data[index],
			sizeof(struct pandata_s));
		memcpy(&osd_hw.free_dst_data_backup[index],
			&osd_hw.free_dst_data[index],
			sizeof(struct pandata_s));
		return true;
	} else
		return false;
}

static void osd_setting_old_hwc(void)
{
	int index = OSD1, output_index = VIU1;
	bool freescale_update = false;
	static u32 osd_enable;

	spin_lock_irqsave(&osd_lock, lock_flags);
	if (!osd_hw.osd_afbcd[index].enable)
		canvas_config(osd_hw.fb_gem[index].canvas_idx,
			osd_hw.fb_gem[index].addr,
			CANVAS_ALIGNED(osd_hw.fb_gem[index].width),
			osd_hw.fb_gem[index].height,
			CANVAS_ADDR_NOWRAP,
			CANVAS_BLKMODE_LINEAR);
	osd_hw.reg[OSD_COLOR_MODE].update_func(index);
	freescale_update = set_old_hwc_freescale(index);
	/* geometry and freescale need update with ioctl */
	osd_hw.reg[DISP_GEOMETRY].update_func(index);
	osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
	if ((osd_hw.free_scale_enable[index]
		&& osd_update_window_axis)
		|| freescale_update) {
		if (!osd_hw.osd_display_debug[output_index]) {
			osd_set_scan_mode(index);
			osd_hw.reg[OSD_FREESCALE_COEF]
				.update_func(index);
			osd_hw.reg[DISP_FREESCALE_ENABLE]
				.update_func(index);
		}
		osd_update_window_axis = false;
	}
	if (osd_enable != osd_hw.enable[index]
		&& (!osd_hw.osd_display_debug[output_index])
		&& (suspend_flag == false)) {
		osd_hw.reg[OSD_ENABLE]
		.update_func(index);
		osd_enable = osd_hw.enable[index];
	}
	spin_unlock_irqrestore(&osd_lock, lock_flags);
	osd_wait_vsync_hw(index);
}

static void osd_setting_viu2(void)
{
	int index = osd_hw.osd_meson_dev.viu2_index;
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];
	static int count;

	count++;
	if (!osd_hw.osd_afbcd[index].enable)
		canvas_config(osd_hw.fb_gem[index].canvas_idx,
			osd_hw.fb_gem[index].addr,
			CANVAS_ALIGNED(osd_hw.fb_gem[index].width),
			osd_hw.fb_gem[index].height,
			CANVAS_ADDR_NOWRAP,
			CANVAS_BLKMODE_LINEAR);
	osd_hw.reg[OSD_COLOR_MODE].update_func(index);
	if (count == 1) {
		/* geometry and freescale need update with ioctl */
		osd_hw.reg[DISP_GEOMETRY].update_func(index);
		osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
		if (!osd_hw.osd_display_debug[VIU2])
			osd_hw.reg[OSD_ENABLE]
			.update_func(index);
	}
	if (!osd_hw.dim_layer[index]) {
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_dimm_ctrl,
			0x00000000);
	} else {
		u32 dimm_rgb = 0;

		dimm_rgb =
			((osd_hw.dim_color[index] & 0xff000000)
			>> 24) << 22;
		dimm_rgb |=
			((osd_hw.dim_color[index] & 0xff0000)
			>> 16) << 12;
		dimm_rgb |=
			((osd_hw.dim_color[index] & 0xff00)
			>> 8) << 2;
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_dimm_ctrl,
			0x40000000 | dimm_rgb);
		VSYNCOSD_WR_MPEG_REG_BITS(
			osd_reg->osd_ctrl_stat2, 0x1, 14, 1);
		VSYNCOSD_WR_MPEG_REG_BITS(
			osd_reg->osd_ctrl_stat2,
			osd_hw.dim_color[index] & 0xff, 6, 8);
	}
	osd_wait_vsync_hw_viu2();
}


int osd_setting_blend(u32 output_index)
{
	int ret;

	if (osd_hw.osd_meson_dev.osd_ver < OSD_HIGH_ONE)
		osd_setting_old_hwc();
	else {
		if (osd_hw.hwc_enable[output_index]) {
			if (output_index == VIU1) {
				ret = osd_setting_order(output_index);
				if (ret < 0)
					return -1;
			} else if (output_index == VIU2) {
				osd_setting_viu2();
			}
		} else {
			osd_setting_default_hwc();
		}
	}
	return 0;
}


void osd_mali_afbc_start(void)
{
	int i, osd_count, afbc_enable = 0;

	osd_count = osd_hw.osd_meson_dev.osd_count;
	if (osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) {
		for (i = 0; i < osd_count; i++) {
			if (osd_hw.osd_afbcd[i].afbc_start) {
				/* enable mali afbc */
				afbc_enable |= (1 << i);
				/* afbc_enable |= 0x10000; */
			}
		}
		if (afbc_enable)
			VSYNCOSD_WR_MPEG_REG(
				VPU_MAFBC_COMMAND,
				(osd_hw.afbc_use_latch ? 2 : 1));
	}
}


static void osd_basic_update_disp_geometry(u32 index)
{
	struct hw_osd_reg_s *osd_reg = &hw_osd_reg_array[index];
	u32 data32;
	u32 buffer_w, buffer_h;

	data32 = (osd_hw.dispdata[index].x_start & 0xfff)
		| (osd_hw.dispdata[index].x_end & 0xfff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		osd_reg->osd_blk0_cfg_w3, data32);
	if (osd_hw.scan_mode[index] == SCAN_MODE_INTERLACE)
		data32 = ((osd_hw.dispdata[index].y_start >> 1) & 0xfff)
			| ((((osd_hw.dispdata[index].y_end + 1)
			>> 1) - 1) & 0xfff) << 16;
	else
		data32 = (osd_hw.dispdata[index].y_start & 0xfff)
			| (osd_hw.dispdata[index].y_end
			& 0xfff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		osd_reg->osd_blk0_cfg_w4, data32);

	if (osd_hw.osd_meson_dev.afbc_type == MALI_AFBC) {
		if (osd_hw.osd_afbcd[index].enable) {
			u64 headr_addr, out_addr;

			/* set frame addr in linear: out_addr_id */
			headr_addr = osd_hw.osd_afbcd[index].phy_addr;
			out_addr = ((u64)osd_hw.osd_afbcd[index].out_addr_id)
					<< 24;
			/*  0:canvas_araddr
			 *  1:linear_araddr
			 */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat, 0x1, 2, 1);
			VSYNCOSD_WR_MPEG_REG(
				osd_reg->osd_blk1_cfg_w4, out_addr);
			/* unpac mali src */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_mali_unpack_ctrl, 0x1, 31, 1);

			/* osd swtich to mali */
			VSYNCOSD_WR_MPEG_REG_BITS(
				OSD_PATH_MISC_CTRL, 0x01, (index + 4), 1);
			/*  0:canvas_araddr
			 *  1:linear_araddr
			 */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat, 0x1, 2, 1);
			/* read from mali afbd decoder */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_blk0_cfg_w0, 0x1, 30, 1);

			VSYNCOSD_WR_MPEG_REG(
				osd_reg->afbc_header_buf_addr_low_s,
				headr_addr & 0xffffffff);
			VSYNCOSD_WR_MPEG_REG(
				osd_reg->afbc_header_buf_addr_high_s,
				(headr_addr >> 32) & 0xffffffff);
			VSYNCOSD_WR_MPEG_REG(osd_reg->afbc_buffer_width_s,
				osd_hw.osd_afbcd[index].frame_width);
			VSYNCOSD_WR_MPEG_REG(osd_reg->afbc_buffer_hight_s,
				osd_hw.osd_afbcd[index].frame_height);

			VSYNCOSD_WR_MPEG_REG(
				osd_reg->afbc_output_buf_addr_low_s,
				out_addr & 0xffffffff);
			VSYNCOSD_WR_MPEG_REG(
				osd_reg->afbc_output_buf_addr_high_s,
				(out_addr >> 32) & 0xffffffff);
			if (osd_hw.osd_afbcd[index].enable) {
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->afbc_boundings_box_x_start_s,
					osd_hw.src_data[index].x);
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->afbc_boundings_box_x_end_s,
					osd_hw.src_data[index].x +
					osd_hw.src_data[index].w - 1);
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->afbc_boundings_box_y_start_s,
					osd_hw.src_data[index].y);
				VSYNCOSD_WR_MPEG_REG(
					osd_reg->afbc_boundings_box_y_end_s,
					osd_hw.src_data[index].y +
					osd_hw.src_data[index].h -  1);
			}
		} else {
			/* osd swtich to mali */
			VSYNCOSD_WR_MPEG_REG_BITS(
				OSD_PATH_MISC_CTRL, 0x0, (index + 4), 1);
			/*  unpac normal src */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_mali_unpack_ctrl, 0x0, 31, 1);
			/* read from ddr */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_blk0_cfg_w0, 0x0, 30, 1);
			/* use canvas addr */
			VSYNCOSD_WR_MPEG_REG_BITS(
				osd_reg->osd_ctrl_stat, 0x0, 2, 1);
		}
	}
	if (osd_hw.free_scale_enable[index]
		   && (osd_hw.free_src_data[index].x_end > 0)
		   && (osd_hw.free_src_data[index].y_end > 0)) {
		/* enable osd free scale */
		data32 = (osd_hw.free_src_data[index].x_start & 0x1fff) |
			 (osd_hw.free_src_data[index].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			osd_reg->osd_blk0_cfg_w1, data32);
		data32 = ((osd_hw.free_src_data[index].y_start
			+ osd_hw.pandata[index].y_start) & 0x1fff)
			| ((osd_hw.free_src_data[index].y_end
			+ osd_hw.pandata[index].y_start) & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_blk0_cfg_w2, data32);
	} else {
		/* normal mode */
		data32 = (osd_hw.pandata[index].x_start & 0x1fff)
			| (osd_hw.pandata[index].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			osd_reg->osd_blk0_cfg_w1, data32);
		data32 = (osd_hw.pandata[index].y_start & 0x1fff)
			| (osd_hw.pandata[index].y_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(osd_reg->osd_blk0_cfg_w2, data32);
	}
	data32 = (osd_hw.src_data[index].x & 0x1fff) |
			 ((osd_hw.src_data[index].x +
			 osd_hw.src_data[index].w - 1) & 0x1fff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[index].osd_blk0_cfg_w1, data32);
	buffer_w = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
	data32 = (osd_hw.src_data[index].y & 0x1fff)
		| ((osd_hw.src_data[index].y +
		osd_hw.src_data[index].h - 1) & 0x1fff) << 16;
	VSYNCOSD_WR_MPEG_REG(osd_reg->osd_blk0_cfg_w2, data32);
	buffer_h = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
	data32 = VSYNCOSD_RD_MPEG_REG(osd_reg->osd_ctrl_stat);
	data32 &= ~0x1ff008;//0x1ff00e;
	data32 |= osd_hw.gbl_alpha[index] << 12;
	/* data32 |= HW_OSD_BLOCK_ENABLE_0; */
	VSYNCOSD_WR_MPEG_REG(osd_reg->osd_ctrl_stat, data32);
	remove_from_update_list(index, DISP_GEOMETRY);
}

static void osd1_basic_update_disp_geometry(void)
{
	u32 data32;
	u32 buffer_w, buffer_h;
	data32 = (osd_hw.dispdata[OSD1].x_start & 0xfff)
		| (osd_hw.dispdata[OSD1].x_end & 0xfff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[OSD1].osd_blk0_cfg_w3, data32);
	if (osd_hw.scan_mode[OSD1] == SCAN_MODE_INTERLACE)
		data32 = ((osd_hw.dispdata[OSD1].y_start >> 1) & 0xfff)
			| ((((osd_hw.dispdata[OSD1].y_end + 1)
			>> 1) - 1) & 0xfff) << 16;
	else
		data32 = (osd_hw.dispdata[OSD1].y_start & 0xfff)
			| (osd_hw.dispdata[OSD1].y_end
			& 0xfff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[OSD1].osd_blk0_cfg_w4, data32);

	if (osd_hw.osd_afbcd[OSD1].enable)
		VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_SIZE_IN,
			((osd_hw.osd_afbcd[OSD1].frame_width
			& 0xffff) << 16) |
			((osd_hw.osd_afbcd[OSD1].frame_height
			& 0xffff) << 0));

	/* enable osd 2x scale */
	if (osd_hw.scale[OSD1].h_enable || osd_hw.scale[OSD1].v_enable) {
		osd1_2x_scale_update_geometry();
		data32 = VSYNCOSD_RD_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w1);
		buffer_w = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
		data32 = VSYNCOSD_RD_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w2);
		buffer_h = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
	} else if (osd_hw.free_scale_enable[OSD1]
		   && (osd_hw.free_src_data[OSD1].x_end > 0)
		   && (osd_hw.free_src_data[OSD1].y_end > 0)) {
		/* enable osd free scale */
		data32 = (osd_hw.free_src_data[OSD1].x_start & 0x1fff) |
			 (osd_hw.free_src_data[OSD1].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w1, data32);
		buffer_w = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
		if (osd_hw.osd_afbcd[OSD1].enable) {
			data32 =
				(osd_hw.free_src_data[OSD1].x_end & 0x1fff) |
				(osd_hw.free_src_data[OSD1].x_start & 0x1fff)
				<< 16;
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_PIXEL_HSCOPE, data32);
			data32 =
				(osd_hw.free_src_data[OSD1].y_end & 0x1fff) |
				(osd_hw.free_src_data[OSD1].y_start & 0x1fff)
				<< 16;
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_PIXEL_VSCOPE, data32);
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_HDR_PTR,
				osd_hw.osd_afbcd[OSD1].phy_addr >> 4);
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_FRAME_PTR,
				osd_hw.osd_afbcd[OSD1].phy_addr >> 4);
			data32 = (0xe4 << 24) |
				(osd_hw.osd_afbcd[OSD1].phy_addr & 0xffffff);
			VSYNCOSD_WR_MPEG_REG(
				OSD1_AFBCD_CHROMA_PTR,
				data32);
		}
		data32 = ((osd_hw.free_src_data[OSD1].y_start
			+ osd_hw.pandata[OSD1].y_start) & 0x1fff)
			| ((osd_hw.free_src_data[OSD1].y_end
			+ osd_hw.pandata[OSD1].y_start) & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(VIU_OSD1_BLK0_CFG_W2, data32);
		buffer_h = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
	} else {
		/* normal mode */
		data32 = (osd_hw.pandata[OSD1].x_start & 0x1fff)
			| (osd_hw.pandata[OSD1].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w1, data32);
		buffer_w = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
		if (osd_hw.osd_afbcd[OSD1].enable) {
			u32 virtual_y_start, virtual_y_end;

			data32 = (osd_hw.pandata[OSD1].x_end & 0x1fff)
				| (osd_hw.pandata[OSD1].x_start & 0x1fff) << 16;
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_PIXEL_HSCOPE, data32);
			virtual_y_start = osd_hw.pandata[OSD1].y_start %
				osd_hw.osd_afbcd[OSD1].frame_height;
			virtual_y_end = osd_hw.pandata[OSD1].y_end -
				osd_hw.pandata[OSD1].y_start + virtual_y_start;
			data32 = (virtual_y_end & 0x1fff) |
				 (virtual_y_start & 0x1fff) << 16;
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_PIXEL_VSCOPE, data32);
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_HDR_PTR,
				osd_hw.osd_afbcd[OSD1].phy_addr >> 4);
			VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_FRAME_PTR,
				osd_hw.osd_afbcd[OSD1].phy_addr >> 4);
			data32 = (0xe4 << 24) |
				(osd_hw.osd_afbcd[OSD1].phy_addr & 0xffffff);
			VSYNCOSD_WR_MPEG_REG(
				OSD1_AFBCD_CHROMA_PTR,
				data32);
		}
		data32 = (osd_hw.pandata[OSD1].y_start & 0x1fff)
			| (osd_hw.pandata[OSD1].y_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD1].osd_blk0_cfg_w2, data32);
		buffer_h = ((data32 >> 16) & 0x1fff) - (data32 & 0x1fff) + 1;
	}
	if (osd_hw.osd_meson_dev.has_dolby_vision) {
		VSYNCOSD_WR_MPEG_REG(
			DOLBY_CORE2A_SWAP_CTRL1,
			((buffer_w + 0x40) << 16)
			| (buffer_h + 0x80 + 0));
		VSYNCOSD_WR_MPEG_REG(
			DOLBY_CORE2A_SWAP_CTRL2,
			(buffer_w << 16) | (buffer_h + 0));
#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_DOLBYVISION
		update_graphic_width_height(buffer_w, buffer_h);
#endif
	}
	if (osd_hw.osd_afbcd[OSD1].enable &&
		!osd_afbc_dec_enable &&
		osd_hw.osd_afbcd[OSD1].phy_addr != 0) {
		VSYNCOSD_WR_MPEG_REG(OSD1_AFBCD_ENABLE, 0x8100);
		osd_afbc_dec_enable = 1;
	}
	data32 = VSYNCOSD_RD_MPEG_REG(hw_osd_reg_array[OSD1].osd_ctrl_stat);
	data32 &= ~0x1ff00e;
	data32 |= osd_hw.gbl_alpha[OSD1] << 12;
	/* data32 |= HW_OSD_BLOCK_ENABLE_0; */
	VSYNCOSD_WR_MPEG_REG(hw_osd_reg_array[OSD1].osd_ctrl_stat, data32);
	remove_from_update_list(OSD1, DISP_GEOMETRY);
}


static void osd2_update_disp_geometry(void)
{
	u32 data32;

	data32 = (osd_hw.dispdata[OSD2].x_start & 0xfff)
		| (osd_hw.dispdata[OSD2].x_end & 0xfff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[OSD2].osd_blk0_cfg_w3, data32);
	if ((osd_hw.scan_mode[OSD2] == SCAN_MODE_INTERLACE) &&
		osd_hw.dispdata[OSD2].y_start >= 0)
		data32 = (osd_hw.dispdata[OSD2].y_start & 0xfff)
			| ((((osd_hw.dispdata[OSD2].y_end + 1
			- osd_hw.dispdata[OSD2].y_start) >> 1)
			+ osd_hw.dispdata[OSD2].y_start - 1)
				& 0xfff) << 16;
	else
		data32 = (osd_hw.dispdata[OSD2].y_start & 0xfff)
			| (osd_hw.dispdata[OSD2].y_end & 0xfff) << 16;
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[OSD2].osd_blk0_cfg_w4, data32);
	if (osd_hw.scale[OSD2].h_enable || osd_hw.scale[OSD2].v_enable) {
#if defined(CONFIG_AMLOGIC_MEDIA_FB_OSD2_CURSOR)
		data32 = (osd_hw.pandata[OSD2].x_start & 0x1fff)
			| (osd_hw.pandata[OSD2].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w1, data32);
		data32 = (osd_hw.pandata[OSD2].y_start & 0x1fff)
			| (osd_hw.pandata[OSD2].y_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w2, data32);
#else
		data32 = (osd_hw.scaledata[OSD2].x_start & 0x1fff) |
			 (osd_hw.scaledata[OSD2].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w1, data32);
		data32 = ((osd_hw.scaledata[OSD2].y_start
				+ osd_hw.pandata[OSD2].y_start) & 0x1fff)
			 | ((osd_hw.scaledata[OSD2].y_end
				+ osd_hw.pandata[OSD2].y_start) & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w2, data32);
#endif
	} else if (osd_hw.free_scale_enable[OSD2]
		   && (osd_hw.free_src_data[OSD2].x_end > 0)
		   && (osd_hw.free_src_data[OSD2].y_end > 0)) {
		/* enable osd free scale */
		data32 = (osd_hw.free_src_data[OSD2].x_start & 0x1fff)
			| (osd_hw.free_src_data[OSD2].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w1, data32);
		data32 = ((osd_hw.free_src_data[OSD2].y_start
			+ osd_hw.pandata[OSD2].y_start) & 0x1fff)
			| ((osd_hw.free_src_data[OSD2].y_end
			+ osd_hw.pandata[OSD2].y_start) & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w2, data32);
	} else {
		data32 = (osd_hw.pandata[OSD2].x_start & 0x1fff)
			| (osd_hw.pandata[OSD2].x_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w1, data32);
		data32 = (osd_hw.pandata[OSD2].y_start & 0x1fff)
			| (osd_hw.pandata[OSD2].y_end & 0x1fff) << 16;
		VSYNCOSD_WR_MPEG_REG(
			hw_osd_reg_array[OSD2].osd_blk0_cfg_w2, data32);
	}
	data32 = VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[OSD2].osd_ctrl_stat);
	data32 &= ~0x1ff000;
	data32 |= osd_hw.gbl_alpha[OSD2] << 12;
	/* data32 |= HW_OSD_BLOCK_ENABLE_0; */
	VSYNCOSD_WR_MPEG_REG(
	hw_osd_reg_array[OSD2].osd_ctrl_stat, data32);
	remove_from_update_list(OSD2, DISP_GEOMETRY);
}


static void osd_update_disp_geometry(u32 index)
{
	if ((osd_hw.block_mode[index]) &&
		(osd_hw.osd_meson_dev.cpu_id <= __MESON_CPU_MAJOR_ID_GXBB)) {
		osd_log_info(
			"osd1_update_disp_geometry: not support block mode\n");
	} else {
		if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
			/* single block */
			if (index == OSD1)
				osd1_basic_update_disp_geometry();
			else if (index == OSD2)
				osd2_update_disp_geometry();
		} else {
			osd_basic_update_disp_geometry(index);
		}
	}
}


static void osd_update_disp_3d_mode(u32 index)
{
	/*step 1 . set pan data */
	/* only called by vsync irq or rdma irq */
	u32  data32;

	if (osd_hw.mode_3d[index].left_right == OSD_LEFT) {
		data32 = (osd_hw.mode_3d[index].l_start & 0x1fff)
			| (osd_hw.mode_3d[index].l_end & 0x1fff) << 16;
		VSYNCOSD_IRQ_WR_MPEG_REG(
			hw_osd_reg_array[index].osd_blk0_cfg_w1, data32);
	} else {
		data32 = (osd_hw.mode_3d[index].r_start & 0x1fff)
			| (osd_hw.mode_3d[index].r_end & 0x1fff) << 16;
		VSYNCOSD_IRQ_WR_MPEG_REG(
			hw_osd_reg_array[index].osd_blk0_cfg_w1, data32);
	}
	osd_hw.mode_3d[index].left_right ^= 1;
}

static void osd_update_fifo(u32 index)
{
	u32 data32;

	data32 = VSYNCOSD_RD_MPEG_REG(
		hw_osd_reg_array[index].osd_fifo_ctrl_stat);
	data32 |= osd_hw.urgent[index] & 1;
	#if 0
	data32 |= 4 << 5; /* hold_fifo_lines */

	/* burst_len_sel: 3=64, g12a = 5 */
	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		data32 |= 1 << 10;
		data32 |= 1 << 31;
	} else
		data32 |= 3 << 10;

	/* fifo_depth_val: 32*8=256 */
	data32 |= (osd_hw.osd_meson_dev.osd_fifo_len
		& 0xfffffff) << 12;
	/*
	 * bit 23:22, fifo_ctrl
	 * 00 : for 1 word in 1 burst
	 * 01 : for 2 words in 1 burst
	 * 10 : for 4 words in 1 burst
	 * 11 : reserved
	 */
	data32 |= 2 << 22;
	/* bit 28:24, fifo_lim */
	data32 |= 2 << 24;
	#endif
	VSYNCOSD_WR_MPEG_REG(
		hw_osd_reg_array[index].osd_fifo_ctrl_stat, data32);
	remove_from_update_list(index, OSD_FIFO);
}

void osd_init_scan_mode(void)
{
	unsigned int output_type = 0;

	output_type = osd_reg_read(VPU_VIU_VENC_MUX_CTRL) & 0x3;
	osd_hw.scan_mode[OSD1] = SCAN_MODE_PROGRESSIVE;
	osd_hw.scan_mode[OSD2] = SCAN_MODE_PROGRESSIVE;
	osd_hw.scan_mode[OSD3] = SCAN_MODE_PROGRESSIVE;
	switch (output_type) {
	case VOUT_ENCP:
		if (osd_reg_read(ENCP_VIDEO_MODE) & (1 << 12)) {
			/* 1080i */
			osd_hw.scan_mode[OSD1] = SCAN_MODE_INTERLACE;
			osd_hw.scan_mode[OSD2] = SCAN_MODE_INTERLACE;
			osd_hw.scan_mode[OSD3] = SCAN_MODE_INTERLACE;
		}
		break;
	case VOUT_ENCI:
		if (osd_reg_read(ENCI_VIDEO_EN) & 1) {
			osd_hw.scan_mode[OSD1] = SCAN_MODE_INTERLACE;
			osd_hw.scan_mode[OSD2] = SCAN_MODE_INTERLACE;
			osd_hw.scan_mode[OSD3] = SCAN_MODE_INTERLACE;
		}
		break;
	}
}

static int osd_extra_canvas_alloc(void)
{
	int osd_num = 2;

	osd_extra_idx[0][0] = EXTERN1_CANVAS;
	osd_extra_idx[0][1] = EXTERN2_CANVAS;
	osd_num = (osd_hw.osd_meson_dev.osd_count - 1) * 2;
	if (canvas_pool_alloc_canvas_table("osd_extra",
		&osd_extra_idx[1][0], osd_num, CANVAS_MAP_TYPE_1)) {
		osd_log_info("allocate osd extra canvas error.\n");
		return -1;
	}
	return 0;
}

void osd_init_hw(u32 logo_loaded, u32 osd_probe,
	struct osd_device_data_s *osd_meson)
{
	u32 idx, data32;
	int err_num = 0;
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
	int i = 0;

	for (i = 0 ; i < HW_OSD_COUNT; i++)
		displayed_bufs[i] = NULL;
#endif

	osd_hw.fb_drvier_probe = osd_probe;

	memcpy(&osd_hw.osd_meson_dev, osd_meson,
		sizeof(struct osd_device_data_s));

	osd_vpu_power_on();
	if (osd_meson->cpu_id == __MESON_CPU_MAJOR_ID_TL1) {
		/* VIU1 2 OSD + 1 VIU2 1 OSD*/
		memcpy(&hw_osd_reg_array[0], &hw_osd_reg_array_tl1[0],
			sizeof(struct hw_osd_reg_s) *
			osd_hw.osd_meson_dev.osd_count);
	} else {
		/* VIU1 3 OSD + 1 VIU2 1 OSD or VIU1 2 OSD*/
		memcpy(&hw_osd_reg_array[0], &hw_osd_reg_array_g12a[0],
			sizeof(struct hw_osd_reg_s) *
			osd_hw.osd_meson_dev.osd_count);
	}
	if (osd_meson->cpu_id == __MESON_CPU_MAJOR_ID_GXTVBB)
		backup_regs_init(HW_RESET_AFBCD_REGS);
	else if (osd_meson->cpu_id == __MESON_CPU_MAJOR_ID_GXM)
		backup_regs_init(HW_RESET_OSD1_REGS);
	else if ((osd_meson->cpu_id >= __MESON_CPU_MAJOR_ID_GXL)
		&& (osd_meson->cpu_id <= __MESON_CPU_MAJOR_ID_TXL))
		backup_regs_init(HW_RESET_OSD1_REGS);
	else if (osd_meson->cpu_id >= __MESON_CPU_MAJOR_ID_G12A)
		backup_regs_init(HW_RESET_MALI_AFBCD_REGS);
	else
		backup_regs_init(HW_RESET_NONE);

	recovery_regs_init();
	for (idx = 0; idx < HW_REG_INDEX_MAX; idx++)
		osd_hw.reg[idx].update_func =
		hw_func_array[idx];

#ifdef CONFIG_AMLOGIC_MEDIA_ENHANCEMENT_VECM
	osd_hdr_on = false;
#endif
	osd_hw.hw_reset_flag = HW_RESET_NONE;
	osd_hw.hwc_enable[VIU1] = 0;
	osd_hw.hwc_enable[VIU2] = 0;
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		osd_hw.hw_cursor_en = 1;
		if (osd_hw.osd_meson_dev.has_rdma)
			osd_hw.hw_rdma_en = 1;
	} else if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		osd_hw.hw_cursor_en = 0;
		if (osd_hw.osd_meson_dev.has_rdma)
			osd_hw.hw_rdma_en = 1;
		/* g12a and g12b need delay */
		supsend_delay = 50;
	}
	/* here we will init default value ,these value only set once . */
	if (!logo_loaded) {
		/* init vpu fifo control register */
		data32 = osd_reg_read(VPP_OFIFO_SIZE);
		if (osd_hw.osd_meson_dev.osd_ver >= OSD_HIGH_ONE) {
			data32 &= ~((0xfff << 20) | 0x3fff);
			data32 |= (osd_hw.osd_meson_dev.vpp_fifo_len) << 20;
			data32 |= osd_hw.osd_meson_dev.vpp_fifo_len + 1;
		} else
			data32 |= osd_hw.osd_meson_dev.vpp_fifo_len;
		osd_reg_write(VPP_OFIFO_SIZE, data32);
		data32 = 0x08080808;
		osd_reg_write(VPP_HOLD_LINES, data32);

		/* init osd fifo control register
		 * set DDR request priority to be urgent
		 */
		data32 = 1;
		/* hold_fifo_lines */
		if (osd_hw.osd_meson_dev.osd_ver >= OSD_HIGH_ONE)
			data32 |= VIU1_DEFAULT_HOLD_LINE << 5;
		else
			data32 |= MIN_HOLD_LINE << 5;
		/* burst_len_sel: 3=64, g12a = 5 */
		if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
			data32 |= 1 << 10;
			data32 |= 1 << 31;
		} else
			data32 |= 3 << 10;
		/*
		 * bit 23:22, fifo_ctrl
		 * 00 : for 1 word in 1 burst
		 * 01 : for 2 words in 1 burst
		 * 10 : for 4 words in 1 burst
		 * 11 : reserved
		 */
		data32 |= 2 << 22;
		/* bit 28:24, fifo_lim */
		data32 |= 2 << 24;

		/* data32_ = data32; */
		/* fifo_depth_val: 32 or 64 *8 = 256 or 512 */
		data32 |= (osd_hw.osd_meson_dev.osd_fifo_len
			& 0xfffffff) << 12;
		for (idx = 0; idx < osd_hw.osd_meson_dev.viu1_osd_count; idx++)
			osd_reg_write(
			hw_osd_reg_array[idx].osd_fifo_ctrl_stat, data32);
		/* osd_reg_write(VIU_OSD2_FIFO_CTRL_STAT, data32_); */
		osd_reg_set_mask(VPP_MISC, VPP_POSTBLEND_EN);
		osd_reg_clr_mask(VPP_MISC, VPP_PREBLEND_EN);
		if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
			osd_vpp_misc =
				osd_reg_read(VPP_MISC) & OSD_RELATIVE_BITS;
			osd_vpp_misc &=
				~(VPP_OSD1_POSTBLEND | VPP_OSD2_POSTBLEND);

			notify_to_amvideo();
			osd_reg_clr_mask(VPP_MISC,
				VPP_OSD1_POSTBLEND | VPP_OSD2_POSTBLEND);
		}
		/* just disable osd to avoid booting hang up */
		data32 = 0x1 << 0;
		data32 |= OSD_GLOBAL_ALPHA_DEF << 12;
		for (idx = 0; idx < osd_hw.osd_meson_dev.viu1_osd_count; idx++)
			osd_reg_write(
				hw_osd_reg_array[idx].osd_ctrl_stat, data32);
		if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE)
			osd_setting_default_hwc();
	}
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		osd_vpp_misc =
			osd_reg_read(VPP_MISC) & OSD_RELATIVE_BITS;
		if (osd_hw.hw_cursor_en) {
			osd_vpp_misc |= (VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			notify_to_amvideo();
			osd_reg_set_mask(VPP_MISC,
				VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			osd_hw.order[OSD1] = OSD_ORDER_10;
		} else {
			osd_vpp_misc &= ~(VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			notify_to_amvideo();
			osd_reg_clr_mask(VPP_MISC,
				VPP_POST_FG_OSD2 | VPP_PRE_FG_OSD2);
			osd_hw.order[OSD1] = OSD_ORDER_01;
		}
	} else if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		osd_hw.order[OSD1] = LAYER_1;
		osd_hw.order[OSD2] = LAYER_2;
		osd_hw.order[OSD3] = LAYER_3;
		for (idx = 0; idx < VIU_COUNT; idx++) {
			osd_hw.disp_info[idx].background_w = 1920;
			osd_hw.disp_info[idx].background_h = 1080;
			osd_hw.disp_info[idx].fullscreen_w = 1920;
			osd_hw.disp_info[idx].fullscreen_h = 1080;
			osd_hw.disp_info[idx].position_x = 0;
			osd_hw.disp_info[idx].position_y = 0;
			osd_hw.disp_info[idx].position_w = 1920;
			osd_hw.disp_info[idx].background_h = 1080;
			osd_hw.vinfo_width[idx] = 1920;
			osd_hw.vinfo_height[idx] = 1080;
		}
		if ((osd_hw.osd_meson_dev.cpu_id ==
			__MESON_CPU_MAJOR_ID_G12A) ||
			((osd_hw.osd_meson_dev.cpu_id ==
			__MESON_CPU_MAJOR_ID_G12B) &&
			is_meson_rev_a()))
			osd_hw.workaround_line = 1;
		for (idx = 0; idx < osd_hw.osd_meson_dev.osd_count; idx++) {
			osd_hw.premult_en[idx] = 0;
			osd_hw.osd_afbcd[idx].format = COLOR_INDEX_32_ABGR;
			osd_hw.osd_afbcd[idx].inter_format =
				MALI_AFBC_32X8_PIXEL << 1 |
				MALI_AFBC_SPLIT_ON;
			osd_hw.osd_afbcd[idx].afbc_start = 0;

			osd_hw.osd_afbcd[idx].out_addr_id = idx + 1;
			if (osd_hw.osd_meson_dev.cpu_id ==
				__MESON_CPU_MAJOR_ID_G12A) {
				osd_hw.afbc_force_reset = 1;
				osd_hw.afbc_regs_backup = 1;
			} else {
				osd_hw.afbc_force_reset = 1;
				osd_hw.afbc_regs_backup = 1;
				data32 = osd_reg_read(MALI_AFBCD_TOP_CTRL);
				osd_reg_write(MALI_AFBCD_TOP_CTRL,
					data32 | 0x800000);
			}

			if (idx < osd_hw.osd_meson_dev.viu1_osd_count) {
				/* TODO: temp set at here,
				 * need move it to uboot
				 */
				osd_reg_set_bits(
				hw_osd_reg_array[idx].osd_fifo_ctrl_stat,
				1, 31, 1);
				osd_reg_set_bits(
				hw_osd_reg_array[idx].osd_fifo_ctrl_stat,
				1, 10, 2);
				osd_reg_set_bits(
				hw_osd_reg_array[idx].osd_fifo_ctrl_stat,
				VIU1_DEFAULT_HOLD_LINE, 5, 5);
				if (osd_hw.osd_meson_dev.cpu_id ==
					__MESON_CPU_MAJOR_ID_G12B)
					osd_reg_set_bits
					(hw_osd_reg_array[idx].osd_ctrl_stat,
					1, 0, 1);
				osd_reg_set_bits(
				hw_osd_reg_array[idx].osd_ctrl_stat,
				0, 31, 1);
				osd_hw.powered[idx] = 1;
			} else
				osd_hw.powered[idx] = 0;
#if 0
			/* enable for latch */
			osd_hw.osd_use_latch[idx] = 1;
			data32 = 0;
			data32 = osd_reg_read(
				hw_osd_reg_array[idx].osd_ctrl_stat);
			data32 |= 0x80000000;
			osd_reg_write(
				hw_osd_reg_array[idx].osd_ctrl_stat, data32);
#endif
		}
		#if 0
		/* TODO: temp power down */
		switch_vpu_mem_pd_vmod(
			VPU_VIU_OSD2,
			VPU_MEM_POWER_DOWN);
		switch_vpu_mem_pd_vmod(
			VPU_VD2_OSD2_SCALE,
			VPU_MEM_POWER_DOWN);
		switch_vpu_mem_pd_vmod(
			VPU_VIU_OSD3,
			VPU_MEM_POWER_DOWN);
		switch_vpu_mem_pd_vmod(
			VPU_OSD_BLD34,
			VPU_MEM_POWER_DOWN);
		#endif
		osd_set_basic_urgent(true);
		osd_set_two_ports(true);
		osd_setting_default_hwc();
	}
	/* disable deband as default */
	if (osd_hw.osd_meson_dev.has_deband)
		osd_reg_write(OSD_DB_FLT_CTRL, 0);
	for (idx = 0; idx < osd_hw.osd_meson_dev.osd_count; idx++) {
		osd_hw.updated[idx] = 0;
		osd_hw.urgent[idx] = 1;
		osd_hw.enable[idx] = DISABLE;
		osd_hw.fb_gem[idx].xres = 0;
		osd_hw.fb_gem[idx].yres = 0;
		osd_hw.gbl_alpha[idx] = OSD_GLOBAL_ALPHA_DEF;
		osd_hw.color_info[idx] = NULL;
		osd_hw.color_backup[idx] = NULL;
		osd_hw.color_key[idx] = 0xffffffff;
		osd_hw.color_key_enable[idx] = 0;
		osd_hw.mode_3d[idx].enable = 0;
		osd_hw.block_mode[idx] = 0;
		osd_hw.osd_reverse[idx] = REVERSE_FALSE;
		osd_hw.free_scale_enable[idx] = 0;
		osd_hw.free_scale_enable_backup[idx] = 0;
		osd_hw.scale[idx].h_enable = 0;
		osd_hw.scale[idx].v_enable = 0;
		osd_hw.free_scale[idx].h_enable = 0;
		osd_hw.free_scale[idx].h_enable = 0;
		osd_hw.free_scale_backup[idx].h_enable = 0;
		osd_hw.free_scale_backup[idx].v_enable = 0;
		osd_hw.free_src_data[idx].x_start = 0;
		osd_hw.free_src_data[idx].x_end = 0;
		osd_hw.free_src_data[idx].y_start = 0;
		osd_hw.free_src_data[idx].y_end = 0;
		osd_hw.free_src_data_backup[idx].x_start = 0;
		osd_hw.free_src_data_backup[idx].x_end = 0;
		osd_hw.free_src_data_backup[idx].y_start = 0;
		osd_hw.free_src_data_backup[idx].y_end = 0;
		osd_hw.free_scale_mode[idx] = 0;
		osd_hw.buffer_alloc[idx] = 0;
		osd_hw.osd_afbcd[idx].enable = 0;
		osd_hw.use_h_filter_mode[idx] = -1;
		osd_hw.use_v_filter_mode[idx] = -1;
		osd_hw.premult_en[idx] = 0;
		/*
		 * osd_hw.rotation_pandata[idx].x_start = 0;
		 * osd_hw.rotation_pandata[idx].y_start = 0;
		 */
		osd_set_dummy_data(idx, 0xff);
	}
	/* hwc_enable == 0 handler */
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
	osd_hw.osd_fence[VIU1][DISABLE].sync_fence_handler =
		sync_render_single_fence;
	osd_hw.osd_fence[VIU1][DISABLE].toggle_buffer_handler =
		osd_toggle_buffer_single;
	/* hwc_enable == 1 handler */
	osd_hw.osd_fence[VIU1][ENABLE].sync_fence_handler =
		sync_render_layers_fence;
	osd_hw.osd_fence[VIU1][ENABLE].toggle_buffer_handler =
		osd_toggle_buffer_layers;
	if (osd_hw.osd_meson_dev.has_viu2) {
		osd_hw.osd_fence[VIU2][DISABLE].sync_fence_handler =
			sync_render_single_fence;
		osd_hw.osd_fence[VIU2][DISABLE].toggle_buffer_handler =
			osd_toggle_buffer_single_viu2;
		/* hwc_enable == 1 handler */
		osd_hw.osd_fence[VIU2][ENABLE].sync_fence_handler =
			sync_render_layers_fence;
		osd_hw.osd_fence[VIU2][ENABLE].toggle_buffer_handler =
			osd_toggle_buffer_layers_viu2;
	}
#endif
	osd_hw.fb_gem[OSD1].canvas_idx = OSD1_CANVAS_INDEX;
	osd_hw.fb_gem[OSD2].canvas_idx = OSD2_CANVAS_INDEX;
	if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
		osd_hw.fb_gem[OSD3].canvas_idx = OSD3_CANVAS_INDEX;
		if (osd_hw.osd_meson_dev.osd_count == 4)
		osd_hw.fb_gem[OSD4].canvas_idx = OSD4_CANVAS_INDEX;
	}
	osd_extra_canvas_alloc();
	osd_hw.antiflicker_mode = 0;
	osd_hw.osd_deband_enable = 1;
	osd_hw.out_fence_fd[VIU1] = -1;
	osd_hw.out_fence_fd[VIU2] = -1;
	osd_hw.blend_bypass = 0;
	osd_hw.afbc_err_cnt = 0;
	if (osd_hw.osd_meson_dev.osd_ver == OSD_SIMPLE) {
		data32 = osd_reg_read(
			hw_osd_reg_array[OSD1].osd_fifo_ctrl_stat);
		/* bit[9:5]: HOLD_FIFO_LINES */
		data32 &= ~(0x1f << 5);
		data32 |= 0x18 << 5;
		osd_reg_write(
			hw_osd_reg_array[OSD1].osd_fifo_ctrl_stat, data32);
	}
	osd_set_deband(osd_hw.osd_deband_enable);
	if (osd_hw.fb_drvier_probe) {
#ifdef CONFIG_AMLOGIC_MEDIA_FB_OSD_SYNC_FENCE
		INIT_LIST_HEAD(&post_fence_list[VIU1]);
		mutex_init(&post_fence_list_lock[VIU1]);
		INIT_LIST_HEAD(&post_fence_list[VIU2]);
		mutex_init(&post_fence_list_lock[VIU2]);
#endif
#ifdef FIQ_VSYNC
		osd_hw.fiq_handle_item.handle = vsync_isr;
		osd_hw.fiq_handle_item.key = (u32)vsync_isr;
		osd_hw.fiq_handle_item.name = "osd_vsync";
		if (register_fiq_bridge_handle(&osd_hw.fiq_handle_item))
			osd_log_err("can't request irq for vsync,err_num=%d\n",
				-err_num);
#else
		err_num = request_irq(int_viu_vsync, &vsync_isr,
					IRQF_SHARED, "osd-vsync", osd_setup_hw);
		if (err_num)
			osd_log_err("can't request irq for vsync,err_num=%d\n",
				-err_num);
		if (osd_hw.osd_meson_dev.has_viu2) {
			err_num = request_irq(int_viu2_vsync, &vsync_viu2_isr,
				IRQF_SHARED, "osd-vsync-viu2", osd_setup_hw);
			if (err_num)
				osd_log_err("can't request irq for viu2 vsync,err_num=%d\n",
				-err_num);
		}
#endif
#ifdef FIQ_VSYNC
		request_fiq(INT_VIU_VSYNC, &osd_fiq_isr);
		request_fiq(INT_VIU_VSYNC, &osd_viu2_fiq_isr);
#endif
	}
	if (osd_hw.hw_rdma_en)
		osd_rdma_enable(2);

}

void set_viu2_format(u32 format)
{
	if (is_yuv_format(format)) {
		set_viu2_rgb2yuv(1);
	} else
		set_viu2_rgb2yuv(0);
}

void osd_init_viu2(void)
{
	u32 idx, data32;
	struct vinfo_s *vinfo;
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	vinfo = get_current_vinfo2();
	if (vinfo && (strcmp(vinfo->name, "invalid") &&
		strcmp(vinfo->name, "null")))
	set_viu2_format(vinfo->viu_color_fmt);
#endif

	idx = osd_hw.osd_meson_dev.viu2_index;
	if (osd_get_logo_index() != LOGO_DEV_VIU2_OSD0) {
		set_viu2_rgb2yuv(1);
		osd_vpu_power_on_viu2();
		/* here we will init default value, these value only set once */
		/* init vpu fifo control register */
		osd_reg_write(VPP2_OFIFO_SIZE, 0x7ff00800);
		/* init osd fifo control register
		 * set DDR request priority to be urgent
		 */
		data32 = 0x1 << 0;
		data32 |= OSD_GLOBAL_ALPHA_DEF << 12;
		osd_reg_write(
			hw_osd_reg_array[idx].osd_ctrl_stat, data32);
		osd_reg_set_bits(
			hw_osd_reg_array[idx].osd_fifo_ctrl_stat,
			1, 31, 1);
		osd_reg_set_bits(
			hw_osd_reg_array[idx].osd_fifo_ctrl_stat,
			1, 10, 2);

		data32 = 0;
		data32 = osd_reg_read(
			hw_osd_reg_array[idx].osd_ctrl_stat);
		data32 |= 0x80000000;
		osd_reg_write(
			hw_osd_reg_array[idx].osd_ctrl_stat, data32);

		data32 = 1;
		data32 |= VIU2_DEFAULT_HOLD_LINE << 5;  /* hold_fifo_lines */
		/* burst_len_sel: 3=64, g12a = 5 */
		if (osd_hw.osd_meson_dev.osd_ver == OSD_HIGH_ONE) {
			data32 |= 1 << 10;
			data32 |= 1 << 31;
		} else
			data32 |= 3 << 10;
		/*
		 * bit 23:22, fifo_ctrl
		 * 00 : for 1 word in 1 burst
		 * 01 : for 2 words in 1 burst
		 * 10 : for 4 words in 1 burst
		 * 11 : reserved
		 */
		data32 |= 2 << 22;
		/* bit 28:24, fifo_lim */
		data32 |= 2 << 24;
		/* data32_ = data32; */
		/* fifo_depth_val: 32 or 64 *8 = 256 or 512 */
		data32 |= (osd_hw.osd_meson_dev.osd_fifo_len
			& 0xfffffff) << 12;
		osd_reg_write(
			hw_osd_reg_array[idx].osd_fifo_ctrl_stat, data32);
	}

	/* enable for latch */
	osd_hw.osd_use_latch[idx] = 1;
	/* init osd reverse */
	osd_get_reverse_hw(idx, &data32);
	if (data32)
		osd_set_reverse_hw(idx, data32, 1);
	viu2_osd_reg_table_init();
	osd_hw.powered[idx] = 1;
}

void osd_cursor_hw(u32 index, s16 x, s16 y, s16 xstart, s16 ystart, u32 osd_w,
		   u32 osd_h)
{
	struct pandata_s disp_tmp;
	int w = 0, h = 0;
	int w_diff = 0;

	if (index != 1)
		return;

	osd_log_dbg(MODULE_CURSOR, "cursor: x=%d, y=%d, x0=%d, y0=%d, w=%d, h=%d\n",
			x, y, xstart, ystart, osd_w, osd_h);

	if (osd_hw.free_scale_mode[OSD1]) {
		if (osd_hw.free_scale_enable[OSD1])
			memcpy(&disp_tmp, &osd_hw.cursor_dispdata[OSD1],
					sizeof(struct pandata_s));
		else
			memcpy(&disp_tmp, &osd_hw.dispdata[OSD1],
					sizeof(struct pandata_s));
	} else
		memcpy(&disp_tmp, &osd_hw.dispdata[OSD1],
				sizeof(struct pandata_s));

	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		if (osd_hw.free_scale_enable[OSD1]) {
			w = osd_hw.free_src_data[OSD1].x_end -
				osd_hw.free_src_data[OSD1].x_start + 1;
			h = osd_hw.free_src_data[OSD1].y_end -
				osd_hw.free_src_data[OSD1].y_start + 1;
		} else {
			w = osd_hw.pandata[OSD1].x_end -
				osd_hw.pandata[OSD1].x_start + 1;
			h = osd_hw.pandata[OSD1].y_end -
				osd_hw.pandata[OSD1].y_start + 1;
		}
		x = w - x;
		y = h - y;
	}
	if (osd_hw.scale[OSD1].h_enable)
		osd_hw.scaledata[OSD2].x_end *= 2;
	if (osd_hw.scale[OSD1].v_enable)
		osd_hw.scaledata[OSD2].y_end *= 2;
	if (osd_hw.scale[OSD2].h_enable && (osd_hw.scaledata[OSD2].x_start > 0)
	    && (osd_hw.scaledata[OSD2].x_end > 0)) {
		x = x * osd_hw.scaledata[OSD2].x_end /
			osd_hw.scaledata[OSD2].x_start;
	}
	if (osd_hw.scale[OSD2].v_enable && (osd_hw.scaledata[OSD2].y_start > 0)
	    && (osd_hw.scaledata[OSD2].y_end > 0)) {
		y = y * osd_hw.scaledata[OSD2].y_end /
			osd_hw.scaledata[OSD2].y_start;
	}
	x += xstart;
	y += ystart;
	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		if (osd_w >= osd_h)
			w_diff = osd_w - osd_h;
		else
			w_diff = 0;
		osd_hw.pandata[OSD2].x_start = w_diff;
		osd_hw.pandata[OSD2].x_end = osd_w - 1;
		osd_hw.pandata[OSD2].y_start = 0;
		osd_hw.pandata[OSD2].y_end = osd_h - 1;
		x -= osd_w;
		y -= osd_h;
		osd_log_dbg(MODULE_CURSOR, "x=%d,y=%d\n", x, y);
	}
	/*
	 * Use pandata to show a partial cursor when it is at the edge because
	 * the registers can't have negative values and because we need to
	 * manually clip the cursor when it is past the edge.  The edge is
	 * hardcoded to the OSD0 area.
	 */
	osd_hw.dispdata[OSD2].x_start = x;
	osd_hw.dispdata[OSD2].y_start = y;
	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		if (x <  disp_tmp.x_start) {
			/* if negative position, set osd x pan. */
			if ((disp_tmp.x_start - x) < osd_w) {
				osd_hw.pandata[OSD2].x_start =
					w_diff;
				osd_hw.pandata[OSD2].x_end =
					osd_w - 1 - (disp_tmp.x_start - x);
			}
			/* set osd x to disp_tmp.x_start */
			osd_hw.dispdata[OSD2].x_start = disp_tmp.x_start;
		}
		if (y < disp_tmp.y_start) {
			/* if negative position, set osd y pan. */
			if ((disp_tmp.y_start - y) < osd_h) {
				osd_hw.pandata[OSD2].y_start = 0;
				osd_hw.pandata[OSD2].y_end =
					osd_h - 1 - (disp_tmp.y_start - y);
			}
			/* set osd y to disp_tmp.y_start */
			osd_hw.dispdata[OSD2].y_start = disp_tmp.y_start;
		}
	} else {
		if (x <  disp_tmp.x_start) {
			/* if negative position, set osd to 0,y and pan. */
			if ((disp_tmp.x_start - x) < osd_w) {
				osd_hw.pandata[OSD2].x_start =
					disp_tmp.x_start - x;
				osd_hw.pandata[OSD2].x_end = osd_w - 1;
			}
			osd_hw.dispdata[OSD2].x_start = 0;
		} else {
			osd_hw.pandata[OSD2].x_start = 0;
			if (x + osd_w > disp_tmp.x_end) {
				/*
				 * if past positive edge,
				 * set osd to inside of the edge and pan.
				 */
				if (x < disp_tmp.x_end)
					osd_hw.pandata[OSD2].x_end =
					disp_tmp.x_end - x;
			} else
				osd_hw.pandata[OSD2].x_end = osd_w - 1;
		}
		if (y < disp_tmp.y_start) {
			if ((disp_tmp.y_start - y) < osd_h) {
				osd_hw.pandata[OSD2].y_start =
					disp_tmp.y_start - y;
				osd_hw.pandata[OSD2].y_end = osd_h - 1;
			}
			osd_hw.dispdata[OSD2].y_start = 0;
		} else {
			osd_hw.pandata[OSD2].y_start = 0;
			if (y + osd_h > disp_tmp.y_end) {
				if (y < disp_tmp.y_end)
					osd_hw.pandata[OSD2].y_end =
					disp_tmp.y_end - y;
			} else
				osd_hw.pandata[OSD2].y_end = osd_h - 1;
		}
	}
	osd_hw.dispdata[OSD2].x_end = osd_hw.dispdata[OSD2].x_start +
		osd_hw.pandata[OSD2].x_end - osd_hw.pandata[OSD2].x_start;
	osd_hw.dispdata[OSD2].y_end = osd_hw.dispdata[OSD2].y_start +
		osd_hw.pandata[OSD2].y_end - osd_hw.pandata[OSD2].y_start;
	add_to_update_list(OSD2, OSD_COLOR_MODE);
	add_to_update_list(OSD2, DISP_GEOMETRY);
	osd_log_dbg(MODULE_CURSOR, "dispdata: %d,%d,%d,%d\n",
		osd_hw.dispdata[OSD2].x_start, osd_hw.dispdata[OSD2].x_end,
		osd_hw.dispdata[OSD2].y_start, osd_hw.dispdata[OSD2].y_end);
}

void osd_cursor_hw_no_scale(u32 index, s16 x, s16 y, s16 xstart, s16 ystart,
			u32 osd_w, u32 osd_h)
{
	struct pandata_s disp_tmp;
	struct vinfo_s *vinfo = NULL;
	int w = 0, h = 0;
	int w_diff = 0;

	if (index != 1)
		return;

	osd_log_dbg(MODULE_CURSOR, "cursor: x=%d, y=%d, x0=%d, y0=%d, w=%d, h=%d\n",
			x, y, xstart, ystart, osd_w, osd_h);

	if (osd_hw.free_scale_mode[OSD1]) {
		if (osd_hw.free_scale_enable[OSD1])
			memcpy(&disp_tmp, &osd_hw.cursor_dispdata[OSD1],
					sizeof(struct pandata_s));
		else
			memcpy(&disp_tmp, &osd_hw.dispdata[OSD1],
					sizeof(struct pandata_s));
	} else
		memcpy(&disp_tmp, &osd_hw.dispdata[OSD1],
				sizeof(struct pandata_s));

	if (osd_hw.field_out_en[VIU1]) {
		disp_tmp.y_start *= 2;
		disp_tmp.y_end *= 2;
	}

	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		vinfo = get_current_vinfo();

		if ((vinfo == NULL) || (vinfo->mode == VMODE_INIT_NULL))
			return;

		w = vinfo->width;
		h = vinfo->height;

		x = w - x;
		y = h - y;
	}

	x += xstart;
	y += ystart;

	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		if (osd_w >= osd_h)
			w_diff = osd_w - osd_h;
		else
			w_diff = 0;
		osd_hw.pandata[OSD2].x_start = w_diff;
		osd_hw.pandata[OSD2].x_end = osd_w - 1;
		osd_hw.pandata[OSD2].y_start = 0;
		osd_hw.pandata[OSD2].y_end = osd_h - 1;
		x -= osd_w;
		y -= osd_h;
		osd_log_dbg(MODULE_CURSOR, "x=%d,y=%d\n", x, y);
	}


	/*
	 * Use pandata to show a partial cursor when it is at the edge because
	 * the registers can't have negative values and because we need to
	 * manually clip the cursor when it is past the edge.  The edge is
	 * hardcoded to the OSD0 area.
	 */
	osd_hw.dispdata[OSD2].x_start = x;
	osd_hw.dispdata[OSD2].y_start = y;
	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		if (x <  disp_tmp.x_start) {
			/* if negative position, set osd x pan. */
			if ((disp_tmp.x_start - x) < osd_w) {
				osd_hw.pandata[OSD2].x_start =
					w_diff;
				osd_hw.pandata[OSD2].x_end =
					osd_w - 1 - (disp_tmp.x_start - x);
			}
			/* set osd x to disp_tmp.x_start */
			osd_hw.dispdata[OSD2].x_start = disp_tmp.x_start;
		}
		if (y < disp_tmp.y_start) {
			/* if negative position, set osd y pan. */
			if ((disp_tmp.y_start - y) < osd_h) {
				osd_hw.pandata[OSD2].y_start = 0;
				osd_hw.pandata[OSD2].y_end =
					osd_h - 1 - (disp_tmp.y_start - y);
			}
			/* set osd y to disp_tmp.y_start */
			osd_hw.dispdata[OSD2].y_start = disp_tmp.y_start;
		}
	} else {
		if (x <  disp_tmp.x_start) {
			/* if negative position, set osd to 0,y and pan. */
			if ((disp_tmp.x_start - x) < osd_w) {
				osd_hw.pandata[OSD2].x_start =
					disp_tmp.x_start - x;
				osd_hw.pandata[OSD2].x_end = osd_w - 1;
			}
			osd_hw.dispdata[OSD2].x_start = 0;
		} else {
			osd_hw.pandata[OSD2].x_start = 0;
			if (x + osd_w > disp_tmp.x_end) {
				/*
				 * if past positive edge,
				 * set osd to inside of the edge and pan.
				 */
				if (x < disp_tmp.x_end)
					osd_hw.pandata[OSD2].x_end =
					disp_tmp.x_end - x;
			} else
				osd_hw.pandata[OSD2].x_end = osd_w - 1;
		}
		if (y < disp_tmp.y_start) {
			if ((disp_tmp.y_start - y) < osd_h) {
				osd_hw.pandata[OSD2].y_start =
					disp_tmp.y_start - y;
				osd_hw.pandata[OSD2].y_end = osd_h - 1;
			}
			osd_hw.dispdata[OSD2].y_start = 0;
		} else {
			osd_hw.pandata[OSD2].y_start = 0;
			if (y + osd_h > disp_tmp.y_end) {
				if (y < disp_tmp.y_end)
					osd_hw.pandata[OSD2].y_end =
					disp_tmp.y_end - y;
			} else
				osd_hw.pandata[OSD2].y_end = osd_h - 1;
		}
	}

	if (osd_hw.field_out_en[VIU1])
		osd_hw.dispdata[OSD2].y_start /= 2;

	osd_hw.dispdata[OSD2].x_end = osd_hw.dispdata[OSD2].x_start +
		osd_hw.pandata[OSD2].x_end - osd_hw.pandata[OSD2].x_start;
	osd_hw.dispdata[OSD2].y_end = osd_hw.dispdata[OSD2].y_start +
		osd_hw.pandata[OSD2].y_end - osd_hw.pandata[OSD2].y_start;
	add_to_update_list(OSD2, OSD_COLOR_MODE);
	add_to_update_list(OSD2, DISP_GEOMETRY);
	osd_log_dbg(MODULE_CURSOR, "dispdata: %d,%d,%d,%d\n",
		osd_hw.dispdata[OSD2].x_start, osd_hw.dispdata[OSD2].x_end,
		osd_hw.dispdata[OSD2].y_start, osd_hw.dispdata[OSD2].y_end);
}

void  osd_suspend_hw(void)
{
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		osd_hw.reg_status_save =
			osd_reg_read(VPP_MISC) & OSD_RELATIVE_BITS;
		osd_vpp_misc &= ~OSD_RELATIVE_BITS;
		notify_to_amvideo();
		osd_reg_clr_mask(VPP_MISC, OSD_RELATIVE_BITS);
		/* VSYNCOSD_CLR_MPEG_REG_MASK(VPP_MISC, OSD_RELATIVE_BITS); */
	} else {
		int i = 0;
		spin_lock_irqsave(&osd_lock, lock_flags);
		suspend_flag = true;
		for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++) {
			if (osd_hw.enable[i]) {
				osd_hw.enable_save[i] = ENABLE;
				osd_hw.enable[i] = DISABLE;
				osd_hw.reg[OSD_ENABLE]
					.update_func(i);
			} else
				osd_hw.enable_save[i] = DISABLE;
		}
		osd_hw.reg_status_save =
			osd_reg_read(VIU_OSD_BLEND_CTRL);
		osd_hw.reg_status_save1 =
			osd_reg_read(OSD1_BLEND_SRC_CTRL);

		osd_hw.reg_status_save3 =
			osd_reg_read(VPP_RDARB_REQEN_SLV);
		osd_hw.reg_status_save4 =
			osd_reg_read(VPU_MAFBC_SURFACE_CFG);
		osd_reg_clr_mask(VIU_OSD_BLEND_CTRL, 0xf0000);
		osd_reg_clr_mask(OSD1_BLEND_SRC_CTRL, 0xf0f);
		if (!enable_vd_zorder) {
			osd_hw.reg_status_save2 =
				osd_reg_read(OSD2_BLEND_SRC_CTRL);
			osd_reg_clr_mask(
				OSD2_BLEND_SRC_CTRL, 0xf0f);
		}
		osd_reg_clr_mask(VPP_RDARB_REQEN_SLV,
			rdarb_reqen_slv);
		osd_reg_write(VPU_MAFBC_SURFACE_CFG, 0);
		osd_reg_write(VPU_MAFBC_COMMAND, 1);
		spin_unlock_irqrestore(&osd_lock, lock_flags);

		if (supsend_delay)
			mdelay(supsend_delay);
	}
	osd_log_info("osd_suspended\n");
}

void osd_resume_hw(void)
{
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		if (osd_hw.reg_status_save &
			(VPP_OSD1_POSTBLEND | VPP_OSD2_POSTBLEND))
			osd_hw.reg_status_save |= VPP_POSTBLEND_EN;
		osd_vpp_misc = osd_hw.reg_status_save & OSD_RELATIVE_BITS;
		notify_to_amvideo();
		osd_reg_set_mask(VPP_MISC, osd_hw.reg_status_save);
		if (osd_hw.enable[OSD2] == ENABLE) {
			osd_vpp_misc |= VPP_OSD2_POSTBLEND;
			osd_reg_set_mask(
				VIU_OSD2_CTRL_STAT,
				1 << 0);
			osd_reg_set_mask(
				VPP_MISC,
				VPP_OSD2_POSTBLEND
				| VPP_POSTBLEND_EN);
		} else {
			osd_vpp_misc &= ~VPP_OSD2_POSTBLEND;
			osd_reg_clr_mask(VPP_MISC,
				VPP_OSD2_POSTBLEND);
			osd_reg_clr_mask(
				VIU_OSD2_CTRL_STAT,
				1 << 0);
		}
		notify_to_amvideo();
	} else {
		int i = 0;
		spin_lock_irqsave(&osd_lock, lock_flags);
		suspend_flag = false;
		for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++) {
			if (osd_hw.enable_save[i]) {
				osd_hw.enable[i] = ENABLE;
				osd_hw.reg[OSD_ENABLE]
					.update_func(i);
			}
		}
		osd_reg_write(VIU_OSD_BLEND_CTRL,
			osd_hw.reg_status_save);
		osd_reg_write(OSD1_BLEND_SRC_CTRL,
			osd_hw.reg_status_save1);
		if (!enable_vd_zorder)
			osd_reg_write(OSD2_BLEND_SRC_CTRL,
				osd_hw.reg_status_save2);
		osd_reg_write(VPP_RDARB_REQEN_SLV,
			osd_hw.reg_status_save3);
		osd_reg_write(VPU_MAFBC_SURFACE_CFG,
			osd_hw.reg_status_save4);
		for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++)
			osd_hw.osd_afbcd[i].afbc_start = 0;
		spin_unlock_irqrestore(&osd_lock, lock_flags);
	}
	osd_log_info("osd_resumed\n");
}

void osd_shutdown_hw(void)
{
#ifdef CONFIG_AMLOGIC_MEDIA_VSYNC_RDMA
	if (osd_hw.osd_meson_dev.has_rdma)
		enable_rdma(0);
#endif
	if (osd_hw.hw_rdma_en)
		osd_rdma_enable(0);
	pr_info("osd_shutdown\n");
}

#ifdef CONFIG_HIBERNATION
static unsigned int fb0_cfg_w0_save;
static u32 __nosavedata free_scale_enable[HW_OSD_COUNT];

void osd_realdata_save_hw(void)
{
	int i;

	for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++)
		free_scale_enable[i] = osd_hw.free_scale_enable[i];
}

void osd_realdata_restore_hw(void)
{
	int i;

	for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++)
		osd_hw.free_scale_enable[i] = free_scale_enable[i];
}

void  osd_freeze_hw(void)
{
#ifdef CONFIG_AMLOGIC_MEDIA_VSYNC_RDMA
	if (osd_hw.osd_meson_dev.has_rdma)
		enable_rdma(0);
#endif
	if (osd_hw.hw_rdma_en) {
		osd_rdma_enable(0);
		if (get_backup_reg(VIU_OSD1_BLK0_CFG_W0,
			&fb0_cfg_w0_save) != 0)
			fb0_cfg_w0_save =
				osd_reg_read(VIU_OSD1_BLK0_CFG_W0);
	} else
		fb0_cfg_w0_save = osd_reg_read(VIU_OSD1_BLK0_CFG_W0);
	pr_debug("osd_freezed\n");
}
void osd_thaw_hw(void)
{
	pr_debug("osd_thawed\n");
	if (osd_hw.hw_rdma_en)
		osd_rdma_enable(2);
#ifdef CONFIG_AMLOGIC_MEDIA_VSYNC_RDMA
	if (osd_hw.osd_meson_dev.has_rdma)
		enable_rdma(1);
#endif
}
void osd_restore_hw(void)
{
	int i;

	osd_reg_write(VIU_OSD1_BLK0_CFG_W0, fb0_cfg_w0_save);
	if (osd_hw.hw_rdma_en)
		osd_rdma_enable(2);
#ifdef CONFIG_AMLOGIC_MEDIA_VSYNC_RDMA
	if (osd_hw.osd_meson_dev.has_rdma)
		enable_rdma(1);
#endif

	if (osd_hw.osd_meson_dev.osd_ver == OSD_SIMPLE)
		osd_update_phy_addr(0);
#ifdef CONFIG_AMLOGIC_MEDIA_CANVAS
	else {
		for (i = 0; i < osd_hw.osd_meson_dev.osd_count; i++)
		canvas_config(osd_hw.fb_gem[i].canvas_idx,
			osd_hw.fb_gem[i].addr,
			CANVAS_ALIGNED(osd_hw.fb_gem[i].width),
			osd_hw.fb_gem[i].height,
			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
	}
#endif
	pr_debug("osd_restored\n");
}
#endif

int osd_get_logo_index(void)
{
	return osd_logo_index;
}

void osd_set_logo_index(int index)
{
	osd_logo_index = index;
}

void osd_get_hw_para(struct hw_para_s **para)
{
	*para = &osd_hw;
}
void osd_get_blending_para(struct hw_osd_blending_s **para)
{
	*para = &osd_blending;
}
void osd_backup_screen_info(
	u32 index,
	unsigned long screen_base,
	unsigned long screen_size)
{
	osd_hw.screen_base_backup[index] = screen_base;
	osd_hw.screen_size_backup[index] = screen_size;
}

void osd_get_screen_info(
	u32 index,
	char __iomem **screen_base,
	unsigned long *screen_size)
{
	*screen_base = (char __iomem *)osd_hw.screen_base[index];
	*screen_size = osd_hw.screen_size[index];
}

int get_vmap_addr(u32 index, u8 __iomem **buf)
{
	unsigned long total_size;
	ulong phys;
	u32 offset, npages;
	struct page **pages = NULL;
	pgprot_t pgprot;
	static u8 *vaddr;
	int i;

	total_size = osd_hw.screen_size[index];

	npages = PAGE_ALIGN(total_size) / PAGE_SIZE;
	phys = osd_hw.screen_base[index];
	offset = phys & ~PAGE_MASK;
	if (offset)
		npages++;
	pages = vmalloc(sizeof(struct page *) * npages);
	if (!pages)
		return -ENOMEM;
	for (i = 0; i < npages; i++) {
		pages[i] = phys_to_page(phys);
		phys += PAGE_SIZE;
	}
	/*nocache*/
	pgprot = pgprot_writecombine(PAGE_KERNEL);
	if (vaddr) {
		/*  unmap prevois vaddr */
		vunmap(vaddr);
		vaddr = NULL;
	}
	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	if (!vaddr) {
		pr_err("the phy(%lx) vmaped fail, size: %d\n",
			phys, npages << PAGE_SHIFT);
		vfree(pages);
		return -ENOMEM;
	}
	vfree(pages);
	*buf = (u8 __iomem *) (vaddr);

	return osd_hw.screen_size[index];
}
#if 0
static int is_new_page(unsigned long addr, unsigned long pos)
{
	static ulong pre_addr;
	u32 offset;
	int ret = 0;

	/* ret == 0 : in same page*/
	if (pos == 0)
		ret = 1;
	else {
		offset = pre_addr & ~PAGE_MASK;
		if ((offset + addr - pre_addr) >= PAGE_SIZE)
			ret = 1;
	}
	pre_addr = addr;
	return ret;
}

ssize_t dd_vmap_write(u32 index, const char __user *buf,
	size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	unsigned long total_size;
	static u8 *vaddr;
	ulong phys;
	u32 offset, npages;
	struct page **pages = NULL;
	struct page *pages_array[2] = {};
	pgprot_t pgprot;
	u8 *buffer, *src;
	u8 __iomem *dst;
	int i, c, cnt = 0, err = 0;

	total_size = osd_hw.screen_size[index];
	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}
	if (count < PAGE_SIZE) {
		/* small than one page, need not vmalloc */
		npages = PAGE_ALIGN(count) / PAGE_SIZE;
		phys = osd_hw.screen_base[index] + p;
		if (is_new_page(phys, p)) {
			/* new page, need call vmap*/
			offset = phys & ~PAGE_MASK;
			if ((offset + count) > PAGE_SIZE)
				npages++;
			for (i = 0; i < npages; i++) {
				pages_array[i] = phys_to_page(phys);
				phys += PAGE_SIZE;
			}
			/*nocache*/
			pgprot = pgprot_writecombine(PAGE_KERNEL);
			if (vaddr) {
				/* unmap prevois vaddr */
				vunmap(vaddr);
				vaddr = NULL;
			}
			vaddr = vmap(pages_array, npages, VM_MAP, pgprot);
			if (!vaddr) {
				pr_err("the phy(%lx) vmaped fail, size: %d\n",
					phys, npages << PAGE_SHIFT);
				return -ENOMEM;
			}
			dst = (u8 __iomem *) (vaddr);
		} else {
			/* in same page just get vaddr + p*/
			dst = (u8 __iomem *) (vaddr + (p & ~PAGE_MASK));
		}
	} else {
		npages = PAGE_ALIGN(count) / PAGE_SIZE;
		phys = osd_hw.screen_base[index] + p;
		offset = phys & ~PAGE_MASK;
		if ((offset + count) > PAGE_SIZE)
			npages++;
		pages = vmalloc(sizeof(struct page *) * npages);
		if (!pages)
			return -ENOMEM;
		for (i = 0; i < npages; i++) {
			pages[i] = phys_to_page(phys);
			phys += PAGE_SIZE;
		}
		/*nocache*/
		pgprot = pgprot_writecombine(PAGE_KERNEL);
		//pgprot = PAGE_KERNEL;
		if (vaddr) {
			/*  unmap prevois vaddr */
			vunmap(vaddr);
			vaddr = NULL;
		}
		vaddr = vmap(pages, npages, VM_MAP, pgprot);
		if (!vaddr) {
			pr_err("the phy(%lx) vmaped fail, size: %d\n",
				phys, npages << PAGE_SHIFT);
			vfree(pages);
			return -ENOMEM;
		}
		vfree(pages);
		dst = (u8 __iomem *) (vaddr);
	}
	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count,
			 GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/* osd_sync() */

	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		src = buffer;

		if (copy_from_user(src, buf, c)) {
			err = -EFAULT;
			break;
		}

		fb_memcpy_tofb(dst, src, c);
		dst += c;
		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}
	kfree(buffer);
	return (cnt) ? cnt : err;
}
#endif
int osd_set_clear(u32 index)
{
	unsigned long total_size;
	ulong phys;
	u32 offset, npages;
	struct page **pages = NULL;
	pgprot_t pgprot;
	static u8 *vaddr;
	u8 __iomem *dst;
	int i, c, count, cnt = 0;

	total_size = osd_hw.screen_size[index];
	npages = PAGE_ALIGN(total_size) / PAGE_SIZE;
	phys = osd_hw.screen_base[index];
	offset = phys & (~PAGE_MASK);
	if (offset)
		npages++;
	pages = vmalloc(sizeof(struct page *) * npages);
	if (!pages)
		return -ENOMEM;
	for (i = 0; i < npages; i++) {
		pages[i] = phys_to_page(phys);
		phys += PAGE_SIZE;
	}
	/*nocache*/
	pgprot = pgprot_writecombine(PAGE_KERNEL);

	vaddr = vmap(pages, npages, VM_MAP, pgprot);
	if (!vaddr) {
		pr_err("the phy(%lx) vmaped fail, size: %d\n",
			phys, npages << PAGE_SHIFT);
		vfree(pages);
		return -ENOMEM;
	}
	vfree(pages);
	dst = (u8 __iomem *) (vaddr);

	count = total_size;
	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		memset(dst, 0, c);
		dst += c;
		cnt += c;
		count -= c;
	}
	if (vaddr) {
		/*  unmap prevois vaddr */
		vunmap(vaddr);
		vaddr = NULL;
	}

	return cnt;
}

static const struct color_bit_define_s *convert_panel_format(u32 format)
{
	const struct color_bit_define_s *color = NULL;

	format &= ~AFBC_EN;
	switch (format) {
	case COLOR_INDEX_02_PAL4:
	case COLOR_INDEX_04_PAL16:
	case COLOR_INDEX_08_PAL256:
	case COLOR_INDEX_16_655:
	case COLOR_INDEX_16_844:
	case COLOR_INDEX_16_6442:
	case COLOR_INDEX_16_4444_R:
	case COLOR_INDEX_16_4642_R:
	case COLOR_INDEX_16_1555_A:
	case COLOR_INDEX_16_4444_A:
	case COLOR_INDEX_16_565:
	case COLOR_INDEX_24_6666_A:
	case COLOR_INDEX_24_6666_R:
	case COLOR_INDEX_24_8565:
	case COLOR_INDEX_24_5658:
	case COLOR_INDEX_24_888_B:
	case COLOR_INDEX_24_RGB:
	case COLOR_INDEX_32_BGRX:
	case COLOR_INDEX_32_XBGR:
	case COLOR_INDEX_32_RGBX:
	case COLOR_INDEX_32_XRGB:
	case COLOR_INDEX_32_BGRA:
	case COLOR_INDEX_32_ABGR:
	case COLOR_INDEX_32_RGBA:
	case COLOR_INDEX_32_ARGB:
	case COLOR_INDEX_YUV_422:
		color = &default_color_format_array[format];
		break;
	}
	return color;
}

static bool osd_direct_render(struct osd_plane_map_s *plane_map)
{
	u32 index = plane_map->plane_index;
	u32 phy_addr = plane_map->phy_addr;
	u32 width_src, width_dst, height_src, height_dst;
	u32 x_start, x_end, y_start, y_end;
	bool freescale_update = false;
	struct pandata_s freescale_dst[HW_OSD_COUNT];
	u32 output_index;

	output_index = get_output_device_id(index);
	phy_addr = phy_addr + plane_map->byte_stride * plane_map->src_y;
	osd_hw.screen_base[index] = phy_addr;
	osd_hw.screen_size[index] =
		plane_map->byte_stride * plane_map->src_h;
	osd_log_dbg(MODULE_RENDER, "canvas_id=%x, phy_addr=%x\n",
		osd_hw.fb_gem[index].canvas_idx, phy_addr);
	if (osd_hw.osd_meson_dev.osd_ver == OSD_SIMPLE) {
		osd_hw.fb_gem[index].addr = phy_addr;
		osd_hw.fb_gem[index].width = plane_map->byte_stride;
		osd_update_phy_addr(0);
	} else {
		canvas_config(osd_hw.fb_gem[index].canvas_idx,
			phy_addr,
			CANVAS_ALIGNED(plane_map->byte_stride),
			plane_map->src_h,
			CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);
	}
	if (osd_hw.hwc_enable[output_index]) {
		/* just get para, need update via do_hwc */
		osd_hw.order[index] = plane_map->zorder;
		switch (plane_map->blend_mode) {
		case BLEND_MODE_PREMULTIPLIED:
			osd_hw.premult_en[index] = 1;
			break;
		case BLEND_MODE_COVERAGE:
		case BLEND_MODE_NONE:
		case BLEND_MODE_INVALID:
			osd_hw.premult_en[index] = 0;
			break;
		}
		//Todo: fence_map.plane_alpha
		osd_hw.osd_afbcd[index].inter_format =
			plane_map->afbc_inter_format & 0x7fffffff;

		osd_hw.src_data[index].x = plane_map->src_x;
		osd_hw.src_data[index].y = plane_map->src_y;
		osd_hw.src_data[index].w = plane_map->src_w;
		osd_hw.src_data[index].h = plane_map->src_h;

		osd_hw.dst_data[index].x = plane_map->dst_x;
		osd_hw.dst_data[index].y = plane_map->dst_y;
		osd_hw.dst_data[index].w = plane_map->dst_w;
		osd_hw.dst_data[index].h = plane_map->dst_h;
		osd_log_dbg2(MODULE_RENDER, "index=%d\n", index);
		osd_log_dbg2(MODULE_RENDER, "order=%d\n",
			osd_hw.order[index]);
		osd_log_dbg2(MODULE_RENDER, "premult_en=%d\n",
			osd_hw.premult_en[index]);
		osd_log_dbg2(MODULE_RENDER, "osd_afbcd_en=%d\n",
			osd_hw.osd_afbcd[index].enable);
		osd_log_dbg2(MODULE_RENDER, "osd_afbcd_inter_format=%d\n",
			osd_hw.osd_afbcd[index].inter_format);
		return 0;
	}

	width_dst = osd_hw.free_dst_data_backup[index].x_end -
		osd_hw.free_dst_data_backup[index].x_start + 1;
	width_src = osd_hw.free_src_data_backup[index].x_end -
		osd_hw.free_src_data_backup[index].x_start + 1;

	height_dst = osd_hw.free_dst_data_backup[index].y_end -
		osd_hw.free_dst_data_backup[index].y_start + 1;
	height_src = osd_hw.free_src_data_backup[index].y_end -
		osd_hw.free_src_data_backup[index].y_start + 1;
	osd_hw.free_scale_enable[index] = 1;

	osd_hw.src_data[index].x = plane_map->src_x;
	osd_hw.src_data[index].y = plane_map->src_y;
	osd_hw.src_data[index].w = plane_map->src_w;
	osd_hw.src_data[index].h = plane_map->src_h;

	if (osd_hw.free_scale_enable[index] ||
		(width_src != width_dst) ||
		(height_src != height_dst) ||
		(plane_map->src_w != plane_map->dst_w) ||
		(plane_map->src_h != plane_map->dst_h)) {
		osd_hw.free_scale[index].h_enable = 1;
		osd_hw.free_scale[index].v_enable = 1;
		osd_hw.free_scale_enable[index] = 0x10001;
		osd_hw.free_scale_mode[index] = 1;
		if (osd_hw.free_scale_enable[index] !=
			osd_hw.free_scale_enable_backup[index]) {
			/* Todo: */
			osd_set_scan_mode(index);
			freescale_update = true;
		}

		osd_hw.pandata[index].x_start = plane_map->src_x;
		osd_hw.pandata[index].x_end =
			plane_map->src_x + plane_map->src_w - 1;
		osd_hw.pandata[index].y_start = 0;
		osd_hw.pandata[index].y_end = plane_map->src_h - 1;

		freescale_dst[index].x_start =
			osd_hw.free_dst_data_backup[index].x_start +
			(plane_map->dst_x * width_dst) / width_src;
		freescale_dst[index].x_end =
			osd_hw.free_dst_data_backup[index].x_start +
			((plane_map->dst_x + plane_map->dst_w) *
			width_dst) / width_src - 1;

		freescale_dst[index].y_start =
			osd_hw.free_dst_data_backup[index].y_start +
			(plane_map->dst_y * height_dst) / height_src;
		freescale_dst[index].y_end =
			osd_hw.free_dst_data_backup[index].y_start +
			((plane_map->dst_y + plane_map->dst_h) *
			height_dst) / height_src - 1;
		if (osd_hw.osd_reverse[index] == REVERSE_TRUE) {
			x_start = osd_hw.vinfo_width[output_index]
				- freescale_dst[index].x_end - 1;
			y_start = osd_hw.vinfo_height[output_index]
				- freescale_dst[index].y_end - 1;
			x_end = osd_hw.vinfo_width[output_index]
				- freescale_dst[index].x_start - 1;
			y_end = osd_hw.vinfo_height[output_index]
				- freescale_dst[index].y_start - 1;
			freescale_dst[index].x_start = x_start;
			freescale_dst[index].y_start = y_start;
			freescale_dst[index].x_end = x_end;
			freescale_dst[index].y_end = y_end;
		} else if (osd_hw.osd_reverse[index] == REVERSE_X) {
			x_start = osd_hw.vinfo_width[output_index]
				- freescale_dst[index].x_end - 1;
			x_end = osd_hw.vinfo_width[output_index]
				- freescale_dst[index].x_start - 1;
			freescale_dst[index].x_start = x_start;
			freescale_dst[index].x_end = x_end;

		} else if (osd_hw.osd_reverse[index] == REVERSE_Y) {
			y_start = osd_hw.vinfo_height[output_index]
				- freescale_dst[index].y_end - 1;
			y_end = osd_hw.vinfo_height[output_index]
				- freescale_dst[index].y_start - 1;
			freescale_dst[index].y_start = y_start;
			freescale_dst[index].y_end = y_end;
		}
		if (memcmp(&(osd_hw.free_src_data[index]),
			&osd_hw.pandata[index],
			sizeof(struct pandata_s)) != 0 ||
			memcmp(&(osd_hw.free_dst_data[index]),
			&freescale_dst[index],
			sizeof(struct pandata_s)) != 0) {
			memcpy(&(osd_hw.free_src_data[index]),
				&osd_hw.pandata[index],
				sizeof(struct pandata_s));
			memcpy(&(osd_hw.free_dst_data[index]),
				&freescale_dst[index],
				sizeof(struct pandata_s));
			freescale_update = true;
			osd_hw.dst_data[index].x =
				osd_hw.free_dst_data[index].x_start;
			osd_hw.dst_data[index].y =
				osd_hw.free_dst_data[index].x_start;
			osd_hw.dst_data[index].w =
				osd_hw.free_dst_data[index].x_end -
				osd_hw.free_dst_data[index].x_start + 1;
			osd_hw.dst_data[index].h =
				osd_hw.free_dst_data[index].y_end -
				osd_hw.free_dst_data[index].y_start + 1;

			if (((height_dst != height_src) ||
				(width_dst != width_src)) &&
				osd_hw.free_dst_data[index].y_end <
						osd_hw.vinfo_height[index] - 1)
				osd_set_dummy_data(index, 0);
			else
				osd_set_dummy_data(index, 0xff);
		}
		osd_log_dbg2(MODULE_RENDER,
			"pandata x=%d,x_end=%d,y=%d,y_end=%d\n",
			osd_hw.pandata[index].x_start,
			osd_hw.pandata[index].x_end,
			osd_hw.pandata[index].y_start,
			osd_hw.pandata[index].y_end);
		osd_log_dbg2(MODULE_RENDER,
			"plane_map:src_x=%d,src_y=%d,src_w=%d,src_h=%d\n",
			plane_map->src_x,
			plane_map->src_y,
			plane_map->src_w,
			plane_map->src_h);
		osd_log_dbg2(MODULE_RENDER,
			"fence_map:dst_x=%d,dst_y=%d,dst_w=%d,dst_h=%d\n",
			plane_map->dst_x,
			plane_map->dst_y,
			plane_map->dst_w,
			plane_map->dst_h);
	} else {
		osd_hw.pandata[index].x_start = 0;
		osd_hw.pandata[index].x_end = plane_map->src_w - 1;
		osd_hw.pandata[index].y_start = 0;
		osd_hw.pandata[index].y_end = plane_map->src_h - 1;

		osd_hw.dispdata[index].x_start = plane_map->dst_x;
		osd_hw.dispdata[index].x_end =
			plane_map->dst_x + plane_map->dst_w - 1;
		osd_hw.dispdata[index].y_start = plane_map->dst_y;
		osd_hw.dispdata[index].y_end =
			plane_map->dst_y + plane_map->dst_h - 1;
		if (osd_hw.osd_reverse[index] == REVERSE_TRUE) {
			x_start = osd_hw.vinfo_width[output_index]
				- osd_hw.dispdata[index].x_end - 1;
			y_start = osd_hw.vinfo_height[output_index]
				- osd_hw.dispdata[index].y_end - 1;
			x_end = osd_hw.vinfo_width[output_index]
				- osd_hw.dispdata[index].x_start - 1;
			y_end = osd_hw.vinfo_height[output_index]
				- osd_hw.dispdata[index].y_start - 1;
			osd_hw.dispdata[index].x_start = x_start;
			osd_hw.dispdata[index].y_start = y_start;
			osd_hw.dispdata[index].x_end = x_end;
			osd_hw.dispdata[index].y_end = y_end;
		} else if (osd_hw.osd_reverse[index] == REVERSE_X) {
			x_start = osd_hw.vinfo_width[output_index]
				- osd_hw.dispdata[index].x_end - 1;
			x_end = osd_hw.vinfo_width[output_index]
				- osd_hw.dispdata[index].x_start - 1;
			osd_hw.dispdata[index].x_start = x_start;
			osd_hw.dispdata[index].x_end = x_end;

		} else if (osd_hw.osd_reverse[index] == REVERSE_Y) {
			y_start = osd_hw.vinfo_height[output_index]
				- osd_hw.dispdata[index].y_end - 1;
			y_end = osd_hw.vinfo_height[output_index]
				- osd_hw.dispdata[index].y_start - 1;
			osd_hw.dispdata[index].y_start = y_start;
			osd_hw.dispdata[index].y_end = y_end;
		}
		osd_hw.dst_data[index].x =
			osd_hw.dispdata[index].x_start;
		osd_hw.dst_data[index].y =
			osd_hw.dispdata[index].x_start;
		osd_hw.dst_data[index].w =
			osd_hw.dispdata[index].x_end -
			osd_hw.dispdata[index].x_start + 1;
		osd_hw.dst_data[index].h =
			osd_hw.dispdata[index].y_end -
			osd_hw.dispdata[index].y_start + 1;

	}
	return freescale_update;
}

static void osd_cursor_move(struct osd_plane_map_s *plane_map)
{
	u32 index = plane_map->plane_index;
	u32 phy_addr = plane_map->phy_addr;
	u32 x_start, x_end, y_start, y_end;
	u32 x, y;
	struct pandata_s disp_tmp;
	struct pandata_s free_dst_data_backup;
	u32 output_index;

	if (index != OSD2)
		return;
	output_index = get_output_device_id(index);
	phy_addr = phy_addr + plane_map->byte_stride * plane_map->src_y;
	osd_hw.screen_base[index] = phy_addr;
	osd_hw.screen_size[index] =
		plane_map->byte_stride * plane_map->src_h;
	canvas_config(osd_hw.fb_gem[index].canvas_idx,
		phy_addr,
		CANVAS_ALIGNED(plane_map->byte_stride),
		plane_map->src_h,
		CANVAS_ADDR_NOWRAP, CANVAS_BLKMODE_LINEAR);


	osd_hw.pandata[index].x_start = plane_map->src_x;
	osd_hw.pandata[index].x_end = plane_map->src_w - 1;
	osd_hw.pandata[index].y_start = plane_map->src_y;
	osd_hw.pandata[index].y_end = plane_map->src_h - 1;

	if (osd_hw.free_scale_mode[OSD1]) {
		if (osd_hw.free_scale_enable[OSD1])
			memcpy(&disp_tmp, &osd_hw.cursor_dispdata[OSD1],
					sizeof(struct pandata_s));
		else
			memcpy(&disp_tmp, &osd_hw.dispdata[OSD1],
					sizeof(struct pandata_s));
	} else
		memcpy(&disp_tmp, &osd_hw.dispdata[OSD1],
				sizeof(struct pandata_s));
	memcpy(&free_dst_data_backup,
		&osd_hw.free_dst_data_backup[OSD1],
		sizeof(struct pandata_s));
	#if 0
	if (osd_hw.free_scale[OSD1].h_enable)
		free_dst_data_backup.x_end =
			osd_hw.free_dst_data_backup[OSD1].x_end * 2;
	if (osd_hw.free_scale[OSD1].v_enable)
		free_dst_data_backup.y_end =
			osd_hw.free_dst_data_backup[OSD1].y_end * 2;
	#endif
	x = plane_map->dst_x;
	y = plane_map->dst_y;
	if (osd_hw.free_src_data_backup[OSD1].x_end > 0 &&
		free_dst_data_backup.x_end > 0) {
		x = x * free_dst_data_backup.x_end /
			osd_hw.free_src_data_backup[OSD1].x_end;
	}
	if (osd_hw.free_src_data_backup[OSD1].y_end > 0 &&
		free_dst_data_backup.y_end > 0) {
		y = y * free_dst_data_backup.y_end /
			osd_hw.free_src_data_backup[OSD1].y_end;
	}

	/*
	 * Use pandata to show a partial cursor when it is at the edge because
	 * the registers can't have negative values and because we need to
	 * manually clip the cursor when it is past the edge.  The edge is
	 * hardcoded to the OSD0 area.
	 */
	osd_hw.dispdata[OSD2].x_start = x;
	osd_hw.dispdata[OSD2].y_start = y;
	if (x <  disp_tmp.x_start) {
		/* if negative position, set osd to 0,y and pan. */
		if ((disp_tmp.x_start - x) < plane_map->src_w) {
			osd_hw.pandata[OSD2].x_start = disp_tmp.x_start - x;
			osd_hw.pandata[OSD2].x_end = plane_map->src_w - 1;
		}
		osd_hw.dispdata[OSD2].x_start = 0;
	} else {
		osd_hw.pandata[OSD2].x_start = 0;
		if (x + plane_map->src_w > disp_tmp.x_end) {
			/*
			 * if past positive edge,
			 * set osd to inside of the edge and pan.
			 */
			if (x < disp_tmp.x_end)
				osd_hw.pandata[OSD2].x_end = disp_tmp.x_end - x;
		} else
			osd_hw.pandata[OSD2].x_end = plane_map->src_w - 1;
	}
	if (y < disp_tmp.y_start) {
		if ((disp_tmp.y_start - y) < plane_map->src_h) {
			osd_hw.pandata[OSD2].y_start = disp_tmp.y_start - y;
			osd_hw.pandata[OSD2].y_end = plane_map->src_h - 1;
		}
		osd_hw.dispdata[OSD2].y_start = 0;
	} else {
		osd_hw.pandata[OSD2].y_start = 0;
		if (y + plane_map->src_h > disp_tmp.y_end) {
			if (y < disp_tmp.y_end)
				osd_hw.pandata[OSD2].y_end = disp_tmp.y_end - y;
		} else
			osd_hw.pandata[OSD2].y_end = plane_map->src_h - 1;
	}
	osd_hw.dispdata[OSD2].x_end = osd_hw.dispdata[OSD2].x_start +
		osd_hw.pandata[OSD2].x_end - osd_hw.pandata[OSD2].x_start;
	osd_hw.dispdata[OSD2].y_end = osd_hw.dispdata[OSD2].y_start +
		osd_hw.pandata[OSD2].y_end - osd_hw.pandata[OSD2].y_start;

	if (osd_hw.osd_reverse[OSD2] == REVERSE_TRUE) {
		x_start = osd_hw.vinfo_width[output_index]
			- osd_hw.dispdata[index].x_end - 1;
		y_start = osd_hw.vinfo_height[output_index]
			- osd_hw.dispdata[index].y_end - 1;
		x_end = osd_hw.vinfo_width[output_index]
			- osd_hw.dispdata[index].x_start - 1;
		y_end = osd_hw.vinfo_height[output_index]
			- osd_hw.dispdata[index].y_start - 1;
		osd_hw.dispdata[index].x_start = x_start;
		osd_hw.dispdata[index].y_start = y_start;
		osd_hw.dispdata[index].x_end = x_end;
		osd_hw.dispdata[index].y_end = y_end;
	} else if (osd_hw.osd_reverse[OSD2] == REVERSE_X) {
		x_start = osd_hw.vinfo_width[output_index]
			- osd_hw.dispdata[index].x_end - 1;
		x_end = osd_hw.vinfo_width[output_index]
			- osd_hw.dispdata[index].x_start - 1;
		osd_hw.dispdata[index].x_start = x_start;
		osd_hw.dispdata[index].x_end = x_end;
		} else if (osd_hw.osd_reverse[OSD2] == REVERSE_Y) {
		y_start = osd_hw.vinfo_height[output_index]
			- osd_hw.dispdata[index].y_end - 1;
		y_end = osd_hw.vinfo_height[output_index]
			- osd_hw.dispdata[index].y_start - 1;
		osd_hw.dispdata[index].y_start = y_start;
		osd_hw.dispdata[index].y_end = y_end;
	}
	osd_log_dbg2(MODULE_CURSOR,
		"plane_map:src_x=%d,src_y=%d,src_w=%d,src_h=%d\n",
		plane_map->src_x,
		plane_map->src_y,
		plane_map->src_w,
		plane_map->src_h);
	osd_log_dbg2(MODULE_CURSOR,
		"fence_map:dst_x=%d,dst_y=%d,dst_w=%d,dst_h=%d\n",
		plane_map->dst_x,
		plane_map->dst_y,
		plane_map->dst_w,
		plane_map->dst_h);
	osd_log_dbg2(MODULE_CURSOR,
		"cursor pandata x=%d,x_end=%d,y=%d,y_end=%d\n",
		osd_hw.pandata[index].x_start,
		osd_hw.pandata[index].x_end,
		osd_hw.pandata[index].y_start,
		osd_hw.pandata[index].y_end);
	osd_log_dbg2(MODULE_CURSOR,
		"cursor dispdata x=%d,x_end=%d,y=%d,y_end=%d\n",
		osd_hw.dispdata[index].x_start,
		osd_hw.dispdata[index].x_end,
		osd_hw.dispdata[index].y_start,
		osd_hw.dispdata[index].y_end);
}

void osd_page_flip(struct osd_plane_map_s *plane_map)
{
	u32 index = plane_map->plane_index;
	const struct color_bit_define_s *color = NULL;
	bool freescale_update = false;
	u32 osd_enable = 0;
	u32 format = 0;
	const struct vinfo_s *vinfo = NULL;
	u32 output_index;

	output_index = get_output_device_id(index);
	if (!osd_hw.hwc_enable[output_index]) {
		if (index >= OSD2)
			return;
	} else {
		if (index > OSD3)
			return;
	}

	osd_hw.buffer_alloc[index] = 1;
	if (osd_hw.osd_fps_start[output_index])
		osd_hw.osd_fps[output_index]++;
	osd_enable = (plane_map->enable & 1) ? ENABLE : DISABLE;

	if (output_index == VIU1)
		vinfo = get_current_vinfo();
#ifdef CONFIG_AMLOGIC_VOUT2_SERVE
	else if (output_index == VIU2)
		vinfo = get_current_vinfo2();
#endif
	if (!vinfo || (!strcmp(vinfo->name, "invalid") ||
				!strcmp(vinfo->name, "null"))) {
		return;
	}
	osd_hw.vinfo_width[output_index] = vinfo->width;
	osd_hw.vinfo_height[output_index] = vinfo->height;

	osd_hw.osd_afbcd[index].enable =
		(plane_map->afbc_inter_format & AFBC_EN) >> 31;
	if (osd_hw.osd_meson_dev.osd_ver <= OSD_NORMAL) {
		if (plane_map->phy_addr && plane_map->src_w
				&& plane_map->src_h && index == OSD1) {
			osd_hw.fb_gem[index].canvas_idx =
				osd_extra_idx[index][ext_canvas_id[index]];
			ext_canvas_id[index] ^= 1;
			color = convert_panel_format(plane_map->format);
			if (color) {
				osd_hw.color_info[index] = color;
			} else
				osd_log_err("fence color format error %d\n",
					plane_map->format);

			freescale_update = osd_direct_render(plane_map);

			if (index == OSD1 &&
				osd_hw.osd_afbcd[index].enable == ENABLE)
				osd_hw.osd_afbcd[index].phy_addr =
					plane_map->phy_addr;
			osd_hw.reg[OSD_COLOR_MODE].update_func(index);
			osd_hw.reg[DISP_GEOMETRY].update_func(index);
			osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
			if ((osd_hw.free_scale_enable[index]
				&& osd_update_window_axis)
				|| freescale_update) {
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[DISP_FREESCALE_ENABLE]
					.update_func(index);
				osd_update_window_axis = false;
			}
			if ((osd_hw.osd_afbcd[index].enable == DISABLE)
				&& (osd_enable != osd_hw.enable[index])
				&& (suspend_flag == false)) {
				osd_hw.enable[index] = osd_enable;
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[OSD_ENABLE]
						.update_func(index);
			}
			if (osd_hw.hw_rdma_en)
				osd_mali_afbc_start();
			osd_wait_vsync_hw(index);
		} else if (plane_map->phy_addr && plane_map->src_w
				&& plane_map->src_h && index == OSD2) {
			color = convert_panel_format(plane_map->format);
			if (color) {
				osd_hw.color_info[index] = color;
			} else
				osd_log_err("fence color format error %d\n",
					plane_map->format);
			if (osd_hw.hw_cursor_en)
				osd_cursor_move(plane_map);
			osd_hw.reg[OSD_COLOR_MODE].update_func(index);
			osd_hw.reg[DISP_GEOMETRY].update_func(index);
			osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
			if ((osd_enable != osd_hw.enable[index])
				&& (suspend_flag == false)) {
				osd_hw.enable[index] = osd_enable;
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[OSD_ENABLE]
						.update_func(index);
			}
			if (osd_hw.hw_rdma_en)
				osd_mali_afbc_start();
		}
	} else {
		if (plane_map->phy_addr && plane_map->src_w
				&& plane_map->src_h) {
#if 1
			osd_hw.fb_gem[index].canvas_idx =
				osd_extra_idx[index][ext_canvas_id[index]];
			ext_canvas_id[index] ^= 1;
#endif
			if (osd_hw.osd_afbcd[index].enable)
				format = plane_map->format | AFBC_EN;
			else
				format = plane_map->format;
			color = convert_panel_format(format);
			if (color) {
				osd_hw.color_info[index] = color;
			} else {
				osd_log_err("fence color format error %d\n",
					plane_map->format);
				return;
			}
			osd_hw.osd_afbcd[index].format =
				color->color_index;
			freescale_update = osd_direct_render(plane_map);

			if (osd_hw.osd_afbcd[index].enable == ENABLE)
				osd_hw.osd_afbcd[index].phy_addr =
					plane_map->phy_addr;
			osd_hw.reg[OSD_COLOR_MODE].update_func(index);
			if (!osd_hw.hwc_enable[output_index]) {
				osd_hw.reg[DISP_GEOMETRY].update_func(index);
				osd_hw.reg[DISP_OSD_REVERSE].update_func(index);
				if ((osd_hw.free_scale_enable[index]
					&& osd_update_window_axis)
					|| freescale_update) {
					if (!osd_hw.osd_display_debug
						[output_index])
					osd_hw.reg[DISP_FREESCALE_ENABLE]
					.update_func(index);
					osd_update_window_axis = false;
				}
			}
			if ((osd_hw.osd_afbcd[index].enable == DISABLE)
				&& (osd_enable != osd_hw.enable[index])
				&& (suspend_flag == false)) {
				osd_hw.enable[index] = osd_enable;
				if (!osd_hw.osd_display_debug[output_index])
					osd_hw.reg[OSD_ENABLE]
						.update_func(index);
			}
			if (osd_hw.hw_rdma_en)
				osd_mali_afbc_start();
			osd_wait_vsync_hw(index);
		}
	}
}

