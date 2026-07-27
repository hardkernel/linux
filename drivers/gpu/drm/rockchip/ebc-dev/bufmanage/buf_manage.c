// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>

#include "../ebc_dev.h"
#include "buf_manage.h"
#include "buf_list.h"

struct buf_info_s {
	unsigned long phy_mem_base;
	char *virt_mem_base;

	struct buf_list_s *buf_list; /* buffer list. */
	int use_buf_is_empty;

	struct buf_list_s *dsp_buf_list; /* dispplay buffer list. */
	int dsp_buf_list_status;
	struct ebc_buf_s *osd_buf;
	struct buf_list_s *osd_buf_list; /* dispplay buffer list. */
	struct mutex osd_buf_lock;
	struct mutex dsp_buf_lock;
	struct mutex ebc_buf_lock;
};

static struct buf_info_s ebc_buf_info;
static DECLARE_WAIT_QUEUE_HEAD(ebc_buf_wq);

int ebc_buf_release(struct ebc_buf_s  *release_buf)
{
	struct ebc_buf_s *temp_buf = release_buf;

	if (temp_buf) {
		if (temp_buf->status == buf_osd) {
			kfree(temp_buf);
		} else {
			temp_buf->status = buf_idle;
			if (1 == ebc_buf_info.use_buf_is_empty) {
				ebc_buf_info.use_buf_is_empty = 0;
				wake_up_interruptible_sync(&ebc_buf_wq);
			}
		}
	}

	return BUF_SUCCESS;
}

int ebc_drop_one_dsp_buf(void)
{
	struct ebc_buf_s *temp_buf;
	int temp_pos;

	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list) {
		if (ebc_buf_info.dsp_buf_list->nb_elt > 0) {
			temp_pos = ebc_buf_info.dsp_buf_list->nb_elt - 1;
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.dsp_buf_list, temp_pos);
			if (temp_buf->undroppable == 0) {
				buf_list_remove(ebc_buf_info.dsp_buf_list, temp_pos);
				ebc_buf_release(temp_buf);
				mutex_unlock(&ebc_buf_info.dsp_buf_lock);
				return BUF_SUCCESS;
			}
		}
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return BUF_ERROR;
}

static int ebc_add_to_dsp_buf_list(struct ebc_buf_s *dsp_buf, int pos)
{
	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list) {
		if (-1 == buf_list_add(ebc_buf_info.dsp_buf_list, (int *)dsp_buf, pos)) {
			ebc_buf_release(dsp_buf);
			mutex_unlock(&ebc_buf_info.dsp_buf_lock);
			return BUF_ERROR;
		}

		if (dsp_buf->status != buf_osd)
			dsp_buf->status = buf_dsp;
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return BUF_SUCCESS;
}

int ebc_add_to_dsp_buf_list_tail(struct ebc_buf_s *dsp_buf)
{
	return ebc_add_to_dsp_buf_list(dsp_buf, -1);
}

int ebc_add_to_dsp_buf_list_head(struct ebc_buf_s *dsp_buf)
{
	return ebc_add_to_dsp_buf_list(dsp_buf, 0);
}

static int ebc_add_to_osd_buf_list(struct ebc_buf_s *dsp_buf, int pos)
{
	int ret = BUF_SUCCESS;

	mutex_lock(&ebc_buf_info.osd_buf_lock);
	if (ebc_buf_info.osd_buf_list) {
		if (-1 == buf_list_add(ebc_buf_info.osd_buf_list, (int *)dsp_buf, pos)) {
			ebc_buf_release(dsp_buf);
			ret = BUF_ERROR;
		}
	}
	mutex_unlock(&ebc_buf_info.osd_buf_lock);
	return ret;
}

int ebc_add_to_osd_buf_list_tail(struct ebc_buf_s *dsp_buf)
{
	return ebc_add_to_osd_buf_list(dsp_buf, -1);
}

int ebc_add_to_osd_buf_list_head(struct ebc_buf_s *dsp_buf)
{
	return ebc_add_to_osd_buf_list(dsp_buf, 0);
}

struct ebc_buf_s *ebc_osd_buf_get(void)
{
	struct ebc_buf_s *buf = NULL;

	mutex_lock(&ebc_buf_info.osd_buf_lock);
	if (ebc_buf_info.osd_buf_list && (ebc_buf_info.osd_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.osd_buf_list, 0);
	}
	mutex_unlock(&ebc_buf_info.osd_buf_lock);

	return buf;
}

struct ebc_buf_s *ebc_osd_buf_pop_get(void)
{
	struct ebc_buf_s *buf = NULL;

	mutex_lock(&ebc_buf_info.osd_buf_lock);
	if (ebc_buf_info.osd_buf_list && (ebc_buf_info.osd_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.osd_buf_list, 0);
		buf_list_remove(ebc_buf_info.osd_buf_list, 0);
	}
	mutex_unlock(&ebc_buf_info.osd_buf_lock);

	return buf;
}

int ebc_get_dsp_list_enum_num(void)
{
	return ebc_buf_info.dsp_buf_list->nb_elt;
}

int ebc_get_osd_list_enum_num(void)
{
	return ebc_buf_info.osd_buf_list->nb_elt;
}

struct ebc_buf_s *ebc_find_buf_by_id(int id)
{
	struct ebc_buf_s *temp_buf;
	int temp_pos;

	if (ebc_buf_info.buf_list) {
		temp_pos = 0;
		while (temp_pos < ebc_buf_info.buf_list->nb_elt) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, temp_pos++);
			if (temp_buf && (temp_buf->id == id))
				return temp_buf;
		}
	}

	return NULL;
}

struct ebc_buf_s *ebc_dsp_buf_get(void)
{
	struct ebc_buf_s *buf = NULL;

	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list && (ebc_buf_info.dsp_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.dsp_buf_list, 0);
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return buf;
}

struct ebc_buf_s *ebc_dsp_buf_pop_get(void)
{
	struct ebc_buf_s *buf = NULL;

	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list && (ebc_buf_info.dsp_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.dsp_buf_list, 0);
		buf_list_remove(ebc_buf_info.dsp_buf_list, 0);
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return buf;
}

int ebc_dsp_buf_next_mode(void)
{
	struct ebc_buf_s *buf = NULL;
	int mode = -1;

	mutex_lock(&ebc_buf_info.dsp_buf_lock);
	if (ebc_buf_info.dsp_buf_list && (ebc_buf_info.dsp_buf_list->nb_elt > 0)) {
		buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.dsp_buf_list, 0);
		mode = buf->buf_mode;
		buf->undroppable = 1;
	}
	mutex_unlock(&ebc_buf_info.dsp_buf_lock);

	return mode;
}

struct ebc_buf_s *ebc_empty_osd_buf_get(void)
{
	if (ebc_buf_info.osd_buf)
		return ebc_buf_info.osd_buf;
	return NULL;
}

struct ebc_buf_s *ebc_osd_buf_clone(void)
{
	struct ebc_buf_s *temp_buf;

	temp_buf = kzalloc(sizeof(*temp_buf), GFP_KERNEL);
	if (NULL == temp_buf)
		return NULL;

	temp_buf->virt_addr = ebc_buf_info.osd_buf->virt_addr;
	temp_buf->id = ebc_buf_info.osd_buf->id;
	temp_buf->status = buf_osd;

	return temp_buf;
}

struct ebc_buf_s *ebc_empty_buf_get(const char *tid_name)
{
	struct ebc_buf_s *temp_buf = NULL;
	int temp_pos;

	mutex_lock(&ebc_buf_info.ebc_buf_lock);
	while (ebc_buf_info.buf_list) {
		temp_pos = 0;

		while (temp_pos < ebc_buf_info.buf_list->nb_elt) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, temp_pos++);
			if (temp_buf) {
				if (temp_buf->status == buf_idle) {
					temp_buf->status = buf_user;
					memcpy(temp_buf->tid_name, tid_name, TASK_COMM_LEN - 1); //store user thread name
					goto OUT;
				}
				// one tid only can get one buf at one time
				else if ((temp_buf->status == buf_user) && (!strncmp(temp_buf->tid_name, tid_name, TASK_COMM_LEN - 1))) {
					printk("[%s]: one tid only can get one buf at one time\n", tid_name);
					goto OUT;
				}
			}
		}
		ebc_buf_info.use_buf_is_empty = 1;
		wait_event_interruptible(ebc_buf_wq, ebc_buf_info.use_buf_is_empty != 1);
	}

OUT:
	mutex_unlock(&ebc_buf_info.ebc_buf_lock);
	return temp_buf;
}

unsigned long ebc_phy_buf_base_get(void)
{
	return ebc_buf_info.phy_mem_base;
}

char *ebc_virt_buf_base_get(void)
{
	return ebc_buf_info.virt_mem_base;
}

int ebc_buf_state_show(char *buf)
{
	int i;
	int ret = 0;
	struct ebc_buf_s *temp_buf;

	ret += sprintf(buf, "dsp_buf num = %d\n", ebc_buf_info.dsp_buf_list->nb_elt);
	if (ebc_buf_info.buf_list) {
		for (i = 0; i < ebc_buf_info.buf_list->nb_elt; i++) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, i);
			ret += sprintf(buf + ret, "ebc_buf[%d]: s = %d, m = %d, tid = %s\n", i, temp_buf->status, temp_buf->buf_mode, temp_buf->tid_name);
		}
	}

	return ret;
}

int ebc_buf_uninit(void)
{
	struct ebc_buf_s *temp_buf;
	int pos;

	if (ebc_buf_info.buf_list) {
		pos = ebc_buf_info.buf_list->nb_elt - 1;
		while (pos >= 0) {
			temp_buf = (struct ebc_buf_s *)buf_list_get(ebc_buf_info.buf_list, pos);
			if (temp_buf)
				kfree(temp_buf);
			buf_list_remove(ebc_buf_info.buf_list, pos);
			pos--;
		}
	}

	return BUF_SUCCESS;
}

int ebc_buf_list_init(void)
{
	int res;

	mutex_init(&ebc_buf_info.dsp_buf_lock);
	mutex_init(&ebc_buf_info.ebc_buf_lock);
	mutex_init(&ebc_buf_info.osd_buf_lock);

	if (buf_list_init(&ebc_buf_info.buf_list, BUF_LIST_MAX_NUMBER))
		return BUF_ERROR;

	if (buf_list_init(&ebc_buf_info.dsp_buf_list, BUF_LIST_MAX_NUMBER)) {
		res = BUF_ERROR;
		goto dsp_list_err;
	}

	if (buf_list_init(&ebc_buf_info.osd_buf_list, BUF_LIST_MAX_NUMBER)) {
		res = BUF_ERROR;
		goto osd_list_err;
	}

	return BUF_SUCCESS;

osd_list_err:
	buf_list_uninit(ebc_buf_info.dsp_buf_list);
dsp_list_err:
	buf_list_uninit(ebc_buf_info.buf_list);

	return res;
}

int ebc_dsp_buf_init(int id, char *vaddr, size_t size)
{
	struct ebc_buf_s *temp_buf;
	int res = 0;

	temp_buf = kzalloc(sizeof(*temp_buf), GFP_KERNEL);
	if (NULL == temp_buf) {
		res = BUF_ERROR;
		goto exit;
	}
	temp_buf->id = id;
	temp_buf->virt_addr = vaddr;
	temp_buf->len = size;
	temp_buf->status = buf_idle;

	if (-1 == buf_list_add(ebc_buf_info.buf_list, (int *)temp_buf, -1)) {
		res = BUF_ERROR;
		goto exit;
	}
	ebc_buf_info.use_buf_is_empty = 0;

exit:
	return res;
}

int ebc_osd_buf_init(int id, char *vaddr, size_t size)
{
	struct ebc_buf_s *temp_buf;
	int res = 0;

	temp_buf = kzalloc(sizeof(*temp_buf), GFP_KERNEL);
	if (NULL == temp_buf) {
		res = BUF_ERROR;
		goto exit;
	}
	temp_buf->id = id;
	temp_buf->virt_addr = vaddr;
	temp_buf->len = size;
	temp_buf->status = buf_osd;

	ebc_buf_info.osd_buf = temp_buf;

exit:
	return res;
}
