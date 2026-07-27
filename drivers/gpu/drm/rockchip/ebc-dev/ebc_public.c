// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include "ebc_dev.h"

static BLOCKING_NOTIFIER_HEAD(ebc_notifier_list);

int ebc_register_notifier(struct notifier_block *nb)
{
	int ret = 0;
	ret = blocking_notifier_chain_register(&ebc_notifier_list, nb);

	return ret;
}
EXPORT_SYMBOL(ebc_register_notifier);

int ebc_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ebc_notifier_list, nb);
}
EXPORT_SYMBOL(ebc_unregister_notifier);

int ebc_notify(unsigned long event)
{
	return blocking_notifier_call_chain(&ebc_notifier_list, event, NULL);
}

int ebc_get_init_log_level(void)
{
	return DEBUG_LEVEL_0;
}

int ebc_overlay_enable_control(void)
{
	/*
	 * (1 << 0) / 0: overlay enable when EBC_ENABLE_OVERLAY
	 * (1 << 1): overlay enable when EBC_PEN_PROXIMITY
	 */
	return 3;
}
