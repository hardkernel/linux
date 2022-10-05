/*
 * drivers/amlogic/media/vout/backlight/bl_extern/bl_extern_i2c.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/amlogic/media/vout/lcd/aml_bl_extern.h>
#include "bl_extern.h"


static struct aml_bl_extern_i2c_dev_s *i2c_device;

struct aml_bl_extern_i2c_dev_s *aml_bl_extern_i2c_get_dev(void)
{
	return i2c_device;
}

int bl_extern_i2c_write(struct i2c_client *i2client,
		unsigned char *buff, unsigned int len)
{
	struct i2c_msg msg;
	int ret = 0;

	if (i2client == NULL) {
		BLEXERR("i2client is null\n");
		return -1;
	}

	msg.addr = i2client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buff;

	ret = i2c_transfer(i2client->adapter, &msg, 1);
	if (ret < 0)
		BLEXERR("i2c write failed [addr 0x%02x]\n", i2client->addr);

	return ret;
}

int bl_extern_i2c_read(struct i2c_client *i2client,
		unsigned char *buff, unsigned int len)
{
	struct i2c_msg msgs[2];
	int ret = 0;

	if (i2client == NULL) {
		BLEXERR("i2client is null\n");
		return -1;
	}

	msgs[0].addr = i2client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buff;
	msgs[1].addr = i2client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buff;

	ret = i2c_transfer(i2client->adapter, msgs, 2);
	if (ret < 0) {
		BLEXERR("%s: i2c transfer failed [addr 0x%02x]\n",
			__func__, i2client->addr);
	}

	return ret;
}

static int bl_extern_i2c_config_from_dts(struct device *dev,
		struct aml_bl_extern_i2c_dev_s *i2c_dev)
{
	int ret;
	struct device_node *np = dev->of_node;
	const char *str;

	ret = of_property_read_string(np, "dev_name", &str);
	if (ret) {
		BLEXERR("failed to get dev_name\n");
		strcpy(i2c_dev->name, "none");
	} else {
		strcpy(i2c_dev->name, str);
	}

	return 0;
}

static int aml_bl_extern_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		BLEXERR("i2c check functionality failed.");
		return -ENODEV;
	}

	i2c_device = kzalloc(sizeof(struct aml_bl_extern_i2c_dev_s),
		GFP_KERNEL);
	if (!i2c_device) {
		BLEXERR("driver malloc error\n");
		return -ENOMEM;
	}
	i2c_device->client = client;
	BLEX("i2c Address: 0x%02x", i2c_device->client->addr);

	bl_extern_i2c_config_from_dts(&client->dev, i2c_device);

	return 0;
}

static int aml_bl_extern_i2c_remove(struct i2c_client *client)
{
	kfree(i2c_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id aml_bl_extern_i2c_id[] = {
	{"bl_extern_i2c", 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aml_bl_extern_i2c_dt_match[] = {
	{
		.compatible = "bl_extern, i2c",
	},
	{},
};
#endif

static struct i2c_driver aml_bl_extern_i2c_driver = {
	.probe  = aml_bl_extern_i2c_probe,
	.remove = aml_bl_extern_i2c_remove,
	.id_table   = aml_bl_extern_i2c_id,
	.driver = {
		.name  = "bl_extern_i2c",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aml_bl_extern_i2c_dt_match,
#endif
	},
};

static int __init aml_bl_extern_i2c_init(void)
{
	int ret;

	if (bl_debug_print_flag)
		BLEX("%s\n", __func__);

	ret = i2c_add_driver(&aml_bl_extern_i2c_driver);
	if (ret) {
		BLEXERR("driver register failed\n");
		return -ENODEV;
	}
	return ret;
}

static void __exit aml_bl_extern_i2c_exit(void)
{
	i2c_del_driver(&aml_bl_extern_i2c_driver);
}


module_init(aml_bl_extern_i2c_init);
module_exit(aml_bl_extern_i2c_exit);

MODULE_AUTHOR("AMLOGIC");
MODULE_DESCRIPTION("bl extern driver");
MODULE_LICENSE("GPL");

