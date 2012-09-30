/*
 * drivers/misc/usb3503.c - usb3503 usb hub driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/platform_data/usb3503.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>

static int usb3503_register_write(struct i2c_client *i2c_dev, char reg,
	char data)
{
	int ret;
	char buf[2];
	struct i2c_msg msg[] = {
		{
			.addr = i2c_dev->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	buf[0] = reg;
	buf[1] = data;

	ret = i2c_transfer(i2c_dev->adapter, msg, 1);
	if (ret < 0)
		pr_err(HUB_TAG "%s: reg: %x data: %x write failed\n",
		__func__, reg, data);

	return ret;
}

static int usb3503_register_read(struct i2c_client *i2c_dev, char reg,
	char *data)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = i2c_dev->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = i2c_dev->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};

	ret = i2c_transfer(i2c_dev->adapter, msgs, 2);
	if (ret < 0)
		pr_err(HUB_TAG "%s: reg: %x read failed\n", __func__, reg);

	return ret;
}

void s5pv210_hsic_port1_power(int enable)
{
	/*TODO:*/
}

static int reg_write(struct i2c_client *i2c_dev, char reg, char req, int retry)
{
	int cnt = retry, err;
	char data;

	pr_debug(HUB_TAG "%s: write %02X, data: %02x\n", __func__, reg, req);
	do {
		err = usb3503_register_write(i2c_dev, reg, req);
		if (err < 0)
			goto exit;
		err = usb3503_register_read(i2c_dev, reg, &data);
		if (err < 0)
			goto exit;
	} while (data != req && cnt--);
exit:
	pr_info(HUB_TAG "%s: write %02X, req:%02x, val:%02x\n", __func__, reg,
		req, data);

	return err;
}

static int reg_update(struct i2c_client *i2c_dev, char reg, char req, int retry)
{
	int cnt = retry, err;
	char data;

	pr_debug(HUB_TAG "%s: update %02X, data: %02x\n", __func__, reg, req);
	do {
		err = usb3503_register_read(i2c_dev, reg, &data);
		if (err < 0)
			goto exit;
		pr_debug(HUB_TAG "%s: read %02X, data: %02x\n", __func__, reg,
			data);
		if ((data & req) == req) {
			pr_debug(HUB_TAG "%s: aleady set data: %02x\n",
				__func__, data);
			break;
		}
		err = usb3503_register_write(i2c_dev, reg, data | req);
		if (err < 0)
			goto exit;
	} while (cnt--);
exit:
	pr_info(HUB_TAG "%s: update %02X, req:%02x, val:%02x\n", __func__, reg,
		req, data);
	return err;
}

static int reg_clear(struct i2c_client *i2c_dev, char reg, char req, int retry)
{
	int cnt = retry, err;
	char data;

	pr_debug(HUB_TAG "%s: clear %X, data %x\n", __func__, reg, req);
	do {
		err = usb3503_register_read(i2c_dev, reg, &data);
		if (err < 0)
			goto exit;
		pr_debug(HUB_TAG "%s: read %02X, data %02x\n", __func__, reg,
			data);
		if (!(data & req)) {
			pr_err(HUB_TAG "%s: aleady cleared data = %02x\n",
				__func__, data);
			break;
		}
		err = usb3503_register_write(i2c_dev, reg, data & ~req);
		if (err < 0)
			goto exit;
	} while (cnt--);
exit:
	pr_info(HUB_TAG "%s: clear %02X, req:%02x, val:%02x\n", __func__, reg,
		req, data);
	return err;
}

static int usb3503_reset(struct usb3503_hubctl *hc, int reset)
{
	if ((hc == NULL) || (hc->reset_n == NULL))
		return -EINVAL;

	/* Assert RESET_N signal */
	hc->reset_n(reset);

	if (reset == 1) {
		/* Wait RefClk when RESET_N is released, otherwise Hub will
		 * not transition to Hub Communication Stage.
		 */
		msleep(100);
	}

	return 0;
}

static int usb3503_set_mode(struct usb3503_hubctl *hc, int mode)
{
	int err = 0;
	struct i2c_client *i2c_dev = hc->i2c_dev;

	pr_info(HUB_TAG "%s: mode = %d\n", __func__, mode);

	switch (mode) {
	case USB3503_MODE_HUB:
		usb3503_reset(hc, 1);

		/* SP_ILOCK: set connect_n, config_n for config */
		err = reg_write(i2c_dev, SP_ILOCK_REG,
			(SPILOCK_CONNECT_N | SPILOCK_CONFIG_N), 3);
		if (err < 0) {
			pr_err(HUB_TAG "SP_ILOCK write fail err = %d\n", err);
			goto exit;
		}

#ifdef USB3503_ES_VER
/* ES version issue
 * USB3503 can't PLL power up under cold circumstance, so enable
 * the Force suspend clock bit
 */
		err = reg_update(i2c_dev, CFGP_REG, CFGP_CLKSUSP, 1);
		if (err < 0) {
			pr_err(HUB_TAG "CFGP update fail err = %d\n", err);
			goto exit;
		}
#endif

		/* PDS : Port2,3 Disable For Self Powered Operation */

		err = -1;
#if defined(CONFIG_USB3503_PORTS_SELF_POWER_DISABLED)
		err = reg_update(i2c_dev, PDS_REG, (PDS_PORT2 | PDS_PORT3), 1);
#endif
		if (err < 0) {
			pr_err(HUB_TAG "PDS update fail err = %d\n", err);
			goto exit;
		}
		/* CFG1 : SELF_BUS_PWR -> Self-Powerd operation */
		err = reg_update(i2c_dev, CFG1_REG, CFG1_SELF_BUS_PWR, 1);
		if (err < 0) {
			pr_err(HUB_TAG "CFG1 update fail err = %d\n", err);
			goto exit;
		}
		/* SP_LOCK: clear connect_n, config_n for hub connect */
		err = reg_clear(i2c_dev, SP_ILOCK_REG,
			(SPILOCK_CONNECT_N | SPILOCK_CONFIG_N), 1);
		if (err < 0) {
			pr_err(HUB_TAG "SP_ILOCK clear err = %d\n", err);
			goto exit;
		}
		hc->mode = mode;

		/* Should be enable the HSIC port1 */

		break;

	case USB3503_MODE_STANDBY:
		usb3503_reset(hc, 0);
		hc->mode = mode;
		break;

	default:
		pr_err(HUB_TAG "%s: Invalid mode %d\n", __func__, mode);
		err = -EINVAL;
		goto exit;
		break;
	}
exit:
	return err;
}

/* sysfs for control */
static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	if (hc->mode == USB3503_MODE_HUB)
		return sprintf(buf, "%s", "hub");
	else if (hc->mode == USB3503_MODE_STANDBY)
		return sprintf(buf, "%s", "standby");

	return 0;
}

static ssize_t mode_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	if (!strncmp(buf, "hub", 3)) {
		/*usb3503_set_mode(hc, USB3503_MODE_HUB);*/
		if (hc->port_enable)
			hc->port_enable(2, 1);
		pr_debug(HUB_TAG "mode set to hub\n");
	} else if (!strncmp(buf, "standby", 7)) {
		/*usb3503_set_mode(hc, USB3503_MODE_STANDBY);*/
		if (hc->port_enable)
			hc->port_enable(2, 0);
		pr_debug(HUB_TAG "mode set to standby\n");
	}
	return size;
}
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);

#ifdef USB3503_SYSFS_DEBUG
static ssize_t read_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned addr;
	char data;
	int err;
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	err = sscanf(buf, "%x", &addr);

	err = usb3503_register_read(hc->i2c_dev, addr, &data);
	if (err < 0) {
		pr_err(HUB_TAG "register read fail\n");
		goto exit;
	}
	pr_info(HUB_TAG "%s: read 0x%x = 0x%x\n", __func__, addr, data);
exit:
	return size;
}
static DEVICE_ATTR(read, 0664, NULL, read_store);

static ssize_t write_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned addr, data;
	int err;
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	err = sscanf(buf, "%x %x", &addr, &data);
	pr_debug(HUB_TAG "%s: addr=%x, data=%x\n", __func__, addr, data);

	err = usb3503_register_write(hc->i2c_dev, addr, data);
	if (err < 0) {
		pr_err(HUB_TAG "register write fail\n");
		goto exit;
	}

	err = usb3503_register_read(hc->i2c_dev, addr, (char *)&data);
	if (err < 0) {
		pr_err(HUB_TAG "register read fail\n");
		goto exit;
	}
	pr_info(HUB_TAG "%s: write 0x%x = 0x%x\n", __func__, addr, data);
exit:
	return size;
}
static DEVICE_ATTR(write, 0664, NULL, write_store);

static ssize_t reset_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned val;
	int err;
	struct usb3503_hubctl *hc = dev_get_drvdata(dev);

	err = sscanf(buf, "%x", &val);
	pr_info(HUB_TAG "%s: val=%x\n", __func__, val);

	usb3503_reset(hc, val);

	return size;
}
static DEVICE_ATTR(reset, 0664, NULL, reset_store);
#endif /* end of USB3503_SYSFS_DEBUG */

int usb3503_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);

	/* Should be disable the HSIC port1 */

	usb3503_reset(hc, 0);
	pr_info(HUB_TAG "suspended\n");

	return 0;
}

int usb3503_resume(struct i2c_client *client)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);

	if (hc->mode == USB3503_MODE_HUB)
		usb3503_set_mode(hc, USB3503_MODE_HUB);

	pr_info(HUB_TAG "resume mode=%s", (hc->mode == USB3503_MODE_HUB) ?
		"hub" : "standby");

	return 0;
}

int usb3503_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct usb3503_hubctl *hc;
	struct usb3503_platform_data *pdata;

	pr_info(HUB_TAG "%s:%d\n", __func__, __LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		pr_err(HUB_TAG "device's platform data is NULL!\n");
		err = -ENODEV;
		goto exit;
	}

	hc = kzalloc(sizeof(struct usb3503_hubctl), GFP_KERNEL);
	if (!hc) {
		pr_err(HUB_TAG "private data alloc fail\n");
		err = -ENOMEM;
		goto exit;
	}
	hc->i2c_dev = client;
	hc->reset_n = pdata->reset_n;
	hc->port_enable = pdata->port_enable;
	if (pdata->initial_mode) {
		usb3503_set_mode(hc, pdata->initial_mode);
		hc->mode = pdata->initial_mode;
	}
	/* For HSIC to USB brige with CMC221
	 * export the hub_set_mode and private data to board modem
	 * it will be handled by PM scenaio.
	 */
	if (pdata->register_hub_handler)
		pdata->register_hub_handler((void (*)(void))usb3503_set_mode,
			(void *)hc);

	i2c_set_clientdata(client, hc);

	err = device_create_file(&client->dev, &dev_attr_mode);
#ifdef USB3503_SYSFS_DEBUG
	err = device_create_file(&client->dev, &dev_attr_read);
	err = device_create_file(&client->dev, &dev_attr_write);
	err = device_create_file(&client->dev, &dev_attr_reset);
#endif
	pr_info(HUB_TAG "%s: probed on  %s mode\n", __func__,
		(hc->mode == USB3503_MODE_HUB) ? "hub" : "standby");
exit:
	return err;
}

static int usb3503_remove(struct i2c_client *client)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);

	pr_debug(HUB_TAG "%s\n", __func__);
	kfree(hc);

	return 0;
}

static const struct i2c_device_id usb3503_id[] = {
	{ USB3503_I2C_NAME, 0 },
	{ }
};

static void usb3503_shutdown(struct i2c_client *client)
{
	struct usb3503_hubctl *hc = i2c_get_clientdata(client);

	pr_err(HUB_TAG "%s:\n", __func__);
	mdelay(10);
	usb3503_set_mode(hc, USB3503_MODE_STANDBY);
}

static struct i2c_driver usb3503_driver = {
	.probe = usb3503_probe,
	.remove = usb3503_remove,
	.suspend = usb3503_suspend,
	.resume = usb3503_resume,
	.shutdown = usb3503_shutdown,
	.id_table = usb3503_id,
	.driver = {
		.name = USB3503_I2C_NAME,
	},
};

static int __init usb3503_init(void)
{
	pr_info(HUB_TAG "USB HUB driver init\n");
	return i2c_add_driver(&usb3503_driver);
}

static void __exit usb3503_exit(void)
{
	pr_info(HUB_TAG "USB HUB driver exit\n");
	i2c_del_driver(&usb3503_driver);
}
module_init(usb3503_init);
module_exit(usb3503_exit);

MODULE_DESCRIPTION("USB3503 USB HUB driver");
MODULE_LICENSE("GPL");
