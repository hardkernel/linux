/*
 * drivers/input/touchscreen/gt801_ts.c
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
//#include <mach/iomux.h>
#include <linux/platform_device.h>
#include "gt801_ts.h"
#include "linux/amlogic/input/common.h"
#include "goodix_queue.h"

#define GT801_REGS_NUM 53
#define GPIO_LOW 0
#define GPIO_HIGH 1

#define NEED_TO_SWAP_XY(pol) 	(((pol) >> 2) & 0x1) // if pol ==4, then need to swap xy
#define NEED_TO_SWAP_Y(pol) 	(((pol) >> 1) & 0x1)
#define NEED_TO_SWAP_X(pol) 	((pol) & 0x1)

/*tochscreen private data*/
static struct workqueue_struct *gt801_wq = NULL;
static struct point_queue finger_list;	//record the fingers list

const unsigned char GT801_RegData[GT801_REGS_NUM] = { 0x19, 0x05, 0x04, 0x28,
		0x02, 0x14, 0x20, 0x10, 0x50, 0xB2, 0x1, 0xE0, 0x3, 0x20, 0x01, 0x23,
		0x45, 0x67, 0x89, 0xAB, 0xCD, 0xE1, 0x00, 0x00, 0x2D, 0x29, 0x45, 0xCF,
		0x20, 0x01, 0x01, 0x83, 0x50, 0x3C, 0x1E, 0xB4, 0x00, 0x2B, 0x27, 0x01,
		0xB4, 0x00, 0x64, 0x32, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01 };

static int i2c_master_reg8_send(const struct i2c_client *client, const char reg,
		const char *buf, int count, int scl_rate) {
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;
	char *tx_buf = (char *) kzalloc(count + 1, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;
	tx_buf[0] = reg;
	memcpy(tx_buf + 1, buf, count);

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = count + 1;
	msg.buf = (char *) tx_buf;

	ret = i2c_transfer(adap, &msg, 1);
	kfree(tx_buf);
	return (ret == 1) ? count : ret;

}

static int i2c_master_reg8_recv(const struct i2c_client *client, const char reg,
		char *buf, int count, int scl_rate) {
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msgs[2];
	int ret;
	char reg_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len = count;
	msgs[1].buf = (char *) buf;

	ret = i2c_transfer(adap, msgs, 2);

	return (ret == 2) ? count : ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt801_ts_early_suspend(struct early_suspend *h);
static void gt801_ts_late_resume(struct early_suspend *h);
#endif

/*read the gt801 register ,used i2c bus*/
static int gt801_read_regs(struct i2c_client *client, u8 reg, u8 buf[],
		unsigned len) {
	int ret;
	ret = i2c_master_reg8_recv(client, reg, buf, len, 200 * 1000);
	if (ret < 0)
		gt801printk("gt801_ts_work_func:i2c_transfer fail =%d\n",ret);
	return ret;
}
/* set the gt801 registe,used i2c bus*/
static int gt801_write_regs(struct i2c_client *client, u8 reg, u8 const buf[],
		unsigned short len) {
	int ret;
	ret = i2c_master_reg8_send(client, reg, buf, len, 200 * 1000);
	if (ret < 0) {
		gt801printk("gt801_ts_work_func:i2c_transfer fail =%d\n",ret);
	}
	return ret;
}
static int setup_resetPin(struct i2c_client *client, struct gt801_ts_data *ts) {
	struct touch_pdata *pdata = client->dev.platform_data;
	int err;

	ts->gpio_reset = pdata->gpio_reset;
	ts->gpio_reset_active_low = pdata->pol;

	gt801printk("%s=%d,%d\n",__FUNCTION__,ts->gpio_reset,ts->gpio_reset_active_low);
	if (!gpio_is_valid(ts->gpio_reset)) {
		dev_err(&client->dev, "no gpio_reset?\n");
		return -EINVAL;
	}

	err = gpio_request(ts->gpio_reset, "gt801_resetPin");
	if (err) {
		dev_err(&client->dev, "failed to request resetPin GPIO%d\n",
				ts->gpio_reset);
		return err;
	}

	err = gpio_direction_output(ts->gpio_reset,
			ts->gpio_reset_active_low ? GPIO_LOW : GPIO_HIGH);
	if (err) {
		dev_err(&client->dev, "failed to pulldown resetPin GPIO%d,err%d\n",
				ts->gpio_reset, err);
		gpio_free(ts->gpio_reset);
		return err;
	}
	mdelay(100);
	gpio_set_value(ts->gpio_reset,
			ts->gpio_reset_active_low ? GPIO_HIGH : GPIO_LOW);
	mdelay(100);

	return 0;
}
static int gt801_init_panel(struct gt801_ts_data *ts) {
	return 0;
}

static int calc_checksum(char* point_data) {
	int count = 0;
	int check_sum = 0;
	//The bit indicate which fingers pressed down
	switch (point_data[1] & 0x1f) {
	case 0:
	case 1:
		for (count = 1; count < 8; count++) {
			check_sum += (int) point_data[count];
		}

		if ((check_sum % 256) != point_data[8]) {
			return -1;
		}
		break;
	case 2:
	case 3:
		for (count = 1; count < 13; count++)
			check_sum += (int) point_data[count];
		if ((check_sum % 256) != point_data[13])
			return -1;
		break;
	default:		//(point_data[1]& 0x1f) > 3
		for (count = 1; count < 34; count++)
			check_sum += (int) point_data[count];
		if ((check_sum % 256) != point_data[34])
			return -1;
	}
	return 0;

}
static void gt801_ts_work_func(struct work_struct *work) {
	struct gt801_ts_data *ts = container_of(work, struct gt801_ts_data, work);
	//struct touch_pdata *pdata = ts->client->dev.platform_data;
	static uint8_t finger_bit = 0;	//last time fingers' state
	uint8_t read_position = 0;
	uint8_t point_data[35] = { 0 };
	uint8_t finger = 0;				//record which finger is changed
	int ret = -1;
	int count = 0;

	ret = gt801_read_regs(ts->client, 0, &point_data[1], 34);
	if (ret < 0) {
		gt801printk("%s:i2c_transfer fail =%d\n",__FUNCTION__,ret);
		if (ts->use_irq) {
			enable_irq(ts->client->irq);
		}
		return;
	}

	ret = calc_checksum(point_data);
	if (ret < 0) {
		//error case
		if (ts->use_irq) {
			enable_irq(ts->client->irq);
		}
		return;
	}

	point_data[1] &= 0x1f;
	finger = finger_bit ^ point_data[1];

	if (finger == 0 && point_data[1] == 0) {
		// no action here
		if (ts->use_irq)
			enable_irq(ts->client->irq);
		return;
	}
	//no fingers and no action
	else if (finger == 0) {				//the same as last time

	} else {
		//check which point(s) DOWN or UP
		for (count = 0; (finger != 0) && (count < MAX_FINGER_NUM); count++) {
			if ((finger & 0x01) == 1)//current bit is 1, so NO.postion finger is change
					{
				if (((finger_bit >> count) & 0x01) == 1)//NO.postion finger is UP
					set_up_point(&finger_list, count);
				else
					add_point(&finger_list, count);
			}
			finger >>= 1;
		}
	}

	for (count = 0; count < finger_list.length; count++) {
		if (finger_list.pointer[count].state == FLAG_UP) {
			finger_list.pointer[count].x = finger_list.pointer[count].y = 0;
			finger_list.pointer[count].pressure = 0;
			continue;
		}

		if (finger_list.pointer[count].num < 3)
			read_position = finger_list.pointer[count].num * 5 + 3;
		else if (finger_list.pointer[count].num == 4)
			read_position = 29;

		if (finger_list.pointer[count].num != 3) {
			finger_list.pointer[count].x =
					(unsigned int) (point_data[read_position] << 8)
							+ (unsigned int) (point_data[read_position + 1]);
			finger_list.pointer[count].y =
					(unsigned int) (point_data[read_position + 2] << 8)
							+ (unsigned int) (point_data[read_position + 3]);
			finger_list.pointer[count].pressure =
					(unsigned int) (point_data[read_position + 4]);
		} else {
			finger_list.pointer[count].x = (unsigned int) (point_data[18] << 8)
					+ (unsigned int) (point_data[25]);
			finger_list.pointer[count].y = (unsigned int) (point_data[26] << 8)
					+ (unsigned int) (point_data[27]);
			finger_list.pointer[count].pressure =
					(unsigned int) (point_data[28]);
		}

		if (NEED_TO_SWAP_XY(ts->pol)) {
			swap(finger_list.pointer[count].x, finger_list.pointer[count].y);
		}
		if(NEED_TO_SWAP_X(ts->pol)) {
			finger_list.pointer[count].x = ts->x_max - finger_list.pointer[count].x;
		}
		if(NEED_TO_SWAP_Y(ts->pol)) {
			finger_list.pointer[count].y = ts->y_max - finger_list.pointer[count].y;
		}
	}

	/* ABS_MT_TOUCH_MAJOR is used as ABS_MT_PRESSURE in android. */
	for (count = 0; count < (finger_list.length); count++) {

		input_mt_slot(ts->input_dev, finger_list.pointer[count].num);

		if (finger_list.pointer[count].state == FLAG_DOWN) {

			gt801printk("%s: dx = %d dy = %d pressure =%d, num =%d\n", __func__,
					finger_list.pointer[count].x,
					finger_list.pointer[count].y,
					finger_list.pointer[count].pressure,
					finger_list.pointer[count].num);

			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
					finger_list.pointer[count].num + 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					finger_list.pointer[count].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					finger_list.pointer[count].y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					finger_list.pointer[count].pressure);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					finger_list.pointer[count].pressure);
		} else {
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}
	input_sync(ts->input_dev);

	del_point(&finger_list);
	finger_bit = point_data[1];

	if (ts->use_irq){
		enable_irq(ts->client->irq);
	}
	return;
}
static enum hrtimer_restart gt801_ts_timer_func(struct hrtimer *timer) {
	struct gt801_ts_data *ts = container_of(timer, struct gt801_ts_data, timer);

	queue_work(gt801_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(1, 1250000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t gt801_ts_irq_handler(int irq, void *dev_id) {
	struct gt801_ts_data *ts = dev_id;
	gt801printk("%s=%d,%d %p \n",__FUNCTION__,ts->client->irq,ts->use_irq,ts);

	if (ts->use_irq) {
		disable_irq_nosync(ts->client->irq);
	}
	queue_work(gt801_wq, &ts->work);
	return IRQ_HANDLED;
}

static int gt801_chip_Init(struct i2c_client *client) {
	u8 i, j;
	int ret = 0;
	u8 start_reg = 0x30;
	u8 buf[GT801_REGS_NUM] = { 0 };

	gt801printk("%s\n",__FUNCTION__);

	for (j = 0; j < 2; j++) {
		ret = gt801_write_regs(client, start_reg, GT801_RegData,
		GT801_REGS_NUM);
		if (ret < 0) {
			gt801printk("\n--%s--Set Register values error !!!\n",__FUNCTION__);
		}

		ret = gt801_read_regs(client, start_reg, buf, GT801_REGS_NUM);
		if (ret < 0) {
			gt801printk("\n--%s--Read Register values error !!!\n",__FUNCTION__);
		}

		for (i = 0; i < GT801_REGS_NUM - 1; i++) {
			if (buf[i] != GT801_RegData[i]) {
				gt801printk("gt801_chip_Init err may be i2c errorat adress=%x var=%x i=%x\n",0x30+i, buf[i],i);
				break;
			}
		}
		if (i == GT801_REGS_NUM - 1)
			break;
		else if (j == 1)
			return -1;

		mdelay(500);
	}
	mdelay(100);

	return ret;
}

static int gt801_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	struct gt801_ts_data *ts;
	struct touch_pdata *pdata = client->dev.platform_data;
	int ret = 0;
	int irq_filter, irq_trigger;
	int error;

	gt801printk("%s \n",__FUNCTION__);

	if (!pdata) {
		dev_err(&client->dev, "empty platform_data\n");
		return ret;
	}
	// print some debug info
	gt801printk("INT pin %d, Reset pin %d, power %d, xres %d, yres %d,pol %d\n ",
			pdata->gpio_interrupt,
			pdata->gpio_reset,
			pdata->gpio_power,
			pdata->xres,
			pdata->yres,
			pdata->pol);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		gt801printk(KERN_ERR "gt801_ts_probe: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		return -ENOMEM;
	}
	// setup work function
	INIT_WORK(&ts->work, gt801_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	//setup reset pin
	ret = setup_resetPin(client, ts);
	if (ret) {
		gt801printk("%s:setup_resetPin fail\n",__FUNCTION__);
		kfree(ts);
		return ret;
	}
	// wait for device redy
	msleep(100);

	// setup default register's values
	ret = gt801_chip_Init(ts->client);
	if (ret < 0) {
		gt801printk("%s:chips init failed\n",__FUNCTION__);
		gpio_free(ts->gpio_reset);
		kfree(ts);
		return ret;
	}

	/* allocate input device */
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		gt801printk(KERN_ERR "%s: Failed to allocate input device\n",__FUNCTION__);
		kfree(ts);
		return -ENOMEM;
	}

	ts->model = 801;
	ts->pol = NEED_TO_SWAP_XY(pdata->pol);
	ts->x_min = 0;
	ts->x_max = pdata->xres;
	ts->y_min = 0;
	ts->y_max = pdata->yres;
	ts->use_irq = 0;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));
	snprintf(ts->name, sizeof(ts->name), "gt%d-touchscreen", ts->model);

	ts->input_dev->phys = ts->phys;
	ts->input_dev->name = ts->name;
	ts->input_dev->dev.parent = &client->dev;
	ts->input_dev->id.vendor = 0x0416;

	set_bit(EV_SYN, (ts->input_dev)->evbit);
	set_bit(EV_ABS, (ts->input_dev)->evbit);

	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

	error = input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM, 0);

	if (error) {
		dev_err(&client->dev, "Error %d initializing slots\n", error);
		kfree(ts);
		return error;
	}

	input_set_abs_params(ts->input_dev,
	ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input_dev,
	ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	input_set_drvdata(ts->input_dev, ts);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		gt801printk(KERN_ERR "%s: Unable to register %s input device\n", __FUNCTION__,ts->input_dev->name);
		input_free_device(ts->input_dev);
		kfree(ts);
		return ret;
	}

	if (client->irq) {
		ret = amlogic_gpio_request_one(pdata->gpio_interrupt, GPIOF_IN,
				"irq_ts");

		if (ret) {
			dev_err(&client->dev, "amlogic_gpio_request_one error!\n");
			return ret;
		}
		amlogic_set_pull_up_down(pdata->gpio_interrupt, 1, "irq_ts");

		gt801printk("set pullup irq =%d\n", irq_source);
		/*
		 IRQ Filter Select : 7
		 Value 0 : No Filtering, Value 1 ~ 7 : value * 3 * 111nS(delay)
		 */
		irq_filter = FILTER_NUM7;

		/*
		 IRQ Trigger Select : (0 ~ 3)
		 GPIO_IRQ_HIGH = 0, GPIO_IRQ_LOW = 1, GPIO_IRQ_RISING = 2, GPIO_IRQ_FALLING = 3
		 */
		irq_trigger = pdata->irq_edge;

		/* GPIO IRQ Setup */
		if ((ret = amlogic_gpio_to_irq(pdata->gpio_interrupt, "irq_test",
				AML_GPIO_IRQ(client->irq, irq_filter, irq_trigger)))) {
			dev_err(&client->dev, "amlogic_gpio_to_irq fail!\n");
			return ret;
		}

		ret = request_irq(client->irq, gt801_ts_irq_handler, IRQF_DISABLED,
				client->name, ts);
		if (ret == 0) {
			gt801printk("%s:register ISR (irq=%d)\n", __FUNCTION__,client->irq);
			ts->use_irq = 1;
		} else {
			dev_err(&client->dev, "request_irq failed irq = %d errno = %d \n",
					client->irq, ret);
		}
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = gt801_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = gt801_ts_early_suspend;
	ts->early_suspend.resume = gt801_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	gt801printk(KERN_INFO "%s: Start touchscreen %s in %s mode\n", __FUNCTION__,ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return ret;
}

static int gt801_ts_remove(struct i2c_client *client) {
	struct gt801_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	gpio_free(ts->gpio_pendown);
	gpio_free(ts->gpio_reset);
	kfree(ts);
	return 0;
}

static int gt801_ts_suspend(struct i2c_client *client, pm_message_t mesg) {
	int ret;
	struct gt801_ts_data *ts = i2c_get_clientdata(client);

	gt801printk("gt801 TS Suspend\n");

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	gpio_set_value(ts->gpio_reset,
			ts->gpio_reset_active_low ? GPIO_LOW : GPIO_HIGH);

	return 0;
}

static void gt801_ts_resume_work_func(struct work_struct *work) {
	struct gt801_ts_data *ts = container_of(work, struct gt801_ts_data, work);
	msleep(50); //touch panel will generate an interrupt when it sleeps out,so as to avoid tihs by delaying 50ms
	enable_irq(ts->client->irq);
	PREPARE_WORK(&ts->work, gt801_ts_work_func);gt801printk("enabling gt801_ts IRQ %d\n", ts->client->irq);
}

static int gt801_ts_resume(struct i2c_client *client) {
	struct gt801_ts_data *ts = i2c_get_clientdata(client);

	gt801_init_panel(ts);

	gt801printk("gt801 TS Resume\n");

	gpio_set_value(ts->gpio_reset,
			ts->gpio_reset_active_low ? GPIO_HIGH : GPIO_LOW);

	if (ts->use_irq) {
		if (!work_pending(&ts->work)) {
			PREPARE_WORK(&ts->work, gt801_ts_resume_work_func);
			queue_work(gt801_wq, &ts->work);
		}
	} else {
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gt801_ts_early_suspend(struct early_suspend *h)
{
	struct gt801_ts_data *ts;
	ts = container_of(h, struct gt801_ts_data, early_suspend);
	gt801_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void gt801_ts_late_resume(struct early_suspend *h)
{
	struct gt801_ts_data *ts;
	ts = container_of(h, struct gt801_ts_data, early_suspend);
	gt801_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id gt801_ts_id[] = { { GT801_TS_NAME, 0 }, { } };

static struct i2c_driver gt801_ts_driver = { .probe = gt801_ts_probe, .remove =
		gt801_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
		.suspend = gt801_ts_suspend, .resume = gt801_ts_resume,
#endif
		.id_table = gt801_ts_id, .driver = { .name = GT801_TS_NAME, }, };

static int gt801_ts_init(void) {
	gt801printk("%s\n",__FUNCTION__);
	gt801_wq = create_singlethread_workqueue("gt801_wq");
	if (!gt801_wq)
		return -ENOMEM;
	return i2c_add_driver(&gt801_ts_driver);
}

static void __exit gt801_ts_exit(void) {
	gt801printk("%s\n",__FUNCTION__);
	i2c_del_driver(&gt801_ts_driver);
	if (gt801_wq)
		destroy_workqueue(gt801_wq);
}

module_init(gt801_ts_init);
module_exit(gt801_ts_exit);

MODULE_DESCRIPTION("gt801 Touchscreen Driver");
MODULE_LICENSE("GPL");
