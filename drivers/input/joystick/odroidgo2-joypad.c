/*
 * SARADC joystick & GPIO Button driver for Linux(Hardkernel ODROIDGO2-Advance)
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
 * Vojtech Pavlik, Simunkova 1594, Prague 8, 182 00 Czech Republic
 */

/*----------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio_keys.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/property.h>
#include <linux/of_gpio.h>

/*----------------------------------------------------------------------------*/
#define DRV_NAME "odroidgo2_joypad"

/*----------------------------------------------------------------------------*/
struct bt_adc {
	/* IIO ADC Channel */
	struct iio_channel *channel;
	/* report value (mV) */
	int old_value;
	/* report type */
	int report_type;
	/* input device init value (mV) */
	int max, min;
	/* calibrated adc value */
	int cal;
	/*  adc scale value */
	int scale;
	/* invert report */
	bool invert;
};

struct bt_gpio {
	/* GPIO Request label */
	const char *label;
	/* GPIO Number */
	int num;
	/* report type */
	int report_type;
	/* report linux code */
	int linux_code;
	/* prev button value */
	bool old_value;
	/* button press level */
	bool active_level;
};

struct joypad {
	struct device *dev;
	int poll_interval;

	/* report enable/disable */
	bool enable;

	/* report reference point */
	bool invert_absx;
	bool invert_absy;

	/* report interval (ms) */
	int bt_gpio_count;
	struct bt_gpio *gpios;
	/* button auto repeat */
	int auto_repeat;

	/* report threshold (mV) */
	int bt_adc_fuzz, bt_adc_flat;
	int bt_adc_x_range, bt_adc_y_range;
	/* adc read value scale */
	int bt_adc_scale;
	/* joystick deadzone control */
	int bt_adc_deadzone;
	int bt_adc_count;
	struct bt_adc *adcs;

	struct mutex lock;

	char name[32];
};

/*----------------------------------------------------------------------------*/
//
// set to the value in the boot.ini file. (if exist)
//
/*----------------------------------------------------------------------------*/
static unsigned int g_button_adc_x_range = 0;
static unsigned int g_button_adc_y_range = 0;
static unsigned int g_button_adc_fuzz = 0;
static unsigned int g_button_adc_flat = 0;
static unsigned int g_button_adc_scale = 0;
static unsigned int g_button_adc_deadzone = 0;

static int __init button_adcx_range_setup(char *str)
{
        if (!str)
                return -EINVAL;

	g_button_adc_x_range = simple_strtoul(str, NULL, 10);

        return 0;
}
__setup("button-adc-x-range=", button_adcx_range_setup);

static int __init button_adcy_range_setup(char *str)
{
        if (!str)
                return -EINVAL;

	g_button_adc_y_range = simple_strtoul(str, NULL, 10);

        return 0;
}
__setup("button-adc-y-range=", button_adcy_range_setup);

static int button_adc_fuzz(char *str)
{
        if (!str)
                return -EINVAL;
	g_button_adc_fuzz = simple_strtoul(str, NULL, 10);
	return 0;
}
__setup("button-adc-fuzz=", button_adc_fuzz);

static int button_adc_flat(char *str)
{
        if (!str)
                return -EINVAL;
	g_button_adc_flat = simple_strtoul(str, NULL, 10);
	return 0;
}
__setup("button-adc-flat=", button_adc_flat);

static int button_adc_scale(char *str)
{
        if (!str)
                return -EINVAL;
	g_button_adc_scale = simple_strtoul(str, NULL, 10);
	return 0;
}
__setup("button-adc-scale=", button_adc_scale);

static int button_adc_deadzone(char *str)
{
        if (!str)
                return -EINVAL;
	g_button_adc_deadzone = simple_strtoul(str, NULL, 10);
	return 0;
}
__setup("button-adc-deadzone=", button_adc_deadzone);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int joypad_adc_read(struct bt_adc *adc)
{
	int value;

	if (iio_read_channel_processed(adc->channel, &value))
		return 0;

	value *= adc->scale;

	return (adc->invert ? (adc->max - value) : value);
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/odroidgo2_joypad/poll_interval [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_poll_interval(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->poll_interval = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_poll_interval(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->poll_interval);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(poll_interval, S_IWUSR | S_IRUGO,
		   joypad_show_poll_interval,
		   joypad_store_poll_interval);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/odroidgo2_joypad/adc_fuzz [r]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_fuzz(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->bt_adc_fuzz);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_fuzz, S_IWUSR | S_IRUGO,
		   joypad_show_adc_fuzz,
		   NULL);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/odroidgo2_joypad/adc_flat [r]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_flat(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->bt_adc_flat);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_flat, S_IWUSR | S_IRUGO,
		   joypad_show_adc_flat,
		   NULL);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/odroidgo2_joypad/enable [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->enable = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->enable);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   joypad_show_enable,
		   joypad_store_enable);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/odroidgo2_joypad/adc_cal [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_adc_cal(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	bool calibration;

	calibration = simple_strtoul(buf, NULL, 10);

	if (calibration) {
		int nbtn;

		mutex_lock(&joypad->lock);
		for (nbtn = 0; nbtn < joypad->bt_adc_count; nbtn++) {
			struct bt_adc *adc = &joypad->adcs[nbtn];

			adc->cal = joypad_adc_read(adc);
			if (!adc->cal) {
				dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
					__func__, nbtn);
				continue;
			}
			adc->old_value = adc->cal;
		}
		mutex_unlock(&joypad->lock);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_cal(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	int nbtn;
	ssize_t pos;

	for (nbtn = 0, pos = 0; nbtn < joypad->bt_adc_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		pos += sprintf(&buf[pos], "adc[%d]->cal = %d ",
				nbtn, adc->cal);
	}
	pos += sprintf(&buf[pos], "\n");
	return pos;
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_cal, S_IWUSR | S_IRUGO,
		   joypad_show_adc_cal,
		   joypad_store_adc_cal);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static struct attribute *joypad_attrs[] = {
	&dev_attr_poll_interval.attr,
	&dev_attr_adc_fuzz.attr,
	&dev_attr_adc_flat.attr,
	&dev_attr_enable.attr,
	&dev_attr_adc_cal.attr,
	NULL,
};

static struct attribute_group joypad_attr_group = {
	.attrs = joypad_attrs,
};

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static void joypad_gpio_check(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn, value;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];

		if (gpio_get_value_cansleep(gpio->num) < 0) {
			dev_err(joypad->dev, "failed to get gpio state\n");
			continue;
		}
		value = gpio_get_value(gpio->num);
		if (value != gpio->old_value) {
			input_event(poll_dev->input,
				gpio->report_type,
				gpio->linux_code,
				(value == gpio->active_level) ? 1 : 0);
			gpio->old_value = value;
		}
	}
	input_sync(poll_dev->input);
}

/*----------------------------------------------------------------------------*/
static void joypad_adc_check(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn, value;

	for (nbtn = 0; nbtn < joypad->bt_adc_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		value = joypad_adc_read(adc);
		if (!value) {
			dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
				__func__, nbtn);
			continue;
		}

		/* Joystick Deadzone check */
		if (joypad->bt_adc_deadzone) {
			if ((value < adc->cal + joypad->bt_adc_deadzone) &&
			    (value > adc->cal - joypad->bt_adc_deadzone))
				value = adc->cal;
		}
		value = value - adc->cal;
		value = value > adc->max ? adc->max : value;
		value = value < adc->min ? adc->min : value;

		if (nbtn == 0)
		{
			// adc-x value is default inverted(h/w)
			input_report_abs(poll_dev->input,
				adc->report_type, value * (-1));
		}
		else
		{
			input_report_abs(poll_dev->input,
				adc->report_type, value);
		}
		adc->old_value = value;
	}
	input_sync(poll_dev->input);
}

/*----------------------------------------------------------------------------*/
static void joypad_poll(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (joypad->enable) {
		joypad_adc_check(poll_dev);
		joypad_gpio_check(poll_dev);
	}
	if (poll_dev->poll_interval != joypad->poll_interval) {
		mutex_lock(&joypad->lock);
		poll_dev->poll_interval = joypad->poll_interval;
		mutex_unlock(&joypad->lock);
	}
}

/*----------------------------------------------------------------------------*/
static void joypad_open(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		gpio->old_value = gpio->active_level ? 0 : 1;
	}
	for (nbtn = 0; nbtn < joypad->bt_adc_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->old_value = joypad_adc_read(adc);
		if (!adc->old_value) {
			dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
				__func__, nbtn);
			continue;
		}
		adc->cal = adc->old_value;
		dev_info(joypad->dev, "%s : adc[%d] adc->cal = %d\n",
			__func__, nbtn, adc->cal);
	}
	/* buttons status sync */
	joypad_adc_check(poll_dev);
	joypad_gpio_check(poll_dev);

	/* button report enable */
	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "%s : opened\n", __func__);
}

/*----------------------------------------------------------------------------*/
static void joypad_close(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	/* button report disable */
	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "%s : closed\n", __func__);
}

/*----------------------------------------------------------------------------*/
static int joypad_adc_setup(struct device *dev, struct joypad *joypad)
{
	int nbtn = 0;

	joypad->adcs = devm_kzalloc(dev, joypad->bt_adc_count *
				sizeof(struct bt_adc), GFP_KERNEL);

	if (!joypad->adcs) {
		dev_err(dev, "%s devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}

	for (nbtn = 0; nbtn < joypad->bt_adc_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		enum iio_chan_type type;

		adc->scale = joypad->bt_adc_scale;
		if (nbtn) {
			adc->channel =
				devm_iio_channel_get(dev, "joy_y");
			adc->report_type = ABS_Y;
			if (joypad->invert_absy)
				adc->invert = true;

			adc->max =  (joypad->bt_adc_y_range / 2) - 1;
			adc->min = -(joypad->bt_adc_y_range / 2);
		}
		else {
			adc->channel =
				devm_iio_channel_get(dev, "joy_x");
			adc->report_type = ABS_X;
			if (joypad->invert_absx)
				adc->invert = true;

			adc->max =  (joypad->bt_adc_x_range / 2) - 1;
			adc->min = -(joypad->bt_adc_x_range / 2);
		}

		if (IS_ERR(adc->channel)) {
			dev_err(dev, "iio channel[%d] get error\n", nbtn);
			return -EINVAL;
		}
		if (!adc->channel->indio_dev)
			return -ENXIO;

		if (iio_get_channel_type(adc->channel, &type))
			return -EINVAL;

		if (type != IIO_VOLTAGE) {
			dev_err(dev, "Incompatible channel %d type %d\n",
				nbtn, type);
			return -EINVAL;
		}
	}
	if (nbtn == 0)
		return -EINVAL;

	return	0;
}

/*----------------------------------------------------------------------------*/
static int joypad_gpio_setup(struct device *dev, struct joypad *joypad)
{
	struct device_node *node, *pp;
	int nbtn;

	node = dev->of_node;
	if (!node)
		return -ENODEV;

	joypad->gpios = devm_kzalloc(dev, joypad->bt_gpio_count *
				sizeof(struct bt_gpio), GFP_KERNEL);

	if (!joypad->gpios) {
		dev_err(dev, "%s devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}

	nbtn = 0;
	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		struct bt_gpio *gpio = &joypad->gpios[nbtn++];
		int error;

		gpio->num = of_get_gpio_flags(pp, 0, &flags);
		if (gpio->num < 0) {
			error = gpio->num;
			dev_err(dev, "Failed to get gpio flags, error: %d\n",
				error);
			return error;
		}

		/* gpio active level(key press level) */
		gpio->active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

		gpio->label = of_get_property(pp, "label", NULL);

		if (gpio_is_valid(gpio->num)) {
			error = devm_gpio_request_one(dev, gpio->num,
						      GPIOF_IN, gpio->label);
			if (error < 0) {
				dev_err(dev,
					"Failed to request GPIO %d, error %d\n",
					gpio->num, error);
				return error;
			}
		}
		if (of_property_read_u32(pp, "linux,code", &gpio->linux_code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				gpio->num);
			return -EINVAL;
		}
		if (of_property_read_u32(pp, "linux,input-type",
				&gpio->report_type))
			gpio->report_type = EV_KEY;
	}
	if (nbtn == 0)
		return -EINVAL;

	return	0;
}

/*----------------------------------------------------------------------------*/
static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	int nbtn, error;

	poll_dev = devm_input_allocate_polled_device(dev);
	if (!poll_dev) {
		dev_err(dev, "no memory for polled device\n");
		return -ENOMEM;
	}

	poll_dev->private	= joypad;
	poll_dev->poll		= joypad_poll;
	poll_dev->poll_interval	= joypad->poll_interval;
	poll_dev->open		= joypad_open;
	poll_dev->close		= joypad_close;

	input = poll_dev->input;

	if (joypad->bt_gpio_count == 18) {
		snprintf(joypad->name, sizeof(joypad->name), "%s v11", DRV_NAME);
		input->id.product = 0x0002;
	}
	else {
		snprintf(joypad->name, sizeof(joypad->name), "%s", DRV_NAME);
		input->id.product = 0x0001;
	}

	input->phys = DRV_NAME "/input0";
	input->name = joypad->name;

	input->id.bustype = BUS_HOST;
	input->id.vendor  = 0x0001;
	input->id.version = 0x0101;

	/* IIO ADC key setup (0 mv ~ 1800 mv) * adc->scale */
	__set_bit(EV_ABS, input->evbit);
	for(nbtn = 0; nbtn < joypad->bt_adc_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		input_set_abs_params(input, adc->report_type,
				adc->min, adc->max,
				joypad->bt_adc_fuzz,
				joypad->bt_adc_flat);
		dev_info(dev,
			"%s : SCALE = %d, ABS min = %d, max = %d,"
			" fuzz = %d, flat = %d, deadzone = %d\n",
			__func__, adc->scale, adc->min, adc->max,
			joypad->bt_adc_fuzz, joypad->bt_adc_flat,
			joypad->bt_adc_deadzone);
	}

	/* GPIO key setup */
	__set_bit(EV_KEY, input->evbit);
	for(nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		input_set_capability(input, gpio->report_type,
				gpio->linux_code);
	}

	if (joypad->auto_repeat)
		__set_bit(EV_REP, input->evbit);

	joypad->dev = dev;

	error = input_register_polled_device(poll_dev);
	if (error) {
		dev_err(dev, "unable to register polled device, err=%d\n",
			error);
		return error;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static void joypad_setup_value_check(struct device *dev, struct joypad *joypad)
{
	/*
		fuzz: specifies fuzz value that is used to filter noise from
			the event stream.
	*/
	if (g_button_adc_fuzz)
		joypad->bt_adc_fuzz = g_button_adc_fuzz;
	else
		device_property_read_u32(dev, "button-adc-fuzz",
					&joypad->bt_adc_fuzz);
	/*
		flat: values that are within this value will be discarded by
			joydev interface and reported as 0 instead.
	*/
	if (g_button_adc_flat)
		joypad->bt_adc_flat = g_button_adc_flat;
	else
		device_property_read_u32(dev, "button-adc-flat",
					&joypad->bt_adc_flat);

	/* Joystick report value control */
	if (g_button_adc_scale)
		joypad->bt_adc_scale = g_button_adc_scale;
	else
		device_property_read_u32(dev, "button-adc-scale",
					&joypad->bt_adc_scale);

	/* Joystick deadzone value control */
	if (g_button_adc_deadzone)
		joypad->bt_adc_deadzone = g_button_adc_deadzone;
	else
		device_property_read_u32(dev, "button-adc-deadzone",
					&joypad->bt_adc_deadzone);

	if (g_button_adc_x_range)
		joypad->bt_adc_x_range = g_button_adc_x_range;
	else
		device_property_read_u32(dev, "button-adc-x-range",
					&joypad->bt_adc_x_range);
	if (g_button_adc_y_range)
		joypad->bt_adc_y_range = g_button_adc_y_range;
	else
		device_property_read_u32(dev, "button-adc-y-range",
					&joypad->bt_adc_y_range);
}

/*----------------------------------------------------------------------------*/
static int joypad_dt_parse(struct device *dev, struct joypad *joypad)
{
	int error = 0;

	/* initialize value check from boot.ini */
	joypad_setup_value_check(dev, joypad);

	device_property_read_u32(dev, "button-adc-count",
				&joypad->bt_adc_count);

	device_property_read_u32(dev, "poll-interval",
				&joypad->poll_interval);

	joypad->auto_repeat = device_property_present(dev, "autorepeat");

	/* change the report reference point? (ADC MAX - read value) */
	joypad->invert_absx = device_property_present(dev, "invert-absx");
	joypad->invert_absy = device_property_present(dev, "invert-absy");
	dev_info(dev, "%s : invert-absx = %d, inveret-absy = %d\n",
		__func__, joypad->invert_absx, joypad->invert_absy);

	joypad->bt_gpio_count = device_get_child_node_count(dev);

	if ((joypad->bt_adc_count == 0) || (joypad->bt_gpio_count == 0)) {
		dev_err(dev, "adc key = %d, gpio key = %d error!",
			joypad->bt_adc_count, joypad->bt_gpio_count);
		return -EINVAL;
	}

	error = joypad_adc_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_gpio_setup(dev, joypad);
	if (error)
		return error;

	return error;
}

/*----------------------------------------------------------------------------*/
static int joypad_probe(struct platform_device *pdev)
{
	struct joypad *joypad;
	struct device *dev = &pdev->dev;
	int error;

	joypad = devm_kzalloc(dev, sizeof(struct joypad), GFP_KERNEL);
	if (!joypad) {
		dev_err(dev, "joypad devm_kzmalloc error!");
		return -ENOMEM;
	}

	/* device tree data parse */
	error = joypad_dt_parse(dev, joypad);
	if (error) {
		dev_err(dev, "dt parse error!(err = %d)\n", error);
		return error;
	}

	mutex_init(&joypad->lock);
	platform_set_drvdata(pdev, joypad);

	error = sysfs_create_group(&pdev->dev.kobj, &joypad_attr_group);
	if (error) {
		dev_err(dev, "create sysfs group fail, error: %d\n",
			error);
		return error;
	}

	/* poll input device setup */
	error = joypad_input_setup(dev, joypad);
	if (error) {
		dev_err(dev, "input setup failed!(err = %d)\n", error);
		return error;
	}
	dev_info(dev, "%s : probe success\n", __func__);
	return 0;
}

/*----------------------------------------------------------------------------*/
static const struct of_device_id joypad_of_match[] = {
	{ .compatible = "odroidgo2-joypad", },
	{},
};

MODULE_DEVICE_TABLE(of, joypad_of_match);

/*----------------------------------------------------------------------------*/
static struct platform_driver joypad_driver = {
	.probe = joypad_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(joypad_of_match),
	},
};

/*----------------------------------------------------------------------------*/
static int __init joypad_init(void)
{
	return platform_driver_register(&joypad_driver);
}

/*----------------------------------------------------------------------------*/
static void __exit joypad_exit(void)
{
	platform_driver_unregister(&joypad_driver);
}

/*----------------------------------------------------------------------------*/
late_initcall(joypad_init);
module_exit(joypad_exit);

/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Hardkernel Co.,LTD");
MODULE_DESCRIPTION("Keypad driver(ADC&GPIO) for ODROIDGO-Advance");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);

/*----------------------------------------------------------------------------*/
