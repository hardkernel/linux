//[*]------------------------------------------------------------------------[*]
//
//
//  I2C INA231(Sensor) driver
//  2013.07.17
//
//
//[*]------------------------------------------------------------------------[*]
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/list.h>

//[*]------------------------------------------------------------------------[*]
#include "ina231.h"
#include "ina231-i2c.h"
#include "ina231-misc.h"

//[*]------------------------------------------------------------------------[*]
//   Global Sensor struct (sensor struct save for ioctl)
//[*]------------------------------------------------------------------------[*]
struct global_sensor {
	struct ina231_sensor	*p;
	struct list_head	list;
};

LIST_HEAD(SensorList);

//[*]------------------------------------------------------------------------[*]
//
// function prototype
//
//[*]------------------------------------------------------------------------[*]
static int	ina231_misc_open	(struct inode *inode, struct file *file);
static long	ina231_misc_ioctl	(struct file *file, unsigned int cmd, unsigned long arg);
void		ina231_misc_remove	(struct device *dev);
int		ina231_misc_probe	(struct ina231_sensor *sensor);

//[*]------------------------------------------------------------------------[*]
static const struct file_operations ina231_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= ina231_misc_open,
	.unlocked_ioctl	= ina231_misc_ioctl,
};

//[*]------------------------------------------------------------------------[*]
static long ina231_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ina231_sensor	*sensor = (struct ina231_sensor *)file->private_data;
	struct ina231_iocreg	ina231, *iocreg;

	if (copy_from_user((void *)&ina231, (void *)arg, sizeof(struct ina231_iocreg)))
		return -EFAULT;
	iocreg = (struct ina231_iocreg *)&ina231;
	switch (cmd) {
		// Get regisger
		case	INA231_IOCGREG:
			iocreg->enable = sensor->pd->enable;
			if (sensor->pd->enable) {
				mutex_lock(&sensor->mutex);
				iocreg->cur_uV = sensor->cur_uV;
				iocreg->cur_uA = sensor->cur_uA;
				iocreg->cur_uW = sensor->cur_uW;
				mutex_unlock(&sensor->mutex);
			}
			else {
				iocreg->cur_uV = 0;
				iocreg->cur_uA = 0;
				iocreg->cur_uW = 0;
			}
			if (copy_to_user((void *)arg, (void *)&ina231, sizeof(struct ina231_iocreg)))
                            return -EFAULT;
			break;
		// Set status
		case	INA231_IOCSSTATUS:
			if (sensor->pd->enable != iocreg->enable) {
				sensor->pd->enable = iocreg->enable;
				if (sensor->pd->enable)  ina231_i2c_enable(sensor);
			}
			if (copy_to_user((void *)arg, (void *)&ina231, sizeof(struct ina231_iocreg)))
                            return -EFAULT;
			break;
		// Set status
		case	INA231_IOCGSTATUS:
			iocreg->enable = sensor->pd->enable;
			memset(iocreg->name, 0x00, sizeof(iocreg->name));
			memcpy(iocreg->name, sensor->pd->name, sizeof(iocreg->name));
			if (copy_to_user((void *)arg, (void *)&ina231, sizeof(struct ina231_iocreg)))
                            return -EFAULT;
			break;
		default :
			printk("%s : unknown message!!\n", __func__);
			break;
	}
	return 0;
}

//[*]------------------------------------------------------------------------[*]
static int ina231_misc_open(struct inode *inode, struct file *file)
{
	struct global_sensor	*gsensor;
	struct list_head	*list_head;

	printk("%s : %d\n", __func__, iminor(inode));

	list_for_each(list_head, &SensorList)
	{
		gsensor = list_entry(list_head, struct global_sensor, list);

		if (gsensor->p->misc->minor == iminor(inode)) {
			printk("find match sensor struct : name = %s\n",
				gsensor->p->pd->name);
			file->private_data = gsensor->p;
		}
	}

	return 0;
}

//[*]------------------------------------------------------------------------[*]
int ina231_misc_probe(struct ina231_sensor *sensor)
{
	int			rc;
	struct miscdevice	*pmisc;
	struct global_sensor	*gsensor;

	if (!(pmisc = devm_kzalloc(&sensor->client->dev,
				sizeof(struct miscdevice),
				GFP_KERNEL))) {
		printk("INA231 Sensor misc struct malloc error!\n");
		return	-ENOMEM;
	}

	pmisc->minor = MISC_DYNAMIC_MINOR;
	pmisc->name = sensor->pd->name;
	pmisc->fops = &ina231_misc_fops;
	pmisc->mode = S_IWUGO | S_IRUGO;

	sensor->misc = pmisc;

	if ((rc = misc_register(sensor->misc)) < 0) {
		printk("%s : INA231 misc register fail!\n", __func__);
		return	rc;
	}

	if(!(gsensor = (struct global_sensor *)devm_kzalloc(&sensor->client->dev,
						sizeof(struct global_sensor),
						GFP_KERNEL))) {
		printk("%s : INA231 global sensor malloc error!\n", __func__);
		return	-ENOMEM;
	}
	else    {
		gsensor->p = sensor;
		list_add(&gsensor->list, &SensorList);
	}

	return 0;
}

//[*]------------------------------------------------------------------------[*]
void ina231_misc_remove(struct device *dev)
{
	struct ina231_sensor	*sensor = dev_get_drvdata(dev);
	struct list_head	*list_head;
	struct global_sensor	*gsensor;

	misc_deregister(sensor->misc);

	list_for_each(list_head, &SensorList)
	{
		gsensor = list_entry(list_head, struct global_sensor, list);
	}
}

//[*]------------------------------------------------------------------------[*]
MODULE_AUTHOR("HardKernel Co., Ltd.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("INA231 Current Sensor Driver");

//[*]------------------------------------------------------------------------[*]
//[*]------------------------------------------------------------------------[*]
