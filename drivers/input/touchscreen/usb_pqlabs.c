/*
 * USB pqlabs driver - 130101
 *
 * Copyright (C) 2001-2004 Jemini
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License as
 *    published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-pqlabseton.c
 * but has been rewritten to be easier to read and use.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/compat.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/hiddev.h>
#include <linux/mm.h>

/* Define these values to match your devices */
#define USB_PQLABS_VENDOR_ID                 0x1EF1
#define USB_PQLABS_PRODUCT_ID                0x0001
#define USB_PQLABS_INTERFACE_CLASS           0xFF
#define USB_PQLABS_INTERFACE_SUBCLASS        0x0
#define USB_PQLABS_INTERFACE_PROTOCOL        0x0
#define CUR_DRIVER_VERSION                   0x130101

static const char *TOUCHSCREEN_NAME = "PQLABS Multi TouchScreen";

struct pqlabs_string_descriptor
{
  __s32 index;
  char value[256];
};

struct pqlabs_cmd_string
{
  __s32 index;
  char  buf[64];
};

#define USB_IOCTL_GET_STRING              _IOR('Q', 0x04, struct pqlabs_string_descriptor)
#define USB_IOCTL_CLEAR_FEATURE           _IOR('Q', 0x05, int)
#define USB_IOCTL_WRITE_HID_CMD           _IOR('Q', 0x06, struct pqlabs_cmd_string)
#define USB_IOCTL_GET_DRIVER_VERSION      _IOR('Q', 0x07, unsigned int)
//#define USB_IOCTL_NEXT_BUF                _IOR('Q', 0x08, int)
//#define USB_IOCTL_START_TRANSFER          _IOR('Q', 0x09, int)



#if 1
#define pq_trace(msg...) \
	do { \
		printk(KERN_DEBUG "pqlabs: " msg); \
	} while (0)
#endif

/* table of devices that work with this driver */
static struct usb_device_id pqlabs_table[] = {
    {USB_DEVICE_AND_INTERFACE_INFO(USB_PQLABS_VENDOR_ID, 
                                   USB_PQLABS_PRODUCT_ID, 
                                   USB_PQLABS_INTERFACE_CLASS, 
                                   USB_PQLABS_INTERFACE_SUBCLASS, 
                                   USB_PQLABS_INTERFACE_PROTOCOL)},
    { }                    /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, pqlabs_table);

/* Get a minor range for your devices from the usb maintainer */
#define USB_pqlabs_MINOR_BASE    192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER        (PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define WRITES_IN_FLIGHT    8
#define MAX_BUFFER_NUMBER   1
/* arbitrarily chosen */

/* Structure to hold all of our device specific stuff */
struct usb_pqlabs {
    struct usb_device       *udev;            /* the usb device for this device */
    struct usb_interface    *interface;        /* the interface for this device */
    struct semaphore        limit_sem;        /* limiting the number of writes in progress */
    struct usb_anchor       submitted;        /* in case we need to retract our submissions */
    struct urb              *bulk_in_urb;        /* the urb to read data with */
    unsigned char           *bulk_in_buffer;    /* the buffer to receive data */
    size_t                  bulk_in_size;        /* the size of the receive buffer */
    size_t                  bulk_in_filled;        /* number of bytes in the buffer */
    size_t                  bulk_in_copied;        /* already copied to user space */
    size_t                  bulk_total;
    __u8                    bulk_in_endpointAddr;    /* the address of the bulk in endpoint */
    __u8                    bulk_out_endpointAddr;    /* the address of the bulk out endpoint */
    int                     errors;            /* the last request tanked */
    int                     open_count;        /* count the number of openers */
    bool                    ongoing_read;        /* a read is going on */
    bool                    processed_urb;        /* indicates we haven't processed the urb */
    bool                    submitted_urb;
    spinlock_t              err_lock;        /* lock for errors */
    struct kref             kref;
    struct mutex            io_mutex;        /* synchronize I/O with disconnect */
    struct completion       bulk_in_completion;    /* to wait for an ongoing read */
    bool                    disconnecting;
    struct input_dev        *input_dev;
    char                    phys[32];
    
    int                     bufferIndex;
    size_t                  total_size;
};
#define to_pqlabs_dev(d) container_of(d, struct usb_pqlabs, kref)
#define MAX_SUPPORT_POINTS            64


static struct usb_driver pqlabs_driver;
static void pqlabs_draw_down(struct usb_pqlabs *dev);

static void pqlabs_delete(struct kref *kref)
{
    struct usb_pqlabs *dev = to_pqlabs_dev(kref);

    usb_free_urb(dev->bulk_in_urb);
    usb_put_dev(dev->udev);
    kfree(dev->bulk_in_buffer);
    kfree(dev);
}

static int pqlabs_open(struct inode *inode, struct file *file)
{
    struct usb_pqlabs *dev;
    struct usb_interface *interface;
    int subminor;
    int retval = 0;

    subminor = iminor(inode);

    interface = usb_find_interface(&pqlabs_driver, subminor);
    if (!interface) 
    {
        pr_err("%s - error, can't find device for minor %d", __func__, subminor);
        retval = -ENODEV;
        goto exit;
    }

    dev = usb_get_intfdata(interface);
    if (!dev) {
        retval = -ENODEV;
        goto exit;
    }

    /* increment our usage count for the device */
    kref_get(&dev->kref);

    /* lock the device to allow correctly handling errors
     * in resumption */
    mutex_lock(&dev->io_mutex);

    if (!dev->open_count++) {
        retval = usb_autopm_get_interface(interface);
            if (retval) {
                dev->open_count--;
                mutex_unlock(&dev->io_mutex);
                kref_put(&dev->kref, pqlabs_delete);
                goto exit;
            }
    }  else { //uncomment this block if you want exclusive open
        retval = -EBUSY;
        dev->open_count--;
        mutex_unlock(&dev->io_mutex);
        kref_put(&dev->kref, pqlabs_delete);
        goto exit;
    } 
    /* prevent the device from being autosuspended */

    /* save our object in the file's private structure */
    dev->bufferIndex = 0;
    file->private_data = dev;
    mutex_unlock(&dev->io_mutex);

exit:
    return retval;
}

static int pqlabs_release(struct inode *inode, struct file *file)
{
    struct usb_pqlabs *dev;

    dev = (struct usb_pqlabs *)file->private_data;
    if (dev == NULL)
        return -ENODEV;

    /* allow the device to be autosuspended */
    mutex_lock(&dev->io_mutex);
    if (!--dev->open_count && dev->interface)
        usb_autopm_put_interface(dev->interface);
    mutex_unlock(&dev->io_mutex);
        spin_lock(&dev->err_lock);
        dev->ongoing_read = 0;
        dev->submitted_urb = 0;
        spin_unlock(&dev->err_lock);

    /* decrement the count on our device */
    kref_put(&dev->kref, pqlabs_delete);
    return 0;
}

static int pqlabs_flush(struct file *file, fl_owner_t id)
{
    struct usb_pqlabs *dev;
    int res;

    dev = (struct usb_pqlabs *)file->private_data;
    if (dev == NULL)
        return -ENODEV;

    /* wait for io to stop */
    mutex_lock(&dev->io_mutex);
    pqlabs_draw_down(dev);

    /* read out errors, leave subsequent opens a clean slate */
    spin_lock_irq(&dev->err_lock);
    res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
    dev->errors = 0;
    spin_unlock_irq(&dev->err_lock);

    mutex_unlock(&dev->io_mutex);

    return res;
}

static void pqlabs_read_bulk_callback(struct urb *urb)
{
    struct usb_pqlabs *dev;
    dev = urb->context;

    /* sync/async unlink faults aren't errors */
    if (urb->status) 
    {
        if (!(urb->status == -ENOENT ||
              urb->status == -ECONNRESET ||
              urb->status == -ESHUTDOWN))
        pr_err("%s - nonzero write bulk status received: %d", __func__, urb->status);
        dev->errors = urb->status;
    } 
    else 
    {
        dev->bulk_in_filled = urb->actual_length;
        //if (dev->bulk_in_filled == 0) dev->bulk_in_filled = dev->bulk_in_size + 1;              
    }

    dev->submitted_urb = 0;
    complete(&dev->bulk_in_completion);
}

static int pqlabs_do_read_io(struct usb_pqlabs *dev, size_t count)
{
    int rv;

    /* prepare a read */
    usb_fill_bulk_urb(dev->bulk_in_urb,
                      dev->udev,
                      usb_rcvbulkpipe(dev->udev,
                      dev->bulk_in_endpointAddr),
                      dev->bulk_in_buffer,
                      min(dev->bulk_in_size, count),
                      pqlabs_read_bulk_callback,
                      dev);

  /* do it */
    rv = usb_submit_urb(dev->bulk_in_urb, GFP_KERNEL);
    if (rv < 0) 
    {
        pr_err("%s - failed submitting read urb, error %d", __func__, rv);
        dev->bulk_in_filled = 0;
        rv = (rv == -ENOMEM) ? rv : -EIO;
    }
    else
    {
        dev->submitted_urb = 1;
    }
    return rv;
}

#define READ_USB_MAX_LENGTH            (8 * 1024)
#define READ_USB_TIMEOUT            (3000)
static ssize_t pqlabs_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
    struct usb_pqlabs *dev;
    int rv;
    // bool ongoing_io;

    dev = (struct usb_pqlabs *)file->private_data;

    /* if we cannot read at all, return EOF */
    if (!dev->bulk_in_urb || !count || count > READ_USB_MAX_LENGTH) return 0;

    if (dev->disconnecting) return -ENODEV;

    /* no concurrent readers */
    rv = mutex_lock_interruptible(&dev->io_mutex);
    if (rv < 0)
    {
        return rv;
    }

    if (!dev->interface) 
    {        /* disconnect() was called */
        rv = -ENODEV;
        goto exit;
    }
        
    dev->bulk_in_filled = 0;
    dev->processed_urb = 0;

    if (!(dev->processed_urb) && dev->submitted_urb)  
    {
        rv = wait_for_completion_interruptible_timeout(&dev->bulk_in_completion, msecs_to_jiffies(READ_USB_TIMEOUT));
        if (rv <= 0)
        {
            usb_kill_urb(dev->bulk_in_urb);
            dev->submitted_urb = 0;
            goto exit;
        }
    }
    else
    {
        rv = pqlabs_do_read_io(dev, count);
        if (rv < 0)
        {
            goto exit;
        }

        while(1)
        {
            rv = wait_for_completion_interruptible_timeout(&dev->bulk_in_completion, msecs_to_jiffies(READ_USB_TIMEOUT));
            if (rv == msecs_to_jiffies(READ_USB_TIMEOUT) && dev->bulk_in_filled == 0)
            {
                rv = wait_for_completion_interruptible_timeout(&dev->bulk_in_completion, msecs_to_jiffies(READ_USB_TIMEOUT));
            }

            if (rv > 0) break;

            if (rv == 0)
            {
                pr_err("%s: timeout!\n", __func__);
                if (dev->disconnecting == 1) 
                {
                    pr_err("%s: disconnected break!\n", __func__);
                    goto exit;
                }
                dev->submitted_urb = 0;
                rv = -1;
                usb_kill_urb(dev->bulk_in_urb);
                goto exit;
            }

            if (rv < 0)
            {
                usb_kill_urb(dev->bulk_in_urb);
                dev->submitted_urb = 0;
                goto exit;
            }
        }
    }

    rv = dev->errors;
    if (rv < 0)
    {
        dev->errors = 0;
        rv = (rv == -EPIPE) ? rv : -EIO;
        goto exit;
    }

    if (dev->bulk_in_filled == 0)
    {
        rv = 0;
        goto exit;
    }

    if (copy_to_user(buffer, dev->bulk_in_buffer, dev->bulk_in_filled))
        rv = -EFAULT;
    else
        rv = dev->bulk_in_filled;
exit:
    mutex_unlock(&dev->io_mutex);
    return rv;
}

static ssize_t pqlabs_write(struct file *file, const char *user_buffer,
              size_t count, loff_t *ppos)
{
  return 0;
}

static long pqlabs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct usb_device *udev;
    struct usb_pqlabs *dev;
    int ret, result;
    int len, idx;
    char *buf;
    int i;
    void __user *user_arg = (void __user *)arg;

    ret = 0;
    len = 0;
    dev = (struct usb_pqlabs *)file->private_data;
    udev = dev->udev;

    if (cmd == USB_IOCTL_GET_STRING)
    {
        if (get_user(idx, (int __user *)user_arg))
        {
            pr_err("pqlabs: get idx failed!\n");
            return -EFAULT; 
        }

        if (udev->state == USB_STATE_SUSPENDED) return -EHOSTUNREACH;

        if ((buf = kmalloc(256, GFP_KERNEL)) == NULL) return -ENOMEM;

        for(i = 0; i < 3; i++)
        {
            result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
                                     USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
                                     (USB_DT_STRING << 8) + idx, 0x0409, buf, 255,
                                     USB_CTRL_GET_TIMEOUT);
            if (result == 0 || result == -EPIPE) continue;
      
            if (result > 1 && ((u8 *)buf)[1] != USB_DT_STRING)
            {
                result = -ENODATA;
                continue;
            }
            break;
        }

        if (result > 0)
        {
            len = buf[0];
            if (copy_to_user(user_arg + sizeof(int), buf, len + 1))
            {
                kfree(buf);
                return -EFAULT;
            }
        }
        kfree(buf);
        return len;
    }
    else if (cmd == USB_IOCTL_CLEAR_FEATURE)
    {
        char dBuf[8];
        if (udev->state == USB_STATE_SUSPENDED) return -EHOSTUNREACH;


        result = usb_control_msg(udev, 
                                 usb_sndctrlpipe(udev, 0),
                                 USB_REQ_CLEAR_FEATURE, 
                                 USB_TYPE_STANDARD | USB_RECIP_ENDPOINT,
                                 0, 0x82, dBuf, 8,
                                 500);
        return result;
    }
    else if (cmd == HIDIOCGSTRING)
    {
        if (get_user(idx, (int __user *)user_arg))
        {
            pr_err("pqlabs: get hid idx failed!\n");
            return -EFAULT; 
        }

        if (udev->state == USB_STATE_SUSPENDED) return -EHOSTUNREACH;

        if ((buf = kmalloc(HID_STRING_SIZE, GFP_KERNEL)) == NULL) return -ENOMEM;

        if ((len = usb_string(udev, idx, buf, HID_STRING_SIZE-1)) < 0) 
        {
            kfree(buf);
            return -EINVAL;
        }

        if (copy_to_user(user_arg+sizeof(int), buf, len+1)) 
        {
            kfree(buf);
            return -EFAULT;
        }

        kfree(buf);  
        return len;  
    }
    else if (cmd == USB_IOCTL_WRITE_HID_CMD)
    {
        if (get_user(idx, (int __user *)user_arg))
        {
            pr_err("pqlabs: get write command length failed!\n");
            return -EFAULT;
        }

        if (idx > 64) idx = 64;

        if (udev->state == USB_STATE_SUSPENDED) return -EHOSTUNREACH;
    
        if ((buf = kmalloc(64, GFP_KERNEL)) == NULL) return -ENOMEM;

        if (copy_from_user(buf, user_arg + sizeof(int), idx)) 
        {
            kfree(buf);
            return -EFAULT;
        }

        pq_trace("idx = %d, buf[0] = %d\n", idx, buf[0]);
        ret = usb_control_msg(udev, 
                              usb_rcvctrlpipe(udev, 0),
                              0xF0, 
                              USB_TYPE_VENDOR | USB_DIR_IN,
                              buf[0], 0, buf, idx,
                              USB_CTRL_GET_TIMEOUT);
        kfree(buf);
    }

    return ret;
}

#define PT_SIZE            10

/* Sysfs method to input simulated
   coordinates to the virtual
   mouse driver */
static ssize_t write_vms(struct device *device, struct device_attribute *attr, const char *buffer, size_t count)
{
    int type, slot;
    int ptCount;
    int i;
    int x, y;
    struct usb_interface *iface = to_usb_interface(device);
    struct usb_pqlabs *dev = (struct usb_pqlabs *)usb_get_intfdata(iface);

    /*  ID Counts + \n
        contactid + x + y + width + height + \n
     */
    ptCount = buffer[0];

    for (i = 0; i < ptCount; i++)
    {
        type = buffer[i * PT_SIZE + 1];
        slot = buffer[i * PT_SIZE + 2];
        x = buffer[i * PT_SIZE + 3] | (buffer[i * PT_SIZE + 4] << 8);
        y = buffer[i * PT_SIZE + 5] | (buffer[i * PT_SIZE + 6] << 8);
    
        if (type == 0x3)  // DOWN OR MOVE
        {
            input_mt_slot(dev->input_dev, slot);
            input_mt_report_slot_state(dev->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(dev->input_dev, ABS_MT_TOUCH_MAJOR, 10);
            input_report_abs(dev->input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(dev->input_dev, ABS_MT_POSITION_Y, y);
        }
        else if (type == 0x2)  // UP
        {
            input_mt_slot(dev->input_dev, slot);
            input_mt_report_slot_state(dev->input_dev, MT_TOOL_FINGER, false);          
        }
    }
    input_sync(dev->input_dev);

    return ptCount;
}

/* Attach the sysfs write method */
DEVICE_ATTR(coordinates, 0644, NULL, write_vms);

/* Attribute Descriptor */
static struct attribute *vms_attrs[] = {
  &dev_attr_coordinates.attr,
  NULL
};

/* Attribute group */
static struct attribute_group vms_attr_group = {
  .attrs = vms_attrs,
};

#if defined CONFIG_COMPAT
static long pqlabs_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned long translated_arg = (unsigned long)compat_ptr(arg);
    return pqlabs_ioctl(file, cmd, translated_arg);
}
#endif

static const struct file_operations pqlabs_fops = {
    .owner          = THIS_MODULE,
    .read           = pqlabs_read,
    .write          = pqlabs_write,
    .open           = pqlabs_open,
    .release        = pqlabs_release,
    .flush          = pqlabs_flush,
    .unlocked_ioctl = pqlabs_ioctl,
#if defined CONFIG_COMPAT
    .compat_ioctl   = pqlabs_compat_ioctl,
#endif
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver pqlabs_class = {
    .name       = "pqlabs_bulk%d",
    .fops       = &pqlabs_fops,
    .minor_base = USB_pqlabs_MINOR_BASE,
};

static int pqlabs_probe(struct usb_interface *interface,
              const struct usb_device_id *id)
{
    struct usb_pqlabs *dev;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    size_t buffer_size;
    int i;
    int retval = -ENOMEM;

    /* allocate memory for our device state and initialize it */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) 
    {
        //pr_err("Out of memory");
        goto error;
    }
    kref_init(&dev->kref);
    sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
    mutex_init(&dev->io_mutex);
    spin_lock_init(&dev->err_lock);
    init_usb_anchor(&dev->submitted);
    init_completion(&dev->bulk_in_completion);

    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;
    dev->disconnecting = false;

    /* set up the endpoint information */
    /* use only the first bulk-in and bulk-out endpoints */
    iface_desc = interface->cur_altsetting;

    for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) 
    {
        endpoint = &iface_desc->endpoint[i].desc;
        if (!dev->bulk_in_endpointAddr && usb_endpoint_is_bulk_in(endpoint)) 
        {
            /* we found a bulk in endpoint */
            buffer_size = READ_USB_MAX_LENGTH; //le16_to_cpu(endpoint->wMaxPacketSize);
            dev->total_size = buffer_size * MAX_BUFFER_NUMBER;
            dev->bulk_in_size = buffer_size;
            dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
            dev->bulk_in_buffer = kmalloc(dev->total_size, GFP_KERNEL);
            if (!dev->bulk_in_buffer) 
            {
                pr_err("Could not allocate bulk_in_buffer");
                goto error;
            }

            dev->bulk_in_urb = usb_alloc_urb(0, GFP_KERNEL);
            if (!dev->bulk_in_urb) 
            {
                pr_err("Could not allocate bulk_in_urb");
                goto error;
            }
        }

        if (!dev->bulk_out_endpointAddr &&
            usb_endpoint_is_bulk_out(endpoint)) {
            /* we found a bulk out endpoint */
            dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
        }
    }

    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    /* we can register the device now, as it is ready */
    retval = usb_register_dev(interface, &pqlabs_class);
    if (retval) {
        /* something prevented us from registering this driver */
        //pr_err("Not able to get a minor for this device.");
        usb_set_intfdata(interface, NULL);
        goto error;
    }
    
    dev->input_dev = input_allocate_device();
    if (dev->input_dev == NULL) 
    {
        retval = -ENOMEM;
        printk("Failed to allocate input device\n");
        goto error;
    }
    
    __set_bit(INPUT_PROP_DIRECT, dev->input_dev->propbit);
    __set_bit(EV_ABS, dev->input_dev->evbit);
    set_bit(ABS_PRESSURE, dev->input_dev->absbit);
    input_mt_init_slots(dev->input_dev, MAX_SUPPORT_POINTS, 0);
    input_set_abs_params(dev->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(dev->input_dev, ABS_MT_POSITION_X, 0, 0x3FFF, 0, 0);
    input_set_abs_params(dev->input_dev, ABS_MT_POSITION_Y, 0, 0x3FFF, 0, 0);
    sprintf(dev->phys, "input/pqlabs");
    dev->input_dev->name = TOUCHSCREEN_NAME;
    dev->input_dev->phys = dev->phys;
    dev->input_dev->id.bustype = BUS_USB;
    dev->input_dev->id.vendor = 0x1EF1;
    dev->input_dev->id.product = 0x0002;
    dev->input_dev->id.version = 0x0001;    //screen firmware version
    
    retval = input_register_device(dev->input_dev);
    if (retval) 
    {
        printk("Probe: Unable to register %s input device\n", dev->input_dev->name);
        retval = -1;
        goto error;
    }
    
    /* Create a sysfs node to read simulated coordinates */
    retval = sysfs_create_group(&interface->dev.kobj, &vms_attr_group);
    //sysfs_create_group(&dev->input_dev->dev.kobj, &vms_attr_group);
    if (retval) {
	    pr_err("cannot create sysfs group err: %d\n", retval);
	    retval = -1;
	    goto error;
    }

    /* let the user know what node this device is now attached to */
    dev_info(&interface->dev,
             "USB pqlabs bulk device now attached to USBpqlabs-%d",
             interface->minor);
    return 0;

error:
    if (dev->input_dev)
    {
      input_free_device(dev->input_dev);
    }
    
    if (dev)
        /* this frees allocated memory */
        kref_put(&dev->kref, pqlabs_delete);
    return retval;
}

static void pqlabs_disconnect(struct usb_interface *interface)
{
    struct usb_pqlabs *dev;
    //int minor = interface->minor;

    dev = usb_get_intfdata(interface);
    dev->disconnecting = true;
    
    mutex_lock(&dev->io_mutex);    
    
    sysfs_remove_group(&interface->dev.kobj, &vms_attr_group);
    //sysfs_remove_group(&dev->input_dev->dev.kobj, &vms_attr_group);
    input_mt_destroy_slots(dev->input_dev);
    input_unregister_device(dev->input_dev);
    if (dev->input_dev)
    {
        input_free_device(dev->input_dev);
    }
    usb_set_intfdata(interface, NULL);

    /* give back our minor */
    usb_deregister_dev(interface, &pqlabs_class);

    /* prevent more I/O from starting */
    dev->interface = NULL;
    mutex_unlock(&dev->io_mutex);

    usb_kill_anchored_urbs(&dev->submitted);

    /* decrement our usage count */
    kref_put(&dev->kref, pqlabs_delete);

    //dev_info(&interface->dev, "USB pqlabseton #%d now disconnected", minor);
}

static void pqlabs_draw_down(struct usb_pqlabs *dev)
{
    int time;

    time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
    if (!time)
        usb_kill_anchored_urbs(&dev->submitted);
    usb_kill_urb(dev->bulk_in_urb);
}

static int pqlabs_suspend(struct usb_interface *intf, pm_message_t message)
{
    struct usb_pqlabs *dev = usb_get_intfdata(intf);

    if (!dev)
        return 0;
    pqlabs_draw_down(dev);
    return 0;
}

static int pqlabs_resume(struct usb_interface *intf)
{
    return 0;
}

static int pqlabs_pre_reset(struct usb_interface *intf)
{
    struct usb_pqlabs *dev = usb_get_intfdata(intf);

    mutex_lock(&dev->io_mutex);
    pqlabs_draw_down(dev);

    return 0;
}

static int pqlabs_post_reset(struct usb_interface *intf)
{
    struct usb_pqlabs *dev = usb_get_intfdata(intf);

    /* we are sure no URBs are active - no locking needed */
    dev->errors = -EPIPE;
    mutex_unlock(&dev->io_mutex);

    return 0;
}

static struct usb_driver pqlabs_driver = {
    .name       = "pqlabs_bulk",
    .probe      = pqlabs_probe,
    .disconnect = pqlabs_disconnect,
    .suspend    = pqlabs_suspend,
    .resume     = pqlabs_resume,
    .pre_reset  = pqlabs_pre_reset,
    .post_reset = pqlabs_post_reset,
    .id_table   = pqlabs_table,
    .supports_autosuspend = 1,
};

static int __init usb_pqlabs_init(void)
{
    int result;

    /* register this driver with the USB subsystem */
    result = usb_register(&pqlabs_driver);
    if (result)
        pr_err("usb_register failed. Error number %d", result);

    return result;
}

static void __exit usb_pqlabs_exit(void)
{
    /* deregister this driver with the USB subsystem */
    usb_deregister(&pqlabs_driver);
}

module_init(usb_pqlabs_init);
module_exit(usb_pqlabs_exit);

MODULE_LICENSE("GPL");
