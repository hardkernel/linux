// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2021 Hardkernel Co., Ltd.
 *
 */

#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#include <linux/soc/rockchip/rk_vendor_storage.h>

#define EFUSE_ODROID_MAGIC              "HKM1"
#define EFUSE_ODROID_MAXSIZ             64
#define EFUSE_MEMORY_SIZE		1024	/* bytes */
#define MAX_OFFSET_UUID			96	/* bytes */

#define EFUSE_INFO_PROVISION		_IO('f', 0x5073)
#define EFUSE_DUMP			_IO('f', 0x5074)
#define EFUSE_CLEAR			_IO('f', 0x5075)
#define EFUSE_OEM_PROVISION		_IO('f', 0x5076)

#define OEM_PROVISION_OFFSET			0x0100

struct odroid_efuse_t {
	u32 magic;      /* EFUSE_ODROID_MAGIC */
	u32 offset;     /* The offset of data in eFuse */
	u32 len;        /* The length of data buffer */
	u8 sum;         /* The checksum of data buffer */
	u8 data[EFUSE_ODROID_MAXSIZ];
} __attribute__ ((__packed__));

ssize_t efuse_read_usr(char *buf, size_t count, u32 offset);
ssize_t efuse_write_usr(char *buf, size_t count, u32 offset);
static int get_mac_offset(u8 *index);

static int char2hex(u8 ch)
{
	if (('0' <= ch) && (ch <= '9'))
		return ch - '0';
	if (('a' <= ch) && (ch <= 'f'))
		return ch - 'a' + 10;
	if (('A' <= ch) && (ch <= 'F'))
		return ch - 'A' + 10;

	return 0;
}

static int (*read_cb)(u32 sec, u32 n_sec, void *p_data);
static int (*write_cb)(u32 sec, u32 n_sec, void *p_data);

static int vendor_read(u32 pos, char *buf, u32 size)
{
	if (!read_cb)
		return -EINVAL;

	return read_cb(pos, size, buf);
}

static int vendor_write(u32 pos, char *buf, u32 size)
{
	char *p = (char *)kmalloc(size, GFP_KERNEL);
	int ret;

	ret = vendor_read(pos, p, size);
	if (!ret) {
		int i;
		for (i = 0; i < size; i++)
			if (*(p + i) != 0xff) {
				ret = -EBUSY;
				break;
			}
	}

	kfree(p);

	return ret ? ret : write_cb(pos, size, buf);
}

static int vendor_clear(u32 pos)
{
	u8 buf[32] = {0,};
	int ret = 0;

	return ret ? ret : write_cb(pos, sizeof(buf), buf);
}

int flash_vendor_dev_ops_register(
		int (*read)(u32 sec, u32 n_sec, void *p_data),
		int (*write)(u32 sec, u32 n_sec, void *p_data))
{
	if (read_cb)
		return -1;

	read_cb = read;
	write_cb = write;

	rk_vendor_register(vendor_read, vendor_write);

	return 0;
}

void rk_devinfo_get_eth_mac(u8 *mac)
{
	u8 bdinfo[32];
	int ret;
	u8 index;

	ret = get_mac_offset(&index);
	if (ret < 0)
		pr_err("failed to read MAC offset\n");

	ret = vendor_read(index * 32, bdinfo, sizeof(bdinfo));
	if (ret == 0) {
		int i;
		for (i = 0; i < 6; i++) {
			mac[i] = (char2hex(bdinfo[20 + i * 2]) << 4)
				| char2hex(bdinfo[21 + i * 2]);
		}
	}
}

ssize_t efuse_read_usr(char *buf, size_t size, u32 offset)
{
	return vendor_read(offset, buf, size);
}

ssize_t efuse_write_usr(char *buf, size_t size, u32 offset)
{
	return vendor_write(offset, buf, size);
}

static int odroid_valid(u8 *data, u32 len, u8 expected)
{
	u8 sum = 0;

	while (len--)
		sum += *data++;

	return (sum == expected) ? 0 : -EINVAL;
}

static int __provision(u8 *data, u32 offset, u32 len, u8 sum)
{
	int ret;

	/* Validate before writing */
	ret = odroid_valid(data, len, sum);
	if (ret < 0) {
		pr_err("%s:%d, Invalid checksum\n", __func__, __LINE__);
		return -EINVAL;
	}

	/* Write */
	ret = efuse_write_usr(data, len, offset);
	if (ret < 0) {
		pr_err("failed to write efuse\n");
		return ret;
	}

	/* Read back */
	ret = efuse_read_usr(data, len, offset);
	if (ret < 0) {
		pr_err("failed to read efuse\n");
		return ret;
	}

	/* Verify */
	ret = odroid_valid(data, len, sum);
	if (ret < 0) {
		pr_err("data mismatch\n");
		return ret;
	}

	return 0;
}

int odroid_valid_packet(struct odroid_efuse_t *fuse, u32 maxlen)
{
	if (!fuse)
		return -EINVAL;

	if (memcmp((void *)&fuse->magic, EFUSE_ODROID_MAGIC,
				sizeof(((struct odroid_efuse_t *)0)->magic))) {
		pr_err("magic number is not matched (%08x)\n", fuse->magic);
		return -EINVAL;
	}

	if (fuse->len > maxlen) {
		pr_err("data size is wrong (%d > %d)\n",
				fuse->len, maxlen);
		return -EINVAL;
	}


	return 0;
}

static int odroid_provision_info(struct odroid_efuse_t *fuse)
{
	int ret;

	ret = odroid_valid_packet(fuse,
			sizeof((struct odroid_efuse_t *)0)->data);
	if (ret < 0)
		return ret;

	if (fuse->offset > MAX_OFFSET_UUID) {
		pr_err("offset error - (%d > %u)\n", fuse->offset, MAX_OFFSET_UUID);
		return -EINVAL;
	}

	return __provision(fuse->data, fuse->offset, fuse->len, fuse->sum);
}

static int offset_by_oem_slot(int n)
{
	if ((n < 0) || n > 3)
		return -EINVAL;

	return OEM_PROVISION_OFFSET + n * 16;
}

static int odroid_provision_oem(struct odroid_efuse_t *fuse)
{
	int offset;
	int ret;

	ret = odroid_valid_packet(fuse, 16);
	if (ret < 0)
		return ret;

	if ((fuse->offset < 0) || (fuse->offset > 3)) {
		pr_err("out of efuse slot (%d)\n", fuse->offset);
		return -EINVAL;
	}

	offset = offset_by_oem_slot(fuse->offset);
	if (offset < 0)
		return -EINVAL;

	return __provision(fuse->data, offset, fuse->len, fuse->sum);
}

static int odroid_clear(u32 offset)
{
	u8 in[32];
	int ret;
	int i, cnt = 0;

	if (offset % 32 != 0) {
		pr_err("align error %d\n", offset);
	}

	ret = efuse_read_usr(in, sizeof(in), offset);
	if (ret < 0) {
		pr_err("failed to read efuse\n");
		return ret;
	}

	for (i = 0; i < sizeof(in); i++)
		if (in[i] == 0xff)
			cnt++;
	if (cnt == 32) {
		pr_err("unused area\n");
		return -EINVAL;
	}

	ret = vendor_clear(offset);
	if (ret < 0) {
		pr_err("failed to clear efuse\n");
		return ret;
	}

	memset(in, 0, sizeof(in));

	/* Read back */
	ret = efuse_read_usr(in, sizeof(in), offset);
	if (ret < 0) {
		pr_err("failed to read efuse\n");
		return ret;
	}

	/* Verify */
	ret = odroid_valid(in, sizeof(in), 0);
	if (ret < 0) {
		pr_err("data mismatch\n");
		return ret;
	}

	return 0;
}

static int get_mac_offset(u8 *index)
{
	u8 in[32];
	int i, ret;
	u8 idx = 0;

	while (idx < 3) {
		ret = efuse_read_usr(in, sizeof(in), idx * 32);
		if (ret < 0) {
			pr_err("failed to read efuse\n");
			return ret;
		}

		for (i = 0; i < sizeof(in); i++) {
			if (in[i] != 0x00) {
				*index = idx;
				return 0;
			}
		}
		idx++;
	}
	*index = idx;

	return 0;
}

static long efuse_unlocked_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct odroid_efuse_t fuse;
	u8 *otp;
	int ret = 0;

	switch (cmd) {
	case EFUSE_INFO_PROVISION:
		ret = copy_from_user(&fuse, argp, sizeof(fuse));
		if (ret != 0) {
			pr_err("%s:%d, copy_from_user fail\n",
					__func__, __LINE__);
			break;
		}

		ret = odroid_provision_info(&fuse);
		if (ret < 0) {
			pr_err("%s:%d, Failed (%d)\n",
					__func__, __LINE__, ret);
			break;
		}
		break;

	case EFUSE_OEM_PROVISION:
		ret = copy_from_user(&fuse, argp, sizeof(fuse));
		if (ret != 0) {
			pr_err("%s:%d, copy_from_user fail\n",
					__func__, __LINE__);
			break;
		}

		ret = odroid_provision_oem(&fuse);
		if (ret < 0) {
			pr_err("%s:%d, Failed (%d)\n",
					__func__, __LINE__, ret);
			break;
		}
		break;

	case EFUSE_DUMP:
		otp = (u8*)kmalloc(EFUSE_MEMORY_SIZE, GFP_KERNEL);
		memset(otp, 0, EFUSE_MEMORY_SIZE);
		ret = vendor_read(0, otp, EFUSE_MEMORY_SIZE);
		print_hex_dump(KERN_INFO, "dump: ", DUMP_PREFIX_OFFSET,
				16, 1, otp, EFUSE_MEMORY_SIZE, true);
		kfree(otp);
		break;

	case EFUSE_CLEAR:
		ret = copy_from_user(&fuse, argp, sizeof(fuse));
		if (ret != 0) {
			pr_err("%s:%d, copy_from_user fail\n",
					__func__, __LINE__);
			break;
		}

		ret = odroid_clear(fuse.offset);
		if (ret < 0) {
			pr_err("%s:%d, Failed (%d)\n",
					__func__, __LINE__, ret);
			break;
		}
		break;
	}

	return ret;
}

static ssize_t uuid_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	u8 in[32];
	u8 index;
	int ret;

	ret = get_mac_offset(&index);
	if (ret < 0) {
		pr_err("failed to read MAC offset\n");
		return ret;
	}

	ret = efuse_read_usr(in, sizeof(in), index*32);
	if (ret < 0) {
		pr_err("failed to read efuse\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE,
			/* 99999999-9999-9999-9999-999999999999 */
			"%c%c%c%c%c%c%c%c-%c%c%c%c-%c%c%c%c-%c%c%c%c-"
			"%c%c%c%c%c%c%c%c%c%c%c%c\n",
			in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7],
			in[8], in[9], in[10], in[11],
			in[12], in[13], in[14], in[15],
			in[16], in[17], in[18], in[19],
			in[20], in[21], in[22], in[23], in[24], in[25],
			in[26], in[27], in[28], in[29], in[30], in[31]);
}
static CLASS_ATTR_RO(uuid);

static ssize_t __oem16_show(struct class *class,
		struct class_attribute *attr, char *buf, int slot)
{
	u8 in[16];
	int offset;
	int ret;

	offset = offset_by_oem_slot(slot);
	if (offset < 0)
		return -EINVAL;

	ret = efuse_read_usr(in, sizeof(in), offset);
	if (ret < 0) {
		pr_err("failed to read efuse\n");
		return ret;
	}

	return snprintf(buf, PAGE_SIZE,
			/* 9999999999999999 */
			"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
			in[0], in[1], in[2], in[3], in[4], in[5], in[6], in[7], in[8],
			in[9], in[10], in[11], in[12], in[13], in[14], in[15]);
}

#define OEM_PROVISION_SLOT(x)						\
	static ssize_t oem##x##_show(struct class *class,		\
			struct class_attribute *attr, char *buf)	\
	{								\
		return __oem16_show(class, attr, buf, x);		\
	}								\
	static CLASS_ATTR_RO(oem##x);					\

OEM_PROVISION_SLOT(0);
OEM_PROVISION_SLOT(1);
OEM_PROVISION_SLOT(2);
OEM_PROVISION_SLOT(3);

static struct attribute *efuse_class_attrs[] = {
	&class_attr_uuid.attr,
	&class_attr_oem0.attr,
	&class_attr_oem1.attr,
	&class_attr_oem2.attr,
	&class_attr_oem3.attr,
	NULL,
};
ATTRIBUTE_GROUPS(efuse_class);

static struct class efuse_class = {
	.name = "efuse",
	.owner = THIS_MODULE,
	.class_groups = efuse_class_groups,
};

static const struct file_operations efuse_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = efuse_unlocked_ioctl,
};

static struct miscdevice efuse_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "efuse",
	.fops = &efuse_fops,
};

static int __init odroid_board_init(void)
{
	class_register(&efuse_class);
	return misc_register(&efuse_device);
}

static void __exit odroid_board_exit(void)
{
	class_unregister(&efuse_class);
	misc_deregister(&efuse_device);
}

late_initcall(odroid_board_init);
module_exit(odroid_board_exit);

MODULE_DESCRIPTION("Hardkernel board driver");
MODULE_AUTHOR("Dongjin Kim <tobetter@gmail.com>");
MODULE_LICENSE("GPL");
