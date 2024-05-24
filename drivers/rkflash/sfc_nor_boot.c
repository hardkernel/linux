// SPDX-License-Identifier: GPL-2.0

/* Copyright (c) 2018 Rockchip Electronics Co. Ltd. */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <crypto/skcipher.h>
#include <linux/scatterlist.h>

#include "sfc_nor.h"
#include "rkflash_api.h"
#include "rkflash_debug.h"

#define VENDOR_PART_NUM			4

#define	FLASH_VENDOR_PART_START		8
#define FLASH_VENDOR_PART_SIZE		8
#define FLASH_VENDOR_ITEM_NUM		62
#define	FLASH_VENDOR_PART_END		\
	(FLASH_VENDOR_PART_START +\
	FLASH_VENDOR_PART_SIZE * VENDOR_PART_NUM - 1)

#define IDB_ALIGN_64			128	/* 64 KB */
#define IDB_ALIGN_32			64	/* 32 KB */

struct SFNOR_DEV *sfnor_dev;

/* SFNOR_DEV sfnor_dev is in the sfc_nor.h */
static int spi_nor_init(void __iomem *reg_addr)
{
	int ret;
	struct id_block_tag *idb_tag;
	struct snor_info_packet *packet;

	sfnor_dev = kzalloc(sizeof(*sfnor_dev), GFP_KERNEL);

	if (!sfnor_dev)
		return -ENOMEM;

	sfc_init(reg_addr);
	ret = snor_init(sfnor_dev);
	if (ret == SFC_OK && sfnor_dev->read_lines == DATA_LINES_X1) {
		struct crypto_skcipher *tfm_arc4;

		tfm_arc4 = crypto_alloc_skcipher("ecb(arc4)", 0,
						 CRYPTO_ALG_ASYNC);
		if (IS_ERR(tfm_arc4)) {
			crypto_free_skcipher(tfm_arc4);
			return SFC_OK;
		}

		idb_tag = kzalloc(NOR_SECS_PAGE * 512, GFP_KERNEL);
		if (!idb_tag) {
			crypto_free_skcipher(tfm_arc4);
			return SFC_OK;
		}

		if (sfc_get_version() >= SFC_VER_4)
			snor_read(sfnor_dev, IDB_ALIGN_32, NOR_SECS_PAGE,
				  idb_tag);
		else
			snor_read(sfnor_dev, IDB_ALIGN_64, NOR_SECS_PAGE,
				  idb_tag);
		packet = (struct snor_info_packet *)&idb_tag->dev_param[0];
		if (idb_tag->id == IDB_BLOCK_TAG_ID) {
			SKCIPHER_REQUEST_ON_STACK(req, tfm_arc4);
			u8 key[16] = {124, 78, 3, 4, 85, 5, 9, 7,
				      45, 44, 123, 56, 23, 13, 23, 17};
			struct scatterlist sg;
			u32 len = sizeof(struct id_block_tag);

			crypto_skcipher_setkey(tfm_arc4, key, 16);
			sg_init_one(&sg, idb_tag, len + 4);
			skcipher_request_set_tfm(req, tfm_arc4);
			skcipher_request_set_callback(req, 0, NULL, NULL);
			skcipher_request_set_crypt(req, &sg, &sg, len + 4,
						   NULL);
			ret = crypto_skcipher_encrypt(req);
			if (!ret) {
				snor_reinit_from_table_packet(sfnor_dev,
							      packet);
				rkflash_print_error("snor reinit, ret= %d\n", ret);
			}
		}
		crypto_free_skcipher(tfm_arc4);
		kfree(idb_tag);
	}

	return ret;
}

static int snor_read_lba(u32 sec, u32 n_sec, void *p_data)
{
	int ret = 0;
	u32 count, offset;
	char *buf;

	if (sec + n_sec - 1 < FLASH_VENDOR_PART_START ||
	    sec > FLASH_VENDOR_PART_END) {
		ret = snor_read(sfnor_dev, sec, n_sec, p_data);
	} else {
		memset(p_data, 0, 512 * n_sec);
		if (sec < FLASH_VENDOR_PART_START) {
			count = FLASH_VENDOR_PART_START - sec;
			buf = p_data;
			ret = snor_read(sfnor_dev, sec, count, buf);
		}
		if ((sec + n_sec - 1) > FLASH_VENDOR_PART_END) {
			count = sec + n_sec - 1 - FLASH_VENDOR_PART_END;
			offset = FLASH_VENDOR_PART_END - sec + 1;
			buf = p_data + offset * 512;
			ret = snor_read(sfnor_dev,
					FLASH_VENDOR_PART_END + 1,
					count, buf);
		}
	}

	return (u32)ret == n_sec ? 0 : ret;
}

static int snor_write_lba(u32 sec, u32 n_sec, void *p_data)
{
	int ret = 0;

	ret = snor_write(sfnor_dev, sec, n_sec, p_data);

	return (u32)ret == n_sec ? 0 : ret;
}

#if !defined(CONFIG_ARCH_ROCKCHIP_ODROID_COMMON)
static int snor_vendor_read(u32 sec, u32 n_sec, void *p_data)
{
	int ret = 0;

	ret = snor_read(sfnor_dev, sec, n_sec, p_data);

	return (u32)ret == n_sec ? 0 : ret;
}

static int snor_vendor_write(u32 sec, u32 n_sec, void *p_data)
{
	int ret = 0;

	ret = snor_write(sfnor_dev, sec, n_sec, p_data);

	return (u32)ret == n_sec ? 0 : ret;
}
#else
extern int snor_prog(struct SFNOR_DEV *p_dev, u32 addr, void *p_data, u32 size);

static int snor_secure_request(u8 cmd)
{
	struct rk_sfc_op op;

	op.sfcmd.d32 = 0;
	op.sfcmd.b.cmd = cmd;
	op.sfctrl.d32 = 0;

	return sfc_request(&op, 0, NULL, 0);
}

int snor_secure_enter(void)
{
	sfnor_dev->secure_on = true;
	return snor_secure_request(0xb1);	// ENSO
}

int snor_secure_exit(void)
{
	sfnor_dev->secure_on = false;
	return snor_secure_request(0xc1);	// EXSO
}

static int snor_vendor_read(u32 addr, u32 size, void *p_data)
{
	u32 nbytes = size;
	char *buf = (char *)kmalloc(size, GFP_KERNEL);
	unsigned char *p;
	int ret = 0;

	snor_secure_enter();

	for (p = buf; nbytes; ) {
		u32 len = min(nbytes, sfnor_dev->max_iosize);
		ret = snor_read_data(sfnor_dev, addr, p, len);
		if (ret != SFC_OK)
			break;

		nbytes -= len;
		addr += len;
		p += len;
	}

	snor_secure_exit();

	if (ret == SFC_OK)
		memcpy(p_data, buf, size);

	kfree(buf);

	return ret;
}

static int snor_vendor_write(u32 addr, u32 size, void *p_data)
{
	char *buf = kmalloc(size, GFP_KERNEL);
	int ret;

	memcpy(buf, p_data, size);

	snor_secure_enter();
	ret = snor_prog(sfnor_dev, addr, buf, size);
	snor_secure_exit();

	kfree(buf);

	return ret;
}
#endif

static int snor_gc(void)
{
	return 0;
}

static unsigned int snor_capacity(void)
{
	return snor_get_capacity(sfnor_dev);
}

static void snor_deinit(void)
{
	snor_disable_QE(sfnor_dev);
	snor_reset_device();
	kfree(sfnor_dev);
}

static int snor_resume(void __iomem *reg_addr)
{
	return spi_nor_init(reg_addr);
}

const struct flash_boot_ops sfc_nor_ops = {
	spi_nor_init,
	snor_read_lba,
	snor_write_lba,
	snor_capacity,
	snor_deinit,
	snor_resume,
	snor_vendor_read,
	snor_vendor_write,
	snor_gc,
	NULL,
};
