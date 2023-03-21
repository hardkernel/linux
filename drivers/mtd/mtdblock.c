// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Direct MTD block device access
 *
 * Copyright © 1999-2010 David Woodhouse <dwmw2@infradead.org>
 * Copyright © 2000-2003 Nicolas Pitre <nico@fluxnic.net>
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>
#include <linux/mutex.h>
#include <linux/major.h>


struct mtdblk_dev {
	struct mtd_blktrans_dev mbd;
	int count;
	struct mutex cache_mutex;
	unsigned char *cache_data;
	unsigned long cache_offset;
	unsigned int cache_size;
	enum { STATE_EMPTY, STATE_CLEAN, STATE_DIRTY } cache_state;
};

#ifdef CONFIG_AMLOGIC_NAND
struct mtdblk_pbbt {
	struct mtdblk_dev mb;
	unsigned int bad_cnt;
	unsigned short *part_bbt;
};
#endif

/*
 * Cache stuff...
 *
 * Since typical flash erasable sectors are much larger than what Linux's
 * buffer cache can handle, we must implement read-modify-write on flash
 * sectors for each block write requests.  To avoid over-erasing flash sectors
 * and to speed things up, we locally cache a whole flash sector while it is
 * being written to until a different sector is required.
 */

static int erase_write (struct mtd_info *mtd, unsigned long pos,
			unsigned int len, const char *buf)
{
	struct erase_info erase;
	size_t retlen;
	int ret;

	/*
	 * First, let's erase the flash block.
	 */
	erase.addr = pos;
	erase.len = len;

	ret = mtd_erase(mtd, &erase);
	if (ret) {
		printk (KERN_WARNING "mtdblock: erase of region [0x%lx, 0x%x] "
				     "on \"%s\" failed\n",
			pos, len, mtd->name);
		return ret;
	}

	/*
	 * Next, write the data to flash.
	 */

	ret = mtd_write(mtd, pos, len, &retlen, buf);
	if (ret)
		return ret;
	if (retlen != len)
		return -EIO;
	return 0;
}


static int write_cached_data (struct mtdblk_dev *mtdblk)
{
	struct mtd_info *mtd = mtdblk->mbd.mtd;
	int ret;

	if (mtdblk->cache_state != STATE_DIRTY)
		return 0;

	pr_debug("mtdblock: writing cached data for \"%s\" "
			"at 0x%lx, size 0x%x\n", mtd->name,
			mtdblk->cache_offset, mtdblk->cache_size);

	ret = erase_write (mtd, mtdblk->cache_offset,
			   mtdblk->cache_size, mtdblk->cache_data);

	/*
	 * Here we could arguably set the cache state to STATE_CLEAN.
	 * However this could lead to inconsistency since we will not
	 * be notified if this content is altered on the flash by other
	 * means.  Let's declare it empty and leave buffering tasks to
	 * the buffer cache instead.
	 *
	 * If this cache_offset points to a bad block, data cannot be
	 * written to the device. Clear cache_state to avoid writing to
	 * bad blocks repeatedly.
	 */
#ifdef CONFIG_AMLOGIC_NAND
	if (!ret && ret != EIO)
		return ret;
	mtdblk->cache_state = STATE_EMPTY;
	return ret;
#else
	if (ret == 0 || ret == -EIO)
		mtdblk->cache_state = STATE_EMPTY;
	return ret;
#endif
}

#ifdef CONFIG_AMLOGIC_NAND
static unsigned long map_block(struct mtdblk_dev *mtdblk, unsigned long pos)
{
	struct mtd_info *mtd = mtdblk->mbd.mtd;
	struct mtdblk_pbbt *pbbt = container_of(mtdblk, struct mtdblk_pbbt, mb);
	int block, i;

	if (!pbbt->part_bbt)
		return pos;

	block = (int)(pos >> mtd->erasesize_shift);
	for (i = 0; i < pbbt->bad_cnt; i++) {
		if (block >= pbbt->part_bbt[i])
			block++;
		else
			break;
	}

	/* form actual position */
	return ((unsigned long)block * mtd->erasesize) |
		(pos & (mtd->erasesize - 1));
}
#endif

static int do_cached_write (struct mtdblk_dev *mtdblk, unsigned long pos,
			    int len, const char *buf)
{
	struct mtd_info *mtd = mtdblk->mbd.mtd;
	unsigned int sect_size = mtdblk->cache_size;
	size_t retlen;
	int ret;

	pr_debug("mtdblock: write on \"%s\" at 0x%lx, size 0x%x\n",
		mtd->name, pos, len);

#ifdef CONFIG_AMLOGIC_NAND
	pos = map_block(mtdblk, pos);
#endif

	if (!sect_size)
		return mtd_write(mtd, pos, len, &retlen, buf);

	while (len > 0) {
		unsigned long sect_start = (pos/sect_size)*sect_size;
		unsigned int offset = pos - sect_start;
		unsigned int size = sect_size - offset;
		if( size > len )
			size = len;

		if (size == sect_size) {
			/*
			 * We are covering a whole sector.  Thus there is no
			 * need to bother with the cache while it may still be
			 * useful for other partial writes.
			 */
			ret = erase_write (mtd, pos, size, buf);
			if (ret)
				return ret;
		} else {
			/* Partial sector: need to use the cache */

			if (mtdblk->cache_state == STATE_DIRTY &&
			    mtdblk->cache_offset != sect_start) {
				ret = write_cached_data(mtdblk);
				if (ret)
					return ret;
			}

			if (mtdblk->cache_state == STATE_EMPTY ||
			    mtdblk->cache_offset != sect_start) {
				/* fill the cache with the current sector */
				mtdblk->cache_state = STATE_EMPTY;
				ret = mtd_read(mtd, sect_start, sect_size,
					       &retlen, mtdblk->cache_data);
				if (ret && !mtd_is_bitflip(ret))
					return ret;
				if (retlen != sect_size)
					return -EIO;

				mtdblk->cache_offset = sect_start;
				mtdblk->cache_size = sect_size;
				mtdblk->cache_state = STATE_CLEAN;
			}

			/* write data to our local cache */
			memcpy (mtdblk->cache_data + offset, buf, size);
			mtdblk->cache_state = STATE_DIRTY;
		}

		buf += size;
		pos += size;
		len -= size;
	}

	return 0;
}


static int do_cached_read (struct mtdblk_dev *mtdblk, unsigned long pos,
			   int len, char *buf)
{
	struct mtd_info *mtd = mtdblk->mbd.mtd;
	unsigned int sect_size = mtdblk->cache_size;
	size_t retlen;
	int ret;

	pr_debug("mtdblock: read on \"%s\" at 0x%lx, size 0x%x\n",
			mtd->name, pos, len);
#ifdef CONFIG_AMLOGIC_NAND
	pos = map_block(mtdblk, pos);
#endif
	if (!sect_size) {
		ret = mtd_read(mtd, pos, len, &retlen, buf);
		if (ret && !mtd_is_bitflip(ret))
			return ret;
		return 0;
	}

	while (len > 0) {
		unsigned long sect_start = (pos/sect_size)*sect_size;
		unsigned int offset = pos - sect_start;
		unsigned int size = sect_size - offset;
		if (size > len)
			size = len;

		/*
		 * Check if the requested data is already cached
		 * Read the requested amount of data from our internal cache if it
		 * contains what we want, otherwise we read the data directly
		 * from flash.
		 */
		if (mtdblk->cache_state != STATE_EMPTY &&
		    mtdblk->cache_offset == sect_start) {
			memcpy (buf, mtdblk->cache_data + offset, size);
		} else {
			ret = mtd_read(mtd, pos, size, &retlen, buf);
			if (ret && !mtd_is_bitflip(ret))
				return ret;
			if (retlen != size)
				return -EIO;
		}

		buf += size;
		pos += size;
		len -= size;
	}

	return 0;
}

static int mtdblock_readsect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct mtdblk_dev *mtdblk = container_of(dev, struct mtdblk_dev, mbd);

#if (IS_ENABLED(CONFIG_AMLOGIC_MTD_NAND) || \
			IS_ENABLED(CONFIG_AMLOGIC_MTD_SPI_NAND))
	return do_cached_read(mtdblk, block << dev->tr->blkshift,
							dev->tr->blksize, buf);
#else
	return do_cached_read(mtdblk, block<<9, 512, buf);
#endif
}

static int mtdblock_writesect(struct mtd_blktrans_dev *dev,
			      unsigned long block, char *buf)
{
	struct mtdblk_dev *mtdblk = container_of(dev, struct mtdblk_dev, mbd);
	if (unlikely(!mtdblk->cache_data && mtdblk->cache_size)) {
		mtdblk->cache_data = vmalloc(mtdblk->mbd.mtd->erasesize);
		if (!mtdblk->cache_data)
			return -EINTR;
		/* -EINTR is not really correct, but it is the best match
		 * documented in man 2 write for all cases.  We could also
		 * return -EAGAIN sometimes, but why bother?
		 */
	}

#if (IS_ENABLED(CONFIG_AMLOGIC_MTD_NAND) || \
			IS_ENABLED(CONFIG_AMLOGIC_MTD_SPI_NAND))
	return do_cached_write(mtdblk, block << dev->tr->blkshift,
							dev->tr->blksize, buf);
#else
	return do_cached_write(mtdblk, block<<9, 512, buf);
#endif
}

static int mtdblock_open(struct mtd_blktrans_dev *mbd)
{
	struct mtdblk_dev *mtdblk = container_of(mbd, struct mtdblk_dev, mbd);
#ifdef CONFIG_AMLOGIC_NAND
	int block_cnt, i, bad_cnt = 0;
	struct mtdblk_pbbt *pbbt = container_of(mtdblk, struct mtdblk_pbbt, mb);
#endif

	pr_debug("mtdblock_open\n");

	if (mtdblk->count) {
		mtdblk->count++;
		return 0;
	}

	if (mtd_type_is_nand(mbd->mtd))
		pr_warn("%s: MTD device '%s' is NAND, please consider using UBI block devices instead.\n",
			mbd->tr->name, mbd->mtd->name);

	/* OK, it's not open. Create cache info for it */
	mtdblk->count = 1;
	mutex_init(&mtdblk->cache_mutex);
	mtdblk->cache_state = STATE_EMPTY;
	if (!(mbd->mtd->flags & MTD_NO_ERASE) && mbd->mtd->erasesize) {
		mtdblk->cache_size = mbd->mtd->erasesize;
		mtdblk->cache_data = NULL;
	}

#ifdef CONFIG_AMLOGIC_NAND
	pbbt->part_bbt =  NULL;
	if (!mtd_can_have_bb(mbd->mtd))
		goto _ok;

	block_cnt = mbd->mtd->size >> mbd->mtd->erasesize_shift;
	for (i = 0; i < block_cnt; i++)
		/*
		 * A valid judgment is made before calling this function.
		 */
		/* coverity[divide_by_zero:SUPPRESS] */
		if (mtd_block_isbad(mbd->mtd, i * mbd->mtd->erasesize))
			bad_cnt++;
	pbbt->bad_cnt = bad_cnt;
	if (bad_cnt) {
		pbbt->part_bbt =
		kmalloc_array(block_cnt, sizeof(*pbbt->part_bbt), GFP_KERNEL);
		bad_cnt = 0;
		for (i = 0; i < block_cnt; i++)
			/*
			 * A valid judgment is made before calling this function.
			 */
			/* coverity[divide_by_zero:SUPPRESS] */
			if (mtd_block_isbad(mbd->mtd, i * mbd->mtd->erasesize))
				pbbt->part_bbt[bad_cnt++] = i;
	}

_ok:
#endif
	pr_debug("ok\n");

	return 0;
}

static void mtdblock_release(struct mtd_blktrans_dev *mbd)
{
	struct mtdblk_dev *mtdblk = container_of(mbd, struct mtdblk_dev, mbd);

	pr_debug("mtdblock_release\n");

	mutex_lock(&mtdblk->cache_mutex);
	write_cached_data(mtdblk);
	mutex_unlock(&mtdblk->cache_mutex);

	if (!--mtdblk->count) {
		/*
		 * It was the last usage. Free the cache, but only sync if
		 * opened for writing.
		 */
		if (mbd->file_mode & FMODE_WRITE)
			mtd_sync(mbd->mtd);
		vfree(mtdblk->cache_data);
	}

	pr_debug("ok\n");
}

static int mtdblock_flush(struct mtd_blktrans_dev *dev)
{
	struct mtdblk_dev *mtdblk = container_of(dev, struct mtdblk_dev, mbd);
	int ret;

	mutex_lock(&mtdblk->cache_mutex);
	ret = write_cached_data(mtdblk);
	mutex_unlock(&mtdblk->cache_mutex);
	mtd_sync(dev->mtd);

	return ret;
}

static void mtdblock_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
#ifdef CONFIG_AMLOGIC_NAND
	int i = 0;
	struct mtdblk_pbbt *pbbt;
#endif
	struct mtdblk_dev *dev;

#ifdef CONFIG_AMLOGIC_NAND
	pbbt = kzalloc(sizeof(*pbbt), GFP_KERNEL);
	if (!pbbt)
		return;
	dev = &pbbt->mb;
#else
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return;
#endif

	dev->mbd.mtd = mtd;
	dev->mbd.devnum = mtd->index;

#if (IS_ENABLED(CONFIG_AMLOGIC_MTD_NAND) || \
			IS_ENABLED(CONFIG_AMLOGIC_MTD_SPI_NAND))
	tr->blksize = mtd->writesize;
	tr->blkshift = ffs(tr->blksize) - 1;
	dev->mbd.size = mtd->size >> tr->blkshift;
#else
	dev->mbd.size = mtd->size >> 9;
#endif

#ifdef CONFIG_AMLOGIC_NAND
	if (!mtd_can_have_bb(mtd))
		goto _ok;

	for (i = 0; i < (mtd->size >> mtd->erasesize_shift); i++)
		if (mtd_block_isbad(mtd, i * mtd->erasesize))
#if (IS_ENABLED(CONFIG_AMLOGIC_MTD_NAND) || \
			IS_ENABLED(CONFIG_AMLOGIC_MTD_SPI_NAND))
			dev->mbd.size -= (mtd->erasesize >> tr->blkshift);
#else
			dev->mbd.size -= (mtd->erasesize >> 9);
#endif
_ok:
#endif
	dev->mbd.tr = tr;

	if (!(mtd->flags & MTD_WRITEABLE))
		dev->mbd.readonly = 1;

#ifdef CONFIG_AMLOGIC_NAND
	if (add_mtd_blktrans_dev(&dev->mbd))
		kfree(pbbt);
#else
	if (add_mtd_blktrans_dev(&dev->mbd))
		kfree(dev);
#endif
}

static void mtdblock_remove_dev(struct mtd_blktrans_dev *dev)
{
	del_mtd_blktrans_dev(dev);
}

static struct mtd_blktrans_ops mtdblock_tr = {
	.name		= "mtdblock",
	.major		= MTD_BLOCK_MAJOR,
	.part_bits	= 0,
	.blksize 	= 512,
	.open		= mtdblock_open,
	.flush		= mtdblock_flush,
	.release	= mtdblock_release,
	.readsect	= mtdblock_readsect,
	.writesect	= mtdblock_writesect,
	.add_mtd	= mtdblock_add_mtd,
	.remove_dev	= mtdblock_remove_dev,
	.owner		= THIS_MODULE,
};

module_mtd_blktrans(mtdblock_tr);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nicolas Pitre <nico@fluxnic.net> et al.");
MODULE_DESCRIPTION("Caching read/erase/writeback block device emulation access to MTD devices");
