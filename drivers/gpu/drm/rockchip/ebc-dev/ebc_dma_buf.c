// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Rockchip Electronics Co., Ltd.
 */

#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/scatterlist.h>
#include <linux/device.h>
#include <linux/dma-heap.h>
#include <linux/poll.h>
#include "ebc_dma_buf.h"

struct ebc_dmabuf {
	phys_addr_t phy_addr;
	size_t size;
	int npages;
	struct page *pages[];
};

static inline struct ebc_dmabuf *to_ebc_dmabuf(struct dma_buf *buf)
{
	return buf->priv;
}

static struct sg_table *ebc_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction dir)
{
	struct ebc_dmabuf *ebcbuf = to_ebc_dmabuf(attachment->dmabuf);
	struct sg_table *st;
	int ret;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table_from_pages(st, ebcbuf->pages, ebcbuf->npages, 0,
					ebcbuf->npages << PAGE_SHIFT, GFP_KERNEL);
	if (ret)
		goto err_free_st;

	ret = dma_map_sgtable(attachment->dev, st, dir, 0);
	if (ret)
		goto err_free_table;

	return st;

err_free_table:
	sg_free_table(st);
err_free_st:
	kfree(st);
	return ERR_PTR(ret);
}

static void ebc_unmap_dma_buf(struct dma_buf_attachment *attachment, struct sg_table *st,
			      enum dma_data_direction dir)
{
	dma_unmap_sgtable(attachment->dev, st, dir, 0);
	sg_free_table(st);
	kfree(st);
}

static int ebc_dmabuf_vmap(struct dma_buf *dma_buf, struct iosys_map *map)
{
	struct ebc_dmabuf *ebcbuf = to_ebc_dmabuf(dma_buf);
	void *vaddr;

	vaddr = vmap(ebcbuf->pages, ebcbuf->npages, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	if (!vaddr)
		return -ENOMEM;
	iosys_map_set_vaddr(map, vaddr);

	return 0;
}

static void ebc_dmabuf_vunmap(struct dma_buf *dma_buf, struct iosys_map *map)
{
	vunmap(map->vaddr);
	iosys_map_clear(map);
}

static int ebc_dmabuf_mmap(struct dma_buf *dma_buf, struct vm_area_struct *vma)
{
	struct ebc_dmabuf *ebcbuf = to_ebc_dmabuf(dma_buf);
	unsigned long pfn;
	int ret;

	pfn = ((unsigned long)(ebcbuf->phy_addr) >> PAGE_SHIFT);

#if defined(CONFIG_ARM)
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
#endif
	ret = remap_pfn_range(vma, vma->vm_start, pfn, vma->vm_end - vma->vm_start,
			      vma->vm_page_prot);
	if (ret)
		return -EAGAIN;

	return 0;
}

static void ebc_dmabuf_release(struct dma_buf *dma_buf)
{
	struct ebc_dmabuf *ebcbuf = to_ebc_dmabuf(dma_buf);

	kfree(ebcbuf);
}

static const struct dma_buf_ops ebc_dmabuf_ops = {
	.map_dma_buf = ebc_map_dma_buf,
	.unmap_dma_buf = ebc_unmap_dma_buf,
	.release = ebc_dmabuf_release,
	.mmap = ebc_dmabuf_mmap,
	.vmap = ebc_dmabuf_vmap,
	.vunmap = ebc_dmabuf_vunmap,
};

struct dma_buf *ebc_get_dma_buf(phys_addr_t phy_addr, size_t size)
{
	struct ebc_dmabuf *ebcbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	int i, npages;

	if (size % PAGE_SIZE)
		return ERR_PTR(-EINVAL);

	npages = size / PAGE_SIZE;
	ebcbuf = kmalloc(sizeof(*ebcbuf) + npages * sizeof(struct page *), GFP_KERNEL);
	if (!ebcbuf)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < npages; i++)
		ebcbuf->pages[i] = phys_to_page(phy_addr + i * PAGE_SIZE);

	ebcbuf->phy_addr = phy_addr;
	ebcbuf->size = size;
	ebcbuf->npages = npages;

	exp_info.ops = &ebc_dmabuf_ops;
	exp_info.size = npages * PAGE_SIZE;
	exp_info.flags = O_RDWR;
	exp_info.priv = ebcbuf;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf))
		goto err;

	return dmabuf;

err:
	kfree(ebcbuf);
	return ERR_PTR(-ENOMEM);
}
EXPORT_SYMBOL(ebc_get_dma_buf);

static int ebc_alloc_buf_from_heap(struct device *dev, struct ebc_dma_buf_t *buf, size_t size, bool cma)
{
	int ret = 0;
	struct dma_heap *heap;
	struct dma_buf *dbuf;
	void *vaddr;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;

	heap = dma_heap_find(cma ? "cma" : "system-dma32");
	if (!heap) {
		heap = dma_heap_find("linux,cma");
		if (!heap) {
			dev_err(dev, "failed to find dma heap\n");
			return -EINVAL;
		}
	}

	dbuf = dma_heap_buffer_alloc(heap, size, O_CLOEXEC|O_RDWR, 0);
	if (IS_ERR(dbuf)) {
		ret = PTR_ERR(dbuf);
		dev_err(dev, "failed to alloc dma buffer from heap, ret: %d\n", ret);
		goto err_alloc_dbuf;
	}

	vaddr = dma_buf_vmap(dbuf, &buf->map) ? NULL : buf->map.vaddr;

	if (vaddr == NULL) {
		dev_err(dev, "dma_buf_vmap failed\n");
		ret = -ENOMEM;
		goto err_vmap;
	}
	memset(vaddr, 0, size);

	attachment = dma_buf_attach(dbuf, dev);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		dev_err(dev, "dma_buf_attach failed, ret: %d\n", ret);
		goto err_attach;
	}

	sgt = dma_buf_map_attachment(attachment, DMA_TO_DEVICE);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		dev_err(dev, "map_attachment failed, ret: %d\n", ret);
		goto err_map;
	}

	buf->dbuf = dbuf;
	buf->attachment = attachment;
	buf->sgt = sgt;
	buf->paddr = sg_dma_address(sgt->sgl);
	buf->vaddr = vaddr;
	buf->size = size;

	return 0;

err_map:
	dma_buf_detach(dbuf, attachment);
err_attach:
	dma_buf_vunmap(dbuf, &buf->map);
err_vmap:
	dma_buf_put(dbuf);
err_alloc_dbuf:

	return ret;
}

static void ebc_free_buf_to_heap(struct ebc_dma_buf_t *buf)
{
	if (buf->dbuf == NULL)
		return;

	if (buf->attachment && buf->sgt) {
		dma_buf_unmap_attachment(buf->attachment, buf->sgt, DMA_TO_DEVICE);
		buf->sgt = NULL;
	}
	if (buf->attachment) {
		dma_buf_detach(buf->dbuf, buf->attachment);
		buf->attachment = NULL;
	}
	if (buf->vaddr) {
		dma_buf_vunmap(buf->dbuf, &buf->map);
		buf->vaddr = NULL;
	}
	dma_buf_put(buf->dbuf);
	buf->dbuf = NULL;
}

int ebc_alloc_dma_buf(struct device *dev, struct ebc_dma_buf_t *buf, size_t size)
{
	return ebc_alloc_buf_from_heap(dev, buf, size, false);
}

int ebc_alloc_dma_cma_buf(struct device *dev, struct ebc_dma_buf_t *buf, size_t size)
{
	return ebc_alloc_buf_from_heap(dev, buf, size, true);
}

void ebc_free_dma_buf(struct ebc_dma_buf_t *buf)
{
	ebc_free_buf_to_heap(buf);
}
