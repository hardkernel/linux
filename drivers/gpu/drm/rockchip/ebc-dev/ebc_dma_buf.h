/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Rockchip Electronics Co., Ltd.
 */

#ifndef EBC_DMA_BUF_H
#define EBC_DMA_BUF_H

#include <linux/dma-buf.h>
#include <linux/version.h>
#include <linux/iosys-map.h>

struct ebc_dma_buf_t {
	unsigned long paddr;
	char *vaddr;
	size_t size;
	int id;

	struct dma_buf *dbuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	struct iosys_map map;
};

/**
 * ebc_get_dma_buf - Convert ebc buffer to dma_buf
 * @phy_addr: Physical address of the ebc buffer
 * @size: Size of the ebc buffer
 *
 * The caller must ensure that the physical address and size of the ebc buffer
 * are aligned to PAGE_SIZE.
 */
struct dma_buf *ebc_get_dma_buf(phys_addr_t phy_addr, size_t size);
int ebc_alloc_dma_buf(struct device *dev, struct ebc_dma_buf_t *buf, size_t size);
int ebc_alloc_dma_cma_buf(struct device *dev, struct ebc_dma_buf_t *buf, size_t size);
void ebc_free_dma_buf(struct ebc_dma_buf_t *buf);

#endif /* EBC_DMA_BUF_H */
