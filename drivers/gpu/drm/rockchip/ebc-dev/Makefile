# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_ROCKCHIP_EBC_DEV)   += rkebc.o ebc_public.o ebc_dma_buf.o
obj-$(CONFIG_ROCKCHIP_EBC_DEV)   += epdlut/
obj-$(CONFIG_ROCKCHIP_EBC_DEV)   += bufmanage/
obj-$(CONFIG_ROCKCHIP_EBC_DEV)   += pmic/
obj-$(CONFIG_ROCKCHIP_EBC_DEV)   += tcon/

AFLAGS_ebc_dev_y8_v8.o += -march=armv8-a+lse
rkebc-$(CONFIG_ARM64) += ebc_dev_y8_v8.o
