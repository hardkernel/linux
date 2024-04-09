#
# arch/arm64/boot/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 2012, ARM Ltd.
# Author: Will Deacon <will.deacon@arm.com>
#
# Based on the ia64 boot/Makefile.
#

OBJCOPYFLAGS_Image :=-O binary -R .note -R .note.gnu.build-id -R .comment -S

targets := Image Image.bz2 Image.gz Image.lz4 Image.lzma Image.lzo Image.zstd

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/Image.bz2: $(obj)/Image FORCE
	$(call if_changed,bzip2)

$(obj)/Image.gz: $(obj)/Image FORCE
	$(call if_changed,gzip)

ifneq ($(CONFIG_AMLOGIC_AMFC), )
#targets += Image.zstd

ifeq ($(srctree),)
srctree := $(patsubst %/,%,$(dir $(CURDIR)))
srctree := $(patsubst %/,%,$(dir $(srctree)))
endif

zstd_src := $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/debug.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/entropy_common.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/error_private.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/fse_decompress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/pool.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/threading.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/xxhash.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/common/zstd_common.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/fse_compress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/hist.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/huf_compress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_compress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_compress_literals.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_compress_sequences.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_compress_superblock.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_double_fast.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_fast.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_lazy.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_ldm.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstd_opt.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/compress/zstdmt_compress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/decompress/huf_decompress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/decompress/huf_decompress_amd64.S
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/decompress/zstd_ddict.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/decompress/zstd_decompress.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/decompress/zstd_decompress_block.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/dictBuilder/cover.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/dictBuilder/divsufsort.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/dictBuilder/fastcover.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/dictBuilder/zdict.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/legacy/zstd_v05.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/legacy/zstd_v06.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/legacy/zstd_v07.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/benchfn.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/benchzstd.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/datagen.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/dibio.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/fileio.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/timefn.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/util.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/zstdcli.c
zstd_src += $(srctree)/common_drivers/drivers/amfc/zstd_tool/zstdcli_trace.c

$(obj)/zstd:
	$(HOSTCC) $(zstd_src) $(LDFLAGS) -o $@
	cp -n $@ $(srctree)/

$(obj)/Image.zstd: $(obj)/Image $(obj)/zstd FORCE
	$(srctree)/zstd -19 $< -o $@
else
#use lz4 instead
$(obj)/Image.zstd: $(obj)/Image FORCE
	$(call if_changed,lz4)
endif

$(obj)/Image.lz4: $(obj)/Image FORCE
	$(call if_changed,lz4)

$(obj)/Image.lzma: $(obj)/Image FORCE
	$(call if_changed,lzma)

$(obj)/Image.lzo: $(obj)/Image FORCE
	$(call if_changed,lzo)
