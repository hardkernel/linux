#
# arch/arm/boot/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1995-2002 Russell King
#
OBJCOPYFLAGS	:=-O binary -R .comment -S

ifeq ($(CONFIG_AMLOGIC_RAMDUMP_TEXTOFFSET),y)
zreladdr-y       += 0x02008000
params_phys-y    := 0x02000100
initrd_phys-y    := 0x02900000
# Note: the following conditions must always be true:
#   ZRELADDR == virt_to_phys(PAGE_OFFSET + TEXT_OFFSET)
#   PARAMS_PHYS must be within 4MB of ZRELADDR
#   INITRD_PHYS must be in RAM
ZRELADDR    := $(zreladdr-y)
PARAMS_PHYS := $(params_phys-y)
INITRD_PHYS := $(initrd_phys-y)
endif

#   ZRELADDR == virt_to_phys(PAGE_OFFSET + TEXT_OFFSET)
ifdef CONFIG_PHYS_OFFSET
add_hex = $(shell printf 0x%x $$(( $(1) + $(2) )) )
ZRELADDR    := $(call add_hex, $(CONFIG_PHYS_OFFSET), $(TEXT_OFFSET))
endif

PHYS_OFFSET := $(CONFIG_PHYS_OFFSET)
export ZRELADDR PARAMS_PHYS PHYS_OFFSET

targets := Image zImage xipImage bootpImage uImage

ifeq ($(CONFIG_XIP_KERNEL),y)

cmd_deflate_xip_data = $(CONFIG_SHELL) -c '$(src)/deflate_xip_data.sh $< $@'

ifeq ($(CONFIG_XIP_DEFLATED_DATA),y)
quiet_cmd_mkxip = XIPZ    $@
cmd_mkxip = $(cmd_objcopy) && $(cmd_deflate_xip_data)
else
quiet_cmd_mkxip = $(quiet_cmd_objcopy)
cmd_mkxip = $(cmd_objcopy)
endif

$(obj)/xipImage: vmlinux FORCE
	$(call if_changed,mkxip)
	@$(kecho) '  Physical Address of xipImage: $(CONFIG_XIP_PHYS_ADDR)'

$(obj)/Image $(obj)/zImage: FORCE
	@echo 'Kernel configured for XIP (CONFIG_XIP_KERNEL=y)'
	@echo 'Only the xipImage target is available in this case'
	@false

else

$(obj)/xipImage: FORCE
	@echo 'Kernel not configured for XIP (CONFIG_XIP_KERNEL!=y)'
	@false

$(obj)/Image: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/compressed/vmlinux: $(obj)/Image FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed $@

$(obj)/zImage:	$(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)

endif

ifneq ($(LOADADDR),)
  UIMAGE_LOADADDR=$(LOADADDR)
else
  ifeq ($(CONFIG_ZBOOT_ROM),y)
    UIMAGE_LOADADDR=$(CONFIG_ZBOOT_ROM_TEXT)
  else
    UIMAGE_LOADADDR=$(ZRELADDR)
  endif
endif

check_for_multiple_loadaddr = \
if [ $(words $(UIMAGE_LOADADDR)) -ne 1 ]; then \
	echo 'multiple (or no) load addresses: $(UIMAGE_LOADADDR)'; \
	echo 'This is incompatible with uImages'; \
	echo 'Specify LOADADDR on the commandline to build an uImage'; \
	false; \
fi

$(obj)/uImage:	$(obj)/zImage FORCE
	@$(check_for_multiple_loadaddr)
	$(call if_changed,uimage)

$(obj)/bootp/bootp: $(obj)/zImage FORCE
	$(Q)$(MAKE) $(build)=$(obj)/bootp $@

$(obj)/bootpImage: $(obj)/bootp/bootp FORCE
	$(call if_changed,objcopy)

subdir-	    := bootp compressed dts
