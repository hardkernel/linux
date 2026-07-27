// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020-2024 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

	.text
	.file	"ebc_dev_y8.c"

	.section	.initcall6s.init,"a",@progbits
__initcall__kmod_rkebc__529_4908_ebc_init6s:
.Ltmp0:
	.word	ebc_init-.Ltmp0
	.text



	.section	.exit.text,"ax",@progbits
	.p2align	2
	.type	ebc_exit,@function
ebc_exit:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x0, ebc_driver
	mov	x29, sp
	add	x0, x0, :lo12:ebc_driver
	bl	platform_driver_unregister
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end0:
	.size	ebc_exit, .Lfunc_end0-ebc_exit

	.section	.init.text,"ax",@progbits
	.p2align	2
	.type	ebc_init,@function
ebc_init:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x0, ebc_driver
	mov	x1, xzr
	add	x0, x0, :lo12:ebc_driver
	mov	x29, sp
	bl	__platform_driver_register
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end1:
	.size	ebc_init, .Lfunc_end1-ebc_init

	.text
	.p2align	2
	.type	ebc_probe,@function
ebc_probe:
	sub	sp, sp, #224
	str	x30, [x18], #8
	stp	x29, x30, [sp, #128]
	adrp	x1, .L.str.1
	stp	x28, x27, [sp, #144]
	adrp	x2, .L.str.2
	stp	x26, x25, [sp, #160]
	add	x29, sp, #128
	stp	x24, x23, [sp, #176]
	add	x1, x1, :lo12:.L.str.1
	stp	x22, x21, [sp, #192]
	add	x2, x2, :lo12:.L.str.2
	stp	x20, x19, [sp, #208]
	mrs	x8, SP_EL0
	add	x19, x0, #16
	mov	x20, x0
	ldr	x8, [x8, #1584]
	mov	x0, x19
	stur	x8, [x29, #-8]
	str	xzr, [sp, #32]
	str	wzr, [sp, #28]
	str	xzr, [sp, #16]
	bl	_dev_info
	mov	x0, x19
	mov	w1, #18640
	mov	w2, #3520
	bl	devm_kmalloc
	cbz	x0, .LBB2_6
	adrp	x8, global_ebc
	adrp	x1, .L.str.3
	mov	x21, x0
	str	x19, [x0]
	add	x1, x1, :lo12:.L.str.3
	add	x5, sp, #40
	str	x0, [x8, :lo12:global_ebc]
	ldr	x0, [x20, #672]
	mov	x2, xzr
	mov	w3, wzr
	mov	w4, wzr
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	stp	xzr, xzr, [sp, #72]
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	bl	__of_parse_phandle_with_args
	cbnz	w0, .LBB2_9
	ldr	x0, [sp, #40]
	cbz	x0, .LBB2_9
	bl	of_find_device_by_node
	cbz	x0, .LBB2_5
	ldr	x8, [x0, #136]
	str	x8, [x21, #8]
	cbnz	x8, .LBB2_11
.LBB2_5:
	mov	w24, #-517
	b	.LBB2_7
.LBB2_6:
	mov	w24, #-12
.LBB2_7:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB2_95
	mov	w0, w24
	ldp	x20, x19, [sp, #208]
	ldp	x22, x21, [sp, #192]
	ldp	x24, x23, [sp, #176]
	ldp	x26, x25, [sp, #160]
	ldp	x28, x27, [sp, #144]
	ldp	x29, x30, [sp, #128]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #224
	ret
.LBB2_9:
	adrp	x1, .L.str.4
	add	x1, x1, :lo12:.L.str.4
.LBB2_10:
	mov	x0, x19
	bl	_dev_err
	mov	w24, #-19
	b	.LBB2_7
.LBB2_11:
	adrp	x9, frame_done_callback
	adrp	x1, .L.str.5
	add	x9, x9, :lo12:frame_done_callback
	add	x1, x1, :lo12:.L.str.5
	mov	x0, x19
	str	x9, [x8, #232]
	bl	nvmem_cell_get
	cmn	x0, #4095
	b.lo	.LBB2_13
	adrp	x1, .L.str.6
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.6
	b	.LBB2_25
.LBB2_13:
	add	x1, sp, #16
	mov	x23, x0
	bl	nvmem_cell_read
	mov	x22, x0
	mov	x0, x23
	bl	nvmem_cell_put
	cmn	x22, #4095
	b.lo	.LBB2_15
	adrp	x1, .L.str.7
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.7
	b	.LBB2_25
.LBB2_15:
	ldrb	w8, [x22]
	cmp	w8, #51
	b.eq	.LBB2_20
	cmp	w8, #53
	b.ne	.LBB2_24
	ldrb	w9, [x22, #1]
	sub	w8, w9, #102
	cmp	w8, #16
	b.hi	.LBB2_22
	adrp	x9, .LJTI2_0
	mov	w23, #3566
	add	x9, x9, :lo12:.LJTI2_0
	adr	x10, .LBB2_19
	ldrb	w11, [x9, x8]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB2_19:
	mov	w23, #3572
	b	.LBB2_29
.LBB2_20:
	ldrb	w8, [x22, #1]
	mov	w23, #3326
	cmp	w8, #38
	b.eq	.LBB2_29
	cmp	w8, #48
	b.eq	.LBB2_29
	b	.LBB2_24
.LBB2_22:
	cmp	w9, #6
	b.ne	.LBB2_24
	mov	w23, #3506
	b	.LBB2_29
.LBB2_24:
	mov	x0, x22
	bl	kfree
	adrp	x1, .L.str.8
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.8
.LBB2_25:
	bl	_dev_err
	ldr	x0, [x21, #8]
	ldr	x8, [x0, #256]
	blr	x8
	cmp	w0, #4
	b.hi	.LBB2_27
	adrp	x8, .Lswitch.table.ebc_probe.255
	add	x8, x8, :lo12:.Lswitch.table.ebc_probe.255
	ldr	w23, [x8, w0, sxtw #2]
	b	.LBB2_30
.LBB2_27:
	mov	w23, #-1
	b	.LBB2_30
.LBB2_28:
	mov	w23, #3576
.LBB2_29:
	mov	x0, x22
	bl	kfree
.LBB2_30:
	adrp	x1, .L.str.9
	ldr	x0, [x20, #672]
	add	x1, x1, :lo12:.L.str.9
	add	x2, sp, #28
	bl	of_get_property
	ldr	w8, [sp, #28]
	asr	w9, w8, #2
	cmp	w8, #3
	str	w9, [sp, #28]
	b.hi	.LBB2_32
	adrp	x1, .L.str.10
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.10
	bl	_dev_err
	mov	w24, #-22
	b	.LBB2_7
.LBB2_32:
	cmp	w8, #4
	b.lt	.LBB2_86
	adrp	x25, .L.str.14
	adrp	x27, .L.str.16
	mov	x22, x0
	mov	x24, xzr
	mov	w26, wzr
	add	x25, x25, :lo12:.L.str.14
	add	x27, x27, :lo12:.L.str.16
	str	w23, [sp, #12]
	b	.LBB2_37
.LBB2_34:
	adrp	x1, .L.str.15
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.15
.LBB2_35:
	bl	_dev_warn
.LBB2_36:
	ldr	w8, [sp, #28]
	add	x24, x24, #1
	cmp	w24, w8
	b.ge	.LBB2_49
.LBB2_37:
	ldr	w8, [x22, x24, lsl #2]
	rev	w0, w8
	bl	of_find_node_by_phandle
	cbz	x0, .LBB2_46
	bl	of_find_i2c_device_by_node
	cbz	x0, .LBB2_47
	mov	x23, x0
	ldr	x8, [x0, #152]
	str	x8, [x21, #16]
	cbnz	x8, .LBB2_48
	add	x28, x23, #32
	mov	x1, x25
	mov	x0, x28
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #24]
	b.hs	.LBB2_34
	add	x2, sp, #32
	mov	x0, x28
	mov	x1, x27
	bl	device_property_read_string
	cbnz	w0, .LBB2_44
	ldr	x0, [sp, #32]
	bl	thermal_zone_get_zone_by_name
	cmn	x0, #4095
	str	x0, [x21, #104]
	b.hs	.LBB2_45
	mov	w26, #1
	b	.LBB2_36
.LBB2_44:
	adrp	x1, .L.str.17
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.17
	b	.LBB2_35
.LBB2_45:
	adrp	x1, .L.str.18
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.18
	bl	_dev_err
	b	.LBB2_36
.LBB2_46:
	adrp	x1, .L.str.11
	add	x1, x1, :lo12:.L.str.11
	b	.LBB2_10
.LBB2_47:
	adrp	x1, .L.str.12
	add	x1, x1, :lo12:.L.str.12
	b	.LBB2_10
.LBB2_48:
	adrp	x1, .L.str.13
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.13
	mov	w2, w24
	bl	_dev_info
	ldr	w8, [sp, #12]
	b	.LBB2_50
.LBB2_49:
	ldr	w8, [sp, #12]
	tbz	w26, #0, .LBB2_86
.LBB2_50:
	cmp	w8, #3326
	str	x19, [x21, #304]
	str	w8, [x21, #2100]
	str	wzr, [x21, #2108]
	b.eq	.LBB2_53
	cmp	w8, #3506
	b.eq	.LBB2_53
	adrp	x1, .L.str.20
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.20
	bl	device_property_present
	tbz	w0, #0, .LBB2_54
.LBB2_53:
	mov	w8, #1
	str	w8, [x21, #2108]
.LBB2_54:
	mov	x22, x21
	adrp	x1, .L.str.21
	add	x1, x1, :lo12:.L.str.21
	mov	x0, x19
	str	x19, [x22, #112]!
	ldp	x8, x9, [x22, #-104]
	stp	x8, x9, [x22, #8]
	bl	device_property_present
	tbz	w0, #0, .LBB2_56
	mov	w8, #3576
	str	w8, [x21, #2104]
.LBB2_56:
	adrp	x1, .L.str.22
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.22
	bl	device_property_present
	tbz	w0, #0, .LBB2_58
	mov	w8, #1
	str	w8, [x21, #2092]
.LBB2_58:
	mov	x0, x21
	bl	ebc_panel_probe
	cbnz	w0, .LBB2_87
	ldp	w9, w8, [x21, #188]
	ldr	w10, [x21, #2108]
	str	w8, [x21, #512]
	str	w9, [x21, #516]
	cbz	w10, .LBB2_62
	ldr	w8, [x21, #272]
	cbnz	w8, .LBB2_88
	ldr	w8, [x21, #280]
	cbnz	w8, .LBB2_88
.LBB2_62:
	ldr	w8, [x21, #264]
	cmp	w8, #230
	b.ne	.LBB2_82
	ldr	w8, [x21, #272]
	cbnz	w8, .LBB2_89
	ldr	w8, [x21, #280]
	cbnz	w8, .LBB2_89
	add	x23, x23, #32
	adrp	x1, .L.str.26
	add	x1, x1, :lo12:.L.str.26
	mov	x0, x23
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #32]
	b.lo	.LBB2_67
	adrp	x1, .L.str.27
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.27
	bl	_dev_warn
.LBB2_67:
	adrp	x1, .L.str.28
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.28
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #40]
	b.lo	.LBB2_69
	adrp	x1, .L.str.29
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.29
	bl	_dev_warn
.LBB2_69:
	adrp	x1, .L.str.30
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.30
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #48]
	b.lo	.LBB2_71
	adrp	x1, .L.str.31
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.31
	bl	_dev_warn
.LBB2_71:
	adrp	x1, .L.str.32
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.32
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #56]
	b.lo	.LBB2_73
	adrp	x1, .L.str.33
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.33
	bl	_dev_warn
.LBB2_73:
	adrp	x1, .L.str.34
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.34
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #64]
	b.lo	.LBB2_75
	adrp	x1, .L.str.35
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.35
	bl	_dev_warn
.LBB2_75:
	adrp	x1, .L.str.36
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.36
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #72]
	b.lo	.LBB2_77
	adrp	x1, .L.str.37
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.37
	bl	_dev_warn
.LBB2_77:
	adrp	x1, .L.str.38
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.38
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #80]
	b.lo	.LBB2_79
	adrp	x1, .L.str.39
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.39
	bl	_dev_warn
.LBB2_79:
	adrp	x1, .L.str.40
	mov	x0, x23
	add	x1, x1, :lo12:.L.str.40
	bl	devm_regulator_get
	cmn	x0, #4095
	str	x0, [x21, #88]
	b.lo	.LBB2_81
	adrp	x1, .L.str.41
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.41
	bl	_dev_warn
.LBB2_81:
	ldr	x8, [x21, #24]
	str	x8, [x21, #96]
.LBB2_82:
	add	x23, x21, #304
	mov	x0, x23
	bl	ebc_lut_table_init
	cbnz	w0, .LBB2_91
	mov	x0, x21
	bl	ebc_ability_init
	mov	x0, x21
	bl	ebc_buffer_manage_init
	cbnz	w0, .LBB2_92
	adrp	x8, nr_cpu_ids
	adrp	x0, __cpu_possible_mask
	add	x0, x0, :lo12:__cpu_possible_mask
	ldr	w1, [x8, :lo12:nr_cpu_ids]
	bl	__bitmap_weight
	str	w0, [x21, #2088]
	mov	x0, x23
	bl	ebc_task_init
	mov	w24, w0
	cbnz	w0, .LBB2_93
	mov	x0, x23
	bl	ebc_other_init
	mov	x0, x21
	mov	x1, x22
	bl	ebc_logo_init
	mov	x0, x23
	str	x21, [x20, #136]
	bl	ebc_sysfs_init
	adrp	x1, .L.str.45
	adrp	x2, .L.str.2
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.45
	add	x2, x2, :lo12:.L.str.2
	bl	_dev_info
	b	.LBB2_7
.LBB2_86:
	adrp	x1, .L.str.19
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.19
	bl	_dev_err
	b	.LBB2_5
.LBB2_87:
	mov	w24, w0
	adrp	x1, .L.str.23
	add	x1, x1, :lo12:.L.str.23
	mov	x0, x19
	mov	w2, w24
	bl	_dev_err
	b	.LBB2_7
.LBB2_88:
	adrp	x1, .L.str.24
	add	x1, x1, :lo12:.L.str.24
	b	.LBB2_90
.LBB2_89:
	adrp	x1, .L.str.25
	add	x1, x1, :lo12:.L.str.25
.LBB2_90:
	mov	x0, x19
	bl	_dev_err
	mov	w24, wzr
	b	.LBB2_7
.LBB2_91:
	adrp	x1, .L.str.42
	mov	w24, w0
	add	x1, x1, :lo12:.L.str.42
	b	.LBB2_94
.LBB2_92:
	adrp	x1, .L.str.43
	mov	w24, w0
	add	x1, x1, :lo12:.L.str.43
	b	.LBB2_94
.LBB2_93:
	adrp	x1, .L.str.44
	add	x1, x1, :lo12:.L.str.44
.LBB2_94:
	mov	x0, x19
	bl	_dev_err
	b	.LBB2_7
.LBB2_95:
	bl	__stack_chk_fail
.Lfunc_end2:
	.size	ebc_probe, .Lfunc_end2-ebc_probe
	.section	.rodata,"a",@progbits
.LJTI2_0:
	.byte	(.LBB2_29-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_29-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_19-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_24-.LBB2_19)>>2
	.byte	(.LBB2_28-.LBB2_19)>>2

	.text
	.p2align	2
	.type	ebc_remove,@function
ebc_remove:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	adrp	x8, global_ebc
	adrp	x0, ebc_misc
	add	x0, x0, :lo12:ebc_misc
	str	x19, [sp, #16]
	mov	x29, sp
	ldr	x19, [x8, :lo12:global_ebc]
	bl	misc_deregister
	mov	x0, x19
	bl	ebc_buffer_manage_uninit
	ldr	x19, [sp, #16]
	mov	w0, wzr
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end3:
	.size	ebc_remove, .Lfunc_end3-ebc_remove

	.p2align	2
	.type	frame_done_callback,@function
frame_done_callback:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	stp	x20, x19, [sp, #32]
	adrp	x19, global_ebc
	mov	w8, #18633
	mov	w9, #18634
	mov	w10, #18635
	str	x21, [sp, #16]
	ldr	x20, [x19, :lo12:global_ebc]
	mov	x29, sp
	ldrb	w8, [x20, x8]
	ldrb	w9, [x20, x9]
	orr	w8, w9, w8
	tst	w8, #0xff
	strb	w8, [x20, x10]
	b.ne	.LBB4_2
	ldrb	w8, [x20, #2086]
	cbz	w8, .LBB4_4
.LBB4_2:
	ldr	w8, [x20, #2096]
	cmp	w8, #1
	b.gt	.LBB4_9
.LBB4_3:
	adrp	x0, ebc_flip_wq
	mov	w1, #1
	add	x0, x0, :lo12:ebc_flip_wq
	strb	wzr, [x20, #1872]
	bl	__wake_up_sync
	adrp	x8, ebc_task
	ldr	x0, [x8, :lo12:ebc_task]
	bl	wake_up_process
	ldp	x20, x19, [sp, #32]
	ldr	x21, [sp, #16]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.LBB4_4:
	mov	w8, #18536
	add	x21, x20, x8
	ldrb	w8, [x21, #65]
	cbz	w8, .LBB4_10
	ldrb	w8, [x21, #92]
	cbz	w8, .LBB4_8
	ldrb	w8, [x21, #93]
	cbz	w8, .LBB4_8
	mov	w8, #1
	strb	w8, [x21, #94]
.LBB4_8:
	ldrb	w8, [x21, #94]
	cbnz	w8, .LBB4_11
	b	.LBB4_2
.LBB4_9:
	ldr	w8, [x20, #184]
	mov	w9, #1
	adrp	x1, .L.str.46
	adrp	x2, .L.str.47
	ldr	x0, [x20, #304]
	add	x1, x1, :lo12:.L.str.46
	sub	w3, w9, w8
	add	x2, x2, :lo12:.L.str.47
	bl	_dev_info
	b	.LBB4_3
.LBB4_10:
	ldr	w8, [x21]
	cmp	w8, #1
	b.ge	.LBB4_2
.LBB4_11:
	ldrb	w8, [x20, #2084]
	cbnz	w8, .LBB4_2
	ldrb	w8, [x20, #2085]
	cbnz	w8, .LBB4_2
	ldr	x0, [x20, #8]
	mov	w1, wzr
	mov	w2, wzr
	ldr	x8, [x0, #240]
	blr	x8
	ldrb	w8, [x20, #1880]
	tbz	w8, #3, .LBB4_15
	ldrb	w8, [x21, #40]
	tbnz	w8, #1, .LBB4_2
.LBB4_15:
	ldr	x21, [x19, :lo12:global_ebc]
	ldr	x19, [x21, #312]
	bl	ebc_get_power_off_delay
	bl	__msecs_to_jiffies
	mov	x3, x0
	add	x2, x21, #320
	mov	w0, #8
	mov	x1, x19
	bl	queue_delayed_work_on
	b	.LBB4_2
.Lfunc_end4:
	.size	frame_done_callback, .Lfunc_end4-frame_done_callback

	.p2align	2
	.type	ebc_panel_probe,@function
ebc_panel_probe:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	mov	x2, x0
	stp	x20, x19, [sp, #32]
	adrp	x1, .L.str.48
	mov	x19, x0
	add	x1, x1, :lo12:.L.str.48
	mov	w3, #1
	ldr	x20, [x2], #188
	mov	x4, xzr
	stp	x22, x21, [sp, #16]
	mov	x29, sp
	ldr	x0, [x20, #656]
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	add	x21, x20, #656
	adrp	x1, .L.str.49
	add	x2, x19, #192
	add	x1, x1, :lo12:.L.str.49
	mov	w3, #1
	mov	x4, xzr
	ldr	x0, [x21]
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.50
	ldr	x0, [x21]
	add	x2, x19, #196
	add	x1, x1, :lo12:.L.str.50
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.51
	ldr	x0, [x21]
	add	x2, x19, #200
	add	x1, x1, :lo12:.L.str.51
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.52
	ldr	x0, [x21]
	add	x2, x19, #216
	add	x1, x1, :lo12:.L.str.52
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.53
	ldr	x0, [x21]
	add	x2, x19, #220
	add	x1, x1, :lo12:.L.str.53
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.54
	ldr	x0, [x21]
	add	x2, x19, #224
	add	x1, x1, :lo12:.L.str.54
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.55
	ldr	x0, [x21]
	add	x2, x19, #228
	add	x1, x1, :lo12:.L.str.55
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.56
	ldr	x0, [x21]
	add	x2, x19, #232
	add	x1, x1, :lo12:.L.str.56
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.57
	ldr	x0, [x21]
	add	x2, x19, #236
	add	x1, x1, :lo12:.L.str.57
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.58
	ldr	x0, [x21]
	add	x2, x19, #240
	add	x1, x1, :lo12:.L.str.58
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.59
	ldr	x0, [x21]
	add	x2, x19, #244
	add	x1, x1, :lo12:.L.str.59
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.60
	ldr	x0, [x21]
	add	x2, x19, #248
	add	x1, x1, :lo12:.L.str.60
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.61
	ldr	x0, [x21]
	add	x2, x19, #252
	add	x1, x1, :lo12:.L.str.61
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	adrp	x1, .L.str.62
	ldr	x0, [x21]
	add	x2, x19, #256
	add	x1, x1, :lo12:.L.str.62
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB5_40
	add	x20, x19, #260
	adrp	x1, .L.str.63
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.63
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_18
	adrp	x1, .L.str.64
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.64
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_18
	str	wzr, [x20]
.LBB5_18:
	add	x20, x19, #264
	adrp	x1, .L.str.65
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.65
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_20
	str	wzr, [x20]
.LBB5_20:
	add	x20, x19, #268
	adrp	x1, .L.str.66
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.66
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_22
	str	wzr, [x20]
.LBB5_22:
	add	x20, x19, #204
	adrp	x1, .L.str.67
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.67
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_24
	str	wzr, [x20]
.LBB5_24:
	add	x20, x19, #208
	adrp	x1, .L.str.68
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.68
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_26
	str	wzr, [x20]
.LBB5_26:
	add	x20, x19, #272
	adrp	x1, .L.str.69
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.69
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_28
	str	wzr, [x20]
.LBB5_28:
	add	x20, x19, #284
	adrp	x1, .L.str.70
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.70
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_30
	str	wzr, [x20]
.LBB5_30:
	add	x20, x19, #288
	adrp	x1, .L.str.71
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.71
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_32
	str	wzr, [x20]
.LBB5_32:
	add	x20, x19, #292
	adrp	x1, .L.str.72
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.72
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	mov	w22, #1
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_34
	str	w22, [x20]
.LBB5_34:
	add	x20, x19, #276
	adrp	x1, .L.str.73
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.73
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_36
	str	wzr, [x20]
.LBB5_36:
	add	x20, x19, #280
	adrp	x1, .L.str.74
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.74
	mov	x2, x20
	mov	w3, #1
	mov	x4, xzr
	mov	w22, #1
	bl	of_property_read_variable_u32_array
	tbz	w0, #31, .LBB5_38
	str	wzr, [x20]
.LBB5_38:
	adrp	x1, .L.str.75
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.75
	mov	x2, xzr
	bl	of_find_property
	cmp	x0, #0
	ldr	x8, [x19, #120]
	cset	w9, ne
	strb	w9, [x19, #296]
	str	wzr, [x8, #104]
	ldr	x8, [x19, #120]
	str	wzr, [x8, #116]
	ldr	w8, [x19, #264]
	cmp	w8, #230
	ldr	w8, [x19, #188]
	cinc	w9, w22, ne
	lsr	w8, w8, w9
	ldr	x9, [x19, #120]
	str	w8, [x9, #120]
	ldr	w8, [x19, #2108]
	cbz	w8, .LBB5_42
	ldp	w8, w9, [x19, #220]
	ldp	w10, w11, [x19, #228]
	mov	w0, wzr
	add	w8, w9, w8
	ldr	x9, [x19, #120]
	add	w8, w8, w10
	add	w8, w8, w11
	str	w8, [x9, #108]
	ldp	w8, w9, [x19, #244]
	ldp	w10, w11, [x19, #252]
	add	w8, w9, w8
	ldr	x9, [x19, #120]
	add	w8, w8, w10
	add	w8, w8, w11
	str	w8, [x9, #112]
	mov	w8, #4
	ldr	x9, [x19, #120]
	str	w8, [x9, #116]
	ldr	x8, [x19, #120]
	ldr	w9, [x8, #108]
	ldr	w10, [x8, #116]
	mul	w9, w10, w9
	str	w9, [x8, #120]
	ldp	w8, w9, [x19, #220]
	add	w8, w9, w8
	ldr	x9, [x19, #120]
	str	w8, [x9, #104]
	b	.LBB5_41
.LBB5_40:
	mov	w0, #-22
.LBB5_41:
	ldp	x20, x19, [sp, #32]
	ldp	x22, x21, [sp, #16]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.LBB5_42:
	mov	w0, wzr
	b	.LBB5_41
.Lfunc_end5:
	.size	ebc_panel_probe, .Lfunc_end5-ebc_panel_probe

	.p2align	2
	.type	ebc_lut_table_init,@function
ebc_lut_table_init:
	sub	sp, sp, #208
	str	x30, [x18], #8
	stp	x29, x30, [sp, #160]
	add	x29, sp, #160
	stp	x22, x21, [sp, #176]
	adrp	x1, .L.str.76
	stp	x20, x19, [sp, #192]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	x19, x0
	add	x1, x1, :lo12:.L.str.76
	add	x5, sp, #72
	mov	x2, xzr
	mov	w3, wzr
	stur	x8, [x29, #-8]
	add	x8, sp, #72
	ldr	x20, [x0]
	mov	w4, wzr
	stp	xzr, xzr, [sp, #56]
	stp	xzr, xzr, [sp, #40]
	ldr	x22, [x20, #656]
	stp	xzr, xzr, [sp, #24]
	stp	xzr, xzr, [sp, #8]
	str	wzr, [sp, #4]
	mov	x0, x22
	stp	xzr, xzr, [x8, #64]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	stp	xzr, xzr, [sp, #72]
	bl	__of_parse_phandle_with_args
	mov	w8, w0
	mov	w0, #-19
	cbnz	w8, .LBB6_9
	ldr	x21, [sp, #72]
	cbz	x21, .LBB6_9
	adrp	x1, .L.str.77
	add	x2, sp, #4
	add	x1, x1, :lo12:.L.str.77
	mov	x0, x22
	bl	of_get_property
	ldr	w1, [sp, #4]
	cmp	w1, #1
	b.lt	.LBB6_5
	bl	pvi_wf_add_custom_mode_table
	cbz	w0, .LBB6_5
	mov	w0, #-19
	b	.LBB6_9
.LBB6_5:
	add	x2, sp, #8
	mov	x0, x21
	mov	w1, wzr
	bl	of_address_to_resource
	cbnz	w0, .LBB6_9
	ldp	x1, x8, [sp, #8]
	mov	x0, x20
	mov	w3, #1
	str	x1, [x19, #1488]
	sub	x8, x8, x1
	add	x21, x8, #1
	sxtw	x2, w21
	str	w21, [x19, #1496]
	bl	devm_memremap
	str	x0, [x19, #1560]
	cbz	x0, .LBB6_11
	bl	epd_lut_from_mem_init
	tbnz	w0, #31, .LBB6_12
.LBB6_8:
	mov	w0, wzr
.LBB6_9:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB6_14
	ldp	x20, x19, [sp, #192]
	ldp	x22, x21, [sp, #176]
	ldp	x29, x30, [sp, #160]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #208
	ret
.LBB6_11:
	adrp	x1, .L.str.78
	adrp	x2, .L__func__.ebc_lut_table_init
	ldr	x0, [x19]
	add	x1, x1, :lo12:.L.str.78
	add	x2, x2, :lo12:.L__func__.ebc_lut_table_init
	mov	w3, #4032
	bl	_dev_err
	mov	w0, #-12
	b	.LBB6_9
.LBB6_12:
	adrp	x1, .L.str.79
	ldr	x0, [x19]
	add	x1, x1, :lo12:.L.str.79
	bl	_dev_err
	ldr	x0, [x19]
	mov	w2, w21
	ldr	x1, [x19, #1560]
	bl	epd_lut_from_file_init
	tbz	w0, #31, .LBB6_8
	adrp	x1, .L.str.80
	ldr	x0, [x19]
	add	x1, x1, :lo12:.L.str.80
	bl	_dev_err
	mov	w0, #-1
	b	.LBB6_9
.LBB6_14:
	bl	__stack_chk_fail
.Lfunc_end6:
	.size	ebc_lut_table_init, .Lfunc_end6-ebc_lut_table_init

	.p2align	2
	.type	ebc_ability_init,@function
ebc_ability_init:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	stp	x20, x19, [sp, #16]
	ldr	x20, [x0]
	mov	x29, sp
	mov	x19, x0
	bl	epd_lut_get_wf_bit
	ldr	w8, [x19, #2100]
	cmp	w0, #32
	cset	w9, eq
	mov	w10, #257
	cmp	w8, #3572
	strb	w9, [x19, #2114]
	strh	w10, [x19, #2112]
	b.ne	.LBB7_3
	ldr	w8, [x19, #272]
	cbnz	w8, .LBB7_3
	ldr	w8, [x19, #268]
	cbz	w8, .LBB7_9
.LBB7_3:
	mov	w8, wzr
.LBB7_4:
	ldr	w9, [x19, #2108]
	strb	w8, [x19, #2117]
	cbnz	w9, .LBB7_6
	ldr	w8, [x19, #264]
	cmp	w8, #230
	b.ne	.LBB7_8
.LBB7_6:
	mov	w8, wzr
.LBB7_7:
	strb	w8, [x19, #2115]
	adrp	x1, .L.str.82
	ldr	x0, [x20, #656]
	add	x1, x1, :lo12:.L.str.82
	mov	x2, xzr
	bl	of_find_property
	cmp	x0, #0
	cset	w8, eq
	strb	w8, [x19, #2116]
	ldp	x20, x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.LBB7_8:
	adrp	x1, .L.str.81
	ldr	x0, [x20, #656]
	add	x1, x1, :lo12:.L.str.81
	mov	x2, xzr
	bl	of_find_property
	cmp	x0, #0
	cset	w8, eq
	b	.LBB7_7
.LBB7_9:
	mov	w8, #1
	b	.LBB7_4
.Lfunc_end7:
	.size	ebc_ability_init, .Lfunc_end7-ebc_ability_init

	.p2align	2
	.type	ebc_buffer_manage_init,@function
ebc_buffer_manage_init:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-64]!
	ldp	w8, w9, [x0, #188]
	stp	x20, x19, [sp, #48]
	mov	x19, x0
	ldr	x20, [x0]
	stp	x24, x23, [sp, #16]
	stp	x22, x21, [sp, #32]
	mov	x29, sp
	mul	w8, w9, w8
	ldr	w9, [x0, #2108]
	str	wzr, [x0, #524]
	str	w8, [x0, #504]
	cbz	w9, .LBB8_2
	ldr	x8, [x19, #120]
	ldp	w9, w10, [x19, #244]
	ldp	w13, w12, [x19, #252]
	ldr	w11, [x8, #108]
	ldr	w8, [x8, #116]
	add	w9, w10, w9
	mul	w22, w11, w9
	add	w9, w13, w12
	madd	w9, w9, w11, w22
	mul	w8, w9, w8
	b	.LBB8_3
.LBB8_2:
	ldr	w9, [x19, #264]
	cmp	w9, #230
	lsr	w9, w8, #2
	lsr	w8, w8, #1
	csel	w22, wzr, wzr, ne
	csel	w8, w9, w8, ne
.LBB8_3:
	add	x1, x19, #536
	sxtw	x2, w8
	mov	x0, x20
	str	w8, [x19, #508]
	bl	ebc_alloc_dma_cma_buf
	cbnz	w0, .LBB8_71
	ldr	x8, [x19, #536]
	add	x1, x19, #608
	ldr	x9, [x19, #544]
	mov	x0, x20
	ldr	x10, [x19, #552]
	ldrsw	x2, [x19, #508]
	stp	x9, x8, [x19, #136]
	str	x10, [x19, #152]
	bl	ebc_alloc_dma_cma_buf
	cbnz	w0, .LBB8_73
	ldr	x8, [x19, #608]
	ldr	x0, [x19, #616]
	ldr	x9, [x19, #624]
	ldr	w10, [x19, #2108]
	stp	x0, x8, [x19, #160]
	str	x9, [x19, #176]
	cbz	w10, .LBB8_37
	ldr	x11, [x19, #120]
	ldr	w8, [x11, #112]
	cbz	w8, .LBB8_35
	ldr	w12, [x19, #284]
	mov	w16, #1024
	mov	x9, xzr
	mov	w10, wzr
	mov	w17, wzr
	mov	w14, wzr
	cmp	w12, #0
	ldr	w11, [x11, #108]
	mov	w12, #1
	cset	w13, ne
	mov	w15, #-16777216
	movk	w16, #65280, lsl #16
	b	.LBB8_9
.LBB8_8:
	add	w10, w10, #1
	add	x9, x9, x11
	cmp	w10, w8
	b.eq	.LBB8_34
.LBB8_9:
	cbz	w11, .LBB8_8
	mov	x0, xzr
	b	.LBB8_13
.LBB8_11:
	and	w3, w13, #0xff
	cmp	x0, x1
	csel	w1, w16, w15, lo
	and	w4, w12, #0xff
	bfi	w1, w3, #12, #8
	add	w3, w9, w0
	bfi	w1, w2, #11, #1
	and	w2, w14, #0xff
	orr	w1, w1, w4, lsl #15
	orr	w1, w1, w17, lsl #14
	orr	w1, w1, w2, lsl #13
	ldr	x2, [x19, #136]
	str	w1, [x2, w3, uxtw #2]
.LBB8_12:
	add	x0, x0, #1
	cmp	x11, x0
	b.eq	.LBB8_8
.LBB8_13:
	ldp	w2, w1, [x19, #244]
	add	w1, w1, w2
	cmp	w10, w1
	b.hs	.LBB8_17
	ldp	w1, w5, [x19, #220]
	ldp	w3, w4, [x19, #236]
	add	w3, w3, w1
	cmp	x0, x3
	add	w4, w4, w3
	cset	w3, hs
	cmp	w10, w2
	cset	w2, hs
	cmp	x0, x4
	cset	w4, lo
	csel	w12, w12, w2, lo
	add	w2, w5, w1
	cmp	x0, x2
	b.lo	.LBB8_19
	ldr	w5, [x19, #228]
	add	w2, w5, w2
	cmp	x0, x2
	b.hs	.LBB8_19
	mov	w2, wzr
	b	.LBB8_20
.LBB8_17:
	ldr	w14, [x19, #252]
	add	w14, w14, w1
	cmp	w10, w14
	b.ls	.LBB8_21
	ldr	w17, [x19, #284]
	mov	w14, wzr
	cmp	w17, #0
	csel	w13, wzr, w13, eq
	b	.LBB8_22
.LBB8_19:
	mov	w2, #1
.LBB8_20:
	tst	w17, #0xff
	and	w17, w3, w4
	cset	w3, eq
	tst	w3, w17
	csinc	w14, w14, wzr, eq
	b	.LBB8_32
.LBB8_21:
	mov	w13, #1
	mov	w14, #1
.LBB8_22:
	ldr	w1, [x19, #220]
	ldr	w17, [x19, #236]
	add	w17, w17, w1
	cmp	x0, x17
	b.lo	.LBB8_25
	ldr	w2, [x19, #240]
	add	w17, w2, w17
	cmp	x0, x17
	b.hs	.LBB8_25
	mov	w17, #1
	b	.LBB8_26
.LBB8_25:
	mov	w17, wzr
.LBB8_26:
	ldr	w2, [x19, #224]
	add	w2, w2, w1
	cmp	x0, x2
	b.lo	.LBB8_31
	ldr	w3, [x19, #288]
	cbnz	w3, .LBB8_29
	ldr	w3, [x19, #228]
.LBB8_29:
	add	w2, w3, w2
	cmp	x0, x2
	b.hs	.LBB8_31
	mov	w2, wzr
	b	.LBB8_32
.LBB8_31:
	mov	w2, #1
.LBB8_32:
	ldr	x3, [x19, #120]
	mov	w1, w1
	ldr	w3, [x3, #116]
	cmp	w3, #2
	b.ne	.LBB8_11
	cmp	x0, x1
	lsl	w3, w12, #15
	cset	w1, lo
	bfi	w3, w17, #14, #1
	lsl	w1, w1, #10
	bfi	w1, w13, #12, #20
	bfi	w1, w2, #11, #1
	add	w2, w9, w0
	orr	w1, w1, w14, lsl #13
	orr	w1, w1, w3
	ldr	x3, [x19, #136]
	strh	w1, [x3, w2, uxtw #1]
	b	.LBB8_12
.LBB8_34:
	ldr	x0, [x19, #160]
.LBB8_35:
	ldr	x1, [x19, #136]
	ldr	x2, [x19, #152]
	bl	memcpy
	ldr	w8, [x19, #2108]
	cbz	w8, .LBB8_37
	ldr	x8, [x19, #120]
	ldr	x10, [x19, #136]
	ldr	x11, [x19, #160]
	ldr	w9, [x8, #116]
	mul	w9, w9, w22
	add	x9, x10, x9
	str	x9, [x19, #1848]
	ldr	w9, [x8, #116]
	mul	w9, w9, w22
	add	x9, x11, x9
	str	x9, [x19, #1856]
	ldr	w9, [x8, #116]
	mul	w9, w9, w22
	add	x9, x10, x9
	str	x9, [x19, #1848]
	ldr	w8, [x8, #116]
	mul	w8, w8, w22
	add	x8, x11, x8
	b	.LBB8_38
.LBB8_37:
	ldr	x9, [x19, #136]
	ldr	x8, [x19, #160]
	str	x9, [x19, #1848]
.LBB8_38:
	ldrb	w9, [x19, #2117]
	str	x8, [x19, #1856]
	cbz	w9, .LBB8_41
	add	x1, x19, #1616
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	bl	ebc_alloc_dma_cma_buf
	cbnz	w0, .LBB8_77
	add	x1, x19, #1688
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	bl	ebc_alloc_dma_cma_buf
	cbnz	w0, .LBB8_78
.LBB8_41:
	bl	ebc_buf_list_init
	cbnz	w0, .LBB8_75
	add	x23, x19, #528
	adrp	x1, .L.str.87
	ldr	x0, [x20, #656]
	add	x1, x1, :lo12:.L.str.87
	mov	x2, x23
	mov	w3, #1
	mov	x4, xzr
	bl	of_property_read_variable_u32_array
	tbnz	w0, #31, .LBB8_44
	ldr	w8, [x23]
	sub	w8, w8, #5
	cmn	w8, #3
	b.hi	.LBB8_45
.LBB8_44:
	mov	w8, #4
	str	w8, [x23]
.LBB8_45:
	mov	x22, xzr
	add	x24, x19, #680
.LBB8_46:
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	mov	x1, x24
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_76
	ldr	w0, [x19, #524]
	add	w8, w0, #1
	str	w8, [x19, #524]
	ldp	x1, x2, [x24, #8]
	str	w0, [x24, #24]
	bl	ebc_dsp_buf_init
	cbnz	w0, .LBB8_70
	ldrsw	x8, [x23]
	add	x22, x22, #1
	add	x24, x24, #72
	cmp	x22, x8
	b.lt	.LBB8_46
	ldrb	w8, [x19, #2115]
	cbz	w8, .LBB8_52
	add	x1, x19, #1112
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_81
	ldr	w8, [x19, #524]
	mov	w1, #255
	ldr	x0, [x19, #1120]
	ldrsw	x2, [x19, #504]
	add	w9, w8, #1
	str	w8, [x19, #1136]
	str	w9, [x19, #524]
	bl	memset
	ldr	w0, [x19, #1136]
	ldr	x1, [x19, #1120]
	ldr	x2, [x19, #1128]
	bl	ebc_osd_buf_init
	cbnz	w0, .LBB8_70
.LBB8_52:
	add	x1, x19, #968
	mov	x0, x20
	mov	w2, #1048576
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_72
	ldr	x8, [x19, #976]
	ldrb	w9, [x19, #2115]
	str	x8, [x19, #1832]
	cbz	w9, .LBB8_58
	add	x1, x19, #1040
	mov	x0, x20
	mov	w2, #1048576
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_72
	ldr	x8, [x19, #1048]
	ldrb	w9, [x19, #2115]
	str	x8, [x19, #1840]
	cbz	w9, .LBB8_58
	mov	w8, #1
	ldrb	w9, [x19, #2116]
	cbnz	w9, .LBB8_59
.LBB8_57:
	cbnz	w8, .LBB8_62
	b	.LBB8_61
.LBB8_58:
	mov	w8, wzr
	str	xzr, [x19, #1840]
	ldrb	w9, [x19, #2116]
	cbz	w9, .LBB8_57
.LBB8_59:
	add	x1, x19, #1184
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_86
	ldr	w9, [x19, #524]
	ldrb	w8, [x19, #2115]
	add	w10, w9, #1
	str	w9, [x19, #1208]
	str	w10, [x19, #524]
	cbnz	w8, .LBB8_62
.LBB8_61:
	ldrb	w8, [x19, #2112]
	cbz	w8, .LBB8_69
.LBB8_62:
	add	x1, x19, #1256
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_79
	ldr	x0, [x19, #1264]
	mov	w1, #255
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	w8, [x19, #524]
	add	x1, x19, #1328
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	add	w9, w8, #1
	str	w8, [x19, #1280]
	str	w9, [x19, #524]
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_80
	ldr	x0, [x19, #1336]
	mov	w1, #255
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	w8, [x19, #524]
	add	x1, x19, #1472
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	add	w9, w8, #1
	str	w8, [x19, #1352]
	str	w9, [x19, #524]
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_84
	ldr	x0, [x19, #1480]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	w8, [x19, #524]
	add	x1, x19, #1544
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	add	w9, w8, #1
	str	w8, [x19, #1496]
	str	w9, [x19, #524]
	bl	ebc_alloc_dma_buf
	cbnz	w0, .LBB8_85
	ldr	x0, [x19, #1552]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	w8, [x19, #524]
	ldrb	w10, [x19, #2112]
	add	w9, w8, #1
	str	w8, [x19, #1568]
	str	w9, [x19, #524]
	cbz	w10, .LBB8_69
	add	x1, x19, #1400
	ldrsw	x2, [x19, #504]
	mov	x0, x20
	bl	ebc_alloc_dma_buf
	mov	w21, w0
	cbnz	w0, .LBB8_87
	ldr	x0, [x19, #1408]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	w8, [x19, #524]
	add	w9, w8, #1
	str	w8, [x19, #1424]
	str	w9, [x19, #524]
	b	.LBB8_90
.LBB8_69:
	mov	w21, wzr
	b	.LBB8_90
.LBB8_70:
	adrp	x1, .L.str.89
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.89
	b	.LBB8_82
.LBB8_71:
	mov	w21, w0
	mov	w2, wzr
	b	.LBB8_74
.LBB8_72:
	adrp	x1, .L.str.91
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.91
	b	.LBB8_88
.LBB8_73:
	mov	w21, w0
	mov	w2, #1
.LBB8_74:
	adrp	x1, .L.str.83
	mov	x0, x20
	add	x1, x1, :lo12:.L.str.83
	b	.LBB8_83
.LBB8_75:
	adrp	x1, .L.str.86
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.86
	b	.LBB8_88
.LBB8_76:
	adrp	x1, .L.str.88
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.88
	b	.LBB8_82
.LBB8_77:
	adrp	x1, .L.str.84
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.84
	b	.LBB8_88
.LBB8_78:
	adrp	x1, .L.str.85
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.85
	b	.LBB8_88
.LBB8_79:
	adrp	x1, .L.str.93
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.93
	b	.LBB8_88
.LBB8_80:
	adrp	x1, .L.str.94
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.94
	b	.LBB8_88
.LBB8_81:
	adrp	x1, .L.str.90
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.90
.LBB8_82:
	mov	x0, x20
	mov	w2, w22
.LBB8_83:
	mov	w3, w21
	bl	_dev_err
	b	.LBB8_89
.LBB8_84:
	adrp	x1, .L.str.95
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.95
	b	.LBB8_88
.LBB8_85:
	adrp	x1, .L.str.96
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.96
	b	.LBB8_88
.LBB8_86:
	adrp	x1, .L.str.92
	mov	w21, w0
	add	x1, x1, :lo12:.L.str.92
	b	.LBB8_88
.LBB8_87:
	adrp	x1, .L.str.97
	add	x1, x1, :lo12:.L.str.97
.LBB8_88:
	mov	x0, x20
	mov	w2, w21
	bl	_dev_err
.LBB8_89:
	mov	x0, x19
	bl	ebc_buffer_manage_uninit
.LBB8_90:
	mov	w0, w21
	ldp	x20, x19, [sp, #48]
	ldp	x22, x21, [sp, #32]
	ldp	x24, x23, [sp, #16]
	ldp	x29, x30, [sp], #64
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end8:
	.size	ebc_buffer_manage_init, .Lfunc_end8-ebc_buffer_manage_init

	.p2align	2
	.type	ebc_task_init,@function
ebc_task_init:
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #16]
	adrp	x3, .L.str.98
	stp	x22, x21, [sp, #32]
	mov	w9, #99
	stp	x20, x19, [sp, #48]
	mrs	x8, SP_EL0
	mov	x19, x0
	adrp	x0, ebc_assist_thread
	ldr	x8, [x8, #1584]
	add	x0, x0, :lo12:ebc_assist_thread
	add	x3, x3, :lo12:.L.str.98
	mov	x1, x19
	mov	w2, #-1
	add	x29, sp, #16
	str	x8, [sp, #8]
	str	w9, [sp]
	bl	kthread_create_on_node
	adrp	x20, ebc_assist_task
	cmn	x0, #4095
	str	x0, [x20, :lo12:ebc_assist_task]
	b.hs	.LBB9_15
	mov	x2, sp
	mov	w1, #1
	bl	sched_setscheduler_nocheck
	ldr	w8, [x19, #1784]
	cmp	w8, #8
	b.lt	.LBB9_3
	adrp	x1, cpu_bit_bitmap+56
	ldr	x0, [x20, :lo12:ebc_assist_task]
	add	x1, x1, :lo12:cpu_bit_bitmap+56
	bl	set_cpus_allowed_ptr
.LBB9_3:
	ldr	x8, [x20, :lo12:ebc_assist_task]
	add	x0, x8, #64
	//APP
.Ltmp2:
	b	.Ltmp1
.Ltmp3:
	.section	.altinstructions,"a",@progbits
.Ltmp4:
	.word	.Ltmp2-.Ltmp4
.Ltmp5:
	.word	alt_cb_patch_nops-.Ltmp5
	.hword	32794
	.byte	.Ltmp3-.Ltmp2
	.byte	.Ltmp6-.Ltmp7
	.text

.Ltmp7:
.Ltmp6:

	//NO_APP
	mov	w8, #1
	//APP
	ldadd	w8, w8, [x0]
	//NO_APP
	cbz	w8, .LBB9_22
.LBB9_5:
	add	w9, w8, #1
	orr	w8, w9, w8
	tbnz	w8, #31, .LBB9_18
.LBB9_6:
	adrp	x0, ebc_thread
	adrp	x3, .L.str.100
	add	x0, x0, :lo12:ebc_thread
	add	x3, x3, :lo12:.L.str.100
	mov	x1, x19
	mov	w2, #-1
	bl	kthread_create_on_node
	adrp	x20, ebc_task
	cmn	x0, #4095
	str	x0, [x20, :lo12:ebc_task]
	b.hs	.LBB9_16
	mov	x2, sp
	mov	w1, #1
	bl	sched_setscheduler_nocheck
	ldr	w8, [x19, #1784]
	cmp	w8, #8
	b.lt	.LBB9_9
	adrp	x1, cpu_bit_bitmap+64
	ldr	x0, [x20, :lo12:ebc_task]
	add	x1, x1, :lo12:cpu_bit_bitmap+64
	bl	set_cpus_allowed_ptr
.LBB9_9:
	ldr	x8, [x20, :lo12:ebc_task]
	add	x0, x8, #64
	//APP
.Ltmp9:
	b	.Ltmp8
.Ltmp10:
	.section	.altinstructions,"a",@progbits
.Ltmp11:
	.word	.Ltmp9-.Ltmp11
.Ltmp12:
	.word	alt_cb_patch_nops-.Ltmp12
	.hword	32794
	.byte	.Ltmp10-.Ltmp9
	.byte	.Ltmp13-.Ltmp14
	.text

.Ltmp14:
.Ltmp13:

	//NO_APP
	mov	w8, #1
	//APP
	ldadd	w8, w8, [x0]
	//NO_APP
	cbz	w8, .LBB9_25
.LBB9_11:
	add	w9, w8, #1
	orr	w8, w9, w8
	tbnz	w8, #31, .LBB9_19
.LBB9_12:
	adrp	x20, .L.str.102
	adrp	x3, .L.str.103
	add	x20, x20, :lo12:.L.str.102
	mov	w1, #14
	add	x3, x3, :lo12:.L.str.103
	mov	x0, x20
	movk	w1, #10, lsl #16
	mov	w2, #1
	bl	alloc_workqueue
	mov	x22, #68719476704
	add	x8, x19, #24
	adrp	x21, delayed_work_timer_fn
	mov	w2, #2097152
	add	x21, x21, :lo12:delayed_work_timer_fn
	mov	x3, xzr
	stp	x0, x22, [x19, #8]
	add	x0, x19, #48
	str	x8, [x19, #24]
	mov	x1, x21
	str	x8, [x19, #32]
	adrp	x8, ebc_delay_power_off
	add	x8, x8, :lo12:ebc_delay_power_off
	mov	x4, xzr
	str	x8, [x19, #40]
	bl	init_timer_key
	adrp	x3, .L.str.104
	mov	w1, #14
	add	x3, x3, :lo12:.L.str.104
	mov	x0, x20
	movk	w1, #10, lsl #16
	mov	w2, #1
	bl	alloc_workqueue
	add	x8, x19, #120
	stp	x0, x22, [x19, #104]
	add	x0, x19, #144
	mov	x1, x21
	mov	w2, #2097152
	mov	x3, xzr
	str	x8, [x19, #120]
	mov	x4, xzr
	str	x8, [x19, #128]
	adrp	x8, ebc_wake_up_for_ghost_rm_last
	add	x8, x8, :lo12:ebc_wake_up_for_ghost_rm_last
	str	x8, [x19, #136]
	bl	init_timer_key
	mov	w0, wzr
.LBB9_13:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldr	x9, [sp, #8]
	cmp	x8, x9
	b.ne	.LBB9_20
	ldp	x20, x19, [sp, #48]
	ldp	x22, x21, [sp, #32]
	ldp	x29, x30, [sp, #16]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	ret
.LBB9_15:
	adrp	x1, .L.str.99
	ldr	x0, [x19]
	add	x1, x1, :lo12:.L.str.99
	b	.LBB9_17
.LBB9_16:
	ldr	x0, [x19]
	adrp	x1, .L.str.101
	add	x1, x1, :lo12:.L.str.101
.LBB9_17:
	bl	_dev_err
	mov	w0, #-1
	b	.LBB9_13
.LBB9_18:
	mov	w1, #1
	b	.LBB9_23
.LBB9_19:
	mov	w1, #1
	b	.LBB9_26
.LBB9_20:
	bl	__stack_chk_fail
.Ltmp1:
.LBB9_21:
	//APP
	// atomic_fetch_add_relaxed
	prfm	pstl1strm, [x0]
.Ltmp15:
	ldxr	w8, [x0]
	add	w9, w8, #1
	stxr	w10, w9, [x0]
	cbnz	w10, .Ltmp15

	//NO_APP
	cbnz	w8, .LBB9_5
.LBB9_22:
	mov	w1, #2
.LBB9_23:
	bl	refcount_warn_saturate
	b	.LBB9_6
.Ltmp8:
.LBB9_24:
	//APP
	// atomic_fetch_add_relaxed
	prfm	pstl1strm, [x0]
.Ltmp16:
	ldxr	w8, [x0]
	add	w9, w8, #1
	stxr	w10, w9, [x0]
	cbnz	w10, .Ltmp16

	//NO_APP
	cbnz	w8, .LBB9_11
.LBB9_25:
	mov	w1, #2
.LBB9_26:
	bl	refcount_warn_saturate
	b	.LBB9_12
.Lfunc_end9:
	.size	ebc_task_init, .Lfunc_end9-ebc_task_init

	.p2align	2
	.type	ebc_other_init,@function
ebc_other_init:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	mov	x8, #8589934592
	add	x9, x0, #1572
	mov	w10, #18208
	stp	x20, x19, [sp, #16]
	mov	x29, sp
	mov	x19, x0
	add	x20, x0, x10
	str	xzr, [x0, #1816]
	str	x8, [x9]
	strb	wzr, [x0, #1568]
	bl	ebc_get_init_log_level
	mov	w8, #1
	ldrb	w9, [x19, #1808]
	mov	x10, #1
	str	w0, [x19, #1792]
	movk	x10, #2, lsl #32
	adds	x0, x19, #1584
	stp	w8, w9, [x20, #36]
	mov	x9, #4294967297
	str	wzr, [x20, #76]
	stur	x10, [x20, #44]
	mov	w10, #18327
	str	x9, [x20, #80]
	ldr	w9, [x19, #212]
	add	x10, x19, x10
	str	wzr, [x20, #4]
	strb	w8, [x20, #32]
	stp	w9, wzr, [x20, #108]
	mov	w9, #-1
	strh	wzr, [x20, #100]
	strb	w8, [x20, #118]
	strb	wzr, [x19, #1782]
	str	w9, [x20, #28]
	str	xzr, [x19, #1464]
	str	xzr, [x19, #1480]
	strb	wzr, [x10, #4]
	str	wzr, [x10]
	strb	w8, [x20]
	str	w9, [x19, #1500]
	str	wzr, [x20, #124]
	str	wzr, [x19, #216]
	b.eq	.LBB10_2
	adrp	x8, .L.str.181
	str	xzr, [x19, #1768]
	add	x8, x8, :lo12:.L.str.181
	str	xzr, [x19, #1760]
	str	xzr, [x19, #1752]
	str	xzr, [x19, #1744]
	str	xzr, [x19, #1736]
	str	xzr, [x19, #1728]
	str	xzr, [x19, #1720]
	str	xzr, [x19, #1712]
	str	xzr, [x19, #1704]
	str	xzr, [x19, #1696]
	str	xzr, [x19, #1688]
	str	xzr, [x19, #1680]
	str	xzr, [x19, #1672]
	str	xzr, [x19, #1664]
	str	xzr, [x19, #1656]
	str	xzr, [x19, #1648]
	str	xzr, [x19, #1640]
	str	xzr, [x19, #1632]
	str	xzr, [x19, #1624]
	str	xzr, [x19, #1616]
	str	xzr, [x19, #1608]
	str	xzr, [x19, #1600]
	str	xzr, [x19, #1592]
	str	x8, [x19, #1584]
.LBB10_2:
	bl	wakeup_source_add
	str	wzr, [x19, #1776]
	ldp	x20, x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end10:
	.size	ebc_other_init, .Lfunc_end10-ebc_other_init

	.p2align	2
	.type	ebc_logo_init,@function
ebc_logo_init:
	sub	sp, sp, #256
	str	x30, [x18], #8
	stp	x29, x30, [sp, #160]
	add	x29, sp, #160
	stp	x28, x27, [sp, #176]
	adrp	x9, saved_command_line
	stp	x26, x25, [sp, #192]
	mov	x25, x0
	stp	x24, x23, [sp, #208]
	stp	x22, x21, [sp, #224]
	stp	x20, x19, [sp, #240]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	x20, x1
	adrp	x1, .L.str.182
	add	x1, x1, :lo12:.L.str.182
	stur	x8, [x29, #-8]
	ldr	x21, [x9, :lo12:saved_command_line]
	str	xzr, [sp, #64]
	ldr	x19, [x0]
	mov	x0, x21
	bl	strstr
	adrp	x1, .L.str.183
	mov	x24, x0
	add	x1, x1, :lo12:.L.str.183
	mov	x0, x21
	bl	strstr
	adrp	x1, .L.str.184
	mov	x26, x0
	ldp	w22, w23, [x20, #76]
	ldr	x0, [x19, #656]
	add	x1, x1, :lo12:.L.str.184
	add	x5, sp, #72
	mov	x2, xzr
	mov	w3, wzr
	mov	w4, wzr
	stp	xzr, xzr, [sp, #48]
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	stp	xzr, xzr, [sp]
	stp	xzr, xzr, [sp, #136]
	stp	xzr, xzr, [sp, #120]
	stp	xzr, xzr, [sp, #104]
	stp	xzr, xzr, [sp, #88]
	stp	xzr, xzr, [sp, #72]
	bl	__of_parse_phandle_with_args
	cbnz	w0, .LBB11_28
	ldr	x0, [sp, #72]
	cbz	x0, .LBB11_28
	mov	x2, sp
	mov	w1, wzr
	bl	of_address_to_resource
	cbnz	w0, .LBB11_30
	ldp	x21, x8, [sp]
	mul	w9, w23, w22
	lsr	w2, w9, #1
	sub	x8, x8, x21
	add	x8, x8, #1
	cmp	w2, w8
	b.gt	.LBB11_31
	sxtw	x22, w8
	mov	x0, x19
	mov	x1, x21
	mov	x2, x22
	mov	w3, #1
	bl	devm_memremap
	cbz	x0, .LBB11_32
	mov	x23, x0
	cbz	x24, .LBB11_8
	adrp	x1, .L.str.188
	add	x2, sp, #68
	add	x1, x1, :lo12:.L.str.188
	mov	x0, x24
	bl	sscanf
	ldr	w2, [sp, #68]
	cmp	x21, x2
	b.ls	.LBB11_34
	mov	x24, xzr
.LBB11_8:
	mov	w28, #1
	cbz	x26, .LBB11_36
.LBB11_9:
	adrp	x1, .L.str.191
	add	x2, sp, #64
	add	x1, x1, :lo12:.L.str.191
	mov	x0, x26
	bl	sscanf
	ldr	w2, [sp, #64]
	cmp	x21, x2
	b.ls	.LBB11_39
	mov	x26, xzr
	mov	w27, #1
	cbnz	w28, .LBB11_37
.LBB11_11:
	adrp	x0, .L.str.195
	add	x0, x0, :lo12:.L.str.195
	bl	ebc_empty_buf_get
	cbz	x0, .LBB11_17
	ldp	w8, w9, [x20, #76]
	mul	w8, w9, w8
	cmp	w8, #2
	b.lo	.LBB11_15
	lsr	w8, w8, #1
	ldr	x9, [x0, #8]
	mov	x10, x24
.LBB11_14:
	ldrb	w11, [x10], #1
	subs	w8, w8, #1
	lsl	w12, w11, #4
	and	w11, w11, #0xf0
	strb	w12, [x9]
	strb	w11, [x9, #1]
	add	x9, x9, #2
	b.ne	.LBB11_14
.LBB11_15:
	mov	w8, #7
	str	xzr, [x0, #40]
	mov	w9, #1
	str	w8, [x0, #32]
	ldr	w8, [x20, #76]
	str	w8, [x0, #48]
	ldr	w8, [x20, #80]
	stp	w8, w9, [x0, #52]
.LBB11_16:
	bl	ebc_add_to_dsp_buf_list_tail
.LBB11_17:
	tbnz	w27, #0, .LBB11_23
	adrp	x0, .L.str.195
	add	x0, x0, :lo12:.L.str.195
	bl	ebc_empty_buf_get
	cbz	x0, .LBB11_23
	ldp	w8, w9, [x20, #76]
	mul	w8, w9, w8
	cmp	w8, #2
	b.lo	.LBB11_22
	lsr	w8, w8, #1
	ldr	x9, [x0, #8]
	mov	x10, x26
.LBB11_21:
	ldrb	w11, [x10], #1
	subs	w8, w8, #1
	lsl	w12, w11, #4
	and	w11, w11, #0xf0
	strb	w12, [x9]
	strb	w11, [x9, #1]
	add	x9, x9, #2
	b.ne	.LBB11_21
.LBB11_22:
	mov	w8, #7
	str	xzr, [x0, #40]
	mov	w9, #1
	str	w8, [x0, #32]
	ldr	w8, [x20, #76]
	str	w8, [x0, #48]
	ldr	w8, [x20, #80]
	stp	w8, w9, [x0, #52]
	bl	ebc_add_to_dsp_buf_list_tail
.LBB11_23:
	adrp	x8, ebc_task
	ldr	x0, [x8, :lo12:ebc_task]
	bl	wake_up_process
	cbz	x24, .LBB11_25
.LBB11_24:
	mov	x0, x24
	bl	kfree
.LBB11_25:
	cbz	x26, .LBB11_27
	mov	x0, x26
	bl	kfree
.LBB11_27:
	mov	x0, x19
	mov	x1, x23
	bl	devm_memunmap
	add	x1, x22, x21
	mov	x0, x21
	bl	ebc_logo_free_reserved_area
.LBB11_28:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB11_45
	ldp	x20, x19, [sp, #240]
	ldp	x22, x21, [sp, #224]
	ldp	x24, x23, [sp, #208]
	ldp	x26, x25, [sp, #192]
	ldp	x28, x27, [sp, #176]
	ldp	x29, x30, [sp, #160]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #256
	ret
.LBB11_30:
	adrp	x1, .L.str.185
	add	x1, x1, :lo12:.L.str.185
	b	.LBB11_33
.LBB11_31:
	adrp	x1, .L.str.186
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.186
	bl	_dev_err
	b	.LBB11_28
.LBB11_32:
	adrp	x1, .L.str.187
	add	x1, x1, :lo12:.L.str.187
.LBB11_33:
	mov	x0, x19
	bl	_dev_err
	b	.LBB11_28
.LBB11_34:
	adrp	x1, .L.str.189
	ldr	x0, [x25, #304]
	add	x1, x1, :lo12:.L.str.189
	bl	_dev_info
	ldp	w9, w8, [x20, #76]
	mov	w1, #3520
	mul	w8, w8, w9
	lsr	w0, w8, #1
	bl	__kmalloc
	cbz	x0, .LBB11_43
	ldp	w9, w8, [x20, #76]
	ldr	w10, [sp, #68]
	mov	x24, x0
	mul	w8, w8, w9
	sub	x9, x10, x21
	add	x1, x23, x9
	lsr	w2, w8, #1
	bl	memcpy
	mov	w28, wzr
	cbnz	x26, .LBB11_9
.LBB11_36:
	mov	w27, #1
	cbz	w28, .LBB11_11
.LBB11_37:
	adrp	x1, .L.str.194
	ldr	x0, [x25, #304]
	add	x1, x1, :lo12:.L.str.194
	bl	_dev_info
	ldr	x0, [x25, #16]
	cbz	x0, .LBB11_41
	bl	ebc_pmic_verity_vcom
	b	.LBB11_42
.LBB11_39:
	adrp	x1, .L.str.192
	ldr	x0, [x25, #304]
	add	x1, x1, :lo12:.L.str.192
	bl	_dev_info
	ldp	w9, w8, [x20, #76]
	mov	w1, #3520
	mul	w8, w8, w9
	lsr	w0, w8, #1
	bl	__kmalloc
	mov	x26, x0
	cbz	x0, .LBB11_44
	ldp	w9, w8, [x20, #76]
	ldr	w10, [sp, #64]
	mov	x0, x26
	mul	w8, w8, w9
	sub	x9, x10, x21
	add	x1, x23, x9
	lsr	w2, w8, #1
	bl	memcpy
	mov	w27, wzr
	cbz	w28, .LBB11_11
	b	.LBB11_37
.LBB11_41:
	ldr	x0, [x25, #24]
	bl	ebc_regulator_verity_vcom
.LBB11_42:
	adrp	x0, .L.str.196
	add	x0, x0, :lo12:.L.str.196
	bl	ebc_empty_buf_get
	ldp	w8, w9, [x20, #76]
	mov	x25, x0
	ldr	x0, [x0, #8]
	mov	w1, #240
	mul	w2, w9, w8
	bl	memset
	mov	w8, #17
	str	xzr, [x25, #40]
	mov	w9, #1
	mov	x0, x25
	str	w8, [x25, #32]
	ldr	w8, [x20, #76]
	str	w8, [x25, #48]
	ldr	w8, [x20, #80]
	stp	w8, w9, [x25, #52]
	b	.LBB11_16
.LBB11_43:
	adrp	x1, .L.str.190
	ldr	x0, [x25, #304]
	add	x1, x1, :lo12:.L.str.190
	bl	_dev_err
	b	.LBB11_27
.LBB11_44:
	adrp	x1, .L.str.193
	ldr	x0, [x25, #304]
	add	x1, x1, :lo12:.L.str.193
	bl	_dev_err
	cbnz	x24, .LBB11_24
	b	.LBB11_25
.LBB11_45:
	bl	__stack_chk_fail
.Lfunc_end11:
	.size	ebc_logo_init, .Lfunc_end11-ebc_logo_init

	.p2align	2
	.type	ebc_sysfs_init,@function
ebc_sysfs_init:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	str	x19, [sp, #16]
	mov	x19, x0
	adrp	x0, ebc_misc
	mov	x29, sp
	add	x0, x0, :lo12:ebc_misc
	bl	misc_register
	adrp	x0, waveform_misc
	add	x0, x0, :lo12:waveform_misc
	bl	misc_register
	ldrb	w8, [x19, #1812]
	cbz	w8, .LBB12_2
	adrp	x0, transition_lut_misc
	add	x0, x0, :lo12:transition_lut_misc
	bl	misc_register
.LBB12_2:
	adrp	x1, dev_attr_waveform_version
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_waveform_version
	bl	device_create_file
	adrp	x1, dev_attr_pmic_name
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_pmic_name
	bl	device_create_file
	adrp	x1, dev_attr_pmic_temp
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_pmic_temp
	bl	device_create_file
	adrp	x1, dev_attr_pmic_vcom
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_pmic_vcom
	bl	device_create_file
	adrp	x1, dev_attr_ebc_version
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_ebc_version
	bl	device_create_file
	adrp	x1, dev_attr_ebc_state
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_ebc_state
	bl	device_create_file
	adrp	x1, dev_attr_ebc_buf_state
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_ebc_buf_state
	bl	device_create_file
	adrp	x1, dev_attr_auto_frame_state
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_auto_frame_state
	bl	device_create_file
	adrp	x1, dev_attr_ebc_debug_level
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_ebc_debug_level
	bl	device_create_file
	adrp	x1, dev_attr_wf_data
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_wf_data
	bl	device_create_file
	adrp	x1, dev_attr_overlay_wf_data
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_overlay_wf_data
	bl	device_create_file
	adrp	x1, dev_attr_ori_wf_data
	ldr	x0, [x19]
	add	x1, x1, :lo12:dev_attr_ori_wf_data
	bl	device_create_file
	ldr	x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end12:
	.size	ebc_sysfs_init, .Lfunc_end12-ebc_sysfs_init

	.p2align	2
	.type	ebc_buffer_manage_uninit,@function
ebc_buffer_manage_uninit:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	stp	x20, x19, [sp, #32]
	mov	x19, x0
	ldr	x8, [x0, #1552]
	str	x21, [sp, #16]
	mov	x29, sp
	cbz	x8, .LBB13_2
	add	x0, x19, #1544
	bl	ebc_free_dma_buf
.LBB13_2:
	ldr	x8, [x19, #1408]
	cbz	x8, .LBB13_4
	add	x0, x19, #1400
	bl	ebc_free_dma_buf
.LBB13_4:
	ldr	x8, [x19, #1336]
	cbz	x8, .LBB13_6
	add	x0, x19, #1328
	bl	ebc_free_dma_buf
.LBB13_6:
	ldr	x8, [x19, #1264]
	cbz	x8, .LBB13_8
	add	x0, x19, #1256
	bl	ebc_free_dma_buf
.LBB13_8:
	ldr	x8, [x19, #1480]
	cbz	x8, .LBB13_10
	add	x0, x19, #1472
	bl	ebc_free_dma_buf
.LBB13_10:
	ldr	x8, [x19, #1192]
	cbz	x8, .LBB13_12
	add	x0, x19, #1184
	bl	ebc_free_dma_buf
.LBB13_12:
	ldr	x8, [x19, #976]
	cbz	x8, .LBB13_14
	add	x0, x19, #968
	bl	ebc_free_dma_buf
.LBB13_14:
	ldr	x8, [x19, #1048]
	cbz	x8, .LBB13_16
	add	x0, x19, #1040
	bl	ebc_free_dma_buf
.LBB13_16:
	ldr	x8, [x19, #1120]
	cbz	x8, .LBB13_18
	add	x0, x19, #1112
	bl	ebc_free_dma_buf
.LBB13_18:
	ldr	w8, [x19, #528]
	cmp	w8, #1
	b.lt	.LBB13_23
	mov	x21, xzr
	add	x20, x19, #680
	b	.LBB13_21
.LBB13_20:
	add	x21, x21, #1
	add	x20, x20, #72
	cmp	x21, w8, sxtw
	b.ge	.LBB13_23
.LBB13_21:
	ldr	x9, [x20, #8]
	cbz	x9, .LBB13_20
	mov	x0, x20
	bl	ebc_free_dma_buf
	ldr	w8, [x19, #528]
	b	.LBB13_20
.LBB13_23:
	bl	ebc_buf_uninit
	ldr	x8, [x19, #544]
	cbz	x8, .LBB13_25
	add	x0, x19, #536
	bl	ebc_free_dma_buf
.LBB13_25:
	ldr	x8, [x19, #616]
	cbz	x8, .LBB13_27
	add	x0, x19, #608
	bl	ebc_free_dma_buf
.LBB13_27:
	ldp	x20, x19, [sp, #32]
	ldr	x21, [sp, #16]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end13:
	.size	ebc_buffer_manage_uninit, .Lfunc_end13-ebc_buffer_manage_uninit

	.p2align	2
	.type	ebc_assist_thread,@function
ebc_assist_thread:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-96]!
	adrp	x8, global_ebc
	stp	x26, x25, [sp, #32]
	stp	x24, x23, [sp, #48]
	mov	w9, #18520
	stp	x22, x21, [sp, #64]
	adrp	x22, ebc_assist_finished_flag_sem
	ldr	x26, [x8, :lo12:global_ebc]
	stp	x20, x19, [sp, #80]
	adrp	x20, .L.str.106
	adrp	x23, .L.str.107
	stp	x28, x27, [sp, #16]
	mov	w25, #2
	add	x20, x20, :lo12:.L.str.106
	add	x27, x26, x9
	add	x21, x26, #304
	mov	w28, #1
	mov	w19, #552
	add	x22, x22, :lo12:ebc_assist_finished_flag_sem
	add	x23, x23, :lo12:.L.str.107
	mov	x29, sp
	//APP
	mrs	x24, SP_EL0
	//NO_APP
	b	.LBB14_3
.LBB14_1:
	ldrsw	x8, [x26, #184]
	mov	x1, x21
	mov	w2, #3
	add	x8, x26, x8, lsl #3
	ldr	x0, [x8, #1848]
	bl	get_full_or_part_data
.LBB14_2:
	mov	x0, x22
	bl	up
	bl	schedule
.LBB14_3:
	str	w25, [x24, #48]
	//APP
	dmb	ish
	//NO_APP
	ldrb	w8, [x27, #113]
	ldrb	w9, [x27, #114]
	orr	w8, w9, w8
	tst	w8, #0xff
	strb	w8, [x27, #115]
	b.ne	.LBB14_5
	ldrb	w8, [x26, #2086]
	cbz	w8, .LBB14_13
.LBB14_5:
	ldr	w8, [x27]
	cbz	w8, .LBB14_10
	cmp	w8, #1
	b.ne	.LBB14_2
	ldr	w8, [x26, #2096]
	cmp	w8, #2
	b.lt	.LBB14_2
	ldr	x0, [x21]
	mov	x1, x23
.LBB14_9:
	mov	x2, x20
	bl	_dev_info
	b	.LBB14_2
.LBB14_10:
	ldr	w8, [x26, #1880]
	cmp	w8, #9
	b.hi	.LBB14_1
	lsl	w8, w28, w8
	tst	w8, w19
	b.eq	.LBB14_1
	ldrsw	x8, [x26, #184]
	mov	x1, x21
	mov	w2, #3
	add	x8, x26, x8, lsl #3
	ldr	x0, [x8, #1848]
	bl	get_auto_data
	b	.LBB14_2
.LBB14_13:
	ldrb	w8, [x27, #81]
	cbz	w8, .LBB14_18
	ldrb	w8, [x27, #108]
	cbz	w8, .LBB14_17
	ldrb	w8, [x27, #109]
	cbz	w8, .LBB14_17
	strb	w28, [x27, #110]
	b	.LBB14_19
.LBB14_17:
	ldrb	w8, [x27, #110]
	cbnz	w8, .LBB14_19
	b	.LBB14_5
.LBB14_18:
	ldr	w8, [x27, #16]
	cmp	w8, #0
	b.gt	.LBB14_5
.LBB14_19:
	ldrb	w8, [x26, #2084]
	cbnz	w8, .LBB14_5
	ldrb	w8, [x26, #2085]
	cbnz	w8, .LBB14_5
	ldr	w8, [x26, #2096]
	cmp	w8, #2
	b.lt	.LBB14_2
	adrp	x1, .L.str.105
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.105
	b	.LBB14_9
.Lfunc_end14:
	.size	ebc_assist_thread, .Lfunc_end14-ebc_assist_thread

	.p2align	2
	.type	ebc_thread,@function
ebc_thread:
	sub	sp, sp, #240
	str	x30, [x18], #8
	adrp	x8, global_ebc
	stp	x20, x19, [sp, #224]
	stp	x29, x30, [sp, #144]
	add	x29, sp, #144
	stp	x24, x23, [sp, #192]
	ldr	x19, [x8, :lo12:global_ebc]
	mov	w8, #18512
	stp	x28, x27, [sp, #160]
	stp	x26, x25, [sp, #176]
	add	x24, x19, x8
	add	x8, x19, #304
	stp	x22, x21, [sp, #208]
	stur	x8, [x29, #-48]
	bl	ebc_empty_osd_buf_get
	add	x8, x19, #416
	mov	w23, wzr
	add	x22, x19, #2096
	mov	w20, #1
	mov	w25, #2
	mov	w28, #1
	str	x8, [sp, #8]
	//APP
	mrs	x8, SP_EL0
	//NO_APP
	str	x8, [sp, #64]
	mov	w8, #1
	stur	wzr, [x29, #-68]
	str	wzr, [sp, #40]
	str	xzr, [sp, #32]
	str	x0, [x19, #1784]
	stp	wzr, w8, [sp, #48]
	stp	x24, x22, [x29, #-64]
	b	.LBB15_4
.LBB15_1:
	ldrsw	x8, [x19, #184]
	mov	w2, #2
	ldur	x1, [x29, #-48]
	add	x8, x19, x8, lsl #3
	ldr	x0, [x8, #1848]
	bl	get_full_or_part_data
.LBB15_2:
	ldr	x8, [sp, #64]
	str	wzr, [x8, #48]
	//APP
	dmb	ish
	//NO_APP
.LBB15_3:
	bl	schedule
	mov	w8, #1
	stur	w8, [x29, #-68]
.LBB15_4:
	adrp	x0, ebc_assist_finished_flag_sem
	add	x0, x0, :lo12:ebc_assist_finished_flag_sem
	bl	down
	tbz	w23, #0, .LBB15_6
	ldrsw	x8, [x19, #184]
	add	x10, x19, #1848
	ldrsw	x2, [x19, #508]
	sub	w9, w20, w8
	ldr	x1, [x10, x8, lsl #3]
	ldr	x0, [x10, w9, sxtw #3]
	bl	memcpy
.LBB15_6:
	ldur	w8, [x29, #-68]
	tbz	w8, #0, .LBB15_39
	ldur	x0, [x29, #-48]
	mov	w1, #128
	bl	ebc_dump_debug_buffers
	ldr	w8, [x19, #520]
	ldrb	w9, [x19, #1872]
	add	w8, w8, #1
	str	w8, [x19, #520]
	cbz	w9, .LBB15_13
	sub	x0, x29, #40
	mov	w1, wzr
	stp	xzr, xzr, [x29, #-16]
	stp	xzr, xzr, [x29, #-32]
	stur	xzr, [x29, #-40]
	bl	init_wait_entry
.LBB15_9:
	adrp	x0, ebc_flip_wq
	sub	x1, x29, #40
	add	x0, x0, :lo12:ebc_flip_wq
	mov	w2, #1
	bl	prepare_to_wait_event
	ldrb	w8, [x19, #1872]
	cbz	w8, .LBB15_12
	cbnz	x0, .LBB15_13
	bl	schedule
	b	.LBB15_9
.LBB15_12:
	adrp	x0, ebc_flip_wq
	sub	x1, x29, #40
	add	x0, x0, :lo12:ebc_flip_wq
	bl	finish_wait
.LBB15_13:
	ldr	w8, [x22]
	cmp	w8, #2
	b.ge	.LBB15_107
.LBB15_14:
	bl	ebc_power_on
	ldr	w8, [x24, #8]
	cmp	w8, #1
	b.ne	.LBB15_25
	adrp	x8, global_ebc
	ldr	w27, [x19, #1880]
	ldp	w9, w10, [x19, #188]
	ldr	x0, [x19, #1624]
	strb	w20, [x19, #1872]
	ldr	x23, [x8, :lo12:global_ebc]
	ldr	w8, [x19, #2096]
	ldr	x21, [x19, #1696]
	ldr	x26, [x19, #1616]
	ldr	x25, [x19, #1688]
	cmp	w8, #2
	b.ge	.LBB15_412
	ldrb	w8, [x24]
	cbz	w8, .LBB15_33
.LBB15_17:
	strb	wzr, [x24]
	ldr	w24, [x23, #1808]
	str	x0, [sp, #56]
	cmp	w24, #64
	b.gt	.LBB15_424
	lsl	w8, w24, #8
	add	x22, x23, #2128
	sxtw	x2, w8
	mov	x0, x22
	mov	w1, wzr
	str	w27, [sp, #28]
	mov	w27, w28
	mov	x28, x21
	ldr	x21, [x23, #1832]
	str	w10, [sp, #44]
	str	w9, [sp, #16]
	bl	memset
	mov	x8, xzr
	b	.LBB15_20
.LBB15_19:
	add	x8, x8, #1
	add	x21, x21, #1
	cmp	x8, #32
	b.eq	.LBB15_31
.LBB15_20:
	mov	x9, xzr
	lsl	x10, x8, #1
	mov	x11, x21
	mov	x12, x22
	b	.LBB15_22
.LBB15_21:
	add	x9, x9, #1
	add	x12, x12, #8
	add	x11, x11, #32
	cmp	x9, #32
	b.eq	.LBB15_19
.LBB15_22:
	cmp	w24, #1
	b.lt	.LBB15_21
	mov	x13, x11
	mov	x14, x12
	mov	x15, x24
.LBB15_24:
	ldrb	w16, [x13]
	subs	x15, x15, #1
	ldr	x17, [x14]
	add	x13, x13, #1024
	lsl	x16, x16, x10
	orr	x16, x16, x17
	str	x16, [x14]
	add	x14, x14, #256
	b.ne	.LBB15_24
	b	.LBB15_21
.LBB15_25:
	ldr	w8, [x19, #2096]
	mov	w12, #24
	ldr	w21, [x19, #184]
	mov	w27, w28
	strb	w20, [x19, #1872]
	cmp	w8, #2
	b.ge	.LBB15_413
	mov	w8, w21
.LBB15_27:
	ldp	w22, w23, [x19, #244]
	sub	w9, w20, w8
	ldr	w8, [x19, #280]
	sxtw	x25, w21
	ldr	w24, [x19, #252]
	str	w9, [x19, #184]
	cbz	w8, .LBB15_29
	ldp	w9, w10, [x19, #188]
	ldr	w11, [x19, #276]
	madd	x26, x25, x12, x19
	mul	w12, w9, w8
	sub	w10, w10, w11
	add	w8, w11, w8
	mul	w11, w11, w9
	mul	w10, w10, w9
	lsr	w21, w12, #2
	mul	w8, w8, w9
	ldr	x9, [x26, #136]
	lsr	w28, w11, #2
	lsr	w2, w10, #2
	lsr	w8, w8, #2
	add	x0, x9, x8
	add	x1, x9, x28
	bl	memmove
	ldr	x8, [x26, #136]
	mov	w1, wzr
	mov	x2, x21
	add	x0, x8, x28
	bl	memset
	mov	w12, #24
.LBB15_29:
	mov	x26, x19
	mov	w3, #1
	add	w8, w23, w22
	add	w21, w8, w24
	ldr	x0, [x26, #112]!
	madd	x25, x25, x12, x26
	ldrsw	x2, [x26, #396]
	ldr	x1, [x25, #32]
	bl	dma_sync_single_for_device
	ldr	x0, [x26, #8]
	mov	w1, wzr
	mov	w2, wzr
	mov	w3, wzr
	mov	w4, wzr
	ldr	x8, [x0, #184]
	blr	x8
	ldr	x0, [x26, #8]
	mov	w2, wzr
	ldr	w1, [x25, #32]
	ldr	x8, [x0, #192]
	blr	x8
	ldr	x0, [x26, #8]
	mov	w1, w21
	mov	w2, wzr
	ldr	x8, [x0, #240]
	blr	x8
	ldr	x0, [x26, #8]
	mov	w1, #1
	ldr	x8, [x0, #224]
	blr	x8
	ldur	x24, [x29, #-64]
	ldrb	w8, [x24, #89]
	cbz	w8, .LBB15_35
	ldr	w8, [x24, #104]
	mov	w25, #2
	ldur	x22, [x29, #-56]
	mov	w28, w27
	add	w8, w8, #1
	str	w8, [x24, #104]
	b	.LBB15_39
.LBB15_31:
	ldr	x0, [x23, #8]
	mov	x1, x22
	mov	w2, w24
	mov	w3, #1
	ldr	x8, [x0, #208]
	blr	x8
	ldur	x24, [x29, #-64]
	mov	x21, x28
	mov	w28, w27
	ldr	w27, [sp, #28]
	ldr	x0, [sp, #56]
	mov	w22, wzr
	ldr	w9, [sp, #16]
	ldr	w10, [sp, #44]
.LBB15_32:
	ldur	x8, [x29, #-56]
	ldr	w8, [x8]
	cmp	w8, #2
	b.ge	.LBB15_425
.LBB15_33:
	ldr	x8, [x19, #1760]
	mul	w22, w10, w9
	mov	x2, x22
	ldr	x1, [x8, #8]
	bl	memcpy
	ldr	x8, [x19, #1768]
	mov	x0, x21
	mov	x2, x22
	ldr	x1, [x8, #8]
	bl	memcpy
	ldp	w8, w9, [x19, #244]
	ldr	w10, [x19, #252]
	mov	x1, x26
	ldr	x0, [x19, #112]
	mov	x2, x22
	mov	w3, #1
	cmp	w27, #2
	add	w8, w9, w8
	cset	w21, eq
	add	w23, w8, w10
	bl	dma_sync_single_for_device
	ldr	x0, [x19, #112]
	mov	x1, x25
	mov	x2, x22
	mov	w3, #1
	bl	dma_sync_single_for_device
	ldr	x0, [x19, #120]
	mov	w1, #4
	ldr	x8, [x0, #216]
	blr	x8
	ldr	x0, [x19, #120]
	mov	w1, w21
	mov	w2, #1
	mov	w3, wzr
	mov	w4, wzr
	ldr	x8, [x0, #184]
	blr	x8
	ldr	x0, [x19, #120]
	mov	w1, w26
	mov	w2, w25
	ldr	x8, [x0, #192]
	blr	x8
	ldr	x0, [x19, #120]
	mov	w1, w23
	mov	w2, wzr
	ldr	x8, [x0, #240]
	blr	x8
	ldr	x0, [x19, #120]
	ldr	w1, [x24, #12]
	ldr	x8, [x0, #224]
	blr	x8
	ldrb	w8, [x24, #89]
	cbz	w8, .LBB15_37
	mov	w8, #257
	strb	w20, [x24, #118]
	strh	w8, [x24, #116]
	b	.LBB15_38
.LBB15_35:
	ldr	w8, [x24, #24]
	mov	w25, #2
	ldur	x22, [x29, #-56]
	mov	w28, w27
	tbnz	w8, #31, .LBB15_39
	sub	w8, w8, #1
	str	w8, [x24, #24]
	b	.LBB15_39
.LBB15_37:
	mov	w8, #-1
	str	w8, [x24, #24]
.LBB15_38:
	ldur	x22, [x29, #-56]
	mov	w25, #2
.LBB15_39:
	ldrb	w8, [x19, #1880]
	tbz	w8, #3, .LBB15_60
	ldr	w8, [x24, #72]
	cbnz	w8, .LBB15_60
	ldrb	w8, [x24, #64]
	tbz	w8, #0, .LBB15_60
	adrp	x8, global_ebc
	ldr	x10, [x8, :lo12:global_ebc]
	ldr	w8, [x10, #192]
	cmp	w8, #1
	b.lt	.LBB15_55
	ldr	w10, [x10, #188]
	mov	w14, wzr
	mov	w9, wzr
	ldr	x11, [x19, #1552]
	ldr	x12, [x19, #1480]
	sub	w16, w10, #1
	ldr	x13, [x19, #1408]
	ldr	x15, [x19, #1336]
	add	x16, x16, #1
	ldr	x17, [x19, #1264]
	b	.LBB15_46
.LBB15_44:
	add	x17, x17, x16
.LBB15_45:
	add	w9, w9, #1
	cmp	w9, w8
	b.eq	.LBB15_56
.LBB15_46:
	cmp	w10, #1
	b.lt	.LBB15_45
	mov	w0, w10
	mov	x1, x17
	b	.LBB15_51
.LBB15_48:
	mov	w3, #248
.LBB15_49:
	and	w14, w2, #0xfc
	strb	w14, [x13]
	mov	w14, #4
	strb	wzr, [x12]
	strb	w3, [x15]
	strb	w25, [x11]
.LBB15_50:
	add	x1, x1, #1
	add	x15, x15, #1
	add	x13, x13, #1
	add	x12, x12, #1
	add	x11, x11, #1
	subs	w0, w0, #1
	b.eq	.LBB15_44
.LBB15_51:
	ldrb	w2, [x1]
	ldrb	w3, [x15]
	cmp	w2, w3
	b.ne	.LBB15_50
	ldrb	w2, [x13]
	tbnz	w2, #0, .LBB15_48
	tbz	w2, #1, .LBB15_50
	mov	w3, #232
	b	.LBB15_49
.LBB15_55:
	mov	w14, wzr
.LBB15_56:
	ldrb	w8, [x24, #122]
	ldrb	w9, [x24, #121]
	orr	w10, w8, w14
	orr	w9, w9, w14
	strb	w10, [x24, #122]
	strb	w9, [x24, #121]
	cbz	w14, .LBB15_59
	ldr	w8, [x22]
	cmp	w8, #0
	b.le	.LBB15_60
	adrp	x1, .L.str.151
	add	x1, x1, :lo12:.L.str.151
	b	.LBB15_430
.LBB15_59:
	orr	w8, w9, w8
	adrp	x10, .L.str.154
	cmp	w8, #0
	adrp	x8, .L.str.153
	ldr	w9, [x22]
	add	x8, x8, :lo12:.L.str.153
	add	x10, x10, :lo12:.L.str.154
	csel	x1, x10, x8, eq
	cmp	w9, #1
	b.ge	.LBB15_430
.LBB15_60:
	ldr	x8, [sp, #64]
	str	w25, [x8, #48]
	//APP
	dmb	ish
	//NO_APP
	ldrb	w8, [x24, #121]
	ldrb	w9, [x24, #122]
	orr	w8, w9, w8
	cbz	w8, .LBB15_66
	str	wzr, [sp, #56]
	ldrb	w9, [x24, #89]
	cbz	w9, .LBB15_67
.LBB15_62:
	ldrb	w10, [x24, #116]
	cbz	w10, .LBB15_65
	ldrb	w10, [x24, #117]
	cbz	w10, .LBB15_65
	strb	w20, [x24, #118]
.LBB15_65:
	ldrb	w10, [x24, #118]
	cmp	w10, #0
	cset	w27, ne
	strb	w8, [x24, #123]
	cbnz	w8, .LBB15_69
	b	.LBB15_68
.LBB15_66:
	ldrb	w9, [x19, #2086]
	cmp	w9, #0
	cset	w9, eq
	str	w9, [sp, #56]
	ldrb	w9, [x24, #89]
	cbnz	w9, .LBB15_62
.LBB15_67:
	ldr	w10, [x24, #24]
	cmp	w10, #1
	cset	w27, lt
	strb	w8, [x24, #123]
	cbnz	w8, .LBB15_69
.LBB15_68:
	ldrb	w10, [x19, #2086]
	cbz	w10, .LBB15_87
.LBB15_69:
	mov	w23, wzr
.LBB15_70:
	ldr	w9, [x22]
	cmp	w9, #2
	b.ge	.LBB15_92
.LBB15_71:
	bl	ebc_get_osd_list_enum_num
	cmp	w0, #0
	mov	w26, w0
	cset	w21, ne
	strb	w21, [x19, #2085]
	bl	ebc_get_dsp_list_enum_num
	mov	w25, w0
	strb	wzr, [x19, #2084]
	cbz	w0, .LBB15_79
	str	w27, [sp, #44]
	mov	w27, w28
	tbnz	w23, #0, .LBB15_74
	ldrb	w8, [x19, #1880]
	tbz	w8, #3, .LBB15_116
.LBB15_74:
	mov	w28, w23
	bl	ebc_dsp_buf_get
	cbz	x0, .LBB15_80
	ldrb	w8, [x24, #119]
	mov	x11, x24
	ldrb	w9, [x24, #89]
	adrp	x10, global_ebc
	ldr	w24, [x24, #40]
	mov	x23, x0
	cmp	w8, #0
	ldrb	w11, [x11, #101]
	cset	w8, ne
	ldr	x10, [x10, :lo12:global_ebc]
	ubfiz	w9, w9, #1, #7
	cmp	w24, #0
	bfi	w21, w8, #1, #1
	orr	w8, w9, w11
	ldr	w10, [x10, #264]
	str	w8, [sp, #48]
	cset	w8, ne
	cmp	w10, #230
	str	w8, [sp, #32]
	b.ne	.LBB15_83
	mov	w6, #2
.LBB15_77:
	mov	w22, #4
.LBB15_78:
	str	w6, [x23, #32]
	b	.LBB15_94
.LBB15_79:
	mov	w27, w28
	mov	w28, w23
.LBB15_80:
	mov	w23, w28
	tbz	w28, #0, .LBB15_116
.LBB15_81:
	mov	w8, #1
	mov	w28, #1
	str	wzr, [x24, #68]
	str	w8, [sp, #52]
.LBB15_82:
	str	wzr, [x24, #72]
	b	.LBB15_119
.LBB15_83:
	ldr	w6, [x23, #32]
	cmp	w21, #0
	cset	w8, ne
	cmp	w6, #21
	b.hi	.LBB15_426
	ldr	w9, [sp, #56]
	mov	w10, #124
	movk	w10, #34, lsl #16
	eor	w9, w9, #0x1
	orr	w8, w9, w8
	lsl	w9, w20, w6
	tst	w9, w10
	b.ne	.LBB15_93
	mov	w10, #36736
	tst	w9, w10
	b.eq	.LBB15_102
.LBB15_86:
	orr	w22, w8, #0x2
	b	.LBB15_94
.LBB15_87:
	cbz	w9, .LBB15_101
	ldrb	w9, [x24, #116]
	cbz	w9, .LBB15_91
	ldrb	w9, [x24, #117]
	cbz	w9, .LBB15_91
	strb	w20, [x24, #118]
.LBB15_91:
	ldrb	w9, [x24, #118]
	cmp	w9, #0
	cset	w23, ne
	b	.LBB15_70
.LBB15_92:
	adrp	x9, .L.str.113
	adrp	x10, .L.str.112
	ldr	w11, [sp, #56]
	cmp	w23, #0
	add	x9, x9, :lo12:.L.str.113
	add	x10, x10, :lo12:.L.str.112
	csel	x3, x10, x9, ne
	cmp	w27, #0
	csel	x4, x10, x9, ne
	cmp	w11, #0
	csel	x5, x10, x9, ne
	tst	w8, #0x4
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.119
	adrp	x2, .L.str.118
	csel	x6, x10, x9, eq
	add	x1, x1, :lo12:.L.str.119
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_71
.LBB15_93:
	orr	w22, w8, #0x4
.LBB15_94:
	ldur	x8, [x29, #-56]
	ldr	w8, [x8]
	cmp	w8, #2
	b.ge	.LBB15_108
	tbnz	w22, #3, .LBB15_109
.LBB15_96:
	ldrb	w8, [x19, #1880]
	ldur	x24, [x29, #-64]
	tbnz	w8, #3, .LBB15_98
	ldr	w8, [x23, #32]
	ldr	w9, [sp, #40]
	ldur	x22, [x29, #-56]
	cmp	w9, w8
	b.eq	.LBB15_111
	b	.LBB15_113
.LBB15_98:
	ldur	x22, [x29, #-56]
	cbz	w28, .LBB15_100
	mov	w23, w28
	ldr	w8, [x24, #76]
	cbz	w8, .LBB15_114
.LBB15_100:
	mov	w8, #2
	str	w21, [sp, #36]
	str	w8, [x24, #76]
	b	.LBB15_80
.LBB15_101:
	ldr	w9, [x24, #24]
	cmp	w9, #1
	cset	w23, lt
	b	.LBB15_70
.LBB15_102:
	mov	w10, #28672
	movk	w10, #1, lsl #16
	tst	w9, w10
	b.eq	.LBB15_426
	cbz	w24, .LBB15_86
	ldr	w9, [sp, #48]
	cbnz	w9, .LBB15_86
	cbz	w21, .LBB15_429
	mov	w6, #14
	mov	w22, #9
	b	.LBB15_78
.LBB15_107:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.117
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.117
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_14
.LBB15_108:
	ldur	x8, [x29, #-48]
	cmp	w24, #0
	adrp	x1, .L.str.120
	adrp	x2, .L.str.118
	cset	w3, ne
	add	x1, x1, :lo12:.L.str.120
	ldr	x0, [x8]
	add	x2, x2, :lo12:.L.str.118
	ldr	w4, [sp, #48]
	mov	w5, w21
	mov	w7, w22
	bl	_dev_info
	tbz	w22, #3, .LBB15_96
.LBB15_109:
	ldp	x24, x22, [x29, #-64]
	ldr	w8, [x23, #32]
	ldr	w9, [sp, #40]
	cmp	w9, w8
	b.ne	.LBB15_113
	ldrb	w8, [x19, #1880]
	tbnz	w8, #3, .LBB15_215
.LBB15_111:
	ldr	w8, [sp, #44]
	cbz	w8, .LBB15_216
.LBB15_112:
	str	w21, [sp, #36]
	strb	w20, [x19, #2084]
	b	.LBB15_80
.LBB15_113:
	mov	w23, w28
	tbz	w28, #0, .LBB15_115
.LBB15_114:
	str	w21, [sp, #36]
	strb	w20, [x19, #2084]
	b	.LBB15_81
.LBB15_115:
	str	w21, [sp, #36]
.LBB15_116:
	ldp	w8, w10, [x24, #68]
	ldr	w9, [x24, #56]
	add	w11, w8, #1
	add	w10, w10, #1
	cmp	w11, w9
	ldr	w9, [x24, #12]
	csinc	w8, wzr, w8, ge
	cmp	w10, w9
	stp	w8, w10, [x24, #68]
	b.ge	.LBB15_118
	mov	w28, w27
	b	.LBB15_119
.LBB15_118:
	ldrb	w8, [x19, #2084]
	mov	w28, w27
	cbz	w8, .LBB15_82
.LBB15_119:
	ldr	w8, [x22]
	cmp	w8, #2
	b.ge	.LBB15_166
.LBB15_120:
	ldrb	w8, [x24, #121]
	mov	w25, #2
	ldrb	w9, [x24, #122]
	orr	w8, w9, w8
	tst	w8, #0xff
	strb	w8, [x24, #123]
	b.ne	.LBB15_122
	ldrb	w8, [x19, #2086]
	cbz	w8, .LBB15_144
.LBB15_122:
	bl	ebc_power_on
	ldrb	w8, [x19, #2085]
	cbz	w8, .LBB15_214
	ldr	w8, [x19, #516]
	ldr	w27, [x19, #512]
	add	w9, w8, #7
	cmp	w8, #0
	csel	w21, w9, w8, lt
	tbz	w28, #0, .LBB15_125
	mov	x0, x19
	mov	w1, #1
	mov	w2, #10
	bl	ebc_lut_update
	ldr	w8, [x19, #1812]
	str	w8, [x24, #20]
.LBB15_125:
	bl	ebc_get_osd_list_enum_num
	cbz	w0, .LBB15_137
	mov	w28, w21
	mov	w21, w23
	mov	x26, xzr
	mov	w22, #-1
	mov	w23, #-1
	mov	w24, #-1
	mov	w25, #-1
	b	.LBB15_129
.LBB15_127:
	cmp	w8, w22
	csel	w22, w8, w22, gt
.LBB15_128:
	bl	ebc_get_osd_list_enum_num
	cbz	w0, .LBB15_134
.LBB15_129:
	cbz	x26, .LBB15_131
	mov	x0, x26
	bl	ebc_buf_release
.LBB15_131:
	bl	ebc_osd_buf_pop_get
	mov	x26, x0
	cbz	x0, .LBB15_128
	ldp	w8, w9, [x26, #40]
	cmp	w8, w25
	csel	w11, w8, w25, lt
	cmp	w25, #0
	csel	w25, w11, w8, ge
	ldp	w10, w8, [x26, #48]
	cmp	w10, w24
	csel	w11, w10, w24, gt
	cmp	w24, #0
	csel	w24, w11, w10, ge
	cmp	w9, w23
	csel	w10, w9, w23, lt
	cmp	w23, #0
	csel	w23, w10, w9, ge
	tbz	w22, #31, .LBB15_127
	mov	w22, w8
	b	.LBB15_128
.LBB15_134:
	cbz	x26, .LBB15_136
	stp	w25, w23, [x26, #40]
	stp	w24, w22, [x26, #48]
.LBB15_136:
	ldp	x24, x22, [x29, #-64]
	mov	w25, #2
	mov	w23, w21
	mov	w21, w28
	b	.LBB15_138
.LBB15_137:
	mov	x26, xzr
.LBB15_138:
	ldrb	w8, [x19, #1880]
	mov	w9, #9
	tst	w8, w9
	b.ne	.LBB15_183
	tbz	w23, #0, .LBB15_148
	ldr	w22, [x19, #504]
	mov	w23, w21
	ldr	x8, [x19, #1768]
	ldr	x0, [x19, #1336]
	sxtw	x2, w22
	add	w9, w22, #7
	ldr	x1, [x8, #8]
	cmp	w22, #0
	csel	w21, w9, w22, lt
	bl	memcpy
	ldrb	w8, [x19, #1880]
	tbnz	w8, #3, .LBB15_167
	cmp	w22, #8
	ldur	x22, [x29, #-56]
	b.lt	.LBB15_170
	ldr	x10, [x19, #1768]
	asr	w9, w21, #3
	ldr	x8, [x19, #1264]
	ldr	x10, [x10, #8]
.LBB15_143:
	ldr	x11, [x10], #8
	subs	w9, w9, #1
	and	x11, x11, #0xf8f8f8f8f8f8f8f8
	str	x11, [x8], #8
	b.ne	.LBB15_143
	b	.LBB15_170
.LBB15_144:
	ldrb	w8, [x24, #89]
	cbz	w8, .LBB15_173
	ldrb	w8, [x24, #116]
	cbz	w8, .LBB15_172
	ldrb	w8, [x24, #117]
	cbz	w8, .LBB15_172
	strb	w20, [x24, #118]
	b	.LBB15_174
.LBB15_148:
	ldr	w13, [x19, #516]
	mov	w28, w21
	ldr	x8, [x19, #1760]
	ldr	x9, [x19, #1768]
	add	w10, w13, #7
	ldr	w11, [x19, #2096]
	cmp	w13, #0
	ldr	w12, [x19, #512]
	ldr	x21, [x19, #1336]
	csel	w23, w10, w13, lt
	ldr	x22, [x19, #1264]
	cmp	w11, #1
	ldr	x24, [x8, #8]
	str	w13, [sp, #28]
	ldr	x25, [x9, #8]
	str	w12, [sp, #44]
	b.gt	.LBB15_428
.LBB15_149:
	asr	w8, w23, #3
	mul	w23, w8, w12
	str	x8, [sp, #16]
	cmp	w23, #1
	b.lt	.LBB15_152
	mov	w8, w23
.LBB15_151:
	ldr	x9, [x25], #8
	subs	w8, w8, #1
	and	x10, x9, #0xf8f8f8f8f8f8f8f8
	and	x9, x9, #0x404040404040404
	str	x10, [x22], #8
	ldr	x10, [x24], #8
	orr	x9, x10, x9
	str	x9, [x21], #8
	b.ne	.LBB15_151
.LBB15_152:
	ldr	x0, [x19, #1552]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldur	x24, [x29, #-64]
	ldrb	w8, [x24, #89]
	cbz	w8, .LBB15_177
	ldr	w8, [x24, #104]
	mov	w25, #2
	ldur	x22, [x29, #-56]
	mov	w21, w28
	strb	wzr, [x24, #89]
	cbz	w8, .LBB15_397
	ldr	w16, [sp, #44]
	mov	x15, #72340172838076673
	ldr	w17, [sp, #28]
	ldr	x0, [sp, #16]
	cmp	w16, #1
	b.lt	.LBB15_178
	mov	w8, wzr
	ldr	x9, [x19, #1480]
	b	.LBB15_158
.LBB15_156:
	add	x9, x9, x10
.LBB15_157:
	add	w8, w8, #1
	cmp	w8, w16
	b.eq	.LBB15_178
.LBB15_158:
	cmp	w17, #8
	b.lt	.LBB15_157
	mov	x10, xzr
	mov	x11, x0
	b	.LBB15_162
.LBB15_160:
	sub	w12, w12, w13
.LBB15_161:
	and	x12, x12, #0xff
	subs	x11, x11, #1
	mul	x12, x12, x15
	str	x12, [x9, x10]
	add	x10, x10, #8
	b.eq	.LBB15_156
.LBB15_162:
	ldr	x12, [x19, #1192]
	cbz	x12, .LBB15_165
	ldr	w13, [x19, #516]
	ldr	w14, [x24, #96]
	asr	w13, w13, #1
	cmp	w14, #10
	mul	w13, w13, w8
	add	x12, x12, w13, sxtw #1
	ldrh	w13, [x12, x10]
	ldr	w12, [x24, #104]
	b.ne	.LBB15_160
	ldr	w14, [x24, #92]
	sdiv	w13, w13, w14
	b	.LBB15_160
.LBB15_165:
	mov	w12, #-1
	b	.LBB15_161
.LBB15_166:
	ldrb	w8, [x19, #2085]
	adrp	x10, .L.str.113
	adrp	x11, .L.str.112
	cmp	w26, #0
	ldrb	w9, [x19, #2084]
	add	x10, x10, :lo12:.L.str.113
	add	x11, x11, :lo12:.L.str.112
	adrp	x1, .L.str.121
	csel	x3, x10, x11, eq
	cmp	w8, #0
	csel	x4, x10, x11, eq
	cmp	w25, #0
	csel	x5, x10, x11, eq
	cmp	w9, #0
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	csel	x6, x10, x11, eq
	add	x1, x1, :lo12:.L.str.121
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB15_120
.LBB15_167:
	ldr	w8, [x19, #504]
	mov	w13, #240
	ldur	x22, [x29, #-56]
	cmp	w8, #1
	b.lt	.LBB15_170
	ldr	x10, [x19, #1760]
	mov	x8, xzr
	ldr	x9, [x19, #1264]
	ldr	x10, [x10, #8]
.LBB15_169:
	ldrb	w11, [x10, x8]
	and	w12, w11, #0xf0
	tst	w11, #0x8
	csel	w11, w12, w13, eq
	strb	w11, [x9, x8]
	add	x8, x8, #1
	ldr	w11, [x19, #504]
	cmp	w8, w11
	b.lt	.LBB15_169
.LBB15_170:
	ldr	x0, [x19, #1552]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	x0, [x19, #1480]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	w8, [x19, #2096]
	mov	w21, w23
	cmp	w8, #1
	b.le	.LBB15_183
	ldr	w8, [x19, #1880]
	adrp	x9, .L.str.171
	add	x9, x9, :lo12:.L.str.171
	adrp	x1, .L.str.169
	adrp	x2, .L.str.167
	ldr	x0, [x19, #304]
	tst	w8, #0x2
	adrp	x8, .L.str.170
	add	x8, x8, :lo12:.L.str.170
	add	x1, x1, :lo12:.L.str.169
	csel	x3, x9, x8, eq
	add	x2, x2, :lo12:.L.str.167
	bl	_dev_info
	b	.LBB15_183
.LBB15_172:
	ldrb	w8, [x24, #118]
	cbnz	w8, .LBB15_174
	b	.LBB15_122
.LBB15_173:
	ldr	w8, [x24, #24]
	cmp	w8, #0
	b.gt	.LBB15_122
.LBB15_174:
	ldrb	w8, [x19, #2084]
	cbnz	w8, .LBB15_122
	ldrb	w8, [x19, #2085]
	cbnz	w8, .LBB15_122
	mov	w23, wzr
	b	.LBB15_340
.LBB15_177:
	ldr	w8, [x24, #12]
	ldr	w9, [x24, #24]
	ldr	x0, [x19, #1480]
	ldrsw	x2, [x19, #504]
	sub	w1, w8, w9
	bl	memset
	ldur	x22, [x29, #-56]
	mov	w25, #2
	mov	w21, w28
.LBB15_178:
	ldrb	w8, [x19, #1880]
	tbz	w8, #2, .LBB15_183
.LBB15_179:
	cmp	w23, #1
	b.lt	.LBB15_182
	ldr	x8, [x19, #1264]
	ldr	x9, [x19, #1336]
.LBB15_181:
	ldr	x10, [x9]
	subs	w23, w23, #1
	orr	x10, x10, #0x404040404040404
	str	x10, [x9], #8
	ldr	x10, [x8]
	and	x10, x10, #0xfbfbfbfbfbfbfbfb
	str	x10, [x8], #8
	b.ne	.LBB15_181
.LBB15_182:
	mov	w8, #3
	str	w8, [x19, #1880]
.LBB15_183:
	ldr	w8, [x19, #1880]
	orr	w8, w8, #0x1
	str	w8, [x19, #1880]
	ldp	w9, w8, [x26, #40]
	ldp	w10, w13, [x26, #48]
	cmp	w9, #0
	add	w14, w9, #7
	add	w11, w10, #7
	csel	w12, w14, w9, lt
	cmp	w10, #0
	csel	w9, w11, w10, lt
	asr	w10, w21, #3
	asr	w9, w9, #3
	sub	w15, w10, #1
	add	w11, w9, #1
	cmp	w11, w10
	sub	w11, w27, #1
	csinc	w9, w15, w9, ge
	cmp	w13, w27
	csel	w10, w13, w11, lt
	cmp	w14, #15
	b.hs	.LBB15_207
	ldr	w13, [x19, #516]
	add	w14, w13, #7
	cmp	w13, #0
	csel	w14, w14, w13, lt
	ldrb	w13, [x24, #119]
	asr	w14, w14, #3
	cmp	w9, w15
	b.ne	.LBB15_208
	cbnz	w8, .LBB15_208
	cmp	w10, w11
	b.ne	.LBB15_208
	mov	w23, wzr
	mov	w11, wzr
	cmp	w8, w10
	b.gt	.LBB15_209
.LBB15_188:
	ldr	x16, [x19, #1768]
	mul	w13, w14, w8
	asr	w12, w12, #3
	ldr	x15, [x19, #1480]
	sxtw	x17, w13
	sxtw	x3, w14
	ldr	x0, [x19, #1552]
	sxtw	x13, w12
	ldr	x14, [x19, #1336]
	add	x17, x13, x17
	ldr	x16, [x16, #8]
	lsl	x2, x17, #3
	ldr	x1, [x19, #1264]
	add	x15, x15, x2
	ldr	x4, [x26, #8]
	add	x14, x14, x2
	add	x16, x16, x2
	add	x0, x0, x2
	add	x17, x1, x2
	mov	w21, wzr
	add	x1, x4, x2
	add	w2, w9, #1
	sxtw	x2, w2
	lsl	x3, x3, #3
	str	w23, [sp, #44]
	b	.LBB15_190
.LBB15_189:
	add	w8, w8, #1
	add	x14, x14, x3
	add	x15, x15, x3
	add	x16, x16, x3
	add	x17, x17, x3
	add	x0, x0, x3
	add	x1, x1, x3
	cmp	w8, w10
	b.gt	.LBB15_210
.LBB15_190:
	cmp	w12, w9
	b.gt	.LBB15_189
	mov	x4, x1
	mov	x5, x0
	mov	x6, x17
	mov	x7, x16
	mov	x23, x15
	mov	x24, x14
	mov	x25, x13
	b	.LBB15_193
.LBB15_192:
	add	x25, x25, #1
	add	x24, x24, #8
	add	x23, x23, #8
	add	x7, x7, #8
	add	x6, x6, #8
	add	x5, x5, #8
	add	x4, x4, #8
	cmp	x25, x2
	b.eq	.LBB15_189
.LBB15_193:
	mov	x30, xzr
	b	.LBB15_196
.LBB15_194:
	strb	w22, [x6, x30]
	strb	wzr, [x23, x30]
.LBB15_195:
	add	x30, x30, #1
	cmp	x30, #8
	b.eq	.LBB15_192
.LBB15_196:
	ldrb	w22, [x4, x30]
	cmp	w22, #255
	b.ne	.LBB15_200
	ldrb	w28, [x6, x30]
	ldrb	w22, [x7, x30]
	cmp	w28, w22
	b.eq	.LBB15_195
	ldrb	w27, [x23, x30]
	orr	w11, w11, #0x1
	cbz	w27, .LBB15_194
	ldrb	w27, [x24, x30]
	cmp	w28, w27
	b.eq	.LBB15_194
	b	.LBB15_206
.LBB15_200:
	ldrb	w28, [x6, x30]
	orr	w11, w11, #0x3
	cmp	w28, w22
	b.eq	.LBB15_195
	ldrb	w27, [x23, x30]
	cbz	w27, .LBB15_203
	ldrb	w27, [x24, x30]
	cmp	w28, w27
	b.ne	.LBB15_206
.LBB15_203:
	strb	w22, [x6, x30]
	strb	wzr, [x23, x30]
	ldrb	w22, [x6, x30]
	cmp	w22, #240
	b.eq	.LBB15_205
	cbnz	w22, .LBB15_195
.LBB15_205:
	strb	w20, [x5, x30]
	b	.LBB15_195
.LBB15_206:
	mov	w21, #1
	b	.LBB15_195
.LBB15_207:
	ldr	w11, [x19, #516]
	add	w13, w11, #7
	cmp	w11, #0
	csel	w11, w13, w11, lt
	ldrb	w13, [x24, #119]
	asr	w14, w11, #3
.LBB15_208:
	mov	w23, #1
	mov	w11, w13
	cmp	w8, w10
	b.le	.LBB15_188
.LBB15_209:
	mov	w21, wzr
	b	.LBB15_211
.LBB15_210:
	ldp	x24, x22, [x29, #-64]
	mov	w25, #2
	ldr	w23, [sp, #44]
	ldrb	w13, [x24, #119]
.LBB15_211:
	and	w8, w11, #0xff
	and	w9, w13, #0x2
	cmp	w8, #1
	ccmp	w9, #0, #0, eq
	ldr	w9, [x22]
	csel	w8, wzr, w11, eq
	cmp	w9, #2
	strb	w8, [x24, #119]
	b.ge	.LBB15_217
	mov	x0, x26
	tbz	w21, #0, .LBB15_218
.LBB15_213:
	bl	ebc_add_to_osd_buf_list_head
	mov	w28, wzr
.LBB15_214:
	mov	w23, wzr
	ldrb	w8, [x19, #2084]
	cbnz	w8, .LBB15_219
	b	.LBB15_340
.LBB15_215:
	ldr	w8, [x24, #68]
	cbz	w8, .LBB15_112
.LBB15_216:
	str	w21, [sp, #36]
	b	.LBB15_80
.LBB15_217:
	ldur	x9, [x29, #-48]
	adrp	x10, .L.str.112
	ldp	w3, w5, [x26, #40]
	tst	w21, #0x1
	add	x10, x10, :lo12:.L.str.112
	ldr	x0, [x9]
	adrp	x9, .L.str.113
	add	x9, x9, :lo12:.L.str.113
	adrp	x1, .L.str.172
	ldp	w4, w6, [x26, #48]
	adrp	x2, .L.str.152
	csel	x7, x9, x10, ne
	and	w8, w8, #0xff
	add	x1, x1, :lo12:.L.str.172
	add	x2, x2, :lo12:.L.str.152
	str	w8, [sp]
	bl	_dev_info
	mov	x0, x26
	tbnz	w21, #0, .LBB15_213
.LBB15_218:
	bl	ebc_buf_release
	mov	w28, wzr
	ldrb	w8, [x19, #2084]
	cbz	w8, .LBB15_340
.LBB15_219:
	bl	ebc_dsp_buf_pop_get
	cbz	x0, .LBB15_340
	ldr	w8, [x22]
	mov	x26, x0
	cmp	w8, #2
	b.ge	.LBB15_404
	ldr	x0, [x19, #1760]
	cbz	x0, .LBB15_224
.LBB15_222:
	ldr	x8, [x19, #1768]
	cmp	x0, x8
	b.eq	.LBB15_225
	bl	ebc_buf_release
.LBB15_224:
	ldr	x8, [x19, #1768]
.LBB15_225:
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	str	x8, [x19, #1760]
	str	x26, [x19, #1768]
	cbnz	x8, .LBB15_227
	str	x26, [x19, #1760]
.LBB15_227:
	adrp	x8, global_ebc
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	w8, [x8, #264]
	cmp	w8, #230
	b.ne	.LBB15_233
	mov	w21, #4
	str	w25, [x26, #32]
.LBB15_229:
	mov	w2, #3
.LBB15_230:
	ldr	w8, [sp, #52]
	str	w2, [x19, #1804]
	tbz	w8, #0, .LBB15_243
	ldp	w9, w10, [x24, #44]
	ldr	w8, [x24, #36]
	mov	x0, x19
	str	w8, [x24, #52]
	stp	w9, w10, [x24, #60]
	ldr	w1, [x26, #32]
	bl	ebc_lut_update
	ldr	w9, [x19, #1808]
	ldr	w10, [x19, #1816]
	ldr	w8, [x24, #40]
	stp	w9, w10, [x24, #12]
	tbnz	w8, #31, .LBB15_241
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	cbnz	w8, .LBB15_242
	b	.LBB15_243
.LBB15_233:
	ldr	w8, [sp, #36]
	ldr	w3, [x26, #32]
	tst	w8, #0xff
	cset	w8, ne
	cmp	w3, #21
	b.hi	.LBB15_416
	ldr	w9, [sp, #56]
	mov	w10, #124
	movk	w10, #34, lsl #16
	eor	w9, w9, #0x1
	orr	w8, w9, w8
	lsl	w9, w20, w3
	tst	w9, w10
	b.ne	.LBB15_237
	mov	w10, #36736
	tst	w9, w10
	b.eq	.LBB15_280
.LBB15_236:
	orr	w21, w8, #0x2
	b	.LBB15_238
.LBB15_237:
	orr	w21, w8, #0x4
.LBB15_238:
	sub	w8, w3, #2
	cmp	w8, #19
	b.hi	.LBB15_229
	adrp	x11, .LJTI15_0
	mov	w2, wzr
	add	x11, x11, :lo12:.LJTI15_0
.Ltmp17:
	adr	x9, .Ltmp17
	ldrsw	x10, [x11, x8, lsl #2]
	add	x9, x9, x10
	br	x9
.LBB15_240:
	mov	w2, #8
	b	.LBB15_230
.LBB15_241:
	adrp	x6, .LJTI15_1
	mov	w8, w9
	add	x6, x6, :lo12:.LJTI15_1
	mov	w7, #65535
.LBB15_242:
	str	w8, [x24, #56]
.LBB15_243:
	ldr	w9, [x26, #32]
	tst	w21, #0xfffffff9
	ldr	w8, [x19, #1880]
	str	w9, [sp, #40]
	mov	w9, #9
	and	w9, w8, w9
	b.eq	.LBB15_249
	cbnz	w9, .LBB15_252
	ldr	w24, [x19, #504]
	mov	w27, w23
	ldr	x8, [x19, #1760]
	ldr	x0, [x19, #1336]
	sxtw	x2, w24
	add	w9, w24, #7
	ldr	x1, [x8, #8]
	cmp	w24, #0
	csel	w9, w9, w24, lt
	asr	w23, w9, #3
	bl	memcpy
	tbnz	w21, #3, .LBB15_286
	cmp	w24, #8
	b.lt	.LBB15_289
	ldr	x9, [x19, #1768]
	mov	w10, w23
	ldr	x8, [x19, #1264]
	ldr	x9, [x9, #8]
.LBB15_248:
	ldr	x11, [x9], #8
	subs	w10, w10, #1
	and	x11, x11, #0xf8f8f8f8f8f8f8f8
	str	x11, [x8], #8
	b.ne	.LBB15_248
	b	.LBB15_289
.LBB15_249:
	cbz	w9, .LBB15_252
	tbnz	w8, #1, .LBB15_292
	tst	w8, #0x4
	adrp	x8, .L.str.175
	adrp	x10, .L.str.176
	ldr	w9, [x22]
	add	x8, x8, :lo12:.L.str.175
	add	x10, x10, :lo12:.L.str.176
	csel	x1, x10, x8, eq
	cmp	w9, #2
	b.lt	.LBB15_293
	b	.LBB15_435
.LBB15_252:
	tbz	w21, #3, .LBB15_256
	tbz	w8, #3, .LBB15_256
.LBB15_254:
	ldur	x0, [x29, #-48]
	bl	fast_update_to_auto_image
.LBB15_255:
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	str	w21, [x19, #1880]
	tbnz	w21, #3, .LBB15_294
	b	.LBB15_295
.LBB15_256:
	tbz	w21, #0, .LBB15_293
	tbz	w8, #0, .LBB15_293
	ldr	w8, [x19, #516]
	mov	w27, w23
	ldr	w9, [x19, #512]
	add	w10, w8, #7
	cmp	w8, #0
	csel	w13, w10, w8, lt
	cmp	w9, #1
	b.lt	.LBB15_407
	ldr	x12, [x19, #1768]
	asr	w13, w13, #3
	ldr	x14, [x19, #1784]
	sub	w16, w13, #1
	mov	w23, wzr
	mov	w10, wzr
	ldr	x11, [x19, #1480]
	add	x16, x16, #1
	ldr	x12, [x12, #8]
	ldr	x14, [x14, #8]
	ldr	x15, [x19, #1336]
	ldr	x17, [x19, #1264]
	b	.LBB15_262
.LBB15_260:
	add	x14, x14, x16, lsl #3
.LBB15_261:
	add	w10, w10, #1
	cmp	w10, w9
	b.eq	.LBB15_408
.LBB15_262:
	cmp	w8, #8
	b.lt	.LBB15_261
	mov	w0, wzr
	mov	x1, x14
	b	.LBB15_265
.LBB15_264:
	add	x17, x17, #8
	add	x15, x15, #8
	add	x1, x1, #8
	add	x12, x12, #8
	add	x11, x11, #8
	add	w0, w0, #1
	cmp	w0, w13
	b.eq	.LBB15_260
.LBB15_265:
	ldr	x4, [x17]
	ldr	x3, [x12]
	cmp	x4, x3
	b.ne	.LBB15_268
	tbz	w21, #2, .LBB15_264
	ldr	x2, [x15]
	orr	x2, x2, #0x404040404040404
	str	x2, [x15]
	b	.LBB15_264
.LBB15_268:
	mov	x2, xzr
	and	w3, w3, #0xff
	and	w4, w4, #0xff
	cmp	w4, w3
	b.eq	.LBB15_276
.LBB15_269:
	ldrb	w5, [x1, x2]
	cmp	w5, #255
	b.ne	.LBB15_277
	ldrb	w5, [x11, x2]
	cbz	w5, .LBB15_272
	ldrb	w5, [x15, x2]
	cmp	w4, w5
	b.ne	.LBB15_275
.LBB15_272:
	strb	w3, [x17, x2]
	tbz	w3, #2, .LBB15_274
	and	w3, w3, #0xfffffffb
	strb	w3, [x17, x2]
	ldrb	w3, [x15, x2]
	orr	w3, w3, #0x4
	strb	w3, [x15, x2]
.LBB15_274:
	strb	wzr, [x11, x2]
	b	.LBB15_277
.LBB15_275:
	mov	w23, #1
	cmp	x2, #7
	b.ne	.LBB15_278
	b	.LBB15_264
.LBB15_276:
	tbnz	w21, #2, .LBB15_279
.LBB15_277:
	cmp	x2, #7
	b.eq	.LBB15_264
.LBB15_278:
	add	x3, x17, x2
	add	x5, x12, x2
	add	x2, x2, #1
	ldrb	w4, [x3, #1]
	ldrb	w3, [x5, #1]
	cmp	w4, w3
	b.ne	.LBB15_269
	b	.LBB15_276
.LBB15_279:
	ldrb	w3, [x15, x2]
	orr	w3, w3, #0x4
	strb	w3, [x15, x2]
	cmp	x2, #7
	b.ne	.LBB15_278
	b	.LBB15_264
.LBB15_280:
	mov	w10, #28672
	movk	w10, #1, lsl #16
	tst	w9, w10
	b.eq	.LBB15_416
	ldr	w9, [sp, #32]
	tst	w9, #0xff
	b.eq	.LBB15_236
	ldr	w9, [sp, #48]
	tst	w9, #0xff
	b.ne	.LBB15_236
	ldr	w8, [sp, #36]
	mov	w21, #9
	tst	w8, #0xff
	b.eq	.LBB15_238
	mov	w8, #14
	str	w8, [x26, #32]
.LBB15_285:
	mov	w2, #1
	b	.LBB15_230
.LBB15_286:
	ldr	w8, [x19, #504]
	mov	w13, #240
	cmp	w8, #1
	b.lt	.LBB15_289
	ldr	x10, [x19, #1760]
	mov	x8, xzr
	ldr	x9, [x19, #1264]
	ldr	x10, [x10, #8]
.LBB15_288:
	ldrb	w11, [x10, x8]
	and	w12, w11, #0xf0
	tst	w11, #0x8
	csel	w11, w12, w13, eq
	strb	w11, [x9, x8]
	add	x8, x8, #1
	ldr	w11, [x19, #504]
	cmp	w8, w11
	b.lt	.LBB15_288
.LBB15_289:
	ldr	x0, [x19, #1552]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldr	x0, [x19, #1480]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	tbnz	w21, #2, .LBB15_298
	ldr	w8, [x22]
	ldur	x24, [x29, #-64]
	cmp	w8, #2
	b.ge	.LBB15_433
	mov	w23, w27
	tbz	w21, #3, .LBB15_255
	b	.LBB15_254
.LBB15_292:
	ldr	w8, [x22]
	cmp	w8, #1
	b.gt	.LBB15_434
.LBB15_293:
	str	w21, [x19, #1880]
	tbz	w21, #3, .LBB15_295
.LBB15_294:
	str	w20, [x24, #76]
.LBB15_295:
	tbz	w21, #3, .LBB15_308
	ldr	w8, [x22]
	cmp	w8, #1
	b.lt	.LBB15_309
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.123
	ldp	w4, w5, [x24, #12]
	adrp	x2, .L.str.124
	ldr	w3, [x26, #32]
	ldr	x0, [x8]
	add	x1, x1, :lo12:.L.str.123
	add	x2, x2, :lo12:.L.str.124
	bl	_dev_info
	b	.LBB15_415
.LBB15_298:
	adrp	x6, .LJTI15_1
	cmp	w24, #8
	ldur	x24, [x29, #-64]
	add	x6, x6, :lo12:.LJTI15_1
	mov	w7, #65535
	b.lt	.LBB15_306
	mov	w8, wzr
	ldr	x9, [x19, #1336]
	subs	w10, w23, #1
	b.eq	.LBB15_304
	add	x11, x9, w10, uxtw #3
	cmp	x11, x9
	b.lo	.LBB15_304
	add	x10, x10, #1
	and	x8, x10, #0x1fffffffe
	mov	x12, x8
	add	x11, x9, x8, lsl #3
	add	x9, x9, #8
.LBB15_302:
	ldp	x13, x14, [x9, #-8]
	subs	x12, x12, #2
	orr	x13, x13, #0x404040404040404
	orr	x14, x14, #0x404040404040404
	stp	x13, x14, [x9, #-8]
	add	x9, x9, #16
	b.ne	.LBB15_302
	mov	x9, x11
	cmp	x10, x8
	b.eq	.LBB15_306
.LBB15_304:
	sub	w8, w23, w8
.LBB15_305:
	ldr	x10, [x9]
	subs	w8, w8, #1
	orr	x10, x10, #0x404040404040404
	str	x10, [x9], #8
	b.ne	.LBB15_305
.LBB15_306:
	ldr	w8, [x22]
	cmp	w8, #2
	b.ge	.LBB15_436
.LBB15_307:
	mov	w8, #3
	mov	w23, w27
	str	w8, [x19, #1880]
.LBB15_308:
	ldr	w8, [x22]
	cmp	w8, #1
	b.ge	.LBB15_414
.LBB15_309:
	ldrb	w8, [x24, #101]
	ldrb	w9, [x19, #2116]
	strb	w8, [x24, #89]
	cbz	w9, .LBB15_338
	ldrb	w9, [x19, #1880]
	tbnz	w9, #0, .LBB15_338
	cbz	w8, .LBB15_339
	ldp	w25, w21, [x24, #108]
	mov	w27, w23
	mov	w8, wzr
	ldr	x23, [x19, #1192]
	str	wzr, [x24, #104]
	strh	wzr, [x24, #116]
	stp	w25, w21, [x24, #92]
	strb	wzr, [x24, #118]
	cbz	x23, .LBB15_431
	ldr	w24, [x19, #516]
	cmp	w24, #1
	b.lt	.LBB15_395
	ldr	w26, [x19, #512]
	cmp	w26, #1
	b.lt	.LBB15_395
	cmp	w25, #1
	b.lt	.LBB15_395
	ldr	w8, [x22]
	cmp	w8, #1
	b.ge	.LBB15_437
	cmp	w21, #10
	b.eq	.LBB15_438
.LBB15_318:
	and	x8, x26, #0xffff
	cbz	w8, .LBB15_438
	mul	w10, w25, w25
	mov	x9, xzr
	ubfx	w11, w24, #1, #16
	ubfx	x13, x26, #1, #16
	lsl	w12, w10, #2
	b	.LBB15_321
.LBB15_320:
	add	x9, x9, #1
	cmp	x9, x8
	b.eq	.LBB15_438
.LBB15_321:
	sub	x17, x9, x13
	cmp	x17, #0
	cinc	w14, w20, lt
	cmp	w17, #0
	cneg	w15, w17, mi
	cbz	w11, .LBB15_320
	asr	x1, x17, #63
	mul	x17, x17, x17
	mov	w16, wzr
	sub	w0, w26, w9
	and	w1, w1, #0x3
	b	.LBB15_326
.LBB15_323:
	sdiv	w2, w0, w25
.LBB15_324:
	cmp	w2, w7
	csel	w2, w2, w7, lt
.LBB15_325:
	add	w16, w16, #1
	cmp	w11, w16
	strh	w2, [x23], #2
	b.eq	.LBB15_320
.LBB15_326:
	lsl	w2, w16, #1
	sub	w3, w21, #1
	cmp	w3, #8
	b.hi	.LBB15_335
.Ltmp18:
	adr	x4, .Ltmp18
	ldrsw	x5, [x6, x3, lsl #2]
	add	x4, x4, x5
	br	x4
.LBB15_328:
	and	w2, w2, #0xffff
	sub	w2, w2, w11
	smaddl	x2, w2, w2, x17
	udiv	x2, x2, x10
	cmp	x2, x7
	csel	w2, w2, w7, lo
	b	.LBB15_325
.LBB15_329:
	and	w2, w2, #0xffff
	subs	w2, w2, w11
	cneg	w2, w2, mi
	add	w2, w2, w15
	b	.LBB15_332
.LBB15_330:
	and	w2, w2, #0xffff
	subs	w2, w2, w11
	cneg	w2, w2, mi
	cmp	w2, w15
	csel	w2, w2, w15, hi
	b	.LBB15_332
.LBB15_331:
	and	w2, w2, #0xffff
	subs	w2, w2, w11
	cneg	w2, w2, mi
	cmp	w2, w15
	csel	w2, w2, w15, lo
.LBB15_332:
	udiv	w2, w2, w25
	b	.LBB15_334
.LBB15_333:
	and	w2, w2, #0xffff
	sub	w2, w2, w11
	cmp	w2, #0
	smaddl	x3, w2, w2, x17
	csel	w2, w1, w14, ge
	orr	w2, w2, #0x4
	udiv	x3, x3, x12
	mul	w2, w2, w3
.LBB15_334:
	cmp	w2, w7
	csel	w2, w2, w7, lo
	b	.LBB15_325
.LBB15_335:
	and	w2, w2, #0xffff
	sdiv	w2, w2, w25
	b	.LBB15_325
.LBB15_336:
	sub	w2, w24, w2, uxth
	sdiv	w2, w2, w25
	b	.LBB15_324
.LBB15_337:
	sdiv	w2, w9, w25
	b	.LBB15_325
.LBB15_338:
	strb	wzr, [x24, #89]
.LBB15_339:
	ldr	w8, [x24, #12]
	str	wzr, [sp, #52]
	str	w8, [x24, #24]
.LBB15_340:
	ldr	w8, [x19, #1880]
	mov	w9, #9
	tst	w8, w9
	b.ne	.LBB15_345
	ldr	w9, [sp, #48]
	tst	w9, #0xff
	b.ne	.LBB15_345
	ldr	w9, [x19, #264]
	cmp	w9, #230
	b.eq	.LBB15_345
	ldrb	w9, [x24, #120]
	cbnz	w9, .LBB15_345
	ldr	w9, [x24, #4]
	cmp	w9, #1
	b.eq	.LBB15_346
.LBB15_345:
	mov	w9, wzr
.LBB15_346:
	str	w9, [x24, #8]
	tbz	w8, #3, .LBB15_349
	ldrb	w8, [x24, #121]
	ldrb	w9, [x24, #122]
	orr	w8, w9, w8
	tst	w8, #0xff
	strb	w8, [x24, #123]
	b.ne	.LBB15_349
	ldrb	w8, [x19, #2086]
	cbz	w8, .LBB15_383
.LBB15_349:
	ldr	w8, [x24, #76]
.LBB15_350:
	cmp	w8, #2
	b.ne	.LBB15_369
	bl	ebc_power_on
	adrp	x8, global_ebc
	ldr	x10, [x8, :lo12:global_ebc]
	ldr	w8, [x10, #192]
	cmp	w8, #1
	b.lt	.LBB15_364
	ldr	w10, [x10, #188]
	mov	w12, wzr
	mov	w9, wzr
	ldr	x11, [x19, #1552]
	ldr	x13, [x19, #1480]
	sub	w16, w10, #1
	ldr	x14, [x19, #1408]
	ldr	x15, [x19, #1336]
	add	x16, x16, #1
	ldr	x17, [x19, #1264]
	b	.LBB15_355
.LBB15_353:
	add	x17, x17, x16
.LBB15_354:
	add	w9, w9, #1
	cmp	w9, w8
	b.eq	.LBB15_365
.LBB15_355:
	cmp	w10, #1
	b.lt	.LBB15_354
	mov	w0, w10
	mov	x1, x17
	b	.LBB15_360
.LBB15_357:
	mov	w2, #248
.LBB15_358:
	mov	w12, #4
	strb	wzr, [x14]
	strb	wzr, [x13]
	strb	w2, [x15]
	strb	w25, [x11]
.LBB15_359:
	add	x1, x1, #1
	add	x15, x15, #1
	add	x14, x14, #1
	add	x13, x13, #1
	add	x11, x11, #1
	subs	w0, w0, #1
	b.eq	.LBB15_353
.LBB15_360:
	ldrb	w2, [x1]
	ldrb	w3, [x15]
	cmp	w2, w3
	b.ne	.LBB15_359
	ldrb	w2, [x14]
	tbnz	w2, #2, .LBB15_357
	tbz	w2, #3, .LBB15_359
	mov	w2, #232
	b	.LBB15_358
.LBB15_364:
	mov	w12, wzr
.LBB15_365:
	ldrb	w8, [x24, #122]
	str	wzr, [x24, #76]
	ldrb	w9, [x24, #121]
	orr	w10, w8, w12
	orr	w9, w9, w12
	strb	w10, [x24, #122]
	strb	w9, [x24, #121]
	cbz	w12, .LBB15_368
	ldr	w8, [x22]
	cmp	w8, #0
	b.le	.LBB15_369
	adrp	x1, .L.str.151
	add	x1, x1, :lo12:.L.str.151
	b	.LBB15_396
.LBB15_368:
	orr	w8, w9, w8
	adrp	x10, .L.str.154
	cmp	w8, #0
	adrp	x8, .L.str.153
	ldr	w9, [x22]
	add	x8, x8, :lo12:.L.str.153
	add	x10, x10, :lo12:.L.str.154
	csel	x1, x10, x8, eq
	cmp	w9, #1
	b.ge	.LBB15_396
.LBB15_369:
	ldur	x0, [x29, #-48]
	mov	w1, #-129
	bl	ebc_dump_debug_buffers
	adrp	x8, ebc_assist_task
	ldr	x0, [x8, :lo12:ebc_assist_task]
	bl	wake_up_process
	ldrb	w8, [x24, #121]
	ldrb	w9, [x24, #122]
	orr	w8, w9, w8
	tst	w8, #0xff
	strb	w8, [x24, #123]
	b.ne	.LBB15_371
	ldrb	w8, [x19, #2086]
	cbz	w8, .LBB15_379
.LBB15_371:
	ldr	w8, [x24, #8]
	cbz	w8, .LBB15_375
	cmp	w8, #1
	b.ne	.LBB15_393
	ldr	w8, [x22]
	cmp	w8, #2
	b.ge	.LBB15_405
.LBB15_374:
	ldr	x8, [sp, #64]
	str	wzr, [x8, #48]
	//APP
	dmb	ish
	//NO_APP
	b	.LBB15_3
.LBB15_375:
	ldr	w8, [x22]
	cmp	w8, #2
	b.ge	.LBB15_406
.LBB15_376:
	ldr	w8, [x19, #1880]
	cmp	w8, #9
	b.hi	.LBB15_1
	lsl	w8, w20, w8
	mov	w9, #552
	tst	w8, w9
	b.eq	.LBB15_1
	ldrsw	x8, [x19, #184]
	mov	w2, #2
	ldur	x1, [x29, #-48]
	add	x8, x19, x8, lsl #3
	ldr	x0, [x8, #1848]
	bl	get_auto_data
	b	.LBB15_2
.LBB15_379:
	ldrb	w8, [x24, #89]
	cbz	w8, .LBB15_388
	ldrb	w8, [x24, #116]
	cbz	w8, .LBB15_387
	ldrb	w8, [x24, #117]
	cbz	w8, .LBB15_387
	strb	w20, [x24, #118]
	b	.LBB15_389
.LBB15_383:
	ldrb	w8, [x24, #89]
	cbz	w8, .LBB15_399
	ldrb	w8, [x24, #116]
	cbz	w8, .LBB15_398
	ldrb	w8, [x24, #117]
	cbz	w8, .LBB15_398
	strb	w20, [x24, #118]
	b	.LBB15_400
.LBB15_387:
	ldrb	w8, [x24, #118]
	cbnz	w8, .LBB15_389
	b	.LBB15_371
.LBB15_388:
	ldr	w8, [x24, #24]
	cmp	w8, #0
	b.gt	.LBB15_371
.LBB15_389:
	ldrb	w8, [x19, #2084]
	cbnz	w8, .LBB15_371
	ldrb	w8, [x19, #2085]
	cbnz	w8, .LBB15_371
	ldr	w8, [x22]
	cmp	w8, #1
	b.ge	.LBB15_441
.LBB15_392:
	stur	wzr, [x29, #-68]
.LBB15_393:
	bl	schedule
	ldr	w8, [x22]
	cmp	w8, #1
	b.le	.LBB15_4
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.129
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.129
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_4
.LBB15_395:
	str	w8, [sp, #52]
	b	.LBB15_439
.LBB15_396:
	ldur	x8, [x29, #-48]
	adrp	x2, .L.str.152
	add	x2, x2, :lo12:.L.str.152
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_369
.LBB15_397:
	ldr	x0, [x19, #1480]
	mov	w1, wzr
	ldrsw	x2, [x19, #504]
	bl	memset
	ldrb	w8, [x19, #1880]
	tbnz	w8, #2, .LBB15_179
	b	.LBB15_183
.LBB15_398:
	ldrb	w8, [x24, #118]
	cbnz	w8, .LBB15_400
	b	.LBB15_349
.LBB15_399:
	ldr	w8, [x24, #24]
	cmp	w8, #0
	b.gt	.LBB15_349
.LBB15_400:
	ldrb	w8, [x19, #2084]
	cbnz	w8, .LBB15_349
	ldrb	w8, [x19, #2085]
	cbnz	w8, .LBB15_349
	ldr	w8, [x24, #76]
	cmp	w8, #1
	b.ne	.LBB15_350
	ldr	x1, [x19, #408]
	mov	w0, #8
	ldr	x2, [sp, #8]
	mov	w3, #3
	bl	queue_delayed_work_on
	b	.LBB15_349
.LBB15_404:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.122
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.122
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	ldr	x0, [x19, #1760]
	cbnz	x0, .LBB15_222
	b	.LBB15_224
.LBB15_405:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.127
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.127
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_374
.LBB15_406:
	adrp	x1, .L.str.128
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.128
	ldr	w3, [x19, #184]
	add	x2, x2, :lo12:.L.str.118
	ldr	w4, [x19, #520]
	ldr	w5, [x19, #1880]
	bl	_dev_info
	b	.LBB15_376
.LBB15_407:
	mov	w23, wzr
.LBB15_408:
	and	w8, w21, #0xfffffffb
	ldr	w9, [x22]
	orr	w8, w8, #0x2
	tst	w21, #0x4
	csel	w24, w21, w8, eq
	cmp	w9, #1
	b.gt	.LBB15_432
.LBB15_409:
	and	w8, w23, #0x1
	mov	w21, w24
	ldur	x24, [x29, #-64]
	mov	w23, w27
	strb	w8, [x19, #2086]
	str	w21, [x19, #1880]
	tbnz	w21, #3, .LBB15_294
	b	.LBB15_295
.LBB15_410:
	mov	w2, #6
	b	.LBB15_230
.LBB15_411:
	mov	w2, #7
	b	.LBB15_230
.LBB15_412:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.146
	adrp	x2, .L.str.147
	str	x0, [sp, #56]
	add	x1, x1, :lo12:.L.str.146
	add	x2, x2, :lo12:.L.str.147
	ldr	x0, [x8]
	mov	w22, w9
	str	w10, [sp, #44]
	bl	_dev_info
	ldr	w10, [sp, #44]
	mov	w9, w22
	ldr	x0, [sp, #56]
	ldrb	w8, [x24]
	cbnz	w8, .LBB15_17
	b	.LBB15_33
.LBB15_413:
	adrp	x1, .L.str.150
	adrp	x2, .L.str.147
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.150
	add	x2, x2, :lo12:.L.str.147
	mov	w3, w21
	bl	_dev_info
	mov	w12, #24
	ldr	w8, [x19, #184]
	b	.LBB15_27
.LBB15_414:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.125
	adrp	x2, .L.str.124
	ldr	w3, [x26, #32]
	ldr	w4, [x24, #12]
	add	x1, x1, :lo12:.L.str.125
	ldr	x0, [x8]
	add	x2, x2, :lo12:.L.str.124
	bl	_dev_info
.LBB15_415:
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	b	.LBB15_309
.LBB15_416:
	ldr	w8, [x22]
	tbz	w8, #31, .LBB15_442
.LBB15_417:
	mov	w21, #4
	str	w21, [x26, #32]
.LBB15_418:
	mov	w2, #5
	b	.LBB15_230
.LBB15_419:
	mov	w2, #4
	b	.LBB15_230
.LBB15_420:
	mov	w2, #14
	b	.LBB15_230
.LBB15_421:
	mov	w2, #13
	b	.LBB15_230
.LBB15_422:
	mov	w2, #2
	b	.LBB15_230
.LBB15_423:
	ldrb	w8, [x24, #88]
	mov	w9, #15
	cmp	w8, #0
	mov	w8, #3
	csel	w2, w8, w9, eq
	b	.LBB15_230
.LBB15_424:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.148
	add	x1, x1, :lo12:.L.str.148
	mov	w2, #-12
	mov	w22, #-12
	mov	w24, w9
	ldr	x0, [x8]
	mov	w23, w10
	bl	_dev_err
	mov	w10, w23
	mov	w9, w24
	ldr	x0, [sp, #56]
	ldur	x24, [x29, #-64]
	b	.LBB15_32
.LBB15_425:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.149
	adrp	x2, .L.str.147
	add	x1, x1, :lo12:.L.str.149
	add	x2, x2, :lo12:.L.str.147
	mov	w3, w22
	ldr	x0, [x8]
	mov	w22, w9
	mov	w23, w10
	bl	_dev_info
	mov	w10, w23
	mov	w9, w22
	ldr	x0, [sp, #56]
	b	.LBB15_33
.LBB15_426:
	ldr	w8, [x22]
	tbz	w8, #31, .LBB15_443
.LBB15_427:
	mov	w6, #4
	b	.LBB15_77
.LBB15_428:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.166
	adrp	x2, .L.str.167
	add	x1, x1, :lo12:.L.str.166
	add	x2, x2, :lo12:.L.str.167
	ldr	x0, [x8]
	bl	_dev_info
	ldr	w12, [sp, #44]
	b	.LBB15_149
.LBB15_429:
	mov	w22, #9
	b	.LBB15_94
.LBB15_430:
	ldur	x8, [x29, #-48]
	adrp	x2, .L.str.152
	add	x2, x2, :lo12:.L.str.152
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_60
.LBB15_431:
	str	w8, [sp, #52]
	b	.LBB15_440
.LBB15_432:
	adrp	x8, .L.str.178
	adrp	x9, .L.str.170
	tst	w21, #0x4
	add	x8, x8, :lo12:.L.str.178
	add	x9, x9, :lo12:.L.str.170
	adrp	x1, .L.str.177
	csel	x3, x9, x8, eq
	ldur	x8, [x29, #-48]
	adrp	x9, .L.str.112
	tst	w23, #0x1
	add	x9, x9, :lo12:.L.str.112
	adrp	x2, .L.str.152
	ldr	x0, [x8]
	adrp	x8, .L.str.113
	add	x8, x8, :lo12:.L.str.113
	add	x1, x1, :lo12:.L.str.177
	csel	x4, x8, x9, ne
	add	x2, x2, :lo12:.L.str.152
	bl	_dev_info
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	b	.LBB15_409
.LBB15_433:
	ldur	x8, [x29, #-48]
	adrp	x9, .L.str.171
	tst	w21, #0x2
	add	x9, x9, :lo12:.L.str.171
	adrp	x1, .L.str.169
	adrp	x2, .L.str.167
	ldr	x0, [x8]
	adrp	x8, .L.str.170
	add	x8, x8, :lo12:.L.str.170
	add	x1, x1, :lo12:.L.str.169
	csel	x3, x9, x8, eq
	add	x2, x2, :lo12:.L.str.167
	bl	_dev_info
	mov	w23, w27
	tbnz	w21, #3, .LBB15_254
	b	.LBB15_255
.LBB15_434:
	adrp	x1, .L.str.174
	add	x1, x1, :lo12:.L.str.174
.LBB15_435:
	ldur	x8, [x29, #-48]
	adrp	x2, .L.str.167
	add	x2, x2, :lo12:.L.str.167
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_255
.LBB15_436:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.168
	adrp	x2, .L.str.167
	add	x1, x1, :lo12:.L.str.168
	add	x2, x2, :lo12:.L.str.167
	ldr	x0, [x8]
	bl	_dev_info
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	b	.LBB15_307
.LBB15_437:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.179
	adrp	x2, .L.str.180
	add	x1, x1, :lo12:.L.str.179
	add	x2, x2, :lo12:.L.str.180
	mov	w3, w21
	ldr	x0, [x8]
	mov	w4, w25
	bl	_dev_info
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	cmp	w21, #10
	b.ne	.LBB15_318
.LBB15_438:
	str	wzr, [sp, #52]
.LBB15_439:
	ldur	x24, [x29, #-64]
.LBB15_440:
	mov	w25, #2
	mov	w23, w27
	b	.LBB15_340
.LBB15_441:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.126
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.126
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_392
.LBB15_442:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.155
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.155
	add	x2, x2, :lo12:.L.str.118
	ldr	x0, [x8]
	bl	_dev_info
	adrp	x6, .LJTI15_1
	mov	w7, #65535
	add	x6, x6, :lo12:.LJTI15_1
	b	.LBB15_417
.LBB15_443:
	ldur	x8, [x29, #-48]
	adrp	x1, .L.str.155
	adrp	x2, .L.str.118
	add	x1, x1, :lo12:.L.str.155
	add	x2, x2, :lo12:.L.str.118
	mov	w3, w6
	ldr	x0, [x8]
	bl	_dev_info
	b	.LBB15_427
.Lfunc_end15:
	.size	ebc_thread, .Lfunc_end15-ebc_thread
	.section	.rodata,"a",@progbits
	.p2align	2
.LJTI15_0:
	.word	.LBB15_229-.Ltmp17
	.word	.LBB15_419-.Ltmp17
	.word	.LBB15_418-.Ltmp17
	.word	.LBB15_410-.Ltmp17
	.word	.LBB15_240-.Ltmp17
	.word	.LBB15_420-.Ltmp17
	.word	.LBB15_421-.Ltmp17
	.word	.LBB15_418-.Ltmp17
	.word	.LBB15_410-.Ltmp17
	.word	.LBB15_240-.Ltmp17
	.word	.LBB15_411-.Ltmp17
	.word	.LBB15_411-.Ltmp17
	.word	.LBB15_285-.Ltmp17
	.word	.LBB15_422-.Ltmp17
	.word	.LBB15_285-.Ltmp17
	.word	.LBB15_230-.Ltmp17
	.word	.LBB15_229-.Ltmp17
	.word	.LBB15_229-.Ltmp17
	.word	.LBB15_229-.Ltmp17
	.word	.LBB15_423-.Ltmp17
	.p2align	2
.LJTI15_1:
	.word	.LBB15_328-.Ltmp18
	.word	.LBB15_329-.Ltmp18
	.word	.LBB15_330-.Ltmp18
	.word	.LBB15_331-.Ltmp18
	.word	.LBB15_333-.Ltmp18
	.word	.LBB15_335-.Ltmp18
	.word	.LBB15_323-.Ltmp18
	.word	.LBB15_336-.Ltmp18
	.word	.LBB15_337-.Ltmp18

	.text
	.p2align	2
	.type	ebc_delay_power_off,@function
ebc_delay_power_off:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	adrp	x8, global_ebc
	stp	x20, x19, [sp, #16]
	mov	x29, sp
	ldr	x20, [x8, :lo12:global_ebc]
	ldr	w8, [x20, #1876]
	cbz	w8, .LBB16_9
	ldr	w8, [x20, #2092]
	cbz	w8, .LBB16_3
	mov	x0, xzr
	bl	ebc_notify
.LBB16_3:
	ldr	x0, [x20, #8]
	str	wzr, [x20, #1876]
	ldr	x8, [x0, #176]
	blr	x8
	ldrb	w8, [x20, #296]
	cbz	w8, .LBB16_5
	mov	w0, #10
	bl	msleep
.LBB16_5:
	ldr	x0, [x20, #16]
	cbz	x0, .LBB16_10
	ldr	x8, [x0, #32]
	mov	w1, wzr
	blr	x8
	mov	w19, wzr
	ldr	w8, [x20, #2080]
	cbz	w8, .LBB16_8
.LBB16_7:
	add	x0, x20, #1888
	str	wzr, [x20, #2080]
	bl	__pm_relax
.LBB16_8:
	ldr	w8, [x20, #2096]
	cmp	w8, #1
	b.ge	.LBB16_11
.LBB16_9:
	ldp	x20, x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.LBB16_10:
	ldr	x0, [x20, #24]
	bl	regulator_disable
	mov	w19, w0
	ldr	w8, [x20, #2080]
	cbnz	w8, .LBB16_7
	b	.LBB16_8
.LBB16_11:
	adrp	x1, .L.str.145
	adrp	x2, .L.str.144
	ldr	x0, [x20, #304]
	add	x1, x1, :lo12:.L.str.145
	add	x2, x2, :lo12:.L.str.144
	mov	w3, w19
	bl	_dev_info
	b	.LBB16_9
.Lfunc_end16:
	.size	ebc_delay_power_off, .Lfunc_end16-ebc_delay_power_off

	.p2align	2
	.type	ebc_wake_up_for_ghost_rm_last,@function
ebc_wake_up_for_ghost_rm_last:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x8, global_ebc
	mov	w9, #18576
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	add	x8, x8, x9
	ldrb	w9, [x8]
	tbz	w9, #1, .LBB17_3
	ldr	w9, [x8, #12]
	cbz	w9, .LBB17_3
	mov	w9, #2
	adrp	x10, ebc_task
	str	w9, [x8, #12]
	ldr	x0, [x10, :lo12:ebc_task]
	bl	wake_up_process
.LBB17_3:
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end17:
	.size	ebc_wake_up_for_ghost_rm_last, .Lfunc_end17-ebc_wake_up_for_ghost_rm_last

	.p2align	2
	.type	get_auto_data,@function
get_auto_data:
	sub	sp, sp, #224
	str	x30, [x18], #8
	stp	x29, x30, [sp, #128]
	add	x29, sp, #128
	stp	x28, x27, [sp, #144]
	adrp	x9, global_ebc
	stp	x26, x25, [sp, #160]
	stp	x24, x23, [sp, #176]
	stp	x22, x21, [sp, #192]
	stp	x20, x19, [sp, #208]
	str	x0, [sp, #48]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	stur	x8, [x29, #-8]
	mov	w8, #18220
	ldr	x23, [x9, :lo12:global_ebc]
	add	x12, x1, x8
	str	w2, [sp, #36]
	ldr	w9, [x23, #272]
	stp	x12, x1, [sp, #16]
	cbz	w9, .LBB18_3
	ldp	w8, w10, [x12]
	ldr	w9, [x12, #8]
	ldr	x11, [x1, #1528]
	ldr	x24, [x1, #960]
	stp	w8, w9, [x29, #-20]
	ldr	x8, [x1, #1536]
	stur	x11, [x29, #-32]
	mov	x9, #2048
	stur	w10, [x29, #-12]
	mov	x10, #3076
	stp	x11, x8, [x29, #-48]
	movk	x9, #6160, lsl #16
	ldp	w11, w8, [x23, #188]
	movk	x10, #7188, lsl #16
	movk	x9, #10272, lsl #32
	movk	x10, #11300, lsl #32
	movk	x9, #14384, lsl #48
	movk	x10, #15412, lsl #48
	ldr	x26, [x1, #1032]
	str	w11, [sp, #60]
	lsr	w20, w11, #3
	ldr	x11, [x23, #120]
	cmp	w8, #0
	cinc	w12, w8, lt
	ldr	x23, [x1, #1176]
	ldr	x27, [x1, #1248]
	asr	w19, w12, #1
	ldr	w28, [x11, #120]
	cmp	w2, #2
	stur	x9, [x29, #-56]
	str	x10, [sp, #64]
	b.ne	.LBB18_9
	mov	w22, wzr
	b	.LBB18_10
.LBB18_3:
	ldr	x14, [x23, #120]
	ldr	x24, [x1, #960]
	ldr	x25, [x1, #1032]
	ldr	w8, [x14, #116]
	ldr	x26, [x1, #1176]
	ldr	x27, [x1, #1248]
	cbz	w8, .LBB18_31
	ldp	w13, w11, [x23, #188]
	ldr	w10, [x12, #8]
	ldp	w9, w12, [x12]
	cmp	w11, #0
	lsr	w15, w13, #3
	str	w13, [sp, #44]
	stp	w9, w10, [x29, #-20]
	cinc	w10, w11, lt
	stur	w12, [x29, #-12]
	ldr	x9, [x1, #1528]
	ldr	x12, [x1, #1536]
	asr	w10, w10, #1
	cmp	w2, #2
	stur	x9, [x29, #-32]
	stp	x9, x12, [x29, #-48]
	b.ne	.LBB18_33
	mov	w12, wzr
	cmp	w12, w10
	b.lt	.LBB18_34
.LBB18_6:
	mov	w11, wzr
.LBB18_7:
	ldr	w10, [sp, #36]
	cmp	w10, #2
	b.ne	.LBB18_66
	ldr	x8, [sp, #16]
	strb	w11, [x8, #110]
	b	.LBB18_67
.LBB18_9:
	mul	w9, w19, w20
	mov	w22, w19
	mov	w19, w8
	sbfiz	x9, x9, #3, #32
	add	x24, x24, x9
	add	x26, x26, x9
	add	x23, x23, x9
	add	x27, x27, x9
.LBB18_10:
	subs	w8, w19, w22
	mul	w9, w22, w28
	mov	w1, wzr
	mul	w2, w8, w28
	ldr	x8, [sp, #48]
	add	x0, x8, x9
	bl	memset
	mov	w8, wzr
	cmp	w19, w22
	b.le	.LBB18_26
	cmp	w20, #1
	adrp	x16, _8_8bit_mask
	adrp	x17, .L__const.get_normal_panel_auto_data.offset
	mov	w8, wzr
	mov	x9, xzr
	csinc	w10, w20, wzr, gt
	sub	x13, x29, #48
	sub	x14, x29, #20
	mov	w15, #1
	add	x16, x16, :lo12:_8_8bit_mask
	add	x17, x17, :lo12:.L__const.get_normal_panel_auto_data.offset
	b	.LBB18_13
.LBB18_12:
	add	w22, w22, #1
	cmp	w22, w19
	b.eq	.LBB18_26
.LBB18_13:
	ldr	x11, [sp, #48]
	mul	w0, w22, w28
	tst	w22, #0x1
	sub	x12, x29, #56
	add	x1, x11, x0
	add	x11, sp, #64
	csel	x0, x12, x11, eq
	ldr	w11, [sp, #60]
	csel	x9, x1, x9, eq
	cmp	w11, #8
	b.lo	.LBB18_12
	mov	x1, xzr
	b	.LBB18_18
.LBB18_15:
	str	x2, [x26]
.LBB18_16:
	lsl	x11, x1, #2
	ldr	w12, [x9, x11]
	orr	w12, w12, w3
	str	w12, [x9, x11]
.LBB18_17:
	add	x24, x24, #8
	add	x26, x26, #8
	add	x23, x23, #8
	add	x27, x27, #8
	add	x1, x1, #1
	cmp	x1, x10
	b.eq	.LBB18_12
.LBB18_18:
	ldr	x4, [x24]
	ldr	x2, [x26]
	eor	x5, x2, x4
	cbz	x5, .LBB18_17
	mov	x6, xzr
	mov	w7, wzr
	mov	w3, wzr
	b	.LBB18_22
.LBB18_20:
	lsl	w11, w15, w30
	and	w11, w11, #0xff
	orr	w8, w8, w11
.LBB18_21:
	add	x6, x6, #1
	cmp	x6, #8
	b.eq	.LBB18_25
.LBB18_22:
	ldr	x20, [x16, x6, lsl #3]
	tst	x20, x5
	b.eq	.LBB18_21
	ldrb	w30, [x27, x6]
	ldr	w21, [x17, x6, lsl #2]
	ldrb	w11, [x23, x6]
	ldr	x30, [x13, x30, lsl #3]
	lsr	x12, x2, x21
	lsr	x21, x4, x21
	lsl	w12, w12, #5
	and	x12, x12, #0x3e0
	bfxil	x12, x21, #0, #5
	add	x21, x30, x11, lsl #10
	add	w11, w11, #1
	ldrb	w12, [x21, x12]
	ldrb	w21, [x0, x6]
	strb	w11, [x23, x6]
	ldrb	w30, [x27, x6]
	lsl	w12, w12, w21
	ldr	w25, [x14, x30, lsl #2]
	orr	w3, w12, w3
	cmp	w25, w11, uxtb
	b.gt	.LBB18_20
	add	x11, x6, #1
	bic	x12, x2, x20
	and	x2, x20, x4
	strb	wzr, [x23, x6]
	strb	wzr, [x27, x6]
	cmp	x6, #7
	orr	x2, x12, x2
	mov	w7, #1
	mov	x6, x11
	b.ne	.LBB18_22
	b	.LBB18_15
.LBB18_25:
	tbz	w7, #0, .LBB18_16
	b	.LBB18_15
.LBB18_26:
	ldr	w12, [sp, #36]
	ldr	x13, [sp, #24]
	cmp	w12, #2
	b.ne	.LBB18_28
	ldr	x9, [sp, #16]
	strb	w8, [x9, #110]
	b	.LBB18_29
.LBB18_28:
	ldr	x9, [sp, #16]
	strb	w8, [x9, #109]
.LBB18_29:
	ldr	w9, [x13, #1792]
	cmp	w9, #2
	b.lt	.LBB18_92
	adrp	x9, .L.str.111
	adrp	x10, .L.str.110
	add	x9, x9, :lo12:.L.str.111
	add	x10, x10, :lo12:.L.str.110
	cmp	w12, #2
	adrp	x11, .L.str.109
	add	x11, x11, :lo12:.L.str.109
	csel	x9, x10, x9, eq
	cmp	w12, #3
	adrp	x10, .L.str.113
	csel	x2, x11, x9, eq
	adrp	x9, .L.str.112
	add	x9, x9, :lo12:.L.str.112
	add	x10, x10, :lo12:.L.str.113
	cmp	w8, #0
	adrp	x1, .L.str.108
	ldp	w4, w5, [x29, #-20]
	ldr	x0, [x13]
	csel	x3, x10, x9, eq
	add	x1, x1, :lo12:.L.str.108
	bl	_dev_info
	b	.LBB18_92
.LBB18_31:
	ldp	w13, w8, [x23, #188]
	ldp	w9, w11, [x12]
	ldr	w10, [x12, #8]
	cmp	w8, #0
	lsr	w20, w13, #3
	stp	w9, w10, [x29, #-20]
	cinc	w10, w8, lt
	stur	w11, [x29, #-12]
	ldr	x9, [x1, #1528]
	ldr	x11, [x1, #1536]
	asr	w28, w10, #1
	ldr	w10, [x14, #120]
	cmp	w2, #2
	str	w13, [sp, #60]
	stp	x9, x11, [x29, #-48]
	str	w10, [sp, #44]
	stur	x9, [x29, #-32]
	b.ne	.LBB18_69
	mov	w22, wzr
	b	.LBB18_70
.LBB18_33:
	mul	w9, w10, w15
	mov	w12, w10
	mov	w10, w11
	sbfiz	x9, x9, #3, #32
	add	x24, x24, x9
	add	x25, x25, x9
	add	x26, x26, x9
	add	x27, x27, x9
	cmp	w12, w10
	b.ge	.LBB18_6
.LBB18_34:
	ldr	w9, [x23, #260]
	cmp	w15, #1
	adrp	x16, _8_8bit_mask
	adrp	x3, .L__const.get_normal_panel_auto_data.offset
	adrp	x4, .L__const.get_normal_panel_soft_tcon_auto_data.shift_table
	mov	w11, wzr
	str	w9, [sp, #60]
	ldr	w9, [x14, #120]
	csinc	w15, w15, wzr, gt
	add	x16, x16, :lo12:_8_8bit_mask
	sub	x0, x29, #48
	sub	x1, x29, #20
	mov	w2, #1
	add	x3, x3, :lo12:.L__const.get_normal_panel_auto_data.offset
	add	x4, x4, :lo12:.L__const.get_normal_panel_soft_tcon_auto_data.shift_table
	str	w9, [sp, #12]
	str	w10, [sp, #40]
	b	.LBB18_36
.LBB18_35:
	ldr	w10, [sp, #40]
	add	w12, w12, #1
	cmp	w12, w10
	b.eq	.LBB18_7
.LBB18_36:
	ldr	w9, [x23, #268]
	mov	w14, w12
	cbz	w9, .LBB18_38
	ldr	w9, [x23, #192]
	mvn	w14, w12
	add	w14, w9, w14
.LBB18_38:
	ldr	w9, [sp, #44]
	cmp	w9, #8
	b.lo	.LBB18_35
	ldr	x9, [x23, #120]
	mov	w5, wzr
	ldr	w10, [sp, #12]
	ldr	w9, [x9, #104]
	mul	w14, w14, w10
	ldr	x10, [sp, #48]
	mul	w9, w9, w8
	add	x14, x10, x14
	add	x6, x14, x9
	b	.LBB18_42
.LBB18_40:
	mov	w14, #2
	strh	w7, [x6]
.LBB18_41:
	add	x6, x6, x14
	add	x24, x24, #8
	add	x25, x25, #8
	add	x26, x26, #8
	add	x27, x27, #8
	add	w5, w5, #1
	cmp	w5, w15
	b.eq	.LBB18_35
.LBB18_42:
	ldr	x22, [x24]
	ldr	x19, [x25]
	eor	x28, x19, x22
	cbz	x28, .LBB18_54
	mov	x30, xzr
	mov	w14, wzr
	mov	w7, wzr
	b	.LBB18_46
.LBB18_44:
	lsl	w9, w2, w9
	orr	w11, w11, w9
.LBB18_45:
	add	x30, x30, #1
	cmp	x30, #8
	b.eq	.LBB18_49
.LBB18_46:
	ldr	x20, [x16, x30, lsl #3]
	tst	x20, x28
	b.eq	.LBB18_45
	ldrb	w9, [x27, x30]
	ldr	w21, [x3, x30, lsl #2]
	ldrb	w10, [x26, x30]
	ldr	x9, [x0, x9, lsl #3]
	lsr	x17, x19, x21
	lsr	x21, x22, x21
	lsl	w17, w17, #5
	and	x17, x17, #0x3e0
	bfxil	x17, x21, #0, #5
	add	x9, x9, x10, lsl #10
	add	w10, w10, #1
	ldrb	w21, [x4, x30]
	ldrb	w17, [x9, x17]
	strb	w10, [x26, x30]
	ldrb	w9, [x27, x30]
	lsl	w17, w17, w21
	ldr	w13, [x1, x9, lsl #2]
	orr	w7, w7, w17
	cmp	w13, w10, uxtb
	b.gt	.LBB18_44
	add	x9, x30, #1
	bic	x10, x19, x20
	and	x13, x20, x22
	strb	wzr, [x26, x30]
	strb	wzr, [x27, x30]
	cmp	x30, #7
	orr	x19, x10, x13
	mov	w14, #1
	mov	x30, x9
	b.ne	.LBB18_46
	b	.LBB18_50
.LBB18_49:
	tbz	w14, #0, .LBB18_51
.LBB18_50:
	str	x19, [x25]
.LBB18_51:
	cmp	w8, #4
	b.eq	.LBB18_57
	cmp	w8, #2
	b.ne	.LBB18_40
	lsr	w9, w7, #8
	mov	w14, #4
	strb	w7, [x6, #2]
	b	.LBB18_64
.LBB18_54:
	cmp	w8, #4
	b.eq	.LBB18_59
	cmp	w8, #2
	b.ne	.LBB18_62
	ldrb	w9, [x6, #1]
	mov	w14, #4
	ldrb	w10, [x6, #3]
	lsl	w9, w9, #8
	lsl	w10, w10, #8
	strh	w9, [x6]
	strh	w10, [x6, #2]
	b	.LBB18_41
.LBB18_57:
	ldr	w9, [sp, #60]
	cbz	w9, .LBB18_63
	ldr	x9, [x23, #120]
	mov	w14, #255
	ldr	w10, [x6]
	and	w13, w7, #0xff00
	ldr	w9, [x9, #100]
	lsl	w9, w14, w9
	bic	w9, w10, w9
	str	w9, [x6]
	ldr	x10, [x23, #120]
	ldr	w10, [x10, #100]
	sub	w10, w10, #8
	lsl	w10, w13, w10
	and	w13, w7, #0xff
	orr	w9, w10, w9
	str	w9, [x6]
	ldr	x10, [x23, #120]
	ldr	w10, [x10, #96]
	lsl	w10, w14, w10
	bic	w9, w9, w10
	str	w9, [x6]
	ldr	x10, [x23, #120]
	ldr	w10, [x10, #96]
	lsl	w10, w13, w10
	orr	w9, w10, w9
	b	.LBB18_61
.LBB18_59:
	ldr	w9, [sp, #60]
	cbz	w9, .LBB18_65
	ldr	x9, [x23, #120]
	mov	w14, #255
	ldr	w13, [x6]
	ldp	w9, w10, [x9, #96]
	lsl	w10, w14, w10
	lsl	w9, w14, w9
	orr	w9, w9, w10
	bic	w9, w13, w9
.LBB18_61:
	mov	w14, #4
	str	w9, [x6]
	b	.LBB18_41
.LBB18_62:
	mov	w14, #2
	strh	wzr, [x6]
	b	.LBB18_41
.LBB18_63:
	lsr	w9, w7, #8
	mov	w14, #8
	strb	w7, [x6, #4]
.LBB18_64:
	strb	w9, [x6]
	b	.LBB18_41
.LBB18_65:
	ldp	w9, w10, [x6]
	mov	w14, #8
	and	w9, w9, #0xffffff00
	and	w10, w10, #0xffffff00
	stp	w9, w10, [x6]
	b	.LBB18_41
.LBB18_66:
	ldr	x8, [sp, #16]
	strb	w11, [x8, #109]
.LBB18_67:
	ldr	x12, [sp, #24]
	ldr	w8, [x12, #1792]
	cmp	w8, #2
	b.lt	.LBB18_92
	adrp	x8, .L.str.111
	adrp	x9, .L.str.110
	add	x8, x8, :lo12:.L.str.111
	add	x9, x9, :lo12:.L.str.110
	cmp	w10, #2
	ldr	x0, [x12]
	ldp	w4, w5, [x29, #-20]
	csel	x8, x9, x8, eq
	adrp	x9, .L.str.109
	add	x9, x9, :lo12:.L.str.109
	cmp	w10, #3
	csel	x2, x9, x8, eq
	and	w3, w11, #0xff
	b	.LBB18_95
.LBB18_69:
	mul	w9, w28, w20
	mov	w22, w28
	mov	w28, w8
	sbfiz	x9, x9, #3, #32
	add	x24, x24, x9
	add	x25, x25, x9
	add	x26, x26, x9
	add	x27, x27, x9
.LBB18_70:
	ldr	w10, [sp, #44]
	subs	w8, w28, w22
	mov	w1, wzr
	mul	w2, w8, w10
	ldr	x8, [sp, #48]
	mul	w9, w22, w10
	add	x0, x8, x9
	bl	memset
	mov	w8, wzr
	cmp	w28, w22
	b.le	.LBB18_88
	cmp	w20, #1
	adrp	x9, _8_8bit_mask
	adrp	x14, .L__const.get_normal_panel_auto_data.offset
	adrp	x15, .L__const.get_normal_panel_auto_data.shift_table
	mov	w8, wzr
	add	x9, x9, :lo12:_8_8bit_mask
	csinc	w10, w20, wzr, gt
	sub	x11, x29, #48
	sub	x12, x29, #20
	mov	w13, #1
	add	x14, x14, :lo12:.L__const.get_normal_panel_auto_data.offset
	add	x15, x15, :lo12:.L__const.get_normal_panel_auto_data.shift_table
	b	.LBB18_73
.LBB18_72:
	add	w22, w22, #1
	cmp	w22, w28
	b.eq	.LBB18_88
.LBB18_73:
	ldr	w16, [x23, #268]
	mov	w17, w22
	cbz	w16, .LBB18_75
	ldr	w16, [x23, #192]
	mvn	w17, w22
	add	w17, w16, w17
.LBB18_75:
	ldr	w16, [sp, #60]
	cmp	w16, #8
	b.lo	.LBB18_72
	ldr	w0, [sp, #44]
	mov	x16, xzr
	mul	w17, w17, w0
	ldr	x0, [sp, #48]
	add	x17, x0, x17
	b	.LBB18_80
.LBB18_77:
	str	x0, [x25]
.LBB18_78:
	strh	w1, [x17, x16, lsl #1]
.LBB18_79:
	add	x24, x24, #8
	add	x25, x25, #8
	add	x26, x26, #8
	add	x27, x27, #8
	add	x16, x16, #1
	cmp	x16, x10
	b.eq	.LBB18_72
.LBB18_80:
	ldr	x2, [x24]
	ldr	x0, [x25]
	eor	x3, x0, x2
	cbz	x3, .LBB18_79
	mov	x4, xzr
	mov	w5, wzr
	mov	w1, wzr
	b	.LBB18_84
.LBB18_82:
	lsl	w6, w13, w7
	orr	w8, w8, w6
.LBB18_83:
	add	x4, x4, #1
	cmp	x4, #8
	b.eq	.LBB18_87
.LBB18_84:
	ldr	x6, [x9, x4, lsl #3]
	tst	x6, x3
	b.eq	.LBB18_83
	ldrb	w7, [x27, x4]
	ldr	w20, [x14, x4, lsl #2]
	ldrb	w21, [x26, x4]
	ldr	x7, [x11, x7, lsl #3]
	lsr	x30, x0, x20
	lsr	x20, x2, x20
	lsl	w30, w30, #5
	and	x30, x30, #0x3e0
	bfxil	x30, x20, #0, #5
	add	x7, x7, x21, lsl #10
	add	w20, w21, #1
	ldrb	w21, [x7, x30]
	strb	w20, [x26, x4]
	ldrb	w7, [x27, x4]
	ldrb	w30, [x15, x4]
	ldr	w19, [x12, x7, lsl #2]
	lsl	w21, w21, w30
	orr	w1, w1, w21
	cmp	w19, w20, uxtb
	b.gt	.LBB18_82
	add	x7, x4, #1
	bic	x0, x0, x6
	and	x5, x6, x2
	strb	wzr, [x26, x4]
	strb	wzr, [x27, x4]
	cmp	x4, #7
	orr	x0, x0, x5
	mov	w5, #1
	mov	x4, x7
	b.ne	.LBB18_84
	b	.LBB18_77
.LBB18_87:
	tbz	w5, #0, .LBB18_78
	b	.LBB18_77
.LBB18_88:
	ldr	w11, [sp, #36]
	ldr	x12, [sp, #24]
	cmp	w11, #2
	b.ne	.LBB18_90
	ldr	x9, [sp, #16]
	strb	w8, [x9, #110]
	b	.LBB18_91
.LBB18_90:
	ldr	x9, [sp, #16]
	strb	w8, [x9, #109]
.LBB18_91:
	ldr	w9, [x12, #1792]
	cmp	w9, #2
	b.ge	.LBB18_94
.LBB18_92:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB18_96
	ldp	x20, x19, [sp, #208]
	ldp	x22, x21, [sp, #192]
	ldp	x24, x23, [sp, #176]
	ldp	x26, x25, [sp, #160]
	ldp	x28, x27, [sp, #144]
	ldp	x29, x30, [sp, #128]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #224
	ret
.LBB18_94:
	adrp	x9, .L.str.111
	adrp	x10, .L.str.110
	add	x9, x9, :lo12:.L.str.111
	add	x10, x10, :lo12:.L.str.110
	cmp	w11, #2
	ldr	x0, [x12]
	csel	x9, x10, x9, eq
	adrp	x10, .L.str.109
	add	x10, x10, :lo12:.L.str.109
	cmp	w11, #3
	ldp	w4, w5, [x29, #-20]
	csel	x2, x10, x9, eq
	and	w3, w8, #0xff
.LBB18_95:
	adrp	x1, .L.str.114
	ldur	w6, [x29, #-12]
	add	x1, x1, :lo12:.L.str.114
	bl	_dev_info
	b	.LBB18_92
.LBB18_96:
	bl	__stack_chk_fail
.Lfunc_end18:
	.size	get_auto_data, .Lfunc_end18-get_auto_data

	.p2align	2
	.type	get_full_or_part_data,@function
get_full_or_part_data:
	sub	sp, sp, #224
	str	x30, [x18], #8
	adrp	x8, global_ebc
	stp	x24, x23, [sp, #176]
	stp	x20, x19, [sp, #208]
	mov	x19, x1
	stp	x29, x30, [sp, #128]
	add	x29, sp, #128
	ldr	x24, [x8, :lo12:global_ebc]
	mov	w8, #18220
	add	x23, x1, x8
	stp	x28, x27, [sp, #144]
	stp	x26, x25, [sp, #160]
	ldr	w9, [x24, #272]
	stp	x22, x21, [sp, #192]
	str	x0, [sp, #40]
	str	w2, [sp, #12]
	cbz	w9, .LBB19_3
	ldr	x8, [x19, #1464]
	mov	x11, #1538
	ldr	w20, [x23]
	movk	x11, #3594, lsl #16
	ldr	w9, [x23, #12]
	movk	x11, #5650, lsl #32
	ldr	x25, [x8, #8]
	movk	x11, #7706, lsl #48
	ldr	x10, [x19, #1456]
	ldp	w12, w8, [x24, #188]
	sub	w22, w20, w9
	mov	x9, #1024
	ldr	x27, [x10, #8]
	movk	x9, #3080, lsl #16
	ldr	x10, [x24, #120]
	movk	x9, #5136, lsl #32
	cmp	w8, #0
	str	w12, [sp, #36]
	lsr	w26, w12, #3
	cinc	w12, w8, lt
	movk	x9, #7192, lsl #48
	ldr	x28, [x19, #1528]
	ldrb	w24, [x23, #77]
	asr	w12, w12, #1
	ldr	w10, [x10, #120]
	cmp	w2, #2
	stp	x11, x9, [x29, #-16]
	b.ne	.LBB19_9
	mov	w21, wzr
	b	.LBB19_10
.LBB19_3:
	ldr	w8, [x24, #264]
	cmp	w8, #230
	b.ne	.LBB19_41
	ldr	x10, [x24, #120]
	sub	x12, x29, #16
	ldr	w8, [x23]
	sub	x15, x29, #8
	ldr	w11, [x23, #12]
	mov	w1, #1024
	ldp	w10, w9, [x10, #116]
	ldr	x13, [x19, #1464]
	mov	w4, #2060
	sub	w3, w8, w11
	ldr	x14, [x19, #1456]
	movk	w1, #3080, lsl #16
	movk	w4, #4, lsl #16
	stur	w9, [x29, #-20]
	cmp	w10, #0
	ldp	w11, w9, [x24, #188]
	csel	x12, x15, x12, eq
	ldr	x13, [x13, #8]
	ldr	x14, [x14, #8]
	stur	w1, [x29, #-8]
	ldr	x6, [x19, #1528]
	stur	w4, [x29, #-16]
	cmp	w9, #0
	lsr	w0, w11, #2
	cinc	w16, w9, lt
	ldrb	w15, [x23, #77]
	asr	w17, w16, #1
	ldr	w16, [x24, #260]
	cmp	w2, #2
	b.ne	.LBB19_47
	mov	w1, wzr
	cmp	w1, w17
	b.lt	.LBB19_48
.LBB19_6:
	mov	w0, #1
.LBB19_7:
	ldr	w11, [sp, #12]
	and	w8, w0, #0x1
	cmp	w11, #2
	b.ne	.LBB19_81
	strb	w8, [x23, #105]
	b	.LBB19_82
.LBB19_9:
	mul	w9, w12, w26
	mov	w21, w12
	mov	w12, w8
	sbfiz	x9, x9, #3, #32
	add	x25, x25, x9
	add	x27, x27, x9
.LBB19_10:
	subs	w8, w12, w21
	mul	w9, w21, w10
	mov	w1, wzr
	stp	w10, w12, [sp, #16]
	mul	w2, w8, w10
	ldr	x8, [sp, #40]
	add	x0, x8, x9
	bl	memset
	ldr	w14, [sp, #20]
	cmp	w14, w21
	b.le	.LBB19_35
	lsl	w8, w22, #10
	cmp	w26, #1
	csinc	w9, w26, wzr, gt
	mov	x17, xzr
	add	x11, x28, w8, sxtw
	lsl	x10, x9, #3
	mov	w8, #1
	sub	x0, x29, #16
	sub	x1, x29, #8
	mov	w15, w22
	ldr	w13, [sp, #16]
	b	.LBB19_14
.LBB19_12:
	ldp	w13, w14, [sp, #16]
	add	x27, x27, x17
	add	x25, x25, x17
	ldr	x17, [sp, #24]
	sub	x0, x29, #16
	sub	x1, x29, #8
.LBB19_13:
	add	w21, w21, #1
	cmp	w21, w14
	b.eq	.LBB19_36
.LBB19_14:
	ldr	x12, [sp, #40]
	mul	w9, w21, w13
	tst	w21, #0x1
	csel	x16, x1, x0, eq
	add	x9, x12, x9
	csel	x17, x9, x17, eq
	ldr	w9, [sp, #36]
	cmp	w9, #8
	b.lo	.LBB19_13
	ldrb	w12, [x16, #1]
	mov	w14, #3
	ldrb	w13, [x16, #2]
	mov	x9, x17
	ldrb	w0, [x16, #3]
	mov	x17, xzr
	ldrb	w1, [x16, #4]
	stp	x13, x12, [x29, #-40]
	lsl	w12, w14, w12
	ldrb	w2, [x16, #5]
	str	x9, [sp, #24]
	ldrb	w3, [x16, #6]
	stp	x1, x0, [x29, #-56]
	stur	w12, [x29, #-20]
	lsl	w12, w14, w13
	ldrb	w30, [x16, #7]
	lsl	w13, w14, w1
	lsl	w28, w14, w2
	lsl	w26, w14, w3
	stur	w12, [x29, #-24]
	lsl	w12, w14, w0
	lsl	w14, w14, w30
	stp	x3, x2, [sp, #56]
	str	w14, [sp, #52]
	b	.LBB19_18
.LBB19_16:
	cmp	w20, #0
	mov	w22, #-1
	cset	w14, lt
	and	w8, w14, w8
.LBB19_17:
	add	x9, x9, #4
	add	x17, x17, #8
	cmp	x10, x17
	b.eq	.LBB19_12
.LBB19_18:
	ldr	x14, [x25, x17]
	ldr	x0, [x27, x17]
	cbz	w24, .LBB19_26
	ldr	x1, [x19, #888]
	cbz	x1, .LBB19_16
	ldr	w2, [x19, #212]
	asr	w22, w17, #1
	asr	w2, w2, #1
	mul	w2, w2, w21
	add	x1, x1, w2, sxtw #1
	ldr	w2, [x23, #84]
	ldrh	w1, [x1, w22, sxtw #1]
	cmp	w2, #10
	b.ne	.LBB19_22
	ldr	w2, [x23, #80]
	sdiv	w1, w1, w2
.LBB19_22:
	ldr	w2, [x23, #92]
	sub	w22, w2, w1
	cmp	w22, w20
	cset	w1, ge
	cmp	w22, w15
	and	w8, w1, w8
	b.eq	.LBB19_26
	tbnz	w22, #31, .LBB19_26
	cmp	w22, w20
	b.ge	.LBB19_26
	ldr	x11, [x19, #1528]
	lsl	w15, w22, #10
	add	x11, x11, w15, sxtw
	mov	w15, w22
.LBB19_26:
	tbnz	w22, #31, .LBB19_17
	cmp	w22, w20
	b.ge	.LBB19_17
	ldr	w1, [x19, #1576]
	cmp	w1, #8
	b.eq	.LBB19_30
	cmp	w1, #2
	b.ne	.LBB19_17
.LBB19_30:
	and	x1, x0, #0xf8f8f8f8f8f8f8f8
	eor	x1, x1, x14
	cbz	x1, .LBB19_17
	ldrb	w2, [x16]
	mov	w3, #3
	ldur	w4, [x29, #-20]
	tst	w1, #0xff
	lsl	w3, w3, w2
	csel	w3, wzr, w3, eq
	tst	x1, #0xff00
	csel	w5, wzr, w4, eq
	ldur	w4, [x29, #-24]
	tst	x1, #0xff0000
	csel	w4, wzr, w4, eq
	tst	x1, #0xff000000
	csel	w7, wzr, w12, eq
	tst	x1, #0xff00000000
	orr	w4, w4, w5
	csel	w5, wzr, w13, eq
	tst	x1, #0xff0000000000
	orr	w4, w4, w7
	csel	w7, wzr, w28, eq
	tst	x1, #0xff000000000000
	orr	w4, w4, w5
	csel	w5, wzr, w26, eq
	orr	w4, w4, w7
	orr	w4, w4, w5
	lsr	x5, x1, #56
	orr	w1, w4, w3
	cbnz	x5, .LBB19_33
	cbnz	w1, .LBB19_34
	b	.LBB19_17
.LBB19_33:
	ldr	w3, [sp, #52]
	orr	w1, w1, w3
.LBB19_34:
	lsl	w3, w0, #2
	lsr	x4, x0, #6
	and	x3, x3, #0x3e0
	and	x4, x4, #0x3e0
	bfxil	x3, x14, #3, #5
	bfxil	x4, x14, #11, #5
	lsr	x5, x0, #14
	ldur	x6, [x29, #-32]
	and	x5, x5, #0x3e0
	lsr	x7, x0, #22
	bfxil	x5, x14, #19, #5
	ldrb	w3, [x11, x3]
	ldrb	w4, [x11, x4]
	and	x7, x7, #0x3e0
	bfxil	x7, x14, #27, #5
	ldrb	w5, [x11, x5]
	lsl	w2, w3, w2
	lsr	x3, x0, #30
	lsl	w4, w4, w6
	ldur	x6, [x29, #-40]
	orr	w2, w4, w2
	lsr	x4, x0, #38
	and	x4, x4, #0x3e0
	ldrb	w7, [x11, x7]
	lsl	w5, w5, w6
	bfxil	x4, x14, #43, #5
	orr	w2, w2, w5
	lsr	x5, x0, #46
	lsr	x0, x0, #54
	and	x3, x3, #0x3e0
	and	x5, x5, #0x3e0
	and	x0, x0, #0x3e0
	bfxil	x3, x14, #35, #5
	bfxil	x5, x14, #51, #5
	bfxil	x0, x14, #59, #5
	ldrb	w14, [x11, x4]
	ldur	x4, [x29, #-48]
	ldrb	w3, [x11, x3]
	ldrb	w5, [x11, x5]
	lsl	w4, w7, w4
	ldrb	w0, [x11, x0]
	orr	w2, w2, w4
	ldur	x4, [x29, #-56]
	lsl	w3, w3, w4
	lsl	w0, w0, w30
	orr	w2, w2, w3
	ldr	x3, [sp, #64]
	lsl	w14, w14, w3
	orr	w14, w2, w14
	ldr	x2, [sp, #56]
	lsl	w2, w5, w2
	orr	w14, w14, w2
	ldr	w2, [x9]
	orr	w14, w14, w0
	and	w14, w14, w1
	orr	w14, w14, w2
	str	w14, [x9]
	b	.LBB19_17
.LBB19_35:
	mov	w8, #1
.LBB19_36:
	ldr	w12, [sp, #12]
	and	w9, w8, #0x1
	cmp	w12, #2
	b.ne	.LBB19_38
	strb	w9, [x23, #105]
	b	.LBB19_39
.LBB19_38:
	strb	w9, [x23, #104]
.LBB19_39:
	ldr	w9, [x19, #1792]
	cmp	w9, #2
	b.lt	.LBB19_165
	adrp	x9, .L.str.111
	adrp	x10, .L.str.110
	add	x9, x9, :lo12:.L.str.111
	add	x10, x10, :lo12:.L.str.110
	cmp	w12, #2
	adrp	x11, .L.str.109
	add	x11, x11, :lo12:.L.str.109
	csel	x9, x10, x9, eq
	cmp	w12, #3
	adrp	x10, .L.str.113
	adrp	x12, .L.str.112
	add	x10, x10, :lo12:.L.str.113
	add	x12, x12, :lo12:.L.str.112
	csel	x2, x11, x9, eq
	tst	w8, #0x1
	adrp	x1, .L.str.115
	ldr	x0, [x19]
	csel	x4, x12, x10, ne
	add	x1, x1, :lo12:.L.str.115
	mov	w3, w22
	bl	_dev_info
	b	.LBB19_165
.LBB19_41:
	ldr	w26, [x23]
	ldr	w9, [x23, #12]
	ldr	x10, [x19, #1464]
	ldr	x11, [x24, #120]
	ldr	x12, [x19, #1456]
	sub	w22, w26, w9
	ldr	x9, [x19, #1528]
	ldr	x27, [x10, #8]
	lsl	w10, w22, #10
	ldr	w8, [x11, #116]
	ldr	x28, [x12, #8]
	add	x20, x9, w10, sxtw
	ldrb	w25, [x23, #77]
	cbz	w8, .LBB19_84
	ldp	w14, w13, [x24, #188]
	ldr	w9, [sp, #12]
	cmp	w13, #0
	lsr	w15, w14, #3
	cinc	w10, w13, lt
	cmp	w9, #2
	asr	w10, w10, #1
	b.ne	.LBB19_86
	mov	w12, wzr
	cmp	w12, w10
	b.lt	.LBB19_87
.LBB19_44:
	mov	w11, #1
.LBB19_45:
	ldr	w12, [sp, #12]
	and	w8, w11, #0x1
	cmp	w12, #2
	b.ne	.LBB19_128
	strb	w8, [x23, #105]
	b	.LBB19_129
.LBB19_47:
	mul	w1, w17, w0
	sbfiz	x1, x1, #2, #32
	add	x13, x13, x1
	add	x14, x14, x1
	mov	w1, w17
	mov	w17, w9
	cmp	w1, w17
	b.ge	.LBB19_6
.LBB19_48:
	lsl	w7, w3, #10
	cmp	w0, #1
	ldrb	w2, [x12, #1]
	csinc	w9, w0, wzr, gt
	ldrb	w4, [x12, #2]
	add	x7, x6, w7, sxtw
	ldrb	w6, [x12, #3]
	mov	w0, #1
	mov	w20, #255
	mov	w21, w3
	stur	w9, [x29, #-24]
	b	.LBB19_50
.LBB19_49:
	add	w1, w1, #1
	cmp	w1, w17
	b.eq	.LBB19_7
.LBB19_50:
	ldr	w22, [x24, #268]
	mov	w25, w1
	cbz	w22, .LBB19_52
	ldr	w22, [x24, #192]
	mvn	w25, w1
	add	w25, w22, w25
.LBB19_52:
	cmp	w11, #4
	b.lo	.LBB19_49
	ldr	x26, [x24, #120]
	mov	w22, wzr
	ldur	w9, [x29, #-20]
	ldr	w26, [x26, #104]
	mul	w25, w25, w9
	ldr	x9, [sp, #40]
	mul	w26, w26, w10
	add	x25, x9, x25
	add	x25, x25, x26
	ldur	w26, [x29, #-24]
	b	.LBB19_56
.LBB19_54:
	mov	w27, #2
	strh	wzr, [x25]
.LBB19_55:
	add	x25, x25, x27
	subs	w26, w26, #1
	add	w22, w22, #8
	b.eq	.LBB19_49
.LBB19_56:
	ldr	w27, [x13]
	ldr	w28, [x14]
	cbz	w15, .LBB19_64
	ldr	x3, [x19, #888]
	cbz	x3, .LBB19_69
	ldr	w30, [x19, #212]
	asr	w9, w22, #1
	asr	w30, w30, #1
	mul	w30, w30, w1
	add	x3, x3, w30, sxtw #1
	ldr	w30, [x23, #84]
	ldrh	w3, [x3, w9, sxtw #1]
	cmp	w30, #10
	b.ne	.LBB19_60
	ldr	w9, [x23, #80]
	sdiv	w3, w3, w9
.LBB19_60:
	ldr	w9, [x23, #92]
	sub	w3, w9, w3
	cmp	w3, w8
	cset	w9, ge
	cmp	w3, w21
	and	w0, w9, w0
	b.eq	.LBB19_64
	tbnz	w3, #31, .LBB19_64
	cmp	w3, w8
	b.ge	.LBB19_64
	ldr	x9, [x19, #1528]
	lsl	w7, w3, #10
	mov	w21, w3
	add	x7, x9, w7, sxtw
.LBB19_64:
	tbnz	w3, #31, .LBB19_70
	cmp	w3, w8
	b.ge	.LBB19_70
	lsl	w9, w28, #2
	cmp	w10, #4
	b.eq	.LBB19_75
	cmp	w10, #2
	b.ne	.LBB19_78
	lsr	x30, x28, #6
	lsr	x5, x28, #14
	and	x9, x9, #0x3e0
	and	x30, x30, #0x3e0
	lsr	x28, x28, #22
	bfxil	x9, x27, #3, #5
	and	x5, x5, #0x3e0
	bfxil	x30, x27, #11, #5
	and	x28, x28, #0x3e0
	bfxil	x5, x27, #19, #5
	orr	x27, x28, x27, lsr #27
	ldrb	w9, [x7, x9]
	ldrb	w30, [x7, x30]
	ldrb	w28, [x12]
	ldrb	w5, [x7, x5]
	ldrb	w27, [x7, x27]
	lsl	w9, w9, w28
	lsl	w28, w30, w2
	orr	w9, w28, w9
	lsl	w5, w5, w4
	lsl	w27, w27, w6
	orr	w9, w9, w5
	orr	w9, w9, w27
	mov	w27, #4
	lsr	w5, w9, #8
	strb	w9, [x25, #2]
	strb	w5, [x25]
	b	.LBB19_55
.LBB19_69:
	cmp	w8, #0
	cset	w3, lt
	and	w0, w3, w0
	mov	w3, #-1
.LBB19_70:
	cmp	w10, #2
	b.eq	.LBB19_74
	cmp	w10, #4
	b.ne	.LBB19_54
	cbz	w16, .LBB19_79
	ldr	x9, [x24, #120]
	ldr	w27, [x25]
	ldp	w9, w5, [x9, #96]
	lsl	w5, w20, w5
	lsl	w9, w20, w9
	orr	w9, w9, w5
	bic	w9, w27, w9
	b	.LBB19_77
.LBB19_74:
	ldrb	w9, [x25, #1]
	mov	w27, #4
	ldrb	w5, [x25, #3]
	lsl	w9, w9, #8
	lsl	w5, w5, #8
	strh	w9, [x25]
	strh	w5, [x25, #2]
	b	.LBB19_55
.LBB19_75:
	lsr	x5, x28, #6
	lsr	x30, x28, #14
	and	x9, x9, #0x3e0
	and	x5, x5, #0x3e0
	lsr	x28, x28, #22
	bfxil	x9, x27, #3, #5
	and	x30, x30, #0x3e0
	bfxil	x5, x27, #11, #5
	and	x28, x28, #0x3e0
	bfxil	x30, x27, #19, #5
	orr	x27, x28, x27, lsr #27
	ldrb	w9, [x7, x9]
	ldrb	w5, [x7, x5]
	ldrb	w28, [x12]
	ldrb	w30, [x7, x30]
	ldrb	w27, [x7, x27]
	lsl	w9, w9, w28
	lsl	w5, w5, w2
	orr	w9, w5, w9
	lsl	w5, w30, w4
	orr	w9, w9, w5
	lsl	w5, w27, w6
	orr	w28, w9, w5
	cbz	w16, .LBB19_80
	ldr	x9, [x24, #120]
	and	w27, w28, #0xff00
	ldr	w5, [x25]
	ldr	w9, [x9, #100]
	lsl	w9, w20, w9
	bic	w9, w5, w9
	str	w9, [x25]
	ldr	x5, [x24, #120]
	ldr	w5, [x5, #100]
	sub	w5, w5, #8
	lsl	w5, w27, w5
	and	w27, w28, #0xff
	orr	w9, w5, w9
	str	w9, [x25]
	ldr	x5, [x24, #120]
	ldr	w5, [x5, #96]
	lsl	w5, w20, w5
	bic	w9, w9, w5
	str	w9, [x25]
	ldr	x5, [x24, #120]
	ldr	w5, [x5, #96]
	lsl	w5, w27, w5
	orr	w9, w5, w9
.LBB19_77:
	mov	w27, #4
	str	w9, [x25]
	b	.LBB19_55
.LBB19_78:
	lsr	x5, x28, #6
	lsr	x30, x28, #14
	and	x9, x9, #0x3e0
	and	x5, x5, #0x3e0
	lsr	x28, x28, #22
	bfxil	x9, x27, #3, #5
	and	x30, x30, #0x3e0
	bfxil	x5, x27, #11, #5
	and	x28, x28, #0x3e0
	bfxil	x30, x27, #19, #5
	orr	x27, x28, x27, lsr #27
	ldrb	w9, [x7, x9]
	ldrb	w5, [x7, x5]
	ldrb	w28, [x12]
	ldrb	w30, [x7, x30]
	ldrb	w27, [x7, x27]
	lsl	w9, w9, w28
	lsl	w5, w5, w2
	orr	w9, w5, w9
	lsl	w5, w30, w4
	orr	w9, w9, w5
	lsl	w5, w27, w6
	orr	w9, w9, w5
	mov	w27, #2
	strh	w9, [x25]
	b	.LBB19_55
.LBB19_79:
	ldp	w9, w5, [x25]
	mov	w27, #8
	and	w9, w9, #0xffffff00
	and	w5, w5, #0xffffff00
	stp	w9, w5, [x25]
	b	.LBB19_55
.LBB19_80:
	lsr	w9, w28, #8
	mov	w27, #8
	strb	w28, [x25, #4]
	strb	w9, [x25]
	b	.LBB19_55
.LBB19_81:
	strb	w8, [x23, #104]
.LBB19_82:
	ldr	w8, [x19, #1792]
	cmp	w8, #2
	b.lt	.LBB19_165
	adrp	x8, .L.str.111
	adrp	x9, .L.str.110
	add	x8, x8, :lo12:.L.str.111
	add	x9, x9, :lo12:.L.str.110
	cmp	w11, #2
	adrp	x10, .L.str.109
	add	x10, x10, :lo12:.L.str.109
	csel	x8, x9, x8, eq
	cmp	w11, #3
	adrp	x9, .L.str.113
	adrp	x11, .L.str.112
	add	x9, x9, :lo12:.L.str.113
	add	x11, x11, :lo12:.L.str.112
	csel	x2, x10, x8, eq
	tst	w0, #0x1
	adrp	x1, .L.str.115
	ldr	x0, [x19]
	csel	x4, x11, x9, ne
	add	x1, x1, :lo12:.L.str.115
	bl	_dev_info
	b	.LBB19_165
.LBB19_84:
	ldp	w10, w8, [x24, #188]
	ldr	w11, [x11, #120]
	cmp	w8, #0
	lsr	w13, w10, #3
	cinc	w9, w8, lt
	stur	w10, [x29, #-24]
	asr	w12, w9, #1
	ldr	w9, [sp, #12]
	stur	w13, [x29, #-40]
	cmp	w9, #2
	b.ne	.LBB19_131
	mov	w21, wzr
	b	.LBB19_132
.LBB19_86:
	mul	w12, w10, w15
	sbfiz	x12, x12, #3, #32
	add	x27, x27, x12
	add	x28, x28, x12
	mov	w12, w10
	mov	w10, w13
	cmp	w12, w10
	b.ge	.LBB19_44
.LBB19_87:
	ldr	w9, [x11, #120]
	cmp	w15, #1
	ldr	w13, [x24, #260]
	mov	w11, #1
	mov	w16, #49152
	mov	w17, #255
	stur	w9, [x29, #-32]
	csinc	w9, w15, wzr, gt
	mov	w0, w22
	stp	w10, w14, [x29, #-24]
	stur	w9, [x29, #-40]
	b	.LBB19_90
.LBB19_88:
	ldp	w10, w14, [x29, #-24]
	add	x28, x28, x1
	add	x27, x27, x1
.LBB19_89:
	add	w12, w12, #1
	cmp	w12, w10
	b.eq	.LBB19_45
.LBB19_90:
	ldr	w1, [x24, #268]
	mov	w2, w12
	cbz	w1, .LBB19_92
	ldr	w1, [x24, #192]
	mvn	w2, w12
	add	w2, w1, w2
.LBB19_92:
	cmp	w14, #8
	b.lo	.LBB19_89
	ldr	x3, [x24, #120]
	mov	x1, xzr
	ldur	w9, [x29, #-32]
	ldr	w3, [x3, #104]
	mul	w2, w2, w9
	ldr	x9, [sp, #40]
	mul	w3, w3, w8
	add	x2, x9, x2
	add	x2, x2, x3
	ldur	w3, [x29, #-40]
	b	.LBB19_96
.LBB19_94:
	mov	w4, #2
	strh	wzr, [x2]
.LBB19_95:
	add	x2, x2, x4
	add	x1, x1, #8
	subs	w3, w3, #1
	b.eq	.LBB19_88
.LBB19_96:
	ldr	x4, [x27, x1]
	ldr	x5, [x28, x1]
	cbz	w25, .LBB19_104
	ldr	x6, [x19, #888]
	cbz	x6, .LBB19_111
	ldr	w7, [x19, #212]
	asr	w21, w1, #1
	asr	w7, w7, #1
	mul	w7, w7, w12
	add	x6, x6, w7, sxtw #1
	ldr	w7, [x23, #84]
	ldrh	w6, [x6, w21, sxtw #1]
	cmp	w7, #10
	b.ne	.LBB19_100
	ldr	w7, [x23, #80]
	sdiv	w6, w6, w7
.LBB19_100:
	ldr	w7, [x23, #92]
	sub	w22, w7, w6
	cmp	w22, w26
	cset	w6, ge
	cmp	w22, w0
	and	w11, w6, w11
	b.eq	.LBB19_104
	tbnz	w22, #31, .LBB19_104
	cmp	w22, w26
	b.ge	.LBB19_104
	ldr	x0, [x19, #1528]
	lsl	w6, w22, #10
	add	x20, x0, w6, sxtw
	mov	w0, w22
.LBB19_104:
	tbnz	w22, #31, .LBB19_112
	cmp	w22, w26
	b.ge	.LBB19_112
	ldr	w6, [x19, #1576]
	cmp	w6, #8
	b.eq	.LBB19_108
	cmp	w6, #2
	b.ne	.LBB19_117
.LBB19_108:
	and	x6, x5, #0xf8f8f8f8f8f8f8f8
	eor	x6, x6, x4
	cbz	x6, .LBB19_112
	tst	w6, #0xff
	lsr	x30, x6, #56
	csel	w7, wzr, w16, eq
	tst	x6, #0xff00
	orr	w21, w7, #0x3000
	csel	w7, w7, w21, eq
	tst	x6, #0xff0000
	orr	w21, w7, #0xc00
	csel	w7, w7, w21, eq
	tst	x6, #0xff000000
	orr	w21, w7, #0x300
	csel	w7, w7, w21, eq
	tst	x6, #0xff00000000
	orr	w21, w7, #0xc0
	csel	w7, w7, w21, eq
	tst	x6, #0xff0000000000
	orr	w21, w7, #0x30
	csel	w7, w7, w21, eq
	tst	x6, #0xff000000000000
	orr	w21, w7, #0xc
	csel	w6, w7, w21, eq
	cbnz	x30, .LBB19_123
	cbnz	w6, .LBB19_118
	b	.LBB19_112
.LBB19_111:
	cmp	w26, #0
	mov	w22, #-1
	cset	w4, lt
	and	w11, w4, w11
.LBB19_112:
	cmp	w8, #4
	b.eq	.LBB19_115
	cmp	w8, #2
	b.ne	.LBB19_94
	ldrb	w9, [x2, #1]
	mov	w4, #4
	ldrb	w10, [x2, #3]
	lsl	w9, w9, #8
	lsl	w10, w10, #8
	strh	w9, [x2]
	strh	w10, [x2, #2]
	b	.LBB19_95
.LBB19_115:
	cbz	w13, .LBB19_121
	ldr	x9, [x24, #120]
	ldr	w14, [x2]
	ldp	w9, w10, [x9, #96]
	lsl	w10, w17, w10
	lsl	w9, w17, w9
	orr	w9, w9, w10
	bic	w9, w14, w9
	b	.LBB19_126
.LBB19_117:
	mov	w6, #65535
.LBB19_118:
	cmp	w8, #4
	b.eq	.LBB19_124
.LBB19_119:
	cmp	w8, #2
	b.ne	.LBB19_122
	lsr	x21, x5, #6
	lsl	w7, w5, #2
	and	x21, x21, #0x3e0
	lsr	x30, x5, #14
	and	x7, x7, #0x3e0
	bfxil	x21, x4, #11, #5
	bfxil	x7, x4, #3, #5
	and	x30, x30, #0x3e0
	lsr	x14, x5, #22
	bfxil	x30, x4, #19, #5
	and	x14, x14, #0x3e0
	lsr	x15, x5, #30
	ldrb	w21, [x20, x21]
	bfxil	x14, x4, #27, #5
	and	x15, x15, #0x3e0
	lsr	x9, x5, #38
	ldrb	w7, [x20, x7]
	bfxil	x15, x4, #35, #5
	and	x9, x9, #0x3e0
	lsr	x10, x5, #46
	ldrb	w30, [x20, x30]
	bfxil	x9, x4, #43, #5
	and	x10, x10, #0x3e0
	lsr	x5, x5, #54
	bfxil	x10, x4, #51, #5
	and	x5, x5, #0x3e0
	ldrb	w14, [x20, x14]
	bfxil	x5, x4, #59, #5
	lsl	w4, w21, #12
	ldrb	w15, [x20, x15]
	orr	w4, w4, w7, lsl #14
	ldrb	w9, [x20, x9]
	orr	w4, w4, w30, lsl #10
	ldrb	w10, [x20, x10]
	orr	w14, w4, w14, lsl #8
	ldrb	w4, [x20, x5]
	orr	w14, w14, w15, lsl #6
	orr	w9, w14, w9, lsl #4
	orr	w9, w9, w10, lsl #2
	orr	w9, w9, w4
	mov	w4, #4
	and	w9, w9, w6
	lsr	w10, w9, #8
	strb	w9, [x2, #2]
	strb	w10, [x2]
	b	.LBB19_95
.LBB19_121:
	ldp	w9, w10, [x2]
	mov	w4, #8
	and	w9, w9, #0xffffff00
	and	w10, w10, #0xffffff00
	stp	w9, w10, [x2]
	b	.LBB19_95
.LBB19_122:
	lsr	x10, x5, #6
	lsl	w9, w5, #2
	and	x10, x10, #0x3e0
	lsr	x14, x5, #14
	and	x9, x9, #0x3e0
	bfxil	x10, x4, #11, #5
	bfxil	x9, x4, #3, #5
	and	x14, x14, #0x3e0
	lsr	x15, x5, #22
	bfxil	x14, x4, #19, #5
	and	x15, x15, #0x3e0
	lsr	x7, x5, #30
	ldrb	w10, [x20, x10]
	bfxil	x15, x4, #27, #5
	and	x7, x7, #0x3e0
	lsr	x21, x5, #38
	ldrb	w9, [x20, x9]
	bfxil	x7, x4, #35, #5
	and	x21, x21, #0x3e0
	lsr	x30, x5, #46
	ldrb	w14, [x20, x14]
	bfxil	x21, x4, #43, #5
	and	x30, x30, #0x3e0
	lsr	x5, x5, #54
	bfxil	x30, x4, #51, #5
	and	x5, x5, #0x3e0
	ldrb	w15, [x20, x15]
	bfxil	x5, x4, #59, #5
	lsl	w10, w10, #12
	ldrb	w4, [x20, x7]
	orr	w9, w10, w9, lsl #14
	ldrb	w10, [x20, x21]
	orr	w9, w9, w14, lsl #10
	ldrb	w14, [x20, x30]
	orr	w9, w9, w15, lsl #8
	ldrb	w15, [x20, x5]
	orr	w9, w9, w4, lsl #6
	mov	w4, #2
	orr	w9, w9, w10, lsl #4
	orr	w9, w9, w14, lsl #2
	orr	w9, w9, w15
	and	w9, w9, w6
	strh	w9, [x2]
	b	.LBB19_95
.LBB19_123:
	orr	w6, w6, #0x3
	cmp	w8, #4
	b.ne	.LBB19_119
.LBB19_124:
	lsr	x10, x5, #6
	lsl	w9, w5, #2
	and	x10, x10, #0x3e0
	lsr	x14, x5, #14
	and	x9, x9, #0x3e0
	bfxil	x10, x4, #11, #5
	bfxil	x9, x4, #3, #5
	and	x14, x14, #0x3e0
	lsr	x15, x5, #22
	bfxil	x14, x4, #19, #5
	and	x15, x15, #0x3e0
	lsr	x7, x5, #30
	ldrb	w10, [x20, x10]
	bfxil	x15, x4, #27, #5
	and	x7, x7, #0x3e0
	lsr	x21, x5, #38
	ldrb	w9, [x20, x9]
	bfxil	x7, x4, #35, #5
	and	x21, x21, #0x3e0
	lsr	x30, x5, #46
	ldrb	w14, [x20, x14]
	bfxil	x21, x4, #43, #5
	and	x30, x30, #0x3e0
	lsr	x5, x5, #54
	bfxil	x30, x4, #51, #5
	and	x5, x5, #0x3e0
	ldrb	w15, [x20, x15]
	bfxil	x5, x4, #59, #5
	lsl	w10, w10, #12
	ldrb	w4, [x20, x7]
	orr	w9, w10, w9, lsl #14
	ldrb	w10, [x20, x21]
	orr	w9, w9, w14, lsl #10
	ldrb	w14, [x20, x30]
	orr	w9, w9, w15, lsl #8
	ldrb	w15, [x20, x5]
	orr	w9, w9, w4, lsl #6
	orr	w9, w9, w10, lsl #4
	orr	w9, w9, w14, lsl #2
	orr	w9, w9, w15
	and	w5, w9, w6
	cbz	w13, .LBB19_127
	ldr	x9, [x24, #120]
	and	w14, w5, #0xff00
	ldr	w10, [x2]
	ldr	w9, [x9, #100]
	lsl	w9, w17, w9
	bic	w9, w10, w9
	str	w9, [x2]
	ldr	x10, [x24, #120]
	ldr	w10, [x10, #100]
	sub	w10, w10, #8
	lsl	w10, w14, w10
	and	w14, w5, #0xff
	orr	w9, w10, w9
	str	w9, [x2]
	ldr	x10, [x24, #120]
	ldr	w10, [x10, #96]
	lsl	w10, w17, w10
	bic	w9, w9, w10
	str	w9, [x2]
	ldr	x10, [x24, #120]
	ldr	w10, [x10, #96]
	lsl	w10, w14, w10
	orr	w9, w10, w9
.LBB19_126:
	mov	w4, #4
	str	w9, [x2]
	b	.LBB19_95
.LBB19_127:
	lsr	w9, w5, #8
	mov	w4, #8
	strb	w5, [x2, #4]
	strb	w9, [x2]
	b	.LBB19_95
.LBB19_128:
	strb	w8, [x23, #104]
.LBB19_129:
	ldr	w8, [x19, #1792]
	cmp	w8, #2
	b.lt	.LBB19_165
	adrp	x8, .L.str.111
	adrp	x9, .L.str.110
	add	x8, x8, :lo12:.L.str.111
	add	x9, x9, :lo12:.L.str.110
	cmp	w12, #2
	adrp	x10, .L.str.109
	add	x10, x10, :lo12:.L.str.109
	csel	x8, x9, x8, eq
	cmp	w12, #3
	adrp	x9, .L.str.112
	adrp	x12, .L.str.113
	add	x9, x9, :lo12:.L.str.112
	add	x12, x12, :lo12:.L.str.113
	csel	x2, x10, x8, eq
	cmp	w25, #0
	ldr	x0, [x19]
	csel	x4, x12, x9, eq
	tst	w11, #0x1
	csel	x5, x9, x12, ne
	b	.LBB19_167
.LBB19_131:
	mul	w9, w12, w13
	mov	w21, w12
	mov	w12, w8
	sbfiz	x9, x9, #3, #32
	add	x27, x27, x9
	add	x28, x28, x9
.LBB19_132:
	subs	w8, w12, w21
	mul	w9, w21, w11
	mov	w1, wzr
	stur	w12, [x29, #-20]
	mul	w2, w8, w11
	ldr	x8, [sp, #40]
	stur	w11, [x29, #-32]
	add	x0, x8, x9
	bl	memset
	ldur	w30, [x29, #-20]
	cmp	w30, w21
	b.le	.LBB19_160
	ldur	w8, [x29, #-40]
	mov	w10, #3
	mov	w11, w22
	ldur	w6, [x29, #-24]
	ldur	w7, [x29, #-32]
	cmp	w8, #1
	csinc	w9, w8, wzr, gt
	mov	w8, #1
	lsl	x9, x9, #3
	b	.LBB19_136
.LBB19_134:
	add	x28, x28, x12
	add	x27, x27, x12
.LBB19_135:
	add	w21, w21, #1
	cmp	w21, w30
	b.eq	.LBB19_161
.LBB19_136:
	ldr	w12, [x24, #268]
	mov	w13, w21
	cbz	w12, .LBB19_138
	ldr	w12, [x24, #192]
	mvn	w13, w21
	add	w13, w12, w13
.LBB19_138:
	cmp	w6, #8
	b.lo	.LBB19_135
	ldr	x14, [sp, #40]
	mul	w13, w13, w7
	mov	x12, xzr
	add	x13, x14, x13
	b	.LBB19_142
.LBB19_140:
	cmp	w26, #0
	mov	w22, #-1
	cset	w14, lt
	and	w8, w14, w8
.LBB19_141:
	add	x13, x13, #2
	add	x12, x12, #8
	cmp	x9, x12
	b.eq	.LBB19_134
.LBB19_142:
	ldr	x14, [x27, x12]
	ldr	x15, [x28, x12]
	cbz	w25, .LBB19_150
	ldr	x16, [x19, #888]
	cbz	x16, .LBB19_140
	ldr	w17, [x19, #212]
	asr	w0, w12, #1
	asr	w17, w17, #1
	mul	w17, w17, w21
	add	x16, x16, w17, sxtw #1
	ldr	w17, [x23, #84]
	ldrh	w16, [x16, w0, sxtw #1]
	cmp	w17, #10
	b.ne	.LBB19_146
	ldr	w17, [x23, #80]
	sdiv	w16, w16, w17
.LBB19_146:
	ldr	w17, [x23, #92]
	sub	w22, w17, w16
	cmp	w22, w26
	cset	w16, ge
	cmp	w22, w11
	and	w8, w16, w8
	b.eq	.LBB19_150
	tbnz	w22, #31, .LBB19_150
	cmp	w22, w26
	b.ge	.LBB19_150
	ldr	x11, [x19, #1528]
	lsl	w16, w22, #10
	add	x20, x11, w16, sxtw
	mov	w11, w22
.LBB19_150:
	tbnz	w22, #31, .LBB19_141
	cmp	w22, w26
	b.ge	.LBB19_141
	ldr	w16, [x19, #1576]
	cmp	w16, #8
	b.eq	.LBB19_154
	cmp	w16, #2
	b.ne	.LBB19_157
.LBB19_154:
	and	x16, x15, #0xf8f8f8f8f8f8f8f8
	eor	x16, x16, x14
	cbz	x16, .LBB19_141
	tst	w16, #0xff
	lsr	x1, x16, #56
	csel	w17, wzr, w10, eq
	tst	x16, #0xff00
	orr	w0, w17, #0xc
	csel	w17, w17, w0, eq
	tst	x16, #0xff0000
	orr	w0, w17, #0x30
	csel	w17, w17, w0, eq
	tst	x16, #0xff000000
	orr	w0, w17, #0xc0
	csel	w17, w17, w0, eq
	tst	x16, #0xff00000000
	orr	w0, w17, #0x300
	csel	w17, w17, w0, eq
	tst	x16, #0xff0000000000
	orr	w0, w17, #0xc00
	csel	w17, w17, w0, eq
	tst	x16, #0xff000000000000
	orr	w0, w17, #0x3000
	csel	w16, w17, w0, eq
	cbnz	x1, .LBB19_158
	cbnz	w16, .LBB19_159
	b	.LBB19_141
.LBB19_157:
	mov	w16, #65535
	b	.LBB19_159
.LBB19_158:
	orr	w16, w16, #0xc000
.LBB19_159:
	lsl	w17, w15, #2
	lsr	x0, x15, #6
	and	x17, x17, #0x3e0
	and	x0, x0, #0x3e0
	lsr	x1, x15, #14
	bfxil	x17, x14, #3, #5
	and	x1, x1, #0x3e0
	bfxil	x0, x14, #11, #5
	lsr	x2, x15, #22
	bfxil	x1, x14, #19, #5
	and	x2, x2, #0x3e0
	lsr	x3, x15, #30
	bfxil	x2, x14, #27, #5
	and	x3, x3, #0x3e0
	lsr	x4, x15, #38
	ldrb	w17, [x20, x17]
	ldrb	w0, [x20, x0]
	bfxil	x3, x14, #35, #5
	and	x4, x4, #0x3e0
	lsr	x5, x15, #46
	ldrb	w1, [x20, x1]
	bfxil	x4, x14, #43, #5
	and	x5, x5, #0x3e0
	lsr	x15, x15, #54
	ldrb	w2, [x20, x2]
	bfxil	x5, x14, #51, #5
	and	x15, x15, #0x3e0
	orr	w17, w17, w0, lsl #2
	bfxil	x15, x14, #59, #5
	ldrb	w14, [x20, x3]
	ldrb	w0, [x20, x4]
	orr	w17, w17, w1, lsl #4
	ldrb	w1, [x20, x5]
	orr	w17, w17, w2, lsl #6
	ldrb	w15, [x20, x15]
	orr	w14, w17, w14, lsl #8
	orr	w14, w14, w0, lsl #10
	orr	w14, w14, w1, lsl #12
	orr	w14, w14, w15, lsl #14
	and	w14, w14, w16
	strh	w14, [x13]
	b	.LBB19_141
.LBB19_160:
	mov	w8, #1
.LBB19_161:
	ldr	w12, [sp, #12]
	and	w9, w8, #0x1
	cmp	w12, #2
	b.ne	.LBB19_163
	strb	w9, [x23, #105]
	b	.LBB19_164
.LBB19_163:
	strb	w9, [x23, #104]
.LBB19_164:
	ldr	w9, [x19, #1792]
	cmp	w9, #2
	b.ge	.LBB19_166
.LBB19_165:
	ldp	x20, x19, [sp, #208]
	ldp	x22, x21, [sp, #192]
	ldp	x24, x23, [sp, #176]
	ldp	x26, x25, [sp, #160]
	ldp	x28, x27, [sp, #144]
	ldp	x29, x30, [sp, #128]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #224
	ret
.LBB19_166:
	adrp	x9, .L.str.111
	adrp	x10, .L.str.110
	add	x9, x9, :lo12:.L.str.111
	add	x10, x10, :lo12:.L.str.110
	cmp	w12, #2
	adrp	x11, .L.str.109
	add	x11, x11, :lo12:.L.str.109
	csel	x9, x10, x9, eq
	cmp	w12, #3
	adrp	x10, .L.str.112
	adrp	x12, .L.str.113
	add	x10, x10, :lo12:.L.str.112
	add	x12, x12, :lo12:.L.str.113
	csel	x2, x11, x9, eq
	cmp	w25, #0
	ldr	x0, [x19]
	csel	x4, x12, x10, eq
	tst	w8, #0x1
	csel	x5, x10, x12, ne
.LBB19_167:
	adrp	x1, .L.str.116
	mov	w3, w22
	add	x1, x1, :lo12:.L.str.116
	bl	_dev_info
	b	.LBB19_165
.Lfunc_end19:
	.size	get_full_or_part_data, .Lfunc_end19-get_full_or_part_data

	.p2align	2
	.type	ebc_dump_debug_buffers,@function
ebc_dump_debug_buffers:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-64]!
	stp	x28, x23, [sp, #16]
	mov	x29, sp
	stp	x22, x21, [sp, #32]
	stp	x20, x19, [sp, #48]
	sub	sp, sp, #1040
	mrs	x8, SP_EL0
	adrp	x9, global_ebc
	ldr	x8, [x8, #1584]
	mov	w21, w1
	mov	x19, x0
	mov	w1, wzr
	mov	w2, #1024
	stur	x8, [x29, #-8]
	mov	w8, #18232
	add	x23, x0, x8
	ldr	w20, [x0, #216]
	add	x0, sp, #8
	ldr	x22, [x9, :lo12:global_ebc]
	bl	memset
	mov	w9, #18329
	mov	w10, #18330
	mov	w8, wzr
	mov	w11, #18331
	ldrb	w9, [x19, x9]
	ldrb	w10, [x19, x10]
	orr	w9, w10, w9
	tst	w9, #0xff
	strb	w9, [x19, x11]
	b.ne	.LBB20_3
	ldrb	w8, [x19, #1782]
	cbz	w8, .LBB20_30
.LBB20_2:
	mov	w8, wzr
.LBB20_3:
	cbz	w21, .LBB20_28
	tbnz	w8, #0, .LBB20_28
	ldr	w8, [x23, #100]
	and	w9, w21, w8
	tbz	w9, #0, .LBB20_8
	ldr	x9, [x19, #1456]
	ldr	x9, [x9, #8]
	cbz	x9, .LBB20_8
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.130
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.130
	mov	w1, #1024
	bl	snprintf
	ldr	x8, [x19, #1456]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	ldr	x1, [x8, #8]
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_8:
	and	w9, w21, w8
	tbz	w9, #1, .LBB20_11
	ldr	x9, [x19, #1464]
	ldr	x9, [x9, #8]
	cbz	x9, .LBB20_11
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.131
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.131
	mov	w1, #1024
	bl	snprintf
	ldr	x8, [x19, #1464]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	ldr	x1, [x8, #8]
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_11:
	and	w9, w21, w8
	tbz	w9, #2, .LBB20_14
	ldr	x9, [x19, #1032]
	cbz	x9, .LBB20_14
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.132
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.132
	mov	w1, #1024
	bl	snprintf
	ldr	x1, [x19, #1032]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_14:
	and	w9, w21, w8
	tbz	w9, #3, .LBB20_17
	ldr	x9, [x19, #960]
	cbz	x9, .LBB20_17
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.133
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.133
	mov	w1, #1024
	bl	snprintf
	ldr	x1, [x19, #960]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_17:
	and	w9, w21, w8
	tbz	w9, #4, .LBB20_20
	ldr	x9, [x19, #1176]
	cbz	x9, .LBB20_20
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.134
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.134
	mov	w1, #1024
	bl	snprintf
	ldr	x1, [x19, #1176]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_20:
	and	w9, w21, w8
	tbz	w9, #5, .LBB20_23
	ldr	x9, [x19, #1104]
	cbz	x9, .LBB20_23
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.135
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.135
	mov	w1, #1024
	bl	snprintf
	ldr	x1, [x19, #1104]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_23:
	and	w9, w21, w8
	tbz	w9, #6, .LBB20_26
	ldr	x9, [x19, #1248]
	cbz	x9, .LBB20_26
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.136
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.136
	mov	w1, #1024
	bl	snprintf
	ldr	x1, [x19, #1248]
	add	x0, sp, #8
	ldrsw	x2, [x19, #200]
	mov	w3, w20
	bl	write_buffer_to_file
	ldr	w8, [x23, #100]
.LBB20_26:
	and	w8, w21, w8
	tbz	w8, #7, .LBB20_28
	ldp	w3, w4, [x22, #188]
	adrp	x2, .L.str.137
	add	x0, sp, #8
	add	x2, x2, :lo12:.L.str.137
	mov	w1, #1024
	bl	snprintf
	ldrsw	x8, [x22, #184]
	add	x0, sp, #8
	ldrsw	x2, [x19, #204]
	mov	w3, w20
	add	x8, x19, x8, lsl #3
	ldr	x1, [x8, #1544]
	bl	write_buffer_to_file
.LBB20_28:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB20_38
	add	sp, sp, #1040
	ldp	x20, x19, [sp, #48]
	ldp	x22, x21, [sp, #32]
	ldp	x28, x23, [sp, #16]
	ldp	x29, x30, [sp], #64
	ldr	x30, [x18, #-8]!
	ret
.LBB20_30:
	ldrb	w8, [x23, #65]
	cbz	w8, .LBB20_35
	ldrb	w8, [x23, #92]
	cbz	w8, .LBB20_34
	ldrb	w8, [x23, #93]
	cbz	w8, .LBB20_34
	mov	w8, #1
	strb	w8, [x23, #94]
.LBB20_34:
	ldrb	w8, [x23, #94]
	cbnz	w8, .LBB20_36
	b	.LBB20_3
.LBB20_35:
	ldr	w8, [x23]
	cmp	w8, #0
	b.gt	.LBB20_2
.LBB20_36:
	ldrb	w8, [x19, #1780]
	cbnz	w8, .LBB20_2
	ldrb	w8, [x19, #1781]
	cmp	w8, #0
	cset	w8, eq
	b	.LBB20_3
.LBB20_38:
	bl	__stack_chk_fail
.Lfunc_end20:
	.size	ebc_dump_debug_buffers, .Lfunc_end20-ebc_dump_debug_buffers

	.p2align	2
	.type	ebc_power_on,@function
ebc_power_on:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	adrp	x8, global_ebc
	str	x21, [sp, #16]
	stp	x20, x19, [sp, #32]
	mov	x29, sp
	ldr	x21, [x8, :lo12:global_ebc]
	add	x0, x21, #320
	bl	cancel_delayed_work_sync
	ldr	w8, [x21, #1876]
	cmp	w8, #1
	b.eq	.LBB21_9
	ldr	w8, [x21, #2092]
	cbz	w8, .LBB21_3
	mov	w0, #1
	bl	ebc_notify
.LBB21_3:
	ldr	w8, [x21, #2080]
	cbnz	w8, .LBB21_5
	mov	w8, #1
	add	x0, x21, #1888
	str	w8, [x21, #2080]
	bl	__pm_stay_awake
.LBB21_5:
	mov	w8, #1
	ldr	x0, [x21, #16]
	add	x19, x21, #112
	str	w8, [x21, #1876]
	cbz	x0, .LBB21_10
	ldr	x8, [x0, #32]
	mov	w1, #1
	blr	x8
	mov	w20, wzr
	ldrb	w8, [x21, #296]
	cbz	w8, .LBB21_8
.LBB21_7:
	mov	w0, #50
	bl	msleep
.LBB21_8:
	ldr	x0, [x21, #8]
	mov	x1, x19
	ldr	x8, [x0, #168]
	blr	x8
	ldr	w8, [x21, #2096]
	cmp	w8, #1
	b.ge	.LBB21_11
.LBB21_9:
	ldp	x20, x19, [sp, #32]
	ldr	x21, [sp, #16]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.LBB21_10:
	ldr	x0, [x21, #24]
	bl	regulator_enable
	mov	w20, w0
	ldrb	w8, [x21, #296]
	cbnz	w8, .LBB21_7
	b	.LBB21_8
.LBB21_11:
	adrp	x1, .L.str.143
	adrp	x2, .L.str.144
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.143
	add	x2, x2, :lo12:.L.str.144
	mov	w3, w20
	bl	_dev_info
	b	.LBB21_9
.Lfunc_end21:
	.size	ebc_power_on, .Lfunc_end21-ebc_power_on

	.p2align	2
	.type	ebc_lut_update,@function
ebc_lut_update:
	sub	sp, sp, #112
	str	x30, [x18], #8
	stp	x29, x30, [sp, #48]
	add	x29, sp, #48
	stp	x24, x23, [sp, #64]
	mov	w9, #18540
	stp	x22, x21, [sp, #80]
	mov	w21, w1
	stp	x20, x19, [sp, #96]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	w20, w2
	mov	x19, x0
	mov	w24, #18512
	stur	x8, [x29, #-8]
	mov	w8, #25
	ldr	w3, [x0, x9]
	stur	w8, [x29, #-12]
	tbnz	w3, #31, .LBB22_5
	ldr	w8, [x19, #2096]
	stur	w3, [x29, #-12]
	tbz	w8, #31, .LBB22_29
.LBB22_2:
	cmp	w3, #51
	b.lo	.LBB22_12
	ldr	w8, [x19, #2096]
	tbz	w8, #31, .LBB22_33
.LBB22_4:
	mov	w3, #50
	stur	w3, [x29, #-12]
	b	.LBB22_12
.LBB22_5:
	ldr	x0, [x19, #16]
	cbz	x0, .LBB22_10
	ldr	x8, [x0, #56]
	sub	x1, x29, #12
	blr	x8
	mov	w8, w0
	cbnz	w8, .LBB22_11
.LBB22_7:
	ldur	w3, [x29, #-12]
	tbz	w3, #31, .LBB22_2
.LBB22_8:
	ldr	w8, [x19, #2096]
	tbz	w8, #31, .LBB22_37
.LBB22_9:
	mov	w3, wzr
	stur	wzr, [x29, #-12]
	b	.LBB22_12
.LBB22_10:
	ldr	x0, [x19, #104]
	sub	x1, x29, #12
	bl	thermal_zone_get_temp
	ldursw	x8, [x29, #-12]
	mov	w9, #19923
	movk	w9, #4194, lsl #16
	mul	x8, x8, x9
	lsr	x9, x8, #63
	asr	x8, x8, #38
	add	w9, w8, w9
	mov	w8, w0
	stur	w9, [x29, #-12]
	cbz	w8, .LBB22_7
.LBB22_11:
	mov	w3, #25
	ldr	w9, [x19, #2096]
	stur	w3, [x29, #-12]
	tbz	w9, #31, .LBB22_36
.LBB22_12:
	mov	w0, w3
	bl	ebc_overlay_temp_fix
	mov	w22, w0
	ldur	w1, [x29, #-12]
	mov	w0, w21
	bl	ebc_normal_temp_fix
	mov	w23, w0
	and	w3, w22, #0xffff
	cmp	w3, #51
	b.lo	.LBB22_15
	ldr	w8, [x19, #2096]
	cmp	w8, #1
	b.ge	.LBB22_30
.LBB22_14:
	ldur	w22, [x29, #-12]
.LBB22_15:
	add	x24, x19, x24
	and	w3, w23, #0xffff
	cmp	w3, #51
	b.lo	.LBB22_18
	ldr	w8, [x19, #2096]
	cmp	w8, #1
	b.ge	.LBB22_31
.LBB22_17:
	ldur	w23, [x29, #-12]
.LBB22_18:
	ldr	w8, [x19, #2096]
	cmp	w8, #1
	b.ge	.LBB22_28
.LBB22_19:
	ldr	w10, [x19, #2108]
	mov	w9, #32
	ldrb	w8, [x24, #32]
	cmp	w21, #16
	ldp	w11, w12, [x24, #80]
	cset	w13, eq
	cmp	w10, #0
	orr	w2, w23, w22, lsl #8
	stp	w9, w8, [sp, #8]
	cset	w8, eq
	ldr	w9, [x24, #60]
	add	x0, x19, #1808
	add	x3, sp, #8
	mov	w1, w20
	stp	w11, w12, [sp, #16]
	stp	w13, w8, [sp, #24]
	str	w9, [sp, #32]
	bl	epd_lut_get
	tbnz	w0, #31, .LBB22_25
	mov	w20, w0
	tbz	w0, #1, .LBB22_22
	ldr	w8, [x19, #2096]
	cmp	w8, #1
	b.ge	.LBB22_34
.LBB22_22:
	tbz	w20, #0, .LBB22_26
.LBB22_23:
	ldr	w8, [x19, #2096]
	cmp	w8, #1
	b.ge	.LBB22_35
.LBB22_24:
	mov	w8, #1
	strb	w8, [x24]
	b	.LBB22_26
.LBB22_25:
	ldr	w8, [x19, #2096]
	cmp	w8, #1
	b.ge	.LBB22_32
.LBB22_26:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB22_38
	ldp	x20, x19, [sp, #96]
	ldp	x22, x21, [sp, #80]
	ldp	x24, x23, [sp, #64]
	ldp	x29, x30, [sp, #48]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #112
	ret
.LBB22_28:
	adrp	x1, .L.str.162
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	and	w3, w22, #0xffff
	and	w4, w23, #0xffff
	ldur	w5, [x29, #-12]
	add	x1, x1, :lo12:.L.str.162
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_19
.LBB22_29:
	adrp	x1, .L.str.156
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.156
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	ldur	w3, [x29, #-12]
	tbz	w3, #31, .LBB22_2
	b	.LBB22_8
.LBB22_30:
	adrp	x1, .L.str.160
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.160
	ldur	w4, [x29, #-12]
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_14
.LBB22_31:
	adrp	x1, .L.str.161
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.161
	ldur	w4, [x29, #-12]
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_17
.LBB22_32:
	adrp	x1, .L.str.163
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.163
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_26
.LBB22_33:
	adrp	x1, .L.str.158
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.158
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_4
.LBB22_34:
	adrp	x1, .L.str.164
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.164
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	tbnz	w20, #0, .LBB22_23
	b	.LBB22_26
.LBB22_35:
	adrp	x1, .L.str.165
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.165
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_24
.LBB22_36:
	adrp	x1, .L.str.159
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.159
	add	x2, x2, :lo12:.L.str.118
	mov	w3, w8
	bl	_dev_info
	ldur	w3, [x29, #-12]
	b	.LBB22_12
.LBB22_37:
	adrp	x1, .L.str.157
	adrp	x2, .L.str.118
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.157
	add	x2, x2, :lo12:.L.str.118
	bl	_dev_info
	b	.LBB22_9
.LBB22_38:
	bl	__stack_chk_fail
.Lfunc_end22:
	.size	ebc_lut_update, .Lfunc_end22-ebc_lut_update

	.p2align	2
	.type	fast_update_to_auto_image,@function
fast_update_to_auto_image:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-96]!
	adrp	x8, global_ebc
	stp	x28, x27, [sp, #16]
	stp	x26, x25, [sp, #32]
	mov	x29, sp
	stp	x24, x23, [sp, #48]
	ldr	x11, [x8, :lo12:global_ebc]
	stp	x22, x21, [sp, #64]
	stp	x20, x19, [sp, #80]
	ldr	w8, [x11, #192]
	subs	w9, w8, #1
	b.lt	.LBB23_40
	mov	w12, #18220
	ldrsw	x11, [x11, #188]
	add	x14, x0, x12
	ldr	x7, [x0, #1480]
	ldr	x20, [x0, #1464]
	mov	w4, #1
	sub	w1, w11, #1
	mov	w10, wzr
	ldr	w3, [x14]
	neg	x15, x11
	ldr	x12, [x0, #1248]
	add	x2, x1, #1
	ldr	x13, [x0, #1176]
	sub	x4, x4, x11
	ldr	w14, [x14, #40]
	sub	w3, w3, #1
	ldr	x17, [x0, #1104]
	add	x6, x11, #1
	ldr	x16, [x0, #1032]
	mov	w19, #5
	ldr	x5, [x0, #960]
	mov	w21, #10
	ldr	x7, [x7, #8]
	ldr	x20, [x20, #8]
	b	.LBB23_5
.LBB23_2:
	add	x17, x17, x23
	add	x12, x12, x23
	add	x13, x13, x23
	add	x17, x17, #1
	add	x16, x16, x23
	add	x5, x5, x23
	add	x20, x20, x23
.LBB23_3:
	add	x7, x7, x2
.LBB23_4:
	add	w10, w10, #1
	cmp	w10, w8
	b.eq	.LBB23_40
.LBB23_5:
	cmp	w11, #1
	b.lt	.LBB23_4
	ldrb	w22, [x20]
	ldrb	w23, [x5]
	cmp	w22, w23
	b.eq	.LBB23_18
	ldrb	w24, [x7]
	cmp	w24, #255
	b.ne	.LBB23_18
	eor	w22, w23, w22
	cmp	w22, #32
	b.lo	.LBB23_15
	ldrb	w22, [x17]
	orr	w22, w22, w19
	strb	w22, [x17]
	cbz	w10, .LBB23_11
	ldrb	w22, [x17, x15]
	orr	w22, w22, w21
	strb	w22, [x17, x15]
.LBB23_11:
	cmp	w10, w9
	b.ge	.LBB23_13
	ldrb	w22, [x17, x11]
	orr	w22, w22, w21
	strb	w22, [x17, x11]
.LBB23_13:
	cbz	w1, .LBB23_15
	ldrb	w22, [x17, #1]
	orr	w22, w22, w21
	strb	w22, [x17, #1]
.LBB23_15:
	ldrb	w22, [x5]
	strb	w22, [x16]
	ldrb	w22, [x20]
	strb	w22, [x5]
	ldrb	w22, [x12]
	cbz	w22, .LBB23_37
.LBB23_16:
	mov	w22, wzr
	strb	wzr, [x12]
.LBB23_17:
	strb	w22, [x13]
.LBB23_18:
	add	x20, x20, #1
	add	x5, x5, #1
	add	x16, x16, #1
	add	x13, x13, #1
	add	x12, x12, #1
	cmp	w11, #1
	b.ne	.LBB23_20
	add	x17, x17, #1
	b	.LBB23_3
.LBB23_20:
	mov	x26, xzr
	add	x22, x17, x4
	add	x24, x17, x6
	add	x25, x7, #1
	b	.LBB23_25
.LBB23_21:
	ldrb	w27, [x5, x26]
	strb	w27, [x16, x26]
	ldrb	w27, [x20, x26]
	strb	w27, [x5, x26]
	ldrb	w27, [x12, x26]
	cbz	w27, .LBB23_34
.LBB23_22:
	mov	w27, wzr
	strb	wzr, [x12, x26]
.LBB23_23:
	strb	w27, [x13, x26]
.LBB23_24:
	mov	x26, x23
	cmp	w1, w23
	b.eq	.LBB23_2
.LBB23_25:
	ldrb	w27, [x20, x26]
	add	x23, x26, #1
	ldrb	w28, [x5, x26]
	cmp	w27, w28
	b.eq	.LBB23_24
	ldrb	w30, [x25, x26]
	cmp	w30, #255
	b.ne	.LBB23_24
	eor	w27, w28, w27
	cmp	w27, #32
	b.lo	.LBB23_21
	add	x27, x17, x26
	ldrb	w28, [x27, #1]
	orr	w28, w28, w19
	strb	w28, [x27, #1]
	cbz	w10, .LBB23_30
	ldrb	w27, [x22, x26]
	orr	w27, w27, w21
	strb	w27, [x22, x26]
.LBB23_30:
	cmp	w10, w9
	b.ge	.LBB23_32
	ldrb	w27, [x24, x26]
	orr	w27, w27, w21
	strb	w27, [x24, x26]
.LBB23_32:
	ldrb	w27, [x17, x26]
	cmp	w23, w1
	orr	w27, w27, w21
	strb	w27, [x17, x26]
	b.ge	.LBB23_21
	add	x27, x17, x26
	ldrb	w28, [x27, #2]
	orr	w28, w28, w21
	strb	w28, [x27, #2]
	b	.LBB23_21
.LBB23_34:
	ldrb	w27, [x13, x26]
	cbz	w27, .LBB23_22
	cmp	w3, w27
	b.le	.LBB23_22
	add	w27, w14, w27
	sub	w27, w3, w27
	bic	w27, w27, w27, asr #31
	b	.LBB23_23
.LBB23_37:
	ldrb	w22, [x13]
	cbz	w22, .LBB23_16
	cmp	w3, w22
	b.le	.LBB23_16
	add	w22, w14, w22
	sub	w22, w3, w22
	bic	w22, w22, w22, asr #31
	b	.LBB23_17
.LBB23_40:
	ldr	w8, [x0, #1792]
	cmp	w8, #2
	b.ge	.LBB23_42
.LBB23_41:
	ldp	x20, x19, [sp, #80]
	ldp	x22, x21, [sp, #64]
	ldp	x24, x23, [sp, #48]
	ldp	x26, x25, [sp, #32]
	ldp	x28, x27, [sp, #16]
	ldp	x29, x30, [sp], #96
	ldr	x30, [x18, #-8]!
	ret
.LBB23_42:
	adrp	x1, .L.str.173
	adrp	x2, .L.str.152
	ldr	x0, [x0]
	add	x1, x1, :lo12:.L.str.173
	add	x2, x2, :lo12:.L.str.152
	bl	_dev_info
	b	.LBB23_41
.Lfunc_end23:
	.size	fast_update_to_auto_image, .Lfunc_end23-fast_update_to_auto_image

	.p2align	2
	.type	write_buffer_to_file,@function
write_buffer_to_file:
	sub	sp, sp, #80
	str	x30, [x18], #8
	stp	x29, x30, [sp, #32]
	add	x29, sp, #32
	stp	x22, x21, [sp, #48]
	stp	x20, x19, [sp, #64]
	mrs	x8, SP_EL0
	mov	x19, x2
	ldr	x8, [x8, #1584]
	cmp	x2, #3
	stur	x8, [x29, #-8]
	stur	w3, [x29, #-12]
	str	xzr, [sp, #8]
	b.hi	.LBB24_2
	adrp	x0, .L.str.138
	mov	x1, x19
	add	x0, x0, :lo12:.L.str.138
	bl	_printk
	b	.LBB24_10
.LBB24_2:
	mov	x22, x1
	mov	w1, #1089
	mov	w2, #420
	mov	x20, x0
	bl	filp_open
	cmn	x0, #4095
	b.lo	.LBB24_4
	adrp	x0, .L.str.139
	mov	x1, x20
	add	x0, x0, :lo12:.L.str.139
	bl	_printk
	b	.LBB24_10
.LBB24_4:
	sub	x2, x19, #4
	add	x3, sp, #8
	mov	x1, x22
	mov	x21, x0
	bl	kernel_write
	tbz	x0, #63, .LBB24_6
	mov	x1, x0
	adrp	x0, .L.str.140
	add	x0, x0, :lo12:.L.str.140
	b	.LBB24_8
.LBB24_6:
	sub	x1, x29, #12
	add	x3, sp, #8
	mov	x0, x21
	mov	w2, #4
	bl	kernel_write
	tbz	x0, #63, .LBB24_9
	mov	x1, x0
	adrp	x0, .L.str.141
	add	x0, x0, :lo12:.L.str.141
.LBB24_8:
	bl	_printk
	mov	x0, x21
	mov	x1, xzr
	bl	filp_close
	b	.LBB24_10
.LBB24_9:
	mov	x0, x21
	mov	x1, xzr
	bl	filp_close
	adrp	x0, .L.str.142
	mov	x1, x19
	add	x0, x0, :lo12:.L.str.142
	mov	x2, x20
	bl	_printk
.LBB24_10:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB24_12
	ldp	x20, x19, [sp, #64]
	ldp	x22, x21, [sp, #48]
	ldp	x29, x30, [sp, #32]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #80
	ret
.LBB24_12:
	bl	__stack_chk_fail
.Lfunc_end24:
	.size	write_buffer_to_file, .Lfunc_end24-write_buffer_to_file

	.p2align	2
	.type	ebc_logo_free_reserved_area,@function
ebc_logo_free_reserved_area:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-96]!
	add	x8, x1, #4095
	stp	x22, x21, [sp, #64]
	stp	x20, x19, [sp, #80]
	and	x20, x0, #0xfffffffffffff000
	and	x21, x8, #0xfffffffffffff000
	str	x27, [sp, #16]
	cmp	x20, x21
	stp	x26, x25, [sp, #32]
	stp	x24, x23, [sp, #48]
	mov	x29, sp
	b.hs	.LBB25_13
	adrp	x22, memstart_addr
	lsr	x8, x0, #6
	mov	x9, #-8589934592
	and	x8, x8, #0x3ffffffffffffc0
	mov	x23, #281474976706560
	adrp	x24, mem_section
	add	x25, x8, x9
	ldr	x8, [x22, :lo12:memstart_addr]
	mov	w26, #4096
	mov	w27, #1
	b	.LBB25_5
.Ltmp19:
.LBB25_2:
	add	x8, x25, x8
	//APP
	// atomic64_andnot
	prfm	pstl1strm, [x8]
.Ltmp20:
	ldxr	x9, [x8]
	bic	x9, x9, x26
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp20
	//NO_APP
.LBB25_3:
	mov	x0, x19
	mov	w1, wzr
	str	w27, [x19, #52]
	bl	__free_pages
	mov	x0, x19
	mov	w1, #1
	bl	adjust_managed_page_count
	ldr	x8, [x22, :lo12:memstart_addr]
.LBB25_4:
	add	x20, x20, #1, lsl #12
	add	x25, x25, #64
	cmp	x20, x21
	b.hs	.LBB25_13
.LBB25_5:
	cmp	x20, x23
	b.hi	.LBB25_4
	ldr	x9, [x24, :lo12:mem_section]
	cbz	x9, .LBB25_4
	lsr	x10, x20, #32
	and	x10, x10, #0xfffffff8
	ldr	x9, [x9, x10]
	lsr	x10, x20, #27
	cmp	x9, #0
	add	x10, x9, w10, uxtb #4
	ccmp	x10, #0, #4, ne
	b.eq	.LBB25_4
	ldr	x10, [x10]
	tbz	w10, #1, .LBB25_4
	tbnz	w10, #3, .LBB25_11
	ubfx	x10, x20, #27, #8
	add	x9, x9, x10, lsl #4
	lsr	x10, x20, #21
	ldr	x9, [x9, #8]
	ldr	x9, [x9]
	lsr	x9, x9, x10
	tbz	w9, #0, .LBB25_4
.LBB25_11:
	asr	x8, x8, #12
	sub	x19, x25, x8, lsl #6
	neg	x8, x8, lsl #6
	//APP
.Ltmp21:
	b	.Ltmp19
.Ltmp22:
	.section	.altinstructions,"a",@progbits
.Ltmp23:
	.word	.Ltmp21-.Ltmp23
.Ltmp24:
	.word	alt_cb_patch_nops-.Ltmp24
	.hword	32794
	.byte	.Ltmp22-.Ltmp21
	.byte	.Ltmp25-.Ltmp26
	.text

.Ltmp26:
.Ltmp25:

	//NO_APP
	add	x8, x25, x8
	//APP
	stclr	x26, [x8]

	//NO_APP
	b	.LBB25_3
.LBB25_13:
	ldp	x20, x19, [sp, #80]
	ldp	x22, x21, [sp, #64]
	ldp	x24, x23, [sp, #48]
	ldp	x26, x25, [sp, #32]
	ldr	x27, [sp, #16]
	ldp	x29, x30, [sp], #96
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end25:
	.size	ebc_logo_free_reserved_area, .Lfunc_end25-ebc_logo_free_reserved_area

	.p2align	2
	.type	ebc_io_ctl,@function
ebc_io_ctl:
	sub	sp, sp, #432
	str	x30, [x18], #8
	stp	x29, x30, [sp, #352]
	add	x29, sp, #352
	stp	x28, x25, [sp, #368]
	stp	x24, x23, [sp, #384]
	stp	x22, x21, [sp, #400]
	stp	x20, x19, [sp, #416]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	adrp	x20, global_ebc
	stur	x8, [x29, #-8]
	sub	w8, w1, #7, lsl #12
	ldr	x23, [x0, #200]
	cmp	w8, #37
	ldr	x21, [x20, :lo12:global_ebc]
	stur	wzr, [x29, #-16]
	stp	xzr, xzr, [x29, #-32]
	stp	xzr, xzr, [x29, #-48]
	stp	xzr, xzr, [x29, #-64]
	stp	xzr, xzr, [x29, #-80]
	b.hi	.LBB26_113
	mov	w9, #18516
	add	x22, x21, x9
	adrp	x9, .LJTI26_0
	add	x9, x9, :lo12:.LJTI26_0
	adr	x10, .LBB26_2
	ldrh	w11, [x9, x8, lsl #1]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB26_2:
	mov	x19, x2
	cbz	x2, .LBB26_28
	cmp	w8, #37
	b.hi	.LBB26_113
	adrp	x9, .LJTI26_1
	add	x9, x9, :lo12:.LJTI26_1
	adr	x10, .LBB26_5
	ldrh	w11, [x9, x8, lsl #1]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB26_5:
	ldr	w8, [x21, #2096]
	cmp	w8, #2
	b.ge	.LBB26_120
.LBB26_6:
	sub	x0, x29, #80
	mov	x1, x19
	mov	w2, #68
	sub	x20, x29, #80
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	add	x0, x20, #48
	bl	ebc_empty_buf_get
	cbz	x0, .LBB26_123
	ldp	w10, w9, [x21, #188]
	ldrsw	x8, [x0, #4]
	stur	wzr, [x29, #-36]
	ldr	w11, [x21, #264]
	mov	x20, x0
	sub	x1, x29, #80
	mov	x0, x19
	stur	w8, [x29, #-80]
	mov	w2, #68
	stp	w9, w10, [x29, #-72]
	stur	w11, [x29, #-64]
	ldr	x9, [x23]
	ldr	w8, [x9, x8, lsl #2]
	stur	w8, [x29, #-16]
	bl	_copy_to_user
	cbz	w0, .LBB26_114
	mov	x0, x20
	bl	ebc_buf_release
	mov	x0, #-14
	b	.LBB26_116
.LBB26_10:
	bl	ebc_overlay_enable_control
	cbz	w0, .LBB26_12
	tbz	w0, #0, .LBB26_13
.LBB26_12:
	ldrb	w8, [x22, #116]
	orr	w8, w8, #0x1
	strb	w8, [x22, #116]
.LBB26_13:
	ldr	w8, [x21, #2096]
	tbnz	w8, #31, .LBB26_114
	adrp	x1, .L.str.218
	ldr	x0, [x21, #304]
	ldrb	w3, [x22, #116]
	add	x1, x1, :lo12:.L.str.218
	b	.LBB26_20
.LBB26_15:
	bl	ebc_overlay_enable_control
	cbz	w0, .LBB26_17
	tbz	w0, #0, .LBB26_18
.LBB26_17:
	ldrb	w8, [x22, #116]
	and	w8, w8, #0xfe
	strb	w8, [x22, #116]
.LBB26_18:
	ldr	w8, [x21, #2096]
	tbnz	w8, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.219
	ldrb	w3, [x22, #116]
	add	x1, x1, :lo12:.L.str.219
.LBB26_20:
	adrp	x2, .L.str.201
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	b	.LBB26_114
.LBB26_21:
	bl	ebc_drop_one_dsp_buf
	cbnz	w0, .LBB26_103
	b	.LBB26_114
.LBB26_22:
	ldr	w8, [x21, #2096]
	tbz	w8, #31, .LBB26_118
.LBB26_23:
	mov	w0, #2
	bl	ebc_notify
	mov	w8, #1
	mov	x0, xzr
	str	w8, [x21, #2120]
	b	.LBB26_115
.LBB26_24:
	ldr	w8, [x21, #2096]
	tbz	w8, #31, .LBB26_119
	ldr	w8, [x21, #2120]
	cbz	w8, .LBB26_27
.LBB26_26:
	mov	w0, #3
	bl	ebc_notify
.LBB26_27:
	mov	x0, xzr
	str	wzr, [x21, #2120]
	b	.LBB26_115
.LBB26_28:
	adrp	x1, .L.str.198
	adrp	x2, .L__func__.ebc_io_ctl
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.198
	add	x2, x2, :lo12:.L__func__.ebc_io_ctl
	bl	_dev_err
	mov	x0, #-14
	b	.LBB26_116
.LBB26_29:
	ldr	w8, [x21, #2096]
	cmp	w8, #2
	b.ge	.LBB26_121
.LBB26_30:
	sub	x0, x29, #80
	mov	x1, x19
	mov	w2, #68
	bl	_copy_from_user
	cbnz	x0, .LBB26_103
	ldur	w0, [x29, #-80]
	bl	ebc_find_buf_by_id
	cbz	x0, .LBB26_115
	ldur	w8, [x29, #-76]
	ldp	w9, w11, [x29, #-60]
	str	w8, [x0, #32]
	ldp	w10, w12, [x29, #-52]
	ldur	w8, [x29, #-36]
	stp	w9, w11, [x0, #40]
	stp	w12, w8, [x0, #52]
	str	w10, [x0, #48]
	bl	ebc_add_to_dsp_buf_list_tail
	b	.LBB26_43
.LBB26_33:
	ldp	w8, w9, [x21, #188]
	ldr	w10, [x21, #264]
	sub	x1, x29, #80
	mov	x0, x19
	mov	w2, #68
	stur	wzr, [x29, #-36]
	stp	w9, w8, [x29, #-72]
	ldp	w11, w8, [x21, #204]
	stur	w10, [x29, #-64]
	stp	w11, w8, [x29, #-44]
	bl	_copy_to_user
	cbz	x0, .LBB26_115
	adrp	x1, .L.str.205
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.205
	bl	_dev_err
	b	.LBB26_103
.LBB26_35:
	ldr	w8, [x21, #2096]
	cmp	w8, #2
	b.ge	.LBB26_122
	bl	ebc_empty_osd_buf_get
	cbz	x0, .LBB26_123
.LBB26_37:
	ldp	w10, w9, [x21, #188]
	ldrsw	x8, [x0, #4]
	stur	wzr, [x29, #-36]
	ldr	w11, [x21, #264]
	sub	x1, x29, #80
	mov	x0, x19
	mov	w2, #68
	stur	w8, [x29, #-80]
	stp	w9, w10, [x29, #-72]
	stur	w11, [x29, #-64]
	ldr	x9, [x23]
	ldr	w8, [x9, x8, lsl #2]
	stur	w8, [x29, #-16]
	bl	_copy_to_user
	b	.LBB26_115
.LBB26_38:
	ldr	w8, [x21, #2096]
	cmp	w8, #2
	b.ge	.LBB26_124
	ldrb	w8, [x21, #2115]
	cbz	w8, .LBB26_103
.LBB26_40:
	sub	x0, x29, #80
	mov	x1, x19
	mov	w2, #68
	bl	_copy_from_user
	cbnz	x0, .LBB26_103
	bl	ebc_osd_buf_clone
	cbz	x0, .LBB26_115
	ldur	w8, [x29, #-76]
	ldur	w9, [x29, #-60]
	str	w8, [x0, #32]
	ldur	x8, [x29, #-56]
	ldur	w10, [x29, #-48]
	str	w9, [x0, #40]
	stur	x8, [x0, #44]
	str	w10, [x0, #52]
	bl	ebc_add_to_osd_buf_list_tail
.LBB26_43:
	ldr	x8, [x20, :lo12:global_ebc]
	add	x0, x8, #416
	bl	cancel_delayed_work_sync
	adrp	x8, ebc_task
	ldr	x0, [x8, :lo12:ebc_task]
	bl	wake_up_process
	b	.LBB26_114
.LBB26_44:
	mov	x0, sp
	mov	w1, wzr
	mov	w2, #272
	bl	memset
	ldr	w25, [x21, #1280]
	ldr	w24, [x21, #1352]
	ldr	w22, [x21, #1496]
	ldrb	w8, [x21, #2115]
	ldr	w20, [x21, #1568]
	cbnz	w8, .LBB26_46
	ldrb	w8, [x21, #2112]
	cbz	w8, .LBB26_103
.LBB26_46:
	mov	x0, sp
	mov	x1, x19
	mov	w2, #272
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldp	w8, w9, [x21, #188]
	str	w25, [sp]
	sxtw	x10, w25
	mov	x1, sp
	mov	x0, x19
	mov	w2, #272
	stp	w9, w8, [sp, #8]
	ldr	x11, [x23]
	ldr	w10, [x11, x10, lsl #2]
	stp	w9, w8, [sp, #76]
	stp	w10, w24, [sp, #64]
	sxtw	x10, w24
	ldr	x11, [x23]
	ldr	w10, [x11, x10, lsl #2]
	stp	w9, w8, [sp, #144]
	sxtw	x8, w22
	stp	w10, w22, [sp, #132]
	ldr	x9, [x23]
	ldr	w8, [x9, x8, lsl #2]
	sxtw	x9, w20
	stp	w8, w20, [sp, #200]
	ldr	w8, [x21, #188]
	str	w8, [sp, #216]
	ldr	w8, [x21, #192]
	str	w8, [sp, #212]
	ldr	x8, [x23]
	ldr	w8, [x8, x9, lsl #2]
	str	w8, [sp, #268]
	b	.LBB26_83
.LBB26_48:
	mov	w8, #1
	b	.LBB26_85
.LBB26_49:
	add	x1, x21, #2080
	b	.LBB26_86
.LBB26_50:
	ldr	w8, [x22, #36]
	ldrb	w9, [x21, #2112]
	str	w8, [sp]
	cbz	w9, .LBB26_114
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w3, [sp]
	ldr	w8, [x21, #2096]
	str	w3, [x22, #36]
	tbnz	w8, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.208
	add	x1, x1, :lo12:.L.str.208
	b	.LBB26_20
.LBB26_54:
	ldr	w8, [x22, #40]
	ldrb	w9, [x21, #2112]
	str	w8, [sp]
	cbz	w9, .LBB26_114
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w3, [sp]
	ldr	w9, [x21, #2096]
	cmn	w3, #1
	csinc	w8, w3, wzr, ne
	str	w8, [x22, #40]
	tbnz	w9, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.210
	add	x1, x1, :lo12:.L.str.210
	b	.LBB26_20
.LBB26_58:
	ldrb	w8, [x21, #2113]
	b	.LBB26_85
.LBB26_59:
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	wzr, [sp]
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w3, [sp]
	ldr	w8, [x21, #516]
	cmp	w3, #1
	b.lt	.LBB26_62
	cmp	w3, w8
	b.le	.LBB26_63
.LBB26_62:
	mov	w3, w8
	str	w8, [sp]
.LBB26_63:
	ldr	w8, [x21, #2096]
	str	w3, [x22, #104]
	tbnz	w8, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.212
	add	x1, x1, :lo12:.L.str.212
	b	.LBB26_20
.LBB26_65:
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	wzr, [sp]
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w3, [sp]
	cmp	w3, #11
	b.lo	.LBB26_108
	str	wzr, [sp]
	b	.LBB26_110
.LBB26_68:
	mov	w8, #-1
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	w8, [sp]
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w3, [sp]
	ldr	w8, [x21, #2096]
	str	w3, [x22, #24]
	tbnz	w8, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.214
	add	x1, x1, :lo12:.L.str.214
	b	.LBB26_20
.LBB26_71:
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	wzr, [sp]
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w8, [sp]
	ldr	w10, [x21, #2096]
	cmp	w8, #0
	cset	w9, ne
	strb	w9, [x22, #28]
	tbnz	w10, #31, .LBB26_114
	ldr	x0, [x21, #304]
	cmp	w8, #0
	adrp	x1, .L.str.215
	cset	w3, ne
	add	x1, x1, :lo12:.L.str.215
	b	.LBB26_20
.LBB26_74:
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	wzr, [sp]
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldrsw	x8, [sp]
	mov	w9, #26215
	movk	w9, #26214, lsl #16
	add	w10, w8, #9
	mul	x9, x8, x9
	cmp	w10, #19
	mov	w10, #39
	ccmp	x8, x10, #0, hs
	lsr	x8, x9, #63
	asr	x9, x9, #34
	add	w8, w9, w8
	csinc	w3, w8, wzr, le
	ldr	w8, [x21, #2096]
	str	w3, [x22, #80]
	tbnz	w8, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.216
	add	x1, x1, :lo12:.L.str.216
	b	.LBB26_20
.LBB26_77:
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	wzr, [sp]
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w8, [sp]
	ldr	w10, [x21, #2096]
	cmp	w8, #0
	cset	w9, ne
	strb	w9, [x22, #84]
	tbnz	w10, #31, .LBB26_114
	ldr	x0, [x21, #304]
	cmp	w8, #0
	adrp	x1, .L.str.217
	cset	w3, ne
	add	x1, x1, :lo12:.L.str.217
	b	.LBB26_20
.LBB26_80:
	sub	x0, x29, #80
	mov	x1, x19
	mov	w2, #68
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	x8, [x21, #1768]
	cbz	x8, .LBB26_123
	ldp	w10, w9, [x21, #188]
	ldrsw	x8, [x8, #4]
	stur	wzr, [x29, #-36]
	ldr	w11, [x21, #264]
	sub	x1, x29, #80
	mov	x0, x19
	mov	w2, #68
	stur	w8, [x29, #-80]
	stp	w9, w10, [x29, #-72]
	stur	w11, [x29, #-64]
	ldr	x9, [x23]
	ldr	w8, [x9, x8, lsl #2]
	stur	w8, [x29, #-16]
.LBB26_83:
	bl	_copy_to_user
	cbnz	w0, .LBB26_103
	b	.LBB26_114
.LBB26_84:
	bl	epd_lut_get_wf_bit
	cmp	w0, #32
	cset	w8, eq
.LBB26_85:
	str	w8, [sp]
	mov	x1, sp
.LBB26_86:
	mov	x0, x19
	mov	w2, #4
	bl	_copy_to_user
	b	.LBB26_115
.LBB26_87:
	mov	w8, #1
	ldrb	w9, [x21, #2113]
	str	w8, [sp]
	cbz	w9, .LBB26_114
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w8, [sp]
	str	w8, [x22, #76]
	b	.LBB26_114
.LBB26_90:
	ldr	w8, [x22, #32]
	ldrb	w9, [x21, #2112]
	str	w8, [sp]
	cbz	w9, .LBB26_114
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w8, [sp]
	cmn	w8, #1
	csinc	w3, w8, wzr, ne
	ldr	w8, [x21, #2096]
	str	w3, [x22, #32]
	tbnz	w8, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.209
	add	x1, x1, :lo12:.L.str.209
	b	.LBB26_20
.LBB26_94:
	ldrb	w8, [x21, #2117]
	str	wzr, [sp]
	cbz	w8, .LBB26_114
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w8, [sp]
	str	w8, [x22]
	b	.LBB26_114
.LBB26_97:
	stp	xzr, xzr, [sp, #48]
	stp	xzr, xzr, [sp, #32]
	stp	xzr, xzr, [sp, #16]
	stp	xzr, xzr, [sp]
	bl	epd_lut_get_wf_version
	adrp	x1, .L.str.199
	mov	x2, x0
	add	x1, x1, :lo12:.L.str.199
	mov	x0, sp
	bl	sprintf
	mov	x1, sp
	mov	x0, x19
	mov	w2, #64
	bl	_copy_to_user
	b	.LBB26_115
.LBB26_98:
	ldr	w8, [x22, #44]
	ldrb	w9, [x21, #2112]
	str	w8, [sp]
	cbz	w9, .LBB26_114
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	bl	_copy_from_user
	cbnz	w0, .LBB26_103
	ldr	w3, [sp]
	ldr	w9, [x21, #2096]
	sub	w8, w3, #1
	cmp	w8, #2
	mov	w8, #2
	csel	w8, w3, w8, lo
	str	w8, [x22, #44]
	tbnz	w9, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.211
	add	x1, x1, :lo12:.L.str.211
	b	.LBB26_20
.LBB26_102:
	bl	ebc_overlay_enable_control
	mov	w20, w0
	mov	x0, sp
	mov	x1, x19
	mov	w2, #4
	str	wzr, [sp]
	bl	_copy_from_user
	cbz	w0, .LBB26_104
.LBB26_103:
	mov	x0, #-14
	b	.LBB26_116
.LBB26_104:
	tbz	w20, #1, .LBB26_106
	ldr	w8, [sp]
	ldrb	w9, [x22, #116]
	cmp	w8, #0
	cset	w8, ne
	and	w9, w9, #0xfffffffd
	orr	w8, w9, w8, lsl #1
	strb	w8, [x22, #116]
.LBB26_106:
	ldr	w8, [x21, #2096]
	tbnz	w8, #31, .LBB26_114
	adrp	x1, .L.str.220
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.220
	ldr	w3, [sp]
	add	x2, x2, :lo12:.L.str.201
	ldrb	w4, [x22, #116]
	bl	_dev_info
	b	.LBB26_114
.LBB26_108:
	cbz	w3, .LBB26_110
	mov	w8, #1
	b	.LBB26_111
.LBB26_110:
	mov	w8, wzr
	mov	w3, wzr
.LBB26_111:
	ldr	w9, [x21, #2096]
	strb	w8, [x22, #97]
	str	w3, [x22, #108]
	tbnz	w9, #31, .LBB26_114
	ldr	x0, [x21, #304]
	adrp	x1, .L.str.213
	add	x1, x1, :lo12:.L.str.213
	b	.LBB26_20
.LBB26_113:
	adrp	x1, .L.str.221
	adrp	x2, .L__func__.ebc_io_ctl
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.221
	add	x2, x2, :lo12:.L__func__.ebc_io_ctl
	bl	_dev_err
.LBB26_114:
	mov	x0, xzr
.LBB26_115:
	sxtw	x0, w0
.LBB26_116:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB26_125
	ldp	x20, x19, [sp, #416]
	ldp	x22, x21, [sp, #400]
	ldp	x24, x23, [sp, #384]
	ldp	x28, x25, [sp, #368]
	ldp	x29, x30, [sp, #352]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #432
	ret
.LBB26_118:
	adrp	x1, .L.str.206
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.206
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	b	.LBB26_23
.LBB26_119:
	adrp	x1, .L.str.207
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.207
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	ldr	w8, [x21, #2120]
	cbnz	w8, .LBB26_26
	b	.LBB26_27
.LBB26_120:
	adrp	x1, .L.str.202
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.202
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	b	.LBB26_6
.LBB26_121:
	adrp	x1, .L.str.203
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.203
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	b	.LBB26_30
.LBB26_122:
	adrp	x1, .L.str.200
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.200
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	bl	ebc_empty_osd_buf_get
	cbnz	x0, .LBB26_37
.LBB26_123:
	mov	x0, #-1
	b	.LBB26_116
.LBB26_124:
	adrp	x1, .L.str.204
	adrp	x2, .L.str.201
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.204
	add	x2, x2, :lo12:.L.str.201
	bl	_dev_info
	ldrb	w8, [x21, #2115]
	cbnz	w8, .LBB26_40
	b	.LBB26_103
.LBB26_125:
	bl	__stack_chk_fail
.Lfunc_end26:
	.size	ebc_io_ctl, .Lfunc_end26-ebc_io_ctl
	.section	.rodata,"a",@progbits
	.p2align	1
.LJTI26_0:
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_10-.LBB26_2)>>2
	.hword	(.LBB26_15-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_113-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_21-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_22-.LBB26_2)>>2
	.hword	(.LBB26_24-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.hword	(.LBB26_2-.LBB26_2)>>2
	.p2align	1
.LJTI26_1:
	.hword	(.LBB26_5-.LBB26_5)>>2
	.hword	(.LBB26_29-.LBB26_5)>>2
	.hword	(.LBB26_33-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_10-.LBB26_5)>>2
	.hword	(.LBB26_15-.LBB26_5)>>2
	.hword	(.LBB26_35-.LBB26_5)>>2
	.hword	(.LBB26_38-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_44-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_113-.LBB26_5)>>2
	.hword	(.LBB26_48-.LBB26_5)>>2
	.hword	(.LBB26_21-.LBB26_5)>>2
	.hword	(.LBB26_49-.LBB26_5)>>2
	.hword	(.LBB26_22-.LBB26_5)>>2
	.hword	(.LBB26_24-.LBB26_5)>>2
	.hword	(.LBB26_50-.LBB26_5)>>2
	.hword	(.LBB26_54-.LBB26_5)>>2
	.hword	(.LBB26_58-.LBB26_5)>>2
	.hword	(.LBB26_59-.LBB26_5)>>2
	.hword	(.LBB26_65-.LBB26_5)>>2
	.hword	(.LBB26_68-.LBB26_5)>>2
	.hword	(.LBB26_71-.LBB26_5)>>2
	.hword	(.LBB26_74-.LBB26_5)>>2
	.hword	(.LBB26_77-.LBB26_5)>>2
	.hword	(.LBB26_80-.LBB26_5)>>2
	.hword	(.LBB26_84-.LBB26_5)>>2
	.hword	(.LBB26_87-.LBB26_5)>>2
	.hword	(.LBB26_90-.LBB26_5)>>2
	.hword	(.LBB26_94-.LBB26_5)>>2
	.hword	(.LBB26_97-.LBB26_5)>>2
	.hword	(.LBB26_98-.LBB26_5)>>2
	.hword	(.LBB26_102-.LBB26_5)>>2

	.text
	.p2align	2
	.type	ebc_open,@function
ebc_open:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-64]!
	adrp	x9, kmalloc_caches+48
	adrp	x8, global_ebc
	stp	x20, x19, [sp, #48]
	mov	x19, x1
	mov	w1, #3520
	mov	w2, #8
	ldr	x0, [x9, :lo12:kmalloc_caches+48]
	stp	x22, x21, [sp, #32]
	ldr	x21, [x8, :lo12:global_ebc]
	stp	x24, x23, [sp, #16]
	mov	x29, sp
	bl	kmalloc_trace
	cbz	x0, .LBB27_4
	ldrsw	x8, [x21, #524]
	mov	x20, x0
	mov	w1, #3520
	lsl	x0, x8, #2
	bl	__kmalloc
	str	x0, [x20]
	cbz	x0, .LBB27_5
	ldr	w8, [x21, #528]
	cmp	w8, #1
	b.lt	.LBB27_12
	mov	x22, xzr
	add	x23, x21, #712
	mov	w24, #1
	b	.LBB27_9
.LBB27_4:
	mov	w0, #-12
	b	.LBB27_33
.LBB27_5:
	mov	x0, x20
	bl	kfree
	mov	w0, #-12
	b	.LBB27_33
.Ltmp27:
.LBB27_6:
	add	x8, x8, #56
	//APP
	// atomic64_add
	prfm	pstl1strm, [x8]
.Ltmp28:
	ldxr	x9, [x8]
	add	x9, x9, #1
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp28
	//NO_APP
.LBB27_7:
	mov	w1, #524288
	bl	dma_buf_fd
	ldursw	x8, [x23, #-8]
	ldr	x9, [x20]
	str	w0, [x9, x8, lsl #2]
	ldr	w8, [x21, #528]
.LBB27_8:
	add	x22, x22, #1
	add	x23, x23, #72
	cmp	x22, w8, sxtw
	b.ge	.LBB27_12
.LBB27_9:
	ldur	x9, [x23, #-24]
	cbz	x9, .LBB27_8
	ldr	x0, [x23]
	ldr	x8, [x0, #8]
	//APP
.Ltmp29:
	b	.Ltmp27
.Ltmp30:
	.section	.altinstructions,"a",@progbits
.Ltmp31:
	.word	.Ltmp29-.Ltmp31
.Ltmp32:
	.word	alt_cb_patch_nops-.Ltmp32
	.hword	32794
	.byte	.Ltmp30-.Ltmp29
	.byte	.Ltmp33-.Ltmp34
	.text

.Ltmp34:
.Ltmp33:

	//NO_APP
	add	x8, x8, #56
	//APP
	stadd	x24, [x8]

	//NO_APP
	b	.LBB27_7
.LBB27_12:
	ldr	x8, [x21, #1120]
	cbz	x8, .LBB27_16
	ldr	x0, [x21, #1144]
	ldr	x8, [x0, #8]
	//APP
.Ltmp36:
	b	.Ltmp35
.Ltmp37:
	.section	.altinstructions,"a",@progbits
.Ltmp38:
	.word	.Ltmp36-.Ltmp38
.Ltmp39:
	.word	alt_cb_patch_nops-.Ltmp39
	.hword	32794
	.byte	.Ltmp37-.Ltmp36
	.byte	.Ltmp40-.Ltmp41
	.text

.Ltmp41:
.Ltmp40:

	//NO_APP
	add	x8, x8, #56
	mov	w9, #1
	//APP
	stadd	x9, [x8]

	//NO_APP
.LBB27_15:
	mov	w1, #524288
	bl	dma_buf_fd
	ldrsw	x8, [x21, #1136]
	ldr	x9, [x20]
	str	w0, [x9, x8, lsl #2]
.LBB27_16:
	ldr	x8, [x21, #1264]
	cbz	x8, .LBB27_20
	ldr	x0, [x21, #1288]
	ldr	x8, [x0, #8]
	//APP
.Ltmp43:
	b	.Ltmp42
.Ltmp44:
	.section	.altinstructions,"a",@progbits
.Ltmp45:
	.word	.Ltmp43-.Ltmp45
.Ltmp46:
	.word	alt_cb_patch_nops-.Ltmp46
	.hword	32794
	.byte	.Ltmp44-.Ltmp43
	.byte	.Ltmp47-.Ltmp48
	.text

.Ltmp48:
.Ltmp47:

	//NO_APP
	add	x8, x8, #56
	mov	w9, #1
	//APP
	stadd	x9, [x8]

	//NO_APP
.LBB27_19:
	mov	w1, #524288
	bl	dma_buf_fd
	ldrsw	x8, [x21, #1280]
	ldr	x9, [x20]
	str	w0, [x9, x8, lsl #2]
.LBB27_20:
	ldr	x8, [x21, #1336]
	cbz	x8, .LBB27_24
	ldr	x0, [x21, #1360]
	ldr	x8, [x0, #8]
	//APP
.Ltmp50:
	b	.Ltmp49
.Ltmp51:
	.section	.altinstructions,"a",@progbits
.Ltmp52:
	.word	.Ltmp50-.Ltmp52
.Ltmp53:
	.word	alt_cb_patch_nops-.Ltmp53
	.hword	32794
	.byte	.Ltmp51-.Ltmp50
	.byte	.Ltmp54-.Ltmp55
	.text

.Ltmp55:
.Ltmp54:

	//NO_APP
	add	x8, x8, #56
	mov	w9, #1
	//APP
	stadd	x9, [x8]

	//NO_APP
.LBB27_23:
	mov	w1, #524288
	bl	dma_buf_fd
	ldrsw	x8, [x21, #1352]
	ldr	x9, [x20]
	str	w0, [x9, x8, lsl #2]
.LBB27_24:
	ldr	x8, [x21, #1480]
	cbz	x8, .LBB27_28
	ldr	x0, [x21, #1504]
	ldr	x8, [x0, #8]
	//APP
.Ltmp57:
	b	.Ltmp56
.Ltmp58:
	.section	.altinstructions,"a",@progbits
.Ltmp59:
	.word	.Ltmp57-.Ltmp59
.Ltmp60:
	.word	alt_cb_patch_nops-.Ltmp60
	.hword	32794
	.byte	.Ltmp58-.Ltmp57
	.byte	.Ltmp61-.Ltmp62
	.text

.Ltmp62:
.Ltmp61:

	//NO_APP
	add	x8, x8, #56
	mov	w9, #1
	//APP
	stadd	x9, [x8]

	//NO_APP
.LBB27_27:
	mov	w1, #524288
	bl	dma_buf_fd
	ldrsw	x8, [x21, #1496]
	ldr	x9, [x20]
	str	w0, [x9, x8, lsl #2]
.LBB27_28:
	ldr	x8, [x21, #1552]
	cbz	x8, .LBB27_32
	ldr	x0, [x21, #1576]
	ldr	x8, [x0, #8]
	//APP
.Ltmp64:
	b	.Ltmp63
.Ltmp65:
	.section	.altinstructions,"a",@progbits
.Ltmp66:
	.word	.Ltmp64-.Ltmp66
.Ltmp67:
	.word	alt_cb_patch_nops-.Ltmp67
	.hword	32794
	.byte	.Ltmp65-.Ltmp64
	.byte	.Ltmp68-.Ltmp69
	.text

.Ltmp69:
.Ltmp68:

	//NO_APP
	add	x8, x8, #56
	mov	w9, #1
	//APP
	stadd	x9, [x8]

	//NO_APP
.LBB27_31:
	mov	w1, #524288
	bl	dma_buf_fd
	ldrsw	x8, [x21, #1568]
	ldr	x9, [x20]
	str	w0, [x9, x8, lsl #2]
.LBB27_32:
	mov	w0, wzr
	str	x20, [x19, #200]
	str	xzr, [x19, #104]
.LBB27_33:
	ldp	x20, x19, [sp, #48]
	ldp	x22, x21, [sp, #32]
	ldp	x24, x23, [sp, #16]
	ldp	x29, x30, [sp], #64
	ldr	x30, [x18, #-8]!
	ret
.Ltmp35:
.LBB27_34:
	add	x8, x8, #56
	//APP
	// atomic64_add
	prfm	pstl1strm, [x8]
.Ltmp70:
	ldxr	x9, [x8]
	add	x9, x9, #1
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp70
	//NO_APP
	b	.LBB27_15
.Ltmp42:
.LBB27_35:
	add	x8, x8, #56
	//APP
	// atomic64_add
	prfm	pstl1strm, [x8]
.Ltmp71:
	ldxr	x9, [x8]
	add	x9, x9, #1
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp71
	//NO_APP
	b	.LBB27_19
.Ltmp49:
.LBB27_36:
	add	x8, x8, #56
	//APP
	// atomic64_add
	prfm	pstl1strm, [x8]
.Ltmp72:
	ldxr	x9, [x8]
	add	x9, x9, #1
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp72
	//NO_APP
	b	.LBB27_23
.Ltmp56:
.LBB27_37:
	add	x8, x8, #56
	//APP
	// atomic64_add
	prfm	pstl1strm, [x8]
.Ltmp73:
	ldxr	x9, [x8]
	add	x9, x9, #1
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp73
	//NO_APP
	b	.LBB27_27
.Ltmp63:
.LBB27_38:
	add	x8, x8, #56
	//APP
	// atomic64_add
	prfm	pstl1strm, [x8]
.Ltmp74:
	ldxr	x9, [x8]
	add	x9, x9, #1
	stxr	w10, x9, [x8]
	cbnz	w10, .Ltmp74
	//NO_APP
	b	.LBB27_31
.Lfunc_end27:
	.size	ebc_open, .Lfunc_end27-ebc_open

	.p2align	2
	.type	ebc_release,@function
ebc_release:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	str	x19, [sp, #16]
	ldr	x19, [x1, #200]
	mov	x29, sp
	ldr	x0, [x19]
	cbz	x0, .LBB28_2
	bl	kfree
.LBB28_2:
	mov	x0, x19
	bl	kfree
	ldr	x19, [sp, #16]
	mov	w0, wzr
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end28:
	.size	ebc_release, .Lfunc_end28-ebc_release

	.p2align	2
	.type	_copy_to_user,@function
_copy_to_user:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	//APP
	mrs	x8, SP_EL0
	//NO_APP
	ldrb	w9, [x8, #70]
	mov	x29, sp
	tbnz	w9, #5, .LBB29_2
	ldr	x10, [x8]
	mov	x9, x0
	tbz	w10, #26, .LBB29_3
.LBB29_2:
	lsl	x9, x0, #8
	and	x9, x0, x9, asr #8
.LBB29_3:
	mov	x10, #549755813888
	cmp	x2, x10
	b.hi	.LBB29_19
	sub	x10, x10, x2
	cmp	x10, x9
	b.lo	.LBB29_19
	//APP
.Ltmp76:
	b	.Ltmp75
.Ltmp77:
	.section	.altinstructions,"a",@progbits
.Ltmp78:
	.word	.Ltmp76-.Ltmp78
.Ltmp79:
	.word	alt_cb_patch_nops-.Ltmp79
	.hword	32769
	.byte	.Ltmp77-.Ltmp76
	.byte	.Ltmp80-.Ltmp81
	.text

.Ltmp81:
.Ltmp80:

	//NO_APP
	//APP
.Ltmp83:
	nop
.Ltmp84:
	.section	.altinstructions,"a",@progbits
.Ltmp85:
	.word	.Ltmp83-.Ltmp85
.Ltmp87:
	.word	.Ltmp86-.Ltmp87
	.hword	29
	.byte	.Ltmp84-.Ltmp83
	.byte	.Ltmp88-.Ltmp86
	.text

	.text	1
.Ltmp86:
	b	.Ltmp82
.Ltmp88:
.Ltmp89:
.org (.Ltmp89-(.Ltmp88-.Ltmp86))+(.Ltmp84-.Ltmp83), 0
.Ltmp90:
.org (.Ltmp90-(.Ltmp84-.Ltmp83))+(.Ltmp88-.Ltmp86), 0
	.text


	//NO_APP
.LBB29_7:
	//APP
.Ltmp91:
	mrs	x9, DAIF
.Ltmp92:
	.section	.altinstructions,"a",@progbits
.Ltmp93:
	.word	.Ltmp91-.Ltmp93
.Ltmp95:
	.word	.Ltmp94-.Ltmp95
	.hword	24
	.byte	.Ltmp92-.Ltmp91
	.byte	.Ltmp96-.Ltmp94
	.text

	.text	1
.Ltmp94:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5384609


.Ltmp96:
.Ltmp97:
.org (.Ltmp97-(.Ltmp96-.Ltmp94))+(.Ltmp92-.Ltmp91), 0
.Ltmp98:
.org (.Ltmp98-(.Ltmp92-.Ltmp91))+(.Ltmp96-.Ltmp94), 0
	.text


	//NO_APP
	//APP
.Ltmp99:
	and	w10, w9, #0x80
.Ltmp100:
	.section	.altinstructions,"a",@progbits
.Ltmp101:
	.word	.Ltmp99-.Ltmp101
.Ltmp103:
	.word	.Ltmp102-.Ltmp103
	.hword	24
	.byte	.Ltmp100-.Ltmp99
	.byte	.Ltmp104-.Ltmp102
	.text

	.text	1
.Ltmp102:
	eor	w10, w9, #0xe0
.Ltmp104:
.Ltmp105:
.org (.Ltmp105-(.Ltmp104-.Ltmp102))+(.Ltmp100-.Ltmp99), 0
.Ltmp106:
.org (.Ltmp106-(.Ltmp100-.Ltmp99))+(.Ltmp104-.Ltmp102), 0
	.text


	//NO_APP
	cbnz	w10, .LBB29_11
	//APP
.Ltmp108:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp109:
	.word	.Ltmp108-.Ltmp109
.Ltmp110:
	.word	.Ltmp107-.Ltmp110
.Ltmp111:
	.xword	gic_nonsecure_priorities-.Ltmp111
	.text


	//NO_APP
	mov	w10, #96
.LBB29_10:
	//APP
.Ltmp112:
	msr	DAIFSet, #3	// arch_local_irq_disable
.Ltmp113:
	.section	.altinstructions,"a",@progbits
.Ltmp114:
	.word	.Ltmp112-.Ltmp114
.Ltmp116:
	.word	.Ltmp115-.Ltmp116
	.hword	24
	.byte	.Ltmp113-.Ltmp112
	.byte	.Ltmp117-.Ltmp115
	.text

	.text	1
.Ltmp115:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd518460a


.Ltmp117:
.Ltmp118:
.org (.Ltmp118-(.Ltmp117-.Ltmp115))+(.Ltmp113-.Ltmp112), 0
.Ltmp119:
.org (.Ltmp119-(.Ltmp113-.Ltmp112))+(.Ltmp117-.Ltmp115), 0
	.text


	//NO_APP
.LBB29_11:
	ldr	x8, [x8, #8]
	//APP
	mrs	x10, TTBR1_EL1
	//NO_APP
	mov	x11, x8
	bfxil	x11, x10, #0, #48
	//APP
	msr	TTBR1_EL1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr	TTBR0_EL1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
.Ltmp120:
	msr	DAIF, x9
.Ltmp121:
	.section	.altinstructions,"a",@progbits
.Ltmp122:
	.word	.Ltmp120-.Ltmp122
.Ltmp124:
	.word	.Ltmp123-.Ltmp124
	.hword	24
	.byte	.Ltmp121-.Ltmp120
	.byte	.Ltmp125-.Ltmp123
	.text

	.text	1
.Ltmp123:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5184609


.Ltmp125:
.Ltmp126:
.org (.Ltmp126-(.Ltmp125-.Ltmp123))+(.Ltmp121-.Ltmp120), 0
.Ltmp127:
.org (.Ltmp127-(.Ltmp121-.Ltmp120))+(.Ltmp125-.Ltmp123), 0
	.text


	//NO_APP
	//APP
.Ltmp129:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp130:
	.word	.Ltmp129-.Ltmp130
.Ltmp131:
	.word	.Ltmp128-.Ltmp131
.Ltmp132:
	.xword	gic_pmr_sync-.Ltmp132
	.text


	//NO_APP
.Ltmp82:
.LBB29_12:
	//APP
	and	x0, x0, #0xff7fffffffffffff

	//NO_APP
	bl	__arch_copy_to_user
	mov	x2, x0
	//APP
.Ltmp134:
	b	.Ltmp133
.Ltmp135:
	.section	.altinstructions,"a",@progbits
.Ltmp136:
	.word	.Ltmp134-.Ltmp136
.Ltmp137:
	.word	alt_cb_patch_nops-.Ltmp137
	.hword	32769
	.byte	.Ltmp135-.Ltmp134
	.byte	.Ltmp138-.Ltmp139
	.text

.Ltmp139:
.Ltmp138:

	//NO_APP
	//APP
.Ltmp141:
	nop
.Ltmp142:
	.section	.altinstructions,"a",@progbits
.Ltmp143:
	.word	.Ltmp141-.Ltmp143
.Ltmp145:
	.word	.Ltmp144-.Ltmp145
	.hword	29
	.byte	.Ltmp142-.Ltmp141
	.byte	.Ltmp146-.Ltmp144
	.text

	.text	1
.Ltmp144:
	b	.Ltmp140
.Ltmp146:
.Ltmp147:
.org (.Ltmp147-(.Ltmp146-.Ltmp144))+(.Ltmp142-.Ltmp141), 0
.Ltmp148:
.org (.Ltmp148-(.Ltmp142-.Ltmp141))+(.Ltmp146-.Ltmp144), 0
	.text


	//NO_APP
.LBB29_14:
	//APP
.Ltmp149:
	mrs	x8, DAIF
.Ltmp150:
	.section	.altinstructions,"a",@progbits
.Ltmp151:
	.word	.Ltmp149-.Ltmp151
.Ltmp153:
	.word	.Ltmp152-.Ltmp153
	.hword	24
	.byte	.Ltmp150-.Ltmp149
	.byte	.Ltmp154-.Ltmp152
	.text

	.text	1
.Ltmp152:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5384608


.Ltmp154:
.Ltmp155:
.org (.Ltmp155-(.Ltmp154-.Ltmp152))+(.Ltmp150-.Ltmp149), 0
.Ltmp156:
.org (.Ltmp156-(.Ltmp150-.Ltmp149))+(.Ltmp154-.Ltmp152), 0
	.text


	//NO_APP
	//APP
.Ltmp157:
	and	w9, w8, #0x80
.Ltmp158:
	.section	.altinstructions,"a",@progbits
.Ltmp159:
	.word	.Ltmp157-.Ltmp159
.Ltmp161:
	.word	.Ltmp160-.Ltmp161
	.hword	24
	.byte	.Ltmp158-.Ltmp157
	.byte	.Ltmp162-.Ltmp160
	.text

	.text	1
.Ltmp160:
	eor	w9, w8, #0xe0
.Ltmp162:
.Ltmp163:
.org (.Ltmp163-(.Ltmp162-.Ltmp160))+(.Ltmp158-.Ltmp157), 0
.Ltmp164:
.org (.Ltmp164-(.Ltmp158-.Ltmp157))+(.Ltmp162-.Ltmp160), 0
	.text


	//NO_APP
	cbnz	w9, .LBB29_18
	//APP
.Ltmp166:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp167:
	.word	.Ltmp166-.Ltmp167
.Ltmp168:
	.word	.Ltmp165-.Ltmp168
.Ltmp169:
	.xword	gic_nonsecure_priorities-.Ltmp169
	.text


	//NO_APP
	mov	w9, #96
.LBB29_17:
	//APP
.Ltmp170:
	msr	DAIFSet, #3	// arch_local_irq_disable
.Ltmp171:
	.section	.altinstructions,"a",@progbits
.Ltmp172:
	.word	.Ltmp170-.Ltmp172
.Ltmp174:
	.word	.Ltmp173-.Ltmp174
	.hword	24
	.byte	.Ltmp171-.Ltmp170
	.byte	.Ltmp175-.Ltmp173
	.text

	.text	1
.Ltmp173:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5184609


.Ltmp175:
.Ltmp176:
.org (.Ltmp176-(.Ltmp175-.Ltmp173))+(.Ltmp171-.Ltmp170), 0
.Ltmp177:
.org (.Ltmp177-(.Ltmp171-.Ltmp170))+(.Ltmp175-.Ltmp173), 0
	.text


	//NO_APP
.LBB29_18:
	//APP
	mrs	x9, TTBR1_EL1
	//NO_APP
	and	x9, x9, #0xffffffffffff
	sub	x10, x9, #1, lsl #12
	//APP
	msr	TTBR0_EL1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr	TTBR1_EL1, x9
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
.Ltmp178:
	msr	DAIF, x8
.Ltmp179:
	.section	.altinstructions,"a",@progbits
.Ltmp180:
	.word	.Ltmp178-.Ltmp180
.Ltmp182:
	.word	.Ltmp181-.Ltmp182
	.hword	24
	.byte	.Ltmp179-.Ltmp178
	.byte	.Ltmp183-.Ltmp181
	.text

	.text	1
.Ltmp181:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5184608


.Ltmp183:
.Ltmp184:
.org (.Ltmp184-(.Ltmp183-.Ltmp181))+(.Ltmp179-.Ltmp178), 0
.Ltmp185:
.org (.Ltmp185-(.Ltmp179-.Ltmp178))+(.Ltmp183-.Ltmp181), 0
	.text


	//NO_APP
	//APP
.Ltmp187:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp188:
	.word	.Ltmp187-.Ltmp188
.Ltmp189:
	.word	.Ltmp186-.Ltmp189
.Ltmp190:
	.xword	gic_pmr_sync-.Ltmp190
	.text


	//NO_APP
.Ltmp140:
.LBB29_19:
	mov	x0, x2
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Ltmp75:
.LBB29_20:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #29, .LBB29_12
	b	.LBB29_7
.Ltmp128:
.LBB29_21:
	//APP
	dsb	sy
	//NO_APP
	b	.LBB29_12
.Ltmp133:
.LBB29_22:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #29, .LBB29_19
	b	.LBB29_14
.Ltmp186:
.LBB29_23:
	//APP
	dsb	sy
	//NO_APP
	b	.LBB29_19
.Ltmp107:
.LBB29_24:
	mov	w10, #160
	b	.LBB29_10
.Ltmp165:
.LBB29_25:
	mov	w9, #160
	b	.LBB29_17
.Lfunc_end29:
	.size	_copy_to_user, .Lfunc_end29-_copy_to_user

	.p2align	2
	.type	_copy_from_user,@function
_copy_from_user:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	stp	x20, x19, [sp, #32]
	mov	x19, x2
	//APP
	mrs	x8, SP_EL0
	//NO_APP
	ldrb	w9, [x8, #70]
	mov	x20, x0
	str	x21, [sp, #16]
	mov	x29, sp
	tbnz	w9, #5, .LBB30_2
	ldr	x10, [x8]
	mov	x9, x1
	tbz	w10, #26, .LBB30_3
.LBB30_2:
	lsl	x9, x1, #8
	and	x9, x1, x9, asr #8
.LBB30_3:
	mov	x21, x19
	mov	x10, #549755813888
	cmp	x19, x10
	b.hi	.LBB30_19
	mov	x21, x19
	sub	x10, x10, x19
	cmp	x10, x9
	b.lo	.LBB30_19
	//APP
.Ltmp192:
	b	.Ltmp191
.Ltmp193:
	.section	.altinstructions,"a",@progbits
.Ltmp194:
	.word	.Ltmp192-.Ltmp194
.Ltmp195:
	.word	alt_cb_patch_nops-.Ltmp195
	.hword	32769
	.byte	.Ltmp193-.Ltmp192
	.byte	.Ltmp196-.Ltmp197
	.text

.Ltmp197:
.Ltmp196:

	//NO_APP
	//APP
.Ltmp199:
	nop
.Ltmp200:
	.section	.altinstructions,"a",@progbits
.Ltmp201:
	.word	.Ltmp199-.Ltmp201
.Ltmp203:
	.word	.Ltmp202-.Ltmp203
	.hword	29
	.byte	.Ltmp200-.Ltmp199
	.byte	.Ltmp204-.Ltmp202
	.text

	.text	1
.Ltmp202:
	b	.Ltmp198
.Ltmp204:
.Ltmp205:
.org (.Ltmp205-(.Ltmp204-.Ltmp202))+(.Ltmp200-.Ltmp199), 0
.Ltmp206:
.org (.Ltmp206-(.Ltmp200-.Ltmp199))+(.Ltmp204-.Ltmp202), 0
	.text


	//NO_APP
.LBB30_7:
	//APP
.Ltmp207:
	mrs	x9, DAIF
.Ltmp208:
	.section	.altinstructions,"a",@progbits
.Ltmp209:
	.word	.Ltmp207-.Ltmp209
.Ltmp211:
	.word	.Ltmp210-.Ltmp211
	.hword	24
	.byte	.Ltmp208-.Ltmp207
	.byte	.Ltmp212-.Ltmp210
	.text

	.text	1
.Ltmp210:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5384609


.Ltmp212:
.Ltmp213:
.org (.Ltmp213-(.Ltmp212-.Ltmp210))+(.Ltmp208-.Ltmp207), 0
.Ltmp214:
.org (.Ltmp214-(.Ltmp208-.Ltmp207))+(.Ltmp212-.Ltmp210), 0
	.text


	//NO_APP
	//APP
.Ltmp215:
	and	w10, w9, #0x80
.Ltmp216:
	.section	.altinstructions,"a",@progbits
.Ltmp217:
	.word	.Ltmp215-.Ltmp217
.Ltmp219:
	.word	.Ltmp218-.Ltmp219
	.hword	24
	.byte	.Ltmp216-.Ltmp215
	.byte	.Ltmp220-.Ltmp218
	.text

	.text	1
.Ltmp218:
	eor	w10, w9, #0xe0
.Ltmp220:
.Ltmp221:
.org (.Ltmp221-(.Ltmp220-.Ltmp218))+(.Ltmp216-.Ltmp215), 0
.Ltmp222:
.org (.Ltmp222-(.Ltmp216-.Ltmp215))+(.Ltmp220-.Ltmp218), 0
	.text


	//NO_APP
	cbnz	w10, .LBB30_11
	//APP
.Ltmp224:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp225:
	.word	.Ltmp224-.Ltmp225
.Ltmp226:
	.word	.Ltmp223-.Ltmp226
.Ltmp227:
	.xword	gic_nonsecure_priorities-.Ltmp227
	.text


	//NO_APP
	mov	w10, #96
.LBB30_10:
	//APP
.Ltmp228:
	msr	DAIFSet, #3	// arch_local_irq_disable
.Ltmp229:
	.section	.altinstructions,"a",@progbits
.Ltmp230:
	.word	.Ltmp228-.Ltmp230
.Ltmp232:
	.word	.Ltmp231-.Ltmp232
	.hword	24
	.byte	.Ltmp229-.Ltmp228
	.byte	.Ltmp233-.Ltmp231
	.text

	.text	1
.Ltmp231:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd518460a


.Ltmp233:
.Ltmp234:
.org (.Ltmp234-(.Ltmp233-.Ltmp231))+(.Ltmp229-.Ltmp228), 0
.Ltmp235:
.org (.Ltmp235-(.Ltmp229-.Ltmp228))+(.Ltmp233-.Ltmp231), 0
	.text


	//NO_APP
.LBB30_11:
	ldr	x8, [x8, #8]
	//APP
	mrs	x10, TTBR1_EL1
	//NO_APP
	mov	x11, x8
	bfxil	x11, x10, #0, #48
	//APP
	msr	TTBR1_EL1, x11
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr	TTBR0_EL1, x8
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
.Ltmp236:
	msr	DAIF, x9
.Ltmp237:
	.section	.altinstructions,"a",@progbits
.Ltmp238:
	.word	.Ltmp236-.Ltmp238
.Ltmp240:
	.word	.Ltmp239-.Ltmp240
	.hword	24
	.byte	.Ltmp237-.Ltmp236
	.byte	.Ltmp241-.Ltmp239
	.text

	.text	1
.Ltmp239:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5184609


.Ltmp241:
.Ltmp242:
.org (.Ltmp242-(.Ltmp241-.Ltmp239))+(.Ltmp237-.Ltmp236), 0
.Ltmp243:
.org (.Ltmp243-(.Ltmp237-.Ltmp236))+(.Ltmp241-.Ltmp239), 0
	.text


	//NO_APP
	//APP
.Ltmp245:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp246:
	.word	.Ltmp245-.Ltmp246
.Ltmp247:
	.word	.Ltmp244-.Ltmp247
.Ltmp248:
	.xword	gic_pmr_sync-.Ltmp248
	.text


	//NO_APP
.Ltmp198:
.LBB30_12:
	//APP
	and	x1, x1, #0xff7fffffffffffff

	//NO_APP
	mov	x0, x20
	mov	x2, x19
	bl	__arch_copy_from_user
	mov	x21, x0
	//APP
.Ltmp250:
	b	.Ltmp249
.Ltmp251:
	.section	.altinstructions,"a",@progbits
.Ltmp252:
	.word	.Ltmp250-.Ltmp252
.Ltmp253:
	.word	alt_cb_patch_nops-.Ltmp253
	.hword	32769
	.byte	.Ltmp251-.Ltmp250
	.byte	.Ltmp254-.Ltmp255
	.text

.Ltmp255:
.Ltmp254:

	//NO_APP
	//APP
.Ltmp257:
	nop
.Ltmp258:
	.section	.altinstructions,"a",@progbits
.Ltmp259:
	.word	.Ltmp257-.Ltmp259
.Ltmp261:
	.word	.Ltmp260-.Ltmp261
	.hword	29
	.byte	.Ltmp258-.Ltmp257
	.byte	.Ltmp262-.Ltmp260
	.text

	.text	1
.Ltmp260:
	b	.Ltmp256
.Ltmp262:
.Ltmp263:
.org (.Ltmp263-(.Ltmp262-.Ltmp260))+(.Ltmp258-.Ltmp257), 0
.Ltmp264:
.org (.Ltmp264-(.Ltmp258-.Ltmp257))+(.Ltmp262-.Ltmp260), 0
	.text


	//NO_APP
.LBB30_14:
	//APP
.Ltmp265:
	mrs	x8, DAIF
.Ltmp266:
	.section	.altinstructions,"a",@progbits
.Ltmp267:
	.word	.Ltmp265-.Ltmp267
.Ltmp269:
	.word	.Ltmp268-.Ltmp269
	.hword	24
	.byte	.Ltmp266-.Ltmp265
	.byte	.Ltmp270-.Ltmp268
	.text

	.text	1
.Ltmp268:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5384608


.Ltmp270:
.Ltmp271:
.org (.Ltmp271-(.Ltmp270-.Ltmp268))+(.Ltmp266-.Ltmp265), 0
.Ltmp272:
.org (.Ltmp272-(.Ltmp266-.Ltmp265))+(.Ltmp270-.Ltmp268), 0
	.text


	//NO_APP
	//APP
.Ltmp273:
	and	w9, w8, #0x80
.Ltmp274:
	.section	.altinstructions,"a",@progbits
.Ltmp275:
	.word	.Ltmp273-.Ltmp275
.Ltmp277:
	.word	.Ltmp276-.Ltmp277
	.hword	24
	.byte	.Ltmp274-.Ltmp273
	.byte	.Ltmp278-.Ltmp276
	.text

	.text	1
.Ltmp276:
	eor	w9, w8, #0xe0
.Ltmp278:
.Ltmp279:
.org (.Ltmp279-(.Ltmp278-.Ltmp276))+(.Ltmp274-.Ltmp273), 0
.Ltmp280:
.org (.Ltmp280-(.Ltmp274-.Ltmp273))+(.Ltmp278-.Ltmp276), 0
	.text


	//NO_APP
	cbnz	w9, .LBB30_18
	//APP
.Ltmp282:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp283:
	.word	.Ltmp282-.Ltmp283
.Ltmp284:
	.word	.Ltmp281-.Ltmp284
.Ltmp285:
	.xword	gic_nonsecure_priorities-.Ltmp285
	.text


	//NO_APP
	mov	w9, #96
.LBB30_17:
	//APP
.Ltmp286:
	msr	DAIFSet, #3	// arch_local_irq_disable
.Ltmp287:
	.section	.altinstructions,"a",@progbits
.Ltmp288:
	.word	.Ltmp286-.Ltmp288
.Ltmp290:
	.word	.Ltmp289-.Ltmp290
	.hword	24
	.byte	.Ltmp287-.Ltmp286
	.byte	.Ltmp291-.Ltmp289
	.text

	.text	1
.Ltmp289:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5184609


.Ltmp291:
.Ltmp292:
.org (.Ltmp292-(.Ltmp291-.Ltmp289))+(.Ltmp287-.Ltmp286), 0
.Ltmp293:
.org (.Ltmp293-(.Ltmp287-.Ltmp286))+(.Ltmp291-.Ltmp289), 0
	.text


	//NO_APP
.LBB30_18:
	//APP
	mrs	x9, TTBR1_EL1
	//NO_APP
	and	x9, x9, #0xffffffffffff
	sub	x10, x9, #1, lsl #12
	//APP
	msr	TTBR0_EL1, x10
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
	msr	TTBR1_EL1, x9
	//NO_APP
	//APP
	isb
	//NO_APP
	//APP
.Ltmp294:
	msr	DAIF, x8
.Ltmp295:
	.section	.altinstructions,"a",@progbits
.Ltmp296:
	.word	.Ltmp294-.Ltmp296
.Ltmp298:
	.word	.Ltmp297-.Ltmp298
	.hword	24
	.byte	.Ltmp295-.Ltmp294
	.byte	.Ltmp299-.Ltmp297
	.text

	.text	1
.Ltmp297:
.set .L__gpr_num_x0, 0
.set .L__gpr_num_w0, 0
.set .L__gpr_num_x1, 1
.set .L__gpr_num_w1, 1
.set .L__gpr_num_x2, 2
.set .L__gpr_num_w2, 2
.set .L__gpr_num_x3, 3
.set .L__gpr_num_w3, 3
.set .L__gpr_num_x4, 4
.set .L__gpr_num_w4, 4
.set .L__gpr_num_x5, 5
.set .L__gpr_num_w5, 5
.set .L__gpr_num_x6, 6
.set .L__gpr_num_w6, 6
.set .L__gpr_num_x7, 7
.set .L__gpr_num_w7, 7
.set .L__gpr_num_x8, 8
.set .L__gpr_num_w8, 8
.set .L__gpr_num_x9, 9
.set .L__gpr_num_w9, 9
.set .L__gpr_num_x10, 10
.set .L__gpr_num_w10, 10
.set .L__gpr_num_x11, 11
.set .L__gpr_num_w11, 11
.set .L__gpr_num_x12, 12
.set .L__gpr_num_w12, 12
.set .L__gpr_num_x13, 13
.set .L__gpr_num_w13, 13
.set .L__gpr_num_x14, 14
.set .L__gpr_num_w14, 14
.set .L__gpr_num_x15, 15
.set .L__gpr_num_w15, 15
.set .L__gpr_num_x16, 16
.set .L__gpr_num_w16, 16
.set .L__gpr_num_x17, 17
.set .L__gpr_num_w17, 17
.set .L__gpr_num_x18, 18
.set .L__gpr_num_w18, 18
.set .L__gpr_num_x19, 19
.set .L__gpr_num_w19, 19
.set .L__gpr_num_x20, 20
.set .L__gpr_num_w20, 20
.set .L__gpr_num_x21, 21
.set .L__gpr_num_w21, 21
.set .L__gpr_num_x22, 22
.set .L__gpr_num_w22, 22
.set .L__gpr_num_x23, 23
.set .L__gpr_num_w23, 23
.set .L__gpr_num_x24, 24
.set .L__gpr_num_w24, 24
.set .L__gpr_num_x25, 25
.set .L__gpr_num_w25, 25
.set .L__gpr_num_x26, 26
.set .L__gpr_num_w26, 26
.set .L__gpr_num_x27, 27
.set .L__gpr_num_w27, 27
.set .L__gpr_num_x28, 28
.set .L__gpr_num_w28, 28
.set .L__gpr_num_x29, 29
.set .L__gpr_num_w29, 29
.set .L__gpr_num_x30, 30
.set .L__gpr_num_w30, 30

.set .L__gpr_num_xzr, 31
.set .L__gpr_num_wzr, 31

	.inst	0xd5184608


.Ltmp299:
.Ltmp300:
.org (.Ltmp300-(.Ltmp299-.Ltmp297))+(.Ltmp295-.Ltmp294), 0
.Ltmp301:
.org (.Ltmp301-(.Ltmp295-.Ltmp294))+(.Ltmp299-.Ltmp297), 0
	.text


	//NO_APP
	//APP
.Ltmp303:
	nop
	.section	__jump_table,"aw",@progbits
	.p2align	3
.Ltmp304:
	.word	.Ltmp303-.Ltmp304
.Ltmp305:
	.word	.Ltmp302-.Ltmp305
.Ltmp306:
	.xword	gic_pmr_sync-.Ltmp306
	.text


	//NO_APP
.Ltmp256:
.LBB30_19:
	cbnz	x21, .LBB30_21
.LBB30_20:
	ldp	x20, x19, [sp, #32]
	mov	x0, x21
	ldr	x21, [sp, #16]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.LBB30_21:
	sub	x8, x19, x21
	mov	w1, wzr
	add	x0, x20, x8
	mov	x2, x21
	bl	memset
	b	.LBB30_20
.Ltmp191:
.LBB30_22:
	adrp	x9, cpu_hwcaps
	ldr	x9, [x9, :lo12:cpu_hwcaps]
	tbnz	w9, #29, .LBB30_12
	b	.LBB30_7
.Ltmp244:
.LBB30_23:
	//APP
	dsb	sy
	//NO_APP
	b	.LBB30_12
.Ltmp249:
.LBB30_24:
	adrp	x8, cpu_hwcaps
	ldr	x8, [x8, :lo12:cpu_hwcaps]
	tbnz	w8, #29, .LBB30_19
	b	.LBB30_14
.Ltmp302:
.LBB30_25:
	//APP
	dsb	sy
	//NO_APP
	b	.LBB30_19
.Ltmp223:
.LBB30_26:
	mov	w10, #160
	b	.LBB30_10
.Ltmp281:
.LBB30_27:
	mov	w9, #160
	b	.LBB30_17
.Lfunc_end30:
	.size	_copy_from_user, .Lfunc_end30-_copy_from_user

	.p2align	2
	.type	waveform_llseek,@function
waveform_llseek:
	adrp	x8, global_ebc
	ldr	x8, [x8, :lo12:global_ebc]
	cbz	w2, .LBB31_6
	cmp	w2, #2
	b.eq	.LBB31_4
	cmp	w2, #1
	b.ne	.LBB31_8
	ldr	x9, [x0, #104]
	b	.LBB31_5
.LBB31_4:
	ldrsw	x9, [x8, #1800]
.LBB31_5:
	add	x1, x9, x1
.LBB31_6:
	tbnz	x1, #63, .LBB31_8
	ldrsw	x8, [x8, #1800]
	cmp	x1, x8
	b.le	.LBB31_9
.LBB31_8:
	mov	x0, #-22
	ret
.LBB31_9:
	str	x1, [x0, #104]
	mov	x0, x1
	ret
.Lfunc_end31:
	.size	waveform_llseek, .Lfunc_end31-waveform_llseek

	.p2align	2
	.type	waveform_read,@function
waveform_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	adrp	x9, global_ebc
	ldr	x8, [x3]
	stp	x22, x21, [sp, #16]
	mov	x29, sp
	stp	x20, x19, [sp, #32]
	ldr	x9, [x9, :lo12:global_ebc]
	ldrsw	x10, [x9, #1800]
	cmp	x8, x10
	b.ge	.LBB32_4
	add	x11, x8, x2
	sub	x12, x10, x8
	cmp	x11, x10
	csel	x19, x12, x2, hi
	lsr	x10, x19, #31
	cbnz	x10, .LBB32_7
	ldr	x9, [x9, #1864]
	mov	x21, x1
	mov	x1, x19
	mov	w2, #1
	mov	x20, x3
	add	x22, x9, x8
	mov	x0, x22
	bl	__check_object_size
	mov	x0, x21
	mov	x1, x22
	mov	x2, x19
	bl	_copy_to_user
	cbz	x0, .LBB32_5
	mov	x19, #-14
	b	.LBB32_6
.LBB32_4:
	mov	x19, xzr
	b	.LBB32_6
.LBB32_5:
	ldr	x8, [x20]
	add	x8, x8, x19
	str	x8, [x20]
.LBB32_6:
	mov	x0, x19
	ldp	x20, x19, [sp, #32]
	ldp	x22, x21, [sp, #16]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.LBB32_7:
	mov	x19, #-14
	//APP
	.section	__bug_table,"aw",@progbits
	.p2align	2
.Ltmp307:
.Ltmp309:
	.word	.Ltmp308-.Ltmp309
	.section	.rodata.str,"aMS",@progbits,1
.Ltmp310:
	.ascii	"include/linux/thread_info.h"
	.byte	0
	.section	__bug_table,"aw",@progbits
.Ltmp311:
	.word	.Ltmp310-.Ltmp311
	.hword	233
	.hword	2307
	.p2align	2
	.text
.Ltmp308:
	brk	#0x800
	//NO_APP
	b	.LBB32_6
.Lfunc_end32:
	.size	waveform_read, .Lfunc_end32-waveform_read

	.p2align	2
	.type	waveform_write,@function
waveform_write:
	sub	sp, sp, #112
	str	x30, [x18], #8
	stp	x29, x30, [sp, #48]
	add	x29, sp, #48
	stp	x24, x23, [sp, #64]
	adrp	x9, global_ebc
	stp	x22, x21, [sp, #80]
	stp	x20, x19, [sp, #96]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	stur	x8, [x29, #-8]
	ldr	x23, [x9, :lo12:global_ebc]
	ldr	x8, [x3]
	ldr	w10, [x23, #2108]
	ldrsw	x9, [x23, #1800]
	cmp	w10, #0
	cset	w24, eq
	cmp	x8, x9
	b.ge	.LBB33_4
	add	x10, x8, x2
	sub	x11, x9, x8
	cmp	x10, x9
	csel	x19, x11, x2, hi
	lsr	x9, x19, #31
	cbnz	x9, .LBB33_8
	ldr	x9, [x23, #1864]
	mov	x21, x1
	mov	x1, x19
	mov	w2, wzr
	mov	x20, x3
	add	x22, x9, x8
	mov	x0, x22
	bl	__check_object_size
	mov	x0, x22
	mov	x1, x21
	mov	x2, x19
	bl	_copy_from_user
	cbz	x0, .LBB33_5
	mov	x19, #-14
	b	.LBB33_6
.LBB33_4:
	mov	x19, #-28
	b	.LBB33_6
.LBB33_5:
	ldr	x8, [x20]
	add	x8, x8, x19
	str	x8, [x20]
	ldr	x0, [x23, #1864]
	bl	epd_lut_from_mem_init
	mov	x8, #32
	add	x0, x23, #1808
	movk	x8, #1, lsl #32
	add	x3, sp, #8
	mov	w1, #3
	mov	w2, #30
	stp	wzr, w24, [sp, #24]
	stp	x8, xzr, [sp, #8]
	str	wzr, [sp, #32]
	bl	epd_lut_get
.LBB33_6:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB33_9
	mov	x0, x19
	ldp	x20, x19, [sp, #96]
	ldp	x22, x21, [sp, #80]
	ldp	x24, x23, [sp, #64]
	ldp	x29, x30, [sp, #48]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #112
	ret
.LBB33_8:
	mov	x19, #-14
	//APP
	.section	__bug_table,"aw",@progbits
	.p2align	2
.Ltmp312:
.Ltmp314:
	.word	.Ltmp313-.Ltmp314
	.section	.rodata.str,"aMS",@progbits,1
.Ltmp315:
	.ascii	"include/linux/thread_info.h"
	.byte	0
	.section	__bug_table,"aw",@progbits
.Ltmp316:
	.word	.Ltmp315-.Ltmp316
	.hword	233
	.hword	2307
	.p2align	2
	.text
.Ltmp313:
	brk	#0x800
	//NO_APP
	b	.LBB33_6
.LBB33_9:
	bl	__stack_chk_fail
.Lfunc_end33:
	.size	waveform_write, .Lfunc_end33-waveform_write

	.p2align	2
	.type	waveform_mmap,@function
waveform_mmap:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	adrp	x8, global_ebc
	ldr	x9, [x1, #16]
	str	x21, [sp, #16]
	mov	x29, sp
	stp	x20, x19, [sp, #32]
	mov	x19, x1
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	w21, [x9, #152]
	ldr	w9, [x1, #40]
	ldr	x20, [x8, #1792]
	cmp	w9, w21
	b.eq	.LBB34_2
	ldr	x0, [x19, #48]
	bl	down_write
	str	w21, [x19, #40]
	ldr	x0, [x19, #48]
	bl	up_write
.LBB34_2:
	ldp	x4, x8, [x19, #24]
	mov	w10, #16384
	lsr	x2, x20, #12
	ldp	x1, x9, [x19]
	movk	w10, #1028, lsl #16
	mov	x0, x19
	orr	x8, x8, x10
	sub	x3, x9, x1
	str	x8, [x19, #32]
	bl	remap_pfn_range
	ldp	x20, x19, [sp, #32]
	mov	w8, #-11
	cmp	w0, #0
	ldr	x21, [sp, #16]
	csel	w0, wzr, w8, eq
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end34:
	.size	waveform_mmap, .Lfunc_end34-waveform_mmap

	.p2align	2
	.type	waveform_open,@function
waveform_open:
	mov	w0, wzr
	str	xzr, [x1, #104]
	ret
.Lfunc_end35:
	.size	waveform_open, .Lfunc_end35-waveform_open

	.p2align	2
	.type	transition_lut_llseek,@function
transition_lut_llseek:
	adrp	x8, global_ebc
	ldr	x8, [x8, :lo12:global_ebc]
	cbz	w2, .LBB36_6
	cmp	w2, #2
	b.eq	.LBB36_4
	cmp	w2, #1
	b.ne	.LBB36_8
	add	x9, x0, #104
	b	.LBB36_5
.LBB36_4:
	add	x9, x8, #1200
.LBB36_5:
	ldr	x9, [x9]
	add	x1, x9, x1
.LBB36_6:
	tbnz	x1, #63, .LBB36_8
	ldr	x8, [x8, #1200]
	cmp	x1, x8
	b.ls	.LBB36_9
.LBB36_8:
	mov	x0, #-22
	ret
.LBB36_9:
	str	x1, [x0, #104]
	mov	x0, x1
	ret
.Lfunc_end36:
	.size	transition_lut_llseek, .Lfunc_end36-transition_lut_llseek

	.p2align	2
	.type	transition_lut_read,@function
transition_lut_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x9, global_ebc
	mov	x8, x2
	mov	x2, x1
	mov	x1, x3
	mov	x3, x8
	mov	w4, #1
	ldr	x9, [x9, :lo12:global_ebc]
	mov	x29, sp
	add	x0, x9, #1184
	bl	transition_copy_data
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end37:
	.size	transition_lut_read, .Lfunc_end37-transition_lut_read

	.p2align	2
	.type	transition_lut_write,@function
transition_lut_write:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	adrp	x9, global_ebc
	str	x19, [sp, #16]
	mov	x8, x2
	mov	x2, x1
	mov	x1, x3
	mov	x3, x8
	ldr	x19, [x9, :lo12:global_ebc]
	mov	w4, wzr
	mov	x29, sp
	add	x0, x19, #1184
	bl	transition_copy_data
	cmp	x0, #1
	b.lt	.LBB38_2
	mov	w8, #18612
	mov	w9, #1
	strb	w9, [x19, x8]
.LBB38_2:
	ldr	x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end38:
	.size	transition_lut_write, .Lfunc_end38-transition_lut_write

	.p2align	2
	.type	transition_lut_mmap,@function
transition_lut_mmap:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-48]!
	adrp	x9, transition_lut_vm_ops
	ldr	x10, [x1, #16]
	add	x9, x9, :lo12:transition_lut_vm_ops
	adrp	x8, global_ebc
	str	x21, [sp, #16]
	mov	x29, sp
	stp	x20, x19, [sp, #32]
	mov	x19, x1
	str	x9, [x1, #120]
	ldr	w9, [x1, #40]
	ldr	w21, [x10, #152]
	ldr	x20, [x8, :lo12:global_ebc]
	cmp	w9, w21
	b.eq	.LBB39_2
	ldr	x0, [x19, #48]
	bl	down_write
	str	w21, [x19, #40]
	ldr	x0, [x19, #48]
	bl	up_write
.LBB39_2:
	ldr	x8, [x19, #32]
	mov	w9, #335806464
	ldr	x21, [sp, #16]
	mov	w0, wzr
	orr	x8, x8, x9
	add	x9, x20, #1184
	str	x8, [x19, #32]
	str	x9, [x19, #144]
	ldp	x20, x19, [sp, #32]
	ldp	x29, x30, [sp], #48
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end39:
	.size	transition_lut_mmap, .Lfunc_end39-transition_lut_mmap

	.p2align	2
	.type	transition_lut_open,@function
transition_lut_open:
	adrp	x8, global_ebc
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	x8, [x8, #1216]
	cmp	x8, #0
	mov	w8, #-19
	csel	w0, w8, wzr, eq
	ret
.Lfunc_end40:
	.size	transition_lut_open, .Lfunc_end40-transition_lut_open

	.p2align	2
	.type	transition_copy_data,@function
transition_copy_data:
	sub	sp, sp, #128
	str	x30, [x18], #8
	stp	x28, x27, [sp, #48]
	ldr	x27, [x1]
	ldr	x8, [x0, #16]
	stp	x29, x30, [sp, #32]
	stp	x26, x25, [sp, #64]
	add	x29, sp, #32
	mov	w26, w4
	stp	x24, x23, [sp, #80]
	subs	x8, x8, x27
	stp	x22, x21, [sp, #96]
	stp	x20, x19, [sp, #112]
	stur	x2, [x29, #-8]
	b.ls	.LBB41_15
	cmp	x8, x3
	ldr	x24, [x0, #48]
	csel	x22, x8, x3, lo
	cbz	x24, .LBB41_16
	ldr	w8, [x24, #8]
	str	x1, [sp]
	cbz	w8, .LBB41_17
	mov	x19, xzr
	mov	x8, xzr
	mov	w21, wzr
	ldr	x23, [x24]
	//APP
	mrs	x28, SP_EL0
	//NO_APP
	stur	w26, [x29, #-12]
	str	x24, [sp, #8]
	b	.LBB41_5
.LBB41_4:
	mov	x0, x23
	add	w21, w21, #1
	bl	sg_next
	ldr	w9, [x24, #8]
	mov	x23, x0
	mov	x8, x20
	cmp	w21, w9
	b.hs	.LBB41_18
.LBB41_5:
	ldr	w9, [x23, #12]
	add	x20, x8, x9
	cmp	x27, x20
	ccmp	x22, x19, #0, lo
	b.ls	.LBB41_4
	ldr	w10, [x23, #8]
	subs	x8, x27, x8
	ldr	x11, [x23]
	csel	x8, xzr, x8, lo
	ldr	w12, [x28, #16]
	sub	x9, x9, x8
	add	x8, x8, x10
	sub	x10, x22, x19
	cmp	x9, x10
	lsl	x11, x11, #6
	add	w12, w12, #1
	and	x11, x11, #0xfffffffffffff000
	csel	x24, x10, x9, hi
	add	x25, x11, x8
	lsr	x8, x24, #31
	str	w12, [x28, #16]
	//APP
	//NO_APP
	ldr	w12, [x28, #2760]
	add	w12, w12, #1
	str	w12, [x28, #2760]
	//APP
	//NO_APP
	tbz	w26, #0, .LBB41_9
	cbnz	x8, .LBB41_20
	ldur	x8, [x29, #-8]
	mov	x0, x25
	mov	x1, x24
	mov	w2, #1
	add	x26, x8, x19
	bl	__check_object_size
	mov	x0, x26
	mov	x1, x25
	mov	x2, x24
	bl	_copy_to_user
	cbz	x0, .LBB41_11
	b	.LBB41_21
.LBB41_9:
	cbnz	x8, .LBB41_23
	ldur	x8, [x29, #-8]
	mov	x0, x25
	mov	x1, x24
	mov	w2, wzr
	add	x26, x8, x19
	bl	__check_object_size
	mov	x0, x25
	mov	x1, x26
	mov	x2, x24
	bl	_copy_from_user
	cbnz	x0, .LBB41_24
.LBB41_11:
	//APP
	//NO_APP
	ldr	w8, [x28, #2760]
	sub	w8, w8, #1
	str	w8, [x28, #2760]
	//APP
	//NO_APP
	ldr	x8, [x28, #16]
	subs	x8, x8, #1
	str	w8, [x28, #16]
	b.eq	.LBB41_14
	ldr	x8, [x28, #16]
	cbz	x8, .LBB41_14
.LBB41_13:
	add	x19, x24, x19
	add	x27, x24, x27
	ldur	w26, [x29, #-12]
	cmp	x19, x22
	ldr	x24, [sp, #8]
	b.lo	.LBB41_4
	b	.LBB41_18
.LBB41_14:
	bl	preempt_schedule
	b	.LBB41_13
.LBB41_15:
	tst	w26, #0x1
	mov	x8, #-28
	csel	x19, xzr, x8, ne
	b	.LBB41_19
.LBB41_16:
	mov	x19, #-28
	b	.LBB41_19
.LBB41_17:
	mov	x19, xzr
.LBB41_18:
	ldr	x8, [sp]
	str	x27, [x8]
.LBB41_19:
	mov	x0, x19
	ldp	x20, x19, [sp, #112]
	ldp	x22, x21, [sp, #96]
	ldp	x24, x23, [sp, #80]
	ldp	x26, x25, [sp, #64]
	ldp	x28, x27, [sp, #48]
	ldp	x29, x30, [sp, #32]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #128
	ret
.LBB41_20:
	//APP
	.section	__bug_table,"aw",@progbits
	.p2align	2
.Ltmp317:
.Ltmp319:
	.word	.Ltmp318-.Ltmp319
	.section	.rodata.str,"aMS",@progbits,1
.Ltmp320:
	.ascii	"include/linux/thread_info.h"
	.byte	0
	.section	__bug_table,"aw",@progbits
.Ltmp321:
	.word	.Ltmp320-.Ltmp321
	.hword	233
	.hword	2307
	.p2align	2
	.text
.Ltmp318:
	brk	#0x800
	//NO_APP
.LBB41_21:
	//APP
	//NO_APP
	ldr	w8, [x28, #2760]
	sub	w8, w8, #1
	str	w8, [x28, #2760]
	//APP
	//NO_APP
	ldr	x8, [x28, #16]
	subs	x8, x8, #1
	str	w8, [x28, #16]
	b.eq	.LBB41_27
	ldr	x8, [x28, #16]
	cbnz	x8, .LBB41_26
	b	.LBB41_27
.LBB41_23:
	//APP
	.section	__bug_table,"aw",@progbits
	.p2align	2
.Ltmp322:
.Ltmp324:
	.word	.Ltmp323-.Ltmp324
	.section	.rodata.str,"aMS",@progbits,1
.Ltmp325:
	.ascii	"include/linux/thread_info.h"
	.byte	0
	.section	__bug_table,"aw",@progbits
.Ltmp326:
	.word	.Ltmp325-.Ltmp326
	.hword	233
	.hword	2307
	.p2align	2
	.text
.Ltmp323:
	brk	#0x800
	//NO_APP
.LBB41_24:
	//APP
	//NO_APP
	ldr	w8, [x28, #2760]
	sub	w8, w8, #1
	str	w8, [x28, #2760]
	//APP
	//NO_APP
	ldr	x8, [x28, #16]
	subs	x8, x8, #1
	str	w8, [x28, #16]
	b.eq	.LBB41_27
	ldr	x8, [x28, #16]
	cbz	x8, .LBB41_27
.LBB41_26:
	mov	x19, #-14
	b	.LBB41_19
.LBB41_27:
	bl	preempt_schedule
	mov	x19, #-14
	b	.LBB41_19
.Lfunc_end41:
	.size	transition_copy_data, .Lfunc_end41-transition_copy_data

	.p2align	2
	.type	transition_lut_vm_fault,@function
transition_lut_vm_fault:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-64]!
	ldr	x8, [x0]
	stp	x20, x19, [sp, #48]
	str	x23, [sp, #16]
	mov	x29, sp
	stp	x22, x21, [sp, #32]
	ldr	x8, [x8, #144]
	ldr	x20, [x8, #48]
	cbz	x20, .LBB42_9
	ldr	w8, [x20, #8]
	cbz	w8, .LBB42_9
	ldr	x9, [x0, #16]
	mov	x19, x0
	mov	x8, xzr
	mov	w21, wzr
	ldr	x0, [x20]
	lsl	x22, x9, #12
	b	.LBB42_4
.LBB42_3:
	add	w21, w21, #1
	bl	sg_next
	ldr	w9, [x20, #8]
	mov	x8, x23
	cmp	w21, w9
	b.hs	.LBB42_9
.LBB42_4:
	ldr	w9, [x0, #12]
	add	x23, x8, x9
	subs	x8, x22, x8
	b.lo	.LBB42_3
	cmp	x22, x23
	b.hs	.LBB42_3
	ldr	x9, [x0]
	lsr	x8, x8, #6
	and	x8, x8, #0x3ffffffffffffc0
	and	x9, x9, #0xfffffffffffffffc
	add	x8, x9, x8
	ldr	x9, [x8, #8]
	sub	x10, x9, #1
	tst	w9, #0x1
	csel	x9, x8, x10, eq
	//APP
.Ltmp328:
	b	.Ltmp327
.Ltmp329:
	.section	.altinstructions,"a",@progbits
.Ltmp330:
	.word	.Ltmp328-.Ltmp330
.Ltmp331:
	.word	alt_cb_patch_nops-.Ltmp331
	.hword	32794
	.byte	.Ltmp329-.Ltmp328
	.byte	.Ltmp332-.Ltmp333
	.text

.Ltmp333:
.Ltmp332:

	//NO_APP
	add	x9, x9, #52
	mov	w10, #1
	//APP
	stadd	w10, [x9]

	//NO_APP
.LBB42_8:
	mov	w0, wzr
	str	x8, [x19, #80]
	b	.LBB42_10
.LBB42_9:
	mov	w0, #2
.LBB42_10:
	ldp	x20, x19, [sp, #48]
	ldp	x22, x21, [sp, #32]
	ldr	x23, [sp, #16]
	ldp	x29, x30, [sp], #64
	ldr	x30, [x18, #-8]!
	ret
.Ltmp327:
.LBB42_11:
	add	x9, x9, #52
	//APP
	// atomic_add
	prfm	pstl1strm, [x9]
.Ltmp334:
	ldxr	w10, [x9]
	add	w10, w10, #1
	stxr	w11, w10, [x9]
	cbnz	w11, .Ltmp334

	//NO_APP
	b	.LBB42_8
.Lfunc_end42:
	.size	transition_lut_vm_fault, .Lfunc_end42-transition_lut_vm_fault

	.p2align	2
	.type	waveform_version_read,@function
waveform_version_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	str	x19, [sp, #16]
	mov	x29, sp
	mov	x19, x2
	bl	epd_lut_get_wf_version
	adrp	x1, .L.str.199
	mov	x2, x0
	add	x1, x1, :lo12:.L.str.199
	mov	x0, x19
	bl	sprintf
	ldr	x19, [sp, #16]
	sxtw	x0, w0
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end43:
	.size	waveform_version_read, .Lfunc_end43-waveform_version_read

	.p2align	2
	.type	pmic_name_read,@function
pmic_name_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x8, global_ebc
	adrp	x1, .L.str.199
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.199
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	x8, [x8, #16]
	add	x2, x8, #8
	bl	sprintf
	sxtw	x0, w0
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end44:
	.size	pmic_name_read, .Lfunc_end44-pmic_name_read

	.p2align	2
	.type	pmic_temp_read,@function
pmic_temp_read:
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #16]
	add	x29, sp, #16
	str	x21, [sp, #32]
	stp	x20, x19, [sp, #48]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	adrp	x20, global_ebc
	mov	x19, x2
	str	x8, [sp, #8]
	ldr	x21, [x20, :lo12:global_ebc]
	str	wzr, [sp, #4]
	bl	ebc_power_on
	ldr	x0, [x21, #16]
	cbz	x0, .LBB45_2
	ldr	x8, [x0, #56]
	add	x1, sp, #4
	blr	x8
	b	.LBB45_3
.LBB45_2:
	ldr	x0, [x21, #104]
	add	x1, sp, #4
	bl	thermal_zone_get_temp
.LBB45_3:
	ldr	x21, [x20, :lo12:global_ebc]
	ldr	x20, [x21, #312]
	bl	ebc_get_power_off_delay
	bl	__msecs_to_jiffies
	mov	x3, x0
	add	x2, x21, #320
	mov	w0, #8
	mov	x1, x20
	bl	queue_delayed_work_on
	adrp	x1, .L.str.229
	ldr	w2, [sp, #4]
	add	x1, x1, :lo12:.L.str.229
	mov	x0, x19
	bl	sprintf
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldr	x9, [sp, #8]
	cmp	x8, x9
	b.ne	.LBB45_5
	ldp	x20, x19, [sp, #48]
	sxtw	x0, w0
	ldp	x29, x30, [sp, #16]
	ldr	x21, [sp, #32]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	ret
.LBB45_5:
	bl	__stack_chk_fail
.Lfunc_end45:
	.size	pmic_temp_read, .Lfunc_end45-pmic_temp_read

	.p2align	2
	.type	pmic_vcom_read,@function
pmic_vcom_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	adrp	x8, global_ebc
	str	x19, [sp, #16]
	mov	x19, x2
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	x0, [x8, #16]
	cbz	x0, .LBB46_2
	ldr	x8, [x0, #64]
	blr	x8
	mov	w2, w0
	b	.LBB46_3
.LBB46_2:
	ldr	x0, [x8, #24]
	bl	regulator_get_voltage
	mov	w8, #19923
	movk	w8, #4194, lsl #16
	smull	x8, w0, w8
	lsr	x9, x8, #63
	asr	x8, x8, #38
	add	w2, w8, w9
.LBB46_3:
	adrp	x1, .L.str.229
	mov	x0, x19
	add	x1, x1, :lo12:.L.str.229
	bl	sprintf
	ldr	x19, [sp, #16]
	sxtw	x0, w0
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end46:
	.size	pmic_vcom_read, .Lfunc_end46-pmic_vcom_read

	.p2align	2
	.type	pmic_vcom_write,@function
pmic_vcom_write:
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #16]
	adrp	x9, global_ebc
	str	x21, [sp, #32]
	mov	w1, wzr
	stp	x20, x19, [sp, #48]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	x20, x2
	add	x2, sp, #4
	mov	x0, x20
	add	x29, sp, #16
	mov	x19, x3
	str	x8, [sp, #8]
	ldr	x21, [x9, :lo12:global_ebc]
	str	wzr, [sp, #4]
	bl	kstrtouint
	cbnz	w0, .LBB47_5
	ldr	x0, [x21, #16]
	cbz	x0, .LBB47_3
	ldr	w1, [sp, #4]
	bl	ebc_pmic_set_vcom
	cbz	w0, .LBB47_7
	b	.LBB47_4
.LBB47_3:
	ldr	x0, [x21, #24]
	ldr	w1, [sp, #4]
	bl	ebc_regulator_set_vcom
	cbz	w0, .LBB47_7
.LBB47_4:
	adrp	x1, .L.str.232
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.232
	bl	_dev_err
	b	.LBB47_6
.LBB47_5:
	adrp	x1, .L.str.231
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.231
	mov	x2, x20
	bl	_dev_err
.LBB47_6:
	mov	x19, #-1
.LBB47_7:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldr	x9, [sp, #8]
	cmp	x8, x9
	b.ne	.LBB47_9
	mov	x0, x19
	ldr	x21, [sp, #32]
	ldp	x20, x19, [sp, #48]
	ldp	x29, x30, [sp, #16]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	ret
.LBB47_9:
	bl	__stack_chk_fail
.Lfunc_end47:
	.size	pmic_vcom_write, .Lfunc_end47-pmic_vcom_write

	.p2align	2
	.type	ebc_version_read,@function
ebc_version_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	mov	x0, x2
	adrp	x1, .L.str.199
	adrp	x2, .L.str.2
	add	x1, x1, :lo12:.L.str.199
	add	x2, x2, :lo12:.L.str.2
	mov	x29, sp
	bl	sprintf
	sxtw	x0, w0
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end48:
	.size	ebc_version_read, .Lfunc_end48-ebc_version_read

	.p2align	2
	.type	ebc_state_read,@function
ebc_state_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x8, global_ebc
	adrp	x1, .L.str.229
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.229
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	w2, [x8, #2080]
	bl	sprintf
	sxtw	x0, w0
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end49:
	.size	ebc_state_read, .Lfunc_end49-ebc_state_read

	.p2align	2
	.type	ebc_buf_state_read,@function
ebc_buf_state_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	mov	x0, x2
	mov	x29, sp
	bl	ebc_buf_state_show
	sxtw	x0, w0
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end50:
	.size	ebc_buf_state_read, .Lfunc_end50-ebc_buf_state_read

	.p2align	2
	.type	ebc_buf_state_write,@function
ebc_buf_state_write:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	adrp	x1, .L.str.236
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.236
	str	x19, [sp, #16]
	mov	x29, sp
	mov	x19, x3
	bl	strstr
	cbz	x0, .LBB51_2
	adrp	x8, global_ebc
	adrp	x1, dev_attr_dump_buffer
	add	x1, x1, :lo12:dev_attr_dump_buffer
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	x0, [x8, #304]
	bl	device_create_file
.LBB51_2:
	mov	x0, x19
	ldr	x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end51:
	.size	ebc_buf_state_write, .Lfunc_end51-ebc_buf_state_write

	.p2align	2
	.type	dump_buffer_write,@function
dump_buffer_write:
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #16]
	adrp	x9, global_ebc
	str	x21, [sp, #32]
	mov	w1, wzr
	stp	x20, x19, [sp, #48]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	x20, x2
	add	x2, sp, #4
	mov	x0, x20
	add	x29, sp, #16
	mov	x19, x3
	str	x8, [sp, #8]
	ldr	x21, [x9, :lo12:global_ebc]
	str	wzr, [sp, #4]
	bl	kstrtouint
	cbnz	w0, .LBB52_4
	mov	w8, #18636
	ldr	w9, [sp, #4]
	str	w9, [x21, x8]
.LBB52_2:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldr	x9, [sp, #8]
	cmp	x8, x9
	b.ne	.LBB52_5
	mov	x0, x19
	ldr	x21, [sp, #32]
	ldp	x20, x19, [sp, #48]
	ldp	x29, x30, [sp, #16]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	ret
.LBB52_4:
	adrp	x1, .L.str.231
	ldr	x0, [x21]
	add	x1, x1, :lo12:.L.str.231
	mov	x2, x20
	bl	_dev_err
	mov	x19, #-1
	b	.LBB52_2
.LBB52_5:
	bl	__stack_chk_fail
.Lfunc_end52:
	.size	dump_buffer_write, .Lfunc_end52-dump_buffer_write

	.p2align	2
	.type	auto_frame_state_read,@function
auto_frame_state_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x8, global_ebc
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	ldp	w10, w9, [x8, #188]
	mul	w9, w10, w9
	cbz	w9, .LBB53_4
	ldr	x11, [x8, #1480]
	mov	w10, wzr
.LBB53_2:
	ldrb	w8, [x11, w10, sxtw]
	cbnz	w8, .LBB53_5
	add	w10, w10, #1
	cmp	w9, w10
	b.ne	.LBB53_2
.LBB53_4:
	mov	w8, #2608
	mov	w0, #2
	strb	wzr, [x2, #2]
	strh	w8, [x2]
	b	.LBB53_6
.LBB53_5:
	adrp	x1, .L.str.229
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.229
	mov	w2, w8
	bl	sprintf
	sxtw	x0, w0
.LBB53_6:
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end53:
	.size	auto_frame_state_read, .Lfunc_end53-auto_frame_state_read

	.p2align	2
	.type	ebc_debug_level_read,@function
ebc_debug_level_read:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	adrp	x8, global_ebc
	adrp	x1, .L.str.229
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.229
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	ldr	w2, [x8, #2096]
	bl	sprintf
	sxtw	x0, w0
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end54:
	.size	ebc_debug_level_read, .Lfunc_end54-ebc_debug_level_read

	.p2align	2
	.type	ebc_debug_level_write,@function
ebc_debug_level_write:
	sub	sp, sp, #64
	str	x30, [x18], #8
	stp	x29, x30, [sp, #16]
	adrp	x9, global_ebc
	str	x21, [sp, #32]
	mov	w1, wzr
	stp	x20, x19, [sp, #48]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	x20, x2
	add	x2, sp, #4
	mov	x0, x20
	add	x29, sp, #16
	mov	x19, x3
	str	x8, [sp, #8]
	ldr	x21, [x9, :lo12:global_ebc]
	str	wzr, [sp, #4]
	bl	kstrtouint
	cbnz	w0, .LBB55_4
	ldr	w8, [sp, #4]
	str	w8, [x21, #2096]
.LBB55_2:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldr	x9, [sp, #8]
	cmp	x8, x9
	b.ne	.LBB55_5
	mov	x0, x19
	ldr	x21, [sp, #32]
	ldp	x20, x19, [sp, #48]
	ldp	x29, x30, [sp, #16]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #64
	ret
.LBB55_4:
	adrp	x1, .L.str.231
	ldr	x0, [x21, #304]
	add	x1, x1, :lo12:.L.str.231
	mov	x2, x20
	bl	_dev_err
	mov	x19, #-1
	b	.LBB55_2
.LBB55_5:
	bl	__stack_chk_fail
.Lfunc_end55:
	.size	ebc_debug_level_write, .Lfunc_end55-ebc_debug_level_write

	.p2align	2
	.type	wf_data_write,@function
wf_data_write:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-96]!
	adrp	x8, global_ebc
	adrp	x1, .L.str.236
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.236
	stp	x26, x25, [sp, #32]
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	stp	x28, x27, [sp, #16]
	stp	x24, x23, [sp, #48]
	stp	x22, x21, [sp, #64]
	ldr	w9, [x8, #1808]
	stp	x20, x19, [sp, #80]
	ldr	w10, [x8, #1816]
	mov	x19, x3
	ldr	x25, [x8, #1832]
	cmp	w10, w9
	csel	w20, w10, w9, gt
	bl	strstr
	cbnz	x0, .LBB56_2
.LBB56_1:
	mov	x0, x19
	ldp	x20, x19, [sp, #80]
	ldp	x22, x21, [sp, #64]
	ldp	x24, x23, [sp, #48]
	ldp	x26, x25, [sp, #32]
	ldp	x28, x27, [sp, #16]
	ldp	x29, x30, [sp], #96
	ldr	x30, [x18, #-8]!
	ret
.LBB56_2:
	adrp	x0, .L.str.242
	mov	w1, w20
	add	x0, x0, :lo12:.L.str.242
	bl	_printk
	adrp	x22, .L.str.243
	adrp	x23, .L.str.244
	mov	x21, xzr
	lsl	x26, x20, #10
	add	x22, x22, :lo12:.L.str.243
	add	x23, x23, :lo12:.L.str.244
.LBB56_3:
	mov	x24, xzr
.LBB56_4:
	mov	x0, x22
	mov	w1, w24
	mov	w2, w21
	bl	_printk
	cmp	w20, #1
	b.lt	.LBB56_7
	add	x8, x21, x24, lsl #5
	mov	x27, xzr
	add	x28, x25, x8
.LBB56_6:
	ldrb	w1, [x28, w27, sxtw]
	mov	x0, x23
	bl	_printk
	add	x27, x27, #1024
	cmp	x26, x27
	b.ne	.LBB56_6
.LBB56_7:
	add	x24, x24, #1
	cmp	x24, #32
	b.ne	.LBB56_4
	add	x21, x21, #1
	cmp	x21, #32
	b.ne	.LBB56_3
	b	.LBB56_1
.Lfunc_end56:
	.size	wf_data_write, .Lfunc_end56-wf_data_write

	.p2align	2
	.type	overlay_wf_data_write,@function
overlay_wf_data_write:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-96]!
	adrp	x8, global_ebc
	adrp	x1, .L.str.236
	mov	x0, x2
	add	x1, x1, :lo12:.L.str.236
	stp	x26, x25, [sp, #32]
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_ebc]
	stp	x20, x19, [sp, #80]
	stp	x28, x27, [sp, #16]
	mov	x19, x3
	stp	x24, x23, [sp, #48]
	ldr	x25, [x8, #1840]
	stp	x22, x21, [sp, #64]
	ldr	w20, [x8, #1812]
	bl	strstr
	cbnz	x0, .LBB57_2
.LBB57_1:
	mov	x0, x19
	ldp	x20, x19, [sp, #80]
	ldp	x22, x21, [sp, #64]
	ldp	x24, x23, [sp, #48]
	ldp	x26, x25, [sp, #32]
	ldp	x28, x27, [sp, #16]
	ldp	x29, x30, [sp], #96
	ldr	x30, [x18, #-8]!
	ret
.LBB57_2:
	adrp	x0, .L.str.242
	mov	w1, w20
	add	x0, x0, :lo12:.L.str.242
	bl	_printk
	adrp	x22, .L.str.243
	adrp	x23, .L.str.244
	mov	x21, xzr
	lsl	x26, x20, #10
	add	x22, x22, :lo12:.L.str.243
	add	x23, x23, :lo12:.L.str.244
.LBB57_3:
	mov	x24, xzr
.LBB57_4:
	mov	x0, x22
	mov	w1, w24
	mov	w2, w21
	bl	_printk
	cmp	w20, #1
	b.lt	.LBB57_7
	add	x8, x21, x24, lsl #5
	mov	x27, xzr
	add	x28, x25, x8
.LBB57_6:
	ldrb	w1, [x28, w27, sxtw]
	mov	x0, x23
	bl	_printk
	add	x27, x27, #1024
	cmp	x26, x27
	b.ne	.LBB57_6
.LBB57_7:
	add	x24, x24, #1
	cmp	x24, #32
	b.ne	.LBB57_4
	add	x21, x21, #1
	cmp	x21, #32
	b.ne	.LBB57_3
	b	.LBB57_1
.Lfunc_end57:
	.size	overlay_wf_data_write, .Lfunc_end57-overlay_wf_data_write

	.p2align	2
	.type	ori_wf_data_write,@function
ori_wf_data_write:
	sub	sp, sp, #160
	str	x30, [x18], #8
	stp	x29, x30, [sp, #64]
	add	x29, sp, #64
	stp	x28, x27, [sp, #80]
	mov	w9, #-1
	stp	x26, x25, [sp, #96]
	mov	w0, #262144
	stp	x24, x23, [sp, #112]
	mov	w1, #3520
	stp	x22, x21, [sp, #128]
	mov	x21, x2
	stp	x20, x19, [sp, #144]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	mov	x19, x3
	stur	x8, [x29, #-8]
	stp	w9, w9, [sp, #8]
	stp	xzr, xzr, [sp, #40]
	stp	xzr, xzr, [sp, #24]
	str	xzr, [sp, #16]
	bl	kmalloc_large
	cbnz	x0, .LBB58_2
	adrp	x0, .L.str.247
	add	x0, x0, :lo12:.L.str.247
	bl	_printk
	b	.LBB58_17
.LBB58_2:
	adrp	x1, .L.str.236
	mov	x20, x0
	str	x0, [sp, #40]
	add	x1, x1, :lo12:.L.str.236
	mov	x0, x21
	bl	strstr
	cbnz	x0, .LBB58_4
	adrp	x0, .L.str.250
	add	x0, x0, :lo12:.L.str.250
	b	.LBB58_16
.LBB58_4:
	adrp	x1, .L.str.248
	add	x2, sp, #12
	add	x1, x1, :lo12:.L.str.248
	add	x3, sp, #8
	mov	x0, x21
	bl	sscanf
	ldp	w3, w2, [sp, #8]
	adrp	x0, .L.str.249
	adrp	x1, .L__func__.ori_wf_data_write
	add	x0, x0, :lo12:.L.str.249
	add	x1, x1, :lo12:.L__func__.ori_wf_data_write
	bl	_printk
	ldr	w1, [sp, #12]
	cmp	w1, #8
	b.hi	.LBB58_15
	ldr	w2, [sp, #8]
	tbnz	w2, #31, .LBB58_15
	cmp	w2, #50
	b.gt	.LBB58_15
	add	x0, sp, #16
	mov	w3, #32
	bl	epd_lut_get_original
	ldr	w21, [sp, #16]
	adrp	x0, .L.str.242
	add	x0, x0, :lo12:.L.str.242
	mov	w1, w21
	bl	_printk
	adrp	x23, .L.str.243
	adrp	x24, .L.str.244
	mov	x22, xzr
	lsl	x26, x21, #10
	add	x23, x23, :lo12:.L.str.243
	add	x24, x24, :lo12:.L.str.244
.LBB58_8:
	mov	x25, xzr
.LBB58_9:
	mov	x0, x23
	mov	w1, w25
	mov	w2, w22
	bl	_printk
	cmp	w21, #1
	b.lt	.LBB58_12
	add	x8, x22, x25, lsl #5
	mov	x27, xzr
	add	x28, x20, x8
.LBB58_11:
	ldrb	w1, [x28, w27, sxtw]
	mov	x0, x24
	bl	_printk
	add	x27, x27, #1024
	cmp	x26, x27
	b.ne	.LBB58_11
.LBB58_12:
	add	x25, x25, #1
	cmp	x25, #32
	b.ne	.LBB58_9
	add	x22, x22, #1
	cmp	x22, #32
	b.ne	.LBB58_8
	mov	x0, x20
	bl	kfree
	b	.LBB58_17
.LBB58_15:
	adrp	x0, .L.str.251
	add	x0, x0, :lo12:.L.str.251
.LBB58_16:
	adrp	x1, .L__func__.ori_wf_data_write
	add	x1, x1, :lo12:.L__func__.ori_wf_data_write
	bl	_printk
.LBB58_17:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldur	x9, [x29, #-8]
	cmp	x8, x9
	b.ne	.LBB58_19
	mov	x0, x19
	ldp	x20, x19, [sp, #144]
	ldp	x22, x21, [sp, #128]
	ldp	x24, x23, [sp, #112]
	ldp	x26, x25, [sp, #96]
	ldp	x28, x27, [sp, #80]
	ldp	x29, x30, [sp, #64]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #160
	ret
.LBB58_19:
	bl	__stack_chk_fail
.Lfunc_end58:
	.size	ori_wf_data_write, .Lfunc_end58-ori_wf_data_write

	.p2align	2
	.type	ebc_suspend,@function
ebc_suspend:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	str	x19, [sp, #16]
	ldr	x19, [x0, #120]
	mov	w8, #1
	mov	x29, sp
	ldr	x0, [x19, #16]
	str	w8, [x19, #2124]
	cbz	x0, .LBB59_2
	ldr	x8, [x0, #40]
	blr	x8
.LBB59_2:
	adrp	x1, .L.str.252
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.252
	bl	_dev_info
	ldr	x19, [sp, #16]
	mov	w0, wzr
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end59:
	.size	ebc_suspend, .Lfunc_end59-ebc_suspend

	.p2align	2
	.type	ebc_resume,@function
ebc_resume:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	str	x19, [sp, #16]
	ldr	x19, [x0, #120]
	mov	x29, sp
	ldr	x0, [x19, #16]
	cbz	x0, .LBB60_2
	ldr	x8, [x0, #48]
	blr	x8
.LBB60_2:
	adrp	x1, .L.str.253
	ldr	x0, [x19, #304]
	add	x1, x1, :lo12:.L.str.253
	str	wzr, [x19, #2124]
	bl	_dev_info
	ldr	x19, [sp, #16]
	mov	w0, wzr
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end60:
	.size	ebc_resume, .Lfunc_end60-ebc_resume

	.type	ebc_driver,@object
	.data
	.p2align	3
ebc_driver:
	.xword	ebc_probe
	.xword	ebc_remove
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	.L.str
	.xword	0
	.xword	0
	.xword	0
	.byte	0
	.zero	3
	.word	0
	.xword	ebc_match
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	ebc_pm
	.xword	0
	.xword	0
	.xword	0
	.byte	0
	.byte	0
	.zero	6
	.size	ebc_driver, 208

	.type	__UNIQUE_ID___addressable_ebc_init530,@object
	.section	.discard.addressable,"aw",@progbits
	.p2align	3
__UNIQUE_ID___addressable_ebc_init530:
	.xword	ebc_init
	.size	__UNIQUE_ID___addressable_ebc_init530, 8

	.type	__exitcall_ebc_exit,@object
	.section	.exitcall.exit,"aw",@progbits
	.p2align	3
__exitcall_ebc_exit:
	.xword	ebc_exit
	.size	__exitcall_ebc_exit, 8

	.type	__UNIQUE_ID_author531,@object
	.section	.modinfo,"a",@progbits
__UNIQUE_ID_author531:
	.asciz	"rkebc.author=Zorro Liu <zorro.liu@rock-chips.com>"
	.size	__UNIQUE_ID_author531, 50

	.type	__UNIQUE_ID_description532,@object
__UNIQUE_ID_description532:
	.asciz	"rkebc.description=ROCKCHIP EBC Driver"
	.size	__UNIQUE_ID_description532, 38

	.type	__UNIQUE_ID_file533,@object
__UNIQUE_ID_file533:
	.asciz	"rkebc.file=$EBC_DEV_PATH/rkebc"
	.size	__UNIQUE_ID_file533, 31

	.type	__UNIQUE_ID_license534,@object
__UNIQUE_ID_license534:
	.asciz	"rkebc.license=GPL v2"
	.size	__UNIQUE_ID_license534, 21

	.type	.L.str,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"ebc-dev"
	.size	.L.str, 8

	.type	ebc_match,@object
	.section	.rodata,"a",@progbits
	.p2align	3
ebc_match:
	.zero	32
	.zero	32
	.asciz	"rockchip,ebc-dev\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000"
	.xword	0
	.zero	200
	.size	ebc_match, 400

	.type	ebc_pm,@object
	.p2align	3
ebc_pm:
	.xword	0
	.xword	0
	.xword	ebc_suspend
	.xword	ebc_resume
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.size	ebc_pm, 184

	.type	.L.str.1,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.1:
	.asciz	"rockchip ebc driver %s probe\n"
	.size	.L.str.1, 30

	.type	.L.str.2,@object
.L.str.2:
	.asciz	"10.09"
	.size	.L.str.2, 6

	.type	global_ebc,@object
	.local	global_ebc
	.comm	global_ebc,8,8
	.type	.L.str.3,@object
.L.str.3:
	.asciz	"ebc_tcon"
	.size	.L.str.3, 9

	.type	.L.str.4,@object
.L.str.4:
	.asciz	"not find ebc tcon\n"
	.size	.L.str.4, 19

	.type	.L.str.5,@object
.L.str.5:
	.asciz	"cpu-code"
	.size	.L.str.5, 9

	.type	.L.str.6,@object
.L.str.6:
	.asciz	"no chip info\n"
	.size	.L.str.6, 14

	.type	.L.str.7,@object
.L.str.7:
	.asciz	"read chip info failed\n"
	.size	.L.str.7, 23

	.type	.L.str.8,@object
.L.str.8:
	.asciz	"chip err\n"
	.size	.L.str.8, 10

	.type	.L.str.9,@object
.L.str.9:
	.asciz	"pmic"
	.size	.L.str.9, 5

	.type	.L.str.10,@object
.L.str.10:
	.asciz	"wrong pmics number\n"
	.size	.L.str.10, 20

	.type	.L.str.11,@object
.L.str.11:
	.asciz	"not find pmic node\n"
	.size	.L.str.11, 20

	.type	.L.str.12,@object
.L.str.12:
	.asciz	"not find pmic i2c client\n"
	.size	.L.str.12, 26

	.type	.L.str.13,@object
.L.str.13:
	.asciz	"use pmic%d\n"
	.size	.L.str.13, 12

	.type	.L.str.14,@object
.L.str.14:
	.asciz	"vcom"
	.size	.L.str.14, 5

	.type	.L.str.15,@object
.L.str.15:
	.asciz	"Failed to get vcom regulator\n"
	.size	.L.str.15, 30

	.type	.L.str.16,@object
.L.str.16:
	.asciz	"thermal-zone"
	.size	.L.str.16, 13

	.type	.L.str.17,@object
.L.str.17:
	.asciz	"Failed to get thermal zone name\n"
	.size	.L.str.17, 33

	.type	.L.str.18,@object
.L.str.18:
	.asciz	"Failed to get thermal device\n"
	.size	.L.str.18, 30

	.type	.L.str.19,@object
.L.str.19:
	.asciz	"Get PMIC device failed\n"
	.size	.L.str.19, 24

	.type	.L.str.20,@object
.L.str.20:
	.asciz	"ebc-use-soft-tcon"
	.size	.L.str.20, 18

	.type	.L.str.21,@object
.L.str.21:
	.asciz	"ebc-repair-enable"
	.size	.L.str.21, 18

	.type	.L.str.22,@object
.L.str.22:
	.asciz	"ebc-dmc-notify-enable"
	.size	.L.str.22, 22

	.type	.L.str.23,@object
.L.str.23:
	.asciz	"failed to probe panel: %d\n"
	.size	.L.str.23, 27

	.type	.L.str.24,@object
.L.str.24:
	.asciz	"soft tcon: not support panel rearrange and gate dummy\n"
	.size	.L.str.24, 55

	.type	.L.str.25,@object
.L.str.25:
	.asciz	"e6: not support panel rearrange and gate dummy\n"
	.size	.L.str.25, 48

	.type	.L.str.26,@object
.L.str.26:
	.asciz	"vpos1"
	.size	.L.str.26, 6

	.type	.L.str.27,@object
.L.str.27:
	.asciz	"Failed to get vpos1 regulator\n"
	.size	.L.str.27, 31

	.type	.L.str.28,@object
.L.str.28:
	.asciz	"vpos2"
	.size	.L.str.28, 6

	.type	.L.str.29,@object
.L.str.29:
	.asciz	"Failed to get vpos2 regulator\n"
	.size	.L.str.29, 31

	.type	.L.str.30,@object
.L.str.30:
	.asciz	"vpos3"
	.size	.L.str.30, 6

	.type	.L.str.31,@object
.L.str.31:
	.asciz	"Failed to get vpos3 regulator\n"
	.size	.L.str.31, 31

	.type	.L.str.32,@object
.L.str.32:
	.asciz	"vneg1"
	.size	.L.str.32, 6

	.type	.L.str.33,@object
.L.str.33:
	.asciz	"Failed to get vneg1 regulator\n"
	.size	.L.str.33, 31

	.type	.L.str.34,@object
.L.str.34:
	.asciz	"vneg2"
	.size	.L.str.34, 6

	.type	.L.str.35,@object
.L.str.35:
	.asciz	"Failed to get vneg2 regulator\n"
	.size	.L.str.35, 31

	.type	.L.str.36,@object
.L.str.36:
	.asciz	"vneg3"
	.size	.L.str.36, 6

	.type	.L.str.37,@object
.L.str.37:
	.asciz	"Failed to get vneg3 regulator\n"
	.size	.L.str.37, 31

	.type	.L.str.38,@object
.L.str.38:
	.asciz	"vgh"
	.size	.L.str.38, 4

	.type	.L.str.39,@object
.L.str.39:
	.asciz	"Failed to get vgh regulator\n"
	.size	.L.str.39, 29

	.type	.L.str.40,@object
.L.str.40:
	.asciz	"vgl"
	.size	.L.str.40, 4

	.type	.L.str.41,@object
.L.str.41:
	.asciz	"Failed to get vgl regulator\n"
	.size	.L.str.41, 29

	.type	.L.str.42,@object
.L.str.42:
	.asciz	"lut table init failed\n"
	.size	.L.str.42, 23

	.type	.L.str.43,@object
.L.str.43:
	.asciz	"buffer manage init failed\n"
	.size	.L.str.43, 27

	.type	.L.str.44,@object
.L.str.44:
	.asciz	"task init failed\n"
	.size	.L.str.44, 18

	.type	.L.str.45,@object
.L.str.45:
	.asciz	"rockchip ebc driver %s probe success\n"
	.size	.L.str.45, 38

	.type	.L.str.46,@object
.L.str.46:
	.asciz	"[%s] buffer %d frame done, wake up ebc task\n"
	.size	.L.str.46, 45

	.type	.L.str.47,@object
.L.str.47:
	.asciz	"FRAME DONE CALLBACK"
	.size	.L.str.47, 20

	.type	ebc_flip_wq,@object
	.data
	.p2align	3
ebc_flip_wq:
	.zero	4
	.zero	4
	.xword	ebc_flip_wq+8
	.xword	ebc_flip_wq+8
	.size	ebc_flip_wq, 24

	.type	ebc_task,@object
	.local	ebc_task
	.comm	ebc_task,8,8
	.type	.L.str.48,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.48:
	.asciz	"panel,width"
	.size	.L.str.48, 12

	.type	.L.str.49,@object
.L.str.49:
	.asciz	"panel,height"
	.size	.L.str.49, 13

	.type	.L.str.50,@object
.L.str.50:
	.asciz	"panel,vir_width"
	.size	.L.str.50, 16

	.type	.L.str.51,@object
.L.str.51:
	.asciz	"panel,vir_height"
	.size	.L.str.51, 17

	.type	.L.str.52,@object
.L.str.52:
	.asciz	"panel,sdck"
	.size	.L.str.52, 11

	.type	.L.str.53,@object
.L.str.53:
	.asciz	"panel,lsl"
	.size	.L.str.53, 10

	.type	.L.str.54,@object
.L.str.54:
	.asciz	"panel,lbl"
	.size	.L.str.54, 10

	.type	.L.str.55,@object
.L.str.55:
	.asciz	"panel,ldl"
	.size	.L.str.55, 10

	.type	.L.str.56,@object
.L.str.56:
	.asciz	"panel,lel"
	.size	.L.str.56, 10

	.type	.L.str.57,@object
.L.str.57:
	.asciz	"panel,gdck-sta"
	.size	.L.str.57, 15

	.type	.L.str.58,@object
.L.str.58:
	.asciz	"panel,lgonl"
	.size	.L.str.58, 12

	.type	.L.str.59,@object
.L.str.59:
	.asciz	"panel,fsl"
	.size	.L.str.59, 10

	.type	.L.str.60,@object
.L.str.60:
	.asciz	"panel,fbl"
	.size	.L.str.60, 10

	.type	.L.str.61,@object
.L.str.61:
	.asciz	"panel,fdl"
	.size	.L.str.61, 10

	.type	.L.str.62,@object
.L.str.62:
	.asciz	"panel,fel"
	.size	.L.str.62, 10

	.type	.L.str.63,@object
.L.str.63:
	.asciz	"panel,panel_bit_depth"
	.size	.L.str.63, 22

	.type	.L.str.64,@object
.L.str.64:
	.asciz	"panel,panel_16bit"
	.size	.L.str.64, 18

	.type	.L.str.65,@object
.L.str.65:
	.asciz	"panel,panel_color"
	.size	.L.str.65, 18

	.type	.L.str.66,@object
.L.str.66:
	.asciz	"panel,mirror"
	.size	.L.str.66, 13

	.type	.L.str.67,@object
.L.str.67:
	.asciz	"panel,width-mm"
	.size	.L.str.67, 15

	.type	.L.str.68,@object
.L.str.68:
	.asciz	"panel,height-mm"
	.size	.L.str.68, 16

	.type	.L.str.69,@object
.L.str.69:
	.asciz	"panel,rearrange"
	.size	.L.str.69, 16

	.type	.L.str.70,@object
.L.str.70:
	.asciz	"panel,sdoe_mode"
	.size	.L.str.70, 16

	.type	.L.str.71,@object
.L.str.71:
	.asciz	"panel,sdce_width"
	.size	.L.str.71, 17

	.type	.L.str.72,@object
.L.str.72:
	.asciz	"panel,lel-keep-clk"
	.size	.L.str.72, 19

	.type	.L.str.73,@object
.L.str.73:
	.asciz	"panel,gds"
	.size	.L.str.73, 10

	.type	.L.str.74,@object
.L.str.74:
	.asciz	"panel,gdl"
	.size	.L.str.74, 10

	.type	.L.str.75,@object
.L.str.75:
	.asciz	"pmic-early-power-on"
	.size	.L.str.75, 20

	.type	.L.str.76,@object
.L.str.76:
	.asciz	"waveform-region"
	.size	.L.str.76, 16

	.type	.L.str.77,@object
.L.str.77:
	.asciz	"wf,mode_table"
	.size	.L.str.77, 14

	.type	.L.str.78,@object
.L.str.78:
	.asciz	"%s:%d: failed to get mem from reserved\n"
	.size	.L.str.78, 40

	.type	.L__func__.ebc_lut_table_init,@object
.L__func__.ebc_lut_table_init:
	.asciz	"ebc_lut_table_init"
	.size	.L__func__.ebc_lut_table_init, 19

	.type	.L.str.79,@object
.L.str.79:
	.asciz	"Failed to read waveform from mem, we use waveform file from kernel file instead\n"
	.size	.L.str.79, 81

	.type	.L.str.80,@object
.L.str.80:
	.asciz	"Failed to read waveform file from kernel, no waveform!!!\n"
	.size	.L.str.80, 58

	.type	.L.str.81,@object
.L.str.81:
	.asciz	"ebc-disable-osd"
	.size	.L.str.81, 16

	.type	.L.str.82,@object
.L.str.82:
	.asciz	"ebc-disable-transition"
	.size	.L.str.82, 23

	.type	.L.str.83,@object
.L.str.83:
	.asciz	"failed to alloc direct fb %d from cma, ret: %d\n"
	.size	.L.str.83, 48

	.type	.L.str.84,@object
.L.str.84:
	.asciz	"failed to alloc prev_lut_buf from cma, ret: %d\n"
	.size	.L.str.84, 48

	.type	.L.str.85,@object
.L.str.85:
	.asciz	"failed to alloc cur_lut_buf from cma, ret: %d\n"
	.size	.L.str.85, 47

	.type	.L.str.86,@object
.L.str.86:
	.asciz	"failed to init ebc buf list, ret: %d\n"
	.size	.L.str.86, 38

	.type	.L.str.87,@object
.L.str.87:
	.asciz	"ebc-max-dsp-fb-num"
	.size	.L.str.87, 19

	.type	.L.str.88,@object
.L.str.88:
	.asciz	"failed to alloc ebc dsp buf %d from dma, ret: %d\n"
	.size	.L.str.88, 50

	.type	.L.str.89,@object
.L.str.89:
	.asciz	"failed to init ebc dsp buf %d from dma, ret: %d\n"
	.size	.L.str.89, 49

	.type	.L.str.90,@object
.L.str.90:
	.asciz	"failed to alloc ebc osd buf %d from dma, ret: %d\n"
	.size	.L.str.90, 50

	.type	.L.str.91,@object
.L.str.91:
	.asciz	"failed to alloc wf_table_buf from dma, ret: %d\n"
	.size	.L.str.91, 48

	.type	.L.str.92,@object
.L.str.92:
	.asciz	"failed to alloc trans_lut from dma, ret: %d\n"
	.size	.L.str.92, 45

	.type	.L.str.93,@object
.L.str.93:
	.asciz	"failed to alloc auto_image_new from dma, ret: %d\n"
	.size	.L.str.93, 50

	.type	.L.str.94,@object
.L.str.94:
	.asciz	"failed to alloc auto_image_old from dma, ret: %d\n"
	.size	.L.str.94, 50

	.type	.L.str.95,@object
.L.str.95:
	.asciz	"failed to alloc auto_frame_count from dma, ret: %d\n"
	.size	.L.str.95, 52

	.type	.L.str.96,@object
.L.str.96:
	.asciz	"failed to alloc auto_wf_index from dma, ret: %d\n"
	.size	.L.str.96, 49

	.type	.L.str.97,@object
.L.str.97:
	.asciz	"failed to alloc auto_image_ghost from dma, ret: %d\n"
	.size	.L.str.97, 52

	.type	.L.str.98,@object
.L.str.98:
	.asciz	"ebc_assist_task"
	.size	.L.str.98, 16

	.type	ebc_assist_task,@object
	.local	ebc_assist_task
	.comm	ebc_assist_task,8,8
	.type	.L.str.99,@object
.L.str.99:
	.asciz	"failed to create ebc_assist_task thread\n"
	.size	.L.str.99, 41

	.type	.L.str.100,@object
.L.str.100:
	.asciz	"ebc_task"
	.size	.L.str.100, 9

	.type	.L.str.101,@object
.L.str.101:
	.asciz	"failed to create ebc_task thread\n"
	.size	.L.str.101, 34

	.type	.L.str.102,@object
.L.str.102:
	.asciz	"%s"
	.size	.L.str.102, 3

	.type	.L.str.103,@object
.L.str.103:
	.asciz	"ebc-delay-power-off"
	.size	.L.str.103, 20

	.type	.L.str.104,@object
.L.str.104:
	.asciz	"ebc-delay-ghost-rm"
	.size	.L.str.104, 19

	.type	.L.str.105,@object
.L.str.105:
	.asciz	"[%s] no buffer, exit\n"
	.size	.L.str.105, 22

	.type	.L.str.106,@object
.L.str.106:
	.asciz	"ASSIST THREAD"
	.size	.L.str.106, 14

	.type	.L.str.107,@object
.L.str.107:
	.asciz	"[%s] lut mode, exit\n"
	.size	.L.str.107, 21

	.type	ebc_assist_finished_flag_sem,@object
	.data
	.p2align	3
ebc_assist_finished_flag_sem:
	.zero	4
	.word	1
	.xword	ebc_assist_finished_flag_sem+8
	.xword	ebc_assist_finished_flag_sem+8
	.size	ebc_assist_finished_flag_sem, 24

	.type	_8_8bit_mask,@object
	.section	.rodata,"a",@progbits
	.p2align	3
_8_8bit_mask:
	.xword	255
	.xword	65280
	.xword	16711680
	.xword	4278190080
	.xword	1095216660480
	.xword	280375465082880
	.xword	71776119061217280
	.xword	-72057594037927936
	.size	_8_8bit_mask, 64

	.type	.L.str.108,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.108:
	.asciz	"[%s] overlay left: %s, frame total: %d, osd frame total: %d\n"
	.size	.L.str.108, 61

	.type	.L.str.109,@object
.L.str.109:
	.asciz	"LEFT"
	.size	.L.str.109, 5

	.type	.L.str.110,@object
.L.str.110:
	.asciz	"RIGHT"
	.size	.L.str.110, 6

	.type	.L.str.111,@object
.L.str.111:
	.asciz	"LEFT & RIGHT"
	.size	.L.str.111, 13

	.type	.L.str.112,@object
.L.str.112:
	.asciz	"true"
	.size	.L.str.112, 5

	.type	.L.str.113,@object
.L.str.113:
	.asciz	"false"
	.size	.L.str.113, 6

	.type	.L__const.get_normal_panel_soft_tcon_auto_data.shift_table,@object
.L__const.get_normal_panel_soft_tcon_auto_data.shift_table:
	.asciz	"\016\f\n\b\006\004\002"
	.size	.L__const.get_normal_panel_soft_tcon_auto_data.shift_table, 8

	.type	.L.str.114,@object
.L.str.114:
	.asciz	"[%s] overlay left: %d, frame total: (%d, %d, %d)\n"
	.size	.L.str.114, 50

	.type	.L__const.get_normal_panel_auto_data.shift_table,@object
	.section	.rodata.cst8,"aM",@progbits,8
.L__const.get_normal_panel_auto_data.shift_table:
	.ascii	"\000\002\004\006\b\n\f\016"
	.size	.L__const.get_normal_panel_auto_data.shift_table, 8

	.type	.L__const.get_normal_panel_auto_data.offset,@object
	.section	.rodata.cst32,"aM",@progbits,32
	.p2align	2
.L__const.get_normal_panel_auto_data.offset:
	.word	3
	.word	11
	.word	19
	.word	27
	.word	35
	.word	43
	.word	51
	.word	59
	.size	.L__const.get_normal_panel_auto_data.offset, 32

	.type	.L.str.115,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.115:
	.asciz	"[%s] dsp frameidx: %d, transition done: %s\n"
	.size	.L.str.115, 44

	.type	.L.str.116,@object
.L.str.116:
	.asciz	"[%s] dsp frameidx: %d, transition (enable: %s, done: %s)\n"
	.size	.L.str.116, 58

	.type	.L.str.117,@object
.L.str.117:
	.asciz	"[%s] ebc is idle, can flip the next frame\n"
	.size	.L.str.117, 43

	.type	.L.str.118,@object
.L.str.118:
	.asciz	"MAIN THREAD"
	.size	.L.str.118, 12

	.type	.L.str.119,@object
.L.str.119:
	.asciz	"[%s] refresh done: %s (dsp: %s, auto: %s), fast ghost rm done: %s\n"
	.size	.L.str.119, 67

	.type	.L.str.120,@object
.L.str.120:
	.asciz	"[%s] need bw repair: 0x%02x, need transition: 0x%02x, need overlay: 0x%02x, next buf_mode: %d, next ebc_mode: %d\n"
	.size	.L.str.120, 114

	.type	.L.str.121,@object
.L.str.121:
	.asciz	"[%s] has osd buf: %s, update osd buf: %s, has dsp buf: %s, update dsp buf: %s\n"
	.size	.L.str.121, 79

	.type	.L.str.122,@object
.L.str.122:
	.asciz	"[%s] update dsp buf for next frame\n"
	.size	.L.str.122, 36

	.type	.L.str.123,@object
.L.str.123:
	.asciz	"[%s] dsp frame start, refresh mode = %d, dsp frame total = %d, total = %d\n"
	.size	.L.str.123, 75

	.type	.L.str.124,@object
.L.str.124:
	.asciz	"FRAME START"
	.size	.L.str.124, 12

	.type	.L.str.125,@object
.L.str.125:
	.asciz	"[%s] dsp frame start, refresh mode = %d, dsp frame total = %d\n"
	.size	.L.str.125, 63

	.type	.L.str.126,@object
.L.str.126:
	.asciz	"[%s] no buffer, frame done\n"
	.size	.L.str.126, 28

	.type	.L.str.127,@object
.L.str.127:
	.asciz	"[%s] update lut_buffer"
	.size	.L.str.127, 23

	.type	.L.str.128,@object
.L.str.128:
	.asciz	"[%s] update direct_buffer-%d, index: %d, mode: %d\n"
	.size	.L.str.128, 51

	.type	.L.str.129,@object
.L.str.129:
	.asciz	"[%s] start processing the next frame\n"
	.size	.L.str.129, 38

	.type	.L.str.130,@object
.L.str.130:
	.asciz	"/data/dump/prev_dsp_buf_Y8_%d_%d.yuv"
	.size	.L.str.130, 37

	.type	.L.str.131,@object
.L.str.131:
	.asciz	"/data/dump/curr_dsp_buf_Y8_%d_%d.yuv"
	.size	.L.str.131, 37

	.type	.L.str.132,@object
.L.str.132:
	.asciz	"/data/dump/auto_image_old_Y8_%d_%d.yuv"
	.size	.L.str.132, 39

	.type	.L.str.133,@object
.L.str.133:
	.asciz	"/data/dump/auto_image_new_Y8_%d_%d.yuv"
	.size	.L.str.133, 39

	.type	.L.str.134,@object
.L.str.134:
	.asciz	"/data/dump/auto_frame_count_Y8_%d_%d.yuv"
	.size	.L.str.134, 41

	.type	.L.str.135,@object
.L.str.135:
	.asciz	"/data/dump/auto_image_ghost_Y8_%d_%d.yuv"
	.size	.L.str.135, 41

	.type	.L.str.136,@object
.L.str.136:
	.asciz	"/data/dump/auto_wf_index_Y8_%d_%d.yuv"
	.size	.L.str.136, 38

	.type	.L.str.137,@object
.L.str.137:
	.asciz	"/data/dump/direct_buffer_Y8_%d_%d.yuv"
	.size	.L.str.137, 38

	.type	.L.str.138,@object
.L.str.138:
	.asciz	"\0013Data size %zu is less than 4 bytes\n"
	.size	.L.str.138, 38

	.type	.L.str.139,@object
.L.str.139:
	.asciz	"\0013Failed to open file: %s\n"
	.size	.L.str.139, 27

	.type	.L.str.140,@object
.L.str.140:
	.asciz	"\0013Failed to write to file: %ld\n"
	.size	.L.str.140, 32

	.type	.L.str.141,@object
.L.str.141:
	.asciz	"\0013Failed to write index: %ld\n"
	.size	.L.str.141, 30

	.type	.L.str.142,@object
.L.str.142:
	.asciz	"\0016Successfully wrote %zu bytes to %s\n"
	.size	.L.str.142, 38

	.type	.L.str.143,@object
.L.str.143:
	.asciz	"[%s] ebc hw power on res:%d\n"
	.size	.L.str.143, 29

	.type	.L.str.144,@object
.L.str.144:
	.asciz	"POWER"
	.size	.L.str.144, 6

	.type	.L.str.145,@object
.L.str.145:
	.asciz	"[%s] ebc hw power off res:%d\n"
	.size	.L.str.145, 30

	.type	.L.str.146,@object
.L.str.146:
	.asciz	"[%s] lut mode flip buffer\n"
	.size	.L.str.146, 27

	.type	.L.str.147,@object
.L.str.147:
	.asciz	"FLIP"
	.size	.L.str.147, 5

	.type	.L.str.148,@object
.L.str.148:
	.asciz	"Failed to set lut in lut mode:%d\n"
	.size	.L.str.148, 34

	.type	.L.str.149,@object
.L.str.149:
	.asciz	"[%s] update lut in lut mode:%d\n"
	.size	.L.str.149, 32

	.type	.L.str.150,@object
.L.str.150:
	.asciz	"[%s] direct mode flip buffer index: %d\n"
	.size	.L.str.150, 40

	.type	.L.str.151,@object
.L.str.151:
	.asciz	"[%s] Ghost exists, needs to be cleared in fast mode.\n"
	.size	.L.str.151, 54

	.type	.L.str.152,@object
.L.str.152:
	.asciz	"AUTO UPDATE"
	.size	.L.str.152, 12

	.type	.L.str.153,@object
.L.str.153:
	.asciz	"[%s] No new ghost needs to be cleared in fast mode.\n"
	.size	.L.str.153, 53

	.type	.L.str.154,@object
.L.str.154:
	.asciz	"[%s] No ghost needs to be cleared in fast mode.\n"
	.size	.L.str.154, 49

	.type	.L.str.155,@object
.L.str.155:
	.asciz	"[%s] ebc buffer mode %d not support!\n"
	.size	.L.str.155, 38

	.type	.L.str.156,@object
.L.str.156:
	.asciz	"[%s] force temperature = %d\n"
	.size	.L.str.156, 29

	.type	.L.str.157,@object
.L.str.157:
	.asciz	"[%s] temperature = %d, out of range0~50 ,use 0 \n"
	.size	.L.str.157, 49

	.type	.L.str.158,@object
.L.str.158:
	.asciz	"[%s] temperature = %d, out of range0~50 ,use 50 \n"
	.size	.L.str.158, 50

	.type	.L.str.159,@object
.L.str.159:
	.asciz	"[%s] ebc_pmic_read_temp failed, ret = %d, use 25 \n"
	.size	.L.str.159, 51

	.type	.L.str.160,@object
.L.str.160:
	.asciz	"[%s] overlay_temperature_fix(%d) overflow, use origin temperature(%d)\n"
	.size	.L.str.160, 71

	.type	.L.str.161,@object
.L.str.161:
	.asciz	"[%s] normal_temperature_fix(%d) overflow, use origin temperature(%d)\n"
	.size	.L.str.161, 70

	.type	.L.str.162,@object
.L.str.162:
	.asciz	"[%s] lut update use overlay_temperature_fix = %d normal_temperature_fix = %d, origin temperature = %d\n"
	.size	.L.str.162, 103

	.type	.L.str.163,@object
.L.str.163:
	.asciz	"[%s] get lut data failed\n"
	.size	.L.str.163, 26

	.type	.L.str.164,@object
.L.str.164:
	.asciz	"[%s] osd lut data update\n"
	.size	.L.str.164, 26

	.type	.L.str.165,@object
.L.str.165:
	.asciz	"[%s] dsp lut data update\n"
	.size	.L.str.165, 26

	.type	.L.str.166,@object
.L.str.166:
	.asciz	"[%s] break part or full mode to auto mode\n"
	.size	.L.str.166, 43

	.type	.L.str.167,@object
.L.str.167:
	.asciz	"MODE CHANGE"
	.size	.L.str.167, 12

	.type	.L.str.168,@object
.L.str.168:
	.asciz	"[%s] part or full mode to auto(full) mode\n"
	.size	.L.str.168, 43

	.type	.L.str.169,@object
.L.str.169:
	.asciz	"[%s] part or full mode to auto(%s) mode\n"
	.size	.L.str.169, 41

	.type	.L.str.170,@object
.L.str.170:
	.asciz	"part"
	.size	.L.str.170, 5

	.type	.L.str.171,@object
.L.str.171:
	.asciz	"fast"
	.size	.L.str.171, 5

	.type	.L.str.172,@object
.L.str.172:
	.asciz	"[%s] overlay osd buffer to auto_image (%d, %d, %d, %d), complete: %s, overlay_enable: %d\n"
	.size	.L.str.172, 90

	.type	.L.str.173,@object
.L.str.173:
	.asciz	"[%s] update fast buffer to auto_image\n"
	.size	.L.str.173, 39

	.type	.L.str.174,@object
.L.str.174:
	.asciz	"[%s] auto(part) mode to part or full mode\n"
	.size	.L.str.174, 43

	.type	.L.str.175,@object
.L.str.175:
	.asciz	"[%s] auto(full) mode to part or full mode\n"
	.size	.L.str.175, 43

	.type	.L.str.176,@object
.L.str.176:
	.asciz	"[%s] auto(fast) mode to part or full mode\n"
	.size	.L.str.176, 43

	.type	.L.str.177,@object
.L.str.177:
	.asciz	"[%s] update %s buffer to auto_image, complete: %s\n"
	.size	.L.str.177, 51

	.type	.L.str.178,@object
.L.str.178:
	.asciz	"full"
	.size	.L.str.178, 5

	.type	.L.str.179,@object
.L.str.179:
	.asciz	"[%s] transition type = %d, step = %d\n"
	.size	.L.str.179, 38

	.type	.L.str.180,@object
.L.str.180:
	.asciz	"TRANSITION"
	.size	.L.str.180, 11

	.type	.L.str.181,@object
.L.str.181:
	.asciz	"ebc"
	.size	.L.str.181, 4

	.type	.L.str.182,@object
.L.str.182:
	.asciz	"ulogo_addr="
	.size	.L.str.182, 12

	.type	.L.str.183,@object
.L.str.183:
	.asciz	"klogo_addr="
	.size	.L.str.183, 12

	.type	.L.str.184,@object
.L.str.184:
	.asciz	"memory-region"
	.size	.L.str.184, 14

	.type	.L.str.185,@object
.L.str.185:
	.asciz	"Couldn't address to resource for reserved memory\n"
	.size	.L.str.185, 50

	.type	.L.str.186,@object
.L.str.186:
	.asciz	"reserved memory not enough, need 0x%x\n"
	.size	.L.str.186, 39

	.type	.L.str.187,@object
.L.str.187:
	.asciz	"Couldn't remap for reserved memory\n"
	.size	.L.str.187, 36

	.type	.L.str.188,@object
.L.str.188:
	.asciz	"ulogo_addr=0x%x"
	.size	.L.str.188, 16

	.type	.L.str.189,@object
.L.str.189:
	.asciz	"have ulogo display, ulogo addr = 0x%x\n"
	.size	.L.str.189, 39

	.type	.L.str.190,@object
.L.str.190:
	.asciz	"malloc ulogo buffer failed\n"
	.size	.L.str.190, 28

	.type	.L.str.191,@object
.L.str.191:
	.asciz	"klogo_addr=0x%x"
	.size	.L.str.191, 16

	.type	.L.str.192,@object
.L.str.192:
	.asciz	"need show klogo, klogo addr = 0x%x\n"
	.size	.L.str.192, 36

	.type	.L.str.193,@object
.L.str.193:
	.asciz	"malloc klogo buffer failed\n"
	.size	.L.str.193, 28

	.type	.L.str.194,@object
.L.str.194:
	.asciz	"no uboot logo, panel init\n"
	.size	.L.str.194, 27

	.type	.L.str.195,@object
.L.str.195:
	.asciz	"ebc_dev_logo"
	.size	.L.str.195, 13

	.type	.L.str.196,@object
.L.str.196:
	.asciz	"ebc_dev_reset"
	.size	.L.str.196, 14

	.type	ebc_misc,@object
	.data
	.p2align	3
ebc_misc:
	.word	243
	.zero	4
	.xword	.L.str.181
	.xword	ebc_ops
	.zero	16
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.hword	0
	.zero	6
	.size	ebc_misc, 80

	.type	waveform_misc,@object
	.p2align	3
waveform_misc:
	.word	244
	.zero	4
	.xword	.L.str.222
	.xword	waveform_ops
	.zero	16
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.hword	0
	.zero	6
	.size	waveform_misc, 80

	.type	transition_lut_misc,@object
	.p2align	3
transition_lut_misc:
	.word	245
	.zero	4
	.xword	.L.str.224
	.xword	transition_lut_ops
	.zero	16
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.hword	0
	.zero	6
	.size	transition_lut_misc, 80

	.type	dev_attr_waveform_version,@object
	.p2align	3
dev_attr_waveform_version:
	.xword	.L.str.226
	.hword	292
	.zero	6
	.xword	waveform_version_read
	.xword	0
	.size	dev_attr_waveform_version, 32

	.type	dev_attr_pmic_name,@object
	.p2align	3
dev_attr_pmic_name:
	.xword	.L.str.227
	.hword	292
	.zero	6
	.xword	pmic_name_read
	.xword	0
	.size	dev_attr_pmic_name, 32

	.type	dev_attr_pmic_temp,@object
	.p2align	3
dev_attr_pmic_temp:
	.xword	.L.str.228
	.hword	292
	.zero	6
	.xword	pmic_temp_read
	.xword	0
	.size	dev_attr_pmic_temp, 32

	.type	dev_attr_pmic_vcom,@object
	.p2align	3
dev_attr_pmic_vcom:
	.xword	.L.str.230
	.hword	420
	.zero	6
	.xword	pmic_vcom_read
	.xword	pmic_vcom_write
	.size	dev_attr_pmic_vcom, 32

	.type	dev_attr_ebc_version,@object
	.p2align	3
dev_attr_ebc_version:
	.xword	.L.str.233
	.hword	292
	.zero	6
	.xword	ebc_version_read
	.xword	0
	.size	dev_attr_ebc_version, 32

	.type	dev_attr_ebc_state,@object
	.p2align	3
dev_attr_ebc_state:
	.xword	.L.str.234
	.hword	292
	.zero	6
	.xword	ebc_state_read
	.xword	0
	.size	dev_attr_ebc_state, 32

	.type	dev_attr_ebc_buf_state,@object
	.p2align	3
dev_attr_ebc_buf_state:
	.xword	.L.str.235
	.hword	420
	.zero	6
	.xword	ebc_buf_state_read
	.xword	ebc_buf_state_write
	.size	dev_attr_ebc_buf_state, 32

	.type	dev_attr_auto_frame_state,@object
	.p2align	3
dev_attr_auto_frame_state:
	.xword	.L.str.238
	.hword	292
	.zero	6
	.xword	auto_frame_state_read
	.xword	0
	.size	dev_attr_auto_frame_state, 32

	.type	dev_attr_ebc_debug_level,@object
	.p2align	3
dev_attr_ebc_debug_level:
	.xword	.L.str.240
	.hword	420
	.zero	6
	.xword	ebc_debug_level_read
	.xword	ebc_debug_level_write
	.size	dev_attr_ebc_debug_level, 32

	.type	dev_attr_wf_data,@object
	.p2align	3
dev_attr_wf_data:
	.xword	.L.str.241
	.hword	128
	.zero	6
	.xword	0
	.xword	wf_data_write
	.size	dev_attr_wf_data, 32

	.type	dev_attr_overlay_wf_data,@object
	.p2align	3
dev_attr_overlay_wf_data:
	.xword	.L.str.245
	.hword	128
	.zero	6
	.xword	0
	.xword	overlay_wf_data_write
	.size	dev_attr_overlay_wf_data, 32

	.type	dev_attr_ori_wf_data,@object
	.p2align	3
dev_attr_ori_wf_data:
	.xword	.L.str.246
	.hword	128
	.zero	6
	.xword	0
	.xword	ori_wf_data_write
	.size	dev_attr_ori_wf_data, 32

	.type	ebc_ops,@object
	.section	.rodata,"a",@progbits
	.p2align	3
ebc_ops:
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	ebc_io_ctl
	.xword	ebc_io_ctl
	.xword	0
	.xword	0
	.xword	ebc_open
	.xword	0
	.xword	ebc_release
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.size	ebc_ops, 272

	.type	.L.str.198,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.198:
	.asciz	"%s: argp NULL\n"
	.size	.L.str.198, 15

	.type	.L__func__.ebc_io_ctl,@object
.L__func__.ebc_io_ctl:
	.asciz	"ebc_io_ctl"
	.size	.L__func__.ebc_io_ctl, 11

	.type	.L.str.199,@object
.L.str.199:
	.asciz	"%s\n"
	.size	.L.str.199, 4

	.type	.L.str.200,@object
.L.str.200:
	.asciz	"[%s] EBC_GET_OSD_BUFFER\n"
	.size	.L.str.200, 25

	.type	.L.str.201,@object
.L.str.201:
	.asciz	"IOCTL"
	.size	.L.str.201, 6

	.type	.L.str.202,@object
.L.str.202:
	.asciz	"[%s] EBC_GET_BUFFER\n"
	.size	.L.str.202, 21

	.type	.L.str.203,@object
.L.str.203:
	.asciz	"[%s] EBC_SEND_BUFFER\n"
	.size	.L.str.203, 22

	.type	.L.str.204,@object
.L.str.204:
	.asciz	"[%s] EBC_SEND_OSD_BUFFER\n"
	.size	.L.str.204, 26

	.type	.L.str.205,@object
.L.str.205:
	.asciz	"EBC_GET_BUFFER_INFO failed\n"
	.size	.L.str.205, 28

	.type	.L.str.206,@object
.L.str.206:
	.asciz	"[%s] EBC_FB_BLANK\n"
	.size	.L.str.206, 19

	.type	.L.str.207,@object
.L.str.207:
	.asciz	"[%s] EBC_FB_UNBLANK\n"
	.size	.L.str.207, 21

	.type	.L.str.208,@object
.L.str.208:
	.asciz	"[%s] EBC_BW_ENABLE_REPAIR = %d\n"
	.size	.L.str.208, 32

	.type	.L.str.209,@object
.L.str.209:
	.asciz	"[%s] EBC_BW_BALANCE_CTL = %d\n"
	.size	.L.str.209, 30

	.type	.L.str.210,@object
.L.str.210:
	.asciz	"[%s] EBC_BW_GHOST_RM_LEVEL = %d\n"
	.size	.L.str.210, 33

	.type	.L.str.211,@object
.L.str.211:
	.asciz	"[%s] EBC_BW_GHOST_RM_TYPE = %d\n"
	.size	.L.str.211, 32

	.type	.L.str.212,@object
.L.str.212:
	.asciz	"[%s] EBC_SET_TRANSITION_WIDTH = %d\n"
	.size	.L.str.212, 36

	.type	.L.str.213,@object
.L.str.213:
	.asciz	"[%s] EBC_SET_TRANSITION_TYPE = %d\n"
	.size	.L.str.213, 35

	.type	.L.str.214,@object
.L.str.214:
	.asciz	"[%s] EBC_SET_FORCE_TEMPERATURE = %d\n"
	.size	.L.str.214, 37

	.type	.L.str.215,@object
.L.str.215:
	.asciz	"[%s] EBC_ENABLE_WAVEFORM_FIX = %d\n"
	.size	.L.str.215, 35

	.type	.L.str.216,@object
.L.str.216:
	.asciz	"[%s] EBC_SET_REGAL_TYPE = %d\n"
	.size	.L.str.216, 30

	.type	.L.str.217,@object
.L.str.217:
	.asciz	"[%s] EBC_FORCE_FULL_USE_REGAL = %d\n"
	.size	.L.str.217, 36

	.type	.L.str.218,@object
.L.str.218:
	.asciz	"[%s] EBC_ENABLE_OVERLAY, overlay_enable_control = %d\n"
	.size	.L.str.218, 54

	.type	.L.str.219,@object
.L.str.219:
	.asciz	"[%s] EBC_DISABLE_OVERLAY, overlay_enable_control = %d\n"
	.size	.L.str.219, 55

	.type	.L.str.220,@object
.L.str.220:
	.asciz	"[%s] EBC_PEN_PROXIMITY = %d, overlay_enable_control = %d\n"
	.size	.L.str.220, 58

	.type	.L.str.221,@object
.L.str.221:
	.asciz	"%s: unknown cmd\n"
	.size	.L.str.221, 17

	.type	.L.str.222,@object
.L.str.222:
	.asciz	"waveform"
	.size	.L.str.222, 9

	.type	waveform_ops,@object
	.section	.rodata,"a",@progbits
	.p2align	3
waveform_ops:
	.xword	0
	.xword	waveform_llseek
	.xword	waveform_read
	.xword	waveform_write
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	waveform_mmap
	.xword	0
	.xword	waveform_open
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.size	waveform_ops, 272

	.type	.L.str.224,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.224:
	.asciz	"transition_lut"
	.size	.L.str.224, 15

	.type	transition_lut_ops,@object
	.section	.rodata,"a",@progbits
	.p2align	3
transition_lut_ops:
	.xword	0
	.xword	transition_lut_llseek
	.xword	transition_lut_read
	.xword	transition_lut_write
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	transition_lut_mmap
	.xword	0
	.xword	transition_lut_open
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.size	transition_lut_ops, 272

	.type	transition_lut_vm_ops,@object
	.p2align	3
transition_lut_vm_ops:
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	transition_lut_vm_fault
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.xword	0
	.size	transition_lut_vm_ops, 112

	.type	.L.str.226,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.226:
	.asciz	"waveform_version"
	.size	.L.str.226, 17

	.type	.L.str.227,@object
.L.str.227:
	.asciz	"pmic_name"
	.size	.L.str.227, 10

	.type	.L.str.228,@object
.L.str.228:
	.asciz	"pmic_temp"
	.size	.L.str.228, 10

	.type	.L.str.229,@object
.L.str.229:
	.asciz	"%d\n"
	.size	.L.str.229, 4

	.type	.L.str.230,@object
.L.str.230:
	.asciz	"pmic_vcom"
	.size	.L.str.230, 10

	.type	.L.str.231,@object
.L.str.231:
	.asciz	"invalid value = %s\n"
	.size	.L.str.231, 20

	.type	.L.str.232,@object
.L.str.232:
	.asciz	"set vcom value failed\n"
	.size	.L.str.232, 23

	.type	.L.str.233,@object
.L.str.233:
	.asciz	"ebc_version"
	.size	.L.str.233, 12

	.type	.L.str.234,@object
.L.str.234:
	.asciz	"ebc_state"
	.size	.L.str.234, 10

	.type	.L.str.235,@object
.L.str.235:
	.asciz	"ebc_buf_state"
	.size	.L.str.235, 14

	.type	.L.str.236,@object
.L.str.236:
	.asciz	"603893"
	.size	.L.str.236, 7

	.type	dev_attr_dump_buffer,@object
	.data
	.p2align	3
dev_attr_dump_buffer:
	.xword	.L.str.237
	.hword	128
	.zero	6
	.xword	0
	.xword	dump_buffer_write
	.size	dev_attr_dump_buffer, 32

	.type	.L.str.237,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.237:
	.asciz	"dump_buffer"
	.size	.L.str.237, 12

	.type	.L.str.238,@object
.L.str.238:
	.asciz	"auto_frame_state"
	.size	.L.str.238, 17

	.type	.L.str.239,@object
.L.str.239:
	.asciz	"0\n"
	.size	.L.str.239, 3

	.type	.L.str.240,@object
.L.str.240:
	.asciz	"ebc_debug_level"
	.size	.L.str.240, 16

	.type	.L.str.241,@object
.L.str.241:
	.asciz	"wf_data"
	.size	.L.str.241, 8

	.type	.L.str.242,@object
.L.str.242:
	.asciz	"lutdata: %d frames\n"
	.size	.L.str.242, 20

	.type	.L.str.243,@object
.L.str.243:
	.asciz	"[%d-->%d]: \n"
	.size	.L.str.243, 13

	.type	.L.str.244,@object
.L.str.244:
	.asciz	"\001c\0017%d, "
	.size	.L.str.244, 9

	.type	.L.str.245,@object
.L.str.245:
	.asciz	"overlay_wf_data"
	.size	.L.str.245, 16

	.type	.L.str.246,@object
.L.str.246:
	.asciz	"ori_wf_data"
	.size	.L.str.246, 12

	.type	.L.str.247,@object
.L.str.247:
	.asciz	"malloc wf_table buffer failed\n"
	.size	.L.str.247, 31

	.type	.L.str.248,@object
.L.str.248:
	.asciz	"603893 %d %d"
	.size	.L.str.248, 13

	.type	.L.str.249,@object
.L.str.249:
	.asciz	"%s: wf mode %d on temp %d\n"
	.size	.L.str.249, 27

	.type	.L__func__.ori_wf_data_write,@object
.L__func__.ori_wf_data_write:
	.asciz	"ori_wf_data_write"
	.size	.L__func__.ori_wf_data_write, 18

	.type	.L.str.250,@object
.L.str.250:
	.asciz	"%s: err code\n"
	.size	.L.str.250, 14

	.type	.L.str.251,@object
.L.str.251:
	.asciz	"%s: err lut_type or temp\n"
	.size	.L.str.251, 26

	.type	.L.str.252,@object
.L.str.252:
	.asciz	"device suspend\n"
	.size	.L.str.252, 16

	.type	.L.str.253,@object
.L.str.253:
	.asciz	"device resume\n"
	.size	.L.str.253, 15

	.type	.Lswitch.table.ebc_probe.255,@object
	.section	.rodata,"a",@progbits
	.p2align	2
.Lswitch.table.ebc_probe.255:
	.word	3566
	.word	3576
	.word	3572
	.word	3506
	.word	3326
	.size	.Lswitch.table.ebc_probe.255, 20

	.ident	"Android (8508608, based on r450784e) clang version 14.0.7 (https://android.googlesource.com/toolchain/llvm-project 4c603efb0cca074e9238af8b4106c30add4418f6)"
	.section	".note.GNU-stack","",@progbits
	.addrsig
	.addrsig_sym ebc_exit
	.addrsig_sym ebc_init
	.addrsig_sym ebc_probe
	.addrsig_sym ebc_remove
	.addrsig_sym frame_done_callback
	.addrsig_sym ebc_assist_thread
	.addrsig_sym ebc_thread
	.addrsig_sym ebc_delay_power_off
	.addrsig_sym delayed_work_timer_fn
	.addrsig_sym ebc_wake_up_for_ghost_rm_last
	.addrsig_sym ebc_io_ctl
	.addrsig_sym ebc_open
	.addrsig_sym ebc_release
	.addrsig_sym waveform_llseek
	.addrsig_sym waveform_read
	.addrsig_sym waveform_write
	.addrsig_sym waveform_mmap
	.addrsig_sym waveform_open
	.addrsig_sym transition_lut_llseek
	.addrsig_sym transition_lut_read
	.addrsig_sym transition_lut_write
	.addrsig_sym transition_lut_mmap
	.addrsig_sym transition_lut_open
	.addrsig_sym transition_lut_vm_fault
	.addrsig_sym waveform_version_read
	.addrsig_sym pmic_name_read
	.addrsig_sym pmic_temp_read
	.addrsig_sym pmic_vcom_read
	.addrsig_sym pmic_vcom_write
	.addrsig_sym ebc_version_read
	.addrsig_sym ebc_state_read
	.addrsig_sym ebc_buf_state_read
	.addrsig_sym ebc_buf_state_write
	.addrsig_sym dump_buffer_write
	.addrsig_sym auto_frame_state_read
	.addrsig_sym ebc_debug_level_read
	.addrsig_sym ebc_debug_level_write
	.addrsig_sym wf_data_write
	.addrsig_sym overlay_wf_data_write
	.addrsig_sym ori_wf_data_write
	.addrsig_sym ebc_suspend
	.addrsig_sym ebc_resume
	.addrsig_sym ebc_driver
	.addrsig_sym __UNIQUE_ID___addressable_ebc_init530
	.addrsig_sym __exitcall_ebc_exit
	.addrsig_sym __UNIQUE_ID_author531
	.addrsig_sym __UNIQUE_ID_description532
	.addrsig_sym __UNIQUE_ID_file533
	.addrsig_sym __UNIQUE_ID_license534
	.addrsig_sym ebc_match
	.addrsig_sym ebc_pm
	.addrsig_sym __cpu_possible_mask
	.addrsig_sym ebc_flip_wq
	.addrsig_sym ebc_assist_finished_flag_sem
	.addrsig_sym cpu_bit_bitmap
	.addrsig_sym ebc_misc
	.addrsig_sym waveform_misc
	.addrsig_sym transition_lut_misc
	.addrsig_sym dev_attr_waveform_version
	.addrsig_sym dev_attr_pmic_name
	.addrsig_sym dev_attr_pmic_temp
	.addrsig_sym dev_attr_pmic_vcom
	.addrsig_sym dev_attr_ebc_version
	.addrsig_sym dev_attr_ebc_state
	.addrsig_sym dev_attr_ebc_buf_state
	.addrsig_sym dev_attr_auto_frame_state
	.addrsig_sym dev_attr_ebc_debug_level
	.addrsig_sym dev_attr_wf_data
	.addrsig_sym dev_attr_overlay_wf_data
	.addrsig_sym dev_attr_ori_wf_data
	.addrsig_sym ebc_ops
	.addrsig_sym cpu_hwcaps
	.addrsig_sym gic_nonsecure_priorities
	.addrsig_sym gic_pmr_sync
	.addrsig_sym waveform_ops
	.addrsig_sym transition_lut_ops
	.addrsig_sym transition_lut_vm_ops
	.addrsig_sym dev_attr_dump_buffer
