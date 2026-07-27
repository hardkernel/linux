// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020-2024 Rockchip Electronics Co., Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

	.text
	.file	"pvi_waveform.c"
	.globl	pvi_wf_add_custom_mode_table
	.p2align	2
	.type	pvi_wf_add_custom_mode_table,@function
pvi_wf_add_custom_mode_table:
	cmp	w1, #11
	b.ne	.LBB0_4
	adrp	x9, custom_mode_table
	ldrb	w8, [x0, #2]
	add	x9, x9, :lo12:custom_mode_table
	strb	w8, [x9]
	ldrb	w8, [x0, #3]
	strb	w8, [x9, #1]
	ldrb	w8, [x0, #4]
	strb	w8, [x9, #2]
	ldrb	w8, [x0, #5]
	strb	w8, [x9, #3]
	ldrb	w8, [x0, #6]
	strb	w8, [x9, #4]
	ldrb	w8, [x0, #7]
	strb	w8, [x9, #5]
	ldrb	w8, [x0, #8]
	strb	w8, [x9, #6]
	ldrb	w8, [x0, #9]
	strb	w8, [x9, #7]
	ldrb	w8, [x0, #10]
	strb	w8, [x9, #8]
	ldrb	w8, [x0]
	strb	w8, [x9, #9]
	ldrb	w8, [x0, #1]
	cmp	w8, #4
	b.eq	.LBB0_5
	cmp	w8, #5
	b.ne	.LBB0_7
	mov	w8, #32
	b	.LBB0_6
.LBB0_4:
	mov	w0, #-1
	ret
.LBB0_5:
	mov	w8, #16
.LBB0_6:
	adrp	x9, custom_mode_table+12
	mov	w0, wzr
	str	w8, [x9, :lo12:custom_mode_table+12]
	ret
.LBB0_7:
	mov	w0, wzr
	ret
.Lfunc_end0:
	.size	pvi_wf_add_custom_mode_table, .Lfunc_end0-pvi_wf_add_custom_mode_table

	.globl	pvi_wf_get_lut
	.p2align	2
	.type	pvi_wf_get_lut,@function
pvi_wf_get_lut:
	sub	sp, sp, #112
	str	x30, [x18], #8
	stp	x29, x30, [sp, #16]
	add	x29, sp, #16
	stp	x28, x27, [sp, #32]
	stp	x26, x25, [sp, #48]
	stp	x24, x23, [sp, #64]
	adrp	x24, global_waveform
	stp	x22, x21, [sp, #80]
	stp	x20, x19, [sp, #96]
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	str	x8, [sp, #8]
	ldr	x8, [x24, :lo12:global_waveform]
	str	wzr, [sp, #4]
	cbz	x8, .LBB1_11
	mov	x20, x3
	mov	w21, w1
	mov	x19, x0
	cbnz	x0, .LBB1_3
	mov	w8, #24
	ldr	x8, [x8]
	cbz	x8, .LBB1_23
.LBB1_3:
	ldr	w8, [x20]
	adrp	x9, need_pic
	cmp	w21, #10
	str	w8, [x9, :lo12:need_pic]
	b.ne	.LBB1_12
	ldr	x8, [x19, #32]
	cbz	x8, .LBB1_21
	adrp	x22, pvi_wf_get_lut.sftemp
	mov	w11, #21846
	ubfx	w10, w2, #8, #8
	movk	w11, #21845, lsl #16
	add	w10, w10, #2
	mov	w12, #43691
	ldr	w9, [x22, :lo12:pvi_wf_get_lut.sftemp]
	mul	w10, w10, w12
	asr	w9, w9, #8
	add	w9, w9, #2
	smull	x9, w9, w11
	lsr	x11, x9, #63
	lsr	x9, x9, #32
	add	w9, w9, w11
	cmp	w9, w10, lsr #17
	b.eq	.LBB1_21
	adrp	x25, waveformdata
	and	w23, w2, #0xffff
	str	x8, [x25, :lo12:waveformdata]
	bl	epd_overlay_lut
	ldr	x10, [x24, :lo12:global_waveform]
	mov	w21, w0
	ldrb	w8, [x10, #38]
	cbz	w8, .LBB1_47
	mov	w9, wzr
.LBB1_8:
	add	x11, x10, w9, sxtw
	ldrb	w11, [x11, #48]
	cmp	w11, w23, lsr #8
	b.hs	.LBB1_72
	add	w9, w9, #1
	cmp	w8, w9
	b.ne	.LBB1_8
	mov	w10, #-1
	mov	w9, w8
	b	.LBB1_73
.LBB1_11:
	mov	w0, #-19
	b	.LBB1_112
.LBB1_12:
	adrp	x8, pvi_wf_get_lut.sftemp
	and	w25, w2, #0xff
	add	w9, w25, #2
	mov	w10, #43691
	adrp	x26, pvi_wf_get_lut.stype
	adrp	x27, pvi_wf_get_lut.normal_repair
	ldrb	w8, [x8, :lo12:pvi_wf_get_lut.sftemp]
	mul	w9, w9, w10
	lsr	w9, w9, #17
	add	w8, w8, #2
	mul	w8, w8, w10
	cmp	w9, w8, lsr #17
	b.ne	.LBB1_16
	ldr	w8, [x26, :lo12:pvi_wf_get_lut.stype]
	cmp	w8, w21
	b.ne	.LBB1_16
	ldr	w8, [x27, :lo12:pvi_wf_get_lut.normal_repair]
	ldr	w9, [x20, #8]
	cmp	w8, w9
	b.ne	.LBB1_16
	ldr	w8, [x20, #12]
	cbz	w8, .LBB1_21
.LBB1_16:
	ldr	x10, [x24, :lo12:global_waveform]
	adrp	x23, waveformdata
	ldr	x9, [x19, #24]
	ldrb	w8, [x10, #38]
	str	x9, [x23, :lo12:waveformdata]
	cbz	w8, .LBB1_22
	mov	w9, wzr
.LBB1_18:
	add	x11, x10, w9, sxtw
	ldrb	w11, [x11, #48]
	cmp	w11, w2, uxtb
	b.hs	.LBB1_24
	add	w9, w9, #1
	cmp	w8, w9
	b.ne	.LBB1_18
	mov	w10, #-1
	mov	w9, w8
	b	.LBB1_25
.LBB1_21:
	mov	w0, wzr
	b	.LBB1_112
.LBB1_22:
	mov	w22, wzr
	b	.LBB1_26
.LBB1_23:
	mov	w0, #-22
	b	.LBB1_112
.LBB1_24:
	mov	w10, w9
.LBB1_25:
	sub	w11, w9, #1
	cmp	w9, w8
	csel	w22, w11, w10, eq
.LBB1_26:
	mov	w0, w21
	bl	get_wf_mode_index
	and	w0, w0, #0xff
	mov	w1, w22
	bl	get_wf_frm_num
	tbnz	w0, #31, .LBB1_86
	mov	w22, w0
	ldr	w8, [x20, #4]
	cbz	w8, .LBB1_111
	cmp	w21, #7
	b.eq	.LBB1_30
	cmp	w21, #1
	b.ne	.LBB1_33
.LBB1_30:
	ldr	w24, [x20, #24]
	mov	w1, wzr
	ldr	x23, [x23, :lo12:waveformdata]
	cmp	w24, #0
	cset	w8, gt
	lsl	w28, w22, w8
	lsl	w8, w22, #10
	sub	w9, w28, w22
	lsl	w9, w9, #10
	add	x0, x23, w8, sxtw
	sxtw	x2, w9
	str	w28, [sp, #4]
	bl	memset
	cbz	w22, .LBB1_68
	mov	w8, w22
	cmp	w22, #1
	b.ne	.LBB1_48
	mov	x9, xzr
	b	.LBB1_66
.LBB1_33:
	sub	w8, w21, #13
	cmp	w8, #1
	b.hi	.LBB1_84
	ldr	x23, [x23, :lo12:waveformdata]
	mov	w1, w22
	ldr	w24, [x20, #8]
	mov	x0, x23
	bl	epd_normal_repair
	cbnz	w0, .LBB1_111
	cbz	w22, .LBB1_111
	sub	w10, w22, #3
	sub	w11, w22, #4
	sub	w12, w22, #5
	mov	w9, w22
	mov	x8, xzr
	lsl	x9, x9, #10
	lsl	x10, x10, #10
	lsl	x11, x11, #10
	lsl	x12, x12, #10
	b	.LBB1_41
.LBB1_37:
	mov	w14, #1
	mov	w15, #2
.LBB1_38:
	strb	w14, [x13, #991]
.LBB1_39:
	strb	w14, [x13, #959]
	strb	w14, [x13, #1023]
	strb	w15, [x13]
.LBB1_40:
	ldrb	w14, [x13, #30]
	add	x8, x8, #1024
	cmp	x9, x8
	strb	wzr, [x13, #958]
	strb	wzr, [x13, #1022]
	strb	w14, [x13, #31]
	strb	w14, [x13, #63]
	strb	w14, [x13, #127]
	strb	w14, [x13, #29]
	strb	w14, [x13, #61]
	strb	w14, [x13, #125]
	ldrb	w14, [x13, #960]
	strb	wzr, [x13, #96]
	strb	wzr, [x13, #32]
	strb	w14, [x13, #961]
	strb	w14, [x13, #993]
	strb	w14, [x13, #929]
	strb	w14, [x13, #963]
	strb	w14, [x13, #995]
	strb	w14, [x13, #931]
	b.eq	.LBB1_111
.LBB1_41:
	add	x13, x23, w8, sxtw
	cbz	w24, .LBB1_40
	ldrb	w14, [x13, #990]
	cmp	x12, x8
	ldrb	w15, [x13]
	strb	wzr, [x13, #990]
	strb	w14, [x13, #989]
	strb	w14, [x13, #1021]
	strb	w14, [x13, #957]
	strb	w15, [x13, #3]
	strb	w15, [x13, #35]
	strb	w15, [x13, #99]
	b.eq	.LBB1_37
	cmp	x11, x8
	b.eq	.LBB1_45
	cmp	x10, x8
	b.ne	.LBB1_46
.LBB1_45:
	mov	w14, #2
	mov	w15, #1
	b	.LBB1_38
.LBB1_46:
	mov	w14, wzr
	mov	w15, wzr
	strb	wzr, [x13, #991]
	b	.LBB1_39
.LBB1_47:
	mov	w20, wzr
	b	.LBB1_74
.LBB1_48:
	sub	x11, x8, #1
	mov	x9, xzr
	and	x10, x11, #0xffffffff
	lsl	x13, x10, #10
	tst	x13, #0xffffffff00000000
	cset	w12, ne
	cmp	xzr, x11, lsr #54
	cset	w10, ne
	tbnz	w13, #31, .LBB1_66
	tbnz	w12, #0, .LBB1_66
	lsr	x12, x11, #32
	cbnz	x12, .LBB1_66
	lsl	x11, x11, #10
	add	x12, x23, x11
	cmp	x12, x23
	b.lo	.LBB1_66
	tbnz	w10, #0, .LBB1_66
	add	x12, x23, #990
	add	x13, x12, x11
	cmp	x13, x12
	b.lo	.LBB1_66
	tbnz	w10, #0, .LBB1_66
	add	x12, x23, #958
	add	x13, x12, x11
	cmp	x13, x12
	b.lo	.LBB1_66
	tbnz	w10, #0, .LBB1_66
	add	x12, x23, #928
	add	x13, x12, x11
	cmp	x13, x12
	b.lo	.LBB1_66
	tbnz	w10, #0, .LBB1_66
	add	x12, x23, #1022
	add	x13, x12, x11
	cmp	x13, x12
	b.lo	.LBB1_66
	tbnz	w10, #0, .LBB1_66
	add	x12, x23, #992
	add	x11, x12, x11
	cmp	x11, x12
	b.lo	.LBB1_66
	tbnz	w10, #0, .LBB1_66
	and	x9, x8, #0xfffffffe
	mov	w10, #1024
	mov	x11, x9
.LBB1_64:
	sub	w13, w10, #1024
	add	x12, x23, w10, sxtw
	subs	x11, x11, #2
	add	w10, w10, #2048
	add	x13, x23, w13, sxtw
	strb	wzr, [x12, #992]
	strb	wzr, [x12, #1022]
	strb	wzr, [x12, #928]
	strb	wzr, [x13, #992]
	strb	wzr, [x13, #1022]
	strb	wzr, [x13, #928]
	strb	wzr, [x13, #958]
	strb	wzr, [x12, #958]
	strb	wzr, [x13]
	strb	wzr, [x12]
	strb	wzr, [x13, #990]
	strb	wzr, [x12, #990]
	b.ne	.LBB1_64
	cmp	x9, x8
	b.eq	.LBB1_68
.LBB1_66:
	sub	x8, x8, x9
	lsl	w9, w9, #10
.LBB1_67:
	add	x10, x23, w9, sxtw
	subs	x8, x8, #1
	add	w9, w9, #1024
	strb	wzr, [x10, #992]
	strb	wzr, [x10, #1022]
	strb	wzr, [x10, #928]
	strb	wzr, [x10, #958]
	strb	wzr, [x10]
	strb	wzr, [x10, #990]
	b.ne	.LBB1_67
.LBB1_68:
	add	x2, sp, #4
	mov	x0, x23
	mov	w1, w22
	mov	w3, w24
	bl	epd_gray2_last_repair
	cbnz	w0, .LBB1_111
	sub	w8, w28, #1
	cmp	w8, w24
	csel	w8, w8, w24, lt
	cmp	w8, #1
	b.lt	.LBB1_111
	mov	w9, wzr
	mov	w10, #1
	mov	w11, #2
	mov	w12, w28
	mov	w13, w8
.LBB1_71:
	lsl	w14, w9, #10
	sdiv	w9, w12, w8
	subs	w13, w13, #1
	add	w12, w12, w28
	add	x14, x23, w14, sxtw
	strb	w10, [x14, #992]
	strb	w10, [x14, #928]
	strb	w11, [x14, #1022]
	strb	w11, [x14, #958]
	b.ne	.LBB1_71
	b	.LBB1_111
.LBB1_72:
	mov	w10, w9
.LBB1_73:
	sub	w11, w9, #1
	cmp	w9, w8
	csel	w20, w11, w10, eq
.LBB1_74:
	mov	w0, w21
	bl	get_wf_mode_index
	and	w0, w0, #0xff
	mov	w1, w20
	bl	get_wf_frm_num
	tbnz	w0, #31, .LBB1_86
	mov	w20, w0
	cmp	w21, #1
	b.ne	.LBB1_83
	ldr	x21, [x25, :lo12:waveformdata]
	cbz	w20, .LBB1_81
	mov	w8, wzr
	mov	x9, xzr
	mov	w10, w20
	add	x11, x21, #31
	add	x12, x21, #992
.LBB1_78:
	sxtw	x14, w8
	mov	x15, #-16
	add	x13, x11, x14
	add	x14, x12, x14
.LBB1_79:
	ldurb	w16, [x13, #-1]
	add	x15, x15, #16
	cmp	x15, #224
	strb	w16, [x13]
	sturb	w16, [x13, #-2]
	add	x13, x13, #64
	ldurb	w16, [x14, #-32]
	sturb	w16, [x14, #-64]
	strb	w16, [x14], #2
	b.lo	.LBB1_79
	add	x9, x9, #1
	add	w8, w8, #1024
	cmp	x9, x10
	b.ne	.LBB1_78
.LBB1_81:
	mov	x0, x21
	mov	w1, w20
	bl	epd_overlay_gray2_repair
	cbnz	w0, .LBB1_83
	lsl	w8, w20, #10
	mov	w9, #2
	sub	w8, w8, #2048
	add	x8, x21, w8, sxtw
	strb	w9, [x8, #991]
	strb	w9, [x8, #959]
	strb	w9, [x8, #1023]
.LBB1_83:
	ldrb	w8, [x22, :lo12:pvi_wf_get_lut.sftemp]
	and	w9, w23, #0xff00
	mov	w0, #2
	str	w20, [x19, #4]
	orr	w8, w8, w9
	str	w8, [x22, :lo12:pvi_wf_get_lut.sftemp]
	b	.LBB1_112
.LBB1_84:
	sub	w8, w21, #5
	cmp	w8, #1
	b.hi	.LBB1_87
	ldr	x0, [x19, #24]
	mov	w1, w22
	ldr	w2, [x20, #12]
	bl	pvi_wf_regal_fix
	b	.LBB1_111
.LBB1_86:
	adrp	x0, .L.str
	add	x0, x0, :lo12:.L.str
	bl	_printk
	mov	w0, #-5
	b	.LBB1_112
.LBB1_87:
	cmp	w21, #15
	b.ne	.LBB1_111
	cbz	w22, .LBB1_111
	ldr	x8, [x23, :lo12:waveformdata]
	mov	w9, w22
	cmp	w22, #1
	b.ne	.LBB1_91
	mov	x10, xzr
	b	.LBB1_109
.LBB1_91:
	sub	x12, x9, #1
	mov	x10, xzr
	and	x11, x12, #0xffffffff
	lsl	x14, x11, #10
	tst	x14, #0xffffffff00000000
	cset	w13, ne
	cmp	xzr, x12, lsr #54
	cset	w11, ne
	tbnz	w14, #31, .LBB1_109
	tbnz	w13, #0, .LBB1_109
	lsr	x13, x12, #32
	cbnz	x13, .LBB1_109
	add	x13, x8, #957
	lsl	x12, x12, #10
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB1_109
	tbnz	w11, #0, .LBB1_109
	add	x13, x8, #1023
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB1_109
	tbnz	w11, #0, .LBB1_109
	add	x13, x8, #991
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB1_109
	tbnz	w11, #0, .LBB1_109
	add	x13, x8, #1022
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB1_109
	tbnz	w11, #0, .LBB1_109
	add	x13, x8, #990
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB1_109
	tbnz	w11, #0, .LBB1_109
	add	x13, x8, #958
	add	x12, x13, x12
	cmp	x12, x13
	b.lo	.LBB1_109
	tbnz	w11, #0, .LBB1_109
	and	x10, x9, #0xfffffffe
	mov	w11, #1024
	mov	x12, x10
.LBB1_107:
	sub	w14, w11, #1024
	add	x13, x8, w11, sxtw
	subs	x12, x12, #2
	add	w11, w11, #2048
	add	x14, x8, w14, sxtw
	ldrb	w15, [x13, #957]
	ldrb	w16, [x14, #957]
	strb	w15, [x13, #958]
	strb	w15, [x13, #990]
	strb	w15, [x13, #1022]
	strb	w15, [x13, #991]
	strb	w16, [x14, #958]
	strb	w16, [x14, #990]
	strb	w16, [x14, #1022]
	strb	w16, [x14, #991]
	strb	w16, [x14, #1023]
	strb	w15, [x13, #1023]
	b.ne	.LBB1_107
	cmp	x10, x9
	b.eq	.LBB1_111
.LBB1_109:
	sub	x9, x9, x10
	lsl	w10, w10, #10
.LBB1_110:
	add	x11, x8, w10, sxtw
	subs	x9, x9, #1
	add	w10, w10, #1024
	ldrb	w12, [x11, #957]
	strb	w12, [x11, #958]
	strb	w12, [x11, #990]
	strb	w12, [x11, #1022]
	strb	w12, [x11, #991]
	strb	w12, [x11, #1023]
	b.ne	.LBB1_110
.LBB1_111:
	adrp	x9, pvi_wf_get_lut.sftemp
	str	w22, [x19]
	add	x9, x9, :lo12:pvi_wf_get_lut.sftemp
	ldr	w8, [sp, #4]
	mov	w0, #1
	str	w21, [x26, :lo12:pvi_wf_get_lut.stype]
	ldrb	w10, [x9, #1]
	str	w8, [x19, #8]
	ldr	w8, [x20, #8]
	bfi	w25, w10, #8, #8
	str	w8, [x27, :lo12:pvi_wf_get_lut.normal_repair]
	str	w25, [x9]
.LBB1_112:
	mrs	x8, SP_EL0
	ldr	x8, [x8, #1584]
	ldr	x9, [sp, #8]
	cmp	x8, x9
	b.ne	.LBB1_114
	ldp	x20, x19, [sp, #96]
	ldp	x22, x21, [sp, #80]
	ldp	x24, x23, [sp, #64]
	ldp	x26, x25, [sp, #48]
	ldp	x28, x27, [sp, #32]
	ldp	x29, x30, [sp, #16]
	ldr	x30, [x18, #-8]!
	add	sp, sp, #112
	ret
.LBB1_114:
	bl	__stack_chk_fail
.Lfunc_end1:
	.size	pvi_wf_get_lut, .Lfunc_end1-pvi_wf_get_lut

	.p2align	2
	.type	get_wf_mode_index,@function
get_wf_mode_index:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	cmp	w0, #16
	mov	x29, sp
	b.hs	.LBB2_17
	sub	w8, w0, #11
	cmp	w8, #5
	b.hs	.LBB2_3
	adrp	x9, .Lswitch.table.get_wf_mode_index
	add	x9, x9, :lo12:.Lswitch.table.get_wf_mode_index
	ldr	x8, [x9, w8, sxtw #3]
	b	.LBB2_4
.LBB2_3:
	mov	w8, w0
.LBB2_4:
	adrp	x9, global_waveform
	ldr	x9, [x9, :lo12:global_waveform]
	ldrb	w10, [x9, #16]
	sub	w9, w10, #9
	cmp	w9, #105
	b.hi	.LBB2_8
	adrp	x11, .LJTI2_0
	add	x11, x11, :lo12:.LJTI2_0
	adr	x12, .LBB2_6
	ldrb	w13, [x11, x9]
	add	x12, x12, x13, lsl #2
	adrp	x9, pvi_mode_table
	add	x9, x9, :lo12:pvi_mode_table
	br	x12
.LBB2_6:
	adrp	x9, pvi_mode_table+72
	add	x9, x9, :lo12:pvi_mode_table+72
	b	.LBB2_15
.LBB2_7:
	adrp	x9, pvi_mode_table+27
	add	x9, x9, :lo12:pvi_mode_table+27
	b	.LBB2_15
.LBB2_8:
	adrp	x9, custom_mode_table
	adrp	x12, pvi_mode_table+72
	add	x9, x9, :lo12:custom_mode_table
	add	x12, x12, :lo12:pvi_mode_table+72
	ldrb	w11, [x9, #9]
	cmp	w10, w11
	csel	x9, x9, x12, eq
	b	.LBB2_15
.LBB2_9:
	adrp	x9, pvi_mode_table+9
	add	x9, x9, :lo12:pvi_mode_table+9
	b	.LBB2_15
.LBB2_10:
	adrp	x9, pvi_mode_table+18
	add	x9, x9, :lo12:pvi_mode_table+18
	b	.LBB2_15
.LBB2_11:
	adrp	x9, pvi_mode_table+36
	add	x9, x9, :lo12:pvi_mode_table+36
	b	.LBB2_15
.LBB2_12:
	adrp	x9, pvi_mode_table+45
	add	x9, x9, :lo12:pvi_mode_table+45
	b	.LBB2_15
.LBB2_13:
	adrp	x9, pvi_mode_table+54
	add	x9, x9, :lo12:pvi_mode_table+54
	b	.LBB2_15
.LBB2_14:
	adrp	x9, pvi_mode_table+63
	add	x9, x9, :lo12:pvi_mode_table+63
.LBB2_15:
	ldrb	w0, [x9, x8]
.LBB2_16:
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.LBB2_17:
	adrp	x0, .L.str.3
	add	x0, x0, :lo12:.L.str.3
	bl	_printk
	mov	w0, #255
	b	.LBB2_16
.Lfunc_end2:
	.size	get_wf_mode_index, .Lfunc_end2-get_wf_mode_index
	.section	.rodata,"a",@progbits
.LJTI2_0:
	.byte	(.LBB2_15-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_9-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_10-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_6-.LBB2_6)>>2
	.byte	(.LBB2_7-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_6-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_11-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_7-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_12-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_13-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_8-.LBB2_6)>>2
	.byte	(.LBB2_14-.LBB2_6)>>2

	.text
	.p2align	2
	.type	get_wf_frm_num,@function
get_wf_frm_num:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-64]!
	adrp	x8, global_waveform
	lsl	w9, w0, #2
	str	x23, [sp, #16]
	mov	x29, sp
	stp	x22, x21, [sp, #32]
	ldr	x14, [x8, :lo12:global_waveform]
	stp	x20, x19, [sp, #48]
	ldrb	w8, [x14, #32]
	add	x8, x14, x8
	add	x8, x8, x9
	ldrb	w9, [x8]
	ldrb	w10, [x8, #1]
	ldrb	w11, [x8, #2]
	ldrb	w12, [x8, #3]
	add	w9, w10, w9
	add	w9, w9, w11
	cmp	w12, w9, uxtb
	b.ne	.LBB3_26
	ldr	w8, [x8]
	lsl	w9, w1, #2
	and	x8, x8, #0xffffff
	add	x8, x14, x8
	add	x9, x8, w9, sxtw
	ldrb	w8, [x9]
	ldrb	w10, [x9, #1]
	ldrb	w11, [x9, #2]
	ldrb	w12, [x9, #3]
	add	w8, w10, w8
	add	w8, w8, w11
	cmp	w12, w8, uxtb
	b.ne	.LBB3_27
	adrp	x20, waveformdata
	ldr	x8, [x20, :lo12:waveformdata]
	cbz	x8, .LBB3_29
	adrp	x21, maxpic
	adrp	x22, need_pic
	ldr	w15, [x9]
	mov	w13, wzr
	mov	w11, wzr
	mov	w19, wzr
	ldr	w10, [x21, :lo12:maxpic]
	mov	w12, wzr
	ldr	w9, [x22, :lo12:need_pic]
	and	x15, x15, #0xffffff
	add	x14, x14, x15
	mov	w17, #1
	cmp	w10, #16
	cset	w16, eq
	cmp	w9, #32
	cset	w15, eq
	ands	w15, w16, w15
	mov	w16, #5
	cinc	w16, w16, ne
	b	.LBB3_6
.LBB3_4:
	mov	w17, #1
.LBB3_5:
	add	w11, w11, #1
	cmp	w19, #254
	b.hi	.LBB3_18
.LBB3_6:
	ldrb	w2, [x14, w11, uxtw]
	cmp	w2, #252
	b.eq	.LBB3_11
	cmp	w2, #255
	b.eq	.LBB3_19
	tbnz	w17, #0, .LBB3_12
.LBB3_9:
	add	w17, w13, #1
	lsl	w0, w12, w15
	lsl	w1, w13, w16
	add	w0, w0, w19, lsl #10
	lsl	w17, w17, w16
	add	w1, w0, w1
	add	w17, w0, w17
	and	w3, w2, #0x3
	ubfx	w4, w2, #2, #2
	strb	w3, [x8, w1, uxtw]
	add	w1, w13, #2
	strb	w4, [x8, w17, uxtw]
	add	w17, w13, #3
	lsl	w1, w1, w16
	lsl	w17, w17, w16
	add	w1, w0, w1
	add	w17, w0, w17
	ubfx	w3, w2, #4, #2
	lsr	w0, w2, #6
	add	w13, w13, #4
	strb	w3, [x8, w1, uxtw]
	cmp	w13, w10
	strb	w0, [x8, w17, uxtw]
	mov	w17, wzr
	b.lo	.LBB3_5
	add	w0, w12, #1
	mov	w13, wzr
	cmp	w0, w10
	csinc	w12, wzr, w12, hs
	cinc	w19, w19, hs
	b	.LBB3_5
.LBB3_11:
	add	w11, w11, #1
	ldrb	w2, [x14, w11, uxtw]
	tbnz	w17, #0, .LBB3_9
.LBB3_12:
	add	w17, w13, #1
	lsl	w0, w12, w15
	lsl	w1, w13, w16
	add	w3, w0, w19, lsl #10
	lsl	w0, w17, w16
	add	w1, w3, w1
	add	w4, w3, w0
	and	w17, w2, #0x3
	ubfx	w0, w2, #2, #2
	strb	w17, [x8, w1, uxtw]
	add	w1, w13, #2
	strb	w0, [x8, w4, uxtw]
	add	w4, w13, #3
	lsl	w1, w1, w16
	lsl	w4, w4, w16
	add	w5, w3, w1
	ubfx	w1, w2, #4, #2
	add	w3, w3, w4
	lsr	w2, w2, #6
	add	w13, w13, #4
	cmp	w13, w10
	strb	w1, [x8, w5, uxtw]
	strb	w2, [x8, w3, uxtw]
	b.lo	.LBB3_14
	add	w3, w12, #1
	mov	w13, wzr
	cmp	w3, w10
	csinc	w12, wzr, w12, hs
	cinc	w19, w19, hs
.LBB3_14:
	add	w11, w11, #1
	ldrb	w3, [x14, w11, uxtw]
	cbnz	w3, .LBB3_16
	b	.LBB3_4
.LBB3_15:
	sub	w3, w3, #1
	tst	w3, #0xff
	b.eq	.LBB3_4
.LBB3_16:
	add	w4, w13, #1
	lsl	w5, w12, w15
	add	w7, w13, #2
	add	w23, w13, #3
	lsl	w6, w13, w16
	add	w5, w5, w19, lsl #10
	lsl	w4, w4, w16
	lsl	w7, w7, w16
	lsl	w23, w23, w16
	add	w6, w5, w6
	add	w4, w5, w4
	add	w7, w5, w7
	add	w5, w5, w23
	add	w13, w13, #4
	cmp	w13, w10
	strb	w17, [x8, w6, uxtw]
	strb	w0, [x8, w4, uxtw]
	strb	w1, [x8, w7, uxtw]
	strb	w2, [x8, w5, uxtw]
	b.lo	.LBB3_15
	add	w4, w12, #1
	mov	w13, wzr
	cmp	w4, w10
	csinc	w12, wzr, w12, hs
	cinc	w19, w19, hs
	b	.LBB3_15
.LBB3_18:
	adrp	x0, .L.str.6
	add	x0, x0, :lo12:.L.str.6
	bl	_printk
	ldr	w10, [x21, :lo12:maxpic]
	ldr	w9, [x22, :lo12:need_pic]
.LBB3_19:
	cmp	w10, #32
	b.ne	.LBB3_31
	cmp	w9, #16
	b.ne	.LBB3_31
	cbz	w19, .LBB3_31
	ldr	x11, [x20, :lo12:waveformdata]
	mov	w8, wzr
	mov	w9, wzr
	add	x10, x11, #16
	add	x11, x11, #7
.LBB3_23:
	mov	w13, w8
	mov	x14, #-2
	add	x12, x10, x13
	and	x13, x13, #0xffffffe0
	add	x13, x11, x13
.LBB3_24:
	ldurb	w15, [x12, #-16]
	add	x14, x14, #2
	cmp	x14, #30
	sturb	w15, [x13, #-7]
	ldurb	w15, [x12, #-14]
	sturb	w15, [x13, #-6]
	ldurb	w15, [x12, #-12]
	sturb	w15, [x13, #-5]
	ldurb	w15, [x12, #-10]
	sturb	w15, [x13, #-4]
	ldurb	w15, [x12, #-8]
	sturb	w15, [x13, #-3]
	ldurb	w15, [x12, #-6]
	sturb	w15, [x13, #-2]
	ldurb	w15, [x12, #-4]
	sturb	w15, [x13, #-1]
	ldurb	w15, [x12, #-2]
	strb	w15, [x13]
	ldrb	w15, [x12]
	strb	w15, [x13, #1]
	ldrb	w15, [x12, #2]
	strb	w15, [x13, #2]
	ldrb	w15, [x12, #4]
	strb	w15, [x13, #3]
	ldrb	w15, [x12, #6]
	strb	w15, [x13, #4]
	ldrb	w15, [x12, #8]
	strb	w15, [x13, #5]
	ldrb	w15, [x12, #10]
	strb	w15, [x13, #6]
	ldrb	w15, [x12, #12]
	strb	w15, [x13, #7]
	ldrb	w15, [x12, #14]
	add	x12, x12, #64
	strb	w15, [x13, #8]
	add	x13, x13, #32
	b.lo	.LBB3_24
	add	w9, w9, #1
	add	w8, w8, #1024
	cmp	w19, w9, uxtb
	b.hi	.LBB3_23
	b	.LBB3_31
.LBB3_26:
	adrp	x0, .L.str.4
	adrp	x1, .L__func__.get_wf_frm_num
	add	x0, x0, :lo12:.L.str.4
	add	x1, x1, :lo12:.L__func__.get_wf_frm_num
	mov	w2, #268
	b	.LBB3_28
.LBB3_27:
	adrp	x0, .L.str.4
	adrp	x1, .L__func__.get_wf_frm_num
	add	x0, x0, :lo12:.L.str.4
	add	x1, x1, :lo12:.L__func__.get_wf_frm_num
	mov	w2, #275
.LBB3_28:
	bl	_printk
	b	.LBB3_30
.LBB3_29:
	adrp	x0, .L.str.5
	add	x0, x0, :lo12:.L.str.5
	bl	_printk
.LBB3_30:
	mov	w19, #-22
.LBB3_31:
	mov	w0, w19
	ldr	x23, [sp, #16]
	ldp	x20, x19, [sp, #48]
	ldp	x22, x21, [sp, #32]
	ldp	x29, x30, [sp], #64
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end3:
	.size	get_wf_frm_num, .Lfunc_end3-get_wf_frm_num

	.p2align	2
	.type	pvi_wf_regal_fix,@function
pvi_wf_regal_fix:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-16]!
	mov	x29, sp
	tbz	w2, #1, .LBB4_4
	cmp	w1, #1
	b.lt	.LBB4_4
	mov	w9, w1
	mov	x8, xzr
	lsl	x9, x9, #10
.LBB4_3:
	add	x10, x0, w8, sxtw
	add	x8, x8, #1024
	ldrb	w11, [x10, #991]
	strb	wzr, [x10]
	ldrb	w12, [x10, #989]
	eor	w13, w11, #0x3
	cmp	w11, #0
	eor	w11, w12, #0x3
	csel	w13, wzr, w13, eq
	cmp	w12, #0
	csel	w11, wzr, w11, eq
	cmp	x9, x8
	strb	w13, [x10, #1]
	strb	w11, [x10, #3]
	b.ne	.LBB4_3
.LBB4_4:
	cmp	w1, #1
	b.lt	.LBB4_27
	mov	w8, w1
	mov	x10, xzr
	lsl	x9, x8, #10
.LBB4_6:
	add	x11, x0, w10, sxtw
	add	x10, x10, #1024
	cmp	x9, x10
	ldrb	w12, [x11, #30]
	strb	wzr, [x11, #96]
	ldrb	w13, [x11, #94]
	strb	wzr, [x11, #32]
	strb	w12, [x11, #31]
	strb	w12, [x11, #29]
	ldrb	w12, [x11, #158]
	strb	w13, [x11, #95]
	strb	w13, [x11, #93]
	ldrb	w13, [x11, #222]
	strb	w12, [x11, #159]
	strb	w12, [x11, #157]
	ldrb	w12, [x11, #286]
	strb	w13, [x11, #223]
	strb	w13, [x11, #221]
	ldrb	w13, [x11, #350]
	strb	w12, [x11, #287]
	strb	w12, [x11, #285]
	ldrb	w12, [x11, #414]
	strb	w13, [x11, #351]
	strb	w13, [x11, #349]
	ldrb	w13, [x11, #478]
	strb	w12, [x11, #415]
	strb	w12, [x11, #413]
	ldrb	w12, [x11, #542]
	strb	w13, [x11, #479]
	strb	w13, [x11, #477]
	ldrb	w13, [x11, #606]
	strb	w12, [x11, #543]
	strb	w12, [x11, #541]
	ldrb	w12, [x11, #670]
	strb	w13, [x11, #607]
	strb	w13, [x11, #605]
	ldrb	w13, [x11, #734]
	strb	w12, [x11, #671]
	strb	w12, [x11, #669]
	ldrb	w12, [x11, #798]
	strb	w13, [x11, #735]
	strb	w13, [x11, #733]
	ldrb	w13, [x11, #862]
	strb	w12, [x11, #799]
	strb	w12, [x11, #797]
	ldrb	w12, [x11, #926]
	strb	w13, [x11, #863]
	strb	w13, [x11, #861]
	ldrb	w13, [x11, #960]
	strb	w12, [x11, #927]
	strb	w12, [x11, #925]
	ldrb	w12, [x11, #896]
	strb	w13, [x11, #963]
	strb	w13, [x11, #961]
	ldrb	w13, [x11, #832]
	strb	w12, [x11, #899]
	strb	w12, [x11, #897]
	ldrb	w12, [x11, #768]
	strb	w13, [x11, #835]
	strb	w13, [x11, #833]
	ldrb	w13, [x11, #704]
	strb	w12, [x11, #771]
	strb	w12, [x11, #769]
	ldrb	w12, [x11, #640]
	strb	w13, [x11, #707]
	strb	w13, [x11, #705]
	ldrb	w13, [x11, #576]
	strb	w12, [x11, #643]
	strb	w12, [x11, #641]
	ldrb	w12, [x11, #512]
	strb	w13, [x11, #579]
	strb	w13, [x11, #577]
	ldrb	w13, [x11, #448]
	strb	w12, [x11, #515]
	strb	w12, [x11, #513]
	ldrb	w12, [x11, #384]
	strb	w13, [x11, #451]
	strb	w13, [x11, #449]
	ldrb	w13, [x11, #320]
	strb	w12, [x11, #387]
	strb	w12, [x11, #385]
	ldrb	w12, [x11, #256]
	strb	w13, [x11, #323]
	strb	w13, [x11, #321]
	ldrb	w13, [x11, #192]
	strb	w12, [x11, #259]
	strb	w12, [x11, #257]
	ldrb	w12, [x11, #128]
	strb	w13, [x11, #195]
	strb	w13, [x11, #193]
	ldrb	w13, [x11, #64]
	strb	w12, [x11, #131]
	strb	w12, [x11, #129]
	strb	w13, [x11, #67]
	strb	w13, [x11, #65]
	b.ne	.LBB4_6
	tbnz	w2, #0, .LBB4_27
	cmp	w1, #1
	b.lt	.LBB4_27
	cmp	w1, #2
	b.hs	.LBB4_11
	mov	x10, xzr
	b	.LBB4_25
.LBB4_11:
	sub	x12, x8, #1
	mov	x10, xzr
	and	x11, x12, #0xffffffff
	lsl	x14, x11, #10
	tst	x14, #0xffffffff00000000
	cset	w13, ne
	cmp	xzr, x12, lsr #54
	cset	w11, ne
	tbnz	w14, #31, .LBB4_25
	tbnz	w13, #0, .LBB4_25
	lsr	x13, x12, #32
	cbnz	x13, .LBB4_25
	lsl	x12, x12, #10
	add	x13, x0, x12
	cmp	x13, x0
	b.lo	.LBB4_25
	tbnz	w11, #0, .LBB4_25
	add	x13, x0, #1023
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB4_25
	tbnz	w11, #0, .LBB4_25
	add	x13, x0, #989
	add	x14, x13, x12
	cmp	x14, x13
	b.lo	.LBB4_25
	tbnz	w11, #0, .LBB4_25
	add	x13, x0, #991
	add	x12, x13, x12
	cmp	x12, x13
	b.lo	.LBB4_25
	tbnz	w11, #0, .LBB4_25
	mov	x11, xzr
	and	x10, x8, #0xfffffffe
	and	x9, x9, #0x3fffffff800
.LBB4_23:
	add	w12, w11, #1024
	add	x13, x0, w11, sxtw
	add	x11, x11, #2048
	add	x12, x0, w12, sxtw
	strb	wzr, [x13, #991]
	strb	wzr, [x12, #991]
	strb	wzr, [x12, #989]
	strb	wzr, [x13, #989]
	ldrb	w14, [x13]
	ldrb	w15, [x12]
	eor	w16, w14, #0x3
	cmp	w14, #0
	eor	w17, w15, #0x3
	csel	w14, wzr, w16, eq
	cmp	w15, #0
	csel	w15, wzr, w17, eq
	cmp	x9, x11
	strb	w14, [x13, #1023]
	strb	w15, [x12, #1023]
	b.ne	.LBB4_23
	cmp	x10, x8
	b.eq	.LBB4_27
.LBB4_25:
	lsl	w9, w10, #10
	sub	x8, x8, x10
.LBB4_26:
	add	x10, x0, w9, sxtw
	add	w9, w9, #1024
	ldrb	w11, [x10]
	strb	wzr, [x10, #991]
	strb	wzr, [x10, #989]
	eor	w12, w11, #0x3
	cmp	w11, #0
	csel	w11, wzr, w12, eq
	subs	x8, x8, #1
	strb	w11, [x10, #1023]
	b.ne	.LBB4_26
.LBB4_27:
	bl	epd_regal_repair
	ldp	x29, x30, [sp], #16
	ldr	x30, [x18, #-8]!
	ret
.Lfunc_end4:
	.size	pvi_wf_regal_fix, .Lfunc_end4-pvi_wf_regal_fix

	.globl	pvi_wf_get_original_lut
	.p2align	2
	.type	pvi_wf_get_original_lut,@function
pvi_wf_get_original_lut:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	adrp	x8, global_waveform
	stp	x20, x19, [sp, #16]
	mov	x29, sp
	ldr	x8, [x8, :lo12:global_waveform]
	cbz	x8, .LBB5_8
	mov	x19, x0
	cbnz	x0, .LBB5_3
	mov	w9, #24
	ldr	x9, [x9]
	cbz	x9, .LBB5_15
.LBB5_3:
	adrp	x10, need_pic
	ldr	x11, [x19, #24]
	adrp	x12, waveformdata
	ldrb	w9, [x8, #38]
	str	w3, [x10, :lo12:need_pic]
	str	x11, [x12, :lo12:waveformdata]
	cbz	w9, .LBB5_9
	mov	w10, wzr
.LBB5_5:
	add	x11, x8, w10, sxtw
	ldrb	w11, [x11, #48]
	cmp	w11, w2
	b.ge	.LBB5_10
	add	w10, w10, #1
	cmp	w9, w10
	b.ne	.LBB5_5
	mov	w8, #-1
	mov	w10, w9
	b	.LBB5_11
.LBB5_8:
	mov	w0, #-19
	b	.LBB5_14
.LBB5_9:
	mov	w20, wzr
	b	.LBB5_12
.LBB5_10:
	mov	w8, w10
.LBB5_11:
	sub	w11, w10, #1
	cmp	w10, w9
	csel	w20, w11, w8, eq
.LBB5_12:
	mov	w0, w1
	bl	get_wf_mode_index
	and	w0, w0, #0xff
	mov	w1, w20
	bl	get_wf_frm_num
	tbnz	w0, #31, .LBB5_16
	mov	w8, w0
	mov	w0, wzr
	str	w8, [x19]
.LBB5_14:
	ldp	x20, x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.LBB5_15:
	mov	w0, #-22
	b	.LBB5_14
.LBB5_16:
	adrp	x0, .L.str
	add	x0, x0, :lo12:.L.str
	bl	_printk
	mov	w0, #-5
	b	.LBB5_14
.Lfunc_end5:
	.size	pvi_wf_get_original_lut, .Lfunc_end5-pvi_wf_get_original_lut

	.globl	pvi_wf_input
	.p2align	2
	.type	pvi_wf_input,@function
pvi_wf_input:
	str	x30, [x18], #8
	stp	x29, x30, [sp, #-32]!
	stp	x20, x19, [sp, #16]
	adrp	x20, global_waveform
	mov	x29, sp
	ldr	x8, [x20, :lo12:global_waveform]
	cbz	x8, .LBB6_2
	mov	w0, #-16
	b	.LBB6_6
.LBB6_2:
	ldrb	w1, [x0, #16]
	mov	x19, x0
	cmp	w1, #114
	b.hi	.LBB6_7
	adrp	x9, .LJTI6_0
	mov	w0, #-8
	add	x9, x9, :lo12:.LJTI6_0
	mov	w8, #32
	adr	x10, .LBB6_4
	ldrb	w11, [x9, x1]
	add	x10, x10, x11, lsl #2
	br	x10
.LBB6_4:
	adrp	x9, maxpic
	str	w8, [x9, :lo12:maxpic]
.LBB6_5:
	adrp	x8, maxpic
	adrp	x0, .L.str.2
	add	x0, x0, :lo12:.L.str.2
	ldr	w2, [x8, :lo12:maxpic]
	bl	_printk
	mov	w0, wzr
	str	x19, [x20, :lo12:global_waveform]
.LBB6_6:
	ldp	x20, x19, [sp, #16]
	ldp	x29, x30, [sp], #32
	ldr	x30, [x18, #-8]!
	ret
.LBB6_7:
	adrp	x8, custom_mode_table+9
	ldrb	w8, [x8, :lo12:custom_mode_table+9]
	cmp	w1, w8
	b.ne	.LBB6_9
	adrp	x8, custom_mode_table+12
	ldr	w8, [x8, :lo12:custom_mode_table+12]
	b	.LBB6_4
.LBB6_9:
	adrp	x0, .L.str.1
	ldrb	w2, [x19, #22]
	add	x0, x0, :lo12:.L.str.1
	bl	_printk
	mov	w0, #-8
	b	.LBB6_6
.Lfunc_end6:
	.size	pvi_wf_input, .Lfunc_end6-pvi_wf_input
	.section	.rodata,"a",@progbits
.LJTI6_0:
	.byte	(.LBB6_6-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_5-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_5-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_5-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_5-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_7-.LBB6_4)>>2
	.byte	(.LBB6_4-.LBB6_4)>>2

	.text
	.globl	pvi_wf_get_version
	.p2align	2
	.type	pvi_wf_get_version,@function
pvi_wf_get_version:
	adrp	x8, global_waveform
	ldr	x9, [x8, :lo12:global_waveform]
	cbz	x9, .LBB7_5
	ldrb	w8, [x9, #64]
	mov	w10, #63
	adrp	x0, pvi_wf_get_version.spi_id_buffer
	add	x0, x0, :lo12:pvi_wf_get_version.spi_id_buffer
	cmp	w8, #63
	csel	w8, w8, w10, lo
	cbz	w8, .LBB7_4
	adrp	x11, pvi_wf_get_version.spi_id_buffer
	add	x9, x9, #65
	mov	x10, x8
	add	x11, x11, :lo12:pvi_wf_get_version.spi_id_buffer
.LBB7_3:
	ldrb	w12, [x9], #1
	subs	x10, x10, #1
	strb	w12, [x11], #1
	b.ne	.LBB7_3
.LBB7_4:
	add	x8, x0, w8, uxtw
	strb	wzr, [x8, #1]
	ret
.LBB7_5:
	mov	x0, xzr
	ret
.Lfunc_end7:
	.size	pvi_wf_get_version, .Lfunc_end7-pvi_wf_get_version

	.globl	pvi_wf_get_wf_bit
	.p2align	2
	.type	pvi_wf_get_wf_bit,@function
pvi_wf_get_wf_bit:
	adrp	x8, global_waveform
	adrp	x9, maxpic
	ldr	x8, [x8, :lo12:global_waveform]
	ldr	w9, [x9, :lo12:maxpic]
	cmp	x8, #0
	csel	w0, wzr, w9, eq
	ret
.Lfunc_end8:
	.size	pvi_wf_get_wf_bit, .Lfunc_end8-pvi_wf_get_wf_bit

	.type	pvi_mode_table,@object
	.section	.rodata,"a",@progbits
	.globl	pvi_mode_table
pvi_mode_table:
	.ascii	"\000\001\001\002\003\003\003\004\003"
	.ascii	"\000\001\007\003\003\005\006\004\005"
	.ascii	"\000\001\001\002\003\004\004\006\005"
	.ascii	"\000\001\007\002\003\004\005\006\004"
	.ascii	"\000\001\005\002\003\003\003\004\003"
	.ascii	"\000\001\001\002\003\004\004\006\004"
	.ascii	"\000\001\001\002\003\004\004\005\004"
	.ascii	"\000\001\001\002\003\004\013\006\n"
	.ascii	"\000\001\001\002\003\004\005\006\004"
	.size	pvi_mode_table, 81

	.type	custom_mode_table,@object
	.bss
	.globl	custom_mode_table
	.p2align	2
custom_mode_table:
	.zero	16
	.size	custom_mode_table, 16

	.type	pvi_wf_get_lut.stype,@object
	.data
	.p2align	2
pvi_wf_get_lut.stype:
	.word	16
	.size	pvi_wf_get_lut.stype, 4

	.type	pvi_wf_get_lut.sftemp,@object
	.p2align	2
pvi_wf_get_lut.sftemp:
	.word	4294967295
	.size	pvi_wf_get_lut.sftemp, 4

	.type	pvi_wf_get_lut.normal_repair,@object
	.local	pvi_wf_get_lut.normal_repair
	.comm	pvi_wf_get_lut.normal_repair,4,4
	.type	global_waveform,@object
	.local	global_waveform
	.comm	global_waveform,8,8
	.type	need_pic,@object
	.p2align	2
need_pic:
	.word	16
	.size	need_pic, 4

	.type	waveformdata,@object
	.local	waveformdata
	.comm	waveformdata,8,8
	.type	.L.str,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str:
	.asciz	"pvi waveform get frame number failed\n"
	.size	.L.str, 38

	.type	maxpic,@object
	.data
	.p2align	2
maxpic:
	.word	16
	.size	maxpic, 4

	.type	.L.str.1,@object
	.section	.rodata.str1.1,"aMS",@progbits,1
.L.str.1:
	.asciz	"pvi : Unknow waveform version 0x%x, 0x%x, may be wrong waveform file\n"
	.size	.L.str.1, 70

	.type	.L.str.2,@object
.L.str.2:
	.asciz	"pvi : input waveform version 0x%x, gray level: %d\n"
	.size	.L.str.2, 51

	.type	pvi_wf_get_version.spi_id_buffer,@object
	.local	pvi_wf_get_version.spi_id_buffer
	.comm	pvi_wf_get_version.spi_id_buffer,64,1
	.type	.L.str.3,@object
.L.str.3:
	.asciz	"pvi: unspport PVI waveform type"
	.size	.L.str.3, 32

	.type	.L.str.4,@object
.L.str.4:
	.asciz	"pvi: %s %d check error\n"
	.size	.L.str.4, 24

	.type	.L__func__.get_wf_frm_num,@object
.L__func__.get_wf_frm_num:
	.asciz	"get_wf_frm_num"
	.size	.L__func__.get_wf_frm_num, 15

	.type	.L.str.5,@object
.L.str.5:
	.asciz	"waveformdata is NULL\n"
	.size	.L.str.5, 22

	.type	.L.str.6,@object
.L.str.6:
	.asciz	"pvi: decodec waveform error, framenum err\n"
	.size	.L.str.6, 43

	.type	.Lswitch.table.get_wf_mode_index,@object
	.section	.rodata,"a",@progbits
	.p2align	3
.Lswitch.table.get_wf_mode_index:
	.xword	5
	.xword	6
	.xword	4
	.xword	3
	.xword	5
	.size	.Lswitch.table.get_wf_mode_index, 40

	.ident	"Android (8508608, based on r450784e) clang version 14.0.7 (https://android.googlesource.com/toolchain/llvm-project 4c603efb0cca074e9238af8b4106c30add4418f6)"
	.section	".note.GNU-stack","",@progbits
	.addrsig
	.addrsig_sym pvi_wf_get_version.spi_id_buffer
