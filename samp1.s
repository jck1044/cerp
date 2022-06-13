	.syntax unified
	.cpu cortex-m0
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 1
	.eabi_attribute 34, 0
	.eabi_attribute 18, 4
	.thumb
	.syntax unified
	.file	"samp.c"
	.text
.Ltext0:
	.cfi_sections	.debug_frame
	.section	.text.tim2_isr,"ax",%progbits
	.align	2
	.global	tim2_isr
	.code	16
	.thumb_func
	.type	tim2_isr, %function
tim2_isr:
.LFB0:
	.file 1 "samp.c"
	.loc 1 37 0
	.cfi_startproc
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.cfi_def_cfa_offset 16
	.cfi_offset 4, -16
	.cfi_offset 5, -12
	.cfi_offset 6, -8
	.cfi_offset 14, -4
	.loc 1 43 0
	ldr	r3, .L12
	ldr	r3, [r3]
	movs	r2, #3
	ands	r2, r3  @ r2 phase
.LVL0:
	.loc 1 47 0
	ldr	r3, .L12+4  @ r3 &SPI1_DR
	movs	r1, #255
	lsls	r1, r1, #8
	rev16	r1, r1  @ byte swap
	str	r1, [r3]
	.loc 1 48 0
	movs	r1, #254
	str	r1, [r3]
.L2:
	.loc 1 49 0 discriminator 1
	ldr	r3, .L12+8
	ldr	r3, [r3]
	lsls	r3, r3, #31
	bpl	.L2
	.loc 1 50 0
	ldr	r1, .L12+4
	ldr	r3, [r1]
	ldr	r0, .L12+12
	lsls	r4, r2, #2
	ldr	r5, [r4, r0]
	mov	ip, r5
	add	r3, r3, ip
	str	r3, [r4, r0]
	.loc 1 51 0
	movs	r3, #2
	str	r3, [r1]
.L3:
	.loc 1 52 0 discriminator 1
	ldr	r3, .L12+8
	ldr	r3, [r3]
	lsls	r3, r3, #31
	bpl	.L3
	.loc 1 53 0
	ldr	r3, .L12+4
	ldr	r3, [r3]
	ldr	r1, .L12+16
	lsls	r0, r2, #2
	ldr	r4, [r0, r1]
	mov	ip, r4
	add	r3, r3, ip
	str	r3, [r0, r1]
.L4:
	.loc 1 54 0 discriminator 1
	ldr	r3, .L12+8
	ldr	r3, [r3]
	lsls	r3, r3, #31
	bpl	.L4
	.loc 1 55 0
	ldr	r3, .L12+4
	ldr	r3, [r3]
	ldr	r1, .L12+20
	lsls	r0, r2, #2
	ldr	r4, [r0, r1]
	mov	ip, r4
	add	r3, r3, ip
	str	r3, [r0, r1]
	.loc 1 57 0
	ldr	r3, .L12+24
	ldr	r1, [r3]
	ldr	r3, .L12+28
	ldr	r3, [r3]
	cmp	r1, r3
	bcs	.L5
	.loc 1 57 0 is_stmt 0 discriminator 1
	ldr	r1, .L12+24
	str	r3, [r1]
	b	.L6
.L5:
	.loc 1 58 0 is_stmt 1
	ldr	r3, .L12+32
	ldr	r1, [r3]
	ldr	r3, .L12+24
	str	r1, [r3]
.L6:
	.loc 1 62 0
	ldr	r3, .L12+36
	ldr	r0, [r3]
	ldr	r1, .L12+40
	ands	r1, r0
	str	r1, [r3]
	.loc 1 63 0
	ldr	r1, [r3]
	movs	r0, #128
	lsls	r0, r0, #7
	orrs	r1, r0
	str	r1, [r3]
	.loc 1 64 0
	ldr	r0, [r3]
	ldr	r1, .L12+44
	ands	r1, r0
	str	r1, [r3]
	.loc 1 65 0
	ldr	r1, [r3]
	movs	r0, #192
	lsls	r0, r0, #6
	orrs	r1, r0
	str	r1, [r3]
	.loc 1 68 0
	ldr	r3, .L12
	ldr	r3, [r3]
	lsls	r3, r3, #23
	lsrs	r3, r3, #23
	cmp	r3, #16
	bne	.L7
	.loc 1 70 0
	movs	r1, #128
	lsls	r1, r1, #11
	ldr	r3, .L12+48
	str	r1, [r3]
	.loc 1 71 0
	ldr	r3, .L12+52
	ldr	r1, [r3]
	ldr	r3, .L12+4
	str	r1, [r3]
.L8:
	.loc 1 72 0 discriminator 1
	ldr	r3, .L12+8
	ldr	r3, [r3]
	lsls	r3, r3, #24
	bmi	.L8
	.loc 1 73 0
	ldr	r1, .L12+48
	movs	r3, #4
	str	r3, [r1]
	.loc 1 74 0
	ldr	r3, .L12+4
	ldr	r0, [r3]
	.loc 1 76 0
	movs	r0, #128
	lsls	r0, r0, #19
	str	r0, [r1]
	.loc 1 77 0
	ldr	r1, .L12+56
	ldr	r1, [r1]
	str	r1, [r3]
.L9:
	.loc 1 78 0 discriminator 1
	ldr	r3, .L12+8
	ldr	r3, [r3]
	lsls	r3, r3, #24
	bmi	.L9
	.loc 1 79 0
	ldr	r1, .L12+48
	movs	r3, #128
	lsls	r3, r3, #3
	str	r3, [r1]
	.loc 1 80 0
	ldr	r3, .L12+4
	ldr	r0, [r3]
	.loc 1 82 0
	movs	r0, #128
	lsls	r0, r0, #20
	str	r0, [r1]
	.loc 1 83 0
	ldr	r1, .L12+60
	ldr	r1, [r1]
	str	r1, [r3]
.L10:
	.loc 1 84 0 discriminator 1
	ldr	r3, .L12+8
	ldr	r3, [r3]
	lsls	r3, r3, #24
	bmi	.L10
	.loc 1 85 0
	movs	r1, #128
	lsls	r1, r1, #4
	ldr	r3, .L12+48
	str	r1, [r3]
	.loc 1 86 0
	ldr	r3, .L12+4
	ldr	r3, [r3]
.L7:
	.loc 1 89 0
	ldr	r3, .L12
	ldr	r1, [r3]
	movs	r3, #254
	lsls	r3, r3, #1
	tst	r1, r3
	bne	.L11
	.loc 1 91 0
	ldr	r4, .L12+12
	lsls	r3, r2, #2
	ldr	r0, .L12+64
	ldr	r5, [r3, r4]
	str	r5, [r3, r0]
	.loc 1 92 0
	movs	r0, #0
	str	r0, [r3, r4]
	.loc 1 93 0
	ldr	r4, .L12+16
	ldr	r5, .L12+68
	ldr	r6, [r3, r4]
	str	r6, [r3, r5]
	.loc 1 94 0
	str	r0, [r3, r4]
	.loc 1 95 0
	ldr	r4, .L12+20
	ldr	r5, .L12+72
	ldr	r6, [r3, r4]
	str	r6, [r3, r5]
	.loc 1 96 0
	str	r0, [r3, r4]
	.loc 1 98 0
	cmp	r2, #3
	bne	.L11
	.loc 1 98 0 is_stmt 0 discriminator 1
	ldr	r2, .L12+76
.LVL1:
	ldr	r3, [r2]
	adds	r3, r3, #1
	str	r3, [r2]
.L11:
	.loc 1 100 0 is_stmt 1
	adds	r1, r1, #1
	ldr	r3, .L12
	str	r1, [r3]
	.loc 1 102 0
	movs	r2, #5
	rsbs	r2, r2, #0
	ldr	r3, .L12+80
	str	r2, [r3]
	.loc 1 105 0
	@ sp needed
	pop	{r4, r5, r6, pc}
.L13:
	.align	2
.L12:
	.word	n
	.word	1073819660
	.word	1073819656
	.word	sumx
	.word	sumy
	.word	sumz
	.word	1073741880
	.word	dly2
	.word	dly1
	.word	1073741848
	.word	-12289
	.word	-16385
	.word	1207960600
	.word	dac0fb
	.word	dac1fb
	.word	dac2fb
	.word	avex
	.word	avey
	.word	avez
	.word	nsamp
	.word	1073741840
	.cfi_endproc
.LFE0:
	.size	tim2_isr, .-tim2_isr
	.section	.text.StartSAMP,"ax",%progbits
	.align	2
	.global	StartSAMP
	.code	16
	.thumb_func
	.type	StartSAMP, %function
StartSAMP:
.LFB1:
	.loc 1 110 0
	.cfi_startproc
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	.cfi_def_cfa_offset 20
	.cfi_offset 4, -20
	.cfi_offset 5, -16
	.cfi_offset 6, -12
	.cfi_offset 7, -8
	.cfi_offset 14, -4
	sub	sp, sp, #12
	.cfi_def_cfa_offset 32
	.loc 1 112 0
	ldr	r4, .L15
	movs	r0, r4
	bl	spi_reset
.LVL2:
	.loc 1 114 0
	movs	r6, #0
	str	r6, [sp, #4]
	str	r6, [sp]
	movs	r3, #0
	movs	r2, #0
	movs	r1, #8
	movs	r0, r4
	bl	spi_init_master
.LVL3:
	.loc 1 116 0
	movs	r1, #16
	movs	r0, r4
	bl	spi_set_data_size
.LVL4:
	.loc 1 117 0
	movs	r0, r4
	bl	spi_enable_software_slave_management
.LVL5:
	.loc 1 118 0
	movs	r0, r4
	bl	spi_set_nss_high
.LVL6:
	.loc 1 120 0
	movs	r0, r4
	bl	spi_enable
.LVL7:
	.loc 1 123 0
	ldr	r4, .L15+4
	movs	r0, r4
	bl	timer_reset
.LVL8:
	.loc 1 124 0
	movs	r5, #128
	lsls	r5, r5, #23
	movs	r0, r5
	bl	timer_reset
.LVL9:
	.loc 1 126 0
	movs	r3, #0
	movs	r2, #0
	movs	r1, #128
	lsls	r1, r1, #1
	movs	r0, r4
	bl	timer_set_mode
.LVL10:
	.loc 1 128 0
	ldr	r7, .L15+8
	ldr	r3, [r7]
	subs	r1, r3, #1
	movs	r0, r4
	bl	timer_set_period
.LVL11:
	.loc 1 130 0
	movs	r2, #6
	movs	r1, #0
	movs	r0, r4
	bl	timer_set_oc_mode
.LVL12:
	.loc 1 131 0
	movs	r1, #0
	movs	r0, r4
	bl	timer_enable_oc_output
.LVL13:
	.loc 1 132 0
	movs	r0, r4
	bl	timer_enable_break_main_output
.LVL14:
	.loc 1 133 0
	ldr	r3, [r7]
	lsrs	r2, r3, #1
	movs	r1, #0
	movs	r0, r4
	bl	timer_set_oc_value
.LVL15:
	.loc 1 135 0
	movs	r2, #7
	movs	r1, #6
	movs	r0, r4
	bl	timer_set_oc_mode
.LVL16:
	.loc 1 136 0
	ldr	r3, [r7]
	lsrs	r2, r3, #1
	movs	r1, #6
	movs	r0, r4
	bl	timer_set_oc_value
.LVL17:
	.loc 1 137 0
	movs	r1, #6
	movs	r0, r4
	bl	timer_enable_oc_output
.LVL18:
	.loc 1 139 0
	movs	r3, #0
	movs	r2, #0
	movs	r1, #128
	lsls	r1, r1, #1
	movs	r0, r5
	bl	timer_set_mode
.LVL19:
	.loc 1 141 0
	ldr	r1, [r7]
	lsrs	r1, r1, #1
	subs	r1, r1, #1
	movs	r0, r5
	bl	timer_set_period
.LVL20:
	.loc 1 143 0
	movs	r2, #3
	movs	r1, #2
	movs	r0, r5
	bl	timer_set_oc_mode
.LVL21:
	.loc 1 144 0
	movs	r1, #2
	movs	r0, r5
	bl	timer_enable_oc_output
.LVL22:
	.loc 1 145 0
	ldr	r3, .L15+12
	ldr	r2, [r3]
	movs	r1, #2
	movs	r0, r5
	bl	timer_set_oc_value
.LVL23:
	.loc 1 149 0
	movs	r1, #68
	movs	r0, r5
	bl	timer_generate_event
.LVL24:
	.loc 1 150 0
	movs	r0, #15
	bl	nvic_enable_irq
.LVL25:
	.loc 1 151 0
	movs	r1, #4
	movs	r0, r5
	bl	timer_enable_irq
.LVL26:
	.loc 1 154 0
	ldr	r2, [r4]
	movs	r3, #1
	orrs	r2, r3
	str	r2, [r4]
	.loc 1 155 0
	ldr	r2, [r5]
	orrs	r2, r3
	str	r2, [r5]
	.loc 1 156 0
	ldr	r2, .L15+16
	ldr	r1, [r2]
	orrs	r3, r1
	str	r3, [r2]
	.loc 1 158 0
	ldr	r3, .L15+20
	str	r6, [r3]
	.loc 1 159 0
	ldr	r3, .L15+24
	str	r6, [r3]
	.loc 1 161 0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L16:
	.align	2
.L15:
	.word	1073819648
	.word	1073818624
	.word	drvper
	.word	dly1
	.word	1073742848
	.word	n
	.word	nsamp
	.cfi_endproc
.LFE1:
	.size	StartSAMP, .-StartSAMP
	.section	.text.setDRVper,"ax",%progbits
	.align	2
	.global	setDRVper
	.code	16
	.thumb_func
	.type	setDRVper, %function
setDRVper:
.LFB2:
	.loc 1 167 0
	.cfi_startproc
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL27:
	push	{r4, r5, r6, lr}
	.cfi_def_cfa_offset 16
	.cfi_offset 4, -16
	.cfi_offset 5, -12
	.cfi_offset 6, -8
	.cfi_offset 14, -4
	movs	r4, r0
.LVL28:
.L18:
	.loc 1 169 0 discriminator 1
	ldr	r0, .L20
	bl	timer_get_counter
.LVL29:
	ldr	r3, .L20+4
	ldr	r3, [r3]
	subs	r3, r3, #10
	cmp	r0, r3
	bhi	.L18
.L19:
	.loc 1 171 0 discriminator 1
	ldr	r0, .L20
	bl	timer_get_counter
.LVL30:
	ldr	r3, .L20+4
	ldr	r3, [r3]
	lsrs	r3, r3, #2
	adds	r3, r3, #1
	cmp	r0, r3
	bcc	.L19
	.loc 1 172 0
	ldr	r5, .L20
	movs	r1, r4
	movs	r0, r5
	bl	timer_set_period
.LVL31:
	.loc 1 173 0
	lsrs	r6, r4, #1
	movs	r1, r6
	movs	r0, #128
	lsls	r0, r0, #23
	bl	timer_set_period
.LVL32:
	.loc 1 174 0
	lsls	r1, r4, #3
	ldr	r0, .L20+8
	bl	timer_set_period
.LVL33:
	.loc 1 175 0
	movs	r2, r6
	movs	r1, #0
	movs	r0, r5
	bl	timer_set_oc_value
.LVL34:
	.loc 1 176 0
	movs	r2, r6
	movs	r1, #6
	movs	r0, r5
	bl	timer_set_oc_value
.LVL35:
	.loc 1 177 0
	ldr	r3, .L20+4
	str	r4, [r3]
	.loc 1 178 0
	@ sp needed
.LVL36:
	pop	{r4, r5, r6, pc}
.L21:
	.align	2
.L20:
	.word	1073818624
	.word	drvper
	.word	1073742848
	.cfi_endproc
.LFE2:
	.size	setDRVper, .-setDRVper
	.text
.Letext0:
	.file 2 "/Users/markw/mystuff/source/gcc-arm/arm-none-eabi/include/machine/_default_types.h"
	.file 3 "/Users/markw/mystuff/source/gcc-arm/arm-none-eabi/include/sys/_stdint.h"
	.file 4 "/Users/markw/mystuff/source/libopencm3-master/include/libopencm3/stm32/common/timer_common_all.h"
	.file 5 "/Users/markw/mystuff/source/libopencm3-master/include/libopencm3/stm32/common/spi_common_all.h"
	.file 6 "/Users/markw/mystuff/source/libopencm3-master/include/libopencm3/stm32/common/spi_common_f03.h"
	.file 7 "/Users/markw/mystuff/source/libopencm3-master/include/libopencm3/cm3/nvic.h"
	.section	.debug_info,"",%progbits
.Ldebug_info0:
	.4byte	0x614
	.2byte	0x4
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.uleb128 0x1
	.4byte	.LASF62
	.byte	0xc
	.4byte	.LASF63
	.4byte	.LASF64
	.4byte	.Ldebug_ranges0+0
	.4byte	0
	.4byte	.Ldebug_line0
	.uleb128 0x2
	.byte	0x1
	.byte	0x6
	.4byte	.LASF0
	.uleb128 0x2
	.byte	0x1
	.byte	0x8
	.4byte	.LASF1
	.uleb128 0x2
	.byte	0x2
	.byte	0x5
	.4byte	.LASF2
	.uleb128 0x2
	.byte	0x2
	.byte	0x7
	.4byte	.LASF3
	.uleb128 0x2
	.byte	0x4
	.byte	0x5
	.4byte	.LASF4
	.uleb128 0x3
	.4byte	.LASF9
	.byte	0x2
	.byte	0x41
	.4byte	0x53
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF5
	.uleb128 0x2
	.byte	0x8
	.byte	0x5
	.4byte	.LASF6
	.uleb128 0x2
	.byte	0x8
	.byte	0x7
	.4byte	.LASF7
	.uleb128 0x4
	.byte	0x4
	.byte	0x5
	.ascii	"int\000"
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF8
	.uleb128 0x3
	.4byte	.LASF10
	.byte	0x3
	.byte	0x30
	.4byte	0x48
	.uleb128 0x5
	.4byte	.LASF18
	.byte	0x1
	.4byte	0x2c
	.byte	0x4
	.2byte	0x439
	.4byte	0xbd
	.uleb128 0x6
	.4byte	.LASF11
	.byte	0
	.uleb128 0x6
	.4byte	.LASF12
	.byte	0x1
	.uleb128 0x6
	.4byte	.LASF13
	.byte	0x2
	.uleb128 0x6
	.4byte	.LASF14
	.byte	0x3
	.uleb128 0x6
	.4byte	.LASF15
	.byte	0x4
	.uleb128 0x6
	.4byte	.LASF16
	.byte	0x5
	.uleb128 0x6
	.4byte	.LASF17
	.byte	0x6
	.byte	0
	.uleb128 0x5
	.4byte	.LASF19
	.byte	0x1
	.4byte	0x2c
	.byte	0x4
	.2byte	0x444
	.4byte	0xff
	.uleb128 0x6
	.4byte	.LASF20
	.byte	0
	.uleb128 0x6
	.4byte	.LASF21
	.byte	0x1
	.uleb128 0x6
	.4byte	.LASF22
	.byte	0x2
	.uleb128 0x6
	.4byte	.LASF23
	.byte	0x3
	.uleb128 0x6
	.4byte	.LASF24
	.byte	0x4
	.uleb128 0x6
	.4byte	.LASF25
	.byte	0x5
	.uleb128 0x6
	.4byte	.LASF26
	.byte	0x6
	.uleb128 0x6
	.4byte	.LASF27
	.byte	0x7
	.byte	0
	.uleb128 0x7
	.4byte	.LASF28
	.byte	0x1
	.byte	0x24
	.4byte	.LFB0
	.4byte	.LFE0-.LFB0
	.uleb128 0x1
	.byte	0x9c
	.4byte	0x124
	.uleb128 0x8
	.4byte	.LASF65
	.byte	0x1
	.byte	0x26
	.4byte	0x41
	.4byte	.LLST0
	.byte	0
	.uleb128 0x7
	.4byte	.LASF29
	.byte	0x1
	.byte	0x6d
	.4byte	.LFB1
	.4byte	.LFE1-.LFB1
	.uleb128 0x1
	.byte	0x9c
	.4byte	0x3bd
	.uleb128 0x9
	.4byte	.LVL2
	.4byte	0x54d
	.4byte	0x14d
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL3
	.4byte	0x559
	.4byte	0x180
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x38
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x53
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x2
	.byte	0x7d
	.sleb128 0
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x2
	.byte	0x7d
	.sleb128 4
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL4
	.4byte	0x565
	.4byte	0x199
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x40
	.byte	0
	.uleb128 0x9
	.4byte	.LVL5
	.4byte	0x570
	.4byte	0x1ad
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL6
	.4byte	0x57c
	.4byte	0x1c1
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL7
	.4byte	0x588
	.4byte	0x1d5
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL8
	.4byte	0x594
	.4byte	0x1e9
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL9
	.4byte	0x594
	.4byte	0x1fd
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL10
	.4byte	0x5a0
	.4byte	0x224
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x3
	.byte	0xa
	.2byte	0x100
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x53
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL11
	.4byte	0x5ac
	.4byte	0x238
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL12
	.4byte	0x5b8
	.4byte	0x257
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x1
	.byte	0x36
	.byte	0
	.uleb128 0x9
	.4byte	.LVL13
	.4byte	0x5c4
	.4byte	0x271
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL14
	.4byte	0x5d0
	.4byte	0x285
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL15
	.4byte	0x5dc
	.4byte	0x29f
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL16
	.4byte	0x5b8
	.4byte	0x2bd
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x36
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x1
	.byte	0x37
	.byte	0
	.uleb128 0x9
	.4byte	.LVL17
	.4byte	0x5dc
	.4byte	0x2d6
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x36
	.byte	0
	.uleb128 0x9
	.4byte	.LVL18
	.4byte	0x5c4
	.4byte	0x2ef
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x36
	.byte	0
	.uleb128 0x9
	.4byte	.LVL19
	.4byte	0x5a0
	.4byte	0x316
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x3
	.byte	0xa
	.2byte	0x100
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x53
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL20
	.4byte	0x5ac
	.4byte	0x32a
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL21
	.4byte	0x5b8
	.4byte	0x348
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x32
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x1
	.byte	0x33
	.byte	0
	.uleb128 0x9
	.4byte	.LVL22
	.4byte	0x5c4
	.4byte	0x361
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x32
	.byte	0
	.uleb128 0x9
	.4byte	.LVL23
	.4byte	0x5dc
	.4byte	0x37a
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x32
	.byte	0
	.uleb128 0x9
	.4byte	.LVL24
	.4byte	0x5e8
	.4byte	0x394
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x2
	.byte	0x8
	.byte	0x44
	.byte	0
	.uleb128 0x9
	.4byte	.LVL25
	.4byte	0x5f4
	.4byte	0x3a7
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x1
	.byte	0x3f
	.byte	0
	.uleb128 0xb
	.4byte	.LVL26
	.4byte	0x5ff
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x34
	.byte	0
	.byte	0
	.uleb128 0x7
	.4byte	.LASF30
	.byte	0x1
	.byte	0xa6
	.4byte	.LFB2
	.4byte	.LFE2-.LFB2
	.uleb128 0x1
	.byte	0x9c
	.4byte	0x49e
	.uleb128 0xc
	.ascii	"per\000"
	.byte	0x1
	.byte	0xa6
	.4byte	0x76
	.4byte	.LLST1
	.uleb128 0x9
	.4byte	.LVL29
	.4byte	0x60b
	.4byte	0x3f8
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x5
	.byte	0xc
	.4byte	0x40012c00
	.byte	0
	.uleb128 0x9
	.4byte	.LVL30
	.4byte	0x60b
	.4byte	0x40f
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x5
	.byte	0xc
	.4byte	0x40012c00
	.byte	0
	.uleb128 0x9
	.4byte	.LVL31
	.4byte	0x5ac
	.4byte	0x429
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x2
	.byte	0x74
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL32
	.4byte	0x5ac
	.4byte	0x444
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x3
	.byte	0x40
	.byte	0x4a
	.byte	0x24
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0x9
	.4byte	.LVL33
	.4byte	0x5ac
	.4byte	0x463
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x5
	.byte	0xc
	.4byte	0x40000400
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x4
	.byte	0x74
	.sleb128 0
	.byte	0x33
	.byte	0x24
	.byte	0
	.uleb128 0x9
	.4byte	.LVL34
	.4byte	0x5dc
	.4byte	0x482
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x30
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.uleb128 0xb
	.4byte	.LVL35
	.4byte	0x5dc
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x50
	.uleb128 0x2
	.byte	0x75
	.sleb128 0
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x51
	.uleb128 0x1
	.byte	0x36
	.uleb128 0xa
	.uleb128 0x1
	.byte	0x52
	.uleb128 0x2
	.byte	0x76
	.sleb128 0
	.byte	0
	.byte	0
	.uleb128 0xd
	.ascii	"n\000"
	.byte	0x1
	.byte	0x16
	.4byte	0x41
	.uleb128 0xe
	.4byte	.LASF31
	.byte	0x1
	.byte	0x17
	.4byte	0x41
	.uleb128 0xf
	.4byte	0x41
	.4byte	0x4c2
	.uleb128 0x10
	.4byte	0x4c2
	.byte	0x3
	.byte	0
	.uleb128 0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF32
	.uleb128 0xe
	.4byte	.LASF33
	.byte	0x1
	.byte	0x19
	.4byte	0x4b2
	.uleb128 0xe
	.4byte	.LASF34
	.byte	0x1
	.byte	0x19
	.4byte	0x4b2
	.uleb128 0xe
	.4byte	.LASF35
	.byte	0x1
	.byte	0x19
	.4byte	0x4b2
	.uleb128 0xe
	.4byte	.LASF36
	.byte	0x1
	.byte	0x1a
	.4byte	0x4b2
	.uleb128 0xe
	.4byte	.LASF37
	.byte	0x1
	.byte	0x1a
	.4byte	0x4b2
	.uleb128 0xe
	.4byte	.LASF38
	.byte	0x1
	.byte	0x1a
	.4byte	0x4b2
	.uleb128 0xe
	.4byte	.LASF39
	.byte	0x1
	.byte	0x1c
	.4byte	0x41
	.uleb128 0xe
	.4byte	.LASF40
	.byte	0x1
	.byte	0x1c
	.4byte	0x41
	.uleb128 0xe
	.4byte	.LASF41
	.byte	0x1
	.byte	0x1c
	.4byte	0x41
	.uleb128 0xe
	.4byte	.LASF42
	.byte	0x1
	.byte	0x1e
	.4byte	0x76
	.uleb128 0xe
	.4byte	.LASF43
	.byte	0x1
	.byte	0x1f
	.4byte	0x76
	.uleb128 0xe
	.4byte	.LASF44
	.byte	0x1
	.byte	0x20
	.4byte	0x76
	.uleb128 0x11
	.4byte	.LASF45
	.4byte	.LASF45
	.byte	0x5
	.2byte	0x15b
	.uleb128 0x11
	.4byte	.LASF46
	.4byte	.LASF46
	.byte	0x5
	.2byte	0x15c
	.uleb128 0x12
	.4byte	.LASF47
	.4byte	.LASF47
	.byte	0x6
	.byte	0x6c
	.uleb128 0x11
	.4byte	.LASF48
	.4byte	.LASF48
	.byte	0x5
	.2byte	0x172
	.uleb128 0x11
	.4byte	.LASF49
	.4byte	.LASF49
	.byte	0x5
	.2byte	0x173
	.uleb128 0x11
	.4byte	.LASF50
	.4byte	.LASF50
	.byte	0x5
	.2byte	0x15e
	.uleb128 0x11
	.4byte	.LASF51
	.4byte	.LASF51
	.byte	0x4
	.2byte	0x492
	.uleb128 0x11
	.4byte	.LASF52
	.4byte	.LASF52
	.byte	0x4
	.2byte	0x498
	.uleb128 0x11
	.4byte	.LASF53
	.4byte	.LASF53
	.byte	0x4
	.2byte	0x4b5
	.uleb128 0x11
	.4byte	.LASF54
	.4byte	.LASF54
	.byte	0x4
	.2byte	0x4ba
	.uleb128 0x11
	.4byte	.LASF55
	.4byte	.LASF55
	.byte	0x4
	.2byte	0x4c1
	.uleb128 0x11
	.4byte	.LASF56
	.4byte	.LASF56
	.byte	0x4
	.2byte	0x4c9
	.uleb128 0x11
	.4byte	.LASF57
	.4byte	.LASF57
	.byte	0x4
	.2byte	0x4c7
	.uleb128 0x11
	.4byte	.LASF58
	.4byte	.LASF58
	.byte	0x4
	.2byte	0x4d7
	.uleb128 0x12
	.4byte	.LASF59
	.4byte	.LASF59
	.byte	0x7
	.byte	0x8b
	.uleb128 0x11
	.4byte	.LASF60
	.4byte	.LASF60
	.byte	0x4
	.2byte	0x493
	.uleb128 0x11
	.4byte	.LASF61
	.4byte	.LASF61
	.byte	0x4
	.2byte	0x4d8
	.byte	0
	.section	.debug_abbrev,"",%progbits
.Ldebug_abbrev0:
	.uleb128 0x1
	.uleb128 0x11
	.byte	0x1
	.uleb128 0x25
	.uleb128 0xe
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1b
	.uleb128 0xe
	.uleb128 0x55
	.uleb128 0x17
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x10
	.uleb128 0x17
	.byte	0
	.byte	0
	.uleb128 0x2
	.uleb128 0x24
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.byte	0
	.byte	0
	.uleb128 0x3
	.uleb128 0x16
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x4
	.uleb128 0x24
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0x8
	.byte	0
	.byte	0
	.uleb128 0x5
	.uleb128 0x4
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x6
	.uleb128 0x28
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1c
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x7
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0x19
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x6
	.uleb128 0x40
	.uleb128 0x18
	.uleb128 0x2117
	.uleb128 0x19
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x8
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x17
	.byte	0
	.byte	0
	.uleb128 0x9
	.uleb128 0x4109
	.byte	0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xa
	.uleb128 0x410a
	.byte	0
	.uleb128 0x2
	.uleb128 0x18
	.uleb128 0x2111
	.uleb128 0x18
	.byte	0
	.byte	0
	.uleb128 0xb
	.uleb128 0x4109
	.byte	0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x31
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xc
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x17
	.byte	0
	.byte	0
	.uleb128 0xd
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3c
	.uleb128 0x19
	.byte	0
	.byte	0
	.uleb128 0xe
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3c
	.uleb128 0x19
	.byte	0
	.byte	0
	.uleb128 0xf
	.uleb128 0x1
	.byte	0x1
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x10
	.uleb128 0x21
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2f
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x11
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3c
	.uleb128 0x19
	.uleb128 0x6e
	.uleb128 0xe
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.byte	0
	.byte	0
	.uleb128 0x12
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0x19
	.uleb128 0x3c
	.uleb128 0x19
	.uleb128 0x6e
	.uleb128 0xe
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_loc,"",%progbits
.Ldebug_loc0:
.LLST0:
	.4byte	.LVL0
	.4byte	.LVL1
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST1:
	.4byte	.LVL27
	.4byte	.LVL28
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL28
	.4byte	.LVL36
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL36
	.4byte	.LFE2
	.2byte	0x2
	.byte	0x73
	.sleb128 0
	.4byte	0
	.4byte	0
	.section	.debug_aranges,"",%progbits
	.4byte	0x2c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.LFB0
	.4byte	.LFE0-.LFB0
	.4byte	.LFB1
	.4byte	.LFE1-.LFB1
	.4byte	.LFB2
	.4byte	.LFE2-.LFB2
	.4byte	0
	.4byte	0
	.section	.debug_ranges,"",%progbits
.Ldebug_ranges0:
	.4byte	.LFB0
	.4byte	.LFE0
	.4byte	.LFB1
	.4byte	.LFE1
	.4byte	.LFB2
	.4byte	.LFE2
	.4byte	0
	.4byte	0
	.section	.debug_line,"",%progbits
.Ldebug_line0:
	.section	.debug_str,"MS",%progbits,1
.LASF33:
	.ascii	"sumx\000"
.LASF34:
	.ascii	"sumy\000"
.LASF20:
	.ascii	"TIM_OCM_FROZEN\000"
.LASF21:
	.ascii	"TIM_OCM_ACTIVE\000"
.LASF54:
	.ascii	"timer_set_oc_mode\000"
.LASF16:
	.ascii	"TIM_OC3N\000"
.LASF25:
	.ascii	"TIM_OCM_FORCE_HIGH\000"
.LASF2:
	.ascii	"short int\000"
.LASF32:
	.ascii	"sizetype\000"
.LASF41:
	.ascii	"dac2fb\000"
.LASF65:
	.ascii	"phase\000"
.LASF11:
	.ascii	"TIM_OC1\000"
.LASF13:
	.ascii	"TIM_OC2\000"
.LASF15:
	.ascii	"TIM_OC3\000"
.LASF9:
	.ascii	"__uint32_t\000"
.LASF49:
	.ascii	"spi_set_nss_high\000"
.LASF17:
	.ascii	"TIM_OC4\000"
.LASF40:
	.ascii	"dac1fb\000"
.LASF55:
	.ascii	"timer_enable_oc_output\000"
.LASF6:
	.ascii	"long long int\000"
.LASF46:
	.ascii	"spi_init_master\000"
.LASF29:
	.ascii	"StartSAMP\000"
.LASF61:
	.ascii	"timer_get_counter\000"
.LASF39:
	.ascii	"dac0fb\000"
.LASF4:
	.ascii	"long int\000"
.LASF18:
	.ascii	"tim_oc_id\000"
.LASF28:
	.ascii	"tim2_isr\000"
.LASF30:
	.ascii	"setDRVper\000"
.LASF62:
	.ascii	"GNU C99 5.4.1 20160919 (release) [ARM/embedded-5-br"
	.ascii	"anch revision 240496] -mthumb -mcpu=cortex-m0 -mflo"
	.ascii	"at-abi=soft -g -Og -std=c99 -fno-common -ffunction-"
	.ascii	"sections -fdata-sections\000"
.LASF24:
	.ascii	"TIM_OCM_FORCE_LOW\000"
.LASF1:
	.ascii	"unsigned char\000"
.LASF22:
	.ascii	"TIM_OCM_INACTIVE\000"
.LASF63:
	.ascii	"samp.c\000"
.LASF0:
	.ascii	"signed char\000"
.LASF7:
	.ascii	"long long unsigned int\000"
.LASF19:
	.ascii	"tim_oc_mode\000"
.LASF10:
	.ascii	"uint32_t\000"
.LASF58:
	.ascii	"timer_generate_event\000"
.LASF12:
	.ascii	"TIM_OC1N\000"
.LASF8:
	.ascii	"unsigned int\000"
.LASF43:
	.ascii	"dly1\000"
.LASF44:
	.ascii	"dly2\000"
.LASF36:
	.ascii	"avex\000"
.LASF37:
	.ascii	"avey\000"
.LASF38:
	.ascii	"avez\000"
.LASF3:
	.ascii	"short unsigned int\000"
.LASF42:
	.ascii	"drvper\000"
.LASF26:
	.ascii	"TIM_OCM_PWM1\000"
.LASF27:
	.ascii	"TIM_OCM_PWM2\000"
.LASF60:
	.ascii	"timer_enable_irq\000"
.LASF56:
	.ascii	"timer_enable_break_main_output\000"
.LASF51:
	.ascii	"timer_reset\000"
.LASF52:
	.ascii	"timer_set_mode\000"
.LASF31:
	.ascii	"nsamp\000"
.LASF5:
	.ascii	"long unsigned int\000"
.LASF64:
	.ascii	"/Users/markw/mystuff/source/LNFGsource/LNmag/LNmag\000"
.LASF14:
	.ascii	"TIM_OC2N\000"
.LASF47:
	.ascii	"spi_set_data_size\000"
.LASF45:
	.ascii	"spi_reset\000"
.LASF23:
	.ascii	"TIM_OCM_TOGGLE\000"
.LASF35:
	.ascii	"sumz\000"
.LASF53:
	.ascii	"timer_set_period\000"
.LASF59:
	.ascii	"nvic_enable_irq\000"
.LASF48:
	.ascii	"spi_enable_software_slave_management\000"
.LASF57:
	.ascii	"timer_set_oc_value\000"
.LASF50:
	.ascii	"spi_enable\000"
	.ident	"GCC: (GNU Tools for ARM Embedded Processors) 5.4.1 20160919 (release) [ARM/embedded-5-branch revision 240496]"
