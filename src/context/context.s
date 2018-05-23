/*
 * context.h
 *
 *  Created on: 22 de mai de 2018
 *      Author: joaobapt
 */

  .syntax unified
	.cpu cortex-m3
	.fpu softvfp
	.thumb

/*
	context_init
	r0: context
	r1: entry
*/
	.section .text.context_init
	.align 2
	.global context_init
	.thumb_func
	.type context_init,%function
context_init:
	str r4, [r0]
	str r5, [r0, #4]
	str r6, [r0, #8]
	str r7, [r0, #12]
	str r8, [r0, #16]
	str r9, [r0, #20]
	str r10, [r0, #24]
	str r11, [r0, #28]
	str sp, [r0, #32]
	str lr, [r0, #36]

	add r2, r0, #4*522
	mov sp, r2
	blx r1
	sub r0, sp, #4*522

	ldr r4, [r0]
	ldr r5, [r0, #4]
	ldr r6, [r0, #8]
	ldr r7, [r0, #12]
	ldr r8, [r0, #16]
	ldr r9, [r0, #20]
	ldr r10, [r0, #24]
	ldr r11, [r0, #28]
	ldr sp, [r0, #32]
	ldr lr, [r0, #36]
	mvn r3, #0
	str r3, [r0, #32]
	bx lr
	.size context_init, .-context_init

/*
	context_switch
	r0: context
*/
	.section .text.context_switch
	.align 2
	.global context_switch
	.thumb_func
	.type context_switch,%function
context_switch:
	ldr r3, [r0, #32]
	mvns r3, r3
	beq ret_pt

	mov r3, r4
	ldr r4, [r0, #0]
	str r3, [r0, #0]

	mov r3, r5
	ldr r5, [r0, #4]
	str r3, [r0, #4]

	mov r3, r6
	ldr r6, [r0, #8]
	str r3, [r0, #8]

	mov r3, r7
	ldr r7, [r0, #12]
	str r3, [r0, #12]

	mov r3, r8
	ldr r8, [r0, #16]
	str r3, [r0, #16]

	mov r3, r9
	ldr r9, [r0, #20]
	str r3, [r0, #20]

	mov r3, r10
	ldr r10, [r0, #24]
	str r3, [r0, #24]

	mov r3, r11
	ldr r11, [r0, #28]
	str r3, [r0, #28]

	mov r3, sp
	ldr sp, [r0, #32]
	str r3, [r0, #32]

	mov r3, lr
	ldr lr, [r0, #36]
	str r3, [r0, #36]

ret_pt:
	bx lr
	.size context_switch, .-context_switch
