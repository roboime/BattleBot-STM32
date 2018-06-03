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
	r0: void* stack_ptr
	r1: uint32_t stack_size
	r2: void (*entry)(context, void*)
	r3: void* ud
*/
	.section .text.context_new
	.align 2
	.global context_new
	.thumb_func
	.type context_new,%function
context_new:
	add r0, r0, r1
	mov r1, lr
	ldr lr, =entry_point+1
	stmfd r0!, {r3}
	stmfd r0!, {r2}
	stmfd r0!, {r4-r11, lr}
	mov lr, r1
	bx lr
	.size context_new, .-context_new

/*
	entry_point
*/
entry_point:
	pop {r2}
	pop {r1}
	blx r2
	mov r3, #0
	mov sp, r3
	b _mid_ctx

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
	tst r0, r0
	beq _ret_pt

	push {r4-r11, lr}
_mid_ctx:
	mov r1, r0
	mov r0, sp
	mov sp, r1
	pop {r4-r11, lr}

_ret_pt:
	bx lr
	.size context_switch, .-context_switch

