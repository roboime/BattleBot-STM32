/*
 * context.h
 *
 *  Created on: 22 de mai de 2018
 *      Author: joaobapt
 */

#ifndef CONTEXT_CONTEXT_H_
#define CONTEXT_CONTEXT_H_

/**
 * Struct that stores a context that can be saved and stored
 */
typedef struct
{
	uint32_t r4, r5, r6, r7, r8, r9, r10, r11, sp, lr;
	uint32_t stack[512];
} context;

/**
 * Initializes a context variable
 */
void context_init(context* ctx, void (*entry)(context*));

/**
 * Switches in/out a context
 */
void context_switch(context* ctx);

#endif /* CONTEXT_CONTEXT_H_ */
