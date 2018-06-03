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
typedef void* context;
typedef context (*context_entry)(context, void*);
#define STACKPTR __attribute__((aligned(8)))

/**
 * Initializes a context variable
 */
context context_new(void* stack_ptr, uint32_t stack_size, context_entry entry, void* ud);

/**
 * Switches in/out a context
 */
context context_switch(context ctx);

#endif /* CONTEXT_CONTEXT_H_ */
