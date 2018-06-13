/*
 * usart.h
 *
 *  Created on: 1 de jun de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_USART_H_
#define MODULES_USART_H_

#include <stdint.h>

/**
 * Initializes the USART peripheral
 */
void usart_init();

/**
 * Starts the coroutine that will handle the USART communications.
 * Every USART communication method MUST happen inside a function
 * that was called with this function
 */
void usart_thread(void(*thread)(void*), void* ud);

/**
 * Reads from the opposing microcontroller using the USART bus.
 * MUST be called inside an usart_thread.
 */
uint8_t usart_read(void* out, uint32_t size);

/**
 * Writes to the opposing microcontroller using the USART bus.
 * MUST be called inside an usart_thread.
 */
uint8_t usart_write(const void* in, uint32_t size);

#define USART_READ_VAR(v) usart_read(&(v), sizeof(v))
#define USART_WRITE_VAR(v) usart_write(&(v), sizeof(v))

#endif /* MODULES_USART_H_ */
