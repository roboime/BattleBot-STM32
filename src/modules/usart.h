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
 * Returns the number of bytes available to read
 */
uint32_t usart_read_available();

/**
 * Reads from the opposing microcontroller using the USART bus.
 * Returns the number of bytes read
 */
uint32_t usart_read(void* out, uint32_t size);

/**
 * Reads a single byte from the USART bus. Returns full -1 if no byte is available
 */
uint32_t usart_read_byte();

/**
 * Returns the number of bytes that can be written
 */
uint32_t usart_write_possible();

/**
 * Writes to the opposing microcontroller using the USART bus.
 * Returns the number of bytes actually written
 */
uint32_t usart_write(const void* in, uint32_t size);

/**
 * Writes a single byte to the USART bus. Returns 1 if the write was successful
 */
uint8_t usart_write_byte(uint8_t byte);

#define USART_READ_VAR(v) usart_read(&(v), sizeof(v))
#define USART_WRITE_VAR(v) usart_write(&(v), sizeof(v))

#endif /* MODULES_USART_H_ */
