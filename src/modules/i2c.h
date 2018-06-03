/*
 * i2c.h
 *
 *  Created on: 24 de mai de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_I2C_H_
#define MODULES_I2C_H_

#include <stdint.h>

/**
 * Initializes the I2C peripheral
 */
void i2c_init();

/**
 * Starts the coroutine that will handle the I2C communications.
 * Every I2C communication method MUST happen inside a function
 * that was called with this function
 */
void i2c_thread(void(*thread)(void*), void* ud);

/**
 * Reads a chunk of data within the I2C bus from the device of address addr
 * MUST be called inside an i2c_thread. Returns 1 on successful and 0 on error
 */
uint8_t i2c_read(uint8_t addr, void* out, uint32_t size);

/**
 * Writes a chunk of data within the I2C bus to the device of address addr
 * MUST be called inside an i2c_thread. Returns 1 on successful and 0 on error
 */
uint8_t i2c_write(uint8_t addr, const void* in, uint32_t size);

#define I2C_READ_VAR(addr, v) i2c_read((addr), &(v), sizeof(v))
#define I2C_WRITE_VAR(addr, v) i2c_write((addr), &(v), sizeof(v))

#endif /* MODULES_I2C_H_ */
