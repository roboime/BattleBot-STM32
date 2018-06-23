/*
 * spi.h
 *
 *  Created on: 24 de mai de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_SPI_H_
#define MODULES_SPI_H_

#include <stdint.h>

/**
 * Initializes the SPI peripheral
 */
void spi_init();

/**
 * Starts the coroutine that will handle the I2C communications.
 * Every SPI communication method MUST happen inside a function
 * that was called with this function
 */
void spi_thread(void(*thread)(void*), void* ud);

/**
 * Reads a chunk of data within the SPI bus from the device of address addr
 * MUST be called inside an spi_thread. Returns 1 on successful and 0 on error
 */
uint8_t spi_read(void* out, uint32_t size);

/**
 * Writes a chunk of data within the SPI bus to the device of address addr
 * MUST be called inside an spi_thread. Returns 1 on successful and 0 on error
 */
uint8_t spi_write(const void* in, uint32_t size);

/**
 * Selects or deselects the SPI slave. There is only one slave, so write 0 to deselect everyone
 * and 1 to select the first slave.
 */
void spi_select(uint32_t slave);

/**
 * Switches the SPI to the normal frequency (562.5 kHz, 0) or fast frequency (18 MHz, 1). The fast
 * frequency should only be used to read data registers from the MPU (too specific, but whatever)
 */
void spi_set_speed(uint32_t spd);

#define SPI_READ_VAR(addr, v) spi_read((addr), &(v), sizeof(v))
#define SPI_WRITE_VAR(addr, v) spi_write((addr), &(v), sizeof(v))

#endif /* MODULES_SPI_H_ */
