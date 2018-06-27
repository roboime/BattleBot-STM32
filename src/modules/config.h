/*
 * config.h
 *
 *  Created on: 27 de jun de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_CONFIG_H_
#define MODULES_CONFIG_H_

#include <stdint.h>

typedef struct
{
	uint32_t id;

	struct { int32_t in_low, in_mid, in_high; } params[6];
} config_struct;

/**
 * Load configurations from FLASH page 62
 */
void config_load();

/*
 * Access to current configurations
 */
config_struct* config_cur();

/**
 * Save configurations to FLASH page 62 and RESET
 */
void __attribute__((noreturn)) config_save();

#endif /* MODULES_CONFIG_H_ */
