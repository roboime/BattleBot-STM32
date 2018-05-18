/*
 * util.h
 *
 *  Created on: 22 de out de 2017
 *      Author: joaobapt
 */

#ifndef UTIL_H_
#define UTIL_H_

#define CFG_INPUT_ANALOG (0UL|0UL)
#define CFG_INPUT_FLOATING (0UL|4UL)
#define CFG_INPUT_PULL_DOWN (0UL|8UL|0UL)
#define CFG_INPUT_PULL_UP (0UL|8UL|16UL)

#define CFG_OUTPUT_GENERAL_PUSH_PULL_10MHZ (1UL|0UL)
#define CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ (1UL|4UL)
#define CFG_OUTPUT_ALTERNATE_PUSH_PULL_10MHZ (1UL|8UL)
#define CFG_OUTPUT_ALTERNATE_OPEN_DRAIN_10MHZ (1UL|12UL)

#define CFG_OUTPUT_GENERAL_PUSH_PULL_2MHZ (2UL|0UL)
#define CFG_OUTPUT_GENERAL_OPEN_DRAIN_2MHZ (2UL|4UL)
#define CFG_OUTPUT_ALTERNATE_PUSH_PULL_2MHZ (2UL|8UL)
#define CFG_OUTPUT_ALTERNATE_OPEN_DRAIN_2MHZ (2UL|12UL)

#define CFG_OUTPUT_GENERAL_PUSH_PULL_50MHZ (3UL|0UL)
#define CFG_OUTPUT_GENERAL_OPEN_DRAIN_50MHZ (3UL|4UL)
#define CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ (3UL|8UL)
#define CFG_OUTPUT_ALTERNATE_OPEN_DRAIN_50MHZ (3UL|12UL)

#define CONFIGURE_GPIO(GPIOx, port, cfg) \
	do { \
		(&(GPIOx)->CRL)[((port)>>3)] &= ~(15UL << (4*((port)&7))); \
		(&(GPIOx)->CRL)[((port)>>3)] |= (((cfg)&15UL) << (4*((port)&7))); \
		if (((cfg)&15UL) == 8UL) \
		{ \
			if ((cfg)&16UL) (GPIOx)->ODR |= 1UL << (port); \
			else (GPIOx)->ODR &= ~(1UL << (port)); \
		} \
	} while (0)

#endif /* UTIL_H_ */
