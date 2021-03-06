/*
 * adc.h
 *
 *  Created on: 20 de jun de 2018
 *      Author: Girardin
 */

#ifndef MODULES_ADC_H
#define MODULES_ADC_H

#include <stdint.h>

extern void adc_init();
extern void adc_request_reading();
extern uint32_t adc_get_voltage(uint32_t cell);
extern uint32_t adc_get_temp(uint32_t id);

#endif /* MODULES_ADC_H_ */
