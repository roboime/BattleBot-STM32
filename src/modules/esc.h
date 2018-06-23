/*
 * esc.h
 *
 *  Created on: 21 de jun de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_ESC_H_
#define MODULES_ESC_H_

#include <stdint.h>

void esc_init(uint32_t esc);

void esc_update();

void esc_control(int32_t p);

void esc_calibration_routine();

int32_t esc_calibration_done();

#endif /* MODULES_ESC_H_ */
