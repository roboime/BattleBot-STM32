/*
 * esc1.h
 *
 *  Created on: 25 de mai de 2018
 *      Author: Girardin
 */

#ifndef MODULES_ESC1_H_
#define MODULES_ESC1_H_

#define PWM_LOW 2500
#define PWM_NEUTRAL 3000
#define PWM_HIGH 3500

extern void esc1_update();
extern void esc1_control(int p);
extern void esc1_calibration_routine();
extern int esc1_calibration_done();

#endif /* MODULES_ESC1_H_ */
