/*
 * mvmt.h
 *
 *  Created on: 25 de mai de 2018
 *      Author: Girardin
 */

#ifndef MODULES_MVMT_H_
#define MODULES_MVMT_H_

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

void mvmt_init();
void mvmt_control(int motor, int p);

#endif /* MODULES_MVMT_H_ */
