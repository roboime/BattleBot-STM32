/*
 * control.h
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>
#include "vectors.h"

void control_input_motion(int32_vec3_t accel, int32_vec3_t gyro);

void control_input_recv_data(int recv_x, int recv_y);
void control_recv_update();

int32_t control_output_left();
int32_t control_output_right();

#endif /* CONTROL_H_ */
