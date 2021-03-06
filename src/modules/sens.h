/*
 * sens.h
 *
 *  Created on: 24 de jun de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_SENS_H_
#define MODULES_SENS_H_

#include <stdint.h>
#include "math/vectors.h"

/**
 * Initializes the sensors module
 */
void sens_init();

/**
 * Returns 1 if the sensors are ready to be used
 */
uint8_t sens_ready();

/**
 * Begins sensors calibration routine
 */
void sens_calibration_routine();

/**
 * Returns 1 if calibration is done (or not started)
 */
uint8_t sens_calibration_done();

/**
 * Collect the last data on the sensors and average
 */
void sens_collect_data();

/**
 * Returns 1 if collection is in way
 */
uint8_t sens_collecting();

/**
 * Returns acceleration
 */
int32_vec3_t sens_get_accel();
int32_vec3_t sens_get_adjusted_accel();

/**
 * Returns angular velocity
 */
int32_vec3_t sens_get_angular();

/**
 * Returns magnetic field
 */
int32_vec3_t sens_get_mag();

#endif /* MODULES_SENS_H_ */
