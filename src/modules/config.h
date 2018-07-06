/*
 * config.h
 *
 *  Created on: 27 de jun de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_CONFIG_H_
#define MODULES_CONFIG_H_

#include <stdint.h>
#include "math/vectors.h"

typedef enum
{
	config_flag_rev_in_x,
	config_flag_rev_in_y,
	config_flag_rev_left_motor,
	config_flag_rev_right_motor,
	config_flag_rev_left_enc,
	config_flag_rev_right_enc,
	config_flag_rev_esc,

	config_flag_disable_esc,
	config_flag_disable_gyro_ctl,
	config_flag_disable_accel_ctl,
	config_flag_disable_enc_ctl,
	config_flag_use_unadjusted_accel,

	config_num_flags,
	config_flag_mpu_cal_done = config_num_flags,
} config_flag;

typedef struct
{
	uint32_t id;

	int32_vec3_t rpos;

	int32_t gyro_kp, gyro_ki, gyro_kd;
	int32_t vel_kp, vel_ki, vel_kd;
	int32_t sf_kp, sf_ki;

	uint16_t flags;

	int16_t acc_cal_x, acc_cal_y, acc_cal_z;
	int16_t gyro_cal_x, gyro_cal_y, gyro_cal_z;
} config_struct;

/**
 * Load configurations from FLASH page 62
 */
void config_load();

/*
 * Access to current configurations
 */
config_struct* config_cur();

/*
 * Quick function to toggle or set a flag
 */
void config_set_flag(config_flag flag, int32_t val);

/*
 * Quick function to get a flag
 */
int32_t config_get_flag(config_flag flag);

/**
 * Save configurations to FLASH page 62 and RESET
 */
void __attribute__((noreturn)) config_save();

#endif /* MODULES_CONFIG_H_ */
