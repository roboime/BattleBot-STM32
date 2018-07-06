/*
 * control.c
 *
 *  Created on: 1 de jul de 2018
 *      Author: Wei
 */

#include "control.h"
#include "math/vectors.h"
#include "math/quaternions.h"
#include "modules/config.h"

static int32_t cur_recv_x, cur_recv_y;
static int32_vec3_t cur_accel, cur_gyro;
static int32_t v_left, v_right;
static int64_t accum_iterm = 0, delta_out = 0;
static int32_t last_wz = 0;

static const int64_t kp = 1LL << 32, ki = 0, kd = 0;

void mahony_sensor_fusion()
{

}

void control_input_recv_data(int recv_x, int recv_y)
{
	cur_recv_x = recv_x;
	cur_recv_y = recv_y;
}

void control_input_motion(int32_vec3_t accel, int32_vec3_t gyro)
{
	cur_accel = accel;
	cur_gyro = gyro;
}


void control_input_enc(int32_t enc_left, int32_t enc_right)
{
	v_right = enc_right;
	v_left = enc_left;
}

void control_recv_update()
{
	if (!config_get_flag(config_flag_disable_gyro_ctl)) return;

	/* velocidade angular */
	int32_t w_setpoint = cur_recv_x;
	int32_t wz = cur_gyro.z;

	/* velocidade linear */
	//v_setpoint = y*v_max/200;

	int64_t error = w_setpoint - wz;

	//proportional term
	int64_t pterm = kp * error;

	//integral term
	accum_iterm += ki * error;

	//derivative term
	int64_t dterm = kd * (wz - last_wz);

	delta_out = pterm + accum_iterm + dterm;

	last_wz = wz;
}

int32_t control_output_left()
{
	return cur_recv_y;
}

int32_t control_output_right()
{
	return cur_recv_y + (delta_out >> 32);
}
