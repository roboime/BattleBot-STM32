/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 8.0.0   2017-11-04

The MIT License (MIT)
Copyright (c) 2009-2017 Atollic AB
Modifications on the project
Copyright (c) 2018 João Baptista de Paula e Silva

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include <stddef.h>
#include <stdio.h>
#include "stm32f10x.h"

#include "util.h"
#include "modules/recv.h"
#include "modules/mvmt.h"
#include "modules/esc.h"
#include "modules/iwdg.h"
#include "modules/usart.h"
#include "modules/esp.h"
#include "modules/enc.h"
#include "modules/adc.h"
#include "modules/spi.h"
#include "modules/sens.h"
#include "modules/config.h"

#include "context/context.h"

const normalization_params
	vert_params = { 2049, 3009, 4032, 195, 0, -195 },
	hor_params = { 2191, 3016, 3768, -195, 0, 195 },
	wpn_params = { 2250, 3048, 3830, 0, 256, 512 };

static void send_collected_data();
static void poll_esp_commands();

static int32_vec3_t accum_accel;
static int32_vec3_t accum_adjusted_accel;
static int32_vec3_t accum_gyro;
static int32_vec3_t accum_mag;
static uint32_t num_samples;

int main(void)
{
	// Enable all GPIO ports so the other functions don't need to
	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
				   RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	CONFIGURE_GPIO(GPIOC, 13, CFG_OUTPUT_GENERAL_PUSH_PULL_10MHZ);

	// Load config file
	config_load();

	// Initialize all primary modules
	recv_init();
	mvmt_init();
	esc_init(0);

	// Initialize the ESP-8266 module
	//usart_init();
	//esp_init();

	// Initialize the sensors module
	spi_init();
	sens_init();

	// Calibrate if not calibrated
	if (!config_get_flag(config_flag_mpu_cal_done))
	{
		while (!sens_ready());
		sens_calibration_routine();
	}

	// Initialize the ADC and encoders
	enc_init();
	adc_init();

	// Initialize the independent watchdog
	iwdg_init();

	// Pass the normalization parameters
	recv_set_normalization_params(1, &wpn_params);
	recv_set_normalization_params(2, &vert_params);
	recv_set_normalization_params(3, &hor_params);

	// Initialize the accumulated data
	accum_accel.x = accum_accel.y = accum_accel.z = 0;
	accum_gyro.x = accum_gyro.y = accum_gyro.z = 0;
	accum_mag.x = accum_mag.y = accum_mag.z = 0;
	num_samples = 0;

	// Enable reception of signal data
	recv_enable();

	int i = 0;

	while (1)
	{
		if (recv_new_frame())
		{
			iwdg_reset();
			adc_request_reading();

			recv_update();
			esc_update();
			enc_update();

			int y = recv_channel(2);
			if (config_get_flag(config_flag_rev_in_y)) y = -y;
			int x = recv_channel(3);
			if (config_get_flag(config_flag_rev_in_x)) x = -x;

			int w = recv_channel(1);
			if (config_get_flag(config_flag_rev_esc)) w = -w;

			control_input_recv_data(x, y);
			esc_control(w);

			i++;
			if (i % 8 == 0) GPIOC->ODR ^= GPIO_ODR_ODR13;

			//send_collected_data();
			//poll_esp_commands();
		}
		//__WFI();

		// 1 kHz control feedback loop
		if (mvmt_require_ctl())
		{
			// Collect sensor data
			if (sens_ready()) sens_collect_data();
			while (sens_collecting());

			int32_vec3_t accel = sens_get_accel();
			int32_vec3_t adjusted_accel = sens_get_adjusted_accel();
			int32_vec3_t gyro = sens_get_angular();
			int32_vec3_t real_accel = config_get_flag(config_flag_use_unadjusted_accel) ? accel : adjusted_accel;

			// Accumulate sensor data for display
			accum_accel = vec_add32(accum_accel, accel);
			accum_adjusted_accel = vec_add32(accum_adjusted_accel, adjusted_accel);
			accum_gyro = vec_add32(accum_gyro, gyro);

			// Input data for the control loop
			control_input_motion(real_accel, gyro);
			control_recv_update();

			// Set the output data
			int32_t left = control_output_left();
			if (config_get_flag(config_flag_rev_left_motor)) left = -left;
			int32_t right = control_output_right();
			if (config_get_flag(config_flag_rev_right_motor)) right = -right;

			mvmt_control(MOTOR_LEFT, left);
			mvmt_control(MOTOR_RIGHT, right);

			num_samples++;
		}
	}
}

#if 0
static void poll_esp_commands()
{
	esp_recv_commands();
	while (esp_new_commands())
	{
		uint8_t buffer[160];
		uint32_t size = esp_next_command(buffer);

		/* Empty commands are ignored */
		if (size == 0) continue;
		uint8_t cmd = buffer[0];

		/* Process depending on the command size */
		if (size == 1)
		{
			switch (cmd)
			{
				/* Reset command */
				case 0x00: NVIC_SystemReset(); break;

				/* Calibrate command */
				case 0x01: sens_calibration_routine(); break;

				/* Save settings to FLASH */
				case 0x02: config_save(); break;

				/* Next frame send the config instead of telemetry data */
				case 0x0C + config_num_flags: break;
			}
		}
		else if (size == 5)
		{
			int32_t x;
			memcpy(&x, buffer+1, sizeof(int32_t));
			config_struct* cfg = config_cur();

			switch (cmd)
			{
				case 0x03: cfg->gyro_kp = x;  break;
				case 0x04: cfg->gyro_ki = x;  break;
				case 0x05: cfg->gyro_kd = x;  break;
				case 0x06: cfg->vel_kp = x;  break;
				case 0x07: cfg->vel_ki = x;  break;
				case 0x08: cfg->vel_kd = x;  break;
				case 0x09: cfg->sf_kp = x; break;
				case 0x0A: cfg->sf_ki = x; break;
				default: break;
			}
		}
		else if (size == 13)
		{
			config_struct* cfg = config_cur();

			switch (cmd)
			{
				case 0x0B: memcpy(&cfg->rpos, buffer+1, 3*sizeof(int32_t)); break;
			}
		}
		else if (size == 2)
		{
			// Set flags
			if (cmd >= 0x0C && cmd < 0x0C + config_num_flags)
				config_set_flag(cmd - 0x0C, buffer[1] != 0);
		}
		else if (size == 8)
		{
			// Easter-egg: joystick control
		}
	}
}

static void send_collected_data()
{
	int32_vec3_t accel = accum_accel;
	int32_vec3_t adjusted_accel = accum_adjusted_accel;
	int32_vec3_t gyro = accum_gyro;

	if (num_samples > 0)
	{
		accel = vec_div32(accel, num_samples);
		adjusted_accel = vec_div32(adjusted_accel, num_samples);
		gyro = vec_div32(gyro, num_samples);
	}

	int16_t flags = sens_ready();

	int16_t data[] =
	{
		// Voltage on each cell
		adc_get_voltage(1),        // 0: voltage 1
		adc_get_voltage(2),        // 2: voltage 2
		adc_get_voltage(3),        // 4: voltage 3
		adc_get_voltage(4),        // 6: voltage 4
		adc_get_voltage(5),        // 8: voltage 5

		// Temperature on each motor
		/*adc_get_temp(0)*/0,           // 10: left temp
		/*adc_get_temp(1)*/0,           // 12: right temp
		/*adc_get_temp(2)*/0,           // 14: weapon temp

		// Current on each motor
		/*adc_get_current(0)*/0,        // 16: left current
		/*adc_get_current(1)*/0,        // 18: right current

		// MPU data
		accel.x, accel.x, accel.z,      // 20, 22, 24: accelerometer
		adjusted_accel.x, adjusted_accel.y, adjusted_accel.z, // 26, 28, 30: adjusted accelerometer
		gyro.x, gyro.y, gyro.z,         // 32, 34, 36: gyroscope

		// Encoder speed output
		enc_speed(MOTOR_LEFT),          // 38: left speed by encoder
		enc_speed(MOTOR_RIGHT),         // 40: right speed by encoder

		// Receiver raw data
		recv_raw_channel(0),            // 42: receiver data channel 1
		recv_raw_channel(1),            // 44: receiver data channel 2
		recv_raw_channel(2),            // 46: receiver data channel 3
		recv_raw_channel(3),            // 48: receiver data channel 4
		recv_raw_channel(4),            // 50: receiver data channel 5
		recv_raw_channel(5),            // 52: receiver data channel 6

		// ESC raw output
		esc_raw_output(),               // 54: ESC raw output

		// Flags
		flags                           // 56: flags
	};

	// Send this data through the ESP
	esp_send(data, sizeof(data));

	// Reset accumulated IMU data
	accum_accel.x = accum_accel.y = accum_accel.z = 0;
	accum_adjusted_accel.x = accum_adjusted_accel.y = accum_adjusted_accel.z = 0;
	accum_gyro.x = accum_gyro.y = accum_gyro.z = 0;
	accum_mag.x = accum_mag.y = accum_mag.z = 0;
	num_samples = 0;
}
#endif

void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}
