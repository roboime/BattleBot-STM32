/*
 * sens.c
 *
 *  Created on: 24 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"
#include "sens.h"
#include "sens-regs.h"
#include "spi.h"

#include <stdbool.h>

// How SPI communications with the MPU-9250 work
// - for writing: you select it (by driving SPI select low), write the register number FIRST and the byte data
//   SECOND, then deselect it (by driving select high); sending more bytes will set the following registers
// - for reading: you select it, write the register number, then READ the data bytes you want to read. With an
//   exception (the FIFO), reading more bytes will read the following registers

// Configuration registers MUST be accessed at low speed, ONLY The FIFO and data registers may be READ at high speed

/* Write registers normally */
inline static void write_regs(const void* data, uint32_t size)
{
	spi_select(1);
	spi_write(data, size);
	spi_select(0);
	spi_wait_us(1);
}

#define WRITE_REGISTERS(...) do { \
	const uint8_t tmp[] = { __VA_ARGS__ }; \
	write_regs(tmp, sizeof(tmp)); \
} while (0)

/* Read registers normally */
inline static void read_regs(uint8_t reg, void* data, uint32_t size)
{
	/* MSB must be set to read */
	reg |= 0x80;
	spi_select(1);
	spi_write(&reg, sizeof(reg));
	spi_read(data, size);
	spi_select(0);
	spi_wait_us(1);
}

/* Important data */
#define READ 0x80

/* Init and calibration state */
static volatile uint32_t init_state = 0, calibration_state = 0, collect_state = 0;

/* FIFO Data collection */
typedef struct
{
	int16_t accel_x, accel_y, accel_z;
	int16_t temp;
	int16_t gyro_x, gyro_y, gyro_z;
} sens_data;

/* Current state */
static volatile int32_vec3_t cur_accel;
static volatile int32_vec3_t cur_gyro;
static volatile int32_vec3_t prev_gyro;

/* Define accelerometer scale range */
#define ACCEL_FS_SEL MPU9250_ACCEL_FS_SEL_16G
#define GYRO_FS_SEL MPU9250_GYRO_FS_SEL_2000DPS

/* The real init thread */
static void sens_init_thread(void* ud)
{
	/* Low speed to configuration registers */
	spi_set_speed(SPI_SPEED_LOW);

	/* Reset the MPU-9250 */
	WRITE_REGISTERS(MPU9250_PWR_MGMNT_1, MPU9250_PWR_RESET);
	/* Delay for a millisecond to wait for the MPU and everything to reset */
	spi_wait_ms(1);

	/* Initial configuration routine */
	/* Set clock source to best one, and enable everything */
	/* Changes in that order: MPU9250_PWR_MGMNT_1, MPU9250_PWR_MGMNT_2 */
	WRITE_REGISTERS(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL, MPU9250_SEN_ENABLE);

	/* Set maximum accelerometer range, maximum gyroscope range */
	/* Configure DLPFs for accelerometer and gyroscope (bandwidth of 184 Hz)
 	   (please change me if measurement is bad) */
	/* Set sample rate divider to 0 */
	/* Changes in that order:
	   MPU9250_SMPDIV, MPU9250_CONFIG, MPU9250_GYRO_CONFIG, MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_CONFIG2 */
	WRITE_REGISTERS(MPU9250_SMPDIV, 0, MPU9250_GYRO_DLPF_92 | 0x40,
		GYRO_FS_SEL, ACCEL_FS_SEL, MPU9250_ACCEL_DLPF_92);

	/* Initialization done */
	init_state = 2;
}

/* Init function */
void sens_init()
{
	/* Opens up a SPI thread to initialize the MPU and the INAs */
	init_state = 1;
	spi_thread(sens_init_thread, 0);
}

/* Check if the sensors are ready */
uint8_t sens_ready()
{
	return init_state == 2 && calibration_state == 0;
}

static void sens_collect_data_thread(void* ud)
{
	prev_gyro = cur_gyro;

	sens_data cur_data;

	/* Switch to high speed */
	spi_set_speed(SPI_SPEED_HIGH);
	/* Read all the MPU registers */
	read_regs(MPU9250_ACCEL_OUT, &cur_data, sizeof(cur_data));
	/* Switch back to low speed */
	spi_set_speed(SPI_SPEED_LOW);

	cur_accel.x = __REVSH(cur_data.accel_x);
	cur_accel.y = __REVSH(cur_data.accel_y);
	cur_accel.z = __REVSH(cur_data.accel_z);

	cur_gyro.x = __REVSH(cur_data.gyro_x);
	cur_gyro.y = __REVSH(cur_data.gyro_y);
	cur_gyro.z = __REVSH(cur_data.gyro_z);

	collect_state = 0;
}

/* Collect data */
void sens_collect_data()
{
	if (!sens_ready() || collect_state == 1) return;

	collect_state = 1;
	spi_thread(sens_collect_data_thread, 0);
}

// Collecting data
uint8_t sens_collecting()
{
	return collect_state;
}

/* Calibration */
static void sens_calibration_thread(void* ud)
{
	/* Discard a first measurement */
	spi_trigger_wait_us(860);

	sens_collect_data_thread(ud);

	/* To hold the calibration measurement */
	int32_vec3_t avg_accel = { 0, 0, 0 };
	int32_vec3_t avg_gyro = { 0, 0, 0 };

	spi_wait_on();

	/* Take 1000 measures (1 second) to calibrate */
	for (int i = 0; i < 1000; i++)
	{
		/* Trigger the SPI timer here, we don't want to wait a lot */
		spi_trigger_wait_us(860);

		/* Read the samples */
		sens_collect_data_thread(ud);

		/* Add them to the bias */
		avg_accel.x += cur_accel.x;
		avg_accel.y += cur_accel.y;
		avg_accel.z += cur_accel.z;
		avg_gyro.x += cur_gyro.x;
		avg_gyro.y += cur_gyro.y;
		avg_gyro.z += cur_gyro.z;

		spi_wait_on();
	}

	/* Get the average */
	avg_accel.x = (avg_accel.x + 500) / 1000;
	avg_accel.y = (avg_accel.y + 500) / 1000;
	avg_accel.z = (avg_accel.z + 500) / 1000;
	avg_gyro.x = (avg_gyro.x + 500) / 1000;
	avg_gyro.y = (avg_gyro.y + 500) / 1000;
	avg_gyro.z = (avg_gyro.z + 500) / 1000;

	/* Remove gravity from accelerometer (in 16g mode, 1g = 4096 units) */
#define ACCEL_GRAVITY_UNIT (int16_t)(16384 >> (ACCEL_FS_SEL >> 3))
	if (avg_accel.z > 0) avg_accel.z -= ACCEL_GRAVITY_UNIT;
	else avg_accel.z += ACCEL_GRAVITY_UNIT;

	/* Build gyroscope bias values (you need to multiply by 2^FS_SEL/4) */
	int16_t gyro_bias[4];
	gyro_bias[0] = 0x1300;
	gyro_bias[1] = __REVSH((int16_t)((-avg_gyro.x << (GYRO_FS_SEL >> 3)) >> 2));
	gyro_bias[2] = __REVSH((int16_t)((-avg_gyro.y << (GYRO_FS_SEL >> 3)) >> 2));
	gyro_bias[3] = __REVSH((int16_t)((-avg_gyro.z << (GYRO_FS_SEL >> 3)) >> 2));

	/* Write the gyroscope bias values */
	write_regs((const uint8_t*)gyro_bias+1, sizeof(gyro_bias)-1);

	/* Read the original accelerometer bias values */
	int32_t accel_bias[3];
	for (int i = 0; i < 3; i++)
	{
		int16_t orig_accel_bias;
		read_regs(0x77 + 3*i, &orig_accel_bias, sizeof(int16_t));
		accel_bias[i] = __REVSH(orig_accel_bias);
	}

	/* Update the accelerometer bias */
	accel_bias[0] -= ((avg_accel.x << (ACCEL_FS_SEL >> 3)) >> 2) & ~1;
	accel_bias[1] -= ((avg_accel.y << (ACCEL_FS_SEL >> 3)) >> 2) & ~1;
	accel_bias[2] -= ((avg_accel.z << (ACCEL_FS_SEL >> 3)) >> 2) & ~1;

	/* Write back the accelerometer values */
	for (int i = 0; i < 3; i++)
	{
		int16_t writeback_accel_bias[2];
		writeback_accel_bias[0] = 0x7700 + 0x300*i;
		writeback_accel_bias[1] = __REVSH((int16_t)accel_bias[i]);
		write_regs((const uint8_t*)writeback_accel_bias + 1, sizeof(writeback_accel_bias)-1);
	}

	/* Done calibration */
	calibration_state = 0;
}

void sens_calibration_routine()
{
	if (!sens_ready()) return;

	calibration_state = 1;
	spi_thread(sens_calibration_thread, 0);
}

uint8_t sens_calibration_done()
{
	return !calibration_state;
}

/* Accelerometer */
int32_vec3_t sens_get_accel()
{
	return cur_accel;
}

/* Gyroscope */
int32_vec3_t sens_get_angular()
{
	return cur_gyro;
}

/* Mag */
int32_vec3_t sens_get_mag()
{
	return (int32_vec3_t){ 0, 0, 0 };
}
