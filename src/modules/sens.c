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

#define DISABLE_INA // Sorry, not for now

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
	spi_wait_us(4);
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
	spi_wait_us(4);
}

/* Write magnetometer register */
inline static void write_mag(uint8_t reg, uint8_t data)
{
	/* Put the data into the I2C data out register */
	/* I2C_SLV0_D0 = data */
	WRITE_REGISTERS(MPU9250_I2C_SLV0_DO, data);

	/* Ask for the I2C master to write a byte to the mag */
	/* I2C_SLV0_ADDR = 0x0C; */
	/* I2C_SLV0_REG = reg */
	/* I2C_SLV0_CTRL = I2C_SLV0_EN | 1 byte */
	WRITE_REGISTERS(MPU9250_I2C_SLV0_ADDR, 0x0C, reg, 0x81);

	/* Delay for 1.5 ms for the write to take effect */
	spi_wait_us(880);
}

/* Read from mag registers */
inline static void read_mag(uint8_t reg, void* out, uint32_t size)
{
	/* Ask for the I2C master to read a byte to the mag */
	/* I2C_SLV0_ADDR = 0x0C, MSB set to read; */
	/* I2C_SLV0_REG = reg */
	/* I2C_SLV0_CTRL = I2C_SLV0_EN | "size" bytes */
	WRITE_REGISTERS(MPU9250_I2C_SLV0_ADDR, 0x8C, reg, 0x80 | size);

	/* Delay for enough time for the I2C transaction to complete */
	/* 895 us */
	spi_wait_us(940);

	/* Read the data */
	read_regs(MPU9250_EXT_SENS_DATA_00, out, size);
}

/* Important data */
#define READ 0x80

/* Init and calibration state */
static volatile uint32_t init_state = 0, calibration_state = 0, collect_state = 0;

/* FIFO Data collection */
typedef struct
{
	int16_t accel_x, accel_y, accel_z;
	int16_t gyro_x, gyro_y, gyro_z;
	int16_t mag_x, mag_y, mag_z, dummy;
#ifndef DISABLE_INA
	int16_t ina_l, ina_r;
#endif
} fifo_data;

/* Fifo area to collect */
static fifo_data fifo[512/sizeof(fifo_data)];

/* Current state */
static volatile int32_vec3_t cur_accel;
static volatile int32_vec3_t cur_gyro;
static volatile int32_vec3_t cur_mag;
#ifndef DISABLE_INA
static volatile int32_t cur_ina_l, cur_ina_r;
#endif

/* Magnetometer calibration data */
static int32_t mag_cal[3];

/* Define accelerometer scale range */
#define ACCEL_FS_SEL MPU9250_ACCEL_FS_SEL_16G
#define GYRO_FS_SEL MPU9250_GYRO_FS_SEL_2000DPS

/* The real init thread */
static void sens_init_thread(void* ud)
{
	/* Low speed to configuration registers */
	spi_set_speed(SPI_SPEED_LOW);

	/* Reset routine */
	/* Set clock source to best one */
	WRITE_REGISTERS(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL);
	/* Enable I2C master mode */
	WRITE_REGISTERS(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN);
	/* Set I2C speed to 400 kHz */
	WRITE_REGISTERS(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK);
	/* Power down the magnetometer */
	write_mag(AK8963_CNTL1, AK8963_PWR_DOWN);
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
	WRITE_REGISTERS(MPU9250_SMPDIV, 0,
		MPU9250_GYRO_DLPF_92 | 0x40, GYRO_FS_SEL, ACCEL_FS_SEL, MPU9250_ACCEL_DLPF_92);

	/* Magnetometer initialization routine */
	/* Enable I2C master mode */
	WRITE_REGISTERS(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN);
	/* Set I2C speed to 400 kHz */
	WRITE_REGISTERS(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK);
	/* Reset the magnetometer */
	write_mag(AK8963_CNTL2,AK8963_RESET);
	/* Wait a bit more */
	spi_wait_ms(1);
	/* Power down mag */
	write_mag(AK8963_CNTL1, AK8963_PWR_DOWN);
	/* Enable access to FUSE ROM (to read calibration values) */
	write_mag(AK8963_CNTL1, AK8963_FUSE_ROM);

	/* Read sensitivity data */
	{
		uint8_t buffer[3];
		read_mag(AK8963_ASA, buffer, 3);
		for (int i = 0; i < 3; i++) mag_cal[i] = ((int32_t)buffer[i] - 128)/2 - 128;
	}

	/* Power down mag */
	write_mag(AK8963_CNTL1, AK8963_PWR_DOWN);
	/* Continuous measurement at 100 Hz */
	write_mag(AK8963_CNTL1, AK8963_CNT_MEAS2);

	/* Command the MPU to read 7 bytes from the mag each sample and two bytes from each INA */
	/* SLV0: MPU, from register HXL, 8 bytes */
	/* SLV1: left INA, register 1, 2 bytes */
	/* SLV2: right INA, register 1, 2 bytes */
	WRITE_REGISTERS(MPU9250_I2C_SLV0_ADDR, 0x80 | 0x0C, AK8963_HXL, 0x88
#ifndef DISABLE_INA
		, 0x80 | 0x40, 1, 0x82, 0x80 | 0x41, 1, 0x82
#endif
	);

	/* Enable FIFO on USER_CONTROL */
	WRITE_REGISTERS(MPU9250_USER_CTRL, 0x40 | MPU9250_I2C_MST_EN);
	/* Put on FIFO accelerometer, gyroscope and all slaves = total 24 bytes */
#ifndef DISABLE_INA
	WRITE_REGISTERS(MPU9250_FIFO_EN, 0x7F);
#else
	/* Put on FIFO accelerometer, gyroscope and just the first slave = total 20 bytes */
	WRITE_REGISTERS(MPU9250_FIFO_EN, 0x79);
#endif

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
	/* Read the FIFO size */
	int16_t fifo_size;
	read_regs(MPU9250_FIFO_COUNT, &fifo_size, sizeof(fifo_size));
	// Invert it
	fifo_size = __REVSH(fifo_size);
	int32_t num_elements = fifo_size/sizeof(fifo_data);
	fifo_size = sizeof(fifo_data) * num_elements;

	/* Switch to high speed */
	spi_set_speed(SPI_SPEED_HIGH);
	/* Read the FIFO */
	read_regs(MPU9250_FIFO_READ, fifo, fifo_size);
	/* Switch back to low speed */
	spi_set_speed(SPI_SPEED_LOW);

	cur_accel.x = 0, cur_accel.y = 0, cur_accel.z = 0;
	cur_gyro.x = 0, cur_gyro.y = 0, cur_gyro.z = 0;
	cur_mag.x = 0, cur_mag.y = 0, cur_mag.z = 0;
#ifndef DISABLE_INA
	cur_ina_l = 0, cur_ina_r = 0;
#endif

	for (uint32_t i = 0; i < num_elements; i++)
	{
		/* Invert accel/gyro, DON'T invert mag, invert INA */
		cur_accel.x += __REVSH(fifo[i].accel_x);
		cur_accel.y += __REVSH(fifo[i].accel_y);
		cur_accel.z += __REVSH(fifo[i].accel_z);
		cur_gyro.x += __REVSH(fifo[i].gyro_x);
		cur_gyro.y += __REVSH(fifo[i].gyro_y);
		cur_gyro.z += __REVSH(fifo[i].gyro_z);
		cur_mag.x += fifo[i].mag_y;
		cur_mag.y += fifo[i].mag_x;
		cur_mag.z -= fifo[i].mag_z;
#ifndef DISABLE_INA
		cur_ina_l += __REVSH(fifo[i].ina_l);
		cur_ina_r += __REVSH(fifo[i].ina_r);
#endif
	}

	cur_accel.x = (cur_accel.x + num_elements/2) / num_elements;
	cur_accel.y = (cur_accel.y + num_elements/2) / num_elements;
	cur_accel.z = (cur_accel.z + num_elements/2) / num_elements;
	cur_gyro.x = (cur_gyro.x + num_elements/2) / num_elements;
	cur_gyro.y = (cur_gyro.y + num_elements/2) / num_elements;
	cur_gyro.z = (cur_gyro.z + num_elements/2) / num_elements;
	cur_mag.x = (cur_mag.x + num_elements/2) / num_elements;
	cur_mag.y = (cur_mag.y + num_elements/2) / num_elements;
	cur_mag.z = (cur_mag.z + num_elements/2) / num_elements;

	cur_mag.x = cur_mag.x * mag_cal[0] / 128;
	cur_mag.y = cur_mag.y * mag_cal[1] / 128;
	cur_mag.z = cur_mag.z * mag_cal[2] / 128;

#ifndef DISABLE_INA
	cur_ina_l = (cur_ina_l + num_elements/2) / num_elements;
	cur_ina_r = (cur_ina_r + num_elements/2) / num_elements;
#endif

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
	spi_trigger_wait_us(19200);

	sens_collect_data_thread(ud);

	/* To hold the calibration measurement */
	int32_vec3_t avg_accel = { 0, 0, 0 };
	int32_vec3_t avg_gyro = { 0, 0, 0 };

	spi_wait_on();

	/* Take 50 measures (1 second) to calibrate */
	for (int i = 0; i < 50; i++)
	{
		/* Trigger the SPI timer here, we don't want to wait a lot */
		spi_trigger_wait_us(19200);

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
	avg_accel.x = (avg_accel.x + 25) / 50;
	avg_accel.y = (avg_accel.y + 25) / 50;
	avg_accel.z = (avg_accel.z + 25) / 50;
	avg_gyro.x = (avg_gyro.x + 25) / 50;
	avg_gyro.y = (avg_gyro.y + 25) / 50;
	avg_gyro.z = (avg_gyro.z + 25) / 50;

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
	__disable_irq();
	int32_vec3_t val = cur_accel;
	__enable_irq();
	return val;
}

/* Gyroscope */
int32_vec3_t sens_get_angular()
{
	__disable_irq();
	int32_vec3_t val = cur_gyro;
	__enable_irq();
	return val;
}

/* Mag */
int32_vec3_t sens_get_mag()
{
	__disable_irq();
	int32_vec3_t val = cur_mag;
	__enable_irq();
	return val;
}

int32_t sens_get_current(uint32_t motor)
{
#ifndef DISABLE_INA
	__disable_irq();
	int32_t val = motor ? cur_ina_r : cur_ina_l;
	__enable_irq();
	return val;
#endif
	return 0;
}
