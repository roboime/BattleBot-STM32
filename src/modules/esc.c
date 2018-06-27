/*
 * esc.c
 *
 *  Created on: 21 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"
#include "util.h"

#include "esc.h"
#include "esc1.h"

void esc_init(uint32_t esc)
{
	/* Configure the GPIO pins */
	CONFIGURE_GPIO(GPIOB, 8, CFG_OUTPUT_GENERAL_PUSH_PULL_2MHZ);
	CONFIGURE_GPIO(GPIOB, 9, CFG_OUTPUT_GENERAL_PUSH_PULL_2MHZ);
	CONFIGURE_GPIO(GPIOB, 6, CFG_OUTPUT_ALTERNATE_PUSH_PULL_10MHZ);

	/* Pre-disable timer 4 */
	TIM4->CR1 &= ~TIM_CR1_CEN;

	/* Configure PWM for channel 1 */
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(7 << 4)) | (6 << 4);

	/* Enable channel 1 */
	TIM4->CCER |= TIM_CCER_CC1E;

	/* Re-enable timer 4 */
	TIM4->CR1 |= TIM_CR1_CEN;
}

/* Pass the functions to the submodules */
void esc_update()
{
	esc1_update();
}

void esc_control(int32_t p)
{
	esc1_control(p);
}

void esc_calibration_routine()
{
	esc1_calibration_routine();
}

int32_t esc_calibration_done()
{
	return esc1_calibration_done();
}

int32_t esc_raw_output()
{
	return TIM4->CCR1;
}
