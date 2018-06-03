/*
 * mvmt.c
 *
 *  Created on: 25 de mai de 2018
 *      Author: Girardin
 */

#include "stm32f10x.h"
#include "util.h"

#include "mvmt.h"

void mvmt_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	CONFIGURE_GPIO(GPIOA, 8, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);
	CONFIGURE_GPIO(GPIOA, 9, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);
	CONFIGURE_GPIO(GPIOA, 10, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);
	CONFIGURE_GPIO(GPIOA, 11, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);

	TIM1->PSC =  17;
	TIM1->ARR = 199;
	TIM1->EGR = TIM_EGR_UG;

	TIM1->CCMR1 = (6 << 4) | (6<<12);
	TIM1->CCMR2 = (6 << 4) | (6<<12);

	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM1->BDTR  |= TIM_BDTR_MOE;

	TIM1->CR1 |= TIM_CR1_CEN;

}

void mvmt_control(int motor, int p)
{
    __IO uint16_t *cha = motor == MOTOR_LEFT ? &TIM1->CCR1 : &TIM1->CCR3;
    __IO uint16_t *chb = motor == MOTOR_LEFT ? &TIM1->CCR2 : &TIM1->CCR4;

    if(p>0)
    {
        *cha = p;
        *chb = 0;
    }
    else if (p<0)
    {
        *cha = 0;
        *chb = -p;
    }
	else{
        *cha = 0;
        *chb = 0;
    }
}
