/*
 * encoder.c
 *
 *  Created on: 1 de jul de 2018
 *      Author: Wei
 */

#include "stm32f10x.h"
#include "enc.h"
#include "util.h"

const uint32_t Ke = 1;
static int32_t velocity_left, velocity_right;

void enc_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN; //liga os clocks dos timers 2 e 3

	CONFIGURE_GPIO(GPIOA, 15, CFG_INPUT_FLOATING);
	CONFIGURE_GPIO(GPIOB,  3, CFG_INPUT_FLOATING);
	CONFIGURE_GPIO(GPIOB,  4, CFG_INPUT_FLOATING);
	CONFIGURE_GPIO(GPIOB,  5, CFG_INPUT_FLOATING);

	//TIMER 2
	AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1; //remap ch1 para PA15 e ch2 para PB3
	TIM2->CCMR1 = (1<<8) | 1; //dois canais para entrada
	TIM2->SMCR = 3; //modo 3 do encoder
	TIM2->CR1 |= TIM_CR1_CEN; //liga o timer 2
	
	//TIMER 3
	AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP; //remap ch1 para PB4 e ch2 para PB5
	TIM3->CCMR1 = (1<<8) | 1; //dois canais para entrada
	TIM3->SMCR = 3; //modo 3 do encoder
	TIM3->CR1 |= TIM_CR1_CEN; //liga o timer 2
}

void enc_update()
{
	//roda direita
	velocity_right = (TIM2->CNT)*Ke;
	
	//roda esquerda
	velocity_left = (TIM3->CNT)*Ke;

	TIM2->CNT =0;
	TIM3->CNT =0;
}

int32_t enc_speed(uint32_t motor){
	if(motor) return velocity_right;
	else return velocity_left;
}
