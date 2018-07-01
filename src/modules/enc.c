/*
 * encoder.c
 *
 *  Created on: 1 de jul de 2018
 *      Author: Wei
 */


const float Ke=25*pi*R/N;
const float pi=3.1415;
float velocity_left;
float velocity_right;

void enc_init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN; //liga os clocks dos timers 2 e 3
	TIM2->CR1 |= TIM_CR1_CEN; //liga o timer 2
	TIM3->CR1 |= TIM_CR1_CEN; //liga o timer 2

	//TIMER 2
	
	AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_PARTIALREMAP; //remap ch1 para PB4 e ch2 para PB5
	TIM2->CCMR1 = 1<<8|1; //dois canais para entrada
	TIM2->SMCR = 3; //modo 3 do encoder
	GPIOB->CRL = (GPIOB->CRL & ~(0XF<<16) & ~(0XF<<20) | (0X1<<16) | (0X1<<16)); //portas PB4PB5 para entrada analógica
	
	//TIMER 3
		
		AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP; //remap ch1 para PB4 e ch2 para PB5
		TIM3->CCMR1 = 1<<8|1; //dois canais para entrada
		TIM3->SMCR = 3; //modo 3 do encoder
		GPIOA->CRH = (GPIOA->CRH & ~(0XF<<28) | (0X1 <<28));
		GPIOB->CRL = (GPIOB->CRL & ~(0XF<<12) | (0X1<<12));
}

void enc_update(){
	TIM2->CNT =0;
	TIM3->CNT =0;
	
	//roda direita
	velocity_right = (TIM2->CNT)*Ke;
	
	//roda esquerda
	velocity_left = (TIM3->CNT)*Ke;
}

int32_t emc_speed(uint32_t motor){
	if(motor){
		return velocity_right;
	}
	else{
		return velocity_left;
	}
}
