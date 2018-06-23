/*
 * adc.c
 *
 *  Created on: 20 de jun de 2018
 *      Author: Girardin
 */

#define ADC_INTERRUPT_PRIORITY 4
int voltage_buffer[9];
int voltage_perm[9];


void adc_int()
{
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;                              //configura o divisor do clock para 6 ciclos

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                             //habilita o clock do ADC no APB2


    ADC1->CR1 = ADC_CR1_SCAN;										//configura o periferico ADC para modo de varredura
    ADC1->CR2 = ADC_CR2_DMA | (7 << 17);							//habilita o DMA


    ADC1->SQR3 = 0 | (1 << 5) | (2 << 10) | (3 << 15) | (4 << 20) | (5 << 25);
    ADC1->SQR2 = 6 | (7 << 5) ;
    ADC1->SQR1 = 8 << 20;											//define como 8 o numero de conversoes

    ADC1->CR2 |= ADC_CR2_ADON;

    for(uint32_t i = 0; i < 12; i++) asm volatile(“”);              //garante os 12 ciclos de espera para iniciar a calibracao

    ADC1->CR2 |= ADC_CR2_CAL;                                       //inicia a calibracao

    while(ADC1->CR2 & ADC_CR2_CAL){}                                //espera a calibracao terminar


    DMA1_Channel1->CCR  =  DMA_CCR1_MINC | DMA_CCR1_TCIE | (3 << 12) | (1<<8) | (1<<10);
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CMAR = voltage_buffer;
	DMA1_Channel1->CNDTR = 9;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_SetPriority(DMA1_Channel1_IRQn, ADC_INTERRUPT_PRIORITY);
}



void adc_request_reading()
{
	DMA1_Channel1->CMAR = voltage_buffer;
	DMA1_Channel1->CNDTR = 9;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;
	ADC1->CR2 |= ADC_CR2_ADON;
}

uint32_t adc_get_voltage(uint32_t cell)
{
	switch(cell)
	{
		case 1:
			return voltage_perm[0];
			break;
		case 2:
			return 2*voltage_perm[1]-voltage_perm[0];
			break;
		case 3:
			return (43*voltage_perm[2])/14 - 2*voltage_perm[1];
			break;
		case 4:
			return (57*voltage_perm[3])/14 - (43*voltage_perm[2])/14;
			break;
		case 5:
			return 5*voltage_perm[4]-(57*voltage_perm[3])/14;
	}
}

uint32_t adc_get_temp(uint32_t id)
{
	//faltam os dados do termistor
}


void DMA1_Channel1_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF1)
	{
		DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1;
	}

	for(int i=0; i<9; i++)
	{
		voltage_perm[i]=voltage_buffer[i];
	}

}
