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
#include "modules/iwdg.h"
#include "modules/spi.h"
#include "modules/usart.h"
#include "modules/esp.h"

#include "context/context.h"

const normalization_params vert_params = { 2049, 3009, 4032, 195, 0, -195 },
	hor_params = { 2191, 3016, 3768, -195, 0, 195 };

int main(void)
{
	// Enable all GPIO ports so the other functions don't need to
	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
				   RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	CONFIGURE_GPIO(GPIOC, 13, CFG_OUTPUT_GENERAL_PUSH_PULL_10MHZ);

	// Initialize all primary modules
	recv_init();
	mvmt_init();

	// Initialize the independent watchdog
	//iwdg_init();

	// Initialize the communications modules
	//i2c_init();
	//usart_init();
	//esp_init();

	// Pass the normalization parameters
	recv_set_normalization_params(2, &vert_params);
	recv_set_normalization_params(3, &hor_params);

	// Enable reception of signal data
	recv_enable();

	int i = 0;

	while (1)
	{
		if (recv_new_frame())
		{
			//iwdg_reset();
			recv_update();

			int y = recv_channel(2);
			int x = recv_channel(3)*2/7;

			mvmt_control(MOTOR_LEFT, y+x);
			mvmt_control(MOTOR_RIGHT, y-x);

			i++;
			if (i % 16 == 0) GPIOC->ODR ^= GPIO_ODR_ODR13;

			/*esp_recv_commands();
			while (esp_new_commands())
			{
				uint8_t buffer[160];
				uint32_t size = esp_next_command(buffer);

				if (size >= 1 && buffer[0] == 2) GPIOC->ODR ^= GPIO_ODR_ODR13;
			}*/
		}
		//__WFI();
	}
}

void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}
