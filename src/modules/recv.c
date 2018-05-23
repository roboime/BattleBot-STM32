/*
 * recv.c
 *
 *  Created on: 4 de nov de 2017
 *      Author: joaobapt
 */

#include "recv.h"
#include "stm32f10x.h"
#include "util.h"
#include "core_cm3.h"

#include <stdio.h>
#include <string.h>

#define CHANNELS 6
#define NUM_SAMPLES 5
#define MAX_DELAY 5

// receiver ISR priority (lower is higher priority)
#define RECV_ISR_PRIORITY 3

#if CHANNELS > 8
#error A maximum of 8 channels is supported
#endif

/* Variables to store persistent but temporary data inside the interrupts */
static uint16_t cur_step;
static uint16_t temp_timer_readings[CHANNELS+1];

/* Variables which act on the boundary between the ISRs and the main thread */
static volatile uint16_t isr_ch_values[CHANNELS];
static volatile uint16_t num_delay;
static volatile uint8_t new_frame;

/* Variables for the main thread, for the median filter */
static uint16_t ch_val_window[CHANNELS][NUM_SAMPLES];
static uint16_t ch_cur_order[CHANNELS][NUM_SAMPLES];
static uint16_t cur_j;

static normalization_params normalization_parameters[CHANNELS];

/* Mux selection */
inline static void internal_mux_select(uint32_t ch)
{
	/* The mux is on ports PB13-PB15. We treat them as a "single" 3-bit selector. */
	GPIOB->ODR = (GPIOB->ODR & ~0xE000) | ((ch & 7) << 13);
}

/* Initialization routine */
void recv_init()
{
	/* Configure the IO ports required for the functionality to work
	 * and that will be connected to the receiver channels */
	CONFIGURE_GPIO(GPIOB, 7, CFG_INPUT_FLOATING); // TIM4 ch2: receiver channel on mux
	CONFIGURE_GPIO(GPIOB, 13, CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ); // mux selection port 0
	CONFIGURE_GPIO(GPIOB, 14, CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ); // mux selection port 1
	CONFIGURE_GPIO(GPIOB, 15, CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ); // mux selection port 2

	/* Ensure TIM4 is enabled on the RCC */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	/* Configure it to tick on a 20 ms interval, with a 0.5 us precision.
	 * This means the prescaler is 72 MHz / 2000000 = 36
	 * and the auto-reload is 40000. */
	TIM4->PSC = 36 - 1;
	TIM4->ARR = 40000 - 1;
	TIM4->EGR = TIM_EGR_UG;

	/* TIM4 channel 2 is linked to the receiver port, so we must set it. */
	TIM4->CCMR1 = TIM_CCMR1_CC2S_0; // configure port 1 as input

	/* Configure interrupt for channel 1 and update on TIM4 */
	TIM4->DIER |= TIM_DIER_CC2IE | TIM_DIER_UIE;

	/* Enable the TIM4 */
	TIM4->CR1 = TIM_CR1_CEN;

	/* Insert the interrupts into the NVIC */
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, RECV_ISR_PRIORITY);

	/* Done with peripheral configuration... now, variable configuration: */
	cur_step = 0;
	cur_j = 0;

	/* Tick the mux to channel zero */
	internal_mux_select(0);

	for (uint32_t i = 0; i < CHANNELS; i++)
	{
		isr_ch_values[i] = 0;
		for (uint32_t j = 0; j < NUM_SAMPLES; j++)
		{
			ch_val_window[i][j] = 0;
			ch_cur_order[i][j] = j;
		}
	}

	num_delay = 0;
	new_frame = 0;
}

/* Now, I will explain with detail the states here: there are CHANNELS+1 states for collecting
 * the channel values. On the first CHANNELS ones, we set the polarity of the input capture
 * unit to rising edge, and after collecting one channel, we "tick" the mux selector to the
 * next channel. On the last channel, we set the polarity to falling edge, and on the final
 * state, we reset to the first one.*/

/* TIM4 IRQ handler */
void TIM4_IRQHandler()
{
	/* Check if the interrupt was really triggered by channel 1 */
	if (TIM4->SR & TIM_SR_CC2IF)
	{
		/* reset the capture and overflow flags */
		TIM4->SR = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);

		/* Read the current timer reading for this state */
		temp_timer_readings[cur_step] = TIM4->CCR2;

		/* Tick the mux and flip the polarity when required */
		if (cur_step < CHANNELS-1) /* Tick the mux */
			internal_mux_select(++cur_step);
		else if (cur_step == CHANNELS-1) /* Set the polarity */
		{
			TIM4->CCER |= TIM_CCER_CC2P;
			cur_step++;
		}
		else /* Reset mux and polarity */
		{
			cur_step = 0;
			internal_mux_select(0);
			TIM4->CCER &= ~TIM_CCER_CC2P;

			/* Here, we also transfer to the definitive variables */
			for (uint32_t i = 0; i < CHANNELS; i++)
			{
				/* Watch for overflow */
				if (temp_timer_readings[i+1] >= temp_timer_readings[i])
					isr_ch_values[i] = temp_timer_readings[i+1] - temp_timer_readings[i];
				else isr_ch_values[i] = 40000 - temp_timer_readings[i] + temp_timer_readings[i+1];
			}

			/* And reset the disconnection counter */
			num_delay = 0;
		}
	}

	/* Check if the update interrupt was triggered */
	if (TIM4->SR & TIM_FLAG_Update)
	{
		TIM4->SR = ~TIM_FLAG_Update;
		new_frame = 1;
	}
}

/* Update routine */
void recv_update()
{
	/* Escalate the priority to prevent the interrupt from occurring */
	__set_BASEPRI(RECV_ISR_PRIORITY);
	/* Transfer the values fetched from the ISRs to the value window */
	for (uint32_t i = 0; i < CHANNELS; i++)
		ch_val_window[i][cur_j] = isr_ch_values[i];

	/* Tick the disconnection detector */
	num_delay++;
	if (num_delay > MAX_DELAY)
		num_delay = MAX_DELAY;
	__set_BASEPRI(0);

	/* Bubblesort the samples */
	for (uint32_t i = 0; i < CHANNELS; i++)
	{
		/* Find the index of the current reading */
		uint32_t k = 0;
		for (; k < NUM_SAMPLES; k++) if (ch_cur_order[i][k] == cur_j) break;

		/* Try to bubble the reading backward */
		while (k > 0 && ch_val_window[i][ch_cur_order[i][k]] <= ch_val_window[i][ch_cur_order[i][k-1]])
		{
			uint16_t tmp = ch_val_window[i][ch_cur_order[i][k]];
			ch_val_window[i][ch_cur_order[i][k]] = ch_val_window[i][ch_cur_order[i][k-1]];
			ch_val_window[i][ch_cur_order[i][k-1]] = tmp;
			k--;
		}

		/* Try to bubble the reading forward */
		while (k < NUM_SAMPLES-1 && ch_val_window[i][ch_cur_order[i][k]] <= ch_val_window[i][ch_cur_order[i][k+1]])
		{
			uint16_t tmp = ch_val_window[i][ch_cur_order[i][k]];
			ch_val_window[i][ch_cur_order[i][k]] = ch_val_window[i][ch_cur_order[i][k+1]];
			ch_val_window[i][ch_cur_order[i][k+1]] = tmp;
			k++;
		}
	}

	/* update the window position */
	cur_j++;
	if (cur_j == NUM_SAMPLES) cur_j = 0;
}

uint32_t recv_num_channels()
{
	return CHANNELS;
}

void recv_set_normalization_params(uint32_t ch, normalization_params* params)
{
	memcpy(normalization_parameters+ch, params, sizeof(normalization_params));
}

int32_t recv_channel(uint32_t ch)
{
	int32_t v = recv_raw_channel(ch);
	if (v == 0) return INT32_MIN;

	normalization_params* pr = normalization_parameters+ch;
	return pr->out_low + (pr->out_high - pr->out_low) * (v - pr->in_low) / (pr->in_high - pr->in_low);
}

int32_t recv_raw_channel(uint32_t ch)
{
	return recv_is_connected() ? isr_ch_values[ch] : 0;
}

void recv_enable()
{
	TIM4->CCER |= TIM_CCER_CC2E;
}

void recv_disable()
{
	TIM4->CCER &= ~TIM_CCER_CC2E;
}

bool recv_is_connected()
{
	return num_delay != MAX_DELAY;
}

bool recv_new_frame()
{
	/* Check if a new frame was passed by reading the variable
	   written by the interrupt register */
	__set_BASEPRI(RECV_ISR_PRIORITY);
	int nf = new_frame;
	new_frame = 0;
	__set_BASEPRI(0);

	return nf;
}
