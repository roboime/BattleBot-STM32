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
#define QUEUE_SIZE 4
#define NUM_SAMPLES 5
#define MAX_DELAY 5

#if CHANNELS > 8
#error A maximum of 8 channels is supported
#endif

// bit band address for TIM4->CCER & TIM_CCER_CC2P
#define RECV_POLARITY_BIT BIT_BANDING_PERIPH(TIM4->CCER, 5)
#define SEL_MUX_BIT_0 BIT_BANDING_PERIPH(GPIOB->ODR, 0)
#define SEL_MUX_BIT_1 BIT_BANDING_PERIPH(GPIOB->ODR, 1)
#define SEL_MUX_BIT_2 BIT_BANDING_PERIPH(GPIOA->ODR, 12)

// receiver ISR priority (lower is higher priority)
#define RECV_ISR_PRIORITY 3

/* Variables to store persistent but temporary data inside the interrupts */
static uint16_t cur_step, cur_k;
static uint16_t temp_timer_readings[CHANNELS+1];

/* Variables which act on the boundary between the ISRs and the main thread */
static volatile uint16_t isr_ch_queue[QUEUE_SIZE][CHANNELS];
static volatile uint16_t num_delay;

/* Variables for the main thread, for the median filter */
static uint16_t ch_val_window[CHANNELS][NUM_SAMPLES];
static uint16_t ch_cur_order[CHANNELS][NUM_SAMPLES];
static uint16_t cur_j;

static normalization_params normalization_parameters[CHANNELS];

/* Mux selection */
/* The mux is on ports PA12, PB1, PB0. It is not possible to "treat" the channel as a single 3-bit. */
inline static void internal_mux_select(uint32_t channel)
{
	SEL_MUX_BIT_0 = channel;
	SEL_MUX_BIT_1 = channel >> 1;
	SEL_MUX_BIT_2 = channel >> 2;
}

/* Initialization routine */
void recv_init()
{
	/* Configure the IO ports required for the functionality to work
	 * and that will be connected to the receiver channels */
	CONFIGURE_GPIO(GPIOB,  7, CFG_INPUT_FLOATING); // TIM4 ch2: receiver channel on mux
	CONFIGURE_GPIO(GPIOB,  0, CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ); // mux selection port 0
	CONFIGURE_GPIO(GPIOB,  1, CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ); // mux selection port 1
	CONFIGURE_GPIO(GPIOA, 12, CFG_OUTPUT_GENERAL_OPEN_DRAIN_10MHZ); // mux selection port 2

	/* Ensure TIM4 is enabled on the RCC */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	/* Configure it to tick on a 20 ms interval, with a 0.5 us precision.
	 * This means the prescaler is 72 MHz / 2000000 = 36
	 * and the auto-reload is 40000. */
	TIM4->PSC = 36 - 1;
	TIM4->ARR = 40000 - 1;
	TIM4->EGR = TIM_EGR_UG;

	/* TIM4 channel 2 is linked to the receiver port, so we must set it. */
	TIM4->CCMR1 = TIM_CCMR1_CC2S_0 | (11 << 12); // configure port 1 as input

	/* Configure interrupt for channel 1 and update on TIM4 */
	TIM4->DIER |= TIM_DIER_CC2IE;

	/* Enable the TIM4 */
	TIM4->CR1 = TIM_CR1_CEN;

	/* Insert the interrupts into the NVIC */
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, RECV_ISR_PRIORITY);

	/* Done with peripheral configuration... now, variable configuration: */
	cur_step = 0;
	cur_k = 0;

	/* Tick the mux to channel zero */
	internal_mux_select(0);

	for (uint32_t i = 0; i < CHANNELS; i++)
	{
		for (uint32_t k = 0; k < QUEUE_SIZE; k++)
			isr_ch_queue[i][k] = 0;

		for (uint32_t j = 0; j < NUM_SAMPLES; j++)
		{
			ch_val_window[i][j] = 0;
			ch_cur_order[i][j] = j;
		}
	}

	num_delay = 0;
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

		/* Read the current timer reading for this state on the queue */
		temp_timer_readings[cur_step] = TIM4->CCR2;

		/* Flip the polarity when required */
		RECV_POLARITY_BIT = cur_step == CHANNELS-1;

		if (cur_step < CHANNELS)
		{
			/* Tick the mux */
			cur_step++;
			if (cur_step != CHANNELS)
				internal_mux_select(cur_step);
		}
		else /* Reset mux */
		{
			cur_step = 0;
			internal_mux_select(0);

			/* Here, we also transfer to the definitive variables */
			for (uint32_t i = 0; i < CHANNELS; i++)
			{
				/* Watch for overflow */
				if (temp_timer_readings[i+1] >= temp_timer_readings[i])
					isr_ch_queue[cur_k][i] = temp_timer_readings[i+1] - temp_timer_readings[i];
				else isr_ch_queue[cur_k][i] = 40000 - temp_timer_readings[i] + temp_timer_readings[i+1];
			}

			/* Tick the queue number */
			cur_k = (cur_k+1)&3;

			/* And reset the disconnection counter */
			num_delay = 0;
		}
	}
}

/* Update routine */
void recv_update()
{
	/* Escalate the priority to prevent the interrupt from occurring */
	__disable_irq();
	/* Tick the disconnection detector */
	num_delay++;
	if (num_delay >= MAX_DELAY)
		num_delay = MAX_DELAY;
	__enable_irq();

	/* Transfer the values fetched from the ISRs to the value window */
	uint32_t prev_k = (cur_k+3)&3;
	uint16_t temp_ch_vals[CHANNELS];

	/* Discard any stray "wrong" frames */
	for (uint32_t i = 0; i < CHANNELS; i++)
	{
		temp_ch_vals[i] = isr_ch_queue[prev_k][i];
		//if (temp_ch_vals[i] < 1800 || temp_ch_vals[i] > 4200) return;
	}

	/* Transfer the values */
	for (uint32_t i = 0; i < CHANNELS; i++)
		ch_val_window[i][cur_j] = temp_ch_vals[i];


	/* Bubblesort the samples */
	for (uint32_t i = 0; i < CHANNELS; i++)
	{
		/* Find the index of the current reading */
		uint32_t k = 0;
		for (; k < NUM_SAMPLES; k++) if (ch_cur_order[i][k] == cur_j) break;

		/* Try to bubble the reading backward */
		while (k > 0 && ch_val_window[i][ch_cur_order[i][k]] < ch_val_window[i][ch_cur_order[i][k-1]])
		{
			uint16_t tmp = ch_cur_order[i][k];
			ch_cur_order[i][k] = ch_cur_order[i][k-1];
			ch_cur_order[i][k-1] = tmp;
			k--;
		}

		/* Try to bubble the reading forward */
		while (k < NUM_SAMPLES-1 && ch_val_window[i][ch_cur_order[i][k]] > ch_val_window[i][ch_cur_order[i][k+1]])
		{
			uint16_t tmp = ch_cur_order[i][k];
			ch_cur_order[i][k] = ch_cur_order[i][k+1];
			ch_cur_order[i][k+1] = tmp;
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

void recv_set_normalization_params(uint32_t ch, const normalization_params* params)
{
	memcpy(normalization_parameters+ch, params, sizeof(normalization_params));
}

int32_t recv_channel(uint32_t ch)
{
	int32_t v = recv_raw_channel(ch);
	if (v == 0) return 0;

	const normalization_params* pr = normalization_parameters+ch;
	if (v >= pr->in_high) return pr->out_high;
	else if (v <= pr->in_low) return pr->out_low;
	else if (v >= pr->in_mid)
		return pr->out_mid + (v - pr->in_mid) * (pr->out_high - pr->out_mid) / (pr->in_high - pr->in_mid);
	else return pr->out_mid + (v - pr->in_mid) * (pr->out_low - pr->out_mid) / (pr->in_low - pr->in_mid);
}

int32_t recv_raw_channel(uint32_t ch)
{
	return recv_is_connected() ? ch_val_window[ch][ch_cur_order[ch][NUM_SAMPLES/2]] : 0;
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
	/* Check if a new frame was passed by reading the flag status */
	bool nf = false;
	if (TIM4->SR & TIM_SR_UIF)
	{
		TIM4->SR = ~TIM_SR_UIF;
		nf = true;
	}

	return nf;
}
