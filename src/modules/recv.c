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
#define MAX_DELAY 5

#if CHANNELS > 8
#error A maximum of 8 channels is supported
#endif

// bit band address for TIM2->SR & TIM2_SR_CC2P
#define RECV_POLARITY_BIT BIT_BANDING_PERIPH(TIM2->CCER, 5)

// receiver ISR priority (lower is higher priority)
#define RECV_ISR_PRIORITY 1

// mux select port
#define RECV_MUX_PORT 13

/* optimization details: we advance the mux data by "ticking" a variable in order
   to set the BSRR register; this is done to recude the amound of reads and writes
   to the registers */
#define MUX_ADVANCE_INIT ((0<<RECV_MUX_PORT) + (7<<(16+RECV_MUX_PORT)))
#define MUX_ADVANCE_BASE ((1<<RECV_MUX_PORT) - (1<<(16+RECV_MUX_PORT)))

/* Variables to store persistent but temporary data inside the interrupts */
static uint16_t cur_step, cur_k;
static uint32_t cur_mux_advance;
static uint16_t temp_timer_readings[CHANNELS+1];

/* Variables which act on the boundary between the ISRs and the main thread */
static volatile uint16_t isr_ch_queue[QUEUE_SIZE][CHANNELS];
static volatile uint16_t num_delay;

/* Variables for the main thread, for the median filter */
static uint16_t ch_vals[CHANNELS];

static normalization_params normalization_parameters[CHANNELS];

/* Mux selection */
/* The mux is on ports PB13-PB15. We treat them as a "single" 3-bit selector. */
inline static void internal_mux_reset()
{
	GPIOB->BSRR = cur_mux_advance = MUX_ADVANCE_INIT;
}

inline static void internal_mux_advance()
{
	GPIOB->BSRR = cur_mux_advance += MUX_ADVANCE_BASE;
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
	internal_mux_reset();

	for (uint32_t i = 0; i < CHANNELS; i++)
	{
		for (uint32_t k = 0; k < QUEUE_SIZE; k++)
			isr_ch_queue[i][k] = 0;

		ch_vals[i] = 0;
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
			internal_mux_advance();
		}
		else /* Reset mux */
		{
			cur_step = 0;
			internal_mux_reset();

			/* Here, we also transfer to the definitive variables */
			for (uint32_t i = 0; i < CHANNELS; i++)
			{
				/* Watch for overflow */
				if (temp_timer_readings[i+1] >= temp_timer_readings[i])
					isr_ch_queue[i][cur_k] = temp_timer_readings[i+1] - temp_timer_readings[i];
				else isr_ch_queue[i][cur_k] = 40000 - temp_timer_readings[i] + temp_timer_readings[i+1];
			}

			/* Tick the queue number */
			cur_k = (cur_k+1) % QUEUE_SIZE;

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
	if (num_delay > MAX_DELAY)
	{
		num_delay = MAX_DELAY;
		/* Reset the interrupt state */
		cur_step = 0;
		internal_mux_reset();
	}
	__enable_irq();

	/* Transfer the values fetched from the ISRs to the value window */
	uint32_t prev_k = (cur_k + QUEUE_SIZE-1) % QUEUE_SIZE;
	for (uint32_t i = 0; i < CHANNELS; i++)
		ch_vals[i] = isr_ch_queue[prev_k][i];
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
	if (v == 0) return INT32_MIN;

	const normalization_params* pr = normalization_parameters+ch;
	if (v >= pr->in_high) return pr->out_high;
	else if (v <= pr->in_low) return pr->out_low;
	else if (v >= pr->in_mid)
		return pr->out_mid + (v - pr->in_mid) * (pr->out_high - pr->out_mid) / (pr->in_high - pr->in_mid);
	else return pr->out_mid + (v - pr->in_mid) * (pr->out_low - pr->out_mid) / (pr->in_low - pr->in_mid);
}

int32_t recv_raw_channel(uint32_t ch)
{
	return recv_is_connected() ? ch_vals[ch] : 0;
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
