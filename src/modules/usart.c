/*
 * usart.c
 *
 *  Created on: 1 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"

#include "usart.h"
#include "context/context.h"
#include "util.h"

#define USART_INTERRUPT_PRIORITY 3
#define USART_WAIT_UNTIL(cond) do { while (!(cond)) usart_context_switch(); } while (0)

static volatile context usart_context = 0;
static volatile bool usart_in_thread = 0, read_complete = 0, write_complete = 0;
static STACKPTR uint32_t usart_stack[512];

/* initializes the USART peripheral with DMA support */
void usart_init()
{
	/* Activate the clock for the USART peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* Configure the GPIO ports */
    CONFIGURE_GPIO(GPIOB, 10, CFG_OUTPUT_ALTERNATE_PUSH_PULL_10MHZ);
    CONFIGURE_GPIO(GPIOB, 11, CFG_INPUT_FLOATING);

    /* Configure the USART peripheral */
    /* Set a baud rate of 400000 Hz */
    /* 36 MHz / 400000 Hz = 90 */
    USART3->BRR = 90;

    /* 2 stop bits */
    USART3->CR2 = 2 << 12;

    /* Enable the USART hardware, interrupt on transmission complete */
    USART3->CR1 = USART_CR1_TCIE | USART_CR1_UE;

	/* Configure the DMA channel 2 for transmission */
	DMA1_Channel2->CCR = DMA_CCR2_MINC | DMA_CCR2_DIR | DMA_CCR2_TCIE | (3 << 12);
	DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;

	/* Configure the DMA channel 3 for receiving */
	DMA1_Channel3->CCR = DMA_CCR3_MINC | DMA_CCR3_TCIE | (3 << 12);
	DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;

	/* Setup the USART3 interrupt */
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_SetPriority(USART3_IRQn, USART_INTERRUPT_PRIORITY);

	/* Enable and setup the DMA channel 2 interrupt */
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_SetPriority(DMA1_Channel2_IRQn, USART_INTERRUPT_PRIORITY);

	/* Enable and setup the DMA channel 3 interrupt */
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_SetPriority(DMA1_Channel3_IRQn, USART_INTERRUPT_PRIORITY);
}

static void usart_context_switch()
{
	__set_BASEPRI(USART_INTERRUPT_PRIORITY);
	usart_context = context_switch(usart_context);
	usart_in_thread = !usart_in_thread;
	__set_BASEPRI(0);
}

/* USART interrupt handler */
void USART3_IRQHandler(void)
{
	usart_context_switch();
}

struct usart_ud
{
	void(*thread)(void*);
	void* ud;
};

/* Thread starter helper */
static context usart_thread_starter(context ctx, void* ud)
{
	usart_in_thread = 1;
	usart_context = ctx;
	__set_BASEPRI(0);

	struct usart_ud *ctx_ud = ud;
	ctx_ud->thread(ctx_ud->ud);

	__set_BASEPRI(USART_INTERRUPT_PRIORITY);
	return usart_context;
}

/* Thread starter */
void usart_thread(void(*thread)(void*), void* ud)
{
	static struct usart_ud ctx_ud;
	ctx_ud.thread = thread;
	ctx_ud.ud = ud;
	__set_BASEPRI(USART_INTERRUPT_PRIORITY);
	usart_context = context_new(usart_stack, sizeof(usart_stack), usart_thread_starter, &ctx_ud);
	usart_context_switch();
}

/* Interrupt handler for the receiver DMA channel */
void DMA1_Channel3_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF3)
	{
		DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3;
		read_complete = 1;
		usart_context_switch();
	}
}

/* Reading routine */
uint8_t usart_read(void* out, uint32_t size)
{
	if (!usart_in_thread) return 0;

	/* Enable receiver DMA */
	USART3->CR3 |= USART_CR3_DMAR;

	/* Enable RX */
	USART3->CR1 |= USART_CR1_RE;

	/* Trigger DMA channel 3 to receive */
	DMA1_Channel3->CMAR = (uint32_t)out;
	DMA1_Channel3->CNDTR = size;

	/* Enable DMA */
	DMA1_Channel3->CCR |= DMA_CCR3_EN;

	/* Wait for the DMA to finish */
	USART_WAIT_UNTIL(read_complete);
	read_complete = 0;

	/* Disable receiver DMA and RX */
	USART3->CR1 &= ~USART_CR1_RE;
	USART3->CR3 &= ~USART_CR3_DMAR;

	return 1;
}

/* Interrupt handler for the transmitter DMA channel */
void DMA1_Channel2_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF2)
	{
		DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;
		write_complete = 1;
		usart_context_switch();
	}
}


/* Writing routine */
uint8_t usart_write(const void* in, uint32_t size)
{
	if (!usart_in_thread) return 0;

	/* Enable transmitter DMA */
	USART3->CR3 |= USART_CR3_DMAT;

	/* Enable TX */
	USART3->CR1 |= USART_CR1_TE;

	/* Trigger DMA channel 2 to transmit */
	DMA1_Channel2->CMAR = (uint32_t)in;
	DMA1_Channel2->CNDTR = size;

	/* Clear TC flag just to be sure */
	USART3->SR = ~USART_SR_TC;

	/* Enable DMA */
	DMA1_Channel2->CCR |= DMA_CCR2_EN;

	/* Wait for the DMA to finish */
	USART_WAIT_UNTIL(write_complete);
	write_complete = 0;

	/* Wait for TC to be asserted and clear it */
	USART_WAIT_UNTIL(USART3->SR & USART_SR_TC);
	USART3->SR = ~USART_SR_TC;

	/* Disable transmitter DMA and TX */
	USART3->CR1 &= ~USART_CR1_TE;
	USART3->CR3 &= ~USART_CR3_DMAT;

	return 1;
}
