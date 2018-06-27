/*
 * spi.c
 *
 *  Created on: 24 de mai de 2018
 *      Author: joaobapt
 */

#include "spi.h"
#include "stm32f10x.h"

#include <stdbool.h>
#include "context/context.h"
#include "util.h"

#define SPI_INTERRUPT_PRIORITY 1
#define SPI_WAIT_UNTIL(cond) do { while (!(cond)) spi_context_switch(); } while (0)

static volatile context spi_context = 0;
static volatile bool spi_in_thread = 0, transfer_complete = 0;
static STACKPTR uint32_t spi_stack[512];

static volatile uint8_t dummy_read = 0, dummy_write = 0;

/* initializes the SPI peripheral with DMA support */
void spi_init()
{
	/* Activate the clock for the SPI peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* Configure the GPIO ports */
	CONFIGURE_GPIO(GPIOB, 12, CFG_OUTPUT_GENERAL_PUSH_PULL_2MHZ);
    CONFIGURE_GPIO(GPIOB, 13, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);
    CONFIGURE_GPIO(GPIOB, 14, CFG_INPUT_FLOATING);
    CONFIGURE_GPIO(GPIOB, 15, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);

    /* Put /CS pin HIGH */
    GPIOB->BSRR = GPIO_BSRR_BS12;

    /* Configure the SPI peripheral */
    SPI2->I2SCFGR = 0;

    /* High polarity, second phase, master, 64 divider (562.5 kHz) */
    SPI2->CR1 = SPI_CR1_CPOL | SPI_CR1_CPHA | (5 << 3) | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI2->CR2 = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    SPI2->CR1 |= SPI_CR1_SPE;

	/* Configure the DMA channel 5 for transmission */
	DMA1_Channel5->CCR = DMA_CCR5_DIR | (3 << 12);
	DMA1_Channel5->CPAR = (uint32_t)&SPI2->DR;

	/* Configure the DMA channel 4 for reception */
	DMA1_Channel4->CCR = DMA_CCR4_TCIE | (3 << 12);
	DMA1_Channel4->CPAR = (uint32_t)&SPI2->DR;

	/* Enable and setup the DMA channel 5 interrupt */
	/* Interrupt for a single channel because "transmission" and "reception" occurs at the same time */
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_SetPriority(DMA1_Channel4_IRQn, SPI_INTERRUPT_PRIORITY);

	/* Enable and setup the SysTick interrupt */
	NVIC_SetPriority(SysTick_IRQn, SPI_INTERRUPT_PRIORITY);
}

static void spi_context_switch()
{
	__disable_irq();
	spi_context = context_switch(spi_context);
	spi_in_thread = !spi_in_thread;
	__enable_irq();
}

struct spi_ud
{
	void(*thread)(void*);
	void* ud;
};

/* Thread starter helper */
static context spi_thread_starter(context ctx, void* ud)
{
	spi_in_thread = 1;
	spi_context = ctx;
	__enable_irq();

	struct spi_ud *ctx_ud = ud;
	ctx_ud->thread(ctx_ud->ud);

	__disable_irq();
	return spi_context;
}

/* Thread starter */
void spi_thread(void(*thread)(void*), void* ud)
{
	static struct spi_ud ctx_ud;
	ctx_ud.thread = thread;
	ctx_ud.ud = ud;
	__disable_irq();
	spi_context = context_new(spi_stack, sizeof(spi_stack), spi_thread_starter, &ctx_ud);
	spi_context_switch();
}

/* Interrupt handler for the DMA channel */
void DMA1_Channel4_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF4)
	{
		DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
		DMA1_Channel5->CCR &= ~DMA_CCR5_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF4 | DMA_IFCR_CGIF4;
		transfer_complete = 1;
		if (!spi_in_thread) spi_context_switch();
	}
}

/* Reading routine */
uint8_t spi_read(void* out, uint32_t size)
{
	if (!spi_in_thread) return 0;

	/* Trigger DMA channel 4 to receive */
	DMA1_Channel4->CMAR = (uint32_t)out;
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR |= DMA_CCR4_MINC;

	/* Trigger DMA channel 5 to a dummy write */
	DMA1_Channel5->CMAR = (uint32_t)&dummy_write;
	DMA1_Channel5->CNDTR = size;
	DMA1_Channel5->CCR &= ~DMA_CCR5_MINC;

	/* Enable both DMAs */
	DMA1_Channel5->CCR |= DMA_CCR5_EN;
	DMA1_Channel4->CCR |= DMA_CCR4_EN;

	/* Wait for the DMA to finish */
	SPI_WAIT_UNTIL(transfer_complete);
	transfer_complete = 0;

	return 1;
}

/* Writing routine */
uint8_t spi_write(const void* in, uint32_t size)
{
	if (!spi_in_thread) return 0;

	/* Trigger DMA channel 4 to a dummy read */
	DMA1_Channel4->CMAR = (uint32_t)&dummy_read;
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CCR &= ~DMA_CCR4_MINC;

	/* Trigger DMA channel 5 to transmit */
	DMA1_Channel5->CMAR = (uint32_t)in;
	DMA1_Channel5->CNDTR = size;
	DMA1_Channel5->CCR |= DMA_CCR5_MINC;

	/* Enable both DMAs */
	DMA1_Channel5->CCR |= DMA_CCR5_EN;
	DMA1_Channel4->CCR |= DMA_CCR4_EN;

	/* Wait for the DMA to finish */
	SPI_WAIT_UNTIL(transfer_complete);
	transfer_complete = 0;

	return 1;
}

/* Chip select */
void spi_select(uint32_t slave)
{
	if (slave) GPIOB->BRR = GPIO_BRR_BR12;
	else GPIOB->BSRR = GPIO_BSRR_BS12;
}

/* Switch frequency */
void spi_set_speed(uint32_t spd)
{
	SPI2->CR1 = (SPI2->CR1 & ~(7 << 3)) | ((spd ? 2 : 5) << 3);
}

/* Wait using SysTick */
void spi_trigger_wait(uint32_t cycles)
{
	if (!spi_in_thread) return;
	if (cycles < 48) return;

	SysTick->LOAD = cycles;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE | SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE;
}

void spi_wait_on()
{
	if (!spi_in_thread) return;
	if (SysTick->CTRL & SysTick_CTRL_ENABLE)
		spi_context_switch();
}

void spi_wait(uint32_t cycles)
{
	if (!spi_in_thread) return;
	if (cycles < 48) return;

	SysTick->LOAD = cycles;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE | SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE;
	spi_context_switch();
}

void SysTick_Handler(void)
{
	if (spi_in_thread) return;

	SysTick->CTRL = 0;
	spi_context_switch();
}
