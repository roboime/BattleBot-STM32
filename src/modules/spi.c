/*
 * spi.c
 *
 *  Created on: 24 de mai de 2018
 *      Author: joaobapt
 */

#include "spi.h"
#include "stm32f10x.h"

#include "stdbool.h"
#include "context/context.h"
#include "util.h"

#define SPI_INTERRUPT_PRIORITY 1
#define SPI_WAIT_UNTIL(cond) do { while (!(cond)) spi_context_switch(); } while (0)

static volatile context spi_context = 0;
static volatile bool spi_in_thread = 0, read_complete = 0, write_complete = 0;
static STACKPTR uint32_t spi_stack[512];

/* initializes the SPI peripheral with DMA support */
void spi_init()
{
	/* Activate the clock for the SPI peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* Configure the GPIO ports */
	CONFIGURE_GPIO(GPIOB, 12, CFG_OUTPUT_ALTERNATE_PUSH_PULL_2MHZ);
    CONFIGURE_GPIO(GPIOB, 13, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);
    CONFIGURE_GPIO(GPIOB, 14, CFG_INPUT_FLOATING);
    CONFIGURE_GPIO(GPIOB, 15, CFG_OUTPUT_ALTERNATE_PUSH_PULL_50MHZ);

    /* Configure the SPI peripheral */
    /* High polarity, second phase, 64 divider (562.5 kHz) */
    SPI2->CR1 = SPI_CR1_CPOL | SPI_CR1_CPHA | (5 << 3) | SPI_CR1_SPE;

	/* Configure the DMA channel 5 for transmission */
	DMA1_Channel5->CCR = DMA_CCR5_MINC | DMA_CCR5_DIR | DMA_CCR5_TCIE | (3 << 12);
	DMA1_Channel5->CPAR = (uint32_t)&SPI2->DR;

	/* Configure the DMA channel 4 for receiving */
	DMA1_Channel4->CCR = DMA_CCR4_MINC | DMA_CCR4_TCIE | (3 << 12);
	DMA1_Channel4->CPAR = (uint32_t)&SPI2->DR;

	/* Enable and setup the DMA channel 5 interrupt */
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	NVIC_SetPriority(DMA1_Channel5_IRQn, SPI_INTERRUPT_PRIORITY);

	/* Enable and setup the DMA channel 4 interrupt */
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_SetPriority(DMA1_Channel4_IRQn, SPI_INTERRUPT_PRIORITY);
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

/* Interrupt handler for the receiver DMA channel */
void DMA1_Channel5_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF5)
	{
		DMA1_Channel5->CCR &= ~DMA_CCR7_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CGIF5;
		read_complete = 1;
		spi_context_switch();
	}
}

/* Reading routine */
uint8_t spi_read(void* out, uint32_t size)
{
	if (!spi_in_thread) return 0;

	/* Enable DMA on receiving */
	SPI2->CR2 |= SPI_CR2_RXDMAEN;

	/* Trigger DMA channel 5 to receive */
	DMA1_Channel5->CMAR = (uint32_t)out;
	DMA1_Channel5->CNDTR = size;

	/* Enable DMA */
	DMA1_Channel5->CCR |= DMA_CCR5_EN;

	/* Read DR to trigger reception */
	SPI2->DR;

	/* Wait for the DMA to finish */
	SPI_WAIT_UNTIL(read_complete);
	read_complete = 0;

	/* Disable DMA on receiving */
	SPI2->CR2 &= ~SPI_CR2_RXDMAEN;

	return 1;
}

/* Interrupt handler for the transmitter DMA channel */
void DMA1_Channel4_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF4)
	{
		DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF4 | DMA_IFCR_CGIF4;
		write_complete = 1;
		spi_context_switch();
	}
}


/* Writing routine */
uint8_t spi_write(const void* in, uint32_t size)
{
	if (!spi_in_thread) return 0;

	/* Enable DMA for SPI */
	SPI2->CR2 |= SPI_CR2_TXDMAEN;

	/* Trigger DMA channel 4 to transmit */
	DMA1_Channel4->CMAR = (uint32_t)in;
	DMA1_Channel4->CNDTR = size;

	/* Enable DMA */
	DMA1_Channel4->CCR |= DMA_CCR4_EN;

	/* Wait for the DMA to finish */
	SPI_WAIT_UNTIL(write_complete);
	write_complete = 0;

	/* Disable DMA for SPI */
	SPI2->CR2 &= ~SPI_CR2_TXDMAEN;

	return 1;
}

/* Chip select */
void spi_select(uint32_t slave)
{
	if (slave) GPIOB->BSRR = GPIO_BSRR_BS12;
	else GPIOB->BRR = GPIO_BRR_BR12;
}

/* Switch frequency */
void spi_set_speed(uint32_t spd)
{
	SPI2->CR1 = (SPI2->CR1 & ~(7 << 3)) | ((spd ? 1 : 5) << 3);
}
