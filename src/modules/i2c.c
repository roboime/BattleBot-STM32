/*
 * i2c.c
 *
 *  Created on: 24 de mai de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"

#include "stdbool.h"
#include "i2c.h"
#include "context/context.h"
#include "util.h"

#define I2C_INTERRUPT_PRIORITY 3
#define I2C_WAIT_UNTIL(cond) do { while (!(cond)) i2c_context_switch(); } while (0)

static volatile context i2c_context = 0;
static volatile bool i2c_in_thread = 0, read_complete = 0, write_complete = 0;
static STACKPTR uint32_t i2c_stack[512];

/* initializes the I2C peripheral with DMA support */
void i2c_init()
{
	/* Activate the clock for the I2C peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	/* Remap the I2C1 pins to 8 and 9 */
    AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;

    /* Configure the I2C peripheral */
    /* Enable event interrupts, tell the peripheral it is running at 36 MHz */
    I2C1->CR2 = I2C_CR2_ITEVTEN | 36;

    /* Set the maximum rise time to 1 us (clock stretching) */
    I2C1->TRISE = 37;

    /* Set the switching time to 5 us (frequency of 100 kHz) */
    I2C1->CCR = 180;

    /* Enable the I2C peripheral */
    I2C1->CR1 |= I2C_CR1_PE;

    /* For some reason, bit 14 of this register must be set */
    I2C1->OAR1 = 0x4000;

    /* Configure the GPIO ports */
    CONFIGURE_GPIO(GPIOB, 8, CFG_OUTPUT_ALTERNATE_OPEN_DRAIN_2MHZ);
    CONFIGURE_GPIO(GPIOB, 9, CFG_OUTPUT_ALTERNATE_OPEN_DRAIN_2MHZ);

	/* Configure the DMA channel 6 for transmission */
	DMA1_Channel6->CCR = DMA_CCR6_MINC | DMA_CCR6_DIR | DMA_CCR6_TCIE | (3 << 12);
	DMA1_Channel6->CPAR = (uint32_t)&I2C1->DR;

	/* Configure the DMA channel 7 for receiving */
	DMA1_Channel7->CCR = DMA_CCR7_MINC | DMA_CCR7_TCIE | (3 << 12);
	DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;

	/* Setup the I2C1 interrupt */
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_SetPriority(I2C1_EV_IRQn, I2C_INTERRUPT_PRIORITY);

	/* Enable and setup the DMA channel 6 interrupt */
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	NVIC_SetPriority(DMA1_Channel6_IRQn, I2C_INTERRUPT_PRIORITY);

	/* Enable and setup the DMA channel 7 interrupt */
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	NVIC_SetPriority(DMA1_Channel7_IRQn, I2C_INTERRUPT_PRIORITY);
}

static void i2c_context_switch()
{
	__disable_irq();
	i2c_context = context_switch(i2c_context);
	i2c_in_thread = !i2c_in_thread;
	__enable_irq();
}

/* I2C interrupt handler */
void I2C1_EV_IRQHandler(void)
{
	i2c_context_switch();
}

struct i2c_ud
{
	void(*thread)(void*);
	void* ud;
};

/* Thread starter helper */
static context i2c_thread_starter(context ctx, void* ud)
{
	i2c_in_thread = 1;
	i2c_context = ctx;
	__enable_irq();

	struct i2c_ud *ctx_ud = ud;
	ctx_ud->thread(ctx_ud->ud);

	__disable_irq();
	return i2c_context;
}

/* Thread starter */
void i2c_thread(void(*thread)(void*), void* ud)
{
	static struct i2c_ud ctx_ud;
	ctx_ud.thread = thread;
	ctx_ud.ud = ud;
	__disable_irq();
	i2c_context = context_new(i2c_stack, sizeof(i2c_stack), i2c_thread_starter, &ctx_ud);
	i2c_context_switch();
}

/* Interrupt handler for the receiver DMA channel */
void DMA1_Channel7_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF7)
	{
		DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF7 | DMA_IFCR_CGIF7;
		read_complete = 1;
		i2c_context_switch();
	}
}

/* Reading routine */
uint8_t i2c_read(uint8_t addr, void* out, uint32_t size)
{
	if (!i2c_in_thread) return 0;

	/* Enable ACK and DMA */
	I2C1->CR1 |= I2C_CR1_ACK;
	I2C1->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;

	/* Trigger DMA channel 7 to receive */
	DMA1_Channel7->CMAR = (uint32_t)out;
	DMA1_Channel7->CNDTR = size;

	/* Enable DMA */
	DMA1_Channel7->CCR |= DMA_CCR7_EN;

	/* Start event */
	I2C1->CR1 |= I2C_CR1_START;
	I2C_WAIT_UNTIL(I2C1->SR1 & I2C_SR1_SB);

	/* Send address with read bit */
	I2C1->DR = (addr << 1) | 1;
	I2C_WAIT_UNTIL(I2C1->SR1 & I2C_SR1_ADDR);
	I2C1->SR1; I2C1->SR2; // Read both to clear ADDR

	/* Wait for the DMA to finish */
	I2C_WAIT_UNTIL(read_complete);
	read_complete = 0;

	/* Disable DMA and ACK */
	I2C1->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
	I2C1->CR1 &= ~I2C_CR1_ACK;

	/* Stop event */
	I2C1->CR1 |= I2C_CR1_STOP;

	return 1;
}

/* Interrupt handler for the transmitter DMA channel */
void DMA1_Channel6_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF6)
	{
		DMA1_Channel6->CCR &= ~DMA_CCR6_EN;
		DMA1->IFCR = DMA_IFCR_CTCIF6 | DMA_IFCR_CGIF6;
		write_complete = 1;
		i2c_context_switch();
	}
}


/* Writing routine */
uint8_t i2c_write(uint8_t addr, const void* in, uint32_t size)
{
	if (!i2c_in_thread) return 0;

	/* Enable DMA for I2C */
	I2C1->CR2 |= I2C_CR2_DMAEN;

	/* Trigger DMA channel 6 to transmit */
	DMA1_Channel6->CMAR = (uint32_t)in;
	DMA1_Channel6->CNDTR = size;

	/* Enable DMA */
	DMA1_Channel6->CCR |= DMA_CCR6_EN;

	/* Start event */
	I2C1->CR1 |= I2C_CR1_START;
	I2C_WAIT_UNTIL(I2C1->SR1 & I2C_SR1_SB);

	/* Send address with write bit */
	I2C1->DR = addr << 1;
	I2C_WAIT_UNTIL(I2C1->SR1 & I2C_SR1_ADDR);
	I2C1->SR1; I2C1->SR2; // Read both to clear ADDR

	/* Wait for the DMA to finish */
	I2C_WAIT_UNTIL(write_complete);
	write_complete = 0;

	/* Disable DMA for I2C */
	I2C1->CR2 &= ~I2C_CR2_DMAEN;

	/* Wait for the transfer to finish */
	I2C_WAIT_UNTIL(I2C1->SR1 & I2C_SR1_BTF);

	/* Stop event */
	I2C1->CR1 |= I2C_CR1_STOP;

	return 1;
}
