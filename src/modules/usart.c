/*
 * usart.c
 *
 *  Created on: 1 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"

#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "util.h"

#define USART_INTERRUPT_PRIORITY 2
#define USART_WAIT_UNTIL(cond) do { while (!(cond)) usart_context_switch(); } while (0)

static char read_queue[512], write_queue[512];
static uint32_t read_total_enqs, read_deq_val;
static uint32_t write_deq_val, write_enq_val;
/* Warning: write_deq_val represents the dequeue value AFTER the DMA is complete
   So the real value of the dequeue value is write_deq_val - DMA1_Channel2->CNDTR */

/* initializes the USART peripheral with DMA support */
void usart_init()
{
	/* Activate the clock for the USART peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    /* Configure the GPIO ports */
    CONFIGURE_GPIO(GPIOB, 10, CFG_OUTPUT_ALTERNATE_PUSH_PULL_10MHZ);
    CONFIGURE_GPIO(GPIOB, 11, CFG_INPUT_FLOATING);

    /* Configure the USART peripheral */
    /* Set a baud rate of 800 kHz */
    /* 36 MHz / 800 kHz = 45 */
    USART3->BRR = 45;

    // debug, remove me
    //USART3->BRR = 625;

    /* 1 stop bit */
    USART3->CR2 = 0;

    USART3->SR = ~USART_SR_TC;

    /* Enable receiver and transmitter DMA */
    USART3->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;

    /* Enable the USART hardware, transmitter and receiver */
    USART3->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	/* Configure the DMA channel 2 for transmission */
	DMA1_Channel2->CCR = DMA_CCR2_MINC | DMA_CCR2_DIR | DMA_CCR2_TCIE | (2 << 12);
	DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;

	/* Configure the DMA channel 3 for reception */
	DMA1_Channel3->CCR = DMA_CCR3_MINC | DMA_CCR3_CIRC | DMA_CCR3_TCIE | (2 << 12);
	DMA1_Channel3->CPAR = (uint32_t)&USART3->DR;
	DMA1_Channel3->CMAR = (uint32_t)read_queue;
	DMA1_Channel3->CNDTR = sizeof(read_queue);

	/* Enable and setup the DMA channel 2 interrupt */
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC_SetPriority(DMA1_Channel2_IRQn, USART_INTERRUPT_PRIORITY);

	/* Enable and setup the DMA channel 3 interrupt */
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC_SetPriority(DMA1_Channel3_IRQn, USART_INTERRUPT_PRIORITY);

	/* Permanently enable the DMA channel 3 interrupt */
	DMA1->IFCR = DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3;
	DMA1_Channel3->CCR |= DMA_CCR3_EN;

	/* Setup the variables */
	read_deq_val = 0;
	read_total_enqs = 0;
	write_deq_val = 0;
	write_enq_val = 0;
}

void DMA1_Channel3_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF3)
	{
		DMA1->IFCR = DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3;
		read_total_enqs++;
	}
}

uint32_t usart_read_available()
{
	// read_enq_rem = sizeof(read_queue) - read_enq_val
	uint32_t read_enq_val = (read_total_enqs+1) * sizeof(read_queue) - DMA1_Channel3->CNDTR;
	if (read_enq_val > sizeof(read_queue) && read_deq_val < read_enq_val - sizeof(read_queue))
		read_deq_val = read_enq_val - sizeof(read_queue);

	return read_enq_val - read_deq_val;
}

/* Reading routine */
uint32_t usart_read(void* out, uint32_t size)
{
	/* Limit the size to the available size */
	uint32_t max_size = usart_read_available();
	if (size > max_size) size = max_size;

	/* If no bytes can be transferred, bail out early */
	if (size == 0) return 0;

	/* Transfer the received data in one go if possible */
	uint32_t read_cnt = read_deq_val % sizeof(read_queue);
	if (read_cnt + size <= sizeof(read_queue))
		memcpy(out, read_queue+read_cnt, size);
	/* Transfer in two goes */
	else
	{
		uint32_t first_chunk = sizeof(read_queue) - read_cnt;
		memcpy(out, read_queue+read_cnt, first_chunk);
		memcpy(out+first_chunk, read_queue, size-first_chunk);
	}

	read_deq_val += size;

	return size;
}

/* Single byte read */
uint32_t usart_read_byte()
{
	/* Check if there is something on the queue */
	if (usart_read_available() < 1) return -1;

	/* Read from the queue */
	uint32_t read_cnt = read_deq_val % sizeof(read_queue);
	uint8_t byte = read_queue[read_cnt];
	read_deq_val++;

	/* Returns the byte */
	return byte;
}

static void usart_enqueue_dma(uint32_t size)
{
	uint32_t write_cnt = write_deq_val % sizeof(write_queue);

	if (size > sizeof(write_queue) - write_cnt)
		size = sizeof(write_queue) - write_cnt;

	DMA1_Channel2->CMAR = (uint32_t)(write_queue + write_cnt);
	DMA1_Channel2->CNDTR = size;
	DMA1_Channel2->CCR |= DMA_CCR2_EN;

	write_deq_val += size;
}

/* Interrupt handler for the receiver DMA channel */
void DMA1_Channel2_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF2)
	{
		DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CGIF2;

		DMA1_Channel2->CCR &= ~DMA_CCR2_EN;

		/* Restart DMA if there is still content to write */
		if (write_enq_val > write_deq_val)
			usart_enqueue_dma(write_enq_val - write_deq_val);
		else DMA1_Channel2->CNDTR = 0;
	}
}

uint32_t usart_write_possible()
{
	uint32_t write_real_deq_val = write_deq_val - DMA1_Channel2->CNDTR;
	return sizeof(write_queue) + write_enq_val - write_real_deq_val;
}

/* Writing routine */
uint32_t usart_write(const void* in, uint32_t size)
{
	/* Limit the size to the available size */
	uint32_t max_size = usart_write_possible();
	if (size > max_size) size = max_size;

	/* If no bytes can be transferred, bail out early */
	if (size == 0) return 0;

	/* Transfer the received data in one go if possible */
	uint32_t write_cnt = write_enq_val % sizeof(write_queue);
	if (write_cnt + size <= sizeof(write_queue))
		memcpy(write_queue+write_cnt, in, size);
	/* Transfer in two goes */
	else
	{
		uint32_t first_chunk = sizeof(write_queue) - write_cnt;
		memcpy(write_queue+write_cnt, in, first_chunk);
		memcpy(write_queue, in+first_chunk, size-first_chunk);
	}

	/* If the transfer is halted, restart it */
	if (DMA1_Channel2->CNDTR == 0)
	{
		USART3->SR = ~USART_SR_TC;
		usart_enqueue_dma(size);
	}

	write_enq_val += size;

	return size;
}

/* Write byte routine */
uint8_t usart_write_byte(uint8_t byte)
{
	/* Check if the write is possible */
	if (usart_write_possible() < 1) return 0;

	/* Write a single byte to the queue */
	uint32_t write_cnt = write_enq_val % sizeof(write_queue);
	write_queue[write_cnt] = byte;

	/* If the transfer is halted, restart it */
	if (DMA1_Channel2->CNDTR == 0)
	{
		USART3->SR = ~USART_SR_TC;
		usart_enqueue_dma(1);
	}

	write_enq_val++;

	return 1;
}
