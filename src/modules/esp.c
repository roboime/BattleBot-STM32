/*
 * esp.c
 *
 *  Created on: 16 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"
#include <stdbool.h>
#include <string.h>

#include "esp.h"
#include "usart.h"

#define COMMAND_QUEUE_SIZE 8
#define MIN(a,b) ((a)<(b)?(a):(b))

static const char sentinel[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* The command queue is implemented as a circular buffer, with two "pointers" */
static char commands[COMMAND_QUEUE_SIZE][160];
static uint32_t command_sizes[COMMAND_QUEUE_SIZE];
static uint32_t cmd_deq_val, cmd_enq_val;
static uint32_t read_size, read_upd, rem_size;
static bool last_enq;

/* Declare in advance bootloader routine */
static __attribute__((noreturn)) void jump_to_bootloader();

void esp_init()
{
	cmd_deq_val = cmd_enq_val = 0;
	read_size = 0;
	read_upd = 0;
	rem_size = 0;
	last_enq = false;
}

/* Send routine */
void esp_send(void* data, uint8_t size)
{
	usart_write(sentinel, sizeof(sentinel));
	usart_write(&size, sizeof(size));
	usart_write(data, size);
}

/* Update routine */
void esp_recv_commands()
{
	uint32_t read_bytes;
	while ((read_bytes = usart_read_available()) > 0)
	{
		if (rem_size > 0)
		{
			uint32_t to_read = MIN(read_bytes, rem_size);
			usart_read(commands[cmd_enq_val]+command_sizes[cmd_enq_val], to_read);
			command_sizes[cmd_enq_val] += to_read;
			rem_size -= to_read;

			if (rem_size == 0)
			{
				cmd_enq_val = (cmd_enq_val + 1) % COMMAND_QUEUE_SIZE;
				last_enq = true;
			}
		}
		else
		{
			uint8_t byte = usart_read_byte();
			if (!(last_enq && cmd_enq_val == cmd_deq_val))
			{
				if (read_size == 5)
				{
					rem_size = byte;
					read_size = 0;
				}
				else if (byte == 0xAA)
				{
					read_size++;
					read_upd = 0;
				}
				else if (byte == 0x77)
				{
					read_size = 0;
					if (++read_upd == 32)
						jump_to_bootloader();
				}
				else read_size = 0;
			}
		}
	}
}

uint32_t esp_new_commands()
{
	/* If both values points to the same location and the last operation was an enqueue
	   the queue is full */
	if (cmd_enq_val == cmd_deq_val)
		return last_enq ? COMMAND_QUEUE_SIZE : 0;
	else if (cmd_enq_val > cmd_deq_val) /* Return the modular difference between both pointers */
		return cmd_enq_val - cmd_deq_val;
	else return COMMAND_QUEUE_SIZE - cmd_deq_val + cmd_enq_val;
}

uint32_t esp_next_command(void* data)
{
	/* Check if there are commands */
	if (esp_new_commands() < 1) return 0;

	uint32_t size = command_sizes[cmd_deq_val];
	memcpy(data, commands[cmd_deq_val], size);

	cmd_deq_val = (cmd_deq_val + 1) % COMMAND_QUEUE_SIZE;
	last_enq = false;

	return size;
}

extern char __bootloader_load_begin[], __bootloader_load_end[], __bootloader_begin[];
void __attribute__((section(".bootloader.text"), noreturn)) do_bootloader();

/* Bootloader routine */
static __attribute__((noreturn)) void jump_to_bootloader()
{
	/* Disable all peripherals except for GPIOB, GPIOC and USART3 */
	RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;
	RCC->APB2ENR = RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
	RCC->APB1ENR = RCC_APB1ENR_USART3EN;

	/* Reset RCC to its original state */
	RCC->CR |= 1;
	RCC->CFGR &= 0xF8FF0000;
	RCC->CR &= 0xFEF6FFFF;
	RCC->CR &= 0xFFFBFFFF;
	RCC->CFGR &= 0xFF80FFFF;
	RCC->CIR = 0x009F0000;

	/* Disable all interrupts */
	__disable_irq();

	/* Go to the bootloader */
	do_bootloader();
}
