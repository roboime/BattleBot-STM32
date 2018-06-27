/*
 * bootloader.c
 *
 *  Created on: 18 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"
#include <stdint.h>

//#define DONT_UPLOAD_BOOTLOADER

#ifndef DONT_UPLOAD_BOOTLOADER

#define BOOTLOADER_TEXT __attribute__((section(".bootloader.text")))

#pragma GCC optimize("Os")

static void BOOTLOADER_TEXT unlock_flash()
{
	if (FLASH->CR & FLASH_CR_LOCK)
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

static void BOOTLOADER_TEXT config_hsi()
{
	// Enable PLL with HSI, 8x
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) IWDG->KR = 0xAAAA;

	// Configure for HSI
	FLASH->ACR |= FLASH_ACR_PRFTBE;
	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
	FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2;

	// Configure PLL for 64 MHz operation
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL)) | RCC_CFGR_PLLMULL16;

	// Enable and wait for PLL to complete
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY)) IWDG->KR = 0xAAAA;

	// Select PLL as clock source
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08) IWDG->KR = 0xAAAA;
}

void BOOTLOADER_TEXT __attribute__((noreturn)) do_bootloader()
{
	// Enable HSI with PLL (64 MHz max)
	config_hsi();

	// Set appropriate USART baud rate
	USART3->BRR = ((uint32_t)USART3->BRR * 8) / 9;

	// Turn LED on
	GPIOC->ODR = 0;

	// Wait for the number of pages to be sent
	uint32_t read_pages = 0;
	uint32_t num_pages = 0;

	// Send upload enter bytes
	for (int i = 0; i < 9; i++)
	{
		while (!(USART3->SR & USART_SR_TXE)) IWDG->KR = 0xAAAA;
		USART3->DR = 0x66;
	}

	// Security sentinel
	for (;;)
	{
		while (!(USART3->SR & USART_SR_RXNE)) IWDG->KR = 0xAAAA;
		uint8_t byte = USART3->DR;

		if (read_pages == 9)
		{
			num_pages = byte;
			break;
		}
		else if (byte == 0x97) read_pages++;
		else read_pages = 0;
	}

	// Send confirmation bytes
	for (int i = 0; i < 9; i++)
	{
		while (!(USART3->SR & USART_SR_TXE)) IWDG->KR = 0xAAAA;
		USART3->DR = 0x63;
	}

	// Blink LED
	GPIOC->ODR ^= GPIO_ODR_ODR13;

	// Wait for all pages, one by one
	for (int j = 0; j < num_pages; j++)
	{
		// Security sentinel
		int read_sent = 0;
		for (;;)
		{
			while (!(USART3->SR & USART_SR_RXNE)) IWDG->KR = 0xAAAA;
			uint8_t byte = USART3->DR;

			if (byte == (uint8_t)((uint8_t)0x33 + (uint8_t)3 * (uint8_t)j))
			{
				if (++read_sent == 12) break;
			}
			else read_sent = 0;
		}

		char page[1024];
		// Read all bytes for the pages
		for (int i = 0; i < 1024; i++)
		{
			while (!(USART3->SR & USART_SR_RXNE)) IWDG->KR = 0xAAAA;
			page[i] = USART3->DR;
		}

		// Clear error flags
		FLASH->SR = ~(FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP);

		volatile uint16_t* flash = (volatile uint16_t*)0;
		uint16_t* page_w = (uint16_t*)page;

		// Erase FLASH page
		unlock_flash();
		FLASH->CR |= FLASH_CR_PER;
		FLASH->AR = FLASH_BASE + 1024*j;
		FLASH->CR |= FLASH_CR_STRT;
		while (FLASH->SR & FLASH_SR_BSY) IWDG->KR = 0xAAAA;
		FLASH->CR &= ~FLASH_CR_PER;

		// Check sanity of FLASH erasure
		for (int i = 0; i < 512; i++)
			if (flash[512*j+i] != 0xFFFF)
				goto error;

		// Program FLASH sequence
		for (int i = 0; i < 512; i++)
		{
			unlock_flash();
			FLASH->CR |= FLASH_CR_PG;
			flash[512*j+i] = page_w[i];
			while (FLASH->SR & FLASH_SR_BSY) IWDG->KR = 0xAAAA;
			FLASH->CR &= ~FLASH_CR_PG;

			if (flash[512*j+i] != page_w[i])
				goto error;
		}

		// Send confirmation bytes
		for (int i = 0; i < 9; i++)
		{
			while (!(USART3->SR & USART_SR_TXE)) IWDG->KR = 0xAAAA;
			USART3->DR = (uint8_t)((uint8_t)0x97 - (uint8_t)7*(uint8_t)j);
		}

		// Blink LED
		GPIOC->ODR ^= GPIO_ODR_ODR13;

		// Continue
		continue;

	error:
		// Send error bytes
		for (int i = 0; i < 9; i++)
		{
			while (!(USART3->SR & USART_SR_TXE)) IWDG->KR = 0xAAAA;
			USART3->DR = (uint8_t)((uint8_t)0x2F - (uint8_t)7*(uint8_t)j);
		}

		// Restart bootloader
		do_bootloader();
	}

	// Wait to send everything on USART
	while (!(USART3->SR & USART_SR_TC)) IWDG->KR = 0xAAAA;

	// Lock the FLASH
	FLASH->CR |= FLASH_CR_LOCK;

	// Reset hardware (copied from NVIC_SystemReset())
	__DSB();
	SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
				 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
				 SCB_AIRCR_SYSRESETREQ_Msk);
	__DSB();
	while(1);
}

#endif
