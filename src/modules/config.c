/*
 * config.c
 *
 *  Created on: 27 de jun de 2018
 *      Author: joaobapt
 */

#include "stm32f10x.h"
#include "config.h"
#include "util.h"

#include <string.h>

#define CONFIG_PAGE (void*)(FLASH_BASE + 62*1024)
#define SAVED_ID 0x19276840

static config_struct cur;

static void config_init()
{
	memset(&cur, 0, sizeof(cur));
	cur.id = SAVED_ID;
}

void config_load()
{
	memcpy(&cur, CONFIG_PAGE, sizeof(config_struct));
	if (cur.id != SAVED_ID) config_init();
}

config_struct* config_cur()
{
	return &cur;
}

void config_set_flag(config_flag flag, int32_t val)
{
	BIT_BANDING_SRAM(cur.flags, flag) = val;
}

int32_t config_get_flag(config_flag flag)
{
	return BIT_BANDING_SRAM(cur.flags, flag);
}

//
// BEGINNING OF THE config_save() ROUTINE AND AUXILIARY ROUTINES
// (yes, flash programming can be daunting)
//

static void unlock_flash()
{
	if (FLASH->CR & FLASH_CR_LOCK)
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
}

static void config_hsi()
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

void __attribute__((noreturn)) config_save()
{
	/* Disable all peripherals */
	RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;
	RCC->APB2ENR = 0;
	RCC->APB1ENR = 0;

	/* Reset RCC to its original state */
	RCC->CR |= 1;
	RCC->CFGR &= 0xF8FF0000;
	RCC->CR &= 0xFEF6FFFF;
	RCC->CR &= 0xFFFBFFFF;
	RCC->CFGR &= 0xFF80FFFF;
	RCC->CIR = 0x009F0000;

	/* Disable all interrupts */
	__disable_irq();

	/* Configure HSI */
	config_hsi();

	/* Erase page 62 */
	// Clear error flags
	FLASH->SR = ~(FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP);

	uint16_t* page = CONFIG_PAGE;
	uint16_t* config = (uint16_t*)&cur;

	// Erase FLASH page
	unlock_flash();
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = (uint32_t)CONFIG_PAGE;
	FLASH->CR |= FLASH_CR_STRT;
	while (FLASH->SR & FLASH_SR_BSY) IWDG->KR = 0xAAAA;
	FLASH->CR &= ~FLASH_CR_PER;

	// Check sanity of FLASH erasure
	for (int i = 0; i < 512; i++)
		if (page[i] != 0xFFFF)
			goto error;

	// Proceed to write the halfwords
	uint32_t num_halfwords = (sizeof(config_struct)+1)/2;

	// Program FLASH sequence
	for (int i = 0; i < num_halfwords; i++)
	{
		unlock_flash();
		FLASH->CR |= FLASH_CR_PG;
		page[i] = config[i];
		while (FLASH->SR & FLASH_SR_BSY) IWDG->KR = 0xAAAA;
		FLASH->CR &= ~FLASH_CR_PG;

		if (page[i] != config[i]) goto error;
	}

error:
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
