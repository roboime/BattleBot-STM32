/*
 * iwdg.c
 *
 *  Created on: 15 de nov de 2017
 *      Author: joaobapt
 */

#include "iwdg.h"
#include "stm32f10x.h"
#include "util.h"

/* Initialization routine */
void iwdg_init()
{
	// Stop the watchdog on breakpoint
	DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

	// Wait to set the watchdog prescaler
	while (IWDG->SR & IWDG_SR_PVU);

	// Enable access and set the prescaler
	IWDG->KR = 0x5555;
	IWDG->PR = 0; // divided by 4

	// Wait to set the watchdog reload
	while (IWDG->SR & IWDG_SR_RVU);

	// Enable access and set the reload
	IWDG->KR = 0x5555;
	IWDG->RLR = 1500 - 4;  // 1500 cycles * 4 (prescaler) / 40 MHz = 150 ms

	// Enable the watchdog
	IWDG->KR = 0xCCCC;
}

/* Reset the watchdog */
void iwdg_reset()
{
	IWDG->KR = 0xAAAA;
}
