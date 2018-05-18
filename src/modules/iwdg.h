/*
 * iwdg.h
 *
 *  Created on: 15 de nov de 2017
 *      Author: joaobapt
 */

#ifndef MODULES_IWDG_H_
#define MODULES_IWDG_H_

/**
 * Setup the independent watchdog to reset after ~150 ms
 */
void iwdg_init();

/**
 * Reset the watchdog timer, to prevent reset
 */
void iwdg_reset();

#endif /* MODULES_IWDG_H_ */
