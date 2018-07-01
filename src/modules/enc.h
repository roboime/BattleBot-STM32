/*
 * enc.h
 *
 *  Created on: 1 de jul de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_ENC_H_
#define MODULES_ENC_H_

#include <stdint.h>

void enc_init();
void enc_update();
int32_t enc_speed(uint32_t motor);

#endif /* MODULES_ENC_H_ */
