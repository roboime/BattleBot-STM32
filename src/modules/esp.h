/*
 * esp.h
 *
 *  Created on: 16 de jun de 2018
 *      Author: joaobapt
 */

#ifndef MODULES_ESP_H_
#define MODULES_ESP_H_

#include <stdint.h>

/* Initializes the ESP communications module */
void esp_init();

/* Sends the payload specified by data with size bytes through the
   ESP protocol. */
void esp_send(void* data, uint8_t size);

/* Seek for new commands */
void esp_recv_commands();

/* Returns the number of commands received that are not yet processed */
uint32_t esp_new_commands();

/* Removes one command from the list and memcpy's it to the destination area
   Returns the command's size, which is guaranteed to be at most 160 */
uint32_t esp_next_command(void* data);

#endif /* MODULES_ESP_H_ */
