/*
 * recv.h
 *
 *  Created on: 4 de nov de 2017
 *      Author: joaobapt
 *
 *  Description: this module provide functions to interface with the receiver and extract
 *               the channel data from it
 */

#ifndef MODULES_RECV_H_
#define MODULES_RECV_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	int32_t in_low, in_mid, in_high;
	int32_t out_low, out_mid, out_high;
} normalization_params;

/**
 * Initializes the receiver's module.
 */
void recv_init();

/**
 * Enables the reception of command signals from the receiver.
 */
void recv_enable();

/**
 * Disables the reception.
 */
void recv_disable();

/**
 * Runs the receptor's update routine, like median sorting and such
 */
void recv_update();

/**
 * The number of channels available to extract from the signals
 */
uint32_t recv_num_channels();

/**
 * Sets the normalization parameters for a channel.
 * \param [in] ch The number of the channel to set the normalization parameter
 * \param [in] params The struct with the normalization parameters for this channel
 */
void recv_set_normalization_params(uint32_t ch, const normalization_params* params);

/**
 * The value received from each channel, already calibrated and normalized
 * from -65535 to 65535
 * \param [in] ch The number of the channel to get the value from
 */
int32_t recv_channel(uint32_t ch);

/**
 * The raw value received from each channel
 * \param [in] ch The number of the channel to get the value from
 */
int32_t recv_raw_channel(uint32_t ch);

/**
 * Check if the receiver is connected and transmiting frames
 */
bool recv_is_connected();

/**
 * Check if a new frame is received
 */
bool recv_new_frame();

#endif /* MODULES_RECV_H_ */
