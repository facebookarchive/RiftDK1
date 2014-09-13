/************************************************************************************

Filename    :   data_packet.h
Content     :   Tracker data packet generation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _DATA_PACKET_H_
#define _DATA_PACKET_H_
#include <stdint.h>
#include "invensense.h"
#include "hmc5983.h"

void data_packet_pack_sensor(uint8_t *buf, int32_t x, int32_t y, int32_t z);

void data_packet_unpack_sensor(const uint8_t *buf, int32_t *x, int32_t *y, int32_t *z);

void data_packet_update(uint16_t timestamp, invensense_data_p inv);

void data_packet_reset(void);

void data_packet_generate(uint16_t command_id, hmc_data_p hmc, bool use_ovr_coordinates);

void data_packet_send(void);

#endif /* _DATA_PACKET_H_ */
