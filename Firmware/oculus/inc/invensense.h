/************************************************************************************

Filename    :   invensense.h
Content     :   Invensense MPU-6xxx configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _INVENSENSE_H_
#define _INVENSENSE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct invensense_data_struct {
    float temperature;
    float accel[3];
    float gyro[3];
} invensense_data_t, *invensense_data_p;

int8_t invensense_init(void);
void invensense_sleep(void);
void invensense_deinit(void);
bool invensense_motion_interrupt(void);
int8_t invensense_read(invensense_data_p data, bool raw);
uint8_t invensense_closest_accel_range(uint8_t accel);
uint16_t invensense_closest_gyro_range(uint16_t gyro);
void invensense_set_ranges(uint8_t accel, uint16_t gyro);
void invensense_get_ranges(uint8_t *accel, uint16_t *gyro);
void invensense_set_register(uint8_t reg, uint8_t payload);

#endif /* _INVENSENSE_H_ */
