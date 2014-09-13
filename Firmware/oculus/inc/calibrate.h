/************************************************************************************

Filename    :   calibrate.h
Content     :   Sensor calibration store, fetch, and apply
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _CALIBRATE_H_
#define _CALIBRATE_H_

#include "invensense.h"
#include "averaging.h"
#include <stdint.h>
#include <stdbool.h>

// size in bytes of the packed offset and scale structure
#define CALIBRATION_SIZE (8+8+8*3+8*3+sizeof(uint16_t))

typedef struct calibrate_struct {
    float temperature;
    float gyro_offset[3];
    float acc_offset[3];
    // scale and cross axis sensitivity
    float gyro_scale[3][3];
    float acc_scale[3][3];
} calibrate_s, *calibrate_p;

void calibrate_init(void);

bool calibrate_store_to_eeprom(void);

void calibrate_offset_from_average(averaging_p avg, bool store);

void calibrate_from_factory(calibrate_p cal);

// Get calibration parameters that were stored on the Tracker
void calibrate_get(uint8_t *buf);

// Store new calibration parameters on the Tracker
void calibrate_set(const uint8_t *buf);

void calibrate_apply(invensense_data_p data);

#endif /* _CALIBRATE_H_ */
