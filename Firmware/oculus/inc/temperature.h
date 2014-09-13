/************************************************************************************

Filename    :   temperature.h
Content     :   Gyro run time temperature compensation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

#include <stdbool.h>

typedef struct offset_bin_struct {
    float temperature_target;
    float temperature_actual;
    float temperature_stored;
    float offset[3];
} offset_bin_s, *offset_bin_p;

void temperature_init(offset_bin_p default_offset);

void temperature_update_bins(offset_bin_p new_offset, bool store);

void temperature_calculate_offset(offset_bin_p current_offset, float temperature);

void temperature_apply_offset(float *gyro);

#endif /* _TEMPERATURE_H_ */
