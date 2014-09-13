/************************************************************************************

Filename    :   averaging.h
Content     :   Gyro sample averaging helper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _AVERAGING_H_
#define _AVERAGING_H_

#include <stdint.h>
#include "invensense.h"

typedef struct averaging_struct {
    uint32_t sample_count;
    double gyro[3];
    double acc[3];
    double temp;
} averaging_s, *averaging_p;

void averaging_reset(averaging_p avg);

void averaging_update(averaging_p avg, invensense_data_p data);

uint32_t averaging_count(averaging_p avg);

void averaging_compute(averaging_p avg, float *gyro, float *acc, float *temp);

#endif /* _AVERAGING_H_ */
