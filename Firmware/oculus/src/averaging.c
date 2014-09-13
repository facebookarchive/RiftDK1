/************************************************************************************

Filename    :   averaging.c
Content     :   Gyro sample averaging helper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "averaging.h"
#include <string.h>

void averaging_reset(averaging_p avg)
{
    memset(avg, 0, sizeof(averaging_s));
}

void averaging_update(averaging_p avg, invensense_data_p data)
{
    avg->gyro[0] += (double)data->gyro[0];
    avg->gyro[1] += (double)data->gyro[1];
    avg->gyro[2] += (double)data->gyro[2];
    avg->acc[0] += (double)data->accel[0];
    avg->acc[1] += (double)data->accel[1];
    avg->acc[2] += (double)data->accel[2];
    avg->temp += (double)data->temperature;

    avg->sample_count++;
}

uint32_t averaging_count(averaging_p avg)
{
    return avg->sample_count;
}

void averaging_compute(averaging_p avg, float *gyro, float *accel, float *temp)
{
    if (gyro) {
        gyro[0] = (float)(avg->gyro[0]/(double)avg->sample_count);
        gyro[1] = (float)(avg->gyro[1]/(double)avg->sample_count);
        gyro[2] = (float)(avg->gyro[2]/(double)avg->sample_count);
    }

    if (accel) {
        accel[0] = (float)(avg->acc[0]/(double)avg->sample_count);
        accel[1] = (float)(avg->acc[1]/(double)avg->sample_count);
        accel[2] = (float)(avg->acc[2]/(double)avg->sample_count);
    }

    if (temp)
        *temp = (float)(avg->temp/(double)avg->sample_count);
}
