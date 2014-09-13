/************************************************************************************

Filename    :   autocalibrate.c
Content     :   Gyro run time zero-rate calibration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "autocalibrate.h"
#include "calibrate.h"
#include "averaging.h"
#include <string.h>
#include <math.h>

#define AUTOCALIBRATE_NUM_SAMPLES 6000
#define AUTOCALIBRATE_LIMIT 3490.66f
#define AUTOCALIBRATE_NOISE 300.0f
#define AUTOCALIBRATE_ALPHA 0.4f

static uint32_t no_motion_count = 0;
static averaging_s avg = {0};
static float moving_avg[3] = {0.0f};

uint32_t autocalibrate_motion_count(void)
{
    return no_motion_count;
}

bool autocalibrate_update(invensense_data_p data, bool autocalibration)
{
    if (!avg.sample_count) {
        // start the calibration with the sample
        averaging_update(&avg, data);
    } else {
        uint8_t i;
        // check if any gyro axis is seeing motion
        for (i = 0; i < 3; i++) {
            // do a moving average to reject short term noise
            moving_avg[i] = data->gyro[i]*AUTOCALIBRATE_ALPHA + moving_avg[i]*(1.0f - AUTOCALIBRATE_ALPHA);

            // Make sure the absolute value is below what is likely motion
            if (fabsf(moving_avg[i]) < AUTOCALIBRATE_LIMIT) {
                float a = (float)(avg.gyro[i]/(double)avg.sample_count);
                // Make sure it is close enough to the current average
                // that it is probably noise and not motion
                if (fabsf(moving_avg[i] - a) < AUTOCALIBRATE_NOISE) {
                    continue;
                }
            }

            // if it failed, but had a reasonable number of samples already,
            // use it for the current offset, but don't store it permanently
            if (autocalibration && (avg.sample_count > (AUTOCALIBRATE_NUM_SAMPLES/2)))
                calibrate_offset_from_average(&avg, 0);

            // if it failed either threshold, reset calibration and return
            averaging_reset(&avg);
            no_motion_count = 0;
            return 0;
        }

        // add the probable no motion sample
        averaging_update(&avg, data);
        no_motion_count++;

        // After ~5 seconds of no motion, use the average as the new zero
        // rate offset
        if (autocalibration && (avg.sample_count > AUTOCALIBRATE_NUM_SAMPLES)) {
            calibrate_offset_from_average(&avg, 1);
            averaging_reset(&avg);

            return 1;
        }
    }

    return 0;
}
