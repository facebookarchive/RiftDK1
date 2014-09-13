/************************************************************************************

Filename    :   user_calibrate.c
Content     :   Gyro manual zero rate calibration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "user_calibrate.h"
#include "averaging.h"
#include "calibrate.h"

static averaging_s avg = {0};

void user_calibrate_reset(void)
{
    averaging_reset(&avg);
}

void user_calibrate_update(invensense_data_p data)
{
    averaging_update(&avg, data);
}

void user_calibrate_end(void)
{
    calibrate_offset_from_average(&avg, 1);
    calibrate_store_to_eeprom();
    user_calibrate_reset();
}
