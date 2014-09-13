/************************************************************************************

Filename    :   autocalibrate.h
Content     :   Gyro run time zero-rate calibration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _AUTOCALIBRATE_H_
#define _AUTOCALIBRATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "invensense.h"

uint32_t autocalibrate_motion_count(void);

bool autocalibrate_update(invensense_data_p data, bool autocalibration);

#endif /* _AUTOCALIBRATE_H_ */
