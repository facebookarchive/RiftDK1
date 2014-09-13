/************************************************************************************

Filename    :   user_calibrate.h
Content     :   Gyro manual zero rate calibration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _USER_CALIBRATE_H_
#define _USER_CALIBRATE_H_

#include "invensense.h"

void user_calibrate_reset(void);

void user_calibrate_update(invensense_data_p data);

void user_calibrate_end(void);

#endif /* _USER_CALIBRATE_H_ */
