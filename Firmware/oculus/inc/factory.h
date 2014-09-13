/************************************************************************************

Filename    :   factory.h
Content     :   Tracker factory calibration interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _FACTORY_H_
#define _FACTORY_H_

#include "invensense.h"
#include "uart.h"

void factory_set_interface(uart_p interface);

void factory_set_command(uint8_t new_command);

void factory_update(invensense_data_p data);

#endif /* _FACTORY_H_ */
