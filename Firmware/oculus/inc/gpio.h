/************************************************************************************

Filename    :   gpio.h
Content     :   Tracker gpio header control
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>

void gpio_init(void);

void gpio_set_direction(uint8_t new_direction);

uint8_t gpio_get_direction(void);

void gpio_set_value(uint8_t new_value);

uint8_t gpio_get_value(void);

#endif /* _GPIO_H_ */