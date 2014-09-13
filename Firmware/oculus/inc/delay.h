/************************************************************************************

Filename    :   delay.h
Content     :   Millisecond delay function
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

void delay_init(void);
void delay_deinit(void);
void delay_ms(uint32_t ms);
void delay_update(void);

#endif /* _DELAY_H_ */