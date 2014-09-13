/************************************************************************************

Filename    :   display_info.h
Content     :   Headset display configuration storage
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _DISPLAY_INFO_H_
#define _DISPLAY_INFO_H_

#include <stdint.h>

void display_info_get(uint8_t *distortion_type, uint8_t *buf);

void display_info_set(uint8_t distortion_type, uint8_t *buf);

#endif /* _DISPLAY_INFO_H_ */
