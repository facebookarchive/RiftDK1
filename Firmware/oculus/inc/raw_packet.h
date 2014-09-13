/************************************************************************************

Filename    :   raw_packet.h
Content     :   Tracker raw packet sending
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _RAW_PACKET_H_
#define _RAW_PACKET_H_

#include <stdint.h>

void raw_packet_update(uint16_t timestamp, int16_t sample);

void raw_packet_generate(void);

void raw_packet_send(void);

#endif /* _RAW_PACKET_H_ */
