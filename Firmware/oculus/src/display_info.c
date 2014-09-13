/************************************************************************************

Filename    :   display_info.c
Content     :   Headset display configuration storage
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "display_info.h"
#include "eeprom_table.h"
#include "eeprom.h"
#include <stdbool.h>
#include <string.h>

// distortion types
// 0 - zeroed out entirely
// 1 - zeroing out just the K values

static const uint8_t default_distortion_type = 1;

static void generate_default_display_info(uint8_t *buf)
{
#ifdef DK_HD_LG
    buf[3] = 0x01; // DistortionType
    *(uint16_t *)(buf+4) = 1920; // ResolutionX
    *(uint16_t *)(buf+6) = 1080; // ResolutionY
    *(uint32_t *)(buf+8) = 120960; // DisplayX
    *(uint32_t *)(buf+12) = 68040; // DisplayY
    *(uint32_t *)(buf+16) = 68040/2; // CenterV
    *(uint32_t *)(buf+24) = 49800; // LensDistanceL
    *(uint32_t *)(buf+28) = 49800; // LensDistanceR
    // For now, just keep the distortion at 0's
    memset(buf+32, 0, 6*sizeof(float));
#elif defined(DK_HD_SHARP)
    // The Sharp 5.85" needs distortion read from firmware
    buf[3] = 0x03; // DistortionType
    *(uint16_t *)(buf+4) = 1920; // ResolutionX
    *(uint16_t *)(buf+6) = 1080; // ResolutionY
    *(uint32_t *)(buf+8) = 129600; // DisplayX
    *(uint32_t *)(buf+12) = 72900; // DisplayY
    *(uint32_t *)(buf+16) = 72900/2; // CenterV
    *(uint32_t *)(buf+24) = 40000; // LensDistanceL
    *(uint32_t *)(buf+28) = 40000; // LensDistanceR
    *(float *)(buf+32) = 1.0; // K1
    *(float *)(buf+36) = 0.22; // K2
    *(float *)(buf+40) = 0.13; // K3
    *(float *)(buf+44) = 0.02; // K4
    memset(buf+48, 0, 2*sizeof(float));
#else /* DK_HD_LG */
    buf[3] = 0x01; // DistortionType
    *(uint16_t *)(buf+4) = 1280; // ResolutionX
    *(uint16_t *)(buf+6) = 800; // ResolutionY
    *(uint32_t *)(buf+8) = 149760; // DisplayX
    *(uint32_t *)(buf+12) = 93600; // DisplayY
    *(uint32_t *)(buf+16) = 93600/2; // CenterV
    *(uint32_t *)(buf+24) = 49800; // LensDistanceL
    *(uint32_t *)(buf+28) = 49800; // LensDistanceR
    // For now, just keep the distortion at 0's
    memset(buf+32, 0, 6*sizeof(float));
#endif /* DK_HD_LG */

    *(uint32_t *)(buf+20) = 63500; // LensSeparation
}

void display_info_get(uint8_t *distortion_type, uint8_t *buf)
{
    bool found_display_info = 1;
    uint16_t temp_distortion = default_distortion_type;
    EE_ReadVariable(EE_DISPLAY_INFO_DISTORTION_TYPE, &temp_distortion);
    *distortion_type = temp_distortion;

    uint16_t *readbuf = (uint16_t *)buf;

    for (uint8_t i = 0; i < EE_DISPLAY_INFO_SIZE; i++) {
        if (EE_ReadVariable(EE_DISPLAY_INFO_START+i, &readbuf[i])) {
            found_display_info = 0;
            break;
        }
    }

    // if display info stuff wasn't found in EEPROM, load the defaults
    if (!found_display_info) {
        // offset back to the start of the buffer
        generate_default_display_info(buf-4);
    }
}

void display_info_set(uint8_t distortion_type, uint8_t *buf)
{
    FLASH_Unlock();

    EE_WriteVariable(EE_DISPLAY_INFO_DISTORTION_TYPE, distortion_type);

    uint16_t *writebuf = (uint16_t *)buf;

    for (uint8_t i = 0; i < EE_DISPLAY_INFO_SIZE; i++) {
        // Write in the parameters two bytes at a time
        EE_WriteVariable(EE_DISPLAY_INFO_START+i, writebuf[i]);
    }

    FLASH_Lock();
}
