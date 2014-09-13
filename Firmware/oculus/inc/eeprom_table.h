/************************************************************************************

Filename    :   eeprom_table.h
Content     :   Map of EEPROM data
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _EEPROM_TABLE_H_
#define _EEPROM_TABLE_H_

#include <stdint.h>

#define EE_NUM_ADDR ((uint8_t)1)
#define EE_NUM_OFFSET_BIN (7)
#define EE_OFFSET_BIN_SIZE (4*2)
#define EE_SCALE_MATRIX_SIZE (3*3*2)
#define EE_DISPLAY_INFO_SIZE (52/2)
#define EE_SERIAL_NUM_SIZE (12/2)
#define EE_MAX_CONTIGUOUS (EE_CAL_VERSION+1)

// The virtual EEPROM implementation uses 16 bit values, so union to a float
typedef union
{
  float f;
  uint16_t s[2];
} float_pack_t;

enum {
    EE_GYRO_OFF_X_L,
    EE_GYRO_OFF_X_H,
    EE_GYRO_OFF_Y_L,
    EE_GYRO_OFF_Y_H,
    EE_GYRO_OFF_Z_L,
    EE_GYRO_OFF_Z_H,
    EE_GYRO_SCALE_START,
    EE_GYRO_SCALE_END = EE_GYRO_SCALE_START + EE_SCALE_MATRIX_SIZE - 1,
    EE_ACC_OFF_X_L,
    EE_ACC_OFF_X_H,
    EE_ACC_OFF_Y_L,
    EE_ACC_OFF_Y_H,
    EE_ACC_OFF_Z_L,
    EE_ACC_OFF_Z_H,
    EE_ACC_SC_X_L,
    EE_ACC_SC_X_H,
    EE_ACC_SC_Y_L,
    EE_ACC_SC_Y_H,
    EE_ACC_SC_Z_L,
    EE_ACC_SC_Z_H,
    EE_TEMPERATURE_L,
    EE_TEMPERATURE_H,
    EE_OFFSET_BIN_START,
    EE_OFFSET_BIN_END = EE_OFFSET_BIN_START + EE_NUM_OFFSET_BIN * EE_OFFSET_BIN_SIZE - 1,
    EE_ACC_SCALE_START,
    EE_ACC_SCALE_END = EE_ACC_SCALE_START + EE_SCALE_MATRIX_SIZE - 1,
    EE_DISPLAY_INFO_DISTORTION_TYPE,
    EE_DISPLAY_INFO_START,
    EE_DISPLAY_INFO_END = EE_DISPLAY_INFO_START + EE_DISPLAY_INFO_SIZE - 1,
    EE_SERIAL_SET,
    EE_SERIAL_NUM_START,
    EE_SERIAL_NUM_END = EE_SERIAL_NUM_START + EE_SERIAL_NUM_SIZE - 1,
    EE_CAL_VERSION,
    EE_DFU_BOOT = 0x0100,
};

// Only list addresses that aren't contiguous from 0 here
static const uint16_t ee_addr_table[EE_NUM_ADDR] = {
    EE_DFU_BOOT,
};

#endif /* _EEPROM_TABLE_H_ */
