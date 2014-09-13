/************************************************************************************

Filename    :   calibrate.c
Content     :   Sensor calibration store, fetch, and apply
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "calibrate.h"
#include "eeprom.h"
#include "eeprom_table.h"
#include "data_packet.h"
#include "temperature.h"
#include "averaging.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define SENSOR_MAX ((1 << 20) - 1)

#define OFFSET_TO_INT(x) ((int32_t)lrintf(x))
#define SCALE_TO_INT(x) ((int32_t)lrintf((x - 1.0f) * (float)SENSOR_MAX))
#define CROSSAXIS_TO_INT(x) ((int32_t)lrintf(x * (float)SENSOR_MAX))

#define INT_TO_OFFSET(x) ((float)x)
#define INT_TO_SCALE(x) (((float)x/(float)SENSOR_MAX) + 1.0f)
#define INT_TO_CROSSAXIS(x) ((float)x/(float)SENSOR_MAX)

enum {
    CALIBRATE_NONE = 0,
    CALIBRATE_DK1 = 1, // This wasn't actually ever stored
    CALIBRATE_HD = 2, // Fixes cross axis gyro sign error
};

// Factory calibration data
static calibrate_s g_cal;
static bool stored_gyro_offset = 0;
static offset_bin_s current_offset;

// Load old calibrated accel scale parameters from before cross axis matrix
static void init_legacy_acc(void)
{
    float_pack_t pack;
    memset(g_cal.acc_scale, 0, sizeof(g_cal.acc_scale));

    if (EE_ReadVariable(EE_ACC_SC_X_L, &pack.s[0]) || EE_ReadVariable(EE_ACC_SC_X_H, &pack.s[1]))
        g_cal.acc_scale[0][0] = 1.0;
    else
        g_cal.acc_scale[0][0] = pack.f;

    if (EE_ReadVariable(EE_ACC_SC_Y_L, &pack.s[0]) || EE_ReadVariable(EE_ACC_SC_Y_H, &pack.s[1]))
        g_cal.acc_scale[1][1] = 1.0;
    else
        g_cal.acc_scale[1][1] = pack.f;

    if (EE_ReadVariable(EE_ACC_SC_Z_L, &pack.s[0]) || EE_ReadVariable(EE_ACC_SC_Z_H, &pack.s[1]))
        g_cal.acc_scale[2][2] = 1.0;
    else
        g_cal.acc_scale[2][2] = pack.f;
}

// We had a bug in DK1's calibration where the cross axis elements on the gyro
// matrix have a sign flipped.  This is a one time fix to correct and store
// the matrix
static void fix_cross_axis(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            // Only non-diagonal members had the bug
            if (i != j)
                g_cal.gyro_scale[i][j] = -g_cal.gyro_scale[i][j];
        }
    }

    // Store the corrected calibration version so we don't run this every time
    calibrate_store_to_eeprom();
}

void calibrate_init(void)
{
    float_pack_t pack;
    bool found_acc_matrix = 0;

    // Make FLASH writable while initializing the virtual EEPROM
    FLASH_Unlock();
    EE_Init();
    FLASH_Lock();

    // Set defaults of no change if any of the values are not found
    if (EE_ReadVariable(EE_GYRO_OFF_X_L, &pack.s[0]) || EE_ReadVariable(EE_GYRO_OFF_X_H, &pack.s[1]))
        g_cal.gyro_offset[0] = 0.0;
    else
        g_cal.gyro_offset[0] = pack.f;

    if (EE_ReadVariable(EE_GYRO_OFF_Y_L, &pack.s[0]) || EE_ReadVariable(EE_GYRO_OFF_Y_H, &pack.s[1]))
        g_cal.gyro_offset[1] = 0.0;
    else
        g_cal.gyro_offset[1] = pack.f;

    if (EE_ReadVariable(EE_GYRO_OFF_Z_L, &pack.s[0]) || EE_ReadVariable(EE_GYRO_OFF_Z_H, &pack.s[1]))
        g_cal.gyro_offset[2] = 0.0;
    else {
        g_cal.gyro_offset[2] = pack.f;
        // mark if we were able to successfully load a gyro offset from EEPROM
        stored_gyro_offset = 1;
    }

    // Read the 3x3 scale and cross axis matrix
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            if (EE_ReadVariable(EE_GYRO_SCALE_START + (i*3 + j)*2, &pack.s[0]) || EE_ReadVariable(EE_GYRO_SCALE_START + (i*3 + j)*2 + 1, &pack.s[1]))
                // Fill missing values with the identity matrix
                g_cal.gyro_scale[i][j] = (i == j) ? 1.0 : 0.0;
            else
                g_cal.gyro_scale[i][j] = pack.f;
        }
    }

    // Read the 3x3 scale and cross axis matrix
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            if (EE_ReadVariable(EE_ACC_SCALE_START + (i*3 + j)*2, &pack.s[0]) || EE_ReadVariable(EE_ACC_SCALE_START + (i*3 + j)*2 + 1, &pack.s[1])) {
                // Fill missing values with the identity matrix
                g_cal.acc_scale[i][j] = (i == j) ? 1.0 : 0.0;
            } else {
                g_cal.acc_scale[i][j] = pack.f;
                found_acc_matrix = 1;
            }
        }
    }

    if (!found_acc_matrix)
        init_legacy_acc();

    if (EE_ReadVariable(EE_ACC_OFF_X_L, &pack.s[0]) || EE_ReadVariable(EE_ACC_OFF_X_H, &pack.s[1]))
        g_cal.acc_offset[0] = 0.0;
    else
        g_cal.acc_offset[0] = pack.f;

    if (EE_ReadVariable(EE_ACC_OFF_Y_L, &pack.s[0]) || EE_ReadVariable(EE_ACC_OFF_Y_H, &pack.s[1]))
        g_cal.acc_offset[1] = 0.0;
    else
        g_cal.acc_offset[1] = pack.f;

    if (EE_ReadVariable(EE_ACC_OFF_Z_L, &pack.s[0]) || EE_ReadVariable(EE_ACC_OFF_Z_H, &pack.s[1]))
        g_cal.acc_offset[2] = 0.0;
    else
        g_cal.acc_offset[2] = pack.f;

    if (EE_ReadVariable(EE_TEMPERATURE_L, &pack.s[0]) || EE_ReadVariable(EE_TEMPERATURE_H, &pack.s[1]))
        g_cal.temperature = 0.0;  // Just pick a fake temperature
    else
        g_cal.temperature = pack.f;

    // If we have calibration, and the calibration was performed with the cross
    // axis sign error, fix it
    if (found_acc_matrix) {
        uint16_t version = 0;
        EE_ReadVariable(EE_CAL_VERSION, &version);
        // CALIBRATE_HD and newer have the fix
        if (version < CALIBRATE_HD) {
            fix_cross_axis();
        }
    }

    // Start at the factory calibrated gyro offset
    memcpy(current_offset.offset, g_cal.gyro_offset, sizeof(float)*3);
    current_offset.temperature_actual = g_cal.temperature;

    // Initialize the gyro offset temperature bins
    temperature_init(&current_offset);
}

bool calibrate_store_to_eeprom(void)
{
    float_pack_t pack;

    FLASH_Unlock();

    // Virtual EEPROM only allows 16 bit words, so split the floats into two int16s
    pack.f = g_cal.gyro_offset[0];
    EE_WriteVariable(EE_GYRO_OFF_X_L, pack.s[0]);
    EE_WriteVariable(EE_GYRO_OFF_X_H, pack.s[1]);

    pack.f = g_cal.gyro_offset[1];
    EE_WriteVariable(EE_GYRO_OFF_Y_L, pack.s[0]);
    EE_WriteVariable(EE_GYRO_OFF_Y_H, pack.s[1]);

    pack.f = g_cal.gyro_offset[2];
    EE_WriteVariable(EE_GYRO_OFF_Z_L, pack.s[0]);
    EE_WriteVariable(EE_GYRO_OFF_Z_H, pack.s[1]);

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            pack.f = g_cal.gyro_scale[i][j];
            EE_WriteVariable(EE_GYRO_SCALE_START + (i*3 + j)*2, pack.s[0]);
            EE_WriteVariable(EE_GYRO_SCALE_START + (i*3 + j)*2 + 1, pack.s[1]);
        }
    }

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            pack.f = g_cal.acc_scale[i][j];
            EE_WriteVariable(EE_ACC_SCALE_START + (i*3 + j)*2, pack.s[0]);
            EE_WriteVariable(EE_ACC_SCALE_START + (i*3 + j)*2 + 1, pack.s[1]);
        }
    }

    pack.f = g_cal.acc_offset[0];
    EE_WriteVariable(EE_ACC_OFF_X_L, pack.s[0]);
    EE_WriteVariable(EE_ACC_OFF_X_H, pack.s[1]);

    pack.f = g_cal.acc_offset[1];
    EE_WriteVariable(EE_ACC_OFF_Y_L, pack.s[0]);
    EE_WriteVariable(EE_ACC_OFF_Y_H, pack.s[1]);

    pack.f = g_cal.acc_offset[2];
    EE_WriteVariable(EE_ACC_OFF_Z_L, pack.s[0]);
    EE_WriteVariable(EE_ACC_OFF_Z_H, pack.s[1]);

    pack.f = g_cal.temperature;
    EE_WriteVariable(EE_TEMPERATURE_L, pack.s[0]);
    EE_WriteVariable(EE_TEMPERATURE_H, pack.s[1]);

    // Store which version of calibration was performed
    EE_WriteVariable(EE_CAL_VERSION, CALIBRATE_HD);

    FLASH_Lock();

    stored_gyro_offset = 1;

    return 1;
}

void calibrate_offset_from_average(averaging_p avg, bool store)
{
    averaging_compute(avg, current_offset.offset, NULL, &current_offset.temperature_actual);

    // If no calibration has ever been done, save the autocalibrated
    // data to EEPROM
    if (store && !stored_gyro_offset) {
        memcpy(g_cal.gyro_offset, current_offset.offset, sizeof(float)*3);
        g_cal.temperature = current_offset.temperature_actual;
        calibrate_store_to_eeprom();
    }

    // Store the calibration data permanently if it is better than one of the
    // existing calibration points
    temperature_update_bins(&current_offset, store);
}

void calibrate_from_factory(calibrate_p cal)
{
    memcpy(&g_cal, cal, sizeof(calibrate_s));

    // burn the parameters to nonvolatile storage
    calibrate_store_to_eeprom();

    // don't store the factory calibration in temperature bins permanently
    memcpy(current_offset.offset, g_cal.gyro_offset, sizeof(float)*3);
    current_offset.temperature_actual = g_cal.temperature;
    temperature_update_bins(&current_offset, 0);
}

// Get calibration parameters that were stored on the Tracker
void calibrate_get(uint8_t *buf)
{
    int32_t x, y, z;

    x = OFFSET_TO_INT(g_cal.acc_offset[0]);
    y = OFFSET_TO_INT(g_cal.acc_offset[1]);
    z = OFFSET_TO_INT(g_cal.acc_offset[2]);
    data_packet_pack_sensor(buf, x, y, z);

    x = OFFSET_TO_INT(g_cal.gyro_offset[0]);
    y = OFFSET_TO_INT(g_cal.gyro_offset[1]);
    z = OFFSET_TO_INT(g_cal.gyro_offset[2]);
    data_packet_pack_sensor(buf+8, x, y, z);

    // load the scale and cross axis matrix
    for (uint8_t i = 0; i < 3; i++) {
        int32_t axis[3];
        for (uint8_t j = 0; j < 3; j++) {
            // The diagonal is scale, the rest is cross axis
            axis[j] = (i == j) ? SCALE_TO_INT(g_cal.acc_scale[i][j]) : CROSSAXIS_TO_INT(g_cal.acc_scale[i][j]);
        }
        data_packet_pack_sensor(buf+16+i*8, axis[0], axis[1], axis[2]);
    }

    // load the scale and cross axis matrix
    for (uint8_t i = 0; i < 3; i++) {
        int32_t axis[3];
        for (uint8_t j = 0; j < 3; j++) {
            // The diagonal is scale, the rest is cross axis
            axis[j] = (i == j) ? SCALE_TO_INT(g_cal.gyro_scale[i][j]) : CROSSAXIS_TO_INT(g_cal.gyro_scale[i][j]);
        }
        data_packet_pack_sensor(buf+40+i*8, axis[0], axis[1], axis[2]);
    }

    *(int16_t *)(buf+64) = (int16_t)OFFSET_TO_INT(g_cal.temperature);
}

// Store new calibration parameters on the Tracker
void calibrate_set(const uint8_t *buf)
{
    int32_t x, y, z;

    data_packet_unpack_sensor(buf, &x, &y, &z);
    g_cal.acc_offset[0] = INT_TO_OFFSET(x);
    g_cal.acc_offset[1] = INT_TO_OFFSET(y);
    g_cal.acc_offset[2] = INT_TO_OFFSET(z);

    data_packet_unpack_sensor(buf+8, &x, &y, &z);
    g_cal.gyro_offset[0] = INT_TO_OFFSET(x);
    g_cal.gyro_offset[1] = INT_TO_OFFSET(y);
    g_cal.gyro_offset[2] = INT_TO_OFFSET(z);

    // parse the scale and cross axis matrix
    for (uint8_t i = 0; i < 3; i++) {
        int32_t axis[3];
        data_packet_unpack_sensor(buf+16+i*8, &axis[0], &axis[1], &axis[2]);
        for (uint8_t j = 0; j < 3; j++) {
            g_cal.acc_scale[i][j] = (i == j) ? INT_TO_SCALE(axis[j]) : INT_TO_CROSSAXIS(axis[j]);
        }
    }

    // parse the scale and cross axis matrix
    for (uint8_t i = 0; i < 3; i++) {
        int32_t axis[3];
        data_packet_unpack_sensor(buf+40+i*8, &axis[0], &axis[1], &axis[2]);
        for (uint8_t j = 0; j < 3; j++) {
            g_cal.gyro_scale[i][j] = (i == j) ? INT_TO_SCALE(axis[j]) : INT_TO_CROSSAXIS(axis[j]);
        }
    }

    g_cal.temperature = (float)*(int16_t *)(buf+64);

    // burn the parameters to nonvolatile storage
    calibrate_store_to_eeprom();

    // also check the new offset to see if it fits any temperature bins
    memcpy(current_offset.offset, g_cal.gyro_offset, sizeof(float)*3);
    current_offset.temperature_actual = g_cal.temperature;
    temperature_update_bins(&current_offset, 1);
}

void calibrate_apply(invensense_data_p data)
{
    float gyro_axes[3] = {0.0};
    float accel_axes[3] = {0.0};

    // Get the gyro offset for the current temperature
    temperature_calculate_offset(&current_offset, data->temperature);

    // Apply the offsets before doing cross axis scaling
    temperature_apply_offset(data->gyro);

    data->accel[0] -= g_cal.acc_offset[0];
    data->accel[1] -= g_cal.acc_offset[1];
    data->accel[2] -= g_cal.acc_offset[2];

    // take cross axis sensitivity into account in the scaling
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            gyro_axes[i] += data->gyro[j]*g_cal.gyro_scale[i][j];
            accel_axes[i] += data->accel[j]*g_cal.acc_scale[i][j];
        }
    }

    memcpy(data->gyro, gyro_axes, sizeof(float)*3);
    memcpy(data->accel, accel_axes, sizeof(float)*3);
}
