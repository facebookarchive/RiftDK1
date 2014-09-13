/************************************************************************************

Filename    :   temperature.c
Content     :   Gyro run time temperature compensation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "temperature.h"
#include "eeprom.h"
#include "eeprom_table.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

typedef struct interpolate_struct {
    float slope[3];
    float intercept[3];
} interpolate_s, *interpolate_p;

// Gyro offset calibration data in temperature bins
static offset_bin_s offset_bins[EE_NUM_OFFSET_BIN];
static offset_bin_s temp_offset = {0.0};
static interpolate_s interp = {{0.0}};

// These values are all in centidegrees celcius
#define OFFSET_BIN_BASE (1000.0f)
#define OFFSET_BIN_WIDTH (500.0f)
#define TEMP_THRESHOLD (100.0f)
#define BIN_NOT_FOUND (255)

void temperature_init(offset_bin_p default_offset)
{
    float bin_temperature = OFFSET_BIN_BASE;
    float_pack_t pack;

    // Read out all of the gyro offset bins from different temperatures
    for (uint8_t i = 0; i < EE_NUM_OFFSET_BIN; i++) {
        bool valid = 1;

        // Read each of the 3 axes of gyro
        for (uint8_t j = 0; j < 3; j++) {
            if (EE_ReadVariable(EE_OFFSET_BIN_START + i*EE_OFFSET_BIN_SIZE + j*2, &pack.s[0]) ||
                EE_ReadVariable(EE_OFFSET_BIN_START + i*EE_OFFSET_BIN_SIZE + j*2 + 1, &pack.s[1])) {
                offset_bins[i].offset[j] = 0.0;
                valid = 0;
                break;
            } else
                offset_bins[i].offset[j] = pack.f;
        }

        if (valid) {
            if (EE_ReadVariable(EE_OFFSET_BIN_START + i*EE_OFFSET_BIN_SIZE + 3*2, &pack.s[0]) ||
                EE_ReadVariable(EE_OFFSET_BIN_START + i*EE_OFFSET_BIN_SIZE + 3*2 + 1, &pack.s[1])) {
                offset_bins[i].temperature_actual = offset_bins[i].temperature_stored = 0.0;
                valid = 0;
            } else
                offset_bins[i].temperature_actual = offset_bins[i].temperature_stored = pack.f;
        }

        // If the bin doesn't exist, just fill it with the factory data
        if (!valid) {
            memcpy(&offset_bins[i].offset, &default_offset->offset, sizeof(float)*3);
            // Only use the factory temperature in the appropriate bin
            if (fabsf(default_offset->temperature_actual - bin_temperature) <= OFFSET_BIN_WIDTH/2.0f)
                offset_bins[i].temperature_actual = default_offset->temperature_actual;
            else
                offset_bins[i].temperature_actual = 0.0;
        }

        offset_bins[i].temperature_target = bin_temperature;
        bin_temperature += OFFSET_BIN_WIDTH;
    }
}

static void temperature_store_bin(uint8_t bin)
{
    float_pack_t pack;
    FLASH_Unlock();

    // Commit the updated bin to EEPROM
    for (uint8_t i = 0; i < 3; i++) {
        pack.f = offset_bins[bin].offset[i];
        EE_WriteVariable(EE_OFFSET_BIN_START + bin*EE_OFFSET_BIN_SIZE + i*2, pack.s[0]);
        EE_WriteVariable(EE_OFFSET_BIN_START + bin*EE_OFFSET_BIN_SIZE + i*2 + 1, pack.s[1]);
    }

    pack.f = offset_bins[bin].temperature_actual;
    EE_WriteVariable(EE_OFFSET_BIN_START + bin*EE_OFFSET_BIN_SIZE + 3*2, pack.s[0]);
    EE_WriteVariable(EE_OFFSET_BIN_START + bin*EE_OFFSET_BIN_SIZE + 3*2 + 1, pack.s[1]);

    FLASH_Lock();
}

void temperature_update_bins(offset_bin_p new_offset, bool store)
{
    // If this is closer to the target temp in any of our current temperature bins, store it
    for (uint8_t i = 0; i < EE_NUM_OFFSET_BIN; i++) {
        float new_diff = fabsf(new_offset->temperature_actual - offset_bins[i].temperature_target);
        // Make sure it is inside of the bin
        if (new_diff <= OFFSET_BIN_WIDTH/2.0f) {
            float old_diff = fabsf(offset_bins[i].temperature_actual - offset_bins[i].temperature_target);
            // Make sure it is closer than the old value
            if (new_diff < old_diff) {
                memcpy(offset_bins[i].offset, new_offset->offset, sizeof(float)*3);
                offset_bins[i].temperature_actual = new_offset->temperature_actual;

                float stored_diff = fabsf(offset_bins[i].temperature_actual - offset_bins[i].temperature_stored);
                // only burn the EEPROM if there is a real change
                if (store && (stored_diff > (TEMP_THRESHOLD/10.0f))) {
                    offset_bins[i].temperature_stored = offset_bins[i].temperature_actual;
                    temperature_store_bin(i);
                }
            }
        }
    }
}

static void temperature_find_closest_bins(float temperature, uint8_t *closest, uint8_t *second)
{
    float closest_diff = OFFSET_BIN_WIDTH*EE_NUM_OFFSET_BIN + OFFSET_BIN_BASE;
    float second_diff = closest_diff;

    for (uint8_t i = 0; i < EE_NUM_OFFSET_BIN; i++) {
        float new_diff = fabsf(temperature - offset_bins[i].temperature_actual);
        if (new_diff < closest_diff) {
            // Previous best is now second best
            *second = *closest;
            second_diff = closest_diff;

            *closest = i;
            closest_diff = new_diff;
        } else if ((fabsf(offset_bins[i].temperature_actual - offset_bins[*closest].temperature_actual) > TEMP_THRESHOLD) && (new_diff < second_diff)) {
            // Try to find a second point that is spaced away from the first point
            // to avoid noise causing a crazy slope
            *second = i;
            second_diff = new_diff;
        }
    }
}

static void temperature_generate_interpolate(float temperature)
{
    uint8_t closest = BIN_NOT_FOUND;
    uint8_t second = BIN_NOT_FOUND;

    temperature_find_closest_bins(temperature, &closest, &second);;

    // If we only have one bin, the offset is constant
    if (second == BIN_NOT_FOUND) {
        for (uint8_t i = 0; i < 3; i++) {
            interp.slope[i] = 0.0f;
            interp.intercept[i] = offset_bins[closest].offset[i];
        }
    } else {
        // Otherwise linearly interpolate between the two closest bins
        float temp_diff = offset_bins[closest].temperature_actual - offset_bins[second].temperature_actual;

        for (uint8_t i = 0; i < 3; i++) {
            interp.slope[i] = (offset_bins[closest].offset[i] - offset_bins[second].offset[i])/temp_diff;
            interp.intercept[i] = offset_bins[closest].offset[i] - interp.slope[i]*offset_bins[closest].temperature_actual;
        }
    }
}

void temperature_calculate_offset(offset_bin_p current_offset, float temperature)
{
    // Recalculate the interpolation when the new temperature is more than the
    // threshold away from the last calculation
    if ((fabsf(temp_offset.temperature_actual - temperature) > TEMP_THRESHOLD/2.0f)) {
        temperature_generate_interpolate(temperature);
        temp_offset.temperature_actual = temperature;
    }

    // prefer the autoaveraged offset over temperature generated ones when it is
    // close enough
    if (fabsf(temperature - current_offset->temperature_actual) > OFFSET_BIN_WIDTH/2.0f) {
        for (uint8_t i = 0; i < 3; i++) {
            temp_offset.offset[i] = interp.slope[i]*temperature + interp.intercept[i];
        }
    } else {
        // still apply the slope when we use autocalibrated offset
        for (uint8_t i = 0; i < 3; i++) {
            temp_offset.offset[i] = current_offset->offset[i] + interp.slope[i]*(temperature - current_offset->temperature_actual);
        }
    }
}

void temperature_apply_offset(float *gyro)
{
    gyro[0] -= temp_offset.offset[0];
    gyro[1] -= temp_offset.offset[1];
    gyro[2] -= temp_offset.offset[2];
}
