/************************************************************************************

Filename    :   feature_config.h
Content     :   Tracker feature reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _FEATURE_CONFIG_H_
#define _FEATURE_CONFIG_H_

#include "hw_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "calibrate.h"
#include "display_info.h"

typedef struct feature_config_struct {
    uint16_t command_id;
    bool use_raw;
    bool calibrate;
    bool use_calibration;
    bool autocalibration;
    bool motion_keep_alive;
    bool command_keep_alive;
    bool use_ovr_coordinates;
    uint8_t interval;
    uint16_t sample_rate;
} feature_config_s, *feature_config_p;

typedef struct feature_calibrate_struct {
    uint16_t command_id;
    uint8_t payload[CALIBRATION_SIZE];
} feature_calibrate_s, *feature_calibrate_p;

typedef struct feature_range_struct {
    uint16_t command_id;
    uint8_t accel_range;
    uint16_t gyro_range;
    uint16_t mag_range;
} feature_range_s, *feature_range_p;

typedef struct feature_reg_struct {
    uint16_t command_id;
    uint8_t device;
    uint8_t reg;
    uint8_t payload;
} feature_register_s, *feature_register_p;

typedef struct feature_dfu_struct {
    uint16_t command_id;
    bool use_dfu;
} feature_dfu_s, *feature_dfu_p;

typedef struct feature_gpio_struct {
    uint16_t command_id;
    uint8_t direction;
    uint8_t value;
} feature_gpio_s, *feature_gpio_p;

typedef struct feature_keep_alive_struct {
    uint16_t command_id;
    uint16_t keep_alive;
} feature_keep_alive_s, *feature_keep_alive_p;

typedef struct feature_display_info_struct {
    uint16_t command_id;
    // The rest of the struct is fetched from display_info functions
} feature_display_info_s, *feature_display_info_p;

typedef struct feature_serial_struct {
    uint16_t command_id;
    // The rest of the struct is fetched from functions
} feature_serial_s, *feature_serial_p;

enum {
    FEATURE_REGISTER_INV = 1,
    FEATURE_REGISTER_HMC = 2
};

// Callbacks for USB
uint8_t *feature_config_getreport(uint16_t length);
uint8_t *feature_config_setreport(uint16_t length);

void feature_config_parse_report(void);
bool feature_config_got_command(bool peek);
uint16_t feature_config_latest_command_id(void);

void feature_config_set(feature_config_p config);
void feature_keep_alive_set(feature_keep_alive_p keep_alive);

bool feature_config_get(feature_config_p config);
bool feature_calibrate_get(feature_calibrate_p calibrate);
bool feature_range_get(feature_range_p range);
bool feature_register_get(feature_register_p reg);
bool feature_dfu_get(feature_dfu_p dfu);
bool feature_keep_alive_get(feature_keep_alive_p keep_alive);

#endif /* _FEATURE_CONFIG_H_ */
