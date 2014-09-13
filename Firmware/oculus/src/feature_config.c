/************************************************************************************

Filename    :   feature_config.c
Content     :   Tracker feature reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "feature_config.h"
#include <stdlib.h>
#include <string.h>
#include "platform_config.h"
#include "usb_core.h"
#include "usb_init.h"
#include "calibrate.h"
#include "invensense.h"
#include "hmc5983.h"
#include "eeprom.h"
#include "eeprom_table.h"
#include "gpio.h"

enum {
    FEATURE_CONFIG = 2,
    FEATURE_CALIBRATE = 3,
    FEATURE_RANGE = 4,
    FEATURE_REGISTER = 5,
    FEATURE_DFU = 6,
    FEATURE_GPIO = 7,
    FEATURE_KEEP_ALIVE = 8,
    FEATURE_DISPLAY_INFO = 9,
    FEATURE_SERIAL = 10
};

// Report ID and Command ID
#define FEATURE_HEADER_SIZE 3

#define FEATURE_CONFIG_SIZE (FEATURE_HEADER_SIZE+4)
#define FEATURE_CALIBRATE_SIZE (FEATURE_HEADER_SIZE+CALIBRATION_SIZE)
#define FEATURE_RANGE_SIZE (FEATURE_HEADER_SIZE+5)
#define FEATURE_REGISTER_SIZE (FEATURE_HEADER_SIZE+3)
#define FEATURE_DFU_SIZE (FEATURE_HEADER_SIZE+1)
#define FEATURE_GPIO_SIZE (FEATURE_HEADER_SIZE+2)
#define FEATURE_KEEP_ALIVE_SIZE (FEATURE_HEADER_SIZE+2)
#define FEATURE_DISPLAY_INFO_SIZE (FEATURE_HEADER_SIZE+53)
#define FEATURE_SERIAL_SIZE (FEATURE_HEADER_SIZE+12)

feature_config_s global_config = {0};
bool global_config_set = 0;
feature_calibrate_s global_calibrate = {0};
bool global_calibrate_set = 0;
feature_range_s global_range = {0};
bool global_range_set = 0;
feature_register_s global_reg = {0};
bool global_reg_set = 0;
feature_dfu_s global_dfu = {0};
bool global_dfu_set = 0;
feature_gpio_s global_gpio = {0};
feature_keep_alive_s global_keep_alive = {0};
bool global_keep_alive_set = 0;
feature_display_info_s global_display_info = {0};
feature_serial_s global_serial = {0};

// Most recent command ID globally
static uint16_t feature_config_command_id = 0;
bool got_command = 0;

// For the largest buffer we need
uint8_t feature_get_buf[FEATURE_CALIBRATE_SIZE];
uint8_t feature_set_buf[FEATURE_CALIBRATE_SIZE];

static void feature_config_pack_config(void)
{
    feature_get_buf[0] = FEATURE_CONFIG;
    *(uint16_t *)(feature_get_buf+1) = global_config.command_id;
    feature_get_buf[3] = global_config.use_raw |
                         (global_config.calibrate << 1) |
                         (global_config.use_calibration << 2) |
                         (global_config.autocalibration << 3) |
                         (global_config.motion_keep_alive << 4) |
                         (global_config.command_keep_alive << 5) |
                         (global_config.use_ovr_coordinates << 6);
    feature_get_buf[4] = global_config.interval;
    // For now, just hard code to 1000 Hz.  This is for forwards compatibility
    // with future trackers with different sample rates.
    *(uint16_t *)(feature_get_buf+5) = global_config.sample_rate = 1000;
}

static void feature_config_pack_range(void)
{
    feature_get_buf[0] = FEATURE_RANGE;
    *(uint16_t *)(feature_get_buf+1) = global_range.command_id;
    invensense_get_ranges(&global_range.accel_range, &global_range.gyro_range);
    feature_get_buf[3] = global_range.accel_range;
    *(uint16_t *)(feature_get_buf+4) = global_range.gyro_range;
    global_range.mag_range = hmc_get_range();
    *(uint16_t *)(feature_get_buf+6) = global_range.mag_range;
}

static void feature_config_pack_register(void)
{
    feature_get_buf[0] = FEATURE_REGISTER;
    *(uint16_t *)(feature_get_buf+1) = global_reg.command_id;
    feature_get_buf[3] = global_reg.device;
    feature_get_buf[4] = global_reg.reg;
    feature_get_buf[5] = global_reg.payload;
}

uint8_t *feature_config_getreport(uint16_t length)
{
    // Switch based on ReportID
    switch (pInformation->USBwValue0) {
        case FEATURE_CONFIG:
            // On the first callback, we supply the length
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_CONFIG_SIZE;
            } else {
                // On the second we return the filled buffer
                feature_config_pack_config();
                return feature_get_buf;
            }
            break;

        case FEATURE_CALIBRATE:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_CALIBRATE_SIZE;
            } else {
                feature_get_buf[0] = FEATURE_CALIBRATE;
                *(uint16_t *)(feature_get_buf+1) = global_calibrate.command_id;
                // Pack in the payload
                calibrate_get(feature_get_buf+3);
                // Calibrate is above 64 bytes, so we get called again with offset
                return feature_get_buf + pInformation->Ctrl_Info.Usb_wOffset;
            }
            break;

        case FEATURE_RANGE:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_RANGE_SIZE;
            } else {
                feature_config_pack_range();
                return feature_get_buf;
            }
            break;

        case FEATURE_REGISTER:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_REGISTER_SIZE;
            } else {
                feature_config_pack_register();
                return feature_get_buf;
            }
            break;

        case FEATURE_DFU:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_DFU_SIZE;
            } else {
                feature_get_buf[0] = FEATURE_DFU;
                *(uint16_t *)(feature_get_buf+1) = global_dfu.command_id;
                feature_get_buf[3] = global_dfu.use_dfu;
                return feature_get_buf;
            }
            break;

        case FEATURE_GPIO:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_GPIO_SIZE;
            } else {
                feature_get_buf[0] = FEATURE_GPIO;
                *(uint16_t *)(feature_get_buf+1) = global_gpio.command_id;
                feature_get_buf[3] = global_gpio.direction;
                feature_get_buf[4] = global_gpio.value = gpio_get_value();
                return feature_get_buf;
            }
            break;

        case FEATURE_KEEP_ALIVE:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_KEEP_ALIVE_SIZE;
            } else {
                feature_get_buf[0] = FEATURE_KEEP_ALIVE;
                *(uint16_t *)(feature_get_buf+1) = global_keep_alive.command_id;
                *(uint16_t *)(feature_get_buf+3) = global_keep_alive.keep_alive;
                return feature_get_buf;
            }
            break;

        case FEATURE_DISPLAY_INFO:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_DISPLAY_INFO_SIZE;
            } else {
                feature_get_buf[0] = FEATURE_DISPLAY_INFO;
                *(uint16_t *)(feature_get_buf+1) = global_display_info.command_id;
                // fill the actual parameters
                display_info_get(feature_get_buf+3, feature_get_buf+4);
                return feature_get_buf;
            }
            break;

        case FEATURE_SERIAL:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_SERIAL_SIZE;
            } else {
                feature_get_buf[0] = FEATURE_SERIAL;
                *(uint16_t *)(feature_get_buf+1) = global_serial.command_id;
                Get_SerialNum((char *)(feature_get_buf+3), 0);
                return feature_get_buf;
            }
            break;

        default:
            break;
    }

    return NULL;
}

uint8_t *feature_config_setreport(uint16_t length)
{
    // Switch based on ReportID
    switch (pInformation->USBwValue0) {
        case FEATURE_CONFIG:
            // On the first callback, we supply the length
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_CONFIG_SIZE;
            } else {
                // On the second, we return the buffer to fill
                return feature_set_buf;
            }
            break;

        case FEATURE_CALIBRATE:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_CALIBRATE_SIZE;
            } else {
                // Calibrate is above 64 bytes, so we get called again with offset
                return feature_set_buf + pInformation->Ctrl_Info.Usb_wOffset;
            }
            break;

        case FEATURE_RANGE:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_RANGE_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        case FEATURE_REGISTER:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_REGISTER_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        case FEATURE_DFU:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_DFU_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        case FEATURE_GPIO:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_GPIO_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        case FEATURE_KEEP_ALIVE:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_KEEP_ALIVE_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        case FEATURE_DISPLAY_INFO:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_DISPLAY_INFO_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        case FEATURE_SERIAL:
            if (!length) {
                pInformation->Ctrl_Info.Usb_wLength = FEATURE_SERIAL_SIZE;
            } else {
                return feature_set_buf;
            }
            break;

        default:
            break;
    }

    return NULL;
}

void feature_config_parse_report(void)
{
    // Get the latest set command ID regardless of report type
    feature_config_command_id = *(uint16_t *)(feature_set_buf+1);
    got_command = 1;

    switch (feature_set_buf[0]) {
        case FEATURE_CONFIG:
            global_config.command_id = *(uint16_t *)(feature_set_buf+1);
            global_config.use_raw = feature_set_buf[3] & 1;
            global_config.calibrate = feature_set_buf[3] & (1 << 1);
            global_config.use_calibration = feature_set_buf[3] & (1 << 2);
            global_config.autocalibration = feature_set_buf[3] & (1 << 3);
            global_config.motion_keep_alive = feature_set_buf[3] & (1 << 4);
            global_config.command_keep_alive = feature_set_buf[3] & (1 << 5);
            global_config.use_ovr_coordinates = feature_set_buf[3] & (1 << 6);
            global_config.interval = feature_set_buf[4];
            // Sample rate is forced to 1000 for now
            global_config.sample_rate = 1000; // *(uint16_t *)(feature_set_buf+5);
            global_config_set = 1;
            break;

        case FEATURE_CALIBRATE:
            // update the last command in on any feature report
            global_calibrate.command_id = *(uint16_t *)(feature_set_buf+1);
            // pass the payload of calibration data
            memcpy(global_calibrate.payload, feature_set_buf+FEATURE_HEADER_SIZE, CALIBRATION_SIZE);
            global_calibrate_set = 1;
            break;

        case FEATURE_RANGE:
            global_range.command_id = *(uint16_t *)(feature_set_buf+1);
            global_range.accel_range = invensense_closest_accel_range(feature_set_buf[3]);
            global_range.gyro_range = invensense_closest_gyro_range(*(uint16_t *)(feature_set_buf+4));
            global_range.mag_range = hmc_closest_range(*(uint16_t *)(feature_set_buf+6));
            global_range_set = 1;
            break;

        case FEATURE_REGISTER:
            global_reg.command_id = *(uint16_t *)(feature_set_buf+1);
            global_reg.device = feature_set_buf[3];
            global_reg.reg = feature_set_buf[4];
            global_reg.payload = feature_set_buf[5];
            global_reg_set = 1;
            break;

        case FEATURE_DFU:
            global_dfu.command_id = *(uint16_t *)(feature_set_buf+1);
            global_dfu.use_dfu = (feature_set_buf[3] == 1);
            global_dfu_set = 1;
            break;

        case FEATURE_GPIO:
            global_gpio.command_id = *(uint16_t *)(feature_set_buf+1);
            global_gpio.direction = feature_set_buf[3];
            gpio_set_direction(global_gpio.direction);
            gpio_set_value(feature_set_buf[4]);
            break;

        case FEATURE_KEEP_ALIVE:
            global_keep_alive.command_id = *(uint16_t *)(feature_set_buf+1);
            global_keep_alive.keep_alive = *(uint16_t *)(feature_set_buf+3);
            global_keep_alive_set = 1;
            break;

        case FEATURE_DISPLAY_INFO:
            global_display_info.command_id = *(uint16_t *)(feature_set_buf+1);
            display_info_set(*(uint8_t *)(feature_set_buf+3), (uint8_t *)(feature_set_buf+4));
            break;

        case FEATURE_SERIAL:
            global_serial.command_id = *(uint16_t *)(feature_set_buf+1);
            // Setting this switches the fake serial to the real one on all
            // future power ons.
            Set_Stored_Serial((char *)feature_set_buf+3);
            break;

        default:
            break;
    }
}

bool feature_config_got_command(bool peek)
{
    bool ret = got_command;

    if (!peek)
        got_command = 0;

    return ret;
}

uint16_t feature_config_latest_command_id(void)
{
    return feature_config_command_id;
}

void feature_config_set(feature_config_p config)
{
    memcpy(&global_config, config, sizeof(feature_config_s));
}

void feature_keep_alive_set(feature_keep_alive_p keep_alive)
{
    memcpy(&global_keep_alive, keep_alive, sizeof(feature_keep_alive_s));
}

bool feature_config_get(feature_config_p config)
{
    // TODO: protect this
    // Only mark as changed if we got a new packet and the contents differ from the last
    if (global_config_set && memcmp(config, &global_config, sizeof(feature_config_s))) {
        global_config_set = 0;
        memcpy(config, &global_config, sizeof(feature_config_s));
        return 1;
    }

    return 0;
}

bool feature_calibrate_get(feature_calibrate_p calibrate)
{
    // TODO: protect this
    if (global_calibrate_set &&  memcmp(calibrate, &global_calibrate, sizeof(feature_calibrate_s))) {
        global_calibrate_set = 0;
        memcpy(calibrate, &global_calibrate, sizeof(feature_calibrate_s));
        return 1;
    }

    return 0;
}

bool feature_range_get(feature_range_p range)
{
    // TODO: protect this
    if (global_range_set && memcmp(range, &global_range, sizeof(feature_range_s))) {
        global_range_set = 0;
        memcpy(range, &global_range, sizeof(feature_range_s));
        return 1;
    }

    return 0;
}

bool feature_register_get(feature_register_p reg)
{
    // TODO: protect this
    // A register could be set twice intentionally here, so don't condition on
    // the contents differing
    if (global_reg_set) {
        global_reg_set = 0;
        memcpy(reg, &global_reg, sizeof(feature_register_s));
        return 1;
    }

    return 0;
}

bool feature_dfu_get(feature_dfu_p dfu)
{
    // TODO: protect this
    if (global_dfu_set && memcmp(dfu, &global_dfu, sizeof(feature_dfu_s))) {
        global_dfu_set = 0;
        memcpy(dfu, &global_dfu, sizeof(feature_dfu_s));
        return 1;
    }

    return 0;
}

bool feature_keep_alive_get(feature_keep_alive_p keep_alive)
{
    // TODO: protect this
    // This report is lightweight, so just let it through if it is the same
    if (global_keep_alive_set) {
        global_keep_alive_set = 0;
        memcpy(keep_alive, &global_keep_alive, sizeof(feature_keep_alive_s));
        return 1;
    }

    return 0;
}
