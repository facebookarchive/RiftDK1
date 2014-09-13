/************************************************************************************

Filename    :   main.c
Content     :   Main loop for Tracker
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "hw_config.h"
#include "usb_lib.h"
#include "spi.h"
#include "invensense.h"
#include "hmc5983.h"
#include "delay.h"
#include "data_packet.h"
#include "feature_config.h"
#include "calibrate.h"
#include "autocalibrate.h"
#include "user_calibrate.h"
#include "factory.h"
#include "eeprom.h"
#include "eeprom_table.h"
#include "usb_pwr.h"
#include "uart.h"

static uint16_t last_command_id = 0;
static bool user_calibrating = 0;
static bool factory_calibrating = 0;
static feature_config_s config = {0};
static feature_calibrate_s calibrate = {0};
static feature_range_s range = {0};
static feature_register_s reg = {0};
static feature_dfu_s dfu = {0};
static feature_keep_alive_s keep_alive = {0};
static uart_s factory = {0};

// TODO: handle the count differently
__IO uint32_t int_count_inv = 0;
__IO uint32_t int_count_hmc = 0;
__IO uint8_t ep1_ready = 1;
__IO uint8_t exit_stop_mode = 1;

static inline void handle_feature_config()
{
    last_command_id = feature_config_latest_command_id();

    // Grab a feature config at the start so it doesn't change
    if (feature_config_get(&config)) {
        // dump partially stored packets when configuration changes
        data_packet_reset();

        // Handle the internal zero rate calibration process
        if (config.calibrate && !user_calibrating) {
            // start the calibration process
            user_calibrating = 1;
            user_calibrate_reset();
        } else if (user_calibrating && !config.calibrate) {
            // finish the calibration process
            user_calibrating = 0;
            user_calibrate_end();
        }
    }

    if (feature_calibrate_get(&calibrate)) {
        // dump partially stored packets when calibration changes
        data_packet_reset();
        calibrate_set(calibrate.payload);
    }

    if (feature_range_get(&range)) {
        // dump partially stored packets when the sensor ranges change
        data_packet_reset();
        invensense_set_ranges(range.accel_range, range.gyro_range);
        hmc_set_range(range.mag_range);
    }

    if (feature_register_get(&reg)) {
        switch (reg.device) {
            case FEATURE_REGISTER_INV:
                // dump partially stored packets only when gyro registers change
                data_packet_reset();
                invensense_set_register(reg.reg, reg.payload);
                break;

            case FEATURE_REGISTER_HMC:
                hmc_set_register(reg.reg, reg.payload);
                break;

            default:
                break;
        }
    }

    if (feature_dfu_get(&dfu)) {
        // Set the parameter in EEPROM so it can be read on boot
        FLASH_Unlock();
        EE_WriteVariable(EE_DFU_BOOT, dfu.use_dfu);
        FLASH_Lock();

        // Restart the device if we are told to
        if (dfu.use_dfu)
            Reset_Device();
    }

    // Just updates the interval used for keep_alive
    feature_keep_alive_get(&keep_alive);
}

int main(void)
{
    // Shut off JTAG and keep SWD on, needs to happen right after reset
    Configure_Debug();

    delay_init();
    // The MPU-6000 needs 100ms to stabilize before accessing registers
    // The HMC5983 needs 50ms to bring up its analog end
    delay_ms(100);

    // Initialize pins and interrupts
    Set_System();

    Enable_EXTInterrupts();

    // if the UARTEN pin is pulled high, it means we are in the factory
    // calibration rig
#ifdef UARTEN_PIN
    factory_calibrating = GPIO_ReadInputDataBit(UARTEN_PORT, UARTEN_PIN);
#endif /* UARTEN_PIN */

    // Set default configuration parameters
    // Default to 500 Hz
    config.interval = 1;
    // Default to command keep alive with a 10 second timeout
    config.command_keep_alive = 1;
    keep_alive.keep_alive = 10000;
    // Default to calibration on and automatic when not factory calibrating
    config.use_calibration = 1;
    config.autocalibration = 1;
    // Set the feature report versions to the default values
    feature_config_set(&config);
    feature_keep_alive_set(&keep_alive);

    // Get the sensors running
    Sensor_Init();

    // Load the calibration data from EEPROM
    calibrate_init();

    // Configure the USB hardware
    USB_Interrupts_Config();
    Set_USBClock();
    USB_Init();

    invensense_data_t inv_data = {0};
    hmc_data_t hmc_data = {{0}};
    uint32_t target = int_count_inv + 1;
    uint32_t last_hmc = 0;
    uint16_t timestamp = 0;
    // make this 16 bit to avoid wrapping before a chance to send occurs
    uint16_t interval_count = 0;
    uint32_t last_command_count = 0;

    if (factory_calibrating) {
        // Only enable UART if we are in the factory
#ifdef FACTORY_PORT
        uart_init(&factory, FACTORY_PORT, FACTORY_SPEED, FACTORY_GPIO_PORT, FACTORY_TX, FACTORY_RX);
        factory_set_interface(&factory);
#endif /* FACTORY_PORT */
    }

    while (1) {
        // A sort of primitive select() on the things we care to wake up for:
        // 0. If we are not suspended or resuming from suspend, and
        // 1. The gyro data ready interrupt has triggered
        // 2. USB got a suspend or resume event
        // 3. A Feature Report came in over USB
        // 4. If we are factory calibrating, a command came in over UART
        // Shut off the CPU clock while waiting
        while ((!exit_stop_mode &&
                !factory_calibrating &&
                (bDeviceState == SUSPENDED)) ||
               ((int_count_inv < target) &&
                !feature_config_got_command(1) &&
                (!factory_calibrating || !uart_available(&factory)))) {

            if ((bDeviceState == SUSPENDED) && !exit_stop_mode && !factory_calibrating) {
                Sensor_Sleep();
                // FIXME: Why doesn't resuming from STOP work?
                // Shut off all the clocks
                Enter_Stop();
                // Turn the clocks back on
                Wake_From_Stop();
                Sensor_Init();
                // Exiting USB Suspend mode due to Tracker motion
                if (exit_stop_mode == 2)
                    Resume(RESUME_INTERNAL);
            } else {
                __WFI();
            }
        }

        // Handle factory calibration commands
        if (factory_calibrating && uart_available(&factory)) {
            factory_set_command(uart_read(&factory));
        }

        // handle feature reports coming in
        if (feature_config_got_command(0)) {
            last_command_count = 0;
        }

        handle_feature_config();

        // Don't do any data reading if the gyro hasn't interrupted yet
        if (int_count_inv < target) continue;
        // Reached the interrupt target, so set the next one for 1ms away
        target++;

        // Read the sensors
        invensense_read(&inv_data, config.use_raw);
        timestamp = int_count_inv;
        // Only read the magnetometer on the first go or if it changed
        if (!last_hmc || (last_hmc != int_count_hmc)) {
            hmc_read(&hmc_data, config.use_raw);
            last_hmc = int_count_hmc;
        }

        // Manual, factory, and automatic calibration are mutually exclusive
        if (factory_calibrating)
            factory_update(&inv_data);
        else if (user_calibrating)
            user_calibrate_update(&inv_data);
        else
            autocalibrate_update(&inv_data, config.autocalibration);

        interval_count++;
        last_command_count++;

        // Only proceed with updating the packet if USB is enabled
        if (bDeviceState == CONFIGURED) {
            if (config.use_calibration) calibrate_apply(&inv_data);

            // Store the latest gyro/accel data
            data_packet_update(timestamp, &inv_data);

            // Load data into the hardware USB buffer if the conditions are met:
            // 1. Previous packet send completed
            // 2. If the device is configured to send on slower intervals, that interval has passed
            // 3. If motion keep alive is on, there has been motion within the last keep_alive.keep_alive milliseconds
            // 4. If command keep alive is on, there has been a feature report set within keep_alive.keep_alive milliseconds
            if (ep1_ready &&
                (interval_count > config.interval) &&
                (!config.motion_keep_alive || (autocalibrate_motion_count() <= keep_alive.keep_alive)) &&
                (!config.command_keep_alive || (last_command_count <= keep_alive.keep_alive))) {
                data_packet_generate(last_command_id, &hmc_data, config.use_ovr_coordinates);

                data_packet_send();
                ep1_ready = 0;
                interval_count = 0;
            }
        }
    }
}
