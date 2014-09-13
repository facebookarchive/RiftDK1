/************************************************************************************

Filename    :   data_packet.c
Content     :   Tracker data packet generation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "data_packet.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define DATA_PACKET_SIZE 62
#define SAMPLE_SIZE 16
#define SAMPLE_START 8

// sign extending trick from http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
struct {int32_t x:21;} s;

static uint8_t packet_buf[DATA_PACKET_SIZE] = {0};
static uint8_t num_samples = 0;
static uint16_t packet_timestamp = 0;
static float gyro_rem[3] = {0};
static float acc_rem[3] = {0};
static invensense_data_t samples[3] = {{0}};

static int32_t accumulate_cast(float in, float *rem)
{
    // Store the remainder when rounding from float to int and apply it
    // before the next rounding.
    in += *rem;

    float rounded = rintf(in);
    *rem = in - rounded;

    return (int32_t)rounded;
}

static void data_packet_add_average(invensense_data_p sample)
{
    // Just add here.  We will divide out to get the average before sending
    samples[0].gyro[0] += sample->gyro[0];
    samples[0].gyro[1] += sample->gyro[1];
    samples[0].gyro[2] += sample->gyro[2];
    samples[0].accel[0] += sample->accel[0];
    samples[0].accel[1] += sample->accel[1];
    samples[0].accel[2] += sample->accel[2];
    samples[0].temperature += sample->temperature;
}

static void data_packet_compute_average(void)
{
    // two of the samples separate
    float num_avg = num_samples - 2;

    // calculate the actual averages
    samples[0].gyro[0] /= num_avg;
    samples[0].gyro[1] /= num_avg;
    samples[0].gyro[2] /= num_avg;
    samples[0].accel[0] /= num_avg;
    samples[0].accel[1] /= num_avg;
    samples[0].accel[2] /= num_avg;
    samples[0].temperature /= num_avg;
}

void data_packet_pack_sensor(uint8_t *buf, int32_t x, int32_t y, int32_t z)
{
    buf[0] = x >> 13;
    buf[1] = x >> 5;
    buf[2] = (x << 3) | ((y >> 18) & 0x07);
    buf[3] = y >> 10;
    buf[4] = y >> 2;
    buf[5] = (y << 6) | ((z >> 15) & 0x3F);
    buf[6] = z >> 7;
    buf[7] = z << 1;
}

void data_packet_unpack_sensor(const uint8_t *buf, int32_t *x, int32_t *y, int32_t *z)
{
    *x = s.x = (buf[0] << 13) | (buf[1] << 5) | ((buf[2] & 0xF8) >> 3);
    *y = s.x = ((buf[2] & 0x07) << 18) | (buf[3] << 10) | (buf[4] << 2) |
                   ((buf[5] & 0xC0) >> 6);
    *z = s.x = ((buf[5] & 0x3F) << 15) | (buf[6] << 7) | (buf[7] >> 1);
}

void data_packet_update(uint16_t timestamp, invensense_data_p inv)
{
    // Mark the timestamp off the first sample in the buffer
    if (!num_samples) {
        packet_timestamp = timestamp;
    } else if (num_samples == 254) {
        // Dump the average at 254 samples and just keep the two separate ones we have
        num_samples = 2;
        packet_timestamp = timestamp - 2;
        // Clear the buffer that was being averaged to
        memset(samples, 0, sizeof(invensense_data_t));
    }

    // Slots are full, start averaging
    if (num_samples >= 3) {
        // Add sample 2 to the average
        data_packet_add_average(&samples[1]);

        // Move the third sample to the second sample spot
        memcpy(&samples[1], &samples[2], sizeof(invensense_data_t));
        // Move the new sample to the third spot
        memcpy(&samples[2], inv, sizeof(invensense_data_t));
    } else {
        // If we aren't full, copy into the first available slot
        memcpy(&samples[num_samples], inv, sizeof(invensense_data_t));
    }

    num_samples++;
}

void data_packet_reset(void)
{
    num_samples = 0;
    memset(samples, 0, sizeof(invensense_data_t));
}

void data_packet_generate(uint16_t command_id, hmc_data_p hmc, bool use_ovr_coordinates)
{
    if (num_samples > 3) data_packet_compute_average();

    packet_buf[0] = 1; // Report ID for the data packet
    packet_buf[1] = num_samples;
    *(uint16_t *)(packet_buf+2) = packet_timestamp;
    *(uint16_t *)(packet_buf+4) = command_id;
    *(int16_t *)(packet_buf+6) = (int16_t)lrintf(samples[0].temperature);

    for (int i = 0; i < (num_samples > 3 ? 3 : num_samples); i++) {
        // TODO: do we actually want to accumulate the remainders?
        int32_t ax = accumulate_cast(samples[i].accel[0], acc_rem);
        int32_t ay = accumulate_cast(samples[i].accel[1], acc_rem+1);
        int32_t az = accumulate_cast(samples[i].accel[2], acc_rem+2);
        int32_t gx = accumulate_cast(samples[i].gyro[0], gyro_rem);
        int32_t gy = accumulate_cast(samples[i].gyro[1], gyro_rem+1);
        int32_t gz = accumulate_cast(samples[i].gyro[2], gyro_rem+2);

        // Put the new sample in the first available spot
        if (use_ovr_coordinates) {
            data_packet_pack_sensor(packet_buf+SAMPLE_START+i*SAMPLE_SIZE, ax, az, -ay);
            data_packet_pack_sensor(packet_buf+SAMPLE_START+i*SAMPLE_SIZE+8, gx, gz, -gy);
        } else {
            data_packet_pack_sensor(packet_buf+SAMPLE_START+i*SAMPLE_SIZE, ax, ay, az);
            data_packet_pack_sensor(packet_buf+SAMPLE_START+i*SAMPLE_SIZE+8, gx, gy, gz);
        }
    }

    // Round the magnetometer, but don't accumulate cast, because consecutive
    // packets may have the same magnetometer sample
    if (use_ovr_coordinates) {
        *(int16_t *)(packet_buf+56) = (int16_t)lrintf(hmc->mag[0]);
        *(int16_t *)(packet_buf+58) = (int16_t)lrintf(hmc->mag[2]);
        *(int16_t *)(packet_buf+60) = (int16_t)lrintf(-hmc->mag[1]);
    } else {
        *(int16_t *)(packet_buf+56) = (int16_t)lrintf(hmc->mag[0]);
        *(int16_t *)(packet_buf+58) = (int16_t)lrintf(hmc->mag[1]);
        *(int16_t *)(packet_buf+60) = (int16_t)lrintf(hmc->mag[2]);
    }

    data_packet_reset();
}

void data_packet_send(void)
{
    USB_SIL_Write(EP1_IN, packet_buf, DATA_PACKET_SIZE);

    // Mark that the endpoint has valid data
    SetEPTxValid(ENDP1);
}
