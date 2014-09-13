/************************************************************************************

Filename    :   hmc5983.c
Content     :   Honeywell HMC5983 interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "hmc5983.h"
#include "stm32f10x.h"
#include "hmc5983_register_map.h"
#include "delay.h"
#include "platform_config.h"
#include "spi.h"
#include <math.h>
#include <string.h>

// bit 7 for read, 6 for address incrementing
#define R(a) (0xC0 | a)

#define ALPHA (0.4f)
#define SCALE (1.461538461538462f) // 1.9/1.3

#define NUM_RANGES 8
static const uint16_t ranges[NUM_RANGES] = {880, 1300, 1900, 2500, 4000, 4700, 5600, 8100};
static const uint8_t bitmasks[NUM_RANGES] = {GN_0_88, GN_1_3, GN_1_9, GN_2_5, GN_4_0, GN_4_7, GN_5_6, GN_8_1};
static const float conversions[NUM_RANGES] = {GN_SCALE_0_88, GN_SCALE_1_3, GN_SCALE_1_9, GN_SCALE_2_5, GN_SCALE_4_0, GN_SCALE_4_7, GN_SCALE_5_6, GN_SCALE_8_1};

typedef struct hmc_struct {
	spi_t spi;
  uint8_t range;
  bool suspended;
  float avg_mag;
  uint32_t avg_count;
} hmc_t, *hmc_p;

static hmc_t g_hmc = {{0}};

int8_t hmc_init(void)
{
    g_hmc.suspended = 0;
    // note that this dev board has only one SPI, so we are re-initing at ~4 MHz to use for both sensors
    spi_init(&g_hmc.spi, HMC_SPI, HMC_SPI_SPEED, HMC_SPI_RCC, HMC_SPI_GPIO, HMC_SPI_GPIO_RCC, HMC_SPI_MISO, HMC_SPI_MOSI, HMC_SPI_SCK, HMC_SPI_SS);

    // turn on temperature compensation, 220 Hz
    spi_write(&g_hmc.spi, CRA, TS | DO_220);

    delay_ms(1);
    // go into continuous measurement mode
    spi_write(&g_hmc.spi, MR, MD_C);

    // default range is 1.3 Gauss
    g_hmc.range = 1;
    g_hmc.avg_count = 0;

    return 0;
}

void hmc_sleep(void)
{
    if (g_hmc.suspended)
        return;
    // go into idle mode, which uses 2 uA
    spi_write(&g_hmc.spi, MR, MD_I);
    delay_ms(1);

    // shut off SPI
    spi_deinit(&g_hmc.spi);
    g_hmc.suspended = 1;
}

// TODO: keep this all fixed point
static inline float convert_mag(int16_t mag)
{
    return 10000.0F*(((float)mag)/conversions[g_hmc.range]);
}

// the HMC5983 has a bug where it randomly applies the wrong gain to the sample,
// resulting in a value 68% of what it should be
static bool update_avg(hmc_data_p data)
{
    // find the magnitude of the magetic vector
    float magnitude = sqrtf(data->mag[0]*data->mag[0] + data->mag[1]*data->mag[1] + data->mag[2]*data->mag[2]);

    // if the average hasn't settled yet, just update and return success
    g_hmc.avg_count++;
    if (g_hmc.avg_count < 100) {
        g_hmc.avg_mag = magnitude*ALPHA + g_hmc.avg_mag*(1.0f - ALPHA);
        return 1;
    }

    if (magnitude/g_hmc.avg_mag > 0.75) {
        // update the long term moving average if the values are good
        g_hmc.avg_mag = magnitude*ALPHA + g_hmc.avg_mag*(1.0f - ALPHA);
        return 1;
    } else {
        // reset the count whenever we see the spurious value and return failure
        g_hmc.avg_count = 0;
        return 0;
    }
}

int8_t hmc_read(hmc_data_p data, bool raw)
{
    if (g_hmc.suspended)
        return 0;

    uint8_t buf[6];
    spi_read(&g_hmc.spi, R(DXRA), (uint8_t *)buf, 6);

    int16_t x = (buf[0] << 8) | buf[1];
    int16_t y = (buf[2] << 8) | buf[3];
    int16_t z = (buf[4] << 8) | buf[5];

    if (raw) {
        data->mag[0] = (float)x;
        data->mag[1] = (float)y;
        data->mag[2] = (float)z;
    } else {
        data->mag[0] = convert_mag(x);
        data->mag[1] = convert_mag(y);
        data->mag[2] = convert_mag(z);

        // if we saw the spurious 68% value, scale it up
        if (!update_avg(data)) {
            data->mag[0] *= SCALE;
            data->mag[1] *= SCALE;
            data->mag[2] *= SCALE;
        }
    }

    return 0;
}

uint16_t hmc_closest_range(uint16_t range)
{
    uint8_t i = NUM_RANGES - 1;
    uint16_t target_range = ranges[i];
    while (i--) {
        if (ranges[i] < range) break;
        target_range = ranges[i];
    }
    return target_range;
}

void hmc_set_range(uint16_t range)
{
    if (g_hmc.suspended)
        return;

    uint8_t i;
    for (i = 0; i < NUM_RANGES; i++) {
        if (range == ranges[i]) {
            if (g_hmc.range != i) {
                g_hmc.range = i;
                spi_write(&g_hmc.spi, CRB, bitmasks[g_hmc.range]);
                delay_ms(1);
            }
            break;
        }
    }
}

uint16_t hmc_get_range(void)
{
    return ranges[g_hmc.range];
}

void hmc_set_register(uint8_t reg, uint8_t payload)
{
    if (g_hmc.suspended)
        return;

    spi_write(&g_hmc.spi, reg, payload);
}
