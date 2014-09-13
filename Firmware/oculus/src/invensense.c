/************************************************************************************

Filename    :   invensense.c
Content     :   Invensense MPU-6xxx configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "invensense.h"
#include "invensense_register_map.h"
#include "stm32f10x.h"
#include "delay.h"
#include "platform_config.h"
#include "spi.h"

#define R(a) (0x80 | a)

#define M_PI 3.14159265358979323846F
#define GRAVITY 9.80665F

#define GYRO_NUM_RANGES 4
static const uint16_t gyro_ranges[GYRO_NUM_RANGES] = {250, 500, 1000, 2000};
static const uint8_t gyro_bitmasks[GYRO_NUM_RANGES] = {FS_SEL_250, FS_SEL_500, FS_SEL_1000, FS_SEL_2000};
static const float gyro_conversions[GYRO_NUM_RANGES] = {FS_SCALE_250, FS_SCALE_500, FS_SCALE_1000, FS_SCALE_2000};

#define ACCEL_NUM_RANGES 4
static const uint8_t accel_ranges[ACCEL_NUM_RANGES] = {2, 4, 8, 16};
static const uint8_t accel_bitmasks[ACCEL_NUM_RANGES] = {AFS_SEL_2, AFS_SEL_4, AFS_SEL_8, AFS_SEL_16};
static const float accel_conversions[ACCEL_NUM_RANGES] = {AFS_SCALE_2, AFS_SCALE_4, AFS_SCALE_8, AFS_SCALE_16};

typedef struct invensense_struct {
	spi_t spi;
  uint8_t gyro_range;
  uint8_t accel_range;
  bool suspended;
} invensense_t, *invensense_p;

static invensense_t g_inv = {{0}};

int8_t invensense_init(void)
{
    spi_init(&g_inv.spi, INV_SPI, INV_SPI_SPEED_LOW, INV_SPI_RCC, INV_SPI_GPIO, INV_SPI_GPIO_RCC, INV_SPI_MISO, INV_SPI_MOSI, INV_SPI_SCK, INV_SPI_SS);
    g_inv.suspended = 0;

    // make sure 100 ms has passed for the registers to stabilize
    // reset the device state
    spi_write(&g_inv.spi, PWR_MGMT_1, DEVICE_RESET);

    delay_ms(1);
    // shut off I2C first since we use SPI
    spi_write(&g_inv.spi, USER_CTRL, I2C_IF_DIS);

    delay_ms(1);
    // also setting SLEEP to 0
    spi_write(&g_inv.spi, PWR_MGMT_1, CLKSEL_GYRO_Y);

    delay_ms(1);
    // set the low pass filter
    spi_write(&g_inv.spi, CONFIG, DLPF_CFG_256);

    delay_ms(1);
    // since the gyro runs at 8kHz, divide the sample rate by 8 (7+1)
    spi_write(&g_inv.spi, SMPLRT_DIV, 7);

    delay_ms(1);
    // Run at 2000 degrees per second
    spi_write(&g_inv.spi, GYRO_CONFIG, FS_SEL_2000);
    g_inv.gyro_range = 3;
    // Accel runs at the default of 2G
    g_inv.accel_range = 0;

    delay_ms(1);
    // Clear interrupt on any read
    spi_write(&g_inv.spi, INT_PIN_CFG, INT_RD_CLEAR);

    delay_ms(1);
    // Turn on data ready interrupts
    spi_write(&g_inv.spi, INT_ENABLE, DATA_RDY_EN);

    // Switch to closer to 20 MHz SPI for sensor reading
    spi_port_config(&g_inv.spi, INV_SPI_SPEED_HIGH);

    return 0;
}

void invensense_sleep(void)
{
    // Don't resuspend
    if (g_inv.suspended)
        return;

    // Go into a sleep mode where we get an interrupt on motion detect
    // Sequence is from the product sheet section 8.1
    // Switch back to 1 MHz SPI for configuration registers
    spi_port_config(&g_inv.spi, INV_SPI_SPEED_LOW);

    delay_ms(1);
    // Reset accel config
    spi_write(&g_inv.spi, ACCEL_CONFIG, 0x00);

    delay_ms(1);
    // Undocumented.  Product sheet says to set it
    spi_write(&g_inv.spi, MOT_DUR, 1);

    delay_ms(1);
    // Product sheet recommends 20*32mg threshold for movement
    spi_write(&g_inv.spi, MOT_THR, 25);

    // Datasheet says wait at least 1ms
    delay_ms(2);

    // Undocumented, but enables digital high pass filter
    spi_write(&g_inv.spi, ACCEL_CONFIG, 0x03);

    delay_ms(1);
    // Go to 5Hz cycle and shut off the gyro
    // this also switches the clock to internal oscillator
    spi_write(&g_inv.spi, PWR_MGMT_2, LP_WAKE_CTRL_5 | STBY_XG | STBY_YG | STBY_ZG);

    delay_ms(1);
    // Go into cycle mode and shut off temp measurement since we wont use it
    spi_write(&g_inv.spi, PWR_MGMT_1, CYCLE | TEMP_DIS);

    delay_ms(1);
    // Switch to interrupt only on motion detect
    spi_write(&g_inv.spi, INT_ENABLE, MOT_INT);

    delay_ms(1);
    // Clear any pending interrupt
    EXTI_ClearITPendingBit(INV_INT_LINE);

    spi_deinit(&g_inv.spi);

    g_inv.suspended = 1;
}

void invensense_deinit(void)
{
    // if we are suspended, bring spi and delay back up
    if (g_inv.suspended) {
        delay_init();
        spi_init(&g_inv.spi, INV_SPI, INV_SPI_SPEED_LOW, INV_SPI_RCC, INV_SPI_GPIO, INV_SPI_GPIO_RCC, INV_SPI_MISO, INV_SPI_MOSI, INV_SPI_SCK, INV_SPI_SS);
    }

    // Full shut down of the device
    spi_port_config(&g_inv.spi, INV_SPI_SPEED_LOW);

    delay_ms(1);
    spi_write(&g_inv.spi, PWR_MGMT_1, DEVICE_RESET);

    delay_ms(1);
    spi_deinit(&g_inv.spi);
}

bool invensense_motion_interrupt(void)
{
    // Just assume that any gyro interrupt while suspended is from motion wakeup
    // This way we don't need to spin up the clocks for SPI yet
    return g_inv.suspended;
/*
    uint8_t buf = 0;
    spi_read(&g_inv.spi, R(INT_STATUS), &buf, 1);

    return (buf & MOT_INT);
*/
}

// TODO: keep this all fixed point
static inline float convert_accel(int16_t accel)
{
    return 10000.0F*GRAVITY*(((float)accel)/accel_conversions[g_inv.accel_range]);
}

static inline float convert_gyro(int16_t gyro)
{
    return 10000.0F*M_PI*(((float)gyro)/(gyro_conversions[g_inv.gyro_range]*180.0F));
}

static inline float convert_temp(int16_t temp)
{
    return 100.0F*(((((float)temp)+521.0F)/340.0F)+35.0F);
}

int8_t invensense_read(invensense_data_p data, bool raw)
{
    if (g_inv.suspended)
        return 0;

    uint8_t buf[14];
    spi_read(&g_inv.spi, R(ACCEL_XOUT_H), (uint8_t *)buf, 14);

    int16_t ax = (buf[0] << 8) | buf[1];
    int16_t ay = (buf[2] << 8) | buf[3];
    int16_t az = (buf[4] << 8) | buf[5];
    int16_t t = (buf[6] << 8) | buf[7];
    int16_t gx = (buf[8] << 8) | buf[9];
    int16_t gy = (buf[10] << 8) | buf[11];
    int16_t gz = (buf[12] << 8) | buf[13];

    if (raw) {
        data->accel[0] = (float)ax;
        data->accel[1] = (float)ay;
        data->accel[2] = (float)az;
        data->temperature = (float)t;
        data->gyro[0] = (float)gx;
        data->gyro[1] = (float)gy;
        data->gyro[2] = (float)gz;
    } else {
        data->accel[0] = convert_accel(ax);
        data->accel[1] = convert_accel(ay);
        data->accel[2] = convert_accel(az);
        data->temperature = convert_temp(t);
        data->gyro[0] = convert_gyro(gx);
        data->gyro[1] = convert_gyro(gy);
        data->gyro[2] = convert_gyro(gz);
    }

    return 0;
}

uint8_t invensense_closest_accel_range(uint8_t accel)
{
    uint8_t i = ACCEL_NUM_RANGES - 1;
    uint8_t target_accel = accel_ranges[i];
    // Grab the smallest range that will include the desired range
    while (i--) {
        if (accel_ranges[i] < accel) break;
        target_accel = accel_ranges[i];
    }
    return target_accel;
}

uint16_t invensense_closest_gyro_range(uint16_t gyro)
{
    uint8_t i = GYRO_NUM_RANGES - 1;
    uint16_t target_gyro = gyro_ranges[i];
    while (i--) {
        if (gyro_ranges[i] < gyro) break;
        target_gyro = gyro_ranges[i];
    }
    return target_gyro;
}

void invensense_set_ranges(uint8_t accel, uint16_t gyro)
{
    if (g_inv.suspended)
        return;

    uint8_t i;
    // find the correct index for the value passed in
    for (i = 0; i < ACCEL_NUM_RANGES; i++) {
        if (accel == accel_ranges[i]) {
            // only set if the value is actually different
            if (g_inv.accel_range != i) {
                g_inv.accel_range = i;
                spi_write(&g_inv.spi, ACCEL_CONFIG, accel_bitmasks[g_inv.accel_range]);
                delay_ms(1);
            }
            break;
        }
    }

    for (i = 0; i < GYRO_NUM_RANGES; i++) {
        if (gyro == gyro_ranges[i]) {
            if (g_inv.gyro_range != i) {
                g_inv.gyro_range = i;
                spi_write(&g_inv.spi, GYRO_CONFIG, gyro_bitmasks[g_inv.gyro_range]);
                delay_ms(1);
            }
            break;
        }
    }
}

void invensense_get_ranges(uint8_t *accel, uint16_t *gyro)
{
    *accel = accel_ranges[g_inv.accel_range];
    *gyro = gyro_ranges[g_inv.gyro_range];
}

void invensense_set_register(uint8_t reg, uint8_t payload)
{
    if (g_inv.suspended)
        return;

    spi_write(&g_inv.spi, reg, payload);
}
