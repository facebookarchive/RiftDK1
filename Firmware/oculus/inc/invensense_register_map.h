/************************************************************************************

Filename    :   invensense_register_map.h
Content     :   Invensense MPU-6xxx register map
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _INVENSENSE_REGISTER_MAP_H_
#define _INVENSENSE_REGISTER_MAP_H_

// Registers and bit settings for the Invensense MPU-6000
// Supports most other Invensense gyros with minor mods

// Registers
#define SELF_TEST_X     0x0D
#define SELF_TEST_Y     0x0E
#define SELF_TEST_Z     0x0F
#define SELF_TEST_A     0x10
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define MOT_THR         0x1F
#define MOT_DUR         0x20
#define FIFO_EN         0x23
#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27
#define I2C_SLV1_ADDR   0x28
#define I2C_SLV1_REG    0x29
#define I2C_SLV1_CTRL   0x2A
#define I2C_SLV2_ADDR   0x2B
#define I2C_SLV2_REG    0x2C
#define I2C_SLV2_CTRL   0x2D
#define I2C_SLV3_ADDR   0x2E
#define I2C_SLV3_REG    0x2F
#define I2C_SLV3_CTRL   0x30
#define I2C_SLV4_ADDR   0x31
#define I2C_SLV4_REG    0x32
#define I2C_SLV4_DO     0x33
#define I2C_SLV4_CTRL   0x34
#define I2C_SLV4_DI     0x35
#define I2C_MST_STATUS  0x36
#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
// Bunch of EXT_SENSE_DATA that we aren't using anyway
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75

// TODO: self test fields
// TODO: CONFIG EXT_SYNC_SET fields

// CONFIG fields - specified for gyro, but effect accel also
#define DLPF_CFG_256    0
#define DLPF_CFG_188    1
#define DLPF_CFG_98     2
#define DLPF_CFG_42     3
#define DLPF_CFG_20     4
#define DLPF_CFG_10     5
#define DLPF_CFG_5      6
#define DLPF_CFG_RES    7 // maybe disables low pass?

// GYRO_CONFIG fields
// full scale rate in degrees per second
#define FS_SEL_250      0
#define FS_SEL_500      (1 << 3)
#define FS_SEL_1000     (2 << 3)
#define FS_SEL_2000     (3 << 3)
// self test enable
#define XG_ST           (1 << 7)
#define YG_ST           (1 << 6)
#define ZG_ST           (1 << 5)

// LSB per degree per second
#define FS_SCALE_250    131.0F
#define FS_SCALE_500    65.5F
#define FS_SCALE_1000   32.8F
#define FS_SCALE_2000   16.4F

// ACCEL_CONFIG fields
// full scale rate in g's
#define AFS_SEL_2       0
#define AFS_SEL_4       (1 << 3)
#define AFS_SEL_8       (2 << 3)
#define AFS_SEL_16      (3 << 3)
// self test enable
#define XA_ST           (1 << 7)
#define YA_ST           (1 << 6)
#define ZA_ST           (1 << 5)

// LSB per g
#define AFS_SCALE_2     16384.0F
#define AFS_SCALE_4     8192.0F
#define AFS_SCALE_8     4096.0F
#define AFS_SCALE_16    2048.0F

// FIFO_EN fields
#define TEMP_FIFO_EN    (1 << 7)
#define XG_FIFO_EN      (1 << 6)
#define YG_FIFO_EN      (1 << 5)
#define ZG_FIFO_EN      (1 << 4)
#define ACCEL_FIFO_EN   (1 << 3)
#define SLV2_FIFO_EN    (1 << 2)
#define SLV1_FIFO_EN    (1 << 1)
#define SLV0_FIFO_EN    1

// TODO: the various I2C related fields

// INT_PIN_CFG fields
#define INT_LEVEL       (1 << 7) // 0 is active high
#define INT_OPEN        (1 << 6) // 0 push-pull, 1 open drain
#define LATCH_INT_EN    (1 << 5) // 0 pulse, 1 latch
#define INT_RD_CLEAR    (1 << 4) // 0 is clear on read INT_STATUS, 1 is read any
#define FSYNC_INT_LEVEL (1 << 3)
#define FSYNC_INT_EN    (1 << 2)
#define I2C_BYPASS_EN   (1 << 1)

// INT_ENABLE fields
#define MOT_EN          (1 << 6)
#define FIFO_OFLOW_EN   (1 << 4)
#define I2C_MST_INT_EN  (1 << 3)
#define DATA_RDY_EN     1

// INT_STATUS fields
#define MOT_INT         (1 << 6)
#define FIFO_OFLOW_INT  (1 << 4)
#define I2C_MST_INT     (1 << 3)
#define DATA_RDY_INT    1

// TODO: More I2C regs

// SIGNAL_PATH_RESET fields
#define GYRO_RESET      (1 << 2)
#define ACCEL_RESET     (1 << 1)
#define TEMP_RESET      1

// TODO: MOT_DETECT_CTRL

// USER_CTRL fields
#define CTRL_FIFO_EN    (1 << 6)
#define MST_EN          (1 << 5)
#define I2C_IF_DIS      (1 << 4) // 1 disables I2C. set this immediately when using SPI
#define FIFO_RESET      (1 << 2) // 1 clears FIFO when FIFO_EN is 0
#define MST_RESET       (1 << 1) // 1 resets I2C when I2C_MST_EN is 0
#define SIG_COND_RESET  1        // resets sensors and sensor data registers

// PWR_MGMT_1 fields
#define DEVICE_RESET    (1 << 7) // 1 resets all registers to default
#define SLEEP           (1 << 6) // defaults to 1
#define CYCLE           (1 << 5) // with sleep off, does periodic wakes per LP_WAKE_CTRL
#define TEMP_DIS        (1 << 4) // 1 turns off temp sense
#define CLKSEL_INT      0        // internal 8 MHz osc
#define CLKSEL_GYRO_X   1
#define CLKSEL_GYRO_Y   2
#define CLKSEL_GYRO_Z   3
#define CLKSEL_EXT_32   4       // external 32.769 KHz ref
#define CLKSEL_EXT_19   5       // external 19.2 MHz ref
#define CLKSEL_STOP     7       // shuts off the clock

// PWR_MGMT_2 fields
#define LP_WAKE_CTRL_1  0           // 1.25 Hz wake up
#define LP_WAKE_CTRL_5  (1 << 6)    // 5 Hz wake up
#define LP_WAKE_CTRL_20 (2 << 6)    // 20 Hz wake up
#define LP_WAKE_CTRL_40 (3 << 6)    // 40 Hz wake up
#define STBY_XA      (1 << 5)
#define STBY_YA      (1 << 4)
#define STBY_ZA      (1 << 3)
#define STBY_XG      (1 << 2)
#define STBY_YG      (1 << 1)
#define STBY_ZG      1

#endif /* _INVENSENSE_REGISTER_MAP_H_ */
