/************************************************************************************

Filename    :   hmc5983_register_map.h
Content     :   Honeywell HMC5983 register map
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _HMC5983_REGISTER_MAP_H_
#define _HMC5983_REGISTER_MAP_H_

#define CRA     0x00
#define CRB     0x01
#define MR      0x02
#define DXRA    0x03
#define DXRB    0x04
#define DYRA    0x05
#define DYRB    0x06
#define DZRA    0x07
#define DZRB    0x08
#define SR      0x09
#define IRA     0x0A
#define IRB     0x0B
#define IRC     0x0C
#define TEMPH   0x31
#define TEMPL   0x32

// CRA fields
// enable temperature compensation
#define TS      (1 << 7)
// average data over how many samples
#define MA_1    0
#define MA_2    (1 << 5)
#define MA_4    (2 << 5)
#define MA_8    (3 << 5)
// data rate
#define DO_0_75 0
#define DO_1_5  (1 << 2)
#define DO_3    (2 << 2)
#define DO_7_5  (3 << 2)
#define DO_15   (4 << 2)
#define DO_30   (5 << 2)
#define DO_75   (6 << 2)
#define DO_220  (7 << 2)
// self test bias
#define MS_NO   0   // normal
#define MS_PB   1   // positive bias
#define MS_NB   2   // negative bias
#define MS_TO   3   // temperature only mode

// CRB fields
// gain
#define GN_0_88 0
#define GN_1_3  (1 << 5)
#define GN_1_9  (2 << 5)
#define GN_2_5  (3 << 5)
#define GN_4_0  (4 << 5)
#define GN_4_7  (5 << 5)
#define GN_5_6  (6 << 5)
#define GN_8_1  (7 << 5)

#define GN_SCALE_0_88 1370.0F
#define GN_SCALE_1_3  1090.0F
#define GN_SCALE_1_9  820.0F
#define GN_SCALE_2_5  660.0F
#define GN_SCALE_4_0  440.0F
#define GN_SCALE_4_7  390.0F
#define GN_SCALE_5_6  330.0F
#define GN_SCALE_8_1  230.0F

// MR fields
#define HS      (1 << 7)    // i2c 3.4MHz
#define LP      (1 << 5)    // lowest power mode
#define SIM     (1 << 2)    // 1 is 3 wire SPI
#define MD_C    0           // continuous measurement
#define MD_S    1           // single measurement
#define MD_I    2           // idle mode

// SR fields
#define DOW     (1 << 4)    // data overwritten
#define LOCK    (1 << 1)    // data registers not being updated
#define RDY     1

#endif /* _HMC5983_REGISTER_MAP_H_ */
