/************************************************************************************

Filename    :   hmc5983.h
Content     :   Honeywell HMC5983 interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _HMC5983_H_
#define _HMC5983_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct hmc_data_struct {
    float mag[3];
} hmc_data_t, *hmc_data_p;

int8_t hmc_init(void);
void hmc_sleep(void);
int8_t hmc_read(hmc_data_p data, bool raw);
uint16_t hmc_closest_range(uint16_t range);
void hmc_set_range(uint16_t range);
uint16_t hmc_get_range(void);
void hmc_set_register(uint8_t reg, uint8_t payload);

#endif /* _HMC5983_H_ */
