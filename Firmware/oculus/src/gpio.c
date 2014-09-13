/************************************************************************************

Filename    :   gpio.c
Content     :   Tracker gpio header control
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "gpio.h"
#include "stm32f10x.h"
#include "platform_config.h"

static uint8_t direction = 0;

#ifdef TRACKER_GPIO_PORT
static GPIO_InitTypeDef pin_init;

#define NUM_PINS TRACKER_GPIO_NUM_PINS

#if NUM_PINS == 8
static uint16_t pins[NUM_PINS] = {TRACKER_GPIO_IO0, TRACKER_GPIO_IO1, TRACKER_GPIO_IO2,
                          TRACKER_GPIO_IO3, TRACKER_GPIO_IO4, TRACKER_GPIO_IO5,
                          TRACKER_GPIO_IO6, TRACKER_GPIO_IO7};
#elif NUM_PINS == 2
static uint16_t pins[NUM_PINS] = {TRACKER_GPIO_IO0, TRACKER_GPIO_IO1};
#endif /* NUM_PINS */

#endif /* TRACKER_GPIO_PORT */

void gpio_init(void)
{
#ifdef TRACKER_GPIO_PORT
    RCC_APB2PeriphClockCmd(TRACKER_GPIO_RCC, ENABLE);

    pin_init.GPIO_Speed = GPIO_Speed_2MHz;
    pin_init.GPIO_Mode = GPIO_Mode_IPD;

    // Initialize all used pins as pulldown inputs
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        pin_init.GPIO_Pin = pins[i];
        GPIO_Init(TRACKER_GPIO_PORT, &pin_init);
    }
#endif /* TRACKER_GPIO_PORT */
}

void gpio_set_direction(uint8_t new_direction)
{
    direction = new_direction;
#ifdef TRACKER_GPIO_PORT
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        // as an input, setting sets pull up or pull down resistor
        pin_init.GPIO_Pin = pins[i];
        pin_init.GPIO_Mode = (direction & (1 << i)) ? GPIO_Mode_Out_PP : GPIO_Mode_IPD;
        GPIO_Init(TRACKER_GPIO_PORT, &pin_init);
    }
#endif /* TRACKER_GPIO_PORT */
}

uint8_t gpio_get_direction(void)
{
    return direction;
}

void gpio_set_value(uint8_t new_value)
{
#ifdef TRACKER_GPIO_PORT
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        // if this is an output
        if (direction & (1 << i)) {
            GPIO_WriteBit(TRACKER_GPIO_PORT, pins[i], (new_value & (1 << i)) ? Bit_SET: Bit_RESET);
        } else {
            // as an input, setting sets pull up or pull down resistor
            pin_init.GPIO_Pin = pins[i];
            pin_init.GPIO_Mode = (new_value & (1 << i)) ? GPIO_Mode_IPU : GPIO_Mode_IPD;
            GPIO_Init(TRACKER_GPIO_PORT, &pin_init);
        }
    }
#endif /* TRACKER_GPIO_PORT */
}

uint8_t gpio_get_value(void)
{
    uint8_t value = 0;
#ifdef TRACKER_GPIO_PORT
    for (uint8_t i = 0; i < NUM_PINS; i++) {
        // if this is an output
        if (direction & (1 << i)) {
            value |= (GPIO_ReadOutputDataBit(TRACKER_GPIO_PORT, pins[i]) << i);
        } else {
            value |= (GPIO_ReadInputDataBit(TRACKER_GPIO_PORT, pins[i]) << i);
        }
    }
#endif /* TRACKER_GPIO_PORT */
    return value;
}
