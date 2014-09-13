/************************************************************************************

Filename    :   delay.c
Content     :   Millisecond delay function
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#include "delay.h"
#ifdef STM32F0XX
#include "stm32f0xx_misc.h"
#else
#include "misc.h"
#endif /* STM32F0XX */

volatile static int32_t counter = 0;

void delay_init(void)
{
#ifdef USE_F102
    // F102 has the CALIB set for 72 MHz even though it runs at 48 MHz
    SysTick->LOAD = (((SysTick->CALIB & SysTick_CALIB_TENMS_Msk) * 2)/3);
#else
    // CALIB is set for 1ms when using reference SysTick clock
    SysTick->LOAD = SysTick->CALIB & SysTick_CALIB_TENMS_Msk;
#endif /* USE_F102 */
    // Enable the interrupt at high priority
    NVIC_SetPriority(SysTick_IRQn, 0);
    // Set the starting value at the top of the countdown
    SysTick->VAL = SysTick->LOAD;
    // Turn it on with the reference (9 MHz) clock and interrupts on
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

void delay_deinit(void)
{
    SysTick->CTRL = 0;
}

void delay_ms(uint32_t ms)
{
    // reset the systick counter
    delay_init();
    counter = ms;
    // Sleep while waiting for the interrupts
    while (counter > 0) __WFI();
}

void delay_update(void)
{
    if (counter > 0) counter--;
}
