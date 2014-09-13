/************************************************************************************

Filename    :   platform_common.h
Content     :   Tracker common header
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _PLATFORM_COMMON_
#define _PLATFORM_COMMON_

#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
 #include "stm32l1xx.h"
#elif defined (STM32F10X_LD) || defined (STM32F10X_MD) || defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #include "stm32f10x.h"
#endif

// Common pin definitions
#if defined(USE_TRACKER_V1) || defined(USE_TRACKERHD)
  #define USB_DISCONNECT                      GPIOA
  #define USB_DISCONNECT_PIN                  GPIO_Pin_2
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
#elif defined(USE_OURSTM)
  #define USB_DISCONNECT                      GPIOC
  #define USB_DISCONNECT_PIN                  GPIO_Pin_13
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOC
#elif defined(USE_LATENCY_TESTER)
  #define USB_DISCONNECT                      GPIOA
  #define USB_DISCONNECT_PIN                  GPIO_Pin_10
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
#endif

#endif /* _PLATFORM_COMMON_ */