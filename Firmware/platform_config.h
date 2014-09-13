/**
  ******************************************************************************
  * @file    platform_config.h
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Evaluation board specific configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include "platform_common.h"

/* Includes ------------------------------------------------------------------*/

#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
 #include "stm32l1xx.h"
#elif defined (STM32F10X_MD) || defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #include "stm32f10x.h"
#endif

/* Define the STM32F10x hardware depending on the used evaluation board */
#if defined (USE_STEVAL_MKI121V1)
  #define USB_DISCONNECT                      GPIOA
  #define USB_DISCONNECT_PIN                  GPIO_Pin_9
  #define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA

  #define LED_PORT                            GPIOA
  #define LED_PIN                             GPIO_Pin_1
  #define RCC_APB2Periph_GPIO_LED             RCC_APB2Periph_GPIOA

  #define INV_SPI                             SPI1
  #define INV_SPI_RCC                         RCC_APB2Periph_SPI1
// APB2 is running at full AHB, needs to be brought down to <= 1 MHz
  #define INV_SPI_SPEED_LOW                   SPI_BaudRatePrescaler_128
// Sensor registers can be read at up to 20 MHz
  #define INV_SPI_SPEED_HIGH                  SPI_BaudRatePrescaler_4
  #define INV_SPI_GPIO                        GPIOA
  #define INV_SPI_GPIO_RCC                    RCC_APB2Periph_GPIOA
  #define INV_SPI_SCK                         GPIO_Pin_5
  #define INV_SPI_MISO                        GPIO_Pin_6
  #define INV_SPI_MOSI                        GPIO_Pin_7
  #define INV_SPI_SS                          GPIO_Pin_4
  #define INV_INT                             GPIO_Pin_0
  #define INV_INT_LINE                        EXTI_Line0
  #define INV_INT_PORT_SOURCE                 GPIO_PortSourceGPIOA
  #define INV_INT_PIN_SOURCE                  GPIO_PinSource0
  #define INV_INT_IRQn                        EXTI0_IRQn

// note that this dev board only has one SPI port exposed
  #define HMC_SPI                             SPI1
  #define HMC_SPI_RCC                         RCC_APB2Periph_SPI1
// APB2 is running at full AHB, needs to be brought down to <= 8 MHz
  #define HMC_SPI_SPEED                       SPI_BaudRatePrescaler_16
  #define HMC_SPI_GPIO                        GPIOA
  #define HMC_SPI_GPIO_RCC                    RCC_APB2Periph_GPIOA
  #define HMC_SPI_SCK                         GPIO_Pin_5
  #define HMC_SPI_MISO                        GPIO_Pin_6
  #define HMC_SPI_MOSI                        GPIO_Pin_7
  #define HMC_SPI_SS                          GPIO_Pin_2
  #define HMC_INT                             GPIO_Pin_3
  #define HMC_INT_LINE                        EXTI_Line3
  #define HMC_INT_PORT_SOURCE                 GPIO_PortSourceGPIOA
  #define HMC_INT_PIN_SOURCE                  GPIO_PinSource3
  #define HMC_INT_IRQn                        EXTI3_IRQn

// There is an onboard gyro on this board
  #define L3G_SPI                             SPI2
  #define L3G_SPI_RCC                         RCC_APB1Periph_SPI2
  // APB2 runs at 36 MHz, needs to be brought below 10
  #define L3G_SPI_SPEED                       SPI_BaudRatePrescaler_4
  #define L3G_SPI_GPIO                        GPIOB
  #define L3G_SPI_GPIO_RCC                    RCC_APB2Periph_GPIOB
  #define L3G_SPI_SCK                         GPIO_Pin_13
  #define L3G_SPI_MISO                        GPIO_Pin_14
  #define L3G_SPI_MOSI                        GPIO_Pin_15
  #define L3G_SPI_SS                          GPIO_Pin_12

#elif defined (USE_TRACKER_V1)
  #define UARTEN_PORT                         GPIOA
  #define UARTEN_PIN                          GPIO_Pin_8
  #define RCC_APB2Periph_GPIO_UARTEN          RCC_APB2Periph_GPIOA

  #define TRACKER_GPIO_PORT                   GPIOB
  #define TRACKER_GPIO_RCC                    RCC_APB2Periph_GPIOB
  #define TRACKER_GPIO_NUM_PINS               8
  #define TRACKER_GPIO_IO0                    GPIO_Pin_0
  #define TRACKER_GPIO_IO1                    GPIO_Pin_1
  #define TRACKER_GPIO_IO2                    GPIO_Pin_4
  #define TRACKER_GPIO_IO3                    GPIO_Pin_5
  #define TRACKER_GPIO_IO4                    GPIO_Pin_10
  #define TRACKER_GPIO_IO5                    GPIO_Pin_11
  #define TRACKER_GPIO_IO6                    GPIO_Pin_6
  #define TRACKER_GPIO_IO7                    GPIO_Pin_7

  #define FACTORY_PORT                        USART1
  #define FACTORY_SPEED                       115200
  #define FACTORY_GPIO_PORT                   GPIOA
  #define FACTORY_TX                          GPIO_Pin_9
  #define FACTORY_RX                          GPIO_Pin_10

  #define INV_SPI                             SPI1
  #define INV_SPI_RCC                         RCC_APB2Periph_SPI1
// APB2 is running at full AHB, needs to be brought down to <= 1 MHz
  #define INV_SPI_SPEED_LOW                   SPI_BaudRatePrescaler_128
// Sensor registers can be read at up to 20 MHz
  #define INV_SPI_SPEED_HIGH                  SPI_BaudRatePrescaler_4
  #define INV_SPI_GPIO                        GPIOA
  #define INV_SPI_GPIO_RCC                    RCC_APB2Periph_GPIOA
  #define INV_SPI_SCK                         GPIO_Pin_5
  #define INV_SPI_MISO                        GPIO_Pin_6
  #define INV_SPI_MOSI                        GPIO_Pin_7
  #define INV_SPI_SS                          GPIO_Pin_4
  #define INV_INT                             GPIO_Pin_0
  #define INV_INT_LINE                        EXTI_Line0
  #define INV_INT_PORT_SOURCE                 GPIO_PortSourceGPIOA
  #define INV_INT_PIN_SOURCE                  GPIO_PinSource0
  #define INV_INT_IRQn                        EXTI0_IRQn

// note that this dev board only has one SPI port exposed
  #define HMC_SPI                             SPI2
  #define HMC_SPI_RCC                         RCC_APB1Periph_SPI2
// APB1 is running at half AHB, needs to be brought down to <= 8 MHz
#ifdef USE_F102
  #define HMC_SPI_SPEED                       SPI_BaudRatePrescaler_4
#else
  #define HMC_SPI_SPEED                       SPI_BaudRatePrescaler_8
#endif /* USE_F102 */
  #define HMC_SPI_GPIO                        GPIOB
  #define HMC_SPI_GPIO_RCC                    RCC_APB2Periph_GPIOB
  #define HMC_SPI_SCK                         GPIO_Pin_13
  #define HMC_SPI_MISO                        GPIO_Pin_14
  #define HMC_SPI_MOSI                        GPIO_Pin_15
  #define HMC_SPI_SS                          GPIO_Pin_12
  #define HMC_INT                             GPIO_Pin_1
  #define HMC_INT_LINE                        EXTI_Line1
  #define HMC_INT_PORT_SOURCE                 GPIO_PortSourceGPIOA
  #define HMC_INT_PIN_SOURCE                  GPIO_PinSource1
  #define HMC_INT_IRQn                        EXTI1_IRQn
  #define HMC_SOC_PORT                        GPIOB
  #define HMC_SOC_PIN                         GPIO_Pin_9
  #define HMC_SOC_RCC                         RCC_APB2Periph_GPIOB

#endif /* USE_STM3210B_EVAL */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

