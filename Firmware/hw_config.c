/************************************************************************************

Filename    :   hw_config.c
Content     :   Hardware initialization for Tracker
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Hardware Configuration & Setup
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


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "spi.h"
#include "invensense.h"
#include "hmc5983.h"
#include "gpio.h"
#include "delay.h"
#include "eeprom.h"
#include "eeprom_table.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

ErrorStatus HSEStartUpStatus;
#ifdef DK_HD
static uint16_t use_uuid_serial = 2;
#else /* DK_HD */
static uint16_t use_uuid_serial = 0;
#endif /* DK_HD */
/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t exit_stop_mode;
/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configuration(void);
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Private functions ---------------------------------------------------------*/

void Configure_Debug(void)
{
  // Shut off JTAG since we only use SWD, so we can re-use the pins
  // This needs to happen shortly after reset
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */

  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Configure the used GPIOs*/
  GPIO_Configuration();

  // Make FLASH writable while initializing the virtual EEPROM
  FLASH_Unlock();
  EE_Init();
  FLASH_Lock();
}

void Reset_Device(void)
{
    // if we have USB disconnect, automatically unplug and
    // reset the device
#ifdef USB_DISCONNECT_PIN
    // Power off the sensors first
    Sensor_Deinit();
    USB_Cable_Config(DISABLE);
    delay_ms(10);
    NVIC_SystemReset();
#endif /* USB_DISCONNECT_PIN */
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
  /* Select USBCLK source */
#ifdef USE_F102
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
#else
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
#endif /* USE_F102 */

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
  exit_stop_mode = 0;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;
  exit_stop_mode = 1;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

void Enter_Stop(void)
{
    // Going into stop mode works, but the USB wakeup interrupt never fires.
    // Instead, just shut off the external resonator and PLL and go to
    // 8 MHz internal RC.  Gets us down to ~1.7 mA.
//    DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STANDBY | DBGMCU_STOP, ENABLE);
//    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);

    RCC_HSICmd(ENABLE);

    /* Wait till HSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET) {}

    /* Select HSI as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);

    /* Disable PLL1 */
    RCC_PLLCmd(DISABLE);

    /* Disable HSE */
    RCC_HSEConfig(RCC_HSE_OFF);

    // Shut off core clocks
    __WFI();
}

// Restart clocks
void Wake_From_Stop(void)
{
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) {}

    /* Enable PLL1 */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL1 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != RCC_CFGR_SWS_PLL) {}
}

void Sensor_Init(void)
{
    delay_init();

    invensense_init();

    hmc_init();
}

void Sensor_Sleep(void)
{
    invensense_sleep();

    hmc_sleep();

    delay_deinit();
}

void Sensor_Deinit(void)
{
    invensense_deinit();

    hmc_sleep();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the EXTI line 18 connected internally to the USB IP */
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18; // USB resume from suspend mode
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

void LED_Set(bool on)
{
#ifdef LED_PIN
  if (on) {
    GPIO_SetBits(LED_PORT, LED_PIN);
  } else {
    GPIO_ResetBits(LED_PORT, LED_PIN);
  }
#endif /* LED_PIN */
}

/*******************************************************************************
* Function Name  : USB_Cable_Config.
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : NewState: new state.
* Output         : None.
* Return         : None
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#ifdef USB_DISCONNECT_PIN
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;

  if (NewState == DISABLE)
  {
    // Go high-Z to disconnect the usb pullup resistor
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
  }
  else
  {
    // pull up the pullup resistor
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* USB_DISCONNECT_PIN */
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void GPIO_Configuration(void)
{
  // Init the pins on our gpio header
  gpio_init();

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#ifdef LED_PIN
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_LED, ENABLE);
  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);
#endif /* LED_PIN */

#ifdef UARTEN_PIN
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_UARTEN, ENABLE);
  GPIO_InitStructure.GPIO_Pin = UARTEN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(UARTEN_PORT, &GPIO_InitStructure);
#endif /* UARTEN_PIN */

#ifdef HMC_SOC_PIN
  // Pull SoC to ground since we don't use it
  RCC_APB2PeriphClockCmd(HMC_SOC_RCC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = HMC_SOC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(HMC_SOC_PORT, &GPIO_InitStructure);
#endif /* HMC_SOC_PIN */
}

void Enable_EXTInterrupts(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable the pin clock */
  RCC_APB2PeriphClockCmd(INV_SPI_GPIO_RCC | HMC_SPI_GPIO_RCC, ENABLE);

  /* Configure pins as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = INV_INT;
  GPIO_Init(INV_SPI_GPIO, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = HMC_INT;
  GPIO_Init(HMC_SPI_GPIO, &GPIO_InitStructure);

  GPIO_EXTILineConfig(INV_INT_PORT_SOURCE, INV_INT_PIN_SOURCE);
  GPIO_EXTILineConfig(HMC_INT_PORT_SOURCE, HMC_INT_PIN_SOURCE);

  /* Configure Key EXTI line to generate an interrupt on rising edges */
  EXTI_InitStructure.EXTI_Line = INV_INT_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = HMC_INT_LINE;
  EXTI_Init(&EXTI_InitStructure);

  /* Clear the Key EXTI line pending bit */
  EXTI_ClearITPendingBit(INV_INT_LINE);
  EXTI_ClearITPendingBit(HMC_INT_LINE);

  /* Enable and set EXTI Interrupt to the lowest priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = INV_INT_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = HMC_INT_IRQn;
  NVIC_Init(&NVIC_InitStructure);
}

static void IntToBase32(uint32_t value, char *buf, uint8_t len, bool unicode)
{
    uint8_t mult = unicode ? 2 : 1;

    for (uint8_t i = 0; i < len; i++) {
        // Select 5 bits
        uint8_t bits = value & 0x1F;

        if (bits < 0xA) {
            buf[i*mult] = bits + '0';
        } else {
            buf[i*mult] = bits + 'A' - 10;
        }

        if (unicode) buf[i*mult + 1] = 0;
        value = value >> 5;
    }
}

static void IntToString(uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;
  }
}

static bool Get_Stored_Serial(char *Serial)
{
    for (uint8_t i = 0; i < 6; i++) {
        uint16_t temp_chars = 0;
        // Give up if serial number incomplete
        if (EE_ReadVariable(EE_SERIAL_NUM_START+i, &temp_chars))
            return 0;

        *(((uint16_t *)Serial)+i) = temp_chars;
    }

    return 1;
}

void Set_Stored_Serial(char *SerialString)
{
    // Use the UUID based method for now
    Set_Use_Serial();

 /*
    // Sanitize the serial
    for (uint8_t i = 0; i < 12; i++) {
        if (SerialString[i] == 0x2C)
            SerialString[i] = '.';
        if ((SerialString[i] >= 0x7F) || (SerialString[i] < 0x20))
            SerialString[i] = '0';
    }

    FLASH_Unlock();
    for (uint8_t i = 0; i < 6; i++) {
        uint16_t temp_chars = *(((uint16_t *)SerialString)+i);
        EE_WriteVariable(EE_SERIAL_NUM_START+i, temp_chars);
    }
    FLASH_Lock();
 */
}

// Legacy serial number check
static bool Get_Use_Serial(void)
{
    if (use_uuid_serial)
        return 1;

    uint16_t serial_set = 0;

    EE_ReadVariable(EE_SERIAL_SET, &serial_set);

    if (serial_set) {
        use_uuid_serial = serial_set;
        return 1;
    }

    return 0;
}

// Legacy method for setting the serial number
void Set_Use_Serial(void)
{
    Get_Use_Serial();

    // Set the serial if it has been set by the legacy UUID method
    if (use_uuid_serial < 2) {
        FLASH_Unlock();
        EE_WriteVariable(EE_SERIAL_SET, 2);
        FLASH_Lock();

        use_uuid_serial = 2;
    }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(char *SerialString, bool UseUnicode)
{
    // Start off with a fake serial number so that all devices look the same,
    // so we can get through factory QA faster
    uint32_t Device_Serial0 = 0xAAAAAAAA, Device_Serial1 = 0xAAAAAAAA;
    char temp_serial[12] = {0};

    // Always use the stored one if it exists
    if (Get_Stored_Serial(temp_serial)) {
        if (UseUnicode) {
            // write the ascii chars
            for (uint8_t i = 0; i < 12; i++) {
                SerialString[i*2+2] = temp_serial[i];
                SerialString[i*2+3] = 0;
            }
        } else {
            memcpy(SerialString, temp_serial, 12);
        }
    } else {
        // Otherwise use the uuid based serial for feature report or if
        // the legacy use serial is set
        if (Get_Use_Serial() || !UseUnicode) {
            uint32_t serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
            uint32_t serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
            uint32_t serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

            // The legacy method of generating a serial from UUID is prone to
            // collision, so only use it on devices that were already set up
            // that way.
            if (use_uuid_serial == 1) {
                Device_Serial0 = serial0 + serial2;
                Device_Serial1 = serial1;

                if (UseUnicode) {
                    IntToUnicode(Device_Serial0, (uint8_t *)&SerialString[2], 8);
                    IntToUnicode(Device_Serial1, (uint8_t *)&SerialString[18], 4);
                } else {
                    IntToString(Device_Serial0, (uint8_t *)&SerialString[0], 8);
                    IntToString(Device_Serial1, (uint8_t *)&SerialString[8], 4);
                }
            } else {
                // Otherwise try to preserve some uniqueness by going to 60 bits
                // Most of the change appears to be happening in the final 32 bits
                Device_Serial0 = serial2 ^ ((serial2 & 0xC000) >> 2);
                Device_Serial1 = serial1 ^ serial0;
                Device_Serial1 = Device_Serial1 ^ ((Device_Serial1 & 0xC000) >> 2);

                // And then encoding as Base32
                if (UseUnicode) {
                    IntToBase32(Device_Serial0, &SerialString[2], 6, 1);
                    IntToBase32(Device_Serial1, &SerialString[14], 6, 1);
                } else {
                    IntToBase32(Device_Serial0, &SerialString[0], 6, 0);
                    IntToBase32(Device_Serial1, &SerialString[6], 6, 0);
                }
            }
        } else {
            // Send AAAAAAAAAAAA as the serial in the factory
            IntToUnicode(Device_Serial0, (uint8_t *)&SerialString[2], 8);
            IntToUnicode(Device_Serial1, (uint8_t *)&SerialString[18], 4);
        }
    }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
