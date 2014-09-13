/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Descriptors for Custom HID Demo
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
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize 64*/
    0x33,                       /*idVendor (0x2833)*/
    0x28,
    0x01,                       /*idProduct = 0x0001*/
    0x00,
    0x18,                       /*bcdDevice rel. 0.18*/
#ifdef USE_F102
    0x01,                       /* 1.x for the F102 revision */
#else
    0x00,
#endif /* USE_F102 */
    1,                          /*Index of string descriptor describing
                                              manufacturer */
    2,                          /*Index of string descriptor describing
                                             product*/
    3,                          /*Index of string descriptor describing the
                                             device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    CUSTOMHID_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: Configuration value */
    0x00,         /* iConfiguration: Index of string descriptor describing
                                 the configuration*/
    0xA0,         /* bmAttributes: Bus powered, support wakeup */
#ifdef DK_HD
    0xFA,         /* MaxPower 500 mA: this current is used for detecting Vbus */
#else
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */
#endif

    /************** Descriptor of Custom HID interface ****************/
    /* 09 */
    0x09,         /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type */
    0x00,         /* bInterfaceNumber: Number of Interface */
    0x00,         /* bAlternateSetting: Alternate setting */
    0x01,         /* bNumEndpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,            /* iInterface: Index of string descriptor */
    /******************** Descriptor of Custom HID HID ********************/
    /* 18 */
    0x09,         /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType */
    (CUSTOMHID_SIZ_REPORT_DESC & 0xFF),/* wItemLength: Total length of Report descriptor */
    (CUSTOMHID_SIZ_REPORT_DESC >> 8),
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 27 */
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

    0x81,          /* bEndpointAddress: Endpoint Address (IN) */
    0x03,          /* bmAttributes: Interrupt endpoint */
    0x40,          /* wMaxPacketSize: 64 Bytes max */
    0x00,
    0x01,          /* bInterval: Polling Interval (1 ms) */
    /* 34 */
  }
  ; /* CustomHID_ConfigDescriptor */
const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
  {
    0x05, 0x03,                    // USAGE_PAGE (VR Controls)
    0x09, 0x05,                    // USAGE (Head Tracker)
    0xa1, 0x01,                    // COLLECTION (Application)

    // Sub-Collection for DK set of reports
    0x06, 0x00, 0xff,              //   USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    //   USAGE (Vendor Defined - DK compatibility)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)

    // IN Report for sensor data
    0x85, 0x01,                    //   REPORT_ID (1)
    // Sample Count
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Timestamp
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // LastCommandID
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Temperature
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Gyro/Accel Sample 0
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Gyro/Accel Sample 1
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Gyro/Accel Sample 2
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Mag Sample
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)

    // Config Report
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x04,                    //   REPORT_COUNT (4)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Calibration Report
    0x85, 0x03,                    //   REPORT_ID (3)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x42,                    //   REPORT_COUNT (66)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Full Scale Range Report
    0x85, 0x04,                    //   REPORT_ID (4)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Register Report
    0x85, 0x05,                    //   REPORT_ID (5)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // DFU Report
    0x85, 0x06,                    //   REPORT_ID (6)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // GPIO Report
    0x85, 0x07,                    //   REPORT_ID (7)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x02,                    //   REPORT_COUNT (2)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Keep Alive Report
    0x85, 0x08,                    //   REPORT_ID (8)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x02,                    //   REPORT_COUNT (2)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Display Info Report
    0x85, 0x09,                    //   REPORT_ID (9)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x35,                    //   REPORT_COUNT (53)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Serial Report
    0x85, 0x0A,                    //   REPORT_ID (10)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x0C,                    //   REPORT_COUNT (12)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    0xc0,                          //   END_COLLECTION

    0xc0                           // END_COLLECTION
  }; /* CustomHID_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "Oculus VR, Inc." */
    'O', 0, 'c', 0, 'u', 0, 'l', 0, 'u', 0, 's', 0, ' ', 0, 'V', 0,
    'R', 0, ',', 0, ' ', 0, 'I', 0, 'n', 0, 'c', 0, '.', 0
  };

const uint8_t CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
#ifdef DK_HD
    'R', 0, 'i', 0, 'f', 0, 't', 0, ' ', 0, 'D', 0, 'K', 0, ' ', 0,
    'H', 0, 'D', 0
#else
    'T', 0, 'r', 0, 'a', 0, 'c', 0, 'k', 0, 'e', 0, 'r', 0, ' ', 0,
    'D', 0, 'K', 0
#endif /* DK_HD */
  };
uint8_t CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0,'3', 0,'2', 0, '1', 0, '0', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

