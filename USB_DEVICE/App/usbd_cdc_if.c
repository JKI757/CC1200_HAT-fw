/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "interface.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static USBD_CDC_LineCodingTypeDef line_coding[CDC_PORT_COUNT] =
{
  {
    .bitrate    = 460800,
    .format     = 0x00,
    .paritytype = 0x00,
    .datatype   = 0x08
  },
  {
    .bitrate    = 460800,
    .format     = 0x00,
    .paritytype = 0x00,
    .datatype   = 0x08
  }
};
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[CDC_PORT_COUNT][APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[CDC_PORT_COUNT][APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
static void CDC_ArmReceive(uint8_t port);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
static void CDC_ArmReceive(uint8_t port)
{
  (void)USBD_CDC_SetRxBufferPort(&hUsbDeviceFS, port, UserRxBufferFS[port]);
  (void)USBD_CDC_ReceivePacketPort(&hUsbDeviceFS, port);
}

/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  for (uint8_t port = 0U; port < CDC_PORT_COUNT; ++port)
  {
    (void)USBD_CDC_SetTxBufferPort(&hUsbDeviceFS, port, UserTxBufferFS[port], 0U);
    CDC_ArmReceive(port);
  }
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  uint8_t port = USBD_CDC_GetLastCmdPort(&hUsbDeviceFS);
  if (port >= CDC_PORT_COUNT) { port = 0U; }

  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
      break;

    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
      break;

    case CDC_SET_LINE_CODING:
      line_coding[port].bitrate    =  (uint32_t)pbuf[0]
                                    | ((uint32_t)pbuf[1] << 8)
                                    | ((uint32_t)pbuf[2] << 16)
                                    | ((uint32_t)pbuf[3] << 24);
      line_coding[port].format     = pbuf[4];
      line_coding[port].paritytype = pbuf[5];
      line_coding[port].datatype   = pbuf[6];
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(line_coding[port].bitrate);
      pbuf[1] = (uint8_t)(line_coding[port].bitrate >> 8);
      pbuf[2] = (uint8_t)(line_coding[port].bitrate >> 16);
      pbuf[3] = (uint8_t)(line_coding[port].bitrate >> 24);
      pbuf[4] = line_coding[port].format;
      pbuf[5] = line_coding[port].paritytype;
      pbuf[6] = line_coding[port].datatype;
      break;

    case CDC_SET_CONTROL_LINE_STATE:
    {
      uint16_t state = 0U;
      if (length >= 2U)
      {
        state = (uint16_t)pbuf[0] | ((uint16_t)pbuf[1] << 8);
      }
      else if (pbuf != NULL)
      {
        const USBD_SetupReqTypedef *setup = (const USBD_SetupReqTypedef *)pbuf;
        state = setup->wValue;
      }

      uint16_t normalized = 0U;
      if ((state & 0x0001U) != 0U || (state & 0x0100U) != 0U)
      {
        normalized |= 0x0001U;
      }
      if ((state & 0x0002U) != 0U || (state & 0x0200U) != 0U)
      {
        normalized |= 0x0002U;
      }

      interface_usb_line_state_changed(port, normalized);
      break;
    }

    case CDC_SEND_BREAK:
    default:
      break;
  }

  return (USBD_OK);
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  uint8_t port = USBD_CDC_GetLastRxPort(&hUsbDeviceFS);
  if (port >= CDC_PORT_COUNT) { port = 0U; }

  interface_usb_receive_callback(port, Buf, *Len);
  CDC_ArmReceive(port);
  return (USBD_OK);
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  if (USBD_CDC_SetTxBufferPort(&hUsbDeviceFS, 0U, Buf, Len) != USBD_OK)
  {
    return USBD_FAIL;
  }
  return USBD_CDC_TransmitPacketPort(&hUsbDeviceFS, 0U);
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t port = USBD_CDC_GetLastTxPort(&hUsbDeviceFS);
  if (port >= CDC_PORT_COUNT) { port = 0U; }
  (void)Buf; (void)Len; (void)epnum; (void)port;
  return (int8_t)USBD_OK;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
