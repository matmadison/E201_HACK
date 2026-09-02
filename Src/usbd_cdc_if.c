/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "main.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Runtime settings shared with main.c. */
volatile uint8_t active_encoder_count = 1U;
volatile uint8_t usb_stream_enabled = 1U;
volatile uint16_t usb_tx_batch_size = 240U;
volatile uint16_t usb_flush_threshold = 48U;
volatile uint8_t usb_force_flush_ms = 1U;

/* Recorder control-command parser state. */
static uint8_t active_cmd_state = 0U;
static uint8_t active_cmd_count = 1U;
static uint8_t stream_cmd_state = 0U;
static uint8_t stream_cmd_value = 0U;
static uint8_t tune_cmd_state = 0U;
static uint8_t tune_cmd_buf[5];
static uint8_t reference_cmd_state = 0U;
static uint8_t reference_cmd_value = 1U;
static uint8_t pc_reset_cmd_state = 0U;
static uint8_t pc_reset_cmd_value = 0U;

#define ACTIVE_ENCODER_CMD_HEADER  0xC3U
#define STREAM_CONTROL_CMD_HEADER  0xC4U
#define USB_TUNING_CMD_HEADER      0xC5U
#define REFERENCE_RESET_CMD_HEADER 0xC6U
#define PC_COUNTER_RESET_CMD_HEADER 0xC7U

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
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

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
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
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
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
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
  /* USER CODE BEGIN 6 */
  uint32_t i;

  if ((Buf != NULL) && (Len != NULL))
  {
    for (i = 0U; i < *Len; i++)
    {
      uint8_t b = Buf[i];

      /* C5 batch_L batch_H threshold_L threshold_H force_ms checksum */
      if ((tune_cmd_state >= 1U) && (tune_cmd_state <= 5U))
      {
        tune_cmd_buf[tune_cmd_state - 1U] = b;
        tune_cmd_state++;
        continue;
      }
      else if (tune_cmd_state == 6U)
      {
        uint8_t expected = USB_TUNING_CMD_HEADER;
        uint8_t k;
        uint16_t batch;
        uint16_t threshold;
        uint8_t force_ms;

        for (k = 0U; k < 5U; k++)
        {
          expected ^= tune_cmd_buf[k];
        }
        expected ^= 0xFFU;

        if (b == expected)
        {
          batch = (uint16_t)tune_cmd_buf[0] |
                  ((uint16_t)tune_cmd_buf[1] << 8);
          threshold = (uint16_t)tune_cmd_buf[2] |
                      ((uint16_t)tune_cmd_buf[3] << 8);
          force_ms = tune_cmd_buf[4];

          if ((batch >= 16U) && (batch <= 1024U) &&
              (threshold >= 1U) && (threshold <= batch) &&
              (force_ms >= 1U) && (force_ms <= 20U))
          {
            usb_tx_batch_size = batch;
            usb_flush_threshold = threshold;
            usb_force_flush_ms = force_ms;
          }
        }

        tune_cmd_state = 0U;
        continue;
      }

      /* C3 active_encoder_count checksum */
      if (active_cmd_state == 1U)
      {
        if ((b >= 1U) && (b <= 4U))
        {
          active_cmd_count = b;
          active_cmd_state = 2U;
        }
        else
        {
          active_cmd_state = 0U;
        }
        continue;
      }
      else if (active_cmd_state == 2U)
      {
        uint8_t expected =
            (uint8_t)((ACTIVE_ENCODER_CMD_HEADER ^ active_cmd_count) ^ 0xFFU);

        if (b == expected)
        {
          active_encoder_count = active_cmd_count;
        }
        active_cmd_state = 0U;
        continue;
      }

      /* C4 enabled checksum */
      if (stream_cmd_state == 1U)
      {
        if (b <= 1U)
        {
          stream_cmd_value = b;
          stream_cmd_state = 2U;
        }
        else
        {
          stream_cmd_state = 0U;
        }
        continue;
      }
      else if (stream_cmd_state == 2U)
      {
        uint8_t expected =
            (uint8_t)((STREAM_CONTROL_CMD_HEADER ^ stream_cmd_value) ^ 0xFFU);

        if (b == expected)
        {
          usb_stream_enabled = stream_cmd_value;
        }
        stream_cmd_state = 0U;
        continue;
      }

      /* C6 enabled checksum: select whether PA2 reference resets TIM2. */
      if (reference_cmd_state == 1U)
      {
        if (b <= 1U)
        {
          reference_cmd_value = b;
          reference_cmd_state = 2U;
        }
        else
        {
          reference_cmd_state = 0U;
        }
        continue;
      }
      else if (reference_cmd_state == 2U)
      {
        uint8_t expected =
            (uint8_t)((REFERENCE_RESET_CMD_HEADER ^ reference_cmd_value) ^ 0xFFU);

        if (b == expected)
        {
          APP_SetReferenceResetEnabled(reference_cmd_value);
        }
        reference_cmd_state = 0U;
        continue;
      }

      /* C7 1 checksum: zero TIM2 and report a PC reset event in the stream. */
      if (pc_reset_cmd_state == 1U)
      {
        if (b == 1U)
        {
          pc_reset_cmd_value = b;
          pc_reset_cmd_state = 2U;
        }
        else
        {
          pc_reset_cmd_state = 0U;
        }
        continue;
      }
      else if (pc_reset_cmd_state == 2U)
      {
        uint8_t expected =
            (uint8_t)((PC_COUNTER_RESET_CMD_HEADER ^ pc_reset_cmd_value) ^ 0xFFU);

        if (b == expected)
        {
          APP_ResetEncoderFromPC();
        }
        pc_reset_cmd_state = 0U;
        continue;
      }

      if (b == ACTIVE_ENCODER_CMD_HEADER)
      {
        active_cmd_state = 1U;
      }
      else if (b == STREAM_CONTROL_CMD_HEADER)
      {
        stream_cmd_state = 1U;
      }
      else if (b == USB_TUNING_CMD_HEADER)
      {
        tune_cmd_state = 1U;
      }
      else if (b == REFERENCE_RESET_CMD_HEADER)
      {
        reference_cmd_state = 1U;
      }
      else if (b == PC_COUNTER_RESET_CMD_HEADER)
      {
        pc_reset_cmd_state = 1U;
      }
    }
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
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
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc;

  if ((Buf == NULL) || (Len == 0U))
  {
    return USBD_FAIL;
  }

  if (hUsbDeviceFS.pClassData == NULL)
  {
    return USBD_BUSY;
  }

  hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0U)
  {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
