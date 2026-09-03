/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_TIMESTAMP_INTERVAL_FRAMES 1000U
#define USB_TX_BUFFER_CAPACITY     1024U
#define BACKGROUND_SERVICE_INTERVAL_MS 500U
#define USB_LED_CHECK_INTERVAL_MS  1000U
#define ENCODER_SENSE_INTERVAL_MS  1000U

/* 12-bit ADC thresholds for the measured PB13 levels:
 * approximately 0 counts unplugged and 166 counts connected. */
#define ENCODER_CONNECT_ADC         100U
#define ENCODER_DISCONNECT_ADC       50U

#define LED_USB_RED_PIN            GPIO_PIN_6
#define LED_USB_GREEN_PIN          GPIO_PIN_7
#define LED_ENCODER_GREEN_PIN      GPIO_PIN_8
#define LED_ENCODER_RED_PIN        GPIO_PIN_9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

extern volatile uint8_t active_encoder_count;
extern volatile uint8_t usb_stream_enabled;
extern volatile uint16_t usb_tx_batch_size;
extern volatile uint16_t usb_flush_threshold;
extern volatile uint8_t usb_force_flush_ms;
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t encoder_frame[10];
static uint8_t timestamp_frame[6];
static uint8_t reference_event_frame[4];
static uint8_t usb_tx_buffer[2][USB_TX_BUFFER_CAPACITY];
static uint8_t usb_fill_buffer = 0U;
static uint16_t usb_fill_length = 0U;
static uint32_t usb_fill_started_ms = 0U;
static uint32_t frames_until_timestamp = DEFAULT_TIMESTAMP_INTERVAL_FRAMES;
static volatile uint32_t timestamp_interval_requested =
    DEFAULT_TIMESTAMP_INTERVAL_FRAMES;
static uint32_t background_service_last_ms = 0U;
static uint32_t usb_led_last_check_ms = 0U;
static uint8_t usb_led_configured = 0xFFU;
static uint32_t encoder_sense_last_ms = 0U;
static uint32_t encoder_sense_filtered = 0U;
static uint8_t encoder_sense_valid = 0U;
static uint8_t encoder_connected = 0U;
static volatile uint8_t pc_reset_pending_mask = 0U;
static volatile uint8_t physical_reference_reset_mask = 0x01U;
static volatile uint8_t physical_reference_event_type = 0U;
static volatile uint16_t physical_reference_capture = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

static uint16_t APP_BuildEncoderFrame(uint8_t *frame, uint16_t capacity);
static void APP_BuildTimestampFrame(uint8_t *frame, uint32_t tick_ms);
static uint8_t USB_AppendFrame(const uint8_t *frame, uint16_t length,
                               uint32_t now);
static uint8_t USB_TryFlush(uint32_t now, uint8_t force_flush);
static void USB_ApplyPendingReferenceEventsAtBatchBoundary(uint32_t now);
static void USB_StreamTask(void);
static void APP_SetEncoderConnectedLED(uint8_t connected);
static void APP_UpdateEncoderStatusLED(uint32_t now);
static void APP_UpdateUsbStatusLED(uint32_t now);
void APP_RequestPCReset(uint8_t encoder_mask);
void APP_SetPhysicalReferenceResetMask(uint8_t encoder_mask);
void APP_RequestTimestampInterval(uint32_t interval_frames);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void APP_RequestPCReset(uint8_t encoder_mask)
{
  /* Called from the USB receive callback.  Defer timer and stream changes to
   * a USB batch boundary in the main loop. */
  pc_reset_pending_mask |= (uint8_t)(encoder_mask & 0x01U);
}

void APP_SetPhysicalReferenceResetMask(uint8_t encoder_mask)
{
  physical_reference_reset_mask = (uint8_t)(encoder_mask & 0x01U);
}

void APP_RequestTimestampInterval(uint32_t interval_frames)
{
  if (interval_frames != 0U)
  {
    /* Consumed only when a timestamp interval is reloaded, avoiding a
     * volatile PC-controlled read in every encoder frame. */
    timestamp_interval_requested = interval_frames;
  }
}

static void APP_BuildReferenceEventFrame(uint8_t *frame,
                                         uint8_t event_type,
                                         uint8_t encoder_mask)
{
  frame[0] = 0xABU;
  frame[1] = event_type;
  frame[2] = encoder_mask;
  frame[3] = (uint8_t)(frame[0] ^ frame[1] ^ frame[2] ^ 0xFFU);
}

static uint16_t APP_BuildEncoderFrame(uint8_t *frame, uint16_t capacity)
{
  uint8_t count = active_encoder_count;
  uint8_t checksum = 0U;
  uint8_t i;
  uint16_t frame_length;
  uint16_t encoder_count =
      (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  if (count < 1U) count = 1U;
  if (count > 4U) count = 4U;

  frame_length = (uint16_t)(2U + (2U * count));
  if ((frame == NULL) || (capacity < frame_length))
  {
    return 0U;
  }

  frame[0] = 0xA5U;
  frame[1] = (uint8_t)(encoder_count & 0xFFU);
  frame[2] = (uint8_t)((encoder_count >> 8) & 0xFFU);

  /* This board has one encoder. Pad E2..E4 with zero if selected by mistake. */
  for (i = 1U; i < count; i++)
  {
    frame[1U + (2U * i)] = 0U;
    frame[2U + (2U * i)] = 0U;
  }

  for (i = 0U; i < (frame_length - 1U); i++)
  {
    checksum ^= frame[i];
  }
  frame[frame_length - 1U] = (uint8_t)(checksum ^ 0xFFU);

  return frame_length;
}

static void APP_SetEncoderConnectedLED(uint8_t connected)
{
  if (connected != 0U)
  {
    HAL_GPIO_WritePin(GPIOA, LED_ENCODER_RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_ENCODER_GREEN_PIN, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, LED_ENCODER_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_ENCODER_RED_PIN, GPIO_PIN_SET);
  }
}

static void APP_UpdateEncoderStatusLED(uint32_t now)
{
  uint32_t sample;
  uint8_t new_state = encoder_connected;

  if ((encoder_sense_valid != 0U) &&
      ((now - encoder_sense_last_ms) < ENCODER_SENSE_INTERVAL_MS))
  {
    return;
  }
  encoder_sense_last_ms = now;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    encoder_connected = 0U;
    APP_SetEncoderConnectedLED(0U);
    return;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 1U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    encoder_connected = 0U;
    APP_SetEncoderConnectedLED(0U);
    return;
  }

  sample = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  /* At a one-second sample interval the 100/50-count hysteresis provides
   * adequate noise rejection without delaying an LED change for seconds. */
  encoder_sense_filtered = sample;
  encoder_sense_valid = 1U;

  /* Hysteresis prevents flicker if the signal sits near a threshold. */
  if ((encoder_connected == 0U) &&
      (encoder_sense_filtered >= ENCODER_CONNECT_ADC))
  {
    new_state = 1U;
  }
  else if ((encoder_connected != 0U) &&
           (encoder_sense_filtered <= ENCODER_DISCONNECT_ADC))
  {
    new_state = 0U;
  }

  if (new_state != encoder_connected)
  {
    encoder_connected = new_state;
    APP_SetEncoderConnectedLED(encoder_connected);
  }
}

static void APP_UpdateUsbStatusLED(uint32_t now)
{
  uint8_t configured;

  if ((usb_led_configured != 0xFFU) &&
      ((now - usb_led_last_check_ms) < USB_LED_CHECK_INTERVAL_MS))
  {
    return;
  }

  usb_led_last_check_ms = now;
  configured = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;

  if (configured == usb_led_configured)
  {
    return;
  }

  usb_led_configured = configured;
  if (configured != 0U)
  {
    HAL_GPIO_WritePin(GPIOA, LED_USB_RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_USB_GREEN_PIN, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, LED_USB_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, LED_USB_RED_PIN, GPIO_PIN_SET);
  }
}

static void APP_BuildTimestampFrame(uint8_t *frame, uint32_t tick_ms)
{
  uint8_t checksum = 0U;
  uint8_t i;

  frame[0] = 0xAAU;
  frame[1] = (uint8_t)(tick_ms & 0xFFU);
  frame[2] = (uint8_t)((tick_ms >> 8) & 0xFFU);
  frame[3] = (uint8_t)((tick_ms >> 16) & 0xFFU);
  frame[4] = (uint8_t)((tick_ms >> 24) & 0xFFU);

  for (i = 0U; i < 5U; i++)
  {
    checksum ^= frame[i];
  }
  frame[5] = (uint8_t)(checksum ^ 0xFFU);
}

static uint16_t USB_GetBatchSize(void)
{
  uint16_t batch_size = usb_tx_batch_size;

  if (batch_size < 16U) batch_size = 16U;
  if (batch_size > USB_TX_BUFFER_CAPACITY)
  {
    batch_size = USB_TX_BUFFER_CAPACITY;
  }
  return batch_size;
}

static uint8_t USB_TryFlush(uint32_t now, uint8_t force_flush)
{
  uint16_t batch_size = USB_GetBatchSize();
  uint16_t threshold = usb_flush_threshold;
  uint8_t force_ms = usb_force_flush_ms;

  if (usb_fill_length == 0U)
  {
    return 1U;
  }

  if (threshold < 1U) threshold = 1U;
  if (threshold > batch_size) threshold = batch_size;
  if (force_ms < 1U) force_ms = 1U;
  if (force_ms > 20U) force_ms = 20U;

  if ((force_flush == 0U) &&
      (usb_fill_length < threshold) &&
      ((now - usb_fill_started_ms) < force_ms))
  {
    return 0U;
  }

  if (CDC_Transmit_FS(usb_tx_buffer[usb_fill_buffer],
                      usb_fill_length) == USBD_OK)
  {
    /* The other buffer is safe to fill while this buffer is transmitted. */
    usb_fill_buffer ^= 1U;
    usb_fill_length = 0U;
    usb_fill_started_ms = 0U;
    return 1U;
  }

  return 0U;
}

static void USB_ApplyPendingReferenceEventsAtBatchBoundary(uint32_t now)
{
  uint8_t pc_reset_mask;
  uint8_t physical_event_type;
  uint16_t physical_capture;
  uint32_t primask;

  /* Reference requests are consumed only at USB batch boundaries, keeping
   * all request polling out of the per-frame streaming path. */
  if (((pc_reset_pending_mask & 0x01U) == 0U) &&
      (physical_reference_event_type == 0U))
  {
    return;
  }

  /* Copy and clear the interrupt-owned request data atomically. */
  primask = __get_PRIMASK();
  __disable_irq();
  pc_reset_mask = (uint8_t)(pc_reset_pending_mask & 0x01U);
  pc_reset_pending_mask &= (uint8_t)~pc_reset_mask;
  physical_event_type = physical_reference_event_type;
  physical_capture = physical_reference_capture;
  physical_reference_event_type = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (physical_event_type != 0U)
  {
    if (physical_event_type == 0x01U)
    {
      uint16_t current_count =
          (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

      /* Move all reference arithmetic to this batch boundary.  Loading the
       * displacement since the captured Z edge preserves motion since the
       * reference while keeping subtraction out of every encoder frame. */
      __HAL_TIM_SET_COUNTER(
          &htim2, (uint16_t)(current_count - physical_capture));
    }

    APP_BuildReferenceEventFrame(reference_event_frame,
                                 physical_event_type, 0x01U);
    (void)USB_AppendFrame(reference_event_frame,
                          sizeof(reference_event_frame), now);
  }

  if (pc_reset_mask != 0U)
  {
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    APP_BuildReferenceEventFrame(reference_event_frame,
                                 0x03U, pc_reset_mask);
    (void)USB_AppendFrame(reference_event_frame,
                          sizeof(reference_event_frame), now);
  }
}

static uint8_t USB_AppendFrame(const uint8_t *frame, uint16_t length,
                               uint32_t now)
{
  uint16_t batch_size = USB_GetBatchSize();

  if ((frame == NULL) || (length == 0U) || (length > batch_size))
  {
    return 0U;
  }

  if ((usb_fill_length + length) > batch_size)
  {
    (void)USB_TryFlush(now, 1U);
    if ((usb_fill_length + length) > batch_size)
    {
      return 0U;
    }
  }

  if (usb_fill_length == 0U)
  {
    usb_fill_started_ms = now;
  }

  memcpy(&usb_tx_buffer[usb_fill_buffer][usb_fill_length], frame, length);
  usb_fill_length += length;
  return 1U;
}

static void USB_StreamTask(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t timestamp_sent = 0U;

  /* One existing time comparison services low-priority commands within
   * 500 ms and calls the separately rate-limited one-second LED checks. */
  if ((now - background_service_last_ms) >=
      BACKGROUND_SERVICE_INTERVAL_MS)
  {
    if (((pc_reset_pending_mask & 0x01U) != 0U) ||
        (physical_reference_event_type != 0U))
    {
      /* Preserve stream ordering: all pre-event samples must be submitted
       * before the AB event and the first post-event encoder sample. */
      if ((usb_fill_length != 0U) && (USB_TryFlush(now, 1U) == 0U))
      {
        return;
      }
      USB_ApplyPendingReferenceEventsAtBatchBoundary(now);
    }

    background_service_last_ms = now;
    APP_UpdateUsbStatusLED(now);
    APP_UpdateEncoderStatusLED(now);
  }

  if (usb_stream_enabled == 0U)
  {
    usb_fill_length = 0U;
    usb_fill_started_ms = 0U;
    frames_until_timestamp = timestamp_interval_requested;
    USB_ApplyPendingReferenceEventsAtBatchBoundary(now);
    usb_fill_length = 0U;
    usb_fill_started_ms = 0U;
    return;
  }

  if (frames_until_timestamp == 0U)
  {
    /* Use the existing timestamp boundary to service physical-reference and
     * PC-zero requests.  Flush all pre-event samples first, then place any
     * AB event ahead of the next encoder sample.  This avoids checking for
     * pending commands at every USB batch boundary. */
    if ((usb_fill_length != 0U) && (USB_TryFlush(now, 1U) == 0U))
    {
      return;
    }

    USB_ApplyPendingReferenceEventsAtBatchBoundary(now);
    APP_BuildTimestampFrame(timestamp_frame, now);
    if (USB_AppendFrame(timestamp_frame, sizeof(timestamp_frame), now) != 0U)
    {
      frames_until_timestamp = timestamp_interval_requested;
      timestamp_sent = 1U;
    }
  }
  else
  {
    uint16_t frame_length =
        APP_BuildEncoderFrame(encoder_frame, sizeof(encoder_frame));

    if ((frame_length != 0U) &&
        (USB_AppendFrame(encoder_frame, frame_length, now) != 0U))
    {
      frames_until_timestamp--;
    }
  }

  (void)USB_TryFlush(now, timestamp_sent);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  APP_SetEncoderConnectedLED(0U);
  APP_UpdateUsbStatusLED(HAL_GetTick());
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Allow USB to initialise before powering the encoder. */
  HAL_Delay(500);

  /* Encoder power ON: pull the PB10 open-drain output low. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /* Allow the encoder supply to settle, then start TIM2 encoder mode. */
  HAL_Delay(100);
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  usb_fill_buffer = 0U;
  usb_fill_length = 0U;
  usb_fill_started_ms = 0U;
  frames_until_timestamp = timestamp_interval_requested;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    USB_StreamTask();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE |
                                     RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc1.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_384CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /* Start all four active-high LED channels switched off. */
  HAL_GPIO_WritePin(GPIOA,
                    LED_USB_RED_PIN | LED_USB_GREEN_PIN |
                    LED_ENCODER_GREEN_PIN | LED_ENCODER_RED_PIN,
                    GPIO_PIN_RESET);

  /* Configure PA6, PA7, PA8 and PA9 as status LED outputs. */
  GPIO_InitStruct.Pin = LED_USB_RED_PIN | LED_USB_GREEN_PIN |
                        LED_ENCODER_GREEN_PIN | LED_ENCODER_RED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure PA2 as the encoder Z/reference interrupt input. */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2)
  {
    /* Capture the exact Z-edge position.  The viewer's C6 mask determines
     * whether this becomes a reset event (type 1) or continue event (type 2). */
    if (physical_reference_event_type == 0U)
    {
      physical_reference_capture =
          (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
      physical_reference_event_type =
          ((physical_reference_reset_mask & 0x01U) != 0U) ? 0x01U : 0x02U;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
