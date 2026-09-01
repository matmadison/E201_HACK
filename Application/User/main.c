/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32L151 single incremental encoder USB CDC streamer - PB10 active-low encoder 5V enable
  ******************************************************************************
  * Encoder wiring used by this variant:
  *   B / TIM2_CH1 = PA0 pin 10
  *   A / TIM2_CH2 = PA1 pin 11
  *   Z / Index    = PA2 pin 12, EXTI2 rising edge
  *
  * USB packet protocol is unchanged from the single-encoder PC program:
  *   Encoder frame:   A5 E1_L E1_H CHK
  *   Timestamp frame: AA T0 T1 T2 T3 CHK
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_encoder.h"
#include "usbd_cdc_if.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * USB throughput controls.
 * ENCODER_SEND_DIVIDER = 1 sends every generated encoder sample.
 * ENCODER_SEND_DIVIDER = 10 sends one in ten.
 */
#define TIMESTAMP_INTERVAL_FRAMES 1000U
#define ENCODER_SEND_DIVIDER      1U

#define USB_FS_PACKET_SIZE        64U
#define USB_TX_BATCH_SIZE         240U
#define USB_FLUSH_THRESHOLD       240U
#define USB_FORCE_FLUSH_MS        1U

/* Encoder 5V enable test:
 * PB10 is treated as an active-low, externally pulled-up enable node.
 * It is configured open-drain and pulled LOW to enable the encoder 5V rail.
 * PB12 and PB13 are left high-impedance analogue/no-pull.
 * PB13 appears connected to the current-monitor OUT, so it must not be driven.
 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
extern volatile uint8_t cdc_tx_busy_fs;

static uint8_t encoder_frame[4];     /* A5 + E1 int16 + checksum */
static uint8_t timestamp_frame[6];   /* AA + uint32 ms + checksum */

static uint8_t usb_fill_buf[USB_TX_BATCH_SIZE];
static uint8_t usb_tx_buf[USB_FS_PACKET_SIZE];
static uint16_t usb_fill_len = 0U;
static uint8_t usb_tx_inflight = 0U;
static uint8_t usb_force_flush = 0U;
static uint32_t usb_fill_started_ms = 0U;

volatile uint8_t x_index_zero_pending = 0U;

static uint32_t encoder_frame_count = 0U;
static uint32_t encoder_send_divider_count = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t USB_AppendFrame(const uint8_t *frame, uint16_t len);
static void USB_PromoteFillToTx(void);
static void USB_PumpTx(void);
static void USB_StreamTask(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t USB_AppendFrame(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len == 0U) || (len > USB_TX_BATCH_SIZE))
    {
        return 0U;
    }

    if ((usb_fill_len + len) > USB_TX_BATCH_SIZE)
    {
        USB_PromoteFillToTx();
        USB_PumpTx();
    }

    if ((usb_fill_len + len) > USB_TX_BATCH_SIZE)
    {
        return 0U;
    }

    if (usb_fill_len == 0U)
    {
        usb_fill_started_ms = HAL_GetTick();
    }

    memcpy(&usb_fill_buf[usb_fill_len], frame, len);
    usb_fill_len += len;
    return 1U;
}

static void USB_PromoteFillToTx(void)
{
    usb_force_flush = 1U;
}

static void USB_PumpTx(void)
{
    uint16_t chunk_len;

    if ((usb_tx_inflight != 0U) && (cdc_tx_busy_fs == 0U))
    {
        usb_tx_inflight = 0U;
    }

    if ((usb_tx_inflight != 0U) || (cdc_tx_busy_fs != 0U) || (usb_fill_len == 0U))
    {
        return;
    }

    if ((usb_force_flush == 0U) && (usb_fill_len < USB_FLUSH_THRESHOLD))
    {
        return;
    }

    chunk_len = usb_fill_len;
    if (chunk_len > USB_FS_PACKET_SIZE)
    {
        chunk_len = USB_FS_PACKET_SIZE;
    }

    memcpy(usb_tx_buf, usb_fill_buf, chunk_len);

    if (CDC_Transmit_FS(usb_tx_buf, chunk_len) == USBD_OK)
    {
        usb_tx_inflight = 1U;
        usb_fill_len = (uint16_t)(usb_fill_len - chunk_len);
        if (usb_fill_len > 0U)
        {
            memmove(usb_fill_buf, &usb_fill_buf[chunk_len], usb_fill_len);
            usb_fill_started_ms = HAL_GetTick();
        }
        else
        {
            usb_force_flush = 0U;
        }
    }
}

static void USB_StreamTask(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t frame_queued = 0U;

    USB_PumpTx();

    if (encoder_frame_count >= TIMESTAMP_INTERVAL_FRAMES)
    {
        APP_BuildTimestampPacket(timestamp_frame, sizeof(timestamp_frame), now);

        if (USB_AppendFrame(timestamp_frame, sizeof(timestamp_frame)) != 0U)
        {
            encoder_frame_count = 0U;
            frame_queued = 1U;
        }
    }
    else
    {
        encoder_send_divider_count++;

        if (encoder_send_divider_count >= ENCODER_SEND_DIVIDER)
        {
            encoder_send_divider_count = 0U;

            APP_Encoders_BuildRel16Packet(encoder_frame, sizeof(encoder_frame));

            if (USB_AppendFrame(encoder_frame, sizeof(encoder_frame)) != 0U)
            {
                encoder_frame_count++;
                frame_queued = 1U;
            }
        }
    }

    if (usb_fill_len >= USB_FLUSH_THRESHOLD)
    {
        USB_PromoteFillToTx();
    }

    if ((usb_fill_len > 0U) && ((now - usb_fill_started_ms) >= USB_FORCE_FLUSH_MS))
    {
        USB_PromoteFillToTx();
    }

    if ((frame_queued != 0U) && (encoder_frame_count == 0U))
    {
        USB_PromoteFillToTx();
    }

    USB_PumpTx();
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  HAL_Delay(100);  /* allow PB10-enabled encoder 5V rail to rise */
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  APP_Encoders_Init();
  HAL_Delay(1000);
  usb_fill_len = 0U;
  usb_tx_inflight = 0U;
  usb_force_flush = 0U;
  encoder_frame_count = 0U;
  encoder_send_divider_count = 0U;
  /* PB10 is held low in open-drain mode to enable the encoder 5V rail. */
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    APP_Encoders_Update();

    if (x_index_zero_pending != 0U)
    {
      x_index_zero_pending = 0U;
      APP_Encoders_ZeroFromXIndex();
    }

    USB_StreamTask();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /*
   * STM32L151 with 16 MHz HSE crystal.
   * PLL VCO = 16 MHz x 6 = 96 MHz.
   * SYSCLK = PLL / 3 = 32 MHz.
   * USB FS clock on USB-capable STM32L1 parts is derived from this PLL setting.
   * Confirm the Clock Configuration tab is valid for your exact STM32L151 part number.
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();

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
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;

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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA0 = TIM2_CH1, PA1 = TIM2_CH2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /* PB10 = active-low encoder 5V enable.
     The original firmware behaviour appears to be:
       PB10 LOW       -> encoder 5V ON
       PB10 released  -> external pull-up to ~4.65V -> encoder 5V OFF
     Use open-drain so the STM32 never drives this externally pulled-up node high. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* preload released before init */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); /* pull low = encoder 5V ON */

  /* PB12/PB13 are sense/monitor-related lines. Do not drive them.
     PB13 appears connected to INA current-monitor OUT.
     PB12 appears connected to encoder 5V sense through 10 MOhm. */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA6/PA7 drive the status LED on the original board; keep them inactive. */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA2 = encoder Z/index input */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_2)
  {
    x_index_zero_pending = 1U;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
