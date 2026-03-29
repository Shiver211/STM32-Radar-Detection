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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  LD6002_WAIT_SOF = 0,
  LD6002_ID_HIGH,
  LD6002_ID_LOW,
  LD6002_LEN_HIGH,
  LD6002_LEN_LOW,
  LD6002_TYPE_HIGH,
  LD6002_TYPE_LOW,
  LD6002_HEAD_CKSUM,
  LD6002_DATA,
  LD6002_DATA_CKSUM
} LD6002_ParseState;

typedef struct
{
  uint16_t id;
  uint16_t len;
  uint16_t type;
  uint8_t data[128];
} LD6002_Frame;

typedef struct
{
  LD6002_ParseState state;
  uint8_t header[7];
  uint8_t header_idx;
  uint16_t data_idx;
  LD6002_Frame frame;
} LD6002_Parser;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LD6002_SOF 0x01U
#define LD6002_MAX_PAYLOAD_LEN 128U
#define LD6002_FRAME_QUEUE_SIZE 8U
#define LD6002_UART_TIMEOUT_MS 50U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t s_uart2_rx_byte;
static LD6002_Parser s_ld6002_parser;
static LD6002_Frame s_frame_queue[LD6002_FRAME_QUEUE_SIZE];
static volatile uint8_t s_frame_head;
static volatile uint8_t s_frame_tail;
static volatile uint32_t s_frame_dropped;
static volatile uint32_t s_cksum_error;
static volatile uint32_t s_len_error;
static volatile uint32_t s_sof_skip;
static uint32_t s_last_stats_tick;
static uint32_t s_last_dropped_report;
static uint32_t s_last_cksum_report;
static uint32_t s_last_len_report;
static uint32_t s_last_sof_report;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LD6002_ParserReset(LD6002_Parser *parser);
static uint8_t LD6002_Checksum(const uint8_t *data, uint16_t len);
static bool LD6002_ConsumeByte(LD6002_Parser *parser, uint8_t byte, LD6002_Frame *out_frame);
static void LD6002_QueuePushFromISR(const LD6002_Frame *frame);
static bool LD6002_QueuePop(LD6002_Frame *frame);
static void LD6002_StartUart2Rx(void);
static uint32_t LD6002_U32Le(const uint8_t *buf);
static float LD6002_F32Le(const uint8_t *buf);
static void LD6002_FormatFloat(float value, char *out, size_t out_size);
static void UART1_SendText(const char *text);
static void UART1_Sendf(const char *fmt, ...);
static void LD6002_LogFrame(const LD6002_Frame *frame);
static void LD6002_ProcessFrames(void);
static void LD6002_ReportStats(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void LD6002_ParserReset(LD6002_Parser *parser)
{
  parser->state = LD6002_WAIT_SOF;
  parser->header_idx = 0U;
  parser->data_idx = 0U;
  parser->frame.id = 0U;
  parser->frame.len = 0U;
  parser->frame.type = 0U;
}

static uint8_t LD6002_Checksum(const uint8_t *data, uint16_t len)
{
  uint8_t cksum = 0U;
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    cksum ^= data[i];
  }

  return (uint8_t)(~cksum);
}

static bool LD6002_ConsumeByte(LD6002_Parser *parser, uint8_t byte, LD6002_Frame *out_frame)
{
  uint8_t expected_cksum;

  switch (parser->state)
  {
  case LD6002_WAIT_SOF:
    if (byte == LD6002_SOF)
    {
      parser->header_idx = 0U;
      parser->data_idx = 0U;
      parser->header[parser->header_idx++] = byte;
      parser->frame.id = 0U;
      parser->frame.len = 0U;
      parser->frame.type = 0U;
      parser->state = LD6002_ID_HIGH;
    }
    else
    {
      s_sof_skip++;
    }
    break;

  case LD6002_ID_HIGH:
    parser->frame.id = ((uint16_t)byte << 8);
    parser->header[parser->header_idx++] = byte;
    parser->state = LD6002_ID_LOW;
    break;

  case LD6002_ID_LOW:
    parser->frame.id |= byte;
    parser->header[parser->header_idx++] = byte;
    parser->state = LD6002_LEN_HIGH;
    break;

  case LD6002_LEN_HIGH:
    parser->frame.len = ((uint16_t)byte << 8);
    parser->header[parser->header_idx++] = byte;
    parser->state = LD6002_LEN_LOW;
    break;

  case LD6002_LEN_LOW:
    parser->frame.len |= byte;
    parser->header[parser->header_idx++] = byte;
    if (parser->frame.len > LD6002_MAX_PAYLOAD_LEN)
    {
      s_len_error++;
      LD6002_ParserReset(parser);
    }
    else
    {
      parser->state = LD6002_TYPE_HIGH;
    }
    break;

  case LD6002_TYPE_HIGH:
    parser->frame.type = ((uint16_t)byte << 8);
    parser->header[parser->header_idx++] = byte;
    parser->state = LD6002_TYPE_LOW;
    break;

  case LD6002_TYPE_LOW:
    parser->frame.type |= byte;
    parser->header[parser->header_idx++] = byte;
    parser->state = LD6002_HEAD_CKSUM;
    break;

  case LD6002_HEAD_CKSUM:
    expected_cksum = LD6002_Checksum(parser->header, (uint16_t)sizeof(parser->header));
    if (byte != expected_cksum)
    {
      s_cksum_error++;
      LD6002_ParserReset(parser);
      break;
    }

    if (parser->frame.len == 0U)
    {
      parser->state = LD6002_DATA_CKSUM;
    }
    else
    {
      parser->data_idx = 0U;
      parser->state = LD6002_DATA;
    }
    break;

  case LD6002_DATA:
    parser->frame.data[parser->data_idx++] = byte;
    if (parser->data_idx >= parser->frame.len)
    {
      parser->state = LD6002_DATA_CKSUM;
    }
    break;

  case LD6002_DATA_CKSUM:
    expected_cksum = LD6002_Checksum(parser->frame.data, parser->frame.len);
    if (byte == expected_cksum)
    {
      *out_frame = parser->frame;
      LD6002_ParserReset(parser);
      return true;
    }

    s_cksum_error++;
    LD6002_ParserReset(parser);
    break;

  default:
    LD6002_ParserReset(parser);
    break;
  }

  return false;
}

static void LD6002_QueuePushFromISR(const LD6002_Frame *frame)
{
  uint8_t next_head = (uint8_t)((s_frame_head + 1U) % LD6002_FRAME_QUEUE_SIZE);

  if (next_head == s_frame_tail)
  {
    s_frame_dropped++;
    return;
  }

  s_frame_queue[s_frame_head] = *frame;
  s_frame_head = next_head;
}

static bool LD6002_QueuePop(LD6002_Frame *frame)
{
  if (s_frame_tail == s_frame_head)
  {
    return false;
  }

  *frame = s_frame_queue[s_frame_tail];
  s_frame_tail = (uint8_t)((s_frame_tail + 1U) % LD6002_FRAME_QUEUE_SIZE);
  return true;
}

static void LD6002_StartUart2Rx(void)
{
  if (HAL_UART_Receive_IT(&huart2, &s_uart2_rx_byte, 1U) != HAL_OK)
  {
    UART1_SendText("[RADAR] UART2 RX start failed\r\n");
  }
}

static uint32_t LD6002_U32Le(const uint8_t *buf)
{
  return ((uint32_t)buf[0]) |
         ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

static float LD6002_F32Le(const uint8_t *buf)
{
  uint32_t raw = LD6002_U32Le(buf);
  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static void LD6002_FormatFloat(float value, char *out, size_t out_size)
{
  int32_t scaled = (int32_t)(value * 1000.0f);
  int32_t abs_scaled = (scaled < 0) ? -scaled : scaled;
  int32_t integer = abs_scaled / 1000;
  int32_t decimal = abs_scaled % 1000;

  if (scaled < 0)
  {
    (void)snprintf(out, out_size, "-%ld.%03ld", (long)integer, (long)decimal);
  }
  else
  {
    (void)snprintf(out, out_size, "%ld.%03ld", (long)integer, (long)decimal);
  }
}

static void UART1_SendText(const char *text)
{
  size_t len = strlen(text);
  if (len > 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, LD6002_UART_TIMEOUT_MS);
  }
}

static void UART1_Sendf(const char *fmt, ...)
{
  char tx_buf[192];
  va_list args;
  int written;

  va_start(args, fmt);
  written = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
  va_end(args);

  if (written > 0)
  {
    uint16_t tx_len = (written >= (int)sizeof(tx_buf)) ? ((uint16_t)sizeof(tx_buf) - 1U) : (uint16_t)written;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, tx_len, LD6002_UART_TIMEOUT_MS);
  }
}

static void LD6002_LogFrame(const LD6002_Frame *frame)
{
  char a[20];
  char b[20];
  char c[20];
  uint16_t i;
  char hex_buf[96];
  int pos = 0;

  switch (frame->type)
  {
  case 0x0F09:
    if (frame->len >= 2U)
    {
      uint16_t is_human = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
      UART1_Sendf("[RADAR] TYPE=0x0F09 ID=0x%04X Human=%u\r\n", frame->id, (unsigned)is_human);
    }
    break;

  case 0x0A13:
    if (frame->len >= 12U)
    {
      float total_phase = LD6002_F32Le(&frame->data[0]);
      float breath_phase = LD6002_F32Le(&frame->data[4]);
      float heart_phase = LD6002_F32Le(&frame->data[8]);
      LD6002_FormatFloat(total_phase, a, sizeof(a));
      LD6002_FormatFloat(breath_phase, b, sizeof(b));
      LD6002_FormatFloat(heart_phase, c, sizeof(c));
      UART1_Sendf("[RADAR] TYPE=0x0A13 ID=0x%04X Total=%s Breath=%s Heart=%s\r\n", frame->id, a, b, c);
    }
    else
    {
      UART1_Sendf("[RADAR] TYPE=0x0A13 LEN_ERR len=%u\r\n", frame->len);
    }
    break;

  case 0x0A14:
    if (frame->len >= 4U)
    {
      float breath_rate = LD6002_F32Le(&frame->data[0]);
      LD6002_FormatFloat(breath_rate, a, sizeof(a));
      UART1_Sendf("[RADAR] TYPE=0x0A14 ID=0x%04X BreathRate=%s bpm\r\n", frame->id, a);
    }
    break;

  case 0x0A15:
    if (frame->len >= 4U)
    {
      float heart_rate = LD6002_F32Le(&frame->data[0]);
      LD6002_FormatFloat(heart_rate, a, sizeof(a));
      UART1_Sendf("[RADAR] TYPE=0x0A15 ID=0x%04X HeartRate=%s bpm\r\n", frame->id, a);
    }
    break;

  case 0x0A16:
    if (frame->len >= 8U)
    {
      uint32_t flag = LD6002_U32Le(&frame->data[0]);
      float range = LD6002_F32Le(&frame->data[4]);
      LD6002_FormatFloat(range, a, sizeof(a));
      UART1_Sendf("[RADAR] TYPE=0x0A16 ID=0x%04X Flag=%lu Range=%s cm\r\n", frame->id, (unsigned long)flag, a);
    }
    break;

  default:
    for (i = 0U; (i < frame->len) && (i < 20U); i++)
    {
      pos += snprintf(&hex_buf[pos], sizeof(hex_buf) - (size_t)pos, "%02X ", frame->data[i]);
      if (pos >= (int)(sizeof(hex_buf) - 4U))
      {
        break;
      }
    }
    UART1_Sendf("[RADAR] TYPE=0x%04X ID=0x%04X LEN=%u DATA=%s\r\n",
                frame->type,
                frame->id,
                frame->len,
                (pos > 0) ? hex_buf : "");
    break;
  }
}

static void LD6002_ProcessFrames(void)
{
  LD6002_Frame frame;

  while (LD6002_QueuePop(&frame))
  {
    LD6002_LogFrame(&frame);
  }
}

static void LD6002_ReportStats(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - s_last_stats_tick) < 1000U)
  {
    return;
  }

  s_last_stats_tick = now;

  if ((s_frame_dropped != s_last_dropped_report) ||
      (s_cksum_error != s_last_cksum_report) ||
      (s_len_error != s_last_len_report) ||
      (s_sof_skip != s_last_sof_report))
  {
    UART1_Sendf("[RADAR][STAT] drop=%lu cksum=%lu len=%lu sof_skip=%lu\r\n",
                (unsigned long)s_frame_dropped,
                (unsigned long)s_cksum_error,
                (unsigned long)s_len_error,
                (unsigned long)s_sof_skip);

    s_last_dropped_report = s_frame_dropped;
    s_last_cksum_report = s_cksum_error;
    s_last_len_report = s_len_error;
    s_last_sof_report = s_sof_skip;
  }
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LD6002_ParserReset(&s_ld6002_parser);
  LD6002_StartUart2Rx();
  UART1_SendText("\r\n[RADAR] UART2->TF parser started\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LD6002_ProcessFrames();
    LD6002_ReportStats();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : RADAR_STATUS_Pin */
  GPIO_InitStruct.Pin = RADAR_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RADAR_STATUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  LD6002_Frame frame;

  if (huart->Instance == USART2)
  {
    if (LD6002_ConsumeByte(&s_ld6002_parser, s_uart2_rx_byte, &frame))
    {
      LD6002_QueuePushFromISR(&frame);
    }

    (void)HAL_UART_Receive_IT(&huart2, &s_uart2_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    (void)HAL_UART_Receive_IT(&huart2, &s_uart2_rx_byte, 1U);
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
