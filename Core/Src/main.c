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
#include "oled.h"

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

typedef enum
{
  OLED_PANEL_VITALS = 0,
  OLED_PANEL_NO_DATA,
  OLED_PANEL_NO_HUMAN,
  OLED_PANEL_DIST_TOO_FAR,
  OLED_PANEL_DIST_TOO_NEAR,
  OLED_PANEL_MODE_COUNT
} OLED_RightPanelMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LD6002_SOF 0x01U
#define LD6002_MAX_PAYLOAD_LEN 128U
#define LD6002_FRAME_QUEUE_SIZE 16U
#define LD6002_UART_TIMEOUT_MS 50U
#define LD6002_NO_DATA_TIMEOUT_MS 1000U
#define LD6002_TARGET_RANGE_MIN_CM 25.0f
#define LD6002_TARGET_RANGE_MAX_CM 100.0f

#define OLED_WAVE_X_START 0U
#define OLED_WAVE_X_END   95U
#define OLED_WAVE_WIDTH   ((OLED_WAVE_X_END - OLED_WAVE_X_START) + 1U)

#define OLED_VALUE_AREA_X_START 96U
#define OLED_VALUE_AREA_X_END   127U
#define OLED_VALUE_AREA_WIDTH   ((OLED_VALUE_AREA_X_END - OLED_VALUE_AREA_X_START) + 1U)

#define OLED_VALUE_LABEL_X OLED_VALUE_AREA_X_START
#define OLED_VALUE_DATA_X  100U

#define OLED_TEXT_2CHAR_X (OLED_VALUE_AREA_X_START + ((OLED_VALUE_AREA_WIDTH - 16U) / 2U))
#define OLED_TEXT_3CHAR_X (OLED_VALUE_AREA_X_START + ((OLED_VALUE_AREA_WIDTH - 24U) / 2U))
#define OLED_TEXT_4CHAR_X (OLED_VALUE_AREA_X_START + ((OLED_VALUE_AREA_WIDTH - 32U) / 2U))

#define OLED_TEXT_LINE1_Y 2U
#define OLED_TEXT_LINE2_Y 5U

#define OLED_ZH_XIN 0U
#define OLED_ZH_LV  1U
#define OLED_ZH_HU  2U
#define OLED_ZH_XI  3U
#define OLED_ZH_WU  4U
#define OLED_ZH_REN 5U
#define OLED_ZH_CUO 10U
#define OLED_ZH_WU2 11U

#define OLED_BREATH_Y_MIN    1U
#define OLED_BREATH_Y_MAX    30U
#define OLED_BREATH_CENTER_Y 15U
#define OLED_BREATH_GAIN     22.0f

#define OLED_HEART_Y_MIN    33U
#define OLED_HEART_Y_MAX    62U
#define OLED_HEART_CENTER_Y 47U
#define OLED_HEART_GAIN     18.0f

#define OLED_WAVE_SPLIT_Y 31U

#define OLED_WAVE_UPDATE_MS 50U
#define OLED_TEXT_UPDATE_MS 200U

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

static bool s_oled_ready;
static bool s_oled_rate_dirty;
static bool s_oled_wave_pending;
static bool s_oled_wave_prev_valid;
static uint8_t s_oled_wave_x;
static uint8_t s_prev_breath_y;
static uint8_t s_prev_heart_y;
static float s_latest_breath_rate;
static float s_latest_heart_rate;
static float s_pending_breath_phase;
static float s_pending_heart_phase;
static uint32_t s_last_oled_wave_tick;
static uint32_t s_last_oled_text_tick;
static uint32_t s_last_radar_frame_tick;
static bool s_radar_data_online;
static bool s_human_detect_valid;
static bool s_human_present;
static bool s_target_range_valid;
static float s_target_range_cm;
static bool s_oled_axes_only_mode;
static OLED_RightPanelMode s_oled_panel_drawn_mode;

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
static uint8_t OLED_MapPhaseToY(float value, uint8_t center_y, float gain, uint8_t y_min, uint8_t y_max);
static void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
static void OLED_DrawWaveAxes(void);
static void OLED_ClearWaveAndKeepAxes(void);
static bool OLED_ShouldAxesOnly(void);
static OLED_RightPanelMode OLED_GetPanelMode(void);
static void OLED_DrawPanelMode(OLED_RightPanelMode mode);
static void OLED_PlotWaveColumn(float breath_phase, float heart_phase);
static void OLED_UpdateVitalsText(void);
static void OLED_SetBreathRate(float rate);
static void OLED_SetHeartRate(float rate);
static void OLED_SetPhases(float breath_phase, float heart_phase);
static void OLED_Service(void);
static void OLED_UI_Init(void);

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

static uint8_t OLED_MapPhaseToY(float value, uint8_t center_y, float gain, uint8_t y_min, uint8_t y_max)
{
  int32_t y = (int32_t)center_y - (int32_t)(value * gain);

  if (y < (int32_t)y_min)
  {
    y = (int32_t)y_min;
  }
  else if (y > (int32_t)y_max)
  {
    y = (int32_t)y_max;
  }

  return (uint8_t)y;
}

static void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int16_t dx = (int16_t)x1 - (int16_t)x0;
  int16_t dy = (int16_t)y1 - (int16_t)y0;
  int16_t sx = (dx >= 0) ? 1 : -1;
  int16_t sy = (dy >= 0) ? 1 : -1;
  int16_t abs_dx = (dx >= 0) ? dx : (int16_t)-dx;
  int16_t abs_dy = (dy >= 0) ? dy : (int16_t)-dy;
  int16_t err = abs_dx - abs_dy;
  int16_t e2;

  while (1)
  {
    OLED_DrawPoint((u8)x0, (u8)y0, 1U);

    if ((x0 == x1) && (y0 == y1))
    {
      break;
    }

    e2 = (int16_t)(2 * err);

    if (e2 > -abs_dy)
    {
      err = (int16_t)(err - abs_dy);
      x0 = (uint8_t)((int16_t)x0 + sx);
    }

    if (e2 < abs_dx)
    {
      err = (int16_t)(err + abs_dx);
      y0 = (uint8_t)((int16_t)y0 + sy);
    }
  }
}

static void OLED_DrawWaveAxes(void)
{
  uint8_t x;

  for (x = OLED_WAVE_X_START; x <= OLED_WAVE_X_END; x++)
  {
    if ((x & 0x01U) == 0U)
    {
      OLED_DrawPoint(x, OLED_WAVE_SPLIT_Y, 1U);
    }

    OLED_DrawPoint(x, OLED_BREATH_CENTER_Y, 1U);
    OLED_DrawPoint(x, OLED_HEART_CENTER_Y, 1U);
  }
}

static void OLED_ClearWaveAndKeepAxes(void)
{
  uint8_t x;
  uint8_t y;

  for (x = OLED_WAVE_X_START; x <= OLED_WAVE_X_END; x++)
  {
    for (y = OLED_BREATH_Y_MIN; y <= OLED_HEART_Y_MAX; y++)
    {
      OLED_DrawPoint(x, y, 0U);
    }
  }

  OLED_DrawWaveAxes();
  OLED_RefreshDirty();
}

static bool OLED_ShouldAxesOnly(void)
{
  if (!s_radar_data_online)
  {
    return true;
  }

  if (s_human_detect_valid && !s_human_present)
  {
    return true;
  }

  return false;
}

static OLED_RightPanelMode OLED_GetPanelMode(void)
{
  if (!s_radar_data_online)
  {
    return OLED_PANEL_NO_DATA;
  }

  if (s_human_detect_valid && !s_human_present)
  {
    return OLED_PANEL_NO_HUMAN;
  }

  if (s_target_range_valid)
  {
    if (s_target_range_cm > LD6002_TARGET_RANGE_MAX_CM)
    {
      return OLED_PANEL_DIST_TOO_FAR;
    }

    if (s_target_range_cm < LD6002_TARGET_RANGE_MIN_CM)
    {
      return OLED_PANEL_DIST_TOO_NEAR;
    }
  }

  return OLED_PANEL_VITALS;
}

static void OLED_DrawPanelMode(OLED_RightPanelMode mode)
{
  OLED_Fill(OLED_VALUE_AREA_X_START, 0U, OLED_VALUE_AREA_X_END, 63U, 0U);

  switch (mode)
  {
  case OLED_PANEL_VITALS:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 0U, OLED_ZH_HU); /* 呼 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 0U, OLED_ZH_XI); /* 吸 */
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 4U, OLED_ZH_XIN); /* 心 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 4U, OLED_ZH_LV); /* 率 */
    break;

  case OLED_PANEL_NO_DATA:
    OLED_ShowString(OLED_TEXT_2CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"No", 8U);
    OLED_ShowString(OLED_TEXT_4CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Data", 8U);
    break;

  case OLED_PANEL_NO_HUMAN:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 0U, OLED_ZH_CUO); /* 错 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 0U, OLED_ZH_WU2); /* 误 */
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 4U, OLED_ZH_WU); /* 无 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 4U, OLED_ZH_REN); /* 人 */
    break;

  case OLED_PANEL_DIST_TOO_FAR:
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"Too", 8U);
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Far", 8U);
    break;

  case OLED_PANEL_DIST_TOO_NEAR:
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"Too", 8U);
    OLED_ShowString(OLED_TEXT_4CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Near", 8U);
    break;

  default:
    break;
  }
}

static void OLED_PlotWaveColumn(float breath_phase, float heart_phase)
{
  uint8_t x = (uint8_t)(OLED_WAVE_X_START + s_oled_wave_x);
  uint8_t y;
  uint8_t breath_y;
  uint8_t heart_y;

  for (y = OLED_BREATH_Y_MIN; y <= OLED_BREATH_Y_MAX; y++)
  {
    OLED_DrawPoint(x, y, 0U);
  }

  for (y = OLED_HEART_Y_MIN; y <= OLED_HEART_Y_MAX; y++)
  {
    OLED_DrawPoint(x, y, 0U);
  }

  OLED_DrawPoint(x, OLED_BREATH_CENTER_Y, 1U);
  OLED_DrawPoint(x, OLED_HEART_CENTER_Y, 1U);

  breath_y = OLED_MapPhaseToY(breath_phase,
                              OLED_BREATH_CENTER_Y,
                              OLED_BREATH_GAIN,
                              OLED_BREATH_Y_MIN,
                              OLED_BREATH_Y_MAX);

  heart_y = OLED_MapPhaseToY(heart_phase,
                             OLED_HEART_CENTER_Y,
                             OLED_HEART_GAIN,
                             OLED_HEART_Y_MIN,
                             OLED_HEART_Y_MAX);

  if (s_oled_wave_prev_valid && (s_oled_wave_x > 0U))
  {
    OLED_DrawLine((uint8_t)(x - 1U), s_prev_breath_y, x, breath_y);
    OLED_DrawLine((uint8_t)(x - 1U), s_prev_heart_y, x, heart_y);
  }
  else
  {
    OLED_DrawPoint(x, breath_y, 1U);
    OLED_DrawPoint(x, heart_y, 1U);
  }

  s_prev_breath_y = breath_y;
  s_prev_heart_y = heart_y;
  s_oled_wave_prev_valid = true;
  s_oled_wave_x++;

  if (s_oled_wave_x >= OLED_WAVE_WIDTH)
  {
    s_oled_wave_x = 0U;
    s_oled_wave_prev_valid = false;
  }

  OLED_RefreshDirty();
}

static void OLED_UpdateVitalsText(void)
{
  OLED_RightPanelMode panel_mode;
  int32_t br = (int32_t)(s_latest_breath_rate + ((s_latest_breath_rate >= 0.0f) ? 0.5f : -0.5f));
  int32_t hr = (int32_t)(s_latest_heart_rate + ((s_latest_heart_rate >= 0.0f) ? 0.5f : -0.5f));
  uint16_t br_u;
  uint16_t hr_u;
  char br_text[8];
  char hr_text[8];

  panel_mode = OLED_GetPanelMode();
  if (panel_mode != s_oled_panel_drawn_mode)
  {
    OLED_DrawPanelMode(panel_mode);
    s_oled_panel_drawn_mode = panel_mode;
  }

  if (panel_mode != OLED_PANEL_VITALS)
  {
    return;
  }

  if (br < 0)
  {
    br = 0;
  }

  if (hr < 0)
  {
    hr = 0;
  }

  if (br > 999)
  {
    br = 999;
  }

  if (hr > 999)
  {
    hr = 999;
  }

  br_u = (uint16_t)br;
  hr_u = (uint16_t)hr;

  (void)snprintf(br_text, sizeof(br_text), "%03u ", br_u);
  (void)snprintf(hr_text, sizeof(hr_text), "%03u ", hr_u);

  OLED_ShowString(OLED_VALUE_DATA_X, 2U, (u8 *)br_text, 8U);
  OLED_ShowString(OLED_VALUE_DATA_X, 6U, (u8 *)hr_text, 8U);
}

static void OLED_SetBreathRate(float rate)
{
  s_latest_breath_rate = rate;
  s_oled_rate_dirty = true;
}

static void OLED_SetHeartRate(float rate)
{
  s_latest_heart_rate = rate;
  s_oled_rate_dirty = true;
}

static void OLED_SetPhases(float breath_phase, float heart_phase)
{
  s_pending_breath_phase = breath_phase;
  s_pending_heart_phase = heart_phase;
  s_oled_wave_pending = true;
}

static void OLED_Service(void)
{
  uint32_t now;
  bool should_axes_only;

  if (!s_oled_ready)
  {
    return;
  }

  now = HAL_GetTick();

  if (s_radar_data_online && ((now - s_last_radar_frame_tick) >= LD6002_NO_DATA_TIMEOUT_MS))
  {
    s_radar_data_online = false;
    s_human_detect_valid = false;
    s_target_range_valid = false;
    s_oled_rate_dirty = true;
  }

  should_axes_only = OLED_ShouldAxesOnly();
  if (should_axes_only && !s_oled_axes_only_mode)
  {
    s_oled_axes_only_mode = true;
    s_oled_wave_pending = false;
    s_oled_wave_prev_valid = false;
    s_oled_wave_x = 0U;
    OLED_ClearWaveAndKeepAxes();
    OLED_UpdateVitalsText();
    s_last_oled_text_tick = now;
    s_oled_rate_dirty = false;
  }
  else if (!should_axes_only && s_oled_axes_only_mode)
  {
    s_oled_axes_only_mode = false;
    s_oled_wave_prev_valid = false;
    OLED_UpdateVitalsText();
    s_last_oled_text_tick = now;
    s_oled_rate_dirty = false;
  }

  if (s_oled_wave_pending && ((now - s_last_oled_wave_tick) >= OLED_WAVE_UPDATE_MS))
  {
    if (!s_oled_axes_only_mode)
    {
      OLED_PlotWaveColumn(s_pending_breath_phase, s_pending_heart_phase);
    }

    s_last_oled_wave_tick = now;
    s_oled_wave_pending = false;
  }

  if (s_oled_rate_dirty && ((now - s_last_oled_text_tick) >= OLED_TEXT_UPDATE_MS))
  {
    OLED_UpdateVitalsText();
    s_last_oled_text_tick = now;
    s_oled_rate_dirty = false;
  }
}

static void OLED_UI_Init(void)
{
  OLED_Init();
  OLED_Clear();

  OLED_DrawWaveAxes();

  OLED_RefreshDirty();

  s_oled_wave_x = 0U;
  s_oled_wave_prev_valid = false;
  s_latest_breath_rate = 0.0f;
  s_latest_heart_rate = 0.0f;
  s_last_radar_frame_tick = HAL_GetTick();
  s_radar_data_online = false;
  s_human_detect_valid = false;
  s_human_present = false;
  s_target_range_valid = false;
  s_target_range_cm = 0.0f;
  s_oled_axes_only_mode = true;
  s_oled_panel_drawn_mode = OLED_PANEL_MODE_COUNT;
  s_oled_rate_dirty = true;
  s_oled_wave_pending = false;
  s_oled_ready = true;
  OLED_UpdateVitalsText();
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

  switch (frame->type)
  {
  case 0x0F09:
    if (frame->len >= 2U)
    {
      uint16_t is_human = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
      bool human_present = (is_human != 0U);

      if ((!s_human_detect_valid) || (s_human_present != human_present))
      {
        s_oled_rate_dirty = true;
      }

      s_human_detect_valid = true;
      s_human_present = human_present;

      UART1_Sendf("[RADAR] TYPE=0x0F09 ID=0x%04X Human=%u\r\n", frame->id, (unsigned)is_human);
    }
    break;

  case 0x0A13:
    if (frame->len >= 12U)
    {
      float breath_phase = LD6002_F32Le(&frame->data[4]);
      float heart_phase = LD6002_F32Le(&frame->data[8]);

      OLED_SetPhases(breath_phase, heart_phase);
    }
    break;

  case 0x0A14:
    if (frame->len >= 4U)
    {
      float breath_rate = LD6002_F32Le(&frame->data[0]);

      OLED_SetBreathRate(breath_rate);

      LD6002_FormatFloat(breath_rate, a, sizeof(a));
      UART1_Sendf("[RADAR] TYPE=0x0A14 ID=0x%04X BreathRate=%s bpm\r\n", frame->id, a);
    }
    break;

  case 0x0A15:
    if (frame->len >= 4U)
    {
      float heart_rate = LD6002_F32Le(&frame->data[0]);

      OLED_SetHeartRate(heart_rate);

      LD6002_FormatFloat(heart_rate, a, sizeof(a));
      UART1_Sendf("[RADAR] TYPE=0x0A15 ID=0x%04X HeartRate=%s bpm\r\n", frame->id, a);
    }
    break;

  case 0x0A16:
    if (frame->len >= 8U)
    {
      uint32_t flag = LD6002_U32Le(&frame->data[0]);
      float range = LD6002_F32Le(&frame->data[4]);

      s_target_range_cm = range;
      s_target_range_valid = true;
      s_oled_rate_dirty = true;

      LD6002_FormatFloat(range, a, sizeof(a));
      UART1_Sendf("[RADAR] TYPE=0x0A16 ID=0x%04X Flag=%lu Range=%s cm\r\n", frame->id, (unsigned long)flag, a);
    }
    break;

  default:
    break;
  }
}

static void LD6002_ProcessFrames(void)
{
  LD6002_Frame frame;

  while (LD6002_QueuePop(&frame))
  {
    s_last_radar_frame_tick = HAL_GetTick();
    if (!s_radar_data_online)
    {
      s_radar_data_online = true;
      s_oled_rate_dirty = true;
    }

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
  OLED_UI_Init();
  LD6002_StartUart2Rx();
  UART1_SendText("\r\n[RADAR] UART2->TF parser started\r\n");
  UART1_SendText("[RADAR] OLED wave+vitals ready\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LD6002_ProcessFrames();
    OLED_Service();
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
    LD6002_ParserReset(&s_ld6002_parser);
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
