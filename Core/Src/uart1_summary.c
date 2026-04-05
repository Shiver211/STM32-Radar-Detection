#include "uart1_summary.h"

#include <stdarg.h>
#include <stdio.h>

/* 串口发送超时与发送节流周期。 */
#define LD6002_UART_TIMEOUT_MS 50U
#define UART1_STREAM_INTERVAL_MS 50U

/* UART1 模块上下文。 */
static UART_HandleTypeDef *s_uart1;
static uint32_t s_last_uart1_stream_tick;

static void UART1_SendTelemetry(const RadarAppState *state, uint32_t now_tick);
static int32_t UART1_RoundToInt(float value);

/* 发送格式化文本到 UART1。 */
static void UART1_Sendf(const char *fmt, ...)
{
  char tx_buf[192];
  va_list args;
  int written;

  if (s_uart1 == NULL)
  {
    return;
  }

  va_start(args, fmt);
  written = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
  va_end(args);

  if (written > 0)
  {
    uint16_t tx_len = (written >= (int)sizeof(tx_buf)) ? ((uint16_t)sizeof(tx_buf) - 1U) : (uint16_t)written;
    (void)HAL_UART_Transmit(s_uart1, (uint8_t *)tx_buf, tx_len, LD6002_UART_TIMEOUT_MS);
  }
}

/* 将 float 四舍五入为 int32，避免依赖 printf 浮点格式化。 */
static int32_t UART1_RoundToInt(float value)
{
  return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

/* 输出结构化遥测数据，便于上位机稳定解析。 */
static void UART1_SendTelemetry(const RadarAppState *state, uint32_t now_tick)
{
  float heart_rate;
  float breath_rate;
  float heart_phase;
  float breath_phase;
  float distance_cm;
  int32_t heart_rate_out;
  int32_t breath_rate_out;
  int32_t heart_phase_out;
  int32_t breath_phase_out;
  int32_t distance_out;
  uint8_t human_present;
  uint8_t online;
  uint8_t range_valid;

  if (state == NULL)
  {
    return;
  }

  heart_rate = state->latest_heart_rate;
  breath_rate = state->latest_breath_rate;
  heart_phase = state->pending_heart_phase;
  breath_phase = state->pending_breath_phase;
  distance_cm = state->target_range_valid ? state->target_range_cm : 0.0f;

  heart_rate_out = UART1_RoundToInt(heart_rate);
  breath_rate_out = UART1_RoundToInt(breath_rate);
  /* 相位放大 1000 倍后输出为整数，兼顾可读性与波形变化。 */
  heart_phase_out = UART1_RoundToInt(heart_phase * 1000.0f);
  breath_phase_out = UART1_RoundToInt(breath_phase * 1000.0f);
  distance_out = UART1_RoundToInt(distance_cm);

  if (heart_rate_out < 0)
  {
    heart_rate_out = 0;
  }

  if (breath_rate_out < 0)
  {
    breath_rate_out = 0;
  }

  if (distance_out < 0)
  {
    distance_out = 0;
  }

  human_present = (uint8_t)((state->human_detect_valid && state->human_present) ? 1U : 0U);
  online = (uint8_t)(state->radar_data_online ? 1U : 0U);
  range_valid = (uint8_t)(state->target_range_valid ? 1U : 0U);

  UART1_Sendf("RADAR,%lu,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u\r\n",
              (unsigned long)now_tick,
              (long)heart_rate_out,
              (long)breath_rate_out,
              (long)heart_phase_out,
              (long)breath_phase_out,
              (long)distance_out,
              (unsigned)human_present,
              (unsigned)online,
              (unsigned)range_valid,
              1U,
              1U);
}

/* 绑定 UART1 句柄并重置节流时间。 */
void UART1_Summary_Init(UART_HandleTypeDef *uart1)
{
  s_uart1 = uart1;
  s_last_uart1_stream_tick = 0U;
}

/* 周期发送业务数据：固定周期主动上报。 */
void UART1_Summary_Service(RadarAppState *state)
{
  uint32_t now;

  if ((state == NULL) || (s_uart1 == NULL))
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - s_last_uart1_stream_tick) < UART1_STREAM_INTERVAL_MS)
  {
    return;
  }

  UART1_SendTelemetry(state, now);
  s_last_uart1_stream_tick = now;
}
