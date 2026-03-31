#include "uart1_summary.h"

#include <stdarg.h>
#include <stdio.h>

/* 串口发送超时与发送节流周期。 */
#define LD6002_UART_TIMEOUT_MS 50U
#define UART1_SUMMARY_INTERVAL_MS 200U

/* UART1 模块上下文。 */
static UART_HandleTypeDef *s_uart1;
static uint32_t s_last_uart1_summary_tick;

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

/* 绑定 UART1 句柄并重置节流时间。 */
void UART1_Summary_Init(UART_HandleTypeDef *uart1)
{
  s_uart1 = uart1;
  s_last_uart1_summary_tick = 0U;
}

/* 周期发送业务汇总：心率、呼吸、距离、人数。 */
void UART1_Summary_Service(RadarAppState *state)
{
  uint32_t now;
  int32_t br;
  int32_t hr;
  int32_t pos;
  uint8_t people;

  if ((state == NULL) || (s_uart1 == NULL))
  {
    return;
  }

  /* 没有数据变更时不发送。 */
  if (!state->uart1_summary_dirty)
  {
    return;
  }

  /* 节流窗口内跳过，避免串口占用过高。 */
  now = HAL_GetTick();
  if ((now - s_last_uart1_summary_tick) < UART1_SUMMARY_INTERVAL_MS)
  {
    return;
  }

  br = (int32_t)(state->latest_breath_rate + ((state->latest_breath_rate >= 0.0f) ? 0.5f : -0.5f));
  hr = (int32_t)(state->latest_heart_rate + ((state->latest_heart_rate >= 0.0f) ? 0.5f : -0.5f));

  /* 固定三位显示，限制在 0..999。 */
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

  if (state->target_range_valid)
  {
    /* 距离有效时输出整数厘米值。 */
    pos = (int32_t)(state->target_range_cm + ((state->target_range_cm >= 0.0f) ? 0.5f : -0.5f));
    if (pos < 0)
    {
      pos = 0;
    }
  }
  else
  {
    /* 距离无效时按 0 输出。 */
    pos = 0;
  }

  /* 当前业务按 0/1 人输出。 */
  people = (uint8_t)((state->human_detect_valid && state->human_present) ? 1U : 0U);

  UART1_Sendf("[Rader]:心率：%ld  呼吸：%ld  位置：%ld  人数：%u\r\n",
              (long)hr,
              (long)br,
              (long)pos,
              (unsigned)people);

  s_last_uart1_summary_tick = now;
  state->uart1_summary_dirty = false;
}
