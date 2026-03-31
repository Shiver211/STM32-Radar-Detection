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
/*
 * LD6002 串口帧按字节解析状态机。
 * 解析器每收到 1 字节就推进一次状态，直到得到完整且校验通过的一帧。
 */
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

/* 一帧完整解析后的数据结构。 */
typedef struct
{
  uint16_t id;
  uint16_t len;
  uint16_t type;
  uint8_t data[128];
} LD6002_Frame;

/* UART 中断解析上下文，保存解析过程中的临时状态。 */
typedef struct
{
  LD6002_ParseState state;
  uint8_t header[7];
  uint8_t header_idx;
  uint16_t data_idx;
  LD6002_Frame frame;
} LD6002_Parser;

/* OLED 右侧信息区显示模式。 */
typedef enum
{
  OLED_PANEL_VITALS = 0,
  OLED_PANEL_NO_DATA,
  OLED_PANEL_NO_HUMAN,
  OLED_PANEL_DIST_TOO_FAR,
  OLED_PANEL_DIST_TOO_NEAR,
  OLED_PANEL_MODE_COUNT
} OLED_RightPanelMode;

/* 距离分类结果，用于右侧提示与波形模式切换。 */
typedef enum
{
  LD6002_RANGE_UNKNOWN = 0,
  LD6002_RANGE_NORMAL,
  LD6002_RANGE_TOO_NEAR,
  LD6002_RANGE_TOO_FAR
} LD6002_RangeState;

/* 左侧波形区域渲染模式。 */
typedef enum
{
  OLED_WAVE_MODE_NORMAL = 0,
  OLED_WAVE_MODE_AXES_ONLY
} OLED_WaveMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 协议与串口基础参数。 */
#define LD6002_SOF 0x01U
#define LD6002_MAX_PAYLOAD_LEN 128U
#define LD6002_FRAME_QUEUE_SIZE 16U
#define LD6002_UART_TIMEOUT_MS 50U
#define LD6002_NO_DATA_TIMEOUT_MS 1000U
#define LD6002_HUMAN_ENTER_CONFIRM_FRAMES 2U
#define LD6002_HUMAN_LEAVE_CONFIRM_FRAMES 1U

/*
 * 距离阈值（单位：cm）。
 * MIN/MAX：进入过近/过远告警的硬阈值。
 * *_RELEASE：基于滤波值的告警释放阈值（抑制抖动）。
 * *_FAST_RELEASE：基于原始值的快速释放阈值（改善恢复速度）。
 */
#define LD6002_TARGET_RANGE_MIN_CM 35.0f
#define LD6002_TARGET_RANGE_MAX_CM 100.0f
#define LD6002_TARGET_RANGE_NEAR_RELEASE_CM 36.0f
#define LD6002_TARGET_RANGE_FAR_RELEASE_CM 99.0f
#define LD6002_TARGET_RANGE_NEAR_FAST_RELEASE_CM 36.0f
#define LD6002_TARGET_RANGE_FAR_FAST_RELEASE_CM 99.0f
#define LD6002_RANGE_FILTER_ALPHA 0.75f

/* 左侧波形区几何范围（x=0..95）。 */
#define OLED_WAVE_X_START 0U
#define OLED_WAVE_X_END   95U
#define OLED_WAVE_WIDTH   ((OLED_WAVE_X_END - OLED_WAVE_X_START) + 1U)

/* 右侧信息区几何范围（x=96..127）。 */
#define OLED_VALUE_AREA_X_START 96U
#define OLED_VALUE_AREA_X_END   127U
#define OLED_VALUE_AREA_WIDTH   ((OLED_VALUE_AREA_X_END - OLED_VALUE_AREA_X_START) + 1U)

#define OLED_VALUE_LABEL_X OLED_VALUE_AREA_X_START
#define OLED_VALUE_DATA_X  100U

/* 英文提示居中时的 x 坐标辅助量。 */
#define OLED_TEXT_2CHAR_X (OLED_VALUE_AREA_X_START + ((OLED_VALUE_AREA_WIDTH - 16U) / 2U))
#define OLED_TEXT_3CHAR_X (OLED_VALUE_AREA_X_START + ((OLED_VALUE_AREA_WIDTH - 24U) / 2U))
#define OLED_TEXT_4CHAR_X (OLED_VALUE_AREA_X_START + ((OLED_VALUE_AREA_WIDTH - 32U) / 2U))

/* 8x16 字体对应页坐标。 */
#define OLED_TEXT_LINE1_Y 5U
#define OLED_TEXT_LINE2_Y 6U

/* 中文字模索引（来源于 oledfont.h 的 Hzk 表）。 */
#define OLED_ZH_XIN 0U
#define OLED_ZH_LV  1U
#define OLED_ZH_HU  2U
#define OLED_ZH_XI  3U
#define OLED_ZH_WU  4U
#define OLED_ZH_REN 5U
#define OLED_ZH_CUO 10U
#define OLED_ZH_WU2 11U

/* 呼吸波形区域与幅度映射参数。 */
#define OLED_BREATH_Y_MIN    1U
#define OLED_BREATH_Y_MAX    30U
#define OLED_BREATH_CENTER_Y 15U
#define OLED_BREATH_GAIN     22.0f

/* 心率波形区域与幅度映射参数。 */
#define OLED_HEART_Y_MIN    33U
#define OLED_HEART_Y_MAX    62U
#define OLED_HEART_CENTER_Y 47U
#define OLED_HEART_GAIN     18.0f

/* 上下波形分割虚线的 y 坐标。 */
#define OLED_WAVE_SPLIT_Y 31U

/* OLED 刷新周期（ms）。 */
#define OLED_WAVE_UPDATE_MS 50U
#define OLED_TEXT_UPDATE_MS 200U

/* 呼吸/心率低值告警阈值及标签闪烁周期（ms）。 */
#define OLED_BREATH_LOW_THRESHOLD 5.0f
#define OLED_HEART_LOW_THRESHOLD 40.0f
#define OLED_LOW_LABEL_BLINK_MS 500U

/* UART1 汇总信息发送周期（ms）。 */
#define UART1_SUMMARY_INTERVAL_MS 200U

/* LED 告警参数（默认使用 PC13 板载灯，低电平点亮）。 */
#define ALERT_LED_GPIO_Port GPIOC
#define ALERT_LED_Pin       GPIO_PIN_13
#define ALERT_LED_ACTIVE_LOW 1U
#define ALERT_LED_BLINK_MS  500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* UART2 按字节接收与解析队列状态。 */
static uint8_t s_uart2_rx_byte;
static LD6002_Parser s_ld6002_parser;
static LD6002_Frame s_frame_queue[LD6002_FRAME_QUEUE_SIZE];
static volatile uint8_t s_frame_head;
static volatile uint8_t s_frame_tail;
static volatile uint32_t s_frame_dropped;
static volatile uint32_t s_cksum_error;
static volatile uint32_t s_len_error;
static volatile uint32_t s_sof_skip;
static bool s_uart1_summary_dirty;
static uint32_t s_last_uart1_summary_tick;
static volatile bool s_uart2_rx_rearm_pending;
static volatile uint32_t s_uart2_rx_rearm_fail_count;
static volatile uint32_t s_uart2_rx_rearm_busy_count;
static uint32_t s_uart2_rx_rearm_ok_count;

/* LED 告警闪烁状态。 */
static bool s_alert_led_on;
static bool s_alert_led_blink_active;
static uint32_t s_last_alert_led_toggle_tick;

/* OLED 显示与最新参数缓存。 */
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
static bool s_oled_low_label_visible;
static uint32_t s_last_oled_low_label_blink_tick;
static bool s_oled_label_state_valid;
static bool s_oled_breath_label_drawn;
static bool s_oled_heart_label_drawn;

/* 有人/无人确认状态（含去抖）。 */
static bool s_human_detect_valid;
static bool s_human_present;
static bool s_human_candidate_present;
static uint8_t s_human_candidate_count;

/* 距离有效性、滤波值与分类状态。 */
static bool s_target_range_valid;
static float s_target_range_cm;
static float s_target_range_filtered_cm;
static LD6002_RangeState s_range_state;

/* 当前已经应用到 OLED 的波形模式和右侧模式。 */
static OLED_WaveMode s_oled_wave_mode;
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
/* UART2 接收：普通路径和错误恢复路径统一入口。 */
static void LD6002_StartUart2Rx(void);
static HAL_StatusTypeDef LD6002_RearmUart2Rx(void);
static void LD6002_ServiceUart2RxRecovery(void);
static void AlertLed_Set(bool on);
static bool AlertLed_ShouldBlink(void);
static void AlertLed_Service(void);
static uint32_t LD6002_U32Le(const uint8_t *buf);
static float LD6002_F32Le(const uint8_t *buf);
static void LD6002_UpdateHumanPresence(bool raw_human_present);
static LD6002_RangeState LD6002_ClassifyRangeState(float raw_range_cm,
                                                    float filtered_range_cm,
                                                    LD6002_RangeState prev_state);
static void LD6002_UpdateTargetRange(uint32_t flag, float range_cm);

static void UART1_Sendf(const char *fmt, ...);
static void UART1_SendSummaryIfNeeded(void);
static void LD6002_LogFrame(const LD6002_Frame *frame);
static void LD6002_ProcessFrames(void);
static void LD6002_ReportStats(void);
static uint8_t OLED_MapPhaseToY(float value, uint8_t center_y, float gain, uint8_t y_min, uint8_t y_max);
static void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
static void OLED_DrawWaveAxes(void);
static void OLED_ClearWaveAndKeepAxes(void);
static OLED_RightPanelMode OLED_GetPanelMode(void);
static OLED_WaveMode OLED_GetWaveMode(void);
static void OLED_UpdateVitalsLabels(void);
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
/* 解析器复位：任何错误或一帧完成后都回到等待 SOF 状态。 */
static void LD6002_ParserReset(LD6002_Parser *parser)
{
  /* 状态机回到初始态，等待下一帧起始字节。 */
  parser->state = LD6002_WAIT_SOF;

  /* 清空头部/数据索引，避免残留索引影响后续解析。 */
  parser->header_idx = 0U;
  parser->data_idx = 0U;

  /* 清零当前帧元信息，保证新帧从干净上下文开始。 */
  parser->frame.id = 0U;
  parser->frame.len = 0U;
  parser->frame.type = 0U;
}

/* LD6002 校验规则：按字节异或后取反。 */
static uint8_t LD6002_Checksum(const uint8_t *data, uint16_t len)
{
  uint8_t cksum = 0U;
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    /* 协议要求：逐字节异或累加。 */
    cksum ^= data[i];
  }

  /* 最终再按位取反得到校验值。 */
  return (uint8_t)(~cksum);
}

/*
 * 按字节推进解析状态机。
 * 返回 true 表示得到一帧完整且校验通过的 LD6002 数据。
 */
static bool LD6002_ConsumeByte(LD6002_Parser *parser, uint8_t byte, LD6002_Frame *out_frame)
{
  uint8_t expected_cksum;

  switch (parser->state)
  {
  /* 等待帧头 SOF。 */
  case LD6002_WAIT_SOF:
    if (byte == LD6002_SOF)
    {
      /* 捕获到 SOF：重置当前帧上下文并进入头部解析。 */
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
      /* 非 SOF 噪声字节，仅计数并继续等待。 */
      s_sof_skip++;
    }
    break;

  /* ID 高/低字节。 */
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

  /* LEN 高/低字节，并做最大长度保护。 */
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
      /* 长度越界：直接丢弃该帧并复位状态机。 */
      s_len_error++;
      LD6002_ParserReset(parser);
    }
    else
    {
      parser->state = LD6002_TYPE_HIGH;
    }
    break;

  /* TYPE 高/低字节。 */
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

  /* 头部校验通过后，进入数据段或数据校验。 */
  case LD6002_HEAD_CKSUM:
    expected_cksum = LD6002_Checksum(parser->header, (uint16_t)sizeof(parser->header));
    if (byte != expected_cksum)
    {
      /* 头部校验失败：整帧无效，回到 SOF 等待态。 */
      s_cksum_error++;
      LD6002_ParserReset(parser);
      break;
    }

    if (parser->frame.len == 0U)
    {
      /* 零长度帧：不进入 DATA，直接等待数据校验字节。 */
      parser->state = LD6002_DATA_CKSUM;
    }
    else
    {
      /* 非零长度：开始接收数据区。 */
      parser->data_idx = 0U;
      parser->state = LD6002_DATA;
    }
    break;

  /* 按 LEN 收集数据区。 */
  case LD6002_DATA:
    /* 逐字节写入载荷缓冲区。 */
    parser->frame.data[parser->data_idx++] = byte;
    if (parser->data_idx >= parser->frame.len)
    {
      /* 数据段收满后，进入数据校验阶段。 */
      parser->state = LD6002_DATA_CKSUM;
    }
    break;

  /* 数据校验通过后输出完整帧。 */
  case LD6002_DATA_CKSUM:
    expected_cksum = LD6002_Checksum(parser->frame.data, parser->frame.len);
    if (byte == expected_cksum)
    {
      /* 数据校验通过：输出完整帧，并复位解析器准备下一帧。 */
      *out_frame = parser->frame;
      LD6002_ParserReset(parser);
      return true;
    }

    /* 数据校验失败：丢弃该帧。 */
    s_cksum_error++;
    LD6002_ParserReset(parser);
    break;

  default:
    LD6002_ParserReset(parser);
    break;
  }

  return false;
}

/* ISR 中入队：满队列时丢帧并计数。 */
static void LD6002_QueuePushFromISR(const LD6002_Frame *frame)
{
  /* 计算入队后的头指针（环形队列）。 */
  uint8_t next_head = (uint8_t)((s_frame_head + 1U) % LD6002_FRAME_QUEUE_SIZE);

  if (next_head == s_frame_tail)
  {
    /* 队列已满：放弃当前帧并累计丢帧计数。 */
    s_frame_dropped++;
    return;
  }

  /* 写入当前 head 位置，并推进 head。 */
  s_frame_queue[s_frame_head] = *frame;
  s_frame_head = next_head;
}

/* 主循环中出队：空队列返回 false。 */
static bool LD6002_QueuePop(LD6002_Frame *frame)
{
  if (s_frame_tail == s_frame_head)
  {
    /* 队列为空，无数据可取。 */
    return false;
  }

  /* 取出 tail 对应帧，并推进 tail。 */
  *frame = s_frame_queue[s_frame_tail];
  s_frame_tail = (uint8_t)((s_frame_tail + 1U) % LD6002_FRAME_QUEUE_SIZE);
  return true;
}

/* 启动 UART2 一字节中断接收。 */
static void LD6002_StartUart2Rx(void)
{
  /* 首次挂接 UART2 的 1 字节中断接收。 */
  (void)LD6002_RearmUart2Rx();
}

/*
 * 尝试重新挂接 UART2 中断接收。
 * 失败时打标记，由主循环持续重试，避免链路静默中断。
 */
static HAL_StatusTypeDef LD6002_RearmUart2Rx(void)
{
  /* 使用 HAL 中断模式持续做“单字节接收-回调-再挂接”循环。 */
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &s_uart2_rx_byte, 1U);

  if (status == HAL_OK)
  {
    /* 挂接成功：清除待恢复标记。 */
    s_uart2_rx_rearm_pending = false;
    return status;
  }

  /* 挂接失败：记录失败并交给主循环恢复逻辑重试。 */
  s_uart2_rx_rearm_pending = true;
  s_uart2_rx_rearm_fail_count++;
  if (status == HAL_BUSY)
  {
    /* HAL 忙状态单独计数，便于后续定位串口占用问题。 */
    s_uart2_rx_rearm_busy_count++;
  }

  return status;
}

/* 主循环中的 UART2 恢复服务：重试并做限频告警日志。 */
static void LD6002_ServiceUart2RxRecovery(void)
{
  if (!s_uart2_rx_rearm_pending)
  {
    /* 当前无恢复需求，直接返回。 */
    return;
  }

  /* 持续重试，直到成功恢复中断接收链路。 */
  if (LD6002_RearmUart2Rx() == HAL_OK)
  {
    s_uart2_rx_rearm_ok_count++;
  }
}

/* 控制告警灯亮灭。 */
static void AlertLed_Set(bool on)
{
  GPIO_PinState state;

  /* 将“逻辑亮灭”映射到实际电平（兼容低电平点亮/高电平点亮）。 */
  if (ALERT_LED_ACTIVE_LOW != 0U)
  {
    state = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  else
  {
    state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }

  /* 下发到 GPIO 并同步软件侧灯状态。 */
  HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, state);
  s_alert_led_on = on;
}

/* 判定是否需要进入 LED 告警闪烁。 */
static bool AlertLed_ShouldBlink(void)
{
  /* 告警条件 1：雷达离线（超时无帧）。 */
  bool no_data_alarm = !s_radar_data_online;

  /* 告警条件 2：人体检测有效且当前无人。 */
  bool no_human_alarm = s_human_detect_valid && !s_human_present;

  /* 告警条件 3：距离有效但处于过近/过远区间。 */
  bool range_alarm = s_target_range_valid &&
                     ((s_range_state == LD6002_RANGE_TOO_NEAR) || (s_range_state == LD6002_RANGE_TOO_FAR));

  return (no_data_alarm || no_human_alarm || range_alarm);
}

/* LED 告警任务：无数据、无人或距离异常时闪烁，否则熄灭。 */
static void AlertLed_Service(void)
{
  uint32_t now = HAL_GetTick();
  bool should_blink = AlertLed_ShouldBlink();

  if (!should_blink)
  {
    /* 没有告警时强制退出闪烁态并关灯。 */
    s_alert_led_blink_active = false;
    if (s_alert_led_on)
    {
      AlertLed_Set(false);
    }
    return;
  }

  if (!s_alert_led_blink_active)
  {
    /* 首次进入告警：立即点亮并记录起始 tick。 */
    s_alert_led_blink_active = true;
    s_last_alert_led_toggle_tick = now;
    AlertLed_Set(true);
    return;
  }

  if ((now - s_last_alert_led_toggle_tick) >= ALERT_LED_BLINK_MS)
  {
    /* 达到闪烁周期则翻转灯状态。 */
    s_last_alert_led_toggle_tick = now;
    AlertLed_Set(!s_alert_led_on);
  }
}

/* 读取小端 uint32。 */
static uint32_t LD6002_U32Le(const uint8_t *buf)
{
  /* 小端拼装：buf[0] 为最低字节，buf[3] 为最高字节。 */
  return ((uint32_t)buf[0]) |
         ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

/* 读取小端 float。 */
static float LD6002_F32Le(const uint8_t *buf)
{
  /* 先按小端读出 32 位原始值，再按 IEEE754 位模式解释为 float。 */
  uint32_t raw = LD6002_U32Le(buf);
  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

/*
 * 有人状态确认（去抖）：
 * 进入有人和离开有人使用不同确认帧数。
 */
static void LD6002_UpdateHumanPresence(bool raw_human_present)
{
  /* 进入/离开采用不同确认帧数，实现非对称去抖。 */
  uint8_t required_frames = raw_human_present ? LD6002_HUMAN_ENTER_CONFIRM_FRAMES : LD6002_HUMAN_LEAVE_CONFIRM_FRAMES;

  if ((!s_human_detect_valid) || (s_human_candidate_present != raw_human_present))
  {
    /* 候选状态变化：重置计数并开始累计。 */
    s_human_candidate_present = raw_human_present;
    s_human_candidate_count = 1U;
  }
  else if (s_human_candidate_count < 0xFFU)
  {
    /* 候选状态稳定：继续累计确认帧数。 */
    s_human_candidate_count++;
  }

  if (s_human_candidate_count >= required_frames)
  {
    if ((!s_human_detect_valid) || (s_human_present != raw_human_present))
    {
      /* 仅在最终状态发生改变时触发 UI/串口刷新。 */
      s_oled_rate_dirty = true;
      s_uart1_summary_dirty = true;
    }

    /* 提交去抖结果为当前“有人/无人”状态。 */
    s_human_detect_valid = true;
    s_human_present = raw_human_present;
  }
}

/*
 * 距离状态分类：
 * 1) 原始值用于快速进入和快速释放；
 * 2) 滤波值用于边界稳定，减少抖动闪烁。
 */
static LD6002_RangeState LD6002_ClassifyRangeState(float raw_range_cm,
                                                    float filtered_range_cm,
                                                    LD6002_RangeState prev_state)
{
  /* 原始值先做硬阈值判断，确保进入告警响应迅速。 */
  if (raw_range_cm < LD6002_TARGET_RANGE_MIN_CM)
  {
    return LD6002_RANGE_TOO_NEAR;
  }

  if (raw_range_cm > LD6002_TARGET_RANGE_MAX_CM)
  {
    return LD6002_RANGE_TOO_FAR;
  }

  switch (prev_state)
  {
  case LD6002_RANGE_TOO_NEAR:
    /* 过近态优先用原始值做快速释放，其次用滤波值抑制边界抖动。 */
    if (raw_range_cm >= LD6002_TARGET_RANGE_NEAR_FAST_RELEASE_CM)
    {
      return LD6002_RANGE_NORMAL;
    }

    return (filtered_range_cm < LD6002_TARGET_RANGE_NEAR_RELEASE_CM) ? LD6002_RANGE_TOO_NEAR : LD6002_RANGE_NORMAL;

  case LD6002_RANGE_TOO_FAR:
    /* 过远态同理：先快速释放，再使用滤波阈值稳定判定。 */
    if (raw_range_cm <= LD6002_TARGET_RANGE_FAR_FAST_RELEASE_CM)
    {
      return LD6002_RANGE_NORMAL;
    }

    return (filtered_range_cm > LD6002_TARGET_RANGE_FAR_RELEASE_CM) ? LD6002_RANGE_TOO_FAR : LD6002_RANGE_NORMAL;

  default:
    /* 正常/未知态：使用滤波值做稳态判定，避免来回跳变。 */
    if (filtered_range_cm < LD6002_TARGET_RANGE_MIN_CM)
    {
      return LD6002_RANGE_TOO_NEAR;
    }

    if (filtered_range_cm > LD6002_TARGET_RANGE_MAX_CM)
    {
      return LD6002_RANGE_TOO_FAR;
    }

    return LD6002_RANGE_NORMAL;
  }
}

/*
 * 处理 0x0A16 距离帧：
 * - flag=1：有效距离，更新滤波和分类；
 * - flag!=1：仅清理距离有效状态，不影响人体存在状态。
 */
static void LD6002_UpdateTargetRange(uint32_t flag, float range_cm)
{
  LD6002_RangeState prev_state;
  LD6002_RangeState next_state;

  if (flag != 1U)
  {
    /* 传感器上报距离无效：清理距离相关状态并触发界面刷新。 */
    if (s_target_range_valid || (s_range_state != LD6002_RANGE_UNKNOWN))
    {
      s_target_range_valid = false;
      s_target_range_cm = 0.0f;
      s_target_range_filtered_cm = 0.0f;
      s_range_state = LD6002_RANGE_UNKNOWN;
      s_oled_rate_dirty = true;
      s_uart1_summary_dirty = true;
    }

    return;
  }

  if (range_cm < 0.0f)
  {
    /* 负距离按 0 处理。 */
    range_cm = 0.0f;
  }

  /* 保留旧状态用于比较是否发生模式变化。 */
  prev_state = s_range_state;

  if (!s_target_range_valid)
  {
    /* 首次有效数据：滤波值直接对齐原始值。 */
    s_target_range_filtered_cm = range_cm;
  }
  else
  {
    /* 后续数据采用一阶低通滤波，平衡响应速度与稳定性。 */
    s_target_range_filtered_cm = (LD6002_RANGE_FILTER_ALPHA * range_cm) +
                                 ((1.0f - LD6002_RANGE_FILTER_ALPHA) * s_target_range_filtered_cm);
  }

  /* 使用“原始值+滤波值+前态”做带滞回的距离分类。 */
  next_state = LD6002_ClassifyRangeState(range_cm, s_target_range_filtered_cm, prev_state);

  if ((!s_target_range_valid) || (next_state != prev_state))
  {
    /* 新建有效状态或状态切换时，触发 OLED/串口摘要刷新。 */
    s_oled_rate_dirty = true;
    s_uart1_summary_dirty = true;
  }

  /* 提交最新距离值与分类结果。 */
  s_target_range_cm = range_cm;
  s_target_range_valid = true;
  s_range_state = next_state;
  /* 距离数值变化本身也需要更新串口摘要。 */
  s_uart1_summary_dirty = true;
}

/* 将相位值映射到屏幕 y 坐标，并限制在有效显示区间。 */
static uint8_t OLED_MapPhaseToY(float value, uint8_t center_y, float gain, uint8_t y_min, uint8_t y_max)
{
  /* 相位值经增益放大后围绕中心线映射到像素坐标。 */
  int32_t y = (int32_t)center_y - (int32_t)(value * gain);

  if (y < (int32_t)y_min)
  {
    /* 上边界保护。 */
    y = (int32_t)y_min;
  }
  else if (y > (int32_t)y_max)
  {
    /* 下边界保护。 */
    y = (int32_t)y_max;
  }

  return (uint8_t)y;
}

/* Bresenham 直线算法：连接相邻采样点，避免波形断裂。 */
static void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  /* 计算增量与步进方向。 */
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
    /* 先绘制当前采样点。 */
    OLED_DrawPoint((u8)x0, (u8)y0, 1U);

    if ((x0 == x1) && (y0 == y1))
    {
      /* 到达终点后结束。 */
      break;
    }

    e2 = (int16_t)(2 * err);

    if (e2 > -abs_dy)
    {
      /* 误差过大时沿 x 方向前进一步。 */
      err = (int16_t)(err - abs_dy);
      x0 = (uint8_t)((int16_t)x0 + sx);
    }

    if (e2 < abs_dx)
    {
      /* 同理根据误差决定是否沿 y 方向前进。 */
      err = (int16_t)(err + abs_dx);
      y0 = (uint8_t)((int16_t)y0 + sy);
    }
  }
}

/* 绘制左侧波形区域的静态坐标轴。 */
static void OLED_DrawWaveAxes(void)
{
  uint8_t x;

  for (x = OLED_WAVE_X_START; x <= OLED_WAVE_X_END; x++)
  {
    /* 中间虚线：隔列绘制。 */
    if ((x & 0x01U) == 0U)
    {
      OLED_DrawPoint(x, OLED_WAVE_SPLIT_Y, 1U);
    }

    /* 呼吸/心率中心线：每列都绘制。 */
    OLED_DrawPoint(x, OLED_BREATH_CENTER_Y, 1U);
    OLED_DrawPoint(x, OLED_HEART_CENTER_Y, 1U);
  }
}

/* 清空波形区并立即重绘坐标轴。 用于异常处理。*/
static void OLED_ClearWaveAndKeepAxes(void)
{
  uint8_t x;
  uint8_t y;

  /* 清除左侧波形显示区（不影响右侧数值区）。 */
  for (x = OLED_WAVE_X_START; x <= OLED_WAVE_X_END; x++)
  {
    for (y = OLED_BREATH_Y_MIN; y <= OLED_HEART_Y_MAX; y++)
    {
      OLED_DrawPoint(x, y, 0U);
    }
  }

  /* 清空后重绘坐标轴，确保界面结构稳定。 */
  OLED_DrawWaveAxes();
  OLED_RefreshDirty();
}

/* 根据在线状态/有人状态/距离状态决定右侧提示模式。 */
static OLED_RightPanelMode OLED_GetPanelMode(void)
{
  /* 优先级 1：离线。 */
  if (!s_radar_data_online)
  {
    return OLED_PANEL_NO_DATA;
  }

  /* 优先级 2：已确认无人。 */
  if (s_human_detect_valid && !s_human_present)
  {
    return OLED_PANEL_NO_HUMAN;
  }

  /* 优先级 3：距离告警。 */
  if (s_target_range_valid)
  {
    if (s_range_state == LD6002_RANGE_TOO_FAR)
    {
      return OLED_PANEL_DIST_TOO_FAR;
    }

    if (s_range_state == LD6002_RANGE_TOO_NEAR)
    {
      return OLED_PANEL_DIST_TOO_NEAR;
    }
  }

  return OLED_PANEL_VITALS;
}

/* 根据右侧模式决定左侧波形是否绘制实时曲线。 */
static OLED_WaveMode OLED_GetWaveMode(void)
{
  /* 左侧波形是否绘制取决于右侧业务模式。 */
  OLED_RightPanelMode panel_mode = OLED_GetPanelMode();

  if ((panel_mode == OLED_PANEL_NO_DATA) ||
      (panel_mode == OLED_PANEL_NO_HUMAN) ||
      (panel_mode == OLED_PANEL_DIST_TOO_FAR) ||
      (panel_mode == OLED_PANEL_DIST_TOO_NEAR))
  {
    /* 异常提示模式下，仅保留坐标轴。 */
    return OLED_WAVE_MODE_AXES_ONLY;
  }

  return OLED_WAVE_MODE_NORMAL;
}

/*
 * 刷新右侧“呼吸/心率”标签。
 * 当对应数值过低时，标签按 s_oled_low_label_visible 状态闪烁。
 */
static void OLED_UpdateVitalsLabels(void)
{
  /* 判定呼吸/心率是否低于告警阈值。 */
  bool breath_low = (s_latest_breath_rate >= 0.0f) && (s_latest_breath_rate < OLED_BREATH_LOW_THRESHOLD);
  bool heart_low = (s_latest_heart_rate >= 0.0f) && (s_latest_heart_rate < OLED_HEART_LOW_THRESHOLD);

  /* 低值时使用闪烁门控控制标签显隐。 */
  bool show_breath_label = (!breath_low) || s_oled_low_label_visible;
  bool show_heart_label = (!heart_low) || s_oled_low_label_visible;

  /* 标签显示状态未变化时不重绘，避免未触发告警时出现可见闪烁。 */
  if (s_oled_label_state_valid &&
      (show_breath_label == s_oled_breath_label_drawn) &&
      (show_heart_label == s_oled_heart_label_drawn))
  {
    return;
  }

  if ((!s_oled_label_state_valid) || (show_breath_label != s_oled_breath_label_drawn))
  {
    /* 先清标签区，再按目标状态重绘。 */
    OLED_Fill(OLED_VALUE_LABEL_X, 0U, (u8)(OLED_VALUE_LABEL_X + 31U), 15U, 0U);

    if (show_breath_label)
    {
      OLED_ShowCHinese(OLED_VALUE_LABEL_X, 0U, OLED_ZH_HU); /* 呼 */
      OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 0U, OLED_ZH_XI); /* 吸 */
    }
  }

  if ((!s_oled_label_state_valid) || (show_heart_label != s_oled_heart_label_drawn))
  {
    /* 心率标签同样按“先擦后画”策略更新。 */
    OLED_Fill(OLED_VALUE_LABEL_X, 32U, (u8)(OLED_VALUE_LABEL_X + 31U), 47U, 0U);

    if (show_heart_label)
    {
      OLED_ShowCHinese(OLED_VALUE_LABEL_X, 4U, OLED_ZH_XIN); /* 心 */
      OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 4U, OLED_ZH_LV); /* 率 */
    }
  }

  /* 记录本次已绘制状态，供下次差量判断。 */
  s_oled_breath_label_drawn = show_breath_label;
  s_oled_heart_label_drawn = show_heart_label;
  s_oled_label_state_valid = true;
}

/* 右侧信息区整屏重绘：先清空，再按模式绘制内容。 */
static void OLED_DrawPanelMode(OLED_RightPanelMode mode)
{
  /* 右侧信息区先整块清屏，避免模式切换残影。 */
  OLED_Fill(OLED_VALUE_AREA_X_START, 0U, OLED_VALUE_AREA_X_END, 63U, 0U);
  s_oled_label_state_valid = false;

  /* 按模式绘制固定文案，数值模式由后续文本刷新补充。 */
  switch (mode)
  {
  case OLED_PANEL_VITALS:
    OLED_UpdateVitalsLabels();
    break;

  case OLED_PANEL_NO_DATA:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO); /* 错 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2); /* 误 */
    OLED_ShowString(OLED_TEXT_2CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"No", 8U);
    OLED_ShowString(OLED_TEXT_4CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Data", 8U);
    break;

  case OLED_PANEL_NO_HUMAN:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO); /* 错 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2); /* 误 */
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 5U, OLED_ZH_WU); /* 无 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 5U, OLED_ZH_REN); /* 人 */
    break;

  case OLED_PANEL_DIST_TOO_FAR:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO); /* 错 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2); /* 误 */
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"Too", 8U);
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Far", 8U);
    break;

  case OLED_PANEL_DIST_TOO_NEAR:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO); /* 错 */
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2); /* 误 */
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"Too", 8U);
    OLED_ShowString(OLED_TEXT_4CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Near", 8U);
    break;

  default:
    break;
  }
}

/* 绘制一列波形（x 方向滚动），包含呼吸与心率两条曲线。 */
static void OLED_PlotWaveColumn(float breath_phase, float heart_phase)
{
  /* 当前要更新的列坐标（从左到右滚动）。 */
  uint8_t x = (uint8_t)(OLED_WAVE_X_START + s_oled_wave_x);
  uint8_t y;
  uint8_t breath_y;
  uint8_t heart_y;

  /* 先擦除该列旧的呼吸波形像素。 */
  for (y = OLED_BREATH_Y_MIN; y <= OLED_BREATH_Y_MAX; y++)
  {
    OLED_DrawPoint(x, y, 0U);
  }

  /* 再擦除该列旧的心率波形像素。 */
  for (y = OLED_HEART_Y_MIN; y <= OLED_HEART_Y_MAX; y++)
  {
    OLED_DrawPoint(x, y, 0U);
  }

  /* 重新打上两条中心参考线。 */
  OLED_DrawPoint(x, OLED_BREATH_CENTER_Y, 1U);
  OLED_DrawPoint(x, OLED_HEART_CENTER_Y, 1U);

  /* 将相位值映射为当前列的目标 y 坐标。 */
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
    /* 非首点时与上一列连线，提升波形连续性。 */
    OLED_DrawLine((uint8_t)(x - 1U), s_prev_breath_y, x, breath_y);
    OLED_DrawLine((uint8_t)(x - 1U), s_prev_heart_y, x, heart_y);
  }
  else
  {
    /* 首点或换行起点时仅打点。 */
    OLED_DrawPoint(x, breath_y, 1U);
    OLED_DrawPoint(x, heart_y, 1U);
  }

  /* 保存本列点位，供下一列连线使用。 */
  s_prev_breath_y = breath_y;
  s_prev_heart_y = heart_y;
  s_oled_wave_prev_valid = true;
  s_oled_wave_x++;

  if (s_oled_wave_x >= OLED_WAVE_WIDTH)
  {
    /* 到达右边界后回卷到左侧并重置连线状态。 */
    s_oled_wave_x = 0U;
    s_oled_wave_prev_valid = false;
  }

  /* 将脏区域一次性刷到屏幕。 */
  OLED_RefreshDirty();
}

/* 更新右侧文字：模式提示或呼吸/心率数值。 */
static void OLED_UpdateVitalsText(void)
{
  OLED_RightPanelMode panel_mode;
  /* 四舍五入到整数，便于固定宽度显示。 */
  int32_t br = (int32_t)(s_latest_breath_rate + ((s_latest_breath_rate >= 0.0f) ? 0.5f : -0.5f));
  int32_t hr = (int32_t)(s_latest_heart_rate + ((s_latest_heart_rate >= 0.0f) ? 0.5f : -0.5f));
  uint16_t br_u;
  uint16_t hr_u;
  char br_text[8];
  char hr_text[8];

  panel_mode = OLED_GetPanelMode();
  if (panel_mode != s_oled_panel_drawn_mode)
  {
    /* 右侧模式变化时先重绘静态内容。 */
    OLED_DrawPanelMode(panel_mode);
    s_oled_panel_drawn_mode = panel_mode;
  }

  if (panel_mode != OLED_PANEL_VITALS)
  {
    /* 非数值模式仅显示提示文案，无需更新数值。 */
    return;
  }

  /* 数值模式下，先根据阈值更新“呼吸/心率”标签显隐。 */
  OLED_UpdateVitalsLabels();

  /* 范围保护：OLED 固定三位显示，限制到 0..999。 */
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

  /* 使用固定三位+空格覆盖旧字符，避免残留。 */
  (void)snprintf(br_text, sizeof(br_text), "%03u ", br_u);
  (void)snprintf(hr_text, sizeof(hr_text), "%03u ", hr_u);

  /* 将两行数字写入右侧数值区。 */
  OLED_ShowString(OLED_VALUE_DATA_X, 2U, (u8 *)br_text, 8U);
  OLED_ShowString(OLED_VALUE_DATA_X, 6U, (u8 *)hr_text, 8U);
}

/* 缓存最新呼吸率并请求文本刷新。 */
static void OLED_SetBreathRate(float rate)
{
  /* 缓存最新呼吸率。 */
  s_latest_breath_rate = rate;

  /* 标记 OLED 文本和 UART 摘要需要刷新。 */
  s_oled_rate_dirty = true;
  s_uart1_summary_dirty = true;
}

/* 缓存最新心率并请求文本刷新。 */
static void OLED_SetHeartRate(float rate)
{
  /* 缓存最新心率。 */
  s_latest_heart_rate = rate;

  /* 标记 OLED 文本和 UART 摘要需要刷新。 */
  s_oled_rate_dirty = true;
  s_uart1_summary_dirty = true;
}

/* 缓存最新相位数据，由 OLED_Service 统一调度绘图。 */
static void OLED_SetPhases(float breath_phase, float heart_phase)
{
  /* 缓存最新相位，交给 OLED 周期任务按节奏绘制。 */
  s_pending_breath_phase = breath_phase;
  s_pending_heart_phase = heart_phase;

  /* 置位“待绘制”标志，避免在中断上下文直接刷屏。 */
  s_oled_wave_pending = true;
}

/*
 * OLED 任务调度：
 * 1) 在线超时检测
 * 2) 模式切换处理
 * 3) 波形刷新
 * 4) 文本刷新
 */
static void OLED_Service(void)
{
  uint32_t now;
  OLED_WaveMode target_wave_mode;
  OLED_RightPanelMode panel_mode;
  bool breath_low;
  bool heart_low;
  bool any_low;

  if (!s_oled_ready)
  {
    /* OLED 尚未初始化完成时，不执行任何显示更新。 */
    return;
  }

  now = HAL_GetTick();

  if (s_radar_data_online && ((now - s_last_radar_frame_tick) >= LD6002_NO_DATA_TIMEOUT_MS))
  {
    /* 长时间未收到雷达帧：回退为离线状态。 */
    s_radar_data_online = false;
    s_human_detect_valid = false;
    s_human_candidate_count = 0U;
    s_target_range_valid = false;
    s_range_state = LD6002_RANGE_UNKNOWN;
    s_oled_rate_dirty = true;
    s_uart1_summary_dirty = true;
  }

  /* 根据状态决定波形模式（实时曲线 or 仅坐标轴）。 */
  target_wave_mode = OLED_GetWaveMode();
  if (target_wave_mode != s_oled_wave_mode)
  {
    /* 模式变化时，重置滚动游标并清空旧波形残留。 */
    s_oled_wave_mode = target_wave_mode;
    s_oled_wave_pending = false;
    s_oled_wave_prev_valid = false;
    s_oled_wave_x = 0U;
    OLED_ClearWaveAndKeepAxes();

    OLED_UpdateVitalsText();
    s_last_oled_text_tick = now;
    s_oled_rate_dirty = false;
  }

  /* 仅在数值模式下启用低值标签闪烁逻辑。 */
  panel_mode = OLED_GetPanelMode();
  if (panel_mode == OLED_PANEL_VITALS)
  {
    breath_low = (s_latest_breath_rate >= 0.0f) && (s_latest_breath_rate < OLED_BREATH_LOW_THRESHOLD);
    heart_low = (s_latest_heart_rate >= 0.0f) && (s_latest_heart_rate < OLED_HEART_LOW_THRESHOLD);
    any_low = breath_low || heart_low;

    if (any_low)
    {
      if ((now - s_last_oled_low_label_blink_tick) >= OLED_LOW_LABEL_BLINK_MS)
      {
        /* 到达闪烁周期：翻转可见性并立即刷新一次标签。 */
        s_last_oled_low_label_blink_tick = now;
        s_oled_low_label_visible = !s_oled_low_label_visible;
        OLED_UpdateVitalsText();
        s_last_oled_text_tick = now;
        s_oled_rate_dirty = false;
      }
    }
    else
    {
      /* 告警解除后确保标签恢复常显。 */
      if (!s_oled_low_label_visible)
      {
        s_oled_low_label_visible = true;
        OLED_UpdateVitalsText();
        s_last_oled_text_tick = now;
        s_oled_rate_dirty = false;
      }

      s_last_oled_low_label_blink_tick = now;
    }
  }
  else
  {
    /* 非数值模式下保持标签可见，避免切回数值时出现延迟状态。 */
    s_oled_low_label_visible = true;
    s_last_oled_low_label_blink_tick = now;
  }

  if (s_oled_wave_pending && ((now - s_last_oled_wave_tick) >= OLED_WAVE_UPDATE_MS))
  {
    if (s_oled_wave_mode == OLED_WAVE_MODE_NORMAL)
    {
      /* 正常模式下按节拍推进一列波形。 */
      OLED_PlotWaveColumn(s_pending_breath_phase, s_pending_heart_phase);
    }

    /* 无论是否绘制，都清除 pending 并更新时间基准。 */
    s_last_oled_wave_tick = now;
    s_oled_wave_pending = false;
  }

  if (s_oled_rate_dirty && ((now - s_last_oled_text_tick) >= OLED_TEXT_UPDATE_MS))
  {
    /* 文本刷新频率低于波形刷新，减少 I2C 带宽占用。 */
    OLED_UpdateVitalsText();
    s_last_oled_text_tick = now;
    s_oled_rate_dirty = false;
  }
}

/* OLED 初始化与运行时状态清零。 */
static void OLED_UI_Init(void)
{
  /* 初始化 OLED 驱动并清屏。 */
  OLED_Init();
  OLED_Clear();

  /* 绘制左侧静态坐标轴，并立即刷新到屏幕。 */
  OLED_DrawWaveAxes();

  OLED_RefreshDirty();

  /* 重置波形滚动状态。 */
  s_oled_wave_x = 0U;
  s_oled_wave_prev_valid = false;

  /* 重置显示值缓存。 */
  s_latest_breath_rate = 0.0f;
  s_latest_heart_rate = 0.0f;

  /* 初始化在线与低值闪烁相关状态。 */
  s_last_radar_frame_tick = HAL_GetTick();
  s_radar_data_online = false;
  s_oled_low_label_visible = true;
  s_last_oled_low_label_blink_tick = HAL_GetTick();

  /* 重置标签绘制状态缓存。 */
  s_oled_label_state_valid = false;
  s_oled_breath_label_drawn = false;
  s_oled_heart_label_drawn = false;

  /* 重置人体/距离业务状态。 */
  s_human_detect_valid = false;
  s_human_present = false;
  s_human_candidate_present = false;
  s_human_candidate_count = 0U;
  s_target_range_valid = false;
  s_target_range_cm = 0.0f;
  s_target_range_filtered_cm = 0.0f;
  s_range_state = LD6002_RANGE_UNKNOWN;

  /* 初始使用坐标轴模式，右侧模式置为无效以触发首次整区重绘。 */
  s_oled_wave_mode = OLED_WAVE_MODE_AXES_ONLY;
  s_oled_panel_drawn_mode = OLED_PANEL_MODE_COUNT;

  /* 启用首次文本刷新。 */
  s_oled_rate_dirty = true;
  s_oled_wave_pending = false;
  s_oled_ready = true;

  /* 输出首帧界面文本。 */
  OLED_UpdateVitalsText();
}

/* 向上位机发送格式化日志。 */
static void UART1_Sendf(const char *fmt, ...)
{
  char tx_buf[192];
  va_list args;
  int written;

  /* 格式化字符串到本地缓冲区。 */
  va_start(args, fmt);
  written = vsnprintf(tx_buf, sizeof(tx_buf), fmt, args);
  va_end(args);

  if (written > 0)
  {
    /* 限制发送长度，确保不越过 tx_buf 边界。 */
    uint16_t tx_len = (written >= (int)sizeof(tx_buf)) ? ((uint16_t)sizeof(tx_buf) - 1U) : (uint16_t)written;

    /* 通过 UART1 阻塞发送给上位机。 */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, tx_len, LD6002_UART_TIMEOUT_MS);
  }
}

/*
 * UART1 仅发送业务汇总：心率、呼吸、距离、人数。
 * 通过 dirty 标记和时间节流，避免串口输出过于频繁。
 */
static void UART1_SendSummaryIfNeeded(void)
{
  uint32_t now;
  int32_t br;
  int32_t hr;
  int32_t pos;
  uint8_t people;

  if (!s_uart1_summary_dirty)
  {
    /* 数据无变化时不发送。 */
    return;
  }

  now = HAL_GetTick();
  if ((now - s_last_uart1_summary_tick) < UART1_SUMMARY_INTERVAL_MS)
  {
    /* 未到发送节流窗口，延后到下个周期。 */
    return;
  }

  /* 将浮点指标四舍五入为整数。 */
  br = (int32_t)(s_latest_breath_rate + ((s_latest_breath_rate >= 0.0f) ? 0.5f : -0.5f));
  hr = (int32_t)(s_latest_heart_rate + ((s_latest_heart_rate >= 0.0f) ? 0.5f : -0.5f));

  /* 三位数显示保护。 */
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

  if (s_target_range_valid)
  {
    /* 距离有效时输出实际距离（四舍五入）。 */
    pos = (int32_t)(s_target_range_cm + ((s_target_range_cm >= 0.0f) ? 0.5f : -0.5f));
    if (pos < 0)
    {
      pos = 0;
    }
  }
  else
  {
    /* 距离无效时输出 0。 */
    pos = 0;
  }

  /* 当前逻辑仅统计 0/1 人。 */
  people = (uint8_t)((s_human_detect_valid && s_human_present) ? 1U : 0U);

  /* 发送中文业务摘要行。 */
  UART1_Sendf("[Rader]:心率：%ld  呼吸：%ld  位置：%ld  人数：%u\r\n",
              (long)hr,
              (long)br,
              (long)pos,
              (unsigned)people);

  /* 清除 dirty 并记录本次发送时间。 */
  s_last_uart1_summary_tick = now;
  s_uart1_summary_dirty = false;
}

/* 按帧类型分发业务逻辑：更新波形、数值和状态。 */
static void LD6002_LogFrame(const LD6002_Frame *frame)
{
  /* 根据 type 字段分发不同业务帧。 */
  switch (frame->type)
  {
  case 0x0F09:
    /* 人体存在标志。 */
    if (frame->len >= 2U)
    {
      /* 低字节在前，拼装 16 位人体标志。 */
      uint16_t is_human = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);

      /* 交给去抖逻辑更新最终有人状态。 */
      LD6002_UpdateHumanPresence(is_human != 0U);
    }
    break;

  case 0x0A13:
    /* 呼吸/心率相位。 */
    if (frame->len >= 12U)
    {
      /* 按协议偏移读取呼吸/心率相位。 */
      float breath_phase = LD6002_F32Le(&frame->data[4]);
      float heart_phase = LD6002_F32Le(&frame->data[8]);

      /* 仅缓存，实际绘图由 OLED 周期任务执行。 */
      OLED_SetPhases(breath_phase, heart_phase);
    }
    break;

  case 0x0A14:
    /* 呼吸率。 */
    if (frame->len >= 4U)
    {
      float breath_rate = LD6002_F32Le(&frame->data[0]);

      /* 更新呼吸率并触发相关刷新标记。 */
      OLED_SetBreathRate(breath_rate);
    }
    break;

  case 0x0A15:
    /* 心率。 */
    if (frame->len >= 4U)
    {
      float heart_rate = LD6002_F32Le(&frame->data[0]);

      /* 更新心率并触发相关刷新标记。 */
      OLED_SetHeartRate(heart_rate);
    }
    break;

  case 0x0A16:
    /* 距离状态与目标距离。 */
    if (frame->len >= 8U)
    {
      /* 前 4 字节是有效标志，后 4 字节是距离值。 */
      uint32_t flag = LD6002_U32Le(&frame->data[0]);
      float range = LD6002_F32Le(&frame->data[4]);

      /* 更新距离滤波与分类状态。 */
      LD6002_UpdateTargetRange(flag, range);
    }
    break;

  default:
    break;
  }
}

/* 主循环消费所有队列帧，并维护在线状态。 */
static void LD6002_ProcessFrames(void)
{
  LD6002_Frame frame;

  /* 逐帧消费队列，直到清空。 */
  while (LD6002_QueuePop(&frame))
  {
    /* 每收到一帧就刷新“最后在线时间”。 */
    s_last_radar_frame_tick = HAL_GetTick();
    if (!s_radar_data_online)
    {
      /* 离线恢复为在线后，触发 UI 模式刷新。 */
      s_radar_data_online = true;
      s_oled_rate_dirty = true;
    }

    /* 按类型分发该帧。 */
    LD6002_LogFrame(&frame);
  }
}

/* 每秒输出一次变化后的统计计数。 */
static void LD6002_ReportStats(void)
{
  /* 当前统计输出策略为“按需发送业务摘要”。 */
  UART1_SendSummaryIfNeeded();
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
  /* 初始化 GPIO（含告警灯引脚）、I2C、两个串口。 */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化雷达解析器上下文。 */
  LD6002_ParserReset(&s_ld6002_parser);

  /* 初始化 OLED UI 与运行态缓存。 */
  OLED_UI_Init();

  /* 启动 UART2 单字节中断接收链路。 */
  LD6002_StartUart2Rx();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1) 保障 UART2 中断接收链路持续在线。 */
    /* UART2 失败重挂接恢复服务。 */
    LD6002_ServiceUart2RxRecovery();

    /* 2) 处理雷达帧队列，更新业务状态缓存。 */
    /* 消费并处理雷达帧。 */
    LD6002_ProcessFrames();

    /* 3) 基于业务状态驱动告警灯。 */
    /* 人员/距离异常 LED 告警闪烁。 */
    AlertLed_Service();

    /* 4) 更新 OLED 波形与右侧文案。 */
    /* 刷新 OLED 波形与状态文本。 */
    OLED_Service();

    /* 5) 向 UART1 输出。 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* 默认关闭告警灯。 */
  if (ALERT_LED_ACTIVE_LOW != 0U)
  {
    HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, GPIO_PIN_RESET);
  }

  /* 配置告警灯引脚为推挽输出。 */
  GPIO_InitStruct.Pin = ALERT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALERT_LED_GPIO_Port, &GPIO_InitStruct);

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
  /* UART2 接收完成中断：解析当前字节并立即重挂接下一字节接收。 */
  LD6002_Frame frame;

  if (huart->Instance == USART2)
  {
    if (LD6002_ConsumeByte(&s_ld6002_parser, s_uart2_rx_byte, &frame))
    {
      LD6002_QueuePushFromISR(&frame);
    }

    (void)LD6002_RearmUart2Rx();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* UART2 错误中断：复位解析器并进入统一重挂接流程。 */
  if (huart->Instance == USART2)
  {
    /* 错误发生后丢弃半帧上下文，避免脏数据串帧。 */
    LD6002_ParserReset(&s_ld6002_parser);

    /* 立即尝试恢复单字节接收链路。 */
    (void)LD6002_RearmUart2Rx();
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
