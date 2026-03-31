#include "radar_service.h"

#include <string.h>

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

#define LD6002_SOF 0x01U
#define LD6002_MAX_PAYLOAD_LEN 128U
#define LD6002_FRAME_QUEUE_SIZE 16U
#define LD6002_NO_DATA_TIMEOUT_MS 1000U
#define LD6002_HUMAN_ENTER_CONFIRM_FRAMES 2U
#define LD6002_HUMAN_LEAVE_CONFIRM_FRAMES 1U

#define LD6002_TARGET_RANGE_MIN_CM 35.0f
#define LD6002_TARGET_RANGE_MAX_CM 100.0f
#define LD6002_TARGET_RANGE_NEAR_RELEASE_CM 36.0f
#define LD6002_TARGET_RANGE_FAR_RELEASE_CM 99.0f
#define LD6002_TARGET_RANGE_NEAR_FAST_RELEASE_CM 36.0f
#define LD6002_TARGET_RANGE_FAR_FAST_RELEASE_CM 99.0f
#define LD6002_RANGE_FILTER_ALPHA 0.75f

static UART_HandleTypeDef *s_uart2;
static RadarAppState *s_state;

static uint8_t s_uart2_rx_byte;
static LD6002_Parser s_ld6002_parser;
static LD6002_Frame s_frame_queue[LD6002_FRAME_QUEUE_SIZE];
static volatile uint8_t s_frame_head;
static volatile uint8_t s_frame_tail;
static volatile uint32_t s_frame_dropped;
static volatile uint32_t s_cksum_error;
static volatile uint32_t s_len_error;
static volatile uint32_t s_sof_skip;
static volatile bool s_uart2_rx_rearm_pending;
static volatile uint32_t s_uart2_rx_rearm_fail_count;
static volatile uint32_t s_uart2_rx_rearm_busy_count;
static uint32_t s_uart2_rx_rearm_ok_count;

static bool s_human_candidate_present;
static uint8_t s_human_candidate_count;

static bool Radar_IsReady(void);
static void LD6002_ParserReset(LD6002_Parser *parser);
static uint8_t LD6002_Checksum(const uint8_t *data, uint16_t len);
static bool LD6002_ConsumeByte(LD6002_Parser *parser, uint8_t byte, LD6002_Frame *out_frame);
static void LD6002_QueuePushFromISR(const LD6002_Frame *frame);
static bool LD6002_QueuePop(LD6002_Frame *frame);
static void LD6002_StartUart2Rx(void);
static HAL_StatusTypeDef LD6002_RearmUart2Rx(void);
static uint32_t LD6002_U32Le(const uint8_t *buf);
static float LD6002_F32Le(const uint8_t *buf);
static void LD6002_UpdateHumanPresence(bool raw_human_present);
static LD6002_RangeState LD6002_ClassifyRangeState(float raw_range_cm,
                                                    float filtered_range_cm,
                                                    LD6002_RangeState prev_state);
static void LD6002_UpdateTargetRange(uint32_t flag, float range_cm);
static void LD6002_LogFrame(const LD6002_Frame *frame);

static bool Radar_IsReady(void)
{
  return (s_uart2 != NULL) && (s_state != NULL);
}

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
  (void)LD6002_RearmUart2Rx();
}

static HAL_StatusTypeDef LD6002_RearmUart2Rx(void)
{
  HAL_StatusTypeDef status;

  if (!Radar_IsReady())
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Receive_IT(s_uart2, &s_uart2_rx_byte, 1U);

  if (status == HAL_OK)
  {
    s_uart2_rx_rearm_pending = false;
    return status;
  }

  s_uart2_rx_rearm_pending = true;
  s_uart2_rx_rearm_fail_count++;
  if (status == HAL_BUSY)
  {
    s_uart2_rx_rearm_busy_count++;
  }

  return status;
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

static void LD6002_UpdateHumanPresence(bool raw_human_present)
{
  uint8_t required_frames = raw_human_present ? LD6002_HUMAN_ENTER_CONFIRM_FRAMES : LD6002_HUMAN_LEAVE_CONFIRM_FRAMES;

  if ((!s_state->human_detect_valid) || (s_human_candidate_present != raw_human_present))
  {
    s_human_candidate_present = raw_human_present;
    s_human_candidate_count = 1U;
  }
  else if (s_human_candidate_count < 0xFFU)
  {
    s_human_candidate_count++;
  }

  if (s_human_candidate_count >= required_frames)
  {
    if ((!s_state->human_detect_valid) || (s_state->human_present != raw_human_present))
    {
      s_state->oled_rate_dirty = true;
      s_state->uart1_summary_dirty = true;
    }

    s_state->human_detect_valid = true;
    s_state->human_present = raw_human_present;
  }
}

static LD6002_RangeState LD6002_ClassifyRangeState(float raw_range_cm,
                                                    float filtered_range_cm,
                                                    LD6002_RangeState prev_state)
{
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
    if (raw_range_cm >= LD6002_TARGET_RANGE_NEAR_FAST_RELEASE_CM)
    {
      return LD6002_RANGE_NORMAL;
    }

    return (filtered_range_cm < LD6002_TARGET_RANGE_NEAR_RELEASE_CM) ? LD6002_RANGE_TOO_NEAR : LD6002_RANGE_NORMAL;

  case LD6002_RANGE_TOO_FAR:
    if (raw_range_cm <= LD6002_TARGET_RANGE_FAR_FAST_RELEASE_CM)
    {
      return LD6002_RANGE_NORMAL;
    }

    return (filtered_range_cm > LD6002_TARGET_RANGE_FAR_RELEASE_CM) ? LD6002_RANGE_TOO_FAR : LD6002_RANGE_NORMAL;

  default:
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

static void LD6002_UpdateTargetRange(uint32_t flag, float range_cm)
{
  LD6002_RangeState prev_state;
  LD6002_RangeState next_state;

  if (flag != 1U)
  {
    if (s_state->target_range_valid || (s_state->range_state != LD6002_RANGE_UNKNOWN))
    {
      s_state->target_range_valid = false;
      s_state->target_range_cm = 0.0f;
      s_state->target_range_filtered_cm = 0.0f;
      s_state->range_state = LD6002_RANGE_UNKNOWN;
      s_state->oled_rate_dirty = true;
      s_state->uart1_summary_dirty = true;
    }

    return;
  }

  if (range_cm < 0.0f)
  {
    range_cm = 0.0f;
  }

  prev_state = s_state->range_state;

  if (!s_state->target_range_valid)
  {
    s_state->target_range_filtered_cm = range_cm;
  }
  else
  {
    s_state->target_range_filtered_cm = (LD6002_RANGE_FILTER_ALPHA * range_cm) +
                                        ((1.0f - LD6002_RANGE_FILTER_ALPHA) * s_state->target_range_filtered_cm);
  }

  next_state = LD6002_ClassifyRangeState(range_cm, s_state->target_range_filtered_cm, prev_state);

  if ((!s_state->target_range_valid) || (next_state != prev_state))
  {
    s_state->oled_rate_dirty = true;
    s_state->uart1_summary_dirty = true;
  }

  s_state->target_range_cm = range_cm;
  s_state->target_range_valid = true;
  s_state->range_state = next_state;
  s_state->uart1_summary_dirty = true;
}

static void LD6002_LogFrame(const LD6002_Frame *frame)
{
  switch (frame->type)
  {
  case 0x0F09:
    if (frame->len >= 2U)
    {
      uint16_t is_human = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
      LD6002_UpdateHumanPresence(is_human != 0U);
    }
    break;

  case 0x0A13:
    if (frame->len >= 12U)
    {
      float breath_phase = LD6002_F32Le(&frame->data[4]);
      float heart_phase = LD6002_F32Le(&frame->data[8]);

      s_state->pending_breath_phase = breath_phase;
      s_state->pending_heart_phase = heart_phase;
      s_state->oled_wave_pending = true;
    }
    break;

  case 0x0A14:
    if (frame->len >= 4U)
    {
      s_state->latest_breath_rate = LD6002_F32Le(&frame->data[0]);
      s_state->oled_rate_dirty = true;
      s_state->uart1_summary_dirty = true;
    }
    break;

  case 0x0A15:
    if (frame->len >= 4U)
    {
      s_state->latest_heart_rate = LD6002_F32Le(&frame->data[0]);
      s_state->oled_rate_dirty = true;
      s_state->uart1_summary_dirty = true;
    }
    break;

  case 0x0A16:
    if (frame->len >= 8U)
    {
      uint32_t flag = LD6002_U32Le(&frame->data[0]);
      float range = LD6002_F32Le(&frame->data[4]);

      LD6002_UpdateTargetRange(flag, range);
    }
    break;

  default:
    break;
  }
}

void Radar_AppStateInit(RadarAppState *state)
{
  if (state == NULL)
  {
    return;
  }

  (void)memset(state, 0, sizeof(*state));
  state->range_state = LD6002_RANGE_UNKNOWN;
}

void Radar_Init(UART_HandleTypeDef *uart2, RadarAppState *state)
{
  s_uart2 = uart2;
  s_state = state;

  if (!Radar_IsReady())
  {
    return;
  }

  LD6002_ParserReset(&s_ld6002_parser);

  s_frame_head = 0U;
  s_frame_tail = 0U;
  s_frame_dropped = 0U;
  s_cksum_error = 0U;
  s_len_error = 0U;
  s_sof_skip = 0U;

  s_uart2_rx_rearm_pending = false;
  s_uart2_rx_rearm_fail_count = 0U;
  s_uart2_rx_rearm_busy_count = 0U;
  s_uart2_rx_rearm_ok_count = 0U;

  s_human_candidate_present = false;
  s_human_candidate_count = 0U;

  s_state->last_radar_frame_tick = HAL_GetTick();

  LD6002_StartUart2Rx();
}

void Radar_ServiceUart2RxRecovery(void)
{
  if (!s_uart2_rx_rearm_pending)
  {
    return;
  }

  if (LD6002_RearmUart2Rx() == HAL_OK)
  {
    s_uart2_rx_rearm_ok_count++;
  }
}

void Radar_ProcessFrames(void)
{
  LD6002_Frame frame;

  if (!Radar_IsReady())
  {
    return;
  }

  while (LD6002_QueuePop(&frame))
  {
    s_state->last_radar_frame_tick = HAL_GetTick();
    if (!s_state->radar_data_online)
    {
      s_state->radar_data_online = true;
      s_state->oled_rate_dirty = true;
      s_state->uart1_summary_dirty = true;
    }

    LD6002_LogFrame(&frame);
  }
}

void Radar_ServiceOnlineTimeout(void)
{
  uint32_t now;

  if (!Radar_IsReady())
  {
    return;
  }

  if (!s_state->radar_data_online)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - s_state->last_radar_frame_tick) >= LD6002_NO_DATA_TIMEOUT_MS)
  {
    s_state->radar_data_online = false;
    s_state->human_detect_valid = false;
    s_human_candidate_count = 0U;
    s_state->target_range_valid = false;
    s_state->target_range_cm = 0.0f;
    s_state->target_range_filtered_cm = 0.0f;
    s_state->range_state = LD6002_RANGE_UNKNOWN;
    s_state->oled_rate_dirty = true;
    s_state->uart1_summary_dirty = true;
  }
}

void Radar_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
  LD6002_Frame frame;

  if ((!Radar_IsReady()) || (huart == NULL) || (huart->Instance != s_uart2->Instance))
  {
    return;
  }

  if (LD6002_ConsumeByte(&s_ld6002_parser, s_uart2_rx_byte, &frame))
  {
    LD6002_QueuePushFromISR(&frame);
  }

  (void)LD6002_RearmUart2Rx();
}

void Radar_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if ((!Radar_IsReady()) || (huart == NULL) || (huart->Instance != s_uart2->Instance))
  {
    return;
  }

  LD6002_ParserReset(&s_ld6002_parser);
  (void)LD6002_RearmUart2Rx();
}
