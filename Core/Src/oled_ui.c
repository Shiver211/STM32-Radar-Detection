#include "oled_ui.h"

#include "oled.h"

#include <stdio.h>

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

/* 左侧波形显示模式。 */
typedef enum
{
  OLED_WAVE_MODE_NORMAL = 0,
  OLED_WAVE_MODE_AXES_ONLY
} OLED_WaveMode;

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

/* 中文字模索引（来源于 oledfont.h）。 */
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

/* 上下波形分割虚线 y 坐标。 */
#define OLED_WAVE_SPLIT_Y 31U

/* OLED 刷新周期（ms）。 */
#define OLED_WAVE_UPDATE_MS 50U
#define OLED_TEXT_UPDATE_MS 200U

/* 低值告警阈值与标签闪烁周期。 */
#define OLED_BREATH_LOW_THRESHOLD 5.0f
#define OLED_HEART_LOW_THRESHOLD 40.0f
#define OLED_LOW_LABEL_BLINK_MS 500U

/* OLED 内部运行状态。 */
static bool s_oled_ready;
static bool s_oled_wave_prev_valid;
static uint8_t s_oled_wave_x;
static uint8_t s_prev_breath_y;
static uint8_t s_prev_heart_y;
static uint32_t s_last_oled_wave_tick;
static uint32_t s_last_oled_text_tick;
static bool s_oled_low_label_visible;
static uint32_t s_last_oled_low_label_blink_tick;
static bool s_oled_label_state_valid;
static bool s_oled_breath_label_drawn;
static bool s_oled_heart_label_drawn;
static OLED_WaveMode s_oled_wave_mode;
static OLED_RightPanelMode s_oled_panel_drawn_mode;

static uint8_t OLED_MapPhaseToY(float value, uint8_t center_y, float gain, uint8_t y_min, uint8_t y_max);
static void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
static void OLED_DrawWaveAxes(void);
static void OLED_ClearWaveAndKeepAxes(void);
static OLED_RightPanelMode OLED_GetPanelMode(const RadarAppState *state);
static OLED_WaveMode OLED_GetWaveMode(const RadarAppState *state);
static void OLED_UpdateVitalsLabels(const RadarAppState *state);
static void OLED_DrawPanelMode(OLED_RightPanelMode mode, const RadarAppState *state);
static void OLED_PlotWaveColumn(float breath_phase, float heart_phase);
static void OLED_UpdateVitalsText(const RadarAppState *state);

/* 将相位值映射到指定 y 区间。 */
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

/* Bresenham 直线绘制，用于连接相邻采样点。 */
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

/* 绘制左侧波形区静态坐标轴。 */
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

/* 清空左侧波形区并保留坐标轴。 */
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

/* 根据在线/人体/距离状态选择右侧显示模式。 */
static OLED_RightPanelMode OLED_GetPanelMode(const RadarAppState *state)
{
  if (!state->radar_data_online)
  {
    return OLED_PANEL_NO_DATA;
  }

  if (state->human_detect_valid && !state->human_present)
  {
    return OLED_PANEL_NO_HUMAN;
  }

  if (state->target_range_valid)
  {
    if (state->range_state == LD6002_RANGE_TOO_FAR)
    {
      return OLED_PANEL_DIST_TOO_FAR;
    }

    if (state->range_state == LD6002_RANGE_TOO_NEAR)
    {
      return OLED_PANEL_DIST_TOO_NEAR;
    }
  }

  return OLED_PANEL_VITALS;
}

/* 根据右侧模式决定左侧波形绘制策略。 */
static OLED_WaveMode OLED_GetWaveMode(const RadarAppState *state)
{
  OLED_RightPanelMode panel_mode = OLED_GetPanelMode(state);

  if ((panel_mode == OLED_PANEL_NO_DATA) ||
      (panel_mode == OLED_PANEL_NO_HUMAN) ||
      (panel_mode == OLED_PANEL_DIST_TOO_FAR) ||
      (panel_mode == OLED_PANEL_DIST_TOO_NEAR))
  {
    return OLED_WAVE_MODE_AXES_ONLY;
  }

  return OLED_WAVE_MODE_NORMAL;
}

/* 刷新“呼吸/心率”标签，低值时按闪烁状态显隐。 */
static void OLED_UpdateVitalsLabels(const RadarAppState *state)
{
  bool breath_low = (state->latest_breath_rate >= 0.0f) && (state->latest_breath_rate < OLED_BREATH_LOW_THRESHOLD);
  bool heart_low = (state->latest_heart_rate >= 0.0f) && (state->latest_heart_rate < OLED_HEART_LOW_THRESHOLD);

  bool show_breath_label = (!breath_low) || s_oled_low_label_visible;
  bool show_heart_label = (!heart_low) || s_oled_low_label_visible;

  if (s_oled_label_state_valid &&
      (show_breath_label == s_oled_breath_label_drawn) &&
      (show_heart_label == s_oled_heart_label_drawn))
  {
    return;
  }

  if ((!s_oled_label_state_valid) || (show_breath_label != s_oled_breath_label_drawn))
  {
    OLED_Fill(OLED_VALUE_LABEL_X, 0U, (u8)(OLED_VALUE_LABEL_X + 31U), 15U, 0U);

    if (show_breath_label)
    {
      OLED_ShowCHinese(OLED_VALUE_LABEL_X, 0U, OLED_ZH_HU);
      OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 0U, OLED_ZH_XI);
    }
  }

  if ((!s_oled_label_state_valid) || (show_heart_label != s_oled_heart_label_drawn))
  {
    OLED_Fill(OLED_VALUE_LABEL_X, 32U, (u8)(OLED_VALUE_LABEL_X + 31U), 47U, 0U);

    if (show_heart_label)
    {
      OLED_ShowCHinese(OLED_VALUE_LABEL_X, 4U, OLED_ZH_XIN);
      OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 4U, OLED_ZH_LV);
    }
  }

  s_oled_breath_label_drawn = show_breath_label;
  s_oled_heart_label_drawn = show_heart_label;
  s_oled_label_state_valid = true;
}

/* 重绘右侧信息区固定内容。 */
static void OLED_DrawPanelMode(OLED_RightPanelMode mode, const RadarAppState *state)
{
  OLED_Fill(OLED_VALUE_AREA_X_START, 0U, OLED_VALUE_AREA_X_END, 63U, 0U);
  s_oled_label_state_valid = false;

  switch (mode)
  {
  case OLED_PANEL_VITALS:
    OLED_UpdateVitalsLabels(state);
    break;

  case OLED_PANEL_NO_DATA:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO);
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2);
    OLED_ShowString(OLED_TEXT_2CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"No", 8U);
    OLED_ShowString(OLED_TEXT_4CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Data", 8U);
    break;

  case OLED_PANEL_NO_HUMAN:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO);
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2);
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 5U, OLED_ZH_WU);
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 5U, OLED_ZH_REN);
    break;

  case OLED_PANEL_DIST_TOO_FAR:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO);
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2);
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"Too", 8U);
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Far", 8U);
    break;

  case OLED_PANEL_DIST_TOO_NEAR:
    OLED_ShowCHinese(OLED_VALUE_LABEL_X, 1U, OLED_ZH_CUO);
    OLED_ShowCHinese((u8)(OLED_VALUE_LABEL_X + 16U), 1U, OLED_ZH_WU2);
    OLED_ShowString(OLED_TEXT_3CHAR_X, OLED_TEXT_LINE1_Y, (u8 *)"Too", 8U);
    OLED_ShowString(OLED_TEXT_4CHAR_X, OLED_TEXT_LINE2_Y, (u8 *)"Near", 8U);
    break;

  default:
    break;
  }
}

/* 绘制一列波形并推进滚动游标。 */
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

/* 刷新右侧文本（提示文案或数值）。 */
static void OLED_UpdateVitalsText(const RadarAppState *state)
{
  OLED_RightPanelMode panel_mode;
  int32_t br = (int32_t)(state->latest_breath_rate + ((state->latest_breath_rate >= 0.0f) ? 0.5f : -0.5f));
  int32_t hr = (int32_t)(state->latest_heart_rate + ((state->latest_heart_rate >= 0.0f) ? 0.5f : -0.5f));
  uint16_t br_u;
  uint16_t hr_u;
  char br_text[8];
  char hr_text[8];

  panel_mode = OLED_GetPanelMode(state);
  if (panel_mode != s_oled_panel_drawn_mode)
  {
    OLED_DrawPanelMode(panel_mode, state);
    s_oled_panel_drawn_mode = panel_mode;
  }

  if (panel_mode != OLED_PANEL_VITALS)
  {
    return;
  }

  OLED_UpdateVitalsLabels(state);

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

/* OLED 初始化：清屏、画轴并初始化运行状态。 */
void OLED_UI_Init(RadarAppState *state)
{
  OLED_Init();
  OLED_Clear();

  OLED_DrawWaveAxes();
  OLED_RefreshDirty();

  s_oled_wave_x = 0U;
  s_oled_wave_prev_valid = false;

  s_oled_low_label_visible = true;
  s_last_oled_low_label_blink_tick = HAL_GetTick();

  s_oled_label_state_valid = false;
  s_oled_breath_label_drawn = false;
  s_oled_heart_label_drawn = false;

  s_oled_wave_mode = OLED_WAVE_MODE_AXES_ONLY;
  s_oled_panel_drawn_mode = OLED_PANEL_MODE_COUNT;

  s_last_oled_wave_tick = HAL_GetTick();
  s_last_oled_text_tick = HAL_GetTick();

  s_oled_ready = true;

  if (state != NULL)
  {
    state->oled_rate_dirty = true;
    state->oled_wave_pending = false;
    OLED_UpdateVitalsText(state);
  }
}

/*
 * OLED 周期服务：
 * 1) 模式切换
 * 2) 低值闪烁
 * 3) 波形刷新
 * 4) 文本刷新
 */
void OLED_UI_Service(RadarAppState *state)
{
  uint32_t now;
  OLED_WaveMode target_wave_mode;
  OLED_RightPanelMode panel_mode;
  bool breath_low;
  bool heart_low;
  bool any_low;

  if ((!s_oled_ready) || (state == NULL))
  {
    return;
  }

  now = HAL_GetTick();

  target_wave_mode = OLED_GetWaveMode(state);
  if (target_wave_mode != s_oled_wave_mode)
  {
    s_oled_wave_mode = target_wave_mode;
    state->oled_wave_pending = false;
    s_oled_wave_prev_valid = false;
    s_oled_wave_x = 0U;
    OLED_ClearWaveAndKeepAxes();

    OLED_UpdateVitalsText(state);
    s_last_oled_text_tick = now;
    state->oled_rate_dirty = false;
  }

  panel_mode = OLED_GetPanelMode(state);
  if (panel_mode == OLED_PANEL_VITALS)
  {
    breath_low = (state->latest_breath_rate >= 0.0f) && (state->latest_breath_rate < OLED_BREATH_LOW_THRESHOLD);
    heart_low = (state->latest_heart_rate >= 0.0f) && (state->latest_heart_rate < OLED_HEART_LOW_THRESHOLD);
    any_low = breath_low || heart_low;

    if (any_low)
    {
      if ((now - s_last_oled_low_label_blink_tick) >= OLED_LOW_LABEL_BLINK_MS)
      {
        s_last_oled_low_label_blink_tick = now;
        s_oled_low_label_visible = !s_oled_low_label_visible;
        OLED_UpdateVitalsText(state);
        s_last_oled_text_tick = now;
        state->oled_rate_dirty = false;
      }
    }
    else
    {
      if (!s_oled_low_label_visible)
      {
        s_oled_low_label_visible = true;
        OLED_UpdateVitalsText(state);
        s_last_oled_text_tick = now;
        state->oled_rate_dirty = false;
      }

      s_last_oled_low_label_blink_tick = now;
    }
  }
  else
  {
    s_oled_low_label_visible = true;
    s_last_oled_low_label_blink_tick = now;
  }

  if (state->oled_wave_pending && ((now - s_last_oled_wave_tick) >= OLED_WAVE_UPDATE_MS))
  {
    if (s_oled_wave_mode == OLED_WAVE_MODE_NORMAL)
    {
      OLED_PlotWaveColumn(state->pending_breath_phase, state->pending_heart_phase);
    }

    s_last_oled_wave_tick = now;
    state->oled_wave_pending = false;
  }

  if (state->oled_rate_dirty && ((now - s_last_oled_text_tick) >= OLED_TEXT_UPDATE_MS))
  {
    OLED_UpdateVitalsText(state);
    s_last_oled_text_tick = now;
    state->oled_rate_dirty = false;
  }
}
