#include "alert_led.h"

#include <stdbool.h>

/* LED 闪烁状态机内部状态。 */
static bool s_alert_led_inited;
static bool s_alert_led_on;
static bool s_alert_led_blink_active;
static uint32_t s_last_alert_led_toggle_tick;

/* 设置 LED 亮灭，自动适配高/低电平点亮。 */
static void AlertLed_Set(bool on)
{
  GPIO_PinState state;

  if (ALERT_LED_ACTIVE_LOW != 0U)
  {
    state = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  else
  {
    state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }

  HAL_GPIO_WritePin(ALERT_LED_GPIO_Port, ALERT_LED_Pin, state);
  s_alert_led_on = on;
}

/* 判断当前是否应进入告警闪烁。 */
static bool AlertLed_ShouldBlink(const RadarAppState *state)
{
  bool no_data_alarm;
  bool no_human_alarm;
  bool range_alarm;

  if (state == NULL)
  {
    return false;
  }

  no_data_alarm = !state->radar_data_online;
  no_human_alarm = state->human_detect_valid && !state->human_present;
  range_alarm = state->target_range_valid &&
                ((state->range_state == LD6002_RANGE_TOO_NEAR) ||
                 (state->range_state == LD6002_RANGE_TOO_FAR));

  return (no_data_alarm || no_human_alarm || range_alarm);
}

/* 初始化 LED 状态机并默认灭灯。 */
void AlertLed_Init(void)
{
  s_alert_led_blink_active = false;
  s_last_alert_led_toggle_tick = HAL_GetTick();
  s_alert_led_inited = true;
  AlertLed_Set(false);
}

/* 周期驱动 LED：异常闪烁，正常熄灭。 */
void AlertLed_Service(const RadarAppState *state)
{
  uint32_t now;
  bool should_blink;

  if (!s_alert_led_inited)
  {
    return;
  }

  now = HAL_GetTick();
  should_blink = AlertLed_ShouldBlink(state);

  if (!should_blink)
  {
    s_alert_led_blink_active = false;
    if (s_alert_led_on)
    {
      AlertLed_Set(false);
    }
    return;
  }

  if (!s_alert_led_blink_active)
  {
    s_alert_led_blink_active = true;
    s_last_alert_led_toggle_tick = now;
    AlertLed_Set(true);
    return;
  }

  if ((now - s_last_alert_led_toggle_tick) >= ALERT_LED_BLINK_MS)
  {
    /* 到达闪烁周期后翻转状态。 */
    s_last_alert_led_toggle_tick = now;
    AlertLed_Set(!s_alert_led_on);
  }
}
