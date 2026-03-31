#ifndef ALERT_LED_H
#define ALERT_LED_H

#include "main.h"
#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LED 告警参数（默认使用 PC13 板载灯，低电平点亮）。 */
#define ALERT_LED_GPIO_Port GPIOC
#define ALERT_LED_Pin       GPIO_PIN_13
#define ALERT_LED_ACTIVE_LOW 1U
#define ALERT_LED_BLINK_MS  500U

/* 初始化告警 LED 状态机并默认灭灯。 */
void AlertLed_Init(void);

/* 根据共享业务状态驱动 LED 闪烁。 */
void AlertLed_Service(const RadarAppState *state);

#ifdef __cplusplus
}
#endif

#endif /* ALERT_LED_H */
