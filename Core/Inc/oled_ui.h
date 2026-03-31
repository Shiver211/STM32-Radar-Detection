#ifndef OLED_UI_H
#define OLED_UI_H

#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 OLED UI：清屏、绘制坐标轴并重置显示状态。 */
void OLED_UI_Init(RadarAppState *state);

/* OLED 周期服务：刷新波形、文本与异常提示。 */
void OLED_UI_Service(RadarAppState *state);

#ifdef __cplusplus
}
#endif

#endif /* OLED_UI_H */
