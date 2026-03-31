#ifndef UART1_SUMMARY_H
#define UART1_SUMMARY_H

#include "main.h"
#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 UART1 汇总发送模块。 */
void UART1_Summary_Init(UART_HandleTypeDef *uart1);

/* 周期发送汇总信息（带脏标记和时间节流）。 */
void UART1_Summary_Service(RadarAppState *state);

#ifdef __cplusplus
}
#endif

#endif /* UART1_SUMMARY_H */
