#ifndef RADAR_SERVICE_H
#define RADAR_SERVICE_H

#include "main.h"
#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化业务状态结构体。 */
void Radar_AppStateInit(RadarAppState *state);

/* 初始化雷达接收模块并启动 UART2 中断接收。 */
void Radar_Init(UART_HandleTypeDef *uart2, RadarAppState *state);

/* UART2 接收恢复服务：主循环中调用，负责失败重挂接。 */
void Radar_ServiceUart2RxRecovery(void);

/* 消费已解析帧并更新共享业务状态。 */
void Radar_ProcessFrames(void);

/* 在线超时处理：长时间无帧时回退为离线状态。 */
void Radar_ServiceOnlineTimeout(void);

/* 串口接收完成回调入口（在 HAL_UART_RxCpltCallback 中调用）。 */
void Radar_UartRxCpltCallback(UART_HandleTypeDef *huart);

/* 串口错误回调入口（在 HAL_UART_ErrorCallback 中调用）。 */
void Radar_UartErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_SERVICE_H */
