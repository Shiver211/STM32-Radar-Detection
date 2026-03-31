#ifndef RADAR_SERVICE_H
#define RADAR_SERVICE_H

#include "main.h"
#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void Radar_AppStateInit(RadarAppState *state);
void Radar_Init(UART_HandleTypeDef *uart2, RadarAppState *state);
void Radar_ServiceUart2RxRecovery(void);
void Radar_ProcessFrames(void);
void Radar_ServiceOnlineTimeout(void);
void Radar_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Radar_UartErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_SERVICE_H */
