#ifndef UART1_SUMMARY_H
#define UART1_SUMMARY_H

#include "main.h"
#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART1_Summary_Init(UART_HandleTypeDef *uart1);
void UART1_Summary_Service(RadarAppState *state);

#ifdef __cplusplus
}
#endif

#endif /* UART1_SUMMARY_H */
