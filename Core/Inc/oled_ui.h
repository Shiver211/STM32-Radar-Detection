#ifndef OLED_UI_H
#define OLED_UI_H

#include "radar_data.h"

#ifdef __cplusplus
extern "C" {
#endif

void OLED_UI_Init(RadarAppState *state);
void OLED_UI_Service(RadarAppState *state);

#ifdef __cplusplus
}
#endif

#endif /* OLED_UI_H */
