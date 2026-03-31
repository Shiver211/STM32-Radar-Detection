#ifndef RADAR_DATA_H
#define RADAR_DATA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  LD6002_RANGE_UNKNOWN = 0,
  LD6002_RANGE_NORMAL,
  LD6002_RANGE_TOO_NEAR,
  LD6002_RANGE_TOO_FAR
} LD6002_RangeState;

typedef struct
{
  bool oled_rate_dirty;
  bool uart1_summary_dirty;

  bool radar_data_online;
  uint32_t last_radar_frame_tick;

  bool human_detect_valid;
  bool human_present;

  bool target_range_valid;
  float target_range_cm;
  float target_range_filtered_cm;
  LD6002_RangeState range_state;

  float latest_breath_rate;
  float latest_heart_rate;

  float pending_breath_phase;
  float pending_heart_phase;
  bool oled_wave_pending;
} RadarAppState;

#ifdef __cplusplus
}
#endif

#endif /* RADAR_DATA_H */
