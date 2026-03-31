#ifndef RADAR_DATA_H
#define RADAR_DATA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 目标距离状态分类结果。 */
typedef enum
{
  LD6002_RANGE_UNKNOWN = 0,
  LD6002_RANGE_NORMAL,
  LD6002_RANGE_TOO_NEAR,
  LD6002_RANGE_TOO_FAR
} LD6002_RangeState;

/*
 * 雷达业务共享状态：
 * 由雷达接收处理模块更新，OLED/UART1/LED 模块按需读取。
 */
typedef struct
{
  /* 显示/发送脏标记。 */
  bool oled_rate_dirty;
  bool uart1_summary_dirty;

  /* 在线状态与最后接收时间。 */
  bool radar_data_online;
  uint32_t last_radar_frame_tick;

  /* 有人状态（含有效标记）。 */
  bool human_detect_valid;
  bool human_present;

  /* 距离值与距离分类状态。 */
  bool target_range_valid;
  float target_range_cm;
  float target_range_filtered_cm;
  LD6002_RangeState range_state;

  /* 最新生理指标。 */
  float latest_breath_rate;
  float latest_heart_rate;

  /* 待绘制相位与波形刷新标记。 */
  float pending_breath_phase;
  float pending_heart_phase;
  bool oled_wave_pending;
} RadarAppState;

#ifdef __cplusplus
}
#endif

#endif /* RADAR_DATA_H */
