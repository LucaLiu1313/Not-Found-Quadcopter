
// 何时需要改这些宏
// 接收机通道顺序与默认不同：改 RC_CH_* 映射到你的 roll/pitch/yaw 实际通道。
// 遥控器中点或行程不同：改 RC_PULSE_MID/RC_PULSE_SPAN。
// 想要更“稳”或“灵敏”：调 RC_DEADZONE 与最大角/速率（ROLL_MAX_DEG、PITCH_MAX_DEG、YAW_RATE_MAX_DEGPS）。
// 想让偏航也用角度模式：把 YAW_MODE_RATE 设为 0，并按 YAW_MAX_DEG 生效。
#include <stdint.h>
#include "PPM.h"

#ifndef RC_PULSE_MID
#define RC_PULSE_MID 1500
#endif

#ifndef RC_PULSE_SPAN
#define RC_PULSE_SPAN 500
#endif

#ifndef RC_DEADZONE
#define RC_DEADZONE 0.05f
#endif

// 通道映射：按你的接收机映射覆盖这些宏（0-based）
#ifndef RC_CH_ROLL
#define RC_CH_ROLL 3
#endif

#ifndef RC_CH_PITCH
#define RC_CH_PITCH 1
#endif

#ifndef RC_CH_YAW
#define RC_CH_YAW 0
#endif

// 最大角度/速率（单位：度/度每秒）
#ifndef ROLL_MAX_DEG
#define ROLL_MAX_DEG 25.0f
#endif

#ifndef PITCH_MAX_DEG
#define PITCH_MAX_DEG 25.0f
#endif

#ifndef YAW_MAX_DEG
#define YAW_MAX_DEG 30.0f
#endif

// 0=偏航角度模式；1=偏航速率模式（需提供 g_pid_dt_sec、g_cur_yaw_rad）
#ifndef YAW_MODE_RATE
#define YAW_MODE_RATE 1
#endif

#if YAW_MODE_RATE
#ifndef YAW_RATE_MAX_DEGPS
#define YAW_RATE_MAX_DEGPS 120.0f
#endif
extern float g_pid_dt_sec;   // 控制周期，单位：秒
extern float g_cur_yaw_rad;  // 当前偏航角，单位：弧度
#endif

// 直接使用 PPM.c 导出的 g_rc_us[]

static inline float clampf(float x, float minv, float maxv) {
  return x < minv ? minv : (x > maxv ? maxv : x);
}

static inline float deadzonef(float x, float dz) {
  return (x > -dz && x < dz) ? 0.0f : x;
}

void getWantedYPR(float yprRAD[3]) {
  const float DEG2RAD = 0.017453292519943295f;
  const float rollMaxRad = ROLL_MAX_DEG * DEG2RAD;
  const float pitchMaxRad = PITCH_MAX_DEG * DEG2RAD;
#if YAW_MODE_RATE
  const float yawRateMaxRadps = YAW_RATE_MAX_DEGPS * DEG2RAD;
#else
  const float yawMaxRad = YAW_MAX_DEG * DEG2RAD;
#endif

  float s_roll = ((int)g_rc_us[RC_CH_ROLL] - RC_PULSE_MID) / (float)RC_PULSE_SPAN;
  float s_pitch = ((int)g_rc_us[RC_CH_PITCH] - RC_PULSE_MID) / (float)RC_PULSE_SPAN;
  float s_yaw = ((int)g_rc_us[RC_CH_YAW] - RC_PULSE_MID) / (float)RC_PULSE_SPAN;

  s_roll = deadzonef(clampf(s_roll, -1.0f, 1.0f), RC_DEADZONE);
  s_pitch = deadzonef(clampf(s_pitch, -1.0f, 1.0f), RC_DEADZONE);
  s_yaw = deadzonef(clampf(s_yaw, -1.0f, 1.0f), RC_DEADZONE);

  // 符号与原工程保持一致：yaw-, pitch-, roll+
#if YAW_MODE_RATE
  yprRAD[0] = g_cur_yaw_rad + (-s_yaw) * yawRateMaxRadps * g_pid_dt_sec;
#else
  yprRAD[0] = (-s_yaw) * yawMaxRad;
#endif
  yprRAD[1] = (-s_pitch) * pitchMaxRad;
  yprRAD[2] = (s_roll) * rollMaxRad;
}
