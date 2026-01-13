#ifndef USER_PID_PID_H
#define USER_PID_PID_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DIM 3

typedef struct {
  float lastError[PID_DIM];
  float pram[PID_DIM][3];
  float intergrator[PID_DIM];
  float control[PID_DIM];
} PID_T;

// 外部提供：控制周期(秒)与角速度(弧度/秒)
extern float g_pid_dt_sec;
extern float gyroRate[3];
extern float g_cur_yaw_rad;
//extern float deltaError;

// 全局 PID 实例：外环(姿态) 与 内环(角速度)
extern PID_T posPID;
extern PID_T ratePID;

// 初始化单个 PID 的参数矩阵 [dim][P,I,D]
void initPID(PID_T *ppid, const float pram[PID_DIM][3]);

// 更新单个 PID：根据误差、周期与积分分离阈值更新输出
void updPID(PID_T *ppid, const float *error, float deltas, float intThre);

// 使用默认参数初始化外环与内环
void initbothPID(void);

// 清空两环积分（上锁/油门低位等场景由上层调用）
void PID_ResetIntegrators(void);

// 两环 PID：输入目标姿态tar、当前姿态cur(均弧度)，输出控制量control
void PID(float *tar, float *cur, float *control);

// 辅助：设置周期(秒)
void PID_SetDtSeconds(float dt_seconds);
// 辅助：陀螺(度/秒)→内部(弧度/秒)
void PID_SetGyroDegps(float gx_degps, float gy_degps, float gz_degps);
// 辅助：IMU输出(度)→cur(弧度)并更新 g_cur_yaw_rad
void PID_UpdateCurFromIMU(float pitch_deg, float roll_deg, float yaw_deg, float cur[3]);

#ifdef __cplusplus
}
#endif

#endif // USER_PID_PID_H

