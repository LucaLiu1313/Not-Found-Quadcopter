#include "pid.h"
#include <math.h>
#include "Serial.h"
// 参数
const float ITHRE = 2;
const float posISepThre = 0.4f;
const float rateIsepThre = 1.5f;
const float DEG2RAD = 0.017453292519943295f;

// 外部提供：周期(秒) 与 陀螺角速度(弧度/秒)
float g_pid_dt_sec = 1.0f;
float gyroRate[3] = {0};
float g_cur_yaw_rad = 0.0f;
float deltaError = 0.0f;

// 两环 PID 实例
PID_T posPID;
PID_T ratePID;

void initPID(PID_T *ppid, const float pram[PID_DIM][3]) {
  for (int i = 0; i < PID_DIM; ++i) {
    for (int j = 0; j < 3; ++j) {
      ppid->pram[i][j] = pram[i][j];
    }
    ppid->intergrator[i] = 0;
    ppid->lastError[i] = 0;
    ppid->control[i] = 0;
  }
}

void updPID(PID_T *ppid, const float *error, float deltas, float intThre) {
  for (int i = 0; i < PID_DIM; ++i) {
    ppid->control[i] = 0;
  }
  
  // P
  for (int i = 0; i < PID_DIM; ++i) {
    ppid->control[i] += ppid->pram[i][0] * error[i];
  }

  // I（带积分分离与限幅）
  for (int i = 0; i < PID_DIM; ++i) {
    if (fabsf(error[i]) > intThre) {
      ppid->intergrator[i] *= 0.3f;
    } else {
      ppid->intergrator[i] += ppid->pram[i][1] *  error[i] * deltas;
    }
    if (fabsf(ppid->intergrator[i]) > ITHRE) {
		ppid->intergrator[i] = (ppid->intergrator[i]>0)? ITHRE:-ITHRE; // 保持与原实现一致
    }
	ppid->control[i] += ppid->intergrator[i];
	//Serial_SendFloatSimple(ppid->intergrator[i], 1);
	//Serial_SendString("\n");
  }

  // D（保护极小 deltas）
  const float safe_dt = deltas > 1e-6f ? deltas : 1e-6f;
  for (int i = 0; i < PID_DIM; ++i) {
    deltaError = error[i] - ppid->lastError[i];
	  if((deltaError > 0)&&i==1){
		  ppid->control[i] += 1.5*ppid->pram[i][2] * (deltaError / safe_dt);
	  }
	  else{
		  ppid->control[i] += ppid->pram[i][2] * (deltaError / safe_dt);
	  } 
  }

  for (int i = 0; i < PID_DIM; ++i) {
    ppid->lastError[i] = error[i];
  }
}

void initbothPID(void) {
  float pramOut[PID_DIM][3] = {
      {0.1, 0, 0.005},
      {0.15, 0, 0.05},
      {0.2, 0, 0.05},
  };
//	float pramOut[PID_DIM][3] = {
//      {0, 0, 0.0003f},
//      {0, 0, 0.0003f},
//      {0, 0, 0.0003f},
//  };
  float pramIn[PID_DIM][3] = {
      {1.1, 2, 0.05},
      {1, 1, 0.05},
      {0.4, 0.5, 0.1},
  };
  initPID(&posPID, pramOut);
  initPID(&ratePID, pramIn);
}

void PID_ResetIntegrators(void) {
  for (int i = 0; i < PID_DIM; ++i) {
    posPID.intergrator[i] = 0;
    ratePID.intergrator[i] = 0;
  }
}

void PID_SetDtSeconds(float dt_seconds) {
  g_pid_dt_sec = dt_seconds;
}

void PID_SetGyroDegps(float gx_degps, float gy_degps, float gz_degps) {
  //const float DEG2RAD = 0.017453292519943295f;
  gyroRate[0] = gx_degps;
  gyroRate[1] = gy_degps;
  gyroRate[2] = gz_degps;
}

void PID_UpdateCurFromIMU(float pitch_deg, float roll_deg, float yaw_deg, float cur[3]) {
  cur[0] = yaw_deg * DEG2RAD;
  cur[1] = pitch_deg * DEG2RAD;
  cur[2] = roll_deg * DEG2RAD;
  g_cur_yaw_rad = cur[0];
}

void PID(float *tar, float *cur, float *control) {
  static float lpfAcc[3] = {0};
  static const float gyroPass = 1.0f;

  for (int i = 0; i < 3; ++i) {
    lpfAcc[i] = gyroPass * cur[i] + (1 - gyroPass) * lpfAcc[i];
  }

  float error[PID_DIM];
  for (int i = 0; i < PID_DIM; ++i) {
    error[i] = tar[i] - lpfAcc[i];
  }

  // 外环（姿态）
  updPID(&posPID, error, g_pid_dt_sec, posISepThre);

  // 内环（角速度），映射：yaw->Z, pitch->Y, roll->X
  //error[0] = posPID.control[0] - gyroRate[2];
	error[1] = posPID.control[1] - gyroRate[1];
	error[2] = posPID.control[2] - gyroRate[0];
//  OLED_ShowFNum(2, 8, posPID.control[0], 5,2);
//  OLED_ShowFNum(2, 1, gyroRate[2], 5,2);
	error[0] = tar[0] - gyroRate[2];
//  error[1] = 1.5*tar[1] - gyroRate[1];
//  error[2] = 3*tar[2] - gyroRate[0];
//    error[0] = 4*tar[0];
//  error[1] = 4*tar[1];
//  error[2] = 4*tar[2];
//  	Serial_SendFloatSimple(error[0], 3);
//	Serial_SendString("  ");
//  	Serial_SendFloatSimple(error[1], 3);
//	Serial_SendString("  ");
//  	Serial_SendFloatSimple(error[2], 3);
//	Serial_SendString("\n");
//    Serial_SendFloatSimple(4*tar[2], 3);
//	Serial_SendString("  ");
//  	Serial_SendFloatSimple(gyroRate[0], 3);
//	Serial_SendString("\n");
  updPID(&ratePID, error, g_pid_dt_sec, rateIsepThre);

  for (int i = 0; i < PID_DIM; ++i) {
    control[i] = ratePID.control[i];
  }
  }
