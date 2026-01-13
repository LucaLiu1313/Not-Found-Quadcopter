#ifndef   _IMU_H_
#define   _IMU_H_
//#include "stm32.h"
//#include "OS.h"
#include <math.h>
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "Delay.h"
#include "MPU6050.h"
#include "HMC5883L.h"  
#include "Serial.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "OLED.h"
#include "OLED.h"
#include "pwm.h"

//误差结构体
struct imu_err{
	int16_t count;
	int16_t AX,AY,GX,GY,GZ;
	int16_t AX_sum,AY_sum,GX_sum,GY_sum,GZ_sum;
};
//void Prepare_Data(void);
float invSqrt(float x);
void quaternion_unit(float * a, float * b, float * c,float * d);
void IMU_update(float *pitch,float *roll,float *yaw);

// 读取当前陀螺角速度（度/秒），基于与 IMU_update 相同的偏置与比例
void IMU_get_gyro_degps(float *gx, float *gy, float *gz);

// 获取/设置 IMU 的积分周期（秒）。IMU 内部使用 halfT，dt = 2*halfT。
float IMU_GetDtSeconds(void);
void IMU_SetDtSeconds(float dt_seconds);

//静态标定校准误差
void err_update(void);
#endif
