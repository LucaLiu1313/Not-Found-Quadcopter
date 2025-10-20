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

//void Prepare_Data(void);
float invSqrt(float x);
void quaternion_unit(float * a, float * b, float * c,float * d);
void IMU_update(float *pitch,float *roll,float *yaw);

#endif
