#include "stm32.h"
#include "OS.h"
#include "hardware.h"
#include "imu.h"
#include <math.h>

#define Acc_Gain  	0.0001220f		    //加速度计单位化G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 	0.0106227f			//角速度计数据转弧度  (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
//#define Gyro_Gain 	0.0609756f			//角速度计数据转化° (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)

extern int16_t AX, AY, AZ;
extern int16_t GX, GY, GZ;
extern float MX, MY, MZ;

//快速求1/sort(x)
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//单位化四元数
void quaternion_unit(float *a, float *b, float *c,float *d){
	float q0 = *a, q1 = *b, q2 = *c, q3 = *d;
    float norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    *a = q0 * norm;
    *b = q1 * norm;
    *c = q2 * norm;
    *d = q3 * norm;
}


//姿态更新
float q0 = 1,q1 = 0,q2 = 0,q3 = 0;
float E0,E1,E2,E3;
float halfT = 0.009f;
float beta = 0.000052914f;
float zero = 0.0f;
void IMU_update(float *roll,float *pitch,float *yaw){
	//提前计算
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	//数据初始化（单位转换）
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	MPU_get_HMCData(&MX, &MY, &MZ);
	float a_x = (float)AX * Acc_Gain;
	float a_y = (float)AY * Acc_Gain;
	float a_z = (float)AZ * Acc_Gain;
	float g_x = (float)(GX + 53) * Gyro_Gain;
	float g_y = (float)(GY + 12) * Gyro_Gain;
	float g_z = (float)(GZ + 15) * Gyro_Gain;
//	OLED_ShowSignedNum(2, 8, g_x, 3);
//	OLED_ShowSignedNum(3, 8, g_y, 3);
//	OLED_ShowSignedNum(4, 8, g_z, 3);
	//float g_x = GX,g_y = GY,g_z = GZ;
	float m_x = MX,m_y = MY,m_z = MZ;
	float E_a[4],E_m[4],q_g[4];
	
	//加速度计更新
	//梯度下降
	quaternion_unit(&zero,&a_x,&a_y,&a_z);
	E_a[0] = 2*(q0 + q2*a_x - q1*a_y - q0*a_z);
	E_a[1] = 2*(q1 - q3*a_x - q0*a_y + q1*a_z);
	E_a[2] = 2*(q2 + q0*a_x - q3*a_y + q2*a_z);
	E_a[3] = 2*(q3 - q1*a_x - q2*a_y - q3*a_z);
//	//固定步长
//	float ut = 0.001f;
//	q_a[0] = q0 - ut*E0;
//	q_a[1] = q1 - ut*E1;
//	q_a[2] = q2 - ut*E2;
//	q_a[3] = q3 - ut*E3;
	//单位化
	//quaternion_unit(q_a,q_a+1,q_a+2,q_a+3);
	
	//磁力计更新
	quaternion_unit(&zero,&m_x,&m_y,&m_z);
	//通过上一时刻四元数计算地球磁向量 [hx, hy, hz] = R(q) × [mx, my, mz]
	float h_x = (q0q0 + q1q1 - q2q2 -q3q3)*m_x + 2*(q1q2 + q0q3) * m_y + 2*(q1q2 -q0q2) * m_z;
	float h_y = 2*(q1q2 - q0q3) * m_x + (q0q0 - q1q1 + q2q2 - q3q3) * m_y + 2*(q0q1 + q2q3) * m_z;
	float h_z = 2*(q0q2 +q1q3) * m_x + 2*(q2q3 - q0q1) * m_y + (q0q0 -q1q1 - q2q2 - q3q3) * m_z;
	float b_x = sqrt(h_x * h_x + h_y * h_y);
	float b_z = h_z;
//	float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
//	hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
//	_2bx = sqrt(hx * hx + hy * hy);
//	_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
//	_4bx = 2.0f * _2bx;
//	_4bz = 2.0f * _2bz;
	float f_mx = (q0q0 + q1q1 - q2q2 - q3q3) * b_x + 2*(q1q3 - q0q2) * b_z - m_x;
	float f_my = 2*(q1q2 - q0q3) * b_x + 2*(q0q1 + q2q3) * b_z - m_y;
	float f_mz = 2*(q0q2 + q1q3) * b_x + (q0q0 - q1q1 - q2q2 + q3q3) * b_z - m_z;
	float q0bx = q0 * b_x;
	float q1bx = q1 * b_x;
	float q2bx = q2 * b_x;
	float q3bx = q3 * b_x;
	float q0bz = q0 * b_z;
	float q1bz = q1 * b_z;
	float q2bz = q2 * b_z;
	float q3bz = q3 * b_z;
	E_m[0] = 2*((q0bx - q2bz) * f_mx +(q1bz - q3bx) * f_my + (q2bx + q0bz) * f_mz);
	E_m[1] = 2*((q1bx + q3bz) * f_mx +(q2bx + q0bz) * f_my + (q3bx - q1bz) * f_mz);
	E_m[2] = 2*((-q2bx - q0bz) * f_mx +(q1bx + q3bz) * f_my + (q0bx - q2bz) * f_mz);
	E_m[3] = 2*((q1bz - q3bx) * f_mx +(q2bz - q0bx) * f_my + (q1bx + q3bz) * f_mz);
	//陀螺仪更新
	q_g[0] = q0 + (-g_x*q1 - g_y*q2 - g_z*q3)*halfT;
	q_g[1] = q1 + (g_x*q0 + g_z*q2 - g_y*q3)*halfT;
	q_g[2] = q2 + (g_y*q0 - g_z*q1 + g_x*q3)*halfT;
	q_g[3] = q3 + (g_z*q0 + g_y*q1 - g_x*q2)*halfT;
	//quaternion_unit(q_g,q_g+1,q_g+2,q_g+3);
//	//验证（直接使用陀螺仪输出）
//	q0 = q_g[0];
//	q1 = q_g[1];
//	q2 = q_g[2];
//	q3 = q_g[3];
	//madgwick实现
	E0 = E_a[0] + E_m[0];
	E1 = E_a[1] + E_m[1];
	E2 = E_a[2] + E_m[2];
	E3 = E_a[3] + E_m[3];
	quaternion_unit(&E0,&E1,&E2,&E3);
	//q0 = q_g[0] - bata*dertaF[0]*halfT*2
	q0 = q_g[0] - beta*E0*halfT*2;
	q1 = q_g[1] - beta*E1*halfT*2;
	q2 = q_g[2] - beta*E2*halfT*2;
	q3 = q_g[3] - beta*E3*halfT*2;

	//单位化四元数
	quaternion_unit(&q0,&q1,&q2,&q3);
//	OLED_ShowSignedNum(1, 1, q0, 3);
//	OLED_ShowSignedNum(1, 4, q1, 3);
//	OLED_ShowSignedNum(1, 8, q2, 3);

	//四元数转欧拉角
	*roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;// + 0.32f
	*pitch = asin(2.f * (q0q2 - q1q3))* 57.3f;// + 0.006f
	*yaw = atan2(2.f * q0q3 + 2.f * q1q2, q0q0 + q1q1 -q2q2 - q3q3)* 57.3f;// + 0.009f
}

