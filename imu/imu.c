#include "imu.h"

#define Acc_Gain  	0.0001220f		    //加速度计单位化G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 	0.0106227f			//角速度计数据转弧度  (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
//#define Gyro_Gain 	0.0609756f			//角速度计数据转化° (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)

extern int16_t AX, AY, AZ;
extern int16_t GX, GY, GZ;
extern int16_t MX, MY, MZ;

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
float halfT = 0.005f;
float beta = 0.000052914f;
float zero = 0.0f;
void IMU_update(float *pitch,float *roll,float *yaw){
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
	HMC5883L_GetData1(&MX, &MY, &MZ);
	float a_x = (float)AX * Acc_Gain;
	float a_y = (float)AY * Acc_Gain;
	float a_z = (float)AZ * Acc_Gain;
	float g_x = (float)(GX + 53) * Gyro_Gain;
	float g_y = (float)(GY + 12) * Gyro_Gain;
	float g_z = (float)(GZ + 15) * Gyro_Gain;
	float m_x = MX,m_y = MY,m_z = MZ;
	float E_a[4],E_m[4],q_g[4];
	quaternion_unit(&zero,&a_x,&a_y,&a_z);
	quaternion_unit(&zero,&m_x,&m_y,&m_z);
	
	//加速度计更新
	//梯度计算
	E_a[0] = 2*(q0 + q2*a_x - q1*a_y - q0*a_z);
	E_a[1] = 2*(q1 - q3*a_x - q0*a_y + q1*a_z);
	E_a[2] = 2*(q2 + q0*a_x - q3*a_y + q2*a_z);
	E_a[3] = 2*(q3 - q1*a_x - q2*a_y - q3*a_z);
	
	//磁力计更新
	//通过上一时刻四元数计算地球磁向量 [hx, hy, hz] = R(q) × [mx, my, mz]
	float h_x = (q0q0 + q1q1 - q2q2 -q3q3)*m_x + 2*(q1q2 + q0q3) * m_y + 2*(q1q3 - q0q2) * m_z;
	float h_y = 2*(q1q2 - q0q3) * m_x + (q0q0 - q1q1 + q2q2 - q3q3) * m_y + 2*(q0q1 + q2q3) * m_z;
	float h_z = 2*(q0q2 +q1q3) * m_x + 2*(q2q3 - q0q1) * m_y + (q0q0 -q1q1 - q2q2 + q3q3) * m_z;
	float b_x = sqrt(h_x * h_x + h_y * h_y);
	float b_z = h_z;
	float q0bx = q0 * b_x;
	float q1bx = q1 * b_x;
	float q2bx = q2 * b_x;
	float q3bx = q3 * b_x;
	float q0bz = q0 * b_z;
	float q1bz = q1 * b_z;
	float q2bz = q2 * b_z;
	float q3bz = q3 * b_z;
	//构造损失函数
	float f_mx = (q0q0 + q1q1 - q2q2 - q3q3) * b_x + 2*(q1q3 - q0q2) * b_z - m_x;
	float f_my = 2*(q1q2 - q0q3) * b_x + 2*(q0q1 + q2q3) * b_z - m_y;
	float f_mz = 2*(q0q2 + q1q3) * b_x + (q0q0 - q1q1 - q2q2 + q3q3) * b_z - m_z;
	//梯度计算
	E_m[0] = 2*((q0bx - q2bz) * f_mx +(q1bz - q3bx) * f_my + (q2bx + q0bz) * f_mz);
	E_m[1] = 2*((q1bx + q3bz) * f_mx +(q2bx + q0bz) * f_my + (q3bx - q1bz) * f_mz);
	E_m[2] = 2*((-q2bx - q0bz) * f_mx +(q1bx + q3bz) * f_my + (q0bx - q2bz) * f_mz);
	E_m[3] = 2*((q1bz - q3bx) * f_mx +(q2bz - q0bx) * f_my + (q1bx + q3bz) * f_mz);
	
	//陀螺仪更新
	q_g[0] = q0 + (-g_x*q1 - g_y*q2 - g_z*q3)*halfT;
	q_g[1] = q1 + (g_x*q0 + g_z*q2 - g_y*q3)*halfT;
	q_g[2] = q2 + (g_y*q0 - g_z*q1 + g_x*q3)*halfT;
	q_g[3] = q3 + (g_z*q0 + g_y*q1 - g_x*q2)*halfT;
	
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
//	Serial_SendString("{\"qx\":");
//	Serial_SendFloatSimple(q0, 1);
//	Serial_SendString(",\"qy\":");
//	Serial_SendFloatSimple(q1, 1);
//	Serial_SendString(",\"qz\":");
//	Serial_SendFloatSimple(q2, 1);
//	Serial_SendString(",\"qw\":");
//	Serial_SendFloatSimple(q3, 1);
//	Serial_SendString("}\n");
	//四元数转欧拉角
//	*pitch = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;// + 0.32f
//	*roll = asin(2.f * (q0q2 - q1q3))* 57.3f;// + 0.006f
	*roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;// + 0.32f
	*pitch = asin(2.f * (q0q2 - q1q3))* 57.3f;// + 0.006f
	*yaw = atan2(2.f * q0q3 + 2.f * q1q2, q0q0 + q1q1 -q2q2 - q3q3)* 57.3f;// + 0.009f
}

