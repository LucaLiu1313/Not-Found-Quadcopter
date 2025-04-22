#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS    0xD0
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	MyI2C_Start ();
	MyI2C_SendByte (MPU6050_ADDRESS);
	MyI2C_ReceiveAck ();
	
	MyI2C_SendByte (RegAddress);
	MyI2C_ReceiveAck ();
	
	MyI2C_SendByte (Data);
	MyI2C_ReceiveAck ();
	
	MyI2C_Stop ();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	MyI2C_Start ();
	MyI2C_SendByte (MPU6050_ADDRESS);
	MyI2C_ReceiveAck ();
	
	MyI2C_SendByte (RegAddress);
	MyI2C_ReceiveAck ();
	
	MyI2C_Start ();
	MyI2C_SendByte (MPU6050_ADDRESS|0x01);//将读写位变为1
	MyI2C_ReceiveAck ();
	
	Data=MyI2C_ReceiveByte ();
	MyI2C_SendAck (1);
	MyI2C_Stop ();
	
	return Data;
	
}  //

void MPU6050_Init(void)
{
	MyI2C_Init ();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1	,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2	,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV		,0x09);//10分频
	MPU6050_WriteReg(MPU6050_CONFIG		,0x06);//滤波max
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG	,0x18);//陀螺仪max
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG	,0x08);//加速度计max
	//MPU6050_WriteReg(MPU6050_USER_CTRL	,0x80);//使能I2C 主模式
	//MPU6050_WriteReg(MPU6050_INT_PIN_CFG	,0x02);//旁路模式
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg (MPU6050_WHO_AM_I);
}


void MPU6050_GetData(int16_t * AccX,int16_t * AccY,int16_t * AccZ,
								int16_t * GyroX,int16_t * GyroY,int16_t *GyroZ)
{
	
	uint8_t DataH,DataL;
	
	DataH=MPU6050_ReadReg (MPU6050_ACCEL_XOUT_H);
	DataL=MPU6050_ReadReg (MPU6050_ACCEL_XOUT_L);
	* AccX = (DataH<<8)|DataL;
	
	DataH=MPU6050_ReadReg (MPU6050_ACCEL_YOUT_H);
	DataL=MPU6050_ReadReg (MPU6050_ACCEL_YOUT_L);
	* AccY = (DataH<<8)|DataL;
	
	DataH=MPU6050_ReadReg (MPU6050_ACCEL_ZOUT_H);
	DataL=MPU6050_ReadReg (MPU6050_ACCEL_ZOUT_L);
	* AccZ = (DataH<<8)|DataL;
	
	DataH=MPU6050_ReadReg (MPU6050_GYRO_XOUT_H);
	DataL=MPU6050_ReadReg (MPU6050_GYRO_XOUT_L);
	* GyroX = (DataH<<8)|DataL;
	
	DataH=MPU6050_ReadReg (MPU6050_GYRO_YOUT_H);
	DataL=MPU6050_ReadReg (MPU6050_GYRO_YOUT_L);
	* GyroY = (DataH<<8)|DataL;
	
	DataH=MPU6050_ReadReg (MPU6050_GYRO_ZOUT_H);
	DataL=MPU6050_ReadReg (MPU6050_GYRO_ZOUT_L);
	* GyroZ = (DataH<<8)|DataL;
	
}



