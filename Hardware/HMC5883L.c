#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "MyI2C.h"
#include "MPU6050.h"

#define HMC5883L_ADDRESS    0x3C

void HMC5883L_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	MyI2C_Start ();
	MyI2C_SendByte (HMC5883L_ADDRESS);
	MyI2C_ReceiveAck ();
	
	MyI2C_SendByte (RegAddress);
	MyI2C_ReceiveAck ();
	
	MyI2C_SendByte (Data);
	MyI2C_ReceiveAck ();
	
	MyI2C_Stop ();
}

uint8_t HMC5883L_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	MyI2C_Start ();
	MyI2C_SendByte (HMC5883L_ADDRESS);
	MyI2C_ReceiveAck ();
	
	MyI2C_SendByte (RegAddress);
	MyI2C_ReceiveAck ();
	
	MyI2C_Start ();
	MyI2C_SendByte (HMC5883L_ADDRESS|0x01);//将读写位变为1
	MyI2C_ReceiveAck ();
	
	Data=MyI2C_ReceiveByte ();
	MyI2C_SendAck (1);
	MyI2C_Stop ();
	
	return Data;
	
}

void HMC5883L_Init(void)
{
	MyI2C_Init ();
	MPU6050_WriteReg(0x6A, 0x00);							//关闭MPU6050的I2C主模式
	MPU6050_WriteReg(0x37, 0x02);							//开启MPU6050旁路模式，直接用主机I2C通信			
	HMC5883L_WriteReg(0x00, 0x18);							//初始化配置寄存器A，默认采样数、最高速、默认测量模式
	HMC5883L_WriteReg(0x01, 0x20);							//初始化配置寄存器B，配置中等增益值
	HMC5883L_WriteReg(0x02, 0x00);							//初始化模式寄存器，选择连续测量模式
	//MPU6050_WriteReg(0x37, 0x00);							//关闭MPU6050旁路模式
	//MPU6050_WriteReg(0x6A, 0x20);							//开启MPU6050的I2C主模式
  }

void MPU_get_HMC(void){
  MPU6050_WriteReg(0x25,HMC5883L_ADDRESS | 0x80);
  MPU6050_WriteReg(0x26,0x03);
  MPU6050_WriteReg(0x27,6 | 0x80);
  MPU6050_WriteReg(0x67, 1);
}

void HMC5883L_GetData(int16_t * magn_x_gs,int16_t *magn_y_gs,int16_t *magn_z_gs)
{
  uint8_t DataH, DataL;
	
	DataH = 0x10;
	DataL = 0x11;
	
	//OLED_ShowChar(1, 1, 'A');
	DataH=MPU6050_ReadReg (0x49);
	DataL=MPU6050_ReadReg (0x4A);
	* magn_x_gs = ((DataH<<8)|DataL);
	
	DataH=MPU6050_ReadReg (0x4D);
	DataL=MPU6050_ReadReg (0x4E);
	* magn_y_gs = ((DataH<<8)|DataL);
	
	DataH=MPU6050_ReadReg (0x4B);
	DataL=MPU6050_ReadReg (0x4C);
	* magn_z_gs = ((DataH<<8)|DataL);
}

void HMC5883L_GetData1(int16_t * magn_x_gs,int16_t *magn_y_gs,int16_t *magn_z_gs)
{
	uint8_t DataH, DataL;
	int16_t t;
	OLED_ShowChar(2, 1, 'A');
	
	DataH=HMC5883L_ReadReg(0x03);
//	OLED_ShowChar(1, 2, 'B');
	DataL=HMC5883L_ReadReg(0x04);
//	OLED_ShowChar(1, 3, 'C');
	t = ((DataH<<8)|DataL);
//	OLED_ShowChar(1, 4, 'D');
	* magn_x_gs = (int16_t)(t);
	
	DataH=HMC5883L_ReadReg(0x07);
	DataL=HMC5883L_ReadReg(0x08);
	t = ((DataH<<8)|DataL);
	* magn_y_gs = (int16_t)(t);
//	OLED_ShowChar(1, 3, 'C');
	
	DataH=HMC5883L_ReadReg(0x05);
	DataL=HMC5883L_ReadReg(0x06);
	* magn_z_gs = ((DataH<<8)|DataL);
	
}
