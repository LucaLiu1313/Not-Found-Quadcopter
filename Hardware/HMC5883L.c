#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "HMC5883L_Reg.h"
#include "Delay.h"
#include "Serial.h"

void HMC5883L_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			Serial_SendString("ERROR");
			/*超时的错误处理代码，可以添加到此处*/
			if (I2C1->SR2 & I2C_SR2_BUSY) {
				Serial_SendString("I2C Bus Busy");
			}
			break;										//跳出等待，不等了
		}
	}
}

void HMC5883L_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	I2C_GenerateSTART(I2C1, ENABLE);										//硬件I2C生成起始条件
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C1, HMC5883L_ADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C1, RegAddress);											//硬件I2C发送寄存器地址
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//等待EV8
	
	I2C_SendData(I2C1, Data);												//硬件I2C发送数据
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTOP(I2C1, ENABLE);											//硬件I2C生成终止条件
}

uint8_t HMC5883L_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C1, ENABLE);										//硬件I2C生成起始条件
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C1, HMC5883L_ADDRESS, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C1, RegAddress);											//硬件I2C发送寄存器地址
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);			//等待EV8_2
	
	I2C_GenerateSTART(I2C1, ENABLE);										//硬件I2C生成重复起始条件
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C1, HMC5883L_ADDRESS, I2C_Direction_Receiver);	//硬件I2C发送从机地址，方向为接收
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//等待EV6
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2C1, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	HMC5883L_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);				//等待EV7
	Data = I2C_ReceiveData(I2C1);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}

void HMC5883L_Init(void)
{
	/*I2C初始化*/
	MyI2C_Init();
	
	MPU6050_WriteReg(MPU6050_USER_CTRL, 0x00);							//关闭MPU6050的I2C主模式
	MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02);							//开启MPU6050旁路模式，直接用主机I2C通信			
	
	HMC5883L_WriteReg(HMC_5883L_CFG_A, 0x70);							//初始化配置寄存器A，默认采样数、默认输出速率、默认测量模式
	HMC5883L_WriteReg(HMC_5883L_CFG_B, 0x60);							//初始化配置寄存器B，配置中等增益值
	HMC5883L_WriteReg(HMC_5883L_MODE, 0x00);							//初始化模式寄存器，选择连续测量模式

	//MPU6050_WriteReg(MPU6050_USER_CTRL, 0x02);
	MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x00);							//关闭MPU6050旁路模式
	MPU6050_WriteReg(MPU6050_USER_CTRL, 0x20);							//开启MPU6050的I2C主模式
	
	Delay_ms(30);
  }

/*用 MPU6050 的“主模式（I²C Master）”来读取 HMC5883L*/
void MPU_get_HMCData(float *MagX, float *MagY, float *MagZ){
	MPU6050_WriteReg(MPU6050_SLV0_ADDR,0x80 | (HMC5883L_ADDRESS >> 1)); 	//读HMC5883L
	MPU6050_WriteReg(MPU6050_SLV0_REG,0x03);						//从0x03 数据输出 X MSB 寄存器开始读
	MPU6050_WriteReg(MPU6050_SLV0_CTRL,0x06 | 0x80);				//启动从设备0，读取6个字节
	MPU6050_WriteReg(0x67, 1);
	
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_HMC_XOUT_H);		//读取磁力计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_HMC_XOUT_L);		//读取磁力计X轴的低8位数据
	*MagX = ((DataH << 8) | DataL) / 660.0f;			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_HMC_YOUT_H);		//读取磁力计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_HMC_YOUT_L);		//读取磁力计Y轴的低8位数据
	*MagY = ((DataH << 8) | DataL) / 660.0f;			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_HMC_ZOUT_H);		//读取磁力计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_HMC_ZOUT_L);		//读取磁力计Z轴的低8位数据
	*MagZ = ((DataH << 8) | DataL) / 660.0f;			//数据拼接，通过输出参数返回
}

/*用 MCU 的 I²C 访问 HMC5883L*/
void HMC5883L_GetData(int16_t * magn_x_gs,int16_t *magn_y_gs,int16_t *magn_z_gs)
{
	uint8_t DataH,DataL;
	
	DataH=HMC5883L_ReadReg (0x03);
	DataL=HMC5883L_ReadReg (0x04);
	* magn_x_gs = (DataH<<8)|DataL;
	
	DataH=HMC5883L_ReadReg (0x07);
	DataL=HMC5883L_ReadReg (0x08);
	* magn_y_gs = (DataH<<8)|DataL;
	
	DataH=HMC5883L_ReadReg (0x05);
	DataL=HMC5883L_ReadReg (0x06);
	* magn_z_gs = (DataH<<8)|DataL;
}
