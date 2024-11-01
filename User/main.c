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

void GPIO_INIT(void)
{
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init (GPIOA,&GPIO_InitStructure);
}
int16_t AX,AY,AZ,GX,GY,GZ,GH;
int16_t  X , Y , Z,GaX;
int32_t Pressure;
int16_t MX,MY,MZ;
int main()
{
	OLED_Init();
	Serial_init();//串口通信
	MyI2C_Init ();
	MPU6050_Init();
	GPIO_INIT();
	HMC5883L_Init();
	
	while(1)
	{
		if(Serial_GetRxData()==1)
		{
 	    GH=MPU6050_ReadReg(MPU6050_PWR_MGMT_1);
        MPU6050_GetData (&AX,&AY,&AZ,&GX,&GY,&GZ);
		
		
			// 发送陀螺仪数据
				Serial_SendNumber(GX, 2);
				Serial_SendString("\n");
				Serial_SendNumber(GY, 2);
				Serial_SendString("\n");
				Serial_SendNumber(GZ, 2);
				Serial_SendString("\n");

				
				// 发送加速度数据
				Serial_SendNumber(AX, 2);
				Serial_SendString("\n");
				Serial_SendNumber(AY, 2);
				Serial_SendString("\n");
				Serial_SendNumber(AZ, 2);
				Serial_SendString("\n");
				Delay_ms (100);//防止烧

		}
		
	if(Serial_GetRxData()==2)
			{
		MPU_get_HMC();
				HMC5883L_GetData(&MX,&MY,&MZ);

				
				// 发送磁力计数据
				Serial_SendNumber(MX, 2);
				Serial_SendString("\n");
				Serial_SendNumber(MY, 2);
				Serial_SendString("\n");
				Serial_SendNumber(MZ, 2);
				Serial_SendString("\n");


		Delay_ms (10);//防止烧
			}
		OLED_ShowSignedNum(2, 1, AX, 5);
		OLED_ShowSignedNum(3, 1, AY, 5);
		OLED_ShowSignedNum(4, 1, AZ, 5);
		OLED_ShowSignedNum(2, 8, GX, 5);
		OLED_ShowSignedNum(3, 8, GY, 5);
		OLED_ShowSignedNum(4, 8, GZ, 5);
		OLED_ShowSignedNum(1, 1, MX, 5);
		OLED_ShowSignedNum(1, 8, MY, 5);
		
	}
			
}


