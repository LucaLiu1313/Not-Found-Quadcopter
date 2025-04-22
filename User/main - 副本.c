#include "stm32.h"                  // Device heade
#include "hardware.h"  
#include "hardwareInit.h"
int main()
{
	

	hardware_Init();
	

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


