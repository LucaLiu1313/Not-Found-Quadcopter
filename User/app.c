#include "stm32.h" 
#include "hardware.h"
#include "OS.h"
#include "app.h"
#include "imu.h"

extern int PPM_Data[8];
extern uint8_t Serial_RxData;

int16_t AX,AY,AZ,GX,GY,GZ;
float MX,MY,MZ;

OS_STK task_motor_stk[APP_CFG_STARTUP_TASK_STK_SIZE]; //定义栈
OS_STK task_sendInfo_stk[APP_CFG_STARTUP_TASK_STK_SIZE]; //定义栈
OS_STK task_getInfo_stk[APP_CFG_STARTUP_TASK_STK_SIZE]; //定义栈

void Task_motor(void *p_arg);
void Task_SendInfo(void *p_arg);
void Task_GetInfo(void *p_arg);
 
 //主任务
void Task_Start(void *p_arg) 
{ 
	(void)p_arg; // 'p_arg' 并没有用到,防止编译器提示警告
	Serial_RxData=8;
	OSTaskCreate(Task_motor,(void *)0, //创建任务 2 
		&task_motor_stk[APP_CFG_STARTUP_TASK_STK_SIZE-1], TASK_Motor_PRIO); 

	OSTaskCreate(Task_SendInfo,(void *)0, //创建任务 3 
		&task_sendInfo_stk[APP_CFG_STARTUP_TASK_STK_SIZE-1], TASK_SendInfo_PRIO); 
	 
	OSTaskCreate(Task_GetInfo,(void *)0, //创建任务 4 
		&task_getInfo_stk[APP_CFG_STARTUP_TASK_STK_SIZE-1], TASK_GetInfo_PRIO);

	//Serial_SendString("TASK_START RUNNING");
	OSTaskSuspend (STARTUP_TASK_PRIO);
 
} 

 //驱动任务
//void Task_motor(void *p_arg) 
//{    
//	(void)p_arg;
//	while(1)
//	{
//		//Serial_SendString("TASK_PWM RUNNING");
//		
//		Serial_SendNumber(5,1);
//		//Serial_SendNumber(Serial_RxData,2);
//		 //OSTaskSuspend ( TASK_Motor_PRIO);
//	    //Serial_SendString(5,1);
//		OSTaskSuspend (TASK_Motor_PRIO);
//		PWM_SetCompare1(PPM_Data[0]);
//		
//// 		OLED_ShowString(1, 3, "HelloWorld!");//显示字符串(1,3,"HelloWorld!")
////		
////		OLED_ShowNum(1, 8, PPM_Data[0], 5);
////	    OLED_ShowNum(2, 8, PPM_Data[1], 5);
////	    OLED_ShowNum(3, 8, PPM_Data[2], 5);
////	    OLED_ShowNum(4, 8, PPM_Data[3], 5);
////	    OLED_ShowNum(1, 2, PPM_Data[4], 5);
////	    OLED_ShowNum(2, 2, PPM_Data[5], 5);
////	    OLED_ShowNum(3, 2, PPM_Data[6], 5);
////	    OLED_ShowNum(4, 2, PPM_Data[7], 5);	
//		OSTimeDly(OS_TICKS_PER_SEC / 10);
//	}
//}


void Task_motor(void *p_arg) 
{    
    (void)p_arg;
    while (1)
    {
        uint16_t ppm_value = 0;
        OS_CPU_SR cpu_sr;
        OS_ENTER_CRITICAL();
        if (ppm_count > 0) { // ??????
            // ????????(head ?????????,????????)
            uint8_t last_index = (ppm_head == 0) ? (PPM_BUFFER_SIZE - 1) : (ppm_head - 1);
            ppm_value = ppm_buffer[last_index];
            // ??:?????,???????
            ppm_tail = ppm_head;
            ppm_count = 0;
        }
        OS_EXIT_CRITICAL();

        if (ppm_count > 0 || finish == 1) { // ????(??????? finish ????)
            PWM_SetCompare1(ppm_value);
					  PWM_SetCompare2(ppm_value);
						PWM_SetCompare3(ppm_value);
						PWM_SetCompare4(ppm_value);
					Serial_SendNumber(5, 1);
        }

        OSTimeDly(OS_TICKS_PER_SEC / 10); // ?? 100ms
    }
}


 //发信任务

void Task_SendInfo(void *p_arg) 
{ 
	(void)p_arg;
//	char buffer[128];// using buffer to avoid data confusing
	float Pitch, Roll, Yaw;
	
	while(1)
	{
		Serial_SendNumber(7,1);
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		MPU_get_HMCData(&MX, &MY, &MZ);
		
		IMU_update(&Roll,&Pitch,&Yaw);
		
		OLED_ShowString(2,1,"Pitch=");
		OLED_ShowString(3,1,"Roll=");
		OLED_ShowString(4,1,"Yaw=");
		OLED_ShowFNum(2, 8, Pitch, 5,2);
		OLED_ShowFNum(3, 8, Roll, 5,2);
		OLED_ShowFNum(4, 8, Yaw, 5,2);
		
		Serial_SendString("{\"roll\":");
		Serial_SendFloatSimple(Roll, 1);
		Serial_SendString(",\"pitch\":");
		Serial_SendFloatSimple(Pitch, 1);
		Serial_SendString(",\"yaw\":");
		Serial_SendFloatSimple(Yaw, 1);
		Serial_SendString("}\n");
		
//		OLED_ShowHexNum(2, 1, AX, 5);
//		OLED_ShowHexNum(3, 1, AY, 5);
//		OLED_ShowHexNum(4, 1, AZ, 5);
//		OLED_ShowHexNum(2, 8, GX, 5);
//		OLED_ShowHexNum(3, 8, GY, 5);
//		OLED_ShowHexNum(4, 8, GZ, 5);
//		OLED_ShowHexNum(1, 1, MX, 5);
//		OLED_ShowHexNum(1, 8, MY, 5);
		OSTimeDly(OS_TICKS_PER_SEC / 10);
	}
}
 //收信任务
void Task_GetInfo(void *p_arg) 
{ 
	(void)p_arg;
	while(1)
	{
		Serial_SendNumber(6,1);
		OSTimeDly(OS_TICKS_PER_SEC / 10); // 延时 100ms
		//Delay_ms(100);
		//OSTimeDlyHMSM(0, 10, 0, 0);
	}
}
