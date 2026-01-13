//#include "stm32f4xx.h"                  // Device header
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_gpio.h"
//#include "Delay.h"
//#include "MPU6050.h"
//#include "HMC5883L.h"  
//#include "Serial.h"
//#include "MyI2C.h"
//#include "MPU6050_Reg.h"
//#include "OLED.h"
//#include "OLED.h"
#include "pwm.h"
#include "Delay.h"
#include "PPM.h"
#include "pid/pid.h"
#include "imu.h"

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
int16_t count,count_idle;
float pitch,roll,yaw,yaw_c,yaw_n;
extern int PPM_Data[8];
float gx_degps, gy_degps, gz_degps;
float control[3];
RCC_ClocksTypeDef RCC_Clocks;
float control_lpf[3] = {0,0,0};
float dError;
// 前向声明：目标生成（User/pid/control.c）
void getWantedYPR(float yprRAD[3]);

int main()
{
	OLED_Init();
	Serial_init();//串口通信
	MyI2C_Init ();
	PWM_Init();
	MPU6050_Init();
	GPIO_INIT();
    HMC5883L_Init();
    PPM_Init();
    initbothPID();
//	PWM_SetCompare1(400);
//	Delay_ms(3000);
//	PWM_SetCompare1(450);
//	Delay_ms(3000);
//	PWM_SetCompare1(400);
//	Delay_ms(15000);
//	Serial_SendString("1");
//    Delay_ms(4000);
//	Serial_SendString("2");
    // 电调校准：高位2000us维持4s -> 低位1000us维持2s（按你的要求）
    PWM_SetCompare1(2000);
    PWM_SetCompare2(2000);
    PWM_SetCompare3(2000);
    PWM_SetCompare4(2000);
	while(count < 100){
		count++;
		IMU_update(&pitch, &roll, &yaw);
		OLED_ShowString(1,1,"waiting");
		yaw_c = yaw;
	}
	err_update();
    PWM_SetCompare1(1000);
    PWM_SetCompare2(1000);
    PWM_SetCompare3(1000);
    PWM_SetCompare4(1000);
	while(count < 150){
		count++;
		IMU_update(&pitch, &roll, &yaw);
		OLED_ShowString(1,1,"waiting");
		yaw_c = yaw;
	}
	
    //Delay_ms(2000);
//	while(count < 80){   
//		IMU_update(&pitch, &roll, &yaw);
//		count++;
//		yaw_c = yaw;
//	}
    //PWM_SetCompare1(1200);	
//	PWM_SetCompare2(450);	
//	PWM_SetCompare3(450);	
//	PWM_SetCompare4(450);	
	//Delay_ms(3000);
    //PWM_SetCompare1(1000);	
//	PWM_SetCompare2(0);	
//	PWM_SetCompare3(0);	
//	PWM_SetCompare4(0);	
//	Serial_SendString("}\n");
	//Delay_ms(15000);
//	Serial_SendString("}\n");
    while(1)
	{	RCC_GetClocksFreq(&RCC_Clocks);
        // 1) 姿态（度）
        IMU_update(&pitch, &roll, &yaw);

        // 2) cur 与 g_cur_yaw_rad（弧度）
        float cur[3];
        PID_UpdateCurFromIMU(pitch, roll, yaw_n, cur);

        // 3) 陀螺角速度（度/秒 -> 弧度/秒）
        IMU_get_gyro_degps(&gx_degps, &gy_degps, &gz_degps);
        PID_SetGyroDegps(gx_degps, gy_degps, gz_degps);

        // 4) 控制周期（秒）：与 IMU 的 dt 保持一致（dt = 2*halfT）
        PID_SetDtSeconds(IMU_GetDtSeconds());

        // 5) 生成目标姿态（弧度）
        float tar[3];
        getWantedYPR(tar);
//		//test
//		tar[0] = 30;
//		tar[1] = 30;
//		tar[2] = 0;
		//dError = (1.5*tar[1] - gyroRate[1]-ratePID.lastError[1])*0.25;
        // 6) 油门低位时清积分（假定 ch2 为油门）
        if (g_rc_us[2] < 1100) {
            PID_ResetIntegrators();
        }

        // 7) 两环 PID 输出
        //float control[3];
        PID(tar, cur, control);

        // 8) 四轴X混控：油门 + PID（姿态/角速度修正）
        static float thr_filt = 1000.0f; // 简单一阶低通滤波的状态
        uint16_t ppm_throttle = g_rc_us[2];
        if (ppm_throttle < 1000) ppm_throttle = 1000;
        if (ppm_throttle > 2000) ppm_throttle = 2000;
        // 低通滤波，抑制 PPM 抖动/跳变（alpha 越大越平滑/更慢）
        const float alpha_th = 0.4f;
        thr_filt = alpha_th * thr_filt + (1.0f - alpha_th) * (float)ppm_throttle;
        const float idle_us = 1050.0f; // 怠速，避免进入停转区间
        float base_us = 1350.0f + (thr_filt - 1100.0f) * 0.33f; // 1100..2000 -> 1350..1650
        if (base_us <= 1350) {
			base_us = idle_us + (thr_filt - 1000.0f)*2;
		}
//        const float K_roll_us = 40.0f;  // 可调：滚转修正增益（us）
//        const float K_pitch_us = 40.0f; // 可调：俯仰修正增益（us）
//        const float K_yaw_us = 25.0f;   // 可调：偏航修正增益（us）
		const float K_roll_us = 20.0f;  // 可调：滚转修正增益（us）
        const float K_pitch_us = 20.0f; // 可调：俯仰修正增益（us）
        const float K_yaw_us = 10.0f;   // 可调：偏航修正增益（us）
		
		const float alpha = 0.8f;
		control_lpf[0] = control[0]*alpha + control_lpf[0]*(1-alpha);
		control_lpf[1] = control[1]*alpha + control_lpf[1]*(1-alpha);
		control_lpf[2] = control[2]*alpha + control_lpf[2]*(1-alpha);
        float d_roll  = K_roll_us  * control_lpf[2]; // control: [yaw, pitch, roll]
        float d_pitch = K_pitch_us * control_lpf[1];
        float d_yaw   = K_yaw_us   * control_lpf[0];

//        float m1 = base_us - d_pitch + d_roll - d_yaw; // 前左
//        float m2 = base_us - d_pitch - d_roll + d_yaw; // 前右
//        float m3 = base_us + d_pitch - d_roll - d_yaw; // 后右
//        float m4 = base_us + d_pitch + d_roll + d_yaw; // 后左
        float m1 = base_us - d_pitch + d_roll + d_yaw; // 前左
        float m2 = base_us - d_pitch - d_roll - d_yaw; // 前右
        float m3 = base_us + d_pitch - d_roll + d_yaw; // 后右
        float m4 = base_us + d_pitch + d_roll - d_yaw; // 后左

        if (m1 < idle_us) m1 = idle_us; //if (m1 > 700) m1 = 450;
        if (m2 < idle_us) m2 = idle_us;// if (m2 > 700) m2 = 450;
        if (m3 < idle_us) m3 = idle_us; //if (m3 > 700) m3 = 450;
        if (m4 < idle_us) m4 = idle_us; //if (m4 > 700) m4 = 450;

        // 低油门时保留怠速，不再降到 400，避免“转一下停一下”
        if (g_rc_us[2] < 1050) {
            m1 = m2 = m3 = m4 = idle_us;
			count_idle++;
        }
		else if(g_rc_us[2] == 1500) count_idle++;
		else count_idle = 0;
		if (count_idle>=20) {
            m1 = m2 = m3 = m4 = 1001;
        }
        PWM_SetCompare1((uint16_t)m1);
        PWM_SetCompare2((uint16_t)m2);
        PWM_SetCompare3((uint16_t)m4);
        PWM_SetCompare4((uint16_t)m3);
//        PWM_SetCompare1((uint16_t)m2);
//        PWM_SetCompare2((uint16_t)m3);
//        PWM_SetCompare3((uint16_t)m1);
//        PWM_SetCompare4((uint16_t)m4);
		//串口发送姿态 {"roll": 10.5, "pitch": -5.2, "yaw": 45.0}{"roll": 10.5, "pitch": -5.2, "yaw": 45.0, "target": 100.5, "actual": 98.2, "out": 85.3}
		yaw_n = (yaw-yaw_c);
		//yaw_n = yaw;
		
		
//		Serial_SendString("{\"roll\":");
//		Serial_SendFloatSimple(roll, 1);
//		Serial_SendString(",\"pitch\":");
//		Serial_SendFloatSimple(pitch, 1);
//		Serial_SendString(",\"yaw\":");
//		Serial_SendFloatSimple(yaw_n, 1);
//		Serial_SendString(",\"target\":");
//		//Serial_SendFloatSimple(1.5*tar[1], 2);
//		Serial_SendFloatSimple(tar[1], 1);
//		Serial_SendString(",\"actual\":");
//		Serial_SendFloatSimple(cur[1], 1);
//		//Serial_SendFloatSimple(gy_degps, 2);
//		Serial_SendString(",\"out\":");
//		Serial_SendFloatSimple(control[1], 1);//ppid->intergrator[i]
//		//Serial_SendFloatSimple(ratePID.intergrator[1], 2);
//		//Serial_SendFloatSimple(dError, 2);
//		Serial_SendString("}\n");
		
//		Serial_SendString("1");
//		int16_t m = 400 + (int16_t)yaw;
//		PWM_SetCompare1(400);

		//OLED显示
//		OLED_ShowString(2,1,"Pitch=");
		OLED_ShowNum(1, 10, base_us, 5);
		//OLED_ShowNum(1, 10, g_rc_us[2], 5);//油门
		OLED_ShowNum(2, 1, g_rc_us[1], 5);//俯仰前1000，后2000
		//OLED_ShowNum(2, 10, g_rc_us[0], 5);//偏航左1000，右2000
		OLED_ShowNum(2, 10, g_rc_us[3], 5);//横滚左1000，右2000
		OLED_ShowNum(3, 1, m1, 4);
		OLED_ShowNum(3, 8, m2, 4);
		OLED_ShowNum(4, 8, m3, 4);
		OLED_ShowNum(4, 1, m4, 4);
//		OLED_ShowNum(3, 1, m2, 4);
//		OLED_ShowNum(3, 8, m3, 4);
//		OLED_ShowNum(4, 8, m4, 4);
//		OLED_ShowNum(4, 1, m1, 4);

//		OLED_ShowString(3,1,"Roll=");
//		OLED_ShowString(4,1,"Yaw=");
////	OLED_ShowFNum(2, 8, pitch, 5,2);
//		OLED_ShowFNum(3, 8, roll, 5,2);
//		OLED_ShowFNum(4, 8, yaw-yaw_c, 5,2);
		if(count < 250){
			count++;
			yaw_c = yaw;
		}
		else{
		OLED_ShowString(1,1,"complete");
		}
        //Delay_ms (10);//防止烧
	}		
}
//if(Serial_GetRxData()==1)
//		{
// 	    GH=MPU6050_ReadReg(MPU6050_PWR_MGMT_1);
//        MPU6050_GetData (&AX,&AY,&AZ,&GX,&GY,&GZ);
//		
//		
//			// 发送陀螺仪数据
//				Serial_SendNumber(GX, 2);
//				Serial_SendString("\n");
//				Serial_SendNumber(GY, 2);
//				Serial_SendString("\n");
//				Serial_SendNumber(GZ, 2);
//				Serial_SendString("\n");

//				
//				// 发送加速度数据
//				Serial_SendNumber(AX, 2);
//				Serial_SendString("\n");
//				Serial_SendNumber(AY, 2);
//				Serial_SendString("\n");
//				Serial_SendNumber(AZ, 2);
//				Serial_SendString("\n");
//				Delay_ms (100);//防止烧

//		}
//		
//	if(Serial_GetRxData()==2)
//			{
//		MPU_get_HMC();
//				HMC5883L_GetData(&MX,&MY,&MZ);

//				
//				// 发送磁力计数据
//				Serial_SendNumber(MX, 2);
//				Serial_SendString("\n");
//				Serial_SendNumber(MY, 2);
//				Serial_SendString("\n");
//				Serial_SendNumber(MZ, 2);
//				Serial_SendString("\n");


//		Delay_ms (10);//防止烧
//			}
//		OLED_ShowSignedNum(2, 1, AX, 5);
//		OLED_ShowSignedNum(3, 1, AY, 5);
//		OLED_ShowSignedNum(4, 1, AZ, 5);
//		OLED_ShowSignedNum(2, 8, GX, 5);
//		OLED_ShowSignedNum(3, 8, GY, 5);
//		OLED_ShowSignedNum(4, 8, GZ, 5);
//		OLED_ShowSignedNum(1, 1, MX, 5);
//		OLED_ShowSignedNum(1, 8, MY, 5);
//		
//	
//	
	


