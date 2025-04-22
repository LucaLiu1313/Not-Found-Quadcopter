#include "hardware.h"
#include "stm32.h"

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


void hardware_Init(void){
OLED_Init();
Serial_init();
MyI2C_Init ();
MPU6050_Init();
GPIO_INIT();
HMC5883L_Init();
PWM_Init();
PPM_Init();
//”Õ√≈–£◊º	
//PWM_SetCompare1(700);
//Delay_ms(4000);
//PWM_SetCompare1(400);
//Delay_ms(2000);
//PWM_SetCompare1(600);
	
}
