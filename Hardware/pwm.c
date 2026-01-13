#include "stm32f4xx.h"                  // Device header
 
void PWM_Init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	//??????,??????????????,
	//???????,????????,????????????,?TIM2_CH
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	
	GPIO_InitTypeDef GPIO_InitStruct1;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct1.GPIO_OType = GPIO_OType_PP; //????
	GPIO_InitStruct1.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStruct1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct1);
	
	GPIO_InitTypeDef GPIO_InitStruct2;
	GPIO_InitStruct2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct2.GPIO_OType = GPIO_OType_PP; //????
	GPIO_InitStruct2.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct2.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStruct2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct2);
	
	
	TIM_InternalClockConfig(TIM3);//??????
	//???????
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 5000-1;	//ARR??,????? 5000-1
	TIM_TimeBaseInitStruct.TIM_Prescaler = 84-1; //PSC???? 84-1
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;//?????
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);//???????
	
	TIM_OCInitTypeDef TIM_OCInitStruct;

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; //?????????PWM1??
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;//????????
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;//????????
	TIM_OCInitStruct.TIM_Pulse = 0;//??CCR???????
	
	 // ?? TIM2_CH1 (PA0)
  TIM_OC1Init(TIM3, &TIM_OCInitStruct);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // ?? TIM2_CH2 (PA1)
  TIM_OC2Init(TIM3, &TIM_OCInitStruct);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // ?? TIM2_CH3 (PA2)
  TIM_OC3Init(TIM3, &TIM_OCInitStruct);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // ?? TIM2_CH3 (PA3)
  TIM_OC4Init(TIM3, &TIM_OCInitStruct);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	
	
	TIM_Cmd(TIM3,ENABLE);//?????
		
}
 
void PWM_SetCompare1(uint32_t ccr) {
    TIM_SetCompare1(TIM3, ccr); // ?? CH1 (PA0) ????
}

void PWM_SetCompare2(uint32_t ccr) {
    TIM_SetCompare2(TIM3, ccr); // ?? CH2 (PA1) ????
}

void PWM_SetCompare3(uint32_t ccr) {
    TIM_SetCompare3(TIM3, ccr); // ?? CH3 (PA2) ????
}

void PWM_SetCompare4(uint32_t ccr) {
    TIM_SetCompare4(TIM3, ccr); // ?? CH4 (PA3) ????
}
