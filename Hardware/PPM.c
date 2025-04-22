#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "OS.h"

void PPM_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    TIM_TimeBaseInitTypeDef TimeBase_InitStructure;
    TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TimeBase_InitStructure.TIM_Prescaler = 84 - 1;        //PSC
    TimeBase_InitStructure.TIM_Period = 65535;        //ARR
    TimeBase_InitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TimeBase_InitStructure);
    
    TIM_ICInitTypeDef IC_InitStructure;
    IC_InitStructure.TIM_Channel = TIM_Channel_2;    //选择CH2
    IC_InitStructure.TIM_ICFilter = 0xF;    //滤波
    IC_InitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;        //上升沿触发
    IC_InitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;        //捕获单元预分频
    IC_InitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;    //直连
    TIM_ICInit(TIM2, &IC_InitStructure);
    
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2 );    //触发源选择TI2FP2
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //从模式选择复位计数器
    
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);        //开启TIM2输入捕获中断
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitTypeDef NVIC_InitStrcuture;
    NVIC_InitStrcuture.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStrcuture.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStrcuture.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStrcuture.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStrcuture);
    
    TIM_Cmd(TIM2, ENABLE);        
}

int Time=0;
int finish=0;
int ch=0;
int PPM_Data[8];
int past;
int delta;

//void TIM2_IRQHandler(void)
//{    
//	//完美规避第一次没有CCR的情况
//	if((TIM2->SR & 4)!=0)//TIMx_CCR1 寄存器中已捕获到计数器值
//	{
//		Time=TIM_GetCapture2(TIM2);
////		if(Time>=4000){
////		finish=0;
////		}
//		//Time= TIM2->CCR2
//	if((Time<= 2200)){
//		finish = 1;
//		}		
//		if(finish==1)
//			{
//		    past=PPM_Data[ch];
//			PPM_Data[ch]=Time;
//			delta=Time-past;
//			ch++;
//			
//			if(ch>7){
//			finish=0;
//			ch=0;
//			}
//			}
////			if(Time>4000)//识别到帧尾
////			{
////				finish=1;
////				ch=0;
////			}    
////似乎不需要判断	
//    }
//	if(delta>10||delta<-10){
//	OSIntEnter();	
//	OSTaskResume (TASK_Motor_PRIO);
//	TIM2->SR &= ~(0x04); //清除标志位
//	OSIntExit();
//	}
//	
//}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & 4) // TIMx_CCR1 已捕获
    {
        Time = TIM_GetCapture2(TIM2);
        if (Time <= 2200)
        {
            finish = 1;
        }
        if (finish == 1)
        {
            PPM_Data[ch] = Time;
            ch++;
            if (ch > 7)
            {
                finish = 0;
                ch = 0;
                // 将 PPM_Data[0] 存入缓冲区
                ppm_buffer[ppm_head] = PPM_Data[0];
                ppm_head = (ppm_head + 1) % PPM_BUFFER_SIZE; // 更新写入位置
                if (ppm_count < PPM_BUFFER_SIZE) {
                    ppm_count++; // 增加计数
                } else {
                    ppm_tail = (ppm_tail + 1) % PPM_BUFFER_SIZE; // 缓冲区满，覆盖最早数据
                }
            }
        }
        TIM2->SR &= ~(0x04); // 清除标志
    }
}