#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "OS.h"
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

void Serial_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //PA9 复用为 USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);//PA10 复用为 USART1
	
	GPIO_InitTypeDef GPIO_InitSturcture;
	GPIO_InitSturcture.GPIO_Mode = GPIO_Mode_AF;//复用
	GPIO_InitSturcture.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitSturcture.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitSturcture.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitSturcture.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitSturcture);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	 
	
	USART_InitTypeDef USART1_InitStructure;
	USART1_InitStructure.USART_BaudRate = 128000;//波特率
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//选择流控
	USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//选择功能，发送or接收
	USART1_InitStructure.USART_Parity = USART_Parity_No;//校验位
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;//停止位
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;//校验就选9，非校验就选8
	USART_Init(USART1,&USART1_InitStructure);
	
	
	

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStrcture;
	NVIC_InitStrcture.NVIC_IRQChannel = USART1_IRQn;//中断通道
	NVIC_InitStrcture.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStrcture.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStrcture.NVIC_IRQChannelSubPriority =1;
	NVIC_Init(&NVIC_InitStrcture);
	USART_Cmd(USART1,ENABLE);
}
//发送字节，uint8_t类型
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

//发送数组
void Serial_SendArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	for(i = 0;i < Length;i++)
	{
		Serial_SendByte(Array[i]);
	}
}
//发送字符串
void Serial_SendString(char *String)
{
	uint8_t i;
	for(i = 0;String[i] != '\0';i++)
	{
		Serial_SendByte(String[i]);
	}
}
//x^y计算
uint32_t Serial_Pow(uint32_t x,uint32_t y)
{
	uint32_t result = 1;
	while(y--)
	{
		result *=x;
	}
	return result;
}
//发送数字，参数分别是数字及其长度
void Serial_SendNumber(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i = 0;i < Length;i++)
	{
		Serial_SendByte(Number/Serial_Pow(10,Length - i - 1)%10 + 0x30);
	}
}


uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}

void USART1_IRQHandler(void)
{
    OSIntEnter();
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        Serial_RxData = USART_ReceiveData(USART1);
		if(OSPrioCur==TASK_GetInfo_PRIO){
		OSTaskSuspend (TASK_GetInfo_PRIO);
			
		}
		else{
		OSTaskResume (TASK_GetInfo_PRIO);
		
		}
        Serial_RxFlag = 1;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    OSIntExit();
}
