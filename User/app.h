#ifndef _APP_H_ 
#define _APP_H_ 

/**************** 用户任务声明*******************/
#include "stm32f4xx.h"     
#define PPM_BUFFER_SIZE 5

void Task_Start(void *p_arg);
void Task_PWM(void *p_arg); 
void Task_SendInfo(void *p_arg); 
void Task_GetInfo(void *p_arg);
extern int finish;

extern uint16_t ppm_buffer[PPM_BUFFER_SIZE]; // ?? PPM_Data[0]
extern uint8_t ppm_head; // ????
extern uint8_t ppm_tail; // ????
extern uint8_t ppm_count; // ?????????


#endif //_APP_H_
