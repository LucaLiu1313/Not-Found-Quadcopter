#ifndef _APP_H_ 
#define _APP_H_ 

/**************** 用户任务声明*******************/
void Task_Start(void *p_arg);
void Task_PWM(void *p_arg); 
void Task_SendInfo(void *p_arg); 
void Task_GetInfo(void *p_arg);
extern int finish;

#endif //_APP_H_
