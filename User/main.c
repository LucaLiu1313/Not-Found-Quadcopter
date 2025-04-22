#include "stm32.h"                  // Device heade 
#include "hardwareInit.h"
#include "OS.h"

static OS_STK startup_task_stk[APP_CFG_STARTUP_TASK_STK_SIZE];

int main()
{
	hardware_Init();
	OSInit(); 
	OSTaskCreate(Task_Start,(void *)0, 
		&startup_task_stk[APP_CFG_STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO); 
	OSStart(); 
	return 0; 
		
}
