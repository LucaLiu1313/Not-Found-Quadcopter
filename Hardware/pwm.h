#ifndef PWM_H
#define PWM_H
#include <stdint.h>
 
void PWM_Init(void);//PWM???
void PWM_SetCompare1(uint32_t ccr);//??CCR??
void PWM_SetCompare2(uint32_t ccr);
void PWM_SetCompare3(uint32_t ccr);
void PWM_SetCompare4(uint32_t ccr);

	
#endif
