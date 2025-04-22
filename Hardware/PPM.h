#ifndef __PPM_H
#define __PPM_H

#include "ucos_ii.h"

void PPM_Init(void);
void TIM2_IRQHandler(void);
extern OS_EVENT *PPMSem;
#endif
