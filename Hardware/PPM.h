#ifndef __PPM_H
#define __PPM_H

#include "ucos_ii.h"

#define PPM_BUFFER_SIZE 5

void PPM_Init(void);
void TIM2_IRQHandler(void);
extern OS_EVENT *PPMSem;

extern uint16_t ppm_buffer[PPM_BUFFER_SIZE]; // ?? PPM_Data[0]
extern uint8_t ppm_head; // ????
extern uint8_t ppm_tail; // ????
extern uint8_t ppm_count; // ?????????

#endif
