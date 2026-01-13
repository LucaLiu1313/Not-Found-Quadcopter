#ifndef __PPM_H
#define __PPM_H

#include <stdint.h>

void PPM_Init(void);
void TIM2_IRQHandler(void);

// 对外导出的通道脉宽（微秒），范围建议限幅到 [900,2100]
extern volatile uint16_t g_rc_us[8];

#endif
