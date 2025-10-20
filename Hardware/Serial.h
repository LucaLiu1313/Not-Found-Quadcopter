#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

void Serial_init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number,uint8_t Length);
void Serial_SendFloat(float number, uint8_t integerLength, uint8_t decimalLength);
void Serial_SendFloatSimple(float number, uint8_t decimalLength);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);
void USART1_IRQHandler(void);

#endif
