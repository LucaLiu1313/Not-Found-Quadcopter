#ifndef __OLED_H
#define __OLED_H

void OLED_Init(void);//初始化,共四行十六列
void OLED_Clear(void);//清屏
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);//显示一个字符(1,1,'A')
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);//显示字符串(1,3,"HelloWorld!")
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);//显示十进制数字(2,1,12345,5)
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);//显示有符号十进制数字(2,7,-66,2)
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);//显示十六进制数字(3,1,0xAA55,4)
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);//显示二进制数字(4,1,0xAA55,16)

#endif
