#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t Line);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

// 浮点显示
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float dat, uint8_t decimal);

// 电压显示专用（格式: x.xxxV，保留3位小数）
void OLED_ShowVoltage(uint8_t Line, uint8_t Column, float voltage);

// 峰峰值显示专用（输入mV，格式 xxxx mV，可显示0~3300mV）
void OLED_ShowVpp_mV(uint8_t Line, uint8_t Column, float vpp_mv);

// 频率显示专用（输入Hz，格式 xxxxx Hz，可显示0~99999Hz）
void OLED_ShowFreq_Hz(uint8_t Line, uint8_t Column, float freq_hz);

#endif
