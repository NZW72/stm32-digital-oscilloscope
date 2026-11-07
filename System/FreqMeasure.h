#ifndef __FREQMEASURE_H
#define __FREQMEASURE_H

#include "stm32f4xx.h"

void FreqMeasure_Init(void);
float FreqMeasure_GetFreq(void);          /* 读并清捕获结果（有新数据时非0） */
float FreqMeasure_GetSmoothed(void);      /* 读平均频率，不触发新捕获 */
void  FreqMeasure_Start(uint8_t N, float f_last_hz);

#endif
