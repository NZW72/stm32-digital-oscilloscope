#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void Delay_Init(u8 SYSCLK);
void Delay_us(u32 nus);
void Delay_ms(u16 nms);

/* 非阻塞毫秒计数：供 HMI 节流/去重使用 */
extern volatile uint32_t uwTick;
void Delay_TickInit(void);          /* 启动 TIM6 1ms 中断 */
uint32_t Delay_GetTick(void);       /* 获取当前 ms */
void Delay_SetTick(uint32_t t);     /* 设置当前 ms（调试用）*/

#endif
