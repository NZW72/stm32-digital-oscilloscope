#ifndef __TJC_HMI_TRANSPORT_H
#define __TJC_HMI_TRANSPORT_H

#include "stm32f4xx.h"

/* ================================================================
 * TJC HMI Transport Layer (Layer 1)
 * 负责：UART 物理初始化、阻塞/非阻塞字节发送、接收环形缓冲、
 *       十六进制日志钩子、透传模式原始字节流发送。
 * 引脚：USART2
 *   TX = PA2  AF7  -> 屏 RX
 *   RX = PA3  AF7  <- 屏 TX
 * 电平：3.3V TTL；GND 必须共地；屏 5V 独立供电
 * ================================================================ */

#ifndef TJC_HMI_RB_SIZE
#define TJC_HMI_RB_SIZE     1024    /* 接收环形缓冲，帧长峰值保护 */
#endif

#ifndef TJC_HMI_TXB_SIZE
#define TJC_HMI_TXB_SIZE    4096    /* 发送缓冲，批量 addt 1024 点安全 */
#endif

typedef struct
{
    uint8_t  buf[TJC_HMI_RB_SIZE];
    volatile uint16_t wi;
    volatile uint16_t ri;
    volatile uint32_t overflow_cnt;
} TJC_RB_t;

/* 日志钩子：每发送/接收一个字节序列可记录，NULL 则关闭 */
typedef void (*TJC_HexLogger_t)(const char *tag, const uint8_t *data, uint16_t len);

void       TJC_Transport_Init(uint32_t baud);         /* 115200 建议 */
void       TJC_Transport_SetLogger(TJC_HexLogger_t fn);

/* ---------- 发送：原始字节 ---------- */
void       TJC_TxByte(uint8_t b);                     /* 单字节阻塞发送 */
void       TJC_TxRaw (const uint8_t *data, uint16_t len); /* 多字节阻塞 */

/* ---------- 发送：带结束符 FF FF FF ---------- */
void       TJC_TxFrame(const uint8_t *data, uint16_t len); /* data + 3*FF */

/* ---------- 接收：环形缓冲（非阻塞，ISR 写）---------- */
uint16_t   TJC_RB_Read (uint8_t *out, uint16_t want); /* 返回实际读到 */
uint16_t   TJC_RB_Avail(void);
void       TJC_RB_Clear(void);
uint32_t   TJC_RB_OverflowCount(void);

/* USART2_IRQHandler 已在 transport.c 中直接定义，stm32f4xx_it.c 不要再重定义 */

#endif
