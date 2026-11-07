#include "TJC_HMI_transport.h"

/* -------------------------------------------------------------- */
/* 内部状态                                                        */
/* -------------------------------------------------------------- */
static TJC_RB_t           s_rb;
static TJC_HexLogger_t    s_log = 0;

static const uint8_t s_footer[3] = {0xFF, 0xFF, 0xFF};

/* -------------------------------------------------------------- */
/* 日志分发（统一标签 TX/RX）                                      */
/* -------------------------------------------------------------- */
static void log_hex(const char *tag, const uint8_t *d, uint16_t n)
{
    if (s_log && d && n) s_log(tag, d, n);
}

/* -------------------------------------------------------------- */
/* GPIO / USART / NVIC Init                                        */
/*   USART2: PA2=TX(AF7) -> 屏 RX                                  */
/*           PA3=RX(AF7) <- 屏 TX                                  */
/*   APB1 42MHz 时钟，波特率发生器分频不同但 StdPeriph 自动处理      */
/* -------------------------------------------------------------- */
void TJC_Transport_Init(uint32_t baud)
{
    GPIO_InitTypeDef        GPIO_InitStruct;
    USART_InitTypeDef       USART_InitStruct;
    NVIC_InitTypeDef        NVIC_InitStruct;

    /* 1. 时钟：GPIOA 在 AHB1，USART2 在 APB1 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 2. PA2=TX(AF7 PushPull), PA3=RX(AF7 PullUp) */
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* 3. UART 8N1 */
    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* 4. NVIC: 接收中断优先级稍低于 DMA/ADC，不抢采样关键路径 */
    NVIC_InitStruct.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_Cmd(USART2, ENABLE);

    /* 5. Ring buffer 清零 */
    s_rb.wi = 0;
    s_rb.ri = 0;
    s_rb.overflow_cnt = 0;
}

void TJC_Transport_SetLogger(TJC_HexLogger_t fn) { s_log = fn; }

/* -------------------------------------------------------------- */
/* TX                                                              */
/* -------------------------------------------------------------- */
void TJC_TxByte(uint8_t b)
{
    /* 阻塞等待 TXE（或超时保护）*/
    uint32_t tmo = 800000UL;     /* 115200 下 ~100ms */
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
        if (--tmo == 0) break;
    }
    USART_SendData(USART2, (uint16_t)b);
    log_hex("TX", &b, 1);
}

void TJC_TxRaw(const uint8_t *data, uint16_t len)
{
    if (!data || !len) return;
    for (uint16_t i = 0; i < len; i++) {
        TJC_TxByte(data[i]);
    }
}

void TJC_TxFrame(const uint8_t *data, uint16_t len)
{
    TJC_TxRaw(data, len);
    TJC_TxRaw(s_footer, 3);
}

/* -------------------------------------------------------------- */
/* RX Ring Buffer （ISR 推，主循环拉）                              */
/* -------------------------------------------------------------- */
uint16_t TJC_RB_Avail(void)
{
    uint16_t wi = s_rb.wi, ri = s_rb.ri;
    return (uint16_t)((wi + TJC_HMI_RB_SIZE - ri) % TJC_HMI_RB_SIZE);
}

void TJC_RB_Clear(void)
{
    __disable_irq();
    s_rb.wi = s_rb.ri = 0;
    __enable_irq();
}

uint32_t TJC_RB_OverflowCount(void) { return s_rb.overflow_cnt; }

uint16_t TJC_RB_Read(uint8_t *out, uint16_t want)
{
    uint16_t got = 0;
    if (!out || !want) return 0;
    __disable_irq();
    while (got < want && s_rb.ri != s_rb.wi) {
        out[got++] = s_rb.buf[s_rb.ri];
        s_rb.ri = (uint16_t)((s_rb.ri + 1) % TJC_HMI_RB_SIZE);
    }
    __enable_irq();
    if (got) log_hex("RX", out, got);
    return got;
}

/* -------------------------------------------------------------- */
/* IRQ Handler：startup 向量表里的名字是 USART2_IRQHandler         */
/* -------------------------------------------------------------- */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint16_t b = USART_ReceiveData(USART2) & 0xFF;
        uint16_t next = (uint16_t)((s_rb.wi + 1) % TJC_HMI_RB_SIZE);
        if (next == s_rb.ri) {
            /* 满了：丢最旧 1 字节（滑动），计数溢出 */
            s_rb.ri = (uint16_t)((s_rb.ri + 1) % TJC_HMI_RB_SIZE);
            s_rb.overflow_cnt++;
        }
        s_rb.buf[s_rb.wi] = (uint8_t)b;
        s_rb.wi = next;
        /* 注意：这里不调用 log_hex，避免 ISR 里重/长耗时 */
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    /* ORE / FE / NE / PE 错误：简单清标志，parser 会自然重同步 */
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE | USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE) != RESET) {
        /* 读 DR + 清 SR 即可 */
        (void)USART_ReceiveData(USART2);
    }
}
