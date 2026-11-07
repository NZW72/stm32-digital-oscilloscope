#include "Delay.h"

static u32 fac_us=0;
static u32 fac_ms=0;

#define SYSTICK_LOAD_MAX    0xFFFFFFUL

void Delay_Init(u8 SYSCLK)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    fac_us=SYSCLK/8;
    fac_ms=fac_us*1000;
}

void Delay_us(u32 nus)
{
    u32 temp;
    u32 total_ticks = nus * fac_us;
    
    while(total_ticks > SYSTICK_LOAD_MAX)
    {
        SysTick->LOAD = SYSTICK_LOAD_MAX;
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do
        {
            temp = SysTick->CTRL;
        }while((temp & 0x01) && !(temp & (1 << 16)));
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0X00;
        total_ticks -= SYSTICK_LOAD_MAX;
    }
    
    if(total_ticks > 0)
    {
        SysTick->LOAD = total_ticks;
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do
        {
            temp = SysTick->CTRL;
        }while((temp & 0x01) && !(temp & (1 << 16)));
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0X00;
    }
}

void Delay_ms(u16 nms)
{
    u32 temp;
    u32 total_ticks = (u32)nms * fac_ms;
    
    while(total_ticks > SYSTICK_LOAD_MAX)
    {
        SysTick->LOAD = SYSTICK_LOAD_MAX;
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do
        {
            temp = SysTick->CTRL;
        }while((temp & 0x01) && !(temp & (1 << 16)));
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0X00;
        total_ticks -= SYSTICK_LOAD_MAX;
    }
    
    if(total_ticks > 0)
    {
        SysTick->LOAD = total_ticks;
        SysTick->VAL = 0x00;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        do
        {
            temp = SysTick->CTRL;
        }while((temp & 0x01) && !(temp & (1 << 16)));
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->VAL = 0X00;
    }
}

/* ================================================================
 * 独立毫秒计数：TIM6 1ms 中断
 * 与 Delay_ms 的 SysTick 完全独立，互不干扰
 * 48MHz APB1 时钟，预分频 48-1，周期 1000-1 = 1ms
 * ================================================================ */
volatile uint32_t uwTick = 0;

void Delay_TickInit(void)
{
    uwTick = 0;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_InitStruct.TIM_Prescaler = 48 - 1;       /* 48MHz / 48 = 1MHz */
    TIM_InitStruct.TIM_Period    = 1000 - 1;     /* 1MHz / 1000 = 1kHz = 1ms */
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_InitStruct);

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM6, ENABLE);
}

/* TIM6 更新中断（在 stm32f4xx_it.c 调用此函数）*/
void TIM6_Update_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        uwTick++;
    }
}

uint32_t Delay_GetTick(void)
{
    return uwTick;
}

void Delay_SetTick(uint32_t t)
{
    uwTick = t;
}
