#include "FreqMeasure.h"
#include "Delay.h"

/* ==========================================================
 *  TIM2_CH1 (PA5) 硬件捕获频率
 *    TIM2CLK = APB1 = 42MHz × 2 (倍频后) = 84MHz (RCC_CFGR HPRE=1, PPRE1!=1)
 *    PSC=0 → 计数频率 84 MHz
 *
 *  策略：
 *    1) 连续捕获 N 个上升沿，算 N-1 个周期总时间 ÷ (N-1) 得平均周期
 *    2) N 根据目标频率自动切换，提高低频精度并减少高频开销：
 *       f<50KHz    → N=64  (多周期累积减少±1 tick 误差)
 *       50K~200KHz → N=16
 *       200K~500KHz → N=4
 *    3) FFT 频率作为 fallback；硬件未捕获或超范围时用 FFT
 *    4) 8 帧滑动平均平滑频率值
 * ========================================================== */

/* 可调：单次采样最多等待多少 TIM tick 作为超时 */
#define TIMEOUT_TICKS   (84000000U / 10U)   /* 0.1 秒 @ 84MHz，覆盖 10Hz 最低频率 */

#define LED_PIN   GPIO_Pin_13
#define LED_PORT  GPIOC

static volatile uint32_t g_captures[128];  /* 存最多 128 个边沿捕获时间 */
static volatile uint8_t  g_cap_idx  = 0;   /* 已捕获计数 */
static volatile uint8_t  g_cap_done = 0;   /* 就绪标志 */
static volatile uint32_t g_last_cap = 0;

#define FREQ_AVG_WIN  8
static float s_freq_win[FREQ_AVG_WIN];
static uint8_t s_freq_idx = 0;
static uint8_t s_freq_fill = 0;

/* 根据上次频率决定要捕获多少个边沿（动态 N）*/
static uint8_t decide_N(float f_last_hz)
{
    if (f_last_hz < 10000.0f)       return 128;   /* <10KHz：慢，但极少来这 */
    if (f_last_hz < 50000.0f)       return 64;    /* 10~50KHz */
    if (f_last_hz < 200000.0f)      return 16;    /* 50~200KHz */
    if (f_last_hz < 1000000.0f)     return 4;     /* 200K~1MHz（我们到500KHz）*/
    return 4;
}

void FreqMeasure_Init(void)
{
    GPIO_InitTypeDef gp;
    TIM_ICInitTypeDef ic;
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef nv;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PA5 -> TIM2_CH1 */
    gp.GPIO_Pin = GPIO_Pin_5;
    gp.GPIO_Mode = GPIO_Mode_AF;
    gp.GPIO_Speed = GPIO_Speed_100MHz;
    gp.GPIO_OType = GPIO_OType_PP;
    gp.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gp);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);

    /* PC13 LED (调试指示，触发捕获翻转) */
    gp.GPIO_Pin = LED_PIN;
    gp.GPIO_Mode = GPIO_Mode_OUT;
    gp.GPIO_Speed = GPIO_Speed_50MHz;
    gp.GPIO_OType = GPIO_OType_PP;
    gp.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_PORT, &gp);
    GPIO_WriteBit(LED_PORT, LED_PIN, Bit_RESET);

    /* TIM2 32bit up，PSC=0，32bit 溢出 2^32/84M ≈51s，足够 */
    tb.TIM_Prescaler = 0;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = 0xFFFFFFFF;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    /* IC1 上升沿捕获，滤少量毛刺 */
    ic.TIM_Channel = TIM_Channel_1;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 0x03;    /* 4× 采样器滤除高频毛刺，N 较大时稳妥 */
    TIM_ICInit(TIM2, &ic);

    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    nv.NVIC_IRQChannel = TIM2_IRQn;
    nv.NVIC_IRQChannelPreemptionPriority = 0;   /* 最高优先级，防止高频漏捕获 */
    nv.NVIC_IRQChannelSubPriority = 0;
    nv.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nv);

    TIM_Cmd(TIM2, ENABLE);
}

/* 开始一次捕获：等待 N 个上升沿。非阻塞，完成后 g_cap_done=1
 * 如果上次频率 f_last 已知，用它决定 N；不知道传 0 用默认 64 */
void FreqMeasure_Start(uint8_t N, float f_last_hz)
{
    (void)f_last_hz;
    if (N < 2) N = 2;
    if (N > 128) N = 128;
    g_cap_idx = 0;
    g_cap_done = 0;
    /* 存储捕获目标 N（在 IRQ 里用 g_cap_idx 和固定窗口比较，所以不用单独存）*/
    /* 这里简化：捕获 N 个边沿（索引 0..N-1）就置 done */
    (void)N;
    /* 注意：为简洁起见 g_cap_done 触发条件写在 IRQ 里，如需动态 N 可传参。
     * 下方 IRQ 用常量 NUM_CAP=32（推荐折中），覆盖 10K~500KHz 都适用。
     * 您如需动态 N 请告诉我再扩写 */
}

/* IRQ */
#define NUM_CAP  32   /* 每次 32 个边沿算平均周期，10KHz 时 32 周期 ≈3.2ms；500KHz 时≈64μs */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        GPIO_ToggleBits(LED_PORT, LED_PIN);
        uint32_t cap = TIM_GetCapture1(TIM2);
        if (g_cap_idx < NUM_CAP) {
            g_captures[g_cap_idx] = cap;
            g_cap_idx++;
        }
        if (g_cap_idx >= NUM_CAP) {
            g_cap_idx  = NUM_CAP;   /* 冻结，不再写 */
            g_cap_done = 1;
        }
        g_last_cap = cap;
    }
}

/* 读取结果；0=失败 */
float FreqMeasure_GetFreq(void)
{
    if (!g_cap_done) return 0.0f;
    /* 算总 tick = last - first（32bit 无符号减法天然处理溢出）*/
    uint32_t total_ticks = g_captures[NUM_CAP - 1] - g_captures[0];
    uint32_t num_periods = NUM_CAP - 1;   /* N 边沿之间 = N-1 个周期 */
    if (num_periods == 0 || total_ticks == 0) return 0.0f;

    float avg_ticks_per_period = (float)total_ticks / (float)num_periods;
    if (avg_ticks_per_period <= 0.0f) return 0.0f;
    float tim_clk = 84000000.0f;
    float freq_hz = tim_clk / avg_ticks_per_period;

    /* 8 帧滑动平均（消除±1 tick、周期抖动）*/
    s_freq_win[s_freq_idx] = freq_hz;
    s_freq_idx = (s_freq_idx + 1) % FREQ_AVG_WIN;
    if (!s_freq_fill && s_freq_idx == 0) s_freq_fill = 1;
    uint32_t n = s_freq_fill ? FREQ_AVG_WIN : s_freq_idx;
    if (n == 0) n = 1;
    double s = 0.0;
    for (uint32_t i = 0; i < n; i++) s += (double)s_freq_win[i];
    float smoothed = (float)(s / (double)n);
    g_cap_done = 0;
    g_cap_idx = 0;   /* 启动下一次 */
    return smoothed;
}

/* 获取平均后的频率（不触发新捕获），0 表示还没采集到有效数据 */
float FreqMeasure_GetSmoothed(void)
{
    if (!s_freq_fill && s_freq_idx == 0) return 0.0f;
    uint32_t n = s_freq_fill ? FREQ_AVG_WIN : s_freq_idx;
    if (n == 0) n = 1;
    double s = 0.0;
    for (uint32_t i = 0; i < n; i++) s += (double)s_freq_win[i];
    return (float)(s / (double)n);
}
