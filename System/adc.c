#include "adc.h"
#include "delay.h"
#include <math.h>
#include <string.h>

// ============================================================
//  Vpp 高精度采集（目标 <4mV 误差 + 显示稳定）
//
//  核心处理链：
//    ADC(DMA, 1024点) → 拷贝到adc_copy
//      → 64重过采样平均 → 16个"高有效位"采样点 ovs_buffer[16]
//      → ovs上取max-min → Vpp_raw_code
//      → Vpp_raw_code * 3300/4096 → Vpp_raw_mV
//      → 8点 SMA(简单滑动平均) → 16点EMA(指数平均)
//      → ADC_GetVppVoltage_Smooth() 输出稳定读数
// ============================================================

volatile uint8_t  adc_new_data = 0;
uint16_t adc_buffer[ADC_BUFFER_SIZE] = {0};
static uint16_t adc_copy[ADC_BUFFER_SIZE];
volatile uint32_t dma_irq_count = 0;

static uint16_t ovs_buffer[OVS_BUFFER_SIZE];   /* 64x平均后的 16 个高有效采样点 */

/* Vpp 平滑状态 */
static float    s_sma_buf[VPP_SMA_WIN];       /* SMA简单滑动平均窗口 */
static uint8_t  s_sma_idx;
static uint8_t  s_sma_filled;
static float    s_ema_val;                     /* EMA最终平滑值 */
static uint8_t  s_ema_inited;

/* 直流平均（mV）缓存 */
static float    s_vdc_mv;

/* ---------------- 内部工具 ---------------- */
static void ADC_StableDelay(void) { Delay_us(200); }

/* 64重过采样：每64个原始ADC取平均 → 1个高有效bit
 * 注意：64是2的幂，/64 可以用移位（但用+1/2四舍五入用除法更清楚，
 * 编译器会自动优化成移位） */
static void do_oversampling_64x(void)
{
    uint16_t ob, i;
    uint32_t sum;

    for (ob = 0; ob < OVS_BUFFER_SIZE; ob++)
    {
        sum = 0;
        for (i = 0; i < OVERSAMPLING_RATIO; i++)
        {
            sum += adc_copy[ob * OVERSAMPLING_RATIO + i];
        }
        /* 四舍五入取平均 */
        ovs_buffer[ob] = (uint16_t)((sum + OVERSAMPLING_RATIO / 2U) / OVERSAMPLING_RATIO);
    }
}

/* ovs上算Vpp(ADC码) */
static uint16_t ovs_vpp_raw_code(void)
{
    uint16_t i;
    uint16_t mx = 0, mn = 4095;
    for (i = 0; i < OVS_BUFFER_SIZE; i++)
    {
        if (ovs_buffer[i] > mx) mx = ovs_buffer[i];
        if (ovs_buffer[i] < mn) mn = ovs_buffer[i];
    }
    return (mx >= mn) ? (mx - mn) : (uint16_t)0;
}

/* ovs上算直流平均(ADC码) */
static uint32_t ovs_avg_code(void)
{
    uint16_t i;
    uint32_t s = 0;
    for (i = 0; i < OVS_BUFFER_SIZE; i++) s += ovs_buffer[i];
    return (s + OVS_BUFFER_SIZE / 2U) / OVS_BUFFER_SIZE;
}

/* ADC码 → mV */
static inline float code_to_mv(uint32_t code)
{
    return (float)code * 3300.0f / 4096.0f;
}

/* ---------------- SMA + EMA 两级平滑 ---------------- */
/* 输入一帧原始Vpp_mV → 输出平滑后mV */
static float vpp_smooth_step(float raw_mv)
{
    /* 第1级：8点SMA */
    float sum;
    uint8_t i, cnt;
    float sma_out;

    s_sma_buf[s_sma_idx] = raw_mv;
    s_sma_idx = (s_sma_idx + 1) % VPP_SMA_WIN;
    if (!s_sma_filled && s_sma_idx == 0) s_sma_filled = 1;

    cnt = s_sma_filled ? VPP_SMA_WIN : (s_sma_idx ? s_sma_idx : 1);
    sum = 0.0f;
    for (i = 0; i < cnt; i++) sum += s_sma_buf[i];
    sma_out = sum / (float)cnt;

    /* 第2级：EMA 指数平均（α=1/32≈3%，显示稳定不跳动）*/
    if (!s_ema_inited)
    {
        s_ema_val = sma_out;
        s_ema_inited = 1;
    }
    else
    {
        /* 等价于：ema = ema + (sma - ema) >> SHIFT
         * 用浮点乘法更直观，编译器足够快 */
        float alpha = 1.0f / (float)(1 << VPP_EMA_SHIFT);
        s_ema_val = s_ema_val + alpha * (sma_out - s_ema_val);
    }

    return s_ema_val;
}

/* ---------------- 对外：初始化 ---------------- */
void ADC_Init_DMA(uint32_t sample_rate)
{
    GPIO_InitTypeDef gp;
    ADC_InitTypeDef  adc;
    DMA_InitTypeDef  dma;
    ADC_CommonInitTypeDef adc_common;

    (void)sample_rate;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,  ENABLE);

    /* PA0 = ADC1_CH0 */
    gp.GPIO_Pin  = GPIO_Pin_0;
    gp.GPIO_Mode = GPIO_Mode_AN;
    gp.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gp);

    /* ADC 通用初始化：关键！设置 ADC 时钟预分频
     * PCLK2 = 84MHz, Div4 → ADC_CLK = 21MHz (≤36MHz 安全值)
     * 不设此寄存器时 ADC 默认使用 HSI 可能高达 42MHz → 静默失败 */
    ADC_CommonStructInit(&adc_common);  /* 先填充默认值 */
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&adc_common);

    /* ADC1：12bit 连续转换，软件触发；
     *   总转换 = 480 + 12 = 492 ADC_CLK 周期
     *   ADC_CLK = PCLK2/4 = 84/4 = 21MHz
     *   → 单次采样耗时 ≈ 492/21M ≈ 23.4us
     *   → 1024点 一帧 ≈ 24ms
     *   → 64重过采样后16个有效点 ≈ 每个有效点 1.5ms
     */
    adc.ADC_Resolution           = ADC_Resolution_12b;
    adc.ADC_ScanConvMode         = DISABLE;
    adc.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T3_TRGO;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign            = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion      = 1;
    ADC_Init(ADC1, &adc);

    ADC_ContinuousModeCmd(ADC1, ENABLE);

    /* 采样时间 = 480 Cycles（最大档）*/
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);

    ADC_Cmd(ADC1, ENABLE);
    Delay_us(500);
    ADC_StableDelay();

    ADC_DMACmd(ADC1, ENABLE);

    /* DMA2 Stream0 CH0：ADC1 → RAM 循环半字传输 */
    DMA_DeInit(DMA2_Stream0);
    dma.DMA_Channel            = DMA_Channel_0;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)adc_buffer;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = ADC_BUFFER_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(DMA2_Stream0, &dma);

    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* 启动连续转换 */
    ADC_SoftwareStartConv(ADC1);
}

/* ---------------- DMA TC 中断 ---------------- */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        dma_irq_count++;

        /* 拷贝 + 过采样（放IRQ里保证数据新鲜；总耗时极少：
         * 1024半字拷贝 + 16次64累加 = STM32F4 168MHz下 <30us）
         */
        memcpy(adc_copy, adc_buffer, sizeof(adc_copy));
        do_oversampling_64x();
        s_vdc_mv = code_to_mv(ovs_avg_code());

        adc_new_data = 1;
    }
}

/* ---------------- 对外：查询新帧 ---------------- */
uint8_t ADC_IsNewDataReady(void)
{
    if (adc_new_data) { adc_new_data = 0; return 1; }
    return 0;
}

/* ---------------- 对外：直接读取 ADC 原始寄存器值 ---------------- */
uint16_t ADC_ReadValue(void)
{
    /* 等待转换完成 */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    /* 直接读 DR 寄存器 */
    return (uint16_t)(ADC1->DR & 0xFFFF);
}

/* ---------------- 对外：Vpp 原始值（仅做过采样，未做时间轴平滑） ---------------- */
float ADC_GetVppVoltage_Raw(void)
{
    uint16_t c = ovs_vpp_raw_code();
    return code_to_mv(c);
}

/* ---------------- 对外：Vpp 稳定读数（<4mV目标，SMA+EMA两级平滑） ---------------- */
float ADC_GetVppVoltage_Smooth(void)
{
    float raw = ADC_GetVppVoltage_Raw();
    return vpp_smooth_step(raw);
}

/* ---------------- 对外：直流平均 ---------------- */
float ADC_GetVoltageAvg(void)
{
    return s_vdc_mv;
}

/* ---------------- 对外：有效值 RMS ---------------- */
float ADC_GetRmsVoltage(void)
{
    uint32_t i;
    double sum_sq = 0.0;
    float mean = 0.0f;

    /* 先算直流平均 */
    for (i = 0; i < ADC_BUFFER_SIZE; i++) {
        mean += (float)adc_copy[i];
    }
    mean /= (float)ADC_BUFFER_SIZE;

    /* 计算 RMS = sqrt(mean((x - dc)^2)) */
    for (i = 0; i < ADC_BUFFER_SIZE; i++) {
        float v = (float)adc_copy[i] - mean;
        sum_sq += (double)v * (double)v;
    }
    float rms_code = (float)sqrt(sum_sq / (double)ADC_BUFFER_SIZE);

    /* ADC码 → mV */
    return rms_code * 3300.0f / 4096.0f;
}

/* ============================================================
 *  FFT 输入预处理：去直流 + 汉宁窗
 *  为了保持 FFT 与 高精度Vpp 兼容，这里直接用 adc_copy 的原始1024点，
 *  不用过采样后的数据（FFT需要高时间分辨率，而不是高幅度分辨率）
 * ============================================================ */
void ADC_Prepare_FFT_Input(float *input_data, uint16_t len, float *sample_rate_out)
{
    uint16_t i;
    double   sum;
    float    mean, x, w;

    if (len > ADC_BUFFER_SIZE) len = ADC_BUFFER_SIZE;

    /* 1) 去直流 */
    sum = 0.0;
    for (i = 0; i < len; i++) sum += (double)adc_copy[i];
    mean = (float)(sum / (double)len);

    /* 2) 归一化到[-1,1) + 汉宁窗 w(n)=0.5*(1-cos(2πn/(N-1))) */
    for (i = 0; i < len; i++)
    {
        x = ((float)adc_copy[i] - mean) / 2048.0f;
        if (len > 1)
            w = 0.5f * (1.0f - cosf(2.0f * 3.14159265358979f * (float)i / (float)(len - 1)));
        else
            w = 1.0f;
        input_data[i] = x * w;
    }

    /* 3) 等效采样率估计（ADC_CLK = PCLK2/4 = 21MHz, 480+12 cycles）
     *    Ts = (480 + 12) / 21e6  →  Fs = 21e6 / 492 ≈ 42683 Hz */
    if (sample_rate_out != NULL)
    {
        *sample_rate_out = 21000000.0f / (480.0f + 12.0f);   /* ≈ 42683 Hz */
    }
}

/* ============================================================
 *  波形数据获取：从 DMA 安全快照 (adc_copy) 提取 N 个点
 *  注意：使用 adc_copy 而非 adc_buffer 避免 DMA 数据竞争
 * ============================================================ */
void ADC_GetWaveformData(uint8_t *out, uint16_t points, uint8_t ymax)
{
    if (!out || points == 0) return;
    if (points > ADC_BUFFER_SIZE) points = ADC_BUFFER_SIZE;

    uint16_t step = ADC_BUFFER_SIZE / points;
    if (step == 0) step = 1;

    /* 计算直流偏置（使用安全快照）*/
    uint32_t adc_sum = 0;
    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        adc_sum += adc_copy[i];
    }
    float adc_avg = (float)adc_sum / (float)ADC_BUFFER_SIZE;

    /* 将 ±2048 码值范围映射到 0~ymax */
    float scale = (float)ymax / 4096.0f;

    for (uint16_t i = 0; i < points; i++) {
        uint16_t idx = i * step;
        if (idx >= ADC_BUFFER_SIZE) idx = ADC_BUFFER_SIZE - 1;
        float val = (float)adc_copy[idx] - adc_avg;
        float mapped = (val + 2048.0f) * scale;
        if (mapped < 0) mapped = 0;
        if (mapped > (float)ymax) mapped = (float)ymax;
        out[i] = (uint8_t)(mapped + 0.5f);
    }
}

/* ============================================================
 *  频谱数据获取：将 magnitude 数组归一化到 0~ymax
 * ============================================================ */
void ADC_GetSpectrumData(const float *magnitude, uint16_t bins,
                         uint16_t start_bin, uint8_t *out,
                         uint16_t points, uint8_t ymax)
{
    if (!magnitude || !out || !points || !bins) return;
    if (start_bin >= bins) return;
    if (start_bin + points > bins) points = (uint16_t)(bins - start_bin);

    /* 找最大值用于归一化 */
    float mag_max = 0.0f;
    for (uint16_t i = 0; i < points; i++) {
        float m = magnitude[start_bin + i];
        if (m > mag_max) mag_max = m;
    }
    if (mag_max < 1e-6f) mag_max = 1.0f;

    float scale = (float)ymax / mag_max;
    for (uint16_t i = 0; i < points; i++) {
        float m = magnitude[start_bin + i];
        float v = m * scale;
        if (v < 0) v = 0;
        if (v > (float)ymax) v = (float)ymax;
        out[i] = (uint8_t)(v + 0.5f);
    }
}
