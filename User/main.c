#include "stm32f4xx.h"
#include "delay.h"
#include "TJC_HMI_client.h"
#include "TJC_HMI_codec.h"
#include "fft.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 *  极致精准版 - 1024点采样 + 2.8MHz采样率 + FFT + 过零检测
 *  目标: 误差 ≤ 3mV, 频率 ≤ 1kHz 误差, 10kHz~600kHz
 *  改进:
 *   - ADC采样时间 15→3 cycles, 采样率 778kHz→2.8MHz
 *   - 60kHz时每周期 46 点 (原来 23 点), 波形更光滑
 *   - FFT 1024点, Nyquist=1.4MHz, 覆盖全频率范围
 *   - 波形: 真实ADC数据 + 线性插值 (不合成, 保留谐波)
 * ============================================================ */

#define HMI_OBJ_VPP     "n_vpp"
#define HMI_OBJ_VRMS    "n_vrms"
#define HMI_OBJ_FREQ    "n_freq"
#define HMI_OBJ_P1VPP   "n_p1_vpp"
#define HMI_OBJ_P2VPP   "n_p2_vpp"

/* 参数配置 */
#define BURST_COUNT    1024
#define MEDIAN_SIZE     7

/* 谐波峰峰值滤波 */
static int32_t s_p1vpp_ring[MEDIAN_SIZE];
static int32_t s_p2vpp_ring[MEDIAN_SIZE];

/* 波形显示参数 */
#define WAVE_Y_MAX     240
#define WAVE_X_POINTS  512
#define WAVE_CH        0

/* 实际采样率: ADC_CLK=42MHz(Div2), 采样3cycles+转换12cycles=15cycles → 2.8MHz */
#define ACTUAL_SAMPLE_RATE  2800000.0f

#ifndef PI
#define PI 3.14159265358979323846
#endif

/* ------- 过零检测参数 ------- */
/* 1024点采样, 1.4MHz采样率, 500kHz时有 1024/(1.4M/500k)=366 个周期 → 需要 370 个过零点 */
#define MAX_CROSSINGS  700
static uint16_t s_crossings[MAX_CROSSINGS];
static uint16_t s_cross_cnt = 0;
static uint16_t s_gaps[MAX_CROSSINGS];

/* 用已保存的过零点数据获取精确的 N 个完整周期窗口 */
static uint8_t GetExactWindow(uint16_t periods, uint16_t *start_idx, uint16_t *len)
{
    uint16_t need = periods + 2;
    if (s_cross_cnt < need) return 1;

    uint16_t s = s_crossings[1];        /* 跳过第 0 个 */
    uint16_t e = s_crossings[1 + periods];
    if (e <= s) return 2;
    if (e > BURST_COUNT) return 3;

    *start_idx = s;
    *len = e - s;
    return 0;
}

/* 采样缓冲区 */
static uint16_t s_buf[BURST_COUNT];

/* 结果缓冲 (用于中位数滤波) */
static int32_t s_vpp_ring[MEDIAN_SIZE];
static int32_t s_rms_ring[MEDIAN_SIZE];
static uint8_t s_ring_idx = 0;
static uint8_t s_ring_fill = 0;

/* 排序用临时数组 */
static int32_t s_sort_tmp[MEDIAN_SIZE];

/* 波形显示缓冲区 */
static uint8_t s_wave_buf[WAVE_X_POINTS];

/* 频率估算值 (Hz) */
static float s_est_freq = 50000.0f;

/* ------- 波形快照（用于冻结显示） ------- */
static uint16_t s_wave_snap[BURST_COUNT];
static float    s_snap_freq = 50000.0f;
static uint8_t  s_wave_frozen = 0;
static uint8_t  s_snap_needed = 0;

/* FFT 平滑频率 */
#define FREQ_SMOOTH_WIN  3
static float s_freq_smooth[FREQ_SMOOTH_WIN];
static uint8_t s_freq_s_idx = 0;
static uint8_t s_freq_s_fill = 0;

/* ADC 初始化 */
static void MyADC_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;
    ADC_CommonInitTypeDef adc_common;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    ADC_CommonStructInit(&adc_common);
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInit(&adc_common);

    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
    ADC_Cmd(ADC1, ENABLE);
    Delay_ms(10);
}

/* 极致采样 - 连续模式, 只触发一次, 读完 DR 自动开始下一次转换 */
static void BurstSample(void)
{
    uint32_t i;
    uint16_t *p = s_buf;

    __disable_irq();

    ADC1->CR2 |= ADC_CR2_SWSTART;   /* 只触发一次, 连续模式自动转换 */
    for (i = 0; i < BURST_COUNT; i++)
    {
        while (!(ADC1->SR & ADC_SR_EOC));
        *p++ = (uint16_t)ADC1->DR;
    }

    __enable_irq();
}

/* 简单 Vpp 计算 (不插值, 直接 max-min) */
static uint16_t CalcVpp_Simple(void)
{
    uint16_t min_v = 0xFFFF, max_v = 0;
    uint16_t *p = s_buf;
    uint32_t i;

    for (i = 0; i < BURST_COUNT; i++)
    {
        uint16_t v = *p++;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
    }
    return max_v - min_v;
}

/* True RMS 计算 (减去直流偏置后再算交流有效值)
 * 公式: RMS = sqrt(Σ(xi - x_avg)² / n)
 * 先求DC偏置(所有采样点平均值)，再减偏置，再平方平均开方
 * 对于正弦波: Vrms = Vpp/2 * 0.707 = Vpp * 0.3535
 */
static uint32_t CalcRMS(void)
{
    /* Step 1: 计算直流偏置 */
    uint32_t sum = 0;
    uint16_t *p = s_buf;
    uint32_t i;
    for (i = 0; i < BURST_COUNT; i++) sum += *p++;
    float avg = (float)sum / (float)BURST_COUNT;

    /* Step 2: 减偏置 + 平方 + 求和 */
    uint64_t sq = 0;
    p = s_buf;
    for (i = 0; i < BURST_COUNT; i++)
    {
        float v = (float)*p++ - avg;   /* 减去DC偏置 */
        sq += (uint64_t)(v * v);
    }

    /* Step 3: 平均 + 开方 */
    uint64_t mean_sq = sq / BURST_COUNT;
    return (uint32_t)__builtin_sqrt((double)mean_sq);
}

/* 插入排序 (用于中位数) */
static void InsertSort(int32_t *arr, uint8_t n)
{
    uint8_t i, j;
    for (i = 1; i < n; i++)
    {
        int32_t key = arr[i];
        j = i;
        while (j > 0 && arr[j - 1] > key)
        {
            arr[j] = arr[j - 1];
            j--;
        }
        arr[j] = key;
    }
}

/* 中位数滤波 */
static int32_t MedianFilter(int32_t new_val, int32_t *ring)
{
    ring[s_ring_idx] = new_val;
    s_ring_idx = (s_ring_idx + 1) % MEDIAN_SIZE;
    if (!s_ring_fill && s_ring_idx == 0) s_ring_fill = 1;

    uint8_t count = s_ring_fill ? MEDIAN_SIZE : s_ring_idx;

    for (uint8_t i = 0; i < count; i++)
    {
        s_sort_tmp[i] = ring[i];
    }
    InsertSort(s_sort_tmp, count);

    return s_sort_tmp[count / 2];
}

/* 快速插入排序 (uint16_t) */
static void SortGap_U16(uint16_t *arr, uint16_t n)
{
    uint16_t i, j;
    for (i = 1; i < n; i++)
    {
        uint16_t k = arr[i];
        j = i;
        while (j > 0 && arr[j - 1] > k)
        {
            arr[j] = arr[j - 1];
            j--;
        }
        arr[j] = k;
    }
}

/* ============================================================
 *  过零中位数法检测频率 (全频率范围, 抗谐波/抗噪声)
 * ============================================================ */
static float EstimateFrequency_ZeroCross(void)
{
    /* 计算直流偏置 */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < BURST_COUNT; i++) sum += s_buf[i];
    float avg = (float)sum / (float)BURST_COUNT;

    /* 找上升沿过零点 */
    uint8_t above = 0;
    uint16_t cross_cnt = 0;

    for (uint16_t i = 1; i < BURST_COUNT && cross_cnt < MAX_CROSSINGS; i++)
    {
        float v0 = (float)s_buf[i-1] - avg;
        float v1 = (float)s_buf[i] - avg;

        if (!above && v0 < 0 && v1 >= 0)
        {
            s_crossings[cross_cnt++] = i;
            above = 1;
        }
        else if (above && v1 < 0)
        {
            above = 0;
        }
    }

    s_cross_cnt = cross_cnt;

    if (cross_cnt < 3) return 0.0f;

    /* 计算间隔数组 */
    uint16_t gap_cnt = 0;
    for (uint16_t i = 1; i < cross_cnt && gap_cnt < MAX_CROSSINGS; i++)
    {
        uint16_t g = s_crossings[i] - s_crossings[i-1];
        if (g >= 2) s_gaps[gap_cnt++] = g;
    }

    if (gap_cnt < 3) return 0.0f;

    /* 排序取中位数 */
    SortGap_U16(s_gaps, gap_cnt);
    uint16_t median_gap = s_gaps[gap_cnt / 2];

    if (median_gap < 2) return 0.0f;

    return ACTUAL_SAMPLE_RATE / (float)median_gap;
}

/* ============================================================
 *  FFT 频率检测 (1024点, 1.4MHz采样率, Nyquist=700kHz, 分辨率1367Hz)
 * ============================================================ */
static float EstimateFrequency_FFT(void)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < BURST_COUNT; i++) sum += s_buf[i];
    float avg = (float)sum / (float)BURST_COUNT;

    /* 去DC + Hann窗 */
    for (uint16_t i = 0; i < BURST_COUNT; i++)
    {
        float x = ((float)s_buf[i] - avg) / 2048.0f;
        float w = 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(BURST_COUNT - 1)));
        input_data[i] = x * w;
    }

    Perform_FFT();

    float amp_out;
    float raw_freq = Find_Fundamental_Frequency(ACTUAL_SAMPLE_RATE, &amp_out);

    if (raw_freq >= 10000.0f && raw_freq <= 700000.0f)
        return raw_freq;
    return 0.0f;
}

/* ============================================================
 *  主频率检测入口
 *  策略: FFT 为主, 过零为辅, 交叉验证
 * ============================================================ */
static float EstimateFrequency(void)
{
    float f_zc = EstimateFrequency_ZeroCross();
    float f_fft = EstimateFrequency_FFT();

    float raw_freq = 0.0f;

    if (f_fft > 0.0f)
        raw_freq = f_fft;           /* FFT 优先 */
    else if (f_zc > 0.0f)
        raw_freq = f_zc;            /* 过零兜底 */

    /* 两者都有效且接近时取平均 (更稳定) */
    if (f_fft > 0.0f && f_zc > 0.0f)
    {
        float ratio = f_zc / f_fft;
        if (ratio > 0.7f && ratio < 1.4f)
            raw_freq = (f_zc + f_fft) / 2.0f;
    }

    /* 限幅 + 平滑 */
    if (raw_freq >= 10000.0f && raw_freq <= 600000.0f)
    {
        s_freq_smooth[s_freq_s_idx] = raw_freq;
        s_freq_s_idx = (s_freq_s_idx + 1) % FREQ_SMOOTH_WIN;
        if (!s_freq_s_fill && s_freq_s_idx == 0) s_freq_s_fill = 1;
        uint8_t n = s_freq_s_fill ? FREQ_SMOOTH_WIN : s_freq_s_idx;
        float avg_f = 0.0f;
        for (uint8_t k = 0; k < n; k++) avg_f += s_freq_smooth[k];
        s_est_freq = avg_f / (float)n;
    }

    return s_est_freq;
}

/* ============================================================
 *  应用事件回调
 *  b1(0x0D)频谱切换已由客户端层内部处理
 *  b2(0x0E): 刷新按钮
 * ============================================================ */
static uint8_t s_force_refresh = 0;

static void AppEventCallback(const TJC_Event_t *ev)
{
    if (!ev) return;
    if (ev->kind == TJC_EVT_CLICK && ev->event == 0x00)
    {
        if (ev->obj_id == 0x0C)        /* 冻结按钮 */
        {
            s_snap_needed = 1;
        }
        else if (ev->obj_id == 0x0E)   /* b2: 刷新按钮 */
        {
            s_force_refresh = 1;
        }
    }
}

/* 波形快照 */
static void TakeWaveSnapshot(void)
{
    memcpy(s_wave_snap, s_buf, BURST_COUNT * sizeof(uint16_t));
    s_snap_freq = s_est_freq;
    s_wave_frozen = 1;
}

/* ============================================================
 *  波形显示: 始终用真实ADC数据 + 线性插值
 *  - 从过零点确定精确 N 个完整周期窗口
 *  - 波谷贴底 (Y=240)
 *  - 冻结时用快照数据
 * ============================================================ */
#define WAVE_TOP_MARGIN  5

static void RefreshWaveform(void)
{
    uint16_t *src = s_wave_frozen ? s_wave_snap : s_buf;

    TJC_Period_t period_mode = TJC_GetPeriodMode();
    uint16_t periods = (period_mode == TJC_PERIOD_3) ? 3 : 1;

    float freq = s_wave_frozen ? s_snap_freq : s_est_freq;
    if (freq < 1000.0f) freq = 50000.0f;

    float samples_per_period = ACTUAL_SAMPLE_RATE / freq;

    /* 精确 N 个完整周期窗口 */
    uint16_t start_idx = 0;
    uint16_t display_samples = 0;

    if (!s_wave_frozen)
    {
        uint8_t ok = GetExactWindow(periods, &start_idx, &display_samples);
        if (ok != 0 || display_samples < 4 || display_samples > BURST_COUNT
            || start_idx + display_samples > BURST_COUNT)
        {
            start_idx = 0;
            float exact_samples = samples_per_period * (float)periods;
            display_samples = (uint16_t)(exact_samples + 0.5f);
        }
    }
    else
    {
        start_idx = 0;
        float exact_samples = samples_per_period * (float)periods;
        display_samples = (uint16_t)(exact_samples + 0.5f);
    }

    if (start_idx + display_samples > BURST_COUNT)
    {
        if (start_idx >= BURST_COUNT) start_idx = 0;
        display_samples = BURST_COUNT - start_idx;
    }
    if (display_samples < 4) display_samples = 4;
    if (display_samples > BURST_COUNT) display_samples = BURST_COUNT;

    /* 找波形范围 (窗口内) */
    uint16_t vmin = 0xFFFF, vmax = 0;
    for (uint16_t i = 0; i < display_samples; i++)
    {
        uint16_t v = src[start_idx + i];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }

    uint16_t vpp_code = vmax - vmin;
    if (vpp_code < 5) vpp_code = 5;

    float scale = (float)(WAVE_Y_MAX - WAVE_TOP_MARGIN) / (float)vpp_code;

    /* 线性插值 (真实数据, 保留谐波形状) */
    for (uint16_t i = 0; i < WAVE_X_POINTS; i++)
    {
        float src_pos = (float)i * (float)display_samples / (float)WAVE_X_POINTS;
        uint16_t idx0 = (uint16_t)src_pos;
        uint16_t idx1 = idx0 + 1;
        if (idx1 >= display_samples) idx1 = display_samples - 1;
        float frac = src_pos - (float)idx0;

        float v0 = (float)src[start_idx + idx0];
        float v1 = (float)src[start_idx + idx1];
        float v = v0 + frac * (v1 - v0);

        float mapped = (float)WAVE_Y_MAX - (v - (float)vmin) * scale;

        if (mapped < 0.0f) mapped = 0.0f;
        if (mapped > (float)WAVE_Y_MAX) mapped = (float)WAVE_Y_MAX;
        s_wave_buf[i] = (uint8_t)(mapped + 0.5f);
    }

    /* 后处理: 3点滑动平均 (仅对显示数据, 不影响 Vpp/RMS) */
    if (WAVE_X_POINTS >= 3)
    {
        uint8_t tmp0 = s_wave_buf[0];
        for (uint16_t i = 1; i < WAVE_X_POINTS - 1; i++)
        {
            uint8_t tmp1 = s_wave_buf[i];
            s_wave_buf[i] = (uint8_t)(((uint16_t)tmp0 + (uint16_t)tmp1 + (uint16_t)s_wave_buf[i+1]) / 3);
            tmp0 = tmp1;
        }
    }

    if (TJC_GetDisplayMode() == TJC_MODE_TIME)
    {
        TJC_RefreshWaveChannel("s0", "s0.id", WAVE_CH,
                               s_wave_buf, WAVE_X_POINTS,
                               WAVE_X_POINTS, 1);
    }
}

/* ============================================================
 *  频谱图显示: 用s1波形控件显示FFT频谱
 * ============================================================ */
static uint8_t s_spec_forced = 0;  /* 强制刷新标志 */

static void RefreshSpectrum(void)
{
    if (TJC_GetDisplayMode() != TJC_MODE_SPEC)
    {
        s_spec_forced = 0;  /* 退出频谱模式时复位 */
        return;
    }

    /* 正常模式: 每3帧刷新一次(降频)
     * 强制模式: 立即刷新一次 */
    static uint8_t frame_cnt = 0;
    if (!s_spec_forced)
    {
        frame_cnt++;
        if (frame_cnt < 3) return;
        frame_cnt = 0;
    }
    else
    {
        s_spec_forced = 0;
        frame_cnt = 0;
    }

    /* 去DC + Hann窗 + FFT */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < BURST_COUNT; i++) sum += s_buf[i];
    float avg = (float)sum / (float)BURST_COUNT;

    for (uint16_t i = 0; i < BURST_COUNT; i++)
    {
        float x = ((float)s_buf[i] - avg) / 2048.0f;
        float w = 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(BURST_COUNT - 1)));
        input_data[i] = x * w;
    }
    Perform_FFT();

    /* 发送前128个频点到s1, 自动归一化 */
    TJC_RefreshSpectrum("s1", "s1.id", 0,
                         magnitude, FFT_HALF_LEN,
                         0, 128, 128);
}

int main(void)
{
    SystemInit();
    Delay_Init(168);
    Delay_TickInit();

    /* 1. 屏幕初始化 + 注册事件回调 */
    TJC_Client_Init(115200, AppEventCallback);
    TJC_SetMinRefreshMs(0);
    Delay_ms(200);
    TJC_Cmd_Page("page0");
    Delay_ms(50);

    /* 2. 配置时域曲线控件 s0 */
    TJC_Cmd_SetAttrInt("s0.xnum", WAVE_X_POINTS);
    TJC_Cmd_SetAttrInt("s0.gridcolor", 0x00FF00);
    TJC_Cmd_SetAttrInt("s0.gridwidth", 1);
    TJC_Cmd_SetAttrInt("s0.bgcolor", 0x000000);
    Delay_ms(20);

    /* 3. 配置频谱曲线控件 s1 */
    TJC_Cmd_SetAttrInt("s1.xnum", 128);
    TJC_Cmd_SetAttrInt("s1.gridcolor", 0x00FF00);
    TJC_Cmd_SetAttrInt("s1.gridwidth", 1);
    TJC_Cmd_SetAttrInt("s1.bgcolor", 0x000000);
    Delay_ms(20);

    /* 4. 设置初始显示模式 */
    TJC_SetDisplayMode(TJC_MODE_TIME);

    /* 5. ADC 初始化 */
    MyADC_Init();

    /* 6. FFT 初始化 */
    FFT_Init();

    /* 7. 初始采样 + 频率估算 + 快照 */
    BurstSample();
    EstimateFrequency();
    TakeWaveSnapshot();

    /* 8. 主循环 */
    while (1)
    {
        /* ① 强制刷新处理: 重置所有滤波环, 重新采样 */
        if (s_force_refresh)
        {
            s_force_refresh = 0;
            /* 清空中位数滤波环 */
            memset(s_vpp_ring, 0, sizeof(s_vpp_ring));
            memset(s_rms_ring, 0, sizeof(s_rms_ring));
            s_ring_idx = 0;
            s_ring_fill = 0;
            /* 清空频率平滑 */
            memset(s_freq_smooth, 0, sizeof(s_freq_smooth));
            s_freq_s_idx = 0;
            s_freq_s_fill = 0;
            /* 清空谐波滤波 */
            memset(s_p1vpp_ring, 0, sizeof(s_p1vpp_ring));
            memset(s_p2vpp_ring, 0, sizeof(s_p2vpp_ring));
            /* 强制重新快照 + 频谱刷新 */
            s_snap_needed = 1;
            s_spec_forced = 1;
        }

        /* ② 连续采样 1024 点 */
        BurstSample();

        /* ② 计算 Vpp 和 RMS */
        uint16_t raw_vpp = CalcVpp_Simple();
        uint32_t raw_rms = CalcRMS();

        /* ③ 转 mV (四舍五入) */
        int64_t vpp_tmp = (int64_t)raw_vpp * 3300 + 2048;
        int64_t rms_tmp = (int64_t)raw_rms * 3300 + 2048;
        int32_t vpp_mv = (int32_t)(vpp_tmp / 4096);
        int32_t rms_mv = (int32_t)(rms_tmp / 4096);

        /* ④ 中位数滤波 */
        int32_t vpp_filt = MedianFilter(vpp_mv, s_vpp_ring);
        int32_t rms_filt = MedianFilter(rms_mv, s_rms_ring);

        /* ⑤ 限幅 */
        if (vpp_filt < 0) vpp_filt = 0;
        if (vpp_filt > 5000) vpp_filt = 5000;
        if (rms_filt < 0) rms_filt = 0;
        if (rms_filt > 5000) rms_filt = 5000;

        /* ⑥ 发送数值到屏幕 */
        TJC_SetNumber(HMI_OBJ_VPP, vpp_filt);
        TJC_SetNumber(HMI_OBJ_VRMS, rms_filt);

        /* ⑦ 频率估算, 显示 kHz (2位小数, HMI dp=2) */
        float freq = EstimateFrequency();
        TJC_SetNumber(HMI_OBJ_FREQ, (int32_t)(freq / 10.0f + 0.5f));

        /* ⑧ 谐波峰峰值计算 (FFT分析) */
        {
            /* 去DC + Hann窗 + FFT */
            uint32_t sum = 0;
            for (uint16_t i = 0; i < BURST_COUNT; i++) sum += s_buf[i];
            float avg = (float)sum / (float)BURST_COUNT;

            for (uint16_t i = 0; i < BURST_COUNT; i++)
            {
                float x = ((float)s_buf[i] - avg) / 2048.0f;
                float w = 0.5f * (1.0f - cosf(2.0f * PI * (float)i / (float)(BURST_COUNT - 1)));
                input_data[i] = x * w;
            }
            Perform_FFT();

            /* 提取基波和2次谐波幅值 */
            float freq_res = ACTUAL_SAMPLE_RATE / (float)FFT_LENGTH;
            uint16_t fund_bin = (uint16_t)(s_est_freq / freq_res + 0.5f);
            if (fund_bin < 1) fund_bin = 1;
            if (fund_bin >= FFT_HALF_LEN) fund_bin = FFT_HALF_LEN - 1;

            float a1 = 0, a2 = 0;
            for (int8_t d = -2; d <= 2; d++)
            {
                uint16_t i1 = fund_bin + d;
                uint16_t i2 = fund_bin * 2 + d;
                if (i1 >= 1 && i1 < FFT_HALF_LEN && magnitude[i1] > a1) a1 = magnitude[i1];
                if (i2 >= 1 && i2 < FFT_HALF_LEN && magnitude[i2] > a2) a2 = magnitude[i2];
            }

            /* 转换为峰峰值(mV)
             * FFT输入: (ADC-DC)/2048, 范围-1~+1
             * 1024点FFT幅值 = 峰值 × N/2 × Hann补偿(2)
             * 所以: 峰值(归一化) = magnitude / (N/2) / 2
             * 峰峰值(归一化) = 峰值 × 2 = magnitude / (N/2)
             * Vpp_mv = magnitude / (N/2) × 2048 × 3300/4096 × 2(Hann补偿)
             *        = magnitude × 2 × 2 × 2048 × 3300 / 1024 / 4096
             *        = magnitude × 3300 / 512 */
            int32_t p1_mv = (int32_t)(a1 * 3300.0f / 512.0f + 0.5f);
            int32_t p2_mv = (int32_t)(a2 * 3300.0f / 512.0f + 0.5f);

            /* 中位数滤波 */
            int32_t p1_filt = MedianFilter(p1_mv, s_p1vpp_ring);
            int32_t p2_filt = MedianFilter(p2_mv, s_p2vpp_ring);

            /* 限幅 */
            if (p1_filt < 0) p1_filt = 0;
            if (p1_filt > 5000) p1_filt = 5000;
            if (p2_filt < 0) p2_filt = 0;
            if (p2_filt > 5000) p2_filt = 5000;

            TJC_SetNumber(HMI_OBJ_P1VPP, p1_filt);
            TJC_SetNumber(HMI_OBJ_P2VPP, p2_filt);
        }

        /* ⑨ 波形快照检查 + 刷新 */
        if (s_snap_needed)
        {
            s_snap_needed = 0;
            TakeWaveSnapshot();
        }
        RefreshWaveform();
        RefreshSpectrum();

        /* ⑨ 屏幕任务 */
        TJC_Client_Task();

        /* ⑩ 延时 100ms */
        Delay_ms(100);
    }
}
