#include "fft.h"

// 全局变量定义
float input_data[FFT_LENGTH] = {0};
float fft_output[FFT_LENGTH] = {0};
float magnitude[FFT_HALF_LEN] = {0};
arm_rfft_fast_instance_f32 fft_handle;

float harmonic_freq[HARMONIC_MAX_NUM] = {0};
float harmonic_amp[HARMONIC_MAX_NUM] = {0};
uint8_t harmonic_cnt = 0;

/**
 * @brief  FFT 参数初始化，上电运行一次
 */
void FFT_Init(void)
{
    arm_rfft_fast_init_f32(&fft_handle, FFT_LENGTH);
}

/**
 * @brief  执行实数 FFT 正变换，计算频谱幅值
 */
void Perform_FFT(void)
{
    // 时域 -> 频域
    arm_rfft_fast_f32(&fft_handle, input_data, fft_output, 0);
    // 计算复数幅值 (前 N/2 个点)
    arm_cmplx_mag_f32(fft_output, magnitude, FFT_HALF_LEN);
}

/**
 * @brief  提取谐波（示例：寻找幅值最大的前 HARMONIC_MAX_NUM 个峰值）
 * @param  sample_rate: 采样率 (Hz)
 */
void Extract_Harmonics(float sample_rate)
{
    float threshold = 0.1f;   // 幅值阈值，可调整
    harmonic_cnt = 0;
    float freq_res = sample_rate / FFT_LENGTH;

    // 从直流之后开始寻找 (i=1)，避免直流干扰
    for (int i = 1; i < FFT_HALF_LEN && harmonic_cnt < HARMONIC_MAX_NUM; i++)
    {
        // 判断是否为局部最大值
        if (magnitude[i] > threshold &&
            magnitude[i] > magnitude[i-1] &&
            magnitude[i] > magnitude[i+1])
        {
            harmonic_freq[harmonic_cnt] = i * freq_res;
            harmonic_amp[harmonic_cnt] = magnitude[i];
            harmonic_cnt++;
        }
    }
}
/**
 * @brief  查找基波频率 —— 使用「最小频率法」
 *          从低频向高频扫描，找到第一个显著的局部峰值作为基波。
 *          这样即使高次谐波幅值更大，基波（最低频率）也能被正确识别。
 * @param  sample_rate: 实际采样率 (Hz)
 * @param  amp_out:     输出基波对应的幅值（可为 NULL）
 * @retval 基波频率 (Hz)，若无信号返回 0
 */
/* 抛物线插值提高频率精度
 * 给定峰值 bin idx (magnitude[idx] 是最大值), 返回精确的小数 bin 位置
 * 公式: delta = 0.5 * (magnitude[idx-1] - magnitude[idx+1]) / (magnitude[idx-1] - 2*magnitude[idx] + magnitude[idx+1])
 * 返回: idx + delta
 */
static float ParabolicInterpPeak(uint16_t idx)
{
    if (idx == 0 || idx >= FFT_HALF_LEN - 1) return (float)idx;

    float a = magnitude[idx - 1];
    float b = magnitude[idx];
    float c = magnitude[idx + 1];

    float denom = a - 2.0f * b + c;
    if (fabsf(denom) < 1e-6f) return (float)idx;

    float delta = 0.5f * (a - c) / denom;

    /* 限制 delta 在 -0.5~0.5 之间 */
    if (delta < -0.5f) delta = -0.5f;
    if (delta > 0.5f) delta = 0.5f;

    return (float)idx + delta;
}

float Find_Fundamental_Frequency(float sample_rate, float *amp_out)
{
    if (amp_out != NULL) *amp_out = 0.0f;

    float freq_res = sample_rate / FFT_LENGTH;   // 频率分辨率 = Fs/N

    // ① 先扫描全谱，找到全局最大幅值（用于自适应阈值）
    float max_mag = 0.0f;
    uint16_t max_idx = 1;
    for (uint16_t i = 1; i < FFT_HALF_LEN; i++)
    {
        if (magnitude[i] > max_mag)
        {
            max_mag = magnitude[i];
            max_idx = i;
        }
    }

    if (max_mag < 0.01f) return 0.0f;   // 无有效信号

    // ② 自适应阈值 = max_mag * 10%（防止噪声触发，同时保证基波（即使幅值小）能被检测到）
    float threshold = max_mag * 0.10f;
    if (threshold < 0.02f) threshold = 0.02f;   // 阈值下限

    // ③ 从低频往高频扫，找第一个超过阈值的局部峰值 → 即基波
    //    从 bin 2 开始（跳过 bin 0 直流 + bin 1 低频噪声）
    for (uint16_t i = 2; i < FFT_HALF_LEN - 1; i++)
    {
        if (magnitude[i] >= threshold &&
            magnitude[i] >= magnitude[i - 1] &&
            magnitude[i] >= magnitude[i + 1])
        {
            // 找到最低频率的显著峰值 → 基波
            if (amp_out != NULL) *amp_out = magnitude[i];
            // ★ 用抛物线插值提高精度
            float precise_bin = ParabolicInterpPeak(i);
            return precise_bin * freq_res;
        }
    }

    // ④ 没找到则回退到最大幅值频率（兜底）
    if (amp_out != NULL) *amp_out = max_mag;
    float precise_bin = ParabolicInterpPeak(max_idx);
    return precise_bin * freq_res;
}