#include "signal_process.h"
#include "fft.h"
#include "adc.h"
#include <math.h>
#include <string.h>

volatile SignalResult signal_res;
Sig_Result_t sig_result;

/* =========================================================
 *  精度保障（核心算法）
 *  目标：幅值 <5mV 误差，频率 10KHz-500KHz
 *
 *  A. 时域参数 (总体 Vpp/Vrms)：
 *      - 用 4096 点，12bit，LSB=0.806mV
 *      - 加 8 帧滑动平均 → 波动 ÷2.8 倍
 *
 *  B. 谐波参数（基波/2次/3次 峰峰值/有效值）
 *      1. FFT 前乘 汉宁窗（Hann window），减少栅栏效应导致的能量泄漏
 *      2. 峰值 bin 采用 3 点抛物线质心插值（Quinn式简化）：
 *           α = (a_{-1}-a_{+1}) / (a_{-1} - 2a_0 + a_{+1}) ，0.5 分母
 *           freq = (k + α) * fres
 *      3. 幅度 = 2*mag[k] * wincf / N；wincf 是窗函数相干性补偿系数：
 *           Hann: 2.0000 （相干增益补偿 2× 实序列幅值归一化 ÷ 0.5）
 *      4. 多帧滑动平均降抖
 * ========================================================= */

#define CODE_TO_MV   (3300.0f / 4096.0f)
#define NUM_HAR      DISPLAY_HARMONIC_MAX   /* 3 */

/* 多帧平均窗口 (vpp/vrms/freq x 每个谐波 + 总体) */
#define AVG_WIN  8
typedef struct {
    float buf_vpp[AVG_WIN];
    float buf_rms[AVG_WIN];
    float buf_freq[AVG_WIN];
    uint8_t idx;
    uint8_t fill;
} avg_win_t;
static avg_win_t s_avg_tot;              /* 总体 */
static avg_win_t s_avg_har[NUM_HAR];     /* 各次谐波 */

/* ---------- 窗口生成：汉宁窗（4096 点不大，运行时计算也行，预先算好更快） ---------- */
static float s_hann[4096];   /* 4096 点 Hann，首次调用初始化 */
static uint8_t s_hann_ready = 0;
static void ensure_hann(uint32_t N)
{
    if (s_hann_ready) return;
    for (uint32_t i = 0; i < N; i++) {
        float x = (float)i / (float)(N - 1);
        /* Hann(n) = 0.5 - 0.5cos(2πn/(N-1)) */
        s_hann[i] = 0.5f - 0.5f * cosf(6.283185307179586f * x);
    }
    s_hann_ready = 1;
}

/* -------- 滑动平均工具 -------- */
static float avg_push(avg_win_t *w, float val)
{
    w->buf_vpp[w->idx] = val;   /* 复用 buf_vpp 作为通用平均缓冲，其他接口用对应数组 */
    (void)w;
    (void)val;
    return 0;
}
static float avg_push_vpp(avg_win_t *w, float v)  { w->buf_vpp[w->idx] = v;  return v; }
static float avg_push_rms(avg_win_t *w, float v)  { w->buf_rms[w->idx] = v;  return v; }
static float avg_push_freq(avg_win_t *w, float v) { w->buf_freq[w->idx] = v; return v; }

static float avg_get(const float *buf, const avg_win_t *w)
{
    uint32_t n = w->fill ? AVG_WIN : w->idx;
    if (n == 0) n = 1;
    double s = 0.0;
    for (uint32_t i = 0; i < n; i++) s += (double)buf[i];
    return (float)(s / (double)n);
}
static void avg_advance(avg_win_t *w)
{
    w->idx = (w->idx + 1) % AVG_WIN;
    if (!w->fill && w->idx == 0) w->fill = 1;
}

/* ---------- 新 API 1：码值→去直流 mV→加 Hann 窗 ---------- */
void SP_PrepareWindowed(const uint16_t *raw, float *out_w, uint32_t N)
{
    ensure_hann(N);
    double sum = 0.0;
    for (uint32_t i = 0; i < N; i++) sum += (double)raw[i];
    float avg_code = (float)(sum / (double)N);
    for (uint32_t i = 0; i < N; i++) {
        float v = ((float)raw[i] - avg_code) * CODE_TO_MV;
        out_w[i] = v * s_hann[i];
    }
}

/* ---------- 新 API 2：总体时域参数（无窗，保证 Vpp 不被窗削峰）---------- */
void SP_ComputeTimeParams(const float *volt_mv, uint32_t N,
                          float *vpp, float *vrms, float *vdc)
{
    (void)vdc;
    if (!volt_mv || N == 0) {
        if (vpp)  *vpp = 0;
        if (vrms) *vrms = 0;
        return;
    }
    float vmax = volt_mv[0];
    float vmin = volt_mv[0];
    double ssq = 0.0;
    for (uint32_t i = 0; i < N; i++) {
        float v = volt_mv[i];
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        ssq += (double)v * (double)v;
    }
    float lvpp  = vmax - vmin;
    float lvrms = sqrtf((float)(ssq / (double)N));
    avg_push_vpp(&s_avg_tot, lvpp);
    avg_push_rms(&s_avg_tot, lvrms);
    lvpp  = avg_get(s_avg_tot.buf_vpp, &s_avg_tot);
    lvrms = avg_get(s_avg_tot.buf_rms, &s_avg_tot);
    avg_advance(&s_avg_tot);
    if (vpp)  *vpp  = lvpp;
    if (vrms) *vrms = lvrms;
}

/* ---------- 新 API 3：频谱分析 → 写入 sig_result.har[] ---------- */

/* 辅助：3 点质心插值（抛物线）
 *   k = 最大 bin index (1 <= k < N/2-1)
 *   返回 α ∈ [-0.5, 0.5]，精频 = (k+α)*fres
 *   同时按 3 bin 总能量用校正系数估峰值 A (mV) */
static float barycentric_alpha(const float *mag, int k, float *amp_corr)
{
    float a_1 = mag[k - 1];
    float a0  = mag[k];
    float a1  = mag[k + 1];
    float den = a_1 - 2.0f * a0 + a1;
    float alpha = 0.0f;
    if (den != 0.0f) {
        alpha = 0.5f * (a_1 - a1) / den;
    }
    /* 限幅，理论 α ∈ [-0.5,0.5]，避免数值抖动溢出 */
    if (alpha >  0.5f) alpha =  0.5f;
    if (alpha < -0.5f) alpha = -0.5f;
    /* Hann 窗 + 3-bin 综合幅值校正（近似公式：
     *   mag_corr = (1 - alpha^2) * a0  ，接近理想峰值；
     *   再乘 2/N * 补偿系数
     *   Hann 的 CF (相干损失补偿) = 2.0 （实序列单边谱幅值归一化） */
    float a = (1.0f - alpha * alpha) * a0;
    if (amp_corr) *amp_corr = a;
    return alpha;
}

void SP_AnalyzeSpectrum(const float *magnitude, uint32_t fft_len, float sample_rate)
{
    if (!magnitude || fft_len < 16) return;
    float N = (float)fft_len;
    float fres = sample_rate / N;
    int half = (int)(fft_len >> 1);
    if (half < 4) return;

    /* 基波搜索：忽略 DC 及极低 bin，从 bin 10 起
     * （4096 点 1.4Msps 时，bin 10 ≈ 3.4KHz，低于 10KHz 的是噪声，避免错峰）
     * 到 half-3 止，留余量给 +1 bin 插值 */
    int start_bin = 10;
    int end_bin   = half - 3;
    if (start_bin > end_bin) start_bin = 2;

    /* 先整体扫描 magnitude 找最大峰作为基波候选 */
    int     best_k = start_bin;
    float   best_a = magnitude[start_bin];
    for (int k = start_bin + 1; k <= end_bin; k++) {
        if (magnitude[k] > best_a) {
            best_a = magnitude[k];
            best_k = k;
        }
    }

    float base_A_mag_corr;
    float alpha0 = barycentric_alpha(magnitude, best_k, &base_A_mag_corr);
    float base_f_hz = ((float)best_k + alpha0) * fres;

    /* Hann 窗相干补偿：×2 (实序列单边) × 2 (Hann 能量集中恢复 0.5÷0.25)
     *    综合系数 Coef_A = 4/N ；A(峰)= Coef_A × mag_corr
     * 更精确系数用实验证：Coef_A = 4.0 / N × 1.0000 （Hann 理想 1kHz 正弦） */
    float coef_A = 4.0f / N;   /* Hann 窗实序列正弦峰值 A = 4/N × 插值后幅度 */

    /* 对基波 + 2次 + 3次循环： */
    for (int h = 0; h < NUM_HAR; h++) {
        int h_order = h + 1;   /* 1/2/3 次 */
        float target_f = base_f_hz * (float)h_order;
        /* 按频率算目标 bin，前后 ±2 搜最大（避免二次谐波落在栅栏）*/
        int k0 = (int)(target_f / fres + 0.5f);
        if (k0 < 1) k0 = 1;
        if (k0 > half - 3) k0 = half - 3;

        int bk = k0;
        float ba = magnitude[k0];
        for (int dk = -2; dk <= 2; dk++) {
            int k = k0 + dk;
            if (k < 1 || k >= half - 2) continue;
            if (magnitude[k] > ba) { ba = magnitude[k]; bk = k; }
        }

        float A_corr;
        float alpha = barycentric_alpha(magnitude, bk, &A_corr);
        float f_hz = ((float)bk + alpha) * fres;
        float A_mV = coef_A * A_corr;   /* 正弦峰值 (mV) */
        float Vpp   = 2.0f * A_mV;      /* 峰峰值 */
        float Vrms  = A_mV * 0.70710678118f; /* ÷√2 */

        /* 多帧滑动平均 */
        avg_push_vpp(&s_avg_har[h],  Vpp);
        avg_push_rms(&s_avg_har[h],  Vrms);
        avg_push_freq(&s_avg_har[h], f_hz);
        float av  = avg_get(s_avg_har[h].buf_vpp, &s_avg_har[h]);
        float ar  = avg_get(s_avg_har[h].buf_rms, &s_avg_har[h]);
        float af  = avg_get(s_avg_har[h].buf_freq, &s_avg_har[h]);
        avg_advance(&s_avg_har[h]);

        sig_result.har[h].freq_hz     = af;
        sig_result.har[h].amp_peak_mv = A_mV;   /* 保存最后一次峰值（不用平均也行，调用者用 Vpp 即可） */
        sig_result.har[h].vpp_mv      = av;
        sig_result.har[h].vrms_mv     = ar;
    }
    sig_result.har_cnt = NUM_HAR;
    sig_result.base_freq_hz = sig_result.har[0].freq_hz;  /* 基波频率由主搜索给出 */
    /* 覆盖用 基波 3 点插值频率（主搜索）作为 base_freq （更精确）*/
    sig_result.base_freq_hz = base_f_hz;
    signal_res.base_freq = sig_result.base_freq_hz;
    (void)alpha0;
}

/* ============= 旧 API 兼容 ============= */

void Calc_TimeDomain_Param(float *volt_buf, uint16_t *adc_buf, uint16_t len)
{
    (void)adc_buf;
    float vpp = 0, vrms = 0, vdc = 0;
    SP_ComputeTimeParams(volt_buf, len, &vpp, &vrms, &vdc);
    signal_res.vpp     = vpp;
    signal_res.v_rms   = vrms;
    sig_result.total_vpp_mv  = vpp;
    sig_result.total_vrms_mv = vrms;
    sig_result.total_vdc_mv  = vdc;
}

void Find_Base_Freq(float fs)
{
    /* 不再重实现，结果由 SP_AnalyzeSpectrum 写入 sig_result.base_freq_hz */
    signal_res.base_freq = sig_result.base_freq_hz;
    (void)fs;
}

void Find_All_Harmonic(float fs, float base_thresh, float harm_thresh)
{
    (void)fs; (void)base_thresh; (void)harm_thresh;
    /* 新 SP_AnalyzeSpectrum 已写入 sig_result.har[]。此处仅同步旧结构：*/
    for (int i = 0; i < NUM_HAR && i < HARMONIC_MAX_NUM; i++) {
        harmonic_freq[i] = sig_result.har[i].freq_hz;
        harmonic_amp[i]  = sig_result.har[i].amp_peak_mv;
    }
    harmonic_cnt = NUM_HAR;
}

/* 旧 API 包装（兼容老代码调用）*/
void Compute_Harmonic_Params(const float *magnitude, uint16_t fft_len, float sample_rate)
{
    SP_AnalyzeSpectrum(magnitude, (uint32_t)fft_len, sample_rate);
}

/* 旧 API（主循环用）：ADC 码值→浮点 mV+去直流。
 * 此函数已在 adc.c 中实现（4参数版本），此处不再重复定义。
 * 如需旧式 3 参数调用，请在调用处增加 NULL 作为 vdc_out 参数。*/
