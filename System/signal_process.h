#ifndef __SIGNAL_PROCESS_H
#define __SIGNAL_PROCESS_H

#include "stm32f4xx.h"
#include <stdint.h>

#ifndef DISPLAY_HARMONIC_MAX
#define DISPLAY_HARMONIC_MAX  3   /* 显示 基波 + 2次 + 3次 */
#endif

/* 单个谐波参数（峰峰值、有效值、峰值、频率）*/
typedef struct {
    float freq_hz;       /* 中心频率(Hz)，经 3 点质心插值校正 */
    float amp_peak_mv;   /* 正弦峰值 A (mV) ，经窗函数增益补偿 */
    float vpp_mv;        /* 峰峰值 = 2*A (mV) */
    float vrms_mv;       /* 有效值 = A/sqrt(2) (mV) */
} HarmonicParam_t;

/* 全局总结果（对外导出）*/
typedef struct {
    float    total_vpp_mv;             /* 总体信号峰峰值 (mV) */
    float    total_vrms_mv;            /* 总体信号有效值 (mV) */
    float    total_vdc_mv;             /* 直流偏置 (mV) */
    float    base_freq_hz;             /* 基波频率 (Hz)，优先硬件捕获，其次 FFT 插值 */
    uint8_t  har_cnt;                  /* 有效谐波数量，一般=基波+谐波 */
    HarmonicParam_t har[DISPLAY_HARMONIC_MAX]; /* har[0]=基波, har[1]=2次, har[2]=3次 */
} Sig_Result_t;

/* 旧版结构体（保持向后兼容）*/
typedef struct {
    float base_freq;
    float vpp;
    float v_rms;
} SignalResult;

extern volatile SignalResult signal_res;   /* 兼容旧代码引用 */
extern Sig_Result_t sig_result;            /* 新版详细结果 */

/* 时域 Vpp/Vrms（保留旧 API）*/
void Calc_TimeDomain_Param(float *volt_buf, uint16_t *adc_buf, uint16_t len);

/* 快速找基波：magnitude 上从 bin 2 起找最大峰
 * 返回频率 Hz，*amp_out 输出未归一化 magnitude */
void Find_Base_Freq(float fs);

/* 旧版谐波搜寻接口（保留兼容，不再用其做精算；新算法用下面 Compute_Harmonic_Params）*/
void Find_All_Harmonic(float fs, float base_thresh, float harm_thresh);

/* ---------- 新版（推荐）：平顶窗 + 3 点质心频偏 + 精确 Vpp/Vrms ---------- */

/* 1) 预处理：码值 -> 浮点 mV，减去 DC，并乘上平顶窗函数 Hann / FlatTop 两种可选
 *   output_volt_w[N] 必须 caller 提供 N 点 float */
void SP_PrepareWindowed(const uint16_t *raw, float *output_volt_w, uint32_t N);

/* 2) 频域分析：输入 magnitude[0..N/2]（实序列 FFT 半谱）和 sample_rate
 *   功能：从 magnitude 里找基波 bin → 3 点质心插值得精频 → 2次/3次 按基波 bin 倍数取
 *         分别用平顶窗补偿系数换算 Vpp/Vrms 写入 sig_result */
void SP_AnalyzeSpectrum(const float *magnitude, uint32_t fft_len, float sample_rate);

/* 3) 时域精算：在窗后 / 或窗前 mV 数据上算总体 Vpp/Vrms
 *  （总体 Vpp 建议用未加窗的 raw→去直流→直接算，避免加窗削峰导致 Vpp 偏小）*/
void SP_ComputeTimeParams(const float *volt_mv, uint32_t N,
                          float *vpp, float *vrms, float *vdc);

#endif
