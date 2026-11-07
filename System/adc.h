#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"

// ============================================================
//  过采样 & 缓冲配置（精度目标：Vpp误差 < 4 mV）
//
//  理论基础：STM32F4 12bit ADC，LSB=3300/4096≈0.806mV
//    单纯12bit：量化噪声 RMS≈LSB/√12≈0.233mV，峰峰值约±3LSB≈±2.4mV
//    但实际还有热噪声、参考电压噪声、电源纹波、采样保持误差
//    → 实测通常 Vpp 跳 5~20 mV（无过采样）
//
//  要稳定到 <4mV 误差 & 显示不跳动：
//    ① 采样时间 84Cycles → 480Cycles（最大，保证采样电容充足充电）
//    ② 软件过采样倍数 OVERSAMPLING_RATIO = 64
//       → 有效位数 ENOB 增加 0.5*log2(64) = 3bit ≈ 15bit
//       → 过采样后噪声降低 √64 = 8倍
//    ③ ADC_BUFFER = 1024点
//       → 过采样后 1024/64 = 16 个"有效采样点"
//       → 再在 16 点上取 max-min 得 Vpp（对慢信号够用；
//          如快信号可减小过采样比，我下面留了宏可改）
//    ④ Vpp 结果做 16 点滑动窗口指数平均（α小=平滑=变化慢）
// ============================================================
#define ADC_BUFFER_SIZE       1024    /* 必须 2 的幂，也方便 FFT 直接吃 */
#define OVERSAMPLING_RATIO    64      /* 64重过采样（改小变快、改变粗糙）*/
#define OVS_BUFFER_SIZE       (ADC_BUFFER_SIZE / OVERSAMPLING_RATIO)  /* 16 */

/* Vpp 平滑：指数移动平均 EMA α = 1/(1<<EMA_SHIFT)
 *   SHIFT=5 → α≈1/32≈3%，新值占3%旧值97%，非常稳定；
 *   要变快改 4→α≈6%、改 3→≈12%、改 6→≈1.5%更慢 */
#define VPP_EMA_SHIFT         5

/* Vpp 额外的简单滑动平均窗口（再消去残留毛刺）*/
#define VPP_SMA_WIN           8

extern volatile uint32_t dma_irq_count;
extern uint16_t adc_buffer[ADC_BUFFER_SIZE];

/* ---------------- API ---------------- */
void ADC_Init_DMA(uint32_t sample_rate);
uint8_t  ADC_IsNewDataReady(void);

/* 直接读取 ADC 原始寄存器值 (调试用，绕过 DMA) */
uint16_t ADC_ReadValue(void);

/* 直接返回已平滑的 Vpp 电压，单位 mV（误差 < 4mV 目标）*/
float    ADC_GetVppVoltage_Smooth(void);

/* 未平滑的原始 Vpp（调试对比用）*/
float    ADC_GetVppVoltage_Raw(void);

/* 直流平均电压 mV */
float    ADC_GetVoltageAvg(void);

/* 有效值 RMS mV */
float    ADC_GetRmsVoltage(void);

/* FFT 输入预处理：去直流 + 汉宁窗；输出采样率估计值 Hz */
void ADC_Prepare_FFT_Input(float *input_data, uint16_t len, float *sample_rate_out);

/* 波形数据获取：从 DMA 安全快照 (adc_copy) 提取 N 个点映射到 0~ymax
 *   out: 输出缓冲区 (uint8_t 数组, 大小 >= points)
 *   points: 输出点数
 *   ymax: Y 轴最大值 (通常 240 对应屏幕曲线控件像素高度) */
void ADC_GetWaveformData(uint8_t *out, uint16_t points, uint8_t ymax);

/* 频谱数据获取：将 magnitude 数组归一化到 0~ymax
 *   magnitude: FFT 幅值数组
 *   bins: 总数组长度
 *   start_bin: 起始 bin (跳过 DC 通常=1)
 *   out: 输出缓冲区
 *   points: 输出点数
 *   ymax: Y 轴最大值 */
void ADC_GetSpectrumData(const float *magnitude, uint16_t bins,
                         uint16_t start_bin, uint8_t *out,
                         uint16_t points, uint8_t ymax);

#endif
