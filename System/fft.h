#ifndef __FFT_H
#define __FFT_H

#include "arm_math.h"
#include "stm32f4xx.h"

#define FFT_LENGTH      1024
#define FFT_HALF_LEN    (FFT_LENGTH / 2)
#define HARMONIC_MAX_NUM 10




// FFT 相关全局变量
extern float input_data[FFT_LENGTH];
extern float fft_output[FFT_LENGTH];
extern float magnitude[FFT_HALF_LEN];
extern arm_rfft_fast_instance_f32 fft_handle;

// 谐波存储
extern float harmonic_freq[HARMONIC_MAX_NUM];
extern float harmonic_amp[HARMONIC_MAX_NUM];
extern uint8_t harmonic_cnt;

void FFT_Init(void);
void Perform_FFT(void);
void Extract_Harmonics(float sample_rate);



// 在 fft.h 中添加以下声明
float Find_Fundamental_Frequency(float sample_rate, float *amp_out);
// 返回基波频率，若未找到返回 0，amp_out 输出该频率对应的幅值



#endif
