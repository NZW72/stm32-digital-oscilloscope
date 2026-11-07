#ifndef __TJC_HMI_CLIENT_H
#define __TJC_HMI_CLIENT_H

#include "stm32f4xx.h"
#include "TJC_HMI_events.h"
#include "TJC_HMI_parser.h"

/* ================================================================
 * TJC HMI Client Layer (Layer 5)
 * 整合 transport/codec/parser/events，提供应用友好 API。
 * 重点：
 *   - 数值/文本下发 节流去重（最小刷新间隔 + 旧值缓存）
 *   - 时域波形 / 频谱图 一键刷新（内部用 addt 握手透传）
 *   - 按钮事件回调（周期切换 / 频谱切换）
 *   - 统一处理 0xFE/0xFD 透传状态机
 * ================================================================ */

/* 配置：曲线控件 y 值限幅（必须同时 ≤ 255 和 ≤ 控件实际像素高度）*/
#ifndef TJC_WAVE_Y_MAX
#define TJC_WAVE_Y_MAX      240U
#endif
#ifndef TJC_FFT_Y_MAX
#define TJC_FFT_Y_MAX       240U
#endif

typedef enum {
    TJC_MODE_TIME = 0,        /* 时域 */
    TJC_MODE_SPEC = 1         /* 频谱 */
} TJC_DispMode_t;

typedef enum {
    TJC_PERIOD_1 = 1,
    TJC_PERIOD_3 = 3
} TJC_Period_t;

/* 应用事件回调：按钮按下/弹起、透传状态、结果码错误日志 */
typedef void (*TJC_AppEventCB)(const TJC_Event_t *ev);

/* ---------- 初始化 ---------- */
void TJC_Client_Init(uint32_t baud, TJC_AppEventCB app_cb);

/* 主循环中每次调用：泵 RB + 解析（非阻塞）*/
void TJC_Client_Task(void);

/* ---------- 数值/文本 下发（带节流/去重）---------- */
/* 刷新节流间隔（毫秒级，基于 SysTick/Delay_ms 粗计时）*/
void TJC_SetMinRefreshMs(uint32_t ms);

/* 写数字控件；若 值相同且未到期则跳过 */
void TJC_SetNumber(const char *full_obj, int32_t value);

/* 写文本；若 文本相同则跳过（ASCII 比较）*/
void TJC_SetText  (const char *full_obj, const char *gb2312_text);

/* 立即刷新（忽略节流）*/
void TJC_FlushAll(void);

/* ---------- 视图切换 ---------- */
void TJC_SetDisplayMode(TJC_DispMode_t m);   /* vis s0/s1 互切 */

/* ---------- 应用查询当前状态（由按键事件回调内部同步）---------- */
TJC_DispMode_t TJC_GetDisplayMode(void);
TJC_Period_t   TJC_GetPeriodMode(void);

/* ---------- 波形刷新（时域 / 频谱）---------- */
/* 时域：把 N 个点（范围 0~255 自动限幅）按通道 ch 画到曲线控件；
 * 若 qty>1 使用 addt 透传；qty==1 用 add。
 *   curve_id_expr = "s0.id"（推荐，不怕 id 变动）
 *   curve_name    = "s0"      （用于 cle）
 *   x_points_limit: UI 端显示的点数上限（s0.xnum，若传 0 不设置）*/
void TJC_RefreshWaveChannel(const char *curve_name,
                            const char *curve_id_expr,
                            uint8_t ch,
                            const uint8_t *y_values,
                            uint16_t qty,
                            uint16_t x_points_limit,
                            uint8_t  clear_before);

/* 频谱：将 magnitude[0..bins-1] 归一化到 0..FFT_Y_MAX 逐点发送
 *   推荐 bins = FFT_HALF_LEN 的前 128~256 低频段展示 */
void TJC_RefreshSpectrum(const char *curve_name,
                         const char *curve_id_expr,
                         uint8_t ch,
                         const float *magnitude,
                         uint16_t bins,
                         uint16_t start_bin,
                         uint16_t show_bins,
                         uint16_t x_points_limit);

/* 透传状态检查：等待 FE 超时(ms)（阻塞 polling）*/
/* 返回 1=收到 0xFE，0=超时 */
uint8_t TJC_WaitTransferReady(uint32_t timeout_ms);
/* 透传完成等待（收 0xFD） */
uint8_t TJC_WaitTransferDone (uint32_t timeout_ms);

/* 带超时保护的波形刷新版本
 *   timeout_ms: 总超时时间（毫秒），建议 20~50ms
 *   返回: 1=成功, 0=失败/超时 */
uint8_t TJC_RefreshWaveChannel_Try(const char *curve_name,
                                   const char *curve_id_expr,
                                   uint8_t ch,
                                   const uint8_t *y_values,
                                   uint16_t qty,
                                   uint16_t x_points_limit,
                                   uint8_t  clear_before,
                                   uint32_t timeout_ms);

/* 状态计数（调试） */
uint32_t TJC_GetNoiseCount(void);
uint32_t TJC_GetFrameCount(void);
uint32_t TJC_GetRBOverflow(void);

#endif
