#include "TJC_HMI_client.h"
#include "TJC_HMI_transport.h"
#include "TJC_HMI_codec.h"
#include "TJC_HMI_events.h"
#include "TJC_HMI_parser.h"
#include "Delay.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* -------------------------------------------------------------- */
/* 内部单例（简洁起见，此处单用户，不做多实例）                     */
/* -------------------------------------------------------------- */
#define TJC_CACHE_MAX_ITEMS     32     /* 最多缓存 32 个对象的上次值/文本，够 n0~n8+s0vis+b0txt 等用 */
#define TJC_CACHE_KEY_LEN       32

typedef struct {
    char    key[TJC_CACHE_KEY_LEN];
    uint8_t used;
    uint8_t is_str;           /* 0=number 1=string */
    int32_t nval;
    char    sval[48];
} TJC_CacheItem_t;

static TJC_Parser_t       s_parser;
static TJC_Events_t       s_events;
static TJC_AppEventCB     s_app_cb;

static volatile uint8_t   s_xfer_ready_flag;   /* 0xFE 置位 */
static volatile uint8_t   s_xfer_done_flag;    /* 0xFD 置位 */

static TJC_DispMode_t     s_mode = TJC_MODE_TIME;
static TJC_Period_t       s_period = TJC_PERIOD_1;

static TJC_CacheItem_t    s_cache[TJC_CACHE_MAX_ITEMS];
static uint32_t           s_min_refresh_ms = 40;   /* 默认 ≤25fps */
static uint32_t           s_last_refresh_tick;     /* 基于 Delay_sys 粗计时，无则用累计调用计数近似 */

/* -------------------------------------------------------------- */
/* 真实 tick（SysTick 1ms 中断计数）                                */
/* -------------------------------------------------------------- */
static uint32_t tick_ms(void)
{
    return Delay_GetTick();
}

/* -------------------------------------------------------------- */
/* Cache 命中                                                     */
/* -------------------------------------------------------------- */
static TJC_CacheItem_t *find_or_alloc(const char *key, uint8_t is_str)
{
    TJC_CacheItem_t *empty = 0;
    for (int i = 0; i < TJC_CACHE_MAX_ITEMS; i++) {
        if (s_cache[i].used) {
            if (strncmp(s_cache[i].key, key, TJC_CACHE_KEY_LEN - 1) == 0)
                return &s_cache[i];
        } else if (!empty) empty = &s_cache[i];
    }
    if (!empty) return 0;
    memset(empty, 0, sizeof(*empty));
    strncpy(empty->key, key, TJC_CACHE_KEY_LEN - 1);
    empty->used   = 1;
    empty->is_str = is_str;
    return empty;
}

/* -------------------------------------------------------------- */
/* 事件分发（parser->events->此处）                                */
/* -------------------------------------------------------------- */
static void on_event(const TJC_Event_t *ev, void *user)
{
    (void)user;
    if (!ev) return;
    switch (ev->kind) {
    case TJC_EVT_TX_XFER_READY: s_xfer_ready_flag = 1; break;
    case TJC_EVT_TX_XFER_DONE:  s_xfer_done_flag  = 1; break;
    case TJC_EVT_CLICK:
        /* 按钮识别：HMI 端 b0/b1 弹起事件里用 printh 发自定义帧
         *   b0 周期按钮: printh 65 00 0C 00 FF FF FF  → obj_id=0x0C
         *   b1 频谱按钮: printh 65 00 0D 00 FF FF FF  → obj_id=0x0D
         *   页面就绪:    printh 65 00 00 02 FF FF FF  → obj_id=0x00 + event=0x02
         * parser 解析后 payload=[page_id,obj_id,event]，直接用 ev->obj_id 匹配
         */
        if (ev->event == 0x00 /* 弹起 */) {
            uint8_t cmd = ev->obj_id;
            if (cmd == 0x0C) {
                /* b0 周期切换 */
                s_period = (s_period == TJC_PERIOD_1) ? TJC_PERIOD_3 : TJC_PERIOD_1;
            } else if (cmd == 0x0D) {
                /* b1 频谱切换 */
                s_mode = (s_mode == TJC_MODE_TIME) ? TJC_MODE_SPEC : TJC_MODE_TIME;
                TJC_SetDisplayMode(s_mode);
            } else if (cmd == 0x00 && ev->payload_len >= 3 && ev->payload[2] == 0x02) {
                /* 页面就绪：event=0x02 */
                TJC_SetDisplayMode(s_mode);
            }
        }
        break;
    default:
        break;
    }
    if (s_app_cb) s_app_cb(ev);   /* 向上冒泡（如应用要记录日志）*/
}

/* -------------------------------------------------------------- */
/* 初始化                                                          */
/* -------------------------------------------------------------- */
void TJC_Client_Init(uint32_t baud, TJC_AppEventCB app_cb)
{
    memset(s_cache, 0, sizeof(s_cache));
    s_xfer_ready_flag = 0;
    s_xfer_done_flag  = 0;
    s_app_cb          = app_cb;
    s_mode            = TJC_MODE_TIME;
    s_period          = TJC_PERIOD_1;

    TJC_Transport_Init(baud);
    TJC_Parser_Init(&s_parser, TJC_Events_FromFrame, &s_events);
    TJC_Events_Init(&s_events, on_event, 0);
}

void TJC_Client_Task(void)
{
    TJC_Parser_PumpFromRB(&s_parser);
}

/* -------------------------------------------------------------- */
/* 节流 / 去重写值                                                 */
/* -------------------------------------------------------------- */
void TJC_SetMinRefreshMs(uint32_t ms)
{
    s_min_refresh_ms = ms;
}

static uint8_t throttle_ok(uint32_t *last_tick_ptr)
{
    uint32_t now = tick_ms();
    if (now - *last_tick_ptr < s_min_refresh_ms) return 0;
    *last_tick_ptr = now;
    return 1;
}

void TJC_SetNumber(const char *full_obj, int32_t value)
{
    if (!full_obj) return;
    TJC_CacheItem_t *ci = find_or_alloc(full_obj, 0);
    if (ci && ci->is_str == 0 && ci->used && ci->nval == value) {
        /* 同值去重，不发 */
        return;
    }
    if (!throttle_ok(&s_last_refresh_tick)) {
        /* 节流期内缓存但不发；下次不同值立即带最新值下发 */
        if (ci) ci->nval = value;
        return;
    }
    TJC_Cmd_SetNumber(full_obj, value);
    if (ci) ci->nval = value;
}

void TJC_SetText(const char *full_obj, const char *gb2312_text)
{
    if (!full_obj) return;
    if (!gb2312_text) gb2312_text = "";
    TJC_CacheItem_t *ci = find_or_alloc(full_obj, 1);
    if (ci && ci->is_str == 1 && strcmp(ci->sval, gb2312_text) == 0) return;
    if (!throttle_ok(&s_last_refresh_tick)) {
        if (ci) strncpy(ci->sval, gb2312_text, sizeof(ci->sval) - 1);
        return;
    }
    TJC_Cmd_SetText(full_obj, gb2312_text);
    if (ci) strncpy(ci->sval, gb2312_text, sizeof(ci->sval) - 1);
}

void TJC_FlushAll(void)
{
    /* 跳过节流，把缓存中所有"脏"对象再发一遍；实际实现为立即放行一次节流 */
    s_last_refresh_tick = 0;
}

/* -------------------------------------------------------------- */
/* 视图切换                                                        */
/* -------------------------------------------------------------- */
void TJC_SetDisplayMode(TJC_DispMode_t m)
{
    s_mode = m;
    if (m == TJC_MODE_TIME) {
        TJC_Cmd_Visibility("s0", 1);
        TJC_Cmd_Visibility("s1", 0);
    } else {
        TJC_Cmd_Visibility("s0", 0);
        TJC_Cmd_Visibility("s1", 1);
    }
}

TJC_DispMode_t TJC_GetDisplayMode(void) { return s_mode; }
TJC_Period_t   TJC_GetPeriodMode (void) { return s_period; }

/* -------------------------------------------------------------- */
/* 透传握手等待                                                    */
/* -------------------------------------------------------------- */
uint8_t TJC_WaitTransferReady(uint32_t timeout_ms)
{
    s_xfer_ready_flag = 0;
    uint32_t t0 = 0;
    while (!s_xfer_ready_flag) {
        TJC_Client_Task();
        Delay_ms(1);
        if (++t0 >= timeout_ms) return 0;
    }
    return 1;
}

uint8_t TJC_WaitTransferDone(uint32_t timeout_ms)
{
    s_xfer_done_flag = 0;
    uint32_t t0 = 0;
    while (!s_xfer_done_flag) {
        TJC_Client_Task();
        Delay_ms(1);
        if (++t0 >= timeout_ms) return 0;
    }
    return 1;
}

/* -------------------------------------------------------------- */
/* 波形/频谱 通道刷新                                              */
/* -------------------------------------------------------------- */
void TJC_RefreshWaveChannel(const char *curve_name,
                            const char *curve_id_expr,
                            uint8_t ch,
                            const uint8_t *y_values,
                            uint16_t qty,
                            uint16_t x_points_limit,
                            uint8_t  clear_before)
{
    if (!curve_name || !curve_id_expr || !y_values || !qty) return;
    if (qty > 1024) qty = 1024;   /* 单次透传上限 */

    if (x_points_limit > 0) {
        /* 设置曲线控件 s0.xnum = 显示点数（必须通过 attr_int 通用）*/
        char attr[32];
        int n = snprintf(attr, sizeof(attr), "%s.xnum", curve_name);
        if (n > 0 && n < (int)sizeof(attr))
            TJC_Cmd_SetAttrInt(attr, (int32_t)x_points_limit);
    }
    if (clear_before) {
        TJC_Cmd_ClearCurve(curve_name, 0);   /* 背景色 0=黑（如您的背景色不同请改）*/
    }

    if (qty == 1) {
        uint8_t v = y_values[0];
        if (v > TJC_WAVE_Y_MAX) v = (uint8_t)TJC_WAVE_Y_MAX;
        TJC_Cmd_AddPoint(curve_id_expr, ch, v);
        return;
    }

    /* qty>1：走 addt */
    s_xfer_ready_flag = 0;
    s_xfer_done_flag  = 0;
    TJC_Cmd_AddtStart(curve_id_expr, ch, qty);

    /* 等待 FE；屏手册说准备约 5ms，实际缓冲满可能更久 */
    if (!TJC_WaitTransferReady(100)) {
        /* 关键：超时 = 屏没准备好（可能 objname 错 / 屏端 busy）。
         * 立刻发 3 个 0xFF 强制屏退出 addt 等待命令态，
         * 否则后续 val=xxx 命令会被当作曲线数据字节吞掉 → 数字再也不刷新 */
        uint8_t reset_bytes[3] = {0xFF, 0xFF, 0xFF};
        TJC_TxRaw(reset_bytes, 3);
        return;
    }

    /* 发送 qty 字节，限幅到 WAVE_Y_MAX */
    for (uint16_t i = 0; i < qty; i++) {
        uint8_t v = y_values[i];
        if (v > TJC_WAVE_Y_MAX) v = (uint8_t)TJC_WAVE_Y_MAX;
        TJC_TxByte(v);
    }
    /* 发送完成后，屏会回复 FD（必须等待并吃掉，避免后续帧污染）*/
    if (!TJC_WaitTransferDone(200)) {
        /* 同样，超时后强制清命令态，防止下一条命令被吞 */
        uint8_t reset_bytes[3] = {0xFF, 0xFF, 0xFF};
        TJC_TxRaw(reset_bytes, 3);
    }
}

/* -------------------------------------------------------------- */
/* 带超时保护的波形刷新                                            */
/* -------------------------------------------------------------- */
uint8_t TJC_RefreshWaveChannel_Try(const char *curve_name,
                                   const char *curve_id_expr,
                                   uint8_t ch,
                                   const uint8_t *y_values,
                                   uint16_t qty,
                                   uint16_t x_points_limit,
                                   uint8_t  clear_before,
                                   uint32_t timeout_ms)
{
    if (!curve_name || !curve_id_expr || !y_values || !qty) return 0;
    if (qty > 1024) qty = 1024;
    if (timeout_ms < 10) timeout_ms = 10;

    if (x_points_limit > 0) {
        char attr[32];
        int n = snprintf(attr, sizeof(attr), "%s.xnum", curve_name);
        if (n > 0 && n < (int)sizeof(attr))
            TJC_Cmd_SetAttrInt(attr, (int32_t)x_points_limit);
    }
    if (clear_before) {
        TJC_Cmd_ClearCurve(curve_name, 0);
    }

    if (qty == 1) {
        uint8_t v = y_values[0];
        if (v > TJC_WAVE_Y_MAX) v = (uint8_t)TJC_WAVE_Y_MAX;
        TJC_Cmd_AddPoint(curve_id_expr, ch, v);
        return 1;   /* 单点立即成功 */
    }

    /* qty>1：走 addt，使用用户指定的短超时 */
    s_xfer_ready_flag = 0;
    s_xfer_done_flag  = 0;
    TJC_Cmd_AddtStart(curve_id_expr, ch, qty);

    /* 等待 FE：使用短超时 */
    uint32_t half_timeout = timeout_ms / 2;
    if (half_timeout < 5) half_timeout = 5;

    if (!TJC_WaitTransferReady(half_timeout)) {
        /* 超时：不要发 0xFF 0xFF 0xFF！这会破坏解析器状态
         * 直接清空接收缓冲区，返回失败 */
        TJC_RB_Clear();
        return 0;
    }

    /* 发送数据 */
    for (uint16_t i = 0; i < qty; i++) {
        uint8_t v = y_values[i];
        if (v > TJC_WAVE_Y_MAX) v = (uint8_t)TJC_WAVE_Y_MAX;
        TJC_TxByte(v);
    }

    /* 等待 FD：使用剩余超时 */
    if (!TJC_WaitTransferDone(half_timeout)) {
        TJC_RB_Clear();
        return 0;
    }

    return 1;   /* 成功 */
}

void TJC_RefreshSpectrum(const char *curve_name,
                         const char *curve_id_expr,
                         uint8_t ch,
                         const float *magnitude,
                         uint16_t bins,
                         uint16_t start_bin,
                         uint16_t show_bins,
                         uint16_t x_points_limit)
{
    if (!magnitude || !bins || !show_bins) return;
    if (start_bin >= bins) return;
    if (start_bin + show_bins > bins) show_bins = (uint16_t)(bins - start_bin);
    if (show_bins > 1024) show_bins = 1024;

    /* 1) 归一化 */
    float fmax = 1e-12f;
    for (uint16_t i = 0; i < show_bins; i++) {
        float m = magnitude[start_bin + i];
        if (m > fmax) fmax = m;
    }
    static uint8_t ybuf[1024];   /* 单次 1024 上限 */
    float scale = (float)TJC_FFT_Y_MAX / fmax;
    for (uint16_t i = 0; i < show_bins; i++) {
        float v = magnitude[start_bin + i] * scale;
        if (v < 0) v = 0;
        if (v > (float)TJC_FFT_Y_MAX) v = (float)TJC_FFT_Y_MAX;
        /* 取 round，uint8 安全 */
        ybuf[i] = (uint8_t)(v + 0.5f);
    }
    /* 2) 发送：先清，再 addt 满 */
    TJC_RefreshWaveChannel(curve_name, curve_id_expr, ch, ybuf, show_bins, x_points_limit, 1);
}

/* -------------------------------------------------------------- */
/* 统计                                                            */
/* -------------------------------------------------------------- */
uint32_t TJC_GetNoiseCount(void) { return s_parser.noise_cnt; }
uint32_t TJC_GetFrameCount(void) { return s_parser.frame_cnt; }
uint32_t TJC_GetRBOverflow(void) { return TJC_RB_OverflowCount(); }
