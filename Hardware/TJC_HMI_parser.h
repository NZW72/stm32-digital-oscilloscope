#ifndef __TJC_HMI_PARSER_H
#define __TJC_HMI_PARSER_H

#include "stm32f4xx.h"

/* ================================================================
 * TJC HMI Parser Layer (Layer 3)
 * 增量字节流状态机解帧，解决：半包、粘包、噪声、并发帧。
 *
 * 按官方 wiki：
 *   0x65 + pageID(1) + objID(1) + event(1) + FF FF FF   点击
 *   0x70 + ASCII[...] + FF FF FF                          字符串变量
 *   0x71 + int32 LE(4)  + FF FF FF                        数值变量
 *   0x00/0x01/0x02../0x24 + FF FF FF                      bkcmd/系统通知
 *   0x86/0x87/0x88/0xFE/0xFD + FF FF FF                   系统事件
 *
 * 策略：
 *   - 每次喂入 1~N 字节，内部维护状态
 *   - 收齐 3 个 0xFF 才宣告一帧完成
 *   - 对 0x65 要求 payload==3
 *   - 对 0x71 要求 payload==4
 *   - 其他帧 payload 任意长度
 *   - 遇到"非 FF 的结束符疑似"+后面不满足 FF FF：丢弃重来（重同步）
 * ================================================================ */

#define TJC_PAYLOAD_MAX     256    /* 安全上界 */

typedef enum {
    TJC_FRAME_NONE = 0,
    TJC_FRAME_CLICK,          /* 0x65 : payload[0]=page, [1]=obj, [2]=event(0弹/1压) */
    TJC_FRAME_STRING,         /* 0x70 : payload = ASCII */
    TJC_FRAME_NUMBER,         /* 0x71 : payload = 4B LE int32 */
    TJC_FRAME_RESULT,         /* 0x00/0x01/0x02..0x24 ： code = 首字节 */
    TJC_FRAME_SYS             /* 0x86/0x87/0x88/0xFE/0xFD …: code = 首字节 */
} TJC_FrameKind_t;

typedef struct
{
    TJC_FrameKind_t kind;
    uint8_t  code;             /* RESULT/SYS: 首字节；CLICK: event */
    uint16_t payload_len;
    uint8_t  payload[TJC_PAYLOAD_MAX];
    /* 辅助：CLICK 专用 */
    uint8_t  page_id;
    uint8_t  obj_id;
    uint8_t  event;            /* 0=弹起 1=按下 */
    /* 辅助：NUMBER 专用 */
    int32_t  number;
} TJC_Frame_t;

/* 回调：每解出一帧调用一次（在 parser_feed 调用上下文中）*/
typedef void (*TJC_FrameHandler_t)(const TJC_Frame_t *frm, void *user);

typedef struct
{
    /* 状态机 */
    enum {
        P_IDLE = 0,
        P_INFRAME,
        P_END1,   /* 见过 1 个 FF */
        P_END2    /* 见过连续 2 个 FF */
    } state;

    uint8_t  sof;                 /* 首字节 */
    uint16_t plen;                /* payload 已写入长度 */
    uint8_t  pbuf[TJC_PAYLOAD_MAX];

    TJC_FrameHandler_t handler;
    void              *user;
    uint32_t           noise_cnt; /* 异常重同步次数 */
    uint32_t           frame_cnt; /* 成功解帧计数 */
} TJC_Parser_t;

void TJC_Parser_Init(TJC_Parser_t *p, TJC_FrameHandler_t cb, void *user);

/* 主循环调用：从 ringbuffer 读出多少喂多少 */
void TJC_Parser_Feed(TJC_Parser_t *p, const uint8_t *data, uint16_t len);

/* 便捷：内部从 transport RB 读并解析一次 */
void TJC_Parser_PumpFromRB(TJC_Parser_t *p);

#endif
