#ifndef __TJC_HMI_EVENTS_H
#define __TJC_HMI_EVENTS_H

#include "TJC_HMI_parser.h"

/* ================================================================
 * TJC HMI Events Layer (Layer 4)
 * 把 parser 给出的 TJC_Frame_t 分发为高层事件。
 * 应用层通过注册回调来接收事件。
 * ================================================================ */

typedef enum {
    TJC_EVT_NONE = 0,
    TJC_EVT_CLICK,           /* 0x65 按键弹/压：page/obj/event + payload*/
    TJC_EVT_STRING,          /* 0x70 字符串回读 */
    TJC_EVT_NUMBER,          /* 0x71 数值回读 */
    TJC_EVT_RESULT_OK,       /* bkcmd: 指令成功 (0x01) */
    TJC_EVT_RESULT_ERR,      /* bkcmd: 指令失败 (0x00, 0x02..0x24) */
    TJC_EVT_SYS_BOOT,        /* 0x88 启动成功 */
    TJC_EVT_SYS_SLEEP,       /* 0x86 自动睡眠 */
    TJC_EVT_SYS_WAKE,        /* 0x87 自动唤醒 */
    TJC_EVT_TX_XFER_READY,   /* 0xFE addt 透传就绪 */
    TJC_EVT_TX_XFER_DONE,    /* 0xFD addt 透传完成 */
    TJC_EVT_SYS_OTHER        /* 其他系统 code */
} TJC_EventKind_t;

typedef struct
{
    TJC_EventKind_t kind;
    uint8_t  code;             /* 原 code / event / bkcmd_errcode */
    /* CLICK */
    uint8_t  page_id;
    uint8_t  obj_id;
    uint8_t  event;            /* 0=弹起 1=按下 2..自定义 */
    /* STRING */
    const uint8_t *str;        /* 非零结尾，长度看 payload_len */
    uint16_t       str_len;
    /* NUMBER */
    int32_t  number;
    /* RAW payload */
    const uint8_t *payload;
    uint16_t       payload_len;
} TJC_Event_t;

/* 应用层回调（parser_feed 线程中触发）*/
typedef void (*TJC_EventCallback_t)(const TJC_Event_t *ev, void *user);

typedef struct
{
    TJC_EventCallback_t  cb;
    void                *user;
} TJC_Events_t;

void TJC_Events_Init(TJC_Events_t *e, TJC_EventCallback_t app_cb, void *app_user);

/* 给 parser 用的统一 handler：parser.handler = TJC_Events_FromFrame;
 * parser.user    = &TJC_Events_t 实例 */
void TJC_Events_FromFrame(const TJC_Frame_t *frm, void *user);

#endif
