#include "TJC_HMI_parser.h"
#include "TJC_HMI_transport.h"
#include <string.h>

void TJC_Parser_Init(TJC_Parser_t *p, TJC_FrameHandler_t cb, void *user)
{
    if (!p) return;
    memset(p, 0, sizeof(*p));
    p->handler = cb;
    p->user    = user;
}

/* 根据首字节判帧类型 */
static TJC_FrameKind_t classify(uint8_t sof, uint8_t *out_code)
{
    /* 0x00..0x24 保留给 RESULT；但 0x1..0x24 部分未完全占用，
     * 这里按官方 return/index.html 里明确列出的 code 视作 RESULT，
     * 其余 code 若与已知的 0x65/70/71/8x/FD/FE 不冲突则默认归类 RESULT 即可 */
    if (sof == 0x65) return TJC_FRAME_CLICK;
    if (sof == 0x70) return TJC_FRAME_STRING;
    if (sof == 0x71) return TJC_FRAME_NUMBER;
    if (sof == 0x86 || sof == 0x87 || sof == 0x88 ||
        sof == 0xFE || sof == 0xFD) {
        if (out_code) *out_code = sof;
        return TJC_FRAME_SYS;
    }
    if (out_code) *out_code = sof;
    return TJC_FRAME_RESULT;
}

static int32_t le4_to_s32(const uint8_t *b)
{
    uint32_t u = (uint32_t)b[0]
               | ((uint32_t)b[1] << 8)
               | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 24);
    return (int32_t)u;
}

static void emit(TJC_Parser_t *p)
{
    TJC_Frame_t f;
    memset(&f, 0, sizeof(f));
    f.kind = classify(p->sof, &f.code);
    switch (f.kind) {
    case TJC_FRAME_CLICK:
        if (p->plen < 3) return;     /* 0x65 需要 3B payload */
        f.page_id = p->pbuf[0];
        f.obj_id  = p->pbuf[1];
        f.event   = p->pbuf[2];
        f.code    = f.event;
        break;
    case TJC_FRAME_NUMBER:
        if (p->plen < 4) return;     /* 0x71 需要 4B int */
        f.number = le4_to_s32(p->pbuf);
        break;
    case TJC_FRAME_STRING:
    case TJC_FRAME_RESULT:
    case TJC_FRAME_SYS:
    default:
        break;
    }
    f.payload_len = p->plen > TJC_PAYLOAD_MAX ? TJC_PAYLOAD_MAX : p->plen;
    memcpy(f.payload, p->pbuf, f.payload_len);

    p->frame_cnt++;
    if (p->handler) p->handler(&f, p->user);
}

static void reset_state(TJC_Parser_t *p)
{
    p->state = P_IDLE;
    p->sof   = 0;
    p->plen  = 0;
}

void TJC_Parser_Feed(TJC_Parser_t *p, const uint8_t *data, uint16_t len)
{
    if (!p || !data || !len) return;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        switch (p->state) {
        case P_IDLE:
            if (b == 0xFF) {
                /* 噪声：前导 FF 忽略并计数 */
                p->noise_cnt++;
                break;
            }
            p->sof  = b;
            p->plen = 0;
            p->state = P_INFRAME;
            break;

        case P_INFRAME:
            if (b == 0xFF) {
                p->state = P_END1;
            } else {
                if (p->plen >= TJC_PAYLOAD_MAX) {
                    /* 溢出：重同步 */
                    p->noise_cnt++;
                    reset_state(p);
                } else {
                    p->pbuf[p->plen++] = b;
                }
            }
            break;

        case P_END1:
            if (b == 0xFF) {
                p->state = P_END2;
            } else {
                /* 之前的 FF 是 payload，落回 P_INFRAME */
                if (p->plen + 1 >= TJC_PAYLOAD_MAX) {
                    p->noise_cnt++;
                    reset_state(p);
                } else {
                    p->pbuf[p->plen++] = 0xFF;
                    p->pbuf[p->plen++] = b;
                    p->state = P_INFRAME;
                }
            }
            break;

        case P_END2:
            if (b == 0xFF) {
                /* 第三个 FF -> 帧结束 */
                emit(p);
                reset_state(p);
            } else {
                /* 前两个 FF 其实是 payload */
                if (p->plen + 2 >= TJC_PAYLOAD_MAX) {
                    p->noise_cnt++;
                    reset_state(p);
                } else {
                    p->pbuf[p->plen++] = 0xFF;
                    p->pbuf[p->plen++] = 0xFF;
                    p->pbuf[p->plen++] = b;
                    p->state = P_INFRAME;
                }
            }
            break;
        }
    }
}

void TJC_Parser_PumpFromRB(TJC_Parser_t *p)
{
    if (!p) return;
    static uint8_t chunk[128];
    for (;;) {
        uint16_t n = TJC_RB_Read(chunk, sizeof(chunk));
        if (n == 0) break;
        TJC_Parser_Feed(p, chunk, n);
        if (n < sizeof(chunk)) break;
    }
}
