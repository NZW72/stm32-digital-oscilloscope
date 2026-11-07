#include "TJC_HMI_events.h"

void TJC_Events_Init(TJC_Events_t *e, TJC_EventCallback_t app_cb, void *app_user)
{
    if (!e) return;
    e->cb   = app_cb;
    e->user = app_user;
}

static TJC_EventKind_t classify_result(uint8_t code)
{
    if (code == 0x01) return TJC_EVT_RESULT_OK;
    return TJC_EVT_RESULT_ERR;   /* 0x00 及 0x02..0x24 均当失败 */
}

static TJC_EventKind_t classify_sys(uint8_t code)
{
    switch (code) {
    case 0x88: return TJC_EVT_SYS_BOOT;
    case 0x86: return TJC_EVT_SYS_SLEEP;
    case 0x87: return TJC_EVT_SYS_WAKE;
    case 0xFE: return TJC_EVT_TX_XFER_READY;
    case 0xFD: return TJC_EVT_TX_XFER_DONE;
    default:   return TJC_EVT_SYS_OTHER;
    }
}

void TJC_Events_FromFrame(const TJC_Frame_t *frm, void *user)
{
    if (!frm || !user) return;
    TJC_Events_t *e = (TJC_Events_t *)user;
    TJC_Event_t ev;
    ev.kind = TJC_EVT_NONE;
    ev.code = frm->code;
    ev.page_id = frm->page_id;
    ev.obj_id  = frm->obj_id;
    ev.event   = frm->event;
    ev.str     = frm->payload;
    ev.str_len = frm->payload_len;
    ev.number  = frm->number;
    ev.payload     = frm->payload;
    ev.payload_len = frm->payload_len;

    switch (frm->kind) {
    case TJC_FRAME_CLICK:   ev.kind = TJC_EVT_CLICK;              break;
    case TJC_FRAME_STRING:  ev.kind = TJC_EVT_STRING;             break;
    case TJC_FRAME_NUMBER:  ev.kind = TJC_EVT_NUMBER;             break;
    case TJC_FRAME_RESULT:  ev.kind = classify_result(frm->code); break;
    case TJC_FRAME_SYS:     ev.kind = classify_sys(frm->code);    break;
    default: break;
    }

    if (e->cb) e->cb(&ev, e->user);
}
