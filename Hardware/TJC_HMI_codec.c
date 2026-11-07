#include "TJC_HMI_codec.h"
#include "TJC_HMI_transport.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* 安全发送缓冲；单条最长 256 字节足够 */
#ifndef TJC_CMD_MAX
#define TJC_CMD_MAX     256
#endif

static char s_scratch[TJC_CMD_MAX];

/* 把拼接好的 ASCII 指令（不含结束符）加尾发送 */
static void send_ascii(const char *s)
{
    if (!s) return;
    uint16_t n = 0;
    while (s[n] && n < TJC_CMD_MAX) n++;
    TJC_TxFrame((const uint8_t *)s, n);
}

/* 安全拼接：返回 <0 表示溢出丢弃 */
static int safe_cat(char *dst, uint16_t dstsz, const char *a, const char *b)
{
    if (!dst || !dstsz || !a) return -1;
    uint16_t la = (uint16_t)strlen(a);
    if (la >= dstsz) return -1;
    memcpy(dst, a, la);
    if (b) {
        uint16_t lb = (uint16_t)strlen(b);
        if (la + lb >= dstsz) return -1;
        memcpy(dst + la, b, lb);
        la += lb;
    }
    dst[la] = 0;
    return (int)la;
}

/* -------------------------------------------------------------- */
/* 命令类                                                          */
/* -------------------------------------------------------------- */
void TJC_Cmd_SendRawString(const char *ascii_cmd)
{
    send_ascii(ascii_cmd);
}

void TJC_Cmd_SetText(const char *full_obj, const char *gb2312_text)
{
    if (!full_obj) return;
    if (!gb2312_text) gb2312_text = "";
    /* 文本属性：obj.txt="str" */
    if (safe_cat(s_scratch, sizeof(s_scratch), full_obj, ".txt=\"") < 0) return;
    uint16_t n = (uint16_t)strlen(s_scratch);
    const char *p = gb2312_text;
    while (*p && n + 1 < sizeof(s_scratch) - 2) { /* 预留 2: " + \0 */
        char c = *p++;
        /* 内部双引号用转义 \" （HMI 语法见官方 return 20 FF FF FF 转义规则）*/
        if (c == '"' && n + 2 < sizeof(s_scratch) - 2) {
            s_scratch[n++] = '\\';
            s_scratch[n++] = '"';
        } else {
            s_scratch[n++] = c;
        }
    }
    s_scratch[n++] = '"';
    s_scratch[n]   = 0;
    send_ascii(s_scratch);
}

void TJC_Cmd_SetNumber(const char *full_obj, int32_t value)
{
    if (!full_obj) return;
    /* obj.val=123 */
    int n = snprintf(s_scratch, sizeof(s_scratch), "%s.val=%ld", full_obj, (long)value);
    if (n > 0 && n < (int)sizeof(s_scratch)) send_ascii(s_scratch);
}

void TJC_Cmd_SetAttrInt(const char *full_attr, int32_t value)
{
    if (!full_attr) return;
    int n = snprintf(s_scratch, sizeof(s_scratch), "%s=%ld", full_attr, (long)value);
    if (n > 0 && n < (int)sizeof(s_scratch)) send_ascii(s_scratch);
}

void TJC_Cmd_SetAttrStr(const char *full_attr, const char *gb2312_text)
{
    if (!full_attr) return;
    if (!gb2312_text) gb2312_text = "";
    if (safe_cat(s_scratch, sizeof(s_scratch), full_attr, "=\"") < 0) return;
    uint16_t n = (uint16_t)strlen(s_scratch);
    const char *p = gb2312_text;
    while (*p && n + 1 < sizeof(s_scratch) - 2) {
        char c = *p++;
        if (c == '"' && n + 2 < sizeof(s_scratch) - 2) {
            s_scratch[n++] = '\\';
            s_scratch[n++] = '"';
        } else {
            s_scratch[n++] = c;
        }
    }
    s_scratch[n++] = '"';
    s_scratch[n] = 0;
    send_ascii(s_scratch);
}

void TJC_Cmd_Visibility(const char *objname, uint8_t show)
{
    if (!objname) return;
    int n = snprintf(s_scratch, sizeof(s_scratch), "vis %s,%d", objname, show ? 1 : 0);
    if (n > 0) send_ascii(s_scratch);
}

void TJC_Cmd_Page(const char *page_name)
{
    if (!page_name) return;
    int n = snprintf(s_scratch, sizeof(s_scratch), "page %s", page_name);
    if (n > 0) send_ascii(s_scratch);
}

void TJC_Cmd_Bkcmd(uint8_t level)
{
    if (level > 3) level = 3;
    int n = snprintf(s_scratch, sizeof(s_scratch), "bkcmd=%d", (int)level);
    if (n > 0) send_ascii(s_scratch);
}

void TJC_Cmd_Dim(uint8_t percent)
{
    if (percent > 100) percent = 100;
    int n = snprintf(s_scratch, sizeof(s_scratch), "dim=%d", (int)percent);
    if (n > 0) send_ascii(s_scratch);
}

void TJC_Cmd_Ref(const char *objname)
{
    if (!objname) return;
    int n = snprintf(s_scratch, sizeof(s_scratch), "ref %s", objname);
    if (n > 0) send_ascii(s_scratch);
}

/* -------------------------------------------------------------- */
/* 波形/曲线                                                       */
/* -------------------------------------------------------------- */
void TJC_Cmd_AddPoint(const char *curve_id_expr, uint8_t ch, uint8_t val)
{
    if (!curve_id_expr) return;
    if (ch > 3) return;          /* 每曲线最多 4 通道 */
    int n = snprintf(s_scratch, sizeof(s_scratch), "add %s,%d,%d",
                     curve_id_expr, (int)ch, (int)val);
    if (n > 0) send_ascii(s_scratch);
}

void TJC_Cmd_ClearCurve(const char *curve_name, uint8_t bgcolor_255)
{
    if (!curve_name) return;
    int n = snprintf(s_scratch, sizeof(s_scratch), "cle %s,%d", curve_name, (int)bgcolor_255);
    if (n > 0) send_ascii(s_scratch);
}

void TJC_Cmd_AddtStart(const char *curve_id_expr, uint8_t ch, uint16_t qty)
{
    if (!curve_id_expr || !qty) return;
    if (ch > 3) return;
    if (qty > 1024) qty = 1024;    /* 官方单次透传最大 1024 */
    int n = snprintf(s_scratch, sizeof(s_scratch), "addt %s,%d,%u",
                     curve_id_expr, (int)ch, (unsigned)qty);
    if (n > 0) send_ascii(s_scratch);
}

/* -------------------------------------------------------------- */
/* 回读                                                            */
/* -------------------------------------------------------------- */
void TJC_Cmd_Get(const char *full_attr)
{
    if (!full_attr) return;
    int n = snprintf(s_scratch, sizeof(s_scratch), "get %s", full_attr);
    if (n > 0) send_ascii(s_scratch);
}

/* -------------------------------------------------------------- */
/* 波特率                                                          */
/* -------------------------------------------------------------- */
void TJC_Cmd_Baud_Transient(uint32_t baud)
{
    int n = snprintf(s_scratch, sizeof(s_scratch), "baud=%lu", (unsigned long)baud);
    if (n > 0) send_ascii(s_scratch);
}

/* 注意：此函数会写屏端 Flash，必须被外部调用者严格限制只能调用 1 次/生命周期 */
static uint8_t s_bauds_saved = 0;
void TJC_Cmd_Baud_SaveFLASH(uint32_t baud)
{
    if (s_bauds_saved) return;
    s_bauds_saved = 1;
    int n = snprintf(s_scratch, sizeof(s_scratch), "bauds=%lu", (unsigned long)baud);
    if (n > 0) send_ascii(s_scratch);
}
