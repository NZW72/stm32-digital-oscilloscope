#ifndef __TJC_HMI_CODEC_H
#define __TJC_HMI_CODEC_H

#include "stm32f4xx.h"

/* ================================================================
 * TJC HMI Codec Layer (Layer 2)
 * 负责：把高层操作编码成 ASCII 指令 + FF FF FF 结束符。
 * 全部 API 产出完整帧（不含 CR/LF），然后调用 transport 发送。
 * 硬约束：
 *   - 文本属性用双引号：t0.txt="xxx"
 *   - 数值属性不加引号：n0.val=123
 *   - 指令/赋值/表达式不要多余空格，不要分号
 * ================================================================ */

/* ---------- 基础：发送任意 ASCII 命令帧 ---------- */
void TJC_Cmd_SendRawString(const char *ascii_cmd);

/* ---------- 控件属性 ---------- */
void TJC_Cmd_SetText   (const char *full_obj, const char *gb2312_text);
void TJC_Cmd_SetNumber (const char *full_obj, int32_t value);
void TJC_Cmd_SetAttrInt(const char *full_attr, int32_t value);
void TJC_Cmd_SetAttrStr(const char *full_attr, const char *gb2312_text);

/* ---------- 显隐 / 跳转 / bkcmd / 亮度 ---------- */
void TJC_Cmd_Visibility(const char *objname, uint8_t show);   /* objname: 如 "s0" */
void TJC_Cmd_Page      (const char *page_name);
void TJC_Cmd_Bkcmd     (uint8_t level);                      /* 0/1/2/3 */
void TJC_Cmd_Dim       (uint8_t percent);                    /* 0~100 */
void TJC_Cmd_Ref       (const char *objname);                /* 强制刷新 */

/* ---------- 波形控件 add / addt / cle ---------- */
/* add s0.id,ch,val    val 0..255,且必须 < 控件像素高度 */
void TJC_Cmd_AddPoint  (const char *curve_id_expr, uint8_t ch, uint8_t val);

/* cle s0, bgcolor */
void TJC_Cmd_ClearCurve(const char *curve_name, uint8_t bgcolor_255);

/* addt s0.id,ch,qty  ；后续需等 0xFE 后送 qty 字节，再收 0xFD */
void TJC_Cmd_AddtStart (const char *curve_id_expr, uint8_t ch, uint16_t qty);

/* 透传期间：直接送 qty 字节（不走 codec 加尾，由调用方在收 0xFE 后 TxRaw）*/

/* ---------- 回读 ---------- */
void TJC_Cmd_Get       (const char *full_attr);   /* get page0.n0.val -> 回 0x71 或 0x70 */

/* ---------- 波特率（baud 临时，bauds 写Flash 只允许发一次！）---------- */
void TJC_Cmd_Baud_Transient(uint32_t baud);
void TJC_Cmd_Baud_SaveFLASH(uint32_t baud);   /* 外部必须加"仅限一次"保护 */

#endif
