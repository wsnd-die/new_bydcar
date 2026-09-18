/**
 * @file k230.h
 * @brief K230 视觉模块通信 — USART3 二进制协议
 *        TX: 单字符命令 'f'/'c'/'x'
 *        RX: 0xB3 [aH][aL] 0xFF             — 角度
 *            0xB3 [aH][aL][xH][xL][yH][yL] 0xFF — 角度+位置 (9字节)
 *            0xB4 [dir] 0xFF                 — 方向
 *        PB10=TX, PB11=RX
 */

#ifndef K230_H
#define K230_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- 跨文件共享的 USART3 接收缓冲（在 k230.c 中定义） ---- */
extern uint8_t rx3;

/* ---- K230 模式常量 ---- */
#define K230_MODE_LINEL      'l'   /* 循迹模式 */
#define K230_MODE_LINER      'r'
#define K230_MODE_CIRCLE    'c'   /* 绕圈模式 */
#define K230_MODE_STOP      'x'   /* 停止（匹配 K230 Python） */

/* ==================== 模式管理 ==================== */
void K230_Init(void);
void K230_RequestMode(uint8_t mode);
void K230_ApplyMode(void);
void K230_SetMode(uint8_t mode);

/* ==================== 数据读取 ==================== */
bool K230_GetLineAngle(float *angle);
bool K230_GetCircleDir(char *dir);
bool K230_GetPosition(float *x, float *y);
bool K230_GetCirclepos(float *cx,float *cy);
void K230_GetDiag(uint32_t *rx_bytes, uint32_t *rx_ok,
                  uint32_t *rx_err, uint32_t *rx_unk);

/* ==================== ISR 接口 ==================== */
void K230_RxProcessByte(void);   /* HAL_UART_RxCpltCallback 中调用 */
void K230_RxRestart(void);       /* HAL_UART_ErrorCallback 中调用 */

#ifdef __cplusplus
}
#endif

#endif
