//
// Created by 35037 on 2026/9/26.
//
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef BYDCAR_G491VET6_NX_UART_H
#define BYDCAR_G491VET6_NX_UART_H
extern uint8_t rx4;

/* ---- K230 模式常量 ---- */
// #define K230_MODE_LINEL      'l'   /* 循迹模式 */
// #define K230_MODE_LINER      'r'
#define NX_MODE_CIRCLE    'c'   /* 绕圈模式 */
#define NX_MODE_STOP      'x'   /* 停止（匹配 K230 Python） */

/* ==================== 模式管理 ==================== */
void NX_Init(void);
void NX_RequestMode(uint8_t mode);
void NX_ApplyMode(void);
void NX_SetMode(uint8_t mode);

/* ==================== 数据读取 ==================== */
bool NX_GetLineAngle(float *angle);
bool NX_GetCircleDir(char *dir);
bool NX_GetPosition(float *x, float *y);
bool NX_GetCirclepos(float *cx,float *cy);
void NX_GetDiag(uint32_t *rx_bytes, uint32_t *rx_ok,
                  uint32_t *rx_err, uint32_t *rx_unk);

/* ==================== ISR 接口 ==================== */
void NX_RxProcessByte(void);   /* HAL_UART_RxCpltCallback 中调用 */
void NX_RxRestart(void);       /* HAL_UART_ErrorCallback 中调用 */

#ifdef __cplusplus
}
#endif

#endif //BYDCAR_G491VET6_NX_UART_H