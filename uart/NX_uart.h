//
// Created by 35037 on 2026/9/26.
//
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef BYDCAR_G491VET6_NX_UART_H
#define BYDCAR_G491VET6_NX_UART_H

/* CubeMX 在 Core/Src/usart.c 生成的 UART4 RX DMA 句柄 (DMA1_Channel5)。
 * 本驱动用它在每次重挂接收后关掉半传输中断，见 NX_StartRx()。
 * 声明方式沿用 hardware/bus/uart2_tbop10.h 对 hdma_usart2_rx 的先例。 */
extern DMA_HandleTypeDef hdma_uart4_rx;

/* ---- K230 模式常量 ---- */
// #define K230_MODE_LINEL      'l'   /* 循迹模式 */
// #define K230_MODE_LINER      'r'
#define NX_MODE_CIRCLE    'c'   /* 绕圈模式 */
#define NX_MODE_YOLO      'y'
#define NX_MODE_STOP      'x'   /* 停止（匹配 K230 Python） */

/* ==================== 模式管理 ==================== */
void NX_Init(void);
void NX_RequestMode(uint8_t mode);
void NX_ApplyMode(void);
void NX_SetMode(uint8_t mode);

/* ==================== 数据读取 ==================== */
// bool NX_GetLineAngle(float *angle);   /* 实现已注释，暂不提供 */
bool NX_GetTrophyRank(char *rank);
/** @brief B3(名次)帧累计收到多少帧 (只增不减)。排查"窗口外丢帧"用, 见 .c 里的说明。 */
uint32_t NX_GetTrophyCount(void);
bool NX_GetCircleDir(char *dir);
bool NX_GetPosition(float *x, float *y);
bool NX_GetCirclepos(float *cx,float *cy);
void NX_GetDiag(uint32_t *rx_bytes, uint32_t *rx_ok,
                  uint32_t *rx_err, uint32_t *rx_unk);

/* ==================== ISR 接口 ==================== */
/* 由 Core/Src/usart.c 的 HAL_UARTEx_RxEventCallback() 分发器调用，逐字节喂状态机。 */
void NX_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void NX_RxProcessByte(uint8_t b);
void NX_ErrorCallback(void);       /* HAL_UART_ErrorCallback 中调用 */

#ifdef __cplusplus
}
#endif

#endif //BYDCAR_G491VET6_NX_UART_H