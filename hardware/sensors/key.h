/**
 * @file    key.h
 * @brief   按键消抖 —— PB0 限位开关 / PA0 启动开关
 *
 * 用法: 固定节拍 (建议 10ms) 调 Key_Update(), 再用 Key_IsPressed / Key_WasPressed 查询;
 * 长按用 Key_WasLongPressed。
 *
 * @note  两个脚沿用 gpio.c 的默认初始化 (输入, 不带上拉/下拉)。接机械开关时若不外加
 *        上下拉电阻, 悬空读数是随机跳变的 —— 消抖只滤抖动, 滤不掉悬空。
 */

#ifndef KEY_H
#define KEY_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/** 连续多少拍读到相同电平才认账。按 10ms 节拍调用时 = 50ms 消抖窗口。 */
#define KEY_DEBOUNCE_SAMPLES   5u

/** 按下时的电平: 1 = 按下读到高, 0 = 按下读到低。 */
#define KEY_ACTIVE_LEVEL       1u

/** 长按判据: 稳定按下持续多少拍。按 10ms 节拍调用时 = 1000ms。 */
#define KEY_LONG_PRESS_TICKS   20u

typedef enum {
    KEY_LIMIT = 0,   /* PB0 限位开关 */
    KEY_START,       /* PA0 启动开关 */
    KEY_COUNT
} KeyId_t;

void Key_Init(void);
void Key_Update(void);
bool Key_IsPressed(KeyId_t k);
bool Key_WasPressed(KeyId_t k);   /* 按下边沿, 读后清 */
bool Key_WasReleased(KeyId_t k);  /* 抬起边沿, 读后清 */
bool Key_WasLongPressed(KeyId_t k); /* 长按事件 (稳定按下持续 KEY_LONG_PRESS_TICKS 拍), 读后清 */

#endif /* KEY_H */
