/**
 * @file grayscale.h
 * @brief 八通道灰度循迹传感器串行驱动 — PA4=CLK 输出, PA5=DAT 输入
 *
 * 硬件: 感为八通道灰度循迹传感器 (串行输出版, 5V 供电共地)
 * 时序: CLK 拉低 → 读 DAT (1bit) → CLK 拉高, 重复 8 次, 先读出的是通道0
 */

#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ======================== 引脚定义 (改线改这里) ======================== */
#define GRAY_CLK_GPIO_Port   GPIOA
#define GRAY_CLK_Pin         GPIO_PIN_4   /* CLK 输出 (PA4 输出仍正常, 保留) */
#define GRAY_DAT_GPIO_Port   GPIOB
#define GRAY_DAT_Pin         GPIO_PIN_7   /* DAT 输入 (原 PA5 烧坏, 挪到 PB7=FT 5V耐受) */

/* 串行时钟半周期 (us), 对应 ~100kHz CLK。过快可能误读, 可调 */
#define GRAY_CLK_HALF_PERIOD_US   5U

/* ======================== 数据结构 ======================== */
typedef struct {
    uint8_t digital;   /* 8 通道黑白状态 (Bit0~7, 1=白地面, 0=黑线) */
    uint8_t is_ok;     /* 1=读取成功 */
} Grayscale_Sensor_t;

/* ======================== API ======================== */
void    Grayscale_Init(Grayscale_Sensor_t *sensor);   /* 配置 CLK=输出, DAT=输入 */
void    Grayscale_Update(Grayscale_Sensor_t *sensor);  /* 串行读一次 8 通道 */
uint8_t Grayscale_Get_Digital(Grayscale_Sensor_t *sensor); /* 取最近一次 8 位状态 */

#endif /* GRAYSCALE_H */
