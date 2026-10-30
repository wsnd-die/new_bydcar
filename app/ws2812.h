#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "stdio.h"
#include "string.h"

/* ============================================================
 * WS2812 驱动（移植自 keysking 教程，按本工程做过适配）
 *
 * 数据脚 **PB4 = TIM16_CH1**，DMA 搬运到 CCR1。
 * 相对原版：htim3 → htim16；缓冲加后置复位（否则不锁存）；组序可切；
 * HAL_Delay → osDelay。
 *
 * ---- 必须满足的硬件 / CubeMX 前提 ----
 *   · SYS → Debug = **Serial Wire**（PB4 复位后默认是 JTAG 的 NJTRST）
 *   · **PB4 → Alternate Function Open Drain**，Speed High
 *   · **PB4 外接 5V 上拉 1k~4.7k** —— 灯珠 VIH = 0.7×VDD = 3.5V，
 *     MCU 推挽只有 3.3V 跨不过去。PB4 是 FT_c(5V 耐受) 脚，安全。
 *   · DMA：TIM16_CH1，MemToPeriph、Half Word、Normal、
 *     **Peripheral Increment = DISABLE**（目的地址固定 CCR1）
 *
 * ---- 备用引脚 ----
 *   PD7 = **TIM2_CH3，复用号 AF2**（不是 AF1，写错毫无输出）。
 *   换过去需同时改 ws2812.c 的 htim16 / TIM_CHANNEL_1 / TIM_DMA_ID_CC1。
 * ============================================================ */

/* 位时序。170MHz / ARR=212（213 tick = 798kHz）；换主频时一起改。 */
#define CODE_ONE_DUTY 156
#define CODE_ZERO_DUTY 50

/* 前 / 后置复位的低电平周期数。100 × 1.253us ≈ 125us，
 * 是规格书的 RESET 下限，只能加不能减。 */
#define RST_PERIOD_NUM 100

/* 级联灯珠数量（按实际焊接数改；比实际多送无害，少送会掉尾） */
#define WS2812_NUM 10

/* 0 = GRB  1 = RGB
 * ⚠ 取 1（RGB）：与实测能正常显色的参照工程一致；改成标准 GRB 反而对不上。 */
#define WS2812_BYTE_ORDER_RGB 1

extern uint32_t ws2812_color[WS2812_NUM];

// 将颜色数组直接更新到 LED，不使用渐变过渡
void ws2812_update(void);

// 渐变的更新LED颜色
void ws2812_gradient(uint8_t steps, uint16_t delay_ms);

// 设置LED颜色（24bit颜色）
void ws2812_set(uint8_t led_id, uint32_t color);

// 设置LED颜色（RGB）
void ws2812_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b);

// 设置所有LED颜色
void ws2812_set_all(uint32_t color);

// RGB转换为24bit颜色
uint32_t rgb_to_color(uint8_t r, uint8_t g, uint8_t b);

// 24bit颜色转换为RGB
void color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b);

// =============== 以下为额外的效果函数 ===============

// 彩虹颜色生成
uint32_t rainbow_color(float frequency, int phase, int center, int width);

// 彩虹效果
void rainbow_effect(uint8_t steps, uint16_t delay_ms);

// =============== 兼容别名（让流程里的调用点不用改） ===============
void WS2812_AllWhite(void);   /* = set_all(白) + update */
void WS2812_AllOff(void);     /* = set_all(黑) + update */

#endif
