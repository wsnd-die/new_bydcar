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
 * ---- 相对原版改了 4 处，原因如下 ----
 * ① 定时器句柄 htim3 → **htim16**（数据脚 = **PB4 = TIM16_CH1**）
 *    keysking 的板子用 PB4 = TIM3_CH1，本工程把它配成了 TIM16_CH1。
 *    ⚠ 原样调用 &htim3 会走 HAL_DMA_Start_IT(NULL,...) → **HardFault**，
 *      因为 htim3 上根本没 link 任何 DMA。
 * ② 数据缓冲加了**后置复位**
 *    原版只有前置的 RST_PERIOD_NUM 个 0。DMA 发完后 CCR 停在最后一个
 *    位元的值（不是 0），数据线继续抖 → 灯珠等不到复位 → **不锁存**，
 *    什么都不显示。
 * ③ 位元组序可切
 *    WS2812 标准是 **GRB**；keysking 原码按 R/G/B 发。
 *    颜色红绿对调就把 WS2812_BYTE_ORDER_RGB 改成 1。
 * ④ HAL_Delay → osDelay（本工程是 FreeRTOS，HAL_Delay 是忙等不让出 CPU）
 *
 * ---- 必须满足的 CubeMX / 硬件前提 ----
 *   · SYS → Debug = **Serial Wire**（PB4 复位后默认是 JTAG 的 NJTRST，
 *     不改成 SWD 的话这个脚被调试口占着）
 *   · **PB4 → Alternate Function Open Drain**，Maximum output speed = High
 *   · **PB4 外接 5V 上拉电阻（1k~4.7k）**
 *     灯珠 VIH = 0.7 x VDD = 3.5V(5V 供电)，MCU 推挽只有 3.3V 跨不过去。
 *     开漏 + 5V 上拉把高电平抬到 5V，才有信号。PB4 是 FT_c(5V 耐受) 脚，安全。
 *   · DMA：TIM16_CH1，MemToPeriph、Half Word、Normal、
 *     **Peripheral Increment = DISABLE**（目的地址固定 CCR1）
 *
 * ---- 备用引脚（要换时看这里）----
 *   PD7 = **TIM2_CH3，复用号是 AF2**（不是 AF1 —— 写错的话 PD7 毫无输出，
 *   这个坑踩过一次）。要换过去需要在 CubeMX 里：
 *     ① 先移除 PB4 的 S_TIM16_CH1 和 TIM16 的 DMA（让出 DMA1_Channel6，
 *        DMA1 六个通道已经全满，否则 TIM2_CH3 选不到通道）；
 *     ② 新增 TIM2：Channel3 = PWM Generation CH3，Prescaler=0，
 *        Counter Period=212；
 *     ③ PD7 → TIM2_CH3，GPIO = Alternate Function **Open Drain**、Speed High；
 *     ④ DMA：TIM2_CH3、DMA1 Channel6、MemToPeriph、**PeriphInc=DISABLE**、
 *        双方 Half Word、Normal、开中断。
 *   生成后把 ws2812.c 的 htim16/TIM_CHANNEL_1/TIM_DMA_ID_CC1 换成
 *   htim2/TIM_CHANNEL_3/TIM_DMA_ID_CC3 即可。
 * ============================================================ */

/* 位时序。⚠ keysking 文档的 66/21 是给 **72MHz / ARR=89** 的；
 * 本工程 170MHz / ARR=212（213 tick = 798kHz），必须按 170/72≈2.36 放大：
 *   66 x 2.36 ≈ 156（917ns → 逻辑 1）
 *   21 x 2.36 ≈ 50 （294ns → 逻辑 0）
 * 两段时长与原版一致，换主频时记得一起改。 */
#define CODE_ONE_DUTY 156
#define CODE_ZERO_DUTY 50

/* 前置 / 后置复位的低电平周期数。100 x 1.253us ≈ 125us —— 这是规格书的
 * RESET 下限，刚好够；想留余量可以加大（只能加不能减）。 */
#define RST_PERIOD_NUM 100

/* 级联灯珠数量（按实际焊接数改；比实际多送无害，少送会掉尾） */
#define WS2812_NUM 10

/* 0 = GRB  1 = RGB
 * ⚠ 取 1（RGB）：与能正常工作的 F103 参照工程 (D:\Desktop\WS2812) 一致 ——
 *   同一批灯带在那边用 RGB 顺序能正常显色，所以这里必须跟着用 RGB。
 *   之前改成标准 GRB 反而和实物对不上。 */
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
