#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>

/* ============================================================
 * WS2812 灯珠驱动（TIM16_CH1 + DMA 半字流）
 *
 * 引脚：PB4 = TIM16_CH1 (AF1)
 * 原理：TIM16 跑 170MHz(PSC=0, ARR=211 → 约 802kHz)，
 *       DMA 每周期把缓冲里的 CCR1 值搬进比较寄存器，
 *       高电平宽度编码出 0/1 位，整段位流级联驱动灯珠。
 *
 * 前提：必须在 CubeMX 里把 PB4 从 TIM3_CH1 改成 TIM16_CH1，
 *       并给 TIM16_CH1 配 DMA（半字、Normal 模式、开中断），
 *       否则 htim16 / hdma_tim16_ch1 不存在，编译不过。
 * ============================================================ */

/* 级联灯珠数量（按实际焊接数量改） */
#define WS2812_LED_NUM        8

/* 位时序（单位：TIM16 计数，1 tick = 1/170MHz ≈ 5.88ns，ARR=211） */
#define WS2812_CODE_ONE_DUTY  119   /* 高电平 119 tick ≈ 0.70us → 逻辑 1 */
#define WS2812_CODE_ZERO_DUTY 60    /* 高电平 60  tick ≈ 0.35us → 逻辑 0 */

/* 帧前/帧后的全低周期数：100 × 1.247us ≈ 125us ≥ 50us 复位/锁存要求 */
#define WS2812_RST_PERIOD_NUM 100

/* 全部灯珠亮白灯（一次发送，约 (200+24*N)*1.25us 完成，期间重复调用会被丢弃） */
void WS2812_AllWhite(void);

/* 全部灯珠熄灭 */
void WS2812_AllOff(void);

#endif /* __WS2812_H__ */