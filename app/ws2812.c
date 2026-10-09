/**
 * @file    ws2812.c
 * @brief   WS2812 灯珠驱动：TIM16_CH1(PB4) + DMA 半字流
 *
 * 一帧 = 前置复位(100 个全低周期) + 每灯 24bit(GRB, 高位在前) + 后置复位。
 * 后置段保证数据发完后数据线保持低电平 → 灯珠锁存本帧。
 * DMA 为 Normal 模式：发完一帧后通道状态回到 READY，可再次调用发送。
 */

#include "ws2812.h"
#include "main.h"
#include "tim.h"

/* 一帧的 CCR 值缓冲。static 保证 DMA 发送期间数据始终有效。
 * 前置/后置复位段恒为 0（static 零初始化，编码只写数据段）。 */
static uint16_t s_ws2812_frame[WS2812_RST_PERIOD_NUM * 2 + WS2812_LED_NUM * 24];

#define WS2812_FRAME_LEN \
    ((uint16_t)(WS2812_RST_PERIOD_NUM * 2 + WS2812_LED_NUM * 24))

/* 数据段起点：跳过前置复位段 */
#define WS2812_DATA_OFFSET (WS2812_RST_PERIOD_NUM)

/**
 * @brief   把一盏灯的颜色编码进缓冲（WS2812 数据序：GRB，高位在前）
 * @param   r   红色亮度 0-255
 * @param   g   绿色亮度 0-255
 * @param   b   蓝色亮度 0-255
 * @param   p   指向该灯 24 个 CCR 值在缓冲中的起点
 */
static void ws2812_fill_led(uint8_t r, uint8_t g, uint8_t b, uint16_t *p)
{
    uint8_t data[3] = { g, r, b };

    for (uint8_t c = 0; c < 3; c++)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            p[c * 8 + i] = ((data[c] >> (7 - i)) & 0x01)
                               ? WS2812_CODE_ONE_DUTY
                               : WS2812_CODE_ZERO_DUTY;
        }
    }
}

/**
 * @brief   启动一次 DMA 发送（约 (200+24*N) × 1.25us，8 灯 ≈ 490us）
 * @note    上一帧还在发送时调用会被 HAL 拒绝（HAL_BUSY），本次直接丢弃。
 */
static void ws2812_send(void)
{
    /* 半字 DMA：缓冲是 uint16_t 数组，API 形参是 uint32_t*，按半字搬运 */
    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1,
                          (uint32_t *)s_ws2812_frame, WS2812_FRAME_LEN);
}

void WS2812_AllWhite(void)
{
    for (uint8_t led = 0; led < WS2812_LED_NUM; led++)
    {
        ws2812_fill_led(255, 255, 255,
                        s_ws2812_frame + WS2812_DATA_OFFSET + led * 24);
    }
    ws2812_send();
}

void WS2812_AllOff(void)
{
    for (uint8_t led = 0; led < WS2812_LED_NUM; led++)
    {
        ws2812_fill_led(0, 0, 0,
                        s_ws2812_frame + WS2812_DATA_OFFSET + led * 24);
    }
    ws2812_send();
}
