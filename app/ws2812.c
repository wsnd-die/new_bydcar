#include "ws2812.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"   /* osDelay —— 任务里不能用 HAL_Delay(忙等, 不让出 CPU) */

// LED颜色
uint32_t ws2812_color[WS2812_NUM] = {0};

// 当前LED颜色
static uint32_t _ws2812_color_current[WS2812_NUM];

/**
 * @brief  上一帧是否还在 DMA 搬运中
 * @note   判据用 DMA 通道自己的 **CCR.EN 位** —— Normal 模式下 CNDTR 减到 0 时
 *         硬件自动清 EN，这是硬件的真话。
 *         **不要用 HAL 的 hdma->State**：它要靠 DMA 完成中断回来才复位，
 *         一旦中断没来就永久误判为"忙"，把之后所有发送都挡死。
 */
static uint8_t ws2812_busy(void)
{
    DMA_HandleTypeDef *hdma = htim16.hdma[TIM_DMA_ID_CC1];

    if (hdma == NULL) {
        return 1u;
    }
    return (hdma->Instance->CCR & DMA_CCR_EN) ? 1u : 0u;
}

/**
 * @brief  直接更新LED颜色
 */
void ws2812_update(void)
{

	static uint16_t ws2812_data[RST_PERIOD_NUM * 2 + WS2812_NUM * 24];
	static uint8_t  s_dbg = 0u;

	/* ← 临时诊断(定位完删): 打印前 6 次, 把整条链问清楚 ——
	 *   没打印      → ws2812_update() 根本没被呼叫(任务没跑 / 没人调)
	 *   busy=1      → 被发送中判断挡住, 一帧都发不出去
	 *   st != 0     → HAL_TIM_PWM_Start_DMA 失败(定时器/DMA 状态卡住)
	 *   st == 0     → 启动成功, 数据应该出去了 → 问题在灯带/接线一侧 */
	if (s_dbg < 6u) {
		s_dbg++;
		printf("[ws] update called #%u, busy=%u\r\n", (unsigned)s_dbg, (unsigned)ws2812_busy());
	}

	/* ⚠ 临时: **拿掉「发送中」守卫**, 与 keysking 原版一致(F103 参照工程就是
	 *    直接调 HAL_TIM_PWM_Start_DMA, 没有 busy 判断)。
	 *    原因: 串口实测 busy 从第 2 次起恒为 1 —— 判据是 DMA 的 CCR.EN 位,
	 *    而它一直没被硬体清掉(说明那笔传输没跑完) ⇒ 守卫把之后所有发送全挡死,
	 *    连"重送一帧"的机会都没有。先按参照工程来, 看 start_dma 的回传值。 */
	/* 诊断结论出来后恢复:
	if (ws2812_busy()) {
		return;
	}
	*/

	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
	{
		uint8_t r, g, b, c0, c1, c2;
		_ws2812_color_current[led_id] = ws2812_color[led_id];
		color_to_rgb(_ws2812_color_current[led_id], &r, &g, &b);

#if WS2812_BYTE_ORDER_RGB
		c0 = r; c1 = g; c2 = b;      /* keysking 原版顺序 */
#else
		c0 = g; c1 = r; c2 = b;      /* WS2812 标准顺序 GRB */
#endif

		uint16_t *p = ws2812_data + RST_PERIOD_NUM + led_id * 24;
		for (uint8_t i = 0; i < 8; i++)
		{
			p[i]      = (c0 << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
			p[i + 8]  = (c1 << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
			p[i + 16] = (c2 << i) & (0x80) ? CODE_ONE_DUTY : CODE_ZERO_DUTY;
		}
	}

	/* 数据脚 PB4 = TIM16_CH1 */
	{
		HAL_StatusTypeDef st = HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1,
													 (uint32_t *)ws2812_data,
													 RST_PERIOD_NUM * 2 + WS2812_NUM * 24);
		if (s_dbg <= 6u) {
			printf("[ws] start_dma st=%d\r\n", (int)st);   /* 0=OK 1=ERROR 2=BUSY */
		}
	}
}

/**
 * @brief  通过渐变方式更新LED颜色（线性插值）
 * @param  steps: 渐变步数
 * @param  delay_ms: 每步之间的延迟时间（毫秒）
 */
void ws2812_gradient(uint8_t steps, uint16_t delay_ms)
{
	static uint8_t start_r[WS2812_NUM], start_g[WS2812_NUM], start_b[WS2812_NUM];
	static float r_step[WS2812_NUM], g_step[WS2812_NUM], b_step[WS2812_NUM];

	if (steps == 0u) {
		return;                 /* 原版没防 0, 会被当除数 */
	}

	// 提取初始颜色，并计算每步的渐变步长
	for (uint8_t i = 0; i < WS2812_NUM; i++)
	{
		color_to_rgb(_ws2812_color_current[i], &start_r[i], &start_g[i], &start_b[i]);
		uint8_t target_r, target_g, target_b;
		color_to_rgb(ws2812_color[i], &target_r, &target_g, &target_b);

		r_step[i] = (float)(target_r - start_r[i]) / steps;
		g_step[i] = (float)(target_g - start_g[i]) / steps;
		b_step[i] = (float)(target_b - start_b[i]) / steps;
	}

	// 逐步渐变
	for (uint8_t step = 1; step <= steps; step++)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			// 计算当前步的颜色
			uint8_t r = (uint8_t)(start_r[led_id] + r_step[led_id] * step);
			uint8_t g = (uint8_t)(start_g[led_id] + g_step[led_id] * step);
			uint8_t b = (uint8_t)(start_b[led_id] + b_step[led_id] * step);

			ws2812_set_rgb(led_id, r, g, b);
		}

		ws2812_update();
		osDelay(delay_ms);      /* 原来是 HAL_Delay: 忙等, 不等让出 CPU */
	}
}

/**
 * @brief  设置LED颜色(RGB格式)
 */
void ws2812_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b)
{
	if (led_id >= WS2812_NUM) {
		return;                 /* 原版没做边界保护 */
	}
	ws2812_color[led_id] = rgb_to_color(r, g, b);
}

/**
 * @brief  设置LED颜色（24bit颜色格式）
 */
void ws2812_set(uint8_t led_id, uint32_t color)
{
	if (led_id >= WS2812_NUM) {
		return;
	}
	ws2812_color[led_id] = color;
}

/**
 * @brief  设置所有LED颜色（24bit颜色格式）
 */
void ws2812_set_all(uint32_t color)
{
	for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
	{
		ws2812_color[led_id] = color;
	}
}

/**
 * @brief  RGB转换为24bit颜色
 */
uint32_t rgb_to_color(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
 * @brief  24bit颜色转换为RGB
 */
void color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
	*r = (color >> 16) & 0xFF;
	*g = (color >> 8) & 0xFF;
	*b = color & 0xFF;
}

// =============== 以下为额外的效果演示函数 ================

uint32_t rainbow_color(float frequency, int phase, int center, int width)
{
	float r = sinf(frequency * phase + 0) * width + center;
	float g = sinf(frequency * phase + 2) * width + center;
	float b = sinf(frequency * phase + 4) * width + center;
	return rgb_to_color((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

void rainbow_effect(uint8_t steps, uint16_t delay_ms)
{
	float frequency = 0.1;
	int center = 128;
	int width = 127;

	for (int i = 0; i < steps; i++)
	{
		for (uint8_t led_id = 0; led_id < WS2812_NUM; led_id++)
		{
			uint32_t color = rainbow_color(frequency, i + led_id * 2, center, width);
			ws2812_set(led_id, color);
		}
		ws2812_update();
		osDelay(delay_ms);
	}
}

// =============== 兼容别名：流程里的调用点用的这两个名字 ===============
void WS2812_AllWhite(void)
{
	ws2812_set_all(0x00FFFFFFu);
	ws2812_update();
}

void WS2812_AllOff(void)
{
	ws2812_set_all(0x00000000u);
	ws2812_update();
}
