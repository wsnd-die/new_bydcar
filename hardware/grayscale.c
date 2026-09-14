/**
 * @file grayscale.c
 * @brief 八通道灰度循迹传感器串行驱动 — PA4=CLK 输出, PA5=DAT 输入
 *
 * 串行协议 (参考 3519 项目):
 *   每 bit: CLK 拉低 → 延时 → 读 DAT → CLK 拉高 → 延时
 *   重复 8 次, i=0 读出的是通道0 (最左侧通道), 存入 digital 的 Bit7。
 */

#include "Common_used.h"
#include "grayscale.h"

static void grayscale_delay_us(uint32_t us)
{
    /* DWT 精确 us 延时 (与 sw_uart 一致): 170MHz 下 cycles = us * 170 */
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    __asm volatile (
        "1: subs %0, #1\n"
        "   bne  1b\n"
        : "+r" (cycles)
        :
        : "cc"
    );
}

void Grayscale_Init(Grayscale_Sensor_t *sensor)
{
    GPIO_InitTypeDef g = {0};

    /* CLK 推挽输出 */
    g.Pin   = GRAY_CLK_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GRAY_CLK_GPIO_Port, &g);

    /* DAT 输入 + 上拉 (感为模块 DAT 常见开漏输出, 上拉才能读到高电平) */
    g.Pin   = GRAY_DAT_Pin;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GRAY_DAT_GPIO_Port, &g);

    /* CLK 空闲拉高 */
    HAL_GPIO_WritePin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, GPIO_PIN_RESET);

    if (sensor) {
        sensor->digital = 0U;
        sensor->is_ok   = 0U;
    }
}

/**
 * @brief 串行读一次, 返回 8 位原始数据。
 * @note  i=0 是通道0 (最左), 存入 Bit7 (与 3519 项目 bit 顺序一致)。
 */
uint8_t Grayscale_Serial_Read(void)
{
    uint8_t data = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        HAL_GPIO_WritePin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, GPIO_PIN_SET);
        grayscale_delay_us(GRAY_CLK_HALF_PERIOD_US);

        HAL_GPIO_WritePin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, GPIO_PIN_RESET);

        if (HAL_GPIO_ReadPin(GRAY_DAT_GPIO_Port, GRAY_DAT_Pin) != GPIO_PIN_RESET) {
            data |= (uint8_t)(1U << i);
        }

        // HAL_GPIO_WritePin(GRAY_CLK_GPIO_Port, GRAY_CLK_Pin, GPIO_PIN_SET);
        // grayscale_delay_us(GRAY_CLK_HALF_PERIOD_US);
    }

    return data;
}

void Grayscale_Update(Grayscale_Sensor_t *sensor)
{
    uint8_t raw;
    uint8_t i;

    if (sensor == NULL) return;

    raw = Grayscale_Serial_Read();

    /* raw: i=0 → 通道0 → 存 Bit7 (与 follow_line 的位序一致) */
    sensor->digital = 0U;
    //raw=~raw;
    //sensor->digital = raw;
    for (i = 0U; i < 8U; i++) {
        if ((raw & (1U << i)) != 0U) {
            sensor->digital |= (uint8_t)(1U << (7U - i));
        }
    }
    sensor->is_ok = 1U;
}

uint8_t Grayscale_Get_Digital(Grayscale_Sensor_t *sensor)
{
    if (sensor == NULL || sensor->is_ok == 0U) {
        return 0U;
    }
    return sensor->digital;
}
