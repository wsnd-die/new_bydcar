/**
 * @file    collect_ir.c
 * @brief   红外对射开关 + 颜色传感器收集判断
 */
#include "Common_used.h"
#include "collect_ir.h"

static bool ir_last = false;   /* 上一采样是否遮挡 true=遮光 */

void IR_Init(void)
{
#if defined(PWR_CR3_UCPD_DBDIS)
    SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);
#endif

    GPIO_InitTypeDef g = {0};
    g.Pin   = IR_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IR_PORT, &g);
}

bool IR_ObjectPresent(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(IR_PORT, IR_PIN);
#if IR_ACTIVE == 0
    return (s == GPIO_PIN_RESET);   /* 遮光=低电平 */
#else
    return (s == GPIO_PIN_SET);     /* 遮光=高电平 */
#endif
}

bool IR_ObjectEntered(void)
{
    bool now = IR_ObjectPresent();

    if (now && !ir_last) {
        osDelay(30);
        now = IR_ObjectPresent();
        if (!now) return false;
    }

    bool fully_in = (ir_last && !now);

    /* 防抖 */
    if (fully_in) {
        osDelay(20);
        now = IR_ObjectPresent();
        if (now) {
            ir_last = now;
            return false;
        }
        fully_in = true;
    }

    ir_last = now;
    return fully_in;
}

bool Collect_WaitEnter(void)
{
    while (!IR_ObjectEntered()) { osDelay(10); }
    return true;
}

/* V1.20.0 删除: Collect_ReadColor() / Collect_ReadColor_NB() / Collect_WaitObject()。
 * 三者读的是 GY-33 颜色传感器的 g_uart2_gy33_* 全局量, 而那些变量的定义只存在于
 * hardware/bus/uart2_tbop10.c 的**注释**里 (整个文件被注释掉了)。它们能过编译
 * 只是因为当时无人调用、被 --gc-sections 整段丢掉 —— 一旦被引用就链接失败。
 * 颜色改由 MSP 芯片经 USART2 回传, 见 hardware/sensors/msp_color.c。 */

