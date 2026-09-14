/**
 * @file    sw_uart.c
 * @brief   软件串口 — PB6=RX, PB7=TX
 * @note
 *   - TX: 阻塞 bit-bang，DWT/NOP 延时（发完一个字节才返回）
 *   - RX: TIM16 硬件定时器做精确位时钟 + EXTI 起始位检测
 *   - 引脚 PB6/PB7 与 OLED 软件 I2C 互斥，通过 SW_UART_ENABLE / OLED_USE_SW_I2C 切换
 */

#include "Common_used.h"

#if SW_UART_ENABLE

#if defined(OLED_USE_SW_I2C)
#error "SW_UART and OLED_USE_SW_I2C both use PB6/PB7 — comment out OLED_USE_SW_I2C in oled.h"
#endif

/* ================================================================
 * 内部宏
 * ================================================================ */

#define SW_UART_BIT_US       (1000000UL / SW_UART_BAUDRATE)

/* EXTI 中断线 */
#define SW_UART_EXTI_LINE    EXTI_LINE_6
#define SW_UART_EXTI_IRQn    EXTI9_5_IRQn

/* TIM7 用于 RX 位时钟（TIM16/17 与 TIM1 共用中断线，不可用） */
#define SW_UART_TIM           TIM7
#define SW_UART_TIM_IRQn      TIM7_IRQn
#define SW_UART_TIM_CLK_EN()  __HAL_RCC_TIM7_CLK_ENABLE()

/* ================================================================
 * 静态变量
 * ================================================================ */

static uint8_t  sw_uart_rx_buf[SW_UART_RX_BUF_SIZE];
static volatile uint16_t sw_uart_rx_head = 0U;
static uint16_t sw_uart_rx_tail = 0U;

static volatile bool sw_uart_tx_busy = false;

/* RX 状态机（TIM7 ISR 驱动） */
static volatile uint8_t  sw_uart_rx_data;
static volatile uint8_t  sw_uart_rx_bit_idx;
static volatile bool     sw_uart_rx_active = false;

/* DEBUG: RX 诊断计数器 (pyOCD 可读) */
volatile uint32_t sw_uart_dbg_exti = 0;   /* EXTI ISR 触发次数 */
volatile uint32_t sw_uart_dbg_start = 0;  /* 起始位确认次数 */
volatile uint32_t sw_uart_dbg_tim7 = 0;   /* TIM7 ISR 触发次数 */
volatile uint32_t sw_uart_dbg_push = 0;   /* 成功接收字节数 */
volatile uint32_t sw_uart_dbg_active = 0; /* rx_active 被跳过次数 */

/* ================================================================
 * 延时（仅 TX 路径使用）
 *
 * 使用汇编循环直接计数 CPU 周期，不受编译器优化等级影响。
 * Cortex-M4: subs + bne ≈ 3~4 cycles/iter
 * ================================================================ */

static void SW_UART_DelayCycles(uint32_t cycles)
{
    if (cycles < 8U) return;
    /* Cortex-M4: subs(1) + bne-taken(2) = 3 cycles/iter; bne-not-taken(1) = 最后一轮 2 cycles */
    cycles /= 3U;
    __ASM volatile (
        "1: subs %0, %0, #1\n"
        "   bne  1b\n"
        : "+r" (cycles)
        :
        : "cc"
    );
}

static void SW_UART_DelayUs(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    SW_UART_DelayCycles(cycles);
}

/* ================================================================
 * GPIO 基础操作
 * ================================================================ */

static void SW_UART_TX_High(void)
{
    HAL_GPIO_WritePin(SW_UART_TX_PORT, SW_UART_TX_PIN, GPIO_PIN_SET);
}

static void SW_UART_TX_Low(void)
{
    HAL_GPIO_WritePin(SW_UART_TX_PORT, SW_UART_TX_PIN, GPIO_PIN_RESET);
}

static uint8_t SW_UART_RX_Read(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(SW_UART_RX_PORT, SW_UART_RX_PIN);
}

/* ================================================================
 * 接收环形缓冲区
 * ================================================================ */

static void SW_UART_RxPush(uint8_t byte)
{
    uint16_t next = (sw_uart_rx_head + 1U) % SW_UART_RX_BUF_SIZE;
    if (next != sw_uart_rx_tail) {
        sw_uart_rx_buf[sw_uart_rx_head] = byte;
        sw_uart_rx_head = next;
    }
    /* DEBUG: 每收到一个字节翻转 LED */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/* ================================================================
 * 初始化
 * ================================================================ */

void SW_UART_Init(void)
{
    /* ---- 1. GPIO 时钟 ---- */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ---- 2. TX (PB7) 推挽输出 ---- */
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = SW_UART_TX_PIN;
        g.Mode  = GPIO_MODE_OUTPUT_PP;
        g.Pull  = GPIO_PULLUP;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(SW_UART_TX_PORT, &g);
    }
    SW_UART_TX_High();

    /* ---- 3. TIM7 时钟使能（必须在 EXTI 中断使能之前！）
     *         否则 EXTI ISR 中访问 TIM7 寄存器会导致 BusFault ---- */
    SW_UART_TIM_CLK_EN();

    SW_UART_TIM->CR1 = 0U;                 /* 先停定时器 */
    SW_UART_TIM->PSC = 0U;                 /* 不分频 */
    SW_UART_TIM->ARR = (uint16_t)((SystemCoreClock / SW_UART_BAUDRATE) - 1U);
    SW_UART_TIM->DIER = TIM_DIER_UIE;      /* 使能更新中断 */
    SW_UART_TIM->EGR = TIM_EGR_UG;         /* 立即更新影子寄存器 */

    HAL_NVIC_SetPriority(SW_UART_TIM_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(SW_UART_TIM_IRQn);

    /* ---- 4. RX (PB6) — 手动配置 EXTI，绕过 HAL 可能的 MODER 误写 ---- */
    {
        uint32_t pinpos = 6U;  /* PB6 */

        /* 4a. 设 MODER = 00 (输入) */
        SW_UART_RX_PORT->MODER = (SW_UART_RX_PORT->MODER & ~(3UL << (pinpos * 2U))) | (0UL << (pinpos * 2U));
        __DSB();
        sw_uart_dbg_start = SW_UART_RX_PORT->MODER;  /* 存 MODER 值 */

        /* 4b. 上拉 (PUPDR = 01) */
        SW_UART_RX_PORT->PUPDR = (SW_UART_RX_PORT->PUPDR & ~(3UL << (pinpos * 2U)))
                                 | (1UL << (pinpos * 2U));

        /* 4c. EXTI 配置: SYSCFG 选通 PB6 → EXTI6 */
        uint32_t exti_cfg_shift = (pinpos % 4U) * 4U;
        uint32_t exti_cfg_reg;
        if (pinpos < 4U) {
            exti_cfg_reg = SYSCFG->EXTICR[0];
            exti_cfg_reg &= ~(0xFUL << exti_cfg_shift);
            exti_cfg_reg |= (1UL << exti_cfg_shift);  /* 1 = PORTB */
            SYSCFG->EXTICR[0] = exti_cfg_reg;
        } else if (pinpos < 8U) {
            exti_cfg_reg = SYSCFG->EXTICR[1];
            exti_cfg_reg &= ~(0xFUL << exti_cfg_shift);
            exti_cfg_reg |= (1UL << exti_cfg_shift);
            SYSCFG->EXTICR[1] = exti_cfg_reg;
        }

        /* 4d. EXTI 下降沿触发，使能中断 */
        EXTI->FTSR1 |= (1UL << pinpos);
        EXTI->RTSR1 &= ~(1UL << pinpos);
        EXTI->IMR1  |= (1UL << pinpos);
    }

    /* 清除残留 EXTI6 pending，再打开 NVIC */
    __HAL_GPIO_EXTI_CLEAR_IT(SW_UART_EXTI_LINE);

    HAL_NVIC_SetPriority(SW_UART_EXTI_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(SW_UART_EXTI_IRQn);

    sw_uart_dbg_exti = 0xBEEF0001;  /* 标记 SW_UART_Init 已完成 */
}

/* ================================================================
 * TX — 阻塞发送（DWT/NOP 延时，对 115200 足够）
 * ================================================================ */

void SW_UART_SendByte(uint8_t data)
{
    uint32_t primask;

    sw_uart_tx_busy = true;

    /* 关全局中断，防止 ISR 打乱位时序（115200 下 1 字节 ≈ 87µs，可接受） */
    primask = __get_PRIMASK();
    __disable_irq();

    /* 起始位 */
    SW_UART_TX_Low();
    SW_UART_DelayUs(SW_UART_BIT_US);

    /* 数据位 LSB first */
    for (uint8_t i = 0U; i < 8U; i++) {
        if (data & 0x01U) {
            SW_UART_TX_High();
        } else {
            SW_UART_TX_Low();
        }
        SW_UART_DelayUs(SW_UART_BIT_US);
        data >>= 1U;
    }

    /* 停止位 */
    SW_UART_TX_High();
    SW_UART_DelayUs(SW_UART_BIT_US);

    /* 恢复中断 */
    if (primask == 0U) {
        __enable_irq();
    }

    sw_uart_tx_busy = false;
}

void SW_UART_SendBytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;
    for (uint16_t i = 0U; i < len; i++) {
        SW_UART_SendByte(data[i]);
    }
}

void SW_UART_SendString(const char *str)
{
    if (str == NULL) return;
    HAL_UART_Transmit(&huart1, (uint8_t *)str, (uint16_t)strlen(str), 100U);
}

void SW_UART_Printf(const char *format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100U);
    }
}

bool SW_UART_TxBusy(void)
{
    return sw_uart_tx_busy;
}

/* ================================================================
 * RX — 环形缓冲区读取（用户侧 API）
 * ================================================================ */

uint16_t SW_UART_Available(void)
{
    uint16_t head = sw_uart_rx_head;
    if (head >= sw_uart_rx_tail) {
        return head - sw_uart_rx_tail;
    }
    return (uint16_t)(SW_UART_RX_BUF_SIZE - sw_uart_rx_tail + head);
}

uint8_t SW_UART_ReadByte(void)
{
    /* 轮询等待至少一字节可用 */
    uint32_t timeout = 100000;
    while (SW_UART_Available() == 0U && --timeout) {
        SW_UART_PollStartBit();
    }
    if (SW_UART_Available() == 0U) return 0U;

    uint8_t byte = sw_uart_rx_buf[sw_uart_rx_tail];
    sw_uart_rx_tail = (sw_uart_rx_tail + 1U) % SW_UART_RX_BUF_SIZE;
    return byte;
}

/* ================================================================
 * RX 轮询 — 替代 EXTI 检测起始位
 * ================================================================ */

/**
 * @brief  轮询 PB6，检测 HIGH→LOW 边沿后启动 TIM7 接收
 */
void SW_UART_PollStartBit(void)
{
    if (sw_uart_rx_active) return;

    /* 检测下降沿: 上次 HIGH + 本次 LOW = 起始位 */
    static uint8_t last = 1;
    uint8_t now = (SW_UART_RX_PORT->IDR & SW_UART_RX_PIN) ? 1 : 0;

    if (last && !now) {
        sw_uart_rx_active   = true;
        sw_uart_rx_bit_idx  = 0U;
        sw_uart_rx_data     = 0U;

        /* CNT=0: 首次溢出在 ~8.7μs (1 bit), 采样 D0 中心 */
        SW_UART_TIM->CNT = 0U;
        SW_UART_TIM->SR  = ~TIM_SR_UIF;
        SW_UART_TIM->CR1 |= TIM_CR1_CEN;
    }
    last = now;
}

/* ================================================================
 * EXTI ISR — 起始位检测，启动 TIM7
 * ================================================================ */

void EXTI9_5_IRQHandler(void)
{
    sw_uart_dbg_exti++;  /* 计数器 + 清所有挂起的 EXTI5-9 */
    EXTI->PR1 = 0x3E0;   /* 清 EXTI5~EXTI9 pending */

    if (__HAL_GPIO_EXTI_GET_IT(SW_UART_EXTI_LINE) == 0U) {
        return;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(SW_UART_EXTI_LINE);

    /* 确认是起始位 */
    if (SW_UART_RX_Read() != 0U) {
        return;
    }

    sw_uart_dbg_start++;

    /*
     * 起始位有效，准备接收。
     *
     * CNT = ARR/2 使首次溢出在 0.5 位后（距边沿 1.5 位），
     * 正好采样 D0 中心。之后每次溢出间隔恰好 1 位。
     */
    if (!sw_uart_rx_active) {
        sw_uart_rx_active   = true;
        sw_uart_rx_bit_idx  = 0U;
        sw_uart_rx_data     = 0U;

        SW_UART_TIM->CNT = SW_UART_TIM->ARR / 2U;
        SW_UART_TIM->SR  = ~TIM_SR_UIF;
        SW_UART_TIM->CR1 |= TIM_CR1_CEN;
    } else {
        sw_uart_dbg_active++;
    }
}

/* ================================================================
 * TIM7 ISR — 逐位采样
 * ================================================================ */

void TIM7_IRQHandler(void)
{
    if ((SW_UART_TIM->SR & TIM_SR_UIF) == 0U) {
        return;
    }
    SW_UART_TIM->SR = ~TIM_SR_UIF;

    sw_uart_dbg_tim7++;

    if (!sw_uart_rx_active) {
        SW_UART_TIM->CR1 &= ~TIM_CR1_CEN;
        return;
    }

    uint8_t idx = sw_uart_rx_bit_idx;

    if (idx < 8U) {
        /* 数据位 D0~D7 */
        if (SW_UART_RX_Read() != 0U) {
            sw_uart_rx_data |= (uint8_t)(1U << idx);
        }
        sw_uart_rx_bit_idx = idx + 1U;
    } else {
        /* 停止位（idx = 8），帧结束 */
        SW_UART_TIM->CR1 &= ~TIM_CR1_CEN;
        sw_uart_rx_active = false;
        sw_uart_dbg_push++;
        SW_UART_RxPush(sw_uart_rx_data);
    }
}

#endif /* SW_UART_ENABLE */
