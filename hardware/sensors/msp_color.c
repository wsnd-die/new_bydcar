/**
 * @file    msp_color.c
 * @brief   MSP 颜色芯片回传 —— USART2 (RX = PA3)。帧格式见 msp_color.h。
 *
 *  走 DMA + IDLE, 与 uart/NX_uart.c 同款: 一次挂满缓冲, 总线 IDLE 或收满时进
 *  Core/Src/usart.c 的 HAL_UARTEx_RxEventCallback() 分发器, 再原地重挂。
 *
 *  与 printf 不冲突: printf 走 huart2 的 TX 侧 (gState), 本驱动只碰 RX 侧
 *  (RxState), HAL 里是两套独立状态机。
 */
#include "Common_used.h"
#include "msp_color.h"

#define MSP_RX_BUF_SIZE  16u

/** DMA 落点缓冲。 */
static uint8_t s_rx_buf[MSP_RX_BUF_SIZE];

static struct {
    volatile uint8_t pending;    /* 1 = 有一帧颜色没被取走 (颜色字符是 r/b) */
    volatile uint8_t color;      /* 最近一帧的帧头后第一个字节 */
    MSP_rx_state_t   rx_state;   /* 只在 ISR 里推进 */

    /* 投票票箱 (V1.27.1): ISR 侧**每收到一帧就 +1**, 由 MSP_Color_Vote() 读并清。
     *
     * 为什么不能拿上面的 `pending` 计票: 它是个**灯**不是**计数器** —— 同一批
     * 连着到的帧把它置 1 多次, `MSP_Color_Take()` 每次只取走最后一帧, 一个
     * 10ms 窗口里的其余几帧票就丢了 ("一帧 = 一票"从这儿断掉)。
     * 颜色字符在 ISR 里就知道, 所以按颜色分开计最准, 不必事后反推。 */
    volatile uint16_t ok_red;
    volatile uint16_t ok_blue;

    /* 调试: 数字段原样留存, 遇非数字即整帧结束 */
    volatile uint8_t  digits[MSP_DIGITS_MAX];
    volatile uint8_t  digits_len;
    volatile uint8_t  frame_ready;   /* 1 = 有一整帧没被 MSP_Color_DebugPoll 打印 */

    /* 诊断 (ISR 写, 任务读, 故 volatile) */
    volatile uint32_t rx_bytes;
    volatile uint32_t rx_chars;
    volatile uint32_t rx_ok;
    volatile uint32_t rx_unknown;
} s_msp;

/**
 * @brief 挂上 DMA-IDLE 接收 (Init / 回调 / 错误恢复都走它)。
 */
static void MSP_StartRx(void)
{
    __HAL_UART_CLEAR_FLAG(&huart2,
        UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_buf, MSP_RX_BUF_SIZE) != HAL_OK) {
        return;
    }

    /* ReceiveToIdle_DMA 内部走 HAL_DMA_Start_IT, 会连半传输(HT)中断一起打开,
     * 缓冲收到一半时同样触发 RxEvent, 把一次接收切成两截。
     * 本驱动按「总线 IDLE + 收满」两种事件处理, 不需要 HT。 */
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void MSP_Color_Init(void)
{
    memset(&s_msp, 0, sizeof(s_msp));   /* rx_state 归零 = MSP_RX_WAIT_A */
    MSP_StartRx();
}

void msp_color_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART2) {
        return;
    }

    for (uint16_t i = 0; i < Size; i++) {
        uint8_t b = s_rx_buf[i];

        s_msp.rx_bytes++;

        /* 帧尾有 \r\n, 空白一律不参与状态机, 在这里吃掉 */
        if (b == '\r' || b == '\n' || b == ' ' || b == '\t' || b == '\0') {
            continue;
        }
        s_msp.rx_chars++;

        switch (s_msp.rx_state)
        {
        case MSP_RX_WAIT_A:
            if (b == 'A') {
                s_msp.rx_state = MSP_RX_WAIT_3;
            }
            break;

        case MSP_RX_WAIT_3:
            if (b == '3') {
                s_msp.rx_state = MSP_RX_COLOR;
            } else if (b != 'A') {
                /* 不是 '3' 也不是 'A' → 这个 'A' 是误配, 回去重找帧头。
                 * 是 'A' 的话就停在这 —— 它本身就是新帧头的第一字节。 */
                s_msp.rx_state = MSP_RX_WAIT_A;
            }
            break;

        case MSP_RX_COLOR:

            s_msp.color      = b;       /* 后到的覆盖先到的 */
            s_msp.digits_len = 0u;
            if (b == 'r' || b == 'b') {
                s_msp.pending = 1u;     /* 给 Take()/Wait() 用 (调试打印 + 单帧等待) */
                s_msp.rx_ok++;

                /* 计票在这里, 不在 Take() —— 见 s_msp.ok_red 的说明 */
                if (b == 'r') {
                    s_msp.ok_red++;
                } else {
                    s_msp.ok_blue++;
                }
            } else {
                s_msp.rx_unknown++;
            }
            s_msp.rx_state = MSP_RX_DIGITS;
            break;

        case MSP_RX_DIGITS:
            if (b >= '0' && b <= '9') {
                if (s_msp.digits_len < (uint8_t)(MSP_DIGITS_MAX - 1u)) {
                    s_msp.digits[s_msp.digits_len++] = b;
                }
                break;              /* 继续收数字 */
            }
            /* 非数字 = 数字段结束, 整帧到齐 */
            s_msp.digits[s_msp.digits_len] = '\0';
            s_msp.frame_ready = 1u;
            s_msp.rx_state = (b == 'A') ? MSP_RX_WAIT_3 : MSP_RX_WAIT_A;
            break;

        default:
            s_msp.rx_state = MSP_RX_WAIT_A;
            break;
        }
    }
    MSP_StartRx();
}

static Color_TypeDef MSP_CharToColor(uint8_t ch)
{
    if (ch == 'r') return COLOR_RED;
    if (ch == 'b') return COLOR_BLUE;
    return COLOR_UNKNOWN;
}

bool MSP_Color_Take(Color_TypeDef *out)
{
    if (!s_msp.pending) {
        return false;
    }

    /* 先把字符取出来再清标志 —— 否则 ISR 可能在两步之间把 color 换成下一帧 */
    const uint8_t ch = s_msp.color;
    s_msp.pending = 0u;

    Color_TypeDef c = MSP_CharToColor(ch);
    // if (c == COLOR_UNKNOWN) {
    //     return false;                       /* 收到了但不认识, 当没收到 */
    // }

    printf("[MSP] color='%c' -> %s\r\n", (char)ch, Color_ToString(c));

    if (out) {
        *out = c;
    }
    return true;
}

bool MSP_Color_Wait(Color_TypeDef *out, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    s_msp.pending = 0u;                     /* 丢掉残留的旧帧 */

    for (;;) {
        if (MSP_Color_Take(out)) {
            return true;
        }
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return false;
        }
        osDelay(10);
    }
}

bool MSP_Color_Vote(uint32_t window_ms, Color_TypeDef *out)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t votes[COLOR_COUNT] = { 0u };    /* 只用 [COLOR_RED] / [COLOR_BLUE] */

    /* 开窗清零: 只算**本窗口内**收到的帧 (V1.27.1)。
     *
     * 与 V1.26.7 的"不预先丢残留帧"是**相反**的取舍, 理由:
     * 票箱一改成计数器, "手上那帧"就变成了"上次 identify_slot 以来的**全部**帧",
     * 其中大部分是转盘转动过程中扫到的旧槽 —— 那才是垃圾。而每帧一票之后,
     * 一个窗口收到的票数远多于从前 (从前每 10ms 顶多 1 票), "票不够"的前提
     * 本身已经不存在了。
     *
     * @note 这里是任务侧清零、ISR 侧自增, 严格说有一拍的竞态: 极少数情况下
     *       开窗那一瞬间到的帧会被漏掉/多算。窗口是几百 ms、且取的是多数,
     *       对结果无影响。 */
    s_msp.ok_red  = 0u;
    s_msp.ok_blue = 0u;

    while ((HAL_GetTick() - t0) < window_ms) {
        osDelay(10);
    }

    /* 窗口结束: 读票并清, 给下一次调用留干净票箱 */
    votes[COLOR_RED]  = s_msp.ok_red;   s_msp.ok_red  = 0u;
    votes[COLOR_BLUE] = s_msp.ok_blue;  s_msp.ok_blue = 0u;

    /* 取多数: 严格大于 → 同票时取先遍历到的 (COLOR_RED) */
    Color_TypeDef best = COLOR_UNKNOWN;
    for (Color_TypeDef i = COLOR_RED; i < COLOR_COUNT; i++) {
        if (votes[i] > votes[best]) {
            best = i;
        }
    }

    printf("[MSP] color votes: r=%u b=%u -> %s\r\n",
           (unsigned)votes[COLOR_RED], (unsigned)votes[COLOR_BLUE],
           (best == COLOR_UNKNOWN) ? "none" : Color_ToString(best));

    if (best == COLOR_UNKNOWN) {
        return false;
    }
    if (out) {
        *out = best;
    }
    return true;
}

bool MSP_Color_DebugPoll(void)
{
    if (!s_msp.frame_ready) {
        return false;
    }
    s_msp.frame_ready = 0u;

    char    buf[MSP_DIGITS_MAX];
    uint8_t n = s_msp.digits_len;
    if (n > (uint8_t)(MSP_DIGITS_MAX - 1u)) {
        n = (uint8_t)(MSP_DIGITS_MAX - 1u);
    }
    for (uint8_t i = 0; i < n; i++) {
        buf[i] = (char)s_msp.digits[i];
    }
    buf[n] = '\0';
    // printf("[MSP-RX] color='%c' rgb_raw=\"%s\"\r\n", (char)s_msp.color, buf);
    (void)buf;      /* 上面那行打印暂时关掉, 先按"已用"处理免得 -Wunused-but-set-variable */
    return true;
}

void MSP_Color_GetDiag(uint32_t *rx_bytes, uint32_t *rx_chars,
                       uint32_t *rx_ok, uint32_t *rx_unknown)
{
    if (rx_bytes)   *rx_bytes   = s_msp.rx_bytes;
    if (rx_chars)   *rx_chars   = s_msp.rx_chars;
    if (rx_ok)      *rx_ok      = s_msp.rx_ok;
    if (rx_unknown) *rx_unknown = s_msp.rx_unknown;
}
