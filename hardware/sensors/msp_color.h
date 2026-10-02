/**
 * @file    msp_color.h
 * @brief   MSP 颜色芯片回传 —— USART2 (RX = PA3)
 *
 *  发送端 sprintf 出来的帧:
 *
 *      "A3" <color> <r><g><b> "B3" \r\n
 *       │     │         │       │
 *      帧头   颜色字符   三个十进制数字   帧尾
 *
 *      例:  color="r", r=200, g=30, b=40
 *           A 3 r 2 0 0 3 0 4 0 B 3 \r \n
 *
 *  **颜色直接用帧头后面那个字符** —— 它是发送端用 %s 发来的 'r' / 'b',
 *  不用从 RGB 推。后面那串数字没有分隔符也没有定宽 (`%d%d%d`), 切不开,
 *  整个丢掉。想用 RGB 得先让发送端改成 `%03d%03d%03d` 或加分隔符。
 *
 *  本驱动只负责「把最新一帧的颜色拿出来」,**不编造颜色** —— 收不到就返回
 *  false, 由调用方决定回退成什么 (见 app/block_collect.c)。
 */
#ifndef MSP_COLOR_H
#define MSP_COLOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "color.h"      /* Color_TypeDef / Color_ToString */

/* usart.c 里定义, 由本驱动关掉它的半传输中断 (同 uart/NX_uart.h 的做法) */
extern DMA_HandleTypeDef hdma_usart2_rx;

/** 收帧状态机 (只在 ISR 里推进)。 */
typedef enum
{
    MSP_RX_WAIT_A = 0,   /* 等帧头第一字节 'A' */
    MSP_RX_WAIT_3,       /* 等帧头第二字节 '3' */
    MSP_RX_COLOR,        /* 取颜色字符 */
    MSP_RX_DIGITS,       /* 收数字段 (原样留着给调试打印), 遇非数字即整帧结束 */
} MSP_rx_state_t;

/** 数字段的最长字节数 (调试打印用, 截断不报错)。 */
#define MSP_DIGITS_MAX   16u

/**
 * 单次取色的等待上限, 单位 ms。★实测调整。
 * 等的是「串口有没有回一帧」, 与颜色识别准不准无关。
 */
#define MSP_COLOR_TIMEOUT_MS   300u

/** ISR 侧: 由 Core/Src/usart.c 的 HAL_UARTEx_RxEventCallback() 分发器调用。 */
void msp_color_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/** 挂上 DMA-IDLE 接收。在 MX_USART2_UART_Init() 之后调一次。 */
void MSP_Color_Init(void);

/**
 * @brief  阻塞等一帧颜色。
 * @param  out         非空时写入 COLOR_RED / COLOR_BLUE。
 * @param  timeout_ms  等待上限。
 * @retval true  拿到了有效颜色; false 超时, 或帧里的字符不是 'r'/'b'(*out 不动)。
 * @note   进入时会先丢掉上一次的残留帧 —— 取色只在夹紧后调, 旧帧不该算数。
 */
bool MSP_Color_Wait(Color_TypeDef *out, uint32_t timeout_ms);

/** 非阻塞取一帧。无新帧返回 false。取到时打印一行 [MSP] 便于调试。 */
bool MSP_Color_Take(Color_TypeDef *out);

/**
 * @brief  调试用: 每收到一整帧就打印它的原始内容, 返回是否打印了。
 *
 *  打印形如 `[MSP-RX] color='r' rgb_raw="2003040"`。
 *
 * @note   **数字段原样打印, 不做切分** —— 发送端 `%d%d%d` 既没有分隔符也没有
 *         定宽, "2003040" 到底是 (200,30,40) 还是 (20,03,040) 无法唯一确定。
 *         看真实长度和量级自己判断; 要程序化解析必须先改发送端为
 *         `%03d%03d%03d` 或加分隔符。
 * @note   任务上下文调用 (里面有 printf, 别在中断里调)。由 app 的轮询循环调。
 * @note   调试用, 确认收帧正常后可以摘掉。
 */
bool MSP_Color_DebugPoll(void);

/**
 * @brief  诊断计数。
 * @param  rx_bytes    收到的总字节数 (含空白)。
 * @param  rx_chars    非空白字节数。
 * @param  rx_ok       解析成功的帧数。
 * @param  rx_unknown  颜色字符不是 'r'/'b' 的帧数。
 * @note   任一参数可为 NULL。rx_ok==0 说明一帧都没拼出来 (帧头对不上);
 *         rx_ok>0 而取不到色说明是别处的问题。
 */
void MSP_Color_GetDiag(uint32_t *rx_bytes, uint32_t *rx_chars,
                       uint32_t *rx_ok, uint32_t *rx_unknown);

#endif /* MSP_COLOR_H */
