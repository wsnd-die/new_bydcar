//
// Created by 35037 on 2026/9/26.
//
#include "NX_uart.h"
#include "Common_used.h"

/* ================================================================
   UART4 RX —— DMA + IDLE

   与 USART3(OPS9, device/ops9_g491_uart3.c) 同款写法：一次挂 64 字节缓冲，
   总线 IDLE 或缓冲收满时进 Core/Src/usart.c 的 HAL_UARTEx_RxEventCallback()
   分发器，逐字节喂下面的状态机，然后重挂。

   V1.15.x 之前是 HAL_UART_Receive_DMA(&huart4, &rx4, 1) —— 逐个字节重挂
   DMA，115200 下约 11500 次/秒 DMA 冷启动，且没有任何"完成"回调把它接上，
   实际收一个字节就停。
   ================================================================ */

#define NX_RX_BUF_SIZE  64u

/** DMA 落点缓冲。收发字节先进这里，再由 NX_RxEventCallback 逐字节喂状态机。 */
static uint8_t s_rx_buf[NX_RX_BUF_SIZE];

/* ---- 接收状态机 ---- */
typedef enum {
    NX_RX_WAIT_A3 = 0,
    NX_RX_WAIT_BX,
    NX_RX_COLLECT,
} NX_rx_state_t;

static struct {
    /* 模式管理 */
    uint8_t  requested_mode;
    uint8_t  current_mode;

    /* 接收状态机 */
    NX_rx_state_t rx_state;
    uint8_t  rx_pkt_type;       /* 0xB3=角度, 0xB4=方向, 0xB5=位置 */
    uint8_t  rx_buf[8];
    uint8_t  rx_idx;

    /* 解析结果 */
    char     dir;              /* 找圆方向 */
    float    pos_x, pos_y;     /* 位置偏移 (像素) */
    float    circle_x, circle_y;

    uint8_t  dir_fresh;
    uint8_t  pos_fresh;
    uint8_t circle_flesh;

    /* 诊断 */
    uint32_t rx_bytes;
    uint32_t rx_ok;
    uint32_t rx_err;
    uint32_t rx_unk;
} NX_ctx;

/* ================================================================ */

/**
 * @brief 挂上 DMA-IDLE 接收（内部用，NX_Init / 回调 / 错误恢复都走它）。
 * @note  与 OPS9 侧同款：清 ORE/FE/NE 后启动，再关掉半传输中断。
 */
static void NX_StartRx(void)
{
    __HAL_UART_CLEAR_FLAG(&huart4,
        UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_rx_buf, NX_RX_BUF_SIZE) != HAL_OK) {
        return;
    }

    /* ReceiveToIdle_DMA 内部走 HAL_DMA_Start_IT，会把半传输(HT)中断一并打开；
     * 缓冲收到一半（32 字节）时 UART_DMARxHalfCplt 同样触发 RxEvent，会把一次
     * 接收切成两截。本驱动按「总线 IDLE + 收满」两种事件处理，不需要 HT。 */
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

void NX_Init(void)
{
    memset(&NX_ctx, 0, sizeof(NX_ctx));
    NX_ctx.rx_state = NX_RX_WAIT_A3;
    NX_StartRx();
}

/**
 * @brief DMA-IDLE 事件：把本批收到的字节逐字节喂状态机，然后重挂。
 * @note  由 Core/Src/usart.c 的 HAL_UARTEx_RxEventCallback() 分发器调用。
 *        帧被跨回调切分不影响解析 —— 状态机状态跨调用保持。
 */
void NX_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART4) {
        return;
    }

    for (uint16_t i = 0; i < Size; i++) {
        NX_RxProcessByte(s_rx_buf[i]);
    }

    NX_StartRx();
}

void NX_RxProcessByte(uint8_t b)
{
    NX_ctx.rx_bytes++;

    switch (NX_ctx.rx_state) {

    case NX_RX_WAIT_A3:
        if (b == 0xA3) {
            NX_ctx.rx_idx = 0;
            NX_ctx.rx_buf[NX_ctx.rx_idx++] = b;
            NX_ctx.rx_state = NX_RX_WAIT_BX;
        }
        break;

    case NX_RX_WAIT_BX:
        if (b == 0xB3 || b == 0xB4 || b==0xB5) {
            NX_ctx.rx_pkt_type = b;
            NX_ctx.rx_buf[NX_ctx.rx_idx++] = b;
            NX_ctx.rx_state = NX_RX_COLLECT;
        } else if (b == 0xA3) {
            /* 重新同步 */
            NX_ctx.rx_idx = 0;
            NX_ctx.rx_buf[NX_ctx.rx_idx++] = b;
        } else {
            NX_ctx.rx_state = NX_RX_WAIT_A3;
            NX_ctx.rx_err++;
        }
        break;

    case NX_RX_COLLECT:
        NX_ctx.rx_buf[NX_ctx.rx_idx++] = b;

        if (b == 0xFF) {
            /* 包结束 */
            if (NX_ctx.rx_pkt_type == 0xB3 && NX_ctx.rx_idx >= 5) {
                /* [A3,B3,aH,aL,(xH,xL,)FF] — 5B=仅角度, 7B=角度+横轴位置 */
                int16_t raw = (int16_t)((NX_ctx.rx_buf[2] << 8) | NX_ctx.rx_buf[3]);
                // NX_ctx.angle = raw / 100.0f;
                // NX_ctx.angle_fresh = 1;

                if (NX_ctx.rx_idx >= 7) {
                    int16_t rx = (int16_t)((NX_ctx.rx_buf[4] << 8) | NX_ctx.rx_buf[5]);
                    NX_ctx.pos_x = rx / 100.0f;
                    NX_ctx.pos_fresh = 1;
                }
                NX_ctx.rx_ok++;
            } else if (NX_ctx.rx_pkt_type == 0xB4 && NX_ctx.rx_idx >= 4) {
                /* 方向: [A3, B4, dir, FF] */
                NX_ctx.dir = (char)NX_ctx.rx_buf[2];
                NX_ctx.dir_fresh = 1;
                NX_ctx.rx_ok++;
            }
            else if (NX_ctx.rx_pkt_type == 0xB5 && NX_ctx.rx_idx >= 7)
            {
                int16_t raw=(int16_t)((NX_ctx.rx_buf[2] << 8) | NX_ctx.rx_buf[3]);
                int16_t rx = (int16_t)((NX_ctx.rx_buf[4] << 8) | NX_ctx.rx_buf[5]);
                NX_ctx.circle_x=raw;
                NX_ctx.circle_y=rx;
                NX_ctx.circle_flesh=1;
                NX_ctx.rx_ok++;
            }

            else {
                NX_ctx.rx_err++;
            }
            NX_ctx.rx_state = NX_RX_WAIT_A3;
        }

        if (NX_ctx.rx_idx >= 8) {
            /* 溢出 */
            NX_ctx.rx_state = NX_RX_WAIT_A3;
            NX_ctx.rx_err++;
        }
        break;
    }
    /* 重挂由 NX_RxEventCallback() 统一负责，本函数只管解析。 */
}

void NX_ErrorCallback(void)
{
    NX_ctx.rx_state = NX_RX_WAIT_A3;
    NX_ctx.rx_idx = 0;
    NX_StartRx();   /* 内部已清 ORE/FE/NEF */
}

/* ---- 发送 ---- */

static void NX_send_cmd(uint8_t cmd)
{
    while (!(UART4->ISR & USART_ISR_TXE)) {}
    UART4->TDR = cmd;
    while (!(UART4->ISR & USART_ISR_TC)) {}
}

/* ---- 模式管理 ---- */

void NX_RequestMode(uint8_t mode)
{
    NX_ctx.requested_mode = mode;
}

void NX_ApplyMode(void)
{
    if (NX_ctx.requested_mode == 0) return;
    if (NX_ctx.requested_mode == NX_ctx.current_mode) return;

    NX_send_cmd(NX_ctx.requested_mode);
    NX_ctx.current_mode = NX_ctx.requested_mode;
}

void NX_SetMode(uint8_t mode)
{
    NX_send_cmd(mode);
    NX_ctx.current_mode = mode;
}

/* ---- 数据读取 ---- */

// bool NX_GetLineAngle(float *angle)
// {
//     if (!NX_ctx.angle_fresh) return false;
//     if (angle) *angle = NX_ctx.angle;
//     NX_ctx.angle_fresh = 0;
//     return true;
// }

bool NX_GetCircleDir(char *dir)
{
    if (!NX_ctx.dir_fresh) return false;
    if (dir) *dir = NX_ctx.dir;
    NX_ctx.dir_fresh = 0;
    return true;
}

bool NX_GetPosition(float *x, float *y)
{
    if (!NX_ctx.pos_fresh) return false;
    if (x) *x = NX_ctx.pos_x;
    if (y) *y = NX_ctx.pos_y;
    NX_ctx.pos_fresh = 0;
    return true;
}
bool NX_GetCirclepos(float *cx,float *cy)
{

    if (cx) *cx = NX_ctx.circle_x;
    if (cy) *cy = NX_ctx.circle_y;
    return true;
}

void NX_GetDiag(uint32_t *rx_bytes, uint32_t *rx_ok,
                  uint32_t *rx_err, uint32_t *rx_unk)
{
    if (rx_bytes) *rx_bytes = NX_ctx.rx_bytes;
    if (rx_ok)    *rx_ok    = NX_ctx.rx_ok;
    if (rx_err)   *rx_err   = NX_ctx.rx_err;
    if (rx_unk)   *rx_unk   = NX_ctx.rx_unk;
}
