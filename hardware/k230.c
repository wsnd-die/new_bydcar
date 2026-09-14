/**
 * @file k230.c
 * @brief K230 视觉模块通信 — USART3 二进制协议
 *
 * TX (STM32 -> K230):  单字符命令 'f'=循迹, 'c'=找圆, 'x'=停止
 * RX (K230 -> STM32):
 *   角度:     0xA3 0xB3 [aH][aL] 0xFF                     → angle = int16/100
 *   角度+位置: 0xA3 0xB3 [aH][aL][xH][xL] 0xFF    → angle + pos_x,y
 *   方向:     0xA3 0xB4 [dir] 0xFF                        → dir_char
 */

#include "Common_used.h"
uint8_t rx3;
/* ---- 接收状态机 ---- */
typedef enum {
    K230_RX_WAIT_A3 = 0,
    K230_RX_WAIT_BX,
    K230_RX_COLLECT,
} k230_rx_state_t;

static struct {
    /* 模式管理 */
    uint8_t  requested_mode;
    uint8_t  current_mode;

    /* 接收状态机 */
    k230_rx_state_t rx_state;
    uint8_t  rx_pkt_type;       /* 0xB3=角度, 0xB4=方向, 0xB5=位置 */
    uint8_t  rx_buf[8];
    uint8_t  rx_idx;

    /* 解析结果 */
    float    angle;            /* 循迹角度 (°) */
    char     dir;              /* 找圆方向 */
    float    pos_x, pos_y;     /* 位置偏移 (像素) */
    float    circle_x, circle_y;
    uint8_t  angle_fresh;
    uint8_t  dir_fresh;
    uint8_t  pos_fresh;
    uint8_t circle_flesh;

    /* 诊断 */
    uint32_t rx_bytes;
    uint32_t rx_ok;
    uint32_t rx_err;
    uint32_t rx_unk;
} k230_ctx;

/* ================================================================ */

void K230_Init(void)
{
    memset(&k230_ctx, 0, sizeof(k230_ctx));
    k230_ctx.rx_state = K230_RX_WAIT_A3;
    HAL_UART_Receive_IT(&huart3, &rx3, 1);
}

void K230_RxProcessByte(void)
{
    uint8_t b = rx3;
    k230_ctx.rx_bytes++;

    switch (k230_ctx.rx_state) {

    case K230_RX_WAIT_A3:
        if (b == 0xA3) {
            k230_ctx.rx_idx = 0;
            k230_ctx.rx_buf[k230_ctx.rx_idx++] = b;
            k230_ctx.rx_state = K230_RX_WAIT_BX;
        }
        break;

    case K230_RX_WAIT_BX:
        if (b == 0xB3 || b == 0xB4 || b==0xB5) {
            k230_ctx.rx_pkt_type = b;
            k230_ctx.rx_buf[k230_ctx.rx_idx++] = b;
            k230_ctx.rx_state = K230_RX_COLLECT;
        } else if (b == 0xA3) {
            /* 重新同步 */
            k230_ctx.rx_idx = 0;
            k230_ctx.rx_buf[k230_ctx.rx_idx++] = b;
        } else {
            k230_ctx.rx_state = K230_RX_WAIT_A3;
            k230_ctx.rx_err++;
        }
        break;

    case K230_RX_COLLECT:
        k230_ctx.rx_buf[k230_ctx.rx_idx++] = b;

        if (b == 0xFF) {
            /* 包结束 */
            if (k230_ctx.rx_pkt_type == 0xB3 && k230_ctx.rx_idx >= 5) {
                /* [A3,B3,aH,aL,(xH,xL,)FF] — 5B=仅角度, 7B=角度+横轴位置 */
                int16_t raw = (int16_t)((k230_ctx.rx_buf[2] << 8) | k230_ctx.rx_buf[3]);
                k230_ctx.angle = raw / 100.0f;
                k230_ctx.angle_fresh = 1;

                if (k230_ctx.rx_idx >= 7) {
                    int16_t rx = (int16_t)((k230_ctx.rx_buf[4] << 8) | k230_ctx.rx_buf[5]);
                    k230_ctx.pos_x = rx / 100.0f;
                    k230_ctx.pos_fresh = 1;
                }
                k230_ctx.rx_ok++;
            } else if (k230_ctx.rx_pkt_type == 0xB4 && k230_ctx.rx_idx >= 4) {
                /* 方向: [A3, B4, dir, FF] */
                k230_ctx.dir = (char)k230_ctx.rx_buf[2];
                k230_ctx.dir_fresh = 1;
                k230_ctx.rx_ok++;
            }
            else if (k230_ctx.rx_pkt_type == 0xB5 && k230_ctx.rx_idx >= 7)
            {
                int16_t raw=(int16_t)((k230_ctx.rx_buf[2] << 8) | k230_ctx.rx_buf[3]);
                int16_t rx = (int16_t)((k230_ctx.rx_buf[4] << 8) | k230_ctx.rx_buf[5]);
                k230_ctx.circle_x=raw;
                k230_ctx.circle_y=rx;
                k230_ctx.circle_flesh=1;
                k230_ctx.rx_ok++;
            }

            else {
                k230_ctx.rx_err++;
            }
            k230_ctx.rx_state = K230_RX_WAIT_A3;
        }

        if (k230_ctx.rx_idx >= 8) {
            /* 溢出 */
            k230_ctx.rx_state = K230_RX_WAIT_A3;
            k230_ctx.rx_err++;
        }
        break;
    }

    HAL_UART_Receive_IT(&huart3, &rx3, 1);
}

void K230_RxRestart(void)
{
    __HAL_UART_CLEAR_FLAG(&huart3,
        UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    k230_ctx.rx_state = K230_RX_WAIT_A3;
    k230_ctx.rx_idx = 0;
    HAL_UART_Receive_IT(&huart3, &rx3, 1);
}

/* ---- 发送 ---- */

static void k230_send_cmd(uint8_t cmd)
{
    while (!(USART3->ISR & USART_ISR_TXE)) {}
    USART3->TDR = cmd;
    while (!(USART3->ISR & USART_ISR_TC)) {}
}

/* ---- 模式管理 ---- */

void K230_RequestMode(uint8_t mode)
{
    k230_ctx.requested_mode = mode;
}

void K230_ApplyMode(void)
{
    if (k230_ctx.requested_mode == 0) return;
    if (k230_ctx.requested_mode == k230_ctx.current_mode) return;

    k230_send_cmd(k230_ctx.requested_mode);
    k230_ctx.current_mode = k230_ctx.requested_mode;
}

void K230_SetMode(uint8_t mode)
{
    k230_send_cmd(mode);
    k230_ctx.current_mode = mode;
}

/* ---- 数据读取 ---- */

bool K230_GetLineAngle(float *angle)
{
    if (!k230_ctx.angle_fresh) return false;
    if (angle) *angle = k230_ctx.angle;
    k230_ctx.angle_fresh = 0;
    return true;
}

bool K230_GetCircleDir(char *dir)
{
    if (!k230_ctx.dir_fresh) return false;
    if (dir) *dir = k230_ctx.dir;
    k230_ctx.dir_fresh = 0;
    return true;
}

bool K230_GetPosition(float *x, float *y)
{
    if (!k230_ctx.pos_fresh) return false;
    if (x) *x = k230_ctx.pos_x;
    if (y) *y = k230_ctx.pos_y;
    k230_ctx.pos_fresh = 0;
    return true;
}
bool K230_GetCirclepos(float *cx,float *cy)
{

    if (cx) *cx = k230_ctx.circle_x;
    if (cy) *cy = k230_ctx.circle_y;
    return true;
}

void K230_GetDiag(uint32_t *rx_bytes, uint32_t *rx_ok,
                  uint32_t *rx_err, uint32_t *rx_unk)
{
    if (rx_bytes) *rx_bytes = k230_ctx.rx_bytes;
    if (rx_ok)    *rx_ok    = k230_ctx.rx_ok;
    if (rx_err)   *rx_err   = k230_ctx.rx_err;
    if (rx_unk)   *rx_unk   = k230_ctx.rx_unk;
}