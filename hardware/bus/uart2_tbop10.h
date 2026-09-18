#ifndef STM32G4_TEST_TBOP10_H
#define STM32G4_TEST_TBOP10_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <string.h>

#ifndef LEGACY_USART2_ODOM_ENABLE
#define LEGACY_USART2_ODOM_ENABLE 1
#endif

#define TB_RX_BUFF_SIZE 64
#define FRAME_TOTAL_LEN 28U
#define TB_TX_BUFF_SIZE 64

#define TB_Header1 0XAA
#define TB_Header2 0XCC
#define TB_Tail1 0XBB
#define TB_Tail2 0XDD

#define DMA_RX_BUF_SIZE 64

#define TBOP_Send(Buffer, DataLen) UART2_Send(Buffer, DataLen)

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

typedef struct
{
    float xdata;
    float ydata;
} TBData_t;

extern TBData_t TB_position;
extern TBData_t TB_speed;

typedef enum
{
    WAIT_HEADER1,
    WAIT_HEADER2,
    RECV_DATA,
    CHECK_TAIL1,
    CHECK_TAIL2,
} UART_STATE;

typedef union
{
    uint8_t data[4];
    float val;
} FloatUnion;

extern float imu_gz;
extern float imu_yaw;
extern uint8_t Flag_TBOFdata;

#if LEGACY_USART2_ODOM_ENABLE
extern UART_STATE fsm_state;
extern uint8_t frame_buf[FRAME_TOTAL_LEN];
extern uint8_t frame_index;
extern uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
extern volatile uint32_t dbg_rx_cb;   /* USART2 DMA 回调次数, 诊断用 */
void UART2_FSM_Parse_Byte(uint8_t byte);
#endif

/* ---- OpenMV 颜色帧 (UART2, AA l_black l_mean A B DD 6字节) ---- */
extern volatile uint8_t g_uart2_color_ready;
extern uint8_t g_uart2_color_l, g_uart2_color_l_mean, g_uart2_color_a, g_uart2_color_b;

/* ---- GY-33 颜色帧 (UART2, 5A 5A 45 03 R G B chk) ---- */
extern volatile uint8_t g_uart2_gy33_ready;
extern uint8_t g_uart2_gy33_r, g_uart2_gy33_g, g_uart2_gy33_b;

void UART2_StartDMAReceive(void);
void UART2_calibrate(void);

#endif
