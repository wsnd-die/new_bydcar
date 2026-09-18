#ifndef OPS9_G491_UART3_H
#define OPS9_G491_UART3_H

#include "stm32g4xx_hal.h"
#include "ops9.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPS9_FRAME_SIZE        28u
#define OPS9_PAYLOAD_SIZE      24u
#define OPS9_UART_BAUDRATE     115200u

typedef struct
{
    float heading_deg;
    float pitch_deg;
    float roll_deg;
    float x_mm;
    float y_mm;
    float heading_rate_dps;
} ops9_data_t;

typedef void (*ops9_frame_callback_t)(const ops9_data_t *data, void *user);
typedef uint8_t  (*ops9_tx_callback_t)(const uint8_t *data, size_t len, void *user);

typedef enum
{
    OPS9_RX_WAIT_HEAD_0D = 0,
    OPS9_RX_WAIT_HEAD_0A,
    OPS9_RX_PAYLOAD,
    OPS9_RX_WAIT_TAIL_0A,
    OPS9_RX_WAIT_TAIL_0D
} ops9_rx_state_t;

typedef struct
{
    ops9_rx_state_t state;
    uint8_t payload[OPS9_PAYLOAD_SIZE];
    size_t payload_index;

    ops9_data_t latest;
    uint32_t valid_frames;
    uint32_t bad_frames;

    ops9_frame_callback_t frame_cb;
    void *frame_cb_user;
} ops9_t;


void ops9_init(ops9_t *ctx, ops9_frame_callback_t frame_cb, void *user);
void ops9_input_byte(ops9_t *ctx, uint8_t byte);
void ops9_input(ops9_t *ctx, const uint8_t *data, size_t len);
uint8_t  ops9_get_latest(const ops9_t *ctx, ops9_data_t *out);

uint8_t ops9_send_calibrate(ops9_tx_callback_t tx, void *user); /* ACTR */
uint8_t ops9_send_zero(ops9_tx_callback_t tx, void *user);      /* ACT0 */
uint8_t ops9_send_set_heading(ops9_tx_callback_t tx, void *user, float heading_deg); /* ACTJ + float */
uint8_t ops9_send_set_x(ops9_tx_callback_t tx, void *user, float x_mm);              /* ACTX + float */
uint8_t ops9_send_set_y(ops9_tx_callback_t tx, void *user, float y_mm);              /* ACTY + float */



/*
 * 固定硬件：
 *   MCU: STM32G491VET6
 *   USART3_TX: PB10, AF7
 *   USART3_RX: PE15, AF7
 *   115200 / 8N1 / no flow control
 *
 * 两种用法：
 *
 * A. 独立初始化 USART3:
 *      if (OPS9_G491_UART3_InitStandalone() != HAL_OK) Error_Handler();
 *
 * B. CubeMX 已经生成 huart3:
 *      OPS9_G491_UART3_Attach(&huart3);
 *    此时 CubeMX 中请把 USART3 配成 PB10 TX / PE15 RX，并打开 USART3 中断。
 */

/* 挂接到 CubeMX/用户已经初始化好的 USART3 句柄，并启动 1 字节中断接收。 */
HAL_StatusTypeDef OPS9_G491_UART3_Attach(UART_HandleTypeDef *huart);

/* 当前驱动使用的 UART 句柄。 */
UART_HandleTypeDef *OPS9_G491_UART3_GetHandle(void);

/*
 * 如果使用 InitStandalone()，USART3_IRQHandler() 中调用它。
 * 如果 CubeMX 的 USART3_IRQHandler 已经调用 HAL_UART_IRQHandler(&huart3)，
 * 则不要重复调用此函数。
 */
void OPS9_G491_UART3_IRQHandler(void);

/* 放进 HAL_UART_RxCpltCallback()。 */
void OPS9_G491_UART3_RxCpltCallback(UART_HandleTypeDef *huart);

/* 放进 HAL_UART_ErrorCallback()，发生 ORE/FE/NE 后自动重启接收。 */
void OPS9_G491_UART3_ErrorCallback(UART_HandleTypeDef *huart);

/* 是否收到过新的合法帧；读取后清除 new-data 标志。 */
uint8_t OPS9_G491_UART3_GetLatest(ops9_data_t *out);

/* 统计信息 */
uint32_t OPS9_G491_UART3_GetValidFrameCount(void);
uint32_t OPS9_G491_UART3_GetBadFrameCount(void);

/* OPS-9 常用命令 */
HAL_StatusTypeDef OPS9_G491_UART3_Zero(void);
HAL_StatusTypeDef OPS9_G491_UART3_StartCalibration(void);
HAL_StatusTypeDef OPS9_G491_UART3_SetHeading(float heading_deg);
HAL_StatusTypeDef OPS9_G491_UART3_SetX(float x_mm);
HAL_StatusTypeDef OPS9_G491_UART3_SetY(float y_mm);

/* 按手册在命令之间留 10ms。 */
HAL_StatusTypeDef OPS9_G491_UART3_SetPose(float heading_deg, float x_mm, float y_mm);

#ifdef __cplusplus
}
#endif

#endif
