#include "ops9_g491_uart3.h"
#include <string.h>
#include "usart.h"   /* CubeMX 生成的 huart3 句柄，供 locator_ops9.init() 挂接 */
static UART_HandleTypeDef s_owned_huart3;
static UART_HandleTypeDef *s_huart = NULL;

static ops9_t s_ops9;
static uint8_t s_rx_byte;
static volatile uint8_t s_new_data = 0;




static float ops9_float_from_le(const uint8_t b[4])
{
    uint32_t u =
        ((uint32_t)b[0]) |
        ((uint32_t)b[1] << 8) |
        ((uint32_t)b[2] << 16) |
        ((uint32_t)b[3] << 24);

    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static void ops9_float_to_le(float f, uint8_t b[4])
{
    uint32_t u = 0;
    memcpy(&u, &f, sizeof(u));

    b[0] = (uint8_t)(u);
    b[1] = (uint8_t)(u >> 8);
    b[2] = (uint8_t)(u >> 16);
    b[3] = (uint8_t)(u >> 24);
}

static void ops9_decode_payload(ops9_t *ctx)
{
    ctx->latest.heading_deg      = ops9_float_from_le(&ctx->payload[0]);
    ctx->latest.pitch_deg        = ops9_float_from_le(&ctx->payload[4]);
    ctx->latest.roll_deg         = ops9_float_from_le(&ctx->payload[8]);
    ctx->latest.x_mm             = ops9_float_from_le(&ctx->payload[12]);
    ctx->latest.y_mm             = ops9_float_from_le(&ctx->payload[16]);
    ctx->latest.heading_rate_dps = ops9_float_from_le(&ctx->payload[20]);

    ctx->valid_frames++;

    if (ctx->frame_cb != NULL)
        ctx->frame_cb(&ctx->latest, ctx->frame_cb_user);
}

void ops9_init(ops9_t *ctx, ops9_frame_callback_t frame_cb, void *user)
{
    if (ctx == NULL)
        return;

    memset(ctx, 0, sizeof(*ctx));
    ctx->state = OPS9_RX_WAIT_HEAD_0D;
    ctx->frame_cb = frame_cb;
    ctx->frame_cb_user = user;
}


/*
 *数据解算
 */
void ops9_input_byte(ops9_t *ctx, uint8_t byte)
{
    if (ctx == NULL)
        return;

    switch (ctx->state)
    {
        case OPS9_RX_WAIT_HEAD_0D:
            if (byte == 0x0D)
                ctx->state = OPS9_RX_WAIT_HEAD_0A;
            break;

        case OPS9_RX_WAIT_HEAD_0A:
            if (byte == 0x0A)
            {
                ctx->payload_index = 0;
                ctx->state = OPS9_RX_PAYLOAD;
            }
            else if (byte != 0x0D)
            {
                ctx->state = OPS9_RX_WAIT_HEAD_0D;
            }
            break;

        case OPS9_RX_PAYLOAD:
            ctx->payload[ctx->payload_index++] = byte;
            if (ctx->payload_index >= OPS9_PAYLOAD_SIZE)
                ctx->state = OPS9_RX_WAIT_TAIL_0A;
            break;

        case OPS9_RX_WAIT_TAIL_0A:
            if (byte == 0x0A)
            {
                ctx->state = OPS9_RX_WAIT_TAIL_0D;
            }
            else
            {
                ctx->bad_frames++;
                ctx->state = (byte == 0x0D) ? OPS9_RX_WAIT_HEAD_0A
                                            : OPS9_RX_WAIT_HEAD_0D;
            }
            break;

        case OPS9_RX_WAIT_TAIL_0D:
            if (byte == 0x0D)
            {
                ops9_decode_payload(ctx);
                ctx->state = OPS9_RX_WAIT_HEAD_0D;
            }
            else
            {
                ctx->bad_frames++;
                ctx->state = (byte == 0x0D) ? OPS9_RX_WAIT_HEAD_0A
                                            : OPS9_RX_WAIT_HEAD_0D;
            }
            break;

        default:
            ctx->payload_index = 0;
            ctx->state = OPS9_RX_WAIT_HEAD_0D;
            break;
    }
}

void ops9_input(ops9_t *ctx, const uint8_t *data, size_t len)
{
    size_t i;

    if (ctx == NULL || data == NULL)
        return;

    for (i = 0; i < len; ++i)
        ops9_input_byte(ctx, data[i]);
}

uint8_t ops9_get_latest(const ops9_t *ctx, ops9_data_t *out)
{
    if (ctx == NULL || out == NULL || ctx->valid_frames == 0)
        return 0;

    *out = ctx->latest;
    return 1;
}

static uint8_t send4(ops9_tx_callback_t tx, void *user, const char cmd[4])
{
    if (tx == NULL)
        return 1;
    return tx((const uint8_t *)cmd, 4u, user);
}

static uint8_t send4_float(ops9_tx_callback_t tx, void *user,
                       const char cmd[4], float value)
{
    uint8_t packet[8];

    if (tx == NULL)
        return -1;

    packet[0] = (uint8_t)cmd[0];
    packet[1] = (uint8_t)cmd[1];
    packet[2] = (uint8_t)cmd[2];
    packet[3] = (uint8_t)cmd[3];
    ops9_float_to_le(value, &packet[4]);

    return tx(packet, sizeof(packet), user);
}

uint8_t ops9_send_calibrate(ops9_tx_callback_t tx, void *user)
{
    static const char cmd[4] = {'A','C','T','R'};
    return send4(tx, user, cmd);
}

uint8_t ops9_send_zero(ops9_tx_callback_t tx, void *user)
{
    static const char cmd[4] = {'A','C','T','0'};
    return send4(tx, user, cmd);
}

uint8_t ops9_send_set_heading(ops9_tx_callback_t tx, void *user, float heading_deg)
{
    static const char cmd[4] = {'A','C','T','J'};
    return send4_float(tx, user, cmd, heading_deg);
}

uint8_t ops9_send_set_x(ops9_tx_callback_t tx, void *user, float x_mm)
{
    static const char cmd[4] = {'A','C','T','X'};
    return send4_float(tx, user, cmd, x_mm);
}

uint8_t ops9_send_set_y(ops9_tx_callback_t tx, void *user, float y_mm)
{
    static const char cmd[4] = {'A','C','T','Y'};
    return send4_float(tx, user, cmd, y_mm);
}


/* HAL_UART_Transmit -> ops9 通用驱动的发送适配层 */
static uint8_t ops9_hal_tx(const uint8_t *data, size_t len, void *user)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)user;

    if (huart == NULL || data == NULL || len > 0xFFFFu)
        return 1;

    return (HAL_UART_Transmit(huart, (uint8_t *)data,
                             (uint16_t)len, 100u) == HAL_OK) ? 0 : 1;
}

static void ops9_on_frame(const ops9_data_t *data, void *user)
{
    (void)data;
    (void)user;

    /* 只置标志，避免在串口中断上下文中做耗时工作。 */
    s_new_data = 1u;
}

static HAL_StatusTypeDef start_rx_it(void)
{
    if (s_huart == NULL)
        return HAL_ERROR;

    return HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

HAL_StatusTypeDef OPS9_G491_UART3_Attach(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART3)
        return HAL_ERROR;

    s_huart = huart;
    ops9_init(&s_ops9, ops9_on_frame, NULL);
    s_new_data = 0u;

    /*
     * 防止之前存在未清状态导致第一次接收失败。
     * HAL UART 错误标志在 IRQ 中会继续处理；这里直接启动即可。
     */
    return start_rx_it();
}


UART_HandleTypeDef *OPS9_G491_UART3_GetHandle(void)
{
    return s_huart;
}


void OPS9_G491_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart != s_huart)
        return;

    ops9_input_byte(&s_ops9, s_rx_byte);

    /* 连续接收下一个字节 */
    (void)start_rx_it();
}

void OPS9_G491_UART3_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart != s_huart)
        return;

    /*
     * HAL 在 ORE / FE / NE 等错误后可能停止当前 IT 接收。
     * 先终止接收，再重新启动。
     */
    (void)HAL_UART_AbortReceive(huart);
    (void)start_rx_it();
}

uint8_t OPS9_G491_UART3_GetLatest(ops9_data_t *out)
{
    uint32_t primask;
    uint8_t  has_data = 0;

    if (out == NULL || s_huart == NULL)
        return 0;

    /*
     * s_ops9.latest 在 USART3 中断里更新。
     * 复制 24 字节结构体时短暂屏蔽中断，防止读到半帧新、半帧旧的数据。
     */
    primask = __get_PRIMASK();
    __disable_irq();

    if (s_new_data != 0u && s_ops9.valid_frames != 0u)
    {
        *out = s_ops9.latest;
        s_new_data = 0u;
        has_data = 1;
    }

    if (primask == 0u)
        __enable_irq();

    return has_data;
}

uint32_t OPS9_G491_UART3_GetValidFrameCount(void)
{
    return s_ops9.valid_frames;
}

uint32_t OPS9_G491_UART3_GetBadFrameCount(void)
{
    return s_ops9.bad_frames;
}

static HAL_StatusTypeDef result_to_hal(uint8_t r)
{
    return (r == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef OPS9_G491_UART3_Zero(void)
{
    if (s_huart == NULL) return HAL_ERROR;
    return result_to_hal(ops9_send_zero(ops9_hal_tx, s_huart));
}

HAL_StatusTypeDef OPS9_G491_UART3_StartCalibration(void)
{
    if (s_huart == NULL) return HAL_ERROR;
    return result_to_hal(ops9_send_calibrate(ops9_hal_tx, s_huart));
}

HAL_StatusTypeDef OPS9_G491_UART3_SetHeading(float heading_deg)
{
    if (s_huart == NULL) return HAL_ERROR;
    return result_to_hal(ops9_send_set_heading(ops9_hal_tx, s_huart, heading_deg));
}

HAL_StatusTypeDef OPS9_G491_UART3_SetX(float x_mm)
{
    if (s_huart == NULL) return HAL_ERROR;
    return result_to_hal(ops9_send_set_x(ops9_hal_tx, s_huart, x_mm));
}

HAL_StatusTypeDef OPS9_G491_UART3_SetY(float y_mm)
{
    if (s_huart == NULL) return HAL_ERROR;
    return result_to_hal(ops9_send_set_y(ops9_hal_tx, s_huart, y_mm));
}

HAL_StatusTypeDef OPS9_G491_UART3_SetPose(float heading_deg, float x_mm, float y_mm)
{
    HAL_StatusTypeDef st;

    st = OPS9_G491_UART3_SetHeading(heading_deg);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    st = OPS9_G491_UART3_SetX(x_mm);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    st = OPS9_G491_UART3_SetY(y_mm);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    return HAL_OK;
}

/* ============================================================
 * LocatorDev_t 适配层 --- locator_ops9 实例
 *
 * 通信与解码由本文件前半部分完成（ops9_t 状态机 + USART3 HAL 粘合）。
 * 此处只做三件事：取最新帧 → 单位换算（mm→m / 度→rad）→ 按
 * PoseData_t 契约填充 valid + timestamp。
 *
 * 上层（业务层）只准经 LocatorDev_t 接口访问本驱动，不得直接调用
 * 上面的 OPS9_G491_UART3_* / ops9_* 函数（CLAUDE.md 第 4、5.2 节）。
 * ============================================================ */

#define OPS9_DEG2RAD  0.01745329252f   /* pi / 180 */
#define OPS9_PI       3.14159265359f

static PoseData_t s_pose;             /* 最新位姿缓存（m / rad） */
static uint32_t   s_last_frame_tick;  /* 最后收到合法帧的时刻（ms） */

/**
 * @brief 设备初始化：挂接 USART3 并复位软件状态
 * @note  USART3 外设初始化由 CubeMX 完成（main.c 的 MX_USART3_UART_Init），
 *        此处只做挂接与状态复位，不重复初始化外设（同 locator_wheel 先例：
 *        外设初始化在别处，本驱动只清软件状态）。
 */
static void ops9_loc_init(void)
{
    if (s_huart == NULL) {
        (void)OPS9_G491_UART3_Attach(&huart3);
    }

    s_pose.x     = 0.0f;
    s_pose.y     = 0.0f;
    s_pose.yaw   = 0.0f;
    s_pose.pitch = 0.0f;
    s_pose.roll  = 0.0f;

    /* OPS9 不输出车体线速度 vx/vy，恒为 0（同 locator_wheel 先例） */
    s_pose.vx = 0.0f;
    s_pose.vy = 0.0f;
    s_pose.wz = 0.0f;

    s_pose.valid     = 0U;
    s_pose.timestamp = 0U;

    s_last_frame_tick = 0U;
}

/**
 * @brief 周期更新：取最新帧、单位换算、刷新位姿缓存
 * @note  这是【有副作用】的接口，必须固定周期调用且全局只调一处。
 *        无新帧时检查帧流超时；超时置 valid=0 并冻结位姿，绝不外推。
 *
 * 坐标映射：此处按直通映射（x/y 即 OPS9 输出坐标，yaw 即 OPS9 heading）。
 * OPS9 的 heading 正方向与安装方位未经上车实测，若与车体系约定
 * （X=前方、Y=左方、yaw CCW 为正，CLAUDE.md 第 3 节）不一致，
 * 只需调整本函数内的符号与轴对应，不影响上层。
 */
static void ops9_loc_update(void)
{
    ops9_data_t raw;

    if (OPS9_G491_UART3_GetLatest(&raw)) {
        /* 单位换算：mm→m、度→rad（PoseData_t 契约，CLAUDE.md 第 3 节） */
        s_pose.x     = raw.x_mm / 1000.0f;
        s_pose.y     = raw.y_mm / 1000.0f;
        s_pose.yaw   = raw.heading_deg * OPS9_DEG2RAD;

        /* yaw 归一化到 [-π, π] */
        while (s_pose.yaw >  OPS9_PI) { s_pose.yaw -= 2.0f * OPS9_PI; }
        while (s_pose.yaw < -OPS9_PI) { s_pose.yaw += 2.0f * OPS9_PI; }

        s_pose.pitch = raw.pitch_deg * OPS9_DEG2RAD;
        s_pose.roll  = raw.roll_deg  * OPS9_DEG2RAD;
        s_pose.wz    = raw.heading_rate_dps * OPS9_DEG2RAD;

        s_pose.valid      = 1U;
        s_pose.timestamp  = HAL_GetTick();
        s_last_frame_tick = s_pose.timestamp;
    } else if ((s_pose.valid != 0U) && (s_last_frame_tick != 0U) &&
               (HAL_GetTick() - s_last_frame_tick > OPS9_FRAME_TIMEOUT_MS)) {
        /* 帧流中断：判离线，位姿冻结并标记不可信 */
        s_pose.valid = 0U;
    }
}

/**
 * @brief 获取最新位姿
 * @note  纯读取，无副作用，可任意频率调用。
 */
static void ops9_loc_get_pose(PoseData_t *pose_out)
{
    if (pose_out == NULL) {
        return;
    }
    *pose_out = s_pose;
}

/**
 * @brief 设备健康检查
 * @return 1 = 帧流正常（在 OPS9_FRAME_TIMEOUT_MS 内收到过合法帧）；0 = 离线/不可信
 */
static uint8_t ops9_loc_is_healthy(void)
{
    return s_pose.valid;
}

/* ============================================================
 * 设备实例（注册见 CLAUDE.md 第 4 节 / 7.1 节）
 * ============================================================ */

const LocatorDev_t locator_ops9 = {
    .init       = ops9_loc_init,
    .update     = ops9_loc_update,
    .get_pose   = ops9_loc_get_pose,
    .is_healthy = ops9_loc_is_healthy,
};
