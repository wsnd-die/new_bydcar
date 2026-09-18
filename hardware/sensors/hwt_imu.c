/**
 * @file    hwt_imu.c
 * @brief   维特智能 (WitMotion) IMU 驱动实现 —— HWT101 / HWT906 通用。
 *          详见 hwt_imu.h 的说明。
 */

#include "hwt_imu.h"
#include "i2c.h"

/* ========================================================================
   全局角度数据
   ======================================================================== */

volatile float    g_hwt_imu_roll       = 0.0f;
volatile float    g_hwt_imu_pitch      = 0.0f;
volatile float    g_hwt_imu_yaw        = 0.0f;
volatile float    g_hwt_imu_yaw_rad    = 0.0f;
volatile uint8_t  g_hwt_imu_data_ready = 0U;
volatile uint8_t  g_hwt_imu_online     = 0U;

/* ========================================================================
   内部状态
   ======================================================================== */

/** 最近一次读到的原始 yaw (deg, -180..180), 未扣零点 */
static float s_yaw_raw = 0.0f;
/** 零点偏移: 设零点那一刻的原始 yaw。g_hwt_imu_yaw = wrap(raw - s_yaw_offset) */
static float s_yaw_offset = 0.0f;

/* 把角度折算到 -180..180 */
static float hwt_imu_wrap_deg(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* 拼小端 16 位 */
static int16_t hwt_imu_make_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

/* ========================================================================
   底层 I2C
   ======================================================================== */

int16_t HWT_IMU_ReadReg(uint8_t reg)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c3, HWT_IMU_I2C_ADDR, (uint16_t)reg,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                         HWT_IMU_TIMEOUT_MS) != HAL_OK)
    {
        return 0;
    }
    return hwt_imu_make_i16(buf);
}

/* ========================================================================
   API
   ======================================================================== */

void HWT_IMU_SetZero(void)
{
    /* 以"当前朝向"作为新的 0° —— 直接记下此刻的原始角即可,
     * 不做累加, 避免反复调用时误差堆积。 */
    s_yaw_offset = s_yaw_raw;

    g_hwt_imu_yaw        = 0.0f;
    g_hwt_imu_yaw_rad    = 0.0f;
}

uint8_t HWT_IMU_Poll(void)
{
    uint8_t buf[6];
    float   roll, pitch, yaw;

    /* Roll/Pitch/Yaw 在 0x3D..0x42 连续排列, 一次读 6 字节 */
    if (HAL_I2C_Mem_Read(&hi2c3, HWT_IMU_I2C_ADDR, HWT_IMU_REG_ROLL,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                         HWT_IMU_TIMEOUT_MS) != HAL_OK)
    {
        g_hwt_imu_online = 0U;
        return 0U;
    }

    roll  = (float)hwt_imu_make_i16(&buf[0]) * HWT_IMU_RAW_TO_DEG;
    pitch = (float)hwt_imu_make_i16(&buf[2]) * HWT_IMU_RAW_TO_DEG;
    yaw   = (float)hwt_imu_make_i16(&buf[4]) * HWT_IMU_RAW_TO_DEG;

    s_yaw_raw = yaw;

    g_hwt_imu_roll    = roll;
    g_hwt_imu_pitch   = pitch;
    g_hwt_imu_yaw     = hwt_imu_wrap_deg(yaw - s_yaw_offset);
    g_hwt_imu_yaw_rad = g_hwt_imu_yaw * (3.14159265358979f / 180.0f);

    g_hwt_imu_online     = 1U;
    g_hwt_imu_data_ready = 1U;
    return 1U;
}

uint8_t HWT_IMU_Init(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c3, HWT_IMU_I2C_ADDR, 2U,
                              HWT_IMU_TIMEOUT_MS) != HAL_OK)
    {
        g_hwt_imu_online = 0U;
        return 0U;
    }

    g_hwt_imu_online = 1U;

    /* 上电时把当前朝向记为零点 —— 调用前请让车头对准希望作为 0° 的方向。
     * 不想要这个行为就删掉下面三行, 之后手动调 HWT_IMU_SetZero()。 */
    s_yaw_offset = 0.0f;
    (void)HWT_IMU_Poll();
    HWT_IMU_SetZero();

    return 1U;
}