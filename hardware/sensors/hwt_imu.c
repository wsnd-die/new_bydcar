/**
 * @file    hwt_imu.c
 * @brief   维特智能 (WitMotion) IMU 驱动实现 —— HWT101 / HWT906 通用。
 *          详见 hwt_imu.h 的说明。
 */

#include "hwt_imu.h"
#include "i2c.h"

/* ========================================================================
   全局姿态角数据
   ======================================================================== */

volatile float    g_hwt_imu_roll       = 0.0f;
volatile float    g_hwt_imu_pitch      = 0.0f;
volatile float    g_hwt_imu_yaw        = 0.0f;
volatile float    g_hwt_imu_yaw_rad    = 0.0f;
volatile uint8_t  g_hwt_imu_data_ready = 0U;
volatile uint8_t  g_hwt_imu_online     = 0U;

/* ========================================================================
   全局扩展数据 (V1.11.0 新增, 块读带回)
   ======================================================================== */

volatile float    g_hwt_imu_acc_x      = 0.0f;
volatile float    g_hwt_imu_acc_y      = 0.0f;
volatile float    g_hwt_imu_acc_z      = 0.0f;
volatile float    g_hwt_imu_gyro_x     = 0.0f;
volatile float    g_hwt_imu_gyro_y     = 0.0f;
volatile float    g_hwt_imu_gyro_z     = 0.0f;
volatile float    g_hwt_imu_gyro_z_rad = 0.0f;
volatile int16_t  g_hwt_imu_mag_x      = 0;
volatile int16_t  g_hwt_imu_mag_y      = 0;
volatile int16_t  g_hwt_imu_mag_z      = 0;
volatile float    g_hwt_imu_temp       = 0.0f;
volatile uint8_t  g_hwt_imu_ext_ok     = 0U;

/* ========================================================================
   内部状态
   ======================================================================== */

/** 最近一次读到的原始 yaw (deg, -180..180), 未扣零点 */
static float s_yaw_raw = 0.0f;
/** 零点偏移: 设零点那一刻的原始 yaw。g_hwt_imu_yaw = wrap(raw - s_yaw_offset) */
static float s_yaw_offset = 0.0f;

/** 0x34~0x40 的原始字节缓存。static —— 26 字节不占调用者的任务栈。 */
static uint8_t s_raw[HWT_IMU_BLOCK_LEN];

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

/* 按寄存器索引从块缓存里取一个 int16。
 * 索引即维特协议编号, 每个索引占 2 字节, 故偏移 = (索引 - 0x34) * 2。
 * 姿态角与扩展量的解析共用这一个取数口, 长块读与退化路径的偏移自然一致。 */
static int16_t hwt_imu_block_i16(uint8_t reg)
{
    uint16_t off = (uint16_t)(reg - HWT_IMU_REG_AX) * 2U;
    return hwt_imu_make_i16(&s_raw[off]);
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
    uint8_t ext_ok = 1U;
    float   roll, pitch, yaw;

    /* 一次事务读回 0x34~0x40 整块: 加速度 + 角速度 + 磁场 + 姿态角 + 温度
     * (13 个寄存器 / 26 字节)。 */
    if (HAL_I2C_Mem_Read(&hi2c3, HWT_IMU_I2C_ADDR, HWT_IMU_REG_AX,
                         I2C_MEMADD_SIZE_8BIT, s_raw, HWT_IMU_BLOCK_LEN,
                         HWT_IMU_TIMEOUT_MS) != HAL_OK)
    {
        /* 退化: 只读姿态角块 0x3D~0x3F, 写回块缓存里对应的位置。
         * 万一模块不接受这么长的块读, 航向不会跟着一起失效 —— 只是拿不到
         * 加速度那几个量, 见 g_hwt_imu_ext_ok。解析偏移两条路径完全一致。 */
        ext_ok = 0U;

        if (HAL_I2C_Mem_Read(&hi2c3, HWT_IMU_I2C_ADDR, HWT_IMU_REG_ROLL,
                             I2C_MEMADD_SIZE_8BIT,
                             &s_raw[HWT_IMU_BLOCK_OFF_ANGLE], 6U,
                             HWT_IMU_TIMEOUT_MS) != HAL_OK)
        {
            g_hwt_imu_online = 0U;
            return 0U;
        }
    }

    if (ext_ok != 0U)
    {
        g_hwt_imu_acc_x = (float)hwt_imu_block_i16(HWT_IMU_REG_AX) * HWT_IMU_ACC_LSB_TO_MS2;
        g_hwt_imu_acc_y = (float)hwt_imu_block_i16(HWT_IMU_REG_AY) * HWT_IMU_ACC_LSB_TO_MS2;
        g_hwt_imu_acc_z = (float)hwt_imu_block_i16(HWT_IMU_REG_AZ) * HWT_IMU_ACC_LSB_TO_MS2;

        g_hwt_imu_gyro_x = (float)hwt_imu_block_i16(HWT_IMU_REG_GX) * HWT_IMU_GYRO_LSB_TO_DPS;
        g_hwt_imu_gyro_y = (float)hwt_imu_block_i16(HWT_IMU_REG_GY) * HWT_IMU_GYRO_LSB_TO_DPS;
        g_hwt_imu_gyro_z = (float)hwt_imu_block_i16(HWT_IMU_REG_GZ) * HWT_IMU_GYRO_LSB_TO_DPS;
        g_hwt_imu_gyro_z_rad = g_hwt_imu_gyro_z * HWT_IMU_DEG2RAD;

        /* 磁场按原始值原样给出 (手册未给缩放系数, 不臆造) */
        g_hwt_imu_mag_x = hwt_imu_block_i16(HWT_IMU_REG_HX);
        g_hwt_imu_mag_y = hwt_imu_block_i16(HWT_IMU_REG_HY);
        g_hwt_imu_mag_z = hwt_imu_block_i16(HWT_IMU_REG_HZ);

        g_hwt_imu_temp  = (float)hwt_imu_block_i16(HWT_IMU_REG_TEMP) * HWT_IMU_TEMP_LSB_TO_C;
    }
    g_hwt_imu_ext_ok = ext_ok;

    /* 姿态角: 两条路径共用同一组块内偏移 */
    roll  = (float)hwt_imu_block_i16(HWT_IMU_REG_ROLL)  * HWT_IMU_RAW_TO_DEG;
    pitch = (float)hwt_imu_block_i16(HWT_IMU_REG_PITCH) * HWT_IMU_RAW_TO_DEG;
    yaw   = (float)hwt_imu_block_i16(HWT_IMU_REG_YAW)   * HWT_IMU_RAW_TO_DEG;

    s_yaw_raw = yaw;

    g_hwt_imu_roll    = roll;
    g_hwt_imu_pitch   = pitch;
    g_hwt_imu_yaw     = hwt_imu_wrap_deg(yaw - s_yaw_offset);
    g_hwt_imu_yaw_rad = g_hwt_imu_yaw * HWT_IMU_DEG2RAD;

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
