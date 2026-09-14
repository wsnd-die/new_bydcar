/**
 * @file    hwt_imu.h
 * @brief   维特智能 (WitMotion) IMU 驱动 —— HWT101 / HWT906 通用。
 *
 *          ====== 硬件接口 ======
 *          I2C3: PC8 = SCL, PB5 = SDA (见 Core/Src/i2c.c 的 MX_I2C3_Init)
 *          7 位地址 0x50, 模块改过地址时同步 HWT_IMU_I2C_ADDR。
 *
 *          ====== 为什么不需要姿态解算 ======
 *          这类模块内部已经跑完姿态融合, 对外只给 roll/pitch/yaw, 没有原始
 *          角速度/加速度。因此工程里原来的 ahrs_mahony / siyuan_imu 两套互补
 *          滤波已不再需要 (已移至 obsolete/imu660/), 导航模块直接取
 *          g_hwt_imu_yaw_rad 当航向即可。
 *
 *          ====== 使用 ======
 *          HWT_IMU_Init();          // 探测在线 + 当前航向设为零点
 *          HWT_IMU_Poll();          // 每 5~10ms 调一次
 *          float yaw = g_hwt_imu_yaw_rad;
 *
 * @note    读角度用阻塞式 HAL_I2C_Mem_Read (一次 6 字节, 1MHz 下约 100µs),
 *          无需 DMA —— 本工程 I2C3 没有配置 DMA 通道。
 */

#ifndef __HWT_IMU_H
#define __HWT_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================
   常量
   ======================================================================== */

#define HWT_IMU_I2C_ADDR        (0x50U << 1)   /* I2C 7 位地址 0x50 → HAL 8 位 */
#define HWT_IMU_TIMEOUT_MS      10U            /* 单次传输超时 (ms)            */

/* WitMotion 角度寄存器 (Roll / Pitch / Yaw 连续, 各 2 字节小端) */
#define HWT_IMU_REG_ROLL        0x3DU
#define HWT_IMU_REG_PITCH       0x3EU
#define HWT_IMU_REG_YAW         0x3FU

/* 原始值 → 角度: ±32768 对应 ±180° */
#define HWT_IMU_RAW_TO_DEG      (180.0f / 32768.0f)

/* ========================================================================
   全局角度数据 (HWT_IMU_Poll 更新)
   ======================================================================== */

/** @brief roll (deg), 传感器原始值, 未做安装方向修正 */
extern volatile float    g_hwt_imu_roll;
/** @brief pitch (deg) */
extern volatile float    g_hwt_imu_pitch;
/** @brief yaw (deg), 已扣除 HWT_IMU_SetZero 设下的零点, 范围 -180..180 */
extern volatile float    g_hwt_imu_yaw;
/** @brief yaw 的弧度形式, 供里程计/导航使用 (与旧 siyuan_yaw 同量纲) */
extern volatile float    g_hwt_imu_yaw_rad;
/** @brief 最近一次 HWT_IMU_Poll 是否读到了新数据 */
extern volatile uint8_t  g_hwt_imu_data_ready;
/** @brief 模块是否在线 (HWT_IMU_Init 探测结果, Poll 失败会清 0) */
extern volatile uint8_t  g_hwt_imu_online;

/* ========================================================================
   API
   ======================================================================== */

/**
 * @brief  初始化: 探测模块是否在线, 并把当前航向设为零点。
 * @retval 1 = 在线, 0 = 无应答 (检查接线/上拉/地址)
 * @note   需在外设初始化之后调用 (MX_I2C3_Init 之后)。
 */
uint8_t HWT_IMU_Init(void);

/**
 * @brief  读一帧 roll/pitch/yaw 并刷新全局量。
 * @retval 1 = 成功, 0 = I2C 无应答 (全局量保持上一次的值, g_hwt_imu_online 清 0)
 */
uint8_t HWT_IMU_Poll(void);

/**
 * @brief  把当前航向记为零点 (车头对准希望作为 0° 的方向时调用)。
 */
void HWT_IMU_SetZero(void);

/**
 * @brief  读单个寄存器的原始值 (调试 / 高级用法)。
 * @param  reg  寄存器地址, 如 HWT_IMU_REG_YAW
 * @return int16 原始值
 */
int16_t HWT_IMU_ReadReg(uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __HWT_IMU_H */