/**
 * @file    HWT906.h
 * @brief   HWT906 陀螺仪设备实例 --- 抽象设备层 (Device)
 *
 * 包装 hardware/sensors/hwt_imu.c 驱动，向上提供 LocatorDev_t 标准接口
 * （CLAUDE.md 第 4 节）。
 *
 * 【重要】本设备是【姿态源】，不是【定位源】：
 *   - 输出：yaw / pitch / roll（rad）、wz（rad/s，直接取陀螺仪 GZ）、
 *     以及三轴线加速度 ax / ay / az（m/s^2）；
 *   - x / y / vx / vy 无数据源，恒为 0；
 *   - 因此不能充当 active_locator 的定位源（定位用 locator_wheel /
 *     locator_ops9）。
 *
 * 【V1.11.0 行为变更】wz 的数据源从"yaw 差分推算"改成了"陀螺仪 GZ
 *   直接读取"。这是本设备对外的唯一行为变更，angle_ctrl.c 的两级 PID
 *   此前按差分信号的噪声特性整定，需重新确认增益。
 *   另：驱动层已把磁场与温度一并读回，但本实例不取 —— 需要就直接读
 *   hardware/sensors/hwt_imu.h 的 g_hwt_imu_mag_* / g_hwt_imu_temp。
 *
 * @note  用法（同 locator_wheel）：
 *            imu_hwt906.init();
 *            imu_hwt906.update();          // 固定周期调用一次（内部 HWT_IMU_Poll）
 *            imu_hwt906.get_pose(&pose);   // 任意频率纯读取
 */

#ifndef __HWT906_DEV_H
#define __HWT906_DEV_H

#include "locator_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const LocatorDev_t imu_hwt906;

#ifdef __cplusplus
}
#endif

#endif /* __HWT906_DEV_H */
