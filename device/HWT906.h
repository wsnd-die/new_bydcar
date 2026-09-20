/**
 * @file    HWT906.h
 * @brief   HWT906 陀螺仪设备实例 --- 抽象设备层 (Device)
 *
 * 包装 hardware/sensors/hwt_imu.c 驱动，向上提供 LocatorDev_t 标准接口
 * （CLAUDE.md 第 4 节）。
 *
 * 【重要】本设备是【姿态源】，不是【定位源】：
 *   - 输出：yaw / pitch / roll（rad）与 wz（rad/s，yaw 差分得到）；
 *   - x / y / vx / vy 无数据源，恒为 0；
 *   - 因此不能充当 active_locator 的定位源（定位用 locator_wheel /
 *     locator_ops9）。
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
