/**
 * @file    drv_wheel_odom.h
 * @brief   轮式里程计定位驱动 --- 抽象设备层的 locator_wheel 实例
 *
 * 规范来源：《机器人嵌入式代码架构规范与修改日志》V1.0 第 6.1 节。
 */

#ifndef _DRV_WHEEL_ODOM_H_
#define _DRV_WHEEL_ODOM_H_

#include "locator_dev.h"

/**
 * @brief 轮式里程计设备实例
 *
 * 定位源契约：本驱动对上层承诺输出【世界系 x / y / yaw】三个量。
 *     x, y —— 四轮编码器增量推算（相对里程）
 *     yaw  —— HWT906 实测航向（hardware/hwt_imu.c 的 g_hwt_imu_yaw_rad）
 *
 * yaw 取自 IMU 是【有意设计】，不是技术债：轮式里程计给不出绝对航向，
 * 而后续替换的 OPS9 定位模块会直接输出 x/y/yaw。两者对上层是同一个契约。
 *
 * @note  用法（业务层/应用层只应经由此结构体访问本驱动）：
 *            locator_wheel.update();          // 固定周期调用一次
 *            locator_wheel.get_pose(&pose);   // 任意频率纯读取
 */
extern const LocatorDev_t locator_wheel;

/**
 * @brief 里程计清零：电机编码器归零 + 位姿 x/y 归零
 * @note  与迁移前 algorithm/Nav_position.c 的 World_Reset() 行为完全一致：
 *        - 4 个电机驱动的编码器计数清零（地址 1~4）
 *        - 位姿 s_pose.x / s_pose.y 清零
 *        - yaw 不归零（由 IMU 实时提供）
 *        - 下一帧 update 以当前编码器为基准重设起点，不跨清零边界累积
 */
void Wheel_Odom_Reset(void);

#endif /* _DRV_WHEEL_ODOM_H_ */
