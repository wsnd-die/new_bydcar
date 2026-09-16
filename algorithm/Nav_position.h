//
// Created by 35037 on 2026/8/12.
//
#ifndef STM32G4_TEST_NAV_POSITION_H
#define STM32G4_TEST_NAV_POSITION_H

#include "Common_used.h"

/* ============================================================
 * 【兼容适配层】—— 新代码请勿使用本文件的接口
 *
 * 里程计实现已迁移至 device/drv_wheel_odom.c 的 locator_wheel 实例
 * （抽象设备层，实现 LocatorDev_t 标准接口）。本文件三个符号只为让既有
 * 调用方免改而保留，属过渡性质，阶段 1 收敛调用点后删除。
 *
 * 新代码请改用：
 *     locator_wheel.update();            // 固定周期，全局只调一处
 *     locator_wheel.get_pose(&pose);     // 任意频率纯读取
 * ============================================================ */

/* 当前里程计位姿（已是 locator_wheel 内部缓存的镜像） */
extern World_Dir_t World_position;

/**
 * @brief 增量式编码器里程计，周期性调用（每 10~20ms）
 * @return 当前世界位姿（同时更新全局 World_position）
 * @warning 【消费型接口】每调用一次就推进一帧积分，调用频率会直接影响
 *          里程计推算结果。这是迁移前既有行为，仅为兼容而保留。
 *          需要纯读取请用 locator_wheel.get_pose()。
 */
World_Dir_t World_position_get(void);

/**
 * @brief 清零编码器里程计并重新起算
 * @note  放完 5 物块 / 奖杯段分界时调用: 转调 Wheel_Odom_Reset(),
 *        清电机编码器与位姿 x/y（yaw 不归零）;
 *        下次 World_position_get 以当前编码器为基准重设起点, 不跨清零边界累积。
 */
void World_Reset(void);

/* ============================================================
 * 惯导 (INS) 已移除
 *
 * 原 Ins_t / g_ins / Ins_Init / Ins_Update 依赖 imu660ra 的原始加速度,
 * 换成只输出欧拉角的 HWT906 后没有数据源, 已删除。原实现见:
 *     obsolete/imu660/Nav_position_INS_reference.c
 *
 * 航向现在直接取 hwt_imu.h 的 g_hwt_imu_yaw_rad (见 Nav_position.c)。
 * ============================================================ */

#endif //STM32G4_TEST_NAV_POSITION_H