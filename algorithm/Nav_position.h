//
// Created by 35037 on 2026/8/12.
//
#ifndef STM32G4_TEST_NAV_POSITION_H
#define STM32G4_TEST_NAV_POSITION_H

#include "Common_used.h"

/* 当前里程计位姿） */
extern World_Dir_t World_position;

/**
 * @brief 增量式编码器里程计，周期性调用（每 10~20ms）
 * @return 当前世界位姿（同时更新全局 World_position）
 */
World_Dir_t World_position_get(void);

/**
 * @brief 清零编码器里程计并重新起算
 * @note  放完 5 物块 / 奖杯段分界时调用: 只清 World_position;
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