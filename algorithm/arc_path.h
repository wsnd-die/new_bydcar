/**
 * @file    arc_path.h
 * @brief   定半径圆弧 —— 车头恒为圆弧切线 (纯几何开环)
 *
 * 几何: ω = v/R。给底盘 (v, ω) 且车头已对准切线, 车自然画出半径 R 的弧。
 *
 * **开环**: 不读 IMU、不修航向, 初始对准误差/打滑都会累积, 轨迹会偏。
 *
 * @warning 全工程同一时刻只能有一个电机指令源 —— 本模块自己下发指令,
 *          入口会关掉 FC 角度环借走独占权。
 * @warning 符号是生死线: R > 0 = 左弧 (CCW)。首次使用前必须架空验证方向。
 */
#ifndef ARC_PATH_H
#define ARC_PATH_H

#include <stdint.h>
#include <stdbool.h>

/* 角速度上限 rad/s，与 angle_ctrl.c 的 CFG_MAX_W 一致。
 * 超出这个量程的 (v, R) 组合几何上无法实现，Arc_Run() 直接拒绝。 */
#define ARC_W_MAX           3.0f

#define ARC_DEF_RADIUS      0.5f    /* m，正值 = 左弧(CCW) */
#define ARC_DEF_SPEED       0.15f   /* m/s */
#define ARC_DEF_SWEEP       360.0f  /* deg，扫过的圆心角；360 = 整圆 */

/**
 * @brief  设置圆弧参数 (下次 Arc_Run 生效)。
 * @note   任一参数为 0, 或 |v/R| > ARC_W_MAX, Arc_Run() 直接返回 false 不驱动。
 */
void Arc_SetParam(float radius_m, float v_mps, float sweep_deg);

/**
 * @brief  阻塞式跑完一段圆弧。
 * @return true = 正常跑完 (含被 g_route_abort 打断); false = 参数非法, 未驱动。
 * @note   自己下发指令, 不经 FC_TASK; 返回前一定已发零速。
 */
bool Arc_Run(void);

/** @brief 中止圆弧: 关掉 FC 角度环并立即发零速。供急停用。 */
void Arc_Abort(void);

#endif /* ARC_PATH_H */
