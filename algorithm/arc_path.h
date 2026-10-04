/**
 * @file    arc_path.h
 * @brief   定半径圆弧 —— 车头恒为圆弧切线（纯几何开环）
 *
 *  几何：θ(t) = θ₀ + s/R，v = ds/dt，ω = v/R —— 角速度与线速度锁定成比例。
 *  只要给底盘 (v, ω = v/R) 且车头已对准 θ₀，车自然画出半径 R 的弧。
 *  麦轮的 Mecanum_Calc(v, w) 正好就是这个模型。
 *
 *  **开环**：本模块不读 IMU、不修航向，只按几何量持续下发指令。初始对准误差、
 *  打滑与地面摩擦都会让航向误差累积且永不修正，实际轨迹会偏 —— 需要守住切线时
 *  得在外层另加航向反馈。
 *
 *  @warning **同一时刻全工程只能有一个电机指令源。** 本模块自己调
 *           `Send_commandmotor()`，入口会关掉 FC 角度环（`g_angle_ctrl_enable = 0`）
 *           借走电机独占权。
 *  @warning **符号是生死线。** R > 0 表示左弧（逆时针，yaw 增大）。若 `Mecanum_Calc`
 *           的 w 方向反了，车会朝反方向画弧。首次使用前必须把车**架空**验证符号。
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
 * @brief  设置圆弧参数（下次 Arc_Run 生效）。
 * @param  radius_m   半径 m。>0 = 左弧(逆时针)，<0 = 右弧。
 * @param  v_mps      线速度 m/s。>0 = 沿车头方向前进。
 * @param  sweep_deg  扫过的圆心角 deg，取绝对值。360 = 整圆。
 * @note   任一参数为 0，或 |v/R| > ARC_W_MAX，Arc_Run() 直接返回 false 不驱动。
 */
void Arc_SetParam(float radius_m, float v_mps, float sweep_deg);

/**
 * @brief  阻塞式跑完一段圆弧。
 * @return true = 正常跑完 swept 角度（**含被中途打断的情况**）；false = 参数非法，未驱动。
 * @note   与 Nav_MoveBody() 同风格。返回前一定已发出零速。
 *         本函数自己下发 `Send_commandmotor()`，不经 FC_TASK。
 * @note   V1.23.0 起可被 `g_route_abort`（见 app/worker_task.h）**中途打断**：
 *         读到就立刻收尾、发出零速、按正常成功返回。打断**不**走 false —— 那个
 *         false 是留给"参数非法"的。
 */
bool Arc_Run(void);

/**
 * @brief  中止圆弧：关掉 FC 角度环并立即发零速。供急停用。
 */
void Arc_Abort(void);

#endif /* ARC_PATH_H */
