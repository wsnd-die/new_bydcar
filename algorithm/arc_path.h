/**
 * @file    arc_path.h
 * @brief   定半径圆弧 —— 车头恒为圆弧切线
 *
 *  几何：
 *      θ(t) = θ₀ + s/R          切向角 = 航向 (车头方向)
 *      v    = ds/dt             线速度
 *      ω    = dθ/dt = v/R       ← 角速度与线速度锁定成比例
 *
 *  即：只要同时给底盘 (v, ω = v/R) 且车头已对准 θ₀，车自然画出半径 R 的弧。
 *  麦轮的 Mecanum_Calc(v, w) 正好就是这个模型，无需任何新的运动学。
 *
 *  ------------------------------------------------------------------
 *  实现分工（这是本模块的核心设计）：
 *
 *      几何前馈  w_ff = v/R      →  负责「走得出弧」
 *      角度环 PID                →  负责「守住切线」，只修残差
 *
 *  缺一不可：
 *    · 只有前馈 —— 初始对准误差、打滑、地面摩擦都会让航向误差**累积且永不
 *      修正**，实际轨迹会螺旋发散；
 *    · 只有 PID —— 它必须靠一个稳态误差才能维持持续转动，且积分要从零爬起来。
 *
 *  前馈经 `g_angle_ctrl_w_ff` 下发（见 app/worker_task.h 的契约说明），
 *  它绕过 angle_ctrl 的两级 PID，直接叠加在内环输出上。
 *
 *  ------------------------------------------------------------------
 *  @warning **同一时刻全工程只能有一个电机指令源。**
 *           Send_commandmotor() 还有 8 处调用者（Circle_base / BollLocator /
 *           NavigationMecanum / mecanum.c）。本模块运行期间它们都不能动，
 *           否则会互相覆盖指令。沿用 NavigationMecanum.c「避免与后面平移
 *           抢电机」的既有约定。
 *
 *  @warning **符号是生死线。** R > 0 表示左弧（逆时针，yaw 增大）。
 *           若 `Mecanum_Calc` 的 w 方向或 `HWT906_GZ_SIGN` 有一处反了，
 *           闭环就是正反馈、会直接发散。首次使用前必须把车**架空**单独验证
 *           符号，判据见 clauderecord/2026-09-25.md。
 */
#ifndef ARC_PATH_H
#define ARC_PATH_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * 参数
 * ================================================================ */

/**
 * 角速度上限，单位 rad/s。
 * 与 `algorithm/angle_ctrl.c` 的 `CFG_MAX_W` 一致 —— 超出这个量程的
 * (v, R) 组合几何上无法实现，`Arc_Run()` 直接拒绝而不是硬跑。
 */
#define ARC_W_MAX           3.0f

#define ARC_DEF_RADIUS      0.5f    /* m，正值 = 左弧(CCW) */
#define ARC_DEF_SPEED       0.15f   /* m/s */
#define ARC_DEF_SWEEP       360.0f  /* deg，扫过的圆心角；360 = 整圆 */

/**
 * 等 IMU 变有效的最长时间，单位 ms。
 * FC_TASK 每 10ms 刷新一次 HWT906，正常一两拍就有效；超过这个时间说明
 * 模块离线或 FC_TASK 没跑 —— 此时航向不可信，**绝不能盲开**。
 */
#define ARC_IMU_WAIT_MS     200u

/* ================================================================
 * API
 * ================================================================ */

/**
 * @brief  设置圆弧参数（下次 Arc_Run 生效）。
 * @param  radius_m   半径，单位 m。**>0 = 左弧(逆时针)，<0 = 右弧**。
 * @param  v_mps      线速度，单位 m/s。**>0 = 沿车头方向前进**。
 * @param  sweep_deg  扫过的圆心角，单位 deg，取绝对值。
 *                    360 = 走一整圈闭合成圆。
 * @note   三个参数任一为 0，或 |v/R| > ARC_W_MAX，`Arc_Run()` 会直接返回
 *         false 不驱动电机。
 */
void Arc_SetParam(float radius_m, float v_mps, float sweep_deg);

/**
 * @brief  阻塞式跑完一段圆弧。
 * @return true = 正常跑完 swept 角度；false = 参数非法或 IMU 不可用，未驱动。
 * @note   **阻塞式**，与 `Nav_MoveBody()` 同风格（NLF_TASK 本就是阻塞式流程
 *         任务）。返回前一定已经关掉角度环并让 FC_TASK 发出零速。
 * @note   调用链：本函数经 `g_angle_*` 契约量驱动 **FC_TASK** 产出电机指令，
 *         自己**不直接调** Send_commandmotor() —— 电机指令源唯一。
 */
bool Arc_Run(void);

/**
 * @brief  中止圆弧：关角度环并清掉线速度与前馈。
 * @note   供急停用（NLF_RunFlow 的 Event_STOP）。`Arc_Run()` 内部收尾也用它。
 *         **不保证立即停轮** —— FC_TASK 在下降沿调 FC_Stop() 发零速，
 *         调用方需按契约 `osDelay(20)` 等它发出去。
 */
void Arc_Abort(void);

#endif /* ARC_PATH_H */
