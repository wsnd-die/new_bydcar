/**
 * @file    angle_ctrl.h
 * @brief   航向串级控制 (角度环 → 角速度环)。步进在 Nav_GoToWorld() 的循环里。
 *
 * 数据源: `locator_ops9` 的 `pose.yaw` / `pose.wz` —— 与导航同源, 避免两套零点。
 * 单位:   一律弧度制, 与 `pose.yaw` / `Mecanum_Calc_Full_V()` 的 w 一致。
 *
 * 结构:
 *     外环  ω_cmd = PID(θ_target − θ_meas)        只 P, 带死区, 输出限幅
 *     内环  w     = PID(ω_cmd    − ω_meas)       P + I
 *
 * `θ_target` 由 `Angle_Enable()` 写成 `yaw_now + wrap(target − yaw_now)`
 * (走近路 + 连续角) ⇒ 起步无跳变, 且与 OPS9 的 yaw 零点是否标定无关。
 *
 * 步进: 由 `Nav_GoToWorld()` 每拍调一次 `Angle_Step()`, 与下发同拍。
 *       **顺序必须是「先 Angle_Enable() 再 Angle_Step()」**。
 *       航向**是否到位只由 `Nav_GoToWorld()` 判**, 本模块不自判。
 *
 * @note V1.33.2 删掉了 S 型轨迹发生器 —— θ_target 直接进外环。
 * @note 本模块**不再依赖 `imu_hwt906`**。
 */

#ifndef ANGLE_CTRL_H
#define ANGLE_CTRL_H

#include <stdint.h>
#include <stdbool.h>

/* ---- 外环: 角度 → 角速度指令 ---- */
#define ANG_KP_TH          8.0f     /* 外环 P (1/s) */
#define ANG_W_CMD_MAX      7.5f      /* 外环输出限幅 (rad/s) */
#define ANG_YAW_DEADBAND   0.0175f   /* 外环角度死区 (rad ≈ 1°) */

/* ---- 内环: 角速度 → 电机指令 ---- */
#define ANG_W_DEADBAND     0.5f      /* wz 死区 (rad/s)。★ 0.5 rad/s ≈ 28.6°/s, 偏大 */
#define ANG_KP_W           0.6f      /* 内环 P */
#define ANG_KI_W           0.00006f

#define ANG_IW_MAX         0.5f      /* 内环积分限幅 */
#define ANG_W_OUT_MAX      3.5f      /* 内环输出限幅 (rad/s) */

/* ============================================================
 * 对外接口
 * ============================================================ */

/** 上电调一次: 清状态、失能 */
void Angle_Init(void);

/**
 * @brief 武装并使能 —— 任务上下文调用, 且此时 OPS9 位姿有效
 * @param yaw_target 目标航向 (rad)
 * @note  内部自读 `locator_ops9.get_pose()` 取当前位姿作起点, 目标写成
 *        `yaw_now + wrap(target − yaw_now)` (走近路) ⇒ 起步无跳变。
 * @warning **每次调用都会清 PID 历史** —— 想换目标又不想清, 用 `Angle_Retarget()`。
 */
void Angle_Enable(float yaw_target);

/** 换目标但**不重置** PID —— 用于连续追踪动目标 (如圆弧切向) */
void Angle_Retarget(float yaw_target);

/** 失能: 停止计算, cmd_w 归零, 清积分。任务上下文调用 */
void Angle_Disable(void);

/** 取最新角速度指令 (rad/s, CCW+)。未使能时返回 0 —— 安全默认 */
float Angle_GetW(void);

/** 单拍步进 —— **由 `Nav_GoToWorld()` 的循环每拍调用一次** (V1.33.0 起)。
 *  @note  调用顺序必须是「先 `Angle_Enable()` 再本函数」。未使能时空转。
 *  @warning 保持纯计算: 不用 FreeRTOS API / printf / malloc / cosf / sinf,
 *           只用 fabsf —— 便于将来再挪回中断。 */
void Angle_Step(void);

/* ---- 遥测 (仅供调试打印) ---- */
typedef struct {
    float    th_ref;    /* 目标航向 (rad) */
    float    yaw_meas;  /* 最近一拍实测航向 (rad) */
    float    w_meas;    /* 最近一拍实测角速度 (rad/s, 已过死区) */
    float    cmd_w;     /* 最终输出 (rad/s) */
    float    e_th;      /* 最近一拍角度误差 (rad, 已过死区) */
    uint32_t tick;      /* 心跳: 每拍 ++ */
} AngleTelem_t;

void Angle_GetTelem(AngleTelem_t *out);

#endif /* ANGLE_CTRL_H */
