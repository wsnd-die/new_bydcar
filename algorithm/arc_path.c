/**
 * @file    arc_path.c
 * @brief   定半径圆弧 —— 车头恒为圆弧切线。原理与设计分工见 arc_path.h。
 */
#include "Common_used.h"
#include "arc_path.h"
#include "worker_task.h"    /* g_angle_ctrl_* 契约量 (唯一定义处: worker_task.c) */
#include "HWT906.h"         /* imu_hwt906 实例: is_healthy() 健康检查 */
#include "hwt_imu.h"        /* g_hwt_imu_yaw: 起始切向角 */

/* 本文件私有的弧度换算宏。刻意不放进 arc_path.h —— 公共头里放通用宏容易被
 * 其它模块连带引入，而 worker_task.c 已自带一个同用途的 RAD2DEG。加 ARC_
 * 前缀是为了在 .c 里一眼看出归属。 */
#define ARC_RAD2DEG(r)  ((r) * 57.2957795131f)

/* ================================================================
 * 参数与状态
 * ================================================================ */

static float s_radius_m  = ARC_DEF_RADIUS;
static float s_v_mps     = ARC_DEF_SPEED;
static float s_sweep_deg = ARC_DEF_SWEEP;

/* ================================================================
 * 外部接口
 * ================================================================ */

void Arc_SetParam(float radius_m, float v_mps, float sweep_deg)
{
    s_radius_m  = radius_m;
    s_v_mps     = v_mps;
    s_sweep_deg = sweep_deg;
}

void Arc_Abort(void)
{
    /* 顺序有意义: 先关使能, 让 FC_TASK 走下降沿去主动刹停, 再清几何量。
     * 反过来的话, FC_TASK 可能刚好在两拍之间读到 "使能还在但速度已是 0"
     * 的中间态 —— 虽然结果也是停, 但语义不干净。 */
    g_angle_ctrl_enable = 0;
    g_angle_ctrl_speed  = 0.0f;
    g_angle_ctrl_w_ff   = 0.0f;
}

bool Arc_Run(void)
{
    const float radius = s_radius_m;
    const float v      = s_v_mps;
    const float sweep  = fabsf(s_sweep_deg);

    float    w_rad, w_deg, theta0;
    uint32_t t0, timeout_ms;

    /* ---- 1. 参数校验: 宁可不动, 不可乱动 ---- */
    if (radius == 0.0f || v == 0.0f || sweep == 0.0f) {
        return false;
    }

    /* 符号由 radius 决定: R>0 → w>0 → 逆时针 → 左弧 */
    w_rad = v / radius;

    if (fabsf(w_rad) > ARC_W_MAX) {
        /* (v, R) 组合要求底盘转得比它转得动的还快, 硬跑只会走出乱轨迹 */
        return false;
    }

    w_deg = ARC_RAD2DEG(w_rad);

    /* 扫完 sweep 所需时间, 留 50% 余量再兜一个 1s。
     * 兜底的意义同 Nav_MoveBody 的 guard: FC_TASK 异常/IMU 掉线时不会死等。 */
    timeout_ms = (uint32_t)((sweep / fabsf(w_deg)) * 1500.0f) + 1000u;

    /* ---- 2. 等 IMU 有效。航向不可信时绝不开环 ---- */
    {
        uint32_t wait0 = HAL_GetTick();
        while (!imu_hwt906.is_healthy()) {
            if ((HAL_GetTick() - wait0) > ARC_IMU_WAIT_MS) {
                return false;   /* 模块离线或 FC_TASK 没跑, 未驱动电机 */
            }
            osDelay(FC_TASK_PERIOD_MS);
        }
    }

    /* ---- 3. 起始切向角: 车头当前朝向就是圆弧的起点切线方向 ---- */
    theta0 = g_hwt_imu_yaw;             /* deg */

    /* ---- 4. 打开角度环并挂上前馈 ----
     * 使能置 1 的上升沿会让 FC_TASK 调 Angle_SetTarget() 复位 PID,
     * 清掉上一次残留的积分。 */
    g_angle_target_yaw  = theta0;
    g_angle_ctrl_speed  = v;
    g_angle_ctrl_w_ff   = w_rad;
    g_angle_ctrl_enable = 1;

    /* ---- 5. 跑弧 ---- */
    t0 = HAL_GetTick();
    for (;;)
    {
        uint32_t elapsed = HAL_GetTick() - t0;
        float    t       = (float)elapsed * 0.001f;

        /* ★★ 动目标 —— 本模块最容易写错的一处 ★★
         *
         * 切向角必须按 w_deg 每拍持续推进。若只在上面设一次 theta0 就不再
         * 更新, PID 会把车的**正常圆弧运动**当成航向误差去反向纠正,
         * 前馈与反馈互相打架, 车要么不转要么乱转。
         *
         * 用 HAL_GetTick 反算 t 而不是自增计数 —— osDelay 的实际周期会因
         * FC_TASK/Send_commandmotor 的 osDelay(5) 而抖动, 累加会漂。 */
        g_angle_target_yaw = theta0 + w_deg * t;

        if (fabsf(w_deg * t) >= sweep) {
            break;                      /* 扫够圆心角, 正常收尾 */
        }
        if (elapsed > timeout_ms) {
            break;                      /* 兜底超时 */
        }

        osDelay(FC_TASK_PERIOD_MS);     /* 与 FC_TASK 同周期, 每拍推一次目标 */
    }

    /* ---- 6. 收尾 ---- */
    Arc_Abort();                        /* 关角度环, 清几何量 */
    osDelay(20);                        /* 契约要求: 等 FC_STOP() 把零速发出去 */

    return true;
}
