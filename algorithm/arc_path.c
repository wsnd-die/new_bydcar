/**
 * @file    arc_path.c
 * @brief   定半径圆弧 —— 车头恒为圆弧切线（纯几何开环）。设计说明见 arc_path.h。
 */
#include "Common_used.h"
#include "arc_path.h"
#include "mecanum.h"
#include "Send_motor.h"
#include "worker_task.h"    /* g_angle_ctrl_enable：借走电机独占权 */

#define ARC_RAD2DEG(r)  ((r) * 57.2957795131f)

static float s_radius_m  = ARC_DEF_RADIUS;
static float s_v_mps     = ARC_DEF_SPEED;
static float s_sweep_deg = ARC_DEF_SWEEP;

static void ARC_Stop(void)
{
    MecanumResult zero = Mecanum_Calc(0.0f, 0.0f);
    Send_commandmotor(&zero);
}

void Arc_SetParam(float radius_m, float v_mps, float sweep_deg)
{
    s_radius_m  = radius_m;
    s_v_mps     = v_mps;
    s_sweep_deg = sweep_deg;
}

void Arc_Abort(void)
{
    g_angle_ctrl_enable = 0;    /* 让 FC 角度环停手，避免两个指令源打架 */
    ARC_Stop();
}

bool Arc_Run(void)
{
    const float radius = s_radius_m;
    const float v      = s_v_mps;
    const float sweep  = fabsf(s_sweep_deg);

    float    w_rad, w_deg, sweep_time_s;
    uint32_t t0, timeout_ms;

    /* 参数校验：宁可不动，不可乱动 */
    if (radius == 0.0f || v == 0.0f || sweep == 0.0f) {
        return false;
    }

    /* 符号由 radius 决定：R>0 → w>0 → 左弧 */
    w_rad = v / radius;

    if (fabsf(w_rad) > ARC_W_MAX) {
        return false;               /* (v, R) 要求的角速度超出底盘量程 */
    }

    w_deg        = ARC_RAD2DEG(w_rad);
    sweep_time_s = sweep / fabsf(w_deg);
    timeout_ms   = (uint32_t)(sweep_time_s * 1500.0f) + 1000u;  /* 50% 余量 + 1s 兜底 */

    g_angle_ctrl_enable = 0;        /* 借走电机：本段由本模块独占指令源 */

    MecanumResult cmd = Mecanum_Calc(v, w_rad);

    t0 = HAL_GetTick();
    for (;;)
    {
        Send_commandmotor(&cmd);    /* 每拍重发，防单帧丢失 */

        uint32_t elapsed = HAL_GetTick() - t0;

        if ((float)elapsed * 0.001f >= sweep_time_s) {
            break;                  /* 扫够圆心角，正常收尾 */
        }
        if (elapsed > timeout_ms) {
            break;                  /* 兜底超时 */
        }

        osDelay(FC_TASK_PERIOD_MS);
    }

    ARC_Stop();
    return true;
}
