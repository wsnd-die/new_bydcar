/**
 * @file    angle_ctrl.c
 * @brief   纯角度控制: 角度环 → 角速度环 (串级 PID)
 *          仅维持目标角度，不做位置导航
 */
#include "Common_used.h"

#define CLAMP(v,lo,hi)  ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define DT              0.01f

/* 默认参数 */
#define CFG_MAX_W       3.0f
#define CFG_MAX_W_DEG   180.0f
#define CFG_YAW_TOL     1.0f   /* 修严: 角度环实际修到 ~1° 再判到位, 否则导航等 4°/2° 会干等满 guard */

static float norm_deg(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* ================================================================
 *  角度串级: 角度环 → 角速度环
 *   输入: 当前 yaw + 角速度
 *   输出: ac->cmd_w, ac->target_w, ac->yaw_err
 * ================================================================ */
void AngleLoop_Update(AngleCtrl *ac,
                    float cur_yaw, float cur_w)
{
    /* ---- 陀螺仪滤波 ---- */
    ac->gyro_filt = ac->gyro_alpha * cur_w
                   + (1.0f - ac->gyro_alpha) * ac->gyro_filt;

    /* 死区: 小于阈值当 0 */
    float w_filt = ac->gyro_filt;
    if (fabsf(w_filt) < ac->gyro_deadband) w_filt = 0.0f;

    /* 缩放 */
    w_filt *= ac->gyro_scale;

    /* ---- 角度环 ---- */
    ac->yaw_err = norm_deg(ac->target_yaw - cur_yaw);

    /* 中层: yaw → target_w (deg/s) */
    ac->target_w = PID_calc(&ac->pid_angle, 0.0f, ac->yaw_err);
    ac->target_w = CLAMP(ac->target_w, -CFG_MAX_W_DEG, CFG_MAX_W_DEG);

    /* 内层: w → cmd_w (rad/s), 用滤波后的角速度做反馈 */
    ac->cmd_w = PID_calc(&ac->pid_w, w_filt, ac->target_w);
    ac->cmd_w = CLAMP(ac->cmd_w, -ac->max_w, ac->max_w);
}

/* ================================================================
 *  整体
 * ================================================================ */

void Angle_Init(AngleCtrl *ac)
{
    memset(ac, 0, sizeof(*ac));
    ac->state  = ANGLE_IDLE;
    ac->max_w  = CFG_MAX_W;
    ac->yaw_tol = CFG_YAW_TOL;

    /* 角度环 */
    const fp32 ak[3] = { 2.0f, 0.0f, 0.001f };
    PID_init(&ac->pid_angle, PID_POSITION, ak, CFG_MAX_W_DEG, 30.0f);

    /* 角速度环 */
    const fp32 wk[3] = { 0.018f, 0.01f, 0.001f };
    PID_init(&ac->pid_w, PID_POSITION, wk, CFG_MAX_W, 0.4f);

    /* 陀螺仪滤波 */
    ac->gyro_alpha    = 0.14f;
    ac->gyro_deadband = 0.15f;
    ac->gyro_scale    = 0.05f;
    ac->gyro_filt     = 0.0f;
}

void Angle_SetTarget(AngleCtrl *ac, float yaw)
{
    ac->target_yaw = yaw;
    ac->state      = ANGLE_MOVING;
    PID_clear(&ac->pid_angle);
    PID_clear(&ac->pid_w);
}

void Angle_UpdateTarget(AngleCtrl *ac, float yaw)
{
    ac->target_yaw = yaw;
    ac->state      = ANGLE_MOVING;
    /* 不重置 PID，适合连续追踪 */
}

void Angle_Update(AngleCtrl *ac,
                float cur_yaw, float cur_w)
{
    if (ac->state != ANGLE_MOVING) {
        ac->cmd_w = 0.0f;
        return;
    }

    /* ---- 角度串级 ---- */
    AngleLoop_Update(ac, cur_yaw, cur_w);

    /* ---- 到达判断 ---- */
    if (fabsf(ac->yaw_err) < ac->yaw_tol) {
        ac->state = ANGLE_ARRIVED;
        ac->cmd_w = 0.0f;
    }
}

bool Angle_Arrived(const AngleCtrl *ac)
{
    return (ac->state == ANGLE_ARRIVED);
}
