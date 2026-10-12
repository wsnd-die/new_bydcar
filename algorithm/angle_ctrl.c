/**
 * @file    angle_ctrl.c
 * @brief   航向串级控制实现。结构与参数说明见 angle_ctrl.h。
 *
 * @note V1.33.2 删掉了 S 型轨迹发生器 —— θ_target 直接进外环, 不再过参考模型。
 */

#include "Common_used.h"
#include "angle_ctrl.h"
#include "pid.h"                  /* pid_type_def / PID_init / PID_calc / PID_clear */
#include "pose_data.h"            /* PoseData_t */
#include "ops9_g491_uart3.h"      /* extern const LocatorDev_t locator_ops9 */
#include <math.h>                 /* fabsf */

/* ---- 控制状态 ----
 * 步进跑在 Nav_GoToWorld() 的循环里, 单上下文、无并发。
 * 保留 volatile 只为调试器能直接看到。 */
static volatile float   s_target = 0.0f;   /* 目标航向 (rad, 已 unwrap 的连续角) */
static volatile uint8_t s_enable = 0u;     /* 1 = 步进计算并更新输出 */

static volatile float    s_cmd_w = 0.0f;   /* 输出角速度指令 (rad/s, CCW+) */
static volatile uint32_t s_tick  = 0u;     /* 心跳 (每拍自增) */

/* ---- 步进私有 ----
 * 两级 PID 用工程既有的 `algorithm/pid.c` (位置式; 状态在调用方结构体里, 可重入)。
 * 它**没有 dt 项**(按采样整定) ⇒ I 增益要按采样率折算, 见 `ANG_KI_W`。
 * 两级都不设 D —— 内环的反馈量本身就是外环误差的导数。 */
static pid_type_def s_pid_th;     /* 外环: 角度 → 角速度指令 (只 P) */
static pid_type_def s_pid_w;      /* 内环: 角速度 → 电机指令 (P + I) */

static volatile float s_w_filt;   /* pose.wz (过死区后) */
static volatile float s_yaw_meas; /* 最近一拍实测航向 (遥测) */
static volatile float s_e_th;     /* 最近一拍角度误差 (rad, 仅遥测) */

/* ---- wz 零偏 ----
 * ⚠ **未实现**: 实测 OPS9 静止时 heading_rate ≈ -0.0074 rad/s ≠ 0。不减掉它,
 * 内环 e_ω 带恒定偏置 ⇒ 车会慢慢自转。这两个变量目前**无人赋值也无人读取**。 */
static volatile float   s_wz_bias   = 0.0f;
static volatile uint8_t s_bias_done = 0u;

/* ==================================================================
 * 小工具
 * ================================================================== */

/** 角度归一化到 [-π, π] */
static float ANG_WrapPi(float a)
{
    const float PI  = 3.14159265358979f;
    const float TAU = 6.28318530717959f;
    while (a >  PI) { a -= TAU; }
    while (a < -PI) { a += TAU; }
    return a;
}

static float ANG_Clamp(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/** 读一次 OPS9 位姿; 无效时返回 false 且不写出参 */
static bool ANG_ReadPose(PoseData_t *p)
{
    locator_ops9.get_pose(p);
    return (p->valid != 0u);
}

/* ==================================================================
 * 对外接口
 * ================================================================== */

void Angle_Init(void)
{
    const fp32 k_th[3] = { ANG_KP_TH, 0.0f,     0.0f };
    const fp32 k_w [3] = { ANG_KP_W,  ANG_KI_W, 0.0f };
    PID_init(&s_pid_th, PID_POSITION, k_th, ANG_W_CMD_MAX, 0.0f);
    PID_init(&s_pid_w,  PID_POSITION, k_w,  ANG_W_OUT_MAX, ANG_IW_MAX);

    s_enable   = 0u;
    __DMB();
    s_target   = 0.0f;
    s_cmd_w    = 0.0f;
    s_w_filt   = 0.0f;
    s_yaw_meas = 0.0f;
    s_e_th     = 0.0f;
    s_tick     = 0u;
}

void Angle_Enable(float yaw_target)
{
    PoseData_t p;

    s_enable = 0u;              /* 先关门, 防读到半初始化状态 */
    __DMB();

    if (!ANG_ReadPose(&p)) {
        return;                 /* 读不到位姿就保持失能 —— 不动作比乱动好 */
    }

    PID_clear(&s_pid_th);       /* 跨段不能带残留 */
    PID_clear(&s_pid_w);

    s_w_filt = p.wz;            /* 用当前实测角速度播种, 免重使能时的顿挫 */

    /* 走近路 + 连续角: 结果与 OPS9 的 yaw 零点是否标定无关 */
    s_target = p.yaw + ANG_WrapPi(yaw_target - p.yaw);

    s_cmd_w = 0.0f;
    __DMB();
    s_enable = 1u;              /* 最后提交 */
}

void Angle_Retarget(float yaw_target)
{
    /* 不重置 PID/输出 —— 只在连续角上把目标挪到"走近路可到"的等价角。
     * 用于圆弧那种目标航向每拍都在变的场合。 */
    s_enable = 0u;
    __DMB();
    s_target = s_target + ANG_WrapPi(yaw_target - s_target);
    s_enable = 1u;
    __DMB();
}

void Angle_Disable(void)
{
    s_enable = 0u;
    __DMB();
    s_cmd_w  = 0.0f;
    s_w_filt = 0.0f;
    PID_clear(&s_pid_th);
    PID_clear(&s_pid_w);
}

float Angle_GetW(void)
{
    return s_cmd_w;             /* 未使能时已被清成 0 —— 安全默认 */
}

void Angle_GetTelem(AngleTelem_t *out)
{
    if (out == NULL) {
        return;
    }
    out->th_ref   = s_target;
    out->yaw_meas = s_yaw_meas;
    out->w_meas   = s_w_filt;
    out->cmd_w    = s_cmd_w;
    out->e_th     = s_e_th;
    out->tick     = s_tick;
}

/**
 * @brief 单拍步进 —— 由 `Nav_GoToWorld()` 的循环每拍调一次
 *
 * @note  顺序必须是「先 `Angle_Enable()` 再本函数」; 未使能时只自增心跳。
 * @note  保持纯计算(只用 fabsf), 便于将来挪回中断。
 */
void Angle_Step(void)
{
    PoseData_t p;
    float e_th, w_cmd;

    s_tick++;
    if (s_enable == 0u) {
        return;
    }


    if (!ANG_ReadPose(&p)) {
        return;                 /* 反馈无效: 保持上一拍输出, 不动作 */
    }
    s_yaw_meas = p.yaw;

    {
        float th_cmd = s_target;
        if (fabsf(ANG_WrapPi(s_target - s_yaw_meas)) < ANG_YAW_DEADBAND) {
            th_cmd = s_yaw_meas;
        }
        w_cmd = PID_calc(&s_pid_th, s_yaw_meas, th_cmd);   /* 内部 error = set − ref */
    }
    w_cmd = ANG_Clamp(w_cmd, -ANG_W_CMD_MAX, ANG_W_CMD_MAX);

    /* ---- 内环: 角速度 → 电机指令 ---- */
    if (fabsf(p.wz) < ANG_W_DEADBAND) {
        p.wz = 0.0f;
    }
    s_w_filt = p.wz;
    s_cmd_w  = PID_calc(&s_pid_w, s_w_filt, w_cmd);
    s_cmd_w  = ANG_Clamp(s_cmd_w, -ANG_W_OUT_MAX, ANG_W_OUT_MAX);

    /* 遥测(死区后的语义) */
    e_th = ANG_WrapPi(s_target - s_yaw_meas);
    if (fabsf(e_th) < ANG_YAW_DEADBAND) {
        e_th = 0.0f;
    }
    s_e_th = e_th;
}
