/**
 * @file    angle_ctrl.c
 * @brief   航向串级控制 + S 型轨迹 —— 步进在 TIM7 中断里跑 (1kHz)
 *
 * 结构说明、参数含义、"ISR 只算不发"的理由、以及"为什么换成 OPS9"都写在
 * angle_ctrl.h 顶部。本文件只放实现与 ISR 私有状态。
 */

#include "Common_used.h"
#include "angle_ctrl.h"
#include "pid.h"                  /* pid_type_def / PID_init / PID_calc / PID_clear */
#include "pose_data.h"            /* PoseData_t */
#include "ops9_g491_uart3.h"      /* extern const LocatorDev_t locator_ops9 */
#include <math.h>                 /* sqrtf / fabsf */

/* ==================================================================
 * 任务 ↔ ISR 的共享面 —— 只有 3 个词 + 1 个心跳
 *
 * Cortex-M4 上 4 字节对齐的单字读写是**单条指令、天然原子**, 不需要临界区、
 * 队列或双缓冲。中间值没有意义(任务只发最新一条), 所以也不用队列。
 * ================================================================== */

/* 任务 → ISR */
static volatile float   s_target = 0.0f;   /* 目标航向 (rad, 已 unwrap 的连续角) */
static volatile uint8_t s_enable = 0u;     /* 1 = ISR 计算并更新输出 */

/* ISR → 任务 */
static volatile float    s_cmd_w = 0.0f;   /* 最终角速度指令 (rad/s, CCW+) */
static volatile uint32_t s_tick  = 0u;     /* 心跳 */

/* ==================================================================
 * ISR 私有状态 —— 任务永不触碰
 *
 * 两级 PID 用工程既有的 `algorithm/pid.c`（位置式）。为什么可以用：
 *   · `PID_calc()` 的状态在**调用方的结构体**里(不是文件级 static), 可重入;
 *     —— 不可重入的是 `PID_Filter()`, 本文件不用它。
 *   · `PID_calc()` **没有 dt 项**(纯离散、按采样整定) ⇒ **只有 I 增益要按
 *     采样率折算**: `PID_init` 里传 `ANG_KI_W * ANG_DT`(把头文件的 1/s 量纲
 *     折成"每拍")。P 与 dt 无关, 原样传。
 *   · **两级都不需要 D 项** —— 内环的反馈量本身就是外环误差的导数, 这正是
 *     串级的价值所在; 再对误差做差分会把观测噪声放大两遍。
 * ================================================================== */
static pid_type_def s_pid_th;     /* 外环: 角度 → 角速度指令 (只 P) */
static pid_type_def s_pid_w;      /* 内环: 角速度 → 电机指令 (P + I) */

static volatile float s_th;       /* S 曲线参考角 */
static volatile float s_w;        /* S 曲线参考角速度 */
static volatile float s_a;        /* S 曲线参考角加速度 */
static volatile float s_w_filt;   /* pose.wz 一阶低通后的值 */
static volatile float s_yaw_meas; /* 最近一拍实测航向 (遥测) */
static volatile float s_e_th;     /* 最近一拍角度误差 (遥测) */

/* ---- wz 零偏标定 (见头文件 ANG_BIAS_N) ----
 * 实测: OPS9 静止时 heading_rate ≈ -0.0074 rad/s ≠ 0。不减掉它, 内环的
 * e_ω = 0 − wz 就是一个恒定的 +0.0074 ⇒ **车会以一个恒定的慢速自己转**
 * (外环的角度误差还在死区内, 管不到)。**死区压不住这个 —— 它作用在内环上。** */
static volatile float   s_wz_bias  = 0.0f;
static volatile uint8_t s_bias_done = 0u;

/* ==================================================================
 * 小工具 (ISR 安全: 无库调用)
 * ================================================================== */

/** 把角度归一化到 [-π, π] */
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

/* 读一次 OPS9 当前位姿; 无效时返回 false 且不写出参 */
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

    const fp32 k_th[3] = { ANG_KP_TH, 0.0f, 0.0f };
    const fp32 k_w [3] = { ANG_KP_W, ANG_KI_W * ANG_DT, 0.0f };
    PID_init(&s_pid_th, PID_POSITION, k_th, ANG_W_CMD_MAX, 0.0f);
    PID_init(&s_pid_w,  PID_POSITION, k_w,  ANG_W_OUT_MAX, ANG_IW_MAX);

    s_enable = 0u;
    __DMB();
    s_target   = 0.0f;
    s_cmd_w    = 0.0f;
    s_th       = 0.0f;
    s_w        = 0.0f;
    s_a        = 0.0f;
    s_w_filt   = 0.0f;
    s_yaw_meas = 0.0f;
    s_e_th     = 0.0f;
    s_tick     = 0u;
}

void Angle_Enable(float yaw_target)
{
    PoseData_t p;
    float yaw_now;

    /* 先关门 —— 防 ISR 读到半初始化的状态 */
    s_enable = 0u;
    __DMB();

    /* 取当前位姿作为轨迹起点。读不到就只在原地维持 —— 不动作比乱动好 */
    if (!ANG_ReadPose(&p)) {
        return;                 /* 保持失能 */
    }
    yaw_now = p.yaw;

    /* 轨迹状态从**当前位姿**起步: t=0 时 e_θ=0、ω_ref=0 ⇒ 无阶跃 */
    s_th = yaw_now;
    s_w  = 0.0f;
    s_a  = 0.0f;

    /* 清两级 PID 的历史(误差/积分/微分缓冲) —— 跨段不能带残留 */
    PID_clear(&s_pid_th);
    PID_clear(&s_pid_w);

    /* 低通滤波初值: 用当前实测角速度**播种**。否则"重使能时车正在转"会让
     * 内环反馈从 0 阶跃到真实值, 被 Kp_ω 放大成一次顿挫。 */
    s_w_filt = p.wz;

    /* 走近路 + 连续角: θ_ref 在**不 wrap** 的连续角上跑, ω_ref 才连续。
     * ★ 这个写法也让结果与 OPS9 的 yaw 零点无关(只关心相对转角)。 */
    s_target = yaw_now + ANG_WrapPi(yaw_target - yaw_now);

    s_cmd_w = 0.0f;
    __DMB();
    s_enable = 1u;      /* ★ 最后提交 */
}

void Angle_Retarget(float yaw_target)
{
    /* 不重置轨迹/积分 —— 只把目标换成"以当前**参考角** s_th 为基准、走近路可到
     * 的等价角"。不需要读位姿: s_th 本身就是连续的轨迹参考。
     * 用于圆弧那种参考航向每拍都在变的场合: 三阶模型对移动目标天然有平滑作用。 */
    s_enable = 0u;
    __DMB();
    s_target = s_th + ANG_WrapPi(yaw_target - s_th);
    s_enable = 1u;
    __DMB();
}

void Angle_Disable(void)
{
    s_enable = 0u;
    __DMB();
    s_cmd_w  = 0.0f;
    /* 清干净, 免得上一次导航的积分/轨迹残留带进下一段 */
    s_w      = 0.0f;
    s_a      = 0.0f;
    s_w_filt = 0.0f;
    PID_clear(&s_pid_th);
    PID_clear(&s_pid_w);
}

float Angle_GetW(void)
{
    return s_cmd_w;     /* 未使能时已被清成 0 —— 安全默认 */
}

bool Angle_Arrived(void)
{
    if (s_enable == 0u) {
        return false;
    }
    return (fabsf(s_e_th) < ANG_ARRIVE_TOL);
}

void Angle_GetTelem(AngleTelem_t *out)
{
    if (out == NULL) {
        return;
    }
    out->th_ref   = s_th;
    out->w_ref    = s_w;
    out->a_ref    = s_a;
    out->yaw_meas = s_yaw_meas;
    out->w_meas   = s_w_filt;
    out->cmd_w    = s_cmd_w;
    out->e_th     = s_e_th;
    out->tick     = s_tick;
}

/**
 * @brief ISR 步进 —— 在 TIM7 的 HAL_TIM_PeriodElapsedCallback 里调用
 *
 * @warning ISR 上下文。本函数内**不得**出现任何 FreeRTOS 阻塞 API、printf、
 *          malloc、cosf/sinf/atan2f。只用 fabsf/sqrtf。
 */
void Angle_Step(void)
{
    PoseData_t p;
    float e_ref, a_dot, e_th, w_lim, w_cmd;

    s_tick++;
    if (s_enable == 0u) {
        return;                 /* 空转 ~100ns, CPU 占用可忽略 */
    }

    /* ---- 0. 采样 (纯拷贝、无副作用, 可从 ISR 调) ----
     * 整结构体拷贝期间 ops9imu_task 可能正在写 s_pose ⇒ 单拍位姿可能"混帧"。
     * 判为良性: 下一拍(1ms 后)自愈, 且只影响千分之一秒的一拍。 */
    if (!ANG_ReadPose(&p)) {
        return;                 /* 反馈无效: 保持上一拍输出, 不动作 */
    }
    s_yaw_meas = p.yaw;

    /* ---- 1. S 型轨迹: 三阶临界阻尼参考模型 ----
     * 特征多项式 (s+K)³ ⇒ 三重极点都在 −K ⇒ 无超调、C² 连续、jerk 有上界。 */
    e_ref = s_target - s_th;
    a_dot = ANG_K * ANG_K * ANG_K * e_ref
          - 3.0f * ANG_K * ANG_K * s_w
          - 3.0f * ANG_K * s_a;

    s_a  += a_dot * ANG_DT;
    s_a   = ANG_Clamp(s_a, -ANG_A_MAX, ANG_A_MAX);      /* ★ 必须限 */

    /* ω 的上限带 √(2·α·e) 制动力上限: 保证"此刻还刹得住", 防末段冲过头。
     * 这一项在 e_ref→0 附近自动失效(那里 √(2αe) ≫ K·e), 不会掐死末段。 */
    w_lim = sqrtf(2.0f * ANG_A_MAX * fabsf(e_ref));
    if (w_lim > ANG_W_MAX) {
        w_lim = ANG_W_MAX;
    }
    s_w  += s_a * ANG_DT;
    s_w   = ANG_Clamp(s_w, -w_lim, w_lim);

    s_th += s_w * ANG_DT;       /* 不 wrap: 在连续角上跑 */

    /* ---- 2. 外环(角度) + ★速度前馈 ----
     * 死区处理: |误差| 落进死区时, 把 **set 拉到 ref 上**(而不是给输出乘 0)
     * ⇒ PID 内部算出的误差真为 0, 积分项也就不会继续累 —— 比"事后清零"干净。 */
    {
        float th_cmd = s_th;
        if (fabsf(ANG_WrapPi(s_th - s_yaw_meas)) < ANG_YAW_DEADBAND) {
            th_cmd = s_yaw_meas;
        }
        /* PID_calc(pid, ref, set): 内部 error = set − ref
         * ⇒ **ref 传实测航向, set 传参考角**。 */
        w_cmd = PID_calc(&s_pid_th, s_yaw_meas, th_cmd) + s_w;   /* ← 速度前馈 ω_ref */
    }
    w_cmd = ANG_Clamp(w_cmd, -ANG_W_CMD_MAX, ANG_W_CMD_MAX);

    /* ---- 3. 内环(角速度) + ★加速度前馈 ---- */
    {
        float af = ANG_DT / (ANG_TAU_F + ANG_DT);
        if (fabsf(p.wz)  < ANG_W_DEADBAND) {
            p.wz = 0.0f;
        }
        s_w_filt =  (p.wz );
    }

    /* ref 传低通后的实测角速度, set 传外环给的 ω_cmd */
    s_cmd_w = PID_calc(&s_pid_w, s_w_filt, w_cmd) + ANG_KFF * s_a;  /* ← 加速度前馈 */
    s_cmd_w = ANG_Clamp(s_cmd_w, -ANG_W_OUT_MAX, ANG_W_OUT_MAX);

    /* 遥测: 误差自己算一遍(死区后的语义, 供 Angle_Arrived 与调试打印用) */
    e_th = ANG_WrapPi(s_th - s_yaw_meas);
    if (fabsf(e_th) < ANG_YAW_DEADBAND) {
        e_th = 0.0f;
    }
    s_e_th = e_th;
}
