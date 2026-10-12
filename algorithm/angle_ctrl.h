/**
 * @file    angle_ctrl.h
 * @brief   航向串级控制 + S 型轨迹 —— 步进在 TIM7 中断里跑 (1kHz)
 *
 * ============================================================================
 * 本模块的定位（V1.32.0 改造）
 *
 * 原版是「角度环 → 角速度环」的串级 PID，但有三个问题：
 *   ① **数据源是 HWT906**（`Angle_Fuction()` 里调 `imu_hwt906`），与导航用的
 *      OPS9 是**两套零点不同**的航向；
 *   ② **`gyro_scale = 0.05`** —— 内环只看到 5% 的真实角速度，等于**没有速度
 *      反馈**，串级形同虚设；
 *   ③ 目标航向是**阶跃**给进去的，起步 `target_w` 直接顶到限幅，末段靠死区糊。
 *
 * 改造后：
 *   · **数据源统一换成 OPS9**（与 `Nav_GoToWorld()` 同源，见下"采样源"）；
 *   · 单位统一为**弧度制**（rad / rad·s⁻¹），与 `pose.yaw`、`pose.wz`、
 *     `Mecanum_Calc_Full_V()` 的 w 完全一致，不再有 deg/rad 换算；
 *   · **目标航向先过 S 型轨迹**再进环（不再阶跃）；
 *   · **加入速度前馈与加速度前馈**；
 *   · `gyro_scale` 这个坏点删除（滤波只保留一阶低通）。
 *
 * ============================================================================
 * 结构（自外向内）
 *
 *   ┌── S 型轨迹发生器 ──────────────────────────────┐
 *   │  三阶临界阻尼参考模型, 状态 (θ_ref, ω_ref, α_ref) │
 *   │  α̇ = K³·e_ref − 3K²·ω_ref − 3K·α_ref            │
 *   └───────────────────────────────────────────────┘
 *              ↓ ω_ref (速度前馈)        ↓ α_ref (加速度前馈)
 *   ┌── 外环: 角度 ──┐
 *   │  ω_cmd = Kp_θ·e_θ + ω_ref
 *   └───────────────┘
 *              ↓
 *   ┌── 内环: 角速度 (带宽高于外环) ──┐
 *   │  w = Kp_ω·e_ω + I_ω + K_ff·α_ref
 *   └──────────────────────────────┘
 *              ↓ cmd_w (rad/s, CCW+)
 *        任务的 Mecanum_Calc_Full_V(vx, vy, cmd_w)
 *
 * ============================================================================
 * 中断/任务分工（★ 关键约束）
 *
 *   ISR  (TIM7, 1kHz)   : 采样 → S 曲线步进 → 串级 PID → 只发布一个 float
 *   任务 (Nav_GoToWorld): 取最新 cmd_w → Mecanum → Send_commandmotor
 *
 *   **为什么 ISR 只算不发**: `Send_commandmotor()` 末尾有 `osDelay(5)`,
 *   CAN 发送链 (`can_SendCmd`) 内有 `osDelay(1)` —— 都是 FreeRTOS 阻塞 API,
 *   **在 ISR 里调用是未定义行为**。
 *
 *   ⚠ **1kHz 只算 ≠ 闭环带宽提高 10 倍**: 指令实际到达电机的速率仍是任务
 *     下发率(约 66~100Hz)。ISR 的收益是「下发时拿的是最新鲜的指令 + 前馈轨迹
 *     光滑 + 控制律与任务环解耦」。**内环带宽目标要按 66~100Hz 算。**
 *
 * ============================================================================
 * 采样源
 *
 *   统一用 `locator_ops9` 的 `pose.yaw` / `pose.wz`（**与导航同源**, 且避免
 *   两套零点打架）。`locator_ops9.get_pose()` 是**纯拷贝、无副作用**, 可以从
 *   ISR 调。
 *
 *   ⚠ 本模块**不再依赖 `imu_hwt906`** —— 原来那条链(`Angle_Fuction` 喂 IMU)
 *     已随本次改造去掉。
 *
 *   ★ **零点无关性**: `Angle_Enable()` 把 θ_ref 初始化到**当前位姿**、目标写成
 *     `yaw_now + wrap(target − yaw_now)`, 所以 OPS9 的 yaw 零点未标定
 *     完全不影响**相对转向**的正确性。
 * ============================================================================
 */

#ifndef ANGLE_CTRL_H
#define ANGLE_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#define ANG_DT             0.00f

/* ---- S 型轨迹发生器 ---- */
/** 轨迹带宽 K (rad/s)。越大越快、jerk 越大。
 *  从物理上限反推: 阶跃响应 α_peak ≈ 0.231·K²·Δθ, 取 α_max=4、Δθ_max≈1.5rad
 *  ⇒ K ≤ √(4/(0.231×1.5)) ≈ 3.4。 */
#define ANG_K              3.5f
/** 轨迹角加速度上限 (rad/s²)。★ 必须限 —— t=0⁺ 时 α̇=K³Δθ, 不限会瞬间冲高 */
#define ANG_A_MAX          4.0f
/** 轨迹角速度上限 (rad/s)。与 NAV_VMAX_W 一致 */
#define ANG_W_MAX          2.5f

/* ---- 外环: 角度 → 角速度指令 ---- */
/** 外环 P (1/s)。因为有速度前馈, 外环只管修残差, 不需要很大 */
#define ANG_KP_TH          10.0f
/** 外环输出限幅 (rad/s) */
#define ANG_W_CMD_MAX      19.5f
/** 外环角度误差死区 (rad)。沿用 NAV_YAW_DEADBAND ≈ 1.58° */
#define ANG_YAW_DEADBAND   0.0175f
#define ANG_W_DEADBAND     0.5f

/* ---- 内环: 角速度 → 电机角速度指令 ---- */
/** wz 一阶低通时间常数 (s)。内环带宽的硬上限是 OPS9 帧率 fs,
 *  低通拐点取 fs/3 附近; 起步 10ms (拐点 ≈16Hz)。 */
#define ANG_TAU_F          0.010f
/** 内环 P */
#define ANG_KP_W           0.7f
/** 内环 I  */
#define ANG_KI_W           0.002f
/** 内环积分限幅  */
#define ANG_IW_MAX         0.5f
/** 加速度前馈超前时间  */
#define ANG_KFF            0.07f
/** 内环输出限幅  */
#define ANG_W_OUT_MAX      6.5f


/** 角度到位阈值  ≈ 2.9° */
#define ANG_ARRIVE_TOL     0.05f

/* ============================================================
 * 对外接口
 * ============================================================ */

/** 上电调用一次: 清状态、失能。之后 ISR 可以一直跑(空转 ~100ns), 由 enable 门控 */
void Angle_Init(void);

/**
 * @brief 武装并使能 —— **必须在任务上下文调用, 且此时 OPS9 位姿有效**
 * @param yaw_target 目标航向 (rad)
 * @note  内部会自己读 `locator_ops9.get_pose()` 取当前位姿作为轨迹起点:
 *        θ_ref 初始化到 yaw_now、ω_ref=α_ref=0、目标写成
 *        `yaw_now + wrap(target − yaw_now)`(走近路 + 连续角) ⇒ 起步无跳变,
 *        且与 OPS9 的 yaw 零点无关。
 * @warning **每次调用都会重置轨迹** —— 想换目标又不想重起步, 用
 *          `Angle_Retarget()`。
 */
void Angle_Enable(float yaw_target);

/** 换目标但**不重置**轨迹/PID —— 用于连续追踪动目标(如圆弧切向) */
void Angle_Retarget(float yaw_target);

/** 失能: 停止计算, cmd_w 归零, 清积分/滤波状态。任务上下文调用 */
void Angle_Disable(void);

/** 取最新角速度指令 (rad/s, CCW+)。未使能时返回 0 —— 安全默认。任务侧调用 */
float Angle_GetW(void);

/** 是否已转到位 (|e_θ| < ANG_ARRIVE_TOL)。任务侧调用 */
bool Angle_Arrived(void);

/** ISR 步进函数 —— **在 TIM7 的 HAL_TIM_PeriodElapsedCallback 里调用**。
 *  @warning ISR 上下文: 内部禁用了 FreeRTOS API / printf / malloc / cosf / sinf。
 *           只用 fabsf / sqrtf (M4F 有 VSQRT.F32)。 */
void Angle_Step(void);

/* ---- 遥测 (仅供调试打印, 任务侧调用) ---- */
typedef struct {
    float    th_ref;    /* S 曲线参考角 (rad) */
    float    w_ref;     /* S 曲线参考角速度 (rad/s) —— 速度前馈量 */
    float    a_ref;     /* S 曲线参考角加速度 (rad/s²) —— 加速度前馈量 */
    float    yaw_meas;  /* 最近一拍实测航向 (rad) */
    float    w_meas;    /* 最近一拍实测角速度 (rad/s, 低通后) */
    float    cmd_w;     /* 最终输出 (rad/s) */
    float    e_th;      /* 最近一拍角度误差 (rad, 已过死区) */
    uint32_t tick;      /* 心跳: ISR 每拍 ++; 任务侧比对可发现 ISR 是否在跑 */
} AngleTelem_t;

void Angle_GetTelem(AngleTelem_t *out);

#endif /* ANGLE_CTRL_H */
