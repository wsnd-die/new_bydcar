/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制 — PID 与 Stanley 两种控制器分开实现
 *        Trace_LineFollow() 按 TRACE_USE_STANLEY 选择调用哪个
 */

#include "Common_used.h"
#include "stdlib.h"
#include "math.h"
#include "trace_tune.h"

/* ======================== 共用状态 (供打印/调试) ======================== */
float g_trace_v      = 0.0f;
float g_trace_w      = 0.0f;
float g_trace_angle  = 0.0f;
float g_trace_posx   = 0.0f;
float g_trace_target = 0.0f;

/* ================================================================
 * PID 控制器 — 串级 PID
 *   外环: 角度偏差 → PID → 目标位置
 *   内环: 位置误差 → PID → 角速度 w → Mecanum_Calc → 电机
 * ================================================================ */
/* 非 static 导出: 供 trace_tune.c 实时调参读写 */
pid_type_def g_pid_angle;
pid_type_def g_pid_pos;
static uint8_t g_pid_inited = 0;
static uint8_t s_start = 0;
static uint32_t s_start_tick = 0;
static float s_w_last = 0.0f;
static uint8_t s_side = 0;   /* 当前循迹方向: 0=左, 1=右 */

/**
 * @brief 设置循迹方向。左/右循迹角度环 PID 参数不同 (ANGLE_KP vs ANGLE_KP_R),
 *        切换方向时强制 PID 用新参数重新初始化, 并清掉旧方向的状态。
 * @note  该函数会被主循环每 10ms 调用一次, 必须只在方向"变化"时才重置 PID,
 *        否则会每周期清空 PID 状态 + 缓启动, 导致角度环失效、车卡在原地。
 */
void Trace_SetSide(uint8_t side)
{
    uint8_t new_side = (side != 0u) ? 1u : 0u;

    if (s_side == new_side) {
        return;              /* 方向未变: 保持 PID 连续运行, 不重置 */
    }
    s_side = new_side;
    g_pid_inited = 0;        /* 下次 Trace_LineFollow_PID 用新参数重新初始化 */
    PID_clear(&g_pid_angle);
    PID_clear(&g_pid_pos);
    s_start = 0;
    s_start_tick = 0;
    s_w_last = 0.0f;
}

void Trace_LineFollow_PID(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 获取 K230 角度 + 位置 ---- */
    float k230_angle, k230_posx = 0.0f;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }
    (void)K230_GetPosition(&k230_posx, NULL);

    /* ---- 2. PID 初始化 ---- */
    if (!g_pid_inited) {
        /* 角度环: 左/右循迹用不同参数 (s_side=1 为右), 调参 override 时用调参值 */
        const float a_kp = s_side ? ANGLE_KP_R : ANGLE_KP;
        const float a_ki = s_side ? ANGLE_KI_R : ANGLE_KI;
        const float a_kd = s_side ? ANGLE_KD_R : ANGLE_KD;
        const fp32 ak[3] = {
            g_tune_control_override ? g_tune_angle_kp : a_kp,
            g_tune_control_override ? g_tune_angle_ki : a_ki,
            g_tune_control_override ? g_tune_angle_kd : a_kd
        };
        const fp32 pk[3] = {
            g_tune_control_override ? g_tune_pos_kp : POS_KP,
            g_tune_control_override ? g_tune_pos_ki : POS_KI,
            g_tune_control_override ? g_tune_pos_kd : POS_KD
        };
        PID_init(&g_pid_angle, PID_POSITION, ak, ANGLE_OUT_MAX, 0.0f);
        PID_init(&g_pid_pos, PID_POSITION, pk,
                 g_tune_control_override ? g_tune_wmax : TRACE_W_MAX,
                 POS_INTEGRAL_MAX);
        g_pid_inited = 1;
        s_start = 1;
        s_start_tick = HAL_GetTick();   /* 记录缓启动起点 */
    }

    /* ---- 3. 外环: 角度 → 目标位置 ---- */
    float target_posx = PID_calc(&g_pid_angle, k230_angle, 0.0f);
    if (target_posx >  ANGLE_OUT_MAX) target_posx =  ANGLE_OUT_MAX;
    if (target_posx < -ANGLE_OUT_MAX) target_posx = -ANGLE_OUT_MAX;

    /* ---- 4. 内环: 位置反馈 → 角速度 w ---- */
    if (g_tune_control_override) {
        k230_posx += g_tune_pos_bias;
    }
    w = PID_calc(&g_pid_pos, k230_posx, target_posx);
    {
        float wmax = g_tune_control_override ? g_tune_wmax : TRACE_W_MAX;
        if (w >  wmax) w =  wmax;
        if (w < -wmax) w = -wmax;

        /* 新增: 每周期 w 变化量限幅, 防止快冲 */
        float dw = w - s_w_last;
        float step = W_RATE_MAX;             /* 每帧最大变化, 建议 0.05~0.15 */
        if (dw >  step) dw =  step;
        if (dw < -step) dw = -step;
        w = s_w_last + dw;
        s_w_last = w;
    }

    /* ---- 5. 速度自适应 ---- */
    v = g_tune_control_override ? g_tune_speed : TRACE_BASE_SPEED;
    {
        float abs_err = fabsf(k230_angle);
        if (abs_err > 40.0f)       v *= 0.5f;
        else if (abs_err > 30.0f)       v *= 0.7f;
        else if (abs_err > 15.0f)  v *= 0.85f;
    }
    /* 缓启动: 起步后 SOFT_START_MS 内 v 从 0 线性爬升到目标速度, 避免猛冲 */
    if (s_start) {
        uint32_t el = HAL_GetTick() - s_start_tick;
        if (el >= SOFT_START_MS) {
            s_start = 0;                /* 爬升完成 */
        } else {
            v = v * (float)el / (float)SOFT_START_MS;
        }
    }

    /* ---- 6. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = k230_angle;
    g_trace_posx   = k230_posx;
    g_trace_target = target_posx;

    /* ---- 7. 麦轮解算 + 发送 ---- */
    Trace_Tune_Record(k230_angle, k230_posx, target_posx, v, w);
    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}

/* ================================================================
 * Stanley 控制器 — 横向误差 e + 航向误差 θ → 转向角 δ → 角速度 w
 *   δ = θ + atan(k·e/v),  w = v·tan(δ)/L
 * ================================================================ */
void Trace_LineFollow_Stanley(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 获取 K230 角度 + 位置 ---- */
    float k230_angle, k230_posx = 0.0f;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }
    (void)K230_GetPosition(&k230_posx, NULL);

    /* ---- 2. 速度自适应 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_err = fabsf(k230_angle);
        if (abs_err > 30.0f)       v = TRACE_BASE_SPEED * 0.8f;
        else if (abs_err > 15.0f)  v = TRACE_BASE_SPEED * 0.9f;
    }
    if (v < MIN_SPEED) v = MIN_SPEED;   /* 防止除零 */

    /* ---- 3. Stanley 控制器 ---- */
    float e     = k230_posx;                    /* 横向误差 (右侧为正) */
    float theta = k230_angle;                   /* 航向误差 (度) */
    float cross_rad = atanf(STANLEY_K * e / v); /* 横向误差项 (弧度) */
    float cross_deg = cross_rad * 180.0f / (float)M_PI;
    float delta_deg = theta + cross_deg;        /* 合成目标转向角, 符号可调 */
    if (delta_deg >  MAX_STEER_DEG) delta_deg =  MAX_STEER_DEG;
    if (delta_deg < -MAX_STEER_DEG) delta_deg = -MAX_STEER_DEG;
    float delta_rad = delta_deg * (float)M_PI / 180.0f;
    w = v * tanf(delta_rad) / WHEEL_BASE;
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 4. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = k230_angle;
    g_trace_posx   = k230_posx;
    g_trace_target = delta_deg;

    /* ---- 5. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}

/* ================================================================
 * 入口 — 按 TRACE_USE_STANLEY 选择控制器
 * ================================================================ */

#define IS_LineFind_USED    1

#define Slow_Findw     0.2
#define Slow_Findv     0.2f

uint8_t  IS_offLine() {
   uint8_t i;
    i=Grayscale_Serial_Read();
    return i==0x00;
}

void Slow_LineFind(SystemMode_t Event) {

    MecanumResult motor;

    switch (Event) {
        case Event_LinFolL:
            /* ---- 3. 全向移动解算 (vx, vy, w=0) ---- */
            motor = Mecanum_Calc_Full(Slow_Findv,0 , Slow_Findw);
            /* ---- 4. 发送电机指令 ---- */
            Send_commandmotor(&motor);
            break;
        case Event_LinFolR:
            /* ---- 3. 全向移动解算 (vx, vy, w=0) ---- */
            motor = Mecanum_Calc_Full(Slow_Findv, 0, -Slow_Findw);
            /* ---- 4. 发送电机指令 ---- */
            Send_commandmotor(&motor);
            break;
         default:
            break;


    }

}



void Trace_LineFollow(SystemMode_t Event)
{
       /* 每循迹周期: 同步调参增益 + 处理串口命令 */
#if IS_LineFind_USED
    if (IS_offLine()) {
        Slow_LineFind(Event);
    }
    else {
        Trace_Tune_Service();

#if TRACE_USE_STANLEY
        Trace_LineFollow_Stanley();
#else
        Trace_LineFollow_PID();
#endif

    }


#else

    Trace_Tune_Service();

#if TRACE_USE_STANLEY
    Trace_LineFollow_Stanley();
#else
    Trace_LineFollow_PID();
#endif


#endif


}