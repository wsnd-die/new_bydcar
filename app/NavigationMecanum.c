
#include "Common_used.h"          /* libc + HAL + FreeRTOS + osDelay */
#include "NavigationMecanum.h"

#include <iso646.h>

#include "mecanum.h"              /* MecanumResult / Mecanum_Calc_Full_V / Mecanum_Vel_Execute */
#include "pid.h"                  /* pid_type_def */
#include "worker_task.h"          /* g_angle_ctrl_enable (角度环契约量) */
#include "ops9_g491_uart3.h"      /* extern const LocatorDev_t locator_ops9 */
#include "pose_data.h"            /* PoseData_t */
#include <math.h>
#include "block_collect.h"

#include "block_basic.h"
#include "Send_motor.h"           /* Send_commandmotor  */
#include "angle_ctrl.h"         /* 串级航向环 (NAV_YAW_CASCADE=1 时用) */

/*  全局量 */
 uint8_t s_idx=0;
World_Dir_t Self_Dir = {0.0f, 0.0f, 0.0f};

World_Dir_t g_waypoints[NAV_WAYPOINT_MAX] = {
    {   0.502536f,-1.081990f,-1.089054f }, /*  1 奖杯点 */
    {   0.708793f,-1.323468f,-0.405103f }, /*  2 奖杯点 */
    {   1.135373f,-1.413104f, 0.065007f }, /*  3 奖杯点 */

    {   1.726253f,-0.292191f, 0.0f }, /*  4 亚军点 */
    {   1.729634f, 0.070066f, 0.0f }, /*  5 冠军点 */
    {   1.713531f, 0.323497f, 0.0f}, /*  6 季军点 */

    {   1.9902776f, 0.464762f, 1.500430f},/*圆弧起点*/
    //{   1.801493f, 0.768879f, 1.459011f }, /*  7 e 点 */
    // {   1.674586f, 1.154120f, 2.231404f }, /*  8 d 点 */
    // {   1.308177f, 1.418482f, 2.770012f }, /*  9 c 点 */
    // {   0.865644f, 1.449459f,-2.979479f }, /* 10 b 点 */
    // {   0.450772f, 1.269985f,-2.411954f }, /* 11 a 点 */

    {   1.076288f,-0.395111f,-1.545575f }, /* 12 摆放e 点 */
    // {   0.946693f, 0.100097f,-1.499596f }, /* 13 摆放d 点 */
    {   0.930956f, 0.113929f,-0.673924f }, /* 14 摆放c 点 */
    {   0.946404f, 0.425504f, 0.818769f }, /* 15 摆放a 点 */
    // {   0.660611f, 0.167082f, 0.855780f }, /* 16 摆放b 点 */


};

uint8_t g_waypoint_count = 10u;

/* ==================================================================
 * 路线段打断
 *
 * g_route_abort (worker_task.c, 由 IR 进料置位) 让 Nav_GoToWorld() 中途放弃
 * 当前目标点。被打断仍返回 false, 靠 Nav_LastAborted() 区分"超时"和"打断"。
 * ================================================================== */
static bool s_nav_aborted = false;

/** 1 = 当前有路线段正被 Nav_GoToWorld 驱动 (Route_AbortRequest 判据用)。 */
volatile uint8_t g_nav_running = 0u;

/** 本段平移速度倍率, 1.0 = 默认。缩放 VMAX / BRK / 近场爬行上限,
 *  航向环不受影响。Nav_GoToWorld() 的三个出口各还原一次。 */
volatile float g_nav_leg_speed = 1.0f;

/** 上一次 Nav_FeDuanPoint() 走完的点号 (1 基); 0 = 没走到 / 已走完。 */
static uint8_t s_last_wp = 0u;

/** 本段近场爬行上限 (m/s)。默认 NAV_CREEP_X_V/_Y_V;
 *  Nav_FeDuanPoint() 遇到减速点时会临时压低, 走完立刻恢复。 */
static float s_near_lim_x = NAV_CREEP_X_V;
static float s_near_lim_y = NAV_CREEP_Y_V;

bool Nav_LastAborted(void)
{
    return s_nav_aborted;
}

uint8_t Nav_LastWaypointNo(void)
{
    return s_last_wp;
}

/* ==================================================================
 * 静态工具
 * ================================================================== */
#define MECANUM_PI   3.141592653589793f
/** @brief 角度归一化到 [-π, π] */
static float NAV_WrapPi(float a)
{
    while (a >  MECANUM_PI) a -= 2.0f * MECANUM_PI;
    while (a < -MECANUM_PI) a += 2.0f * MECANUM_PI;
    return a;
}

static float NAV_Clamp(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/**
 * @brief 软启动: 对速度指令做加速度斜坡
 * @param cur    当前指令值
 * @param target 期望指令值
 * @param acc    加速度限幅 (m/s² 或 rad/s²)
 * @param dt     控制周期 (s)
 * @retval 斜坡后的指令值
 */
static float NAV_Ramp(float cur, float target, float acc, float dt)
{
    float max_dv = acc * dt;
    return cur + NAV_Clamp(target - cur, -max_dv, max_dv);
}

#if NAV_XY_PROFILE
/**
 * @brief 平移轴参考速度 —— 按误差在近场/远场两律之间平滑切换。
 *
 *      lim_far  = √(2·a_brk·|e|)          远场: 制动曲线上限
 *      lim_near = NAV_CREEP_X_V / _Y_V    近场: 爬行上限
 *      w        = smoothstep(|e|; LO, HI)
 *      v        = clamp(Kp·e − Kd·v_now, ±lerp(lim_near, lim_far, w))
 */
static float NAV_AxisRef(float err, float v_max, float a_brk, float kp,
                         float kd, float v_now, char choice)
{
    /* 近/远场切换权重 w ∈ [0,1]: |e| ≤ LO → 0 (纯近场), ≥ HI → 1 (纯远场)。
     * smoothstep 两端导数为 0 —— 切换点连续、无拐点。 */
    float ae = fabsf(err);
    float w  = NAV_Clamp((ae - NAV_SWITCH_LO) / (NAV_SWITCH_HI - NAV_SWITCH_LO),
                         0.0f, 1.0f);
    w = w * w * (3.0f - 2.0f * w);

    if (choice == 'x')
    {
        float x_v   = kp * err - kd * v_now;            /* 近场律: 线性 P + D  */
        float x_lim  = sqrtf(2.0f * a_brk * ae);        /* 远场律: 制动曲线上限 */
        float x_near = s_near_lim_x * g_nav_leg_speed;  /* 近场爬行上限 × 本段倍率 */
        x_lim = x_near + (x_lim - x_near) * w;          /* 按 w 混到近场爬行上限 */

        return NAV_Clamp(NAV_Clamp(x_v, -x_lim, x_lim), -v_max, v_max);
    }
    if (choice == 'y')
    {
        float y_v   = kp * err - kd * v_now;            /* 近场律: 线性 P + D */
        float y_lim  = sqrtf(2.0f * a_brk * ae);        /* 远场律: 制动曲线上限 */
        float y_near = s_near_lim_y * g_nav_leg_speed;  /* 近场爬行上限 × 本段倍率 */
        y_lim = y_near + (y_lim - y_near) * w;          /* 按 w 到近场爬行上限 */

        return NAV_Clamp(NAV_Clamp(y_v, -y_lim, y_lim), -v_max, v_max);
    }

    return 0.0f;
}
#endif /* NAV_XY_PROFILE */


static void NAV_Stop(void)
{
    MecanumResult z = Mecanum_Calc_Full_V(0.0f, 0.0f, 0.0f);
    Send_commandmotor(&z);
}

/* 世界系位置闭环 */

bool Nav_GoToWorld(float target_x, float target_y, float target_yaw)
{
    s_nav_aborted = false;

    g_angle_ctrl_enable = 0;
    osDelay(10);

#if !NAV_XY_PROFILE
    pid_type_def pid_x, pid_y;
    fp32 k_x[3] = {2.3f, 0.0f, 2.91f};
    PID_init(&pid_x, PID_POSITION, k_x, NAV_VMAX_X, 0.0f);
    fp32 k_y[3] = {2.3f, 0.0f, 2.91f};
    PID_init(&pid_y, PID_POSITION, k_y, NAV_VMAX_Y, 0.0f);
#endif

    float vx_cmd = 0.0f, vy_cmd = 0.0f;

    uint32_t t0 = osKernelGetTickCount();
    uint8_t  arrive  = 0u;   /* 连续到达 tick 数 */
    uint8_t  invalid = 0u;   /* 反馈连续无效 tick 数 */


    uint8_t  dt_armed = 0u;
    uint32_t t_prev   = 0u;
    uint32_t dt_sum   = 0u, dt_n = 0u;


    uint8_t s0_set = 0u;
    float   s0x = 0.0f, s0y = 0.0f, s0yaw = 0.0f;
    float   ovs_x = 0.0f, ovs_y = 0.0f, ovs_yaw = 0.0f;
    float   last_ex = 0.0f, last_ey = 0.0f, last_eyaw = 0.0f;

    PoseData_t pose;
    uint8_t    heading_armed = 0u;

    g_nav_running = 1u;

    for (;;)
    {
        locator_ops9.get_pose(&pose);

        {
            uint32_t t_now = osKernelGetTickCount();
            if (dt_armed) { dt_sum += (t_now - t_prev); dt_n++; }
            else          { dt_armed = 1u; }
            t_prev = t_now;
        }


        if (g_route_abort)
        {
            g_route_abort = 0u;
            NAV_Stop();
            if (pose.valid)
            {
                Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
            }
            s_nav_aborted = true;
            Angle_Disable();      /* 关串级航向环 */
            g_nav_running = 0u;
            g_nav_leg_speed = 1.0f;   /* 本段倍率用完即还原 */
            return false;
        }

        if (!pose.valid)
        {
            invalid++;
            arrive = 0;
            if (invalid > NAV_MAX_INVALID_TICKS)
            {
                vx_cmd = 0.0f; vy_cmd = 0.0f;
                NAV_Stop();
            }
        }
        else
        {
            invalid = 0;

#if NAV_YAW_CASCADE
            if (!heading_armed) {
                Angle_Enable(target_yaw);
                heading_armed = 1u;
            }
#endif

            float ex   = target_x - pose.x;
            float ey   = target_y - pose.y;
            float eyaw = NAV_WrapPi(target_yaw - pose.yaw);

            /* 调参仪表: 累计过冲量 (ovs 含 yaw) */
            if (!s0_set) {
                s0x   = (ex   >= 0.0f) ? 1.0f : -1.0f;
                s0y   = (ey   >= 0.0f) ? 1.0f : -1.0f;
                s0yaw = (eyaw >= 0.0f) ? 1.0f : -1.0f;
                s0_set = 1u;
            }
            float ex_x =ex-last_ex;;
            float ex_y = ey-last_ey;
            if (-s0x   * ex   > ovs_x)   ovs_x   = -s0x   * ex;
            if (-s0y   * ey   > ovs_y)   ovs_y   = -s0y   * ey;
            if (-s0yaw * eyaw > ovs_yaw) ovs_yaw = -s0yaw * eyaw;
            last_ex = ex; last_ey = ey; last_eyaw = eyaw;


#if NAV_YAW_CASCADE

            Angle_Step();
            float w_w = Angle_GetW();
#else
            /* 旧: 手写单环 PD (NAV_YAW_CASCADE=0 时的回退, 逐字节等价于改前) */
            float eyaw_dz = (fabsf(eyaw) < NAV_YAW_DEADBAND) ? 0.0f : eyaw;
            float wz = (fabsf(pose.wz) < NAV_W_DEADBAND) ? 0.0f : pose.wz;
            float w_w = NAV_KP_YAW * eyaw_dz - NAV_KD_YAW * wz;
            w_w = NAV_Clamp(w_w, -NAV_VMAX_W, NAV_VMAX_W);
#endif

#if NAV_XY_PROFILE

            /* VMAX / BRK 乘本段倍率; KP / KD 不乘 (那是控制律形状, 不是快慢)。 */
            float vx_ref = NAV_AxisRef(ex, NAV_VMAX_X * g_nav_leg_speed,
                                       NAV_BRK_X * g_nav_leg_speed, NAV_KP_X_LIN,
                                       NAV_KD_X_LIN, ex_x, 'x');
            float vy_ref = NAV_AxisRef(ey, NAV_VMAX_Y * g_nav_leg_speed,
                                       NAV_BRK_Y * g_nav_leg_speed, NAV_KP_Y_LIN,
                                       NAV_KD_Y_LIN, ex_y, 'y');
#else
            float vx_ref = PID_calc(&pid_x, pose.x, target_x);
            float vy_ref = PID_calc(&pid_y, pose.y, target_y);
#endif

            vx_cmd = NAV_Ramp(vx_cmd, vx_ref, NAV_ACC_XY, NAV_DT);
            vy_cmd = NAV_Ramp(vy_cmd, vy_ref, NAV_ACC_XY, NAV_DT);


            float c = cosf(pose.yaw), s = sinf(pose.yaw);
            float bvx =  vx_cmd * c + vy_cmd * s;
            float bvy = -vx_cmd * s + vy_cmd * c;
            (void)bvx; (void)bvy;

            MecanumResult res = Mecanum_Calc_Full_V(bvx, bvy, w_w);
            Send_commandmotor(&res);

            if (fabsf(ex) <= NAV_TOL_XY && fabsf(ey) <= NAV_TOL_XY &&
                fabsf(eyaw) <= NAV_TOL_YAW &&
                fabsf(vx_cmd) <= NAV_ARRIVE_VMAX && fabsf(vy_cmd) <= NAV_ARRIVE_VMAX)
            {
                if (++arrive >= NAV_ARRIVE_TICKS)
                {
                    printf("[NAV] ARRIVE t=%ums ex=%.3f ey=%.3f eyaw=%.3f vx=%.2f vy=%.2f | ovs=%.3f/%.3f ovsyaw=%.4f dt=%.1fms(n=%u)\r\n",
                           (unsigned)(osKernelGetTickCount() - t0),
                           ex, ey, eyaw, vx_cmd, vy_cmd, ovs_x, ovs_y, ovs_yaw,
                           (dt_n > 0u) ? ((float)dt_sum / (float)dt_n) : 0.0f,
                           (unsigned)dt_n);
                    break;
                }
            }
            else
            {
                arrive = 0;
            }
        }
        if ((osKernelGetTickCount() - t0) >= NAV_TIMEOUT_MS)
        {
            NAV_Stop();
            printf("[NAV] TIMEOUT t=%ums ex=%.3f ey=%.3f eyaw=%.3f ovsyaw=%.4f valid=%u vx=%.2f vy=%.2f dt=%.1fms(n=%u)\r\n",
                   (unsigned)(osKernelGetTickCount() - t0),
                   last_ex, last_ey, last_eyaw, ovs_yaw, (unsigned)pose.valid, vx_cmd, vy_cmd,
                   (dt_n > 0u) ? ((float)dt_sum / (float)dt_n) : 0.0f,
                   (unsigned)dt_n);
            if (pose.valid)
            {
                Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
            }
            Angle_Disable();      /* 关串级航向环 (出口二: 超时) */
            g_nav_running = 0u;
            g_nav_leg_speed = 1.0f;   /* 本段倍率用完即还原 */
            return false;
        }

        osDelay(NAV_LOOP_TICKS);   /* 10ms → 100Hz */
    }

    NAV_Stop();
    Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
    Angle_Disable();          /* 关串级航向环  */
    g_nav_running = 0u;
    g_nav_leg_speed = 1.0f;   /* 本段倍率用完即还原 */
    return true;
}

bool Nav_RunWaypoints(void)
{
    for (uint8_t i = 0; i < g_waypoint_count; i++)
    {
        if (!Nav_GoToWorld(g_waypoints[i].x, g_waypoints[i].y, g_waypoints[i].yaw))
            return false;
    }
    return true;
}


bool Nav_FeDuanPoint(void)
{

    /* 上界取 g_waypoint_count, 但先夹一道 NAV_WAYPOINT_MAX —— 该表是手工维护的,
     * count 写大了会读越界。 */
    uint8_t count = g_waypoint_count;
    if (count > (uint8_t)NAV_WAYPOINT_MAX) {
        count = (uint8_t)NAV_WAYPOINT_MAX;
    }

    if (s_idx >= count) {
        s_last_wp = 0u;                 /* 路线已走完: 清掉"刚走过的点号" */
        return true;
    }

    s_last_wp = (uint8_t)(s_idx + 1u);  /* 本段走的点号 (1 基), 给蹭料判断用 */


    if (s_idx == (uint8_t)NAV_SLOW_WP_IDX) {
        s_near_lim_x = NAV_SLOW_WP_LIM;
        s_near_lim_y = NAV_SLOW_WP_LIM;
    }
    if (s_idx == (uint8_t)NAV_SLOW_WP_IDX2)
    {
        s_near_lim_x = 0.45f;
        s_near_lim_y = 0.45f;
    }
    else {
        s_near_lim_x = NAV_CREEP_X_V;
        s_near_lim_y = NAV_CREEP_Y_V;
    }

    bool ok = Nav_GoToWorld(g_waypoints[s_idx].x,
                            g_waypoints[s_idx].y,
                            g_waypoints[s_idx].yaw);

    s_near_lim_x = NAV_CREEP_X_V;       /* 恢复默认, 免泄漏到别的调用者 */
    s_near_lim_y = NAV_CREEP_Y_V;

    if (!ok) {
        if (Nav_LastAborted()) {
            printf("[NAV] 第 %u 点被 IR 打断, 游标 -> %u\r\n",
                   (unsigned)s_idx, (unsigned)(s_idx + 1u));
            s_idx++;
            return true;
        }
        printf("[NAV] 分点导航第 %u 点超时, 游标停在原点待重试\r\n",
               (unsigned)s_idx);
        s_last_wp = 0u;                 /* 超时没走到, 不算"到过这个点" */
        return false;
    }

    s_idx++;
    return true;
}

bool Nav_MoveBody(float target_x, float target_y, float target_yaw) {

    float targetworld_x=target_x+Self_Dir.x,
    targetworld_y=target_y+Self_Dir.y,
    targetworld_yaw=target_yaw+Self_Dir.yaw;

    return Nav_GoToWorld(targetworld_x ,targetworld_y  , targetworld_yaw );


}


/* ==================================================================
 * 定半径圆弧 (OPS9 位置闭环)
 *
 * 圆心由起点位姿推得; 每拍: 切向匀速 (软启动) + 径向 P 拉回 + 航向
 * 前馈 v/R 与切线误差 P。扫够 |Angle| 退出, 超时 / 打断在此收尾。
 * ================================================================== */

void Nav_Cricle(Nav_Cricle_t Cricle_t, float Radius, float Angle)
{
    const float R     = fabsf(Radius);
    const float sweep = fabsf(Angle);
    const float dir   = (Cricle_t == Nav_CricleR) ? -1.0f : 1.0f; /* L=CCW(+), R=CW(-) */
    Angle_Disable();

    if (R < 0.01f || sweep < 0.01f) {
        printf("[NAV-ARC] 参数非法 R=%.3f A=%.3f, 不动作\r\n", R, sweep);
        return;
    }
    BlockBasic_LiftToAbs(2);
    /* 借走电机控制权 (退出不恢复角度环) */
    g_angle_ctrl_enable = 0;
    osDelay(10);

    /* 起点位姿 → 圆心 (左弧在车左侧 R 处, 右弧在右侧) */
    PoseData_t pose;
    locator_ops9.get_pose(&pose);
    for (uint32_t wait = 0u; !pose.valid && wait < 500u; wait += 20u) {
        osDelay(5);
        locator_ops9.get_pose(&pose);
    }
    if (!pose.valid) {
        printf("[NAV-ARC] 起点 OPS9 无效, 放弃\r\n");
        return;
    }

    float c0 = cosf(pose.yaw), s0 = sinf(pose.yaw);
    float cx = pose.x - dir * R * s0;   /* dir=+1(左): R·(-sin,cos); dir=-1(右): R·(sin,-cos) */
    float cy = pose.y + dir * R * c0;

    float    v_cmd   = 0.0f;    /* 线速度软启动当前值 */
    float    swept   = 0.0f;    /* 已扫过圆心角 (带方向, 逐拍累加) */
    float    last_th = atan2f(pose.y - cy, pose.x - cx);
    uint8_t  invalid = 0u;
    uint32_t t0      = osKernelGetTickCount();
    uint32_t timeout_ms = (uint32_t)(sweep * R / NAV_ARC_V * 1.5f * 1000.0f) + 2000u;

    g_nav_running = 1u;

    for (;;)
    {
        locator_ops9.get_pose(&pose);


        if (BlockCollect_IsDone())
        {
            printf("[NAV-ARC] ABORT 采集完成 swept=%.3f rad\r\n", swept);
            break;
        }

        if (!pose.valid)
        {
            invalid++;
            if (invalid > NAV_MAX_INVALID_TICKS) { v_cmd = 0.0f; NAV_Stop(); }
            osDelay(NAV_LOOP_TICKS);
            continue;
        }
        invalid = 0u;

        v_cmd = NAV_Ramp(v_cmd, NAV_ARC_V, NAV_ARC_ACC, NAV_DT);

        /* 扫角累加: 每拍增量很小, wrap 到 ±π 后跨边界不丢圈数 */
        float th = atan2f(pose.y - cy, pose.x - cx);
        swept += NAV_WrapPi(th - last_th);
        last_th = th;

        if (swept * dir >= sweep)       /* 沿所选方向扫够目标角 */
        {
            printf("[NAV-ARC] DONE dir=%s swept=%+.3f rad R=%.3f\r\n",
                   (Cricle_t == Nav_CricleR) ? "R" : "L", swept, R);
            break;
        }

        /* 径向误差 (正 = 圆外) → 修正速度拉回圆上 */
        float rx = pose.x - cx, ry = pose.y - cy;
        float r  = sqrtf(rx * rx + ry * ry);
        if (r < 1e-3f) { r = 1e-3f; }   /* 车恰在圆心 (理论上不会), 防除零 */
        float e_r = r - R;
        float nx  = rx / r, ny = ry / r;          /* 圆心 → 车 单位矢 */
        float tx  = -dir * ny, ty = dir * nx;     /* 切向单位矢 (沿前进方向) */

        float vr  = NAV_Clamp(-NAV_ARC_KP_R * e_r, -NAV_ARC_VMAX_R, NAV_ARC_VMAX_R);
        float vwx = v_cmd * tx + vr * nx;
        float vwy = v_cmd * ty + vr * ny;

        /* 航向: 前馈 v/R + 切线误差 P - wz D */
        float w_ff     = dir * v_cmd / R;
        float e_yaw    = NAV_WrapPi(atan2f(ty, tx) - pose.yaw);
        float e_yaw_dz = (fabsf(e_yaw) < NAV_YAW_DEADBAND) ? 0.0f : e_yaw;
        float wz       = (fabsf(pose.wz) < NAV_YAW_DEADBAND) ? 0.0f : pose.wz;
        float w_w = w_ff + NAV_KP_YAW * e_yaw_dz - NAV_KD_YAW * PID_Filter(0.2f, wz);
        w_w = NAV_Clamp(w_w, -NAV_VMAX_W, NAV_VMAX_W);

        /* 世界 → 车体 → 四轮 */
        float c = cosf(pose.yaw), s = sinf(pose.yaw);
        float bvx = vwx * c + vwy * s;
        float bvy = -vwx * s + vwy * c;
        MecanumResult res = Mecanum_Calc_Full_V(bvx, bvy, w_w);
        Send_commandmotor(&res);

        if ((osKernelGetTickCount() - t0) >= timeout_ms)
        {
            printf("[NAV-ARC] TIMEOUT swept=%.3f/%+.3f rad R=%.3f\r\n",
                   swept, dir * sweep, R);
            break;
        }

        osDelay(10);
    }

    NAV_Stop();
    if (pose.valid) {
        Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
    }

    g_nav_running = 0u;
    NLF_Request(Event_Navigation);
}



#define SHORT_VMAX         0.3f    /* 短距平移速度上限 m/s */
#define SHORT_VX           1.9f
#define SHORT_VY           2.3f
#define SHORT_KD_X         1.0f    /* 平移 D: 每拍误差变化量增益 (★初值, 待实测) */
#define SHORT_KD_Y         1.0f    /* 平移 D: 每拍误差变化量增益 */
#define SHORT_TOL_XY       0.005f   /* 到达容差 1cm */
#define SHORT_TOL_YAW      0.05f   /* 到达容差 ~2.9° */
#define SHORT_ARRIVE_V     0.1f    /* 到达时指令速度须已衰减到 m/s */
#define SHORT_ARRIVE_TICKS 4u      /* 连续 N 拍判到达 */
#define SHORT_TIMEOUT_MS   3000u   /* 短距超时 ms */

void NAV_SHORT_WORD(float x, float y, float yaw)
{

    osDelay(10);

    /* 车体量 → 世界目标 (以入口时的 OPS9 位姿为基准) */
    PoseData_t pose;
    locator_ops9.get_pose(&pose);
    if (!pose.valid)
    {
        printf("[SHORT] 起点 OPS9 无效, 不动作\r\n");
        return;
    }
    float c0 = cosf(pose.yaw), s0 = sinf(pose.yaw);
    float tx   = pose.x + (x * c0 - y * s0);
    float ty   = pose.y + (x * s0 + y * c0);
    float tyaw = NAV_WrapPi(pose.yaw + yaw);

    float vx_cmd = 0.0f, vy_cmd = 0.0f;
    float last_ex = 0.0f, last_ey = 0.0f;
    uint8_t  exy_armed = 0u;        /* 第一拍没有"上一拍误差", D 项先不参与 */
    uint8_t  arrive   = 0u;
    uint8_t  invalid  = 0u;
    uint8_t  heading_armed = 0u;
    uint32_t t0 = osKernelGetTickCount();
    s_nav_aborted = false;
    g_nav_running = 1u;

    for (;;)
    {
        locator_ops9.get_pose(&pose);   /* 只读, ops9imu 任务在喂 */

        /* 出口一: IR 进料打断 (语义与 Nav_GoToWorld 相同) */
        if (g_route_abort)
        {
            g_route_abort = 0u;
            s_nav_aborted = true;
            break;
        }

        if (!pose.valid)
        {
            if (++invalid > NAV_MAX_INVALID_TICKS)
            {
                vx_cmd = 0.0f; vy_cmd = 0.0f;
                NAV_Stop();
            }
            osDelay(NAV_LOOP_TICKS);
            continue;
        }
        invalid = 0u;

        float ex   = tx - pose.x;
        float ey   = ty - pose.y;
        float eyaw = NAV_WrapPi(tyaw - pose.yaw);

        /* 航向: 与 Nav_GoToWorld 同一套开关 */
// #if NAV_YAW_CASCADE
//         if (!heading_armed) {
//             Angle_Enable(tyaw);         /* 起点由 angle_ctrl 自己读 OPS9, 无跳变 */
//             heading_armed = 1u;
//         }
//         Angle_Step();
//         float w_w = Angle_GetW();
// #else
        float eyaw_dz = (fabsf(eyaw) < NAV_YAW_DEADBAND) ? 0.0f : eyaw;
        float wz      = (fabsf(pose.wz) < NAV_W_DEADBAND) ? 0.0f : pose.wz;
        float w_w = NAV_KP_YAW * eyaw_dz - NAV_KD_YAW * wz;
        w_w = NAV_Clamp(w_w, -NAV_VMAX_W, NAV_VMAX_W);
// #endif

        /* 平移 PD: D 对每拍误差变化量 (逼近时 dex<0 → 减速 = 阻尼) */
        float dex = exy_armed ? (ex - last_ex) : 0.0f;
        float dey = exy_armed ? (ey - last_ey) : 0.0f;
        last_ex = ex; last_ey = ey;
        exy_armed = 1u;

        float vx_ref = NAV_Clamp(SHORT_VX * ex + SHORT_KD_X * dex, -SHORT_VMAX, SHORT_VMAX);
        float vy_ref = NAV_Clamp(SHORT_VY * ey + SHORT_KD_Y * dey, -SHORT_VMAX, SHORT_VMAX);
        vx_cmd = NAV_Ramp(vx_cmd, vx_ref, NAV_ACC_XY, NAV_DT);
        vy_cmd = NAV_Ramp(vy_cmd, vy_ref, NAV_ACC_XY, NAV_DT);

        /* 世界 → 车体 → 四轮 */
        float c = cosf(pose.yaw), s = sinf(pose.yaw);
        float bvx =  vx_cmd * c + vy_cmd * s;
        float bvy = -vx_cmd * s + vy_cmd * c;
        MecanumResult res = Mecanum_Calc_Full_V(bvx, bvy, w_w);
        Send_commandmotor(&res);

        if (fabsf(ex) <= SHORT_TOL_XY && fabsf(ey) <= SHORT_TOL_XY &&
            fabsf(eyaw) <= SHORT_TOL_YAW &&
            fabsf(vx_cmd) <= SHORT_ARRIVE_V && fabsf(vy_cmd) <= SHORT_ARRIVE_V)
        {
            if (++arrive >= SHORT_ARRIVE_TICKS) break;   /* 出口二: 到达 */
        }
        else
        {
            arrive = 0u;
        }

        if ((osKernelGetTickCount() - t0) >= SHORT_TIMEOUT_MS) break;   /* 出口三: 超时 */

        osDelay(NAV_LOOP_TICKS);
    }

    /* 三个出口共用收尾: 零速停车 + 刷新 Self_Dir + 关航向环 */
    NAV_Stop();
    if (pose.valid) {
        Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
    }
    Angle_Disable();
    g_nav_running = 0u;
}


