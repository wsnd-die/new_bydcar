/**
 * @file    NavigationMecanum.c
 * @brief   世界系位置闭环 —— OPS9 位姿反馈 → 三轴 PD → 电机速度环
 *
 *  控制链:
 *     目标 (tx, ty, tyaw) ─┬─ x/y 轴 pid_type_def (Ki=0, 即 PD)
 *                         └─ yaw 轴手写 PD (误差须 wrap 到 ±π)
 *          世界系期望速度 → 软启动加速度斜坡 (缓启动)
 *          → 世界→车体旋转 (当前 yaw)
 *          → Mecanum_Calc_Full_V(vx, vy, w) 逆解
 *          → Send_commandmotor() 下发电机速度环 (Emm_V5_Vel_Control)
 *
 *  运行环境: 阻塞式流程任务上下文 (NLF_TASK 的 NLF_RunFlow, 以及
 *  gripper_task 里 Place() 调 Nav_MoveBody 的那条路), 100Hz (osDelay(10))。
 *  与角度环的电机控制权契约见 worker_task.c 文件头:
 *  入口先置 g_angle_ctrl_enable = 0 并 osDelay(20), 等角度环下降沿零速,
 *  此后本文件独占电机命令 —— 但**退出时不把该标志恢复成 1**, 见 Nav_GoToWorld。
 *
 *  反馈源: locator_ops9 (device/ops9_g491_uart3.c), 世界系 x/y/yaw,
 *  单位 m / rad。update() 由 ops9imu_fuction 任务每 100ms 调一次
 *  (app_freertos.c 的 osDelay(100); 驱动侧帧超时 OPS9_FRAME_TIMEOUT_MS=500ms),
 *  本文件只调 get_pose() (GetLatest 是消费式读取, 多调 update 会抢帧)。
 */
#include "Common_used.h"          /* libc + HAL + FreeRTOS + osDelay */
#include "NavigationMecanum.h"
#include "mecanum.h"              /* MecanumResult / Mecanum_Calc_Full_V / Mecanum_Vel_Execute */
#include "pid.h"                  /* pid_type_def */
#include "worker_task.h"          /* g_angle_ctrl_enable (角度环契约量) */
#include "ops9_g491_uart3.h"      /* extern const LocatorDev_t locator_ops9 */
#include "pose_data.h"            /* PoseData_t */
#include <math.h>
#include "Send_motor.h"           /* Send_commandmotor (下游执行器) */

/* ==================================================================
 * 全局量 (唯一定义处, extern 声明在 NavigationMecanum.h)
 * ================================================================== */

World_Dir_t Self_Dir = {0.0f, 0.0f, 0.0f};

/* ==================================================================
 * 分点导航路径点表 (世界坐标系, 单位 m / rad)
 *
 * 坐标是 2026-09-28 现场示教值 (手推车到每个物理点, 读 locator_ops9.get_pose
 * 记录)。yaw 一律是**弧度**, 直接取自 OPS9, 不要再乘 NAV_DEG2RAD。
 *
 * ⚠ 配合 worker_task.c 的 NF_AUTOSTART=1, 这张表就是"上电即发车"的路线。
 *   改表前后务必确认车周围清空。
 *
 * @note 表内每行右边是示教时记的物理点名字 (现场命名, 与比赛场地的
 *       a~e / 二维码点 / 放置点对应)。示教顺序即行驶顺序。
 * @note 改完记得把 g_waypoint_count 同步成实际行数。
 * ================================================================== */
World_Dir_t g_waypoints[NAV_WAYPOINT_MAX] = {
    {   0.475699f, -1.061652f, -1.022789f }, /*  1 奖杯二维码点 */
    {   0.763282f, -1.356906f, -0.426024f }, /*  2 亚军点 */
    {   1.151263f, -1.354195f, -0.170347f }, /*  3 亚军点 */
    {   1.790909f, -0.229022f, -0.041333f }, /*  4 亚军点 */
    {   1.801615f,  0.036682f, -0.028979f }, /*  5 冠军点 */
    {   1.805676f,  0.286101f, -0.002642f }, /*  6 季军点 */
    {   2.000804f,  0.715744f,  1.647523f }, /*  7 e 点 */
    {   1.855496f,  1.132472f,  2.191320f }, /*  8 c 点 */
    {   1.517152f,  1.423086f,  2.705060f }, /*  9 d 点 */
    {   1.077295f,  1.484286f,  3.134072f }, /* 10 a 点 */
    {   0.619106,1.325792,-2.544706      }, /* 11 b 点 ← ⚠ 仍是旧占位值, 待示教 */
    {   1.002958f, -0.372130f, -1.650208f }, /* 12 e 点 */
    {   0.864636f,  0.124412f, -1.390370f }, /* 13 c 点 */
    {   0.983620f,  0.128080f, -0.876166f }, /* 14 d 点 */
    {   0.926343f,  0.700827f, -0.078535f }, /* 15 a 点 */
    {   0.622895f,  0.540434f, -0.094821f }, /* 16 b 点 */
    {   0.007499f,  0.0f, -0.127119f }, /* 17 回家点 */
};

/* 实际点数 —— 必须与上表行数一致 (Nav_FeDuanPoint 的游标上界) */
uint8_t g_waypoint_count = 17u;

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

/** @brief 零速停车 (Mecanum_Calc_Full_V(0,0,0) → 执行器) */
static void NAV_Stop(void)
{
    MecanumResult z = Mecanum_Calc_Full_V(0.0f, 0.0f, 0.0f);
    Send_commandmotor(&z);
}

/* ==================================================================
 * 世界系位置闭环
 * ================================================================== */

bool Nav_GoToWorld(float target_x, float target_y, float target_yaw)
{
    /* 1. 夺回电机控制权: 按契约关角度环, 等 FC_TASK 下降沿零速
     *    (worker_task.c:28-35) */
    g_angle_ctrl_enable = 0;
    osDelay(20);

    /* 2. x/y 轴用 pid_type_def (Ki=0 即 PD, PID_POSITION 位置式)。
     *    yaw 不用它: PID_calc 内部误差不 wrap, 跨 ±π 会跳 2π, 手写。 */
    pid_type_def pid_x, pid_y;
    fp32 k[3] = {NAV_KP_XY, 0.0f, NAV_KD_XY};
    PID_init(&pid_x, PID_POSITION, k, NAV_VMAX_XY, 0.0f);
    PID_init(&pid_y, PID_POSITION, k, NAV_VMAX_XY, 0.0f);

    /* 3. 软启动斜坡状态 (世界系) */
    float vx_cmd = 0.0f, vy_cmd = 0.0f, w_cmd = 0.0f;
    float prev_eyaw = 0.0f;

    uint32_t t0 = osKernelGetTickCount();
    uint8_t  arrive  = 0u;   /* 连续到达 tick 数 */
    uint8_t  invalid = 0u;   /* 反馈连续无效 tick 数 */

    PoseData_t pose;

    for (;;)
    {
        locator_ops9.get_pose(&pose);   /* 只读, 不调 update (ops9imu 任务在喂) */

        if (!pose.valid)
        {
            invalid++;
            arrive = 0;
            if (invalid > NAV_MAX_INVALID_TICKS)
            {
                vx_cmd = 0.0f; vy_cmd = 0.0f; w_cmd = 0.0f;
                NAV_Stop();
            }
        }
        else
        {
            invalid = 0;

            float ex   = target_x - pose.x;
            float ey   = target_y - pose.y;
            float eyaw = NAV_WrapPi(target_yaw - pose.yaw);

            /* PD 输出 = 世界系期望速度 (m/s / rad/s) */
            float vx_w = PID_calc(&pid_x, pose.x, target_x);
            float vy_w = PID_calc(&pid_y, pose.y, target_y);
            float w_w  = NAV_KP_YAW * eyaw + NAV_KD_YAW * (eyaw - prev_eyaw);
            prev_eyaw = eyaw;
            w_w = NAV_Clamp(w_w, -NAV_VMAX_W, NAV_VMAX_W);

            /* 软启动: 三轴独立加速度斜坡 (缓启动) */
            vx_cmd = NAV_Ramp(vx_cmd, vx_w, NAV_ACC_XY, NAV_DT);
            vy_cmd = NAV_Ramp(vy_cmd, vy_w, NAV_ACC_XY, NAV_DT);
            w_cmd  = NAV_Ramp(w_cmd,  w_w,  NAV_ACC_W,  NAV_DT);

            /* 世界 → 车体 (BollLocator.c:203-207 同式, 用当前 yaw) */
            float c = cosf(pose.yaw), s = sinf(pose.yaw);
            float bvx =  vx_cmd * c + vy_cmd * s;
            float bvy = -vx_cmd * s + vy_cmd * c;

            MecanumResult res = Mecanum_Calc_Full_V(bvx, bvy, w_cmd);
            Send_commandmotor(&res);

            /* 到达: 三轴误差均入容差, 连续 NAV_ARRIVE_TICKS 拍 */
            if (fabsf(ex) <= NAV_TOL_XY && fabsf(ey) <= NAV_TOL_XY &&
                fabsf(eyaw) <= NAV_TOL_YAW)
            {
                if (++arrive >= NAV_ARRIVE_TICKS)
                    break;
            }
            else
            {
                arrive = 0;
            }
        }

        /* 超时: 零速停车, 仅反馈有效时刷新 Self_Dir */
        if ((osKernelGetTickCount() - t0) >= NAV_TIMEOUT_MS)
        {
            NAV_Stop();
            if (pose.valid)
            {
                Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
            }
            return false;
        }

        osDelay(NAV_LOOP_TICKS);   /* 10ms → 100Hz */
    }

    /* 到达: 零速停车 + 刷新 Self_Dir */
    NAV_Stop();
    Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
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

/**
 * @brief  分点导航 —— 按 g_waypoints[] 逐点推进（一次一步）
 *
 * 每次调用只驱动到**一个**路径点, 游标记在函数内的 static 里。
 * "一次一步"是刻意的: worker_task.c 的 NF_Stage_Navigation() 每被流程调到
 * 一次就推进一站, 站点之间流程还能经 NF_DispatchNext() 分发到循迹等其它阶段。
 * 若改成一次走完整张表, 这些站点之间的分发机会就没了。
 *
 * @note 到达判据 / 超时 / 反馈失效处理都在 Nav_GoToWorld() 里, 本函数只做推进。
 * @note 与旧副本 (4eedf6a, 2026-09-14) 的差异: 旧版把第 13 点之后的两点塞在
 *       同一次调用里 (PontIntex == 13 的特判), 这里不再特判 —— 表中 13/14 号
 *       就是普通点, 按顺序一次一个。
 *
 * @warning 游标只在**成功**时推进, 超时的点下次会重试。而调用方
 *          NF_Stage_Navigation() 目前**忽略本函数的返回值** —— 若某个点因
 *          OPS9 离线等原因持续失败, 流程会永远卡在 Navigation 阶段。
 *          真出现这种情况, 需要给这里加失败上限计数, 或让调用方检查返回值。
 *
 * @return true   本点已到达 (或整条路线已走完)
 * @return false  本点超时未到达 (游标不推进, 下次重试)
 */
bool Nav_FeDuanPoint(void)
{
    static uint8_t s_idx = 0u;

    /* 上界取 g_waypoint_count, 但先夹一道 NAV_WAYPOINT_MAX —— 该表是手工维护的,
     * count 写大了会读越界。 */
    uint8_t count = g_waypoint_count;
    if (count > (uint8_t)NAV_WAYPOINT_MAX) {
        count = (uint8_t)NAV_WAYPOINT_MAX;
    }

    if (s_idx >= count) {
        return true;                    /* 路线已走完, 恒真 */
    }

    if (!Nav_GoToWorld(g_waypoints[s_idx].x,
                       g_waypoints[s_idx].y,
                       g_waypoints[s_idx].yaw)) {
        printf("[NAV] 分点导航第 %u 点超时, 游标停在原点待重试\r\n",
               (unsigned)s_idx);
        return false;
    }

    s_idx++;
    return true;
}

bool Nav_MoveBody(float target_x, float target_y, float target_yaw) {

    float targetworld_x=target_x+Self_Dir.x,
    targetworld_y=target_y+Self_Dir.y,
    targetworld_yaw=target_yaw+Self_Dir.yaw;

    /* 返回值此前直接漏写, 函数声明为 bool 却没有 return —— 调用方拿到的
     * 是随机的寄存器残留值 (C11 6.9.1p12, 非 void 函数跑到 } 是 UB)。
     * 这里补上, 语义取 Nav_GoToWorld 的成败。 */
    return Nav_GoToWorld(targetworld_x ,targetworld_y  , targetworld_yaw );


}

/* ==================================================================
 * 流程打桩 (TODO: 待补真实实现)
 *
 * 本函数此前**只有 app/NavigationMecanum.h 的声明, 全工程无定义**,
 * app/worker_task.c 的 NLF_RunFlow() 一调用就报 undefined reference。
 * 按用户决定 (2026-09-28) 先补成打桩, 把流程骨架串通, 真实现以后再填。
 *
 * (同批打桩的 Nav_FeDuanPoint() 已补成真实实现, 见上面 Nav_RunWaypoints 之后。)
 * ================================================================== */

/**
 * @brief  [打桩] 循迹完成后按实测位置校准点位。
 * @param  is_trophy true=奖杯循迹(LinFolR)校准亚军点, false=物料循迹(LinFolL)校准 a 点
 * @note   真实实现应在循迹结束后用当前 OPS9 位姿回写 g_tt.pos[] / 路径点表;
 *         现版本只记日志。可以参照 app/ColorIdentif.c 的 SetPos()。
 * @warning 打桩期间点位**不会被校准**, 循迹段结束后的定位误差不会修正。
 */
void Nav_CalibrateAfterTrace(bool is_trophy)
{
    printf("[NAV-STUB] Nav_CalibrateAfterTrace(%s) called, no-op\r\n",
           is_trophy ? "trophy" : "material");
}


