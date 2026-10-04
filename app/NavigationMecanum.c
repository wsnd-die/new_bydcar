
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


World_Dir_t g_waypoints[NAV_WAYPOINT_MAX] = {
    {   0.38503f,-1.20312f,-0.76168f }, /*  1 奖杯点 */
    {   0.71070f,-1.46381f,-0.26075f }, /*  2 奖杯点 */
    {   1.12916f,-1.51226f,0.14742f }, /*  3 奖杯点 */
    {   1.72208f,-0.25822f,-0.00016f }, /*  4 亚军点 */
    {   1.72903f,0.00858f,-0.01333f }, /*  5 冠军点 */
    {   1.72384f,0.25749f,0.03638f }, /*  6 季军点 */

    {   1.92988f,0.74853f,1.78417f },   /*  7 e 点 */
    {   1.72166f,1.14528f,2.19026f },   /*  8 c 点 */
    {   1.37402f, 1.41876f, 2.74400f }, /*  9 d 点 */
    {   0.92016f, 1.46772f,-3.03864f }, /* 10 a 点 */
    {   0.49178f, 1.31843f,-2.47222f }, /* 11 b 点 */

    {   0.98947f,-0.46437f,-1.58871f }, /* 12 摆放e 点 */
    {   0.92926f, 0.02798f,-1.63934f }, /* 13 摆放d 点 */
    {   0.92926f, 0.02798f,-0.81427f }, /* 14 摆放c 点 */
    {   0.80875f, 0.62927f,0.0f }, /* 15 摆放a 点 */
    {   0.50875f, 0.47927f,0.0f }, /* 16 摆放b 点 */

    {   0.0f,  0.0f, 0.0f }, /* 17 回家点 */
};

uint8_t g_waypoint_count = 17u;

/* ==================================================================
 * V1.23.0: 路线段打断
 *
 * `g_route_abort` (定义在 worker_task.c, 由 gripper_task 的 IR 进料置位)
 * 让 Nav_GoToWorld() 能中途放弃当前目标点。返回值仍是 bool ——
 * 被打算 = false, 靠 Nav_LastAborted() 区分"超时"和"被打断"。
 * ================================================================== */
static bool s_nav_aborted = false;

bool Nav_LastAborted(void)
{
    return s_nav_aborted;
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
 * @brief 平移轴参考速度 —— 近场线性 P, 远场用制动曲线当上限。
 *
 *      v_ref = clamp(Kp·e, ±√(2·a·|e|), ±v_max)
 *
 * **近场** (|e| < 2a/Kp²): v = Kp·e —— 线性、过零连续、增益有限, 跟原来的纯 P
 * 一样温和。
 *
 * **远场** (|e| > 2a/Kp²): 制动曲线成为上限。|v| ≤ √(2·a·|e|) 就是"此刻还
 * 刹得住", 高速接近时不会冲过头 —— 这是纯 P 原本缺的那一环。
 *
 * @warning **曲线只能当上限, 不能当参考**(V1.24.1 修正)。上一版直接拿
 *          √(2·a·e) 做参考, 有两个致命毛病: ①等效增益在 e→0 时发散
 *          (d√e/de → ∞); ②死区边界是"0 → √(2·a·deadband)"的**阶跃**。
 *          位置噪声一到, 指令就在 0 和 0.2 m/s 之间以控制频率来回跳 ——
 *          表现就是**跑到点位来回晃, 比纯 P 还差**。
 *          (原来的死区也一并删了: 线性项在 e=0 处连续到 0, 没有边界可抖。)
 *
 * @param err    位置误差 (m), 带符号
 * @param v_max  速度上限 m/s
 * @param a_brk  制动曲线减速度 m/s² (**只当上限**; 必须 ≤ 斜坡能给的减速度)
 * @param kp     近场比例增益 1/s。调大→尾巴更短; 抖/过冲就往下调
 * @retval 参考速度 m/s (世界系)
 */
static float NAV_AxisRef(float err, float v_max, float a_brk, float kp,char choice)
{
    if (choice=='x')
    {
        float x_v  = kp * err;                          /* 近场参考 (线性, 过零连续) */
        float x_vc = sqrtf(2.0f * a_brk * fabsf(err));  /* 远场上限 (此刻还刹得住) */

        if (x_v >  x_vc) x_v =  x_vc;
        if (x_v < -x_vc) x_v = -x_vc;

        return NAV_Clamp(x_v, -v_max, v_max);
    }
    if (choice == 'y')
    {
        float y_v  = kp * err;                          /* 近场参考 (线性, 过零连续) */
        float y_vc = sqrtf(2.0f * a_brk * fabsf(err));  /* 远场上限 (此刻还刹得住) */

        if (y_v >  y_vc) y_v =  y_vc;
        if (y_v < -y_vc) y_v = -y_vc;

        return NAV_Clamp(y_v, -v_max, v_max);
    }
}
#endif /* NAV_XY_PROFILE */

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
    /* 0. 清掉上一次的打断标记 —— 本次的结果只反映本次 */
    s_nav_aborted = false;

    /* 1. 夺回电机控制权: 按契约关角度环, 等 FC_TASK 下降沿零速
     *    (worker_task.c:28-35) */
    g_angle_ctrl_enable = 0;
    osDelay(20);

    /* 2. x/y 轴默认走制动曲线规划 (NAV_XY_PROFILE=1), 不再用 PID_calc。
     *    yaw 本来就不用它: PID_calc 内部误差不 wrap, 跨 ±π 会跳 2π, 手写。
     *    NAV_XY_PROFILE=0 时退回旧的位置 PD, 方便 A/B 对比。 */
#if !NAV_XY_PROFILE
    pid_type_def pid_x, pid_y;
    fp32 k_x[3] = {2.3f, 0.0f, 0.91f};
    PID_init(&pid_x, PID_POSITION, k_x, NAV_VMAX_XY, 0.0f);
    fp32 k_y[3] = {1.3f, 0.0f, 0.91f};
    PID_init(&pid_y, PID_POSITION, k_y, NAV_VMAX_XY, 0.0f);
#endif

    /* 3. 软启动斜坡状态 (世界系) */
    float vx_cmd = 0.0f, vy_cmd = 0.0f;

    uint32_t t0 = osKernelGetTickCount();
    uint8_t  arrive  = 0u;   /* 连续到达 tick 数 */
    uint8_t  invalid = 0u;   /* 反馈连续无效 tick 数 */

    /* ── 调参仪表 (只在出循环时打一行, 不占循环时间) ──
     * ovs_x / ovs_y: 本段**过冲量** (m), 0 = 没冲过目标点 —— 这是调
     *   NAV_BRK_XY / NAV_KP_XY_LIN 时要盯的唯一的客观量。
     *   算法: 记第一拍误差的符号 s0, 之后每拍算 -s0·e 取最大。误差从不反号
     *   时该值恒负(不过冲); 一旦冲过目标点就变正, 数值就是冲出去多远。
     * last_*: 超时分支看不到作用域里的 ex/ey/eyaw (它们在循环内层声明),
     *   所以每拍存一份, 让超时那行也能打出"差了多少"。 */
    uint8_t s0_set = 0u;
    float   s0x = 0.0f, s0y = 0.0f;
    float   ovs_x = 0.0f, ovs_y = 0.0f;
    float   last_ex = 0.0f, last_ey = 0.0f, last_eyaw = 0.0f;

    PoseData_t pose;

    for (;;)
    {
        locator_ops9.get_pose(&pose);   /* 只读, 不调 update (ops9imu 任务在喂) */

        /* V1.23.0 打断: 外部 (IR 进料) 要求立刻放弃当前这一段。
         * 消费掉这次请求后按"未到达"返回, 由 Nav_LastAborted() 区分超时。 */
        if (g_route_abort)
        {
            g_route_abort = 0u;
            NAV_Stop();
            if (pose.valid)
            {
                Self_Dir.x = pose.x; Self_Dir.y = pose.y; Self_Dir.yaw = pose.yaw;
            }
            s_nav_aborted = true;
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

            float ex   = target_x - pose.x;
            float ey   = target_y - pose.y;
            float eyaw = NAV_WrapPi(target_yaw - pose.yaw);

            /* 调参仪表: 累计过冲量 + 末态 (见函数开头的说明) */
            if (!s0_set) {
                s0x = (ex >= 0.0f) ? 1.0f : -1.0f;
                s0y = (ey >= 0.0f) ? 1.0f : -1.0f;
                s0_set = 1u;
            }
            if (-s0x * ex > ovs_x) ovs_x = -s0x * ex;
            if (-s0y * ey > ovs_y) ovs_y = -s0y * ey;
            last_ex = ex; last_ey = ey; last_eyaw = eyaw;


            float eyaw_dz = (fabsf(eyaw) < NAV_YAW_DEADBAND) ? 0.0f : eyaw;
            float wz = (fabsf(pose.wz) < NAV_YAW_DEADBAND) ? 0.0f : pose.wz;
            /* 航向: 手写 PD (同前), 不受 NAV_XY_PROFILE 影响 */
            float w_w = NAV_KP_YAW * eyaw_dz - NAV_KD_YAW * PID_Filter(0.25,wz);
            w_w = NAV_Clamp(w_w, -NAV_VMAX_W, NAV_VMAX_W);

#if NAV_XY_PROFILE

            float vx_ref = NAV_AxisRef(ex, NAV_VMAX_X, NAV_BRK_X, NAV_KP_X_LIN,'x');
            float vy_ref = NAV_AxisRef(ey, NAV_VMAX_Y, NAV_BRK_Y, NAV_KP_Y_LIN,'y');
#else
            float vx_ref = PID_calc(&pid_x, pose.x, target_x);
            float vy_ref = PID_calc(&pid_y, pose.y, target_y);
#endif

            vx_cmd = NAV_Ramp(vx_cmd, vx_ref, NAV_ACC_XY, NAV_DT);
            vy_cmd = NAV_Ramp(vy_cmd, vy_ref, NAV_ACC_XY, NAV_DT);

            /* 世界 → 车体 (BollLocator.c:203-207 同式, 用当前 yaw) */
            float c = cosf(pose.yaw), s = sinf(pose.yaw);
            float bvx =  vx_cmd * c + vy_cmd * s;
            float bvy = -vx_cmd * s + vy_cmd * c;

            MecanumResult res = Mecanum_Calc_Full_V(bvx, bvy, w_w);
            Send_commandmotor(&res);

            /* 到达: 三轴误差均入容差 **且指令速度已收下来**, 连续 NAV_ARRIVE_TICKS 拍。
             * 速度门限是必要的: 曲线规划会带着 ~0.3 m/s 穿过容差区, 只看位置就会在
             * 还在跑的时候判"到了", 之后的滑行把车带出容差。 */
            if (fabsf(ex) <= NAV_TOL_XY && fabsf(ey) <= NAV_TOL_XY &&
                fabsf(eyaw) <= NAV_TOL_YAW &&
                fabsf(vx_cmd) <= NAV_ARRIVE_VMAX && fabsf(vy_cmd) <= NAV_ARRIVE_VMAX)
            {
                if (++arrive >= NAV_ARRIVE_TICKS)
                {
                    printf("[NAV] ARRIVE t=%ums ex=%.3f ey=%.3f eyaw=%.3f vx=%.2f vy=%.2f | ovs=%.3f/%.3f\r\n",
                           (unsigned)(osKernelGetTickCount() - t0),
                           ex, ey, eyaw, vx_cmd, vy_cmd, ovs_x, ovs_y);
                    break;
                }
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
            printf("[NAV] TIMEOUT t=%ums ex=%.3f ey=%.3f eyaw=%.3f valid=%u vx=%.2f vy=%.2f\r\n",
                   (unsigned)(osKernelGetTickCount() - t0),
                   last_ex, last_ey, last_eyaw, (unsigned)pose.valid, vx_cmd, vy_cmd);
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
        if (Nav_LastAborted()) {
            printf("[NAV] 第 %u 点被 IR 打断, 游标 -> %u\r\n",
                   (unsigned)s_idx, (unsigned)(s_idx + 1u));
            s_idx++;
            return true;
        }
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


