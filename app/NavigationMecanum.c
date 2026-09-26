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
 *          → Mecanum_Vel_Execute() 下发电机速度环 (Emm_V5_Vel_Control)
 *
 *  运行环境: NLF_TASK (阻塞式流程任务), 100Hz (osDelay(10))。
 *  与 FC_TASK 的电机控制权契约见 worker_task.c 文件头:
 *  入口先置 g_angle_ctrl_enable = 0 并 osDelay(20), 等 FC_TASK 下降沿零速,
 *  此后本文件独占电机命令。
 *
 *  反馈源: locator_ops9 (device/ops9_g491_uart3.c), 世界系 x/y/yaw,
 *  单位 m / rad。update() 由 ops9imu_fuction 任务每 ~6ms 调一次,
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

/* ==================================================================
 * 全局量 (唯一定义处, extern 声明在 NavigationMecanum.h)
 * ================================================================== */

World_Dir_t Self_Dir = {0.0f, 0.0f, 0.0f};

World_Dir_t g_waypoints[NAV_WAYPOINT_MAX];
uint8_t     g_waypoint_count = 0;

/* ==================================================================
 * 静态工具
 * ================================================================== */

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
    Mecanum_Vel_Execute(&z);
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
            /* 反馈无效: 短时 (≤ NAV_MAX_INVALID_TICKS) 冻结指令继续跑,
             * 超限则零速保持并清斜坡, 恢复后从零重新软启动。
             * 当前 OPS9 UART3 接收链未修时 valid 恒 0, 本路径是稳态:
             * 零速保持 → 超时返回 false, 全程不动车, 不误驱动。 */
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
            Mecanum_Vel_Execute(&res);

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
