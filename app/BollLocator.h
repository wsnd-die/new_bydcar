//
// Created by ZHUHUI on 2026/8/6.
//
// BollLocator — 基于 TB_position 定位器的位置闭环控制器
//
// 闭环流程:
//   TB_position (当前世界坐标 mm) → 位置误差 → 速度-时间预测融合
//   → 梯形速度规划 → cmd_vx/cmd_vy
//   → Mecanum_Calc_Full() → Send_commandmotor() → Emm_V5_Vel_Control
//
// 特性:
//   - 缓启 (加速度限制 ramp-up)
//   - 缓停 (根据融合距离动态减速)
//   - v·dt 预测剩余距离，与 TB_position 测量互补滤波
//   - 支持前进 / 横移 / 斜移
//

#ifndef STM32G4_TEST_BOLLLOCATOR_H
#define STM32G4_TEST_BOLLLOCATOR_H

#include "Common_used.h"

/* ============================================================
 * 默认参数
 * ============================================================ */
#define BL_DEFAULT_MAX_SPEED    0.5f    /* 最大速度 m/s */
#define BL_DEFAULT_ACCEL        0.3f    /* 加速度 m/s² */
#define BL_DEFAULT_DECEL        0.4f    /* 减速度 m/s² */
#define BL_DEFAULT_POS_TOL      8.0f    /* 到达阈值 mm */
#define BL_DT                   0.01f   /* 控制周期 s (100Hz) */

/* ============================================================
 * 控制器结构体
 * ============================================================ */
typedef struct {
    /* ---- 目标 (世界坐标 mm) ---- */
    float target_x;
    float target_y;

    /* ---- 速度参数 ---- */
    float max_speed;     /* 最大线速度 m/s */
    float accel;         /* 加速度 m/s² */
    float decel;         /* 减速度 m/s² */
    float pos_tol;       /* 位置到达阈值 mm */

    /* ---- 预测融合参数 ---- */
    float fuse_alpha;    /* 互补滤波系数 (0~1):
                            1.0=纯测量, 0.0=纯预测
                            推荐 0.7~0.85: 以测量为主, 预测提速 */

    /* ---- 内部状态 ---- */
    bool  active;        /* 是否激活 */
    float cur_speed;     /* 当前指令速度 m/s */
    float last_tb_x;     /* 上次 TB_position.xdata (mm), 检测更新 */
    float last_tb_y;     /* 上次 TB_position.ydata (mm) */

    /* ---- 诊断 ---- */
    float dist_meas;     /* 测量剩余距离 mm (来自 TB_position) */
    float dist_fused;    /* 融合剩余距离 mm (滤波后) */
    float dist;          /* 最终使用距离 mm */
    float err_x;         /* 世界 X 偏差 mm */
    float err_y;         /* 世界 Y 偏差 mm */
    float travel_est;    /* 本周期预估行程 mm */

    /* ---- 输出 (车体坐标: vx=前进, vy=左移) ---- */
    float cmd_vx;        /* m/s */
    float cmd_vy;        /* m/s */
} BollLocator;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化控制器为默认参数
 */
void BL_Init(BollLocator *bl);

/**
 * @brief 设置目标点 (世界坐标 mm) 并启动位置环
 * @param x_mm  目标 X 坐标 (mm), 与 TB_position.xdata 同坐标系
 * @param y_mm  目标 Y 坐标 (mm), 与 TB_position.ydata 同坐标系
 */
void BL_SetTarget(BollLocator *bl, float x_mm, float y_mm);

/**
 * @brief 位置环主更新 — 在任务循环中以 100Hz 调用
 *
 * 内部流程:
 *   1. 读取 TB_position 测量距离
 *   2. v·dt 预测本周期行程, 互补滤波融合剩余距离
 *   3. 梯形速度规划 (缓启/缓停)
 *   4. 世界→车体坐标变换 + 逆运动学 + 发送电机命令
 */
void BL_Update(BollLocator *bl);

/**
 * @brief 是否已到达目标
 */
bool BL_Arrived(const BollLocator *bl);

/**
 * @brief 阻塞式移动到目标点: 设目标 → 等到达 → 停止
 * @param x_mm  目标 X 坐标 (mm)
 * @param y_mm  目标 Y 坐标 (mm)
 * @param timeout_ms  超时时间 (ms), 0 = 不限
 * @return true=到达, false=超时
 */
bool BL_MoveTo(BollLocator *bl, float x_mm, float y_mm, uint32_t timeout_ms);

/**
 * @brief 立即停止并退出位置环
 */
void BL_Stop(BollLocator *bl);

#endif //STM32G4_TEST_BOLLLOCATOR_H