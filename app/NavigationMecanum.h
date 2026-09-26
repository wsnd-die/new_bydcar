#ifndef __NAVIGATION_MECANUM_H
#define __NAVIGATION_MECANUM_H

#include <stdbool.h>
#include <stdint.h>
#include "Mecanum_Move.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 类型定义
 * ============================================================ */

/*
 * 世界坐标系约定：
 *   X 轴：前方（车头朝向 0° 时正对的方向）
 *   Y 轴：左方（右手系，Z 轴向上）
 *   yaw：世界航向角，弧度制，0 = 正对 X 轴，CCW 为正
 */
typedef struct {
    float x;       /* 世界 X 坐标，单位：m */
    float y;       /* 世界 Y 坐标，单位：m */
    float yaw;     /* 世界航向角，单位：rad，范围 [-π, π] */
} World_Dir_t;

/* 当前自身位姿（世界坐标系） */
extern World_Dir_t Self_Dir;

/* ============================================================
 * 路径点
 * ============================================================ */

/* 最大路径点数量 */
#define NAV_WAYPOINT_MAX  32

/* ============================================================
 * 世界系位置闭环参数 (100Hz, OPS9 反馈)
 * 全部为初版整定值, 上机按实际响应调, 改完记 clauderecord。
 * ============================================================ */
#define NAV_DT                 0.01f   /* 名义控制周期 s (执行器内 osDelay(5) 使实际 ~15ms) */
#define NAV_LOOP_TICKS         10u     /* osDelay(10) → 名义 100Hz */
#define NAV_TIMEOUT_MS         10000u  /* 单点超时 ms */
#define NAV_KP_XY              1.0f    /* 平移 P: 0.4m 误差 → 0.4 m/s */
#define NAV_KD_XY              0.0f    /* 平移 D: 首版关 (OPS9 噪声放大风险) */
#define NAV_KP_YAW             2.0f    /* 航向 P: 0.5rad 误差 → 1 rad/s */
#define NAV_KD_YAW             0.0f    /* 航向 D: 首版关 */
#define NAV_VMAX_XY            0.4f    /* 平移速度限幅 m/s */
#define NAV_VMAX_W             1.0f    /* 角速度限幅 rad/s */
#define NAV_ACC_XY             0.3f    /* 平移加速度 m/s² (软启动) */
#define NAV_ACC_W              1.0f    /* 角加速度 rad/s² (软启动) */
#define NAV_TOL_XY             0.03f   /* 到达容差 3cm */
#define NAV_TOL_YAW            0.05f   /* 到达容差 ~2.9° */
#define NAV_ARRIVE_TICKS       5u      /* 连续 5 拍判到达 (抗单帧抖动) */
#define NAV_MAX_INVALID_TICKS  20u     /* OPS9 离线容忍 0.2s, 超限零速保持 */

/*
 * 路径点数组（世界坐标系）
 * 每个元素: { X(m), Y(m), yaw(rad) }
 * yaw 可使用 MECANUM_DEG_TO_RAD 辅助书写，例如 90.0f * MECANUM_DEG_TO_RAD
 */
extern World_Dir_t g_waypoints[NAV_WAYPOINT_MAX];
extern uint8_t      g_waypoint_count;

/* ============================================================
 * 函数声明
 * ============================================================ */

void Chassis_WorldMoveTest(void);

/**
 * @brief 导航到目标世界坐标（世界系位置闭环）
 *
 * 以 OPS9 位姿 (locator_ops9.get_pose, 世界系 x/y/yaw) 为反馈，
 * 三轴并行 PD (Ki=0) 输出世界系期望速度 → 软启动加速度斜坡 →
 * 世界→车体旋转 → Mecanum_Calc_Full_V → Mecanum_Vel_Execute
 * 下发电机速度环。阻塞直到到达或超时，退出时零速停车。
 *
 * @note 运行于 NLF_TASK 上下文；入口按契约关角度环
 *       (g_angle_ctrl_enable=0 + osDelay(20)) 独占电机控制权。
 *       反馈无效按 NAV_MAX_INVALID_TICKS 容忍，超限零速保持。
 * @param target_x    目标世界 X 坐标，单位：m
 * @param target_y    目标世界 Y 坐标，单位：m
 * @param target_yaw  目标世界航向角，单位：rad
 * @return true       已到达（连续 NAV_ARRIVE_TICKS 拍误差入容差）
 * @return false      超时（NAV_TIMEOUT_MS，含反馈长期无效）
 */
bool Nav_GoToWorld(float target_x, float target_y, float target_yaw);
bool Nav_FeDuanPoint(void);

/**
 * @brief 循迹完成后按实测位置校准 a 点 / 亚军点
 *
 * @param is_trophy true=奖杯循迹(LinFolR)校准亚军点, false=物料循迹(LinFolL)校准a点
 */
void Nav_CalibrateAfterTrace(bool is_trophy);
/**
 * @brief 依次执行所有路径点
 *
 * 从 g_waypoints[0] 到 g_waypoints[g_waypoint_count-1]，
 * 逐点调用 Nav_GoToWorld。阻塞直到全部完成或中途失败。
 *
 * @return true  全部路径点执行成功
 * @return false 中途某点执行失败
 */
bool Nav_RunWaypoints(void);

/* ============================================================
 * 车体坐标运动 (Body-frame)
 * ============================================================ */

/**
 * @brief 车体坐标运动 — 前进/左移/旋转，阻塞直到到位或超时
 *
 * 坐标系: forward(+) 朝车头, left(+) 朝车体左方, rotate(+) CCW
 * 内部调用 Mecanum_MoveWithEncoder 执行，并自动更新 Self_Dir。
 *
 * @param forward_m  前进距离 (m)，负值=后退
 * @param left_m     左移距离 (m)，负值=右移
 * @param rotate_rad 旋转角度 (rad)，正值=CCW
 * @return true      运动执行成功
 * @return false     解算或执行失败
 */
bool Nav_MoveBody(float forward_m, float left_m, float rotate_rad);

/** @brief 前进/后退 (m)，正=前进，负=后退 */
bool Nav_MoveForward(float distance_m);

/** @brief 左移/右移 (m)，正=左移，负=右移 */
bool Nav_MoveLeft(float distance_m);

/** @brief 原地旋转 (rad)，正=CCW */
bool Nav_Rotate(float angle_rad);

/** @brief 速度模式到位控制(世界系目标), 编码器+陀螺仪定位 */
bool Nav_TrackPose(float tx, float ty, float tyaw);


#ifdef __cplusplus
}
#endif

#endif /* __NAVIGATION_MECANUM_H */