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
 * @brief 导航到目标世界坐标
 *
 * 根据当前位姿 Self_Dir 与目标位姿的差值，
 * 调用 Mecanum_CalculateWorldMove 完成平移 + 旋转的同步规划，
 * 然后执行并等待动作完成，最后更新 Self_Dir。
 *
 * @param target_x    目标世界 X 坐标，单位：m
 * @param target_y    目标世界 Y 坐标，单位：m
 * @param target_yaw  目标世界航向角，单位：rad
 * @return true       已到达或运动已执行
 * @return false      解算失败或执行失败
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