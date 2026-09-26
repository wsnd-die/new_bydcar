/**
 * @file    pose_data.h
 * @brief   全局位姿统一数据结构 --- 抽象设备层 (Device)
 *
 * 规范来源：《机器人嵌入式代码架构规范与修改日志》V1.0 第 2 节。
 *
 * @note    本文件为【目标架构】定义。当前工程实际在用的位姿结构是
 *          app/NavigationMecanum.h 中的 World_Dir_t（仅 x / y / yaw），
 *          两者尚未统一。迁移方案见 CLAUDE.md 第 2 节。
 *
 * @warning 禁止修改已有字段的顺序与类型；
 *          新增字段只能追加到结构体末尾，以保证前向兼容。
 *
 * @note    V1.11.0 在末尾追加了 ax / ay / az（IMU 三轴线加速度）。
 *          这三个字段目前只有 imu_hwt906（device/HWT906.c）会填；
 *          locator_wheel / locator_ops9 没有加速度数据源，恒为 0。
 */

#ifndef _POSE_DATA_H_
#define _POSE_DATA_H_

#include <stdint.h>

/**
 * @brief 全局位姿数据结构体
 * @note 所有定位源输出、融合算法输入输出必须使用此结构
 * @strong 禁止修改字段顺序，新增字段只能追加到末尾
 *
 * 坐标系约定（沿用现有工程约定，见 app/NavigationMecanum.h）：
 *   X 轴：前方（车头朝向 0° 时正对的方向）
 *   Y 轴：左方（右手系，Z 轴向上）
 *   yaw：世界航向角，弧度制，0 = 正对 X 轴，CCW 为正，范围 [-π, π]
 */
typedef struct {
    float x;          /* 全局X坐标，单位：m */
    float y;          /* 全局Y坐标，单位：m */
    float yaw;        /* 航向角，单位：rad */
    float pitch;      /* 俯仰角，单位：rad */
    float roll;       /* 横滚角，单位：rad */

    float vx;         /* X轴线速度，单位：m/s */
    float vy;         /* Y轴线速度，单位：m/s */
    float wz;         /* 航向角速度，单位：rad/s */

    uint8_t  valid;     /* 数据有效性标志：0-无效，1-有效 */
    uint32_t timestamp; /* 数据时间戳，单位：ms */

    /* ---- 以下为 V1.11.0 追加，前向兼容 ---- */
    float ax;         /* X轴线加速度，单位：m/s^2，传感器本体坐标系 */
    float ay;         /* Y轴线加速度，单位：m/s^2 */
    float az;         /* Z轴线加速度，单位：m/s^2 */
} PoseData_t;

#endif /* _POSE_DATA_H_ */