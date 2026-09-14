/**
 * @file    angle_ctrl.h
 * @brief   纯角度控制: 角度环 → 角速度环 (串级 PID)
 *          仅维持目标角度，不做位置角度控制
 */
#ifndef ANGLE_CTRL_H
#define ANGLE_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

typedef enum {
    ANGLE_IDLE    = 0,
    ANGLE_MOVING  = 1,
    ANGLE_ARRIVED = 2
} AngleState;

typedef struct {
    AngleState state;

    /* 目标 */
    float target_yaw;     /* deg, 目标朝向 */

    /* ---- 角度环 (中层) ---- */
    pid_type_def pid_angle;  /* yaw_err(deg) → target_w(deg/s) */

    /* ---- 角速度环 (内层) ---- */
    pid_type_def pid_w;   /* w_err(deg/s) → cmd_w(rad/s) */

    /* 限幅 */
    float max_w;          /* rad/s */

    /* 阈值 */
    float yaw_tol;        /* deg */

    /* ---- 陀螺仪滤波 ---- */
    float gyro_alpha;      /* 低通系数: 0~1, 越小滤波越强 */
    float gyro_deadband;   /* 死区 deg/s, 低于此值置零 */
    float gyro_scale;      /* 缩放系数: 0~1 */
    float gyro_filt;       /* 滤波后的角速度 deg/s */

    /* 输出 */
    float cmd_w;

    /* 诊断 */
    float yaw_err, target_w;
} AngleCtrl;

/* 角度环 + 角速度环 */
void AngleLoop_Update(AngleCtrl *ac,
                    float cur_yaw, float cur_w);

/* 整体 */
void  Angle_Init        (AngleCtrl *ac);
void  Angle_SetTarget   (AngleCtrl *ac, float yaw);
void  Angle_UpdateTarget(AngleCtrl *ac, float yaw);  /* 仅更新目标，不重置PID */
void  Angle_Update      (AngleCtrl *ac,
                       float cur_yaw, float cur_w);
bool  Angle_Arrived     (const AngleCtrl *ac);

#endif
