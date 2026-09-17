/**
 * @file  Circle_base.h
 * @brief 找圆跟随控制模块
 *        根据 K230 圆检测的方向结果，控制麦轮底盘向圆移动
 *
 * 方向定义 (K230 发送):
 *   'O' = 圆心在中心区域  → 停止/抓取
 *   'W' = 圆心偏右        → 右移
 *   'E' = 圆心偏左        → 左移
 *   'N' = 圆心偏上        → 前进
 *   'S' = 圆心偏下        → 后退
 */

#ifndef __CIRCLE_BASE_H
#define __CIRCLE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 找圆控制参数 ======================== */
#define CIRCLE_SPEED_V     0.05f    /* 寻圆线速度 (m/s) */
#define CIRCLE_SPEED_VY    0.05f    /* 寻圆横移速度 (m/s) */
#define CIRCLE_STOP_TIME   10U    /* 圆心对准后停车时间 (ms) */

/* ======================== 函数声明 ======================== */

void Circle_Follow(void);

extern bool flag_finish;
extern float g_circle_speed;  /* 找圆速度系数 */
extern float g_circle_vx;
extern float g_circle_vy;
extern char  g_circle_dir;
extern float g_circle_avg_x;   /* 稳定确认后的平均圆心偏差 (供放置) */
extern float g_circle_avg_y;

#ifdef __cplusplus
}
#endif

#endif /* __CIRCLE_BASE_H */