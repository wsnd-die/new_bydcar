/**
 * @file  Circle_base.h
 * @brief 找圆跟随 —— 按 K230 圆心偏差控制麦轮底盘移动
 *
 *  'O' = 居中 (停止/抓取), 'W' = 偏右, 'E' = 偏左, 'N' = 偏上, 'S' = 偏下
 */

#ifndef __CIRCLE_BASE_H
#define __CIRCLE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>



/* ======================== 函数声明 ======================== */

void Circle_Follow(void);

extern bool flag_finish;
extern float g_circle_k;      /* 像素偏差 → 线速度的线性系数。
                               * **不要从外部赋值** —— Circle_Follow() 每拍重算 */
extern float g_circle_vx;
extern float g_circle_vy;
extern char  g_circle_dir;
extern float g_circle_avg_x;   /* 稳定确认后的平均圆心偏差 (供放置) */
extern float g_circle_avg_y;

#ifdef __cplusplus
}
#endif

#endif /* __CIRCLE_BASE_H */