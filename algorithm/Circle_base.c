/**
 * @file  Circle_base.c
 * @brief 找圆跟随控制实现
 *        K230 圆检测方向 → Mecanum_Calc_Full 全向移动 → 电机指令
 */

#include "Common_used.h"
#include "Circle_base.h"
#include "mecanum.h"
#include "Send_motor.h"
#include "k230.h"
/* ======================== 全局状态 ======================== */

float g_circle_vx = 0.0f;
float g_circle_vy = 0.0f;
char  g_circle_dir = '?';
float g_circle_speed = 1.0f;  /* 速度系数: 1.0=快(左侧), 0.4=慢(右侧) */

/* 按 xy 圆心偏差分档调速: 远(>20px)快速接近, 近(≤20px)慢速微调 */
#define CIRCLE_XY_FAST_TH   10.0f   /* 偏差阈值 (像素) */
#define CIRCLE_XY_V_FAST    0.15f   /* 远距离速度 (m/s) */
#define CIRCLE_XY_V_SLOW    0.02f   /* 近距离速度 (m/s) */

/* 找圆稳定: 连续 'O' 确认次数, 达到才认为已居中, 防止方向抖动误判/一直动 */
#define CIRCLE_O_STABLE_CNT  10U
static uint8_t s_o_cnt = 0U;
static float s_sum_x = 0.0f;   /* 确认期间圆心偏差累加 */
static float s_sum_y = 0.0f;
static char  s_last_dir = '?'; /* 上次方向: 摄像头每3帧才发一次, 无新帧时保持, 避免垃圾值 */

float g_circle_avg_x = 0.0f;   /* 稳定确认后的平均偏差 (供放置用, 更稳) */
float g_circle_avg_y = 0.0f;

/* ======================== 找圆跟随主函数 ======================== */

void Circle_Follow(void)
{
    char dir = s_last_dir;   /* 默认保持上次方向 */
    MecanumResult motor;
    float cx = 0.0f, cy = 0.0f;

    /* ---- 1. 获取圆方向 (无新帧时保持上次, 不重置计数) ---- */
    if (NX_GetCircleDir(&dir)) {
        s_last_dir = dir;
    }

    /* ---- 1.4 读圆心偏差 → 分档调速 (远>20px快, 近≤20px慢) ---- */
    NX_GetCirclepos(&cx, &cy);
    {
        float dist = sqrtf(cx*cx + cy*cy);
        g_circle_speed = (dist > CIRCLE_XY_FAST_TH) ? CIRCLE_XY_V_FAST : CIRCLE_XY_V_SLOW;
    }

    /* ---- 1.5 稳定确认: 连续 N 次 'O' 才确认居中 ----
     * 确认期间 g_circle_dir 置 ' '(不触发放置), 车保持静止,
     * 避免 'O' 抖动导致车一会停一会动 / 误判提前放置 */
    if (dir == 'O') {
        s_sum_x += cx;
        s_sum_y += cy;
        if (s_o_cnt < CIRCLE_O_STABLE_CNT) s_o_cnt++;
        if (s_o_cnt >= CIRCLE_O_STABLE_CNT) {
            g_circle_dir = 'O';
            g_circle_avg_x = s_sum_x / (float)CIRCLE_O_STABLE_CNT;   /* 平均偏差 */
            g_circle_avg_y = s_sum_y / (float)CIRCLE_O_STABLE_CNT;
        } else {
            g_circle_dir = ' ';
        }
    } else {
        s_o_cnt = 0;
        s_sum_x = 0.0f;
        s_sum_y = 0.0f;
        g_circle_dir = dir;
    }

    /* ---- 2. 方向 → 速度映射 (速度按 xy 距离分档) ---- */
    switch (dir) {
    case 'O':
        g_circle_vx = 0.0f;
        g_circle_vy = 0.0f;
        break;

    case 'N':
        /* 圆心偏上: 前进 */
        g_circle_vx = -g_circle_speed;
        g_circle_vy = 0.0f;
        break;

    case 'S':
        /* 圆心偏下: 后退 */
        g_circle_vx = g_circle_speed;
        g_circle_vy = 0.0f;
        break;

    case 'W':
        /* 圆心偏右: 右移 */
        g_circle_vx = 0.0f;
        g_circle_vy = -g_circle_speed;
        break;

    case 'E':
        /* 圆心偏左: 左移 */
        g_circle_vx = 0.0f;
        g_circle_vy = g_circle_speed;
        break;

    default:
        /* 未知方向: 停止 */
        break;
    }

    /* ---- 3. 全向移动解算 (vx, vy, w=0) ---- */
    motor = Mecanum_Calc_Full_V(g_circle_vx, g_circle_vy, 0.0f);

    /* ---- 4. 发送电机指令 ---- */
    Send_commandmotor(&motor);
}
