/**
 * @file  Circle_base.c
 * @brief 找圆跟随控制实现
 *        K230 圆检测方向 → Mecanum_Calc_Full 全向移动 → 电机指令
 */

#include "Common_used.h"
#include "Circle_base.h"
#include "mecanum.h"
#include "Send_motor.h"
#include "NX_uart.h"
/* ======================== 全局状态 ======================== */

float g_circle_vx = 0.0f;
float g_circle_vy = 0.0f;
char  g_circle_dir = '?';

float g_circle_k = 0.0f;


#define CIRCLE_XY_FAST_TH   23.0f   /* 偏差阈值 (像素) */
#define CIRCLE_XY_V_FAST    0.003f   /* 系数: 偏差 > 10px 时 (m/s per px) */
#define CIRCLE_XY_V_SLOW    0.0012f   /* 系数: 偏差 ≤ 10px 时 (m/s per px) */

/* 找圆稳定: 连续 'O' 确认次数, 达到才认为已居中, 防止方向抖动误判/一直动 */
#define CIRCLE_O_STABLE_CNT  30U
static uint8_t s_o_cnt = 0U;
static float s_sum_x = 0.0f;   /* 确认期间圆心偏差累加 */
static float s_sum_y = 0.0f;
static char  s_last_dir = '?'; /* 上次方向: 摄像头每3帧才发一次, 无新帧时保持, 避免垃圾值 */
static float s_last_cx = 0.0f; /* 上次圆心偏差: NX_GetCirclepos() 是消费式的(V1.25.6),
                                * 无新帧时保持, 否则那几拍会拿到 0 → 分档调速一直判慢速 */
static float s_last_cy = 0.0f;

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
    if (NX_GetCirclepos(&cx, &cy)) {
        s_last_cx = cx;
        s_last_cy = cy;
    } else {
        cx = s_last_cx;
        cy = s_last_cy;
    }
    {
        float dist = sqrtf(cx*cx + cy*cy);
        g_circle_k = (dist > CIRCLE_XY_FAST_TH) ? CIRCLE_XY_V_FAST : CIRCLE_XY_V_SLOW;
        // printf("%f\r\n",dist);
    }

    if (dir == 'O') {
        s_sum_x = cx;
        s_sum_y = cy;
        s_o_cnt++;

        if (s_o_cnt >= CIRCLE_O_STABLE_CNT) {
            g_circle_avg_x = s_sum_x ;   /* ← 先算平均 */
            g_circle_avg_y = s_sum_y ;
            g_circle_dir   = 'O';
            s_o_cnt = 0;                                 /* ← 再清零 */
            s_sum_x = 0.0f;
            s_sum_y = 0.0f;
        } else {
            g_circle_dir = ' ';
        }
    } else {
        s_o_cnt = 0;
        s_sum_x = 0.0f;
        s_sum_y = 0.0f;
        g_circle_dir = dir;
    }

        g_circle_vx=cy*g_circle_k;
        g_circle_vy= cx*g_circle_k;

    if (dir == 'O') {
        g_circle_vx=0;
        g_circle_vy= 0;
    }

    /* ---- 2. 方向 → 速度映射 (速度按 xy 距离分档) ---- */
    // switch (dir) {
    // case 'O':
    //     g_circle_vx = 0.0f;
    //     g_circle_vy = 0.0f;
    //     break;
    //
    // case 'N':
    //     /* 圆心偏上: 前进 */
    //     g_circle_vx = g_circle_k;
    //     g_circle_vy = 0.0f;
    //     break;
    //
    // case 'S':
    //     /* 圆心偏下: 后退 */
    //     g_circle_vx = -g_circle_k;
    //     g_circle_vy = 0.0f;
    //     break;
    //
    // case 'W':
    //     /* 圆心偏右: 右移 */
    //     g_circle_vx = 0.0f;
    //     g_circle_vy = g_circle_k;
    //     break;
    //
    // case 'E':
    //     /* 圆心偏左: 左移 */
    //     g_circle_vx = 0.0f;
    //     g_circle_vy = -g_circle_k;
    //     break;
    //
    // default:
    //     /* 未知方向: 停止 */
    //     break;
    // }

    motor = Mecanum_Calc_Full_V(g_circle_vx, g_circle_vy, 0.0f);

    Send_commandmotor(&motor);
}
