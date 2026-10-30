#ifndef __NAVIGATION_MECANUM_H
#define __NAVIGATION_MECANUM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 类型定义
 * ============================================================ */

/* 世界坐标系: X 前方, Y 左方 (右手系), yaw 弧度 CCW 为正, 范围 [-π, π] */
typedef struct {
    float x;       /* 世界 X 坐标, m */
    float y;       /* 世界 Y 坐标, m */
    float yaw;     /* 世界航向角, rad, [-π, π] */
} World_Dir_t;

/* 当前自身位姿 (世界坐标系) */
extern World_Dir_t Self_Dir;

/** 1 = 当前有路线段正被 Nav_GoToWorld 驱动 (Route_AbortRequest 判据用)。 */
extern volatile uint8_t g_nav_running;

/* ============================================================
 * 路径点
 * ============================================================ */

/* 最大路径点数量 */
#define NAV_WAYPOINT_MAX  32

/** 角度 → 弧度 (π/180)，常量表达式，可用于静态初始化。 */
#define NAV_DEG2RAD       0.01745329252f

/* 世界系位置闭环参数 (OPS9 反馈)。全部为初版整定值，改完记 clauderecord。 */
#define NAV_DT                 0.011f   /* 名义控制周期 s (执行器内 osDelay(5) 使实际 ~15ms) */
#define NAV_LOOP_TICKS         5u     /* osDelay(10) → 名义 100Hz */
#define NAV_TIMEOUT_MS         10000u  /* 单点超时 ms */
#define NAV_KP_XY              3.2f    /* 平移 P: 0.4m 误差 → 0.4 m/s */
#define NAV_KD_XY              0.5f    /* 平移 D: 首版关 (OPS9 噪声放大风险) */
#define NAV_KP_YAW             5.0f    /* 航向 P: 0.5rad 误差 → 1 rad/s */
#define NAV_KD_YAW             0.27f    /* 航向 D: 首版关 */
#define NAV_VMAX_X            2.5f    /* x平移速度限幅 m/s */
#define NAV_VMAX_Y            2.5f    /* y平移速度限幅 m/s */
#define NAV_VMAX_W             2.5f    /* 角速度限幅 rad/s */
#define NAV_ACC_XY             1.5f    /* 平移加速度 m/s² (软启动) */
#define NAV_ACC_W              1.5f    /* 角加速度 rad/s² (软启动) */
#define NAV_TOL_XY             0.07f   /* 到达容差 3cm */
#define NAV_TOL_YAW            0.12f   /* 到达容差 ~2.9° */
#define NAV_YAW_DEADBAND       0.0275f /* rad ≈ 1.0° (取容差的 1/3) */
#define NAV_W_DEADBAND         0.047f
#define NAV_ARRIVE_TICKS       5u      /* 连续 5 拍判到达 (抗单帧抖动) */
#define NAV_MAX_INVALID_TICKS  20u     /* OPS9 离线容忍 0.3s, 超限零速保持 */

/* ============================================================
 * 平移轴速度规划 —— 固定距离硬切换
 *
 *      |e| ≤ NAV_SWITCH_D : PID      v = Kp·e − Kd·v_now
 *      |e| > NAV_SWITCH_D : 制动曲线 v = ±√(2·a_brk·|e|)
 *
 * 曲线只当远场上限 —— 它在 e→0 处增益发散，当参考会"到点来回晃"。
 * ⚠ 两条律在 |e| = D 处不连续；全部为初值，上机再调。
 * ============================================================ */
#ifndef NAV_XY_PROFILE
#define NAV_XY_PROFILE   1        /* 1 = 近场 P + 制动上限; 0 = 退回旧的位置 PD (A/B 用) */
#endif

/** 1 = 串级航向环 (angle_ctrl), 步进在 Nav_GoToWorld() 循环里;
 *  0 = 退回手写单环 PD (A/B 回退用)。 */
#ifndef NAV_YAW_CASCADE
#define NAV_YAW_CASCADE  1
#endif
#define NAV_KP_X_LIN    3.3f
#define NAV_KP_Y_LIN    4.7f
#define NAV_KD_X_LIN    1.0f
#define NAV_KD_Y_LIN    1.5f
#define NAV_BRK_X       0.45f
#define NAV_BRK_Y       0.55f
#define NAV_ARRIVE_VMAX  0.6f


#define NAV_SWITCH_LO   0.3f   /* |e| ≤ LO → 纯近场律 (爬行封顶) */
#define NAV_SWITCH_HI   0.7f   /* |e| ≥ HI → 纯远场律 (制动曲线上限) */
#define NAV_CREEP_X_V   0.55f   /* X 轴近场爬行速度上限 m/s —— **默认值** */
#define NAV_CREEP_Y_V   0.55f   /* Y 轴近场爬行速度上限 m/s —— **默认值** */


#define NAV_SLOW_WP_IDX   7u     /* "12 摆放e 点" (g_waypoints[] 下标 7) */
#define NAV_SLOW_WP_IDX2  6u
#define NAV_SLOW_WP_LIM   0.35f   /* 走这个点时的近场爬行上限 m/s (X/Y 共用) */

#define NAV_HOME_SPEED   1.4f

extern volatile float g_nav_leg_speed;


#define NAV_ARC_V         0.55f    /* 圆弧线速度 m/s */
#define NAV_ARC_ACC       0.9f     /* 线速度软启动加速度 m/s² */
#define NAV_ARC_KP_R      1.6f     /* 径向误差修正增益 1/s (把车拉回圆上) */
#define NAV_ARC_VMAX_R    0.6f     /* 径向修正速度限幅 m/s */



extern World_Dir_t g_waypoints[NAV_WAYPOINT_MAX];
extern uint8_t     g_waypoint_count;
extern uint8_t     s_idx;
/* ============================================================
 * 函数声明
 * ============================================================ */


/**
 * @brief 导航到目标世界坐标 (OPS9 位置闭环, 阻塞直到到达或超时)
 *
 * @note 入口按契约关角度环 (g_angle_ctrl_enable=0), **退出不恢复** ——
 *       调用方需要角度环得自己置 1。
 * @note 反馈无效按 NAV_MAX_INVALID_TICKS 容忍, 超限零速保持。
 * @param target_x    目标世界 X, m
 * @param target_y    目标世界 Y, m
 * @param target_yaw  目标世界航向, rad
 * @return true 已到达; false 超时 (含反馈长期无效)
 */
bool Nav_GoToWorld(float target_x, float target_y, float target_yaw);


void Nav_SetLegSpeed(float k);

/**
 * @brief 分点导航 —— 按 g_waypoints[] 逐点推进 (一次一步)
 *
 * @note 到达判据 / 超时 / 反馈失效都在 Nav_GoToWorld() 里, 本函数只推进游标。
 * @note 到达或被打断都推进游标 (打断 = 改奔下一点); 只有超时不推进、下次重试。
 * @warning 游标只加不校验, 路线点用光后恒返回 true 空转。
 * @return true 本点到达 / 被打断跳过 / 路线已走完; false 超时
 */
bool Nav_FeDuanPoint(void);

/**
 * @brief 上一次 Nav_FeDuanPoint() 走完的点号 (1 基); 0 = 没走到 / 已走完。
 */
uint8_t Nav_LastWaypointNo(void);

/**
 * @brief 世界系相对移动 —— 目标 = Self_Dir + (target_x, target_y, target_yaw)
 *
 * @note **不是车体坐标**: 传 (d, 0, 0) 只在车头正对世界 +X 时才是"前进"。
 * @note target_yaw 相加后未归一化到 ±π, 大角度需调用方自行 wrap。
 */
bool Nav_MoveBody(float target_x, float target_y, float target_yaw);

/**
 * @brief  上一次 Nav_GoToWorld() 是被打断的 (而不是超时)。
 * @warning 必须在关心那次调用之后立刻调, 中间不能夹会走导航的调用。
 */
bool Nav_LastAborted(void);

/** 圆弧转向: L = 左弧 (CCW, 圆心在车左侧), R = 右弧 (CW, 圆心在车右侧)。 */
typedef enum {
    Nav_CricleL = 0,   /* 向左走弧 */
    Nav_CricleR,       /* 向右走弧 */
} Nav_Cricle_t;

/**
 * @brief 定半径圆弧 —— 基于 OPS9 位置闭环, 阻塞直到走完或超时
 *
 * @note Angle 单位是弧度; 支持 g_route_abort 打断。任何出口都零速停车。
 * @warning Radius / Angle 只取绝对值 —— 方向只能靠 Cricle_t 给。
 * @param Cricle_t  Nav_CricleL 左弧 / Nav_CricleR 右弧
 * @param Radius    半径, m (>0)
 * @param Angle     扫过的圆心角, rad (>0)
 */
void Nav_Cricle(Nav_Cricle_t Cricle_t, float Radius, float Angle);

/**
 * @brief 依次执行所有路径点 (逐点 Nav_GoToWorld, 阻塞)。
 * @return true 全部成功; false 中途某点失败
 */
bool Nav_RunWaypoints(void);

 void NAV_SHORT_WORD(float x, float y, float yaw);

#ifdef __cplusplus
}
#endif

#endif /* __NAVIGATION_MECANUM_H */