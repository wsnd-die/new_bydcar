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

/** 角度 → 弧度 (π/180)。写路径点 yaw 用: -90.0f * NAV_DEG2RAD
 *  @note 常量表达式, 可直接用于数组静态初始化。 */
#define NAV_DEG2RAD       0.01745329252f

/* ============================================================
 * 世界系位置闭环参数 (100Hz, OPS9 反馈)
 * 全部为初版整定值, 上机按实际响应调, 改完记 clauderecord。
 * ============================================================ */
#define NAV_DT                 0.01f   /* 名义控制周期 s (执行器内 osDelay(5) 使实际 ~15ms) */
#define NAV_LOOP_TICKS         10u     /* osDelay(10) → 名义 100Hz */
#define NAV_TIMEOUT_MS         10000u  /* 单点超时 ms */
#define NAV_KP_XY              1.2f    /* 平移 P: 0.4m 误差 → 0.4 m/s */
#define NAV_KD_XY              0.0f    /* 平移 D: 首版关 (OPS9 噪声放大风险) */
#define NAV_KP_YAW             2.0f    /* 航向 P: 0.5rad 误差 → 1 rad/s */
#define NAV_KD_YAW             0.0f    /* 航向 D: 首版关 */
#define NAV_VMAX_XY            1.6f    /* 平移速度限幅 m/s */
#define NAV_VMAX_W             2.0f    /* 角速度限幅 rad/s */
#define NAV_ACC_XY             1.5f    /* 平移加速度 m/s² (软启动) */
#define NAV_ACC_W              2.1f    /* 角加速度 rad/s² (软启动) */
#define NAV_TOL_XY             0.02f   /* 到达容差 3cm */
#define NAV_TOL_YAW            0.05f   /* 到达容差 ~2.9° */
#define NAV_ARRIVE_TICKS       3u      /* 连续 5 拍判到达 (抗单帧抖动) */
#define NAV_MAX_INVALID_TICKS  20u     /* OPS9 离线容忍 0.3s, 超限零速保持 */

/**
 * 分点导航路径点表（世界坐标系）。
 *
 * 每个元素: { X(m), Y(m), yaw(rad) }，yaw 用角度写更直观: 90.0f * NAV_DEG2RAD。
 *
 * @warning 表里的坐标是 **2026-09-14 旧场地**的实测值，上机前必须逐点复核。
 *          配合 worker_task.c 的 NF_AUTOSTART=1 时，这张表就是"上电即发车"
 *          的路线，填错会直接把车开出去。
 * @note    改完表记得同步 `g_waypoint_count`（Nav_FeDuanPoint 的游标上界）。
 */
extern World_Dir_t g_waypoints[NAV_WAYPOINT_MAX];
extern uint8_t      g_waypoint_count;

/* ============================================================
 * 函数声明
 * ============================================================ */


/**
 * @brief 导航到目标世界坐标（世界系位置闭环）
 *
 * 以 OPS9 位姿 (locator_ops9.get_pose, 世界系 x/y/yaw) 为反馈，
 * 三轴并行 PD (Ki=0) 输出世界系期望速度 → 软启动加速度斜坡 →
 * 世界→车体旋转 → Mecanum_Calc_Full_V → Send_commandmotor
 * 下发电机速度环。阻塞直到到达或超时，退出时零速停车。
 *
 * @note 入口按契约关角度环（g_angle_ctrl_enable=0 + osDelay(20)）独占电机
 *       控制权，**但退出时不恢复该标志** —— 调用方若后续需要角度环，得自己
 *       重新置 1（对照 algorithm/arc_path.c 的 Arc_Run/Arc_Abort 是有借有还的）。
 * @note 反馈无效按 NAV_MAX_INVALID_TICKS 容忍：短时冻结指令继续跑，
 *       超限零速保持并清斜坡，恢复后从零重新软启动。
 * @param target_x    目标世界 X 坐标，单位：m
 * @param target_y    目标世界 Y 坐标，单位：m
 * @param target_yaw  目标世界航向角，单位：rad
 * @return true       已到达（连续 NAV_ARRIVE_TICKS 拍误差入容差）
 * @return false      超时（NAV_TIMEOUT_MS，含反馈长期无效）
 */
bool Nav_GoToWorld(float target_x, float target_y, float target_yaw);

/**
 * @brief 分点导航 —— 按 g_waypoints[] 逐点推进
 *
 * 每次调用驱动到**一个**路径点（内部游标记进度），走完整张表后恒返回 true。
 * "一次一步"是刻意的：worker_task.c 的 NF_Stage_Navigation() 每被流程调到
 * 一次就推进一站，站点之间流程还能经 NF_DispatchNext() 分发到循迹等其它阶段。
 *
 * @note 到达判据、超时、反馈失效处理都在 Nav_GoToWorld() 里，本函数只做推进。
 * @note 与旧副本 (4eedf6a) 的差异：旧版把第 13 点之后的两点塞在同一次调用里
 *       （`PontIntex == 13` 的特判），这里不再特判，表中 13/14 号就是普通点。
 *
 * @warning 游标只在**成功**时推进，超时的点下次会重试。而调用方
 *          NF_Stage_Navigation() 目前**忽略本函数的返回值** —— 若某个点因
 *          OPS9 离线等原因持续失败，流程会永远卡在 Navigation 阶段。
 *          真出现这种情况，需要给本函数加失败上限计数，或让调用方检查返回值。
 *
 * @return true   本点已到达（或整条路线已走完）
 * @return false  本点超时未到达（游标不推进，下次重试）
 */
bool Nav_FeDuanPoint(void);

/**
 * @brief 世界系相对移动 —— 以当前 Self_Dir 为基准走一个相对位移
 *
 * 三个参数都是**世界系增量**，不是车体量：
 *     目标 = Self_Dir + (target_x, target_y, target_yaw)
 * 内部转调 Nav_GoToWorld() 做闭环，阻塞直到到位或超时。
 *
 * @note **不是车体坐标。** 想让车沿车头方向走 d 米，得先把 d 投到世界系；
 *       直接传 (d, 0, 0) 只有车头正对世界 +X 轴时才等于"前进"。
 * @note target_yaw 与 Self_Dir.yaw 相加后**未归一化到 ±π**，调用方若传大角度
 *       需自行 wrap。
 * @note 旧版同名函数是"先原地转到 rotate_rad，再按车体 forward/left 平移"，
 *       与本实现语义完全不同 —— 本头文件此前贴的是旧版文档，已按现实现更正。
 */
bool Nav_MoveBody(float target_x, float target_y, float target_yaw);

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



#ifdef __cplusplus
}
#endif

#endif /* __NAVIGATION_MECANUM_H */