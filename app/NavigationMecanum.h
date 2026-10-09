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

/** 1 = 当前有一段路线正被 Nav_GoToWorld 驱动（进入驱动循环置 1, 三个出口清 0）。
 *  @note 由 worker_task.c 的 `Route_AbortRequest()` 当判据用: **没有段在跑时
 *        进料事件不置打断标志** —— 否则迟到的进料会打在下一段头上, 使其刚起步
 *        就被"打断", 而游标照常推进 → 平白吃掉一个点位。 */
extern volatile uint8_t g_nav_running;

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
#define NAV_TOL_XY             0.02f   /* 到达容差 3cm */
#define NAV_TOL_YAW            0.05f   /* 到达容差 ~2.9° */
#define NAV_YAW_DEADBAND       0.0175f /* rad ≈ 1.0° (取容差的 1/3) */
#define NAV_ARRIVE_TICKS       5u      /* 连续 5 拍判到达 (抗单帧抖动) */
#define NAV_MAX_INVALID_TICKS  20u     /* OPS9 离线容忍 0.3s, 超限零速保持 */

/* ============================================================
 * 平移轴速度规划 —— V1.24.1
 *
 *      v_ref = clamp(Kp·e, ±√(2·a·|e|), ±V_max)
 *
 * 近场线性 P: 和原来的纯 P 一样温和、过零连续, 不会在点位上抖。
 * 远场制动曲线做**上限**: |v| ≤ √(2·a·|e|) 即"此刻还刹得住", 补上纯 P 缺的
 * 减速约束, 使提速与不过冲不再矛盾。
 *
 * @warning 曲线**只能当上限, 不能当参考** (V1.24.0 的错就在这): 当参考时它在
 *          e→0 处等效增益发散, 且死区边界是 "0 → √(2·a·deadband)" 的阶跃,
 *          位置噪声一到就变成**到点来回晃**, 比纯 P 还差。
 *
 * ⚠ 全部为初值, 上机按实际响应调, 改完记 clauderecord。
 * ============================================================ */
#ifndef NAV_XY_PROFILE
#define NAV_XY_PROFILE   1        /* 1 = 近场 P + 制动上限; 0 = 退回旧的位置 PD (A/B 用) */
#endif
#define NAV_KP_X_LIN    7.3f
#define NAV_KP_Y_LIN    7.3f
#define NAV_KD_X_LIN    0.3f
#define NAV_KD_Y_LIN    0.3f
#define NAV_BRK_X       0.8f
#define NAV_BRK_Y       0.8f
#define NAV_ARRIVE_VMAX  0.19f    /* 到位速度门限 m/s: 必须 > Kp·NAV_TOL_XY (=0.08) 留余量,
                                   * 否则会在容差边缘一直判定不上、卡着不走 */

/* ============================================================
 * 近场/远场控制律切换 —— V1.28.1
 *
 *   远场 (|e| ≥ HI): 制动曲线上限 |v| ≤ √(2·a_brk·|e|)  —— 此刻还刹得住
 *   近场 (|e| ≤ LO): 低速爬行上限 |v| ≤ NAV_CREEP_X_V/_Y_V —— 末端慢走不冲过头
 *   中间: 用 smoothstep 权重把两个上限**平滑混合**(不是 if 分段, 无阶跃)
 *
 * ⚠ 全部为初值, 上机按实际响应调, 改完记 clauderecord。
 * ============================================================ */
#define NAV_SWITCH_LO   0.2f   /* |e| ≤ LO → 纯近场律 (爬行封顶) */
#define NAV_SWITCH_HI   0.4f   /* |e| ≥ HI → 纯远场律 (制动曲线上限) */
#define NAV_CREEP_X_V   0.48f   /* X 轴近场爬行速度上限 m/s */
#define NAV_CREEP_Y_V   0.38f   /* Y 轴近场爬行速度上限 m/s */
/* ★ 上面两个爬行上限**必须 < NAV_ARRIVE_VMAX**, 否则到位判据的 `|v| ≤ NAV_ARRIVE_VMAX`
 *   永远满足不了 —— 低速段 v 会收敛到爬行封顶值, 而封顶值比门限还大。 */

/* ============================================================
 * 圆弧导航 (Nav_Cricle) 参数 —— V1.26.0, OPS9 闭环
 * ⚠ 全部为初值, 上机按实际响应调, 改完记 clauderecord。
 * ============================================================ */
#define NAV_ARC_V         0.5f    /* 圆弧线速度 m/s */
#define NAV_ARC_ACC       0.8f     /* 线速度软启动加速度 m/s² */
#define NAV_ARC_KP_R      1.2f     /* 径向误差修正增益 1/s (把车拉回圆上) */
#define NAV_ARC_VMAX_R    0.5f     /* 径向修正速度限幅 m/s */



extern World_Dir_t g_waypoints[NAV_WAYPOINT_MAX];
extern uint8_t     g_waypoint_count;
extern uint8_t     s_idx;
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
 * @note   V1.23.0 起游标在**两种**情况下都推进：到达，或**被打断**
 *         （`Nav_LastAborted()`）—— 打断的语义是"改奔下一个点"，不是重试。
 *         只有**超时**才不推进、下次重试。
 *
 * @warning 游标只加不校验：一轮物料采集有 5 次 IR 边沿 = 烧掉 5 个点。
 *          路线只有 `g_waypoint_count` 个点，用光后本函数恒返回 true 空转。
 *
 * @return true   本点已到达 / 被打断跳过 / 整条路线已走完
 * @return false  本点超时未到达（游标不推进，下次重试）
 */
bool Nav_FeDuanPoint(void);

/**
 * @brief 上一次 Nav_FeDuanPoint() **走完**的点号（1 基, 与 g_waypoints[] 的注释编号一致）。
 * @return 0    还没走过点 / 该点超时没走到 / 路线已走完
 * @note   给 worker_task.c 的"到位后向前蹭料"挑点位用的（见 NF_CREEP_WP[]）。
 */
uint8_t Nav_LastWaypointNo(void);

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
 * @brief  上一次 `Nav_GoToWorld()` 是不是被**打断**的（而不是超时）。
 *
 *  V1.23.0 起 `Nav_GoToWorld()` 可以被 `g_route_abort`（见 worker_task.h）
 *  中途打断，此时它仍然返回 `false` —— 靠本函数区分"超时"和"被打断"。
 *
 * @warning **必须在关心那次 `Nav_GoToWorld()` 之后立刻调用**，中间不能夹别的
 *          会走导航的调用（`Place()`、`Nav_MoveBody()` 等都会重置这个标记）。
 */
bool Nav_LastAborted(void);

/**
 * @brief 循迹完成后按实测位置校准 a 点 / 亚军点
 *
 * @param is_trophy true=奖杯循迹(LinFolR)校准亚军点, false=物料循迹(LinFolL)校准a点
 */
void Nav_CalibrateAfterTrace(bool is_trophy);

/** 圆弧转向: L = 左弧 (CCW, 圆心在车左侧), R = 右弧 (CW, 圆心在车右侧)。 */
typedef enum {
    Nav_CricleL = 0,   /* 向左走弧 */
    Nav_CricleR,       /* 向右走弧 */
} Nav_Cricle_t;

/**
 * @brief 定半径圆弧 —— 基于 OPS9 位置闭环, 阻塞直到走完或超时
 *
 * 从当前位姿出发, 以半径 Radius 走一条圆弧, 扫过圆心角 Angle 后停下。
 * 方向由 Cricle_t 给出: Nav_CricleL 向左 (CCW), Nav_CricleR 向右 (CW)。
 *
 * 控制结构: 圆心由起点位姿推得 (左弧圆心在车左侧 R 处); 每拍用 OPS9
 * 算出当前扫过的圆心角与径向误差 —— 切向匀速 (软启动) + 径向 P 修正 +
 * 航向"前馈 v/R + 切线误差 P"闭环, 经 Mecanum_Calc_Full_V 下发电机。
 *
 * @note  Angle 单位是**弧度** (与全工程角度约定一致), 度数写 NAV_DEG2RAD 倍。
 * @note  入口按契约关角度环 (g_angle_ctrl_enable=0 + osDelay(20)) 独占电机,
 *        退出**不恢复**该标志 (与 Nav_GoToWorld 同, 调用方需要角度环自己置 1)。
 * @note  反馈无效按 NAV_MAX_INVALID_TICKS 容忍; 超时按几何时长 ×1.5 + 2s 兜底;
 *        支持 g_route_abort 打断 (IR 进料)。任何出口都零速停车并刷新 Self_Dir。
 * @warning Radius / Angle 只取绝对值 —— 方向只能靠 Cricle_t 给, 传负值不报错。
 *
 * @param Cricle_t  方向: Nav_CricleL 左弧 / Nav_CricleR 右弧
 * @param Radius    圆弧半径, 单位 m (>0)
 * @param Angle     扫过的圆心角, 单位 rad (>0)
 */
void Nav_Cricle(Nav_Cricle_t Cricle_t, float Radius, float Angle);
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