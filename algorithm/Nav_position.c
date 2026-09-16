//
// Created by 35037 on 2026/8/12.
//
#include "Nav_position.h"
#include "drv_wheel_odom.h"

/* ============================================================
 * 兼容适配层 (阶段 0)
 *
 * 里程计的实际实现已迁移到 device/drv_wheel_odom.c 的 locator_wheel 实例。
 * 本文件保留 World_position / World_position_get() / World_Reset() 三个旧符号，
 * 内部转调 locator_wheel，使现有调用方（app/NavigationMecanum.c 等）无需改动。
 *
 * 注意：World_position_get() 保持迁移前的【消费型】语义 —— 每调用一次就推进
 *       一帧积分。这是既有行为，不是缺陷，阶段 0 不改变它。
 *       阶段 1 会建 app/task_chassis.c 固定周期调 update()，调用方改走纯读的
 *       get_pose()，届时本文件即可删除。
 * ============================================================ */

/* 当前里程计位姿（世界坐标 m / rad），上电原点 (0,0,0) */
World_Dir_t World_position = {0.0f, 0.0f, 0.0f};

/**
 * @brief 清零里程计并重新起算 (放完 5 物块后调用, 奖杯段从 0 重新记)
 *        同时把 4 个电机驱动的编码器计数值清零, 使直接读编码器也是 0
 */
void World_Reset(void)
{
    Wheel_Odom_Reset();

    /* 镜像清零: 保持 World_position 在下次 World_position_get() 之前也是 0 */
    World_position.x = 0.0f;
    World_position.y = 0.0f;
    /* yaw 由 IMU 实时给, 不归零 */
}

/**
 * @brief 增量式编码器里程计
 * @return 当前世界位姿（同时更新全局 World_position）
 * @note  适配层: 转调 locator_wheel，行为与迁移前逐位一致。
 */
World_Dir_t World_position_get(void)
{
    PoseData_t p;

    /* 保持"消费型"语义: 先推进一帧积分, 再纯读出结果 */
    locator_wheel.update();
    locator_wheel.get_pose(&p);

    World_position.x   = p.x;
    World_position.y   = p.y;
    World_position.yaw = p.yaw;

    return World_position;
}


/* ============================================================
 * 惯导 (INS) 已移除
 *
 * 原来的 Ins_Init / Ins_Update / g_ins 靠 imu660ra 的原始加速度做二重积分
 * 推位置, 换成 HWT906 后没有原始加速度数据源; 它依赖的 Mahony 姿态解算
 * (siyuan_get_quat) 也已一并移除。原代码完整保留在:
 *     obsolete/imu660/Nav_position_INS_reference.c
 * 以后若接入带原始 IMU 数据的传感器, 可从那里取回。
 *
 * 现在定位只用 device/drv_wheel_odom.c 的 locator_wheel, 航向取自
 * hwt_imu.h 的 g_hwt_imu_yaw_rad。
 * ============================================================ */
