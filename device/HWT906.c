/**
 * @file    HWT906.c
 * @brief   HWT906 陀螺仪设备实例实现 --- 抽象设备层的 imu_hwt906 实例
 *
 * 包装 hardware/sensors/hwt_imu.c：init/update 直接转调 HWT_IMU_Init /
 * HWT_IMU_Poll，位姿缓存统一为 PoseData_t（CLAUDE.md 第 3 节）。
 *
 * 定位源契约（CLAUDE.md 2.1 节）：本设备只承诺【姿态】——yaw / pitch /
 * roll / wz / 三轴线加速度。x / y / vx / vy 无数据源恒为 0，
 * 不能充当 active_locator。
 *
 * 数据流：HWT906 内部完成姿态融合，I2C3 一次事务读回 0x34~0x40 整块
 * （加速度 + 角速度 + 磁场 + 姿态角 + 温度，见 hwt_imu.h）。本实例只取其中三样：
 *   - 姿态角     → yaw / pitch / roll；
 *   - 陀螺仪 GZ  → wz（V1.11.0 起直接取用，此前是 yaw 差分推算）；
 *   - 三轴线加速度 → ax / ay / az。
 * 磁场与温度本实例不取，留在驱动层全局量里（g_hwt_imu_mag_* / g_hwt_imu_temp），
 * 需要就直接读 hwt_imu.h —— 不必为了这两个量再扩 PoseData_t。
 */

#include "HWT906.h"
#include "hwt_imu.h"

/* ============================================================
 * 内部常量与状态
 * ============================================================ */

#define HWT906_DEG2RAD 0.01745329252f   /* pi / 180 */

/* 陀螺仪 Z 轴符号。+1 = "GZ 为正" 与 "yaw 增大" 同向
 * （CCW 为正，与 PoseData_t 的坐标系约定一致）。
 * 【台架判据】手转车身使 yaw 增大，若 gz 读数为负则改成 -1 ——
 * 符号搞反等于角度环正反馈，会直接发散，接线后必须第一个确认这条。 */
#define HWT906_GZ_SIGN  (+1.0f)

static PoseData_t s_pose;              /* 最新姿态缓存（rad / m/s^2） */

/* ============================================================
 * LocatorDev_t 四函数
 * ============================================================ */

/**
 * @brief 设备初始化：探测模块在线并把当前航向设为零点
 * @note  I2C3 外设初始化由 CubeMX 完成（MX_I2C3_Init），此处只做模块
 *        探测与零点设定（HWT_IMU_Init 的既有语义）。
 *        【接线注意】HWT_IMU_Init 会重置航向零点，全工程只能在一处调用：
 *        当前 app/worker_task.c 的 FC_TASK 启动时已调用过一次，若本实例
 *        init() 也接入，两者重复调用不会出问题（都在车未动时），但语义上
 *        应二选一。
 */
static void hwt906_loc_init(void)
{
    (void)HWT_IMU_Init();

    s_pose.x     = 0.0f;
    s_pose.y     = 0.0f;
    s_pose.yaw   = 0.0f;
    s_pose.pitch = 0.0f;
    s_pose.roll  = 0.0f;

    /* 姿态源无位置/线速度数据源，恒为 0（同 locator_wheel 先例） */
    s_pose.vx = 0.0f;
    s_pose.vy = 0.0f;
    s_pose.wz = 0.0f;

    s_pose.ax = 0.0f;
    s_pose.ay = 0.0f;
    s_pose.az = 0.0f;

    s_pose.valid     = 0U;
    s_pose.timestamp = 0U;
}

/**
 * @brief 周期更新：读一帧 0x34~0x40 整块，刷新姿态缓存
 * @note  这是【有副作用】的接口（内部 HWT_IMU_Poll 阻塞读 I2C3，读 26 字节，
 *        阻塞时间随总线速率而定），必须固定周期调用且全局只调一处。
 *        读失败时位姿冻结并置 valid=0，绝不外推。
 */
static void hwt906_loc_update(void)
{
    uint32_t now;

    if (!HWT_IMU_Poll()) {
        s_pose.valid = 0U;              /* I2C 无应答：冻结并标记不可信 */
        return;
    }

    now = HAL_GetTick();

    s_pose.yaw   = g_hwt_imu_yaw_rad;   /* rad，已扣零点，范围 ±π */
    s_pose.pitch = g_hwt_imu_pitch * HWT906_DEG2RAD;
    s_pose.roll  = g_hwt_imu_roll  * HWT906_DEG2RAD;

    if (g_hwt_imu_ext_ok != 0U)
    {
        /* 角速度：V1.11.0 起直接取陀螺仪 GZ（此前由 yaw 差分得到）。
         * 陀螺仪给的是真实角速度，不受 yaw 的更新率与量化限制。
         * 【注意】angle_ctrl.c 的两级 PID 此前按差分信号的噪声特性整定，
         * 换源后需重新确认增益，详见 clauderecord/2026-09-25.md。 */
        s_pose.wz = g_hwt_imu_gyro_z_rad * HWT906_GZ_SIGN;

        s_pose.ax = g_hwt_imu_acc_x;
        s_pose.ay = g_hwt_imu_acc_y;
        s_pose.az = g_hwt_imu_acc_z;
    }
    else
    {
        /* 长块读退化了：只有姿态角是新的，陀螺仪/加速度那几个全局量还是
         * 上一拍的值。宁可给 0 也不把陈旧值当新数据发出去 —— 陈旧角速度
         * 喂进速度环比 0 更危险。退化与否由 g_hwt_imu_ext_ok 反映。 */
        s_pose.wz = 0.0f;
        s_pose.ax = 0.0f;
        s_pose.ay = 0.0f;
        s_pose.az = 0.0f;
    }

    s_pose.valid     = 1U;
    s_pose.timestamp = now;
}

/**
 * @brief 获取最新姿态
 * @note  纯读取，无副作用，可任意频率调用。
 */
static void hwt906_loc_get_pose(PoseData_t *pose_out)
{
    if (pose_out == NULL) {
        return;
    }
    *pose_out = s_pose;
}

/**
 * @brief 设备健康检查
 * @return 1 = 最近一次 update 成功读到模块；0 = 离线/不可信
 */
static uint8_t hwt906_loc_is_healthy(void)
{
    return s_pose.valid;
}

/* ============================================================
 * 设备实例（注册见 CLAUDE.md 第 4 节 / 7.1 节）
 * ============================================================ */

const LocatorDev_t imu_hwt906 = {
    .init       = hwt906_loc_init,
    .update     = hwt906_loc_update,
    .get_pose   = hwt906_loc_get_pose,
    .is_healthy = hwt906_loc_is_healthy,
};
