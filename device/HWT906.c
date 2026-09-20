/**
 * @file    HWT906.c
 * @brief   HWT906 陀螺仪设备实例实现 --- 抽象设备层的 imu_hwt906 实例
 *
 * 包装 hardware/sensors/hwt_imu.c：init/update 直接转调 HWT_IMU_Init /
 * HWT_IMU_Poll，位姿缓存统一为 PoseData_t（CLAUDE.md 第 3 节）。
 *
 * 定位源契约（CLAUDE.md 2.1 节）：本设备只承诺【姿态】——yaw / pitch /
 * roll / wz。x / y / vx / vy 无数据源恒为 0，不能充当 active_locator。
 *
 * 数据流：HWT906 模块内部完成姿态融合，I2C3 只读回 roll/pitch/yaw
 * （见 hardware/sensors/hwt_imu.h 的说明），无原始角速度 —— wz 由
 * yaw 差分得到（与 app/worker_task.c FC_TASK 角度环同法）。
 */

#include "HWT906.h"
#include "hwt_imu.h"

/* ============================================================
 * 内部状态
 * ============================================================ */

#define HWT906_DEG2RAD 0.01745329252f   /* pi / 180 */
#define HWT906_PI      3.14159265359f

static PoseData_t s_pose;              /* 最新姿态缓存（rad） */

/* wz 差分状态：HWT906 无原始角速度，由 yaw 差分得到 */
static float    s_prev_yaw_rad = 0.0f;
static uint32_t s_prev_tick    = 0U;
static uint8_t  s_have_prev    = 0U;

/* 角差折算到 -π..π（差分前先归一，避免 yaw 在 ±π 跳变时产生 2π 尖峰） */
static float hwt906_wrap_pi(float rad)
{
    while (rad >  HWT906_PI) { rad -= 2.0f * HWT906_PI; }
    while (rad < -HWT906_PI) { rad += 2.0f * HWT906_PI; }
    return rad;
}

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

    s_pose.valid     = 0U;
    s_pose.timestamp = 0U;

    s_prev_yaw_rad = 0.0f;
    s_prev_tick    = 0U;
    s_have_prev    = 0U;
}

/**
 * @brief 周期更新：读一帧 roll/pitch/yaw，刷新姿态缓存
 * @note  这是【有副作用】的接口（内部 HWT_IMU_Poll 阻塞读 I2C3，约
 *        100µs），必须固定周期调用且全局只调一处。
 *        读失败时位姿冻结并置 valid=0，绝不外推。
 */
static void hwt906_loc_update(void)
{
    uint32_t now;
    float    dt;

    if (!HWT_IMU_Poll()) {
        s_pose.valid = 0U;              /* I2C 无应答：冻结并标记不可信 */
        return;
    }

    now = HAL_GetTick();

    s_pose.yaw   = g_hwt_imu_yaw_rad;   /* rad，已扣零点，范围 ±π */
    s_pose.pitch = g_hwt_imu_pitch * HWT906_DEG2RAD;
    s_pose.roll  = g_hwt_imu_roll  * HWT906_DEG2RAD;

    /* 角速度：由 yaw 差分得到（与 FC_TASK 角度环同法） */
    if (s_have_prev != 0U) {
        dt = (float)(now - s_prev_tick) / 1000.0f;
        s_pose.wz = (dt > 0.001f)
                    ? hwt906_wrap_pi(s_pose.yaw - s_prev_yaw_rad) / dt
                    : 0.0f;
    } else {
        s_pose.wz = 0.0f;
    }
    s_prev_yaw_rad = s_pose.yaw;
    s_prev_tick    = now;
    s_have_prev    = 1U;

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
