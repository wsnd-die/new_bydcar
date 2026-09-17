/**
 * @file    drv_wheel_odom.c
 * @brief   轮式里程计定位驱动 --- 抽象设备层的 locator_wheel 实例实现
 *
 * 由 algorithm/Nav_position.c 的 World_position_get() 迁移而来。
 * 积分算法逐行保持原样，仅把原先散在函数内的 static 状态
 * （first / prev / 清零标志）收进本文件，使其不再对外可见。
 *
 * 阶段 0 约束：行为与迁移前【逐位一致】，调用方一律不改。
 *
 * 定位源契约：
 *   本驱动对上层承诺的输出是【世界系 x / y / yaw 三个量】。
 *   x、y 由四轮编码器增量推算；yaw 直接取 hardware/hwt_imu.c 的
 *   g_hwt_imu_yaw_rad（HWT906 实测航向）。
 *
 *   yaw 取自 IMU 是【有意设计】，不是待还的技术债：
 *   轮式里程计本身给不出绝对航向，而后续要替换成的 OPS9 定位模块会直接
 *   输出 x/y/yaw。两者对上层是同一个契约，业务层切换定位源时只看
 *   LocatorDev_t，不需要知道 yaw 从哪来。因此不要把 IMU 读取从这里拆出去。
 */

#include "drv_wheel_odom.h"
#include "Common_used.h"

/* ============================================================
 * 内部状态
 *
 * 迁移前这些是 World_position_get() 内的 static 局部变量及文件级标志，
 * 现在收进驱动内部；外部只能经 LocatorDev_t 接口访问，无法直接触碰。
 * ============================================================ */

static PoseData_t  s_pose;              /* 最新位姿缓存 */
static EncoderData s_prev;              /* 上一帧编码器值 */
static uint8_t     s_first     = 1U;    /* 首帧标志：只记基准，不积分 */
static uint8_t     s_reset_req = 0U;    /* 清零请求：置 1 后下帧重设基准 */

/**
 * @brief 设备初始化：清空软件状态
 * @note  编码器基准由第一帧 update 建立，此处不读硬件。
 *        电机驱动（Emm_V5）的外设初始化在别处完成，本驱动不重复初始化。
 */
static void wheel_odom_init(void)
{
    s_pose.x   = 0.0f;
    s_pose.y   = 0.0f;
    s_pose.yaw = 0.0f;
    s_pose.pitch = 0.0f;
    s_pose.roll  = 0.0f;

    /* 轮式里程计给不出车体速度，这三个字段本驱动不填充，恒为 0 */
    s_pose.vx = 0.0f;
    s_pose.vy = 0.0f;
    s_pose.wz = 0.0f;

    s_pose.valid     = 0U;
    s_pose.timestamp = 0U;

    s_first     = 1U;
    s_reset_req = 0U;
}

/**
 * @brief 周期更新：读编码器、积分、更新内部缓存
 * @note  这是【有副作用】的接口，必须固定周期调用且全局只调一处。
 *        读取失败时保留上次位姿但置 valid=0，绝不推进积分。
 */
static void wheel_odom_update(void)
{
    EncoderData enc;
    float d_fwd, d_side, fwd_mm, side_mm;
    float yaw = g_hwt_imu_yaw_rad;      /* IMU 实测航向 rad (HWT906 直接给角度) */

    if (!Mecanum_Read_AllPositions(&enc, 20)) {
        s_pose.valid = 0U;              /* 读失败：位姿冻结并标记不可信 */
        return;
    }

    /* 清零后：以当前编码器为基准重新起算，不跨清零点累积 */
    if (s_reset_req) {
        s_prev      = enc;
        s_reset_req = 0U;
        s_pose.valid     = 1U;
        s_pose.timestamp = HAL_GetTick();
        return;
    }
    if (s_first) {
        s_first = 0U;
        s_prev  = enc;
        s_pose.valid     = 1U;
        s_pose.timestamp = HAL_GetTick();
        return;
    }

    /* 麦轮正解 (右轮与左轮编码器反号, 取反统一: 后退全负/前进全正) */
    d_fwd  = (float)(enc.fl - s_prev.fl - (enc.fr - s_prev.fr) +
                     enc.rl - s_prev.rl - (enc.rr - s_prev.rr)) / 4.0f;
    d_side = (float)(-(enc.fl - s_prev.fl) - (enc.fr - s_prev.fr) +
                     enc.rl - s_prev.rl + (enc.rr - s_prev.rr)) / 4.0f;
    s_prev = enc;

    /* 脉冲→mm */
    Odometry_Apply_Calib(d_fwd, d_side, &fwd_mm, &side_mm);
    s_pose.x += (fwd_mm * cosf(yaw) - side_mm * sinf(yaw)) / 2000.0f;
    s_pose.y += (fwd_mm * sinf(yaw) + side_mm * cosf(yaw)) / 2000.0f;
    s_pose.yaw = yaw;

    s_pose.valid     = 1U;
    s_pose.timestamp = HAL_GetTick();
}

/**
 * @brief 获取最新位姿
 * @note  纯读取，无副作用，可任意频率调用，不影响积分推进。
 */
static void wheel_odom_get_pose(PoseData_t *pose_out)
{
    if (pose_out == NULL) {
        return;
    }
    *pose_out = s_pose;
}

/**
 * @brief 设备健康检查
 * @return 1 = 最近一次 update 成功读到编码器，位姿可信；0 = 不可信
 */
static uint8_t wheel_odom_is_healthy(void)
{
    return s_pose.valid;
}

/* ============================================================
 * 清零
 * ============================================================ */

void Wheel_Odom_Reset(void)
{
    /* 电机驱动编码器归零 (地址同 Mecanum_Read_AllPositions: 1~4) */
    Emm_V5_Reset_CurPos_To_Zero(1);
    Emm_V5_Reset_CurPos_To_Zero(2);
    Emm_V5_Reset_CurPos_To_Zero(3);
    Emm_V5_Reset_CurPos_To_Zero(4);

    s_pose.x = 0.0f;
    s_pose.y = 0.0f;
    /* yaw 由 IMU 实时给, 不归零 */
    s_reset_req = 1U;   /* 下次 update 以当前编码器为基准, 不跨清零点累积 */
}

/* ============================================================
 * 设备实例
 * ============================================================ */

const LocatorDev_t locator_wheel = {
    .init       = wheel_odom_init,
    .update     = wheel_odom_update,
    .get_pose   = wheel_odom_get_pose,
    .is_healthy = wheel_odom_is_healthy,
};
