//
// Created by ZHUHUI on 2026/8/6.
//
// BollLocator — 基于 TB_position 定位器的位置闭环控制器
//
// 闭环链路:
//   TB_position (世界坐标 mm, 串口2定位器)
//   → 测量距离 + v·dt 预测融合 → 梯形速度规划 (缓启/缓停)
//   → 世界→车体坐标变换 (锁定 imu_yaw)
//   → Mecanum_Calc_Full(cmd_vx, cmd_vy, 0)
//   → Send_commandmotor() → Emm_V5_Vel_Control() → FDCAN1
//
// 预测融合说明:
//   meas_dist  = |TB_position − target|                    (mm, 测量值)
//   travel_est = cur_speed(m/s) × dt(s) × 1000(mm/m)      (mm, 预估行程)
//   pred_dist  = meas_dist − travel_est                    (mm, 预测剩余)
//   fused_dist = α × meas_dist + (1−α) × pred_dist        (mm, 互补滤波)
//
// 效果: 当车体正在运动时, pred_dist < meas_dist, 融合距离领先于测量,
//       控制回路提前减速, 减少超调; 停止后 travel_est≈0, 退化为纯测量。
//

#include "BollLocator.h"

/* ============================================================
 * 辅助宏
 * ============================================================ */
#define CLAMP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define DEG2RAD(d)        ((d) * 0.01745329252f)
#define SQ(x)             ((x) * (x))

/* ============================================================
 * 初始化
 * ============================================================ */
void BL_Init(BollLocator *bl)
{
    if (!bl) return;
    memset(bl, 0, sizeof(*bl));
    bl->max_speed  = BL_DEFAULT_MAX_SPEED;
    bl->accel      = BL_DEFAULT_ACCEL;
    bl->decel      = BL_DEFAULT_DECEL;
    bl->pos_tol    = BL_DEFAULT_POS_TOL;
    bl->fuse_alpha = 0.80f;   /* 80% 测量 + 20% 预测 */
    bl->active     = false;
    bl->cur_speed  = 0.0f;
}

/* ============================================================
 * 设置目标 — 激活位置环
 * ============================================================ */
void BL_SetTarget(BollLocator *bl, float x_mm, float y_mm)
{
    if (!bl) return;
    bl->target_x   = x_mm;
    bl->target_y   = y_mm;
    bl->active     = true;
    bl->cur_speed  = 0.0f;

    /* 记录起点, 用于后续检测 TB_position 更新 */
    bl->last_tb_x  = TB_position.xdata;
    bl->last_tb_y  = TB_position.ydata;
}

/* ============================================================
 * 停止
 * ============================================================ */
void BL_Stop(BollLocator *bl)
{
    if (!bl) return;
    bl->active    = false;
    bl->cur_speed = 0.0f;
    bl->cmd_vx    = 0.0f;
    bl->cmd_vy    = 0.0f;
    bl->travel_est = 0.0f;

    MecanumResult motor = Mecanum_Calc_Full(0.0f, 0.0f, 0.0f);
    Send_commandmotor(&motor);
}

/* ============================================================
 * 到达判断
 * ============================================================ */
bool BL_Arrived(const BollLocator *bl)
{
    if (!bl) return true;
    return (!bl->active) || (bl->dist < bl->pos_tol);
}

/* ============================================================
 * 核心: 位置环更新 (每 10ms 调用一次)
 *
 * 步骤:
 *   1. 读 TB_position → 测量距离 meas_dist
 *   2. 预测融合: travel_est = v × dt × 1000, fused = α·meas + (1-α)·(meas−travel)
 *   3. 梯形速度规划: 加速段 / 减速段
 *   4. 坐标变换 + 逆运动学 + 发送
 * ============================================================ */
void BL_Update(BollLocator *bl)
{
    if (!bl || !bl->active) return;

    /* ============================================================
     * 1. 读取当前位置 + 测量距离
     * ============================================================ */
    float cur_x = TB_position.xdata;  /* mm */
    float cur_y = TB_position.ydata;  /* mm */

    float dx = bl->target_x - cur_x;
    float dy = bl->target_y - cur_y;
    bl->err_x = dx;
    bl->err_y = dy;

    /* 测量距离 (纯 TB_position) */
    float meas_dist = sqrtf(dx * dx + dy * dy);  /* mm */

    /* ============================================================
     * 2. v·dt 预测融合
     *
     * travel_est: 本控制周期内车体预估行程 (mm)
     *   cur_speed 单位 m/s, BL_DT 单位 s
     *   travel_est = cur_speed × dt × 1000
     *
     * pred_dist: 预测到达目标还需走的距离
     *   = meas_dist − travel_est
     *
     * fused_dist: 互补滤波
     *   α 接近 1 → 以测量为准; α 接近 0 → 以预测为准
     * ============================================================ */
    float travel_est = bl->cur_speed * BL_DT * 1000.0f;   /* mm */
    bl->travel_est = travel_est;

    /* 预测剩余距离, 不小于 0 */
    float pred_dist = meas_dist - travel_est;
    if (pred_dist < 0.0f) pred_dist = 0.0f;

    /* 互补滤波融合 */
    bl->dist_meas = meas_dist;
    bl->dist_fused = bl->fuse_alpha * meas_dist
                   + (1.0f - bl->fuse_alpha) * pred_dist;

    /* 最终使用的距离 */
    bl->dist = bl->dist_fused;

    /* ============================================================
     * 3. 到达判断 (用测量值防止预测不准导致提前退出)
     * ============================================================ */
    if (meas_dist < bl->pos_tol) {
        bl->active    = false;
        bl->cur_speed = 0.0f;
        bl->cmd_vx    = 0.0f;
        bl->cmd_vy    = 0.0f;
        MecanumResult motor = Mecanum_Calc_Full(0.0f, 0.0f, 0.0f);
        Send_commandmotor(&motor);
        return;
    }

    /* ============================================================
     * 4. 梯形速度规划 (缓启 / 缓停)
     *
     * 减速距离: stop_dist = v² / (2 × decel)
     *   当 fused_dist ≤ stop_dist 时进入减速段
     *
     * 加速段: cur_speed 以 accel 斜率 ramp 到 max_speed
     * ============================================================ */
    float dist_m   = bl->dist * 0.001f;           /* mm → m */
    float stop_dist = (bl->cur_speed * bl->cur_speed)
                    / (2.0f * bl->decel);          /* m */

    float target_speed;

    if (dist_m <= stop_dist) {
        /*
         * 减速段: v_allow = √(2 × decel × dist)
         * 保证以 decel 斜率恰好停在目标
         */
        target_speed = sqrtf(2.0f * bl->decel * dist_m);
        if (target_speed < 0.02f) target_speed = 0.02f;
        target_speed = CLAMP(target_speed, 0.0f, bl->max_speed);
    } else {
        /* 巡航/加速段 */
        target_speed = bl->max_speed;
    }

    /* 缓启: 加速度限制 ramp */
    float dv = target_speed - bl->cur_speed;
    float max_dv = bl->accel * BL_DT;
    bl->cur_speed += CLAMP(dv, -max_dv, max_dv);

    /* 下限保护 */
    if (bl->cur_speed < 0.005f) bl->cur_speed = 0.005f;

    /* ============================================================
     * 5. 世界误差 → 车体速度 (锁定当前 yaw)
     *
     * 旋转矩阵将世界系 (dx, dy) 转到车体系 (body_dx, body_dy):
     *   body_dx =  dx·cosθ + dy·sinθ   (车体前进)
     *   body_dy = -dx·sinθ + dy·cosθ   (车体左移)
     * ============================================================ */
    float yaw_rad = DEG2RAD(imu_yaw);
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);
    float body_dx =  dx * cos_y + dy * sin_y;
    float body_dy = -dx * sin_y + dy * cos_y;

    /* 归一化方向 */
    float dir_x = (bl->dist > 0.0f) ? (body_dx / bl->dist) : 0.0f;
    float dir_y = (bl->dist > 0.0f) ? (body_dy / bl->dist) : 0.0f;

    /* 输出速度 = 方向 × 当前速度 */
    bl->cmd_vx = dir_x * bl->cur_speed;
    bl->cmd_vy = dir_y * bl->cur_speed;

    /* ============================================================
     * 6. 逆运动学 + 发送电机命令
     * ============================================================ */
    MecanumResult motor = Mecanum_Calc_Full(bl->cmd_vx, bl->cmd_vy, 0.0f);
    Send_commandmotor(&motor);
}

/* ============================================================
 * 阻塞式移动: 设目标 → 等到达 → 停止
 * ============================================================ */
bool BL_MoveTo(BollLocator *bl, float x_mm, float y_mm, uint32_t timeout_ms)
{
    if (!bl) return false;

    BL_SetTarget(bl, x_mm, y_mm);

    uint32_t t0 = osKernelGetTickCount();

    while (!BL_Arrived(bl)) {
        BL_Update(bl);
        osDelay(10);  /* 100Hz 控制周期 */

        /* 超时检查 */
        if (timeout_ms > 0) {
            uint32_t elapsed = osKernelGetTickCount() - t0;
            if (elapsed >= timeout_ms) {
                BL_Stop(bl);
                return false;
            }
        }
    }

    BL_Stop(bl);
    return true;
}
