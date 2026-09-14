//
// Created by 35037 on 2026/8/12.
//
#include "Nav_position.h"

/* 当前里程计位姿（世界坐标 m / rad），上电原点 (0,0,0) */
World_Dir_t World_position = {0.0f, 0.0f, 0.0f};

/* 清零请求: 置 1 后下次 World_position_get 以当前编码器为基准重设起点 */
static volatile uint8_t s_world_reset = 0U;

/**
 * @brief 清零里程计并重新起算 (放完 5 物块后调用, 奖杯段从 0 重新记)
 *        同时把 4 个电机驱动的编码器计数值清零, 使直接读编码器也是 0
 */
void World_Reset(void)
{
    /* 电机驱动编码器归零 (地址同 Mecanum_Read_AllPositions: 1~4) */
    Emm_V5_Reset_CurPos_To_Zero(1);
    Emm_V5_Reset_CurPos_To_Zero(2);
    Emm_V5_Reset_CurPos_To_Zero(3);
    Emm_V5_Reset_CurPos_To_Zero(4);

    World_position.x = 0.0f;
    World_position.y = 0.0f;
    /* yaw 由 IMU 实时给, 不归零 */
    s_world_reset = 1U;   /* 下次读取以当前编码器为基准, 不跨清零点累积 */
}

/**
 * @brief 增量式编码器里程计
 * @return 当前世界位姿（同时更新全局 World_position）
 */
World_Dir_t World_position_get(void)
{
    static uint8_t     first = 1;
    static EncoderData prev;
    EncoderData enc;
    float d_fwd, d_side, fwd_mm, side_mm;
    float yaw = g_hwt_imu_yaw_rad;      /* IMU 实测航向 rad (HWT906 直接给角度) */

    if (!Mecanum_Read_AllPositions(&enc, 20)) return World_position;

    /* 清零后: 以当前编码器为基准重新起算, 不跨清零点累积 */
    if (s_world_reset) {
        prev = enc;
        s_world_reset = 0U;
        return World_position;
    }
    if (first) { first = 0; prev = enc; return World_position; }



    /* 麦轮正解 (右轮与左轮编码器反号, 取反统一: 后退全负/前进全正) */
    d_fwd  = (float)(enc.fl - prev.fl - (enc.fr - prev.fr) +
                     enc.rl - prev.rl - (enc.rr - prev.rr)) / 4.0f;
    d_side = (float)(-(enc.fl - prev.fl) - (enc.fr - prev.fr) +
                     enc.rl - prev.rl + (enc.rr - prev.rr)) / 4.0f;
    prev = enc;

    /* 脉冲→mm */
    Odometry_Apply_Calib(d_fwd, d_side, &fwd_mm, &side_mm);
    World_position.x += (fwd_mm * cosf(yaw) - side_mm * sinf(yaw)) / 2000.0f;
    World_position.y += (fwd_mm * sinf(yaw) + side_mm * cosf(yaw)) / 2000.0f;
    World_position.yaw = yaw;
    // printf("2431:%.2f,%.2f,%.2f,%.2f\r\n",(float)enc.rl,(float)enc.rr,(float)enc.fl,(float)enc.fr);

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
 * 现在定位只用上面的编码器里程计 World_position_get(), 航向取自
 * hwt_imu.h 的 g_hwt_imu_yaw_rad。
 * ============================================================ */
