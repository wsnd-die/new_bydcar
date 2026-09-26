#ifndef __MECANUM_H
#define __MECANUM_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 麦轮底盘几何参数 ======================== */
#define MEC_WHEELBASE       0.181f   /* 轴距 m（前后轮中心距）*/
#define MEC_TRACK_WIDTH     0.164f   /* 轮距 m（左右轮中心距）*/
#define MEC_WHEEL_RADIUS    3.75f  /* 轮子半径 cm（=37.5mm，与 Mecanum_Move.c 的 wheel_radius_m=0.0375 一致）*/
#define MEC_SPEED_COEFF     224.058f /* 速度换算系数 (m/s → RPM) */
#define MEC_STOP_THRESHOLD  1e-3f    /* 静止判断阈值 */
#define MEC_LOW_SPEED_LIMIT 0.20f    /* 低速阈值 m/s */
#define MEC_LOW_SPEED_GAIN  1.0f     /* 低速放大系数 */
#define MEC_MIN_MOTOR_SPEED 5.0f     /* 最小启动转速 */
/* ======================== 麦轮解算结果结构体 ======================== */
typedef struct {
    uint16_t fl_speed;      /* 前左轮转速 (0-65535 RPM) */
    uint16_t fr_speed;      /* 前右轮转速 (0-65535 RPM) */
    uint16_t rl_speed;      /* 后左轮转速 (0-65535 RPM) */
    uint16_t rr_speed;      /* 后右轮转速 (0-65535 RPM) */
    uint8_t  fl_dir;        /* 前左轮方向: 1=CW正转, 0=CCW反转 */
    uint8_t  fr_dir;        /* 前右轮方向: 1=CW正转, 0=CCW反转 */
    uint8_t  rl_dir;        /* 后左轮方向: 1=CW正转, 0=CCW反转 */
    uint8_t  rr_dir;        /* 后右轮方向: 1=CW正转, 0=CCW反转 */
} MecanumResult;

/* ======================== 函数声明 ======================== */

/**
  * @brief  麦轮逆运动学解算（单轴模型: V + ω）
  * @param  v : 机器人线速度 (m/s)，前进为正
  * @param  w : 机器人角速度 (rad/s)，逆时针为正
  * @retval MecanumResult 四个轮子的转速和方向
  */
MecanumResult Mecanum_Calc(float v, float w);

/**
  * @brief  麦轮全向逆运动学解算（三自由度: Vx + Vy + ω）
  * @param  vx : X轴线速度 (m/s)，前进为正
  * @param  vy : Y轴线速度 (m/s)，左移为正
  * @param  w  : 角速度 (rad/s)，逆时针为正
  * @retval MecanumResult 四个轮子的转速和方向
  */
MecanumResult Mecanum_Calc_Full(float vx, float vy, float w);

uint32_t malu_cm_topluse_s(float cm);

/* ======================== 编码器 ======================== */
typedef struct {
    int32_t fl, fr, rl, rr;   /* 四轮编码器累计值 */
} EncoderData;

uint8_t Mecanum_Read_Speed(uint8_t id, int16_t *rpm, uint32_t timeout_ms);
uint8_t Mecanum_Read_Position(uint8_t id, int32_t *pos, uint32_t timeout_ms);
uint8_t Mecanum_Read_AllPositions(EncoderData *enc, uint32_t timeout_ms);

/* ============ 编码器脉冲 → 毫米 ============ */

/**
  * @brief  编码器增量(脉冲) → 实际位移(mm)
  * @note   唯一的换算口，device/drv_wheel_odom.c 每次推算都经此。
  *         V1.14.0 起只有粗略估算一条路径（R=3.75cm, 3200脉冲/圈），
  *         原 TBOP 标定分支已随标定整块移除。
  */
void Odometry_Apply_Calib(float enc_dx, float enc_dy, float *mm_x, float *mm_y);

#ifdef __cplusplus
}
#endif

#endif