/**
 * siyuan_imu.c — 高级姿态解算 (带碰撞屏蔽/误差限幅/在线零偏估计)
 */

#include "../hardware/Common_used.h"
#include "siyuan_imu.h"

/* ============================================================
 * 全局状态
 * ============================================================ */

static SiyuanAHRS_t siyuan_ahrs;

float siyuan_roll  = 0.0f;
float siyuan_pitch = 0.0f;
float siyuan_yaw   = 0.0f;

float siyuan_gyro_z_bias = 0.0f;
float siyuan_gyro_z_rate = 0.0f;

/* ============================================================
 * 安全倒数平方根
 * ============================================================ */

static float inv_sqrt_safe(float x)
{
    if (x <= 1e-12f) return 0.0f;
    return 1.0f / sqrtf(x);
}

/* ============================================================
 * 初始化
 * ============================================================ */

void siyuan_ahrs_init(void)
{
    siyuan_ahrs.q0 = 1.0f;
    siyuan_ahrs.q1 = 0.0f;
    siyuan_ahrs.q2 = 0.0f;
    siyuan_ahrs.q3 = 0.0f;

    siyuan_ahrs.integralFBx = 0.0f;
    siyuan_ahrs.integralFBy = 0.0f;
    siyuan_ahrs.integralFBz = 0.0f;

    siyuan_ahrs.twoKp = 2.0f * SIYUAN_KP_DEFAULT;
    siyuan_ahrs.twoKi = 2.0f * SIYUAN_KI_DEFAULT;
}

/* ============================================================
 * Mahony 互补滤波更新 (增强版)
 *
 * + 加速度模长拒绝 (防碰撞)
 * + 误差限幅 (ERR_LIMIT)
 * + 逐轴 Kp/Ki 缩放
 * + 积分限幅
 * ============================================================ */

void siyuan_ahrs_update(SiyuanAHRS_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float qa, qb, qc;
    float accNormSq, accNorm;
    float qNormSq;

    static int accRejectCnt = 0;

    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;

    if (dt <= 0.0f || dt > 0.05f) return;

    /* ---- 加速度模长检测 ---- */
    accNormSq = ax * ax + ay * ay + az * az;
    accNorm = sqrtf(accNormSq);

    if (accNorm < ACC_NORM_MIN || accNorm > ACC_NORM_MAX)
        accRejectCnt = ACC_REJECT_COUNT;

    /* ---- 加速度修正 (仅在数据有效时) ---- */
    if (accRejectCnt > 0)
    {
        accRejectCnt--;
    }
    else if (accNormSq > 1e-6f)
    {
        recipNorm = inv_sqrt_safe(accNormSq);
        if (recipNorm > 0.0f)
        {
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            /* 重力方向 */
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            /* 误差 (叉积) */
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            /* 误差限幅 */
            if (halfex > ERR_LIMIT) halfex = ERR_LIMIT;
            if (halfex < -ERR_LIMIT) halfex = -ERR_LIMIT;
            if (halfey > ERR_LIMIT) halfey = ERR_LIMIT;
            if (halfey < -ERR_LIMIT) halfey = -ERR_LIMIT;
            if (halfez > ERR_LIMIT) halfez = ERR_LIMIT;
            if (halfez < -ERR_LIMIT) halfez = -ERR_LIMIT;

            /* 积分项 (逐轴增益) */
            if (ahrs->twoKi > 0.0f)
            {
                ahrs->integralFBx += ahrs->twoKi * KI_X_SCALE * halfex * dt;
                ahrs->integralFBy += ahrs->twoKi * KI_Y_SCALE * halfey * dt;
                ahrs->integralFBz += ahrs->twoKi * KI_Z_SCALE * halfez * dt;

                if (ahrs->integralFBx > INTEGRAL_LIMIT) ahrs->integralFBx = INTEGRAL_LIMIT;
                if (ahrs->integralFBx < -INTEGRAL_LIMIT) ahrs->integralFBx = -INTEGRAL_LIMIT;
                if (ahrs->integralFBy > INTEGRAL_LIMIT) ahrs->integralFBy = INTEGRAL_LIMIT;
                if (ahrs->integralFBy < -INTEGRAL_LIMIT) ahrs->integralFBy = -INTEGRAL_LIMIT;
                if (ahrs->integralFBz > INTEGRAL_LIMIT) ahrs->integralFBz = INTEGRAL_LIMIT;
                if (ahrs->integralFBz < -INTEGRAL_LIMIT) ahrs->integralFBz = -INTEGRAL_LIMIT;

                gx += ahrs->integralFBx;
                gy += ahrs->integralFBy;
                gz += ahrs->integralFBz;
            }
            else
            {
                ahrs->integralFBx = 0.0f;
                ahrs->integralFBy = 0.0f;
                ahrs->integralFBz = 0.0f;
            }

            /* 比例项 (逐轴增益) */
            gx += ahrs->twoKp * KP_X_SCALE * halfex;
            gy += ahrs->twoKp * KP_Y_SCALE * halfey;
            gz += ahrs->twoKp * KP_Z_SCALE * halfez;
        }
    }

    /* ---- 四元数微分方程 ---- */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    /* ---- 归一化 ---- */
    qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    recipNorm = inv_sqrt_safe(qNormSq);
    if (recipNorm <= 0.0f) return;

    ahrs->q0 = q0 * recipNorm;
    ahrs->q1 = q1 * recipNorm;
    ahrs->q2 = q2 * recipNorm;
    ahrs->q3 = q3 * recipNorm;
}

/* ============================================================
 * 四元数 → 欧拉角
 * ============================================================ */

void siyuan_quat_to_euler(float q0, float q1, float q2, float q3,
                          float *roll, float *pitch, float *yaw)
{
    float sinp;

    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2));

    sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f) sinp = 1.0f;
    if (sinp <= -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);

    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3)) * SIYUAN_YAW_SCALE;  /* 比例标度校准 */
}

/* ============================================================
 * 姿态四元数导出
 * ============================================================ */

void siyuan_get_quat(float q[4])
{
    if (!q) return;
    q[0] = siyuan_ahrs.q0;
    q[1] = siyuan_ahrs.q1;
    q[2] = siyuan_ahrs.q2;
    q[3] = siyuan_ahrs.q3;
}

/* ============================================================
 * 陀螺仪 Z 轴零偏校准 (静止时调用)
 * ============================================================ */

void siyuan_gyro_calibrate(void)
{
    float sum = 0.0f;

    for (int i = 0; i < GYRO_CALIB_NUM; i++)
    {
        imu660ra_get_gyro();

        /* Body Z = -Sensor X */
        float raw_gyro_z = -imu660ra_gyro_transition(imu660ra_gyro_x);

        sum += raw_gyro_z;
        HAL_Delay(5);
    }

    siyuan_gyro_z_bias = sum / GYRO_CALIB_NUM;
}

/* ============================================================
 * 陀螺仪 Z 轴低通滤波 (去零偏 + LPF)
 * ============================================================ */

float siyuan_update_gyro_rate(float gz)
{
    float raw = gz - siyuan_gyro_z_bias;

    siyuan_gyro_z_rate = GYRO_RATE_LPF_ALPHA * raw +
                         (1.0f - GYRO_RATE_LPF_ALPHA) * siyuan_gyro_z_rate;

    return siyuan_gyro_z_rate;
}

/* ============================================================
 * 姿态更新 (一步到位)
 *
 * 读传感器 → 坐标映射 → Z轴滤波 → 静止检测 →
 * 在线零偏估计 → Mahony 解算 → 输出欧拉角
 * ============================================================ */

void siyuan_degree_update(float *yaw, float *pitch, float *roll, float dt)
{
    float ax, ay, az;
    float gx, gy, gz;

    static float gz_bias_online = 0.0f;
    static float yaw_hold       = 0.0f;

    /* 1. 读传感器 + 坐标映射 */
    imu660ra_get_acc();
    imu660ra_get_gyro();

    ax =  imu660ra_acc_transition(imu660ra_acc_z);
    ay =  imu660ra_acc_transition(imu660ra_acc_y);
    az = -imu660ra_acc_transition(imu660ra_acc_x);

    gx =  imu660ra_gyro_transition(imu660ra_gyro_z);
    gy =  imu660ra_gyro_transition(imu660ra_gyro_y);
    gz = -imu660ra_gyro_transition(imu660ra_gyro_x);

    /* 2. Z 轴滤波 */
    gz = siyuan_update_gyro_rate(gz);

    /* 3. dps → rad/s */
    float gx_rad = gx * DEG_TO_RAD;
    float gy_rad = gy * DEG_TO_RAD;
    float gz_rad = gz * DEG_TO_RAD;

    /* 4. 在线零偏补偿 (先减后判, 关键顺序!) */
    gz_rad -= gz_bias_online;

    /* 5. 静止检测 — 补偿后仍接近零 → 抑制积分 + 微调零偏 */
    if (fabsf(gx_rad) < GYRO_STATIC_THRESH &&
        fabsf(gy_rad) < GYRO_STATIC_THRESH &&
        fabsf(gz_rad) < GYRO_STATIC_THRESH)
    {
        gz_bias_online += gz_rad * 0.003f;  /* 微调残余 (加快静止收敛) */
        gz_rad = 0.0f;
    }

    /* 6. Mahony 解算 (实测 dt) */
    siyuan_ahrs_update(&siyuan_ahrs,
                       gx_rad, gy_rad, gz_rad,
                       ax, ay, az,
                       dt);

    /* 7. 四元数 → 欧拉角 */
    siyuan_quat_to_euler(siyuan_ahrs.q0, siyuan_ahrs.q1,
                         siyuan_ahrs.q2, siyuan_ahrs.q3,
                         roll, pitch, yaw);

    /* 8. 静止时保持 yaw — 必须三轴都静止才锁值。
     *    若只看 gx/gy, 原地偏航时 gz 大而 gx/gy≈0 会误判静止,
     *    导致 yaw 被冻结成阶梯状, 角度环来回抖。 */
    if (fabsf(gx_rad) < GYRO_STATIC_THRESH &&
        fabsf(gy_rad) < GYRO_STATIC_THRESH &&
        fabsf(gz_rad) < GYRO_STATIC_THRESH)
    {
        *yaw = yaw_hold;
    }
    else
    {
        yaw_hold = *yaw;
    }
}

/* ============================================================
 * IMU 任务 (周期 200Hz 调用)
 * ============================================================ */

void siyuan_imu_task(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    float dt;

    if (last_tick == 0) {
        last_tick = now;
    }
    dt = (float)(now - last_tick) / 1000.0f;
    if (dt <= 0.0f) dt = 0.005f;
    if (dt > 0.05f) dt = 0.05f;   /* 防任务卡顿导致 dt 异常 */
    last_tick = now;

    siyuan_degree_update(&siyuan_yaw, &siyuan_pitch, &siyuan_roll, dt);
}
