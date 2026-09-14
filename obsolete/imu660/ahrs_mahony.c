#include "ahrs_mahony.h"
#include "../hardware/imu660.h"
#include "main.h"

/* ============================================================
 * 陀螺仪零偏 (rad/s)
 * ============================================================ */

float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;
EulerAngle e ;

/* ============================================================
 * 姿态解算内部状态
 * ============================================================ */

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

/* ============================================================
 * 快速倒数平方根
 * ============================================================ */

static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* ============================================================
 * 校准陀螺仪零偏 (传感器必须静止)
 *
 * 内部读取 IMU660RA 陀螺仪原始数据,
 * 取平均后转为 rad/s 存入 gyro_bias_x/y/z
 * ============================================================ */

void calibrate_gyro(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    /* 丢弃前 100 个不稳定点 */
    for (int i = 0; i < 100; i++) {
        imu660ra_get_gyro();
        HAL_Delay(5);
    }

    /* 采集 2000 个样本 (约 6s) */
    const int samples = 2000;
    for (int i = 0; i < samples; i++) {
        imu660ra_get_gyro();

        sum_x += imu660ra_gyro_transition(imu660ra_gyro_x);
        sum_y += imu660ra_gyro_transition(imu660ra_gyro_y);
        sum_z += imu660ra_gyro_transition(imu660ra_gyro_z);

        HAL_Delay(3);
    }

    gyro_bias_x = sum_x / samples * DEG_TO_RAD;
    gyro_bias_y = sum_y / samples * DEG_TO_RAD;
    gyro_bias_z = sum_z / samples * DEG_TO_RAD;
}

/* ============================================================
 * 初始化姿态解算
 * ============================================================ */

void MahonyAHRS_Init(void)
{
    float ax, ay, az;
    float roll, pitch;
    float cr, sr, cp, sp;

    /* ---- 用重力加速度初始化 Roll / Pitch ---- */
    imu660ra_get_acc();

    /* 坐标轴映射 (同 MahonyAHRS_Update) */
    ax =  imu660ra_acc_transition(imu660ra_acc_z);
    ay =  imu660ra_acc_transition(imu660ra_acc_y);
    az = -imu660ra_acc_transition(imu660ra_acc_x);

    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    /* roll/pitch → 四元数 (yaw=0) */
    cr = cosf(roll  * 0.5f);
    sr = sinf(roll  * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    /* 积分项预加载零偏 (传感器坐标 → 体坐标)
     * Body X ← Sensor Z  →  integralFBx = -gyro_bias_z
     * Body Y ← Sensor Y  →  integralFBy = -gyro_bias_y
     * Body Z ← -Sensor X →  integralFBz = +gyro_bias_x
     */
    integralFBx = -gyro_bias_z;
    integralFBy = -gyro_bias_y;
    integralFBz = +gyro_bias_x;
}

/* ============================================================
 * Mahony 互补滤波更新
 *
 * 内部自动:
 *   1. 读取 IMU660RA 加速度 + 陀螺仪
 *   2. 坐标轴映射 (安装方向适配)
 *   3. 六轴 Mahony 互补滤波
 *   4. 更新内部四元数
 *
 * 参数: dt = 采样间隔 (秒)
 * ============================================================ */

void MahonyAHRS_Update(float dt)
{
    float ax, ay, az;
    float gx, gy, gz;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT;

    if (dt <= 0.0f) return;

    /* ---- 读取传感器 ---- */
    imu660ra_get_acc();
    imu660ra_get_gyro();

    /* ---- 坐标轴映射 (安装方向适配) ----
     * Body X = Sensor Z
     * Body Y = Sensor Y
     * Body Z = -Sensor X
     */
    ax =  imu660ra_acc_transition(imu660ra_acc_z);
    ay =  imu660ra_acc_transition(imu660ra_acc_y);
    az = -imu660ra_acc_transition(imu660ra_acc_x);

    gx =  imu660ra_gyro_transition(imu660ra_gyro_z) * DEG_TO_RAD;
    gy =  imu660ra_gyro_transition(imu660ra_gyro_y) * DEG_TO_RAD;
    gz = -imu660ra_gyro_transition(imu660ra_gyro_x) * DEG_TO_RAD;

    halfT = 0.5f * dt;

    /* ---- 加速度计归一化 ---- */
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.00001f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    /* ---- 由四元数推算重力方向 ---- */
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* ---- 重力方向误差 (叉积) ---- */
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    /* ---- 积分误差 (消除陀螺仪零偏) ---- */
    integralFBx += MAHONY_KI * ex * dt;
    integralFBy += MAHONY_KI * ey * dt;

    /* ---- 陀螺仪修正 ---- */
    gx += MAHONY_KP * ex + integralFBx;
    gy += MAHONY_KP * ey + integralFBy;
    gz += MAHONY_KP * ez + integralFBz;

    /* ---- 一阶龙格-库塔更新四元数 ---- */
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    /* ---- 四元数归一化 ---- */
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

/* ============================================================
 * 获取四元数
 * ============================================================ */

Quaternion MahonyAHRS_GetQuaternion(void)
{
    Quaternion q;
    q.q0 = q0; q.q1 = q1; q.q2 = q2; q.q3 = q3;
    return q;
}

/* ============================================================
 * 获取欧拉角 (弧度)
 * ============================================================ */

EulerAngle MahonyAHRS_GetEuler(void)
{
    EulerAngle e;
    e.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    e.pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    e.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    return e;
}

/* ============================================================
 * 获取欧拉角 (度)
 * ============================================================ */

EulerAngle MahonyAHRS_GetEuler_deg(void)
{
    e = MahonyAHRS_GetEuler();
    e.roll  *= RAD_TO_DEG;
    e.pitch *= RAD_TO_DEG;
    e.yaw   *= RAD_TO_DEG;
    return e;
}
