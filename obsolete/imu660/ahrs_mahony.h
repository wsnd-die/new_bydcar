#ifndef __AHRS_MAHONY_H__
#define __AHRS_MAHONY_H__

#include <math.h>

/* Mahony filter gains (可调参) */
#define MAHONY_KP   2.0f
#define MAHONY_KI   0.03f

/* 角度转换常量 */
#define DEG_TO_RAD  0.01745329251994329577f
#define RAD_TO_DEG  57.295779513082320876f

/* 四元数 */
typedef struct {
    float q0, q1, q2, q3;
} Quaternion;

/* 欧拉角 (rad 或 deg) */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle;

/* 陀螺仪零偏 (rad/s), 由 calibrate_gyro() 填入 */
extern float gyro_bias_x;
extern float gyro_bias_y;
extern float gyro_bias_z;
extern EulerAngle e;

/* ============================================================
 * API
 * ============================================================ */

/* 校准陀螺仪零偏 (传感器必须静止) */
void calibrate_gyro(void);

/* 初始化姿态解算, 加载零偏到积分项 */
void MahonyAHRS_Init(void);

/* Mahony 互补滤波更新 (内部读取 IMU660RA + 坐标映射)
 * dt: 采样周期 (秒)
 * 结果通过 MahonyAHRS_GetEuler / MahonyAHRS_GetQuaternion 获取
 */
void MahonyAHRS_Update(float dt);

Quaternion MahonyAHRS_GetQuaternion(void);
EulerAngle MahonyAHRS_GetEuler(void);
EulerAngle MahonyAHRS_GetEuler_deg(void);

#endif
