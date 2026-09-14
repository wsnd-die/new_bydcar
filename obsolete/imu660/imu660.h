#ifndef __IMU660RA_H__
#define __IMU660RA_H__

#include "main.h"
#include <stdint.h>
#include <math.h>

/* ============================================================
 * BMI270 / IMU660RA Register
 * ============================================================ */

#define IMU660RA_SPI_W              0x00
#define IMU660RA_SPI_R              0x80

#define IMU660RA_CHIP_ID            0x00
#define IMU660RA_ERR_REG            0x02
#define IMU660RA_STATUS             0x03

#define IMU660RA_ACC_ADDRESS        0x0C
#define IMU660RA_GYRO_ADDRESS       0x12

#define IMU660RA_INTERNAL_STATUS    0x21

#define IMU660RA_ACC_CONF           0x40
#define IMU660RA_ACC_RANGE          0x41
#define IMU660RA_GYR_CONF           0x42
#define IMU660RA_GYR_RANGE          0x43

#define IMU660RA_INIT_CTRL          0x59
#define IMU660RA_INIT_ADDR_0        0x5B
#define IMU660RA_INIT_ADDR_1        0x5C
#define IMU660RA_INIT_DATA          0x5E
#define IMU660RA_INTERNAL_ERROR     0x5F

#define IMU660RA_PWR_CONF           0x7C
#define IMU660RA_PWR_CTRL           0x7D

extern const unsigned char imu660ra_config_file[8192];
/* ============================================================
 * 软件 CS
 *
 * 根据你的 CubeMX GPIO 名称修改
 * ============================================================ */

#define IMU660RA_CS_LOW() \
    HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin, GPIO_PIN_RESET)

#define IMU660RA_CS_HIGH() \
    HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin, GPIO_PIN_SET)


/* ============================================================
 * 原始数据
 * ============================================================ */

extern int16_t imu660ra_acc_x;
extern int16_t imu660ra_acc_y;
extern int16_t imu660ra_acc_z;

extern int16_t imu660ra_gyro_x;
extern int16_t imu660ra_gyro_y;
extern int16_t imu660ra_gyro_z;


/* 转换系数 */
extern float imu660ra_transition_factor[2];


/* ============================================================
 * 姿态角
 * 单位：degree
 * ============================================================ */

extern float imu660ra_roll;
extern float imu660ra_pitch;
extern float imu660ra_yaw;


/* 四元数 */
extern float imu660ra_q[4];


/* ============================================================
 * 转换宏
 * ============================================================ */

#define imu660ra_acc_transition(acc_value) \
    ((float)(acc_value) / imu660ra_transition_factor[0])

#define imu660ra_gyro_transition(gyro_value) \
    ((float)(gyro_value) / imu660ra_transition_factor[1])


/* ============================================================
 * IMU660RA 基础驱动
 * ============================================================ */

void    imu660ra_get_acc(void);
void    imu660ra_get_gyro(void);

/*
 * 返回：
 * 1 = 初始化成功
 * 0 = 初始化失败
 */
uint8_t imu660ra_init(void);


/* ============================================================
 * SPI 底层通信
 * ============================================================ */

uint8_t IMU660RA_ReadReg(uint8_t reg);

void IMU660RA_WriteReg(uint8_t reg, uint8_t data);

uint16_t IMU660RA_ReadReg16b(uint8_t reg_low);

void IMU660RA_ReadMulti(uint8_t reg,
                        uint8_t *buf,
                        uint8_t len);


/* ============================================================
 * 姿态解算 Mahony
 * ============================================================ */

void quat_to_euler(float q[4],
                   float *roll,
                   float *pitch,
                   float *yaw);

void IMU660RA_AttitudeUpdate(float dt);

void IMU660RA_AttitudeInit(void);


#endif
