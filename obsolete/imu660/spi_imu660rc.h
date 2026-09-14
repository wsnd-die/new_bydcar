#ifndef INC_33333_SPI_IMU660RC_H
#define INC_33333_SPI_IMU660RC_H

#include "main.h"

// ----------------------- 引脚定义 -----------------------
#define SPI_IMU660RC_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define SPI_IMU660RC_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define SPI_IMU660RC_PORT       GPIOB
#define SPI_IMU660RC_SCLK_PIN   GPIO_PIN_13
#define SPI_IMU660RC_MISO_PIN   GPIO_PIN_14
#define SPI_IMU660RC_MOSI_PIN   GPIO_PIN_15

// ----------------------- 寄存器地址 -----------------------
#define IMU660RC_FUNC_CFG_ACCESS    ( 0x01 )
#define IMU660RC_INT2_CTRL          ( 0x0E )
#define IMU660RC_CHIP_ID            ( 0x0F )
#define IMU660RC_CTRL1              ( 0x10 )
#define IMU660RC_CTRL2              ( 0x11 )
#define IMU660RC_CTRL3              ( 0x12 )
#define IMU660RC_CTRL4              ( 0x13 )
#define IMU660RC_CTRL5              ( 0x14 )
#define IMU660RC_CTRL6              ( 0x15 )
#define IMU660RC_CTRL7              ( 0x16 )
#define IMU660RC_CTRL8              ( 0x17 )
#define IMU660RC_CTRL9              ( 0x18 )
#define IMU660RC_CTRL10             ( 0x19 )
#define IMU660RC_CTRL_STATUS        ( 0x1A )
#define IMU660RC_STATUS_REG         ( 0x1E )
#define IMU660RC_OUT_TEMP_L         ( 0x20 )
#define IMU660RC_OUT_TEMP_H         ( 0x21 )
#define IMU660RC_OUTX_L_G           ( 0x22 )
#define IMU660RC_OUTX_H_G           ( 0x23 )
#define IMU660RC_OUTY_L_G           ( 0x24 )
#define IMU660RC_OUTY_H_G           ( 0x25 )
#define IMU660RC_OUTZ_L_G           ( 0x26 )
#define IMU660RC_OUTZ_H_G           ( 0x27 )
#define IMU660RC_OUTX_L_A           ( 0x28 )
#define IMU660RC_OUTX_H_A           ( 0x29 )
#define IMU660RC_OUTY_L_A           ( 0x2A )
#define IMU660RC_OUTY_H_A           ( 0x2B )
#define IMU660RC_OUTZ_L_A           ( 0x2C )
#define IMU660RC_OUTZ_H_A           ( 0x2D )

#define IMU660RC_PAGE_SEL           ( 0x02 )
#define IMU660RC_EMB_FUNC_EN_A      ( 0x04 )
#define IMU660RC_PAGE_RW            ( 0x17 )
#define IMU660RC_SFLP_ODR           ( 0x5E )
#define IMU660RC_EMB_FUNC_CFG       ( 0x63 )

// ----------------------- 量程宏定义 -----------------------
// 加速度计量程（CTRL1_XL 的 FS_XL[1:0]）
#define ACCEL_FS_2G         0x00
#define ACCEL_FS_4G         0x02
#define ACCEL_FS_8G         0x03
#define ACCEL_FS_16G        0x01

// 陀螺仪量程（CTRL6_C 的 FS_G[3:0]，位于 bit7~4）
#define GYRO_FS_125DPS      0x00
#define GYRO_FS_250DPS      0x10    // 0001 0000
#define GYRO_FS_500DPS      0x20
#define GYRO_FS_1000DPS     0x30
#define GYRO_FS_2000DPS     0x40
#define GYRO_FS_4000DPS     0xC0    // 特殊值 1100 0000

// 默认量程
#define IMU660RC_ACC_FS_DEFAULT   ACCEL_FS_16G
#define IMU660RC_GYRO_FS_DEFAULT  GYRO_FS_4000DPS

// ----------------------- 数据结构 -----------------------
typedef struct {
    uint8_t acc_fs;     // 当前加速度计量程配置值
    uint8_t gyro_fs;    // 当前陀螺仪量程配置值
    float   acc_sensitivity;   // 加速度系数（4096 对应 ±8g）
    float   gyro_sensitivity;  // 陀螺仪系数（16.384 对应 ±2000dps）
} IMU660RC_ConfigType;

typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    float   ax, ay, az;      // 加速度 g 值
    float   gx, gy, gz;      // 角速度 dps
    float   roll, pitch, yaw;// 欧拉角（度）
    float   q[4];            // 四元数
} IMU660RC_DataType;
/* 姿态解算专用变量 */
static float att_q[4] = {1, 0, 0, 0};    // 四元数
static float integral_fb[3] = {0, 0, 0}; // 积分项（用于 PI 中的 I）
static uint32_t last_tick = 0;           // 用于计算 dt
static float gyro_bias[3] = {0, 0, 0};
/* 陀螺仪零偏（度/秒） */
static float Rc_gyro_bias_x = 0, Rc_gyro_bias_y = 0, Rc_gyro_bias_z = 0;

/* 滤波用上一帧角度 */
static float roll_last = 0, pitch_last = 0, yaw_last = 0;

extern IMU660RC_ConfigType imu_cfg;
extern IMU660RC_DataType  imu_data;

// ----------------------- 函数声明 -----------------------
uint8_t IMU660RC_ReadRegs(uint8_t reg);
void    IMU660RC_WriteRegs(uint8_t reg, uint8_t data);
uint16_t    IMU660RC_ReadReg16b(uint8_t reg_low);
void IMU660RC_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint8_t len);

void    IMU660RC_Init(void);           // 普通初始化
void    IMU660RC_Init_SFLP(void);      // 开启内置姿态解算
void    IMU660RC_ReadAcc(void);
void    IMU660RC_ReadGyro(void);
void    IMU660RC_ReadEuler(void);      // 从 SFLP 读取欧拉角
void    IMU660RC_ReadQuat(void);       // 读取四元数
void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw);
void IMU660RC_AttitudeUpdate(float dt);
void IMU660RC_AttitudeInit(void);

static uint32_t fp16_to_float(uint16_t h);
#endif