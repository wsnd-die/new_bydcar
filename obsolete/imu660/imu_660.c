#include "imu660.h"
#include "spi.h"

#include <stdio.h>
#include <math.h>


extern SPI_HandleTypeDef hspi2;

#include "Common_used.h"
/* ============================================================
 * 原始数据
 * ============================================================ */

int16_t imu660ra_acc_x = 0;
int16_t imu660ra_acc_y = 0;
int16_t imu660ra_acc_z = 0;

int16_t imu660ra_gyro_x = 0;
int16_t imu660ra_gyro_y = 0;
int16_t imu660ra_gyro_z = 0;


/*
 * 默认：
 *
 * ACC ±8g
 * 4096 LSB/g
 *
 * GYRO ±2000 dps
 * 16.384 LSB/(deg/s)
 */
float imu660ra_transition_factor[2] =
{
    4096.0f,
    16.384f
};


/* ============================================================
 * 姿态
 * ============================================================ */

float imu660ra_roll  = 0.0f;
float imu660ra_pitch = 0.0f;
float imu660ra_yaw   = 0.0f;


/*
 * quaternion
 *
 * q[0] = w
 * q[1] = x
 * q[2] = y
 * q[3] = z
 */
float imu660ra_q[4] =
{
    1.0f,
    0.0f,
    0.0f,
    0.0f
};


/* ============================================================
 * Mahony 参数
 * ============================================================ */

static float mahony_integral_x = 0.0f;
static float mahony_integral_y = 0.0f;
static float mahony_integral_z = 0.0f;


/*
 * Kp 越大：
 * 加速度修正越强，响应越快
 *
 * Ki：
 * 用于补偿 gyro bias
 *
 * 可以后期调参
 */
#define MAHONY_KP        2.0f
#define MAHONY_KI        0.02f

#define DEG_TO_RAD       0.01745329251994329577f
#define RAD_TO_DEG       57.295779513082320876f


/* ============================================================
 * SPI timeout
 * ============================================================ */


#define IMU660RA_CONFIG_TIMEOUT   100

/******************************************************************************
 *                          SPI 底层通信函数
 *
 *  关键：BMI270/IMU660RA 的 SPI 读取需要分步进行（CS 全程拉低）：
 *    1. 发送地址字节
 *    2. 发送 dummy 字节，接收 dummy → 丢弃
 *    3. 发送 dummy 字节，接收 data → 保存
 *  不能把地址+数据合成一次连续的 16 时钟传输。芯片需要间隙准备数据。
 ******************************************************************************/

#define IMU660RA_SPI_TIMEOUT  100U

/**
 * @brief  写 IMU660RA 单个寄存器（CS 内分两步：地址→数据）
 */
/* ============================================================
 * 写单寄存器
 * ============================================================ */

void IMU660RA_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];

    tx[0] = reg | IMU660RA_SPI_W;
    tx[1] = data;

    IMU660RA_CS_LOW();

    HAL_SPI_Transmit(
        &hspi2,
        tx,
        2,
        IMU660RA_SPI_TIMEOUT
    );

    IMU660RA_CS_HIGH();
}

/**
 * @brief  写 IMU660RA 多个寄存器（CS 内：地址→data[]）
 */
static void imu660ra_write_registers(uint8_t reg, const uint8_t *data, uint32_t len)
{
    uint8_t addr = reg | IMU660RA_SPI_W;

    IMU660RA_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &addr, 1, IMU660RA_SPI_TIMEOUT);
    if (data != NULL && len > 0) {
        HAL_SPI_Transmit(&hspi2, (uint8_t *)data, (uint16_t)len, IMU660RA_SPI_TIMEOUT);
    }
    IMU660RA_CS_HIGH();
}

/* ============================================================
 * 读单寄存器
 *
 * BMI270 SPI read:
 *
 * Address
 * Dummy
 * Data
 *
 * 第一个返回 byte 必须丢掉
 * ============================================================ */

uint8_t IMU660RA_ReadReg(uint8_t reg)
{
    uint8_t addr;

    uint8_t tx_dummy = 0xFF;

    uint8_t rx_dummy = 0;
    uint8_t data = 0;

    addr = reg | IMU660RA_SPI_R;

    IMU660RA_CS_LOW();

    /* 发送寄存器地址 */
    HAL_SPI_Transmit(
        &hspi2,
        &addr,
        1,
        IMU660RA_SPI_TIMEOUT
    );

    /* BMI270 dummy byte */
    HAL_SPI_TransmitReceive(
        &hspi2,
        &tx_dummy,
        &rx_dummy,
        1,
        IMU660RA_SPI_TIMEOUT
    );

    /* 真正的数据 */
    HAL_SPI_TransmitReceive(
        &hspi2,
        &tx_dummy,
        &data,
        1,
        IMU660RA_SPI_TIMEOUT
    );

    IMU660RA_CS_HIGH();

    return data;
}


/* ============================================================
 * 读取连续两个寄存器
 *
 * BMI270 sensor data:
 * low byte first
 * ============================================================ */

uint16_t IMU660RA_ReadReg16b(uint8_t reg_low)
{
    uint8_t data[2];

    IMU660RA_ReadMulti(
        reg_low,
        data,
        2
    );

    return ((uint16_t)data[1] << 8) |
           ((uint16_t)data[0]);
}

/* ============================================================
 * 连续读取寄存器
 * ============================================================ */

void IMU660RA_ReadMulti(uint8_t reg,
                        uint8_t *buf,
                        uint8_t len)
{
    uint8_t addr;
    uint8_t dummy_tx = 0xFF;
    uint8_t dummy_rx;

    uint8_t i;

    if(buf == NULL || len == 0)
    {
        return;
    }

    addr = reg | IMU660RA_SPI_R;

    IMU660RA_CS_LOW();

    /* 发送起始地址 */
    HAL_SPI_Transmit(
        &hspi2,
        &addr,
        1,
        IMU660RA_SPI_TIMEOUT
    );

    /*
     * BMI270 SPI 第一个返回 byte 是 dummy
     */
    HAL_SPI_TransmitReceive(
        &hspi2,
        &dummy_tx,
        &dummy_rx,
        1,
        IMU660RA_SPI_TIMEOUT
    );

    /*
     * 真正数据
     */
    for(i = 0; i < len; i++)
    {
        HAL_SPI_TransmitReceive(
            &hspi2,
            &dummy_tx,
            &buf[i],
            1,
            IMU660RA_SPI_TIMEOUT
        );
    }

    IMU660RA_CS_HIGH();
}


/******************************************************************************
 *                       IMU660RA 传感器驱动函数
 ******************************************************************************/

static HAL_StatusTypeDef imu660ra_load_config(void)
{
    uint8_t addr;
    HAL_StatusTypeDef status;

    addr = IMU660RA_INIT_DATA | IMU660RA_SPI_W;

    IMU660RA_CS_LOW();

    /* 地址 */
    status = HAL_SPI_Transmit(
        &hspi2,
        &addr,
        1,
        100
    );

    if(status != HAL_OK)
    {
        IMU660RA_CS_HIGH();
        return status;
    }

    /* 完整 8KB */
    status = HAL_SPI_Transmit(
        &hspi2,
        (uint8_t *)imu660ra_config_file,
        sizeof(imu660ra_config_file),
        500
    );

    IMU660RA_CS_HIGH();

    return status;
}


uint8_t imu660ra_init(void)
{
    uint8_t id;
    uint8_t int_sta;
    HAL_StatusTypeDef hal_ret;

    printf("\r\n========== IMU660RA INIT ==========\r\n");

    /* -------------------------------------------------
     * 1. CS 空闲高
     * ------------------------------------------------- */
    IMU660RA_CS_HIGH();

    HAL_Delay(20);

    /* -------------------------------------------------
     * 2. 第一次读，用于 I2C -> SPI 切换
     * ------------------------------------------------- */
    (void)IMU660RA_ReadReg(IMU660RA_CHIP_ID);

    HAL_Delay(1);

    /* -------------------------------------------------
     * 3. 正式检查 CHIP_ID
     * ------------------------------------------------- */
    id = IMU660RA_ReadReg(IMU660RA_CHIP_ID);

    printf("CHIP_ID = 0x%02X\r\n", id);

    if(id != 0x24)
    {
        printf("ERROR: CHIP_ID\r\n");
        return 0;
    }

    printf("SPI communication OK\r\n");

    /* -------------------------------------------------
     * 4. 关闭 advanced power save
     * ------------------------------------------------- */
    IMU660RA_WriteReg(IMU660RA_PWR_CONF, 0x00);

    /*
     * 手册要求 >= 450 us
     * HAL_Delay(1) = 1ms
     */
    HAL_Delay(1);

    /* -------------------------------------------------
     * 5. 准备加载初始化配置
     * ------------------------------------------------- */
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x00);

    /* -------------------------------------------------
     * 6. 写完整 8192 字节 config
     * ------------------------------------------------- */
    printf("Loading 8192-byte config...\r\n");

    hal_ret = imu660ra_load_config();

    if(hal_ret != HAL_OK)
    {
        printf("ERROR: config SPI TX failed = %d\r\n",
               (int)hal_ret);

        return 0;
    }

    printf("Config SPI TX complete\r\n");

    /* -------------------------------------------------
     * 7. 完成初始化
     * ------------------------------------------------- */
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x01);

    /* 手册最大约 20ms */
    HAL_Delay(25);

    /* -------------------------------------------------
     * 8. 检查 INTERNAL_STATUS
     * ------------------------------------------------- */
    int_sta = IMU660RA_ReadReg( IMU660RA_INTERNAL_STATUS );

    printf("INTERNAL_STATUS = 0x%02X\r\n", int_sta);

    if((int_sta & 0x01) == 0)
    {
        printf("ERROR: BMI270 config init failed!\r\n");

        return 0;
    }

    printf("BMI270 config init OK\r\n");

    /* -------------------------------------------------
     * 9. 开启 ACC + GYRO + TEMP
     * ------------------------------------------------- */
    IMU660RA_WriteReg(IMU660RA_PWR_CTRL, 0x0E);

    /* ACC 50Hz */
    IMU660RA_WriteReg(IMU660RA_ACC_CONF, 0xA7);

    /* GYRO 200Hz */
    IMU660RA_WriteReg(IMU660RA_GYR_CONF, 0xA9);

    /* ACC ±8g */
    IMU660RA_WriteReg(IMU660RA_ACC_RANGE, 0x02);
    imu660ra_transition_factor[0] = 4096.0f;

    /* GYRO ±2000 dps */
    IMU660RA_WriteReg(IMU660RA_GYR_RANGE, 0x00);
    imu660ra_transition_factor[1] = 16.384f;

    /*
     * Gyro 从 suspend 到 normal 启动时间较长，
     * 给它一点时间
     */
    HAL_Delay(50);

    printf("========== IMU660RA INIT OK ==========\r\n");

    return 1;
}

/* ============================================================
 * 获取加速度计原始值
 * ============================================================ */

void imu660ra_get_acc(void)
{
    uint8_t dat[6];

    IMU660RA_ReadMulti(
        IMU660RA_ACC_ADDRESS,
        dat,
        6
    );

    imu660ra_acc_x =
        (int16_t)(
            ((uint16_t)dat[1] << 8) |
             (uint16_t)dat[0]
        );

    imu660ra_acc_y =
        (int16_t)(
            ((uint16_t)dat[3] << 8) |
             (uint16_t)dat[2]
        );

    imu660ra_acc_z =
        (int16_t)(
            ((uint16_t)dat[5] << 8) |
             (uint16_t)dat[4]
        );
}

/* ============================================================
 * 获取陀螺仪原始值
 * ============================================================ */

void imu660ra_get_gyro(void)
{
    uint8_t dat[6];

    IMU660RA_ReadMulti(
        IMU660RA_GYRO_ADDRESS,
        dat,
        6
    );

    imu660ra_gyro_x =
        -(int16_t)(
            ((uint16_t)dat[1] << 8) |
             (uint16_t)dat[0]
        );

    imu660ra_gyro_y =
        (int16_t)(
            ((uint16_t)dat[3] << 8) |
             (uint16_t)dat[2]
        );

    imu660ra_gyro_z =
        (int16_t)(
            ((uint16_t)dat[5] << 8) |
             (uint16_t)dat[4]
        );
}

/******************************************************************************
 *                         辅助工具函数
 ******************************************************************************/
static float inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static float LimitFloat(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float AbsFloat(float x)
{
    return x >= 0.0f ? x : -x;
}
static float gyro_z_filtered = 0.0f;

float update_gyro_rate(float input)
{
    const float alpha = 0.2f;

    gyro_z_filtered += alpha * (input - gyro_z_filtered);

    return gyro_z_filtered;
}

/**
 * @brief  四元数转欧拉角（ZYX 顺序）
 * @param  q[4]   输入四元数 (q0=qw, q1=qx, q2=qy, q3=qz)
 * @param  roll   输出横滚角（度）
 * @param  pitch  输出俯仰角（度）
 * @param  yaw    输出偏航角（度，0~360）
 */
void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Roll (绕 X 轴)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    // Pitch (绕 Y 轴) — 限制 asinf 入参范围防止数值溢出
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = LimitFloat(sin_pitch, -1.0f, 1.0f);
    *pitch = asinf(sin_pitch) * RAD_TO_DEG;

    // Yaw (绕 Z 轴)
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    // 归一化到 0~360 度
    if (*yaw < 0.0f) {
        *yaw += 360.0f;
    }
}

/******************************************************************************
 *                       Mahony 互补滤波姿态解算
 ******************************************************************************/

void IMU660RA_AttitudeUpdate(float dt)
{
    float ax, ay, az;
    float gx, gy, gz;

    float norm;

    float vx, vy, vz;
    float ex, ey, ez;

    float q0, q1, q2, q3;

    float q_dot0;
    float q_dot1;
    float q_dot2;
    float q_dot3;


    if(dt <= 0.0f)
    {
        return;
    }


    /* ==============================
     * 读取原始数据
     * ============================== */

    imu660ra_get_acc();
    imu660ra_get_gyro();


    /* ==============================
     * 根据实际安装方向重新映射坐标轴
     *
     * Body X = Sensor Z
     * Body Y = Sensor Y
     * Body Z = -Sensor X
     * ============================== */

    ax = imu660ra_acc_transition(
        imu660ra_acc_z
    );

    ay = imu660ra_acc_transition(
        imu660ra_acc_y
    );

    az = -imu660ra_acc_transition(
        imu660ra_acc_x
    );


    gx = imu660ra_gyro_transition(
        imu660ra_gyro_z
    );

    gy = imu660ra_gyro_transition(
        imu660ra_gyro_y
    );

    gz = -imu660ra_gyro_transition(
        imu660ra_gyro_x
    );


    /*
     * 如果你的 update_gyro_rate()
     * 是 Z 轴零偏/滤波函数
     */
    gz = update_gyro_rate(gz);


    /* deg/s -> rad/s */

    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;


    /* ==============================
     * quaternion
     * ============================== */

    q0 = imu660ra_q[0];
    q1 = imu660ra_q[1];
    q2 = imu660ra_q[2];
    q3 = imu660ra_q[3];


    /* ==============================
     * ACC normalization
     * ============================== */

    norm = ax * ax +
           ay * ay +
           az * az;

    if(norm > 0.000001f)
    {
        norm = 1.0f / sqrtf(norm);

        ax *= norm;
        ay *= norm;
        az *= norm;


        /* 当前四元数预测的重力方向 */

        vx = 2.0f *
             (q1 * q3 - q0 * q2);

        vy = 2.0f *
             (q0 * q1 + q2 * q3);

        vz = q0 * q0
           - q1 * q1
           - q2 * q2
           + q3 * q3;


        /* 重力方向误差 */

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;


        /* 积分补偿 */

        if(MAHONY_KI > 0.0f)
        {
            mahony_integral_x +=
                MAHONY_KI * ex * dt;

            mahony_integral_y +=
                MAHONY_KI * ey * dt;

            mahony_integral_z +=
                MAHONY_KI * ez * dt;


            gx += mahony_integral_x;
            gy += mahony_integral_y;
            gz += mahony_integral_z;
        }
        else
        {
            mahony_integral_x = 0.0f;
            mahony_integral_y = 0.0f;
            mahony_integral_z = 0.0f;
        }


        /* 比例补偿 */

        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }


    /* ==============================
     * quaternion integration
     * ============================== */

    q_dot0 =
        0.5f *
        (-q1 * gx
         -q2 * gy
         -q3 * gz);

    q_dot1 =
        0.5f *
        ( q0 * gx
         +q2 * gz
         -q3 * gy);

    q_dot2 =
        0.5f *
        ( q0 * gy
         -q1 * gz
         +q3 * gx);

    q_dot3 =
        0.5f *
        ( q0 * gz
         +q1 * gy
         -q2 * gx);


    q0 += q_dot0 * dt;
    q1 += q_dot1 * dt;
    q2 += q_dot2 * dt;
    q3 += q_dot3 * dt;


    /* ==============================
     * Quaternion normalization
     * ============================== */

    norm =
        q0 * q0 +
        q1 * q1 +
        q2 * q2 +
        q3 * q3;

    if(norm > 0.000001f)
    {
        norm = 1.0f / sqrtf(norm);

        q0 *= norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
    }


    imu660ra_q[0] = q0;
    imu660ra_q[1] = q1;
    imu660ra_q[2] = q2;
    imu660ra_q[3] = q3;


    quat_to_euler(
        imu660ra_q,
        &imu660ra_roll,
        &imu660ra_pitch,
        &imu660ra_yaw
    );
}

/* ============================================================
 * 姿态初始化
 *
 * 建议：
 * 调用时 IMU 保持静止
 * ============================================================ */
void IMU660RA_AttitudeInit(void)
{
    float ax;
    float ay;
    float az;

    float roll;
    float pitch;

    float cr;
    float sr;
    float cp;
    float sp;


    mahony_integral_x = 0.0f;
    mahony_integral_y = 0.0f;
    mahony_integral_z = 0.0f;


    imu660ra_get_acc();


    /* 安装方向变换 */

    ax = imu660ra_acc_transition(
        imu660ra_acc_z
    );

    ay = imu660ra_acc_transition(
        imu660ra_acc_y
    );

    az = -imu660ra_acc_transition(
        imu660ra_acc_x
    );


    /*
     * 用重力初始化 Roll/Pitch
     */

    roll = atan2f(
        ay,
        az
    );

    pitch = atan2f(
        -ax,
        sqrtf(
            ay * ay +
            az * az
        )
    );


    /*
     * yaw 初值 = 0
     */

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);

    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);


    imu660ra_q[0] =
        cr * cp;

    imu660ra_q[1] =
        sr * cp;

    imu660ra_q[2] =
        cr * sp;

    imu660ra_q[3] =
        -sr * sp;


    quat_to_euler(
        imu660ra_q,
        &imu660ra_roll,
        &imu660ra_pitch,
        &imu660ra_yaw
    );
}
