//
// Created by 35037 on 2026/4/30.
//
#include "Common_used.h"
#include "spi_imu660rc.h"

IMU660RC_ConfigType imu_cfg = {0};
IMU660RC_DataType  imu_data = {0};

void IMU660RC_Init(void)
{
    uint8_t whoami = IMU660RC_ReadRegs(IMU660RC_CHIP_ID);
    if (whoami != 0x70)
    {
        printf("IMU660RC_Init() 失败, WHO_AM_I = 0x%02X\r\n", whoami);
        return;
    }

    // 1. 软件复位 (逐飞方式：FUNC_CFG_ACCESS 写 0x04)
    IMU660RC_WriteRegs(IMU660RC_FUNC_CFG_ACCESS, 0x04);
    HAL_Delay(30);

    // 2. 启用块更新及地址自增
    IMU660RC_WriteRegs(IMU660RC_CTRL3, 0x44);

    // 3. 配置加速度计量程 (写入 CTRL8)
    //    同时设置 acc_sensitivity (原始值 / 灵敏度 =  g 值)
    //    可选 ±2g/±4g/±8g/±16g，这里默认用 ±16g
    {
        uint8_t acc_range_val = 0x03;   // 0x00:±2g  0x01:±4g  0x02:±8g  0x03:±16g
        float   acc_sens = 2049.18f;    // 对应 ±16g 的灵敏度
        // 如果想改量程，手动修改上面两行即可
        IMU660RC_WriteRegs(IMU660RC_CTRL8, acc_range_val);
        imu_cfg.acc_fs = acc_range_val;
        imu_cfg.acc_sensitivity = acc_sens;
    }

    // 4. 配置陀螺仪量程 (写入 CTRL6)
    //    可选 ±125/±250/±500/±1000/±2000/±4000 dps，这里默认用 ±2000dps
    {
        uint8_t gyro_range_val = 0x04;  // 0x00:±125  0x01:±250  0x02:±500  0x03:±1000  0x04:±2000  0x0C:±4000
        float   gyro_sens = 14.2857f;   // 对应 ±2000dps 的灵敏度
        IMU660RC_WriteRegs(IMU660RC_CTRL6, gyro_range_val);
        imu_cfg.gyro_fs = gyro_range_val;
        imu_cfg.gyro_sensitivity = gyro_sens;
    }

    // 5. 设置传感器工作模式与输出频率
    IMU660RC_WriteRegs(IMU660RC_CTRL1, 0x15);   // 加速度计：高精度模式，208 Hz (可根据需要调)
    IMU660RC_WriteRegs(IMU660RC_CTRL2, 0x18);   // 陀螺仪：  高精度模式，208 Hz

    // 6. 开启低通滤波 (LPF1 / LPF2)
    IMU660RC_WriteRegs(IMU660RC_CTRL7, 0x01);   // 使能陀螺仪 LPF1
    IMU660RC_WriteRegs(IMU660RC_CTRL9, 0x08);   // 使能加速度计 LPF2

    printf("IMU660RC_Init() 成功\r\n");
}

// void IMU660RC_Init_SFLP(void)
// {
//     // 基础初始化不变...
//     IMU660RC_Init();
//
//     // 进入嵌入功能配置
//     IMU660RC_WriteRegs(CTRL5_C, 0x80);            // FUNC_CFG_ACCESS = 0x80
//     IMU660RC_WriteRegs(PAGE_SEL, 0x80);        // PAGE_SEL = 0x80
//     IMU660RC_WriteRegs(PAGE_RW, 0x01);         // 先进入主嵌入页 (重要: 这里可能需要 0x01)
//     // 写入嵌入功能寄存器
//     IMU660RC_WriteRegs(EMB_FUNC_INIT_A, 0x01);// 复位 SFLP
//     HAL_Delay(1);
//     IMU660RC_WriteRegs(EMB_FUNC_CFG, 0x30);    // 0x30, 不是 0x20
//     IMU660RC_WriteRegs(EMB_FUNC_EN_A, 0x02);   // 0x02, 不是 0x01
//     IMU660RC_WriteRegs(SFLP_ODR, 0x5B);        // 120Hz (0x43 | (3<<3))
//
//     IMU660RC_WriteRegs(0x0E, 0x80);          // INT2_CTRL: 使能 INT2 输出数据就绪
//     IMU660RC_WriteRegs(CTRL4_C, 0x08);       // CTRL4_C: 使能 SFLP 数据就绪信号路由到 INT2
//     IMU660RC_WriteRegs(CTRL1_XL, 0x16);      // 加速度计 ODR 改为 120Hz (匹配 SFLP)
//     IMU660RC_WriteRegs(CTRL2_G,  0x16);
//     // 退出
//     IMU660RC_WriteRegs(PAGE_RW, 0x00);
//     IMU660RC_WriteRegs(PAGE_SEL, 0x00);
//     IMU660RC_WriteRegs(CTRL5_C, 0x00);
//     printf("IMU660RC_Init_SFLP() 成功\n");
// }

void IMU660RC_ReadAcc(void)
{
    uint8_t buf[6];
    IMU660RC_ReadMultiRegs(IMU660RC_OUTX_L_A, buf, 6);  // 使用正确的批量读函数
    imu_data.ax_raw = (int16_t)((buf[1]<<8) | buf[0]);
    imu_data.ay_raw = (int16_t)((buf[3]<<8) | buf[2]);
    imu_data.az_raw = (int16_t)((buf[5]<<8) | buf[4]);
    imu_data.ax = imu_data.ax_raw / imu_cfg.acc_sensitivity;
    imu_data.ay = imu_data.ay_raw / imu_cfg.acc_sensitivity;
    imu_data.az = imu_data.az_raw / imu_cfg.acc_sensitivity;


}
void IMU660RC_ReadGyro(void) {
    uint8_t buf[6];
    IMU660RC_ReadMultiRegs(IMU660RC_OUTX_L_G, buf, 6);
    imu_data.gx_raw = (int16_t)((buf[1]<<8) | buf[0]);
    imu_data.gy_raw = (int16_t)((buf[3]<<8) | buf[2]);
    imu_data.gz_raw = (int16_t)((buf[5]<<8) | buf[4]);
    imu_data.gx = imu_data.gx_raw / imu_cfg.gyro_sensitivity;
    imu_data.gy = imu_data.gy_raw / imu_cfg.gyro_sensitivity;
    imu_data.gz = imu_data.gz_raw / imu_cfg.gyro_sensitivity;

}
void IMU660RC_WriteRegs(uint8_t reg,uint8_t data)
{
    uint8_t tx[2] = {reg&0x7F,data};
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_Transmit(&hspi2,tx,2,1000);
    SPI_IMU660RC_CS_HIGH();
}
// void IMU660RC_ReadEuler(void) {
//     uint8_t buf[6];
//     IMU660RC_ReadMultiRegs(ANGLE_ROLL_L, buf, 6);
//     int16_t roll  = (int16_t)((buf[1]<<8) | buf[0]);
//     int16_t pitch = (int16_t)((buf[3]<<8) | buf[2]);
//     int16_t yaw   = (int16_t)((buf[5]<<8) | buf[4]);
//     imu_data.roll  = roll / 100.0f;
//     imu_data.pitch = pitch / 100.0f;
//     imu_data.yaw   = yaw / 100.0f;
// }

// void IMU660RC_ReadQuat(void) {
//     uint8_t i;
//     uint16_t buff[4];
//     uint8_t *buff_ptr = (uint8_t *)buff;
//
//     // 进入嵌入功能页
//     IMU660RC_WriteRegs(0x01, 0x80);      // FUNC_CFG_ACCESS
//     IMU660RC_WriteRegs(PAGE_RW, 0x20);   // 写 0x20
//     IMU660RC_WriteRegs(PAGE_SEL, 0x31);  // 写 0x31
//
//     for(i = 0; i < 4; i++) {
//         IMU660RC_WriteRegs(0x08, (uint8_t)(0x4C + i*2 + 0));
//         buff_ptr[i*2 + 0] = IMU660RC_ReadRegs(0x09);   // 低字节
//         IMU660RC_WriteRegs(0x08, (uint8_t)(0x4C + i*2 + 1));
//         buff_ptr[i*2 + 1] = IMU660RC_ReadRegs(0x09);   // 高字节
//     }
//
//     // 退出嵌入页面
//     IMU660RC_WriteRegs(PAGE_RW, 0x00);
//     IMU660RC_WriteRegs(0x01, 0x00);
//
//     // 半精度转浮点 + 归一化
//     float temp[4];
//     *(uint32_t*)(&temp[0]) = fp16_to_float(buff[0]);
//     *(uint32_t*)(&temp[1]) = fp16_to_float(buff[1]);
//     *(uint32_t*)(&temp[2]) = fp16_to_float(buff[2]);
//     *(uint32_t*)(&temp[3]) = fp16_to_float(buff[3]);
//
//     float n = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
//     if (n > 0.001f) {
//         n = temp[3] < 0.0f ? -n : n;   // 逐飞库的特殊处理
//         imu_data.q[0] = temp[1] / n;
//         imu_data.q[1] = temp[2] / n;
//         imu_data.q[2] = temp[0] / n;
//         imu_data.q[3] = temp[3] / n;
//     } else {
//         imu_data.q[0] = 0.0f;
//         imu_data.q[1] = 0.0f;
//         imu_data.q[2] = 0.0f;
//         imu_data.q[3] = 1.0f;
//     }
//     //printf("buff: %04X %04X %04X %04X\n", buff[0], buff[1], buff[2], buff[3]);
// }
uint8_t IMU660RC_ReadRegs(uint8_t reg)
{
    uint8_t tx[2] = {reg|0x80,0xFF};//tx[0]1号位为1是读寄存器后七位为地址 tx[1]为填充也可以只用一个
    uint8_t rx[2] = {0,0};
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2,tx,rx,2,1000);
    SPI_IMU660RC_CS_HIGH();
    return rx[1];
}

uint16_t IMU660RC_ReadReg16b(uint8_t reg_low)//输入的是低位地址,+1为高位
{
    uint8_t data_l=IMU660RC_ReadRegs(reg_low);
    uint8_t data_h=IMU660RC_ReadRegs(reg_low+1);
    return (uint16_t)((data_h<<8)|data_l);
}
void IMU660RC_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx[len + 1];
    tx[0] = reg | 0x80;
    for (int i = 0; i < len; i++) tx[i + 1] = 0xFF;   // dummy 字节

    uint8_t rx[len + 1];
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + 1, 1000);
    SPI_IMU660RC_CS_HIGH();

    memcpy(buf, &rx[1], len);   // 忽略第一个接收字节
}
void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw) {
    float sqx = q[0] * q[0];
    float sqy = q[1] * q[1];
    float sqz = q[2] * q[2];

    *roll  = atan2f(2.0f * (q[1]*q[3] + q[0]*q[2]), 1.0f - 2.0f*(sqy + sqx));
    *yaw = -asinf(2.0f * (q[0]*q[3] - q[1]*q[2]));
    *pitch   = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(sqx + sqz));

    *roll  *= 57.29578f;
    *pitch *= 57.29578f;
    *yaw   *= 57.29578f;
    if (*yaw < 0) *yaw += 360.0f;
}

static uint32_t fp16_to_float(uint16_t h)
{
    uint32_t f_sgn;
    uint16_t h_exp;
    uint32_t f_exp;
    uint32_t f_sig;

    h_exp = (h & 0x7c00u);
    f_sgn = ((uint32_t)h & 0x8000u) << 16;
    switch (h_exp)
    {
    case 0x0000u:   // 0 or subnormal
        {
            uint16_t h_sig = (h & 0x03ffu);
            // Signed zero
            if (h_sig == 0)
            {
                return f_sgn;
            }
            // Subnormal
            h_sig <<= 1;
            while ((h_sig & 0x0400u) == 0)
            {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
            return f_sgn + f_exp + f_sig;
        }
    case 0x7c00u: // inf or NaN
        {
            // All-ones exponent and a copy of the significand
            return f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
        }
    default: // normalized
        {
            // Just need to adjust the exponent and shift
            return f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
        }
    }
}
/*
 * @brief Mahony 互补滤波姿态更新
 * @param dt 距离上次更新的时间（秒），若不传则内部用 HAL_GetTick 计算
 */
void IMU660RC_AttitudeUpdate(float dt)
{
    float Kp = 0.5f;    // 比例增益，越大越信任加速度计
    float Ki = 0.01f;   // 积分增益，消除长期漂移

    // ---- 获取角速度并转为 rad/s
    IMU660RC_ReadGyro();
    float gx = (imu_data.gx - gyro_bias[0]) * 0.0174533f;  // deg/s -> rad/s
    float gy = (imu_data.gy - gyro_bias[1]) * 0.0174533f;
    float gz = (imu_data.gz - gyro_bias[2]) * 0.0174533f;

    // ---- 获取加速度，归一化
    IMU660RC_ReadAcc();
    float ax = imu_data.ax, ay = imu_data.ay, az = imu_data.az;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.01f) return;           // 加速度太小，无法修正
    ax /= norm; ay /= norm; az /= norm;

    // ---- 计算当前四元数对应的重力方向
    float vx = 2.0f * (att_q[1] * att_q[3] - att_q[0] * att_q[2]);
    float vy = 2.0f * (att_q[0] * att_q[1] + att_q[2] * att_q[3]);
    float vz = att_q[0] * att_q[0] - att_q[1] * att_q[1] - att_q[2] * att_q[2] + att_q[3] * att_q[3];

    // ---- 误差 = 加速度计测量 × 重力推算方向（叉积）
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    // ---- PI 修正陀螺仪
    integral_fb[0] += Ki * ex * dt;
    integral_fb[1] += Ki * ey * dt;
    integral_fb[2] += Ki * ez * dt;

    gx += Kp * ex + integral_fb[0];
    gy += Kp * ey + integral_fb[1];
    gz += Kp * ez + integral_fb[2];

    // ---- 一阶龙格-库塔更新四元数
    float q0 = att_q[0], q1 = att_q[1], q2 = att_q[2], q3 = att_q[3];
    float half_dt = 0.5f * dt;
    att_q[0] += half_dt * (-q1 * gx - q2 * gy - q3 * gz);
    att_q[1] += half_dt * ( q0 * gx + q2 * gz - q3 * gy);
    att_q[2] += half_dt * ( q0 * gy - q1 * gz + q3 * gx);
    att_q[3] += half_dt * ( q0 * gz + q1 * gy - q2 * gx);

    // 归一化四元数
    norm = sqrtf(att_q[0]*att_q[0] + att_q[1]*att_q[1] + att_q[2]*att_q[2] + att_q[3]*att_q[3]);
    if (norm > 0.001f) {
        att_q[0] /= norm; att_q[1] /= norm; att_q[2] /= norm; att_q[3] /= norm;
    }

    // ---- 四元数转欧拉角（输出，角度制）
    quat_to_euler(att_q, &imu_data.roll, &imu_data.pitch, &imu_data.yaw);
}
/**
 * @brief Mahony 互补滤波姿态更新
 * @param dt 距离上次更新的时间间隔（秒）
 */
/**
 * @brief 互补滤波姿态初始化（获取零偏并设置初始四元数）
 */
void IMU660RC_AttitudeInit(void)
{
    /* 1. 采集陀螺仪零偏 */
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for (int i = 0; i < 200; i++) {
        IMU660RC_ReadGyro();
        gx_sum += imu_data.gx;
        gy_sum += imu_data.gy;
        gz_sum += imu_data.gz;
        HAL_Delay(2);
    }
    gyro_bias[0] = gx_sum / 200;
    gyro_bias[1] = gy_sum / 200;
    gyro_bias[2] = gz_sum / 200;

    /* 2. 用加速度计初始化 roll 和 pitch */
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (int i = 0; i < 50; i++) {
        IMU660RC_ReadAcc();
        ax_sum += imu_data.ax;
        ay_sum += imu_data.ay;
        az_sum += imu_data.az;
        HAL_Delay(2);
    }
    float ax = ax_sum / 50;
    float ay = ay_sum / 50;
    float az = az_sum / 50;

    // 归一化
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.01f) { ax = 0; ay = 0; az = 1; }
    else { ax /= norm; ay /= norm; az /= norm; }

    // 通过加速度计计算初始 roll/pitch（yaw 设为 0）
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    // 转为四元数
    float cy = cosf(0 * 0.5f);
    float sy = sinf(0 * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    att_q[0] = cr * cp * cy + sr * sp * sy;
    att_q[1] = sr * cp * cy - cr * sp * sy;
    att_q[2] = cr * sp * cy + sr * cp * sy;
    att_q[3] = cr * cp * sy - sr * sp * cy;

    // 归一化四元数
    norm = sqrtf(att_q[0]*att_q[0] + att_q[1]*att_q[1] + att_q[2]*att_q[2] + att_q[3]*att_q[3]);
    if (norm > 0.001f) {
        att_q[0] /= norm; att_q[1] /= norm; att_q[2] /= norm; att_q[3] /= norm;
    }

    // 清零积分项
    integral_fb[0] = 0; integral_fb[1] = 0; integral_fb[2] = 0;

    last_tick = HAL_GetTick();

    // 初始欧拉角赋值
    imu_data.roll  = roll  * 57.2958f;
    imu_data.pitch = pitch * 57.2958f;
    imu_data.yaw   = 0.0f;
}