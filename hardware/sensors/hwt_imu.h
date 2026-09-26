/**
 * @file    hwt_imu.h
 * @brief   维特智能 (WitMotion) IMU 驱动 —— HWT101 / HWT906 通用。
 *
 *          ====== 硬件接口 ======
 *          I2C3: PC8 = SCL, PB5 = SDA (见 Core/Src/i2c.c 的 MX_I2C3_Init)
 *          7 位地址 0x50, 模块改过地址时同步 HWT_IMU_I2C_ADDR。
 *
 *          ====== 读什么 ======
 *          HWT906 是九轴模块(三轴陀螺仪 + 三轴加速度计 + 三轴磁力计),
 *          内部跑完姿态融合后, 寄存器 0x34~0x40 依次是:
 *
 *              0x34..0x36   AX AY AZ          加速度
 *              0x37..0x39   GX GY GZ          角速度
 *              0x3A..0x3C   HX HY HZ          磁场
 *              0x3D..0x3F   Roll Pitch Yaw    姿态角
 *              0x40         TEMP              温度
 *
 *          V1.11.0 起 HWT_IMU_Poll() 一次事务把这一整块(13 个寄存器 / 26 字节)
 *          读回; 此前只读 0x3D 起 6 字节的姿态角一路。
 *
 *          【寻址方式 —— 别再踩这个坑】维特 I2C 的寄存器地址是"索引",
 *          与串口/Modbus 协议的编号一致(0x34=AX … 0x40=TEMP), 每个索引对
 *          2 字节小端。所以"从 0x3D 读 6 字节"正好是 Roll/Pitch/Yaw 三个
 *          寄存器 —— 不是"每个寄存器只占 1 个地址"。下面的块读偏移就是按
 *          (索引差 × 2) 算的。
 *
 *          ====== 量程 ======
 *          加速度/角速度量程可在模块里改(加速度 ±2/4/8/16g, 角速度
 *          ±250/500/1000/2000°/s), 换量程必须同步改下面三个换算宏, 否则
 *          数值会差整数倍。台架核对方法见 clauderecord/2026-09-25.md。
 *
 *          ====== 使用 ======
 *          HWT_IMU_Init();            // 探测在线 + 当前航向设为零点
 *          HWT_IMU_Poll();            // 每 5~10ms 调一次(全工程只调一处)
 *          float yaw = g_hwt_imu_yaw_rad;
 *          float wz  = g_hwt_imu_gyro_z_rad;
 *
 * @note    【V1.14.2 起】读数据走 I2C3 + DMA：HAL_I2C_Mem_Read_DMA() 启动传输，
 *          由 I2C3_EV/ER 中断推进，本驱动用二值信号量等完成回调后才返回。
 *          对外仍是「阻塞到数据到手」的语义，上界受 HWT_IMU_TIMEOUT_MS 约束。
 *
 *          原先这里写着「读数据用阻塞式 HAL_I2C_Mem_Read …… 26 字节量级 DMA
 *          收益不明显, 却要额外引入传输完成同步」。那半句是对的：26B @400kHz
 *          只有约 0.65ms。改用 DMA 换来的不是速度，而是让地址相位与收尾由中断
 *          承担、把等待交还给调度器；代价是必须补上 I2C3_EV/ER 的 NVIC 与
 *          IRQHandler（Core/Src/i2c.c 的 I2C3_MspInit、Core/Src/stm32g4xx_it.c），
 *          否则传输永远完不成、hi2c3.State 卡在 BUSY_RX。
 *
 * @warning HWT_IMU_Poll() / HWT_IMU_ReadReg() **只能在任务上下文调用**
 *          （内部要 acquire 信号量）。目前唯一调用者是 FC_TASK。
 *          调度器未启动时会自动退化为轮询 HAL 状态机，不会崩。
 */

#ifndef __HWT_IMU_H
#define __HWT_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================
   常量
   ======================================================================== */

#define HWT_IMU_I2C_ADDR        (0x50U << 1)   /* I2C 7 位地址 0x50 → HAL 8 位 */
#define HWT_IMU_TIMEOUT_MS      10U            /* 单次传输超时 (ms)            */

/* ---- 寄存器索引 (维特协议编号，每个索引 = 2 字节小端) ---- */

/* 加速度 */
#define HWT_IMU_REG_AX          0x34U
#define HWT_IMU_REG_AY          0x35U
#define HWT_IMU_REG_AZ          0x36U
/* 角速度 */
#define HWT_IMU_REG_GX          0x37U
#define HWT_IMU_REG_GY          0x38U
#define HWT_IMU_REG_GZ          0x39U
/* 磁场 (9 轴模块才有数据; 模块跑 6 轴算法时这几个寄存器读回 0) */
#define HWT_IMU_REG_HX          0x3AU
#define HWT_IMU_REG_HY          0x3BU
#define HWT_IMU_REG_HZ          0x3CU
/* 姿态角 */
#define HWT_IMU_REG_ROLL        0x3DU
#define HWT_IMU_REG_PITCH       0x3EU
#define HWT_IMU_REG_YAW         0x3FU
/* 温度 */
#define HWT_IMU_REG_TEMP        0x40U

/* ---- 块读布局 (0x34..0x40 连续，一次事务读回) ---- */

#define HWT_IMU_BLOCK_NREG      (HWT_IMU_REG_TEMP - HWT_IMU_REG_AX + 1U)  /* 13 */
#define HWT_IMU_BLOCK_LEN       (HWT_IMU_BLOCK_NREG * 2U)                 /* 26 */
/* 退化路径把姿态角块塞回同一缓冲区，偏移 = (0x3D - 0x34) * 2 */
#define HWT_IMU_BLOCK_OFF_ANGLE (((HWT_IMU_REG_ROLL - HWT_IMU_REG_AX) * 2U))

/* ---- 原始值 → 物理量 ----
 * 下面三个量程宏必须与模块内的实际配置一致: 静止时三轴合矢量应为 1g,
 * 读数若是 1g 的整数倍偏差, 就按倍数改 HWT_IMU_ACC_RANGE_G。 */
#define HWT_IMU_ACC_RANGE_G     16.0f          /* ±2 / 4 / 8 / 16 g        */
#define HWT_IMU_GYRO_RANGE_DPS  2000.0f        /* ±250 / 500 / 1000 / 2000 */

#define HWT_IMU_RAW_TO_DEG      (180.0f / 32768.0f)
#define HWT_IMU_DEG2RAD         (3.14159265358979f / 180.0f)
#define HWT_IMU_ACC_LSB_TO_MS2  (HWT_IMU_ACC_RANGE_G * 9.80665f / 32768.0f)
#define HWT_IMU_GYRO_LSB_TO_DPS (HWT_IMU_GYRO_RANGE_DPS / 32768.0f)
#define HWT_IMU_TEMP_LSB_TO_C   (1.0f / 100.0f)

/* ========================================================================
   全局数据 (HWT_IMU_Poll 更新)
   ======================================================================== */

/** @brief roll (deg), 传感器原始值, 未做安装方向修正 */
extern volatile float    g_hwt_imu_roll;
/** @brief pitch (deg) */
extern volatile float    g_hwt_imu_pitch;
/** @brief yaw (deg), 已扣除 HWT_IMU_SetZero 设下的零点, 范围 -180..180 */
extern volatile float    g_hwt_imu_yaw;
/** @brief yaw 的弧度形式, 供里程计/导航使用 (与旧 siyuan_yaw 同量纲) */
extern volatile float    g_hwt_imu_yaw_rad;
/** @brief 最近一次 HWT_IMU_Poll 是否读到了新数据 */
extern volatile uint8_t  g_hwt_imu_data_ready;
/** @brief 模块是否在线 (HWT_IMU_Init 探测结果, Poll 失败会清 0) */
extern volatile uint8_t  g_hwt_imu_online;

/* ---- 以下为 V1.11.0 新增: 块读带回来的其余八轴 + 温度 ---- */

/** @brief 三轴线加速度 (m/s^2), 传感器本体坐标系, 未做安装方向修正 */
extern volatile float    g_hwt_imu_acc_x;
extern volatile float    g_hwt_imu_acc_y;
extern volatile float    g_hwt_imu_acc_z;
/** @brief 三轴角速度 (deg/s), 传感器本体坐标系 */
extern volatile float    g_hwt_imu_gyro_x;
extern volatile float    g_hwt_imu_gyro_y;
extern volatile float    g_hwt_imu_gyro_z;
/** @brief Z 轴角速度的弧度形式 (rad/s), 供 PoseData_t.wz 直接取用 */
extern volatile float    g_hwt_imu_gyro_z_rad;
/** @brief 三轴磁场原始值 (mG —— 手册未给缩放系数, 按原始值原样给出) */
extern volatile int16_t  g_hwt_imu_mag_x;
extern volatile int16_t  g_hwt_imu_mag_y;
extern volatile int16_t  g_hwt_imu_mag_z;
/** @brief 模块温度 (degC) */
extern volatile float    g_hwt_imu_temp;

/**
 * @brief 最近一次 Poll 是否读到了完整块 (0x34~0x40)。
 * @note  0 = 长块读失败, 已退化成只读姿态角块 —— 此时上面那些扩展量
 *        (加速度/角速度/磁场/温度) 保持上一次的值, 不要当新数据用。
 */
extern volatile uint8_t  g_hwt_imu_ext_ok;

/* ========================================================================
   API
   ======================================================================== */

/**
 * @brief  初始化: 探测模块是否在线, 并把当前航向设为零点。
 * @retval 1 = 在线, 0 = 无应答 (检查接线/上拉/地址)
 * @note   需在外设初始化之后调用 (MX_I2C3_Init 之后)。
 */
uint8_t HWT_IMU_Init(void);

/**
 * @brief  读一帧 0x34~0x40 全块并刷新全局量。
 * @retval 1 = 成功, 0 = I2C 无应答 (全局量保持上一次的值, g_hwt_imu_online 清 0)
 * @note   长块读失败时会自动退化成只读姿态角块(0x3D~0x3F), 此时仍返回 1,
 *         但 g_hwt_imu_ext_ok 置 0 —— 航向不会跟着一起失效。
 */
uint8_t HWT_IMU_Poll(void);

/**
 * @brief  把当前航向记为零点 (车头对准希望作为 0° 的方向时调用)。
 */
void HWT_IMU_SetZero(void);

/**
 * @brief  读单个寄存器的原始值 (调试 / 高级用法)。
 * @param  reg  寄存器地址, 如 HWT_IMU_REG_YAW
 * @return int16 原始值
 */
int16_t HWT_IMU_ReadReg(uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __HWT_IMU_H */
