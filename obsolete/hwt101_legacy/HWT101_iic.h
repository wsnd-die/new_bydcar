/**
 * @file    HWT101_iic.h
 * @brief   HWT101 (WitMotion) 传感器完整驱动模块。
 *
 *          ====== 硬件接口 ======
 *          I2C1: PA15=SCL, PB9=SDA, 地址 0x50
 *          RX:  DMA1_Channel1 (DMA 异步传输 + FreeRTOS 信号量同步)
 *          TX:  阻塞模式（写操作为短帧，DMA 无优势）
 *
 *          ====== 传输模式选择 ======
 *          #define HWT101_USE_I2C     → I2C 模式（默认，DMA 读）
 *          #define HWT101_USE_SERIAL  → 串口模式（UART4）
 *          两者互斥，同时定义时 I2C 优先生效。
 *
 *          ====== 使用示例 ======
 *          HWT101_I2C_Init();          // 创建 DMA 信号量（FreeRTOS 启动前）
 *          HWT101_HAL_Init();          // 注册回调、配置传感器（FreeRTOS 启动后）
 *          HWT101_PollAngles();        // 每 10~20ms 轮询一次
 *          float yaw = HWT101_GetZeroYaw();
 *
 * @version 0.3
 * @date    2026-08-02
 * @copyright Copyright (c) 2026
 */

#ifndef HWT101_IIC_H
#define HWT101_IIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 依赖 ---- */
#include "stm32g4xx_hal.h"
#include "wit_protocol.h"
#include <stdint.h>

/* ========================================================================
   传输模式选择（二选一）
   ======================================================================== */

#define HWT101_USE_I2C          /* 使用 I2C 模式（默认） */
/* #define HWT101_USE_SERIAL */ /* 使用串口模式           */

#if !defined(HWT101_USE_I2C) && !defined(HWT101_USE_SERIAL)
#error "Must define HWT101_USE_I2C or HWT101_USE_SERIAL"
#endif

/* ========================================================================
   常量
   ======================================================================== */

#define HWT101_I2C_ADDR         (0x50U << 1)         /* I2C 7→8 位地址            */
#define HWT101_I2C_TIMEOUT_MS   100U                 /* I2C 超时 (ms)              */
#define HWT101_SERIAL_ADDR      0x50U                /* 设备地址（串口 / I2C 共用）*/
#define HWT101_SERIAL_RX_SIZE   64U                  /* 串口接收缓冲大小            */

/* ========================================================================
   外部引用（I2C 外设句柄）
   ======================================================================== */

extern I2C_HandleTypeDef  hi2c1;
extern DMA_HandleTypeDef  hdma_i2c1_rx;

/* ========================================================================
   全局角度数据（由协议层回调更新）
   ======================================================================== */

extern volatile float    g_hwt101_roll;
extern volatile float    g_hwt101_pitch;
extern volatile float    g_hwt101_yaw;
extern volatile uint8_t  g_hwt101_data_ready;

/* ========================================================================
   API
   ======================================================================== */

/* ---------- 初始化 ---------- */

/**
 * @brief  HWT101 I2C 底层初始化（创建 DMA 同步信号量）。
 * @note   必须在 FreeRTOS API 可用之后、HWT101_HAL_Init() 之前调用。
 */
void    HWT101_I2C_Init(void);

/**
 * @brief  HWT101 传感器完整初始化（注册协议回调 + 配置传感器）。
 * @retval WIT_HAL_OK(0) / WIT_HAL_ERROR(-1) / WIT_HAL_INVAL(-2)
 * @note   需在 FreeRTOS 启动后、外设初始化完成后调用。
 */
int32_t HWT101_HAL_Init(void);

/* ---------- 运行期 ---------- */

/**
 * @brief  获取传感器是否在线。
 * @retval 1 在线 / 0 不在线
 */
int32_t HWT101_IsOnline(void);

/**
 * @brief  读取单个寄存器当前值（调试 / 高级使用）。
 * @param  reg  寄存器枚举值（Roll / Pitch / Yaw / …）
 * @return 寄存器值（int16 原始值）
 */
int16_t HWT101_ReadReg(uint32_t reg);

/**
 * @brief  获取相对偏航角（减去初始零偏）。
 * @return 相对偏航角 (°)
 */
float   HWT101_GetZeroYaw(void);

/**
 * @brief  I2C 模式：主动轮询角度数据。
 * @retval WIT_HAL_OK / WIT_HAL_ERROR
 * @note   放在 FreeRTOS 任务中每 10~20ms 调用一次。
 *         串口模式下无需调用（硬件自动推送）。
 */
int32_t HWT101_PollAngles(void);

/* ---------- 串口模式专用 ---------- */

/**
 * @brief  将接收到的 1 字节送入协议栈解析。
 * @param  data  1 字节数据
 * @note   在 UART4 RX ISR 中调用（仅 HWT101_USE_SERIAL 模式）。
 */
void    HWT101_FeedSerialByte(uint8_t data);

/**
 * @brief  批量解析一帧 11 字节 0x55 数据包。
 * @param  data  11 字节数据包指针
 * @note   仅 HWT101_USE_SERIAL 模式有效。
 */
void    HWT101_ParsePacket(uint8_t *data);

/* ---------- 底层 I2C（供协议层 / 调试使用） ---------- */

HAL_StatusTypeDef HWT101_IsReady(void);
HAL_StatusTypeDef HWT101_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HWT101_WriteReg(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* HWT101_IIC_H */