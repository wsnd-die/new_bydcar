/**
 * @file    Common_used.h
 * @brief   项目统一公共头文件 — 汇总所有常用 include、extern 声明与宏定义
 *
 * 使用方式：每个 .c 文件仅需 #include "Common_used.h"，
 *          即可获得 HAL、FreeRTOS、外设驱动、硬件模块等全部常用头文件。
 *
 * 注意：该文件会被 STM32CubeMX 自动生成的 main.h 间接包含的场景所依赖，
 *       请勿在 CubeMX 重新生成时覆盖本文件。
 */

#ifndef _COMMON_USED_
#define _COMMON_USED_

/* ============================================================
 * 1. 标准 C 库
 * ============================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <string.h>
#include <stdarg.h>
#include "block_basic.h"
#include "QRcode.h"

/* ============================================================
 * 2. STM32G4 HAL / CMSIS
 * ============================================================ */
#include "main.h"               /* → stm32g4xx_hal.h + GPIO Pin 宏 */
#include "stm32g4xx.h"          /* CMSIS Device Header */

/* ============================================================
 * 3. FreeRTOS / CMSIS-RTOS V2
 * ============================================================ */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "semphr.h"

/* ============================================================
 * 4. STM32CubeMX 外设头文件
 * ============================================================ */
#include "gpio.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* ============================================================
 * 5. 硬件驱动模块
 * ============================================================ */
#include "emm_5v.h"
#include "Send_motor.h"
#include "oled.h"
#include "oled_data.h"
/*
 * IMU660 已停用 (换成维特 HWT906, 见硬件 I2C3)。原驱动 imu660.h / imu_660.c /
 * spi_imu660rc.* 与依赖它的姿态解算 ahrs_mahony.* / siyuan_imu.* 都已移到
 * obsolete/imu660/。现在航向直接取 hwt_imu.h 的 g_hwt_imu_yaw / _yaw_rad。
 */
#include "hwt_imu.h"
#include "../algorithm/mecanum.h"
#include "uart2_tbop10.h"
#include "angle_ctrl.h"
#include "block_basic.h"
#include "Trace_base.h"
#include "Circle_base.h"
#include "color.h"
#include "collect_ir.h"
#include "ColorIdentif.h"
#include "QRcode.h"
#include "k230.h"
#include "../Core/Inc/can.h"
#include "pid.h"
/* ARM DSP 库（麦轮使用） */
#include "arm_math.h"

/* ------------------------------------------------------------
 * 角度换算常量
 * 原来由 ahrs_mahony.h 顺带提供, 该文件已移入 obsolete/imu660/,
 * 这里补回来以免其他模块 (NavigationMecanum 等) 依赖它时找不到。
 * ------------------------------------------------------------ */
#ifndef DEG_TO_RAD
#define DEG_TO_RAD  0.01745329251994329577f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG  57.295779513082320876f
#endif

/* ============================================================
 * 6. 应用层模块
 * ============================================================ */
#include "banyuntask.h"
#include "Mecanum_Move.h"
#include "NavigationMecanum.h"
#include "block_basic.h"
#include "Nav_position.h"
#include "grayscale.h"
#include "GrayTrace.h"
/* ============================================================
 * 7. 项目全局宏定义
 * ============================================================ */
#define use_xing_che   0
#define ni_he_mode     0

/* 串口1 接收电机数据 */
#define RX_BUF_SIZE 10

/* ============================================================
 * 8. 项目全局 extern 变量声明
 * ============================================================ */

/* --- 串口标志 --- */
extern uint8_t FlagOFMotor, FlagOFYuyin;
extern uint8_t Data_uart1[25], Data_uart3[20];

/* --- 串口3 语音数据 --- */
extern char *buffer[23];

/* --- 电机数据 --- */
extern uint16_t left_vel, right_vel;
extern uint8_t left_acc, left_dir;
extern uint8_t right_acc, right_dir;
extern float motor_v, motor_w, data_angle;
extern volatile float front_angle;

/* --- 语音数据 --- */
extern uint8_t buffer_flag;
extern uint8_t buf;

/* --- 角度控制任务 (FC_TASK) --- */
extern volatile uint8_t g_angle_ctrl_enable;   /* 1=使能角度控制 */
extern volatile float   g_angle_target_yaw;    /* 目标角度 (deg) */

/* --- 速度模式导航开关 --- */
extern volatile uint8_t g_nav_speed_mode;      /* 1=下次导航用速度模式 */

/* ============================================================
 * 9. 项目全局函数原型
 * ============================================================ */
void Uart3_deel(void);
void Uart1_DMA_IDLE_Start(void);
void shell_print(uint8_t *x);        /* 解析上位机电机数据 */
void shell_print3(uint8_t *x);       /* 解析上位机语音数据 */
void Send_commendyu(void);           /* 发送电机命令 */
/* Send_commandmotor() 原型已收归 hardware/Send_motor.h，见上方 include。 */
void Servo_SetAngle(float Angle);
void UART3_Send(uint8_t *DATA, uint8_t len);
void Guan_dao(float DT);

#endif /* _COMMON_USED_ */