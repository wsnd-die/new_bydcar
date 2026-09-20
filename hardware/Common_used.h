/**
 * @file    Common_used.h
 * @brief   工程公共头 —— 只聚合底座: libc + HAL/CMSIS + FreeRTOS/CMSIS-RTOS2
 *
 * 使用方式：每个 .c 首行 #include "Common_used.h"，即可拿到上述公共设施
 *          （含 huart1/2/3、hi2c3、htim3 等外设句柄声明）。
 *
 * @warning 本文件**刻意不 include 任何业务层或应用层的头**。
 *
 *   V1.3.0 之前它把全部 hardware 头、`../algorithm/mecanum.h`、以及全部 app 层头
 *   （banyuntask / Mecanum_Move / NavigationMecanum / Nav_position）
 *   一次性拉进来，使 include 图退化成完全图 —— 任何一层的任何文件都能看见其它层
 *   的任何符号，CLAUDE.md 第 1 节「面向接口编程、驱动与业务完全解耦」形同虚设。
 *   同时它还挂着十余个全工程无定义的悬空 extern，靠 --gc-sections 回收才没炸。
 *
 *   现在每个 .c 必须**自己 include 它真正依赖的模块头**：
 *       需要 Send_commandmotor → #include "Send_motor.h"
 *       需要 MecanumResult     → #include "mecanum.h"
 *       需要 g_hwt_imu_yaw     → #include "hwt_imu.h"
 *   这是有意的约束, 不要为了省事把模块头加回本文件。
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
 * 4. STM32CubeMX 外设头文件（句柄声明）
 *
 *    保留在公共头里是有意的: 它们属 HAL 层设施, 不属四层中的任何一层,
 *    且每个 .c 基本都要碰其中一两个。去掉只会让每个文件都写一长串重复的
 *    CubeMX include, 换不来任何解耦收益。
 *
 *    注意 can.h 不在此列 —— 它属 FDCAN 驱动自身 (hardware/actuators/emm_v5.c
 *    与 algorithm/mecanum.c 需要 can_SendCmd), 由用到的 .c 自己 include。
 * ============================================================ */
#include "gpio.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#endif /* _COMMON_USED_ */
