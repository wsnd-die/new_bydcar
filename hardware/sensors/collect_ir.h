/**
 * @file    collect_ir.h
 * @brief   红外对射开关判断物体进入 + 颜色读取 — 收集判断
 */
#ifndef COLLECT_IR_H
#define COLLECT_IR_H

#include "color.h"
#include <stdbool.h>

/* 红外对射开关引脚 (按实际接线修改) */
#define IR_PORT    GPIOF
#define IR_PIN     GPIO_PIN_10
#define IR_ACTIVE  1        /* 实测: 遮挡=低电平(GPIO_RESET), 未遮挡=高电平; 故 0=遮光低电平=检测到 */

void          IR_Init(void);
bool          IR_ObjectPresent(void);   /* 当前是否有物体遮光 */
bool          IR_ObjectEntered(void);   /* 物体完全进入: 电平先变0(遮挡)再变1(恢复)才返回 true */
bool          Collect_WaitEnter(void);  /* 只等一个物体进入 */

/* V1.20.0 删除: Collect_ReadColor() / Collect_ReadColor_NB() / Collect_WaitObject()。
 * 它们依赖 GY-33 的 g_uart2_gy33_* 全局量, 而后者只在注释里定义 —— 一旦被调用
 * 就链接失败。颜色现在由 MSP 芯片经 USART2 回传, 见 hardware/sensors/msp_color.h。 */

#endif
