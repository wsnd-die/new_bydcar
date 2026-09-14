/**
 * @file    collect_ir.h
 * @brief   红外对射开关判断物体进入 + 颜色读取 — 收集判断
 */
#ifndef COLLECT_IR_H
#define COLLECT_IR_H

#include "color.h"
#include <stdbool.h>

/* 红外对射开关引脚 (按实际接线修改) */
#define IR_PORT    GPIOB
#define IR_PIN     GPIO_PIN_6
#define IR_ACTIVE  0        /* 实测: 遮挡=低电平(GPIO_RESET), 未遮挡=高电平; 故 0=遮光低电平=检测到 */

void          IR_Init(void);
bool          IR_ObjectPresent(void);   /* 当前是否有物体遮光 */
bool          IR_ObjectEntered(void);   /* 物体完全进入: 电平先变0(遮挡)再变1(恢复)才返回 true */
bool          Collect_WaitEnter(void);  /* 只等一个物体进入, 不读色 */
Color_TypeDef Collect_ReadColor(void);    /* 只读颜色(多帧平均), 不等待进入 */
Color_TypeDef Collect_ReadColor_NB(void); /* 非阻塞取一帧: 有帧立即判色, 无帧立即返回 UNKNOWN */
Color_TypeDef Collect_WaitObject(void);   /* 等一个物体进入并读色, 返回颜色 */

#endif
