/**
 * @file color.h
 * @brief GY-33 颜色传感器 UART 模式驱动
 *
 * 校准流程:
 *   1. 调用 Color_Init() 初始化
 *   2. 把传感器对准已知颜色圆柱，调用 Color_Calibrate(color, r, g, b)
 *      → 内部存储该颜色的 RGB 参考值
 *   3. 运行时调用 Color_DetectDominant() → 返回最接近的匹配颜色
 */
#ifndef __COLOR_H
#define __COLOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 颜色传感器选择: 0=GY-33, 1=OpenMV */
#ifndef USE_OPENMV_COLOR
#define USE_OPENMV_COLOR  0
#endif

/* 颜色枚举 */
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_WHITE,
    COLOR_BLACK,
    COLOR_COUNT
} Color_TypeDef;

/* 传感器数据 */
typedef struct {
    uint16_t raw_red, raw_green, raw_blue, raw_clear;
    uint16_t lux, color_temperature;
    uint8_t  red, green, blue;
    uint8_t  l, a, b;         /* OpenMV Lab 原始值 (A/B 已偏移+128) */
    uint8_t  sensor_color;
    uint8_t  online;
} Color_DataTypeDef;

/* 校准数据: 每颜色存一组 RGB 参考值 */
typedef struct {
    uint8_t r, g, b;      /* 参考 RGB */
    uint16_t tolerance;   /* 容差 (默认 30) */
    uint8_t enabled;      /* 1=已校准 */
} Color_Calib_t;

extern Color_Calib_t g_color_calib[COLOR_COUNT];

/* 环境光基准 (空槽读数) */
typedef struct {
    uint8_t r, g, b;
    uint8_t tolerance;    /* 环境光容差 (默认 20) */
    uint8_t enabled;      /* 1=已校准 */
} Color_Ambient_t;

extern Color_Ambient_t g_color_ambient;

/* ---- API ---- */
HAL_StatusTypeDef Color_Init(void);
HAL_StatusTypeDef Color_SetLedLevel(uint8_t level);
HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data);
Color_TypeDef    Color_Judge(const Color_DataTypeDef *data);
Color_TypeDef    Color_DetectDominant(void);
const char*      Color_ToString(Color_TypeDef color);

/**
 * @brief  校准某一颜色: 读取当前传感器 RGB 存为参考
 * @param  color  要校准的颜色枚举
 */
void Color_Calibrate(Color_TypeDef color);

/**
 * @brief  校准环境光: 在空槽时调用，记录环境光基准
 */
void Color_CalibAmbient(void);

/**
 * @brief  保存校准数据到备份寄存器 (掉电保存)
 * @note   GY-33: 连续输出模式 Auto=1 已掉电保存; LED 亮度需发 A5 CC 71 保存
 */
void Color_CalibSave(void);
void Color_CalibLoad(void);

#ifdef __cplusplus
}
#endif
#endif
