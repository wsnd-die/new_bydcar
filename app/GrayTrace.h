/**
 * @file GrayTrace.h
 * @brief 八通道灰度循迹控制器 — Track_Err 查表 + 比例转向 + 速度自适应
 *
 * 流程: 灰度 8 通道数字量 → Track_Err 查表偏差 error → 比例 w = error·Gain
 *       → 速度自适应(误差大减速) + v_smooth 低通 → Mecanum_Calc(v, w) → 麦轮
 *
 * 偏差约定 (同参考 Trace_base / Track_Err):
 *   error > 0 → 黑线在车偏右 (需左转修正)
 *   若实车转向相反 → 把 GRAY_ERR_TO_W_GAIN 取负即可
 */

#ifndef GRAYTRACE_H
#define GRAYTRACE_H

#include "grayscale.h"      /* 已含 <stdint.h> */

/* ======================== 循迹参数 (可调) ======================== */
#define GRAY_BASE_SPEED   0.40f     /* 基础线速度 m/s */
#define GRAY_W_MAX        2.1f     /* 最大角速度 rad/s */
#define GRAY_ERR_TO_W_GAIN 0.32f   /* 误差→角速度比例增益 (查表 err±9 → w≈±1.8) */
#define GRAY_K230_FF_GAIN  -0.0f  /* K230 线角度前馈增益 (rad/s per °, 弯道提前转向) */
#define GRAY_ERR_MAX      9.0f     /* 误差限幅 (查表最大 |err|=9, 供速度自适应归一化) */

/* 速度低通 v_smooth: 加速慢/减速快, 起步从 0 缓启动 */
#define GRAY_VSMOOTH_ACCEL 0.18f   /* 加速系数 (小 → 起步缓提速) */
#define GRAY_VSMOOTH_BRAKE 0.55f   /* 减速系数 (大 → 过弯快速减速) */
#define GRAY_SLOW_FACTOR   0.5f   /* 误差自适应: 满偏差时基础速度衰减比例 */
#define GRAY_IDLE_RESET_MS 200U    /* 距上次更新超过该值 → 新一段循迹, 重新缓启动 */

/* 保留供 trace_tune 串口调参引用 (本控制器已改比例转向, 不再用 PID) */
#define GRAY_KP           0.08f    /* PID P */
#define GRAY_KI           0.0f     /* PID I */
#define GRAY_KD           0.08f    /* PID D */

/* ======================== 数据结构 ======================== */
typedef struct {
    Grayscale_Sensor_t sensor;   /* 灰度传感器状态 */
    float              cur_speed;    /* v_smooth 当前线速度 (从 0 缓启动) */
    uint32_t           last_tick;    /* 上次更新 tick (用于检测新一段循迹) */
    uint8_t            inited;
} GrayTrace_t;

/* ======================== API ======================== */
void GrayTrace_Init(GrayTrace_t *gt);            /* 初始化 GPIO */
void GrayTrace_Update(GrayTrace_t *gt);          /* 读灰度 → 查表 → 比例 → 麦轮输出 */
float GrayTrace_Calc_Error(GrayTrace_t *gt);     /* 只算 Track_Err 偏差 (供调试) */
uint8_t Grayscale_Serial_Read(void);
/* ======================== 串口调参 (USART1, '$' 前缀, 与 trace_tune 的 '#' 不冲突) ======================== */
#define GRAY_TUNE_CMD_MAX 24U
bool GrayTrace_Tune_OnByte(uint8_t b);   /* USART1 RX 回调分流: $egain / $ffgain / $get */
extern float g_gray_err_gain;            /* 反馈增益 (上电默认 GRAY_ERR_TO_W_GAIN) */
extern float g_gray_ff_gain;             /* 前馈增益 (上电默认 GRAY_K230_FF_GAIN) */

#endif /* GRAYTRACE_H */
