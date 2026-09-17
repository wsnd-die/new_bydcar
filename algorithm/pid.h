/**
 * @file    pid.h
 * @brief   通用 PID 控制器 — 位置式 / 增量式 / 积分分离增量式
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef float fp32;

typedef enum {
    PID_POSITION    = 0,   /* 位置式 PID */
    PID_DELTA       = 1,   /* 增量式 PID */
    PID_DELTA_INTE  = 2    /* 带积分分离的增量式 PID */
} pid_mode_e;

typedef struct {
    pid_mode_e mode;
    fp32 Kp, Ki, Kd;
    fp32 max_out;           /* 输出限幅 */
    fp32 max_iout;          /* 积分限幅 */
    fp32 I_threshold;       /* 积分分离阈值 */

    fp32 set;               /* 设定值 */
    fp32 fdb;               /* 反馈值 */
    fp32 error[3];          /* error[0]:当前, [1]:上次, [2]:上上次 */
    fp32 Dbuf[3];           /* 微分缓冲 */
    fp32 Pout, Iout, Dout, out;
} pid_type_def;

void PID_init (pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_calc (pid_type_def *pid, fp32 ref, fp32 set);
void PID_clear(pid_type_def *pid);

#endif