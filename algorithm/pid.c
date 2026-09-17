/**
 * @file    pid.c
 * @brief   通用 PID — 位置式 / 增量式 / 积分分离增量式
 */
#include "Common_used.h"

#define LIMIT_MAX(input, max)   \
    do {                        \
        if ((input) > (max))    \
            (input) = (max);    \
        else if ((input) < -(max)) \
            (input) = -(max);   \
    } while (0)

/**
 * @brief  PID 初始化
 * @param  pid      句柄指针
 * @param  mode     模式: PID_POSITION / PID_DELTA / PID_DELTA_INTE
 * @param  PID      三参数数组 {Kp, Ki, Kd}
 * @param  max_out  输出限幅
 * @param  max_iout 积分限幅
 */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
        return;

    pid->mode        = (pid_mode_e)mode;
    pid->Kp          = PID[0];
    pid->Ki          = PID[1];
    pid->Kd          = PID[2];
    pid->max_out     = max_out;
    pid->max_iout    = max_iout;
    pid->I_threshold = 0.0f;

    PID_clear(pid);
}

/**
 * @brief  PID 计算
 * @param  pid  句柄指针
 * @param  ref  反馈值 (实际)
 * @param  set  设定值 (目标)
 * @retval 控制输出
 */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
        return 0.0f;

    /* 更新误差历史 */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set      = set;
    pid->fdb      = ref;
    pid->error[0] = set - ref;

    if (pid->mode == PID_POSITION)
    {
        /* ---- 位置式 PID ---- */
        pid->Pout     = pid->Kp * pid->error[0];
        pid->Iout    += pid->Ki * pid->error[0];
        pid->Dbuf[2]  = pid->Dbuf[1];
        pid->Dbuf[1]  = pid->Dbuf[0];
        pid->Dbuf[0]  = (pid->error[0] - pid->error[1]);
        pid->Dout     = pid->Kd * pid->Dbuf[0];

        LIMIT_MAX(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LIMIT_MAX(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        /* ---- 增量式 PID ---- */
        pid->Pout     = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout     = pid->Ki * pid->error[0];
        pid->Dbuf[2]  = pid->Dbuf[1];
        pid->Dbuf[1]  = pid->Dbuf[0];
        pid->Dbuf[0]  = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout     = pid->Kd * pid->Dbuf[0];
        pid->out     += pid->Pout + pid->Iout + pid->Dout;

        LIMIT_MAX(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA_INTE)
    {
        /* ---- 带积分分离的增量式 PID ---- */
        pid->Pout     = pid->Kp * (pid->error[0] - pid->error[1]);

        if (fabsf(pid->error[0]) < pid->I_threshold) {
            pid->Iout = 0.0f;
        } else {
            pid->Iout = pid->Ki * pid->error[0];
        }

        pid->Dbuf[2]  = pid->Dbuf[1];
        pid->Dbuf[1]  = pid->Dbuf[0];
        pid->Dbuf[0]  = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout     = pid->Kd * pid->Dbuf[0];
        pid->out     += pid->Pout + pid->Iout + pid->Dout;

        LIMIT_MAX(pid->out, pid->max_out);
    }

    return pid->out;
}

/**
 * @brief  清零 PID 所有状态
 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
        return;

    pid->set  = 0.0f;
    pid->fdb  = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0]  = pid->Dbuf[1]  = pid->Dbuf[2]  = 0.0f;
    pid->out  = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
}
