/**
 * @file    worker_task.h
 * @brief   Worker 任务 —— FC_TASK / NLF_TASK
 *
 *    FC_TASK   周期 100ms, 发送 NX 工作模式
 *    NLF_TASK  阻塞式跑完整条比赛流程 (Mode → 执行体见 NLF_RunFlow())
 */
#ifndef WORKER_TASK_H
#define WORKER_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "banyuntask.h"     /* SystemMode_t / TaskCommand_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 任务参数
 * ================================================================ */

/** 任务周期, 单位 ms。 */
#define AG_TASK_PERIOD_MS       10u
#define FC_TASK_PERIOD_MS       100u

#define FC_TASK_STACK_WORDS     256u     /* 1 KB  (对应 .ioc: FC_TASK,256) */
#define NLF_TASK_STACK_WORDS    1024u    /* 4 KB  (对应 .ioc: NLF_TASK,1024) */

/* NLF_TASK 的线程标志 */
#define NLF_FLAG_RUN            0x01u

/* ================================================================
 * 句柄 (定义在 worker_task.c)
 * ================================================================ */
extern osThreadId_t fcTaskHandle;
extern osThreadId_t nlfTaskHandle;

/* FC_TASK 与应用层的契约量 (定义在 worker_task.c) */
extern volatile uint8_t g_angle_ctrl_enable;
extern volatile float   g_angle_target_yaw;

/* 路线段打断 (跨任务): gripper_task 在 IR 进料时置位, Nav_GoToWorld() / Arc_Run()
 * 读到就停车收尾。
 * @note 电平不是队列 —— 没人消费会一直留着, 没有路线在跑的入口要先清。 */
extern volatile uint8_t g_route_abort;

/** @brief 请求打断当前路线段。@note 默认空实现 (NF_ABORT_ON_FEED = 0)。 */
void Route_AbortRequest(void);

/* 圆弧/平移量 (arc_path 用)。
 * @warning g_angle_ctrl_w_ff 是前馈, 绕过 PID 直接叠加在内环输出上,
 *          不受 angle_ctrl 的限幅约束 —— 调用方自己保证量程。 */
extern volatile float   g_angle_ctrl_speed;  /* 目标线速度 m/s, >0 前进 */
extern volatile float   g_angle_ctrl_w_ff;   /* 前馈角速度 rad/s, CCW 为正 */

/* ================================================================
 * API
 * ================================================================ */

/** @brief FC_TASK 入口 (osThreadFunc_t 签名)。 */
void FC_Fuction(void);

/** @brief NLF_TASK 入口 (osThreadFunc_t 签名)。 */
void NLF_Fuction(void);

 /** @brief AC_TASK 入口 (osThreadFunc_t 签名)。 */
 void Angle_Fuction(void);

/**
 * @brief  请求 NLF_TASK 执行一个 Mode。由 defaultTask 调度器调用。
 * @param  mode  待执行的系统事件, 见 banyuntask.h 的 SystemMode_t。
 * @note   只暂存最新的一个 Mode: 同一时刻只跑一条流程。若 NLF_TASK
 *         尚未创建 (句柄为 NULL), 只暂存不唤醒, 不报错。
 */
void NLF_Request(SystemMode_t mode);

/**
 * @brief  流程主体 —— 由 NLF_TASK 在收到事件后调用。
 * @param  mode  要执行的流程。
 *
 * @note  流程编排与顺序表都在 worker_task.c。本函数**阻塞式**, 一个阶段
 *        跑完才返回 (找圆/放置各有超时上限)。
 */
void NLF_RunFlow(SystemMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* WORKER_TASK_H */
