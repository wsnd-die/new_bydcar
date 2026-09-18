/**
 * @file    worker_task.h
 * @brief   Worker 任务 —— 系统事件队列的消费者
 *
 *  架构 (见 app/banyuntask.h 的文件头):
 *
 *      驱动源 → defaultTask 调度器 → Worker 任务
 *
 *  本文件实现两个 Worker:
 *
 *    FC_TASK   周期 10ms 的角度环 (角度环 → 角速度环 串级 PID)。
 *              由 g_angle_ctrl_enable 门控, 目标角 g_angle_target_yaw,
 *              执行体在 algorithm/angle_ctrl.c。
 *              同时负责周期性刷新 HWT906 —— 本任务周期最短, 由它统一喂。
 *
 *    NLF_TASK  阻塞式跑完整条比赛流程。调度器用线程标志唤醒,
 *              具体 Mode → 执行体的映射见 worker_task.c 的 NLF_RunFlow()。
 *
 *  @note V1.3.0 建立。此前 FC_TASK / NLF_TASK 只存在于注释里,
 *        全工程只有 defaultTask 一个空循环任务。
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

/**
 * FC_TASK 周期, 单位 ms。
 * @warning angle_ctrl.c 的两级 PID 增益按 10ms 整定 (见该文件 DT 宏),
 *          改这里等于改整定。另注: 下发用的 Send_commandmotor() 内部含
 *          osDelay(5), 所以角速度环看到的实际 dt 会略大于本值 ——
 *          任务里用实测 dt 做差分补偿, 不依赖本常量算角速度。
 */
#define FC_TASK_PERIOD_MS       10u

/* 栈深, 单位: 字 (1 字 = 4 字节)。
 *
 * FC_TASK 很浅: 只有几个局部标量, AngleCtrl 实例是 static 的, 不在栈上。
 *
 * NLF_TASK 要跑整条调用链 —— NLF_RunFlow → Nav_RunWaypoints → Nav_GoToWorld
 * → Mecanum_MoveWithEncoder → Send_commandmotor, 且沿途多处调用 printf
 * (Mecanum_Move.c / NavigationMecanum.c / GrayTrace.c 等)。
 * newlib-nano 带 -u _printf_float 时, 一次 printf 就要几百字节栈,
 * 所以这里给足 4KB, 不要按"只放局部变量"估。 */
#define FC_TASK_STACK_WORDS     256u     /* 1 KB  */
#define NLF_TASK_STACK_WORDS    1024u    /* 4 KB  */

/* NLF_TASK 的线程标志 */
#define NLF_FLAG_RUN            0x01u

/* ================================================================
 * 句柄 (定义在 worker_task.c)
 * ================================================================ */
extern osThreadId_t fcTaskHandle;
extern osThreadId_t nlfTaskHandle;

/* ================================================================
 * FC_TASK 与应用层的契约量 (唯一定义处: worker_task.c)
 *
 * 调用方置 g_angle_ctrl_enable = 1 打开角度环, 目标角写 g_angle_target_yaw;
 * 收敛后置 0, FC_TASK 随即主动刹停。
 * 完整契约说明见 worker_task.c 文件头。
 * ================================================================ */
extern volatile uint8_t g_angle_ctrl_enable;
extern volatile float   g_angle_target_yaw;

/* ================================================================
 * API
 * ================================================================ */

/** @brief FC_TASK 入口 (osThreadFunc_t 签名)。 */
void FC_Task (void *argument);

/** @brief NLF_TASK 入口 (osThreadFunc_t 签名)。 */
void NLF_Task(void *argument);

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
 * @note   各 Mode 的执行体已存在, 但**整条流程的顺序编排尚未确定**,
 *         见 worker_task.c 中的说明与 CLAUDE.md 变更日志 V1.3.0。
 */
void NLF_RunFlow(SystemMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* WORKER_TASK_H */
