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
#define AG_TASK_PERIOD_MS       10u
#define FC_TASK_PERIOD_MS       100u
/* 栈深, 单位: 字 (1 字 = 4 字节)。
 *
 * FC_TASK 很浅: 只有几个局部标量, AngleCtrl 实例是 static 的, 不在栈上。
 *
 * NLF_TASK 要跑整条调用链 —— NLF_RunFlow → Nav_GoToWorld / Place /
 * TT_RotateByQR → BlockBasic_TurntableTo → SCS 总线, 且沿途多处调用 printf。
 * newlib-nano 带 -u _printf_float 时, 一次 printf 就要几百字节栈;
 * servo_scs.h 另要求 SCS 调用方留 ≥1KB 余量。故给足 4KB,
 * 不要按"只放局部变量"估。
 *
 * @warning **这两个宏已不是栈深的生效处** (2026-09-28 起)。
 *          任务由 CubeMX 按 .ioc 的 FREERTOS.Tasks01 创建, 栈深以那里为准;
 *          此前 NLF_TASK 实际只拿到 1KB, 与本文件声明的 4KB 不符, 已改正。
 *          改栈深请改 .ioc, 并回来同步这两个值以免误导。 */
#define FC_TASK_STACK_WORDS     256u     /* 1 KB  (对应 .ioc: FC_TASK,256) */
#define NLF_TASK_STACK_WORDS    1024u    /* 4 KB  (对应 .ioc: NLF_TASK,1024) */

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

/* ---- V1.23.0 追加: 路线段打断 (跨任务) ------------------------------
 *
 *  置位方: `gripper_task` —— `app/block_collect.c` 的 `wait_block_entered()`
 *          在 IR 检测到物块进料时调 `Route_AbortRequest()`。
 *  消费方: 正在跑路线的那个函数 —— `Nav_GoToWorld()` / `Arc_Run()` 在自己的
 *          轮询循环里读到就 **停车、清零、提前收尾**。
 *
 *  语义: **"别走当前这一段了, 立刻切下一段"**。用户要的是"物块一进料口就
 *  改奔下一个点", 不是"走完这段再说"。
 *
 *  @note 这是**电平不是队列** —— 没人消费时它会一直留着, 于是下一段路线
 *        第一拍就命中, 白跳一格。所以每个"没有路线在跑"的入口
 *        (如 `NF_Stage_GoHome()`) 都要先把它清掉。
 *  @note 单核 Cortex-M4 上单字节 volatile 的读/写是单条指令, 天然原子,
 *        **不需要临界区**。读的一方 (NLF_TASK) 优先级更高, 写的一方
 *        (gripper_task) 只写不读。
 *  @note **刻意不用 `NLF_Request()` / 线程标志**: 打断发生时 NLF_TASK 正阻塞在
 *        `Nav_GoToWorld()` 的 `osDelay()` 里, 根本不在 `osThreadFlagsWait` 上,
 *        线程标志对它毫无作用; 而且会把 `NLF_FLAG_RUN` 留成置位, 等当前流程
 *        返回后**多跑一次 `NLF_RunFlow`**。 */
extern volatile uint8_t g_route_abort;

/** @brief 请求打断当前路线段 (下一段立刻开始)。见 `g_route_abort`。
 *
 *  @note **当前默认是空实现**(`NF_ABORT_ON_FEED = 0`, 见 worker_task.c):
 *        在"分点导航 + 到位蹭料"模式下, 物块是**车到位之后**才被 creep 顶进
 *        进料口的 —— 那时没有段可以打断; 而进料事件上报得晚 50~150ms(采集侧的
 *        IR 判据内部有去抖), 迟到的打断会打到**下一段**头上, 让它刚起步就被
 *        打断而游标照常推进 → **平白跳一个点位**。
 *        (剪掉 creep、改回"边开边收"时, 把那个宏改回 1 即可恢复本机制。) */
void Route_AbortRequest(void);

/* ---- V1.13.0 追加: 圆弧/平移量 ------------------------------------
 * @note **默认全 0**, 因此不设置它们时 FC_TASK 的行为与 V1.12.0 完全一致
 *       (纯原地转向, 线速度为 0)。这是向后兼容的扩展, 不动上面两条。
 *
 * 用法 (跑圆弧, 见 algorithm/arc_path.c):
 *     g_angle_ctrl_speed  = v;        // 线速度
 *     g_angle_ctrl_w_ff   = v / R;    // ★ 前馈角速度, 圆弧的几何量
 *     g_angle_ctrl_enable = 1;
 *
 * @warning `g_angle_ctrl_w_ff` 是**前馈**, 它绕过 angle_ctrl 的两级 PID,
 *          直接叠加在内环的输出上 (见 worker_task.c 的 FC_Task)。
 *          加在输出而非 PID 目标上是有意的: angle_ctrl.c 的 `gyro_scale = 0.05`
 *          使内环只看到 5% 的真实角速度, 不是真正的速度环, 前馈走目标会被揉坏。
 *          副作用是它**不受 angle_ctrl 的 CFG_MAX_W 限幅约束** ——
 *          调用方自己保证量程, arc_path 用 ARC_W_MAX 做这道校验。 */
extern volatile float   g_angle_ctrl_speed;  /* 目标线速度 m/s, >0 前进 */
extern volatile float   g_angle_ctrl_w_ff;   /* 前馈角速度 rad/s, 逆时针(CCW)为正 */

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
 * @note  2026-09-28 起这里已是**完整的比赛流程编排**, 不再是空壳:
 *
 *            Event_Navigation  中继站, 查顺序表派发下一阶段
 *            Event_LinFolL     左循迹/收集物块   [打桩中]
 *            Event_FindCircle  找圆 → 放一个物块
 *            Event_LinFolR     右循迹/收集奖杯   [打桩中]
 *            Event_PlaceDown   放一个奖杯
 *            Event_GoHome      回家停车
 *
 *        顺序表与硬编码默认值在 worker_task.c 的 4.1 / 4.2 两节。
 *        不扫二维码, 奖杯顺序与槽位颜色暂由硬编码顶上, 等 NX 报文接入
 *        (每个表上都标了「★ NX 接入点」)。
 *
 * @warning 本函数**阻塞式**: 一个阶段跑完才返回 (找圆阶段有 30s 上限)。
 *          FC_TASK / angle_Task 必须能抢占它, 否则控制周期被拉长。
 */
void NLF_RunFlow(SystemMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* WORKER_TASK_H */
