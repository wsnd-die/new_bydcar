/**
 * @file    task.h
 * @brief   事件队列
 *
 *  子事件 (LinFolL/LinFolR/PlaceDown/Navigation/QRCode/FindCircle):
 *    在 Worker 任务 (NLF_TASK 等) 中按 Mode 执行
 */

#ifndef __TASK_H
#define __TASK_H

/*
 * FreeRTOS.h / queue.h 已由 Common_used.h 统一提供，
 * 本文件不再重复包含。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 系统事件（驱动源 → defaultTask 调度器 → Worker 任务）
 * ============================================================ */
typedef enum {
     Event_Navigation           = 1,   /* 导航到目标点 */
     Event_LinFolL              = 2,   /* 循迹左 (收集物块) */
     Event_LinFolR              = 3,   /* 循迹右 (收集奖杯) */
     Event_STOP                 = 4,   /* 停止 */
     Event_STEERING_ROTATE      = 5,   /* 舵机旋转 */
     Event_QRCode               = 6,   /* 识别二维码 → SetQR(idx) */
     Event_FindCircle           = 7,   /* 找圆 */
     Event_PickUp               = 8,   /* 拾取(转盘颜色收集) */
     Event_PlaceDown            = 9,   /* 放置 */
     Event_GoHome               = 10,   /* 回家 */
} SystemMode_t;

/* ============================================================
 * 命令结构体
 * ============================================================ */
typedef struct {
    uint8_t      k;       /* 1=有效 */
    SystemMode_t Mode;
} TaskCommand_t;

/* ============================================================
 * 全局队列
 * ============================================================ */

extern QueueHandle_t systemEventQueue ;


/* ============================================================
 * API
 * ============================================================ */
void           task_init(void);
void           task_send(SystemMode_t mode);
TaskCommand_t  task_recive(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_H */
