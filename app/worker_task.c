/**
 * @file    worker_task.c
 * @brief   Worker 任务实现 —— FC_TASK (角度环) / NLF_TASK (流程)
 *          架构说明见 worker_task.h。
 */
#include "Common_used.h"
#include "angle_ctrl.h"
#include "mecanum.h"
#include "hwt_imu.h"
#include "Send_motor.h"
#include "Trace_base.h"
#include "Circle_base.h"
#include "NavigationMecanum.h"
#include "banyuntask.h"
#include "worker_task.h"

/* ==================================================================
 * 一、FC_TASK 与 NavigationMecanum 之间的契约量
 *
 * 这两个量的 extern 声明在 hardware/Common_used.h:139-140,
 * 本文件是它们**唯一的定义处**。
 *
 * 背景: V1.3.0 之前它们被 app/NavigationMecanum.c 引用却全工程无定义。
 * 之所以一直没暴露成链接错误, 是因为整个 app/ 未被任何任务引用,
 * 被 -Wl,--gc-sections 整段回收了 —— 未引用段里的未定义符号不参与解析。
 * 一旦任务真正接上, 这两个量就必须有定义。
 *
 * 契约 (调用方见 app/NavigationMecanum.c:184-195):
 *   1. 调用方写 g_angle_target_yaw = 目标角度(deg), 再置 g_angle_ctrl_enable = 1;
 *   2. 调用方轮询 g_hwt_imu_yaw 直到误差收敛 (容差 2°/3°);
 *   3. 调用方置 g_angle_ctrl_enable = 0, 再 osDelay(20) 等本任务停止输出。
 *
 * 第 3 步的 osDelay(20) 是有意义的: FC_TASK 在下降沿必须**主动下发一次
 * 零速**, 否则电机保持最后一次收到的速度指令不放手。
 * ================================================================== */
volatile uint8_t g_angle_ctrl_enable = 0;    /* 1 = 打开角度环 */
volatile float   g_angle_target_yaw  = 0.0f; /* 目标航向 (deg), 与 g_hwt_imu_yaw 同量纲 */

osThreadId_t fcTaskHandle  = NULL;
osThreadId_t nlfTaskHandle = NULL;

/* 角度环状态。AngleCtrl 约 230 字节, 放 static 不放栈上。 */
static AngleCtrl s_fc;

/* 调度器 → NLF_TASK 的事件暂存。
 * 同一时刻只跑一条流程, 且调度器先写 Mode 再置标志, 故无需再加一层队列。 */
static volatile SystemMode_t s_nlf_pending = Event_STOP;

/* ==================================================================
 * 二、工具
 * ================================================================== */

/** @brief 角度归一到 [-180, 180)。 */
static float norm_deg180(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* ==================================================================
 * 三、FC_TASK —— 角度环
 * ================================================================== */

/**
 * @brief 关闭角度环时的主动刹停。
 * @note  发送零速而不是"什么都不发": 闭环步进电机在速度模式下会一直执行
 *        最后一次收到的目标速度。Mecanum_Calc(0,0) 四轮速度均为 0。
 */
static void FC_Stop(void)
{
    MecanumResult zero = Mecanum_Calc(0.0f, 0.0f);
    Send_commandmotor(&zero);
}

void FC_Task(void *argument)
{
    (void)argument;

    float    prev_yaw  = 0.0f;
    uint8_t  has_prev  = 0;      /* prev_yaw 是否有效, 避免第一拍算出巨大差分 */
    uint8_t  was_on    = 0;      /* 上一拍的 g_angle_ctrl_enable, 用于取边沿 */
    uint32_t prev_tick = osKernelGetTickCount();

    /* 探测在线并把当前航向设为零点。返回值忽略: 掉线情况由 g_hwt_imu_online 反映。 */
    HWT_IMU_Init();
    Angle_Init(&s_fc);

    for (;;)
    {
        /* HWT906 没有中断/DRDY 输出, 只能轮询刷新 (见 hardware/hwt_imu.h)。
         * 本任务是全工程周期最短的, 由它统一刷新, 里程计与导航共用同一份数据。 */
        HWT_IMU_Poll();

        uint32_t now   = osKernelGetTickCount();
        uint32_t dt_ms = now - prev_tick;
        prev_tick = now;

        float yaw = g_hwt_imu_yaw;   /* deg, 已扣零点, -180..180 */

        /* HWT906 只输出欧拉角, 拿不到原始角速度 (见 hwt_imu.h:10-13),
         * 角速度环的反馈只能靠 yaw 差分。用实测 dt 而不是常量:
         * 下发用的 Send_commandmotor() 内含 osDelay(5), 实际周期会大于 10ms。 */
        float w_deg = 0.0f;
        if (has_prev && dt_ms > 0u) {
            w_deg = norm_deg180(yaw - prev_yaw) * (1000.0f / (float)dt_ms);
        }
        prev_yaw = yaw;
        has_prev = 1;

        if (g_angle_ctrl_enable)
        {
            if (!was_on) {
                /* 上升沿: 复位 PID, 清掉上一次残留的积分 */
                Angle_SetTarget(&s_fc, g_angle_target_yaw);
                was_on = 1;
            } else {
                /* 连续追踪: 只更新目标, 不复位 PID */
                Angle_UpdateTarget(&s_fc, g_angle_target_yaw);
            }

            Angle_Update(&s_fc, yaw, w_deg);

            /* cmd_w 单位 rad/s, 与 Mecanum_Calc 的 w 同量纲 */
            MecanumResult cmd = Mecanum_Calc(0.0f, s_fc.cmd_w);
            Send_commandmotor(&cmd);
        }
        else if (was_on)
        {
            /* 下降沿: 刹停, 让调用方的 osDelay(20) 有意义 */
            FC_Stop();
            s_fc.state = ANGLE_IDLE;
            was_on = 0;
        }

        osDelay(FC_TASK_PERIOD_MS);
    }
}

/* ==================================================================
 * 四、NLF_TASK —— 流程
 * ================================================================== */

void NLF_Request(SystemMode_t mode)
{
    s_nlf_pending = mode;
    if (nlfTaskHandle != NULL) {
        osThreadFlagsSet(nlfTaskHandle, NLF_FLAG_RUN);
    }
}

void NLF_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t flags = osThreadFlagsWait(NLF_FLAG_RUN, osFlagsWaitAny, osWaitForever);

        if ((flags & NLF_FLAG_RUN) != 0u) {
            NLF_RunFlow(s_nlf_pending);
        }
    }
}

void NLF_RunFlow(SystemMode_t mode)
{
    switch (mode)
    {
        case Event_LinFolL:
        case Event_LinFolR:
            /* Trace_base.c:239 内部按这两个 Mode 选左/右循迹 */
            Trace_LineFollow(mode);
            break;

        case Event_FindCircle:
            Circle_Follow();
            break;

        case Event_Navigation:
        case Event_GoHome:
            Nav_RunWaypoints();
            break;

        case Event_STOP:
            /* 急停: 关角度环 (FC_TASK 随即主动刹停)。
             * 注意流程任务自身若正阻塞在导航/循迹里, 本分支拦不住它。 */
            g_angle_ctrl_enable = 0;
            break;

        /* ---- 以下四个的执行体都已存在, 但入参来源未定, 故暂不接线 ----
         *
         *   Event_QRCode          → SetQR(idx)
         *                           idx 需先从 QR_deel() 解析出 (Jang_Num / Yan_Num),
         *                           见 hardware/QRcode.c 与 ColorIdentif.c:84
         *   Event_PickUp          → BlockBasic_LiftTo() + ColorIdentif 槽位表
         *   Event_PlaceDown       → BL_Update() (app/BollLocator.c:98) / Place()
         *   Event_STEERING_ROTATE → BlockBasic_TurntableTo() / Servo_*
         *
         * 另: 整条流程的顺序编排 (谁发第一个事件、各步之间怎么衔接)
         *     尚未确定, 见 CLAUDE.md 变更日志 V1.3.0 备注。
         */
        default:
            break;
    }
}
