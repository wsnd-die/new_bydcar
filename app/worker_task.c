/**
 * @file    worker_task.c
 * @brief   Worker 任务实现 —— FC_TASK (角度环) / NLF_TASK (流程)
 *          架构说明见 worker_task.h。
 */
#include "Common_used.h"
#include "angle_ctrl.h"
#include "mecanum.h"
#include "HWT906.h"
#include "Send_motor.h"
#include "Circle_base.h"
#include "NavigationMecanum.h"
#include "arc_path.h"
#include "banyuntask.h"
#include "worker_task.h"

#include "../uart/NX_uart4.h"

/* ==================================================================
 * 一、FC_TASK 与 NavigationMecanum 之间的契约量
 *
 * 这两个量的 extern 声明在 app/worker_task.h:77-78,
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

/* V1.13.0 追加。默认 0 → FC_TASK 行为与前版一致 (纯原地转向)。 */
volatile float   g_angle_ctrl_speed  = 0.0f; /* 目标线速度 m/s, >0 前进 */
volatile float   g_angle_ctrl_w_ff   = 0.0f; /* 前馈角速度 rad/s, CCW 为正 */

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

/* 实例输出 rad，角度环按 deg 工作（angle_ctrl.h），量纲在此换算 */
#define RAD2DEG(r) ((r) * 57.2957795131f)

/* ==================================================================
 * 三、FC_TASK —— 角度环
 * ================================================================== */

/**
 * @brief 关闭角度环时的主动刹停。
 * @note  发送零速而不是"什么都不发": 闭环步进电机在速度模式下会一直执行
 *        最后一次收到的目标速度。Mecanum_Calc(0,0) 四轮速度均为 0。
 */
static void AG_Stop(void)
{
    MecanumResult zero = Mecanum_Calc(0.0f, 0.0f);
    Send_commandmotor(&zero);
}
void Angle_Fuction(void)
{
    uint8_t  was_on    = 0;      /* 上一拍的 g_angle_ctrl_enable, 用于取边沿 */

    imu_hwt906.init();
    Angle_Init(&s_fc);

    for (;;)
    {
        PoseData_t pose;

        imu_hwt906.update();
        imu_hwt906.get_pose(&pose);

        float yaw   = RAD2DEG(pose.yaw);
        float w_deg = RAD2DEG(pose.wz);

        if (g_angle_ctrl_enable)
        {
            if (!was_on) {
                Angle_SetTarget(&s_fc, g_angle_target_yaw);
                was_on = 1;
            } else {
                Angle_UpdateTarget(&s_fc, g_angle_target_yaw);
            }

            Angle_Update(&s_fc, yaw, w_deg);

            /* ---- 前馈 + 平移 (V1.13.0) ----------------------------------
             * 线速度与角速度一起下发, 才是圆弧: (v, ω=v/R) → 车头恒为切线。
             * V1.13.0 之前这里是 Mecanum_Calc(0.0f, s_fc.cmd_w) —— 线速度写死
             * 0, 只能原地转向, 走不出弧。
             *
             * @note `g_angle_ctrl_w_ff` **加在内环输出之后**, 是有意的:
             *       angle_ctrl.c 的 `gyro_scale = 0.05` 使内环只看到 5% 的
             *       真实角速度, 它不是真正的速度环。前馈若加在 PID 目标上
             *       (target_w) 会被这个缩放揉坏; 加在输出上则几何量直通,
             *       PID 只在此基础上修残差。详见 worker_task.h 的契约说明。
             *
             * cmd_w 单位 rad/s, 与 Mecanum_Calc 的 w 同量纲。 */
            MecanumResult cmd = Mecanum_Calc(g_angle_ctrl_speed,
                                             s_fc.cmd_w + g_angle_ctrl_w_ff);
            Send_commandmotor(&cmd);
        }
        else if (was_on)
        {
            /* 下降沿: 刹停, 让调用方的 osDelay(20) 有意义 */
            AG_Stop();
            s_fc.state = ANGLE_IDLE;
            was_on = 0;
        }
        osDelay(AG_TASK_PERIOD_MS);
    }
}
void FC_Fuction(void)
{
    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();
    osDelay(FC_TASK_PERIOD_MS);
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

void NLF_Fuction(void)
{

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
        case Event_FindCircle:
            Circle_Follow();
            break;

        case Event_Navigation:
        case Event_GoHome:
            Nav_RunWaypoints();
            break;

        case Event_ArcRun:
            /* 跑一段定半径圆弧 (车头恒为切线)。参数先用默认值 ——
             * 需要改半径/速度/角度就在触发前调 Arc_SetParam()。
             * 见 algorithm/arc_path.h。 */
            Arc_Run();
            break;

        case Event_STOP:
            /* 急停: 关角度环 (FC_TASK 随即主动刹停)。
             * 注意流程任务自身若正阻塞在导航/循迹里, 本分支拦不住它 ——
             * 包括正阻塞在 Arc_Run() 里的情况 (那份超时保护是兜底, 不是急停)。 */
            Arc_Abort();                /* = 关环 + 清线速度/前馈, 取代原来那句裸的置 0 */
            break;

        /* ---- 循迹整体已移除 (V1.6.0) ----
         * Event_LinFolL / Event_LinFolR 不再有执行体: algorithm/Trace_base.c
         * 与 app/GrayTrace.c 已删除。枚举值保留在 banyuntask.h 里未动,
         * 若将来换用别的循迹方案, 在这里接一个新分支即可。
         */

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
