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

#include "../uart/NX_uart.h"

/* ---- 比赛流程用到的执行体 (见 4.3 各阶段执行体) ----
 * @note 这些是驱动层/业务层的头, 应用层直接 include 是**分层债** ——
 *       规范 (CLAUDE.md §1) 目标是应用层只经业务层接口。当前工程尚未落地
 *       那层封装, 旧版 NLF_TASK 也是这么直调的, 故沿用并在变更记录里记一笔。 */
#include "ColorIdentif.h"        /* TT_Init / TT_SetColor / SetQR / TT_RotateByQR ... */
#include "QRcode.h"              /* Jang_type: champion / second_place / third_place */
#include "block_basic.h"         /* Place / BlockBasic_TurntableTo / BlockBasic_LiftTo */
#include "drv_wheel_odom.h"      /* Wheel_Odom_Reset (旧代码的 World_Reset) */

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

        float yaw   = -RAD2DEG(pose.yaw);
        float w_deg = -RAD2DEG(pose.wz);
        // printf("%f,%f\r\n",yaw,w_deg);
        if (g_angle_ctrl_enable)
        {
            if (!was_on) {
                Angle_SetTarget(&s_fc, g_angle_target_yaw);
                was_on = 1;
            } else {
                Angle_UpdateTarget(&s_fc, g_angle_target_yaw);
            }

            Angle_Update(&s_fc, yaw, w_deg);
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
    static bool s_kicked = false;

    /* ---- 起跑 ----------------------------------------------------
     * 旧 NLF_TASK 起来就先 task_send(Event_Navigation) 自动开流程。这里保留
     * 同样的行为, 但走 NLF_Request (线程标志) 而不是事件队列 —— 因为队列的
     * 生产者至今为零, 且队列那条路仍留给将来的上位机/调试指令。
     *
     * 由 NF_AUTOSTART 控制 (见 4.2): 置 0 则必须由外部 task_send() 或其他任务
     * 调 NLF_Request() 才会起步。 */
    if (!s_kicked) {
        s_kicked = true;
#if NF_AUTOSTART
        NLF_Request(Event_Navigation);
#endif
    }

    uint32_t flags = osThreadFlagsWait(NLF_FLAG_RUN, osFlagsWaitAny, osWaitForever);

    if ((flags & NLF_FLAG_RUN) != 0u) {
        NLF_RunFlow(s_nlf_pending);
    }
}

/* ==================================================================
 * 4.1  比赛流程的硬编码默认值   ★★★ 临时, 等 NX 报文接入后删除 ★★★
 *
 * 背景 (2026-09-28): 旧版流程里这两组数据来自 K230 扫二维码 ——
 * Task1 左码 (0~15) 定五个物块槽位, Task2 右码 (1~6) 定三个奖杯槽位。
 * 现在决定**不再扫二维码**, 改由上位机 NX (UART4) 回传:
 *     · 收集奖杯的 1/2/3 顺序
 *     · 收集到的颜色物块
 *
 * NX 侧本次**一行未动** (uart/NX_uart.c 协议未扩展), 所以先在这里写死一组
 * 默认值把流程串通。等 NX 协议定下来, 只需改 4.2 的 NF_FlowSeed() ——
 * 下面每个表上都标了 ★ NX 接入点, 流程主体一行都不用改。
 *
 * @warning 改这里等于改比赛策略, 上机前务必确认。
 * ================================================================== */

#define NF_RANK_COUNT        3u    /* 奖杯个数 */
#define NF_TASK1_SLOT_COUNT  5u    /* 物块槽位数 */




/** 奖杯放置顺序 (按放置先后): 亚军 → 冠军 → 季军。
 *  对应旧 NLF_TASK 的 rank[3] = {second_place, champion, third_place}。
 *  ★ NX 接入点: 换成 NX 回传的 1/2/3 顺序。 */
static const Jang_type NF_RANK[NF_RANK_COUNT] = {
    second_place, champion, third_place,
};

/** 每个奖杯对应的转盘工位 (1~5)。
 *  原由 Slop_dirjang() 从 QR 右码 T2[Jang_Num-1][...] 解出; QR 移除后直接写死。
 *  取值 = T2 第 1 行 {金, 银, 铜} 的倒序映射, 与 Jang_Num=1 时 Slop_dirjang()
 *  的输出逐项一致 (冠军→3, 亚军→2, 季军→1)。
 *  ★ NX 接入点: NX 若直接回传工位号就换本表; 若回传 QR 图案号, 换回
 *    Slop_dirjang() (那需要把 Jang_Num 也一并喂进去)。 */
static const uint8_t NF_TROPHY_SLOT[4] = {
    0u,   /* [0] 占位 —— Jang_type 从 1 (champion) 开始, 下标 0 不使用 */
    3u,   /* champion      → 转盘工位 3 */
    2u,   /* second_place  → 转盘工位 2 */
    1u,   /* third_place   → 转盘工位 1 */
};

/** 各奖杯放置时的丝杆高度 (mm)。。 */
static const uint16_t NF_PLACE_HEIGHT[4] = { 0u, 37u, 28u, 17u };

/** 五个槽位里实际放的物块颜色。
 *  ★ NX 接入点: 换成 NX 回传的「收集到的颜色物块」。
 *  @note 旧流程这里由 BsRt_task + 颜色传感器**本地读色**填入; 按用户决定
 *        (2026-09-28) 下位机不再读色, 改由 NX 回传, 故现在写死。 */
static const Color_TypeDef NF_SLOT_COLORS[NF_TASK1_SLOT_COUNT] = {
    COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK,
};

/** 物块任务的槽位映射图案号 (0~15), 即旧 QR 左码解出的序号。
 *  ★ NX 接入点: 换成 NX 回传的映射序号。 */
#define NF_TASK1_QR_IDX     0u

/* ==================================================================
 * 4.2  流程顺序表与状态
 * ================================================================== */

/** 顺序表的一项: 跑哪个阶段、连跑几次。
 *  对应旧 NLF_TASK 的 Navafter_mode[] / NavafterNum[] 两张平行数组 ——
 *  这里合成了一个结构体, 免得两数组长度对不上。 */
typedef struct {
    SystemMode_t mode;
    uint8_t      times;
} NF_Stage_t;


/* ==================================================================
 * ⚠⚠【临时改动 2026-09-28 —— 正在单独测分点导航, 测完必须改回来】⚠⚠
 *
 * 换成单站表后, 每个中继站都再派发一次 Navigation, 于是 NLF_TASK 一次循环
 * 走**一个**路径点, 走完整张表后自动 GoHome。好处是不受循迹打桩、
 * 找圆 30s 超时这些噪声干扰, 能单独验证 Nav_FeDuanPoint() 的推进和
 * g_waypoints[] 里的坐标。
 *
 * 恢复办法: 删掉下面这张单站表, 把 #if 0 改成 #if 1 (或直接删掉那两行)。
 *
 * @note 用 #if 0 而不是块注释包住原表 —— 原表里本来就有块注释, 嵌套会炸。
 * ================================================================== */
static const NF_Stage_t NF_STAGES[] = {
    { Event_Navigation, 30u },   /* 足够走完 17 个点, 多出来的次数空转 */
};
#define NF_STAGE_COUNT  (sizeof(NF_STAGES) / sizeof(NF_STAGES[0]))

#if 0   /* ---- 原表: 完整比赛流程 (测完改回 #if 1) ---- */
static const NF_Stage_t NF_STAGES[] = {
    { Event_LinFolL,    1u },   /* 左循迹: 收集物块 */
    { Event_FindCircle, 5u },   /* 找圆 ×5: 一个个放物块 */
    { Event_LinFolR,    1u },   /* 右循迹: 收集奖杯 */
    { Event_PlaceDown,  3u },   /* 放奖杯 ×3 */
};
#endif

/* 流程进度。对应旧 NLF_TASK 的 P_Nava / NavafterNum[P_Nava] / i / flag_finish。 */
static uint8_t s_stage_idx  = 0u;   /* 当初的 P_Nava */
static uint8_t s_stage_left = 0u;   /* 当初的 NavafterNum[P_Nava] */
static uint8_t s_place_idx  = 0u;   /* 当初的 i, 奖杯放置进度 */
static bool    s_place_latch = false; /* 当初的 flag_finish, 本阶段转盘是否已就位 */
static bool    s_flow_seeded = false; /* NF_FlowSeed() 的一次性门闩 */

/** 找圆阶段的等待上限。Circle_Follow() 是单拍函数, 靠外层循环推进;
 *  上位机没接 / 没识别到圆心时 g_circle_dir 永远不会变成 'O',
 *  没有这道超时流程会永久卡死在这里。 */
#define NF_CIRCLE_TIMEOUT_MS   30000u

/** 上电是否自动开跑。0 = 等调度器收到事件再跑。 */
#ifndef NF_AUTOSTART
#define NF_AUTOSTART  0
#endif

/**
 * @brief 把 4.1 的硬编码默认值灌进业务层的数据结构。只跑一次。
 * @note  这是 NX 接入时**唯一**要改的函数体。
 */
static void NF_FlowSeed(void)
{
    if (s_flow_seeded) {
        return;
    }
    s_flow_seeded = true;

    TT_Init();
    for (uint8_t s = 0; s < NF_TASK1_SLOT_COUNT; s++) {
        TT_SetColor(s, NF_SLOT_COLORS[s]);
    }
    /* 打开 TT_RotateByQR() 的"QR 模式"分支
     * 它按 task_color[] 查物理槽位, 比默认的"1→5 顺转"更贴合旧行为。 */
    SetQR(NF_TASK1_QR_IDX);

    /* --- 流程进度复位 --- */
    s_stage_idx   = 0u;
    s_stage_left  = NF_STAGES[0].times;
    s_place_idx   = 0u;
    s_place_latch = false;
    TT_RotateReset();
}

/**
 * @brief 取下一个阶段并推进进度 (旧 NLF_TASK 里那段 NavafterNum 自减逻辑)。
 * @param  out  非空时写入下一个阶段。
 * @return true 顺序表还有下一项; false 已跑完 (调用方应转 Event_GoHome)。
 * @note   推进规则与旧代码逐行一致: 先派发**当前**项, 再自减;
 *         减到 0 才把下标推进到下一项。
 */
static bool NF_DispatchNext(SystemMode_t *out)
{
    if (s_stage_idx >= NF_STAGE_COUNT) {
        return false;
    }
    *out = NF_STAGES[s_stage_idx].mode;

    if (s_stage_left > 0u) {
        s_stage_left--;
    }
    if (s_stage_left == 0u) {
        s_stage_idx++;
        s_stage_left = (s_stage_idx < NF_STAGE_COUNT)
                       ? NF_STAGES[s_stage_idx].times : 0u;
    }
    return true;
}

/* ==================================================================
 * 4.3  各阶段执行体
 *
 * @note  与旧 NLF_TASK 一致: 每个阶段**跑完自己那一段**才返回, 返回前用
 *        NLF_Request() 把下一个阶段挂上 (旧代码用的是 task_send() 走事件队列,
 *        这里走线程标志, 少一趟调度器往返 —— 见 worker_task.h 的契约)。
 *        所有阶段最终都回到 Event_Navigation 这个"中继站", 由它去查顺序表。
 * ================================================================== */

/** 中继站: 跑完一段导航, 派发顺序表里的下一项; 表跑完就回家。 */
static void NF_Stage_Navigation(void)
{
    Nav_FeDuanPoint();

    SystemMode_t next;
    if (NF_DispatchNext(&next)) {
        printf("[FLOW] -> %d (stage %u, left %u)\r\n",
               (int)next, (unsigned)s_stage_idx, (unsigned)s_stage_left);
        NLF_Request(next);
    } else {
        printf("[FLOW] all stages done -> GoHome\r\n");
        AG_Stop();
        NLF_Request(Event_GoHome);
    }
}


static void NF_Stage_LinFolL(void)
{
    printf("[FLOW-STUB] LinFolL (左循迹/收集物块) 未接线, 直接跳过\r\n");

    AG_Stop();
    Nav_CalibrateAfterTrace(false);     /* TODO: 打桩 */
    NLF_Request(Event_Navigation);
}

/**
 * @brief [打桩] 右循迹 —— 收集奖杯。
 * @note  同 NF_Stage_LinFolL(), 旧完成条件是 g_trophy_done==1。
 */
static void NF_Stage_LinFolR(void)
{
    printf("[FLOW-STUB] LinFolR (右循迹/收集奖杯) 未接线, 直接跳过\r\n");

    AG_Stop();
    Nav_CalibrateAfterTrace(true);      /* TODO: 打桩 */
    NLF_Request(Event_Navigation);
}

/**
 * @brief 找圆 → 对准 → 放一个物块。
 * @note  与旧代码的两处关键差异, 都是必须的:
 *        1. **Circle_Follow() 是单拍函数** —— 它不阻塞、不自循环、无退出条件
 *           (algorithm/Circle_base.c:36-116), 跑一次只发 10ms 的一拍电机指令。
 *           旧代码把它放在按拍的轮询状态机里所以能跑; 这里改成显式 while 循环。
 *        2. **加了超时** —— 见 NF_CIRCLE_TIMEOUT_MS 的说明。
 */
static void NF_Stage_FindCircle(void)
{
    uint32_t t0;

    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();


    t0 = HAL_GetTick();
    while (g_circle_dir != 'O') {
        Circle_Follow();
        if (HAL_GetTick() - t0 > NF_CIRCLE_TIMEOUT_MS) {
            printf("[FLOW] FindCircle 超时 (%ums), 放弃本拍\r\n",
                   (unsigned)NF_CIRCLE_TIMEOUT_MS);
            AG_Stop();
            NLF_Request(Event_Navigation);
            return;
        }
        osDelay(10);
    }

    /* 转盘依次转到每个颜色所在槽位, 转完放料。
     * TT_RotateByQR() 每次只转一格 (内部 osDelay(500)), 要循环到它返回 false。 */
    while (TT_RotateByQR()) {
        /* 每次调用推进一格 */
    }

    Place('O', g_circle_avg_x, g_circle_avg_y, 0u);

    g_circle_dir = ' ';         /* 清残留, 让下一次找圆重新判定 */
    TT_RotateReset();
    Wheel_Odom_Reset();         /* 旧代码的 World_Reset(): 放置完清零里程计 */

    NLF_Request(Event_Navigation);
}


static void NF_Stage_PlaceDown(void)
{
    Jang_type rank = NF_RANK[s_place_idx % NF_RANK_COUNT];
    uint8_t   slot = NF_TROPHY_SLOT[(uint8_t)rank];
    uint32_t  t0;

    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();
    g_circle_speed = 1.0f;

    /* 1) 转盘转到该奖杯所在工位 —— 每个奖杯只在第一次进入本阶段时转一次。
     *    季军分支在旧代码里还附带一个先下降的预动作。 */
    if (!s_place_latch) {
        if (rank == third_place) {
            BlockBasic_LiftTo(DOWN, 14u);
        }
        BlockBasic_TurntableTo(slot);
        s_place_latch = true;
    }

    /* 2) 找圆对准 (同 NF_Stage_FindCircle, Circle_Follow 需外层循环) */
    t0 = HAL_GetTick();
    while (g_circle_dir != 'O') {
        Circle_Follow();
        if (HAL_GetTick() - t0 > NF_CIRCLE_TIMEOUT_MS) {
            printf("[FLOW] PlaceDown 找圆超时, 放弃本拍\r\n");
            AG_Stop();
            NLF_Request(Event_Navigation);
            return;
        }
        osDelay(10);
    }

    /* 3) 放置 + 收尾 */
    Place('O', g_circle_avg_x, g_circle_avg_y, NF_PLACE_HEIGHT[(uint8_t)rank]);
    printf("[FLOW] PlaceDown rank=%d slot=%u done\r\n", (int)rank, (unsigned)slot);

    if (rank == second_place) {
        BlockBasic_LiftTo(UP, 48u);   /* 亚军: 放完先把丝杆升起 */
    }

    s_place_idx++;
    s_place_latch = false;
    g_circle_dir  = ' ';

    NLF_Request(Event_Navigation);
}

/** 回家: 走既有路径点表, 然后停车。 */
static void NF_Stage_GoHome(void)
{
    Nav_RunWaypoints();         /* g_waypoint_count 当前恒 0 → 立即返回 */
    AG_Stop();
}

/* ==================================================================
 * 4.4  流程出口
 * ================================================================== */

void NLF_RunFlow(SystemMode_t mode)
{
    NF_FlowSeed();      /* 幂等; 首次调用时灌硬编码默认值 */

    switch (mode)
    {
        case Event_Navigation:
            NF_Stage_Navigation();
            break;

        case Event_LinFolL:
            NF_Stage_LinFolL();
            break;

        case Event_LinFolR:
            NF_Stage_LinFolR();
            break;

        case Event_FindCircle:
            NF_Stage_FindCircle();
            break;

        case Event_PlaceDown:
            NF_Stage_PlaceDown();
            break;

        case Event_GoHome:
            NF_Stage_GoHome();
            break;

        case Event_ArcRun:
            /* 跑一段定半径圆弧  */
            Arc_Run();
            break;

        case Event_STOP:
            /* 急停 */
            Arc_Abort();                /* = 关环 + 清线速度/前馈 */
            AG_Stop();
            break;

        /* Event_QRCode / Event_PickUp / Event_STEERING_ROTATE 暂不接线:
         *   Event_QRCode          —— 按用户决定 (2026-09-28) 不再扫二维码,
         *                            NX 回传顺序后本事件应彻底废弃;
         *   Event_PickUp          —— 收集已在循迹段完成, 无独立动作;
         *   Event_STEERING_ROTATE —— 转盘动作已内联进 FindCircle / PlaceDown。 */
        default:
            printf("[FLOW] unhandled mode %d\r\n", (int)mode);
            break;
    }
}
