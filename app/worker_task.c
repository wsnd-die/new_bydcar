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


#include "ColorIdentif.h"        /* TT_Init / TT_SetColor / SetQR / TT_RotateByQR ... */
#include "block_collect.h"       /* BlockCollect_Start / IsDone (V1.20.0) */
#include "QRcode.h"              /* Jang_type: champion / second_place / third_place */
#include "block_basic.h"         /* Place / BlockBasic_TurntableTo / BlockBasic_LiftTo */
#include "drv_wheel_odom.h"      /* Wheel_Odom_Reset (旧代码的 World_Reset) */

volatile uint8_t g_angle_ctrl_enable = 0;    /* 1 = 打开角度环 */
volatile float   g_angle_target_yaw  = 0.0f; /* 目标航向 (deg), 与 g_hwt_imu_yaw 同量纲 */


osThreadId_t fcTaskHandle  = NULL;
osThreadId_t nlfTaskHandle = NULL;


static AngleCtrl s_fc;

static volatile SystemMode_t s_nlf_pending = Event_STOP;

#define RAD2DEG(r) ((r) * 57.2957795131f)


static void AG_Stop(void)
{
    MecanumResult zero = Mecanum_Calc(0.0f, 0.0f);
    Send_commandmotor(&zero);
}
void Angle_Fuction(void)
{
    uint8_t  was_on    = 0;

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
            MecanumResult cmd = Mecanum_Calc(0,
                                             s_fc.cmd_w );

            Send_commandmotor(&cmd);
        }
        else if (was_on)
        {
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



#define NF_RANK_COUNT        3u    /* 奖杯个数 */
#define NF_TASK1_SLOT_COUNT  5u    /* 物块槽位数 */




/** 奖杯放置顺序 (按放置先后): 亚军 → 冠军 → 季军。
 *  对应旧 NLF_TASK 的 rank[3] = {second_place, champion, third_place}。
 *  ★ NX 接入点: 换成 NX 回传的 1/2/3 顺序。 */
static const Jang_type NF_RANK[NF_RANK_COUNT] = {
    second_place, champion, third_place,
};

/* V1.21.1 **删除** `NF_TROPHY_SLOT[]` (名次 → 固定转盘工位: 冠军→3/亚军→2/季军→1)。
 *
 * 它是旧 QR 方案的产物 —— 那时 QR 码直接告诉你"冠军在哪个工位", 所以可以做一张
 * 名次→工位的固定表。现在奖杯落在哪个槽由**收集顺序**决定 (第 N 个进槽 N),
 * 名次是收完才填进 g_tt.trophy[] 的, 固定表与实际情况对不上, 照它转盘会拿错奖杯。
 * 摆放阶段已改成反查 g_tt.trophy[] (见 ColorIdentif.c 的 SlotByTrophy)。 */

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


typedef struct {
    SystemMode_t mode;
    uint8_t      times;
} NF_Stage_t;



static const NF_Stage_t NF_STAGES[] = {
    { Event_Navigation, 30u },
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
    Nav_CalibrateAfterTrace(false);
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

/** 采集阶段的等待上限。4 个槽 × (IR 超时 10s + 转盘 + 夹取 + 读色), 留一倍余量。 */
#define NF_COLLECT_TIMEOUT_MS   120000u

/**
 * @brief 物块采集 —— 交给独立的 blockcol 任务跑, 本阶段只等它出结果。
 * @note  采集序列本身阻塞得很重 (等 IR、等总线往返), 直接写在 NLF_TASK 里会一直
 *        占着这个高优先级任务 (osPriorityHigh)。丢给 blockcol (Normal) 之后本阶段
 *        只需要轮询完成标志, 中途照样让得出去。
 */
static void NF_Stage_Collect(BlockCollectStage_t stage)
{
    printf("[FLOW] Collect: 请求采集 (%s)\r\n",
           (stage == COLLECT_TROPHY) ? "奖杯" : "物料");

    BlockCollect_SetStage(stage);
    BlockCollect_Reset();
    BlockCollect_Start();

    uint32_t t0 = HAL_GetTick();
    while (!BlockCollect_IsDone()) {
        if ((HAL_GetTick() - t0) > NF_COLLECT_TIMEOUT_MS) {
            printf("[FLOW] Collect 超时, 强行推进\r\n");
            break;
        }
        osDelay(20);
    }

    printf("[FLOW] Collect: 完成\r\n");
    NLF_Request(Event_Collect_R);
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
    uint32_t  t0;

    /* 名次 → 槽位要**反查** g_tt.trophy[], 不能用那张旧的 NF_TROPHY_SLOT[]。
     * 后者是旧 QR 方案的固定映射 (冠军→槽3), 而奖杯现在落在哪个槽由
     * **收集顺序**决定 (第 N 个进槽 N), 名次是收完才填进 g_tt.trophy[] 的 ——
     * 两者对不上, 照抄那张表会拿错奖杯。详见 V1.21.1 记录。 */
    const uint8_t slot_idx = SlotByTrophy((uint8_t)rank);   /* g_tt 下标 0~2 / SLOT_NONE */
    const uint8_t tslot    = (slot_idx == SLOT_NONE) ? 0u
                                                     : (uint8_t)(slot_idx + 1u);  /* 转盘槽 1~3 */

    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();
    g_circle_speed = 1.0f;

    /* 1) 转盘转到该奖杯所在工位 —— 每个奖杯只在第一次进入本阶段时转一次。
     *    季军分支在旧代码里还附带一个先下降的预动作。 */
    if (!s_place_latch) {
        if (slot_idx == SLOT_NONE) {
            /* 采集阶段没把名次填进来 (或填了别的值)。不猜, 记一条日志跳过本拍,
             * 否则会转到某个不相干的槽去放。 */
            printf("[FLOW] PlaceDown rank=%d 在 g_tt.trophy[] 里找不到, 跳过\r\n",
                   (int)rank);
            s_place_idx++;
            s_place_latch = false;
            NLF_Request(Event_Navigation);
            return;
        }

        if (rank == third_place) {
            BlockBasic_LiftTo(DOWN, 14u);
        }
        BlockBasic_TurntableTo(tslot);
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
    printf("[FLOW] PlaceDown rank=%d slot=%u done\r\n", (int)rank, (unsigned)tslot);

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

        case Event_Collect_L:
            /* 物块采集: 圆锥 + 槽 2~5 夹取 + 读形状/颜色, 结果写进 g_tt */
            NF_Stage_Collect(COLLECT_MATERIAL);
            break;

        case Event_Collect_R:
            /* 奖杯采集: 槽 1~3 等进来 + 投票定名次 + 夹紧, 结果写进 g_tt.trophy[] */
            NF_Stage_Collect(COLLECT_TROPHY);
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
