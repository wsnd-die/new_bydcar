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
#include "collect_ir.h"          /* IR_ObjectPresent —— 蹭料用 (不能用 IR_ObjectEntered, 见 NF_CreepForward) */
#include "drv_wheel_odom.h"      /* Wheel_Odom_Reset (旧代码的 World_Reset) */
#include "ws2812.h"

volatile uint8_t g_angle_ctrl_enable = 0;    /* 1 = 打开角度环 */
volatile float   g_angle_target_yaw  = 0.0f;

volatile uint8_t g_route_abort = 0u;
volatile static uint8_t NAV_count=0;

/** 物块进料时是否打断当前路线段。
 *
 *  0  = 不打断
 *  1 只在 `Nav_GoToWorld()` 正在驱动一段时接受打断。 */
#ifndef NF_ABORT_ON_FEED
#define NF_ABORT_ON_FEED  0
#endif

void Route_AbortRequest(void)
{
#if NF_ABORT_ON_FEED
    /* 旧语义: 有段在跑才打断 —— 没有段可打断的进料直接丢掉 */
    if (g_nav_running) {
        g_route_abort = 1u;
    }
#endif
}


osThreadId_t fcTaskHandle  = NULL;
osThreadId_t nlfTaskHandle = NULL;



static volatile SystemMode_t s_nlf_pending = Event_STOP;

#define RAD2DEG(r) ((r) * 57.2957795131f)


static void AG_Stop(void)
{
    MecanumResult zero = Mecanum_Calc(0.0f, 0.0f);
    Send_commandmotor(&zero);
}
/**
 * @brief 航向串级环的任务调用
 *
 */
void Angle_Fuction(void)
{
    uint8_t was_on = 0;

    Angle_Init();

    for (;;)
    {
        if (g_angle_ctrl_enable && !was_on) {
            Angle_Enable(g_angle_target_yaw);
            was_on = 1;
        }
        else if (!g_angle_ctrl_enable && was_on) {
            Angle_Disable();
            AG_Stop();
            was_on = 0;
        }
        osDelay(AG_TASK_PERIOD_MS);
    }
}
void FC_Fuction(void)
{
    NX_SetMode(NX_MODE_YOLO);
    NX_ApplyMode();
    osDelay(FC_TASK_PERIOD_MS);
}

/*
 * NLF_TASK —— 流程
 */

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




/** 奖杯放置顺序 : 亚军 → 冠军 → 季军。 */
static const Jang_type NF_RANK[NF_RANK_COUNT] = {
    second_place, champion, third_place,
};




static const int16_t NF_PLACE_HEIGHT[4] = { 0, -10, -9, -5 };

/** 五个槽位里实际放的物块颜色。 */
static const Color_TypeDef NF_SLOT_COLORS[NF_TASK1_SLOT_COUNT] = {
    COLOR_BLUE,COLOR_BLUE ,COLOR_RED,COLOR_RED,
};
static const  BlockShape_t NF_SLOT_SHAP[NF_TASK1_SLOT_COUNT] = {
    SHAPE_CYLINDER, SHAPE_RECT,SHAPE_CYLINDER,SHAPE_RECT,
};
#define NF_TASK1_QR_IDX     0u

/*  流程顺序表与状态 */


typedef struct {
    SystemMode_t mode;
    uint8_t      times;
} NF_Stage_t;



// static const NF_Stage_t NF_STAGES[] = {
//     { Event_Navigation, 16u },
// };
#define NF_STAGE_COUNT  (sizeof(NF_STAGES) / sizeof(NF_STAGES[0]))

#if 1   /* ---- 完整比赛流程   ---- */
static const NF_Stage_t NF_STAGES[] = {
      { Event_Collect_R, 1u },
     // { Event_Navigation, 4u },
      { Event_PlaceDown, 3u },
     {Event_Collect_L,  1u },
    { Event_FindCircle,  3u },   /* 1 黄锥 + 4 物块 */
};

#endif

static uint8_t s_stage_idx  = 0u;
static uint8_t s_stage_left = 0u;
static uint8_t s_place_idx  = 0u;   /*  奖杯放置进度 */
static bool    s_place_latch = false; /*  本阶段转盘是否已就位 */
static bool    s_flow_seeded = false;

/** 找圆阶段的等待上限。 */
#define NF_CIRCLE_TIMEOUT_MS   60000u

/** 找圆失败时的次数上限。 */
#define NF_CIRCLE_RETRY_MAX    3u

/** 上电是否自动开跑。0 = 等调度器收到事件再跑。 */
#ifndef NF_AUTOSTART
#define NF_AUTOSTART  0
#endif

/** 把 4.1 的硬编码默认值灌进业务层的数据结构。只跑一次。*/
static void NF_FlowSeed(void)
{
    if (s_flow_seeded) {
        return;
    }
    s_flow_seeded = true;

    TT_Init();
    for (uint8_t s = 0; s < NF_TASK1_SLOT_COUNT-1; s++) {
        TT_SetColor(s, NF_SLOT_COLORS[s]);
        TT_SetShape(s, NF_SLOT_SHAP[s]);
    }

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


#ifndef NF_ROUTE_ARC
#define NF_ROUTE_ARC  0
#endif

#if NF_ROUTE_ARC
/**
 * 圆弧模式的路线表 —— 逐段执行, 交替"导航到起点"与"跑一段弧"。
 *
 * 弧替代的点位:
 *   [1] 物料弧 —— g_waypoints[] 下标 6~10 的 e/c/d/a/b
 *   [3] 奖杯弧 —— 下标 0~2 的 奖杯二维码点/亚军点/亚军点
 *
 * ★ 起点坐标暂借现有表里的点, **待现场示教**。
 * ★ 弧参数先用 Arc_SetParam 的默认那组, **待标定** (360°@0.15m/s ≈ 21 秒)。
 */
static const struct {
    uint8_t     arc;          /* 0 = 导航到 start; 1 = 跑一段弧 */
    World_Dir_t start;        /* arc == 0 时用 */
    float       r, v, sweep;  /* arc == 1 时用 */
} NF_ARC_ROUTE[] = {
    /* ★待示教: 物料弧起点 (暂借 6 号"季军点") */
    { 0u, { 1.72384f, 0.25749f, 0.03638f }, 0.0f, 0.0f, 0.0f },
    /* ★待标定: 物料弧 */
    { 1u, { 0.0f, 0.0f, 0.0f }, ARC_DEF_RADIUS, ARC_DEF_SPEED, ARC_DEF_SWEEP },
    /* ★待示教: 奖杯弧起点 (暂借 1 号"奖杯二维码点") */
    { 0u, { 0.38503f, -1.20312f, -0.76168f }, 0.0f, 0.0f, 0.0f },
    /* ★待标定: 奖杯弧 */
    { 1u, { 0.0f, 0.0f, 0.0f }, ARC_DEF_RADIUS, ARC_DEF_SPEED, ARC_DEF_SWEEP },
};
#define NF_ARC_ROUTE_COUNT  (sizeof(NF_ARC_ROUTE) / sizeof(NF_ARC_ROUTE[0]))
#endif /* NF_ROUTE_ARC */

/**
 * @brief 走一段路线。`true` = 这一段走完了 (或被跳过)。
 *
 * @note  这是**路线源**的唯一入口, 编译期在"分点导航"和"圆弧"之间切换。
 *        换成别的走法 (循迹、光流…) 时只改这里, 流程一行不用动。
 * @note  两种模式下被 `g_route_abort` 打断的语义一致: 立刻收尾、返回,
 *        由调用方决定要不要推进 (见 Nav_LastAborted / NF_Stage_Navigation)。
 */
static bool NF_RouteStep(void)
{

    g_route_abort = 0u;

#if NF_ROUTE_ARC
    static uint8_t s_arc_idx = 0u;

    if (s_arc_idx >= NF_ARC_ROUTE_COUNT) {
        return true;                    /* 路线走完, 空转 (与分点导航同语义) */
    }

    const typeof(NF_ARC_ROUTE[0]) *st = &NF_ARC_ROUTE[s_arc_idx];

    bool ok;
    if (st->arc) {
        Arc_SetParam(st->r, st->v, st->sweep);
        ok = Arc_Run();
        if (!ok) {
            printf("[FLOW] 第 %u 段弧参数非法, 跳过\r\n", (unsigned)s_arc_idx);
        }
    } else {
        ok = Nav_GoToWorld(st->start.x, st->start.y, st->start.yaw);
    }

    if (ok) {
        s_arc_idx++;
    }
    return ok;
#else
    return Nav_FeDuanPoint();           /* 默认: 一次走一个 g_waypoints[] 点 */
#endif
}


static BlockCollectStage_t cur_stage = COLLECT_TROPHY;

#define NF_CREEP_FWD_M       0.3f    /* 前进距离上限 (m) */
#define NF_CREEP_VMPS        0.4f    /* 速度 (m/s)。) */
#define NF_CREEP_TIMEOUT_MS  2300u    /* 总时长上限 (ms)*/
#define NF_CREEP_TICK_MS     10u      /* 控制周期 (ms) */

/** 需要"到位后向前"的点位 (1 基点号, 与 NavigationMecanum.c 的
 *  g_waypoints[] 注释编号一致): 1~3 奖杯点。 */
static const uint8_t NF_CREEP_WP[] = {
    1u, 2u, 3u,
};

/** @brief 点号是否在表里。 */
static bool NF_NeedCreep(uint8_t wp)
{
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(NF_CREEP_WP) / sizeof(NF_CREEP_WP[0])); i++) {
        if (NF_CREEP_WP[i] == wp) {
            return true;
        }
    }
    return false;
}


static void NF_CreepForward(void)
{
    uint32_t t0      = osKernelGetTickCount();
    float    travel  = 0.0f;
    bool     ir_last = IR_ObjectPresent();   /* 起点先采一拍当作"上一拍" */

    while ((travel < NF_CREEP_FWD_M) &&
           ((osKernelGetTickCount() - t0) < NF_CREEP_TIMEOUT_MS))
    {
        MecanumResult cmd = Mecanum_Calc_Full_V(NF_CREEP_VMPS, 0.0f, 0.0f);
        Send_commandmotor(&cmd);             /* 车体系*/
        osDelay(NF_CREEP_TICK_MS);
        travel += NF_CREEP_VMPS * ((float)NF_CREEP_TICK_MS / 1000.0f);

        bool now  = IR_ObjectPresent();
        bool fell = (ir_last && !now);       /* 遮光 → 恢复 */
        ir_last   = now;

        if (fell) {
            osDelay(20);                     /* 防毛刺 */
            if (!IR_ObjectPresent()) {
                break;                       /*  物块进去了 */
            }
            ir_last = true;                  /* 还在遮光 */
        }
    }

    AG_Stop();
    g_route_abort = 0u;                      /* 消化过程中来的打断 */
    printf("[FLOW] creep: travel=%.0fmm\r\n", travel * 1000.0f);
}

static void NF_Stage_Navigation(void)
{
    bool arrived = NF_RouteStep();

    if (arrived && !Nav_LastAborted() && NF_NeedCreep(Nav_LastWaypointNo())) {
        osDelay(200);
        NF_CreepForward();
    }

    NAV_count++;
    uint8_t need = (cur_stage == COLLECT_TROPHY) ? 4u : 1u;
    if (NAV_count == need)
    {
        printf("[FLOW] %s collected\r\n",(cur_stage == COLLECT_TROPHY) ? "trophy" : "block");
    }
    if (NAV_count < need)
    {
        NLF_Request(Event_Navigation);
        return;
    }
    SystemMode_t next;
    if (NF_DispatchNext(&next)) {
        printf("[FLOW] -> %d (stage %u, left %u)\r\n",
               (int)next, (unsigned)s_stage_idx, (unsigned)s_stage_left);
        NLF_Request(next);
    } else {
        printf("[FLOW] all stages done -> GoHome\r\n");
        AG_Stop();
        NLF_Request(Event_GoHome);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 80.0f / 180 * 2000 + 500);
    }
}


/** 采集阶段的等待上限。4 个槽 × (IR 超时 10s + 转盘 + 夹取 + 读色), 留一倍余量。 */
/**
 * @brief 物块/奖杯采集 —— **只把请求挂给 gripper_task, 不等结果, 立刻返回**。
 *
 *  采集 (等 IR / 转盘 / 夹爪 / 读色) 全程跑在 `gripper_task` 里, 与 NLF_TASK
 *  的导航**并行**。本阶段唯一的作用就是"把这一轮请求挂上去", 挂完就落回
 *  `Event_Navigation` 继续走点。
 *
 * @note V1.23.0 之前这里会**阻塞轮询 `BlockCollect_IsDone()` 最长 120 秒**
 *       (`NF_COLLECT_TIMEOUT_MS`) —— 那正是"收集完才开导航"的病根, 已整段删除。
 *       现在这套设计是: **导航一直在跑, 物块在途中收**。
 * @note 流程**不再感知采集何时完成**。`BlockCollect_IsRunning()/IsDone()`
 *       API 保留为查询式, 将来真需要等的阶段自己拿它轮询 (见 block_collect.h)。
 * @note 采集**推进导航**的通道不在这里, 而在 `gripper_task` 侧的 IR 进料 →
 *       `Route_AbortRequest()` → `Nav_GoToWorld()` 打断。见 worker_task.h。
 */
static void NF_Stage_Collect(BlockCollectStage_t stage)
{
    // printf("[FLOW] Collect: 挂请求 (%s), 不等\r\n",
    //        (stage == COLLECT_TROPHY) ? "奖杯" : "物料");
    NAV_count=0;
    cur_stage = stage;
    BlockCollect_SetStage(stage);
    if (stage==COLLECT_TROPHY)
    {
        NX_RequestMode(NX_MODE_YOLO);
       NLF_Request(Event_Navigation);
    }
    else
    {
        NX_RequestMode(NX_MODE_CIRCLE);
        NLF_Request(Event_NavCircleL);
    }
    NX_ApplyMode();

    BlockCollect_Reset();
    BlockCollect_Start();            /* 只置请求; */

    if (stage == COLLECT_BLOCK) {
        TT_RotateReset();
    }

    // NLF_Request(Event_Navigation);   /* 立刻进导航, 采集并行 */
}

static void NF_Stage_FindCircle(void)
{
    uint32_t t0;

    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();
    BlockBasic_LiftToAbs(0);
    BlockBasic_TurntableTo(BLOCK_FIRST_SLOT);
    t0 = HAL_GetTick();
    while (g_circle_dir != 'O') {
        Circle_Follow();
        // printf("[FLOW] FindCircle %f,%f\r\n",g_circle_avg_x,g_circle_avg_y);
        if (HAL_GetTick() - t0 > NF_CIRCLE_TIMEOUT_MS) {
            printf("[FLOW] FindCircle 超时 (%ums), 放弃本拍\r\n",
                   (unsigned)NF_CIRCLE_TIMEOUT_MS);
            AG_Stop();
            NLF_Request(Event_Navigation);
            return;
        }
    }

    if (TT_RotateByQR()) {
        printf("[FLOW] FindCircle %f,%f 转盘物理槽 %u\r\n",g_circle_avg_x,g_circle_avg_y, (unsigned)TT_CurrentSlot());
        Place('O', g_circle_avg_x, g_circle_avg_y, 48u, TT_CurrentSlot());
    } else {
        printf("[FLOW] FindCircle: 该物块在 g_tt 里找不到或已放完, 跳过\r\n");
    }

    g_circle_dir = ' ';         /* 清残留, 让下一次找圆重新判定 */
    //TT_RotateReset();
    //Wheel_Odom_Reset();

    NLF_Request(Event_Navigation);
    // NLF_Request(Event_FindCircle);
}


static void NF_Stage_PlaceDown(void)
{
    Jang_type rank = NF_RANK[s_place_idx % NF_RANK_COUNT];
    uint32_t  t0;

    const uint8_t slot_idx = SlotByTrophy((uint8_t)rank);   /* g_tt 下标 0~2 / SLOT_NONE */
    const uint8_t tslot    = (slot_idx == SLOT_NONE) ? 0u: (uint8_t)(slot_idx + 1u);  /* 转盘槽 1~3 */

    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();


    if (!s_place_latch) {
        if (slot_idx == SLOT_NONE) {
            printf("[FLOW] PlaceDown rank=%d 在 g_tt.trophy[] 里找不到, 跳过\r\n",
                   (int)rank);
            s_place_idx++;
            s_place_latch = false;
            NLF_Request(Event_Navigation);
            return;
        }

        if (rank == third_place) {
            BlockBasic_LiftToAbs(8.0f);    /* 季军预下降: 降到 5mm  */
            // osDelay(170);
        }
        BlockBasic_TurntableTo(tslot);
        BlockBasic_TurntableTo(tslot);
        s_place_latch = true;
    }

    t0 = HAL_GetTick();
    while (g_circle_dir != 'O') {
        Circle_Follow();
        if (HAL_GetTick() - t0 > NF_CIRCLE_TIMEOUT_MS) {
            printf("[FLOW] PlaceDown 找圆超时, 放弃本拍\r\n");
            AG_Stop();
            NLF_Request(Event_Navigation);
            return;
        }
        // osDelay(10);
    }
    /*  放置 + 收尾 */
    Place('O', g_circle_avg_x,g_circle_avg_y , NF_PLACE_HEIGHT[(uint8_t)rank], tslot);   /* 松开该奖杯所在的槽 */
    printf("[FLOW] PlaceDown rank=%d slot=%u done\r\n", (int)rank, (unsigned)tslot);

    if (rank == second_place) {
        BlockBasic_LiftToAbs(50.5f);   /* 亚军: 放完升到 43mm ) */
        // osDelay(250);
    }

    s_place_idx++;
    s_place_latch = false;
    g_circle_dir  = ' ';

    NLF_Request(Event_Navigation);
}
static void NF_Start(void)
{
    SystemMode_t mode = Event_Navigation;

    if (!NF_DispatchNext(&mode)) {
        printf("[FLOW] 表已跑完, 忽略启动键\r\n");
        return;
    }
    NLF_Request(mode);
}


void NF_StartSecnd() {
    SystemMode_t mode = Event_Navigation;      /* 初值只是兜底, 见下 */
    OPS9_G491_UART3_SetPose(2.084f, -257.49f, 1723.84f);
    s_idx=6;
    s_stage_idx=2;
    NLF_Request(mode);

}

static void NF_Stage_GoHome(void)
{
    g_route_abort = 0u;

    if (g_waypoint_count > 0u) {
        g_nav_leg_speed = NAV_HOME_SPEED;
        (void)Nav_GoToWorld(0.05f, 0.05f ,0.0f);
    }

    AG_Stop();
}

/* 流程出口*/

void NLF_RunFlow(SystemMode_t mode)
{
    NF_FlowSeed();

    switch (mode)
    {
        case Event_Navigation:
            NF_Stage_Navigation();
            break;

        case Event_Collect_L:
            /* 物块采集: 圆锥 + 槽 2~5 夹取 + 读形状/颜色, 结果写进 g_tt */
            NF_Stage_Collect(COLLECT_BLOCK);
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

        case Event_NavCircleL:
            /* 跑一段定半径圆弧  */
            Nav_Cricle(Nav_CricleL ,  0.87f, 170*NAV_DEG2RAD );
            break;
        case Event_NavCircleR:
            Nav_Cricle(Nav_CricleR , 0.93f ,170*NAV_DEG2RAD );
            break;
        case Event_START:
            NF_Start();
            break;
        case Event_STARTSecnd:
            NF_StartSecnd();
            break;
        case Event_STOP:
            /* 急停 */
            Arc_Abort();                /*  关环  清线速度/前馈 */
            AG_Stop();
            break;

        default:
            printf("[FLOW] unhandled mode %d\r\n", (int)mode);
            break;
    }
}
