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

volatile uint8_t g_angle_ctrl_enable = 0;    /* 1 = 打开角度环 */
volatile float   g_angle_target_yaw  = 0.0f; /* 目标航向 (deg), 与 g_hwt_imu_yaw 同量纲 */

/* V1.23.0: 路线段打断。语义与用法见 worker_task.h 的说明。 */
volatile uint8_t g_route_abort = 0u;
volatile static uint8_t NAV_count=0;

/** 物块进料时是否打断当前路线段。
 *
 *  0 (默认) = **不打断** —— "分点导航 + 到位蹭料"模式下唯一正确的选择:
 *      creep 保证了"车**到位之后**物块才进料", 那时根本没有段可以打断;
 *      而这个进料事件是从 gripper_task 上报的, 它的 `IR_ObjectEntered()` 内部有
 *      50/100ms 去抖(osDelay), 比导航侧的检测**晚 50~150ms** 才认账 ——
 *      迟到的打断正好落进**下一段刚起步**的窗口里, 把下一段打成"被打断"
 *      (而游标照常 s_idx++) → **平白跳过一个点位**。
 *      现场表现: 收完第 1 个奖杯就跳掉第 2 个点位。
 *
 *      关掉之后流程变成一步一点、完全确定:
 *          驱动到点N → 到位 → creep 把物块顶进进料口 → 采集侧收到 IR → 去点 N+1
 *
 *  1 = 旧语义(**边开边收**): 只在 `Nav_GoToWorld()` 正在驱动一段时接受打断。
 *      剪掉 creep、改回"一边开一边把物块扫进进料口"时才需要它。 */
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
    /* NF_ABORT_ON_FEED == 0: 直接丢弃, 理由见上面的宏说明。 */
}


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
    NX_SetMode(NX_MODE_CIRCLE);
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
static const uint16_t NF_PLACE_HEIGHT[4] = { 0u, 5u, 5u, 5u };

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



// static const NF_Stage_t NF_STAGES[] = {
//     { Event_Navigation, 16u },
// };
#define NF_STAGE_COUNT  (sizeof(NF_STAGES) / sizeof(NF_STAGES[0]))

#if 1   /* ---- 原表: 完整比赛流程 (测完改回 #if 1) ---- */
static const NF_Stage_t NF_STAGES[] = {
    { Event_Collect_R, 1u },
{ Event_Navigation, 1u },
    { Event_PlaceDown, 3u },
    { Event_Collect_L,    1u },
    { Event_FindCircle,  5u },
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
#define NF_CIRCLE_TIMEOUT_MS   60000u

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
    /* 打断(g_route_abort)是给**正在跑的那一段**用的 —— Nav_GoToWorld 在段内看到它
     * 就把这一段提前结束。但**段与段之间**到达的进料事件没有段可打断, 标志会一直
     * 挂着, 直到下一段刚起步时被消费 → 下一段瞬间返回(Nav_LastAborted), 而游标
     * 照常 s_idx++ → **平白吃掉一个点位**。
     *
     * 现场表现: 收完第 1 个奖杯后"莫名跳一个点位"(V1.24.4 修)。
     * 成因是 V1.24.3 的"到位后向前蹭料": 物块是在**导航段结束之后**才被顶进去的,
     * 那个进料事件正好落进这个窗口。NF_CreepForward() 结尾清一次只能挡住"蹭的
     * 过程中"来的; 蹭完才来的挡不住, 两个任务看到 IR 的时刻差 10~15ms, 谁先谁后
     * 是随机的 —— 所以是偶发。
     *
     * @note 这**不影响**正常打断: 段**中途**来的进料仍然由 Nav_GoToWorld 内部消费。 */
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

/**
 * 中继站: 走一段路线, 派发顺序表里的下一项; 表跑完就回家。
 *
 * V1.23.0: 走路线的那一句由 `Nav_FeDuanPoint()` 换成了 `NF_RouteStep()`
 * (路线源可编译期切换)。**除此之外一字未改** —— 收集与导航的解耦不在这
 * 条路径上, 而在 `NF_Stage_Collect()` 不再阻塞 (见该函数)。
 */
static BlockCollectStage_t cur_stage = COLLECT_TROPHY;
/* ==================================================================
 * 到位后"向前蹭料" (V1.24.3)
 *
 * 采集点位上, 位置环判"到位"时车头离物块还差一点点, 进料口 IR 不触发 →
 * 采集侧 wait_block_entered() 干等 5s 超时, 物块收不到。
 *
 * 所以在流程派发下一个阶段**之前**, 原地向前低速蹭一小段, 一直蹭到物块
 * "完全进入"进料口为止。距离和时长两道限幅, 蹭不进去也不会一直顶着。
 * ================================================================== */
#define NF_CREEP_FWD_M       0.13f    /* 前进距离上限 (m) —— 第一道限幅 */
#define NF_CREEP_VMPS        0.3f    /* 蹭的速度 (m/s)。顶不动就往上提 (0.10) */
#define NF_CREEP_TIMEOUT_MS  2400u    /* 总时长上限 (ms) —— 第二道限幅 */
#define NF_CREEP_TICK_MS     10u      /* 蹭的控制周期 (ms) */

/** 需要"到位后向前蹭"的点位 (1 基点号, 与 NavigationMecanum.c 的
 *  g_waypoints[] 注释编号一致): 1~3 奖杯点 + 7~11 物料点 e/c/d/a/b, 共 8 个。
 *  ★ 只改这张表就换点位。 */
static const uint8_t NF_CREEP_WP[] = {
    1u, 2u, 3u, 7u, 8u, 9u, 10u, 11u,
};

/** @brief 点号是否在"要蹭"的表里。 */
static bool NF_NeedCreep(uint8_t wp)
{
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(NF_CREEP_WP) / sizeof(NF_CREEP_WP[0])); i++) {
        if (NF_CREEP_WP[i] == wp) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 向前低速蹭, 直到物块**完全进入**进料口 / 到达距离上限 / 超时。
 *
 * @note "完全进入"的判据与 collect_ir.c 的 `IR_ObjectEntered()` **一致**
 *       (上一拍遮光 → 这一拍恢复), 但**必须用本函数自己的边沿状态**:
 *       collect_ir 里那个 static 归 gripper_task 的 wait_block_entered() 专用,
 *       两边共用一个状态会让彼此的判据都错乱 —— 所以这里只读纯电平的
 *       `IR_ObjectPresent()`, 自己判边沿。
 * @note 结束时**消费掉 `g_route_abort`** —— 蹭的过程中物块进来, 采集侧会
 *       `Route_AbortRequest()`; 而这一段导航早就结束了, 不消化掉的话
 *       **下一段导航刚进去就被"打断", 直接跳过下一个点位**。
 * @note 车体系前行(`Mecanum_Calc_Full_V(v,0,0)`), 不依赖 OPS9 —— 就是它把车
 *       停在了物块前面, 不能再靠它。
 */
static void NF_CreepForward(void)
{
    uint32_t t0      = osKernelGetTickCount();
    float    travel  = 0.0f;
    bool     ir_last = IR_ObjectPresent();   /* 起点先采一拍当作"上一拍" */

    while ((travel < NF_CREEP_FWD_M) &&
           ((osKernelGetTickCount() - t0) < NF_CREEP_TIMEOUT_MS))
    {
        MecanumResult cmd = Mecanum_Calc_Full_V(NF_CREEP_VMPS, 0.0f, 0.0f);
        Send_commandmotor(&cmd);             /* 车体系: 车头朝哪就往前哪 */
        osDelay(NF_CREEP_TICK_MS);
        travel += NF_CREEP_VMPS * ((float)NF_CREEP_TICK_MS / 1000.0f);

        bool now  = IR_ObjectPresent();
        bool fell = (ir_last && !now);       /* 遮光 → 恢复 = 疑似"完全进入" */
        ir_last   = now;

        if (fell) {
            osDelay(20);                     /* 复确认一拍, 防毛刺 */
            if (!IR_ObjectPresent()) {
                break;                       /* 确实恢复了 → 物块进去了 */
            }
            ir_last = true;                  /* 是毛刺: 当作还在遮光 */
        }
    }

    AG_Stop();
    g_route_abort = 0u;                      /* 见函数头: 消化蹭的过程中来的打断 */
    printf("[FLOW] creep: travel=%.0fmm\r\n", travel * 1000.0f);
}

static void NF_Stage_Navigation(void)
{
    bool arrived = NF_RouteStep();


    if (arrived && !Nav_LastAborted() && NF_NeedCreep(Nav_LastWaypointNo())) {
        NF_CreepForward();
    }

    NAV_count++;
    uint8_t need = (cur_stage == COLLECT_TROPHY) ? 3u : 5u;
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
    printf("[FLOW] Collect: 挂请求 (%s), 不等\r\n",
           (stage == COLLECT_TROPHY) ? "奖杯" : "物料");
    NAV_count=0;
    cur_stage = stage;
    if (stage==COLLECT_TROPHY)
    {
        NX_RequestMode(NX_MODE_YOLO);
    }
    else
    {
        NX_RequestMode(NX_MODE_CIRCLE);
    }
    NX_ApplyMode();
    BlockCollect_SetStage(stage);
    BlockCollect_Reset();
    BlockCollect_Start();            /* 只置请求; gripper_task 下一拍开始跑 */

    NLF_Request(Event_Navigation);   /* 立刻进导航, 采集并行 */
}


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

    while (TT_RotateByQR()) {
        /* 每次调用推进一格 */
    }

    Place('O', g_circle_avg_x, g_circle_avg_y, 0u, TT_CurrentSlot());   /* 物料: 松开刚转到门口那个槽 */

    g_circle_dir = ' ';         /* 清残留, 让下一次找圆重新判定 */
    TT_RotateReset();
    Wheel_Odom_Reset();

    NLF_Request(Event_Navigation);
}


static void NF_Stage_PlaceDown(void)
{
    Jang_type rank = NF_RANK[s_place_idx % NF_RANK_COUNT];
    uint32_t  t0;

    const uint8_t slot_idx = SlotByTrophy((uint8_t)rank);   /* g_tt 下标 0~2 / SLOT_NONE */
    const uint8_t tslot    = (slot_idx == SLOT_NONE) ? 0u: (uint8_t)(slot_idx + 1u);  /* 转盘槽 1~3 */

    NX_RequestMode(NX_MODE_CIRCLE);
    NX_ApplyMode();
    g_circle_speed = 1.0f;

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
            BlockBasic_LiftToAbs(5.0f);    /* 季军预下降: 降到 5mm (等价原 DOWN,33: 38-33) */
            osDelay(1200);
        }
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
        osDelay(10);
    }

    /* 3) 放置 + 收尾 */
    Place('O', g_circle_avg_x, g_circle_avg_y, NF_PLACE_HEIGHT[(uint8_t)rank], tslot);   /* 奖杯: 松开该奖杯所在的槽 */
    printf("[FLOW] PlaceDown rank=%d slot=%u done\r\n", (int)rank, (unsigned)tslot);

    if (rank == second_place) {
        BlockBasic_LiftToAbs(43.0f);   /* 亚军: 放完升到 43mm (等价原 UP,20: 23+20) */
        osDelay(1000);
    }

    s_place_idx++;
    s_place_latch = false;
    g_circle_dir  = ' ';

    NLF_Request(Event_Navigation);
}
static void NF_Start(void)
{
    SystemMode_t mode = Event_Navigation;      /* 初值只是兜底, 见下 */

    if (!NF_DispatchNext(&mode)) {
        /* 表已跑完: NF_DispatchNext() 返回 false 且**不写 out**,
         * 原来这里会把未初始化的栈值当阶段号发出去 → 随机跳一个阶段。
         * 见 clauderecord 2026-10-04 的备注。 */
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

/** 回家: 只走到 g_waypoints[] 的最后一行 (表里标的"17 回家点"), 然后停车。
 *  @note  V1.24.2 之前这里调 `Nav_RunWaypoints()`, 那是**从 0 号点开始把整张
 *         表再走一遍**(17 个点) —— 加上流程自己的十几段导航, 表现就是"连着
 *         跑两遍"。回家就该只走回家点, 不是再巡一圈。 */
static void NF_Stage_GoHome(void)
{
    /* 先清掉可能残留的打断请求 —— 它是**电平不是队列**, 采集跑完时若还挂着,
     * 会让这一段第一拍就被打断、直接跳过。见 worker_task.h 的 g_route_abort。 */
    g_route_abort = 0u;

    if (g_waypoint_count > 0u) {
        const World_Dir_t *home = &g_waypoints[g_waypoint_count - 1u];
        (void)Nav_GoToWorld(home->x, home->y, home->yaw);
    }
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

        case Event_ArcRun:
            /* 跑一段定半径圆弧  */
            Arc_Run();
            break;

        case Event_START:
            NF_Start();
            break;
        case Event_STARTSecnd:
            NF_StartSecnd();
            break;
        case Event_STOP:
            /* 急停 */
            Arc_Abort();                /* = 关环 + 清线速度/前馈 */
            AG_Stop();
            break;


        default:
            printf("[FLOW] unhandled mode %d\r\n", (int)mode);
            break;
    }
}
