/**
 * @file    block_collect.c
 * @brief   物块收集逻辑。槽位编号约定与设计说明见 block_collect.h。
 *
 *  **不自己起任务**: 由 gripper_task 的 for(;;) 每圈调一次 BlockCollect_Poll()。
 *  舵机总线也归 gripper_task 管 (SCS_BusInit 在它入口), 所以这里不必再等总线就绪。
 */
#include "Common_used.h"
#include "block_collect.h"
#include "collect_ir.h"       /* IR_ObjectEntered */
#include "msp_color.h"        /* MSP_Color_Wait */
#include "block_basic.h"      /* 转盘 / 夹爪 / 形状 */
#include "ColorIdentif.h"     /* g_tt */
#include "NX_uart.h"          /* NX_GetTrophyRank */
#include "worker_task.h"      /* Route_AbortRequest —— IR 进料推进导航 (V1.23.0) */

/* ================================================================
 * 时序常量 —— ★ 全部是占位值, 上机实测后调
 * ================================================================ */

/** 转盘/夹爪动作走完之后等机构停稳, 再去读。读早了两样都会读到中途值。 */
#define COLLECT_SETTLE_MS        100u
/** 等一个物块进料的上限。超时则该槽不写数据, 直接跳下一槽。
 *  @note 奖杯分支的名次投票就发生在这一段等待里, 所以它同时是投票窗口的长度。 */
#define COLLECT_IR_TIMEOUT_MS   5000u

/* ================================================================
 * 状态
 * ================================================================ */

static volatile bool s_req     = false;   /* 有采集请求待处理 */
static volatile bool s_running = false;
static volatile bool s_done    = false;

/** 本次采集跑哪条分支。由 BlockCollect_SetStage() 设定, 不自动翻转。 */
static volatile BlockCollectStage_t s_stage = COLLECT_BLOCK;
/* ================================================================
 * 内部
 * ================================================================ */

/**
 * @brief NX 回传的奖杯名次字符 → 内部编码。
 * @param ch  NX 回的字符: **'a' = 冠军,  'b' = 亚军,  'c' = 季军**
 *            (大小写都收)。其余字符一律当脏数据。
 * @return 1=冠军 2=亚军 3=季军 (与 Jang_type 同编码); 不是这几个字符时返回 0。
 *
 * @note   NX 实际发的是字母不是数字 —— 见 V1.21.3 记录。要改对应关系只动这里。
 */
static uint8_t trophy_char_to_rank(char ch)
{
    switch (ch) {
    case 'a': case 'A': return 1u;   /* 冠军 */
    case 'b': case 'B': return 2u;   /* 亚军 */
    case 'c': case 'C': return 3u;   /* 季军 */
    default:            return 0u;
    }
}

/**
 * @brief 等一个物块完全进来, 带超时; 顺带在等待期间给 NX 报的奖杯名次投票。
 *
 * @param timeout_ms  等物块进来的上限。
 * @param rank_out    非空时: 对这段等待里 NX 报的名次**投票**, 物块进来时把票多的
 *                    那个写入 (1=冠军 2=亚军 3=季军, 与 Jang_type 同编码);
 *                    **一票都没有时写 0**。传 NULL 表示不关心名次 (物料分支用)。
 *                    NX 发的是字母 'a'/'b'/'c', 经 trophy_char_to_rank() 换算。
 * @retval true=物块进来了; false=超时。
 *
 * @note  **IR_ObjectEntered() 全工程只能有这一个调用者** —— 它内部有个 static
 *        的上一拍状态 (collect_ir.c 的 ir_last), 两个调用者会互相破坏。
 *
 * @note  为什么投票: NX 的名次是视觉给的, 同一个奖杯在视野里会被反复识别, 结果
 *        可能抖动或中途跳变。把「物块完全进来之前这段时间」收到的名次全数一遍
 *        再取多数, 比只认最后一帧稳。**投票窗口就是本函数等待的那一整段** ——
 *        转盘已经转到位、物块还没完全进来的时候。
 * @note  开始投票前会**先丢掉一帧残留** —— 上一轮的名次帧若还挂在 trophy_fresh
 *        上, 不清掉会被算进这一轮的票。
 * @note  **NX_GetTrophyRank() 是消费式的**, 全工程只有这里调 (FC_TASK 里原来
 *        那份调试打印已经摘掉, 否则两边抢帧)。
 */
static bool wait_block_entered(uint32_t timeout_ms, uint8_t *rank_out)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t votes[4] = { 0u, 0u, 0u, 0u };   /* 下标 1~3 对应名次 1~3 */
    char     r;

    if (rank_out) {
        *rank_out = 0u;
        if (NX_GetTrophyRank(&r)) {
            printf("[COLLECT] (开窗丢残留帧 '%c')\r\n", r);
        }
    }

    for (;;) {
        if (rank_out && NX_GetTrophyRank(&r)) {
            uint8_t rk = trophy_char_to_rank(r);   /* 'a'/'b'/'c' → 1/2/3 */
            if (rk != 0u) {
                votes[rk]++;
            }
        }

        if (IR_ObjectEntered()) {

            Route_AbortRequest();

            if (rank_out) {
                uint8_t best = 0u;
                for (uint8_t i = 1u; i <= 3u; i++) {
                    if (votes[i] > votes[best]) {
                        best = i;             /* 严格大于 → 同票时取名次靠前的 */
                    }
                }
                *rank_out = best;             /* 0 = 一票都没有, 名次未定 */
                printf("[COLLECT] trophy votes 1:%u 2:%u 3:%u -> rank=%u\r\n",
                       (unsigned)votes[1], (unsigned)votes[2],
                       (unsigned)votes[3], (unsigned)best);
            }
            return true;
        }

        if ((HAL_GetTick() - t0) > timeout_ms) {
            return false;
        }
        osDelay(10);
    }
}

/**
 * @brief MSP 没回颜色时的兜底: 前两个槽红、后两个槽蓝。
 */
static Color_TypeDef collect_default_color(uint8_t slot)
{
    return (slot <= 3u) ? COLOR_RED : COLOR_BLUE;
}

/**
 * @brief 形状读不出来时的兜底 (与 collect_default_color 同款)。
 *
 * 与上面那张颜色表**配对**用, 保证 4 个 `(颜色, 形状)` 恰好铺满 `T1[]`:
 *
 *     槽2 红+方    槽3 红+圆     (collect_default_color: 槽<=3 → 红)
 *     槽4 蓝+方    槽5 蓝+圆     (collect_default_color: 槽>3  → 蓝)
 *
 * @note 哪个槽配方/圆**不影响能不能找到** —— `TT_SeekBlock()` 是按
 *       `(颜色, 形状)` 的值去搜槽的, 只要四个组合齐全就行。它只决定
 *       "方/圆分别摆到哪个点位"(即 `T1[]` 里的先后)。
 * @note ★ 占位值, NX 接入后换成回传值。
 */
static BlockShape_t collect_default_shape(uint8_t slot)
{
    return (slot == 2u || slot == 4u) ? SHAPE_RECT : SHAPE_CYLINDER;
}

/**
 * @brief 读一颗已经夹住的物块 (形状 + 颜色), 存进 g_tt。
 * @param slot  物理槽号 2~5。调用时该槽必须**已经夹紧**、且转盘已经把它
 *              转到颜色传感器前面并停稳。
 */
static void identify_slot(uint8_t slot)
{
    const uint8_t s = (uint8_t)(slot - 1u);   /* 物理槽 → g_tt 下标 */

    int          raw   = BlockBasic_GripperRaw(slot);
    BlockShape_t shape = BlockBasic_ShapeFromRaw(raw, slot);


    Color_TypeDef c;
    if (!MSP_Color_Vote(MSP_COLOR_VOTE_MS, &c)) {
        c = collect_default_color(slot);
        printf("[COLLECT] slot %u: color 一票没有 -> default\r\n", (unsigned)slot);
    }

    if (shape == SHAPE_UNKNOWN) {
        shape = collect_default_shape(slot);
        printf("[COLLECT] slot %u: shape unknown -> default\r\n", (unsigned)slot);
    }

    TT_SetColor(s, c);
    TT_SetShape(s, (uint8_t)shape);
    TT_SetRawAngle(s, (int16_t)raw);
    TT_SetCollected(s, true);

    printf("[COLLECT] slot=%u raw=%d shape=%u color=%u\r\n",
           (unsigned)slot, raw, (unsigned)shape, (unsigned)c);
}


/** 默认颜色分配 —— **2 红 + 2 蓝**, 必须与 ColorIdentif.c 的 `T1[]` 配比一致,
 *  否则 `TT_BlocksCoverTable()` 还是判"凑不齐"、改写也白改。 */
static const Color_TypeDef NF_DEF_COLOR[4] = {
    COLOR_RED, COLOR_RED, COLOR_BLUE, COLOR_BLUE,
};
static const BlockShape_t NF_DEF_SHAPE[4] = {
    SHAPE_RECT, SHAPE_CYLINDER, SHAPE_RECT, SHAPE_CYLINDER,
};

/**
 * @brief 收集结果的颜色配比凑不齐 `T1[]` 时, 把**已采集**的槽整批换成默认表。
 *
 * **为什么需要**: 摆放阶段 `TT_SeekBlock()` 是按**颜色**去 `T1[]` 里逐个搜槽的,
 * 而 `T1[]` 要的是 2 红 + 2 蓝。识别一旦不可信 (颜色一票都没有 → 走
 * `collect_default_color()`; 或四个槽认出同一个颜色), 某种颜色的槽就不够 →
 * **那几件永远放不出去**。
 *
 * 所以这里不猜"哪个槽错了", 而是按颜色对账; 不齐就按默认表整批改写。
 * 与奖杯分支的 `trophy_fill_missing()` 同一思路。
 *
 * @note 只改**已采集**的槽 —— 漏料的槽不给默认值, 颜色保持 `TT_Init()` 后的
 *       `COLOR_UNKNOWN`, `TT_SeekBlock()` 自然搜不到它, 转盘不会跑去那个空槽
 *       空放一次。少来几个物块就只摆几件, 剩下的照常摆完。
 * @note **形状这一维已经不参与摆放** (V1.28.0), 这里仍写 `NF_DEF_SHAPE` 只是
 *       让 `g_tt` 里不留半套数据, 没人读它。
 */
static void block_fill_defaults(void)
{
    if (TT_BlocksCoverTable()) {
        return;             /* 识别结果可用, 一个槽都不动 */
    }

    printf("[COLLECT] 颜色 凑不齐 -> 已采集的槽整批走默认表\r\n");

    uint8_t k = 0u;
    for (uint8_t slot = BLOCK_FIRST_SLOT; slot <= BLOCK_LAST_SLOT; slot++) {
        const uint8_t s = (uint8_t)(slot - 1u);

        if (!TT_IsCollected(s)) {
            continue;       /* 漏料的槽不硬塞组合 */
        }

        TT_SetColor(s, NF_DEF_COLOR[k]);
        TT_SetShape(s, (uint8_t)NF_DEF_SHAPE[k]);
        printf("[COLLECT] slot %u -> default color=%u shape=%u\r\n",
               (unsigned)slot, (unsigned)NF_DEF_COLOR[k],
               (unsigned)NF_DEF_SHAPE[k]);
        k++;                /* 最多 4 个已采集槽, 不会越界 */
    }
}


static void trophy_fill_missing(void)
{
    uint8_t miss = 3u;   /* 名次未定的槽下标; 3 = 没有 */
    uint8_t used = 0u;   /* bit1~3 对应已出现的名次 1~3 */

    for (uint8_t s = 0u; s < 3u; s++) {
        uint8_t r = g_tt.trophy[s];
        if (r == 0u) {
            if (miss != 3u) return;              /* 缺 2 个以上 → 推不出来 */
            miss = s;
        } else if (used & (uint8_t)(1u << r)) {
            return;                              /* 名次重复 → 视觉给错, 不补 */
        } else {
            used |= (uint8_t)(1u << r);
        }
    }
    if (miss == 3u) return;                      /* 三个都识别出来了 */

    for (uint8_t r = 1u; r <= 3u; r++) {
        if (!(used & (uint8_t)(1u << r))) {
            TT_SetTrophy(miss, r);
            printf("[COLLECT] 排除法: 槽%u 名次未定 -> 补为 %u (%s)\r\n",
                   (unsigned)(miss + 1u), (unsigned)r,
                   (r == 1u) ? "冠军" : (r == 2u) ? "亚军" : "季军");
            return;
        }
    }
}

/**
 * @brief 跑完一轮: 圆锥先过, 然后槽 2~5 依次「夹紧 → 转一步 → 识别」。
 *
 *      圆锥进 → IR → 转到槽 2
 *      块2进  → IR → 夹紧槽2 → 转到槽3 → 识别槽2
 *      块3进  → IR → 夹紧槽3 → 转到槽4 → 识别槽3
 *      块4进  → IR → 夹紧槽4 → 转到槽5 → 识别槽4
 *      块5进  → IR → 夹紧槽5 → 转 340°关门 → 识别槽5
 */
static void collect_slots(void)
{

    TT_Init();
    if (s_stage == COLLECT_BLOCK)
    {
        (void)BlockBasic_TurntableTo(CONE_SLOT);
        printf("[COLLECT] waiting cone\r\n");
        if (!wait_block_entered(COLLECT_IR_TIMEOUT_MS, NULL)) {   /* NULL = 不投票 */
            printf("[COLLECT] no cone, abort\r\n");
        }

        (void)BlockBasic_TurntableTo(BLOCK_FIRST_SLOT);

        for (uint8_t slot = BLOCK_FIRST_SLOT; slot <= BLOCK_LAST_SLOT; slot++)
        {
            if (wait_block_entered(COLLECT_IR_TIMEOUT_MS, NULL)) {   /* NULL = 不投票 */
                /* 顺序不能反 (V1.28.0): identify_slot() 里第一件事就是
                 * BlockBasic_GripperRaw(slot) —— 读夹爪**当前**位置, 而形状判定的
                 * 前提就是"夹紧状态下回读"(见 block_basic.c 的 Slot_Shape[] 注释)。
                 * 夹紧放后面 → raw 读到的是松开时的空程位置, g_tt.raw_angle 和
                 * 那行 `slot=..raw=..` 日志全废, 阈值再也没法标定。
                 * 而且 MSP_Color_Vote() 要在里面等 MSP_COLOR_VOTE_MS, 那段时间
                 * 物块必须已经被夹住。 */
                (void)BlockBasic_GripperClamp(slot);
                identify_slot(slot);
            } else {
                printf("[COLLECT] slot %u: no block, skip\r\n", (unsigned)slot);
            }


            if (slot < BLOCK_LAST_SLOT) {
                (void)BlockBasic_TurntableTo((uint8_t)(slot + 1u));
            } else {
                Servo_Angle(BLOCK_CLOSE_DOOR);
            }
        }

        block_fill_defaults();
    }
    else
    {
        // BlockBasic_LiftTo(UP,10);
        printf("[COLLECT] waiting trophy\r\n");
        (void)BlockBasic_TurntableTo(TROPHY_FIRST_SLOT);

        for (uint8_t slot = TROPHY_FIRST_SLOT; slot <= TROPHY_LAST_SLOT; slot++)
        {
            uint8_t rank = 0u;
            if (!wait_block_entered(COLLECT_IR_TIMEOUT_MS, &rank)) {
                printf("[COLLECT] trophy slot %u: no block, skip\r\n", (unsigned)slot);
                continue;
            }

            if (rank != 0u) {
                TT_SetTrophy((uint8_t)(slot - 1u), rank);
                printf("[COLLECT] trophy slot=%u rank=%u  (B3总帧=%u)\r\n",
                       (unsigned)slot, (unsigned)rank,
                       (unsigned)NX_GetTrophyCount());
            } else {
                printf("[COLLECT] trophy slot=%u: 一票都没有, 该槽名次未定  (B3总帧=%u)\r\n",
                       (unsigned)slot, (unsigned)NX_GetTrophyCount());
            }

            osDelay(100);
            if (slot < TROPHY_LAST_SLOT) {
                (void)BlockBasic_TurntableTo((uint8_t)(slot + 1u));
            } else {
                Servo_Angle(TROPHY_CLOSE_DOOR);
            }
            (void)BlockBasic_GripperClamp(slot);
        }

        /* 三个槽都过完了, 现在才谈得上"缺哪一个" */
        trophy_fill_missing();

        BlockBasic_LiftToAbs(30.0f);   /* 奖杯收完抬到 28mm (等价原 UP,25: 3+25) */
    }
}

/* ================================================================
 * 外部接口
 * ================================================================ */

void BlockCollect_SetStage(BlockCollectStage_t stage)
{
    s_stage = stage;
}

void BlockCollect_Start(void)
{
    if (s_running) {
        return;
    }
    s_done = false;
    s_req  = true;
}

void BlockCollect_Reset(void)
{
    s_req     = false;
    // s_running = false;
    s_done    = false;
}

bool BlockCollect_IsRunning(void) { return s_running; }
bool BlockCollect_IsDone(void)    { return s_done; }

void BlockCollect_Poll(void)
{
    if (!s_req) {
        return;         /* 常态: 没有请求, 立刻返回 */
    }

    s_req     = false;
    s_running = true;
    s_done    = false;

    collect_slots();

    s_running = false;
    s_done    = true;
    printf("[COLLECT] done\r\n");
}
