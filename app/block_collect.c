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

/* ================================================================
 * 时序常量 —— ★ 全部是占位值, 上机实测后调
 * ================================================================ */

/** 转盘/夹爪动作走完之后等机构停稳, 再去读。读早了两样都会读到中途值。 */
#define COLLECT_SETTLE_MS        200u
/** 等一个物块进料的上限。超时则该槽不写数据, 直接跳下一槽。
 *  @note 奖杯分支的名次投票就发生在这一段等待里, 所以它同时是投票窗口的长度。 */
#define COLLECT_IR_TIMEOUT_MS   10000u

/* ================================================================
 * 状态
 * ================================================================ */

static volatile bool s_req     = false;   /* 有采集请求待处理 */
static volatile bool s_running = false;
static volatile bool s_done    = false;

/** 本次采集跑哪条分支。由 BlockCollect_SetStage() 设定, 不自动翻转。 */
static volatile BlockCollectStage_t s_stage = COLLECT_MATERIAL;
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
        (void)NX_GetTrophyRank(&r);           /* 丢掉上一轮的残留帧 */
    }

    for (;;) {
        if (rank_out && NX_GetTrophyRank(&r)) {
            uint8_t rk = trophy_char_to_rank(r);   /* 'a'/'b'/'c' → 1/2/3 */
            if (rk != 0u) {
                votes[rk]++;
            }
        }

        if (IR_ObjectEntered()) {
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
    if (!MSP_Color_Wait(&c, MSP_COLOR_TIMEOUT_MS)) {
        c = collect_default_color(slot);
        printf("[COLLECT] slot %u: color timeout -> default\r\n", (unsigned)slot);
    }

    TT_SetColor(s, c);
    TT_SetShape(s, (uint8_t)shape);
    TT_SetRawAngle(s, (int16_t)raw);
    TT_SetCollected(s, true);

    printf("[COLLECT] slot=%u raw=%d shape=%u color=%u\r\n",
           (unsigned)slot, raw, (unsigned)shape, (unsigned)c);
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
static void collect_all_slots(void)
{

    TT_Init();
    if (s_stage == COLLECT_MATERIAL)
    {
        printf("[COLLECT] waiting cone\r\n");
        if (!wait_block_entered(COLLECT_IR_TIMEOUT_MS, NULL)) {   /* NULL = 不投票 */
            printf("[COLLECT] no cone, abort\r\n");
            return;     /* 圆锥都没来,  直接放弃这一轮 */
        }
        (void)BlockBasic_TurntableTo(BLOCK_FIRST_SLOT);

        for (uint8_t slot = BLOCK_FIRST_SLOT; slot <= BLOCK_LAST_SLOT; slot++)
        {
            if (!wait_block_entered(COLLECT_IR_TIMEOUT_MS, NULL)) {   /* NULL = 不投票 */
                printf("[COLLECT] slot %u: no block, skip\r\n", (unsigned)slot);
                continue;
            }

            (void)BlockBasic_GripperClamp(slot);
            osDelay(COLLECT_SETTLE_MS);

            if (slot < BLOCK_LAST_SLOT) {
                (void)BlockBasic_TurntableTo((uint8_t)(slot + 1u));
            } else {
                Servo_Angle(BLOCK_CLOSE_DOOR);
            }
            osDelay(COLLECT_SETTLE_MS);     /* 等转盘停稳再读 */

            identify_slot(slot);
        }
    }
    else
    {
        // BlockBasic_LiftTo(UP,10);
        printf("[COLLECT] waiting trophy\r\n");
        (void)BlockBasic_TurntableTo(TROPHY_FIRST_SLOT);

        for (uint8_t slot = TROPHY_FIRST_SLOT; slot <= TROPHY_LAST_SLOT; slot++)
        {
            /* 等物块进来的同时把 NX 报的名次投出来 —— 名次在**夹住之前**就定好。
             * 投票窗口 = 这段等待, 也就是"物块完全进来前的一段时间"。 */
            uint8_t rank = 0u;
            if (!wait_block_entered(COLLECT_IR_TIMEOUT_MS, &rank)) {
                printf("[COLLECT] trophy slot %u: no block, skip\r\n", (unsigned)slot);
                continue;
            }

            if (rank != 0u) {
                TT_SetTrophy((uint8_t)(slot - 1u), rank);
                printf("[COLLECT] trophy slot=%u rank=%u\r\n",
                       (unsigned)slot, (unsigned)rank);
            } else {
                printf("[COLLECT] trophy slot=%u: 一票都没有, 该槽名次未定\r\n",
                       (unsigned)slot);
            }

            (void)BlockBasic_GripperClamp(slot);

            if (slot < TROPHY_LAST_SLOT) {
                (void)BlockBasic_TurntableTo((uint8_t)(slot + 1u));
            } else {
                Servo_Angle(TROPHY_CLOSE_DOOR);
            }
        }
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
    s_running = false;
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

    collect_all_slots();

    s_running = false;
    s_done    = true;
    printf("[COLLECT] done\r\n");
}
