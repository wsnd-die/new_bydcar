/**
 * @file    block_collect.c
 * @brief   物块 / 奖杯采集。
 */
#include "Common_used.h"
#include "block_collect.h"
#include "collect_ir.h"       /* IR_ObjectEntered */
#include "msp_color.h"        /* MSP_Color_Vote */
#include "block_basic.h"      /* 转盘 / 夹爪 / 形状 */
#include "ColorIdentif.h"     /* g_tt */
#include "NX_uart.h"          /* NX_GetTrophyRank */
#include "worker_task.h"      /* Route_AbortRequest */
#include "ops9_g491_uart3.h"  /* locator_ops9 */
#include "pose_data.h"        /* PoseData_t */
#include "servo_scs.h"



#define COLLECT_SETTLE_MS           100u     /* 等机构停稳再读 */
#define COLLECT_IR_HARD_TIMEOUT_MS  5000u    /* X 模式硬超时, 防 OPS9 离线时死等 */
#define COLLECT_IR_TIMEOUT_MS       5000u    /* 奖杯分支: 同时是名次投票窗口, 必须固定 */
/* 传给 wait_block_entered 表示"不按 X 退出"; X 可能为 0, 不能用 0 当哨兵 */
#define COLLECT_X_OFF               (-1.0e30f)

/** 车体 X 触发点 (世界系, m), 下标 = 转盘槽号, [0] 未用。
 *  只有"等圆锥"在用; 槽 2~5 和奖杯走 IR + 超时 —— 弧段 pose.x 非单调,
 *  X 判据会被来回穿过而假触发。
 *  此路线 X 递减, "跨进来" = 从 > 值变为 <= 值。 */
static const float COLLECT_TRIGGER_X[] = {
    0.0f,
    1.751493f,
    1.604586f,
    1.228177f,
    0.795644f,
    0.350772f,
};
//{   1.801493f, 0.768879f, 1.459011f }, /*  7 e 点 */
// {   1.674586f, 1.154120f, 2.231404f }, /*  8 d 点 */
// {   1.308177f, 1.418482f, 2.770012f }, /*  9 c 点 */
// {   0.865644f, 1.449459f,-2.979479f }, /* 10 b 点 */
// {   0.450772f, 1.269985f,-2.411954f }, /* 11 a 点 */

static volatile bool s_req     = false;   /* 有待处理的采集请求 */
static volatile bool s_running = false;
extern volatile bool s_done    = false;

static volatile BlockCollectStage_t s_stage = COLLECT_BLOCK;   /* 由 SetStage() 指定 */
/* ================================================================
 * 内部
 * ================================================================ */

/** NX 名次字符转内部编码: 'a'/'b'/'c' → 1/2/3 (大小写都收), 其他返回 0。 */
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
 * @brief 等物块完全进来, 同时给 NX 报的奖杯名次投票。
 *
 * 退出条件: IR 触发, 或车体 X 跨过 x_trigger (边沿 + 一次性武装)。
 * x_trigger == COLLECT_X_OFF 时只认 IR + timeout_ms。
 *
 * @param rank_out  非空时写入票数最多的名次 (0 = 未定); NULL = 不投票。
 *
 * @note IR_ObjectEntered() 全工程只能有一个调用者 (内部有 static 状态)。
 * @note 奖杯分支必须固定等待时长, 否则投票窗口变化会影响票数。
 * @note NX_GetTrophyRank() 是消费式, 全工程只在这里调用。
 */
static bool wait_block_entered(float x_trigger, uint32_t timeout_ms, uint8_t *rank_out)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t votes[4] = { 0u, 0u, 0u, 0u };   /* 下标 1~3 对应名次 1~3 */
    char     r;
    PoseData_t p;
    uint8_t  armed = 0u;      /* 0=未判定  1=已武装(等跨过)  2=本次作废(不用 X) */

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

        if (x_trigger > COLLECT_X_OFF) {
            locator_ops9.get_pose(&p);
            if (p.valid) {
                if (armed == 0u) {
                    /* 只判定一次: 弧段 X 非单调, 允许重新武装会假触发;
                     * 一开始就在内侧则永久作废, 该槽只靠 IR。 */
                    armed = (p.x > x_trigger) ? 1u : 2u;
                } else if (armed == 1u && p.x <= x_trigger) {
                    printf("[COLLECT] x=%.2f 跨过 (<=%.2f), 正常夹取\r\n",
                           p.x, x_trigger);
                    return true;
                }
            }

            /* 硬超时: OPS9 离线时 pose.x 冻结, 永远等不到 */
            if ((HAL_GetTick() - t0) > COLLECT_IR_HARD_TIMEOUT_MS) {
                printf("[COLLECT] 等 X 超时 %ums, 本槽不再等\r\n",
                       (unsigned)COLLECT_IR_HARD_TIMEOUT_MS);
                return false;
            }
        } else {
            if ((HAL_GetTick() - t0) > timeout_ms) {
                return false;
            }
        }

        osDelay(10);
    }
}

/** MSP 没回颜色时的兜底: 槽 2~3 红, 槽 4~5 蓝。 */
static Color_TypeDef collect_default_color(uint8_t slot)
{
    return (slot <= 3u) ? COLOR_RED : COLOR_BLUE;
}

/** 形状兜底, 与颜色兜底配对 (槽2 红方 / 槽3 红圆 / 槽4 蓝方 / 槽5 蓝圆)。★ 占位值。 */
static BlockShape_t collect_default_shape(uint8_t slot)
{
    return (slot == 2u || slot == 4u) ? SHAPE_RECT : SHAPE_CYLINDER;
}

/** 读已夹紧物块的形状和颜色, 写入 g_tt。调用前该槽必须夹紧并停稳。 */
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


/* 默认颜色/形状: 2 红 + 2 蓝, 必须与 ColorIdentif.c 的 T1[] 配比一致。 */
static const Color_TypeDef NF_DEF_COLOR[4] = {
    COLOR_RED, COLOR_RED, COLOR_BLUE, COLOR_BLUE,
};
static const BlockShape_t NF_DEF_SHAPE[4] = {
    SHAPE_RECT, SHAPE_CYLINDER, SHAPE_RECT, SHAPE_CYLINDER,
};

/**
 * @brief 颜色配比凑不齐 T1[] 时, 把已采集的槽整批换成默认表。
 *
 * 只改已采集的槽, 漏料的槽保持 COLOR_UNKNOWN, 不会跑去空槽空放。
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
 * @brief 跑一轮采集: 先等圆锥, 再按槽号「等物块 → 夹紧 → 转一步 → 识别」。
 *
 * 物料分支只有等圆锥带 X 判据; 奖杯分支全走 IR, 等待时长即投票窗口。
 */
static void collect_slots(void)
{

    TT_Init();
    if (s_stage == COLLECT_BLOCK)
    {
        (void)BlockBasic_TurntableTo(CONE_SLOT);
        if (!wait_block_entered(COLLECT_TRIGGER_X[CONE_SLOT], 0u, NULL)) {
            printf("[COLLECT] no cone, abort\r\n");
        }
        for (uint8_t id = SERVO_ID_SCS0009_MIN; id <= SERVO_ID_SCS0009_MAX; id++) {
            for (uint8_t i=0;i<=5;i++)
            {
                servo_set_pos(id, SCS_OPEN);
                osDelay(1);
            }
        }
        (void)BlockBasic_TurntableTo(BLOCK_FIRST_SLOT);

        for (uint8_t slot = BLOCK_FIRST_SLOT; slot <= BLOCK_LAST_SLOT; slot++)
        {
            printf("[COLLECT] waiting slot %u (IR, 超时 %ums)\r\n",
                   (unsigned)slot, (unsigned)COLLECT_IR_TIMEOUT_MS);
            if (wait_block_entered(COLLECT_X_OFF, COLLECT_IR_TIMEOUT_MS, NULL)) {
                (void)BlockBasic_GripperClamp(slot);
                identify_slot(slot);
            } else {
                printf("[COLLECT] slot %u: 没等到物块, skip\r\n", (unsigned)slot);
            }


            if (slot < BLOCK_LAST_SLOT) {
                (void)BlockBasic_TurntableTo((uint8_t)(slot + 1u));
            } else {
                Servo_Angle(BLOCK_CLOSE_DOOR);
                for (uint8_t id = SERVO_ID_SCS0009_MIN; id <= SERVO_ID_SCS0009_MAX; id++) {
                    for (uint8_t k=0;k<=5;k++)
                    {
                        servo_set_pos(id, SCS_CLOSE);
                        osDelay(1);
                    }

                }
            }
        }

        block_fill_defaults();
    }
    else
    {
        printf("[COLLECT] waiting trophy\r\n");
        (void)BlockBasic_TurntableTo(TROPHY_FIRST_SLOT);

        for (uint8_t slot = TROPHY_FIRST_SLOT; slot <= TROPHY_LAST_SLOT; slot++)
        {
            uint8_t rank = 0u;
            if (!wait_block_entered(COLLECT_X_OFF, COLLECT_IR_TIMEOUT_MS, &rank)) {
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

            osDelay(190);
            if (slot < TROPHY_LAST_SLOT) {
                (void)BlockBasic_TurntableTo((uint8_t)(slot + 1u));
            } else {
                Servo_Angle(TROPHY_CLOSE_DOOR);
            }
            (void)BlockBasic_GripperClamp(slot);
        }

        /* 三个槽过完, 用排除法补缺失名次 */
        trophy_fill_missing();
        NX_RequestMode(NX_MODE_CIRCLE);
        NX_ApplyMode();
        BlockBasic_LiftToAbs(31.5f);   /* 奖杯收完抬升 */

        // BlockBasic_LiftToAbs(31.0f);   /* 备选高度 */
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

void servo_set_pos(uint8_t id, uint16_t pos)
{
    if (id == SERVO_ID_STS3032) {
        (void)SCS_WritePosEx(id, (int16_t)pos, STS_SPEED, STS_ACC);
    } else {
        (void)SCS_WritePos(id, pos, SCS_TIME, SCS_SPEED);
    }
}


