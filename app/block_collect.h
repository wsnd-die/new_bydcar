/**
 * @file    block_collect.h
 * @brief   物块收集 —— 等 IR → 夹紧 → 转一步 → 读形状/颜色 → 存 g_tt
 *
 *  一轮时序 (**夹和识别错开一拍**):
 *      圆锥 → 转到槽 2                       [不夹不识别]
 *      块2  → 夹紧槽2 → 转到槽3 → 识别槽2
 *      ...
 *      块5  → 夹紧槽5 → 转 340°关门 → 识别槽5
 *
 *  颜色传感器装在"转过一步之后"的位置, 夹完立刻读色只会读到别的槽。
 *  物理槽 1 固定是黄锥, 不夹不读, 不写进 g_tt (Color_TypeDef 里没有「黄」)。
 *
 *  槽位编号: 物理槽 N = g_tt 下标 N-1 = BlockBasic_TurntableTo(N) = 夹爪 ID N+1。
 *
 *  分支由调用方传入 (BlockCollect_SetStage):
 *    COLLECT_MATERIAL → 物料: 槽 2~5
 *    COLLECT_TROPHY   → 奖杯: 槽 1~3, 不读颜色/形状; 名次由 NX_GetTrophyRank()
 *                       在等待期间投票得出 ('a'/'b'/'c' → 1/2/3),
 *                       存进 g_tt.trophy[slot-1]。
 *
 *  @warning NX_GetTrophyRank() 是消费式的 —— 全工程只能有采集侧这一个调用点。
 */
#ifndef BLOCK_COLLECT_H
#define BLOCK_COLLECT_H

#include <stdint.h>
#include <stdbool.h>

/** 本次采集跑哪条分支。。 */
typedef enum {
    COLLECT_BLOCK = 0,
    COLLECT_TROPHY   = 1,
} BlockCollectStage_t;

/**
 * @brief  设定本次采集跑哪条分支。
 * @note   必须在 BlockCollect_Start() 之前调。分支不自动翻转、不随 Reset()
 *         复位, 每轮都要由调用方显式设一次。
 */
void BlockCollect_SetStage(BlockCollectStage_t stage);

/** 物理槽 1 = 黄色圆锥, 固定已知, 不夹取不读色。 */
#define CONE_SLOT    1u
/** 实际采集的槽位范围: 物理槽 2~5 (共 4 个)。 */
#define BLOCK_FIRST_SLOT   2u
#define BLOCK_LAST_SLOT    5u
#define TROPHY_FIRST_SLOT  1u
#define TROPHY_LAST_SLOT   3u

/**
 * @brief  推一拍 —— 放在 gripper_task 的 for(;;) 里每圈调一次。
 * @note   没有请求时立刻返回; 有请求时这一拍同步跑完整轮采集 (最长
 *         5 × COLLECT_IR_TIMEOUT_MS)。所以不能丢进 NLF_TASK。
 */
void BlockCollect_Poll(void);

/** 请求跑一轮采集。任务下一拍开始, 跑完置 IsDone()。已在跑时忽略。 */
void BlockCollect_Start(void);

/** 清状态标志 (不动 g_tt —— 每轮开头自己会 TT_Init)。 */
void BlockCollect_Reset(void);

void servo_set_pos(uint8_t id, uint16_t pos);

bool BlockCollect_IsRunning(void);
bool BlockCollect_IsDone(void);

#endif /* BLOCK_COLLECT_H */
