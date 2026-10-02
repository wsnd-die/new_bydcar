/**
 * @file    block_collect.h
 * @brief   物块收集 —— 等 IR → 夹紧 → 转一步 → 读形状/颜色 → 存 g_tt
 *
 *  一轮的完整时序 (**夹和识别错开一拍**, 这是关键):
 *
 *      圆锥进 → IR → 转到槽 2                      [不夹不识别]
 *      块2进  → IR → 夹紧槽2 → 转到槽3 → 识别槽2
 *      块3进  → IR → 夹紧槽3 → 转到槽4 → 识别槽3
 *      块4进  → IR → 夹紧槽4 → 转到槽5 → 识别槽4
 *      块5进  → IR → 夹紧槽5 → 转 340°关门 → 识别槽5
 *
 *  **为什么错开**: 颜色传感器装在「转盘转过一步之后」那颗物块停的位置上。
 *  夹完立刻读色只会读到别的槽。第 5 个没有下一槽可转, 改成转
 *  `BLOCK_TURNTABLE_DOOR_DEG` (槽5 + 38°) 顺手把门带上。
 *  识别 = 读夹爪舵机 raw(判形状) + 读 MSP 颜色(超时回退默认值) → 存 g_tt。
 *
 *  物理槽 1 固定是**黄色圆锥**, 不夹不读, 不写进 g_tt —— Color_TypeDef 里
 *  没有「黄」, 且它是硬件驱动层向上输出的接口, 不能为它扩枚举。
 *
 *  ------------------------------------------------------------------
 *  槽位编号约定 (全程统一, 不要再引入第二套):
 *
 *      物理槽 1 = g_tt 下标 0 = BlockBasic_TurntableTo(1) = 夹爪 ID 2  ← 黄锥
 *      物理槽 2 = g_tt 下标 1 = BlockBasic_TurntableTo(2) = 夹爪 ID 3
 *      物理槽 3 = g_tt 下标 2 = BlockBasic_TurntableTo(3) = 夹爪 ID 4
 *      物理槽 4 = g_tt 下标 3 = BlockBasic_TurntableTo(4) = 夹爪 ID 5
 *      物理槽 5 = g_tt 下标 4 = BlockBasic_TurntableTo(5) = 夹爪 ID 6
 *
 *  `slot - 1` 换 g_tt 下标这一步已在 ColorIdentif.c 的 TT_RotateByQR 里有先例。
 *
 *  ------------------------------------------------------------------
 *  跑哪条分支由**调用方传进来** (见 BlockCollect_SetStage):
 *
 *    COLLECT_MATERIAL → 物料: 圆锥先过, 槽 2~5「夹紧 → 转一步 → 识别」(见上)
 *    COLLECT_TROPHY   → 奖杯: 槽 1~3「等进来 → 投票定名次 → 夹紧 → 转一步」
 *
 *  奖杯**不读颜色也不读形状** —— 它的名次是 NX 视觉给的, 走 `NX_GetTrophyRank()`
 *  拿。**NX 发的是字母 'a'(冠军) / 'b'(亚军) / 'c'(季军)**, 由 block_collect.c 的
 *  `trophy_char_to_rank()` 换算; 别的字符当脏数据丢, 一票都没有则该槽名次未定。
 *  名次在**等物块进来的那段等待里投票**决定 (取票多者, 夹住之前就定好),
 *  存进 `g_tt.trophy[slot-1]` (1=冠军 2=亚军 3=季军, 与 Jang_type 同编码)。
 *
 *  @warning `NX_GetTrophyRank()` 是消费式的 —— 全工程只能有采集侧这一个调用点
 *           (见 block_collect.c 的 wait_block_entered)。FC_TASK 里原来那份调试
 *           打印已摘掉。
 */
#ifndef BLOCK_COLLECT_H
#define BLOCK_COLLECT_H

#include <stdint.h>
#include <stdbool.h>

/** 本次采集跑哪条分支。。 */
typedef enum {
    COLLECT_MATERIAL = 0,
    COLLECT_TROPHY   = 1,
} BlockCollectStage_t;

/**
 * @brief  设定本次采集跑哪条分支。
 * @note   **必须在 BlockCollect_Start() 之前调**, 否则这一拍已经按旧的分支跑了。
 * @note   分支**不会被自动翻转**, 也不由 BlockCollect_Reset() 复位 ——
 *         每次要跑采集都由调用方显式设一次, 免得"上一轮跑过什么"这种隐式状态
 *         串到下一轮。
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
 * @brief  推一拍。**放在 gripper_task 的 for(;;) 里每圈调一次**(V1.20.1)。
 * @note   没有采集请求时**立刻返回**, 什么都不做 —— 所以调用方可以直接
 *         `for (;;) { BlockCollect_Poll(); osDelay(20); }`。
 *         有请求时这一拍会**同步跑完整轮采集**: 5 次等 IR (每次上限
 *         `COLLECT_IR_TIMEOUT_MS`)。物块正常到位时每槽几百毫秒, 一轮几秒;
 *         一个都没来才是 5 × 10s 的最坏值。这是有意的 —— 采集要等 IR、等
 *         舵机总线, 丢进 NLF_TASK 会把那个高优先级流程任务占死。
 */
void BlockCollect_Poll(void);

/** 请求跑一轮采集。任务下一拍开始, 跑完置 IsDone()。已在跑时忽略。 */
void BlockCollect_Start(void);

/** 清掉三个状态标志 (不清 g_tt 的采集字段 —— 每轮开头自己会 TT_Init)。 */
void BlockCollect_Reset(void);

bool BlockCollect_IsRunning(void);
bool BlockCollect_IsDone(void);

#endif /* BLOCK_COLLECT_H */
