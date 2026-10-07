/**
 * @file    ColorIdentif.h
 * @brief   转盘识别存储 — QR 码颜色映射 → 槽位 → 点位
 *
 * 转盘 5 个槽位(A~E)，随机对应 5 种颜色(Task1) 或 3 个奖杯(Task2)。
 *
 * 流程:
 *   1. 颜色传感器收集颜色 → Color_DetectDominant()
 *   2. K230 扫 QR 码 → pattern 序号 0~15
 *   3. SetQR(idx) 解析槽位映射
 *   4. SlotByColor(颜色) → 查到槽位
 *   5. TgtPos(槽位, &x, &y, &yaw) → 传给导航前往
 */

#ifndef COLOR_IDENTIF_H
#define COLOR_IDENTIF_H

#include <stdint.h>
#include "color.h"               /* Color_TypeDef */

/* ============================================================
 * 槽位
 * ============================================================ */
enum {
    SLOT_1 = 0, SLOT_2, SLOT_3, SLOT_4, SLOT_5,
    SLOT_NONE = 0xFF
};

/* ============================================================
 * 槽位 → 导航点位
 * ============================================================ */
typedef struct {
    float x, y, yaw;
} TgtPos_t;

/* ============================================================
 * 转盘存储 (全局单例)
 * ============================================================ */
typedef struct {
    uint8_t idx;              /* QR pattern 序号 */
    uint8_t ok;               /* 1=已解析 */

    /* 槽位上的内容 */
    uint8_t cnt;              /* 有效槽位数: Task1=5, Task2=3 */
    uint8_t task_color[5];    /* QR 任务顺序: slot[A..E] 要求放的颜色 (SetQR 写入, 与物理映射分离) */
    uint8_t color[5];         /* 物理: slot[A..E] 实际放的物块颜色 (TT_SetColor 写入) */
    uint8_t trophy[3];        /* slot[A..C] 奖杯: 1=金奖 2=银奖 3=铜奖 */

    /* 颜色 → 槽位 反向索引 (Task1) */
    uint8_t rev[COLOR_COUNT]; /* rev[COLOR_RED] = SLOT_X */

    /* 槽位 → 世界坐标 (标定后填入) */
    TgtPos_t pos[5];          /* pos[A..E] */

    /* ---- V1.20.0 追加（只能往后加，保证前向兼容）----
     * 物块采集结果。槽位下标与 color[] 同一套 (0 = 物理槽 1)。 */
    uint8_t  shape[5];        /* 槽位 → BlockShape_t 的取值(见 block_basic.h); 0 = 未采集 */
    int16_t  raw_angle[5];    /* 夹紧时回读的 SCS0009 原始位置; 有效与否看 collected */
    uint8_t  collected;       /* bit0..4 = 物理槽 1..5 已完成采集 */

} TT_t;  /* Turntable */

extern TT_t    g_tt;
extern uint8_t T2[6][3];

/* ============================================================
 * API
 * ============================================================ */
void TT_Init(void);                              /* 初始化 */
void SetQR(uint8_t idx);                         /* 设置 QR 序号, 解析映射 */
void TT_SetColor(uint8_t slot, Color_TypeDef c);  /* 存检测到的颜色到槽位 */
uint8_t SlotByColor(Color_TypeDef c);            /* 颜色 → 槽位 */
uint8_t ColorAtSlot(uint8_t slot);               /* 槽位 → 颜色 */

/* ---- V1.20.0 追加: 物块采集结果 (形状 / 夹紧回读角度 / 是否已采集) ----
 * @note 三个 Set 都会拒绝 slot >= 5。raw_angle 是否有效**一律以 collected 为准**,
 *       不要靠 raw_angle 的初值判断 —— TT_Init() 是 memset(0), 初值是 0 不是 -1。 */
void    TT_SetShape(uint8_t slot, uint8_t shape);   /* shape 取 BlockShape_t */
void    TT_SetRawAngle(uint8_t slot, int16_t raw);
void    TT_SetCollected(uint8_t slot, bool done);
uint8_t ShapeAtSlot(uint8_t slot);                  /* 未采集返回 SHAPE_UNKNOWN(0) */
int16_t RawAngleAtSlot(uint8_t slot);               /* 未采集返回 -1 */
bool    TT_IsCollected(uint8_t slot);

/* ---- 奖杯名次 (V1.21.0) ----
 * 取值与 Jang_type 同编码: **1=冠军 2=亚军 3=季军** (见 ColorIdentif.h 的 trophy[] 注释)。
 * @note 槽位下标是 0~2, 对应转盘物理槽 1~3 (只有 3 个奖杯位)。 */
void    TT_SetTrophy(uint8_t slot, uint8_t rank);
uint8_t TrophyAtSlot(uint8_t slot);                 /* 未采集返回 0 */

/**
 * @brief  名次 → 它在哪个槽 (反查)。
 * @param  rank  1=冠军 2=亚军 3=季军。
 * @return g_tt 下标 0~2 (对应转盘物理槽 1~3)；没找到返回 SLOT_NONE。
 *
 * @note   与 SlotByColor() 同风格 (0 基下标 + SLOT_NONE 表示没找到)。
 * @note   **摆放阶段必须用这个, 不能用 NF_TROPHY_SLOT[]** —— 那张表是旧 QR 方案
 *         的固定映射 (冠军→槽3), 而奖杯现在落在哪个槽由**收集顺序**决定,
 *         名次是收完才填进 g_tt.trophy[] 的, 两者对不上。见 V1.21.1 记录。
 */
uint8_t SlotByTrophy(uint8_t rank);
bool    TT_RotateByQR(void);                     /* 每次转一个槽位, 返回 false=已全部转完 */
/** @brief `TT_RotateByQR()` 最近一次**转到门口**的物理槽号 (1~5); 还没转过时返回 0。
 *  @note  给 `Place()` 松夹爪用 —— 物料摆放是**逐个**放的, 一次全松开会把
 *         剩下几个一起掉下去, 所以必须知道"当前是哪个槽在门口"。 */
uint8_t TT_CurrentSlot(void);
void    TT_RotateReset(void);                    /* 重置旋转进度 */

/**
 * @brief  T1[] 要的四个 `(颜色,形状)` 组合, 是不是每个都能在**已采集**的槽里找到。
 * @retval true   四个组合齐全 —— 摆放阶段四行都能搜到槽, 四个物块都放得出去。
 * @retval false  有组合无人匹配 —— 对应那一轮的 FindCircle 会"找不到"、跳过。
 *
 * @note   **这是摆放的前提条件, 不是诊断**: `TT_SeekBlock()` 是按组合逐个搜槽的,
 *         两行若落在同一个槽上, 另一行就永远空手。识别不可信时 (形状阈值表未标定
 *         → 恒判 `SHAPE_RECT`, 见 `block_basic.c` 的 `Slot_Shape[]`) 必然凑不齐。
 *         采集侧拿到 `false` 时应把已采集的槽整批改写成默认表
 *         —— 见 `app/block_collect.c` 的 `block_fill_defaults()`。
 */
bool    TT_BlocksCoverTable(void);
bool    TT_IsDone(void);                         /* 检查是否全部转完 */
void TogetPos(uint8_t slot, float *x, float *y, float *yaw);  /* 取点位坐标 */
void SetPos(uint8_t slot, float x, float y, float yaw);     /* 标定点位 */

#endif /* COLOR_IDENTIF_H */