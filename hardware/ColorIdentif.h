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

#include "Common_used.h"

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
bool    TT_RotateByQR(void);                     /* 每次转一个槽位, 返回 false=已全部转完 */
void    TT_RotateReset(void);                    /* 重置旋转进度 */
bool    TT_IsDone(void);                         /* 检查是否全部转完 */
void TogetPos(uint8_t slot, float *x, float *y, float *yaw);  /* 取点位坐标 */
void SetPos(uint8_t slot, float x, float y, float yaw);     /* 标定点位 */

#endif /* COLOR_IDENTIF_H */