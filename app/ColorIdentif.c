/**
 * @file    ColorIdentif.c
 * @brief   转盘识别存储 — QR 码映射 → 槽位 → 点位
 *
 * 流程:  收集颜色 → 扫 QR → SetQR(idx) → SlotByColor(颜色) → TgtPos(槽位) → 导航
 */

#include "Common_used.h"
#include "ColorIdentif.h"
#include "block_basic.h"

/* ============================================================
 * 全局
 * ============================================================ */
TT_t g_tt;

/* 默认颜色顺序 (按颜色查找槽位), 可直接改 */
static const uint8_t g_tt_default_color[5] = {
    COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK
};
static uint8_t g_tt_rotate_idx = 0;  /* 当前旋转进度 */


/* ============================================================
 * Task1 QR 映射表 (16 种 × 5 槽位)
 *
 * 原始: QRcode_left[16][5][2][10]
 *   {{"black","A"},{"white","B"},{"red","C"},{"green","D"},{"blue","E"}}
 *   → A=black  B=white  C=red  D=green  E=blue
 *
 * 下表: T1[pattern][slot], slot: 0=A 1=B 2=C 3=D 4=E
 * ============================================================ */
static const uint8_t T1[16][5] = {
    {COLOR_BLACK, COLOR_WHITE, COLOR_RED,   COLOR_GREEN, COLOR_BLUE }, /* 0  */
    {COLOR_WHITE, COLOR_BLACK, COLOR_RED,   COLOR_GREEN, COLOR_BLUE }, /* 1  */
    {COLOR_WHITE, COLOR_BLACK, COLOR_GREEN, COLOR_RED,   COLOR_BLUE }, /* 2  */
    {COLOR_BLUE,  COLOR_WHITE, COLOR_BLACK, COLOR_RED,   COLOR_GREEN}, /* 3  */
    {COLOR_WHITE, COLOR_RED,   COLOR_BLUE,  COLOR_BLACK, COLOR_GREEN}, /* 4  */
    {COLOR_BLACK, COLOR_RED,   COLOR_BLUE,  COLOR_WHITE, COLOR_GREEN}, /* 5  */
    {COLOR_BLUE,  COLOR_GREEN, COLOR_BLACK, COLOR_WHITE, COLOR_RED  }, /* 6  */
    {COLOR_GREEN, COLOR_WHITE, COLOR_BLUE,  COLOR_BLACK, COLOR_RED  }, /* 7  */
    {COLOR_WHITE, COLOR_GREEN, COLOR_BLACK, COLOR_BLUE,  COLOR_RED  }, /* 8  */
    {COLOR_BLACK, COLOR_RED,   COLOR_BLUE,  COLOR_GREEN, COLOR_WHITE}, /* 9  */
    {COLOR_RED,   COLOR_BLUE,  COLOR_GREEN, COLOR_BLACK, COLOR_WHITE}, /* 10 */
    {COLOR_GREEN, COLOR_RED,   COLOR_BLACK, COLOR_BLUE,  COLOR_WHITE}, /* 11 */
    {COLOR_WHITE, COLOR_RED,   COLOR_BLUE,  COLOR_GREEN, COLOR_BLACK}, /* 12 */
    {COLOR_RED,   COLOR_GREEN, COLOR_WHITE, COLOR_BLUE,  COLOR_BLACK}, /* 13 */
    {COLOR_BLUE,  COLOR_WHITE, COLOR_GREEN, COLOR_RED,   COLOR_BLACK}, /* 14 */
    {COLOR_GREEN, COLOR_BLUE,  COLOR_RED,   COLOR_WHITE, COLOR_BLACK}, /* 15 */
};

/* ============================================================
 * Task2 QR 映射表 (6 种 × 3 槽位)
 *
 * 原始: QRcode_right[6][3][2][10]
 *   {{"third","C"},{"second","B"},{"first","A"}}
 *   → C=铜奖  B=银奖  A=金奖
 *
 * 下表: T2[pattern][slot], slot: 0=A 1=B 2=C
 *   1=金奖(first)  2=银奖(second)  3=铜奖(third)
 * ============================================================ */
uint8_t T2[6][3] = {
    {1, 2, 3},  /* A=金 B=银 C=铜 */
    {1, 3, 2},  /* A=金 B=铜 C=银 */
    {2, 1, 3},  /* A=银 B=金 C=铜 */
    {2, 3, 1},  /* A=银 B=铜 C=金 */
    {3, 1, 2},  /* A=铜 B=金 C=银 */
    {3, 2, 1},  /* A=铜 B=银 C=金 */
};

/* ============================================================
 * TT_Init
 * ============================================================ */
void TT_Init(void)
{
    memset(&g_tt, 0, sizeof(g_tt));
    g_tt.idx = 0xFF;
    for (uint8_t i = 0; i < COLOR_COUNT; i++)
        g_tt.rev[i] = SLOT_NONE;
}

/* ============================================================
 * SetQR — 核心: QR 序号 → 解析槽位映射
 * ============================================================ */
void SetQR(uint8_t idx)
{
    g_tt.idx  = idx;
    g_tt.ok   = 1;
    g_tt.cnt  = 5;    /* 固定物块任务: 5 个槽位 */

    if (idx >= 16) { g_tt.ok = 0; return; }

    /* 只写 QR 任务顺序, 不碰 g_tt.color[]/g_tt.rev[] ——
     * 那两块是收集阶段 TT_SetColor 写的物理映射, 这里绝不能覆盖 */
    for (uint8_t s = 0; s < 5; s++)
        g_tt.task_color[s] = T1[idx][s];   /* 槽位→任务要求颜色 */
}

/* ============================================================
 * TT_SetColor — 存检测到的颜色到槽位
 * ============================================================ */
void TT_SetColor(uint8_t slot, Color_TypeDef c)
{
    if (slot >= 5 || c == COLOR_UNKNOWN || c >= COLOR_COUNT) return;

    g_tt.color[slot] = c;         /* 槽位 → 颜色 */
    g_tt.rev[c]      = slot;      /* 颜色 → 槽位 (反向) */
    g_tt.ok          = 1;
}

/* ============================================================
 * SlotByColor — 颜色 → 槽位
 * ============================================================ */
uint8_t SlotByColor(Color_TypeDef c)
{
    if (c == COLOR_UNKNOWN || c >= COLOR_COUNT)
        return SLOT_NONE;
    return g_tt.rev[c];
}

/* ============================================================
 * ColorAtSlot — 槽位 → 颜色
 * ============================================================ */
uint8_t ColorAtSlot(uint8_t slot)
{
    if (!g_tt.ok || slot >= 5) return COLOR_UNKNOWN;
    return g_tt.color[slot];
}

/* ============================================================
 * V1.20.0: 物块采集结果 (形状 / 夹紧回读角度 / 是否已采集)
 * ============================================================ */
void TT_SetShape(uint8_t slot, uint8_t shape)
{
    if (slot >= 5) return;
    g_tt.shape[slot] = shape;
}

void TT_SetRawAngle(uint8_t slot, int16_t raw)
{
    if (slot >= 5) return;
    g_tt.raw_angle[slot] = raw;
}

void TT_SetCollected(uint8_t slot, bool done)
{
    if (slot >= 5) return;
    if (done) {
        g_tt.collected |=  (uint8_t)(1u << slot);
    } else {
        g_tt.collected &= (uint8_t)~(1u << slot);
    }
}

uint8_t ShapeAtSlot(uint8_t slot)
{
    if (slot >= 5) return SHAPE_UNKNOWN;
    return g_tt.shape[slot];
}

int16_t RawAngleAtSlot(uint8_t slot)
{
    if (slot >= 5 || !(g_tt.collected & (1u << slot))) return -1;
    return g_tt.raw_angle[slot];
}

bool TT_IsCollected(uint8_t slot)
{
    if (slot >= 5) return false;
    return (g_tt.collected & (1u << slot)) != 0u;
}

/* ============================================================
 * V1.21.0: 奖杯名次 (转盘槽 1~3 ↔ 下标 0~2)
 * 取值 1=冠军 2=亚军 3=季军, 与 Jang_type / NX 的 '1'/'2'/'3' 同编码。
 * ============================================================ */
void TT_SetTrophy(uint8_t slot, uint8_t rank)
{
    if (slot >= 3) return;
    g_tt.trophy[slot] = rank;
}

uint8_t TrophyAtSlot(uint8_t slot)
{
    if (slot >= 3) return 0u;
    return g_tt.trophy[slot];
}

/* 名次 → 槽位 (反查)。没找到返回 SLOT_NONE。 */
uint8_t SlotByTrophy(uint8_t rank)
{
    if (rank == 0u) return SLOT_NONE;      /* 0 = 未采集, 不是合法名次 */

    for (uint8_t s = 0; s < 3u; s++) {
        if (g_tt.trophy[s] == rank) {
            return s;
        }
    }
    return SLOT_NONE;
}

/* ============================================================
 * TT_RotateByQR — 按 QR 颜色顺序, 旋转到每个颜色所在物理槽位
 * ============================================================ */
/** `TT_RotateByQR()` 最近转到门口的**物理槽号** (1~5); 0 = 还没转过。
 *  给 `Place()` 松夹爪用 (物料摆放逐个放, 必须知道当前是哪个槽在门口)。 */
static uint8_t g_tt_last_slot = 0u;

bool TT_RotateByQR(void)
{
    uint8_t cnt = (g_tt.ok && g_tt.idx < 16) ? g_tt.cnt : 5;
    uint8_t slot;

    /* cnt 未设置(=0)时按 5 处理, 保证找圆进度能推进 */
    if (cnt == 0) cnt = 5;
    if (g_tt_rotate_idx >= cnt) return false;

    if (g_tt.ok && g_tt.idx < 16) {
        /* QR 模式: 第 i 个圆需要 task_color[i], 用物理反向索引找它实际所在槽 */
        uint8_t need = g_tt.task_color[g_tt_rotate_idx];
        slot = SlotByColor(need);          /* g_tt.rev[] = 收集阶段写入的物理槽位 */
        if (slot == SLOT_NONE) slot = g_tt_rotate_idx;   /* 兜底: 保证转盘一定转 */
    } else {
        /* 默认固定顺序: 直接按槽位号 1→2→3→4→5 */
        slot = g_tt_rotate_idx;
    }

    g_tt_rotate_idx++;
    BlockBasic_TurntableTo(slot + 1);
    g_tt_last_slot = (uint8_t)(slot + 1u);   /* 记为"当前在门口"的物理槽号 (1~5) */
    osDelay(500);
    return true;
}

uint8_t TT_CurrentSlot(void)
{
    return g_tt_last_slot;
}

void TT_RotateReset(void)
{
    g_tt_rotate_idx = 0;
}

bool TT_IsDone(void)
{
    uint8_t cnt = (g_tt.ok && g_tt.idx < 16) ? g_tt.cnt : 5;
    if (cnt == 0) cnt = 5;
    return g_tt_rotate_idx >= cnt;
}

/* ============================================================
 * TogetPos — 取点位坐标
 * ============================================================ */


void TogetPos(uint8_t slot, float *x, float *y, float *yaw)
{
    if (slot >= 5) {
        if (x) *x = 0; if (y) *y = 0; if (yaw) *yaw = 0;
        return;
    }
    if (x)   *x   = g_tt.pos[slot].x;
    if (y)   *y   = g_tt.pos[slot].y;
    if (yaw) *yaw = g_tt.pos[slot].yaw;
}

/* ============================================================
 * SetPos — 标定点位
 * ============================================================ */
void SetPos(uint8_t slot, float x, float y, float yaw)
{
    if (slot >= 5) return;
    g_tt.pos[slot].x   = x;
    g_tt.pos[slot].y   = y;
    g_tt.pos[slot].yaw = yaw;
}