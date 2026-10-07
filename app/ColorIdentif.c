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


/* ============================================================
 * Task1 QR 映射表 (16 种 × 5 槽位)
 *
 * 原始: QRcode_left[16][5][2][10]
 *   {{"black","A"},{"white","B"},{"red","C"},{"green","D"},{"blue","E"}}
 *   → A=black  B=white  C=red  D=green  E=blue
 *
 * 下表: T1[pattern][slot], slot: 0=A 1=B 2=C 3=D 4=E
 * ============================================================ */

static uint8_t T1[4][2]=
{
   {COLOR_BLUE,SHAPE_RECT},
    {COLOR_BLUE,SHAPE_CYLINDER},
    {COLOR_RED,SHAPE_RECT},
    {COLOR_RED,SHAPE_CYLINDER},
};

/** T1[] 的行数 = 摆放阶段要走的轮数 (每行对应一轮 FindCircle)。 */
#define T1_ROWS   (sizeof(T1) / sizeof(T1[0]))

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

/** `TT_SeekBlock()` 的推进游标: 0 = 还没转过, 1..3 = 已按 T1[1..3] 匹配过。
 *  `TT_RotateReset()` 归零。提到文件级是因为它要从 TT_SeekBlock 外重置。 */
static uint8_t s_seek_slot = 0u;

uint8_t TT_SeekBlock() {
    if (s_seek_slot == 0) {
        s_seek_slot = 1u;
        return 1;                       /* 第 1 轮: 物理槽 1 的黄锥 */
    }
    if (s_seek_slot > T1_ROWS) {
        return 0;                       /* T1[] 全部过完 */
    }


    for (uint8_t s = 1u; s <= 4u; s++) {
        if (g_tt.color[s]==T1[s_seek_slot-1][0] && g_tt.shape[s]==T1[s_seek_slot-1][1]) {
            s_seek_slot++;
            return (uint8_t)(s + 1u);   /* 下标 → 物理槽 */
        }
    }

    /* 找不到: 游标**照样推进** (V1.27.0)。
     *
     * 原先是直接 `return 0` 把游标留在原地, 于是下一轮 FindCircle 又从头搜
     * **同一个**组合。而 NF_STAGES 给的是固定 `{Event_FindCircle, 5u}` ——
     * 只要有一个组合匹配不上 (漏料 / 识别不可信), 它就会把**后面所有轮次**
     * 全部吃掉, 剩下的物块一件都放不出去。现场表现就是"漏一个物块之后,
     * 剩下的也不放了"。
     *
     * T1[] 是"每轮推进一行"的表, 不是"直到找到为止"的重试队列 ——
     * 匹配不上就丢掉这一行, 把轮次让给后面的组合。 */
    s_seek_slot++;
    return 0;
}

/* ============================================================
 * TT_BlocksCoverTable — T1[] 要的四个组合是否齐全
 * ============================================================ */
bool TT_BlocksCoverTable(void)
{
    for (uint8_t k = 0u; k < T1_ROWS; k++) {
        bool found = false;

        for (uint8_t s = 1u; s <= 4u && !found; s++) {
            if (TT_IsCollected(s) &&
                g_tt.color[s] == T1[k][0] &&
                g_tt.shape[s] == T1[k][1]) {
                found = true;
            }
        }

        if (!found) {
            return false;
        }
    }
    return true;    /* 四行都有对应的槽 —— 四个物块都放得出去 */
}

void TT_RotateReset(void)
{
    s_seek_slot = 0u;
}
bool TT_RotateByQR(void)
{

    uint8_t slot;

    /* cnt 未设置(=0)时按 5 处理, 保证找圆进度能推进 */


   slot= TT_SeekBlock();

    if (slot!=0) {
        BlockBasic_TurntableTo(slot);
        g_tt_last_slot = slot;   /* 记为"当前在门口"的物理槽号 (1~5) */
        osDelay(500);
        return true;
    }
    return false;
}

uint8_t TT_CurrentSlot(void)
{
    return g_tt_last_slot;
}


/* ============================================================
 * TogetPos — 取点位坐标
 * ============================================================ */


void TogetPos(uint8_t slot, float *x, float *y, float *yaw)
{
    if (slot >= 5) {
        if (x)   *x   = 0;
        if (y)   *y   = 0;
        if (yaw) *yaw = 0;
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