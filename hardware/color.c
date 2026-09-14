/**
 * @file color.c
 * @brief 颜色传感器 — GY-33 / OpenMV 双驱动
 *   - USE_OPENMV_COLOR 0: GY-33,  帧: 5A 5A type qty data[qty] chk
 *   - USE_OPENMV_COLOR 1: OpenMV, 帧: AA CC L A B BB DD (Lab 值, OpenMV 端采集)
 */
#include "Common_used.h"
#include <stdlib.h>  /* abs() */

/* ---- 内部参数 ---- */
#define CALIB_TOLERANCE   30U
#define WARMUP_COUNT      1U
#define SAMPLE_COUNT      5U
#define SKIP_COUNT        1U
#define ACCEPT_COUNT      2U

/* ================================================================
 * 手动校准常量 (OpenMV Lab 模式)
 * 校准流程: COLOR_CALIB_MODE 1 → 看串口 Lab 值 → 填入下方 → 重新编译
 * ================================================================ */
#if USE_OPENMV_COLOR == 0
/* 环境光 (空槽) */
#define LAB_AMB_L   34
#define LAB_AMB_A   125
#define LAB_AMB_B   129
#define LAB_AMB_TOL 15

/* 各颜色参考 Lab 值 + 通道权重 + 欧几里得距离容差 */
/* 权重: 黑白只看 L, 红绿蓝只看 A/B */
#define LAB_RED_L    89
#define LAB_RED_A    168
#define LAB_RED_B    140
#define LAB_RED_WL   0
#define LAB_RED_WA   3
#define LAB_RED_WB   1
#define LAB_RED_TOL  30

#define LAB_GREEN_L  83
#define LAB_GREEN_A  97
#define LAB_GREEN_B  148
#define LAB_GREEN_WL 0
#define LAB_GREEN_WA 3
#define LAB_GREEN_WB 1
#define LAB_GREEN_TOL 30

#define LAB_BLUE_L   96
#define LAB_BLUE_A   125
#define LAB_BLUE_B   90
#define LAB_BLUE_WL  0
#define LAB_BLUE_WA  1
#define LAB_BLUE_WB  3
#define LAB_BLUE_TOL 30

/* 黑/白: 白=高L+低黑占比, 黑=高黑占比 */
#define WHITE_L_MIN       81    /* l_mean >= 此值 且 */
#define WHITE_BLACK_MAX   40    /* l_black <= 此值 → 白色 */
#define BLACK_RATIO_MIN  180    /* l_black >= 此值 → 黑色 */
#endif /* USE_OPENMV_COLOR */

/* ---- 校准数据 (全局) ---- */
Color_Calib_t  g_color_calib[COLOR_COUNT];
Color_Ambient_t g_color_ambient;

/* ================================================================
 * 协议层 — GY-33
 * ================================================================ */
#if USE_OPENMV_COLOR == 0

static void Color_SendCmd(uint8_t cmd)
{
    uint8_t txbuf[3];
    txbuf[0] = 0xA5U;
    txbuf[1] = cmd;
    txbuf[2] = (uint8_t)((0xA5U + cmd) & 0xFFU);
    HAL_UART_Transmit(&huart2, txbuf, 3U, 50U);
}

static bool Color_ReadFrame(uint8_t *r, uint8_t *g, uint8_t *b)
{
    int retry = 500;
    while (!g_uart2_gy33_ready && --retry > 0) {
        osDelay(2);
    }
    if (retry <= 0) return false;

    *r = g_uart2_gy33_r;
    *g = g_uart2_gy33_g;
    *b = g_uart2_gy33_b;
    g_uart2_gy33_ready = 0;
    return true;
}

HAL_StatusTypeDef Color_Init(void)
{
    Color_SendCmd(0x81U);
    HAL_Delay(50U);
    return HAL_OK;
}

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    if (level > 10U) return HAL_ERROR;
    Color_SendCmd(0x60U | level);
    return HAL_OK;
}

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
    uint8_t r = 0, g = 0, b = 0;
    if (data == NULL) return HAL_ERROR;
    if (!Color_ReadFrame(&r, &g, &b)) {
        data->online = 0U;
        return HAL_ERROR;
    }
    data->red = r; data->green = g; data->blue = b;
    data->online = 1U;
    return HAL_OK;
}

Color_TypeDef Color_Judge(const Color_DataTypeDef *data)
{
    if (data == NULL || data->online == 0U) return COLOR_UNKNOWN;
    uint8_t r = data->red, g = data->green, b = data->blue;

    if (g_color_ambient.enabled) {
        int dr = abs((int)r - g_color_ambient.r);
        int dg = abs((int)g - g_color_ambient.g);
        int db = abs((int)b - g_color_ambient.b);
        if (dr <= g_color_ambient.tolerance &&
            dg <= g_color_ambient.tolerance &&
            db <= g_color_ambient.tolerance)
            return COLOR_UNKNOWN;
    }

    for (int i = COLOR_RED; i < COLOR_COUNT; i++) {
        Color_Calib_t *c = &g_color_calib[i];
        if (!c->enabled) continue;
        int dr = abs((int)r - c->r), dg = abs((int)g - c->g), db = abs((int)b - c->b);
        if (dr <= (int)c->tolerance &&
            dg <= (int)c->tolerance &&
            db <= (int)c->tolerance)
            return (Color_TypeDef)i;
    }

    if (r >= 150 && g >= 150 && b >= 150) return COLOR_WHITE;
    if (b >= r && b >= g && b >= 100)      return COLOR_BLUE;
    if (r >= g && r >= b && r >= 100)      return COLOR_RED;
    if (g >= r && g >= b && g >= 100)      return COLOR_GREEN;
    return COLOR_BLACK;
}

/* ================================================================
 * 协议层 — OpenMV: AA CC L A B BB DD (7字节 Lab 帧)
 *   L: 0~100 (明度), A/B: 原始值+128 偏移为 0~255
  * 判断层 — Lab 欧几里得距离校准
 * ================================================================ */
#else

/* ---- 帧解析: AA L A B R G B DD (8字节), UART2 DMA 接收 ---- */
static bool Color_ReadFrame(uint8_t *l_black, uint8_t *l_mean,
                            uint8_t *a, uint8_t *b)
{
    int retry = 500;
    while (!g_uart2_color_ready && --retry > 0) {
        osDelay(2);
    }
    if (retry <= 0) return false;

    *l_black = g_uart2_color_l;
    *l_mean  = g_uart2_color_l_mean;
    *a       = g_uart2_color_a;
    *b       = g_uart2_color_b;
    g_uart2_color_ready = 0;
    return true;
}

HAL_StatusTypeDef Color_Init(void)
{
    return HAL_OK;  /* OpenMV 上电自启 */
}

HAL_StatusTypeDef Color_SetLedLevel(uint8_t level)
{
    (void)level;
    return HAL_OK;  /* OpenMV 自带补光 */
}

HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data)
{
    uint8_t l_black = 0, l_mean = 0, a = 0, b = 0;
    if (data == NULL) return HAL_ERROR;
    if (!Color_ReadFrame(&l_black, &l_mean, &a, &b)) {
        data->online = 0U;
        return HAL_ERROR;
    }
    data->l   = l_black;  /* 黑色像素占比 */
    data->red = l_mean;   /* Lab L 0~100, 用于环境光/白色 */
    data->a   = a;        /* Lab A+128 */
    data->b   = b;        /* Lab B+128 */
    data->online = 1U;
    return HAL_OK;
}

Color_TypeDef Color_Judge(const Color_DataTypeDef *data)
{
    if (data == NULL || data->online == 0U) return COLOR_UNKNOWN;

    uint8_t a = data->a, b = data->b;

    /* 1. 红/绿/蓝: Lab 加权距离 */
    {
        struct { uint8_t l, a, b; uint8_t wl, wa, wb; uint8_t tol; Color_TypeDef color; }
        static const ref[] = {
            { LAB_RED_L,   LAB_RED_A,   LAB_RED_B,   LAB_RED_WL,   LAB_RED_WA,   LAB_RED_WB,   LAB_RED_TOL,   COLOR_RED   },
            { LAB_GREEN_L, LAB_GREEN_A, LAB_GREEN_B, LAB_GREEN_WL, LAB_GREEN_WA, LAB_GREEN_WB, LAB_GREEN_TOL, COLOR_GREEN },
            { LAB_BLUE_L,  LAB_BLUE_A,  LAB_BLUE_B,  LAB_BLUE_WL,  LAB_BLUE_WA,  LAB_BLUE_WB,  LAB_BLUE_TOL,  COLOR_BLUE  },
        };
        uint32_t best_dist_sq = 0xFFFFFFFFUL;
        Color_TypeDef best = COLOR_UNKNOWN;
        for (size_t i = 0; i < sizeof(ref)/sizeof(ref[0]); i++) {
            int da = (int)a - ref[i].a;
            int db = (int)b - ref[i].b;
            uint32_t dist_sq = (uint32_t)(
                (int)ref[i].wa * da * da +
                (int)ref[i].wb * db * db);
            uint32_t tol_sq = (uint32_t)ref[i].tol * ref[i].tol;
            if (dist_sq <= tol_sq && dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                best = ref[i].color;
            }
        }
        if (best != COLOR_UNKNOWN) return best;
    }

    /* 2. 黑/白 */
    {
        if (data->red >= WHITE_L_MIN && data->l <= WHITE_BLACK_MAX) return COLOR_WHITE;
        if (data->l >= BLACK_RATIO_MIN) return COLOR_BLACK;
    }

    /* 3. 空槽 (环境光) */
    {
        int dl = (int)data->red - LAB_AMB_L;
        int da = (int)a - LAB_AMB_A;
        int db = (int)b - LAB_AMB_B;
        uint32_t dist_sq = (uint32_t)(dl * dl + da * da + db * db);
        uint32_t tol_sq = (uint32_t)LAB_AMB_TOL * LAB_AMB_TOL;
        if (dist_sq <= tol_sq)
            return COLOR_UNKNOWN;
    }

    return COLOR_UNKNOWN;
}

#endif /* USE_OPENMV_COLOR */

/* ================================================================
 * 共用
 * ================================================================ */

Color_TypeDef Color_DetectDominant(void)
{
    uint8_t counts[COLOR_COUNT] = {0U};
    Color_TypeDef dominant = COLOR_UNKNOWN;
    uint8_t max_count = 0U;

    for (uint8_t w = 0; w < WARMUP_COUNT; w++) { Color_Init(); HAL_Delay(5U); }

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        Color_DataTypeDef d;
        if (Color_ReadData(&d) == HAL_OK) {
            Color_TypeDef c = Color_Judge(&d);
            if (i >= SKIP_COUNT && c > COLOR_UNKNOWN && c < COLOR_COUNT)
                counts[c]++;
        }
        HAL_Delay(5U);
    }

    for (int i = COLOR_RED; i < COLOR_COUNT; i++)
        if (counts[i] > max_count) { max_count = counts[i]; dominant = (Color_TypeDef)i; }

    return (max_count >= ACCEPT_COUNT) ? dominant : COLOR_UNKNOWN;
}

const char *Color_ToString(Color_TypeDef color)
{
    switch (color) {
    case COLOR_RED:   return "RED";
    case COLOR_GREEN: return "GREEN";
    case COLOR_BLUE:  return "BLUE";
    case COLOR_WHITE: return "WHITE";
    case COLOR_BLACK: return "BLACK";
    default:          return "UNKNOWN";
    }
}

/* ---- Flash 存储 ---- */
#define FLASH_CALIB_PAGE   255U
#define FLASH_CALIB_ADDR   0x0807F800U
#define FLASH_PAGE_SIZE    0x800U

/* ================================================================
 * 校准 — 仅 GY-33
 * ================================================================ */
#if USE_OPENMV_COLOR == 0

void Color_Calibrate(Color_TypeDef color)
{
    if (color <= COLOR_UNKNOWN || color >= COLOR_COUNT) return;

    Color_DataTypeDef d;
    if (Color_ReadData(&d) == HAL_OK) {
        g_color_calib[color].r = d.red;
        g_color_calib[color].g = d.green;
        g_color_calib[color].b = d.blue;
        g_color_calib[color].tolerance = CALIB_TOLERANCE;
        g_color_calib[color].enabled = 1U;
    }
}

void Color_CalibAmbient(void)
{
    Color_DataTypeDef d;
    if (Color_ReadData(&d) == HAL_OK) {
        g_color_ambient.r = d.red;
        g_color_ambient.g = d.green;
        g_color_ambient.b = d.blue;
        g_color_ambient.tolerance = 20U;
        g_color_ambient.enabled = 1U;
    }
}

/* ---- Flash magic ---- */
#define FLASH_CALIB_MAGIC  0x434F4C52U

static uint32_t CalibChecksum(void)
{
    uint32_t sum = 0;
    uint8_t *p = (uint8_t *)g_color_calib;
    for (size_t i = 0; i < sizeof(g_color_calib); i++) sum += p[i];
    p = (uint8_t *)&g_color_ambient;
    for (size_t i = 0; i < sizeof(g_color_ambient); i++) sum += p[i];
    return sum;
}

void Color_CalibSave(void)
{
    uint32_t header[2] = { FLASH_CALIB_MAGIC, CalibChecksum() };
    uint32_t page_start = FLASH_CALIB_ADDR;
    uint32_t addr;

    __disable_irq();
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_1,
        .Page      = FLASH_CALIB_PAGE,
        .NbPages   = 1U,
    };
    uint32_t err;
    HAL_FLASHEx_Erase(&erase, &err);

    addr = page_start;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, *(uint64_t *)header);
    addr += 8;

    uint32_t blob[12];
    memcpy(&blob[0], g_color_calib, sizeof(g_color_calib));
    memcpy(&blob[9], &g_color_ambient, sizeof(g_color_ambient));
    for (int i = 0; i < 12; i += 2) {
        uint64_t d = (uint64_t)blob[i] | ((uint64_t)blob[i+1] << 32);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, d);
        addr += 8;
    }

    HAL_FLASH_Lock();
    __enable_irq();
}

void Color_CalibLoad(void)
{
    for (int i = 0; i < COLOR_COUNT; i++) {
        g_color_calib[i].enabled = 0U;
        g_color_calib[i].tolerance = CALIB_TOLERANCE;
    }
    g_color_ambient.enabled = 0U;

    uint32_t *flash = (uint32_t *)FLASH_CALIB_ADDR;
    if (flash[0] != FLASH_CALIB_MAGIC) return;

    uint32_t blob[12];
    for (int i = 0; i < 12; i++) blob[i] = flash[2 + i];

    memcpy(g_color_calib, &blob[0], sizeof(g_color_calib));
    memcpy(&g_color_ambient, &blob[9], sizeof(g_color_ambient));

    if (flash[1] != CalibChecksum()) {
        for (int i = 0; i < COLOR_COUNT; i++) g_color_calib[i].enabled = 0U;
        g_color_ambient.enabled = 0U;
    }
}

#else

/* 手动校准模式: 校准时打印 Lab 值, 用户填入 color.c 顶部 #define 常量 */
void Color_CalibSave(void)   { printf("[CALIB] Manual mode: copy Lab values above\r\n"); }
void Color_CalibLoad(void)   {}
void Color_Calibrate(Color_TypeDef color)
{
    Color_DataTypeDef d;
    if (Color_ReadData(&d) == HAL_OK) {
        printf("[CALIB] %s: L=%d A=%d B=%d  ← copy to #define\r\n",
               Color_ToString(color), d.l, d.a, d.b);
    }
}
void Color_CalibAmbient(void)
{
    Color_DataTypeDef d;
    if (Color_ReadData(&d) == HAL_OK) {
        printf("[CALIB] AMBIENT: L=%d A=%d B=%d  ← copy to LAB_AMB_*\r\n", d.l, d.a, d.b);
    }
}
#endif /* USE_OPENMV_COLOR */
