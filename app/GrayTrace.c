/**
 * @file GrayTrace.c
 * @brief 八通道灰度循迹控制器 — Track_Err 查表 + 比例转向 + 速度自适应
 *
 * 误差: 按《8路循迹模块》Track_Err 映射表由灰度字节计算 (同参考 Trace_base):
 *   - 全白(丢线)/全黑/未识别图案 → err=0 (直行)
 * 转向: 反馈 w = err × GRAY_ERR_TO_W_GAIN + 前馈 K230角度 × GRAY_K230_FF_GAIN
 *       (灰度查表误差做反馈, K230 线角度做弯道前馈)
 * 速度: 误差自适应 (误差越大越慢) + v_smooth 低通 (加速慢/减速快),
 *       起步/新一段循迹从 0 缓启动 (v_smooth 从 0 起步 + 空闲超时重置)
 */

#include "Common_used.h"
#include "GrayTrace.h"

/* trace_tune.c 串口调参 (#gkp/#gki/#gkd) 引用的灰度 PID 全局。
 * 本控制器已改比例转向, 该 PID 不参与控制, 仅保留定义保证链接通过。 */
pid_type_def g_pid_gray;

/* ============================================================
 * 串口调参 (USART1, '$' 前缀 — 与 trace_tune 的 '#' 不冲突):
 *   $egain 0.1    → 反馈增益 g_gray_err_gain
 *   $ffgain -0.06 → 前馈增益 g_gray_ff_gain (负数 = 翻转方向)
 *   $get          → 打印当前两个增益
 * ============================================================ */
float g_gray_err_gain = GRAY_ERR_TO_W_GAIN;   /* 上电默认 = 头文件宏 */
float g_gray_ff_gain  = GRAY_K230_FF_GAIN;

#define GRAY_TUNE_CMD_MAX 24U
static char    s_tune_line[GRAY_TUNE_CMD_MAX];
static uint8_t s_tune_len    = 0U;
static uint8_t s_in_tune     = 0U;
static volatile uint8_t s_tune_pending = 0U;

bool GrayTrace_Tune_OnByte(uint8_t b)
{
    if (!s_in_tune) {
        if (b != '$') return false;          /* 只认 $, 与 trace_tune 的 # 不冲突 */
        s_in_tune = 1U;
        s_tune_len = 0U;
        return true;
    }
    if (b == '\r' || b == '\n') {
        s_tune_line[s_tune_len] = '\0';
        s_tune_pending = 1U;
        s_in_tune = 0U;
        s_tune_len = 0U;
        return true;
    }
    if (s_tune_len < GRAY_TUNE_CMD_MAX - 1U) {
        s_tune_line[s_tune_len++] = (char)b;
    }
    return true;
}

static void GrayTrace_Tune_Process(void)
{
    char  cmd[12];
    char *p   = s_tune_line;
    uint8_t n = 0U;
    float  val;

    while (*p && *p != ' ' && *p != '\t' && n < sizeof(cmd) - 1U) {
        cmd[n++] = *p++;
    }
    cmd[n] = '\0';
    while (*p == ' ' || *p == '\t') p++;
    val = (float)atof(p);

    if (strcmp(cmd, "egain") == 0) {
        g_gray_err_gain = val;
        printf("$egain=%.4f\r\n", (double)g_gray_err_gain);
    } else if (strcmp(cmd, "ffgain") == 0) {
        g_gray_ff_gain = val;
        printf("$ffgain=%.4f\r\n", (double)g_gray_ff_gain);
    } else if (strcmp(cmd, "get") == 0) {
        printf("$egain=%.4f $ffgain=%.4f\r\n",
               (double)g_gray_err_gain, (double)g_gray_ff_gain);
    } else {
        printf("$gray-tune: $egain X, $ffgain X, $get\r\n");
    }
}

static void GrayTrace_Tune_Service(void)
{
    if (s_tune_pending) {
        s_tune_pending = 0U;
        GrayTrace_Tune_Process();
    }
}

void GrayTrace_Init(GrayTrace_t *gt)
{
    if (gt == NULL) return;

    Grayscale_Init(&gt->sensor);
    gt->cur_speed  = 0.0f;   /* v_smooth 从 0 起步 → 缓启动 */
    gt->last_tick  = 0U;
    gt->inited = 1U;
}

float GrayTrace_Calc_Error(GrayTrace_t *gt)
{
    uint8_t d;

    if (gt == NULL) return 0.0f;
    d = Grayscale_Get_Digital(&gt->sensor);

    /* Track_Err 查表 (黑线=0; 若转向相反 → 调 GRAY_ERR_TO_W_GAIN 符号) */
    switch (d) {
        case 0xe7: return  0.0f;  /* 中间 */
        case 0xcf: return  3.5f;  /* 右侧小偏差 */
        case 0x9f: return  5.0f;  /* 右侧中等偏差 */
        case 0x3f: return  6.0f;  /* 右侧较大偏差 */
        case 0xf3: return -3.5f;  /* 左侧小偏差 */
        case 0xf9: return -5.0f;  /* 左侧中等偏差 */
        case 0xfc: return -6.0f;  /* 左侧较大偏差 */
        case 0xef: return  2.0f;  /* 右侧极微偏差 */
        case 0xdf: return  3.0f;  /* 右侧微小偏差 */
        case 0xbf: return  3.5f;  /* 右侧微小偏差 */
        case 0x7f: return  7.0f;  /* 右侧极限偏差 */
        case 0xf7: return -2.0f;  /* 左侧极微偏差 */
        case 0xfb: return -3.0f;  /* 左侧微小偏差 */
        case 0xfd: return -4.5f;  /* 左侧较大偏差 */
        case 0xfe: return -7.0f;  /* 左侧极限偏差 */
        case 0x1f: return  8.0f;  /* 右侧极限偏差 */
        case 0xf8: return -3.0f;  /* 左侧微小偏差 */
        case 0x8f: return  9.0f;  /* 右侧极限偏差 */
        default:   return  0.0f;  /* 未识别 / 全白(丢线) / 全黑 */
    }
}

void GrayTrace_Update(GrayTrace_t *gt)
{
    float v, w, error;
    float slow_level, v_target, k;
    MecanumResult motor;

    if (gt == NULL || !gt->inited) return;

    GrayTrace_Tune_Service();             /* 处理 $egain/$ffgain 串口命令 */
    Grayscale_Update(&gt->sensor);        /* 读 8 通道 */
    error = GrayTrace_Calc_Error(gt);     /* Track_Err 查表 */

    /* 反馈: w = err × 增益 (方向不对 → GRAY_ERR_TO_W_GAIN 取负) */
    w = error * g_gray_err_gain;

    /* 前馈: K230 线角度 → 按线斜率提前转向 (无新角度时前馈=0, 退回纯灰度反馈) */
    {
        float ka;
        if (K230_GetLineAngle(&ka)) {
            w += ka * g_gray_ff_gain;   /* 前馈方向反了 → GRAY_K230_FF_GAIN 取负 */
        }
    }
    if (w >  GRAY_W_MAX) w =  GRAY_W_MAX;
    if (w < -GRAY_W_MAX) w = -GRAY_W_MAX;
     printf ("%.2f\r\n", w);

    /* 速度自适应: 误差越大越慢 */
    slow_level = fabsf(error) / GRAY_ERR_MAX;
    if (slow_level > 1.0f) slow_level = 1.0f;
    v_target = GRAY_BASE_SPEED * (1.0f - GRAY_SLOW_FACTOR * slow_level);
    if (v_target < 0.0f) v_target = 0.0f;

    /* 新一段循迹 (停止超时): v_smooth 归零, 重新缓启动 */
    if ((HAL_GetTick() - gt->last_tick) > GRAY_IDLE_RESET_MS) {
        gt->cur_speed = 0.0f;
    }
    gt->last_tick = HAL_GetTick();

    /* v_smooth 低通: 加速慢 / 减速快 */
    k = (v_target < gt->cur_speed) ? GRAY_VSMOOTH_BRAKE : GRAY_VSMOOTH_ACCEL;
    gt->cur_speed += k * (v_target - gt->cur_speed);
    v = gt->cur_speed;

    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}

void GrayTrace_Update_two(GrayTrace_t *gt)   /* 备用入口, 与 Update 逻辑一致 */
{
    float v, w, error;
    float slow_level, v_target, k;
    MecanumResult motor;

    if (gt == NULL || !gt->inited) return;

    GrayTrace_Tune_Service();             /* 处理 $egain/$ffgain 串口命令 */
    Grayscale_Update(&gt->sensor);        /* 读 8 通道 */
    error = GrayTrace_Calc_Error(gt);     /* Track_Err 查表 */

    /* 反馈: w = err × 增益 (方向不对 → GRAY_ERR_TO_W_GAIN 取负) */
    w = error * g_gray_err_gain;

    /* 前馈: K230 线角度 → 按线斜率提前转向 (无新角度时前馈=0, 退回纯灰度反馈) */
    {
        float ka;
        if (K230_GetLineAngle(&ka)) {
            w += ka * g_gray_ff_gain;   /* 前馈方向反了 → GRAY_K230_FF_GAIN 取负 */
        }
    }
    if (w >  GRAY_W_MAX) w =  GRAY_W_MAX;
    if (w < -GRAY_W_MAX) w = -GRAY_W_MAX;
     printf ("%.2f\r\n", w);

    /* 速度自适应: 误差越大越慢 */
    slow_level = fabsf(error) / GRAY_ERR_MAX;
    if (slow_level > 1.0f) slow_level = 1.0f;
    v_target = GRAY_BASE_SPEED * (1.0f - GRAY_SLOW_FACTOR * slow_level);
    if (v_target < 0.0f) v_target = 0.0f;

    /* 新一段循迹 (停止超时): v_smooth 归零, 重新缓启动 */
    if ((HAL_GetTick() - gt->last_tick) > GRAY_IDLE_RESET_MS) {
        gt->cur_speed = 0.0f;
    }
    gt->last_tick = HAL_GetTick();

    /* v_smooth 低通: 加速慢 / 减速快 */
    k = (v_target < gt->cur_speed) ? GRAY_VSMOOTH_BRAKE : GRAY_VSMOOTH_ACCEL;
    gt->cur_speed += k * (v_target - gt->cur_speed);
    v = gt->cur_speed;

    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}
