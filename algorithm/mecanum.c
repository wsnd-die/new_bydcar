#include "../hardware/Common_used.h"
#include "mecanum.h"
#include "can.h"
#include "emm_5v.h"
/**
  * @brief  麦轮单轮转速转换
  * @param  raw_speed : 原始计算速度值 (m/s 等效值)
  * @param  dir       : 输出方向指针 (1=CW, 0=CCW)
  * @retval 处理后的 PWM/RPM 值 (uint16_t)
  */
static uint16_t Mecanum_ProcessWheel(float raw_speed, uint8_t *dir)
{
    /* 1. 方向判断 */
    *dir = (raw_speed >= 0.0f) ? 1 : 0;

    /* 2. 取绝对值 */
    float abs_speed = fabsf(raw_speed);

    /* 3. 最小转速保护*/
    if (abs_speed > 0.0f) {
        abs_speed = fmaxf(abs_speed, MEC_MIN_MOTOR_SPEED);
    }

    /* 4. 限幅到 uint16_t 范围 */
    return (uint16_t)fminf(abs_speed, 65535.0f);
}

/**
  * @brief  麦轮逆运动学解算（单轴模型: V + ω）
  * @note   麦轮正向运动学：
  *         Vx = (ω1 + ω2 + ω3 + ω4) * R / 4
  *         Vy = (-ω1 + ω2 + ω3 - ω4) * R / 4
  *         ω  = (-ω1 + ω2 - ω3 + ω4) * R / (4 * (a + b))
  *
  *         逆解（Vy=0 时）：
  *         ω1 = (V - (a+b)·ω) / R     前左
  *         ω2 = (V + (a+b)·ω) / R     前右
  *         ω3 = (V - (a+b)·ω) / R     后左
  *         ω4 = (V + (a+b)·ω) / R     后右
  *
  *         轮子布局（俯视图）：
  *          前左(ω1) ─── 前右(ω2)
  *             │    ↑x(前)   │
  *             │             │
  *          后左(ω3) ─── 后右(ω4)
  */
MecanumResult Mecanum_Calc(float v, float w)
{
    MecanumResult res = {0, 0, 0, 0, 0, 0, 0, 0};

    /* 静止直接返回 */
    if (fabsf(v) < MEC_STOP_THRESHOLD &&
        fabsf(w) < MEC_STOP_THRESHOLD) {
        return res;
    }

    /* 计算几何因子: (a + b)
     * a = 半轮距 = TRACK_WIDTH / 2
     * b = 半轴距 = WHEELBASE / 2
     */
    float half_track = MEC_TRACK_WIDTH / 2.0f;   /* a */
    float half_base  = MEC_WHEELBASE / 2.0f;     /* b */
    float geo_factor = half_track + half_base;    /* a + b */

    /* ---------- 逆运动学解算 ----------
     * ω_i = (Vx ∓ Vy ∓ (a+b)·ωz) × SPEED_COEFF
     *
     * 单轴模型 Vy=0，所以简化为：
     * ω_1 = V - geo_factor * w    前左
     * ω_2 = V + geo_factor * w    前右
     * ω_3 = V - geo_factor * w    后左
     * ω_4 = V + geo_factor * w    后右
     */
    float fl_raw = ( v - geo_factor * w) * MEC_SPEED_COEFF;
    float fr_raw = ( v + geo_factor * w) * MEC_SPEED_COEFF;
    float rl_raw = ( v - geo_factor * w) * MEC_SPEED_COEFF;
    float rr_raw = ( v + geo_factor * w) * MEC_SPEED_COEFF;

    /* 低速动力补偿 */
    if (fabsf(v) < MEC_LOW_SPEED_LIMIT) {
        fl_raw *= MEC_LOW_SPEED_GAIN;
        fr_raw *= MEC_LOW_SPEED_GAIN;
        rl_raw *= MEC_LOW_SPEED_GAIN;
        rr_raw *= MEC_LOW_SPEED_GAIN;
    }

    /* 方向 + 限幅处理 */
    res.fl_speed = Mecanum_ProcessWheel(fl_raw, &res.fl_dir);
    res.fr_speed = Mecanum_ProcessWheel(fr_raw, &res.fr_dir);
    res.rl_speed = Mecanum_ProcessWheel(rl_raw, &res.rl_dir);
    res.rr_speed = Mecanum_ProcessWheel(rr_raw, &res.rr_dir);

    return res;
}

/**
  * @brief  麦轮全向逆运动学解算（三自由度: Vx + Vy + ω）
  * @note   逆解公式（全部3个自由度）：
  *         ω1 = (Vx - Vy - (a+b)·ω) × coeff   前左（\辊）
  *         ω2 = (Vx + Vy + (a+b)·ω) × coeff   前右（/辊）
  *         ω3 = (Vx + Vy - (a+b)·ω) × coeff   后左（/辊）
  *         ω4 = (Vx - Vy + (a+b)·ω) × coeff   后右（\辊）
  *
  *         其中：
  *         - Vx: 前向速度, Vy: 左向速度, ω: 旋转角速度
  *         - a = 半轮距, b = 半轴距
  *         - coeff = 1 / R (或含单位换算的 SPEED_COEFF)
  */
MecanumResult Mecanum_Calc_Full_V(float vx, float vy, float w)
{
    MecanumResult res = {0, 0, 0, 0, 0, 0, 0, 0};
    /* 静止直接返回 */
    if (fabsf(vx) < MEC_STOP_THRESHOLD &&
        fabsf(vy) < MEC_STOP_THRESHOLD &&
        fabsf(w)  < MEC_STOP_THRESHOLD) {
        return res;
    }
    /* 几何因子 a + b */
    float geo_factor = MEC_TRACK_WIDTH / 2.0f + MEC_WHEELBASE / 2.0f;
    /* ---------- 全自由度逆运动学 ----------
     * ω_i = (Vx ∓ Vy ∓ (a+b)·ω) × SPEED_COEFF
     *
     * 辊子方向（标准麦轮布局）：
     *   前左 / 后右 : \ 型（Vsy 项取 -Vy）
     *   前右 / 后左 : / 型（Vsy 项取 +Vy）
     */
    float fl_raw = ( vx - vy - geo_factor * w) * MEC_SPEED_COEFF;
    float fr_raw = ( vx + vy + geo_factor * w) * MEC_SPEED_COEFF;
    float rl_raw = ( vx + vy - geo_factor * w) * MEC_SPEED_COEFF;
    float rr_raw = ( vx - vy + geo_factor * w) * MEC_SPEED_COEFF;
    /* 低速动力补偿 */
    if (fabsf(vx) < MEC_LOW_SPEED_LIMIT &&
        fabsf(vy) < MEC_LOW_SPEED_LIMIT) {
        fl_raw *= MEC_LOW_SPEED_GAIN;
        fr_raw *= MEC_LOW_SPEED_GAIN;
        rl_raw *= MEC_LOW_SPEED_GAIN;
        rr_raw *= MEC_LOW_SPEED_GAIN;
    }

    /* 方向 + 限幅处理 */
    res.fl_speed = Mecanum_ProcessWheel(fl_raw, &res.fl_dir);
    res.fr_speed = Mecanum_ProcessWheel(fr_raw, &res.fr_dir);
    res.rl_speed = Mecanum_ProcessWheel(rl_raw, &res.rl_dir);
    res.rr_speed = Mecanum_ProcessWheel(rr_raw, &res.rr_dir);

    return res;
}

/* ================================================================
 *  速度模式执行器 —— 把 MecanumResult 下发四轮 (Emm_V5 速度环)
 *  极性映射逐字节镜像 hardware/actuators/Send_motor.c 的
 *  Send_commandmotor(): 前右/前左的方向位取反是底盘装机的硬件约定,
 *  两处必须保持一致, 改动任何一边都要同步另一边。
 * ================================================================ */
void Mecanum_Vel_Execute(const MecanumResult *res)
{
    if (res == NULL)
        return;

    Emm_V5_Vel_Control(1, !res->fr_dir, res->fr_speed, 0, 0); /* 1号=前右 */
    Emm_V5_Vel_Control(2,  res->rl_dir, res->rl_speed, 0, 0); /* 2号=后左 */
    Emm_V5_Vel_Control(3, !res->fl_dir, res->fl_speed, 0, 0); /* 3号=前左 */
    Emm_V5_Vel_Control(4,  res->rr_dir, res->rr_speed, 0, 0); /* 4号=后右 */
    osDelay(5);
    Emm_V5_Synchronous_motion(0);
}

uint32_t malu_cm_topluse_s(float cm)
{
    /* 脉冲 = 厘米 / 周长(2πR) × 每圈脉冲数
     * 注意周长是 2πR 不是 πR, 之前漏了 ×2 会多算一倍脉冲 */
    /* 用带 f 后缀的字面量而非 <math.h> 的 M_PI: M_PI 是 double, 会让整个
     * 表达式提升为双精度运算, 既变慢又改变舍入。此值与 CMSIS-DSP 的 PI 一致。 */
    return (uint32_t)(cm / (2.0f * MEC_WHEEL_RADIUS * 3.14159265358979f) * 3200);
}

/* ================================================================
 *  编码器读取 (通过 CAN → Emm_V5 电机)
 * ================================================================ */

extern volatile uint8_t  can_rx_flag;
extern FDCAN_RxHeaderTypeDef can_rx_header;
extern uint8_t can_rx_data[8];

/* ---- 读取单电机实时转速 (RPM) ---- */
uint8_t Mecanum_Read_Speed(uint8_t id, int16_t *rpm, uint32_t timeout_ms)
{
    uint8_t cmd[3] = {id, 0x35, 0x6B};  // S_VEL
    uint32_t start;

    if (rpm == NULL) return 0;

    can_rx_flag = 0;
    if (can_SendCmd(cmd, 3) == 0) return 0;

    start = osKernelGetTickCount();
    while ((osKernelGetTickCount() - start) < timeout_ms) {
        if (can_rx_flag) {
            can_rx_flag = 0;
            uint8_t rx_id = (uint8_t)(can_rx_header.Identifier >> 8);

            /* 与位置读取同规律: [命令0x35][0x01][转速int16大端][校验0x6B]
             * rpm = data[2..3], 校验 = data[4] */
            if (can_rx_header.IdType == FDCAN_EXTENDED_ID &&
                rx_id == id &&
                can_rx_data[0] == 0x35 &&
                can_rx_data[4] == 0x6B)
            {
                *rpm = (int16_t)((can_rx_data[2] << 8) | can_rx_data[3]);
                return 1;
            }
        }
        osDelay(1);
    }
    return 0;
}

/* ---- 读取单电机实时位置 (编码器累计值) ---- */
uint8_t Mecanum_Read_Position(uint8_t id, int32_t *pos, uint32_t timeout_ms)
{
    uint8_t cmd[3] = {id, 0x36, 0x6B};  // S_CPOS
    uint32_t Pos_midil;
    uint32_t start;

    if (pos == NULL) return 0;

    can_rx_flag = 0;
    if (can_SendCmd(cmd, 3) == 0) return 0;

    start = osKernelGetTickCount();
    while ((osKernelGetTickCount() - start) < timeout_ms) {
        if (can_rx_flag) {
            can_rx_flag = 0;
            uint8_t rx_id = (uint8_t)(can_rx_header.Identifier >> 8);

            /* 实测帧: 36 01 00 00 00 05 6B
             * 格式: [命令0x36][0x01][位置int32大端][校验0x6B]
             * 位置 = data[2..5], 校验 = data[6] */
            if (can_rx_header.IdType == FDCAN_EXTENDED_ID &&
                rx_id == id &&
                can_rx_data[0] == 0x36 &&
                can_rx_data[6] == 0x6B)
            {
                Pos_midil = ((uint32_t)can_rx_data[2] << 24) |
                                 ((uint32_t)can_rx_data[3] << 16) |
                                 ((uint32_t)can_rx_data[4] << 8)  |
                                 ((uint32_t)can_rx_data[5] << 0);
                if (can_rx_data[1]==0x01) {
                    *pos=(int32_t)Pos_midil;
                    *pos=-(*pos);
                }
                else if (can_rx_data[1]==0x00) {
                    *pos=(int32_t)Pos_midil;
                }
                return 1;
            }
        }
        osDelay(1);
    }
    return 0;
}

/* ---- 一次性读取 4 个电机的位置 ---- */
uint8_t Mecanum_Read_AllPositions(EncoderData *enc, uint32_t timeout_ms)
{
    if (enc == NULL) return 0;

    if (!Mecanum_Read_Position(3, &enc->fl, timeout_ms)) return 0;  // 前左
    if (!Mecanum_Read_Position(1, &enc->fr, timeout_ms)) return 0;  // 前右
    if (!Mecanum_Read_Position(2, &enc->rl, timeout_ms)) return 0;  // 后左
    if (!Mecanum_Read_Position(4, &enc->rr, timeout_ms)) return 0;  // 后右

    return 1;
}

/* ================================================================
 *  编码器脉冲 → 毫米
 *
 *  唯一的换算口 —— device/drv_wheel_odom.c 的 wheel_odom_update() 每次
 *  推算都经此把脉冲转成 mm。
 *
 *  V1.14.0 动作：原实现在此之前还有一整块「里程计自动标定」（以 TBOP
 *  定位器为基准反推 scale_x/scale_y 的状态机），已整块移除。理由有二：
 *    · 三个入口函数（Calib_Start / Calib_Update / Is_Calibrated）全工程
 *      零调用，是死代码；
 *    · 它让业务层直接 include 硬件层的 uart2_tbop10.h 并读 TB_position
 *      全局量，违反 CLAUDE.md §5.2.3 的跨层约束。
 *
 *  本函数随之剥离：原实现分「已标定用 scale_x/scale_y」与「未标定用粗略
 *  估算」两条路径，而 g_calib.state 恒为 CALIB_IDLE（状态机从无入口），
 *  标定那条**在此之前就不可达**。故只保留下面这条，数值与删除前逐位一致。
 *
 *  粗略估算: R=3.75cm(轮径≈75mm), 3200脉冲/圈
 *  注意 MEC_WHEEL_RADIUS 单位是 cm, 所以 rough 单位是 cm/脉冲 */
void Odometry_Apply_Calib(float enc_dx, float enc_dy, float *mm_x, float *mm_y)
{
    float rough = (2.0f * 3.14159265f * MEC_WHEEL_RADIUS) / 3200.0f;

    *mm_x = enc_dx * rough;
    *mm_y = enc_dy * rough;
}
