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
    float fl_raw = ( v + geo_factor * w) * MEC_SPEED_COEFF;
    float fr_raw = ( v - geo_factor * w) * MEC_SPEED_COEFF;
    float rl_raw = ( v + geo_factor * w) * MEC_SPEED_COEFF;
    float rr_raw = ( v - geo_factor * w) * MEC_SPEED_COEFF;

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


uint32_t malu_cm_topluse_s(float cm)
{
    return (uint32_t)(cm / (2.0f * MEC_WHEEL_RADIUS * 3.14159265358979f) * 3200);
}

/* ================================================================
 *  位置模式定距移动 —— 按车体位移算四个轮子的脉冲数, 直接发下去
 *
 *  与 Mecanum_Calc_Full_V 同一套逆解, 只是把"速度"换成"位移"
 *  (两者是线性的, 系数相同), 再把每个轮子的行程换算成 Emm_V5 位置模式的
 *  脉冲数。开环: 发完就等, 不读编码器。
 *
 *  极性映射逐字节镜像 hardware/actuators/Send_motor.c 的 Send_commandmotor():
 *      地址 1 = 前右, 方向位取反
 *      地址 2 = 后左, 方向位原样
 *      地址 3 = 前左, 方向位取反
 *      地址 4 = 后右, 方向位原样
 *  两处必须保持一致, 改任何一边都要同步另一边。
 * ================================================================ */

/** 发一个轮子的定距位置指令。dist_m 带符号, >0 = 该轮的"正转"方向。 */
static void Mecanum_MoveOneWheel(uint8_t addr, float dist_m, uint16_t vel_rpm)
{
    if (fabsf(dist_m) < MEC_POS_MIN_M) {
        return;                     /* 太短, 不值得发, 也免得脉冲数算成 0 */
    }

    /* malu_cm_topluse_s() 收 cm 且返回**无符号** —— 符号在这里自己处理 */
    uint32_t clk = malu_cm_topluse_s(fabsf(dist_m) * 100.0f);
    if (clk == 0u) {
        return;
    }

    uint8_t dir;
    if (addr == 1u || addr == 3u) {
        dir = (dist_m > 0.0f) ? 0u : 1u;   /* 前右 / 前左: 取反 */
    } else {
        dir = (dist_m > 0.0f) ? 1u : 0u;   /* 后左 / 后右: 原样 */
    }

    /* raF = 0 → 相对运动 (走这么多脉冲就停); snF = 0 → 立即执行, 不等同步广播。
     * 四条 CAN 帧前后脚发出去, 间隔亚毫秒级, 定距平移够用。 */
    Emm_V5_Pos_Control(addr, dir, vel_rpm, MEC_POS_ACC, clk, 0, 0);
}

/** 四个轮子都报到位了吗 (每个轮子一次 CAN 往返, 内部超时 50ms)。 */
static bool Mecanum_AllReached(void)
{
    /* 顺序不重要: 用 && 短路, 第一个没到就不再问后面三个, 省总线 */
     return Emm_V5_Is_Reached(1u);
    // && Emm_V5_Is_Reached(2u) &&
    //        Emm_V5_Is_Reached(3u) && Emm_V5_Is_Reached(4u);
}

bool Mecanum_MoveBodyPos(float fwd_m, float left_m, uint16_t timeout_ms)
{
    /* 麦轮逆解只取平移两项 (w = 0)。与 Mecanum_Calc_Full_V 同式。 */
    const float fl = fwd_m - left_m;
    const float fr = fwd_m + left_m;
    const float rl = fwd_m + left_m;
    const float rr = fwd_m - left_m;
if (fwd_m>=0)
{
    Mecanum_MoveOneWheel(1u, fr, MEC_POS_VEL_RPM);   /* 前右 */
    Mecanum_MoveOneWheel(2u, rl, MEC_POS_VEL_RPM);   /* 后左 */
    Mecanum_MoveOneWheel(3u, fl, MEC_POS_VEL_RPM);   /* 前左 */
    Mecanum_MoveOneWheel(4u, rr, MEC_POS_VEL_RPM);   /* 后右 */
}
    else
    {
        Mecanum_MoveOneWheel(1u, fr, MEC_POS_VELh_RPM);   /* 前右 */
        Mecanum_MoveOneWheel(2u, rl, MEC_POS_VELh_RPM);   /* 后左 */
        Mecanum_MoveOneWheel(3u, fl, MEC_POS_VELh_RPM);   /* 前左 */
        Mecanum_MoveOneWheel(4u, rr, MEC_POS_VELh_RPM);   /* 后右 */
    }
    if (timeout_ms == 0u) {
        return true;            /* 调用方说不用等 */
    }


    osDelay(MEC_POS_START_DELAY_MS);

    uint32_t t0 = HAL_GetTick();
    for (;;) {
        if (Mecanum_AllReached()) {
            return true;                        /* 四个都到位 */
        }
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return false;
        }
        osDelay(5);
    }
}

/* ================================================================
 *  编码器读取
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


void Odometry_Apply_Calib(float enc_dx, float enc_dy, float *mm_x, float *mm_y)
{
    float rough = (2.0f * 3.14159265f * MEC_WHEEL_RADIUS) / 3200.0f;

    *mm_x = enc_dx * rough;
    *mm_y = enc_dy * rough;
}
