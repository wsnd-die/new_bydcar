#include "Common_used.h"

#define MECANUM_SYNC_ADDR   0U
#define MECANUM_EPSILON     1.0e-7f
#define MECANUM_OMEGA_SIGN  (-1)   /* 硬件实测旋转方向与推导相反，整体翻转 omega；定 -1 为正确值 */



  MecanumConfig_t g_mecanum_config = {
    .wheel_radius_m = 0.0375f,
    .half_length_m = 0.088f,
    .half_width_m = 0.0782f,
    .gear_ratio = 1.0f,
    .pulse_per_rev =3200 ,
    .max_motor_rpm = 400,
    .min_move_time_s = 0.1f,

    /* 驱动器逻辑方向: 1=正向, 0=反向 */
    .forward_dir[MECANUM_ADDR_FR] = 0U,
    .forward_dir[MECANUM_ADDR_RL] = 1U,
    .forward_dir[MECANUM_ADDR_FL] = 0U,
    .forward_dir[MECANUM_ADDR_RR] = 1U,

    /* 驱动器加速度: 脉冲/秒^2 */
    .acceleration = 220U
};

/* ---- 内部工具函数 ---- */

static float Mecanum_AbsFloat(float value)
{
    return value >= 0.0f ? value : -value;
}

static uint32_t Mecanum_RoundUint32(float value)
{
    return (uint32_t)(value + 0.5f);
}

static uint16_t Mecanum_RoundUint16(float value)
{
    return (uint16_t)(value + 0.5f);
}

/**
 * @brief 将轮子的逻辑方向转换为驱动器dir
 */
static uint8_t Mecanum_GetDriverDir(
    const MecanumConfig_t *config,
    uint8_t addr,
    float signed_distance
)
{
    uint8_t forward_dir;

    forward_dir =
        config->forward_dir[addr] != 0U ? 1U : 0U;

    if (signed_distance >= 0.0f) {
        return forward_dir;
    }

    return forward_dir == 0U ? 1U : 0U;
}

/* ============================================================
 * 核心解算实现（内部函数）
 * ============================================================ */

/**
 * @brief 车体坐标麦轮解算（带速度因子）
 *
 * @param speed_ratio  速度因子，1.0 = 全速，0.35 = 35% 最大 RPM
 */
static bool Mecanum_CalcMoveImpl(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    float speed_ratio,
    MecanumMove_t *move
)
{
    float distance[5] = {0.0f};
    float max_abs_motor_rev = 0.0f;
    float duration_s;
    float k;
    uint8_t addr;

    if ((config == NULL) ||
        (move == NULL) ||
        (config->wheel_radius_m <= 0.0f) ||
        (config->half_length_m < 0.0f) ||
        (config->half_width_m < 0.0f) ||
        (config->gear_ratio <= 0.0f) ||
        (config->pulse_per_rev == 0U) ||
        (config->max_motor_rpm == 0U) ||
        (config->min_move_time_s < 0.0f)) {
        return false;
    }

    /* 速度因子限幅 */
    if (speed_ratio <= 0.0f) {
        speed_ratio = 1.0f;
    }
    if (speed_ratio > 1.0f) {
        speed_ratio = 1.0f;
    }

    k = config->half_length_m +
        config->half_width_m;

    /*
     * X型麦轮距离逆运动学。
     *
     * body_dx > 0：向车头前方
     * body_dy > 0：向车体左方
     * dtheta > 0：逆时针
     */

    /* 1号：前右 FR */
    distance[MECANUM_ADDR_FR] =
        body_dx_m +
        body_dy_m +
        k * dtheta_rad;

    /* 2号：后左 RL */
    distance[MECANUM_ADDR_RL] =
        body_dx_m +
        body_dy_m -
        k * dtheta_rad;

    /* 3号：前左 FL */
    distance[MECANUM_ADDR_FL] =
        body_dx_m -
        body_dy_m -
        k * dtheta_rad;

    /* 4号：后右 RR */
    distance[MECANUM_ADDR_RR] =
        body_dx_m -
        body_dy_m +
        k * dtheta_rad;

    move->duration_s = 0.0f;
    move->has_motion = false;

    for (addr = 1U; addr <= 4U; addr++) {
        float wheel_rev;
        float abs_motor_rev;

        wheel_rev =
            distance[addr] /
            (2.0f *
             MECANUM_PI *
             config->wheel_radius_m);

        move->motor[addr].wheel_distance_m =
            distance[addr];

        move->motor[addr].motor_rev =
            wheel_rev * config->gear_ratio;

        move->motor[addr].motor_rpm_signed = 0.0f;
        move->motor[addr].vel = 0U;
        move->motor[addr].clk = 0U;

        move->motor[addr].dir =
            Mecanum_GetDriverDir(
                config,
                addr,
                distance[addr]
            );

        abs_motor_rev =
            Mecanum_AbsFloat(
                move->motor[addr].motor_rev
            );

        if (abs_motor_rev > max_abs_motor_rev) {
            max_abs_motor_rev = abs_motor_rev;
        }
    }

    /*
     * 所有轮子的运动量都接近0。
     */
    if (max_abs_motor_rev < MECANUM_EPSILON) {
        return true;
    }

    /*
     * 运动量最大的电机按 (max_motor_rpm * speed_ratio) 运行，
     * 以此计算共同完成时间。
     *
     * speed_ratio = 1.0  → 全速，行为不变
     * speed_ratio = 0.35 → 所有电机 RPM 降至 35%，耗时为 1/0.35 倍
     */
    {
        float effective_max_rpm;

        effective_max_rpm =
            (float)config->max_motor_rpm * speed_ratio;

        if (effective_max_rpm < 1.0f) {
            effective_max_rpm = 1.0f;   /* 保底 1 RPM */
        }

        duration_s =
            max_abs_motor_rev *
            60.0f /
            effective_max_rpm;
    }

    if (duration_s < config->min_move_time_s) {
        duration_s = config->min_move_time_s;
    }

    if (duration_s <= 0.0f) {
        return false;
    }

    move->duration_s = duration_s;
    move->has_motion = true;

    for (addr = 1U; addr <= 4U; addr++) {
        float abs_motor_rev;
        float abs_motor_rpm;
        float pulse_float;

        abs_motor_rev =
            Mecanum_AbsFloat(
                move->motor[addr].motor_rev
            );

        move->motor[addr].motor_rpm_signed =
            move->motor[addr].motor_rev *
            60.0f /
            duration_s;

        abs_motor_rpm =
            Mecanum_AbsFloat(
                move->motor[addr].motor_rpm_signed
            );

        pulse_float =
            abs_motor_rev *
            (float)config->pulse_per_rev;

        if ((pulse_float > (float)UINT32_MAX) ||
            (abs_motor_rpm > (float)UINT16_MAX)) {
            return false;
        }

        move->motor[addr].clk =
            Mecanum_RoundUint32(pulse_float);

        if (move->motor[addr].clk > 0U) {
            move->motor[addr].vel =
                Mecanum_RoundUint16(abs_motor_rpm);

            if (move->motor[addr].vel == 0U) {
                move->motor[addr].vel = 1U;
            }
        } else {
            /*
             * 该轮不需要运动。
             * 速度给1，避免部分驱动器不接受零速度。
             */
            move->motor[addr].vel = 1U;
        }
    }

    return true;
}

/* ============================================================
 * 公开 API — 车体坐标
 * ============================================================ */

bool Mecanum_CalculateMove(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    MecanumMove_t *move
)
{
    return Mecanum_CalcMoveImpl(
        config, body_dx_m, body_dy_m, dtheta_rad,
        1.0f,    /* 全速 */
        move
    );
}

bool Mecanum_CalculateMoveEx(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    float speed_ratio,
    MecanumMove_t *move
)
{
    return Mecanum_CalcMoveImpl(
        config, body_dx_m, body_dy_m, dtheta_rad,
        speed_ratio,
        move
    );
}

/* ============================================================
 * 公开 API — 世界坐标
 * ============================================================ */

/**
 * @brief World->Body 坐标转换（内部复用）
 */
static bool Mecanum_WorldToBody(
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    float *body_dx,
    float *body_dy
)
{
    float cos_start;
    float sin_start;
    float dx_start_body;
    float dy_start_body;

    cos_start = cosf(start_theta_rad);
    sin_start = sinf(start_theta_rad);

    dx_start_body =
        cos_start * world_dx_m +
        sin_start * world_dy_m;

    dy_start_body =
       -sin_start * world_dx_m +
        cos_start * world_dy_m;

    /*
     * 不旋转时，世界位移只需做普通坐标旋转。
     */
    if (Mecanum_AbsFloat(dtheta_rad) < 1.0e-6f) {
        *body_dx = dx_start_body;
        *body_dy = dy_start_body;
    } else {
        float a;
        float b;
        float denominator;

        /*
         * 固定车体速度并匀速旋转时：
         *
         * dx_world_body0 = a*body_dx - b*body_dy
         * dy_world_body0 = b*body_dx + a*body_dy
         */
        a = sinf(dtheta_rad) / dtheta_rad;

        b =
            (1.0f - cosf(dtheta_rad)) /
            dtheta_rad;

        denominator = a * a + b * b;

        if (denominator < 1.0e-8f) {
            return false;
        }

        /*
         * 对上面的关系求逆，得到需要输入给
         * 车体坐标麦轮解算器的累计运动量。
         */
        *body_dx =
            (a * dx_start_body +
             b * dy_start_body) /
            denominator;

        *body_dy =
            (-b * dx_start_body +
              a * dy_start_body) /
            denominator;
    }

    return true;
}

bool Mecanum_CalculateWorldMove(
    const MecanumConfig_t *config,
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    MecanumMove_t *move
)
{
    float body_dx;
    float body_dy;

    if ((config == NULL) || (move == NULL)) {
        return false;
    }

    if (!Mecanum_WorldToBody(
            world_dx_m, world_dy_m,
            start_theta_rad, dtheta_rad,
            &body_dx, &body_dy)) {
        return false;
    }

    return Mecanum_CalcMoveImpl(
        config, body_dx, body_dy, dtheta_rad,
        1.0f,
        move
    );
}

/* ============================================================
 * 公开 API — 梯形速度斜坡
 * ============================================================ */

/**
 * @brief 一次逻辑运动 -> 两段物理 move（快速接近 + 慢速精停）
 */
bool Mecanum_CalcRampedMoves(
    const MecanumConfig_t *config,
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    const MecanumRamp_t *ramp,
    MecanumMove_t *move_fast,
    MecanumMove_t *move_slow,
    float *total_duration_s
)
{
    float body_dx, body_dy;
    float fast_ratio, slow_speed;
    float remain_ratio;

    if ((config == NULL) ||
        (ramp == NULL) ||
        (move_fast == NULL) ||
        (move_slow == NULL) ||
        (total_duration_s == NULL)) {
        return false;
    }

    /* ---- 1. World -> Body ---- */
    if (!Mecanum_WorldToBody(
            world_dx_m, world_dy_m,
            start_theta_rad, dtheta_rad,
            &body_dx, &body_dy)) {
        return false;
    }

    /* ---- 2. 参数限幅 ---- */
    fast_ratio = ramp->fast_ratio;
    if (fast_ratio <= 0.0f)  fast_ratio = 0.85f;
    if (fast_ratio >= 1.0f)  fast_ratio = 0.95f;

    slow_speed = ramp->slow_speed;
    if (slow_speed <= 0.0f)  slow_speed = 0.35f;
    if (slow_speed >  1.0f)  slow_speed = 1.0f;

    remain_ratio = 1.0f - fast_ratio;

    /* ---- 3. Phase 1: 快速接近段 ---- */
    *move_fast = (MecanumMove_t){0};
    if (!Mecanum_CalcMoveImpl(
            config,
            body_dx * fast_ratio,
            body_dy * fast_ratio,
            dtheta_rad * fast_ratio,
            1.0f,           /* 全速 */
            move_fast)) {
        return false;
    }

    /* ---- 4. Phase 2: 慢速精停段 ---- */
    *move_slow = (MecanumMove_t){0};
    if (!Mecanum_CalcMoveImpl(
            config,
            body_dx * remain_ratio,
            body_dy * remain_ratio,
            dtheta_rad * remain_ratio,
            slow_speed,     /* 降速 */
            move_slow)) {
        return false;
    }

    *total_duration_s =
        move_fast->duration_s + move_slow->duration_s;

    return true;
}

/* ============================================================
 * 执行与停止
 * ============================================================ */

bool Mecanum_ExecuteMove(
    const MecanumConfig_t *config,
    const MecanumMove_t *move
)
{


    /* 临界区 */


    uint8_t addr;

    if ((config == NULL) || (move == NULL)) {
        return false;
    }

    if (!move->has_motion) {
        return true;
    }

    /*
     * 四轮命令暂存，等待同步触发。
     */
    // taskENTER_CRITICAL();
    for (addr = 1U; addr <= 4U; addr++) {
        Emm_V5_Pos_Control(
            addr,
            move->motor[addr].dir,
            move->motor[addr].vel,
            config->acceleration,
            move->motor[addr].clk,

            false, /* raF=false：相对位置模式 */
            false  /* snF=false：多机同步 */
        );
        osDelay(2);

    }
    Emm_V5_Synchronous_motion(0);

    return true;
}

void Mecanum_StopAll(void)
{
    uint8_t addr;

    for (addr = 1U; addr <= 4U; addr++) {
        Emm_V5_Stop_Now(
            addr,
            false
        );
    }
}

/* ============================================================
 * 速度模式 API — 底盘全向速度驱动（定时）
 * ============================================================
 *
 * 车体坐标：vx=前进(+), vy=左移(+), omega=逆时针CCW(+)。
 * 逆运动学：第 i 轮线速度 v_i = KIN[i][FORWARD]*vx + KIN[i][LEFT]*vy + KW_i*omega。
 * X 型麦轮布局与 Mecanum_CalcMoveImpl 同源。
 * 四轮间 5ms 间隔发送，防 CAN 总线并发丢帧。
 * RPM 上限由 g_mecanum_config.max_motor_rpm 限制。
 */

/**
 * @brief 速度模式：使能四轮电机
 */
void Mecanum_EnableAll(void)
{
    uint8_t addr;

    for (addr = 1U; addr <= 4U; addr++) {
        Emm_V5_En_Control(addr, true, false);
    }
    Emm_V5_Synchronous_motion(0);
    osDelay(60);
}

/**
 * @brief 速度模式：减速停止四轮
 */
void Mecanum_VelocityStop(uint8_t acc)
{
    uint8_t addr;

    for (addr = 1U; addr <= 4U; addr++) {
        Emm_V5_Vel_Control(addr, 0U, 0U, acc, false);
        osDelay(5);
    }
    Emm_V5_Synchronous_motion(0);
}

/* ============================================================
 * 单段位置模式运动 — 到位检测，精确停在目标位置
 * ============================================================
 *
 * 相比 Mecanum_CalcRampedMoves（两段式），只发一段位置指令，
 * 驱动器内部对脉冲计数，走到目标 clk 自动停止。
 * 四轮同步触发，先等理论耗时，再读领队轮到位标志确认。
 */

bool Mecanum_MoveWithEncoder(const MecanumConfig_t *config,
                             float body_dx_m, float body_dy_m, float dtheta_rad,
                             float speed_ratio, uint8_t acc, uint32_t timeout_ms)
{
    MecanumMove_t move;
    uint8_t  leader_addr;
    uint32_t max_clk;
    uint8_t  addr;
    uint32_t move_ms;
    int      retry;

    if (config == NULL) {
        return false;
    }

    /* ---- 1. 逆运动学解算 → 各轮 clk + vel + dir + duration_s ---- */
    move = (MecanumMove_t){0};
    if (!Mecanum_CalcMoveImpl(config, body_dx_m, body_dy_m, dtheta_rad,
                              speed_ratio, &move)) {
        return false;
    }

    if (!move.has_motion) {
        return true;
    }

    /* ---- 2. 找领队轮（脉冲最多，耗时最长）---- */
    max_clk = 0U;
    leader_addr = 1U;
    for (addr = 1U; addr <= 4U; addr++) {
        if (move.motor[addr].clk > max_clk) {
            max_clk = move.motor[addr].clk;
            leader_addr = addr;
        }
    }

    /* ---- 3. 位置模式执行（驱动器计脉冲，到目标自动停）---- */
    if (!Mecanum_ExecuteMove(config, &move)) {
        return false;
    }

    /* ---- 4. 等理论耗时（期间不查 CAN，电机正在跑）---- */
    move_ms = (uint32_t)(move.duration_s * 1500.0f);
    osDelay(move_ms);

    /* ---- 5. 轮询领队轮到位标志（位置模式已自动停, 此处只是确认, 少轮询几次即可）---- */
    {
        uint32_t poll_t0 = osKernelGetTickCount();   /* 调试: 量化到位轮询耗时 */
        int reached = 0;
        for (retry = 0; retry < 5; retry++) {
            if (Emm_V5_Is_Reached(leader_addr) == 1) {
                reached = 1;
                break;
            }
            osDelay(50);
        }
        printf("[MOVE] dur=%.2fs poll=%lums reached=%d leader=%u\r\n",
               (double)move.duration_s,
               (unsigned long)(osKernelGetTickCount() - poll_t0),
               reached, (unsigned)leader_addr);
        if (reached) return true;
    }

    /* 超时未读到到位标志也继续（位置模式应该已经自停） */
    return true;
}

/**
 * @brief World坐标版单段位置模式运动（World→Body 转换 + Mecanum_MoveWithEncoder）
 */
bool Mecanum_WorldMoveWithEncoder(const MecanumConfig_t *config,
                                  float world_dx_m, float world_dy_m,
                                  float start_theta_rad, float dtheta_rad,
                                  float speed_ratio, uint8_t acc,
                                  uint32_t timeout_ms)
{
    float body_dx, body_dy;

    if (config == NULL) {
        return false;
    }

    if (!Mecanum_WorldToBody(world_dx_m, world_dy_m,
                             start_theta_rad, dtheta_rad,
                             &body_dx, &body_dy)) {
        return false;
    }

    return Mecanum_MoveWithEncoder(config, body_dx, body_dy, dtheta_rad,
                                   speed_ratio, acc, timeout_ms);
}

/**
 * @brief 速度模式：以指定车体速度运行一段时间后自动减速停止
 *
 * @param vx_m_s      车体前进速度，向前为正，单位：m/s
 * @param vy_m_s      车体左移速度，向左为正，单位：m/s
 * @param omega_rad_s 旋转角速度，逆时针(CCW)为正，单位：rad/s
 * @param duration_ms 运行时长，单位：ms
 * @param acc         驱动器加速度参数（透传给 Emm_V5_Vel_Control）
 */
void Mecanum_VelocityMove(float vx_m_s, float vy_m_s, float omega_rad_s,
                          uint32_t duration_ms, uint8_t acc)
{
    const MecanumConfig_t *cfg = &g_mecanum_config;
    float k;
    float wheel_linear[5];
    uint8_t addr;

    float w; /* 经符号修正后的实际旋转角速度 */

    k = cfg->half_length_m + cfg->half_width_m;

    /* 硬件实测旋转方向与运动学推导相反，MECANUM_OMEGA_SIGN = -1 整体翻转 */
    w = omega_rad_s * (float)MECANUM_OMEGA_SIGN;

    /*
     * X 型麦轮逆运动学（与 Mecanum_CalcMoveImpl 同源）：
     *   FR(1): +vx + vy + k*ω
     *   RL(2): +vx + vy - k*ω
     *   FL(3): +vx - vy - k*ω
     *   RR(4): +vx - vy + k*ω
     */
    wheel_linear[MECANUM_ADDR_FR] = vx_m_s + vy_m_s + k * w;
    wheel_linear[MECANUM_ADDR_RL] = vx_m_s + vy_m_s - k * w;
    wheel_linear[MECANUM_ADDR_FL] = vx_m_s - vy_m_s - k * w;
    wheel_linear[MECANUM_ADDR_RR] = vx_m_s - vy_m_s + k * w;

    for (addr = 1U; addr <= 4U; addr++) {
        float abs_linear;
        float rpm_float;
        uint16_t rpm;
        uint8_t dir;

        abs_linear = Mecanum_AbsFloat(wheel_linear[addr]);

        /* 线速度 → RPM：rpm = (v / (2πR)) × 60 × gear_ratio */
        rpm_float = (abs_linear /
                    (2.0f * MECANUM_PI * cfg->wheel_radius_m)) *
                    60.0f * cfg->gear_ratio;

        rpm = Mecanum_RoundUint16(rpm_float);

        /* RPM 上限保护 */
        if (rpm > cfg->max_motor_rpm) {
            rpm = cfg->max_motor_rpm;
        }

        /* 最小启动转速：避免 0 速命令被驱动器忽略 */
        if (abs_linear > MECANUM_EPSILON && rpm == 0U) {
            rpm = 1U;
        }

        dir = Mecanum_GetDriverDir(cfg, addr, wheel_linear[addr]);

        Emm_V5_Vel_Control(addr, dir, rpm, acc, false);

        /* ★ 5ms 间隔，防 CAN 总线并发丢帧 */
        osDelay(5);
    }

    /* 同步触发四轮同时启动 */
    Emm_V5_Synchronous_motion(0);

    /* 等待运行时长 */
    osDelay(duration_ms);

    /* 自动减速停止 */
    Mecanum_VelocityStop(acc);
}

/* ============================================================
 * 指定路径动作（速度模式，以通电位置为原点、车头朝前 +Y 为基准）
 * ============================================================
 *
 * 坐标系：+X 向右(东)，+Y 向前(北)，车头初始朝 +Y。
 * 段① 前进 0.37m           → (0, 0.37)
 * 段② 原地左转 90°(停1s)    → 车头朝 -X(西)
 * 段③ 前进 0.66m(停2s)      → (-0.66, 0.37)
 * 段④ 横向左移 0.20m(停1s)  → (-0.66, 0.17)   （车头仍朝西，左=南=-Y）
 * 段⑤ 半圆弧 r=1.10、弧朝西凸：段④末车头恰朝西即弧下端点(起点)切线，无需预转；
 *      车体以 vx=ω*R 沿车头前进、同时顺时针偏航，车头沿弧切线行进 → (-0.66, 2.01)。
 *
 * 速度/角速度均为保守值，便于观察；
 * 如某段不到位调 V / W_TURN / W_ARC / TURN_L_DEG。
 * 弧末端实测多走~15°(停稳前轮滑动)，已用 A_ARC_DEG 补偿
 * （当前 150，配合 omega 为正即等效顺时针弧；实得≈180°）。
 */
void Mecanum_RunPath(void)
{
    const uint8_t  ACC      = 100;
    const float    V        = 0.2f;    /* 平移速度 m/s */
    const float    W_TURN   = 0.50f;    /* 原地旋转角速度 rad/s（>0=左转/CCW） */
    const float    TURN_L_DEG = 85.0f;  /* 左转指令角(deg)：实测左转比指令多~15°，指令角调到80°实得≈90° */
    const float    W_ARC    = 0.30f;    /* 半圆弧角速度 rad/s（车体沿弧前进并顺时针偏航） */
    const float    R_ARC    = 0.9f;    /* 半圆弧半径 m */
    const uint32_t REST1    = 1000;
    const uint32_t REST2    = 2000;
    const uint32_t REST3    = 1000;

    /* 开头统一使能四轮 */
    Mecanum_EnableAll();

    printf("PATH: origin(0,0) heading +Y\r\n");

    /* ---- 段① 前进 0.37m ---- */
    printf("PATH seg1: forward 0.37 -> (0,0.37)\r\n");
    Mecanum_VelocityMove(V, 0.0f, 0.0f,
                         (uint32_t)(0.37f / V * 1000.0f), ACC);

    /* ---- 段② 原地左转（车头 北→西），停留 1s ----
     * 实测左转比指令多~15°，故指令角减到 TURN_L_DEG(80°) 使实际回到约 90°。 */
    printf("PATH seg2: turn LEFT (cmd %.0f deg), rest 1s\r\n", (double)TURN_L_DEG);
    Mecanum_VelocityMove(0.0f, 0.0f, -W_TURN,
                         (uint32_t)((TURN_L_DEG / 180.0f * MECANUM_PI) / W_TURN * 1000.0f),
                         ACC);
    osDelay(REST1);

    /* ---- 段③ 前进 0.66m（车头朝西，沿车头走），停留 2s ---- */
    printf("PATH seg3: forward 0.66 -> (-0.66,0.37), rest 2s\r\n");
    Mecanum_VelocityMove(V, 0.0f, 0.0f,
                         (uint32_t)(0.66f / V * 1000.0f), ACC);
    osDelay(REST2);

    /* ---- 段④ 横向左移 0.20m（车头朝西，左=南=-Y），停留 1s ---- */
    printf("PATH seg4: left 0.20 -> (-0.66,0.17), rest 1s\r\n");
    Mecanum_VelocityMove(0.0f, V, 0.0f,
                         (uint32_t)(0.23f / V * 1000.0f), ACC);
    osDelay(REST3);

    /* ---- 段⑤ 半圆弧（车体沿弧线行进，车头=弧切线方向，弧朝西凸）----
     * 段④结束车头恰朝西，正是弧下端点（起点）的切线方向，无需预转。
     * 车体以 vx=ω*R 沿车头前进、同时顺时针(右)偏航 ω，车头随弧切线：
     * 西→西凸侧→北→东，圆心在车头右侧(北)1.10m。
     * vx = +W_ARC * R_ARC（前进）, vy = 0, omega = +W_ARC（经 forward_dir 等效顺时针西凸）。
     * 时长 = (A_ARC_DEG/180*π) / W_ARC。 */
    {
        float v_forw;
        const float A_ARC_DEG = 150.0f; /* arc cmd yaw deg: 实测调优；配合 omega 为正实得≈180° 顺时针弧 */

        v_forw = W_ARC * R_ARC;

        printf("PATH seg5: semicircle r=%.2f (heading follows arc, bulge west) -> (-0.66,2.01)\r\n",
               (double)R_ARC);
        Mecanum_VelocityMove(v_forw, 0.0f, W_ARC,
                             (uint32_t)((A_ARC_DEG / 180.0f * MECANUM_PI) / W_ARC * 1000.0f),
                             ACC);
    }

    printf("PATH: done\r\n");
}
