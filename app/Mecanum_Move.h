#ifndef __MECANUM_MOVE_H
#define __MECANUM_MOVE_H

/*
 * 说明：stdbool / stdint / Common_used / tim 等头文件
 * 已由 Common_used.h 统一提供，本文件不再重复包含。
 * （Common_used.h 在包含本文件之前已处理上述依赖）
 */
#ifdef __cplusplus
extern "C" {
#endif

#define MECANUM_PI          3.14159265358979323846f
#define MECANUM_DEG_TO_RAD  (MECANUM_PI / 180.0f)
#define MECANUM_RAD_TO_DEG  (180.0f / MECANUM_PI)

/*
 * 电机地址：
 * 1号：前右 FR
 * 2号：后左 RL
 * 3号：前左 FL
 * 4号：后右 RR
 */
#define MECANUM_ADDR_FR  1U
#define MECANUM_ADDR_RL  2U
#define MECANUM_ADDR_FL  3U
#define MECANUM_ADDR_RR  4U
/**
 * @brief 麦轮底盘参数
 */
typedef struct {
    /* 麦轮半径，单位：m */
    float wheel_radius_m;

    /* 底盘中心到轮轴的前后距离，单位：m */
    float half_length_m;

    /* 底盘中心到轮子的左右距离，单位：m */
    float half_width_m;

    /*
     * 减速比：电机转数 / 轮子转数
     * 直连填写1.0
     */
    float gear_ratio;

    /* 电机转一圈对应的位置指令脉冲数 */
    uint32_t pulse_per_rev;

    /* 最大电机转速，单位：RPM */
    uint16_t max_motor_rpm;

    /* 最短运动时间，单位：s */
    float min_move_time_s;

    /* EMM_V5位置模式加速度参数 */
    uint8_t acceleration;

    /*
     * 轮子向车体前方滚动时，对应驱动器的dir值。
     *
     * forward_dir[1]：前右
     * forward_dir[2]：后左
     * forward_dir[3]：前左
     * forward_dir[4]：后右
     */
    uint8_t forward_dir[5];

} MecanumConfig_t;

/**
 * @brief 单个电机的运动命令
 */
typedef struct {
    /* 带符号的轮缘距离，单位：m */
    float wheel_distance_m;

    /* 带符号的电机转数 */
    float motor_rev;

    /* 带符号的电机转速，单位：RPM */
    float motor_rpm_signed;

    /* 发送给驱动器的速度 */
    uint16_t vel;

    /* 发送给驱动器的位置脉冲数 */
    uint32_t clk;

    /* 发送给驱动器的方向 */
    uint8_t dir;

} MecanumMotorCmd_t;

/**
 * @brief 一次底盘运动的解算结果
 */
typedef struct {
    /*
     * motor[1]：前右
     * motor[2]：后左
     * motor[3]：前左
     * motor[4]：后右
     */
    MecanumMotorCmd_t motor[5];

    /* 四轮理论共同完成时间，单位：s */
    float duration_s;

    /* 是否存在有效运动 */
    bool has_motion;

} MecanumMove_t;

/**
 * @brief 梯形速度斜坡配置
 *
 * 将一次逻辑运动拆为两段物理执行：
 *   Phase 1 - 快速接近段：里程 x fast_ratio，全速运行
 *   Phase 2 - 慢速精停段：剩余里程，降速运行
 *
 * 两段保持相同的平移/旋转比例，轨迹几何不变。
 */
typedef struct {
    float fast_ratio;   /* 快速段里程占比，0.0~1.0，推荐 0.85 */
    float slow_speed;   /* 慢速段速度因子，0.0~1.0，推荐 0.35 */
} MecanumRamp_t;


extern MecanumConfig_t g_mecanum_config;




/**
 * @brief 按车体坐标解算组合运动
 *
 * @param config      底盘参数
 * @param body_dx_m   车体前进累计量，向前为正，单位：m
 * @param body_dy_m   车体横移累计量，向左为正，单位：m
 * @param dtheta_rad  旋转角度，逆时针为正，单位：rad
 * @param move        输出解算结果
 */
bool Mecanum_CalculateMove(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    MecanumMove_t *move
);

/**
 * @brief 按车体坐标解算组合运动（带速度因子）
 *
 * @param speed_ratio  速度因子，0.0~1.0
 *                     1.0 = 全速（等效 CalculateMove）
 *                     0.35 = 最大电机转速降至 35%
 */
bool Mecanum_CalculateMoveEx(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    float speed_ratio,
    MecanumMove_t *move
);

/**
 * @brief 按世界坐标解算组合运动
 *
 * @param config           底盘参数
 * @param world_dx_m       世界X方向位移，单位：m
 * @param world_dy_m       世界Y方向位移，单位：m
 * @param start_theta_rad  动作开始时车头的世界航向，单位：rad
 * @param dtheta_rad       本次需要旋转的角度，逆时针为正
 * @param move             输出解算结果
 */
bool Mecanum_CalculateWorldMove(
    const MecanumConfig_t *config,
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    MecanumMove_t *move
);

/**
 * @brief 梯形速度解算：一次逻辑运动 -> 两段物理 move
 *
 * 先完成 World->Body 转换，再按 ramp 配置将
 * body 位移拆分为快速段和慢速段分别解算。
 *
 * @param config            底盘参数
 * @param world_dx_m        世界X位移，单位：m
 * @param world_dy_m        世界Y位移，单位：m
 * @param start_theta_rad   动作开始时车头航向，单位：rad
 * @param dtheta_rad        旋转角度，逆时针为正
 * @param ramp              斜坡配置
 * @param move_fast         输出：快速段 move
 * @param move_slow         输出：慢速段 move
 * @param total_duration_s  输出：两段总耗时（= fast + slow）
 * @return true             解算成功
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
);

/**
 * @brief 发送四轮相对位置命令并同步启动
 */
bool Mecanum_ExecuteMove(
    const MecanumConfig_t *config,
    const MecanumMove_t *move
);

/**
 * @brief 立即停止四个电机
 */
void Mecanum_StopAll(void);

/* ============================================================
 * 速度模式 API — 底盘全向速度驱动（定时）
 * ============================================================
 *
 * 车体坐标：vx=前进(+), vy=左移(+), omega=逆时针CCW(+)，单位 m/s, m/s, rad/s。
 * 逆运动学与 Mecanum_CalculateMove 同源（X 型麦轮）。
 * 四轮间 5ms 间隔发送，防 CAN 总线并发丢帧。
 * RPM 上限由 g_mecanum_config.max_motor_rpm 限制。
 *
 * 使用方式：
 *   Mecanum_VelocityMove(0.2f, 0, 0, 1000, 80);   // 前进 0.2m/s 持续 1s
 *   Mecanum_VelocityMove(0, 0, 0.5f, 500, 80);     // 原地逆时针旋转 0.5rad/s 持续 0.5s
 *   Mecanum_VelocityStop(80);                        // 减速停止
 */

/**
 * @brief 速度模式：以指定车体速度运行一段时间后自动停止
 *
 * @param vx_m_s      车体前进速度，向前为正，单位：m/s
 * @param vy_m_s      车体左移速度，向左为正，单位：m/s
 * @param omega_rad_s 旋转角速度，逆时针为正，单位：rad/s
 * @param duration_ms 运行时长，单位：ms
 * @param acc         驱动器加速度参数（透传给 Emm_V5_Vel_Control）
 */
void Mecanum_VelocityMove(float vx_m_s, float vy_m_s, float omega_rad_s,
                          uint32_t duration_ms, uint8_t acc);

/**
 * @brief 速度模式：减速停止四轮
 *
 * @param acc  驱动器加速度参数
 */
void Mecanum_VelocityStop(uint8_t acc);

/**
 * @brief 使能四轮电机（速度模式使用前调用）
 */
void Mecanum_EnableAll(void);

/**
 * @brief 指定路径动作（速度模式，以通电位置为原点、车头朝前 +Y 为基准）
 *
 * 坐标系：+X 向右(东)，+Y 向前(北)，车头初始朝 +Y。
 * 段① 前进 0.37m           → (0, 0.37)
 * 段② 原地左转 90°(停1s)    → 车头朝 -X(西)
 * 段③ 前进 0.66m(停2s)      → (-0.66, 0.37)
 * 段④ 横向左移 0.20m(停1s)  → (-0.66, 0.17)
 * 段⑤ 半圆弧 r=1.10、弧朝西凸 → (-0.66, 2.01)
 *
 * 速度/角速度均为保守值；如某段不到位可调内部常量。
 */
void Mecanum_RunPath(void);

/* ============================================================
 * 单段位置模式 — 驱动器计脉冲，精确停在目标位置
 * ============================================================
 *
 * 相比两段式斜坡（Mecanum_CalcRampedMoves），只发一段位置指令，
 * 驱动器内部对脉冲计数，走到目标 clk 自动停止，定位精确。
 * 四轮同步触发，一次走完。
 *
 * 使用方式：
 *   Mecanum_MoveWithEncoder(&g_mecanum_config,
 *       0.5f, 0, 0,       // body: 前进 0.5m
 *       1.0f,              // 全速
 *       80,                // acc
 *       10000);            // 超时 10s
 */

/**
 * @brief 单段位置模式：一次走完，驱动器计脉冲精确停止
 *
 * @param config      底盘参数
 * @param body_dx_m   车体前进距离，向前为正，单位：m
 * @param body_dy_m   车体左移距离，向左为正，单位：m
 * @param dtheta_rad  旋转角度，逆时针为正，单位：rad
 * @param speed_ratio 速度比例 (0.0~1.0)
 * @param acc         加速度参数（透传给 Emm_V5_Pos_Control）
 * @param timeout_ms  超时保护，单位：ms
 * @return true       运动完成
 */
bool Mecanum_MoveWithEncoder(const MecanumConfig_t *config,
                             float body_dx_m, float body_dy_m, float dtheta_rad,
                             float speed_ratio, uint8_t acc, uint32_t timeout_ms);

/**
 * @brief World坐标版（World→Body 转换 + Mecanum_MoveWithEncoder）
 */
bool Mecanum_WorldMoveWithEncoder(const MecanumConfig_t *config,
                                  float world_dx_m, float world_dy_m,
                                  float start_theta_rad, float dtheta_rad,
                                  float speed_ratio, uint8_t acc,
                                  uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MECANUM_MOVE_H */
