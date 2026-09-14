/*
 * 从 algorithm/Nav_position.c 摘出来的 INS (惯导) 部分 —— 仅作参考, 不参与编译。
 *
 * 为什么删掉: 这套东西靠 imu660ra 的**原始加速度**做二重积分推位置, 而换上的
 * HWT906 只输出欧拉角, 没有原始加速度, 所以它没有数据源了。同时它依赖
 * siyuan_imu 的 Mahony 四元数 (siyuan_get_quat), 那套滤波也已一并移除。
 *
 * 如果以后换了带原始 IMU 数据的传感器想把它接回来:
 *   1. 把下面的宏 / Ins_t / g_ins / Ins_Init / Ins_Update 挪回 Nav_position.c
 *      (以及 Nav_position.h 里对应的声明);
 *   2. 把第 120~136 行的读 IMU 部分换成新传感器的接口;
 *   3. 在任务里周期性调用 Ins_Update()。
 *
 * ---------------------------------------------------------------------------
 * 以下为原文件 algorithm/Nav_position.c 第 71~176 行的原样内容:
 * ---------------------------------------------------------------------------
 */

/* ============================================================
 * 惯导 (INS) — 纯 IMU 加速度积分推位
 *
 * 流程: 车体加速度(IMU, g) → Mahony 四元数旋转到世界系
 *       → 扣重力 → 扣零偏 → 积分得速度 → 再积分得位置
 *
 * 漂移控制(纯 IMU 无外部参考):
 *   - 静止判定: 加速度模长≈1g 且陀螺≈0 且持续 N 帧
 *   - 静止时在线学习水平零偏 (bias_x/y) 并泄放速度
 *   - 因此位置会随运行时间缓慢漂移, 长时间定位建议
 *     周期性用编码器里程计 World_position_get() 重置 g_ins
 *
 * 注意: 复用 siyuan_imu 已更新的 IMU 全局量, 不要再做 SPI 读,
 *       请在 siyuan_imu_task 之后(同一任务或紧随其后)调用。
 * ============================================================ */
#define INS_G             9.80665f  /* 重力加速度 m/s² */
#define INS_ZUPT_ACC_MIN  0.90f     /* 静止判定: 加速度模长下限 (g) */
#define INS_ZUPT_ACC_MAX  1.10f     /* 静止判定: 加速度模长上限 (g) */
#define INS_ZUPT_GYRO     0.20f     /* 静止判定: 陀螺模长上限 (rad/s) */
#define INS_ZUPT_N        30U       /* 连续判定帧数 (200Hz×30≈150ms) */
#define INS_BIAS_LPF      0.02f     /* 零偏学习低通系数 */
#define INS_LEAK_RATE     3.0f      /* 静止时速度泄放速率 (1/s) */

Ins_t g_ins = {0};

void Ins_Init(void)
{
    memset(&g_ins, 0, sizeof(g_ins));
    g_ins.inited    = 1;
    g_ins.last_tick = HAL_GetTick();
}

void Ins_Update(void)
{
    static uint8_t quiet_cnt = 0;
    float q[4];
    float ab[3], aw[3];
    float acc_norm, gyro_norm;
    float dt;
    uint32_t now = HAL_GetTick();

    if (!g_ins.inited) return;

    /* ---- 1. dt ---- */
    dt = (float)(now - g_ins.last_tick) / 1000.0f;
    if (dt <= 0.0f)  dt = 0.005f;
    if (dt >  0.05f) dt = 0.05f;
    g_ins.last_tick = now;

    /* ---- 2. 读取 IMU (复用 siyuan_imu 已更新的全局量, 勿重复 SPI 读) ---- */
    /* 车体系加速度, 轴映射与 siyuan_imu 一致, 单位 g: 前/左/上 */
    ab[0] =  imu660ra_acc_transition(imu660ra_acc_z);
    ab[1] =  imu660ra_acc_transition(imu660ra_acc_y);
    ab[2] = -imu660ra_acc_transition(imu660ra_acc_x);

    /* 陀螺模长 (rad/s) — 仅用于静止判定 */
    {
        float gx = imu660ra_gyro_transition(imu660ra_gyro_z);
        float gy = imu660ra_gyro_transition(imu660ra_gyro_y);
        float gz = -imu660ra_gyro_transition(imu660ra_gyro_x);
        gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz) * DEG_TO_RAD;
    }

    /* ---- 3. 车体→世界 旋转 (Mahony 四元数, 与 World_position_get 的
     *         yaw 旋转约定一致: 世界 X 前, Y 左, Z 上) ---- */
    siyuan_get_quat(q);
    {
        float w=q[0], x=q[1], y=q[2], z=q[3];
        aw[1] = (1.0f-2.0f*(y*y+z*z))*ab[0] + 2.0f*(x*y-w*z)*ab[1] + 2.0f*(x*z+w*y)*ab[2];
        aw[0] = 2.0f*(x*y+w*z)*ab[0] + (1.0f-2.0f*(x*x+z*z))*ab[1] + 2.0f*(y*z-w*x)*ab[2];
        aw[2] = 2.0f*(x*z-w*y)*ab[0] + 2.0f*(y*z+w*x)*ab[1] + (1.0f-2.0f*(x*x+y*y))*ab[2];
    }
    /* g → m/s², 并扣除重力 (世界系 Z 向上, 静止时测得 +1g) */
    aw[1] *= INS_G;
    aw[0] *= INS_G;
    aw[2]  = aw[2] * INS_G - INS_G;

    /* ---- 4. 静止判定: 加速度模长≈1g 且陀螺≈0 且持续 N 帧 ---- */
    acc_norm = sqrtf(ab[0]*ab[0] + ab[1]*ab[1] + ab[2]*ab[2]);
    if (acc_norm > INS_ZUPT_ACC_MIN && acc_norm < INS_ZUPT_ACC_MAX &&
        gyro_norm < INS_ZUPT_GYRO) {
        if (quiet_cnt < INS_ZUPT_N) quiet_cnt++;
    } else {
        quiet_cnt = 0;
    }
    g_ins.stationary = (quiet_cnt >= INS_ZUPT_N);

    /* ---- 5. 静止时: 在线学习水平零偏 + 泄放速度 ---- */
    if (g_ins.stationary) {
        g_ins.bias_x += INS_BIAS_LPF * (aw[1] - g_ins.bias_x);
        g_ins.bias_y += INS_BIAS_LPF * (aw[0] - g_ins.bias_y);
        float k = 1.0f - INS_LEAK_RATE * dt;
        if (k < 0.0f) k = 0.0f;
        g_ins.vx *= k;
        g_ins.vy *= k;
    }

    /* ---- 6. 扣零偏 + 积分: a→v→p (只跟踪水平面 x/y) ---- */
    aw[1] -= g_ins.bias_x;
    aw[0] -= g_ins.bias_y;

    g_ins.vx -= aw[1] * dt;
    g_ins.vy += aw[0] * dt;
    g_ins.x  -= g_ins.vx * dt;
    g_ins.y  += g_ins.vy * dt;
}

/* ---------------------------------------------------------------------------
 * 以及 Nav_position.h 中对应的声明 (也已移除):
 *
 *   typedef struct {
 *       float x, y;            // 世界位置 (m)
 *       float vx, vy;          // 世界速度 (m/s)
 *       uint8_t inited;
 *       uint8_t stationary;    // 最近是否判定为静止 (ZUPT)
 *       uint32_t last_tick;
 *       float bias_x, bias_y;  // 水平加速度零偏估计 (m/s², 静止时在线学习)
 *   } Ins_t;
 *
 *   extern Ins_t g_ins;
 *   void Ins_Init(void);
 *   void Ins_Update(void);
 * ---------------------------------------------------------------------------
 */