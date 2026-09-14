#include "Common_used.h"

/* ============================================================
 * 全局变量
 * ============================================================ */

/* 当前自身位姿（世界坐标系），上电默认原点 */
World_Dir_t Self_Dir = {0.0f, 0.0f, 0.0f};

/* 循迹完成后置 1: 下一次 FeDuan 用速度模式纠正到点 */
volatile uint8_t g_nav_speed_mode = 0;

/*
 * 路径点数组（世界坐标系）
 *
 * 字段: { x(m), y(m), yaw(rad) }
 *
 * 角度可使用 MECANUM_DEG_TO_RAD 辅助:
 *   N 度 → N * MECANUM_DEG_TO_RAD
 *
 * 修改此数组内容和你需要的目标点，
 * 同时更新 g_waypoint_count 为实际点数。
 */
World_Dir_t g_waypoints[NAV_WAYPOINT_MAX] = {

    /* ---- 示例路径（可根据实际修改）---- */


    {0.428f,    0.726f,  0.0f  * MECANUM_DEG_TO_RAD },//奖杯二维码点

    {-0.16f,      0.15f,      -90.0f * MECANUM_DEG_TO_RAD },//物料循线点

    {    -0.836f,     -0.466f,   0.0f  * MECANUM_DEG_TO_RAD },  /*a点*/
    {    -0.235f,     -0.21f,   0.0f  * MECANUM_DEG_TO_RAD },/*b点*/
    {    0.358f,     -0.60f,   0.0f  * MECANUM_DEG_TO_RAD },  /*c点 */
    {    -0.2f,     -0.10f,  0.0f  * MECANUM_DEG_TO_RAD },  /*d点*/
    {    0.21f,     -0.5f,   0.0f  * MECANUM_DEG_TO_RAD },  /* e点 */

    {  -0.163f,    0.191f, 0.0f * MECANUM_DEG_TO_RAD },  /* 奖杯二维码点*/

    {   -0.1f,    -0.24f, 90.0f * MECANUM_DEG_TO_RAD },  /* 奖杯寻线点 */

    {    0.405f,     1.03f,  0.0f * MECANUM_DEG_TO_RAD },  /* 亚军点*/
    {    0.06f,    0.27f,  0.0f * MECANUM_DEG_TO_RAD },  /* 冠军点 */
    {    0.06f,    0.27f,   0.0f * MECANUM_DEG_TO_RAD },  /* 季军点 */

    {-0.483f,-0.0,0},//回家点
    {0,-0.244f,0},
    {-1.078f,-0.18,0},

};

/* 实际使用的路径点数量 */
uint8_t  g_waypoint_count = 13;


/* ============================================================
 * 循迹后点位校准
 *
 * 循迹完成时车停的位置 (World_position_get) 与设计坐标有偏差,
 * 用"实测终点 + 固定偏移"重算 a 点 / 亚军点:
 *
 *   目标点 = 实测循迹终点 + (目标点设计值 - 循迹终点设计值)
 *
 * 偏移量可现场调整, 改下面宏即可 (单位: m)。
 * ============================================================ */

/* ---- 目标点设计坐标 (世界系, m) ---- */
#define CALIB_A_X        (-0.844f)     /* a点设计值 */
#define CALIB_A_Y        (-0.466f)
#define CALIB_YAJUN_X    ( 0.48f)     /* 亚军点设计值 */
#define CALIB_YAJUN_Y    ( 1.065f)

/* ---- 循迹终点设计坐标 (世界系, m) ---- */
#define TRACE_END_A_X    ( 2.186f/2)       /* 物料循迹(LinFolL)终点设计值 */
#define TRACE_END_A_Y    ( 0.337f/2)
#define TRACE_END_YAJUN_X (1.126f/2)      /* 奖杯循迹(LinFolR)终点设计值 */
#define TRACE_END_YAJUN_Y (0.985f/2)

/* ---- 固定偏移 = 目标点设计值 - 循迹终点设计值 ---- */
#define CALIB_A_OFF_X    (CALIB_A_X - TRACE_END_A_X)
#define CALIB_A_OFF_Y    (CALIB_A_Y - TRACE_END_A_Y)
#define CALIB_YAJUN_OFF_X (CALIB_YAJUN_X - TRACE_END_YAJUN_X)
#define CALIB_YAJUN_OFF_Y (CALIB_YAJUN_Y - TRACE_END_YAJUN_Y)


/* ============================================================
 * 内部辅助函数 / 配置
 * ============================================================ */

/**
 * @brief 将角度归一化到 [-PI, PI]
 */
static float Nav_NormalizeAngle(float angle)
{
    while (angle > MECANUM_PI) {
        angle -= 2.0f * MECANUM_PI;
    }
    while (angle < -MECANUM_PI) {
        angle += 2.0f * MECANUM_PI;
    }
    return angle;
}

/* ============================================================
 * 公开函数
 * ============================================================ */

bool Nav_GoToWorld(float target_x, float target_y, float target_yaw)
{
    float world_dx, world_dy, dtheta, dist;
    uint32_t timeout_ms;
    bool success;

    /* ---- 1. 计算世界坐标系下的差值 ---- */
    world_dx = target_x - Self_Dir.x;
    world_dy = target_y - Self_Dir.y;

    /* 角度差取最短路径 */
    dtheta = Nav_NormalizeAngle(target_yaw - Self_Dir.yaw);

    /* ---- 2. 到位判断 ---- */
    dist = sqrtf(world_dx * world_dx + world_dy * world_dy);
    if (dist < 0.005f && fabsf(dtheta) < 0.01f) {
        return true;
    }

    /* ---- 3. 计算超时（预估耗时 × 3，保底 5s）---- */
    {
        float est_s = dist / 0.05f;  /* 0.05 m/s 保守估计最慢速度 */
        if (est_s < 5.0f) est_s = 5.0f;
        timeout_ms = (uint32_t)(est_s * 1000.0f);  /* ×2 安全余量 */
    }

    /* ---- 4. 速度模式 + 编码器反馈，单段走完 ---- */
    success = Mecanum_WorldMoveWithEncoder(
        &g_mecanum_config,
        world_dx, world_dy,
        Self_Dir.yaw,
        dtheta,
        1.0f,          /* 全速 */
        g_mecanum_config.acceleration,           /* acc */
        timeout_ms
    );

    if (!success) {
        return false;
    }

    /* ---- 5. 更新自身位姿 ---- */
    Self_Dir.x   = target_x;
    Self_Dir.y   = target_y;
    Self_Dir.yaw = Nav_NormalizeAngle(Self_Dir.yaw + dtheta);

    return true;
}
/*==============================================
 * 车体坐标运动 (Body-frame)
 *   平移: 麦轮解算 (世界坐标, 不含旋转)
 *   旋转: 交由 FC_TASK 角度环 (串级PID) 执行
 * ============================================================ */


/**
 * @brief 车体坐标运动
 *        forward_m/left_m: 车体坐标平移量 (m), 麦轮单独执行
 *        rotate_rad:       平移到位后用 AngleCtrl 旋转 (rad, CCW+)
 */
bool Nav_MoveBody(float forward_m, float left_m, float rotate_rad)
{
    float dist;
    bool  success;

    dist = sqrtf(forward_m * forward_m + left_m * left_m);
    if (dist < 0.005f && fabsf(rotate_rad) < 0.01f) {
        return true;
    }

    {
        float target_deg = rotate_rad * (180.0f / M_PI);
        float err;
        uint16_t guard = 0;

        g_angle_target_yaw = target_deg;
        g_angle_ctrl_enable = 1;              /* 打开 FC_TASK 角度环 */

        while (guard++ < 150) {               /* 超时约8s, 防角度环异常卡死 */
            err = target_deg - g_hwt_imu_yaw;
            while (err >  180.0f) err -= 360.0f;
            while (err < -180.0f) err += 360.0f;
            if (fabsf(err) <= 2.0f) break;
            osDelay(10);
        }
        g_angle_ctrl_enable = 0;              /* 转完关闭, 避免与后面平移抢电机 */
        osDelay(20);                          /* 等 FC_TASK 停止输出 */
        Self_Dir.yaw = g_hwt_imu_yaw_rad;     /* 用实测航向, 不要硬设目标 */
    }
    /* ---- 3. 平移段: 车体坐标 → 麦轮 (dtheta=0, 不旋转) ---- */
    if (dist >= 0.005f) {
        float est_s = dist / 0.05f;
        if (est_s < 3.0f) est_s = 3.0f;
        uint32_t timeout_ms = (uint32_t)(est_s * 1000.0f);

        success = Mecanum_MoveWithEncoder(
            &g_mecanum_config,
            forward_m, left_m, 0.0f,  /* dtheta=0, 纯平移 */
            1.0f, g_mecanum_config.acceleration, timeout_ms
        );
        if (!success) return false;
        float cos_y = cosf(Self_Dir.yaw), sin_y = sinf(Self_Dir.yaw);
        Self_Dir.x += forward_m * cos_y - left_m * sin_y;
        Self_Dir.y += forward_m * sin_y + left_m * cos_y;
    }

    /* ---- 第二段角度矫正: 只用实测漂移, 超阈值才修, 到位收紧, 结束写实测航向 ---- */
    {
        float target_deg = rotate_rad * (180.0f / M_PI);
        float drift;
        uint16_t guard = 0;

        drift = target_deg - g_hwt_imu_yaw;
        while (drift >  180.0f) drift -= 360.0f;
        while (drift < -180.0f) drift += 360.0f;

        if (fabsf(drift) > 3.5f) {            /* 真实漂移超阈值才修, 避免空转 */
            float err;

            g_angle_target_yaw = target_deg;
            g_angle_ctrl_enable = 1;          /* 打开 FC_TASK 角度环 */

            while (guard++ < 100) {           /* 超时1s, 防角度环异常卡死 */
                err = target_deg - g_hwt_imu_yaw;
                while (err >  180.0f) err -= 360.0f;
                while (err < -180.0f) err += 360.0f;
                if (fabsf(err) <= 3.0f) break;   /* 与第一段一致; FC_TASK 容差已收紧到1°, 实际能修到更小 */
                osDelay(10);
            }

            g_angle_ctrl_enable = 0;          /* 转完关闭, 避免与后面平移抢电机 */
            osDelay(20);                      /* 等 FC_TASK 停止输出 */
        }
        Self_Dir.yaw = g_hwt_imu_yaw_rad;     /* 用实测航向, 不要硬设目标 */
    }


    return true;
}

/** @brief 前进/后退 (车体坐标) */
bool Nav_MoveForward(float distance_m)
{
    return Nav_MoveBody(distance_m, 0.0f, 0.0f);
}

/** @brief 左移/右移 (车体坐标) */
bool Nav_MoveLeft(float distance_m)
{
    return Nav_MoveBody(0.0f, distance_m, 0.0f);
}

/** @brief 原地旋转 */
bool Nav_Rotate(float angle_rad)
{
    return Nav_MoveBody(0.0f, 0.0f, angle_rad);
}

void Nav_CalibrateAfterTrace(bool is_trophy)
{
    World_Dir_t p = World_position_get();

    if (is_trophy) {
        g_waypoints[9].x = p.x/2 + CALIB_YAJUN_OFF_X;
        g_waypoints[9].y = p.y/2 + CALIB_YAJUN_OFF_Y;
        printf("[CAL] 亚军点 <- (%.3f,%.3f) 终点=(%.3f,%.3f)\r\n",
               (double)g_waypoints[9].x, (double)g_waypoints[9].y,
               (double)p.x, (double)p.y);
    } else {
        g_waypoints[2].x = p.x/2 + CALIB_A_OFF_X;
        g_waypoints[2].y = p.y/2 + CALIB_A_OFF_Y;
        printf("[CAL] a点 <- (%.3f,%.3f) 终点=(%.3f,%.3f)\r\n",
               (double)g_waypoints[2].x, (double)g_waypoints[2].y,
               (double)p.x, (double)p.y);
    }
}


/* ============================================================
 * 速度模式到位控制 — 编码器+陀螺仪定位
 * ============================================================ */
#define NAV_KP_POS   1.2f    /* 位置P: m/s 每 m 误差 */
#define NAV_KP_YAW   2.5f    /* 航向P: rad/s 每 rad 误差 */
#define NAV_V_MAX    0.5f    /* 最大线速度 m/s */
#define NAV_W_MAX    1.8f    /* 最大角速度 rad/s */
#define NAV_POS_TOL  0.015f  /* 到位距离 m */
#define NAV_YAW_TOL  0.04f   /* 到位航向 rad */
#define NAV_TIMEOUT  500U    /* 超时 ×10ms ≈5s */

/**
 * @brief 速度模式到位控制(世界系目标)
 *        用编码器+陀螺仪里程计 World_position_get() 做反馈,
 *        位置误差→vx/vy, 航向误差→w, Mecanum_Calc_Full 输出速度。
 * @param tx,ty,tyaw 目标世界坐标(m)/航向(rad)
 */
bool Nav_TrackPose(float tx, float ty, float tyaw)
{
    uint16_t guard = 0;
    World_Dir_t pos;
    MecanumResult motor;

    g_angle_ctrl_enable = 0;   /* 不用 FC_TASK 角度环, 防止抢电机 */

    while (guard++ < NAV_TIMEOUT) {
        pos = World_position_get();          /* 编码器+陀螺仪里程计 */

        float dx = tx - pos.x;
        float dy = ty - pos.y;
        float dyaw = Nav_NormalizeAngle(tyaw - pos.yaw);
        float dist = sqrtf(dx * dx + dy * dy);

        /* 到位: 停车 + 更新自身位姿 */
        if (dist < NAV_POS_TOL && fabsf(dyaw) < NAV_YAW_TOL) {
            motor = Mecanum_Calc_Full(0.0f, 0.0f, 0.0f);
            Send_commandmotor(&motor);
            Self_Dir = pos;
            return true;
        }

        /* 位置 P 控制: 世界速度 → 限幅 → 旋转到车体系 */
        float vx_w = NAV_KP_POS * dx;
        float vy_w = NAV_KP_POS * dy;
        float v_w_mag = sqrtf(vx_w * vx_w + vy_w * vy_w);
        if (v_w_mag > NAV_V_MAX) {
            vx_w *= NAV_V_MAX / v_w_mag;
            vy_w *= NAV_V_MAX / v_w_mag;
        }
        float v_bx =  vx_w * cosf(pos.yaw) + vy_w * sinf(pos.yaw);  /* 车体前进 */
        float v_by = -vx_w * sinf(pos.yaw) + vy_w * cosf(pos.yaw);  /* 车体左移 */

        /* 航向 P 控制 */
        float w = NAV_KP_YAW * dyaw;
        if (w >  NAV_W_MAX) w =  NAV_W_MAX;
        if (w < -NAV_W_MAX) w = -NAV_W_MAX;

        motor = Mecanum_Calc_Full(v_bx, v_by, w);
        Send_commandmotor(&motor);
        osDelay(10);
    }

    /* 超时: 停车 */
    motor = Mecanum_Calc_Full(0.0f, 0.0f, 0.0f);
    Send_commandmotor(&motor);
    Self_Dir = World_position_get();
    return false;
}

bool Nav_FeDuanPoint() {
    static uint8_t PontIntex=0;



    /* 临界区 */


    /* 纯位置模式: 固定用 Mecanum_MoveWithEncoder, 不再走速度模式(Nav_TrackPose) */
    {
        uint8_t idx = PontIntex++;   /* 先取再推进, 避免同一表达式内多次读写(UB) */
        if (!Nav_MoveBody(g_waypoints[idx].x,
                          g_waypoints[idx].y,
                          g_waypoints[idx].yaw)) {
            return false;
        }
    }
    if (PontIntex==13)
    {
        Nav_MoveBody(g_waypoints[PontIntex].x,
                          g_waypoints[PontIntex].y,
                          g_waypoints[PontIntex].yaw);
        PontIntex++;

        Nav_MoveBody(g_waypoints[PontIntex].x,
                          g_waypoints[PontIntex].y,
                          g_waypoints[PontIntex].yaw);
        PontIntex++;
    }

    return true;
}


/**
 * @brief 依次执行所有路径点
 */
bool Nav_RunWaypoints(void)
{
    for (uint8_t i = 0; i < g_waypoint_count; i++) {
        if (!Nav_MoveBody(g_waypoints[i].x,
                           g_waypoints[i].y,
                           g_waypoints[i].yaw)) {
            return false;
        }
    }

    return true;
}


/* ============================================================
 * 测试函数（保留）
 * ============================================================ */

void Chassis_WorldMoveTest(void)
{
    MecanumMove_t move;
    bool success;

    success = Mecanum_CalculateWorldMove(
        &g_mecanum_config,

        0.00f,                         /* 世界X：前进50 cm */
       0.0f,                          /* 世界Y：向右50 cm */
        0.0f * MECANUM_DEG_TO_RAD,    /* 开始航向0° */
        90.0f * MECANUM_DEG_TO_RAD,   /* 逆时针旋转50° */

        &move
    );

    if (success) {
        Mecanum_ExecuteMove(
            &g_mecanum_config,
            &move
        );
    }
}













