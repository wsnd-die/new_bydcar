/**
 * @file    block_basic.c
 * @brief   物块基础机构封装：丝杆升降、双机械臂升降、转盘定位。
 * @note    车型 1 使用丝杆机构；车型 2 使用双机械臂机构。
 */
#include "Common_used.h"

#define CLAMP_FLOAT(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define DEG2RAD(d)              ((d) * 0.01745329252f)
#define RAD2DEG(r)              ((r) * 57.2957795131f)

/**
 * @brief   软件记录的转盘角度，单位 deg。
 * @note    1号位置对应 BLOCK_TURNTABLE_HOME_DEG，后续位置每个间隔 BLOCK_TURNTABLE_STEP_DEG，总共 BLOCK_TURNTABLE_POS_COUNT 个位置。
 */
static float angle_servo = BLOCK_TURNTABLE_HOME_DEG;

#if BLOCK_USE_DUAL_ARM
typedef struct {
    float front_angle_deg;
    float rear_angle_deg;
} BlockDualArmPos;

static const BlockDualArmPos block_dual_arm_pos_table[] = {
    {170.0f, 90.0f},   /* pos 1: 初始位置 */
    {72.0f,  79.0f},   /* pos 2: 最低点 */
    {94.0f,  101.0f},  /* pos 3: 第二位置 */
};

#define BLOCK_DUAL_ARM_POS_COUNT \
    ((uint8_t)(sizeof(block_dual_arm_pos_table) / sizeof(block_dual_arm_pos_table[0])))
#endif

/**
 * @brief   物块舵机角度写入,把角度转换成 TIM3 PWM 比较值。
 * @param   channel         TIM3 通道号，CH1/CH2/CH3。
 * @param   angle_deg       目标角度，单位 deg。
 * @param   full_angle_deg  舵机最大角度，180 或 360
 *          TIM_CHANNEL_1 -> 双舵机前级 CH1
 *          TIM_CHANNEL_2 -> 转盘舵机 CH2
 *          TIM_CHANNEL_3 -> 双舵机后级 CH3
 */
static void block_servo_write(uint32_t channel, float angle_deg)
{
    float angle = CLAMP_FLOAT(angle_deg, 0.0f , BLOCK_SERVO_DEG);
    float pulse = 500.0f + (angle / BLOCK_SERVO_DEG) * (2500.0f - 500.0f);

    __HAL_TIM_SET_COMPARE(&htim3, channel, (uint32_t)(pulse + 0.5f));
}

/**
 * @brief   将任意角度归一化到 [0, 360)。
 * @param   angle_deg  输入角度，单位 deg。
 * @return  归一化后的角度，单位 deg。
 */
static float normalize_servo(float angle_deg)
{
    while (angle_deg >= 0.0f) {
        angle_deg -= BLOCK_SERVO_DEG;
    }
    while (angle_deg < 0.0f) {
        angle_deg += BLOCK_SERVO_DEG;
    }
    return angle_deg;
}

/**
 * @brief   物块位置编号转换为转盘角度。
 * @param   block_pos   物块位置编号，合法范围为 1~5。
 * @return  float       转盘角度，单位 deg。
 * @note    1 -> HOME, 2 -> HOME + 72 deg, ..., 5 -> HOME + 288 deg
 */
static float turntable_target_angle(uint8_t block_pos)
{
    return normalize_servo(BLOCK_TURNTABLE_HOME_DEG +
                         (float)(block_pos - BLOCK_TURNTABLE_FIRST_POS) *
                         BLOCK_TURNTABLE_STEP_DEG);
}

/**
 * @brief   转盘角度写入。
 * @param   angle_deg   目标角度，单位 deg。
 * @note    360 度位置舵机，CH2。
 */
static void turntable_write_angle(float angle_deg)
{
    block_servo_write(TIM_CHANNEL_2, normalize_servo(angle_deg));
}

/**
 * @brief   根据编译期选定的车型执行对应升降机构，并统一返回转盘后退距离。
 * @param   dir         升降方向，0=下降，1=上升。
 * @param   pos         双机械臂型为位置表编号；丝杆型为目标升高高度，单位 mm。
 * @return  float       >=0 为转盘相对后退距离，<0 表示参数错误或运动失败。
 */
float BlockBasic_LiftTo(uint8_t dir, float pos)
{

#if BLOCK_USE_DUAL_ARM
    uint8_t arm_pos = (uint8_t)pos;

    /* 双机械臂型：第二个参数作为预设位置表编号使用。 */
    if (pos < 1.0f || pos > (float)BLOCK_DUAL_ARM_POS_COUNT || pos != (float)arm_pos) {
        return -1.0f;
    }

    BlockBasic_DualArmSetPos(arm_pos);
    return 0.0f;
#else
    /* 丝杆型：位置模式, 每次走 pos 距离。lift_current 记账防超限 */
    {
        static float lift_current = 0.0f;  /* 已累计的绝对位置 (mm) */
        float next;

        if (dir == UP)
            next = lift_current + pos;
        else
            next = lift_current - pos;

        /* 超量程: 不执行, 返回 0, 只能往反方向走 */
        if (next < 0.0f || next > BLOCK_LIFT_MAX_MM)
            return 0.0f;

        lift_current = next;
        uint32_t pulse = (uint32_t)(pos * BLOCK_STEPPER_PULSE_PER_MM);
        if (dir == 0)
        {
            Emm_V5_Pos_Control(5, 0, 800, 255, pulse, 0, 0);
        }
        else
        {
            Emm_V5_Pos_Control(5, 1, 800, 255, pulse, 0, 0);
        }
        return 0.0f;
    }
#endif
}



#if BLOCK_USE_DUAL_ARM
/**
 * @brief   计算双舵机角度和转盘退距。
 * @param   height_mm   目标上升高度，单位 mm。
 * @return  BlockArmResult  计算结果。
 */
BlockArmResult BlockBasic_ArmCalc(float height_mm)
{
    /* 采用数学建模公式：S1(x_t) = 90° + arcsin(sin(-12°) + \frac {x_t} {10.5})
                       S2 = S1 - 9.51° */
    float height = CLAMP_FLOAT(height_mm,
                               BLOCK_ARM_MIN_HEIGHT_MM,
                               BLOCK_ARM_MAX_HEIGHT_MM);
    float height_cm = height / 10.0f; // 将高度从 mm 转换为 cm
    float asin_arg = sinf(DEG2RAD(BLOCK_ARM_INIT_DEG)) +
                     height_cm / BLOCK_ARM_LINK_CM;
    float servo1_deg;

    asin_arg = CLAMP_FLOAT(asin_arg, -1.0f, 1.0f);
    servo1_deg = 90.0f + RAD2DEG(asinf(asin_arg));

    BlockArmResult result;
    result.front_angle_deg = servo1_deg;
    result.rear_angle_deg = servo1_deg - BLOCK_ARM_S2_OFFSET_DEG;
    result.turntable_retreat_mm = height * BLOCK_ARM_RETREAT_PER_MM;
    return result;
}

/**
 * @brief   双机械臂预设位置控制。
 * @param   pos  位置编号：1=初始(170°,30°)  2=最低(72°,79°)  3=第二位置(94°,101°)
 * @note    CH1 → 前级舵机（舵机1），CH3 → 后级舵机（舵机2），均为 180° 舵机。
 *          驱动公式：角度/180° * 2000 + 500 us
 */
void BlockBasic_DualArmSetPos(uint8_t pos)
{
    const BlockDualArmPos *target;

    if (pos < 1u || pos > BLOCK_DUAL_ARM_POS_COUNT) {
        return;
    }

    target = &block_dual_arm_pos_table[pos - 1u];

    /* 舵机1 前级 → CH1 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                          (uint32_t)(target->front_angle_deg / 180.0f * 2000.0f + 500.0f + 0.5f));

    /* 舵机2 后级 → CH3 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                          (uint32_t)(target->rear_angle_deg / 180.0f * 2000.0f + 500.0f + 0.5f));
}
#endif


/**
 * @brief  将转盘转动到指定位置。
 * @param  block_pos  目标位置，单位个。
 * @return BlockStatus  执行状态。
 */

BlockStatus BlockBasic_TurntableTo(uint8_t block_pos)
{
    if (block_pos < BLOCK_TURNTABLE_FIRST_POS ||
        block_pos >= BLOCK_TURNTABLE_FIRST_POS + BLOCK_TURNTABLE_POS_COUNT) {
        return BLOCK_ERR_PARAM;
    }

    angle_servo = turntable_target_angle(block_pos);
    turntable_write_angle(angle_servo);
    return BLOCK_OK;
}

MecanumConfig_t Place_config = {
    .wheel_radius_m = 0.0375f,
    .half_length_m = 0.088f,
    .half_width_m = 0.0782f,
    .gear_ratio = 1.0f,
    .pulse_per_rev =3200 ,
    .max_motor_rpm = 140,
    .min_move_time_s = 0.1f,

    /* 驱动器逻辑方向: 0=正向(与 Send_motor 速度环 !dir 后一致)
     * 原 RL/RR 误设 1, 导致 Place 位置模式下后轮反转, 车打转 */
    .forward_dir[MECANUM_ADDR_FR] = 0U,
    .forward_dir[MECANUM_ADDR_RL] = 1U,
    .forward_dir[MECANUM_ADDR_FL] = 0U,
    .forward_dir[MECANUM_ADDR_RR] = 1U,

    /* 驱动器加速度: 脉冲/秒^2 */
    .acceleration = 100U
};
/**
 * @brief  重置软件记录的转盘当前角度，并立即输出该角度 PWM。
 * @param  angle_deg  当前机械角度，单位 deg；会归一化到 0~360。
 */
void Servo_Angle(float angle_deg)
{
    angle_servo = normalize_servo(angle_deg);
    turntable_write_angle(angle_servo);
}

void Servo_SetAngle(float Angle)
{
    if(Angle>=125){Angle=125;}
    if(Angle<=37){Angle=37;}
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Angle / 180 * 2000 + 500);

}
/* K230 圆心像素 → 车体横向位移 (m/像素)。比例/方向需实测调, 反了取负 */
#define PLACE_CIRCLE_SCALE_M  0.001f

void Place(char dir,float x,float y,uint16_t height)
{
    MecanumMove_t move;
    if (dir == 'O')
    {
        /* 圆心 xy → 放置补量 (保留) */
        float fwd  = 0.068f - y * PLACE_CIRCLE_SCALE_M;
        float left = -x * PLACE_CIRCLE_SCALE_M;
        if (Mecanum_CalculateMove(&Place_config, fwd, left, 0.0f, &move))
        {
            Mecanum_ExecuteMove(&Place_config, &move);
            osDelay((uint32_t)(move.duration_s * 2000.0f) + 50U);
        }
        BlockBasic_LiftTo(DOWN,height);
        osDelay(950);

        /* 后退 0.05 m（车体坐标：-X 为后退） */
        if (Mecanum_CalculateMove(&Place_config, -0.145f, 0.0f, 0.0f, &move))
        {
            Mecanum_ExecuteMove(&Place_config, &move);
            osDelay((uint32_t)(move.duration_s * 2000.0f) + 50U);
        }
    }
}
