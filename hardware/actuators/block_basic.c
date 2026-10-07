/**
 * @file    block_basic.c
 * @brief   物块基础机构封装：丝杆升降、双机械臂升降、转盘定位。
 * @note    车型 1 使用丝杆机构；车型 2 使用双机械臂机构。
 */
#include "Common_used.h"
#include "block_basic.h"
#include "emm_5v.h"
#include "mecanum.h"        /* Mecanum_MoveBodyPos —— Place() 的定距移动 */
#include "key.h"
#include "servo_scs.h"      /* 转盘 STS3032 总线舵机: SCS_WritePosEx (V1.16.0) */
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
 * @brief   转盘角度写入 —— UART5 总线上的 STS3032 绝对位置舵机。
 * @param   angle_deg   目标角度，单位 deg；先归一化到 [0, 360)。
 *
 * @note    V1.16.0 起转盘不再走 TIM3_CH2 的 PWM：原 `block_servo_write()`
 *          已删除（本函数是它在工程里唯一的调用者），改用 SCS_WritePosEx()。
 *          角度 → 位置：pos = 角度 / 360 × 4095，即 12 位单圈绝对值
 *          0~4095 ↔ 0~360°。HOME = 0.1° → pos ≈ 1。
 *
 * @warning **量程端点 0 与 4095 在物理上是同一个点。** HOME 落在 pos≈1，
 *          正好贴在跳变点上：手推几度就可能让读数跨端跳变，位置环会把误差
 *          算成「差一整圈」，于是顺着推的方向转满一圈才回来。这是
 *          Core/Src/app_freertos.c 里记录过的同一个坑（SDK 的 STS_CENTER
 *          注释）。本次按要求**未加相位偏移**；若实测出现整圈反转，
 *          加一个 180° 的相位偏移把 HOME 挪到量程中段即可。
 *
 * @note    本函数**阻塞**在总线收发上（厂商协议发一帧收一帧，见
 *          servo_scs.h）。舵机不在线时每字节最多等 SCS_UART_RX_TIMEOUT_MS，
 *          单条命令的总代价可能到百毫秒级 —— 只应在低频命令任务里调用。
 *          调用任务需要 ≥1KB 栈余量（servo_scs.h 的栈要求说明）。
 * @note    总线未初始化（SCS_BusInit() 未跑）时 SCS_WritePosEx() 仍会发帧，
 *          只是没有互斥保护，不会静默丢弃。
 */
static void turntable_write_angle(float angle_deg)
{
    float    angle = normalize_servo(angle_deg);
    uint16_t pos   = (uint16_t)(angle / BLOCK_SERVO_DEG *
                                (float)BLOCK_TURNTABLE_SERVO_POS_MAX + 0.5f);

    if (pos > (uint16_t)BLOCK_TURNTABLE_SERVO_POS_MAX) {
        pos = (uint16_t)BLOCK_TURNTABLE_SERVO_POS_MAX;
    }

    (void)SCS_WritePosEx((uint8_t)BLOCK_TURNTABLE_SERVO_ID,
                         (int16_t)pos,
                         BLOCK_TURNTABLE_SERVO_SPEED,
                         BLOCK_TURNTABLE_SERVO_ACC);
}

/* ================================================================
 * 夹爪 (SCS0009, ID 2~6) —— 槽位 1~5 与 ID 2~6 一一对应
 *
 * 用 SCS_WritePos() 而不是 SCS_WritePosEx(): 前者是 SCSCL 系列(SCS0009)
 * 的写位置接口, 会在锁内把总线字节序切成大端; 后者是 SMS_STS 系列(转盘
 * ID 1)用的。两个系列的 API 不能互换 —— 见 servo_scs.h 的说明。
 * ================================================================ */

/* 槽位 ↔ 舵机 ID 的映射是假设, 与 servo_scs.h 的 ID 范围对不上就编译不过 */
_Static_assert(BLOCK_GRIPPER_SERVO_ID(BLOCK_TURNTABLE_FIRST_POS) == SERVO_ID_SCS0009_MIN,
               "夹爪 ID 起点与 SERVO_ID_SCS0009_MIN 不符");
_Static_assert(BLOCK_GRIPPER_SERVO_ID(BLOCK_TURNTABLE_POS_COUNT) == SERVO_ID_SCS0009_MAX,
               "夹爪 ID 终点与 SERVO_ID_SCS0009_MAX 不符");

static bool gripper_slot_bad(uint8_t slot)
{
    return (slot < BLOCK_TURNTABLE_FIRST_POS) || (slot > BLOCK_TURNTABLE_POS_COUNT);
}

BlockStatus BlockBasic_GripperClamp(uint8_t slot)
{
    if (gripper_slot_bad(slot)) {
        return BLOCK_ERR_PARAM;
    }
    (void)SCS_WritePos(BLOCK_GRIPPER_SERVO_ID(slot), SCS_CLOSE, SCS_TIME, SCS_SPEED);
    return BLOCK_OK;
}

BlockStatus BlockBasic_GripperRelease(uint8_t slot)
{
    if (gripper_slot_bad(slot)) {
        return BLOCK_ERR_PARAM;
    }
    (void)SCS_WritePos(BLOCK_GRIPPER_SERVO_ID(slot), SCS_OPEN, SCS_TIME, SCS_SPEED);
    return BLOCK_OK;
}

int BlockBasic_GripperRaw(uint8_t slot)
{
    if (gripper_slot_bad(slot)) {
        return -1;
    }
    return Scs0009_ReadRaw(BLOCK_GRIPPER_SERVO_ID(slot));
}

/* 逐槽的形状判定阈值 —— 夹紧后回读的 SCS0009 原始位置 (量程 0~1024)。
 * 下标 = 物理槽号 - SLOT_SHAPE_FIRST, 即 [0]=槽2 [1]=槽3 [2]=槽4 [3]=槽5。
 *
 * ★ 必须是**夹紧状态下**实测的两簇分布中点: 舵机顶着物块停住, raw 是「夹到该
 *   物块的位置」, 不是自由行程位置。标定就靠 app/block_collect.c 每槽打印的
 *   那行 `raw=` 日志 —— 两种物块各夹 20 次, 取两类不重叠区间的中点。
 * ★ 待迁: config/param_config.h 落地后搬过去 (CLAUDE.md §7.3)。
 *
 * @note 定义放 .c 不放 .h —— 带初始化器的文件作用域数组是**定义**,
 *       写在头文件里会被每个包含它的 .c 各生成一份, 链接期 multiple definition。 */
#define SLOT_SHAPE_FIRST   2u          /* 表覆盖的物理槽起点 (>0 是因为槽 1 是黄锥, 判它没意义) */
#define SLOT_SHAPE_COUNT   4u
static const int Slot_Shape[SLOT_SHAPE_COUNT] = { 640, 641, 637, 641 };

_Static_assert(SLOT_SHAPE_FIRST + SLOT_SHAPE_COUNT - 1u <= BLOCK_TURNTABLE_POS_COUNT,
               "形状阈值表越过了转盘槽位上限");

BlockShape_t BlockBasic_ShapeFromRaw(int raw_angle, uint8_t slot)
{
    if (raw_angle < 0 ||
        slot < SLOT_SHAPE_FIRST ||
        slot >= SLOT_SHAPE_FIRST + SLOT_SHAPE_COUNT) {
        return SHAPE_UNKNOWN;   /* 读失败 / 槽号不在表内, 都不能猜 */
    }
    return (raw_angle > Slot_Shape[slot - SLOT_SHAPE_FIRST])
           ? SHAPE_RECT : SHAPE_CYLINDER;
}

/* ================================================================
 * 丝杆升降
 *
 * `lift_current` = **软件记录的丝杆绝对高度** (mm)。绝对版与旧的相对版
 * **共用这一份** —— 两个 API 混用时高度才不会互相错位。
 * 上电/归零后必须与机械实际位置一致: BPlace_SetZero() 压到限位归零之后
 * 调 BlockBasic_LiftSync(0.0f)。
 * ================================================================ */
static float lift_current = 0.0f;

void BlockBasic_LiftSync(float cur_mm)
{
    lift_current = CLAMP_FLOAT(cur_mm, 0.0f, BLOCK_LIFT_MAX_MM);
}

/**
 * @brief   丝杆升降 —— **绝对位置**(指定点位)指令。
 * @param   target_mm  目标高度, 单位 mm, 有效范围 [0, BLOCK_LIFT_MAX_MM]。
 * @retval  true   已下发 (方向与脉冲由内部按当前位置算)
 * @retval  false  越界未执行 / 双机械臂车型不支持
 *
 * @note    调用方只管说"升到多少 mm", 不必记当前高度、也不用心算相对量。
 *          旧的相对版 `BlockBasic_LiftTo(dir, delta)` 是**顺序相关**的 ——
 *          中间某一拍被跳过(例如 PlaceDown 走了"找不到名次, 跳过"分支),
 *          后面所有高度就整体错位; 新代码请用本函数。
 * @note    已经停在该高度上时直接返回 true, 不发 CAN。
 */
bool BlockBasic_LiftToAbs(float target_mm)
{
#if BLOCK_USE_DUAL_ARM
    (void)target_mm;
    return false;                   /* 双机械臂型按预设位置表动作, 没有"绝对高度" */
#else
    float    delta;
    uint32_t pulse;

    if (target_mm < 0.0f || target_mm > BLOCK_LIFT_MAX_MM) {
        return false;               /* 越界: 不动, 让调用方知道 */
    }

    delta = target_mm - lift_current;
    if (delta == 0.0f) {
        return true;                /* 已经在那儿了 */
    }

    lift_current = target_mm;

    pulse = (uint32_t)(((delta > 0.0f) ? delta : -delta) *
                       BLOCK_STEPPER_PULSE_PER_MM);

    /* 方向编码沿用旧 API: 上行 = 0, 下行 = 1 (见下面 BlockBasic_LiftTo 的实际行为,
     * 注意它上面的注释里"0=下降"是**写反了**的 —— 枚举里 UP = 0) */
    Emm_V5_Pos_Control(5, (delta > 0.0f) ? 0u : 1u, 1600, 0, pulse, 0, 0);
    return true;
#endif
}

/**
 * @brief   根据编译期选定的车型执行对应升降机构，并统一返回转盘后退距离。
 * @param   dir         升降方向: **0 = UP(上升), 1 = DOWN(下降)**。
 * @param   pos         双机械臂型为位置表编号；丝杆型为相对移动量，单位 mm。
 * @return  float       >=0 为转盘相对后退距离，<0 表示参数错误或运动失败。
 *
 * @note    **相对量, 顺序相关** —— 见上面 LiftToAbs 的说明。新代码用绝对版。
 * @warning 超量程时**静默返回 0 且不动作**(调用方无从区分), 需要知道成败请用
 *          `BlockBasic_LiftToAbs()`, 它返回 bool。
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
    {
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
            Emm_V5_Pos_Control(5, 0, 3000, 0, pulse, 0, 0);
        }
        else
        {
            Emm_V5_Pos_Control(5, 1, 3000, 0, pulse, 0, 0);
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

/**
 * @brief  重置软件记录的转盘当前角度，并立即输出该角度 PWM。
 * @param  angle_deg  当前机械角度，单位 deg；会归一化到 0~360。
 */
void Servo_Angle(float angle_deg)
{
    angle_servo = normalize_servo(angle_deg);
    turntable_write_angle(angle_servo);
}

static float Now_Angle=127;
#define ANGLE_STEP     30.0f
#define CostTime      1.0f

static void  Servo_AngleAcc(float angle_deg) {
    float error,step;
    error=angle_deg-Now_Angle;
    step=error/ANGLE_STEP;

    for (uint8_t i=0;i<ANGLE_STEP;i++) {
        Now_Angle+=step;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,Now_Angle/ 180 * 2000 + 500);
        osDelay(CostTime/ANGLE_STEP*1000);
    }


}
void Servo_SetAngle(float Angle)
{

    if(Angle>=130){Angle=130;}
    if(Angle<=37){Angle=37;}

    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Angle / 180 * 2000 + 500);
    Servo_AngleAcc(Angle);

}
/* K230 圆心像素 → 车体横向位移 (m/像素)。比例/方向需实测调, 反了取负 */
#define PLACE_CIRCLE_SCALE_M  0.005f

/** 等四个轮子都到位的上限 (ms)。★实测调 —— 这是**超时**, 不是固定延时:
 *  正常走完会提前返回, 只有卡住/掉线才真的等满。 */
#define PLACE_MOVE_TIMEOUT_MS   1500u

void Place(char dir,float x,float y,uint16_t height,uint8_t slot)
{
    if (dir == 'O')
    {

        float fwd  = 0.068f - y * PLACE_CIRCLE_SCALE_M;
        float left =  x * PLACE_CIRCLE_SCALE_M;

        BlockBasic_GripperRelease(slot);
        if (!Mecanum_MoveBodyPos(fwd, left, PLACE_MOVE_TIMEOUT_MS)) {
            printf("[PLACE] 前移没等齐到位 (超时)\r\n");
        }
        if (height!=0)
        {
            BlockBasic_LiftTo(DOWN, height);
            osDelay(200);
        }
   /* 松开正在放的这个槽 = 解锁 */

        /* 后退 0.05 m (车体 -X 方向) */
        if (!Mecanum_MoveBodyPos(-0.14f, 0.0f, PLACE_MOVE_TIMEOUT_MS)) {
            printf("[PLACE] 后退没等齐到位 (超时)\r\n");
        }
    }
}

bool BPlace_SetZero(void)
{
    uint8_t block_pos=0;
    block_pos=Key_IsPressed(KEY_LIMIT);
    // printf("%d\r\n",block_pos);
     if (!block_pos) {
         Emm_V5_Pos_Control(5,1,1000,0,1600,0,0);
         return false;
     }
    Emm_V5_Stop_Now(5,0);
    return true;
}

