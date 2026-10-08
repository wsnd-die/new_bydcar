/**
 * @file block_basic.h  
 * @brief 本模块负责物块相关机构的上层换算和动作下发
 * @version 0.1
 * @date 2026-07-30
 * @copyright Copyright (c) 2026
 * @note 物块机构的丝杆升降和双机械臂升降分别对应两台车的情况
 *       1. 丝杆车型：把目标升高高度换算成 5 号 EMM 步进电机位置模式脉冲。
 *       2. 双机械臂车型：把目标升高高度换算成 CH1/CH3 两个舵机角度。
 *       3. 转盘机构：把 1~5 号物块位置换算成 UART5 总线上 STS3032 的
 *          绝对位置（0~4095 ↔ 0~360°），走 SCS_WritePosEx()。V1.16.0 起
 *          不再占用任何 TIM 通道 —— 原先的 TIM3_CH2 PWM 路径已删除。
 */
#ifndef BLOCK_BASIC_H
#define BLOCK_BASIC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 车型编译期选择：0 = 丝杆型，1 = 双机械臂型
 * ================================================================ */
#define BLOCK_USE_DUAL_ARM              0

/* 丝杆每上升 1mm 需要的 EMM 位置模式脉冲数。 */
#define BLOCK_STEPPER_PULSE_PER_MM       1600.0f
#define BLOCK_LIFT_MAX_MM                100.0f   /* 丝杆最大行程 (mm) */


#define BLOCK_SERVO_DEG              360.0f


/** 总线上的舵机 ID。 */
#define BLOCK_TURNTABLE_SERVO_ID         1u
/** 位置量程上限：12 位单圈绝对值，0~4095 对应 0~360°（中位 2048）。 */
#define BLOCK_TURNTABLE_SERVO_POS_MAX    4095u
/** 运行速度，原始寄存器值；0 = 用寄存器内部值。 */
#define BLOCK_TURNTABLE_SERVO_SPEED      0u
/** 加速度，原始寄存器值 0~254；0 = 不控加速度直冲最高速。 */
#define BLOCK_TURNTABLE_SERVO_ACC        0u

#if BLOCK_USE_DUAL_ARM
/* 双机械臂参数：CH1 为前级舵机（舵机1），CH3 为后级舵机（舵机2）。 */
#define BLOCK_ARM_MIN_HEIGHT_MM          0.0f
#define BLOCK_ARM_MAX_HEIGHT_MM          100.0f
#define BLOCK_ARM_LINK_CM                10.5f
#define BLOCK_ARM_HEIGHT_MM_PER_CM       10.0f
#define BLOCK_ARM_INIT_DEG               (-12.0f)
#define BLOCK_ARM_S2_OFFSET_DEG          9.51f
#define BLOCK_ARM_RETREAT_PER_MM         0.0f
#endif

/* 转盘位置编号从 1 开始，合法范围为 1~5。 */
#define BLOCK_TURNTABLE_FIRST_POS        1u
#define BLOCK_TURNTABLE_POS_COUNT        5u
#define BLOCK_TURNTABLE_HOME_DEG         10.0f
#define BLOCK_TURNTABLE_STEP_DEG         (BLOCK_SERVO_DEG / BLOCK_TURNTABLE_POS_COUNT)
/**
 * 关门角度, 单位 deg。第 5 槽之后再往前多转一点把门带上, 用 Servo_Angle() 下发。
 * 340 = HOME(14) + 4 × STEP(72) [= 槽5] + 38。★ 那 38° 是实测值, 原样来自
 * Core/Src/app_freertos.c 里注释掉的 `// Servo_Angle(340);`。
 */
#define BLOCK_CLOSE_DOOR         340.0f
#define TROPHY_CLOSE_DOOR        82.0f
/* 单次最大角度步长。分段移动用于降低 360 度位置舵机自动走最短路径的风险。 */
#define BLOCK_TURNTABLE_STEP_LIMIT_DEG   72.0f
#define BLOCK_TURNTABLE_STEP_DELAY_MS    10u

#define x_limit 0.003f
#define y_limit 0.003f
    typedef  enum
    {
        UP = 0,
        DOWN = 1,

    }Action;

typedef enum {
    BLOCK_OK = 0,
    BLOCK_ERR_PARAM = 1
} BlockStatus;

/* ================================================================
 * 夹爪 (SCS0009, ID 2~6) —— 与转盘槽位一一对应
 * ================================================================ */

/** 槽位 → 夹爪舵机 ID。槽 1→ID2 … 槽 5→ID6。
 *  @note 这是**约定**, 仓库里没有别处写明, 首次上机需逐个确认。 */
#define BLOCK_GRIPPER_SERVO_ID(slot)   ((uint8_t)((slot) + 1u))

/** 物块形状。0 留给「未知/未采集」, 与 g_tt.shape[] 的初值一致。 */
typedef enum {
    SHAPE_UNKNOWN  = 0,
    SHAPE_RECT     = 1,   /* 长方体 */
    SHAPE_CYLINDER = 2    /* 圆柱 */
} BlockShape_t;

/** @brief 夹紧槽位 slot 的夹爪 (写 SCS_CLOSE)。 */
BlockStatus BlockBasic_GripperClamp(uint8_t slot);
/** @brief 松开槽位 slot 的夹爪 (写 SCS_OPEN)。 */
BlockStatus BlockBasic_GripperRelease(uint8_t slot);
/** @brief 回读槽位 slot 夹爪的当前位置。@retval 原始位置 0~1024; -1 = 总线失败。 */
int BlockBasic_GripperRaw(uint8_t slot);
/**
 * @brief  原始位置 → 形状。
 * @param  slot  物理槽号。判定阈值是**逐槽**的, 见 block_basic.c 的 Slot_Shape[]。
 * @retval raw < 0(总线失败) 或 slot 不在 2~5 时返回 SHAPE_UNKNOWN —— 不猜。
 */
BlockShape_t BlockBasic_ShapeFromRaw(int raw_angle, uint8_t slot);

#if BLOCK_USE_DUAL_ARM
/* 双机械臂高度换算结果。 */
typedef struct {
    float front_angle_deg;       /* 前级舵机 CH1 角度，单位 deg。 */
    float rear_angle_deg;        /* 后级舵机 CH3 角度，单位 deg。 */
    float turntable_retreat_mm;  /* 转盘相对后退距离，单位 mm。 */
} BlockArmResult;
#endif

/**
 * @brief  物块升降统一入口。
 * @param  dir        升降方向，0=下降，1=上升。
 * @param  pos        双机械臂型为位置表编号；丝杆型为目标升高高度，单位 mm。
 * @retval >=0        转盘相对后退距离，单位 mm。
 * @retval <0         参数错误或运动失败。
 */
    float BlockBasic_LiftTo(uint8_t dir, float pos);

/**
 * @brief  丝杆升降 —— **绝对位置**（指定点位）指令。
 * @param  target_mm  目标高度, 单位 mm, 有效范围 [0, BLOCK_LIFT_MAX_MM]
 * @retval true   已下发（方向与脉冲由内部按当前位置算出来）
 * @retval false  越界未执行 / 双机械臂车型不支持
 *
 * @note   调用方只管说"升到多少 mm"，不必记当前高度、也不用心算相对量。
 *         旧的相对版 `BlockBasic_LiftTo(dir, delta)` 是**顺序相关**的 ——
 *         中间某一拍被跳过（例如某次 PlaceDown 跳到"找不到"分支），
 *         后面所有高度就整体错位；新代码请用本函数。
 * @note   已经停在该高度时直接返回 true，不发 CAN。
 * @note   两个 API **共用同一份**"软件记录的当前高度"，混用不会错位。
 * @warning 软件高度必须与机械零点一致：`BPlace_SetZero()` 把丝杆压到限位归零
 *          之后要调 `BlockBasic_LiftSync(0)`（目前上电顺序恰好满足 —— 归零后
 *          才第一次动；以后若有二次归零，记得补这一句）。
 */
bool BlockBasic_LiftToAbs(float target_mm);

/**
 * @brief  把软件记录的丝杆高度同步成机械实际位置。
 * @param  cur_mm  当前实际高度，单位 mm；会夹到 [0, BLOCK_LIFT_MAX_MM]。
 * @note   归零（压限位）之后调用，例如 `BlockBasic_LiftSync(0.0f)`。
 */
void BlockBasic_LiftSync(float cur_mm);

#if BLOCK_USE_DUAL_ARM
/**
 * @brief  只计算双机械臂高度对应关系，不实际输出 PWM。
 * @param  height_mm  目标升高高度，单位 mm。
 * @retval 前级舵机角度、后级舵机角度、转盘相对后退距离。
 */
BlockArmResult BlockBasic_ArmCalc(float height_mm);

/**
 * @brief  双机械臂预设位置控制（CH1 前级 + CH3 后级）。
 * @param  pos  1=初始(170°,30°)  2=最低(72°,79°)  3=第二位置(94°,101°)
 */
void BlockBasic_DualArmSetPos(uint8_t pos);
#endif

/**
 * @brief  转盘转到指定物块位置。
 * @param  block_pos  物块位置编号，合法范围为 1~5。
 * @retval BLOCK_OK / BLOCK_ERR_PARAM
 *
 * @note   转盘按 UART5 总线上的 STS3032 绝对位置舵机处理（V1.16.0 起），
 *         位置量程 0~4095 对应 0~360°。若换成连续旋转速度型舵机，
 *         本接口只能作为框架，不能保证绝对角度定位。
 * @warning 本函数据此**阻塞**在总线收发上（厂商协议为发一帧收一帧），
 *          舵机不在线时每字节最多等 SCS_UART_RX_TIMEOUT_MS。只应在低频
 *          命令任务里调用，不要放进控制环。
 */
BlockStatus BlockBasic_TurntableTo(uint8_t block_pos);

/**
 * @brief  重置软件记录的转盘当前角度，并立即输出该角度 PWM。
 * @param  angle_deg  当前机械角度，单位 deg；会归一化到 0~360。
 *
 * @note   上电后如果转盘实际位置不在 BLOCK_TURNTABLE_HOME_DEG，
 *         应先调用本函数同步软件状态。
 */
void Servo_Angle(float angle_deg);

/**
 * @brief  放置: 前移到位 → 丝杆下降 → **松开该槽夹爪(解锁)** → 后退。
 * @param  dir     'O' = 正常放置 (传其它值啥也不做)
 * @param  x, y    找圆给出的偏差量 (m), 经 PLACE_CIRCLE_SCALE_M 折算
 * @param  height  丝杆下降高度 (mm); 0 = 不下降
 * @param  slot    **正在放的那个物理槽号 (1~5)** —— 用来松开对应的夹爪:
 *                 奖杯路径传 `tslot`, 物料路径传 `TT_CurrentSlot()`。
 * @note   V1.24.5 加了 slot 参数: 原来这里调的是**无参**的
 *         `BlockBasic_GripperRelease()`, 而它是要槽号的 —— 编译不过
 *         (`too few arguments`); 而且"松开哪个夹爪"本来也只能由调用方告诉。
 */
void Place(char dir,float x,float y,int16_t height,uint8_t slot);

void Servo_SetAngle(float Angle);
bool BPlace_SetZero(void) ;
#ifdef __cplusplus
}
#endif

#endif
