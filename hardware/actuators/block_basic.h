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
#define BLOCK_LIFT_MAX_MM                70.0f   /* 丝杆最大行程 (mm) */


#define BLOCK_SERVO_DEG              360.0f

/* ================================================================
 * 转盘舵机：飞特 STS3032 总线舵机（UART5，SMS_STS 系列）
 *
 * V1.16.0 起转盘由 TIM3_CH2 的 PWM 舵机改为总线舵机，位置写走
 * SCS_WritePosEx()（见 servo_scs.h），不再占用任何 TIM 通道。
 *
 * 与 UART5 上 ID2~6 的 SCS0009 分属两个相反的字节序系列：
 * SCS_WritePosEx() 会自己在锁内把总线字节序切成小端，调用方不必管。
 * ================================================================ */
/** 总线上的舵机 ID。与 Core/Src/app_freertos.c 的 SERVO_ID_STS3032 是同一颗。 */
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
#define BLOCK_TURNTABLE_HOME_DEG         0.0f
#define BLOCK_TURNTABLE_STEP_DEG         (BLOCK_SERVO_DEG / BLOCK_TURNTABLE_POS_COUNT)
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
void Place(char dir,float x,float y,uint16_t height);
    /*
     *
     *
     */
void Servo_SetAngle(float Angle);

#ifdef __cplusplus
}
#endif

#endif
