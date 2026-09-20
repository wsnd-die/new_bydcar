/**
 * @file    Send_motor.h
 * @brief   麦轮底盘执行器下发 —— MecanumResult → 4 路 EMM_V5 → FDCAN2 (PB12/PB13)
 * @note    本头文件为 V1.3.0 补建。在此之前 Send_motor.c 是本工程唯一没有
 *          头文件的 .c，Send_commandmotor() 的声明散落在 Common_used.h 与
 *          algorithm/mecanum.c 两处手写 extern 中，接口没有单一出处。
 *          补建后调用方统一 #include "Send_motor.h"。
 */
#ifndef SEND_MOTOR_H
#define SEND_MOTOR_H

#include "mecanum.h"   /* MecanumResult */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  把麦轮解算结果下发给四路闭环步进电机，并触发同步运动。
 * @param  data  麦轮解算结果（四轮目标转速 + 方向），见 MecanumResult。
 * @note   内部含 osDelay(5) 与一次 Emm_V5_Synchronous_motion，会阻塞调用
 *         任务约 5ms。本函数属硬件驱动层，不参与任何运动学换算。
 */
void Send_commandmotor(MecanumResult *data);

#ifdef __cplusplus
}
#endif

#endif /* SEND_MOTOR_H */
