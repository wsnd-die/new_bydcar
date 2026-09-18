#ifndef __SEND_MOTOR_H
#define __SEND_MOTOR_H

#include "../algorithm/mecanum.h"      /* MecanumResult */

/* 麦轮四轮速度/方向命令下发（Emm_V5 电机驱动，见 hardware/Send_motor.c） */
void Send_commandmotor(MecanumResult *data);

#endif
