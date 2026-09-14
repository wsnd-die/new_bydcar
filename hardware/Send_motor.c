#include "Common_used.h"
uint16_t left_vel,right_vel;
uint8_t left_acc,left_dir;
uint8_t right_acc,right_dir;
float motor_v,motor_w;  

//舵机拟合改这个变量front_angle，完成后把为它赋拟合后的常数。
volatile float front_angle=94.35285048f;
extern TIM_HandleTypeDef htim3;
// AckermanResult motor_data;
extern MecanumResult motor_data;

//uint8_t flag_ok;

void Send_commandmotor(MecanumResult *data)
{
	Emm_V5_Vel_Control(1, !data->fr_dir, data->fr_speed, 130, 0);  /* 1号=前右 */
	Emm_V5_Vel_Control(2, data->rl_dir, data->rl_speed, 130, 0);   /* 2号=后左 */
	Emm_V5_Vel_Control(3, !data->fl_dir, data->fl_speed, 130, 0);  /* 3号=前左 */
	Emm_V5_Vel_Control(4, data->rr_dir, data->rr_speed, 130, 0);  /* 4号=后右 */
    osDelay(5);
	Emm_V5_Synchronous_motion(0);
}




