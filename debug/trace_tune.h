#ifndef TRACE_TUNE_H
#define TRACE_TUNE_H

#include <stdint.h>
#include <stdbool.h>

/* Call once from the line-follow control cycle. */
void Trace_Tune_Service(void);

/* Call from the USART1 RX callback. */
bool Trace_Tune_OnByte(uint8_t b);

/* Runtime PID gains and limits. */
extern float g_tune_angle_kp;
extern float g_tune_angle_ki;
extern float g_tune_angle_kd;
extern float g_tune_gray_kp;
extern float g_tune_gray_ki;
extern float g_tune_gray_kd;
extern float g_tune_pos_kp;
extern float g_tune_pos_ki;
extern float g_tune_pos_kd;
extern float g_tune_speed;
extern float g_tune_wmax;
extern float g_tune_pos_bias;
extern volatile uint8_t g_tune_control_override;

/* Telemetry state. */
extern volatile uint8_t g_tune_monitor;
extern volatile uint16_t g_tune_monitor_rate;

/* Increment steps used by <param>+ and <param>-. */
extern float g_tune_step_akp;
extern float g_tune_step_aki;
extern float g_tune_step_akd;
extern float g_tune_step_gkp;
extern float g_tune_step_gkd;
extern float g_tune_step_gki;
extern float g_tune_step_pkp;
extern float g_tune_step_pki;
extern float g_tune_step_pkd;
extern float g_tune_step_vmax;
extern float g_tune_step_wmax;

/* Record one completed PID control sample. */
void Trace_Tune_Record(float angle, float position, float target_position,
                       float v, float w);

#endif
