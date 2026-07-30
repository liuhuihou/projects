#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#define MOTOR_PWM_LIMIT (7800)

void Motor_Init(void);
void Motor_Stop(void);
void Motor_Brake(void);
void Motor_SetDuty(int32_t left_duty, int32_t right_duty);

#endif
