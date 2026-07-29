#include "motor_driver.h"
#include "board_hardware.h"

static int32_t clamp_duty(int32_t duty)
{
    if (duty > MOTOR_PWM_LIMIT) return MOTOR_PWM_LIMIT;
    if (duty < -MOTOR_PWM_LIMIT) return -MOTOR_PWM_LIMIT;
    return duty;
}

static void motor_a_direction(int32_t duty)
{
    if (duty > 0) {
        HW_GPIO_WRITE(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN1_PIN, 1);
        HW_GPIO_WRITE(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN2_PIN, 0);
    } else if (duty < 0) {
        HW_GPIO_WRITE(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN1_PIN, 0);
        HW_GPIO_WRITE(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN2_PIN, 1);
    } else {
        HW_MOTOR_A_COAST();
    }
}

static void motor_b_direction(int32_t duty)
{
    if (duty > 0) {
        HW_GPIO_WRITE(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN1_PIN, 1);
        HW_GPIO_WRITE(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN2_PIN, 0);
    } else if (duty < 0) {
        HW_GPIO_WRITE(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN1_PIN, 0);
        HW_GPIO_WRITE(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN2_PIN, 1);
    } else {
        HW_MOTOR_B_COAST();
    }
}

void Motor_Init(void)
{
    Motor_Stop();
}

void Motor_Stop(void)
{
    HW_MOTOR_A_SET_DUTY(0U);
    HW_MOTOR_B_SET_DUTY(0U);
    HW_MOTOR_A_COAST();
    HW_MOTOR_B_COAST();
}

void Motor_SetDuty(int32_t left_duty, int32_t right_duty)
{
    left_duty = clamp_duty(left_duty);
    right_duty = clamp_duty(right_duty);

    motor_a_direction(right_duty);
    motor_b_direction(left_duty);

    HW_MOTOR_A_SET_DUTY((uint32_t)((right_duty < 0) ? -right_duty : right_duty));
    HW_MOTOR_B_SET_DUTY((uint32_t)((left_duty < 0) ? -left_duty : left_duty));
}
