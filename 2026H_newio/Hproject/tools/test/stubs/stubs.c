/*
 * Host stubs for the peripherals the code under test writes to. The transmit
 * path records bytes instead of shifting them out a UART; the stepper records
 * the last speed command so the balance harness can report it.
 */

#include "ti_msp_dl_config.h"
#include "stepper_driver.h"
#include "stepper_feedback.h"
#include "control_config.h"
#include "vehicle_controller.h"

uint8_t  stub_tx_buf[256];
uint32_t stub_tx_len;

void DL_UART_Main_transmitDataBlocking(void *uart, uint8_t byte)
{
    (void)uart;
    if (stub_tx_len < sizeof(stub_tx_buf)) {
        stub_tx_buf[stub_tx_len++] = byte;
    }
}

/* Stepper state, observable by the harness. */
int32_t stub_stepper_speed;
int     stub_stepper_enabled;
int32_t stub_feedback_count;

void Stepper_Init(void)
{
    stub_stepper_speed = 0;
    stub_stepper_enabled = 0;
    stub_feedback_count = BALANCE_LEVEL_AB_COUNT;
}
void Stepper_Enable(void)  { stub_stepper_enabled = 1; }
void Stepper_Disable(void) { stub_stepper_enabled = 0; }

void Stepper_SetSpeed(int32_t steps_per_sec)
{
    stub_stepper_speed = steps_per_sec;
}

int32_t Stepper_GetPosition(void)   { return 0; }
void    Stepper_ResetPosition(void) { }

uint8_t StepperFeedback_IsTravelCommandAllowed(int32_t steps_per_sec)
{
    (void)steps_per_sec;
    return 1U;
}

void StepperFeedback_GetSnapshot(StepperFeedbackSnapshot *snapshot)
{
    if (snapshot == 0) return;
    snapshot->quadrature_count = stub_feedback_count;
    snapshot->index_count = 0;
    snapshot->invalid_transition_count = 0U;
    snapshot->pwm_period_ticks = 0U;
    snapshot->pwm_high_ticks = 0U;
    snapshot->pwm_angle_tenths = 0U;
    snapshot->pwm_valid = 0U;
    snapshot->phase_a = 0U;
    snapshot->phase_b = 0U;
    snapshot->index_level = 0U;
}

ControlMode Control_GetMode(void) { return CTRL_STOP; }
float Control_GetStartRampScale(void) { return 1.0f; }
