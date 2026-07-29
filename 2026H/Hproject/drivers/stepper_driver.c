#include "stepper_driver.h"
#include "board_hardware.h"

static volatile int32_t s_position;
static volatile int32_t s_target_speed;  /* steps/sec, signed */
static volatile uint32_t s_step_interval; /* ticks between steps (0=stopped) */
static volatile uint32_t s_tick_counter;

void Stepper_Init(void)
{
    s_position = 0;
    s_target_speed = 0;
    s_step_interval = 0;
    s_tick_counter = 0;
    Stepper_Disable();
}

void Stepper_Enable(void)
{
    HW_GPIO_LOW(HW_STEPPER_EN_PORT, HW_STEPPER_EN_PIN); /* Active low enable */
}

void Stepper_Disable(void)
{
    HW_GPIO_HIGH(HW_STEPPER_EN_PORT, HW_STEPPER_EN_PIN);
}

void Stepper_SetSpeed(int32_t steps_per_sec)
{
    s_target_speed = steps_per_sec;

    if (steps_per_sec > 0) {
        HW_GPIO_HIGH(HW_STEPPER_DIR_PORT, HW_STEPPER_DIR_PIN);
        /* Assume Stepper_Tick is called at 10kHz (100us period) */
        s_step_interval = 10000U / (uint32_t)steps_per_sec;
    } else if (steps_per_sec < 0) {
        HW_GPIO_LOW(HW_STEPPER_DIR_PORT, HW_STEPPER_DIR_PIN);
        s_step_interval = 10000U / (uint32_t)(-steps_per_sec);
    } else {
        s_step_interval = 0;
    }
    if (s_step_interval == 0 && steps_per_sec != 0) {
        s_step_interval = 1; /* Clamp to maximum speed */
    }
    s_tick_counter = 0;
}

int32_t Stepper_GetPosition(void) { return s_position; }
void Stepper_ResetPosition(void) { s_position = 0; }

void Stepper_Tick(void)
{
    if (s_step_interval == 0) return;

    if (++s_tick_counter >= s_step_interval) {
        s_tick_counter = 0;
        /* Generate step pulse */
        HW_GPIO_HIGH(HW_STEPPER_STEP_PORT, HW_STEPPER_STEP_PIN);
        __NOP(); __NOP(); __NOP(); __NOP();
        HW_GPIO_LOW(HW_STEPPER_STEP_PORT, HW_STEPPER_STEP_PIN);

        if (s_target_speed > 0) s_position++;
        else s_position--;
    }
}
