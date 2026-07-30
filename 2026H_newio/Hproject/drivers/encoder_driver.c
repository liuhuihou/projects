#include "encoder_driver.h"
#include "board_hardware.h"

/* Verified against a stopwatch: the reported speed matches reality, so the
 * 13-line / 2-counts-per-line / 30:1 scaling is correct. */
#define ENCODER_LINES_PER_MOTOR_REV (13.0f)
#define ENCODER_EDGE_FACTOR         (2.0f)
#define ENCODER_GEAR_RATIO          (30.0f)

volatile int32_t g_encoder_a_count;
volatile int32_t g_encoder_b_count;

static uint8_t encoder_a_phase_a(void)
{
    return HW_GPIO_READ(HW_MOTOR_A_ENCODER_PORT, HW_MOTOR_A_ENCODER_A_PIN) ? 1U : 0U;
}

static uint8_t encoder_a_phase_b(void)
{
    return HW_GPIO_READ(HW_MOTOR_A_ENCODER_PORT, HW_MOTOR_A_ENCODER_B_PIN) ? 1U : 0U;
}

static uint8_t encoder_b_phase_a(void)
{
    return HW_GPIO_READ(HW_MOTOR_B_ENCODER_PORT, HW_MOTOR_B_ENCODER_A_PIN) ? 1U : 0U;
}

static uint8_t encoder_b_phase_b(void)
{
    return HW_GPIO_READ(HW_MOTOR_B_ENCODER_PORT, HW_MOTOR_B_ENCODER_B_PIN) ? 1U : 0U;
}

void Encoder_Init(void)
{
    g_encoder_a_count = 0;
    g_encoder_b_count = 0;
}

void GROUP1_IRQHandler(void)
{
    const uint32_t status_a = DL_GPIO_getEnabledInterruptStatus(
        HW_MOTOR_A_ENCODER_PORT,
        HW_MOTOR_A_ENCODER_A_PIN | HW_MOTOR_A_ENCODER_B_PIN);
    const uint32_t status_b = DL_GPIO_getEnabledInterruptStatus(
        HW_MOTOR_B_ENCODER_PORT,
        HW_MOTOR_B_ENCODER_A_PIN | HW_MOTOR_B_ENCODER_B_PIN);

    if ((status_a & HW_MOTOR_A_ENCODER_A_PIN) != 0U)
        g_encoder_a_count += encoder_a_phase_b() ? 1 : -1;
    if ((status_a & HW_MOTOR_A_ENCODER_B_PIN) != 0U)
        g_encoder_a_count += encoder_a_phase_a() ? -1 : 1;
    if ((status_b & HW_MOTOR_B_ENCODER_A_PIN) != 0U)
        g_encoder_b_count += encoder_b_phase_b() ? 1 : -1;
    if ((status_b & HW_MOTOR_B_ENCODER_B_PIN) != 0U)
        g_encoder_b_count += encoder_b_phase_a() ? -1 : 1;

    DL_GPIO_clearInterruptStatus(HW_MOTOR_A_ENCODER_PORT,
        HW_MOTOR_A_ENCODER_A_PIN | HW_MOTOR_A_ENCODER_B_PIN);
    DL_GPIO_clearInterruptStatus(HW_MOTOR_B_ENCODER_PORT,
        HW_MOTOR_B_ENCODER_A_PIN | HW_MOTOR_B_ENCODER_B_PIN);
}

void Encoder_TakeCounts(int32_t *encoder_a, int32_t *encoder_b)
{
    __disable_irq();
    *encoder_a = g_encoder_a_count;
    *encoder_b = g_encoder_b_count;
    g_encoder_a_count = 0;
    g_encoder_b_count = 0;
    __enable_irq();
}

float Encoder_CountsToRpm(int32_t count, uint32_t sample_ms)
{
    const float counts_per_wheel_rev = ENCODER_LINES_PER_MOTOR_REV
                                     * ENCODER_EDGE_FACTOR
                                     * ENCODER_GEAR_RATIO;
    if (sample_ms == 0U) return 0.0f;
    return ((float)count * 60000.0f) / (counts_per_wheel_rev * (float)sample_ms);
}
