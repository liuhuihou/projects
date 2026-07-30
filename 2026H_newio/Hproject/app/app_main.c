/*
 * H题 - 车载平衡滚球运动控制系统
 * Main application entry point.
 * Platform: MSPM0G3507 (C07A + S28A)
 */

#include "ti_msp_dl_config.h"
#include "board_hardware.h"
#include "app_config.h"
#include "control_config.h"

/* Drivers */
#include "motor_driver.h"
#include "encoder_driver.h"
#include "line_sensor.h"
#include "oled_driver.h"
#include "button_input.h"
#include "stepper_driver.h"
#include "camera_uart.h"

/* Control */
#include "vehicle_controller.h"
#include "balance_controller.h"

/* App */
#include "competition_mode.h"
#include "line_follow_task.h"
#include "balance_task.h"

/* ==================== Helpers ==================== */

static uint32_t app_time_ms(void)
{
    return Control_GetTickCount() * CONTROL_PERIOD_MS;
}

static int32_t rpm_to_cm_s_x10(float rpm)
{
    float value = rpm * APP_WHEEL_CIRCUMFERENCE_CM * 10.0f / 60.0f;
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static const char *state_str(CompetitionState st)
{
    switch (st) {
        case STATE_IDLE:    return "IDLE  ";
        case STATE_RUNNING: return "RUN   ";
        case STATE_DONE:    return "DONE  ";
        default:            return "???   ";
    }
}
static void oled_show_mode_select(void)
{
    CompetitionQuestion mode = Competition_GetMode();
    CompetitionState state = Competition_GetState();
    uint32_t elapsed = Competition_GetElapsedMs();

    OLED_ShowString(0, 0, "H BALANCE CAR");
    /* Row 2: selected sub-question */
    switch (mode) {
        case COMP_Q2: OLED_ShowString(0, 2, "Q2:LAP 20S  "); break;
        case COMP_Q3: OLED_ShowString(0, 2, "Q3:BALL +-5 "); break;
        case COMP_Q4: OLED_ShowString(0, 2, "Q4:AB+BALL  "); break;
        case COMP_Q5: OLED_ShowString(0, 2, "Q5:LAP+BALL "); break;
        case COMP_Q6: OLED_ShowString(0, 2, "Q6:LAP+POS  "); break;
        default: break;
    }

    OLED_ShowString(0, 3, state_str(state));

    /* Rows 4/5: measured vs target wheel speed in cm/s.
     * Format "L:12.3 T:25.0" - actual on the left, target after T. */
    OLED_ShowString(0, 4, "L:");
    OLED_ShowTenths(2, 4, rpm_to_cm_s_x10(Control_GetLeftRpm()), 2);
    OLED_ShowString(8, 4, "T:");
    OLED_ShowTenths(10, 4, rpm_to_cm_s_x10(Control_GetLeftTargetRpm()), 2);

    OLED_ShowString(0, 5, "R:");
    OLED_ShowTenths(2, 5, rpm_to_cm_s_x10(Control_GetRightRpm()), 2);
    OLED_ShowString(8, 5, "T:");
    OLED_ShowTenths(10, 5, rpm_to_cm_s_x10(Control_GetRightTargetRpm()), 2);

    /* Row 6: elapsed time while running, key hint otherwise */
    if (state == STATE_RUNNING || state == STATE_DONE) {
        OLED_ShowString(0, 6, "T:");
        OLED_ShowTenths(2, 6, (int32_t)(elapsed / 100), 3);
        OLED_ShowString(8, 6, "S      ");
    } else {
        OLED_ShowString(0, 6, "1CLK:RUN 2CLK:SW");
    }

    /* Always show line sensor channels and line error on row 7.
     * Channel count comes from LINE_SENSOR_COUNT so an 8-channel board fills
     * two more columns without an edit here. "--" instead of the bit pattern
     * means the I2C sensor is not answering. */
    {
        uint8_t ir = LineSensor_ReadRaw();
        uint8_t i;
        OLED_ShowString(0, 7, "IR:");
        if (LineSensor_IsOnline()) {
            for (i = 0U; i < LINE_SENSOR_COUNT; ++i) {
                OLED_ShowChar((uint8_t)(3U + i), 7,
                              (ir & (uint8_t)(1U << (LINE_SENSOR_COUNT - 1U - i)))
                                  ? '1' : '0');
            }
        } else {
            for (i = 0U; i < LINE_SENSOR_COUNT; ++i) {
                OLED_ShowChar((uint8_t)(3U + i), 7, '-');
            }
        }
        OLED_ShowString(10, 7, "E:");
        OLED_ShowSignedInt(12, 7, Control_GetLineError(), 2);
    }
}

/* K230 UART RX interrupt handler.
 * Must be named after the generated K230_INST_IRQHandler macro (UART1 now).
 * The previous name, K230_UART_IRQHandler, matched no vector table entry, so
 * it was never called and no camera byte ever reached the parser. */
void K230_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(HW_K230_UART) ==
        DL_UART_MAIN_IIDX_RX) {
        uint8_t byte = DL_UART_Main_receiveData(HW_K230_UART);
        Camera_FeedByte(byte);
    }
}

/* ==================== Main ==================== */

int main(void)
{
    uint32_t last_oled_tick;
    uint32_t last_btn_tick;
    uint32_t last_debug_tick;
    uint32_t last_line_tick;

    SYSCFG_DL_init();

    /* Initialize all modules */
    DL_Timer_startCounter(HW_MOTOR_PWM_TIMER);
    LineSensor_Init();
    Encoder_Init();
    Motor_Init();
    Control_Init();
    Stepper_Init();
    Camera_Init();
    Balance_Init();
    Button_Init();
    LineFollow_Init();
    BalanceTask_Init();
    Competition_Init();

    OLED_Init();
    OLED_Clear();
    /* Draw the first frame while interrupts are still masked: the software
     * SPI bit-bang must not be interrupted mid-byte or the panel latches
     * garbage and stays blank. */
    oled_show_mode_select();

    /* Enable interrupts */
    NVIC_ClearPendingIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* Enable K230 UART RX interrupt */
    DL_UART_Main_enableInterrupt(HW_K230_UART, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);

    /* Prime the line sensor cache before the control timer starts filtering
     * it, so the first few control cycles work from a real sample instead of
     * the all-white initial value. */
    (void)LineSensor_Poll();

    last_oled_tick = 0;
    last_btn_tick = 0;
    last_line_tick = 0;

    while (1) {
        const uint32_t now_ms = app_time_ms();

        /* Scan on a fixed 10 ms cadence, as the original working version did.
         * Calling it on every loop iteration re-samples the same now_ms value
         * many times, which made the click windows behave unpredictably. */
        if ((now_ms - last_btn_tick) >= 10U) {
            last_btn_tick = now_ms;
            Button_Update(now_ms);
        }

        /* Competition state machine update */
        Competition_Update(now_ms);

        /* Line sensor I2C read. This is the only place it happens: the
         * transfer blocks for tens of microseconds and would stall the 100 Hz
         * control ISR - with the motors latched at their last duty - if the
         * sensor stretched the clock or dropped off the bus. Control_Tick()
         * only filters the cached sample.
         * Polled at 5 ms, twice the control rate, so every control cycle sees
         * a sample no older than one period. __WFI() below wakes on the
         * control timer at 10 ms and on every encoder edge, so the loop comes
         * round often enough to hold this cadence. */
        if ((now_ms - last_line_tick) >= 5U) {
            last_line_tick = now_ms;
            (void)LineSensor_Poll();
        }

        /* OLED refresh every 300ms (software SPI is slow, keep it sparse) */
        if ((now_ms - last_oled_tick) >= 300U) {
            last_oled_tick = now_ms;
            oled_show_mode_select();
        }

        __WFI();
    }
}
