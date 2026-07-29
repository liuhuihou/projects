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
#include "debug_uart.h"
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

static const char *state_str(CompetitionState st)
{
    switch (st) {
        case STATE_IDLE:    return "SELECT";
        case STATE_READY:   return "READY ";
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

    OLED_ShowString(0, 0, "H: BALANCE CAR");
    OLED_ShowString(0, 2, "MODE:");
    /* Show mode name (pad to 11 chars) */
    switch (mode) {
        case COMP_Q2: OLED_ShowString(5, 2, "Q2:LAP 20S "); break;
        case COMP_Q3: OLED_ShowString(5, 2, "Q3:BALL+-5 "); break;
        case COMP_Q4: OLED_ShowString(5, 2, "Q4:AB+BALL "); break;
        case COMP_Q5: OLED_ShowString(5, 2, "Q5:LAP+BAL "); break;
        case COMP_Q6: OLED_ShowString(5, 2, "Q6:LAP+POS "); break;
        default: break;
    }

    OLED_ShowString(0, 4, state_str(state));

    /* Show elapsed time as seconds.tenths */
    if (state == STATE_RUNNING || state == STATE_DONE) {
        OLED_ShowString(0, 6, "T:");
        OLED_ShowTenths(2, 6, (int32_t)(elapsed / 100), 3);
        OLED_ShowString(8, 6, "S");
    } else {
        OLED_ShowString(0, 6, "1CLK:GO 2CLK:SW");
    }
}

/* K230 UART RX interrupt handler */
void K230_UART_IRQHandler(void)
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
    OLED_ShowString(0, 0, "H: BALANCE CAR");
    OLED_ShowString(0, 2, "INIT OK");
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

    last_oled_tick = 0;
    last_btn_tick = 0;

    while (1) {
        const uint32_t now_ms = app_time_ms();

        /* Button scan every 10ms */
        if ((now_ms - last_btn_tick) >= 10U) {
            last_btn_tick = now_ms;
            Button_Update(now_ms);
        }

        /* Competition state machine update */
        Competition_Update(now_ms);

        /* OLED refresh every 200ms */
        if ((now_ms - last_oled_tick) >= 200U) {
            last_oled_tick = now_ms;
            oled_show_mode_select();
        }

        __WFI();
    }
}
