#include "competition_mode.h"

#include "app_config.h"
#include "balance_controller.h"
#include "bluetooth_service.h"
#include "board_hardware.h"
#include "button_input.h"
#include "control_config.h"
#include "encoder_driver.h"
#include "line_follow_task.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "oled_driver.h"
#include "vehicle_controller.h"

#define DOUBLE_CLICK_WINDOW_MS (350U)

static const char *const s_mode_names[COMPETITION_MODE_COUNT] = {
    "Q2 LINE LAP",
    "Q3 BAL STATIC",
    "Q4 LINE+BLC",
    "Q5 LINE A LAP",
    "Q6 LINE+TARGET"
};

static uint32_t app_time_ms(void)
{
    return Control_GetTickCount() * CONTROL_PERIOD_MS;
}

static void app_process_background(void)
{
    BluetoothService_Process(app_time_ms());
}

static int32_t speed_to_cm_s_x10(float rpm)
{
    float value = rpm * APP_WHEEL_CIRCUMFERENCE_CM * 10.0f / 60.0f;
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void oled_show_status(CompetitionMode mode, uint8_t running)
{
    uint8_t ir = LineSensor_ReadRaw();
    uint8_t i;

    OLED_ShowString(0U, 0U, s_mode_names[mode]);
    OLED_ShowTenths(2U, 1U, speed_to_cm_s_x10(Control_GetLeftRpm()), 2U);
    OLED_ShowTenths(9U, 1U, speed_to_cm_s_x10(Control_GetLeftTargetRpm()), 2U);
    OLED_ShowTenths(2U, 2U, speed_to_cm_s_x10(Control_GetRightRpm()), 2U);
    OLED_ShowTenths(9U, 2U, speed_to_cm_s_x10(Control_GetRightTargetRpm()), 2U);
    OLED_ShowSignedInt(3U, 4U, Control_GetLineError(), 3U);

    for (i = 0U; i < 6U; ++i) {
        OLED_ShowChar((uint8_t)(3U + i), 3U,
                      (ir & (uint8_t)(1U << (5U - i))) ? '1' : '0');
    }
    OLED_ShowString(0U, 7U, running ? "RUN" : "BLS1 START 2 SEL");
}

static void app_hardware_init(void)
{
    SYSCFG_DL_init();
    DL_Timer_startCounter(HW_MOTOR_PWM_TIMER);
    LineSensor_Init();
    Encoder_Init();
    Motor_Init();
    Control_Init();
    BluetoothService_Init(app_time_ms());
    OLED_Init();
    OLED_Clear();

    NVIC_ClearPendingIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    LineFollowTask_Init();
    BalanceController_Init();
}

static void app_start_mode(CompetitionMode mode)
{
    LineFollowTask_Stop();
    BalanceController_Stop();

    switch (mode) {
        case COMPETITION_Q2:
            LineFollowTask_Start(LINE_FOLLOW_Q2_ONE_LAP);
            break;
        case COMPETITION_Q3:
            BalanceController_Start(BALANCE_TARGET_CENTER);
            break;
        case COMPETITION_Q4:
            BalanceController_Start(BALANCE_TARGET_CENTER);
            LineFollowTask_Start(LINE_FOLLOW_Q4_WITH_BALANCE);
            break;
        case COMPETITION_Q5:
            LineFollowTask_Start(LINE_FOLLOW_Q5_RETURN_TO_A);
            break;
        case COMPETITION_Q6:
            BalanceController_Start(BALANCE_TARGET_SPECIFIED);
            LineFollowTask_Start(LINE_FOLLOW_Q6_WITH_TARGET_BALANCE);
            break;
        default:
            break;
    }
}

static void app_tick_tasks(uint32_t now_ms)
{
    LineFollowTask_Tick(now_ms);
    BalanceController_Tick(now_ms);
}

void CompetitionMode_Run(void)
{
    CompetitionMode selected_mode = COMPETITION_Q2;
    uint8_t pending_single = 0U;
    uint32_t first_click_ms = 0U;
    uint8_t running = 0U;
    uint32_t last_oled_tick;

    app_hardware_init();
    ButtonInput_Init(app_time_ms());
    oled_show_status(selected_mode, 0U);

    last_oled_tick = Control_GetTickCount();
    while (1) {
        const uint32_t now_ms = app_time_ms();
        const uint8_t events = ButtonInput_Poll(now_ms);

        if (running == 0U) {
            if ((events & BUTTON_EVENT_BLS_CLICK) != 0U) {
                if ((pending_single != 0U) &&
                    ((uint32_t)(now_ms - first_click_ms) <=
                     DOUBLE_CLICK_WINDOW_MS)) {
                    pending_single = 0U;
                    selected_mode = (CompetitionMode)(
                        ((uint32_t)selected_mode + 1U) % COMPETITION_MODE_COUNT);
                    oled_show_status(selected_mode, 0U);
                } else {
                    pending_single = 1U;
                    first_click_ms = now_ms;
                }
            }

            if ((pending_single != 0U) &&
                ((uint32_t)(now_ms - first_click_ms) >
                 DOUBLE_CLICK_WINDOW_MS)) {
                pending_single = 0U;
                app_start_mode(selected_mode);
                running = 1U;
                oled_show_status(selected_mode, 1U);
            }
        } else {
            app_tick_tasks(now_ms);
        }

        app_process_background();
        if ((uint32_t)(Control_GetTickCount() - last_oled_tick) >= 50U) {
            last_oled_tick = Control_GetTickCount();
            oled_show_status(selected_mode, running);
        }
        __WFI();
    }
}
