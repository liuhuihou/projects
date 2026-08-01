/*
 * H题 - 车载平衡滚球运动控制系统
 * Main application entry point.
 * Platform: MSPM0G3507 (C07A + S28A)
 */

#include "ti_msp_dl_config.h"
#include "board_hardware.h"
#include "control_config.h"

/* Drivers */
#include "motor_driver.h"
#include "encoder_driver.h"
#include "line_sensor.h"
#include "oled_driver.h"
#include "button_input.h"
#include "stepper_driver.h"
#include "stepper_feedback.h"
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
        case STATE_IDLE:    return "IDLE  ";
        case STATE_LEVELING:return "LEVEL ";
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
    const uint32_t display_elapsed =
        (state == STATE_IDLE) ? 0U : elapsed;

    /* Common timer for Q2..Q6: zero while idle, running time after start,
     * and the frozen result after automatic or manual stop. */
    OLED_ShowString(0, 0, "TIME:");
    OLED_ShowTenths(5, 0, (int32_t)(display_elapsed / 100U), 3);
    OLED_ShowString(10, 0, "S          ");

    /* Protocol counters are no longer part of the competition display. */
    OLED_ShowString(0, 1, "                     ");

    /* Row 2: selected sub-question */
    switch (mode) {
        case COMP_Q2: OLED_ShowString(0, 2, "Q2:LAP 20S  "); break;
        case COMP_Q3: OLED_ShowString(0, 2, "Q3:STEPPER  "); break;
        case COMP_Q4: OLED_ShowString(0, 2, "Q4:AB+BALL  "); break;
        case COMP_Q5: OLED_ShowString(0, 2, "Q5:LAP+BALL "); break;
        case COMP_Q6: OLED_ShowString(0, 2, "Q6:LAP+POS  "); break;
        default: break;
    }

    OLED_ShowString(0, 3, state_str(state));

    /* Rest of row 3: K230 vision link. Columns 0..5 hold the state string, so
     * this starts at 7 and a row holds 21 characters.
     *   "K-- ---"   no frame in the last 200 ms: link down, check wiring
     *   "K?? ---"   frames arriving but the ruler is not locked
     *   "K 123 27"  position in mm, then the camera's own frame rate
     * A lowercase p in place of the leading space means the position came from
     * extrapolation rather than a fresh detection. The distinction matters when
     * debugging: a dead link and a ball out of view look identical otherwise. */
    {
        const CameraData *cam = Camera_GetData();
        uint32_t now = app_time_ms();
        /* Every branch must fill the same span, columns 7..16, or switching to
         * a shorter one leaves the tail of the previous message on screen.
         * ShowSignedInt writes a sign plus `width` columns, which is what makes
         * the last branch reach 16. */
        if (!Camera_IsDataFresh(now, 200U)) {
            OLED_ShowString(7, 3, "K-- ---   ");
        } else if ((cam->flags & CAMERA_FLAG_VALID) == 0U) {
            OLED_ShowString(7, 3, "K?? ");
            OLED_ShowNum(11, 3, cam->fps, 3);
            OLED_ShowString(14, 3, "   ");
        } else {
            OLED_ShowChar(7, 3, 'K');
            OLED_ShowChar(8, 3,
                          ((cam->flags & CAMERA_FLAG_DETECTED) != 0U) ? ' ' : 'p');
            OLED_ShowSignedInt(9, 3, (int)cam->ball_pos_mm, 4);
            OLED_ShowChar(14, 3, ' ');
            OLED_ShowNum(15, 3, cam->fps, 2);
        }
    }

    /* Rows 4/5 are reserved for the balance mechanism in every camera-ball
     * mode.  Wheel-speed telemetry was removed because no competition mode
     * uses it for setup or judging. */
    if (mode != COMP_Q2) {
        StepperFeedbackSnapshot feedback;
        StepperFeedback_GetSnapshot(&feedback);
        OLED_ShowString(0, 4, "STEP:");
        OLED_ShowSignedInt(5, 4, (int)Stepper_GetPosition(), 6);
        OLED_ShowString(12, 4, "         ");
        OLED_ShowString(0, 5, "AB:");
        OLED_ShowSignedInt(3, 5, (int)feedback.quadrature_count, 6);
        OLED_ShowString(10, 5, "T:");
        OLED_ShowSignedInt(12, 5, (int)Balance_GetTargetAb(), 5);
        OLED_ShowString(18, 5, "   ");
    } else {
        OLED_ShowString(0, 4, "                     ");
        OLED_ShowString(0, 5, "                     ");
    }

    /* Levelling prompts take priority over Q3's normal visual telemetry. */
    if (state == STATE_LEVELING) {
        OLED_ShowString(0, 6, "LEVELING SHORT:STOP  ");
    } else if (mode == COMP_Q6 && state == STATE_READY &&
               Competition_IsQ6BallHoldActive() == 0U) {
        OLED_ShowString(0, 6, "WAIT BALL           ");
    } else if (state == STATE_READY) {
        OLED_ShowString(0, 6, "READY 1CLK:RUN       ");
    } else if (mode == COMP_Q3 &&
               (state == STATE_RUNNING || state == STATE_DONE)) {
        const CameraData *cam = Camera_GetData();
        OLED_ShowString(0, 6, "P:");
        OLED_ShowSignedInt(2, 6, (int)cam->ball_pos_mm, 4);
        OLED_ShowString(8, 6, "T:");
        OLED_ShowSignedInt(10, 6, (int)Balance_GetTarget(), 4);
        OLED_ShowString(15, 6, "      ");
    } else if (state == STATE_RUNNING || state == STATE_DONE) {
        OLED_ShowString(0, 6, "T:");
        OLED_ShowTenths(2, 6, (int32_t)(elapsed / 100), 3);
        OLED_ShowString(8, 6, "S      ");
    } else if (mode == COMP_Q3 || mode == COMP_Q4 ||
               mode == COMP_Q5 || mode == COMP_Q6) {
        OLED_ShowString(0, 6, "HOLD1S:LEVEL 2CLK:SW ");
    } else {
        OLED_ShowString(0, 6, "1CLK:RUN 2CLK:SW");
    }

    /* Always show line sensor channels and line error on row 7.
     * Leftmost digit is the leftmost channel, matching the driver's bit
     * convention. "--" instead of the bit pattern means the I2C sensor is not
     * answering.
     * The error column is derived from LINE_SENSOR_COUNT rather than written
     * out: at 8 channels the digits reach column 10, which is where "E:" used
     * to start, so a fixed 10 would have overwritten the last channel. A row
     * holds 21 characters (128 px / 6), so there is room to spare. */
    if (mode == COMP_Q3) {
        const CameraData *cam = Camera_GetData();
        OLED_ShowString(0, 7, "V:");
        OLED_ShowSignedInt(2, 7, (int)cam->ball_vel_mm_s, 4);
        OLED_ShowString(8, 7, "O:");
        OLED_ShowSignedInt(10, 7, (int)Balance_GetOutput(), 5);
        OLED_ShowString(16, 7, "     ");
    } else {
        const uint8_t err_col = (uint8_t)(3U + LINE_SENSOR_COUNT + 1U);
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
        /* Blank the gap so a shorter channel count cannot leave a stale digit
         * between the bit pattern and the error. */
        OLED_ShowChar((uint8_t)(3U + LINE_SENSOR_COUNT), 7, ' ');
        OLED_ShowString(err_col, 7, "E:");
        OLED_ShowSignedInt((uint8_t)(err_col + 2U), 7,
                           Control_GetLineError(), 2);
    }
}

/* K230 UART RX interrupt handler.
 * Must be named after the generated K230_INST_IRQHandler macro (UART1 now).
 * The previous name, K230_UART_IRQHandler, matched no vector table entry, so
 * it was never called and no camera byte ever reached the parser. */
void K230_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(HW_K230_UART)) {
        case DL_UART_MAIN_IIDX_RX: {
            /* Drain the whole FIFO, not one byte. The FIFO is enabled, so one
             * interrupt can cover several bytes; taking only the first left the
             * rest to arrive as a burst on the next interrupt at best, and be
             * overrun at worst. A 12-byte frame at 115200 is ~1 ms, and the
             * camera sends ~27 of them a second, so this loop is short. */
            uint32_t now_ms = app_time_ms();
            while (!DL_UART_Main_isRXFIFOEmpty(HW_K230_UART)) {
                Camera_FeedByte(DL_UART_Main_receiveData(HW_K230_UART), now_ms);
            }
            break;
        }
        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            /* Reading the IIDX already cleared the flag. Nothing to salvage
             * from a corrupted byte - the CRC would reject the frame anyway,
             * and the parser resyncs on the next 0xAA 0x55. */
            break;
        default:
            break;
    }
}

/* ==================== Main ==================== */

int main(void)
{
    uint32_t last_oled_tick;
    uint32_t last_btn_tick;
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

    /* Start encoder feedback only after the first bit-banged OLED frame. */
    StepperFeedback_Init();

    /* Enable interrupts */
    NVIC_ClearPendingIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* Enable K230 UART RX interrupt */
    DL_UART_Main_enableInterrupt(HW_K230_UART, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
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
