/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "vehicle_controller.h"
#include "encoder_driver.h"
#include "line_sensor.h"
#include "motor_driver.h"
#include "board_hardware.h"
#include "oled_driver.h"

static int32_t speed_to_cm_s_x10(float rpm)
{
    float value = rpm * APP_WHEEL_CIRCUMFERENCE_CM * 10.0f / 60.0f;
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void oled_show_status(uint8_t running)
{
    uint8_t ir = LineSensor_ReadRaw();
    uint8_t i;

    OLED_ShowTenths(2, 1, speed_to_cm_s_x10(Control_GetLeftRpm()), 2);
    OLED_ShowTenths(9, 1, speed_to_cm_s_x10(Control_GetLeftTargetRpm()), 2);
    OLED_ShowTenths(2, 2, speed_to_cm_s_x10(Control_GetRightRpm()), 2);
    OLED_ShowTenths(9, 2, speed_to_cm_s_x10(Control_GetRightTargetRpm()), 2);
    OLED_ShowSignedInt(3, 4, Control_GetLineError(), 3);

    for (i = 0U; i < 6U; ++i) {
        OLED_ShowChar((uint8_t)(3U + i), 3,
                      (ir & (uint8_t)(1U << (5U - i))) ? '1' : '0');
    }
    OLED_ShowString(0, 7, running ? "RUN LINE" : "WAIT BLS");
}

/* C07A BLS is active high: PA18 is pulled down and driven to 3.3 V when pressed. */
static void wait_for_start_key(void)
{
    while (!HW_GPIO_READ(HW_BLS_KEY_PORT, HW_BLS_KEY_PIN)) {
        __WFI();
    }

    while (HW_GPIO_READ(HW_BLS_KEY_PORT, HW_BLS_KEY_PIN)) {
        __WFI();
    }

    Control_SetBaseSpeed(APP_START_SPEED_CM_S * 60.0f /
                         APP_WHEEL_CIRCUMFERENCE_CM);
    Control_SetMode(CTRL_LINE);
}

int main(void)
{
    SYSCFG_DL_init();

    DL_Timer_startCounter(HW_MOTOR_PWM_TIMER);
    LineSensor_Init();
    Encoder_Init();
    Motor_Init();
    Control_Init();
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "LINE FOLLOW");
    OLED_ShowString(0, 1, "L:");
    OLED_ShowString(7, 1, "T:");
    OLED_ShowString(0, 2, "R:");
    OLED_ShowString(7, 2, "T:");
    OLED_ShowString(0, 3, "IR:");
    OLED_ShowString(0, 4, "E:");
    OLED_ShowString(8, 4, "CM/S");
    OLED_ShowString(0, 6, "J1=R J2=L");
    oled_show_status(0U);

    NVIC_ClearPendingIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_A_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_EnableIRQ(HW_MOTOR_B_ENCODER_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    wait_for_start_key();

    {
        uint32_t last_oled_tick = Control_GetTickCount();
        while (1) {
            const uint32_t now = Control_GetTickCount();

            if ((uint32_t)(now - last_oled_tick) >= 50U) {
                last_oled_tick = now;
                oled_show_status(1U);
            }
            __WFI();
        }
    }
}
