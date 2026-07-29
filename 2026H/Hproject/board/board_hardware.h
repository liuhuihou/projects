/*
 * Hardware interface for H题 - 车载平衡滚球运动控制系统
 * Platform: C07A (MSPM0G3507) + S28A baseboard
 * Pin assignments unchanged from reference project.
 */
#ifndef BOARD_HARDWARE_H
#define BOARD_HARDWARE_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ======================== Clock ======================== */
#define HW_CPU_CLOCK_HZ                 CPUCLK_FREQ

/* ======================== Motor PWM (TB6612) ======================== */
#define HW_MOTOR_PWM_TIMER              PWM_0_INST
#define HW_MOTOR_PWM_PERIOD_TICKS       (8000U)
#define HW_MOTOR_A_PWM_CHANNEL          GPIO_PWM_0_C0_IDX
#define HW_MOTOR_B_PWM_CHANNEL          GPIO_PWM_0_C1_IDX

/* ======================== GPIO Helpers ======================== */
#define HW_GPIO_HIGH(_port, _pin)       DL_GPIO_setPins((_port), (_pin))
#define HW_GPIO_LOW(_port, _pin)        DL_GPIO_clearPins((_port), (_pin))
#define HW_GPIO_READ(_port, _pin)       (DL_GPIO_readPins((_port), (_pin)) != 0U)
#define HW_GPIO_WRITE(_port, _pin, _v)  do { \
    if ((_v) != 0U) {                    \
        HW_GPIO_HIGH((_port), (_pin));   \
    } else {                             \
        HW_GPIO_LOW((_port), (_pin));    \
    }                                    \
} while (0)

/* ======================== TB6612 Direction ======================== */
#define HW_MOTOR_A_DIR_PORT             MOTOR_A_DIR_PORT
#define HW_MOTOR_A_IN1_PIN              MOTOR_A_DIR_AIN1_PIN
#define HW_MOTOR_A_IN2_PIN              MOTOR_A_DIR_AIN2_PIN
#define HW_MOTOR_B_DIR_PORT             MOTOR_B_DIR_PORT
#define HW_MOTOR_B_IN1_PIN              MOTOR_B_DIR_BIN1_PIN
#define HW_MOTOR_B_IN2_PIN              MOTOR_B_DIR_BIN2_PIN

#define HW_MOTOR_A_SET_DUTY(_duty)      \
    DL_TimerA_setCaptureCompareValue(HW_MOTOR_PWM_TIMER, (_duty), HW_MOTOR_A_PWM_CHANNEL)
#define HW_MOTOR_B_SET_DUTY(_duty)      \
    DL_TimerA_setCaptureCompareValue(HW_MOTOR_PWM_TIMER, (_duty), HW_MOTOR_B_PWM_CHANNEL)
#define HW_MOTOR_A_COAST() do {          \
    HW_GPIO_LOW(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN1_PIN); \
    HW_GPIO_LOW(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN2_PIN); \
} while (0)
#define HW_MOTOR_B_COAST() do {          \
    HW_GPIO_LOW(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN1_PIN); \
    HW_GPIO_LOW(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN2_PIN); \
} while (0)
/* ======================== Encoders ======================== */
#define HW_MOTOR_A_ENCODER_PORT         ENCODERA_PORT
#define HW_MOTOR_A_ENCODER_A_PIN        ENCODERA_E1A_PIN
#define HW_MOTOR_A_ENCODER_B_PIN        ENCODERA_E1B_PIN
#define HW_MOTOR_A_ENCODER_IRQN         ENCODERA_INT_IRQN
#define HW_MOTOR_B_ENCODER_PORT         ENCODERB_PORT
#define HW_MOTOR_B_ENCODER_A_PIN        ENCODERB_E2A_PIN
#define HW_MOTOR_B_ENCODER_B_PIN        ENCODERB_E2B_PIN
#define HW_MOTOR_B_ENCODER_IRQN         ENCODERB_INT_IRQN

/* ======================== MPU6050 (I2C) ======================== */
#define HW_MPU6050_I2C                  I2C_MPU_INST
#define HW_MPU6050_I2C_ADDRESS          (0x68U)
#define HW_MPU6050_INT_PORT             MPU6050_INT_PORT
#define HW_MPU6050_INT_PIN              MPU6050_INT_INT_PIN

/* ======================== OLED (Software SPI) ======================== */
#define HW_OLED_RST_PORT                OLED_RST_PORT
#define HW_OLED_RST_PIN                 OLED_RST_PIN
#define HW_OLED_DC_PORT                 OLED_DC_PORT
#define HW_OLED_DC_PIN                  OLED_DC_PIN
#define HW_OLED_SCL_PORT                OLED_SCL_PORT
#define HW_OLED_SCL_PIN                 OLED_SCL_PIN
#define HW_OLED_SDA_PORT                OLED_SDA_PORT
#define HW_OLED_SDA_PIN                 OLED_SDA_PIN

/* ======================== UARTs ======================== */
#define HW_USB_UART                     USB_INST
#define HW_K230_UART                    K230_INST
#define HW_K230_BAUD                    K230_BAUD_RATE

/* ======================== IR Line Sensors (6-ch) ======================== */
#define RYZD_SENSOR_COUNT               (6U)
#define IR_SENSOR_COUNT                 RYZD_SENSOR_COUNT
#define HW_IR_PORT(_n)                  HW_IR_PORT_I(_n)
#define HW_IR_PORT_I(_n)                IR_SENSORS_IR##_n##_PORT
#define HW_IR_PIN(_n)                   HW_IR_PIN_I(_n)
#define HW_IR_PIN_I(_n)                 IR_SENSORS_IR##_n##_PIN
#define HW_IR_RAW(_n)                   HW_GPIO_READ(HW_IR_PORT(_n), HW_IR_PIN(_n))
#define HW_IR_DETECTED(_n)             HW_IR_RAW(_n)

/* ======================== Buttons ======================== */
/* BLS: active HIGH when pressed (C07A V1.1) */
#define HW_BLS_KEY_PORT                 KEY_PORT
#define HW_BLS_KEY_PIN                  KEY_BLS_PIN
#define HW_BLS_ACTIVE_LEVEL             (1U)

/* RESET is hardware reset pin - cannot be used as GPIO.
 * Press RESET to reboot the MCU (full system reset). */

/* ======================== Status LED ======================== */
#define HW_STATUS_LED_PORT              LED_PORT
#define HW_STATUS_LED_PIN               LED_led_PIN

/* ======================== Stepper Motor (Balance) ======================== */
/* Step/Dir interface to stepper driver (A4988/DRV8825/TMC2209).
 * Defined in SysConfig as GPIO group "STEPPER": STEP=PB12, DIR=PB13, EN=PB4 */
#define HW_STEPPER_STEP_PORT            STEPPER_PORT
#define HW_STEPPER_STEP_PIN             STEPPER_STEP_PIN
#define HW_STEPPER_DIR_PORT             STEPPER_PORT
#define HW_STEPPER_DIR_PIN              STEPPER_DIR_PIN
#define HW_STEPPER_EN_PORT              STEPPER_PORT
#define HW_STEPPER_EN_PIN               STEPPER_EN_PIN

/* ======================== Battery ADC ======================== */
#define HW_BATTERY_ADC                  BATTERY_ADC_INST
#define HW_BATTERY_ADC_MEM              BATTERY_ADC_ADCMEM_0
#define HW_BATTERY_DIVIDER_GAIN         (11UL)

#endif /* BOARD_HARDWARE_H */
