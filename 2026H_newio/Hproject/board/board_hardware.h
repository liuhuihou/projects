/*
 * Hardware interface for H题 - 车载平衡滚球运动控制系统
 * Platform: C07A (MSPM0G3507SPTR, LQFP-48) + S28A baseboard
 *
 * Interfaces reworked away from the reference line-follower project:
 *   - Line sensor is a Hiwonder I2C board on I2C0 (PA0 SDA / PA1 SCL),
 *     replacing 6 parallel GPIO inputs. Channel count is configurable.
 *   - Stepper (D36A) ST1 pulse is hardware PWM on PB16; DIR1 = PB17,
 *     EN1 = PA12 (active HIGH).
 *   - K230D moved to UART1 on the adjacent pair PB6/PB7.
 * The MPU6050 and Bluetooth sockets are unused and now carry the line
 * sensor bus and the K230D link respectively.
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
#define HW_MOTOR_A_BRAKE() do {          \
    HW_GPIO_HIGH(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN1_PIN); \
    HW_GPIO_HIGH(HW_MOTOR_A_DIR_PORT, HW_MOTOR_A_IN2_PIN); \
} while (0)
#define HW_MOTOR_B_BRAKE() do {          \
    HW_GPIO_HIGH(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN1_PIN); \
    HW_GPIO_HIGH(HW_MOTOR_B_DIR_PORT, HW_MOTOR_B_IN2_PIN); \
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

/* =================== Line Sensor Bus (Hiwonder I2C) =================== */
/* Wired through the unused MPU6050 socket (H5). PA0/PA1 are the only 5 V
 * tolerant pins on this device, so this is the one safe place for the
 * sensor's 5 V logic. Pull-ups are on the sensor board.
 * H5's supply pin is ambiguous in the baseboard schematic (netlist text says
 * 3V3, the drawing shows 5 V), and the sensor needs 5 V - take power from J4
 * and borrow only SDA/SCL/GND from H5 until measured. */
#define HW_LINE_I2C                     LINE_I2C_INST
#define HW_LINE_I2C_SPEED_HZ            LINE_I2C_BUS_SPEED_HZ

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

/* ================== Line Sensor (Hiwonder, I2C) ================== */
/* Channel count is the one number to change when swapping the 6-channel
 * board for the 8-channel one. Everything downstream - filter state, the
 * weight table, the all-black mask, the OLED readout - derives from it.
 * The API returns the channel bitmask in a uint8_t, so 8 is the ceiling. */
#define HW_LINE_SENSOR_COUNT            (6U)

/* 7-bit device address. The 6- and 8-channel boards ship with the same
 * default; verify against the sticker if the board has been re-addressed. */
#define HW_LINE_SENSOR_I2C_ADDR         (0x5DU)

/* Register map. STATE returns one byte, one bit per channel, 1 = black line.
 * ANALOG returns COUNT 16-bit little-endian values, THRESHOLD is writable
 * with the same layout. Only STATE is used in the control path; the other
 * two are here for calibration work. */
#define HW_LINE_SENSOR_REG_STATE        (5U)
#define HW_LINE_SENSOR_REG_ANALOG       (6U)
#define HW_LINE_SENSOR_REG_THRESHOLD    (22U)

/* Set to 1 if the sensor reports bit 0 as the leftmost channel. The driver
 * normalises to this project's convention (highest bit = leftmost), so
 * flipping this is all that is needed if the board is mounted reversed or
 * the register order turns out to be the other way round. */
#define HW_LINE_SENSOR_BIT0_IS_LEFT     (0U)

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

/* ==================== Stepper Motor (Balance beam) ==================== */
/* D36A driver (ATD5984) on J10. ST1 is hardware PWM so the pulse train runs
 * without CPU involvement; DIR1 and EN1 are plain GPIO.
 *   ST1  = PB16 -> D36A ST1   (TIMG7_CCP1)
 *   DIR1 = PB17 -> D36A DIR1
 *   EN1  = PA12 -> D36A EN1
 * The D36A control header is single-ended (ST1/DIR1/EN1 + GND), not a
 * differential PUL+/PUL- pair: tie D36A GND to board GND. Leave the D36A 5 V
 * pin unconnected - the S28A has its own 12 V->5 V module and paralleling
 * two supplies makes them fight. ST1 is rising-edge triggered, which is why
 * it needs a real pulse train rather than a level. */
#define HW_STEPPER_PWM_TIMER            PWM_STEPPER_INST
#define HW_STEPPER_PWM_CHANNEL          GPIO_PWM_STEPPER_C1_IDX
#define HW_STEPPER_PWM_CLK_HZ           PWM_STEPPER_INST_CLK_FREQ
#define HW_STEPPER_PWM_IRQN             PWM_STEPPER_INST_INT_IRQN

#define HW_STEPPER_DIR_PORT             STEPPER_DIR1_PORT
#define HW_STEPPER_DIR_PIN              STEPPER_DIR1_PIN
#define HW_STEPPER_EN_PORT              STEPPER_EN1_PORT
#define HW_STEPPER_EN_PIN               STEPPER_EN1_PIN

/* EN1 drives the ATD5984 SLEEP input, so it is active HIGH: HIGH = driver
 * awake and holding torque, LOW = coils released. This is the opposite of
 * the A4988/DRV8825 convention the old code assumed. */
#define HW_STEPPER_EN_ACTIVE_LEVEL      (1U)

/* ======================== Battery ADC ======================== */
#define HW_BATTERY_ADC                  BATTERY_ADC_INST
#define HW_BATTERY_ADC_MEM              BATTERY_ADC_ADCMEM_0
#define HW_BATTERY_DIVIDER_GAIN         (11UL)

#endif /* BOARD_HARDWARE_H */
