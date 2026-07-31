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
#define HW_MOTOR_A_ENCODER_IRQN         GPIOA_INT_IRQn
#define HW_MOTOR_B_ENCODER_PORT         ENCODERB_PORT
#define HW_MOTOR_B_ENCODER_A_PIN        ENCODERB_E2A_PIN
#define HW_MOTOR_B_ENCODER_B_PIN        ENCODERB_E2B_PIN
#define HW_MOTOR_B_ENCODER_IRQN         GPIOB_INT_IRQn

/* =================== Line Sensor Bus (Hiwonder I2C) =================== */
/* Wired through the unused MPU6050 socket (H5). PA0/PA1 are the only 5 V
 * tolerant pins on this device, so this is the one safe place for the
 * sensor's 5 V logic.
 * Pull-ups come from two places on a C07A V1.1 core board: R11/R12 (10k to
 * 5 V, on PA0A/PA1A) plus whatever the sensor board carries. Do not add more,
 * and never pull up to 3V3 - it would fight the on-board 5 V pull-ups. A V1.0
 * core board has no R11/R12, so there the sensor board is the only source.
 * H5 supplies 5 V (baseboard schematic REV 15.0, with a 0.1 uF decoupling
 * cap), which is what the sensor needs - all four wires go to H5, no separate
 * power feed required. H5 pin 7 is PA7 (the old MPU6050 INT), left unused. */
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
/* Channel count. Everything downstream - filter state, the weight table, the
 * all-black mask, the OLED readout - derives from it. The API returns the
 * channel bitmask in a uint8_t, so 8 is the ceiling.
 * Now a LineFollower_8CH v1.0 board (107 x 31.2 mm, 85 mA at 5 V). Its
 * protocol is identical to the 6-channel board's: same address, same three
 * register bases, same "write reg + STOP, then read" access. */
#define HW_LINE_SENSOR_COUNT            (8U)

/* 7-bit device address, fixed - the 8-channel manual lists 0x5D with no
 * strapping option. */
#define HW_LINE_SENSOR_I2C_ADDR         (0x5DU)

/* Register map, confirmed against the 8-channel protocol tables.
 * STATE returns one byte, one bit per channel, 1 = black line.
 * ANALOG returns COUNT 16-bit little-endian values (channels 1..8 at 6, 8,
 * 10 ... 20), THRESHOLD is writable with the same layout (22, 24 ... 36).
 * Only STATE is used in the control path; the other two are here for
 * calibration work. */
#define HW_LINE_SENSOR_REG_STATE        (5U)
#define HW_LINE_SENSOR_REG_ANALOG       (6U)
#define HW_LINE_SENSOR_REG_THRESHOLD    (22U)

/* Set to 1 if the sensor reports bit 0 as the leftmost channel. The driver
 * normalises to this project's convention (highest bit = leftmost), so
 * flipping this is all that is needed if the board is mounted reversed or
 * the register order turns out to be the other way round.
 *
 * 1 for this board: the vendor's own example extracts channel n as
 * (state >> n) & 1 and prints it as "State(n+1)", so bit 0 is S1, and the
 * silkscreen runs S1..S8 left to right (the Hiwonder logo and the KEY button
 * are at the S8 end). With the board mounted connector-edge-to-the-rear, S1
 * is on the car's left, so bit 0 is the leftmost channel and the driver has
 * to reverse it.
 * This also fixes the steering sign, not just the display: the raw and the
 * normalised mask are the same byte, so while this read 0 the weighted error
 * came out mirrored and the car corrected the wrong way. */
#define HW_LINE_SENSOR_BIT0_IS_LEFT     (1U)

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
 * it needs a real pulse train rather than a level.
 * CONFLICT: J10 is wired in parallel with the gamepad socket U3 (GND / 3V3 /
 * PB17 / PB16 / PA12 / PA27) - these three signals are shared. Never plug a
 * gamepad in while the stepper is on J10; both ends would drive the same
 * nets. The two sockets are mutually exclusive. */
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

/* Stepper encoder feedback on J6: A/B/Z GPIO plus PWM absolute position. */
#define HW_STEPPER_ENC_PORT             STEPPER_ENC_PORT
#define HW_STEPPER_ENC_A_PIN            STEPPER_ENC_A_PIN
#define HW_STEPPER_ENC_B_PIN            STEPPER_ENC_B_PIN
#define HW_STEPPER_ENC_Z_PIN            STEPPER_ENC_Z_PIN
#define HW_STEPPER_ENC_READ_A()         HW_GPIO_READ(HW_STEPPER_ENC_PORT, HW_STEPPER_ENC_A_PIN)
#define HW_STEPPER_ENC_READ_B()         HW_GPIO_READ(HW_STEPPER_ENC_PORT, HW_STEPPER_ENC_B_PIN)
#define HW_STEPPER_ENC_READ_Z()         HW_GPIO_READ(HW_STEPPER_ENC_PORT, HW_STEPPER_ENC_Z_PIN)

#define HW_STEPPER_POS_CAP_TIMER        STEPPER_POS_CAP_INST
#define HW_STEPPER_POS_CAP_IRQN         STEPPER_POS_CAP_INST_INT_IRQN
#define HW_STEPPER_POS_CAP_LOAD         STEPPER_POS_CAP_INST_LOAD_VALUE
#define HW_STEPPER_POS_CAP_TICK_HZ      (10000000UL)
#define HW_STEPPER_POS_CAP_START() \
    DL_TimerG_startCounter(HW_STEPPER_POS_CAP_TIMER)
#define HW_STEPPER_POS_CAP_RELOAD() \
    DL_TimerG_setTimerCount(HW_STEPPER_POS_CAP_TIMER, HW_STEPPER_POS_CAP_LOAD)
#define HW_STEPPER_POS_CAP_PERIOD_TICKS() \
    (HW_STEPPER_POS_CAP_LOAD - DL_TimerG_getCaptureCompareValue( \
        HW_STEPPER_POS_CAP_TIMER, DL_TIMER_CC_0_INDEX))
#define HW_STEPPER_POS_CAP_HIGH_TICKS() \
    (HW_STEPPER_POS_CAP_LOAD - DL_TimerG_getCaptureCompareValue( \
        HW_STEPPER_POS_CAP_TIMER, DL_TIMER_CC_1_INDEX))

/* ======================== Battery ADC ======================== */
#define HW_BATTERY_ADC                  BATTERY_ADC_INST
#define HW_BATTERY_ADC_MEM              BATTERY_ADC_ADCMEM_0
#define HW_BATTERY_DIVIDER_GAIN         (11UL)

#endif /* BOARD_HARDWARE_H */
