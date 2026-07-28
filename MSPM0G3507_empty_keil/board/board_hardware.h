/*
 * Application-facing hardware interface for the S28A + C07A car.
 *
 * Pin mux and peripheral initialization remain in generated/ti_msp_dl_config.*.
 * This file only gives stable names and small access macros to application code.
 */
#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* Clock and timer resources. */
#define HW_CPU_CLOCK_HZ                 CPUCLK_FREQ
#define HW_MOTOR_PWM_TIMER              PWM_0_INST
#define HW_MOTOR_PWM_PERIOD_TICKS       (8000U)
#define HW_MOTOR_A_PWM_CHANNEL          GPIO_PWM_0_C0_IDX
#define HW_MOTOR_B_PWM_CHANNEL          GPIO_PWM_0_C1_IDX
#define HW_MOTOR_A_PWM_PORT             GPIO_PWM_0_C0_PORT
#define HW_MOTOR_A_PWM_PIN              GPIO_PWM_0_C0_PIN
#define HW_MOTOR_B_PWM_PORT             GPIO_PWM_0_C1_PORT
#define HW_MOTOR_B_PWM_PIN              GPIO_PWM_0_C1_PIN

/* Generic GPIO helpers. */
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

/* TB6612 direction inputs. Motor direction depends on the actual motor wiring. */
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

/* Compatibility names used by common TB6612 application code. */
#define AIN_PORT                        HW_MOTOR_A_DIR_PORT
#define AIN_AIN1_PIN                    HW_MOTOR_A_IN1_PIN
#define AIN_AIN2_PIN                    HW_MOTOR_A_IN2_PIN
#define BIN_PORT                        HW_MOTOR_B_DIR_PORT
#define BIN_BIN1_PIN                    HW_MOTOR_B_IN1_PIN
#define BIN_BIN2_PIN                    HW_MOTOR_B_IN2_PIN
#define AIN1_PORT                       HW_MOTOR_A_DIR_PORT
#define AIN1_PIN                        HW_MOTOR_A_IN1_PIN
#define AIN2_PORT                       HW_MOTOR_A_DIR_PORT
#define AIN2_PIN                        HW_MOTOR_A_IN2_PIN
#define BIN1_PORT                       HW_MOTOR_B_DIR_PORT
#define BIN1_PIN                        HW_MOTOR_B_IN1_PIN
#define BIN2_PORT                       HW_MOTOR_B_DIR_PORT
#define BIN2_PIN                        HW_MOTOR_B_IN2_PIN

/* Quadrature encoder inputs. Both phases are configured for GPIO interrupts. */
#define HW_MOTOR_A_ENCODER_PORT         ENCODERA_PORT
#define HW_MOTOR_A_ENCODER_A_PIN        ENCODERA_E1A_PIN
#define HW_MOTOR_A_ENCODER_B_PIN        ENCODERA_E1B_PIN
#define HW_MOTOR_A_ENCODER_IRQN         ENCODERA_INT_IRQN
#define HW_MOTOR_B_ENCODER_PORT         ENCODERB_PORT
#define HW_MOTOR_B_ENCODER_A_PIN        ENCODERB_E2A_PIN
#define HW_MOTOR_B_ENCODER_B_PIN        ENCODERB_E2B_PIN
#define HW_MOTOR_B_ENCODER_IRQN         ENCODERB_INT_IRQN

/* MPU6050 controller bus. AD0 low selects 0x68; AD0 high selects 0x69. */
#define HW_MPU6050_I2C                  I2C_MPU_INST
#define HW_MPU6050_SDA_PORT             GPIO_I2C_MPU_SDA_PORT
#define HW_MPU6050_SDA_PIN              GPIO_I2C_MPU_SDA_PIN
#define HW_MPU6050_SCL_PORT             GPIO_I2C_MPU_SCL_PORT
#define HW_MPU6050_SCL_PIN              GPIO_I2C_MPU_SCL_PIN
#define HW_MPU6050_INT_PORT             MPU6050_INT_PORT
#define HW_MPU6050_INT_PIN              MPU6050_INT_INT_PIN
#define HW_MPU6050_I2C_ADDRESS          (0x68U)
#define HW_MPU6050_I2C_ADDRESS_ALT      (0x69U)

/* Software-serial OLED interface on the S28A OLED connector. */
#define HW_OLED_RST_PORT                OLED_RST_PORT
#define HW_OLED_RST_PIN                 OLED_RST_PIN
#define HW_OLED_DC_PORT                 OLED_DC_PORT
#define HW_OLED_DC_PIN                  OLED_DC_PIN
#define HW_OLED_SCL_PORT                OLED_SCL_PORT
#define HW_OLED_SCL_PIN                 OLED_SCL_PIN
#define HW_OLED_SDA_PORT                OLED_SDA_PORT
#define HW_OLED_SDA_PIN                 OLED_SDA_PIN

/* UART resources. TX/RX names describe the MSPM0 side of each connection. */
#define HW_USB_UART                     USB_INST
#define HW_USB_UART_TX_PORT             GPIO_USB_TX_PORT
#define HW_USB_UART_TX_PIN              GPIO_USB_TX_PIN
#define HW_USB_UART_RX_PORT             GPIO_USB_RX_PORT
#define HW_USB_UART_RX_PIN              GPIO_USB_RX_PIN
#define HW_BLUETOOTH_UART               BLUETOOTH_INST
#define HW_BLUETOOTH_TX_PORT            GPIO_BLUETOOTH_TX_PORT
#define HW_BLUETOOTH_TX_PIN             GPIO_BLUETOOTH_TX_PIN
#define HW_BLUETOOTH_RX_PORT            GPIO_BLUETOOTH_RX_PORT
#define HW_BLUETOOTH_RX_PIN             GPIO_BLUETOOTH_RX_PIN
#define HW_BLUETOOTH_BAUD               BLUETOOTH_BAUD_RATE
#define HW_K230_UART                    K230_INST
#define HW_K230_TX_PORT                GPIO_K230_TX_PORT
#define HW_K230_TX_PIN                 GPIO_K230_TX_PIN
#define HW_K230_RX_PORT                GPIO_K230_RX_PORT
#define HW_K230_RX_PIN                 GPIO_K230_RX_PIN
#define HW_K230_BAUD                   K230_BAUD_RATE

/* Battery divider: 12 V -> 10 kOhm -> PA15 -> 1 kOhm -> GND. */
#define HW_BATTERY_ADC                 BATTERY_ADC_INST
#define HW_BATTERY_ADC_MEM             BATTERY_ADC_ADCMEM_0
#define HW_BATTERY_ADC_PORT            GPIO_BATTERY_ADC_C0_PORT
#define HW_BATTERY_ADC_PIN             GPIO_BATTERY_ADC_C0_PIN
#define HW_BATTERY_DIVIDER_TOP_OHM     (10000UL)
#define HW_BATTERY_DIVIDER_BOTTOM_OHM  (1000UL)
#define HW_BATTERY_DIVIDER_GAIN        (11UL)

/* Six connected RYZD physical channels: CH2 through CH7. */
#define RYZD_SENSOR_COUNT              (6U)
#define IR_SENSOR_COUNT                RYZD_SENSOR_COUNT
#define HW_IR_PORT(_n)                 HW_IR_PORT_I(_n)
#define HW_IR_PORT_I(_n)               IR_SENSORS_IR##_n##_PORT
#define HW_IR_PIN(_n)                  HW_IR_PIN_I(_n)
#define HW_IR_PIN_I(_n)                IR_SENSORS_IR##_n##_PIN
#define HW_IR_RAW(_n)                  HW_GPIO_READ(HW_IR_PORT(_n), HW_IR_PIN(_n))
/* Preserve the RYZD electrical level: 1 = black, 0 = white. */
#define HW_IR_DETECTED(_n)             HW_IR_RAW(_n)

#define HW_IR1_PORT                   IR_SENSORS_IR1_PORT
#define HW_IR1_PIN                    IR_SENSORS_IR1_PIN
#define HW_IR2_PORT                   IR_SENSORS_IR2_PORT
#define HW_IR2_PIN                    IR_SENSORS_IR2_PIN
#define HW_IR3_PORT                   IR_SENSORS_IR3_PORT
#define HW_IR3_PIN                    IR_SENSORS_IR3_PIN
#define HW_IR4_PORT                   IR_SENSORS_IR4_PORT
#define HW_IR4_PIN                    IR_SENSORS_IR4_PIN
#define HW_IR5_PORT                   IR_SENSORS_IR5_PORT
#define HW_IR5_PIN                    IR_SENSORS_IR5_PIN
#define HW_IR6_PORT                   IR_SENSORS_IR6_PORT
#define HW_IR6_PIN                    IR_SENSORS_IR6_PIN

/* Physical-channel aliases; HW_IR1..HW_IR6 remain compact software indexes. */
#define RYZD_CH2_PORT                 HW_IR1_PORT
#define RYZD_CH2_PIN                  HW_IR1_PIN
#define RYZD_CH3_PORT                 HW_IR2_PORT
#define RYZD_CH3_PIN                  HW_IR2_PIN
#define RYZD_CH4_PORT                 HW_IR3_PORT
#define RYZD_CH4_PIN                  HW_IR3_PIN
#define RYZD_CH5_PORT                 HW_IR4_PORT
#define RYZD_CH5_PIN                  HW_IR4_PIN
#define RYZD_CH6_PORT                 HW_IR5_PORT
#define RYZD_CH6_PIN                  HW_IR5_PIN
#define RYZD_CH7_PORT                 HW_IR6_PORT
#define RYZD_CH7_PIN                  HW_IR6_PIN
#define RYZD_CH2_DETECTED()           HW_IR_DETECTED(1)
#define RYZD_CH3_DETECTED()           HW_IR_DETECTED(2)
#define RYZD_CH4_DETECTED()           HW_IR_DETECTED(3)
#define RYZD_CH5_DETECTED()           HW_IR_DETECTED(4)
#define RYZD_CH6_DETECTED()           HW_IR_DETECTED(5)
#define RYZD_CH7_DETECTED()           HW_IR_DETECTED(6)

/* Board-local controls. */
#define HW_BLS_KEY_PORT                KEY_PORT
#define HW_BLS_KEY_PIN                 KEY_BLS_PIN
#define HW_S28A_KEY_PORT               S28A_BUTTON_PORT
#define HW_S28A_KEY_PIN                S28A_BUTTON_S28A_KEY_PIN
#define HW_STATUS_LED_PORT             LED_PORT
#define HW_STATUS_LED_PIN              LED_led_PIN

#endif /* HARDWARE_INTERFACE_H */
