#ifndef TEST_STEPPER_BOARD_HARDWARE_H
#define TEST_STEPPER_BOARD_HARDWARE_H

#include <stdint.h>

#define HW_STEPPER_PWM_TIMER            ((void *)0x1)
#define HW_STEPPER_PWM_CHANNEL          (1U)
#define HW_STEPPER_PWM_CLK_HZ           (1000000U)
#define HW_STEPPER_PWM_IRQN             (7)
#define HW_STEPPER_DIR_PORT             ((void *)0x2)
#define HW_STEPPER_DIR_PIN              (1U << 17)
#define HW_STEPPER_EN_PORT              ((void *)0x3)
#define HW_STEPPER_EN_PIN               (1U << 12)
#define HW_STEPPER_EN_ACTIVE_LEVEL      (1U)

#define PWM_STEPPER_INST_IRQHandler     TIMG7_IRQHandler

#define DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE (0U)
#define DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT  (1U)
#define DL_TIMER_INTERRUPT_ZERO_EVENT       (1U)
#define DL_TIMER_IIDX_ZERO                  (1U)

extern uint8_t stub_stepper_timer_running;
extern uint8_t stub_stepper_enable_level;
extern uint8_t stub_stepper_direction_level;
extern uint32_t stub_stepper_load;
extern uint32_t stub_stepper_compare;
extern uint32_t stub_stepper_pending_interrupt;

void stub_gpio_write(void *port, uint32_t pin, uint8_t value);
#define HW_GPIO_WRITE(_port, _pin, _value) \
    stub_gpio_write((_port), (_pin), (uint8_t)(_value))
#define HW_GPIO_LOW(_port, _pin) HW_GPIO_WRITE((_port), (_pin), 0U)

void DL_TimerG_setCaptCompUpdateMethod(void *timer, uint32_t method,
                                      uint32_t channel);
void DL_TimerG_setCaptureCompareValue(void *timer, uint32_t value,
                                     uint32_t channel);
void DL_TimerG_stopCounter(void *timer);
void DL_Timer_disableShadowFeatures(void *timer);
void DL_TimerG_setLoadValue(void *timer, uint32_t value);
void DL_TimerG_setTimerCount(void *timer, uint32_t value);
void DL_TimerG_enableShadowFeatures(void *timer);
void DL_TimerG_startCounter(void *timer);
void DL_TimerG_enableInterrupt(void *timer, uint32_t interrupt);
uint32_t DL_TimerG_getPendingInterrupt(void *timer);

void NVIC_SetPriority(int irqn, uint32_t priority);
void NVIC_ClearPendingIRQ(int irqn);
void NVIC_EnableIRQ(int irqn);

#endif
