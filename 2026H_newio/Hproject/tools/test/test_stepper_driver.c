#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "stepper_driver.h"
#include "board_hardware.h"

uint8_t stub_stepper_timer_running;
uint8_t stub_stepper_enable_level;
uint8_t stub_stepper_direction_level;
uint32_t stub_stepper_load;
uint32_t stub_stepper_compare;
uint32_t stub_stepper_pending_interrupt;

void stub_gpio_write(void *port, uint32_t pin, uint8_t value)
{
    (void)pin;
    if (port == HW_STEPPER_EN_PORT) {
        stub_stepper_enable_level = value;
    } else if (port == HW_STEPPER_DIR_PORT) {
        stub_stepper_direction_level = value;
    }
}

void DL_TimerG_setCaptCompUpdateMethod(void *timer, uint32_t method,
                                      uint32_t channel)
{
    (void)timer;
    (void)method;
    (void)channel;
}

void DL_TimerG_setCaptureCompareValue(void *timer, uint32_t value,
                                     uint32_t channel)
{
    (void)timer;
    (void)channel;
    stub_stepper_compare = value;
}

void DL_TimerG_stopCounter(void *timer)
{
    (void)timer;
    stub_stepper_timer_running = 0U;
}

void DL_Timer_disableShadowFeatures(void *timer) { (void)timer; }

void DL_TimerG_setLoadValue(void *timer, uint32_t value)
{
    (void)timer;
    stub_stepper_load = value;
}

void DL_TimerG_setTimerCount(void *timer, uint32_t value)
{
    (void)timer;
    (void)value;
}

void DL_TimerG_enableShadowFeatures(void *timer) { (void)timer; }

void DL_TimerG_startCounter(void *timer)
{
    (void)timer;
    stub_stepper_timer_running = 1U;
}

void DL_TimerG_enableInterrupt(void *timer, uint32_t interrupt)
{
    (void)timer;
    (void)interrupt;
}

uint32_t DL_TimerG_getPendingInterrupt(void *timer)
{
    (void)timer;
    return stub_stepper_pending_interrupt;
}

void NVIC_SetPriority(int irqn, uint32_t priority)
{
    (void)irqn;
    (void)priority;
}

void NVIC_ClearPendingIRQ(int irqn) { (void)irqn; }
void NVIC_EnableIRQ(int irqn) { (void)irqn; }

void TIMG7_IRQHandler(void);

static void emit_step(void)
{
    stub_stepper_pending_interrupt = DL_TIMER_IIDX_ZERO;
    TIMG7_IRQHandler();
}

int main(void)
{
    int i;

    Stepper_Init();
    assert(stub_stepper_timer_running == 0U);
    assert(stub_stepper_enable_level == 0U);

    Stepper_ResetPosition();
    Stepper_Enable();
    Stepper_SetSpeed(300);
    assert(stub_stepper_enable_level == 1U);
    assert(stub_stepper_direction_level == 1U);
    assert(stub_stepper_timer_running == 1U);
    assert(stub_stepper_load == 3332U);

    for (i = 0; i < 200; ++i) emit_step();
    assert(Stepper_GetPosition() == 200);
    assert(stub_stepper_timer_running == 1U);

    Stepper_SetSpeed(-300);
    assert(stub_stepper_direction_level == 0U);
    assert(stub_stepper_timer_running == 1U);
    for (i = 0; i < 400; ++i) emit_step();
    assert(Stepper_GetPosition() == -200);
    assert(stub_stepper_timer_running == 1U);
    Stepper_SetSpeed(0);
    assert(stub_stepper_timer_running == 0U);
    assert(stub_stepper_compare == 0U);

    puts("stepper driver basic tests passed");
    return 0;
}
