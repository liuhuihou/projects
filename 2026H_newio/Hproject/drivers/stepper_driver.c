#include "stepper_driver.h"
#include "board_hardware.h"

static volatile int32_t s_position;
static volatile int8_t s_dir;      /* +1, -1, or 0 when stopped */
static uint32_t s_freq_hz;         /* 0 = no pulse train */

/* Timer period for a given pulse rate. The counter ticks at
 * HW_STEPPER_PWM_CLK_HZ (1 MHz), and in EDGE_ALIGN_UP the output period is
 * (load + 1) ticks, so load = clk/freq - 1. */
static uint32_t load_for_freq(uint32_t freq_hz)
{
    uint32_t load = (HW_STEPPER_PWM_CLK_HZ / freq_hz);
    if (load == 0U) load = 1U;
    return load - 1U;
}

static void set_en(uint8_t on)
{
#if (HW_STEPPER_EN_ACTIVE_LEVEL != 0U)
    HW_GPIO_WRITE(HW_STEPPER_EN_PORT, HW_STEPPER_EN_PIN, on);
#else
    HW_GPIO_WRITE(HW_STEPPER_EN_PORT, HW_STEPPER_EN_PIN, (on != 0U) ? 0U : 1U);
#endif
}

static void pulses_stop(void)
{
    /* Drop the compare to 0 before halting so the output is left low rather
     * than frozen mid-pulse: with the counter stopped the pin would hold
     * whatever level it had, and a static high keeps the D36A input
     * optocoupler conducting for no reason. CC updates are shadowed to the
     * zero event, so switch to immediate for this one write. */
    DL_TimerG_setCaptCompUpdateMethod(HW_STEPPER_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, HW_STEPPER_PWM_CHANNEL);
    DL_TimerG_setCaptureCompareValue(HW_STEPPER_PWM_TIMER, 0U,
                                     HW_STEPPER_PWM_CHANNEL);
    DL_TimerG_stopCounter(HW_STEPPER_PWM_TIMER);
    s_freq_hz = 0U;
    s_dir = 0;
}
/* Start the train from stopped. The LOAD and CC registers are shadowed to
 * the timer's zero event, which never arrives while the counter is halted,
 * so shadowing is switched off for these two writes and restored before the
 * counter runs. Getting this wrong is silent: the timer would start on
 * whatever period it last held. */
static void pulses_start(uint32_t freq_hz, int8_t dir)
{
    const uint32_t load = load_for_freq(freq_hz);

    DL_Timer_disableShadowFeatures(HW_STEPPER_PWM_TIMER);
    DL_TimerG_setCaptCompUpdateMethod(HW_STEPPER_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, HW_STEPPER_PWM_CHANNEL);

    DL_TimerG_setLoadValue(HW_STEPPER_PWM_TIMER, load);
    DL_TimerG_setCaptureCompareValue(HW_STEPPER_PWM_TIMER, load / 2U,
                                     HW_STEPPER_PWM_CHANNEL);
    DL_TimerG_setTimerCount(HW_STEPPER_PWM_TIMER, 0U);

    DL_TimerG_setCaptCompUpdateMethod(HW_STEPPER_PWM_TIMER,
        DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, HW_STEPPER_PWM_CHANNEL);
    DL_TimerG_enableShadowFeatures(HW_STEPPER_PWM_TIMER);

    s_freq_hz = freq_hz;
    s_dir = dir;
    DL_TimerG_startCounter(HW_STEPPER_PWM_TIMER);
}

/* Retune the period while the train is running. Both writes are buffered to
 * the next zero event, so the pulse in flight finishes at its original width
 * and the new rate begins on a clean cycle - no truncated or stretched step
 * for the driver to mis-read as the balance loop adjusts speed. */
static void pulses_retune(uint32_t freq_hz)
{
    const uint32_t load = load_for_freq(freq_hz);
    DL_TimerG_setLoadValue(HW_STEPPER_PWM_TIMER, load);
    DL_TimerG_setCaptureCompareValue(HW_STEPPER_PWM_TIMER, load / 2U,
                                     HW_STEPPER_PWM_CHANNEL);
    s_freq_hz = freq_hz;
}
void Stepper_Init(void)
{
    s_position = 0;
    s_dir = 0;
    s_freq_hz = 0U;

    HW_GPIO_LOW(HW_STEPPER_DIR_PORT, HW_STEPPER_DIR_PIN);
    /* Disable() stops the pulse train before dropping the enable line. */
    Stepper_Disable();

    /* One interrupt per pulse, used only to count steps. At the 5 kHz ceiling
     * that is 5000 ISRs/s of a handful of instructions each - well under 1%
     * of the 80 MHz core - and it is the only way to know beam travel, since
     * TIMG7 has no repeat counter to divide the rate down. Priority 3 (the
     * lowest on this core) keeps it behind the control loop and the encoders. */
    NVIC_SetPriority(HW_STEPPER_PWM_IRQN, 3);
    DL_TimerG_enableInterrupt(HW_STEPPER_PWM_TIMER,
                              DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(HW_STEPPER_PWM_IRQN);
    NVIC_EnableIRQ(HW_STEPPER_PWM_IRQN);
}

void Stepper_Enable(void)
{
    set_en(1U);
}

void Stepper_Disable(void)
{
    /* Stop pulsing before releasing the coils. Cutting the enable while
     * pulses are still going leaves the driver's internal step counter out of
     * step with the motor's actual position. */
    pulses_stop();
    set_en(0U);
}
void Stepper_SetSpeed(int32_t steps_per_sec)
{
    uint32_t freq;
    int8_t dir;

    if (steps_per_sec > 0) {
        dir = 1;
        freq = (uint32_t)steps_per_sec;
    } else if (steps_per_sec < 0) {
        dir = -1;
        freq = (uint32_t)(-steps_per_sec);
    } else {
        pulses_stop();
        return;
    }

    if (freq > STEPPER_MAX_FREQ_HZ) freq = STEPPER_MAX_FREQ_HZ;
    if (freq < STEPPER_MIN_FREQ_HZ) {
        /* Slower than the timer can express; hold instead of creeping. */
        pulses_stop();
        return;
    }

    if (dir != s_dir) {
        /* Direction change: halt the train, settle DIR1, then restart. The
         * ATD5984 latches DIR on the pulse edge, so moving it mid-pulse could
         * put one microstep the wrong way. Restarting also resets the phase,
         * which matters when the balance loop crosses zero and reverses. */
        pulses_stop();
        HW_GPIO_WRITE(HW_STEPPER_DIR_PORT, HW_STEPPER_DIR_PIN,
                      (dir > 0) ? 1U : 0U);
        pulses_start(freq, dir);
    } else if (freq != s_freq_hz) {
        pulses_retune(freq);
    }
}

int32_t Stepper_GetPosition(void) { return s_position; }
void Stepper_ResetPosition(void) { s_position = 0; }

void PWM_STEPPER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(HW_STEPPER_PWM_TIMER) ==
        DL_TIMER_IIDX_ZERO) {
        /* One zero event per output period, i.e. one emitted pulse. */
        if (s_dir > 0) {
            ++s_position;
        } else if (s_dir < 0) {
            --s_position;
        }
    }
}
