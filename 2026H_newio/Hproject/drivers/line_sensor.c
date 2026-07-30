#include "line_sensor.h"
#include "board_hardware.h"

/* The API hands the channel mask back in a uint8_t. */
#if (LINE_SENSOR_COUNT > 8U)
#error "LINE_SENSOR_COUNT above 8 does not fit the uint8_t channel mask"
#endif

/* Spin limit on each bus wait. At 400 kHz a byte takes ~25 us, so a healthy
 * transfer clears in a few hundred iterations; this bounds a dead or stuck
 * bus to well under a millisecond of main-loop time instead of hanging. */
#define LINE_I2C_TIMEOUT_LOOPS  (20000U)

static uint8_t s_filter_count[LINE_SENSOR_COUNT];
static uint8_t s_filtered;
/* Written by Poll() in the main loop, read by Update() in the control ISR.
 * A single byte, so the handoff needs no lock - the ISR either sees the
 * previous sample or the new one, never a mix. */
static volatile uint8_t s_raw;
static uint8_t s_fail_count;

static uint8_t i2c_wait_idle(void)
{
    uint32_t guard = LINE_I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(HW_LINE_I2C) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--guard == 0U) return 0U;
    }
    return 1U;
}

/* Register read: address byte as a short write, STOP, then a read transfer.
 * Hiwonder's own Arduino driver does exactly this (write() then
 * requestFrom()), so the sensor expects the stop between the two phases
 * rather than a repeated start. */
static uint8_t i2c_read_reg(uint8_t reg, uint8_t *dst, uint32_t len)
{
    uint32_t guard;
    uint32_t i;

    if (!i2c_wait_idle()) return 0U;

    DL_I2C_flushControllerTXFIFO(HW_LINE_I2C);
    DL_I2C_fillControllerTXFIFO(HW_LINE_I2C, &reg, 1U);
    DL_I2C_startControllerTransfer(HW_LINE_I2C, HW_LINE_SENSOR_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    guard = LINE_I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(HW_LINE_I2C) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (--guard == 0U) return 0U;
    }
    /* ERROR covers a NACK on the address byte, i.e. no sensor on the bus. */
    if ((DL_I2C_getControllerStatus(HW_LINE_I2C) &
         DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        return 0U;
    }
    if (!i2c_wait_idle()) return 0U;

    DL_I2C_startControllerTransfer(HW_LINE_I2C, HW_LINE_SENSOR_I2C_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (i = 0U; i < len; ++i) {
        guard = LINE_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(HW_LINE_I2C)) {
            if (--guard == 0U) return 0U;
            if ((DL_I2C_getControllerStatus(HW_LINE_I2C) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                return 0U;
            }
        }
        dst[i] = DL_I2C_receiveControllerData(HW_LINE_I2C);
    }
    return 1U;
}
/* Bring the sensor's bit order into this project's convention: highest
 * active bit = leftmost channel, bit 0 = rightmost. */
static uint8_t normalise_bits(uint8_t v)
{
#if (HW_LINE_SENSOR_BIT0_IS_LEFT != 0U)
    uint8_t out = 0U;
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_COUNT; ++i) {
        if ((v & (uint8_t)(1U << i)) != 0U) {
            out |= (uint8_t)(1U << (LINE_SENSOR_COUNT - 1U - i));
        }
    }
    return out;
#else
    return (uint8_t)(v & LINE_SENSOR_ALL_MASK);
#endif
}

void LineSensor_Init(void)
{
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_COUNT; ++i) {
        s_filter_count[i] = 0U;
    }
    s_filtered = 0U;
    s_raw = 0U;
    /* Start at the limit so the sensor only counts as online once a read has
     * actually succeeded, rather than reporting healthy before the first
     * transfer has happened. */
    s_fail_count = LINE_SENSOR_FAIL_LIMIT;
}

uint8_t LineSensor_Poll(void)
{
    uint8_t state = 0U;

    if (i2c_read_reg(HW_LINE_SENSOR_REG_STATE, &state, 1U)) {
        s_raw = normalise_bits(state);
        s_fail_count = 0U;
        return 1U;
    }

    /* Hold the last good sample for a few failures, then fall back to
     * all-white. Zero is the line-lost code the follower already handles by
     * holding and decaying the previous correction, which is the right
     * behaviour for a sensor that has dropped off the bus. */
    if (s_fail_count < LINE_SENSOR_FAIL_LIMIT) {
        ++s_fail_count;
        if (s_fail_count >= LINE_SENSOR_FAIL_LIMIT) {
            s_raw = 0U;
        }
    }
    return 0U;
}

uint8_t LineSensor_IsOnline(void)
{
    return (s_fail_count < LINE_SENSOR_FAIL_LIMIT) ? 1U : 0U;
}
void LineSensor_Update(void)
{
    const uint8_t raw = s_raw;
    uint32_t i;
    for (i = 0U; i < LINE_SENSOR_COUNT; ++i) {
        const uint8_t mask = (uint8_t)(1U << (LINE_SENSOR_COUNT - 1U - i));
        if ((raw & mask) != 0U) {
            if (s_filter_count[i] < LINE_SENSOR_FILTER_N) ++s_filter_count[i];
            if (s_filter_count[i] >= LINE_SENSOR_FILTER_N) s_filtered |= mask;
        } else {
            if (s_filter_count[i] > 0U) --s_filter_count[i];
            if (s_filter_count[i] == 0U) s_filtered &= (uint8_t)~mask;
        }
    }
}

uint8_t LineSensor_ReadRaw(void) { return s_raw; }
uint8_t LineSensor_Read(void) { return s_filtered; }

uint8_t LineSensor_GetSteeringError(float *error)
{
    const uint8_t state = s_filtered;
    uint8_t started = 0U;
    uint8_t ended = 0U;
    int sum = 0;
    int active_count = 0;
    uint32_t i;

    if (error == 0) return 0U;
    *error = 0.0f;

    /* No channel sees black: the line is lost. The caller must decide what
     * to do (hold the previous steering correction rather than go straight). */
    if (state == 0U) return 0U;

    /* Every channel black: a perpendicular bar such as the start/stop line.
     * Report a valid zero error so the car drives straight across it. */
    if (state == LINE_SENSOR_ALL_MASK) return 1U;

    for (i = 0U; i < LINE_SENSOR_COUNT; ++i) {
        const uint8_t mask = (uint8_t)(1U << (LINE_SENSOR_COUNT - 1U - i));
        const uint8_t black = ((state & mask) != 0U) ? 1U : 0U;
        if (black != 0U) {
            if (ended != 0U) return 0U;
            started = 1U;
            /* Half-channel units, symmetric about centre and odd-valued so
             * no channel sits exactly at zero: 6 channels give
             * -5,-3,-1,1,3,5 (unchanged from the GPIO version) and 8 give
             * -7..7. Wider boards therefore report proportionally larger
             * errors, so LINE_KP scales with the physical array width rather
             * than needing a new gain per channel count. */
            sum += (int)(2U * i) - (int)(LINE_SENSOR_COUNT - 1U);
            ++active_count;
        } else if (started != 0U) {
            ended = 1U;
        }
    }

    *error = (float)sum / (float)active_count;
    return 1U;
}
