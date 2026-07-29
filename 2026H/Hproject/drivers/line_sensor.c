#include "line_sensor.h"

#include "board_hardware.h"

/* Each channel needs LINE_SENSOR_FILTER_N agreeing samples to change state. */
static uint8_t s_filter_count[RYZD_SENSOR_COUNT];
static uint8_t s_filtered;

void LineSensor_Init(void)
{
    uint32_t i;

    for (i = 0U; i < RYZD_SENSOR_COUNT; ++i) {
        s_filter_count[i] = 0U;
    }
    s_filtered = 0U;
}

/* Bit 5 is the leftmost physical channel; bit 0 is the rightmost.
 * The raw level is preserved: 1 = black, 0 = white. */
uint8_t LineSensor_ReadRaw(void)
{
    uint8_t state = 0U;

    if (HW_IR_RAW(1)) state |= (uint8_t)(1U << 5);
    if (HW_IR_RAW(2)) state |= (uint8_t)(1U << 4);
    if (HW_IR_RAW(3)) state |= (uint8_t)(1U << 3);
    if (HW_IR_RAW(4)) state |= (uint8_t)(1U << 2);
    if (HW_IR_RAW(5)) state |= (uint8_t)(1U << 1);
    if (HW_IR_RAW(6)) state |= (uint8_t)(1U << 0);

    return state;
}

void LineSensor_Update(void)
{
    const uint8_t raw = LineSensor_ReadRaw();
    uint32_t i;

    for (i = 0U; i < RYZD_SENSOR_COUNT; ++i) {
        const uint8_t mask = (uint8_t)(1U << (5U - i));

        if ((raw & mask) != 0U) {
            if (s_filter_count[i] < LINE_SENSOR_FILTER_N) {
                ++s_filter_count[i];
            }
            if (s_filter_count[i] >= LINE_SENSOR_FILTER_N) {
                s_filtered |= mask;
            }
        } else {
            if (s_filter_count[i] > 0U) {
                --s_filter_count[i];
            }
            if (s_filter_count[i] == 0U) {
                s_filtered &= (uint8_t)~mask;
            }
        }
    }
}

uint8_t LineSensor_Read(void)
{
    return s_filtered;
}

uint8_t LineSensor_GetSteeringError(float *error)
{
    /* RYZD CH2 through CH7 are six evenly spaced physical channels. */
    static const int8_t weights[RYZD_SENSOR_COUNT] = {-5, -3, -1, 1, 3, 5};
    const uint8_t state = s_filtered;
    uint8_t started = 0U;
    uint8_t ended = 0U;
    int sum = 0;
    int active_count = 0;
    uint32_t i;

    if (error == 0) {
        return 0U;
    }
    *error = 0.0f;

    /* No black channel and full coverage are neutral straight states. */
    if (state == 0U || state == 0x3FU) {
        return 0U;
    }

    /* A steerable state contains exactly one contiguous black group. */
    for (i = 0U; i < RYZD_SENSOR_COUNT; ++i) {
        const uint8_t mask = (uint8_t)(1U << (5U - i));
        const uint8_t black = ((state & mask) != 0U) ? 1U : 0U;

        if (black != 0U) {
            if (ended != 0U) {
                return 0U;
            }
            started = 1U;
            sum += weights[i];
            ++active_count;
        } else if (started != 0U) {
            ended = 1U;
        }
    }

    *error = (float)sum / (float)active_count;
    return 1U;
}
