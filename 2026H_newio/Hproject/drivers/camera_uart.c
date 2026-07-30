#include "camera_uart.h"
#include "board_hardware.h"

static CameraData s_data;
static uint8_t s_rx_buf[CAMERA_FRAME_LEN];
static uint8_t s_rx_idx;

void Camera_Init(void)
{
    s_data.ball_pos_mm = 0;
    s_data.valid = 0;
    s_data.last_update_ms = 0;
    s_rx_idx = 0;
}

void Camera_FeedByte(uint8_t byte)
{
    s_rx_buf[s_rx_idx] = byte;

    /* State machine for frame synchronization */
    if (s_rx_idx == 0) {
        if (byte == CAMERA_FRAME_HEADER1) s_rx_idx = 1;
        return;
    }
    if (s_rx_idx == 1) {
        if (byte == CAMERA_FRAME_HEADER2) {
            s_rx_idx = 2;
        } else {
            s_rx_idx = 0;
        }
        return;
    }

    s_rx_idx++;

    if (s_rx_idx >= CAMERA_FRAME_LEN) {
        /* Validate checksum: XOR of bytes 0..3 */
        uint8_t chk = s_rx_buf[0] ^ s_rx_buf[1] ^ s_rx_buf[2] ^ s_rx_buf[3];
        if (chk == s_rx_buf[4]) {
            s_data.ball_pos_mm = (int16_t)((uint16_t)s_rx_buf[2] |
                                           ((uint16_t)s_rx_buf[3] << 8));
            s_data.valid = 1;
            /* last_update_ms set externally by caller or ISR timestamp */
        }
        s_rx_idx = 0;
    }
}

const CameraData *Camera_GetData(void)
{
    return &s_data;
}

uint8_t Camera_IsDataFresh(uint32_t now_ms, uint32_t timeout_ms)
{
    if (!s_data.valid) return 0;
    return ((now_ms - s_data.last_update_ms) < timeout_ms) ? 1U : 0U;
}
