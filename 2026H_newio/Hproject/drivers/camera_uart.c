#include "camera_uart.h"
#include "board_hardware.h"
#include "ti_msp_dl_config.h"

/*
 * Receive is a byte-at-a-time state machine so it can run straight out of the
 * UART ISR with no buffering above it. s_frame_len doubles as the parser state:
 * 0 = hunting for SYNC0, 1 = SYNC0 seen, 2.. = collecting a known frame.
 */

static CameraData s_data;
static CameraStats s_stats;

static uint8_t s_buf[CAMERA_MAX_FRAME_LEN];
static uint8_t s_frame_len;      /* bytes collected so far                   */
static uint8_t s_expect_len;     /* total length of the frame being read     */
static uint8_t s_have_seq;       /* 0 until the first frame sets the SEQ ref */

uint8_t Camera_Crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0U;
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((uint8_t)(crc << 1) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

void Camera_Init(void)
{
    s_data.ball_pos_mm = 0;
    s_data.ball_vel_mm_s = 0;
    s_data.flags = 0U;
    s_data.seq = 0U;
    s_data.fps = 0U;
    s_data.valid = 0U;
    s_data.last_update_ms = 0U;

    s_stats.rx_bytes = 0U;
    s_stats.rx_frames = 0U;
    s_stats.rx_pongs = 0U;
    s_stats.crc_errors = 0U;
    s_stats.resyncs = 0U;
    s_stats.seq_gaps = 0U;
    s_stats.tx_frames = 0U;

    s_frame_len = 0U;
    s_expect_len = 0U;
    s_have_seq = 0U;
}

/*
 * Abandon the frame under construction and reconsider `byte` as a possible
 * first sync byte, rather than discarding it with the rest.
 *
 * The rescan matters: noise whose tail happens to be 0xAA 0x55 arms the parser,
 * and then the next real frame's own leading 0xAA arrives where a type byte was
 * expected. Dropping that byte outright loses a frame that was perfectly well
 * formed - and at 27 fps a lost frame is a 37 ms hole in the control loop.
 */
static void resync_on(uint8_t byte)
{
    s_stats.resyncs++;
    if (byte == CAMERA_SYNC0) {
        s_buf[0] = byte;
        s_frame_len = 1U;
    } else {
        s_frame_len = 0U;
    }
    s_expect_len = 0U;
}

/* Commit a validated MSG_BALL frame to s_data. */
static void accept_ball_frame(uint32_t now_ms)
{
    uint8_t seq = s_buf[4];

    if (s_have_seq != 0U) {
        /* Unsigned 8-bit difference wraps correctly, so this counts drops
         * across the 255->0 boundary without a special case. A difference of
         * exactly 1 is the normal case. */
        uint8_t step = (uint8_t)(seq - s_data.seq);
        if (step > 1U) {
            s_stats.seq_gaps += (uint32_t)(step - 1U);
        }
    }
    s_have_seq = 1U;

    s_data.seq = seq;
    s_data.flags = s_buf[5];
    s_data.ball_pos_mm = (int16_t)((uint16_t)s_buf[6] |
                                   ((uint16_t)s_buf[7] << 8));
    s_data.ball_vel_mm_s = (int16_t)((uint16_t)s_buf[8] |
                                     ((uint16_t)s_buf[9] << 8));
    s_data.fps = s_buf[10];
    s_data.valid = 1U;
    s_data.last_update_ms = now_ms;
    s_stats.rx_frames++;
}

void Camera_FeedByte(uint8_t byte, uint32_t now_ms)
{
    s_stats.rx_bytes++;

    /* Hunting for the first sync byte. */
    if (s_frame_len == 0U) {
        if (byte == CAMERA_SYNC0) {
            s_buf[0] = byte;
            s_frame_len = 1U;
        } else {
            s_stats.resyncs++;
        }
        return;
    }

    /* Second sync byte. A repeated 0xAA keeps us armed rather than dropping
     * back to hunting - otherwise the pattern AA AA 55 ... would lose a frame
     * that was in fact perfectly framed after the stutter. */
    if (s_frame_len == 1U) {
        if (byte == CAMERA_SYNC1) {
            s_buf[1] = byte;
            s_frame_len = 2U;
        } else if (byte == CAMERA_SYNC0) {
            s_stats.resyncs++;
        } else {
            s_frame_len = 0U;
            s_stats.resyncs++;
        }
        return;
    }

    /* Type byte decides how many bytes still to come. */
    if (s_frame_len == 2U) {
        if (byte == CAMERA_MSG_BALL) {
            s_expect_len = CAMERA_BALL_FRAME_LEN;
        } else if (byte == CAMERA_MSG_PONG) {
            s_expect_len = CAMERA_PONG_FRAME_LEN;
        } else {
            /* Unknown type: this was not a frame start after all. */
            resync_on(byte);
            return;
        }
        s_buf[2] = byte;
        s_frame_len = 3U;
        return;
    }

    /* Length byte must match what the type implies, or the sync was luck. */
    if (s_frame_len == 3U) {
        uint8_t want = (s_buf[2] == CAMERA_MSG_BALL) ? CAMERA_BALL_PAYLOAD_LEN
                                                     : CAMERA_PONG_PAYLOAD_LEN;
        if (byte != want) {
            resync_on(byte);
            return;
        }
        s_buf[3] = byte;
        s_frame_len = 4U;
        return;
    }

    s_buf[s_frame_len] = byte;
    s_frame_len++;

    if (s_frame_len < s_expect_len) {
        return;
    }

    /* Full frame in hand. CRC covers bytes 2 .. len-2, i.e. type, length and
     * payload but not the sync pair - the sync bytes are constant and carry no
     * information worth protecting. */
    {
        uint8_t crc = Camera_Crc8(&s_buf[2], (uint32_t)(s_expect_len - 3U));
        if (crc == s_buf[s_expect_len - 1U]) {
            if (s_buf[2] == CAMERA_MSG_BALL) {
                accept_ball_frame(now_ms);
            } else {
                s_stats.rx_pongs++;
            }
        } else {
            s_stats.crc_errors++;
        }
    }

    s_frame_len = 0U;
    s_expect_len = 0U;
}

const CameraData *Camera_GetData(void)
{
    return &s_data;
}

const CameraStats *Camera_GetStats(void)
{
    return &s_stats;
}

uint8_t Camera_IsDataFresh(uint32_t now_ms, uint32_t timeout_ms)
{
    if (s_data.valid == 0U) {
        return 0U;
    }
    /* Unsigned wrap makes this correct across a tick-counter rollover. */
    return ((uint32_t)(now_ms - s_data.last_update_ms) < timeout_ms) ? 1U : 0U;
}

uint8_t Camera_IsBallUsable(uint32_t now_ms, uint32_t timeout_ms)
{
    if (Camera_IsDataFresh(now_ms, timeout_ms) == 0U) {
        return 0U;
    }
    /* No ruler lock means the position is not in millimetres at all. */
    if ((s_data.flags & CAMERA_FLAG_VALID) == 0U) {
        return 0U;
    }
    /* Accept a short extrapolation: the K230 only extrapolates for a few
     * frames before it gives up and clears FLAG_VALID anyway. Requiring
     * FLAG_DETECTED here would chatter on every single dropped detection. */
    return 1U;
}

uint8_t Camera_SendCommand(uint8_t cmd, uint8_t arg)
{
    uint8_t frame[CAMERA_CMD_FRAME_LEN];
    uint32_t i;

    frame[0] = CAMERA_SYNC0;
    frame[1] = CAMERA_SYNC1;
    frame[2] = CAMERA_MSG_CMD;
    frame[3] = CAMERA_CMD_PAYLOAD_LEN;
    frame[4] = cmd;
    frame[5] = arg;
    frame[6] = Camera_Crc8(&frame[2], 4U);

    for (i = 0U; i < CAMERA_CMD_FRAME_LEN; ++i) {
        DL_UART_Main_transmitDataBlocking(HW_K230_UART, frame[i]);
    }
    s_stats.tx_frames++;
    return 1U;
}

uint8_t Camera_Ping(uint8_t arg)
{
    return Camera_SendCommand(CAMERA_CMD_PING, arg);
}
