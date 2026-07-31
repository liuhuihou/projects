#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#include <stdint.h>

/*
 * K230D (ATK-DNK230D) vision link - MCU end.
 *
 * The K230 tracks the steel ball inside the tube and streams its position on
 * UART1 (PB6/PB7) at 115200 8N1. This module is the receive side of that link
 * plus a small command channel back the other way.
 *
 * The wire format is defined once, in k230D/protocol.py, and mirrored here.
 * Both ends must agree byte for byte; the round-trip test in
 * tools/test_protocol.c checks that they do.
 *
 * Uplink, K230 -> MCU, 12 bytes, MSG_BALL
 *   0  0xAA
 *   1  0x55
 *   2  TYPE   = 0x01
 *   3  LEN    = 7
 *   4  SEQ            increments per frame, wraps at 256
 *   5  FLAGS          CAMERA_FLAG_*
 *   6  pos_l          int16 little-endian, millimetres from the origin
 *   7  pos_h
 *   8  vel_l          int16 little-endian, millimetres per second
 *   9  vel_h
 *  10  FPS            camera loop rate, whole frames per second
 *  11  CRC8           over bytes 2..10
 *
 * Uplink, K230 -> MCU, 6 bytes, MSG_PONG - reply to CAMERA_CMD_PING
 *   0..3 0xAA 0x55 0x02 0x01, 4 ARG (echoed), 5 CRC8 over bytes 2..4
 *
 * Downlink, MCU -> K230, 7 bytes, MSG_CMD
 *   0..3 0xAA 0x55 0x81 0x02, 4 CMD, 5 ARG, 6 CRC8 over bytes 2..5
 *
 * A frame arrives on every camera iteration, ball or no ball. That is what
 * makes "the link is dead" and "the ball is out of sight" distinguishable:
 * frames keep coming with CAMERA_FLAG_DETECTED clear in the second case. Only
 * check freshness to detect the first.
 *
 * CRC8 is poly 0x07, init 0x00, no reflection, no final xor. It replaced an
 * XOR checksum that was close to useless here: 0xAA ^ 0x55 is a constant, so
 * the XOR only ever protected pos_l ^ pos_h, and the same bit flipping in both
 * position bytes cancelled out undetected.
 */

#define CAMERA_SYNC0            (0xAAU)
#define CAMERA_SYNC1            (0x55U)

#define CAMERA_MSG_BALL         (0x01U)
#define CAMERA_MSG_PONG         (0x02U)
#define CAMERA_MSG_CMD          (0x81U)

#define CAMERA_BALL_PAYLOAD_LEN (7U)
#define CAMERA_BALL_FRAME_LEN   (12U)
#define CAMERA_PONG_PAYLOAD_LEN (1U)
#define CAMERA_PONG_FRAME_LEN   (6U)
#define CAMERA_CMD_PAYLOAD_LEN  (2U)
#define CAMERA_CMD_FRAME_LEN    (7U)

/* Longest frame we ever have to buffer. */
#define CAMERA_MAX_FRAME_LEN    (CAMERA_BALL_FRAME_LEN)

/* FLAGS bits in a MSG_BALL frame. */
#define CAMERA_FLAG_DETECTED    (0x01U)  /* real detection, not extrapolated */
#define CAMERA_FLAG_PREDICTED   (0x02U)  /* motion model contributed to pos   */
#define CAMERA_FLAG_VALID       (0x04U)  /* ruler locked, so pos is metric    */
#define CAMERA_FLAG_SATURATED   (0x08U)  /* pos or vel hit the int16 clamp    */

/* Commands the MCU can send. */
#define CAMERA_CMD_PING         (0x01U)
#define CAMERA_CMD_SET_ZERO     (0x02U)  /* ARG ignored: zero at current pos  */
#define CAMERA_CMD_SET_SEND     (0x03U)  /* ARG 0 stop stream, 1 resume       */

typedef struct {
    int16_t  ball_pos_mm;      /* along-tube position, mm from the origin */
    int16_t  ball_vel_mm_s;    /* K230-filtered velocity, mm/s           */
    uint8_t  flags;            /* CAMERA_FLAG_* from the last frame      */
    uint8_t  seq;              /* SEQ of the last frame                  */
    uint8_t  fps;              /* camera loop rate reported by the K230  */
    uint8_t  valid;            /* 1 once any well-formed frame arrived   */
    uint32_t last_update_ms;   /* app_time_ms() when that frame landed   */
} CameraData;

/* Link health. All counters are free-running and never reset after Init. */
typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_frames;        /* well-formed MSG_BALL frames            */
    uint32_t rx_pongs;
    uint32_t crc_errors;
    uint32_t resyncs;          /* bytes discarded hunting for 0xAA 0x55  */
    uint32_t seq_gaps;         /* dropped frames, from SEQ discontinuity  */
    uint32_t tx_frames;
} CameraStats;

void Camera_Init(void);

/*
 * Feed one received byte. now_ms is the caller's timestamp, used to stamp a
 * completed frame - passing it in keeps this driver free of any dependency on
 * the control layer's tick counter. Safe to call from the UART ISR.
 */
void Camera_FeedByte(uint8_t byte, uint32_t now_ms);

const CameraData *Camera_GetData(void);
const CameraStats *Camera_GetStats(void);

/*
 * Link is alive: a well-formed frame arrived within timeout_ms. Says nothing
 * about whether the ball was visible.
 */
uint8_t Camera_IsDataFresh(uint32_t now_ms, uint32_t timeout_ms);

/*
 * Position is usable for closed-loop control: link fresh, ruler locked, and
 * the ball actually seen or only briefly extrapolated. This is the one to gate
 * the balance PID on.
 */
uint8_t Camera_IsBallUsable(uint32_t now_ms, uint32_t timeout_ms);

/* Blocking transmit of one command frame. Returns 1 on success. */
uint8_t Camera_SendCommand(uint8_t cmd, uint8_t arg);
uint8_t Camera_Ping(uint8_t arg);

/* Shared with the K230 encoder; exposed for the round-trip test. */
uint8_t Camera_Crc8(const uint8_t *data, uint32_t len);

#endif
