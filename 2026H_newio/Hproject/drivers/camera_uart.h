#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#include <stdint.h>

/*
 * K230D Box (正点原子) UART communication driver.
 * Protocol: The K230 sends ball position data frames via UART.
 * Frame format (to be confirmed with actual K230 firmware):
 *   Header(2B) + BallPos_mm(int16) + Checksum(1B)
 *   Header: 0xAA 0x55
 *   BallPos_mm: signed 16-bit, -120 to +120 (mm from center O)
 *   Checksum: XOR of all preceding bytes
 */

#define CAMERA_FRAME_HEADER1    (0xAAU)
#define CAMERA_FRAME_HEADER2    (0x55U)
#define CAMERA_FRAME_LEN        (5U)

typedef struct {
    int16_t ball_pos_mm;    /* Ball position relative to center O, in 0.1mm */
    uint8_t valid;          /* 1 if last frame was valid */
    uint32_t last_update_ms;
} CameraData;

void Camera_Init(void);

/* Call from UART3 RX interrupt or polling */
void Camera_FeedByte(uint8_t byte);

/* Get latest ball position data */
const CameraData *Camera_GetData(void);

/* Check if data is stale (timeout) */
uint8_t Camera_IsDataFresh(uint32_t now_ms, uint32_t timeout_ms);

#endif
