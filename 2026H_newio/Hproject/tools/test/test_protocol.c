/*
 * Host harness that drives the real MCU-side protocol parser and balance PID
 * from a text script on stdin, so tools/test/test_protocol.py can check them
 * against the K230's own encoder in k230D/protocol.py.
 *
 * Nothing here is compiled into the firmware. Build it with:
 *   gcc -I../../drivers -I../../control -Istubs test_protocol.c stubs/stubs.c \
 *       ../../drivers/camera_uart.c ../../control/balance_controller.c -o test
 *
 * Script commands, one per line:
 *   crc   <hex...>            print CRC8 over the given bytes
 *   feed  <now_ms> <hex...>   push bytes through Camera_FeedByte
 *   data                      print the current CameraData
 *   stats                     print the link counters
 *   fresh <now_ms> <timeout>  print Camera_IsDataFresh / Camera_IsBallUsable
 *   send  <cmd> <arg>         Camera_SendCommand, print the captured TX bytes
 *   ping  <arg>               Camera_Ping, print the captured TX bytes
 *   enable / disable          Balance_Enable / Balance_Disable
 *   target <mm>               Balance_SetTarget
 *   tick  <now_ms>            Balance_Tick, print tracking/error/output/stepper
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_uart.h"
#include "balance_controller.h"
#include "control_config.h"
#include "stepper_driver.h"
#include "ti_msp_dl_config.h"

extern int32_t stub_stepper_speed;
extern int     stub_stepper_enabled;

/* Parse whitespace-separated hex bytes from *cursor into out. */
static uint32_t parse_hex(char **cursor, uint8_t *out, uint32_t max)
{
    uint32_t n = 0U;
    while (n < max) {
        char *end;
        long v;
        while (**cursor == ' ' || **cursor == '\t') (*cursor)++;
        if (**cursor == '\0' || **cursor == '\n') break;
        v = strtol(*cursor, &end, 16);
        if (end == *cursor) break;
        out[n++] = (uint8_t)(v & 0xFF);
        *cursor = end;
    }
    return n;
}

static void print_tx(void)
{
    uint32_t i;
    printf("tx %u", (unsigned)stub_tx_len);
    for (i = 0U; i < stub_tx_len; ++i) {
        printf(" %02X", stub_tx_buf[i]);
    }
    printf("\n");
}

int main(void)
{
    char line[1024];

    Camera_Init();
    Balance_Init();
    Stepper_Init();

    while (fgets(line, sizeof(line), stdin) != NULL) {
        char *cursor = line;
        char cmd[32];
        int used = 0;

        if (sscanf(line, "%31s%n", cmd, &used) != 1) continue;
        if (cmd[0] == '#') continue;
        cursor = line + used;

        if (strcmp(cmd, "crc") == 0) {
            uint8_t buf[256];
            uint32_t n = parse_hex(&cursor, buf, sizeof(buf));
            printf("crc %02X\n", Camera_Crc8(buf, n));

        } else if (strcmp(cmd, "feed") == 0) {
            uint8_t buf[512];
            unsigned long now = strtoul(cursor, &cursor, 10);
            uint32_t n = parse_hex(&cursor, buf, sizeof(buf));
            uint32_t i;
            for (i = 0U; i < n; ++i) {
                Camera_FeedByte(buf[i], (uint32_t)now);
            }
            printf("fed %u\n", (unsigned)n);

        } else if (strcmp(cmd, "data") == 0) {
            const CameraData *d = Camera_GetData();
            printf("data pos=%d vel=%d flags=%02X seq=%u fps=%u valid=%u ts=%u\n",
                   (int)d->ball_pos_mm, (int)d->ball_vel_mm_s,
                   (unsigned)d->flags, (unsigned)d->seq, (unsigned)d->fps,
                   (unsigned)d->valid, (unsigned)d->last_update_ms);

        } else if (strcmp(cmd, "stats") == 0) {
            const CameraStats *s = Camera_GetStats();
            printf("stats bytes=%u frames=%u pongs=%u crc_err=%u resync=%u "
                   "gaps=%u tx=%u\n",
                   (unsigned)s->rx_bytes, (unsigned)s->rx_frames,
                   (unsigned)s->rx_pongs, (unsigned)s->crc_errors,
                   (unsigned)s->resyncs, (unsigned)s->seq_gaps,
                   (unsigned)s->tx_frames);

        } else if (strcmp(cmd, "fresh") == 0) {
            unsigned long now = strtoul(cursor, &cursor, 10);
            unsigned long tmo = strtoul(cursor, &cursor, 10);
            printf("fresh fresh=%u usable=%u\n",
                   (unsigned)Camera_IsDataFresh((uint32_t)now, (uint32_t)tmo),
                   (unsigned)Camera_IsBallUsable((uint32_t)now, (uint32_t)tmo));

        } else if (strcmp(cmd, "send") == 0 || strcmp(cmd, "ping") == 0) {
            unsigned long a = strtoul(cursor, &cursor, 10);
            stub_tx_len = 0U;
            if (strcmp(cmd, "ping") == 0) {
                Camera_Ping((uint8_t)a);
            } else {
                unsigned long b = strtoul(cursor, &cursor, 10);
                Camera_SendCommand((uint8_t)a, (uint8_t)b);
            }
            print_tx();

        } else if (strcmp(cmd, "enable") == 0) {
            Balance_Enable();
            printf("enabled %u\n", (unsigned)Balance_IsEnabled());

        } else if (strcmp(cmd, "disable") == 0) {
            Balance_Disable();
            printf("enabled %u\n", (unsigned)Balance_IsEnabled());

        } else if (strcmp(cmd, "target") == 0) {
            Balance_SetTarget((int16_t)strtol(cursor, &cursor, 10));
            printf("target %d\n", (int)Balance_GetTarget());

        } else if (strcmp(cmd, "tick") == 0) {
            unsigned long now = strtoul(cursor, &cursor, 10);
            Balance_Tick((uint32_t)now);
            printf("tick track=%u err=%d out=%d step=%ld en=%d\n",
                   (unsigned)Balance_IsTracking(), (int)Balance_GetError(),
                   (int)Balance_GetOutput(), (long)stub_stepper_speed,
                   stub_stepper_enabled);

        } else if (strcmp(cmd, "reset") == 0) {
            /* Back to power-on state, so a case that cares about absolute
             * counter values does not inherit whatever the previous case
             * left behind. */
            Camera_Init();
            Balance_Init();
            Stepper_Init();
            stub_tx_len = 0U;
            printf("reset\n");

        } else if (strcmp(cmd, "config") == 0) {
            printf("config period=%u timeout=%u use_vel=%d kp=%g ki=%g kd=%g "
                   "ilim=%g olim=%d\n",
                   (unsigned)BALANCE_PERIOD_MS,
                   (unsigned)BALANCE_DATA_TIMEOUT_MS,
                   BALANCE_USE_CAMERA_VELOCITY,
                   (double)BALANCE_KP, (double)BALANCE_KI, (double)BALANCE_KD,
                   (double)BALANCE_INTEGRAL_LIMIT,
                   (int)BALANCE_OUTPUT_LIMIT);

        } else {
            printf("?%s\n", cmd);
        }
        fflush(stdout);
    }
    return 0;
}
