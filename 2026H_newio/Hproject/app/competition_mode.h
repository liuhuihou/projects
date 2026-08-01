#ifndef COMPETITION_MODE_H
#define COMPETITION_MODE_H

#include <stdint.h>

/*
 * Competition mode state machine.
 *
 * BLS button:
 *   Q2 single-click = start the selected sub-question
 *   Q3-Q5 long-press 1 s = level, then single-click = start
 *   Q6 long-press 1 s = level, wait for a ball, then single-click = start
 *   double-click = switch to the next sub-question (only while idle)
 *   running single-click = stop; done single-click = clear result
 *
 * RESET is the hardware reset pin and reboots the MCU.
 */

typedef enum {
    COMP_Q2 = 0,   /* Pure line follow, 1 lap <= 20s */
    COMP_Q3,       /* Static ball control +-5cm */
    COMP_Q4,       /* Line A->B + ball center, <= 8s */
    COMP_Q5,       /* Full lap + ball center, <= 30s */
    COMP_Q6,       /* Full lap + ball any position, <= 30s */
    COMP_MODE_COUNT
} CompetitionQuestion;

typedef enum {
    STATE_IDLE = 0,   /* Select mode; Q3-Q6 wait for a long press */
    STATE_LEVELING,   /* Q3-Q6 moving the fixed startup pose to horizontal */
    STATE_READY,      /* Tube horizontal; Q6 also waits for a usable ball */
    STATE_RUNNING,    /* Task executing */
    STATE_DONE        /* Task finished, showing the result */
} CompetitionState;

void Competition_Init(void);

/* Call in main loop - handles button events and state transitions */
void Competition_Update(uint32_t now_ms);

/* Getters for display */
CompetitionQuestion Competition_GetMode(void);
CompetitionState Competition_GetState(void);
uint32_t Competition_GetElapsedMs(void);
uint8_t Competition_IsQ6BallHoldActive(void);

/* Force stop (e.g., from external trigger) */
void Competition_ForceStop(void);

#endif
