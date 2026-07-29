#ifndef COMPETITION_MODE_H
#define COMPETITION_MODE_H

#include <stdint.h>

/*
 * Competition mode state machine.
 * Manages sub-question selection and execution.
 *
 * Button mapping (C07A):
 *   BLS:   double-click = switch mode, single-click = start
 *   RESET: double-click = switch mode, single-click = start (redundant)
 *
 * Note: RESET按键是用于重置的，在某些场景下也可作为辅助按键使用。
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
    STATE_IDLE = 0,   /* Waiting for mode selection */
    STATE_READY,      /* Mode selected, waiting for start */
    STATE_RUNNING,    /* Task executing */
    STATE_DONE        /* Task complete, showing result */
} CompetitionState;

void Competition_Init(void);

/* Call in main loop - handles button events and state transitions */
void Competition_Update(uint32_t now_ms);

/* Getters for display */
CompetitionQuestion Competition_GetMode(void);
CompetitionState Competition_GetState(void);
uint32_t Competition_GetElapsedMs(void);

/* Force stop (e.g., from external trigger) */
void Competition_ForceStop(void);

#endif
