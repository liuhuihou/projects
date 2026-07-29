#ifndef COMPETITION_MODE_H
#define COMPETITION_MODE_H

typedef enum {
    COMPETITION_Q2 = 0U,
    COMPETITION_Q3,
    COMPETITION_Q4,
    COMPETITION_Q5,
    COMPETITION_Q6,
    COMPETITION_MODE_COUNT
} CompetitionMode;

void CompetitionMode_Run(void);

#endif
