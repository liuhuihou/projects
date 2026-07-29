#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

/* Raw quadrature counts are updated by GROUP1_IRQHandler. */
extern volatile int32_t g_encoder_a_count;
extern volatile int32_t g_encoder_b_count;

void Encoder_Init(void);
void Encoder_TakeCounts(int32_t *encoder_a, int32_t *encoder_b);
float Encoder_CountsToRpm(int32_t count, uint32_t sample_ms);

#endif
