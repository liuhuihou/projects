#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

/* Three consistent samples give 30 ms debounce at the 10 ms control period. */
#define LINE_SENSOR_FILTER_N (3U)

void LineSensor_Init(void);
void LineSensor_Update(void);
uint8_t LineSensor_ReadRaw(void);
uint8_t LineSensor_Read(void);
uint8_t LineSensor_GetSteeringError(float *error);

#endif
