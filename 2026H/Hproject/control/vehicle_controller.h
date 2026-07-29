#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    CTRL_STOP = 0,
    CTRL_STRAIGHT,
    CTRL_LINE
} ControlMode;

void Control_Init(void);
void Control_Tick(void);
void Control_SetMode(ControlMode mode);
ControlMode Control_GetMode(void);
void Control_SetBaseSpeed(float rpm);
void Control_SetLineBias(float rpm);
float Control_GetLeftRpm(void);
float Control_GetRightRpm(void);
float Control_GetBaseSpeedRpm(void);
float Control_GetLeftTargetRpm(void);
float Control_GetRightTargetRpm(void);
int32_t Control_GetLeftDuty(void);
int32_t Control_GetRightDuty(void);
int Control_GetLineError(void);
uint8_t Control_GetLineState(void);
uint32_t Control_GetTickCount(void);

#endif
