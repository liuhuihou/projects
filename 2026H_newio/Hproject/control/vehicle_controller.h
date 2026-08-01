#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include <stdint.h>

typedef enum {
    CTRL_STOP = 0,
    CTRL_STRAIGHT,
    CTRL_LINE
} ControlMode;

typedef enum {
    CTRL_LINE_PROFILE_Q2_Q4 = 0,
    CTRL_LINE_PROFILE_Q5_Q6
} ControlLineProfile;

void Control_Init(void);
void Control_Tick(void);
void Control_SetMode(ControlMode mode);
ControlMode Control_GetMode(void);
void Control_SetLineProfile(ControlLineProfile profile);
ControlLineProfile Control_GetLineProfile(void);
void Control_SetBaseSpeed(float rpm);
/* Linearly ramp both wheel targets from zero to their requested values over
 * duration_ms. A duration of zero preserves the original immediate start. */
void Control_SetStartRamp(uint32_t duration_ms);
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

/* Distance tracking for lap detection */
int32_t Control_GetLeftDistance(void);
int32_t Control_GetRightDistance(void);
void Control_ResetDistance(void);

#endif
