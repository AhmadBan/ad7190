#ifndef ELEVATOR_INTERFACE_H
#define ELEVATOR_INTERFACE_H

#include <MOhmMeterEvent.h>
#include "./BaseState.h"



typedef struct mOhmState
{
    StateContext_t me;  /* derive from BaseState */
    float voltage1;      /* indicates the elevator current floor */
    float voltage2;       /* indicates the elevator target floor */
    uint32_t timeToCalibrate;

} MOhmMeter_t;

/* Constuctor to initialize context state object */
void mOhmMeter_costructor(MOhmMeter_t *me);

#endif
