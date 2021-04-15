#ifndef ELEVATOR_STATE_H
#define ELEVATOR_STATE_H

#include "./BaseState.h"
#include "./MOhmMeterEvent.h"
#include "./MOhmMeterInterface.h"







#define TRUE  1
#define FALSE 0




State mOhmMeter_initial(MOhmMeter_t *me, Event_t const *e);   /* StateHandler to handle initialize state */
State mOhmMeter_ready(MOhmMeter_t *me, Event_t const *e);     /* StateHandler to handle ready state */

#endif
