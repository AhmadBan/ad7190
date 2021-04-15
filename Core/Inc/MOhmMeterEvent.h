#ifndef ELEVATOR_EVENT_H
#define ELEVATOR_EVENT_H

#include "./BaseEvent.h"
#define MAILQUEUE_EVENTS 5

enum ElevatorSignals
{                             /* all signals for the Elevator FSM */
  FLOORBUTTON_SIG = USER_SIG, /* Push button to target floor */

  DOOR_SIG, /* Signal from parking door sensor opening */

  LIMITSWITCH_SIG, /* Signal from  limiter switch */

  HALSENSOR_SIG, /* Signal from HAL sensor */

  OVERWEIGHT_SIG,
  OVERTEMPERATURE_SIG

};


typedef struct LimitSWEvt
{
    Event_t ancestor;
    uint8_t status; /* 1 = UP Limit ,2 = DOWN Limit, 0 =None ,3 =both limit switch*/
} LimitSWEvt_t;

typedef struct OverWeightEvt
{
    Event_t ancestor;
    uint8_t status; /* 0 means no OverWeight 1 means OverWeight */
} OverWeightEvt_t;

typedef struct OverTemperatureEvt
{
    Event_t ancestor;
    uint8_t status; /* 0 means no OverTemperature 1 means OverTemperature */
} OverTemperatureEvt_t;

typedef struct FloorBtnEvt
{
    Event_t ancestor;
    uint8_t targetFloor;
} FloorBtnEvt_t;

typedef struct DoorEvt
{
    Event_t ancestor;
    uint8_t floor;
    uint8_t status; //0 close 1 open
} DoorEvt_t;

typedef struct HalSensorEvt
{
    Event_t ancestor;
    uint8_t status; /* 0 means not detected, 1 means Detected */
} HalSensorEvt_t;
/**  
 * Assign biggest size of all event as a constant size
 * dynamic memory allocation of events 
 **/
#define EVENT_SIZE size_of(DoorEvt_t)
#define EVENT_TYPE DoorEvt_t

void ElevatorEvent_constructor(void);
#endif
