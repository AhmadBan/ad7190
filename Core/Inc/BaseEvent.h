#ifndef BASE_EVENT_H
#define BASE_EVENT_H
#include "stdint.h"
typedef uint8_t Signal_t;

typedef struct EventTag
{                 /* the event structure */
    Signal_t sig; /* signal of the event */
} Event_t;

enum ReservedSignals
{	EMPTY_SIG=0,
    ENTRY_SIG = 1, /* signal for coding entry actions */
    EXIT_SIG,      /* signal for coding exit actions */
    INIT_SIG,      /* signal for coding initial transitions */
    USER_SIG       /* first signal that can be used in user applications */
};

void BaseEvent_constructor(void);
#endif
