#include "../inc/BaseState.h"
#include "../inc/MOhmMeterEvent.h"

Event_t exitEvt, entryEvt;
void Base_init(StateContext_t *me, Event_t const *e)
{

    (*me->state)(me, e); /* execute the top-most initial transition */
                         /* enter the target */
    (void)(*me->state)(me, &entryEvt);
}
/* file qfsm_dis.c------------------------------------------------------*/
void Base_dispatch(StateContext_t *me, Event_t const *e)
{
    StateHandler s = me->state;
    State r = (*s)(me, e); /* call the event handler */
    if (r == EVT_TRAN)
    {
        (void)(*me->state)(me, &entryEvt); /*enter target*/
    }
}
