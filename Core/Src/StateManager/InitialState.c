#include "../inc/MOhmMeterState.h"

/*
* @brief  Initializes the Elevator starting state
* @param  me: pointer to a context structure
*         that contains the extended state information and current state handler.
* @param  e: Pointer to current event catched,
*         that contains the extended event information and current event Signal
*         not used here just to keep signature.
* @retval state status Handled,Ignored or Transient
*/
State mOhmMeter_initial(MOhmMeter_t *me, Event_t const *e)
{

//	if (me->status == 0)
//	{
//		if (me->betweenTwoFloor)
//		{
//			me->directionMovement = DOWN_DIRECTION;
//			return TRAN(&Elevator_move);
//		}
//		else
//		{
//			return TRAN(&Elevator_ready);
//		}
//	}
//	else
//	{
//		return TRAN(&Elevator_emergency);
//	}

}

