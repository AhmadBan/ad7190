#include "../inc/MOhmMeterState.h"
#include "../inc/MOhmMeterInterface.h"
enum{
	RESET = 0,
	SET
};

//----------------------------------Constructor----------------
void mOhmMeter_costructor(MOhmMeter_t *me)
{


	((StateContext_t *)me)->state = (StateHandler)mOhmMeter_initial;/*  Initialize StateHandler function */

	Base_dispatch((StateContext_t *)me, (void*)0); /* process Initialize StateHandler */
}
