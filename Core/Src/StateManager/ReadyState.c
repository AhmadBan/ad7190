
#include "../inc/mOhmMeterState.h"


/**
 * @brief  State handler for in ready state
 * @param  me: Pointer to a context structure
 *         that contains the extended state information that need to be preserved .
 * @param  e: Pointer to current event catched,
 *         that contains the extended event information and current event Signal .
 * @retval State: Status Handled,Ignored or Transient
 */
State mOhmMeter_ready(MOhmMeter_t *me, Event_t const *e)
{

	return 0;
}
