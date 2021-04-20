/***************************************************************************//**
 *   @file   TIME.h
 *   @brief  Header file of TIME Driver.
 *   @author
********************************************************************************/
#ifndef __TIME_H__
#define __TIME_H__

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
#include "main.h"
/*! Initializes the timer used in this driver. */
uint8_t TIME_Init(void);

/*! The timer begins to count in steps of microseconds(us) until the user calls a
stop measurement function. */
void TIME_StartMeasure(void);

/*! Stops the measurement process when the functions is called. */
uint32_t TIME_StopMeasure(void);

/*! Creates a delay of microseconds. */
void TIME_DelayUs(uint16_t usUnits);

/*! Creates a delay of milliseconds. */
void TIME_DelayMs(uint16_t msUnits);

#endif // __TIME_H__
