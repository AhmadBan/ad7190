/***************************************************************************//**
 *   @file   TIME.c
 *   @brief  Implementation of TIME Driver.
 *   @author
********************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "TIME.h"

/******************************************************************************/
/************************ Variables Declarations ******************************/
/******************************************************************************/


/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Initializes the timer used in this driver.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char TIME_Init(void)
{
    // Add your code here.
}

/***************************************************************************//**
 * @brief The timer begins to count in steps of microseconds(us) until the user
 *        calls a stop measurement function.
 *
 * @return None.
*******************************************************************************/
void TIME_StartMeasure(void)
{
    // Add your code here.
}

/***************************************************************************//**
 * @brief Stops the measurement process when the functions is called.
 *
 * @return Time(in microseconds) elapsed since the measurement began.
*******************************************************************************/
unsigned long TIME_StopMeasure(void)
{
    // Add your code here.
}

/***************************************************************************//**
 * @brief Creates a delay of microseconds.
 *
 * @return None.
*******************************************************************************/
void TIME_DelayUs(unsigned short usUnits)
{
    // Add your code here.
}

/***************************************************************************//**
 * @brief Creates a delay of milliseconds.
 *
 * @return None.
*******************************************************************************/
void TIME_DelayMs(unsigned short msUnits)
{
    HAL_Delay(msUnits);
}
