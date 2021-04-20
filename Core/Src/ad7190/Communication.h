/***************************************************************************//**
 *   @file   Communication.h
 *   @brief  Header file of Communication Driver.
 *   @author
********************************************************************************/

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "main.h"
/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/


/*! Initializes the SPI communication peripheral. */
uint8_t SPI_Init(uint8_t lsbFirst,
                       uint32_t clockFreq,
                       uint8_t clockPol,
                       uint8_t clockEdg);

/*! Initializes the SPI communication peripheral. */
uint8_t SPI_Init(uint8_t lsbFirst,
                       uint32_t clockFreq,
                       uint8_t clockPol,
                       uint8_t clockEdg);

/*! Reads data from SPI. */
uint8_t SPI_Read(uint8_t slaveDeviceId,
					   uint8_t *address,
                       uint8_t* data,
                       uint8_t bytesNumber);

/*! Writes data to SPI. */
uint8_t SPI_Write(uint8_t slaveDeviceId,
                        uint8_t* data,
                        uint8_t bytesNumber);

#endif /* _COMMUNICATION_H_ */
