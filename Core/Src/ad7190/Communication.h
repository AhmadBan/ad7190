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
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg);

/*! Initializes the SPI communication peripheral. */
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg);

/*! Reads data from SPI. */
unsigned char SPI_Read(unsigned char slaveDeviceId,
					   uint8_t *address,
                       unsigned char* data,
                       unsigned char bytesNumber);

/*! Writes data to SPI. */
unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber);

#endif /* _COMMUNICATION_H_ */
