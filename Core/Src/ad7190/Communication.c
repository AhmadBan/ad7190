/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author
 ********************************************************************************/


/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"
#include "main.h"
#include "AD7190.h"

extern SPI_HandleTypeDef hspi1;

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
 *******************************************************************************/
unsigned char SPI_Init(unsigned char lsbFirst,
		unsigned long clockFreq,
		unsigned char clockPol,
		unsigned char clockEdg)
{
	/* Add code here. */
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
 *******************************************************************************/
//TODO:add implementation for slaveid
unsigned char SPI_Read(unsigned char slaveDeviceId,uint8_t* address,unsigned char* data,
		unsigned char bytesNumber)
{




	ADI_PART_CS_LOW;
	if(HAL_SPI_TransmitReceive(&hspi1, address, data, bytesNumber, 10)!=HAL_OK){
		ADI_PART_CS_HIGH;
		return -1;
	}
	ADI_PART_CS_HIGH;



	return bytesNumber;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
 *******************************************************************************/
unsigned char SPI_Write(unsigned char slaveDeviceId,
		unsigned char* data,
		unsigned char bytesNumber)
{
	ADI_PART_CS_LOW;
	if(HAL_SPI_Transmit(&hspi1, data, bytesNumber,10)!=HAL_OK)
		return -1;
	ADI_PART_CS_HIGH;
	return bytesNumber;
}
