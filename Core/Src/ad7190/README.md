# Getting Started

simply run **AD7190_Reset()** and wait for at least 500 micro seconds and then call **AD7190_Init** in your application and then check if it runs successfully

```c
//Resets AD7190 all registers to default value
	AD7190_Reset();
	//wait at least 500uS after reset due to datasheet
	HAL_Delay(1);
	//check if AD7190 is working correctly then initialize its registers
	if(!AD7190_Init()){
		//send some message to user in Console
		//error led blinking here
		HAL_Delay(1000);
		while(1);
	}

```

to find out if ADC conversion is ready, One must either polling ready pin or connect it to an interrupt generator. To start conversion in continous mode simply pull CS pin to low.

```c
//enable interrupt on MISO pin and reset interrupt flag
	enableInterruptOnMiso(GPIO_PIN_4);
	//pull cs low so MISO start toggling on data-ready
	ADI_PART_CS_LOW;
	/* USER CODE END 2 */

```

# Porting
To port driver implement custom code into **Communication.c** and **TIME.c** files.c

```c

uint8_t SPI_Read(uint8_t slaveDeviceId,uint8_t* address,uint8_t* data,uint8_t bytesNumber)
{
	ADI_PART_CS_LOW;
	if(HAL_SPI_TransmitReceive(&hspi1, address, data, bytesNumber, 10)!=HAL_OK){
		ADI_PART_CS_HIGH;
		return -1;
	}
	ADI_PART_CS_HIGH;
	return bytesNumber;
}


uint8_t SPI_Write(uint8_t slaveDeviceId,uint8_t* data,uint8_t bytesNumber)
{
	ADI_PART_CS_LOW;
	if(HAL_SPI_Transmit(&hspi1, data, bytesNumber,10)!=HAL_OK)
		return -1;
	ADI_PART_CS_HIGH;
	return bytesNumber;
}


void TIME_DelayMs(uint16_t msUnits)
{
    HAL_Delay(msUnits);
}
```