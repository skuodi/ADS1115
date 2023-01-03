#include "ads1115.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define I2C_HANDLE &hi2c1

void ADS1115_I2C_Send(uint8_t devAddr, uint8_t * data, uint16_t size)
{
	HAL_I2C_Master_Transmit(I2C_HANDLE,devAddr,data,size,5);	
}

void ADS1115_I2C_Receive(uint8_t devAddr, uint8_t * data, uint16_t size)
{
	HAL_I2C_Master_Receive(I2C_HANDLE,devAddr,data,size,5);	
}


/**
 * @brief: 	ADS1115_Reset	- uses an I2C general call reset command to perform the equivalent of a power-on reset
 * @param:	adc				-	ADS1115 struct
 * @retval: none
 * */
void ADS1115_Reset(ADS1115* adc)
{
	uint8_t resetCommand = 0x06;
	ADS1115_I2C_Send(adc->address, &resetCommand, 1);
}

/**
 * @brief:	ADS1115_Init	-	initialises an ADS1115 device 
 * @param:	adc				-	the struct of the ADS1115 device to be initialized
 * @retval:	none
 * */
void ADS1115_Init(ADS1115* adc)
{
	uint16_t configValue = 0;	//variable holds config register values-to-be
	uint8_t buf[3] = {0};

	configValue |= (adc->opStat		<< 	15) 
				| (adc->inputMux 	<<	12) 
				| (adc->fullRange	<<	9) 
				| (adc->opMode 		<<	8) 
				| (adc->sampleRate	<<	5) 
				| (adc->compMode		<<	4) 
				| (adc->compPol		<<	3) 
				| (adc->compLatch	<<	2) 
				| adc->compQue		<<	0;


	buf[0] = ADS1115_REG_CONFIG;
	buf[1] = (configValue >> 8);		//MSB
	buf[2] = (configValue & 0xFF);		//LSB
	ADS1115_I2C_Send(adc->address,buf,3);

	buf[0] = ADS1115_REG_LO_TH;
	buf[1] = (adc->loThresh >> 8);		//MSB
	buf[2] = (adc->loThresh & 0xFF);		//LSB
	ADS1115_I2C_Send(adc->address,buf,3);

	buf[0] = ADS1115_REG_HI_TH;
	buf[1] = (adc->hiThresh >> 8);		//MSB
	buf[2] = (adc->hiThresh & 0xFF);		//LSB
	ADS1115_I2C_Send(adc->address,buf,3);
}


/**
 * @brief: ADS1115_Get_Single_Conversion - starts and returns the result of a single conversion
 * @param: adc							 - ADS1115 device
 * @param: channel						 - single ended input channel
 * 										 - valid values are 0,1,2 or 3
 * @reval: conv							 - 16-bit signed integer conversion result
 * */
int16_t ADS1115_Get_Single_Conversion(ADS1115* adc , uint8_t channel)
{
	uint8_t buf[3] = {0};

	channel += 4;	//multiplexer single-ended mode setting bits
	channel &= 0x07;//masking irrelevant bits

	buf[0] = ADS1115_REG_CONFIG;
	buf[1] |= 	(ADS1115_OS_START_CONV << 	15-8) 
				| (channel 			<<	12-8) 
				| (adc->fullRange	<<	9-8) 
				| (ADS1115_MODE_SINGLE_CONV	<<	8-8);

	buf[2] |= 	(adc->sampleRate		<<	5) 
				| (adc->compMode		<<	4) 
				| (adc->compPol		<<	3) 
				| (adc->compLatch	<<	2) 
				| adc->compQue		<<	0;

	ADS1115_I2C_Send(adc->address,buf,3);

	while(!(ADS1115_Get_Config(adc) & (1<<15))); //while a conversion is ongoing

	return ADS1115_Get_Last_Result(adc);
}

/**
 * @brief: ADS1115_Get_Last_Result	- returns the contents of the ADS1115 conversion result register
 * @param: adc						- ADS1115 device
 * @reval: conv						- 16-bit signed integer conversion result
 * */
int16_t ADS1115_Get_Last_Result(ADS1115* adc)
{
	int16_t conv = 0;
	uint8_t buf[3] = {0};

	buf[0] = ADS1115_REG_CONV;
	ADS1115_I2C_Send(adc->address,buf,1);

	ADS1115_I2C_Receive(adc->address,buf,2);
	
	if (buf[0] & (1<<8))
		conv = ~( ((uint16_t)buf[0] << 8) | buf[1] ) + 1;
	else
		conv = ((uint16_t)buf[0] << 8) | buf[1];
	
	return conv;
}


uint16_t ADS1115_Get_Config(ADS1115* adc)
{
	uint8_t buf[3] = {0};

	buf[0] = ADS1115_REG_CONFIG;
	ADS1115_I2C_Send(adc->address,buf,1);

	ADS1115_I2C_Receive(adc->address,buf,2);

	return (uint16_t)buf[0] <<8 | buf[1];

}