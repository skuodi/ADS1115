#ifndef _ADS1115_H_
#define _ADS1115_H_ 

#include <stdint.h>

//############################Register Addresses########################################//
#define ADS1115_REG_CONV	0x00
#define ADS1115_REG_CONFIG	0x01
#define ADS1115_REG_LO_TH	0x02
#define ADS1115_REG_HI_TH	0x03

//############################CONFIG Register Bits########################################//

//Device Operational Status
typedef enum 
{
	ADS1115_OS_IDLE 	= 0x00,
	ADS1115_OS_START_CONV = 0x01
}ADS1115_OS;

//analog multiplexer input selection
typedef enum 
{
	ADS1115_MUX_AINP0_AINN1	= 0x00,
	ADS1115_MUX_AINP0_AINN3	= 0x01,
	ADS1115_MUX_AINP1_AINN3	= 0x02,
	ADS1115_MUX_AINP2_AINN3	= 0x03,
	ADS1115_MUX_AINP0_GND	= 0x04,
	ADS1115_MUX_AINP1_GND	= 0x05,
	ADS1115_MUX_AINP2_GND	= 0x06,
	ADS1115_MUX_AINP3_GND	= 0x07
}ADS1115_MUX;


//Programmable Gain Amplifier FullScaleRange Selection
//	the last three have the same effect
typedef enum 
{
	ADS1115_PGA_FSR_6V144	= 0x00,
	ADS1115_PGA_FSR_4V096	= 0x01,
	ADS1115_PGA_FSR_2V048	= 0x02,
	ADS1115_PGA_FSR_1V024	= 0x03,
	ADS1115_PGA_FSR_0V512	= 0x04,
	ADS1115_PGA_FSR_0V256_1	= 0x05,
	ADS1115_PGA_FSR_0V256_2	= 0x06,
	ADS1115_PGA_FSR_0V256_3	= 0x07
	
}ADS1115_PGA_FSR;

//Device Operating Mode
typedef enum 
{
	ADS1115_MODE_CONTINUOUS_CONV	= 0x00,
	ADS1115_MODE_SINGLE_CONV		= 0x01

}ADS1115_MODE;

//Data Rate Selection
typedef enum
{
	ADS1115_DR_8SPS	 	= 0x00,
	ADS1115_DR_16SPS 	= 0x01,
	ADS1115_DR_32SPS 	= 0x02,
	ADS1115_DR_64SPS 	= 0x03,
	ADS1115_DR_128SPS 	= 0x04,
	ADS1115_DR_250SPS 	= 0x05,
	ADS1115_DR_475SPS 	= 0x06,
	ADS1115_DR_860SPS 	= 0x07
} ADS1115_DR;

//Comparator Mode
typedef enum 
{
	ADS1115_COMP_MODE_TRADITIONAL = 0x00,
	ADS1115_COMP_MODE_WINDOW 	= 0x01

}ADS1115_COMP_MODE;

//ALERT/RDY Pin Polarity
typedef enum 
{
	ADS1115_COMP_POL_ACTIVE_LOW = 0x00,
	ADS1115_COMP_POL_ACTIVE_HIGH = 0x01
}ADS1115_COMP_POL;

//Comparator Latching
typedef enum 
{
	ADS1115_COMP_LATCH_DISABLED = 0x00,
	ADS1115_COMP_LATCH_ENABLED = 0x01
}ADS1115_COMP_LATCH;

//Comparator Alert Signal Queue and Disable	
typedef enum 
{
	ADS1115_COMP_ALERT_QUE_ONE 		= 0x00,
	ADS1115_COMP_ALERT_QUE_TWO 		= 0x01,
	ADS1115_COMP_ALERT_QUE_FOUR 	= 0x02,
	ADS1115_COMP_ALERT_PIN_DISABLED = 0x03	
}ADS1115_COMP_ALERT_QUE;


typedef struct 
{
	uint8_t address; //7-bit left adjusted device address
	ADS1115_OS opStat;
	ADS1115_MUX inputMux;
	ADS1115_PGA_FSR fullRange;
	ADS1115_MODE opMode;
	ADS1115_DR sampleRate;
	ADS1115_COMP_MODE compMode;
	ADS1115_COMP_POL compPol;
	ADS1115_COMP_LATCH compLatch;
	ADS1115_COMP_ALERT_QUE compQue;
	uint16_t hiThresh;
	uint16_t loThresh;

}ADS1115;


//############################Functions########################################//
void ADS1115_I2C_Send(uint8_t devAddr, uint8_t * data, uint16_t size);
void ADS1115_I2C_Receive(uint8_t devAddr, uint8_t * data, uint16_t size);

void ADS1115_Reset(ADS1115* adc);
void ADS1115_Init(ADS1115* adc);

uint16_t ADS1115_Get_Config(ADS1115* adc);
int16_t ADS1115_Get_Single_Conversion(ADS1115* adc , uint8_t channel);
int16_t ADS1115_Get_Last_Result(ADS1115* adc);

#endif