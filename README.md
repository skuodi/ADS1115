# ADS1115 C Library

A C library for the ADS1115 4-channel 16-bit ADC by Texas Instruments.
The repo contains the library files in the [ADS1115]() directory while the rest of the files are an STM32F401 example using I2C1 to read conversions and print the raw 16-bit value as an emulated serial device via USB

## Usage

### Single-ended ADC

The device needs to be set up before use by creating an ADS1115 struct and passing desired settings then calling the Init function

```c
ADS1115 adc1;
adc1.address    = 0x48 <<  1;
adc1.opStat     = ADS1115_OS_IDLE; 
adc1.inputMux 	= ADS1115_MUX_AINP0_GND; 
adc1.fullRange  = ADS1115_PGA_FSR_4V096;
adc1.opMode     = ADS1115_MODE_SINGLE_CONV;
adc1.sampleRate = ADS1115_DR_8SPS;
adc1.compMode   = ADS1115_COMP_MODE_TRADITIONAL;
adc1.compPol	= ADS1115_COMP_POL_ACTIVE_LOW;
adc1.compLatch	= ADS1115_COMP_LATCH_DISABLED;
adc1.compQue	= ADS1115_COMP_ALERT_PIN_DISABLED;

ADS1115_Init(&adc1);
```

Then, either get a single ADC reading using the ``ADS1115_Get_Single_Conversion()`` command,
or set it in continuous conversion mode

```c
adc1.opMode     = ADS1115_MODE_CONTINUOUS_CONV;
```
and read the latest conversion results using ``ADS1115_Get_Last_Result()``.

### Comparator
To use the comparator, set the multiplexer input to any two channels. The comparator negative(non-inverting) input can only be either channel 1 or channel 3, and positive(inverting input) can only be 0, 1 or 2.
Only the following combinations are valid:
```c
	adc1.inputMux = ADS1115_MUX_AINP0_AINN1;//Positive = channel 0, negative=channel 1
	adc1.inputMux = ADS1115_MUX_AINP0_AINN3;//Positive = channel 0, negative=channel 3
	adc1.inputMux = ADS1115_MUX_AINP1_AINN3;//Positive = channel 1, negative=channel 3
	//or
	adc1.inputMux = ADS1115_MUX_AINP2_AINN3;//Positive = channel 2, negative=channel 3
```
Set the comparator mode, output pin polarity, latching (optional) and thresholds. Then call the Init function to apply the settings.

## Porting
The library has been tested on STM32F401 but can easily be ported to any other MCU board/platform by replacing the I2C function definitions (HAL_I2C... for STM32) in the [library source file](https://github.com/skuodi/ADS1115/tree/main/ADS1115/ads1115.c) with those specific to your device.
