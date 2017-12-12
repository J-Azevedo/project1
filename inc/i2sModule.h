/**
  ******************************************************************************
  * @file    I2S_Module.h
  * @author  Joao Azevedo
  * @brief   This file contains all the functions prototypes for the SPI 
  *          firmware library. 
  ****************************************************************************** 
 Define to prevent recursive inclusion -------------------------------------*/

#ifndef __I2S_MODULE
#define __I2S_MODULE

#define DECIMATION_FACTOR           64
#define SAMPLE_FREQUENCY            16000 //16 khz pcm output
#define OUT_FREQ                    SAMPLE_FREQUENCY/2
#define INPUT_CHANNELS              1
#define PDM_Input_Buffer_SIZE       (OUT_FREQ / 1000 *DECIMATION_FACTOR * INPUT_CHANNELS/8 )
#define PCM_Output_Buffer_SIZE      (OUT_FREQ / 1000 * INPUT_CHANNELS)
#define Buffer_Input_SIZE           2048
#define VOLUME                      30


/*******************************************************************************
* 														Public Function Headers 												  *
********************************************************************************/

void i2sInit(void);

#endif 
