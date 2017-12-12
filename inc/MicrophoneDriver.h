/**
  ******************************************************************************
  * @file    MicrophoneDriver.h
  * @author  Joao Reis
  * @brief   This file contains the prototypes for the initialization of the 
							microphone and also the start and stop of the recording
  ****************************************************************************** 
 Define to prevent recursive inclusion -------------------------------------*/


#ifndef __MICROPHONE_DRIVER
#define __MICROPHONE_DRIVER
#include <stdint.h>

/*******************************************************************************
* 														Public Defines          												  *
********************************************************************************/

#define BUFFER_MIC_SIZE 									8000
/* Audio recording frequency in Hz */
#define SAMPLE_FREQUENCY                  16000  

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE     					  64

/* PCM buffer output size */
#define PCM_OUT_SIZE            					16

extern float recordData[BUFFER_MIC_SIZE];


/*******************************************************************************
* 														Public Function Headers 												  *
********************************************************************************/

void microphoneInit(void);//, uint32_t, uint32_t);
void microphoneStop(void);
void microphoneStart(void);//uint16_t*, uint32_t);

#endif //__MICROPHONE_DRIVER
