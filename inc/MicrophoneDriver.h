/**
  ******************************************************************************
  * @file    MicrophoneDriver.h
	* @author  Joao Azevedo and Joao Reis
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

void microphoneInit(void);
void microphoneStop(void);
void microphoneStart(void);

#endif //__MICROPHONE_DRIVER
