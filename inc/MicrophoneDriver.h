/**
  ******************************************************************************
  * @file    MicrophoneDriver.h
  * @author  Joao Azevedo

  * @brief   This file contains the prototypes for the initialization of the 
							microphone and also the start and stop of the recording
  ****************************************************************************** 


 Define to prevent recursive inclusion -------------------------------------*/


#ifndef __MICROPHONE_DRIVER
#define __MICROPHONE_DRIVER

/*******************************************************************************
* 														Public Defines          												  *
********************************************************************************/

#define BUFFER_MIC_SIZE 16000

/*******************************************************************************
* 														Public Function Headers 												  *
********************************************************************************/


void microphoneInit(unsigned int rate);
void microphoneStop(void);
void microphoneStart(void);


















#endif //__MICROPHONE_DRIVER
