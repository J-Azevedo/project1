/**
  ******************************************************************************
  * @file    MicrophoneDriver.h
  * @author  Joao Azevedo

  * @brief   This file contains all the functions prototypes for the SPI 
  *          firmware library. 
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


















#endif //__MICROPHONE_DRIVER