/**
  ******************************************************************************
  * @file    MAX30100.h
  * @author  Afonso Santos and Joao Azevedo
  * @date    15-December-2017
  ******************************************************************************/

#ifndef __MAX30100__
#define __MAX30100__


#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stm32f4xx.h>
#include "stm32f4xx_dma.h"
#include "MAX30100_Registers.h"
#include "i2cModule.h"
typedef struct max30100Data_t
{
  float spo2;
	int bpm;

} max30100Data;

uint8_t I2CReadACK(void);
uint8_t I2CReadNACK(void);
void I2CStop(void);
int setAddr (uint8_t address);
uint8_t ReadByte (uint8_t address);
int ReadFIFO(void);
int ResetMAX(void);
int ModuleConfig(void);
void dataProcessing(void);


#endif		// __MAX30100__

