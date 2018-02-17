/**
  ******************************************************************************
  * @file    spiModule.h
  * @author  Joao Azevedo and Joao Reis 
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPIMODULE_H__
#define __SPIMODULE_H__

#include "stm32f4xx.h"

#define DUMMY_BYTE 										0xFF

void spiInit(void);
uint8_t spiTransmit(uint8_t);



void tSpiInit(void);

#endif /*__SPIMODULE_H__*/

