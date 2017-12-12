/**
  ******************************************************************************
  * @file    spiModule.h
  * @author  Joao 
  * @date    15-November-2017
  * @brief   This file contains all the functions prototypes for the SPI 
  *          firmware library. 
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2CMODULE_H__
#define __I2CMODULE_H__

#include "stm32f4xx.h"

void i2cInit(void);
void i2cWrite(uint8_t, uint8_t, uint8_t);
uint8_t i2cRead(uint8_t, uint8_t);


#endif /*__I2CMODULE_H__*/

