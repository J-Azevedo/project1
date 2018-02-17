/**
  ******************************************************************************
  * @file    i2cModule.h
  * @author  Joao Azevedo and Joao Reis
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2CMODULE_H__
#define __I2CMODULE_H__

#include "stm32f4xx.h"

void i2cInit(void);
void i2cWrite(uint8_t, uint8_t, uint8_t);
uint8_t i2cRead(uint8_t, uint8_t);


#endif /*__I2CMODULE_H__*/

