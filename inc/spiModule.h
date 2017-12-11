/**
  ******************************************************************************
  * @file    spiModule.h
  * @author  Joao 
  * @date    15-November-2017
  * @brief   This file contains all the functions prototypes for the SPI 
  *          firmware library. 
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPIMODULE_H__
#define __SPIMODULE_H__

#include "stm32f4xx.h"

void spiInit(void);
void tSpiInit(void);

#endif /*__SPIMODULE_H__*/

