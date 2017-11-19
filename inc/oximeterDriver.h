/**
  ******************************************************************************
  * @file    oximeterDriver.h
  * @author  Joao 
  * @date    15-November-2017
  * @brief   This file contains all the functions prototypes for the heart rate and 
	oximeter sensor driver.
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OXIMETER_H__
#define __OXIMETER_H__

#include "stm32f4xx.h"

void oximeterInit(void);
/*int oximeterStart(void);
void oximeterWrite(l3gd20Registers_t, uint8_t);
uint8_t oximeterRead(l3gd20Registers_t, readingType);*/

#endif /*__OXIMETER_H__*/