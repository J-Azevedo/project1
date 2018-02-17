/**
  ******************************************************************************
  * @file    timerModule.h
  * @author  Joao Reis
  * @date    15-November-2017
  * @brief   This file contains all the functions prototypes for the gyroscope 
	(L3GD20) driver.
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f4xx.h"

#define   APB1_FREQ        (SystemCoreClock / 4)                  // Clock driving TIM3
#define 	TIMER3_CLOCK     1000        														// TIM3 counter clock (prescaled APB1)
#define   TIM_PRESCALER    (((APB1_FREQ) / (TIMER3_CLOCK))-1)     //calculate APB1 prescaler to get TIMER3_CLOCK of 1kHz 
																																	//Timer 3 clock tick = 1ms

void timStart(void);
void timStop(void);
void timInit(void);
//extern l3gd20Data data;

#endif /*__TIMER_H__*/
