/* Includes ------------------------------------------------------------------*/
#include <FreeRTOS.h>
#include "pushButtonDriver.h"
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f4_discovery.h"
#include <task.h>
#include <semphr.h>
/******************************************************************************
*								Public Variables
*******************************************************************************/

extern xSemaphoreHandle xSemPBFinish; 

extern xTaskHandle xTskGyroAcquisition;


/*****************************************************************************
*			Public Functions
******************************************************************************/

void pushButtonInit(void)
{
	  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
}

void EXTI0_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	//portBASE_TYPE xYieldRequired;
	
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		EXTI_DeInit(); //disable external interrupt //Deinitializes the EXTI peripheral
		
		//xTaskResumeFromISR( xTskGyroAcquisition );
		xSemaphoreGiveFromISR(xSemPBFinish, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

