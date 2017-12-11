/* Includes ------------------------------------------------------------------*/
#include "pushButtonDriver.h"
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI

/******************************************************************************
*								Public Variables
*******************************************************************************/




/******************************************************************************
*								Private Variables
*******************************************************************************/



/******************************************************************************
*								Private Headers
*******************************************************************************/




/*****************************************************************************
*			Private Functions
******************************************************************************/
static void gpioInit(void)
{//PD11
	
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
    
	/*enable NSS GPIO clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 

  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	

	
	//interrupt pin configuration

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
    
	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    
	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;//see this may be wrong ->probably it should only interupt in one of them
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);
 
	/* Add IRQ vector to NVIC */
	/* PD1 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			EXTI_ClearITPendingBit(EXTI_Line0);
			EXTI_ClearFlag(EXTI_Line0);
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
	

	


}




/*****************************************************************************
*			Public Functions
******************************************************************************/

void pushButtonInit(void)
{
	gpioInit();


}

/* Handle PA0 interrupt */
void EXTI0_IRQHandler(void) {
	/* Make sure that interrupt flag is set */

	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Do your stuff when PD0 is changed */
		
	/* we take the semaphore that is shared with the Transmit Data() so the transmit data can continue -> the semaphore can be taken here
		and if we are waiting for an ACK and the overflow time finishes we can also take it in the timer ISR*/  	
		
		/*we indicate that the ack was received*/
			
		
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
					EXTI_ClearFlag(EXTI_Line0);
					/*
			before we exit the interrupt we have to stop the timer so it doesn't occur a overflow that migth create an error in our program
					TIM_Cmd(TIM3, DISABLE);
			*/
	}
}
