#include "timerModule.h"
#include "buzzerDriver.h"


/******************************************************************************
*								Private Headers
*******************************************************************************/

//static void timInit(void);


/******************************************************************************
*								Private Variables
*******************************************************************************/

//static uint16_t current_count = 0;                      // To sample the counter


/*****************************************************************************
*								Public Functions
******************************************************************************/

void timStart(void)
{
	timInit();
	TIM_Cmd(TIM3, ENABLE);
}

void timStop(void)
{
	TIM_Cmd(TIM3, DISABLE);
}
/*****************************************************************************
*								Private Functions
******************************************************************************/

 /* TIM3 Configuration ---------------------------------------------------
   TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
                                                
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect. 
	*/

void timInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//uint16_t period=1000;
	uint16_t period=2;
	
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = period; //1000 ticks * 2ms = 2s // Autoreload value (ARR) 
  TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALER;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 						//enabling global interrupt
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                     // Enabling TIM3 Ch.1 interrupts
}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)             // Just a precaution (RESET = 0) 
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);                  // Clear TIM3 Ch.1 flag
		buzzerStop(); 					 // read raw data from gyro
		timStop();
  }
}
