/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "buzzerDriver.h"

/******************************************************************************
*								Public Variables
*******************************************************************************/




/******************************************************************************
*								Private Variables
*******************************************************************************/



/******************************************************************************
*								Private Headers
*******************************************************************************/

static void timInit(void);
static void gpioInit(void);



/*****************************************************************************
*			Private Functions
******************************************************************************/


static void timInit(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
//	uint16_t PrescalerValue = 0; //get this right to the specific clock value
//	uint16_t period=665; 
//	 /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;//need to be reviewed later to get real values for our project

//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = period;
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//	
//	  /* PWM1 Mode configuration: Channe3 */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = period/2;//duty cycle of 50%
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

//  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	uint16_t PrescalerValue =  (42000-1); //get this right to the specific clock value
	uint16_t period=2000-1; 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	 /* Compute the prescaler value */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	int i=SystemCoreClock;
  //PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;//need to be reviewed later to get real values for our project
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	  /* PWM1 Mode configuration: Channe3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = period/2;//duty cycle of 50%
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	



}

static void gpioInit(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);


  /* GPIOD Configuration: TIM4 CH3 (PD10) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_TIM4);


}






/*****************************************************************************
*			Public Functions
******************************************************************************/





void buzzerInit(void)
{
	gpioInit();
	timInit();
	
	
}


void buzzerStart(void)
{
  TIM_Cmd(TIM4, ENABLE);


}


void buzzerStop(void)
{
  TIM_Cmd(TIM4, DISABLE);


}
