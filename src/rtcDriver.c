/* Includes ------------------------------------------------------------------*/
#include "rtcDriver.h"
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "stm32f4xx_rtc.h"              // Keil::Device:StdPeriph Drivers:RTC


/******************************************************************************
*								Public Variables
*******************************************************************************/




/******************************************************************************
*								Private Variables
*******************************************************************************/



/******************************************************************************
*								Private Headers
*******************************************************************************/

static void rtcInit(void);
static void alarmInit(void);


/*****************************************************************************
*			Private Functions
******************************************************************************/

static void rtcInit(void)
{
	RTC_InitTypeDef rtcStruct;
	RTC_TimeTypeDef rtcTime;
//	RTC_DateTypeDef rtcDate;
// (+) Enable the RTC domain access (see description in the section above) -> do this later
 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset RTC Domain */
  RCC_BackupResetCmd(ENABLE);
  RCC_BackupResetCmd(DISABLE);

  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	
  /* Enable the LSE OSC */
	RCC_LSEConfig(RCC_LSE_ON);
	
	RTC_WriteProtectionCmd(DISABLE);
	RTC_EnterInitMode();
	while(RTC_WaitForSynchro()==ERROR);
	
	RTC_StructInit(&rtcStruct);
	RTC_Init(&rtcStruct);
	
	RTC_TimeStructInit(&rtcTime);
	RTC_SetTime(RTC_Format_BCD,&rtcTime);
	/*RTC_DateStructInit(&rtcDate);
	RTC_SetDate(RTC_Format_BCD,&rtcDate);
	*/
	
	
	
	RTC_ExitInitMode();
	RTC_WriteProtectionCmd(ENABLE);


}
static void alarmInit(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
  RTC_AlarmTypeDef RTC_AlarmStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* EXTI configuration */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  /* Set the alarm A Masks */
  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_All;
  RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
  
  /* Set alarm A sub seconds and enable SubSec Alarm : generate 8 interrupts per Second */
  //RTC_AlarmSubSecondConfig(RTC_Alarm_A, 0xFF, RTC_AlarmSubSecondMask_SS14_5);->see this at home

  /* Enable alarm A interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);


}


/*****************************************************************************
*			Public Functions
******************************************************************************/

void rtcDriverInit(void)
{
	rtcInit();
	alarmInit();
}

/* Handle PA0 interrupt */
void RTC_Alarm_IRQHandler(void)
{
	
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line17) != RESET) {
		/* Do your stuff when rtc alarm*/
		
		/*in this interrupt we create a message for update with the current value of heart rate*/
		

		
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line17);

	}
}
