/* Scheduler includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "transceiverDriver.h"
#include "RFM69registers.h"
#include "spiModule.h"
#include "buzzerDriver.h"
#include "pushButtonDriver.h"
#include "gyroscopeDriver.h"
#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

/* Library includes. */
#include <stm32f4xx.h>

void prvSetupLed(void);

void vLEDTask( void *pvParameters )
{
	prvSetupLed();
	for( ;; )
	{
		/* Toogle the LED bit */
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
		vTaskDelay(1000 / portTICK_RATE_MS);	
	}
}


int main()
{
//	portBASE_TYPE task1_pass;
//	
//	/* Create Task */
//	task1_pass = xTaskCreate( vLEDTask, "Task_Led", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
//	
//	if( task1_pass == pdPASS )
//	{
//			/* Start the Scheduler */ 
//			vTaskStartScheduler(); 
//	}
//	else
//	{
//			/* ERROR! Creating the Tasks */
//			return -2;
//	}
	short int data[10]={0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21};

	transceiverInit();
	transmitData(&data);
	
	//while(i!=0x24)
//	i=readReg(0x0C);
	/*
	while(i!=( RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00))
	i=readReg(REG_OPMODE);*/
/*	buzzerInit();
	buzzerStart();*/
//	pushButtonInit();
	

	while(1);
	return 0;
}


void prvSetupLed(void)
{
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct;
	// Enabling GPIO peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// GPIO peripheral properties specification
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; // LED3 GPIO pin
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}
