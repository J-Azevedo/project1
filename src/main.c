/* Scheduler includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
/*stm libraries*/

/*Project includes*/
#include "transceiverDriver.h"
#include "RFM69registers.h"
#include "spiModule.h"
#include "buzzerDriver.h"
#include "pushButtonDriver.h"
#include "gyroscopeDriver.h"
/*Task includes*/
#include "gyroAcquisitionTask.h"
#include "gyroProcessingTask.h"
#include "rfTransmissionTask.h"
/* Library includes. */
#include <stm32f4xx.h>

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif



void prvSetupLed(void);

/*************************************
*task handler declaration
*************************************/
xTaskHandle xTskGyroAcquisition;
xTaskHandle xTskGyroProcessing;
xTaskHandle xTskRfTransmission;

/*************************************
*queue declaration
*************************************/
xQueueHandle xQGyroData;
xQueueHandle xTransmissionData;

/*************************************
*semaphore declaration
*************************************/
//xSemaphoreHandle xSemGyroDataProcessing;
xSemaphoreHandle xSemMicrophoneStart;

/*************************************
*queue initialization
*************************************/

int queueInitialization()
{
	//xQGyroData=xQueueCreate( 20, sizeof( l3gd20Data ) );
	xQGyroData=xQueueCreate( 20, sizeof( float ) );
	if(xQGyroData==NULL)
	{
		return 1;
	}
	xTransmissionData=xQueueCreate(20,sizeof(rfMessage));
	if(xTransmissionData==NULL)
	{
		return 1;
	}
return 0;

}


/*************************************
*semaphore initialization
*************************************/
int semaphoreInitialization()
{

 
	vSemaphoreCreateBinary(xSemMicrophoneStart);
	if(xSemMicrophoneStart==NULL)
	{
		return 1;
	}
return 0;
}


void vLEDTask( void *pvParameters )
{
	prvSetupLed();
/* TickType_t xLastWakeTime;
 const TickType_t xFrequency = 10;*/

     // Initialise the xLastWakeTime variable with the current time.
    // xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		/* Toogle the LED bit */
		GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		vTaskDelay(1000 / portTICK_RATE_MS);	
//		  vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}
}

void vGyroAcquisitionTask( void *pvParameters )
{
	//gyroStart();
	for( ;; )
	{
		gyroAcquisitionTask();

	}
}

void vGyroProcessingtTask( void *pvParameters)
{
	for( ;; )
	{
		gyroProcessingTask();

	}

}
void vRfTransmissionTask(void *pvParameters)
{
		for( ;; )
	{
		rfTransmissionTask();

	}
}



int main()
{
	portBASE_TYPE task1_pass;
	portBASE_TYPE task2_pass;
	portBASE_TYPE task3_pass;

		gyroStart();
	semaphoreInitialization();
 queueInitialization();
	/* Create Task */
	task1_pass = xTaskCreate( vGyroAcquisitionTask, "Gyro_Acquisition_task", configMINIMAL_STACK_SIZE, NULL, 1, xTskGyroAcquisition );
	task2_pass = xTaskCreate( vGyroProcessingtTask, "Gyro_Processing_task", configMINIMAL_STACK_SIZE, NULL, 1, xTskGyroProcessing );
	task3_pass= xTaskCreate(vRfTransmissionTask,"RF_Transmission_Task",configMINIMAL_STACK_SIZE, NULL, 1,xTskRfTransmission);
	if( task1_pass == pdPASS )
	{
			/* Start the Scheduler */ 
			vTaskStartScheduler(); 
	}
	else
	{
			/* ERROR! Creating the Tasks */
			return -2;
	}
//	short int data[10]={0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21};

//	transceiverInit();
//	transmitData(&data);
	
	//while(i!=0x24)
//	i=readReg(0x0C);
	/*
	while(i!=( RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00))
	i=readReg(REG_OPMODE);*/
/*	buzzerInit();
	buzzerStart();*/
//	pushButtonInit();
	

	return 0;
}


void prvSetupLed(void)
{
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct;
	// Enabling GPIO peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// GPIO peripheral properties specification
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15; // LED3 GPIO pin
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}
