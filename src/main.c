/* Scheduler includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
/*stm libraries*/

/*Project includes*/
#include "transceiverDriver.h"
#include "RFM69registers.h"
#include "spiModule.h"
#include "buzzerDriver.h"
#include "MAX30100.h"
#include "pushButtonDriver.h"
#include "gyroscopeDriver.h"
#include "microphoneDriver.h"
#include "stm32f4_discovery.h"
#include "wordData.h"
#include "i2cModule.h"

/*Task includes*/
#include "microphoneAcquisitionTask.h"
#include "microphoneProcessingTask.h"
#include "gyroAcquisitionTask.h"
#include "gyroProcessingTask.h"
#include "rfTransmissionTask.h"
#include "oximeterAcquisitionTask.h"
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
xTaskHandle xTskMicrophoneAcquisition;
xTaskHandle xTskMicrophoneProcessing;
xTaskHandle xTskPBAcquisition;
xTaskHandle xTskRfTransmission;
xTaskHandle xTskOximiterAcquition;


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
xSemaphoreHandle xSemGyroAcquisitionFinish;
xSemaphoreHandle xSemGyroProcessingFinish;
xSemaphoreHandle xSemMicRecordingFinish;
xSemaphoreHandle xSemPBFinish; 
xSemaphoreHandle xSemTransmitFinish; 
xSemaphoreHandle xSemOximeterAcquisitionFinish;
 xSemaphoreHandle xSemAckReceived;
xSemaphoreHandle xSemOximeterAcquisitionFinish;

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
	xTransmissionData=xQueueCreate(1,sizeof(rfMessage));//xTransmissionData=xQueueCreate(20,sizeof(rfMessage));
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
	
	vSemaphoreCreateBinary(xSemMicRecordingFinish);
	if(xSemMicRecordingFinish==NULL)
	{
		return 1;
	}
	
	vSemaphoreCreateBinary(xSemPBFinish);
	if(xSemPBFinish==NULL)
	{
		return 1;
	}
	
	vSemaphoreCreateBinary(xSemTransmitFinish);
	if(xSemTransmitFinish==NULL)
	{
		return 1;
	}

	vSemaphoreCreateBinary(xSemGyroAcquisitionFinish);
	if(xSemTransmitFinish==NULL)
	{
		return 1;
	}
	
	vSemaphoreCreateBinary(xSemGyroProcessingFinish);
	if(xSemTransmitFinish==NULL)
	{
		return 1;
	}
	
	vSemaphoreCreateBinary(xSemOximeterAcquisitionFinish);
	if(xSemOximeterAcquisitionFinish==NULL)
	{
		return 1;
	}
	
	/* Semaphores must be taken in the beginning to reset the counter */
	xSemaphoreTake( xSemMicrophoneStart, portMAX_DELAY);
	xSemaphoreTake( xSemMicRecordingFinish, portMAX_DELAY);
	xSemaphoreTake( xSemPBFinish, portMAX_DELAY); //do not take to pass at first on gyroacquisitiontask
	xSemaphoreTake( xSemTransmitFinish, portMAX_DELAY);
	xSemaphoreTake( xSemGyroAcquisitionFinish, portMAX_DELAY);
	xSemaphoreTake( xSemGyroProcessingFinish, portMAX_DELAY);
	xSemaphoreTake( xSemOximeterAcquisitionFinish, portMAX_DELAY);
	return 0;
}



void vGyroAcquisitionTask( void *pvParameters )
{
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

void vMicrophoneAcquisitionTask(void *pvParameters)
{
		for( ;; )
	{
		microphoneAcquisitionTask();
	}
}

void vMicrophoneProcessingTask(void *pvParameters)
{
		for( ;; )
	{
		microphoneProcessingTask();
	}
}

void vPBAcquisitionTask(void *pvParameters)
{
		for( ;; )
	{
		xSemaphoreTake(xSemTransmitFinish, portMAX_DELAY);
		pushButtonInit(); //enables external interrupt
		xSemaphoreTake( xSemPBFinish, portMAX_DELAY );
		vTaskResume(xTskGyroAcquisition);
	}
}
void vOximeterAcquisitionTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 120000/portTICK_RATE_MS;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		 oximeterAcquisitionTask(xLastWakeTime,xFrequency);
	}
}


int main()
{	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //needs to be applied before any NVIC initialization
	
	portBASE_TYPE task1_pass;
	portBASE_TYPE task2_pass;
	portBASE_TYPE task3_pass;
	portBASE_TYPE task4_pass;
	portBASE_TYPE task5_pass;
	portBASE_TYPE task6_pass;
	portBASE_TYPE task7_pass;

/*				PERIPHERAL INITIALIZATION 			*/
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	
	microphoneInit();
	buzzerInit();
	gyroStart();
	i2cInit();
	transceiverInit();
	semaphoreInitialization();
	queueInitialization();
	
	/* Create Task */
	task1_pass = xTaskCreate( vMicrophoneAcquisitionTask, "Microphone_Acquisition_task", configMINIMAL_STACK_SIZE, NULL, 2, &xTskMicrophoneAcquisition );
	task2_pass = xTaskCreate( vMicrophoneProcessingTask, "Microphone_Processing_task", 500, NULL, 2, &xTskMicrophoneProcessing );
	task3_pass = xTaskCreate( vPBAcquisitionTask, "PB_Acquisition_task", configMINIMAL_STACK_SIZE, NULL, 1, &xTskPBAcquisition );
	task4_pass = xTaskCreate( vRfTransmissionTask,"RF_Transmission_Task",configMINIMAL_STACK_SIZE, NULL, 5, &xTskRfTransmission );
	task5_pass = xTaskCreate( vGyroAcquisitionTask, "Gyro_Acquisition_task", configMINIMAL_STACK_SIZE, NULL, 3, &xTskGyroAcquisition );
	task6_pass = xTaskCreate( vGyroProcessingtTask, "Gyro_Processing_task", configMINIMAL_STACK_SIZE, NULL, 3, &xTskGyroProcessing );
	task7_pass = xTaskCreate(	vOximeterAcquisitionTask,"Oximiter_Acquitision_task",configMINIMAL_STACK_SIZE,NULL,4,&xTskOximiterAcquition);
	if( task1_pass == pdPASS && task2_pass == pdPASS && task3_pass == pdPASS && task4_pass == pdPASS && task5_pass == pdPASS && task6_pass == pdPASS &&	task7_pass==pdPASS )
	{
			/* Start the Scheduler */ 
			vTaskStartScheduler(); 
	}
	else
	{
			/* ERROR! Creating the Tasks */
			return -2;
	}
	return 0;
}

