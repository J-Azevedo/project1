/* Includes ------------------------------------------------------------------*/
#include "MAX30100.h"
#include "oximeterAcquisitionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "main.h"
#include "rfTransmissionTask.h"

extern 	max30100Data lastAcquiredData;//variable that stores the last data that was acquired
extern xSemaphoreHandle xSemOximeterAcquisitionFinish;
extern xQueueHandle xTransmissionData;


void oximeterAcquisitionTask(TickType_t xLastWakeTime,TickType_t xFrequency)
{
	rfMessage oximeterMsg;

 
  // Wait for the next cycle.
  vTaskDelayUntil( &xLastWakeTime, xFrequency );
	ModuleConfig();
	xSemaphoreTake( xSemOximeterAcquisitionFinish,portMAX_DELAY );
	ResetMAX();
	dataProcessing();
	if(lastAcquiredData.bpm>120||lastAcquiredData.bpm<50||lastAcquiredData.spo2<85)
	{
		oximeterMsg.priority=4;
		oximeterMsg.type='A';
		
		//post in queue to send message
		xQueueSend( xTransmissionData, &oximeterMsg, 0 );
	}
}

