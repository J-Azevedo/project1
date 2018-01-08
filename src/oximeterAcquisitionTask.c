/* Includes ------------------------------------------------------------------*/
#include "MAX30100.h"
#include "oximeterAcquisitionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

extern 	max30100Data lastAcquiredData;//variable that stores the last data that was acquired
extern xSemaphoreHandle xSemOximeterAcquisitionFinish;

void oximeterAcquisitionTask(TickType_t xLastWakeTime,TickType_t xFrequency)
{

 
        // Wait for the next cycle.
  vTaskDelayUntil( &xLastWakeTime, xFrequency );
	ModuleConfig();
	while(xSemaphoreGive( xSemOximeterAcquisitionFinish )!= pdTRUE);
	ResetMAX();
	if(lastAcquiredData.bpm>120||lastAcquiredData.bpm<50||lastAcquiredData.spo2<85)
	{
		//post in queue to send message
	}


}
