/* Includes ------------------------------------------------------------------*/
#include "gyroscopeDriver.h"
#include "gyroAcquisitionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
extern xQueueHandle xQGyroData;

extern xSemaphoreHandle xSemPBFinish;

void gyroAcquisitionTask()
{
	static int i=0;

	static	l3gd20Data data[100];
	float x=0;
	//acquire the data from the gyroscope
	data[i]=gyroReadAxisValue();
	i++;
	//send the data to the back of the queue, to be read in the processing task
	xQueueSendToBack(xQGyroData,&x,(TickType_t)50);
	//delay the task for a period of 50 ms 
	vTaskDelay(50 / portTICK_RATE_MS);	
	if(i==99)
		i=0;
}
