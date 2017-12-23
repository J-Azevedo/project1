/* Includes ------------------------------------------------------------------*/
#include "gyroscopeDriver.h"
#include "gyroAcquisitionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "main.h"

extern xQueueHandle xQGyroData;

void gyroAcquisitionTask()
{
static int i=0;
	//l3gd20Data data;
static	l3gd20Data data[100];
	static int dx[500];
		static int dy[500];
		static int dz[500];
	//i++;
	data[i++]=gyroReadAxisValue();

	//i++;
//	xQueueSendToBack(xQGyroData,&data,(TickType_t)50);
	vTaskDelay(50 / portTICK_RATE_MS);	
	if(i==99)
	{
			i++;
	}
	
//4.28
	//4.60
	//4.44
	//3.53
	//4.2125
//log > truedata02.log

}