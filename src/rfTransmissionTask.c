/* Includes ------------------------------------------------------------------*/
#include "transceiverDriver.h"
#include "rfTransmissionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>



extern xQueueHandle xTransmissionData;
void rfTransmissionTask()
{
	char messageData[7]={0};
	rfMessage Data;
	xQueueReceive(xTransmissionData,&Data,portMAX_DELAY);
	messageData[0]=Data.type;
	messageData[1]=Data.priority;
	//messageData[2]=->room
	//messageData[3-6]->health data;
	transmitData(&messageData);
}
