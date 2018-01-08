/* Includes ------------------------------------------------------------------*/
#include "transceiverDriver.h"
#include "rfTransmissionTask.h"
#include "MAX30100.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>


extern 	max30100Data lastAcquiredData;//variable that stores the last data that was acquired
extern xQueueHandle xTransmissionData;

void rfTransmissionTask()
{
	char messageData[7]={0};
	rfMessage Data;
	xQueueReceive(xTransmissionData,&Data,portMAX_DELAY);
	messageData[0]=Data.type;
	messageData[1]=Data.priority;
	//messageData[2]=->room
	messageData[3]=lastAcquiredData.bpm;
	messageData[4]=(char)lastAcquiredData.spo2;
	//messageData[3-6]->health data;
	transmitData(&messageData);
}
