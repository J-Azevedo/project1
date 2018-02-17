/* Includes ------------------------------------------------------------------*/
#include "transceiverDriver.h"
#include "rfTransmissionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

extern xSemaphoreHandle xSemMicProcessingFinish;
extern xSemaphoreHandle xSemTransmitFinish;
int room=4;
extern xQueueHandle xTransmissionData;
int bpm;
float spo2;

void rfTransmissionTask()
{
	char messageData[7]={0};
	rfMessage msgInput;
	xQueueReceive( xTransmissionData, &msgInput, portMAX_DELAY );
	messageData[0]=msgInput.type;
	messageData[1]=msgInput.priority;
	messageData[2]=room;
	messageData[3]=bpm;
	messageData[4]=(int)spo2;
	transmitData(messageData);
	xSemaphoreGive(xSemTransmitFinish);
}
