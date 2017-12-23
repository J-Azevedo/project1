/* Includes ------------------------------------------------------------------*/
#include "gyroscopeDriver.h"
#include "gyroProcessingTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "main.h"

extern xQueueHandle xQGyroData;
static l3gd20Data lastcaptures[20];

void gyroProcessingTask()
{
static int i=0;
	l3gd20Data data;


	



}