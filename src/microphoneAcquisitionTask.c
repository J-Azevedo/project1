/* Includes ------------------------------------------------------------------*/
#include "i2sModule.h"
#include "microphoneDriver.h"
#include "microphoneAcquisitionTask.h"
#include "buzzerDriver.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stdlib.h>
#include <stdio.h>

extern xSemaphoreHandle xSemMicRecordingFinish;
extern xSemaphoreHandle xSemGyroProcessingFinish;

void microphoneAcquisitionTask(void)
{
	xSemaphoreTake(xSemGyroProcessingFinish, portMAX_DELAY);
	buzzerStart();
	vTaskDelay(1000);
	buzzerStop();
	microphoneStart();
}

