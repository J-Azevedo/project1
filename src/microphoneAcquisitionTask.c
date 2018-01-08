/* Includes ------------------------------------------------------------------*/
#include "i2sModule.h"
#include "microphoneDriver.h"
#include "microphoneAcquisitionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

extern xSemaphoreHandle xSemMicRecordingFinish;

void microphoneAcquisitionTask(void)
{
	//xSemaphoreTake(xSem_Gyro_Processing_Finish, portMAX_DELAY);
	microphoneStart();
	while(1); //stay here to not start microphone again
}

