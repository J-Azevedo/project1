/* Includes ------------------------------------------------------------------*/
#include "microphoneDriver.h"
#include "microphoneProcessingTask.h"
#include "wordRecognizer.h"
#include "wordData.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stdlib.h>
#include "main.h"
#include "stm32f4_discovery.h"

extern xSemaphoreHandle xSemMicRecordingFinish;
extern xSemaphoreHandle xSemMicProcessingFinish;

void microphoneProcessingTask(void)
{
	volatile unsigned char word;
//	uint32_t ulNotificationValue;
//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
	
	wordRecognizerInit(&vocabulary); // Initializes the recognition process //calc. hamming window, nfft, linear space, etc	
	
	xSemaphoreTake(xSemMicRecordingFinish, portMAX_DELAY);
//	ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );
//	if( ulNotificationValue == 1 )
//  {
//      /* The transmission ended as expected. */
//  }
//  else
//  {
//      /* The call to ulTaskNotifyTake() timed out. */
//  }
	
	
	word = wordRecognize(&vocabulary, recordData, BUFFER_MIC_SIZE);// recognition process
	if(word==0)
		STM_EVAL_LEDOn(LED4);
	else if(word==0x01)
		STM_EVAL_LEDOn(LED5);
	else if(word==0x02)
		STM_EVAL_LEDOn(LED6);

	xSemaphoreGive(xSemMicProcessingFinish);
}
