/* Includes ------------------------------------------------------------------*/
#include "microphoneDriver.h"
#include "microphoneProcessingTask.h"
#include "rfTransmissionTask.h"
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

extern xQueueHandle xTransmissionData;

void microphoneProcessingTask(void)
{
	volatile unsigned char word;
	rfMessage microphoneMsg;

	wordRecognizerInit(&vocabulary); // initializes the recognition process / calc. hamming window, nfft, linear space	
	
	xSemaphoreTake( xSemMicRecordingFinish, portMAX_DELAY );
	
	word = wordRecognize(&vocabulary, recordData, BUFFER_MIC_SIZE); //return word recognized
	
	if(word==0)
		STM_EVAL_LEDOn(LED4);
	else if(word==0x01)
		STM_EVAL_LEDOn(LED5);
	else if(word==0x02)
		STM_EVAL_LEDOn(LED6);

	microphoneMsg.type='A';
	microphoneMsg.priority=word+1;
	
	xQueueSend( xTransmissionData, &microphoneMsg, 0 );
}
