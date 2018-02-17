/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "dmaModule.h"
#include <FreeRTOS.h>
#include "i2sModule.h"
#include "microphoneDriver.h"
#include "pdm_filter.h"
#include <task.h>
#include "stm32f4xx_dma.h"              // Keil::Device:StdPeriph Drivers:DMA
#include "wordData.h"
#include "stm32f4_discovery.h"
#include <semphr.h>

/******************************************************************************
*								Public Variables
*******************************************************************************/
extern xSemaphoreHandle xSemMicRecordingFinish; 
extern PDMFilter_InitStruct filter;

/******************************************************************************
*								Private Variables
*******************************************************************************/
static unsigned short int memory_buffer0[PCM_OUT_SIZE];/* PCM output. The application
must pass to the function the pointer to the first sample of the channel to be obtained. */
static unsigned short int memory_buffer1[PCM_OUT_SIZE];
volatile static unsigned short int record_buffer0[INTERNAL_BUFF_SIZE];/* PDM input; the application must pass to the function
the pointer to the first input sample of the microphone that must be processed.*/
volatile static unsigned short int record_buffer1[INTERNAL_BUFF_SIZE];

/******************************************************************************
*								Private Headers
*******************************************************************************/
static void dmaNVICInit(void);
static void dmaBufferSwap(short int*);

/*****************************************************************************
*		Private	Functions
******************************************************************************/
static void dmaNVICInit(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	
	  /* I2S DMA IRQ Channel configuration */
  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 14;
  //NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}

static void dmaBufferSwap(short int* buffer)
{
	static int j = 0;
	unsigned int i;
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	for(i = 0; i < 16; i=i+2)
		recordData[j++] = buffer[i]/ 32767.0; //PCM output copied to global variable recordData
	
	if(j == BUFFER_MIC_SIZE)
	{
		microphoneStop();
		j = 0;
		
		xSemaphoreGiveFromISR(xSemMicRecordingFinish, &xHigherPriorityTaskWoken); //signal end of recording to process

		/* If xHigherPriorityTaskWoken was set to true we should yield.  The actual macro used here is
    port specific. */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	}
}

/*****************************************************************************
*		Public	Functions
******************************************************************************/

void dmaInit(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* Configure the DMA Stream */
  DMA_Cmd(DMA1_Stream3, DISABLE);
  DMA_DeInit(DMA1_Stream3);
  /* Set the parameters to be configured */
  DMA_InitStruct.DMA_Channel = DMA_Channel_0;  
  DMA_InitStruct.DMA_PeripheralBaseAddr = 0x4000380C; //I2S address
  DMA_InitStruct.DMA_Memory0BaseAddr = (unsigned int)record_buffer0; //pdm input address
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = 64;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &DMA_InitStruct);
	DMA_DoubleBufferModeConfig(DMA1_Stream3, (unsigned int)record_buffer1, DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);
  /* Enable DMA Stream Transfer Complete interrupt */  
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	
	dmaNVICInit();

  /* Enable the I2S DMA request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);	
}

/*DMA Interruption - cpu receives an interrupt from the DMA controller when transfer operation is done (TC=0)*/
void DMA1_Stream3_IRQHandler(void)
{
	unsigned int i;
	/* Transfer complete from micrphone to DMA interrupt, in 1st iteration buffer0 fills, 
	buffer1 has nothing to process. 2nd iteration buffer0 sends data to process, buffer 1 fills*/
  if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET) 
	{
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream3))
		{
			for(i = 0; i < 64; i++)
				record_buffer0[i] = HTONS(record_buffer0[i]); //turn into big endian format
			/*process a millisecond of PDM data from a single microphone.
			64 refers to decimation factor, LSB refers to representation of record_buffer0, returns 16 samples*/
			PDM_Filter_64_LSB((uint8_t *)record_buffer0, (uint16_t *)memory_buffer0, VOLUME, &filter); 
			dmaBufferSwap((short int*)memory_buffer0);
		}
		else
		{
			for(i = 0; i < 64; i++)
				record_buffer1[i] = HTONS(record_buffer1[i]);
			PDM_Filter_64_LSB((uint8_t *)record_buffer1, (uint16_t *)memory_buffer1, VOLUME, &filter); //convert pdm to pcm
			dmaBufferSwap((short int*)memory_buffer1);
		}
	}
}
