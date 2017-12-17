/* Includes ------------------------------------------------------------------*/
#include "i2sModule.h"
#include "microphoneDriver.h"
#include "pdm_filter.h"
#include "stm32f4xx_dma.h"              // Keil::Device:StdPeriph Drivers:DMA
#include "WordsData.h"

/******************************************************************************
*								Public Variables
*******************************************************************************/

float recordData[BUFFER_MIC_SIZE];


/******************************************************************************
*								Private Variables
*******************************************************************************/

volatile static unsigned short int record_buffer0[INTERNAL_BUFF_SIZE];/* PDM input; the application must pass to the function
the pointer to the first input sample of the microphone that must be processed.*/
volatile static unsigned short int record_buffer1[INTERNAL_BUFF_SIZE];
static unsigned short int memory_buffer0[PCM_OUT_SIZE];/* PCM output. The application
must pass to the function the pointer to the first sample of the channel to be obtained. */
static unsigned short int memory_buffer1[PCM_OUT_SIZE];
static PDMFilter_InitStruct filter;

/* Main buffer pointer for the recorded data storing */
/* Temporary data sample */
//static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
//static uint32_t InternalBufferSize = 0;

/******************************************************************************
*								Private Headers
*******************************************************************************/

static void Delay(volatile unsigned int n_count);
static void dmaInit(void);
//static void microphoneNVICInit(void);
static void microphoneGPIOInit(void);
static void dmaBufferSwap(short int*);


/*****************************************************************************
*			Private Functions
******************************************************************************/
static void Delay(volatile unsigned int n_count){
  for(; n_count != 0; n_count--);
}

static void dmaInit(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
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
	
  /* I2S DMA IRQ Channel configuration */
  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 14;
  //NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  /* Enable the I2S DMA request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);	
}

static void dmaBufferSwap(short int* buffer)
{
	static int j = 0;
	unsigned int i;
	volatile unsigned char word;
	
	for(i = 0; i < 16; i=i+2)
		recordData[j++] = buffer[i]; // / 32768.0; //PCM output copied to global variable recordData
	
	if(j == BUFFER_MIC_SIZE)
	{
		microphoneStop();
		j = 0;
		word = WordRecognize(&vocabulary, recordData, BUFFER_MIC_SIZE);// recognition process


		//Meter aqui semaphore para indicar fim de leitura
		//implementaçao em fase seguinte
		
//		xSemaphoreGiveFromISR(xSem_Rec_Finish, &xHigherPriorityTaskWoken);
//		
//		
//		if(xHigherPriorityTaskWoken == pdTRUE)
//		{
//			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//		}
//  
//		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

//static void microphoneNVICInit(void)
//{
//  NVIC_InitTypeDef NVIC_InitStructure;

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
//  /* Configure the SPI interrupt priority */
//  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//}

static void microphoneGPIOInit(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
  
  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
}

/*DMA Interruption - cpu receives an interrupt from the DMA controller when transfer operation is done (TC=0)*/
void DMA1_Stream3_IRQHandler(void)
{
	unsigned int i;
	int32_t n_samples;
	// Transfer complete from micrphone to DMA interrupt, in 1st iteration buffer0 fills, buffer1 has nothing to process
	//2nd iteration buffer0 sends data to process, buffer 1 fills
  if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET) //DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3)
	{
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3); //DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream3))
		{
			for(i = 0; i < 64; i++)
				record_buffer0[i] = HTONS(record_buffer0[i]); //turn into big endian format
			n_samples=PDM_Filter_64_LSB((uint8_t *)record_buffer0, (uint16_t *)memory_buffer0, VOLUME, &filter); /*process a millisecond of PDM data from a single microphone.
			They return a number of PCM samples equal to the frequency defined in the filter initialization (rate), divided by 1000.*/
			//64 refers to decimation factor, LSB refers to representation of record_buffer0, returns 16 samples
			dmaBufferSwap((short int*)memory_buffer0);
		}
		else
		{
			for(i = 0; i < 64; i++)
				record_buffer1[i] = HTONS(record_buffer1[i]);
			n_samples=PDM_Filter_64_LSB((uint8_t *)record_buffer1, (uint16_t *)memory_buffer1, VOLUME, &filter); //convert pdm to pcm
			dmaBufferSwap((short int*)memory_buffer1);
		}
	}
}

//void SPI2_IRQHandler(void)
//{  
//	u16 app;
//  /* Check if data are available in SPI Data register */
//  if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
//  {
//    app = SPI_I2S_ReceiveData(SPI2);
//    record_buffer0[InternalBufferSize++] = HTONS(app);
//    
//    /* Check to prevent overflow condition */
//    if (InternalBufferSize >= INTERNAL_BUFF_SIZE)
//    {
//      InternalBufferSize = 0;
//      
//      PDM_Filter_64_LSB((uint8_t *)record_buffer0, (uint16_t *)memory_buffer0, VOLUME , (PDMFilter_InitStruct *)&filter);
//    }
//  }
//}

/*****************************************************************************
*			Public Functions
******************************************************************************/

void microphoneInit()//, uint32_t BitRes, uint32_t ChnlNbr)
{
	Delay(10000);
	
	/*Before calling the PDM_Filter_Init() function, the application code must enable the clock of
the STM32 microcontroller CRC peripheral (configuration done in RCC register)*/
  // Enable CRC module  
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	
  /* Filter LP & HP Init */
  filter.LP_HZ = (SAMPLE_FREQUENCY / 2.0); //Defines the low pass filter cut-off frequency. To reduce the sampling frequency
  filter.HP_HZ = 10; /*Defines the high pass filter cut frequency. To remove the signal DC offset,  It has
	been implemented via an IIR filter with a cut-off frequency below the audible frequency
	range in order to preserve signal quality.*/
  filter.Fs = SAMPLE_FREQUENCY; //Defines the frequency output of the filter in Hz.
  filter.Out_MicChannels = 1; /*Defines the number of microphones in the output stream; this
	parameter is used to interlace different microphones in the output buffer. Each sample
	is a 16-bit value.*/
  filter.In_MicChannels = 1; /*Define the number of microphones in the input stream. This
	parameter is used to specify the interlacing of microphones in the input buffer. The
	PDM samples are grouped eight by eight in u8 format (8-bit).*/
  PDM_Filter_Init(&filter);

	microphoneGPIOInit();
	
	
  i2sInit();
	
  dmaInit();

	Delay(10000);

}

void microphoneStart()//uint16_t* pbuf, uint32_t size)
{
//	/* Store the location and size of the audio buffer */
//  pAudioRecBuf = pbuf;
//  AudioRecCurrSize = size;
//    
	
	//Enable DMA stream for I2S peripheral
	DMA_Cmd(DMA1_Stream3, ENABLE);
	while (DMA_GetCmdStatus(DMA1_Stream3) != ENABLE);
	
	//Enable the I2S peripheral
	if ((SPI2->I2SCFGR & 0x0400) == 0)
		I2S_Cmd(SPI2, ENABLE);
	
}

void microphoneStop()
{
	//Disable DMA stream for I2S peripheral
	DMA_Cmd(DMA1_Stream3,DISABLE);
	
	// Clear all the DMA flags 
  DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3);
	
	//Disable the I2S peripheral
	I2S_Cmd(SPI2, DISABLE);
}

