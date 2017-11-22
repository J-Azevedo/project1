/* Includes ------------------------------------------------------------------*/
#include "I2S_Module.h"
#include "MicrophoneDriver.h"
#include "pdm_filter.h"
#include "stm32f4xx_dma.h"              // Keil::Device:StdPeriph Drivers:DMA


/******************************************************************************
*								Public Variables
*******************************************************************************/

float recordData[BUFFER_MIC_SIZE];


/******************************************************************************
*								Private Variables
*******************************************************************************/

static DMA_InitTypeDef dma_init_structure;
static PDMFilter_InitStruct filter;

volatile static unsigned short int record_buffer0[64];
volatile static unsigned short int record_buffer1[64];
static unsigned short int memory_buffer0[64];
static unsigned short int memory_buffer1[64];


/******************************************************************************
*								Private Headers
*******************************************************************************/


static void DMA_init(void);



/*****************************************************************************
*			Private Functions
******************************************************************************/


static void DMA_init(void)
{
	NVIC_InitTypeDef nvic_init_structure;
	
	RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
		/* Configure the DMA Stream */
  DMA_Cmd(DMA1_Stream3, DISABLE);
  DMA_DeInit(DMA1_Stream3);
  /* Set the parameters to be configured */
  dma_init_structure.DMA_Channel = DMA_Channel_0;  
  dma_init_structure.DMA_PeripheralBaseAddr = 0x4000380C;
  dma_init_structure.DMA_Memory0BaseAddr = (unsigned int)record_buffer0;
  dma_init_structure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_init_structure.DMA_BufferSize = 64;
  dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  //dma_init_structure.DMA_Mode = DMA_Mode_Normal;
	dma_init_structure.DMA_Mode = DMA_Mode_Circular;
  dma_init_structure.DMA_Priority = DMA_Priority_VeryHigh;
  dma_init_structure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma_init_structure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  dma_init_structure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dma_init_structure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream3, &dma_init_structure);
	DMA_DoubleBufferModeConfig(DMA1_Stream3, (unsigned int)record_buffer1, DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);
  /*Enable the selected DMA interrupts */
  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
  /* I2S DMA IRQ Channel configuration */
  nvic_init_structure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 14;
  //nvic_init_structure.NVIC_IRQChannelSubPriority = 0;
  nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init_structure);

  /* Enable the I2S DMA request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
		
}


/*DMA Interruption*/
void DMA1_Stream3_IRQHandler(void){
	unsigned int i;
	// Transfer complete interrupt 
  if(DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) != RESET)
	{
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream3))
		{

			for(i = 0; i < 64; i++)
				record_buffer0[i] = HTONS(record_buffer0[i]);
			PDM_Filter_64_LSB((uint8_t *)record_buffer0, (uint16_t *)memory_buffer0, 32, &filter);

		//	MicBufferSwap((short int*)buffer0_m);
		}
		else
		{
			for(i = 0; i < 64; i++)
				record_buffer1[i] = HTONS(record_buffer1[i]);
			PDM_Filter_64_LSB((uint8_t *)record_buffer1, (uint16_t *)memory_buffer1, 32, &filter);

			//	MicBufferSwap((short int*)buffer1_m);
		}
	}
}






/*****************************************************************************
*			Public Functions
******************************************************************************/




void microphoneInit(unsigned int rate)
{
  // Enable CRC module 
 // RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	
  /* Filter LP & HP Init */
  filter.LP_HZ = rate/4;
  filter.HP_HZ = 200;
  filter.Fs = rate;
  filter.Out_MicChannels = 1;
  filter.In_MicChannels = 1;
  PDM_Filter_Init(&filter);

  
  /* Configure the SPI */
  I2S_module_init(rate);
	
  /* Configure the DMA */
  DMA_init();
	

}

void microphoneStart()
{
	
	//Enable DMA stream for I2S peripheral
	DMA_Cmd(DMA1_Stream3,ENABLE);
	
	//Enable the I2S peripheral
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

void dmaBufferSwap(short int* buffer )
{
	static int j = 0;
	unsigned int i;

	
	for(i = 0; i < 16; i=i+2)
	recordData[j++] = buffer[i]; /// 32768.0;->wht this
	
	if(j == BUFFER_MIC_SIZE){
		microphoneStop();
		j = 0;

		//Meter aqui semaphore para indicar fim de leitura
		//implementaçao em fase seguinte
		
	/*	xSemaphoreGiveFromISR(xSem_Rec_Finish, &xHigherPriorityTaskWoken);
		
		
		if(xHigherPriorityTaskWoken == pdTRUE)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
  
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);*/
	}


}









