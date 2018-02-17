/* Includes ------------------------------------------------------------------*/
#include <FreeRTOS.h>
#include "i2sModule.h"
#include "dmaModule.h"
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

float recordData[BUFFER_MIC_SIZE];
PDMFilter_InitStruct filter;

/******************************************************************************
*								Private Headers
*******************************************************************************/

static void microphoneGPIOInit(void);

/*****************************************************************************
*			Private Functions
******************************************************************************/
static void microphoneGPIOInit(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
  
  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
}

/*****************************************************************************
*			Public Functions
******************************************************************************/
void microphoneInit()
{	
	/*Before calling the PDM_Filter_Init() function, the application code must enable the clock of
	the STM32 microcontroller CRC peripheral (configuration done in RCC register)*/
  // Enable CRC module  
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
  	
  /* Filter LP & HP Init */
  filter.LP_HZ = (I2S_AudioFreq_16k / 2.0); //Defines the low pass filter cut-off frequency. To reduce the sampling frequency
  filter.HP_HZ = 10; /*Defines the high pass filter cut frequency. To remove the signal DC offset,  It has
	been implemented via an IIR filter with a cut-off frequency below the audible frequency
	range in order to preserve signal quality.*/
  filter.Fs = I2S_AudioFreq_16k; //Defines the frequency output of the filter in Hz.
  filter.Out_MicChannels = 1; /*Defines the number of microphones in the output stream; this
	parameter is used to interlace different microphones in the output buffer. Each sample
	is a 16-bit value.*/
  filter.In_MicChannels = 1; /*Define the number of microphones in the input stream. This
	parameter is used to specify the interlacing of microphones in the input buffer. The
	PDM samples are grouped eight by eight in u8 format (8-bit).*/
  PDM_Filter_Init((PDMFilter_InitStruct *)&filter);

	microphoneGPIOInit();
  i2sInit();
	dmaInit();
}

void microphoneStart()
{
	STM_EVAL_LEDOn(LED3);
	
	//Enable DMA stream for I2S peripheral
	DMA_Cmd(DMA1_Stream3, ENABLE);
	while (DMA_GetCmdStatus(DMA1_Stream3) != ENABLE);
	
	//Enable the I2S peripheral
	if ((SPI2->I2SCFGR & 0x0400) == 0)
		I2S_Cmd(SPI2, ENABLE);
}

void microphoneStop()
{
	STM_EVAL_LEDOff(LED3);
	//Disable DMA stream for I2S peripheral
	DMA_Cmd(DMA1_Stream3,DISABLE);
	
	// Clear all the DMA flags 
  DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_FEIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3);
	
	//Disable the I2S peripheral
	I2S_Cmd(SPI2, DISABLE);
}
