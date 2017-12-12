/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "i2sModule.h"

/******************************************************************************
*								Private Headers
*******************************************************************************/


/*****************************************************************************
*			Functions
******************************************************************************/

void i2sInit(void)
{
	I2S_InitTypeDef I2S_InitStructure;

	RCC_PLLI2SConfig(258,3); //258 PLLI2SN, 3 PLLI2SR
	RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
	RCC_PLLI2SCmd(ENABLE);
  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = 32000;
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the Rx buffer not empty interrupt */
  //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

}


