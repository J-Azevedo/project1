#include "spiModule.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"



void spiInit()
{
	SPI_InitTypeDef *SPI_InitStruct;

	/**
  * @brief  Fills each SPI_InitStruct member with its default value.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure which will be initialized.
  * @retval None
  */
	SPI_StructInit(SPI_InitStruct); //SPI_DataSize member=8bits
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //enable peripheral clock	
	
	/**
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the SPI_InitStruct.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
	SPI_Init(SPI1, SPI_InitStruct);
	/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
	SPI_Cmd(SPI1, ENABLE);
}

void spiWrite(uint8_t data)
{
	
}
