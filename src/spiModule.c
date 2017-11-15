#include "spiModule.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

void spiInit()
{
	SPI_InitTypeDef *SPI_InitStruct;

	/* Initialize the SPI_Direction member */
  SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStruct->SPI_Mode = SPI_Mode_Slave;
  /* initialize the SPI_DataSize member */
  SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct->SPI_NSS = SPI_NSS_Hard;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct->SPI_CRCPolynomial = 7;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //enable peripheral clock	
	
	SPI_Init(SPI1, SPI_InitStruct); //this function applies the configurations made above

	/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
	SPI_Cmd(SPI1, ENABLE);
}

