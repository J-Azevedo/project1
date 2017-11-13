#include "spiModule.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"



void spiInit()
{
	SPI_InitTypeDef *SPI_InitStruct;
	GPIO_InitTypeDef *GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //enable SCK, MOSI, MISO and NSS GPIO clocks
	
	/*(#) Peripherals alternate function: 
       (++) Connect the pin to the desired peripherals' Alternate Function (AF) 
            using GPIO_PinAFConfig() function
       (++) Configure the desired pin in alternate function by: 
            GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
       (++) Select the type, pull-up/pull-down and output speed via GPIO_PuPd, 
            GPIO_OType and GPIO_Speed members
       (++) Call GPIO_Init() function In I2S mode, if an external clock source is 
            used then the I2S CKIN pin should be also configured in Alternate 
            function Push-pull pull-up mode. */
	
	
	//can use GPIO_StructInit() to fill each GPIO_InitStruct member with its default value
	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
		
	/* SPI SCK pin configuration */	
	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_5; // GPIO pin		
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOA, GPIO_InitStruct);		
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); 
	
	/* SPI MOSI pin configuration */
  GPIO_InitStruct->GPIO_Pin =  GPIO_Pin_7; //isto sao masks logo n se substituem
  GPIO_Init(GPIOA, GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	/* SPI MISO pin configuration */
  GPIO_InitStruct->GPIO_Pin =  GPIO_Pin_6; //isto sao masks logo n se substituem
  GPIO_Init(GPIOA, GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	
	/* SPI NSS pin configuration */
  GPIO_InitStruct->GPIO_Pin =  GPIO_Pin_15; //isto sao masks logo n se substituem
  GPIO_Init(GPIOA, GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI1);
	
	/**
  * @brief  Fills each SPI_InitStruct member with its default value.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure which will be initialized.
  * @retval None
  */
	SPI_StructInit(SPI_InitStruct);
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
	/**
  * @brief  Configures the data size for the selected SPI.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  SPI_DataSize: specifies the SPI data size.
  *          This parameter can be one of the following values:
  *            @arg SPI_DataSize_16b: Set data frame format to 16bit
  *            @arg SPI_DataSize_8b: Set data frame format to 8bit
  * @retval None
  */
	SPI_DataSizeConfig(SPI1, SPI_DataSize_8b); 
}