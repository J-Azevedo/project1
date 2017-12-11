#include "spiModule.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

void gpioInit(void);
void tGpioInit(void);


void spiInit()
{
	gpioInit();
	SPI_InitTypeDef SPI_InitStruct;
	SPI_StructInit(&SPI_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //enable peripheral clock

	/* Initialize the SPI_Direction member */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  /* initialize the SPI_DataSize member */
  //SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16  ; //5MHz clock
  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI1, &SPI_InitStruct); //this function applies the configurations made above

	/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
	SPI_Cmd(SPI1, ENABLE);
	
}

void gpioInit()
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //enable SCK, MOSI, MISO and NSS GPIO clocks
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; //configuring SCK, MISO, MOSI and NSS
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1); //SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1); //MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1); //MOSI
    




}

void tSpiInit()
{
	tGpioInit();
	
	SPI_InitTypeDef SPI_InitStruct;
	SPI_StructInit(&SPI_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); //enable peripheral clock

	/* Initialize the SPI_Direction member */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  /* initialize the SPI_DataSize member */
  //SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16  ; //5MHz clock
  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI3, &SPI_InitStruct); //this function applies the configurations made above

	/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
	SPI_Cmd(SPI3, ENABLE);
	
}

void tGpioInit()
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //enable SCK, MOSI, MISO and NSS GPIO clocks
	
	/*enable NSS GPIO clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; //configuring SCK, MISO, MOSI and NSS
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); //SCK
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3); //MISO
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3); //MOSI
    	
	//SPI NSS pin configuration 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_8; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
GPIO_SetBits(GPIOA, GPIO_Pin_8); //put NSS pin to HIGH



}
	

