#include "spiModule.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

void gpioInit(void);
void tGpioInit(void);


void spiInit()
{
	SPI_InitTypeDef SPI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	SPI_StructInit(&SPI_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //enable peripheral clock
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
	
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; //5MHz clock
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI1, &SPI_InitStruct); //this function applies the configurations made above
	
	SPI_Cmd(SPI1, ENABLE);
}

uint8_t spiTransmit(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //indicates when the TX buffer is empty and ready for new data
	SPI_I2S_SendData(SPI1, (uint8_t)data);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET); 
	return (uint8_t)SPI_I2S_ReceiveData(SPI1);  
}



	

