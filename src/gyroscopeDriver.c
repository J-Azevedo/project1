#include "gyroscopeDriver.h"
#include "stm32f4xx_spi.h"


void gyroInit(int8_t cs, int8_t miso, int8_t mosi, int8_t clk) 
{
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
}
