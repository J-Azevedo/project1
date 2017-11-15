/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"              // Keil::Device:StdPeriph Drivers:SPI
#include "I2S_Module.h"

/******************************************************************************
*								Private Headers
*******************************************************************************/
static void I2S_init(unsigned int rate);
static void I2S_GPIO_init(void);


/*****************************************************************************
*			Functions
******************************************************************************/


void I2S_module_init(unsigned int rate)
{
	I2S_init(rate);
	I2S_GPIO_init();
	
	
	
	
}
static void I2S_init(unsigned int rate)
{
	 I2S_InitTypeDef i2s_init_structure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	//I2S configuration
	SPI_DeInit(SPI2);
	i2s_init_structure.I2S_AudioFreq = rate*2;
  i2s_init_structure.I2S_Standard = I2S_Standard_LSB;
  i2s_init_structure.I2S_DataFormat = I2S_DataFormat_16b;
  i2s_init_structure.I2S_CPOL = I2S_CPOL_High;
  i2s_init_structure.I2S_Mode = I2S_Mode_MasterRx;
  i2s_init_structure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  // Initialize the I2S peripheral with the structure
  I2S_Init(SPI2, &i2s_init_structure);
	
	

	RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
	RCC_PLLI2SCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY)==RESET);
	
	
	
	
}


static void I2S_GPIO_init()
{
	GPIO_InitTypeDef gpio_Init_structure;

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  gpio_Init_structure.GPIO_Mode = GPIO_Mode_AF;
  gpio_Init_structure.GPIO_OType = GPIO_OType_PP;
  gpio_Init_structure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  gpio_Init_structure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  gpio_Init_structure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &gpio_Init_structure);
  
  /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
  
  /* SPI MOSI pin configuration */
  gpio_Init_structure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_Init(GPIOC, &gpio_Init_structure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
}
