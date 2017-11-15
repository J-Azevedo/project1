#include "gyroscopeDriver.h"
#include "stm32f4xx_spi.h"
#include "i2cModule.h"

void oximeterInit(void)
{
	GPIO_InitTypeDef *GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //enable SDA, SCL  and SMBA (when used) GPIO clocks
	
	/*(#) Peripherals alternate function: 
        (++) Connect the pin to the desired peripherals' Alternate 
             Function (AF) using GPIO_PinAFConfig() function
        (++) Configure the desired pin in alternate function by:
             GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
        (++) Select the type, pull-up/pull-down and output speed via 
             GPIO_PuPd, GPIO_OType and GPIO_Speed members
        (++) Call GPIO_Init() function
             Recommended configuration is Push-Pull, Pull-up, Open-Drain.
             Add an external pull up if necessary (typically 4.7 KOhm).      */
	
		//here i could use GPIO_StructInit() to fill each GPIO_InitStruct member with its default value
	GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
		
	/* I2C1 SCL pin configuration */	
	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_8; // GPIO pin		
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOB, GPIO_InitStruct);		
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_I2C1); 
	
	/* I2C1 SDA pin configuration */
  GPIO_InitStruct->GPIO_Pin =  GPIO_Pin_9; //isto sao masks logo n se substituem
  GPIO_Init(GPIOB, GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
}
