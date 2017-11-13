#include "gyroscopeDriver.h"
#include "stm32f4xx_spi.h"


void gyroInit(int8_t cs, int8_t miso, int8_t mosi, int8_t clk) 
{
	// GPIO structure declaration
	GPIO_InitTypeDef GPIO_InitStruct;
	// Enabling GPIO peripheral clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// GPIO peripheral properties specification
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5; // GPIO pin
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
