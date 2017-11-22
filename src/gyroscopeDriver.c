#include "gyroscopeDriver.h"
#include "spiModule.h"

/******************************************************************************
*								Private Variables
*******************************************************************************/
//static l3gd20Data data;
l3gd20Data data;

void gyroInit() 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
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
	
	
	//here i could use GPIO_StructInit() to fill each GPIO_InitStruct member with its default value
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
		
	/* SPI SCK pin configuration */	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5; // GPIO pin		->esta linha é que fode tudo
	
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOA, &GPIO_InitStruct);		
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  //Connect the pin to the desired peripherals' Alternate Function (AF)
	
	/* SPI MOSI pin configuration */
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_7; //isto sao masks logo n se substituem
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	/* SPI MISO pin configuration */
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	
	/* SPI NSS pin configuration */
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_4; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);
}

int gyroStart()
{
	gyroInit();
	spiInit();
	GPIO_SetBits(GPIOA, GPIO_Pin_4); //put NSS pin to HIGH
	
	uint8_t id = gyroRead(L3GD20_REGISTER_WHO_AM_I, ONE_TIME);
	if(id != L3GD20H_ID)
		return 0;
	
	gyroWrite(L3GD20_REGISTER_CTRL_REG1, 0x0F); // Switch to normal mode and enable all three channels
	gyroWrite(L3GD20_REGISTER_CTRL_REG4, 0x00); // choose resolution 250DPS
	return 1;
}	

void gyroWrite(l3gd20Registers_t reg, uint8_t data)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_5); //put SCK pin to HIGH
	GPIO_ResetBits(GPIOA, GPIO_Pin_4); //put NSS pin to LOW
	
	SPI_I2S_SendData(SPI1, reg); //send register to write data on
	SPI_I2S_SendData(SPI1, data); //write data on register reg
	
	GPIO_SetBits(GPIOA, GPIO_Pin_4); //put NSS pin to HIGH
}

uint8_t gyroRead(l3gd20Registers_t reg, readingType type)
{
	uint8_t readByte;
	uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
	
	if(type==ONE_TIME) //if we want to read once one register
	{
		//GPIO_SetBits(GPIOA, GPIO_Pin_5); //put SCK pin to HIGH
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); //put NSS pin to LOW
		//GPIO_ResetBits(GPIOA, GPIO_Pin_4); //put NSS pin to LOW
		
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //wait until transfer is complete
		SPI_I2S_SendData(SPI1, (uint8_t)reg | 0x80); //set READ bit to read after
				
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET); //wait until receive complete		
		readByte=(uint8_t)SPI_I2S_ReceiveData(SPI1);  //read most recent received data by the SPI1
		
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); //put NSS pin to HIGH
		//GPIO_SetBits(GPIOA, GPIO_Pin_4); //put NSS pin to HIGH
	}
	else if(type==MULTIPLE_TIMES) //if we want to read multiple times incrementing address 
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_5); //put SCK pin to HIGH
		GPIO_ResetBits(GPIOA, GPIO_Pin_4); //put NSS pin to LOW
		
		SPI_I2S_SendData(SPI1, L3GD20_REGISTER_OUT_X_L | 0x80 | 0x40); //SPI read, autoincrement
		
		xlo=(uint8_t)SPI_I2S_ReceiveData(SPI1); 
		xhi=(uint8_t)SPI_I2S_ReceiveData(SPI1);
		ylo=(uint8_t)SPI_I2S_ReceiveData(SPI1);
		yhi=(uint8_t)SPI_I2S_ReceiveData(SPI1);
		zlo=(uint8_t)SPI_I2S_ReceiveData(SPI1);
		zhi=(uint8_t)SPI_I2S_ReceiveData(SPI1);
		
		GPIO_SetBits(GPIOA, GPIO_Pin_4); //put NSS pin to HIGH
		
		//shift values to create properly formed integer (low byte first)
		data.x = (int16_t)(xlo | (xhi << 8));
		data.y = (int16_t)(ylo | (yhi << 8));
		data.z = (int16_t)(zlo | (zhi << 8));
		
		// Compensate values depending on the resolution
		data.x *= L3GD20_SENSITIVITY_250DPS;
    data.y *= L3GD20_SENSITIVITY_250DPS;
    data.z *= L3GD20_SENSITIVITY_250DPS;
	}
	
	return readByte; //in case of MULTIPLE_TIMES, returning readByte means nothing
}

