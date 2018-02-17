#include "gyroscopeDriver.h"
#include "spiModule.h"
#include <stdlib.h>


/******************************************************************************
*								Private Headers
*******************************************************************************/

static void gyroInit(void);
static void gyroWrite(l3gd20Registers_t, uint8_t);


/******************************************************************************
*								Public Variables
*******************************************************************************/
l3gd20Data data_d;
l3gd20Data data_dps;



/******************************************************************************
*								Private Functions
*******************************************************************************/

static void gyroInit() 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1; //configuring  NSS
	
	GPIO_Init(GPIOC, &GPIO_InitStruct); //initialize all parameters assigned above
}

static void gyroWrite(l3gd20Registers_t reg, uint8_t data)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_1); //put NSS pin to LOW
	
	spiTransmit(reg); //send register to write data on
	spiTransmit(data); //write data on register reg
	
	GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
}

/*static uint8_t spiTransmit(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //indicates when the TX buffer is empty and ready for new data
	SPI_I2S_SendData(SPI1, (uint8_t)data);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET); 
	return (uint8_t)SPI_I2S_ReceiveData(SPI1);  
}*/

/******************************************************************************
*								Public Functions
*******************************************************************************/

void gyroStart(void)
{
	gyroInit();
	spiInit();
	GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
	
	uint8_t id = gyroReadOneByte(L3GD20_REGISTER_WHO_AM_I, ONE_TIME);
	while(id != L3GD20H_ID)
		id = gyroReadOneByte(L3GD20_REGISTER_WHO_AM_I, ONE_TIME);
	
	gyroWrite(L3GD20_REGISTER_CTRL_REG1, 0x0F); // Switch to normal mode and enable all three channels
	gyroWrite(L3GD20_REGISTER_CTRL_REG4, 0x00); // choose resolution 250DPS
}	

uint8_t gyroReadOneByte(l3gd20Registers_t reg, readingType type)
{
	uint8_t readByte=0;
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
		
	spiTransmit(reg | 0x80);	 //append read bit
	readByte=spiTransmit(DUMMY_BYTE);
		
	GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
	
	return readByte; //in case of MULTIPLE_TIMES, returning readByte means nothing
}

l3gd20Data gyroReadAxisValue()
{
	uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
	l3gd20Data axis;
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_1); //put NSS pin to LOW
	
	spiTransmit(L3GD20_REGISTER_OUT_X_L | 0x80 | 0x40);
	
	xlo=spiTransmit(DUMMY_BYTE); 
	xhi=spiTransmit(DUMMY_BYTE);
	ylo=spiTransmit(DUMMY_BYTE);
	yhi=spiTransmit(DUMMY_BYTE);
	zlo=spiTransmit(DUMMY_BYTE);
	zhi=spiTransmit(DUMMY_BYTE);
	
	GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
	
	//shift values to create properly formed integer (low byte first)
	axis.x = (int16_t)(xlo | (xhi << 8));
	axis.y = (int16_t)(ylo | (yhi << 8));
	axis.z = (int16_t)(zlo | (zhi << 8));
		
	//convert raw data to degrees per second, according to chosen sensitivity (250dps)
	axis.x *= L3GD20_SENSITIVITY_250DPS; // 8.75 mdps/digit, each number represents 8.75 mdps degrees
  axis.y *= L3GD20_SENSITIVITY_250DPS;
  axis.z *= L3GD20_SENSITIVITY_250DPS;
	
	return axis;
}
