#include "gyroscopeDriver.h"
#include "spiModule.h"
#include "i2cModule.h"

/******************************************************************************
*								Private Variables
*******************************************************************************/
l3gd20Data data;

void gyroInit() 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //enable SCK, MOSI, MISO and NSS GPIO clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; //configuring SCK, MISO, MOSI and NSS
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1; //configuring  NSS
	
	GPIO_Init(GPIOC, &GPIO_InitStruct); //initialize all parameters assigned above
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1); //SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1); //MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1); //MOSI
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1); //NSS
	
	/*GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct); //initialize all parameters assigned above
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); //SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); //SDA*/
}

int gyroStart()
{
	gyroInit();
	spiInit();
	GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
	
	uint8_t id = gyroRead(L3GD20_REGISTER_WHO_AM_I, ONE_TIME);
	if(id != L3GD20H_ID)
		return 0;
	
	gyroWrite(L3GD20_REGISTER_CTRL_REG1, 0x0F); // Switch to normal mode and enable all three channels
	gyroWrite(L3GD20_REGISTER_CTRL_REG4, 0x00); // choose resolution 250DPS
	return 1;
}	

void gyroWrite(l3gd20Registers_t reg, uint8_t data)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_1); //put NSS pin to LOW
	
	gyroTransmit(reg); //send register to write data on
	/*while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //wait until transfer is ready
	SPI_I2S_SendData(SPI1, reg); //send register to write data on*/
	
	gyroTransmit(data); //write data on register reg
	/*while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //wait until transfer is ready
	SPI_I2S_SendData(SPI1, data); //write data on register reg*/
	
	GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
}

uint8_t gyroRead(l3gd20Registers_t reg, readingType type)
{
	uint8_t readByte=0;
	uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
	
	if(type==ONE_TIME) //if we want to read once one register
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1);
		
		gyroTransmit(reg | 0x80);	 //append read bit
		readByte=gyroTransmit(DUMMY_BYTE);
		
		GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
	}
	else if(type==MULTIPLE_TIMES) //if we want to read multiple times incrementing address 
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_1); //put NSS pin to LOW
		
		gyroTransmit(L3GD20_REGISTER_OUT_X_L | 0x80 | 0x40);
		
		xlo=gyroTransmit(DUMMY_BYTE); 
		xhi=gyroTransmit(DUMMY_BYTE);
		ylo=gyroTransmit(DUMMY_BYTE);
		yhi=gyroTransmit(DUMMY_BYTE);
		zlo=gyroTransmit(DUMMY_BYTE);
		zhi=gyroTransmit(DUMMY_BYTE);
		
		GPIO_SetBits(GPIOC, GPIO_Pin_1); //put NSS pin to HIGH
		
		//shift values to create properly formed integer (low byte first)
		data.x = (int16_t)(xlo | (xhi << 8));
		data.y = (int16_t)(ylo | (yhi << 8));
		data.z = (int16_t)(zlo | (zhi << 8));
		
		//converter dados para dps
		data.x *= L3GD20_SENSITIVITY_250DPS; // 8.75 mdps/LSB
    data.y *= L3GD20_SENSITIVITY_250DPS;
    data.z *= L3GD20_SENSITIVITY_250DPS;
	}
	
	return readByte; //in case of MULTIPLE_TIMES, returning readByte means nothing
}

/*uint16_t gyroTransmit(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //indicates when the TX buffer is empty and ready for new data
	uint16_t d=(data<<8 | DUMMY_BYTE);
	uint16_t value=0;
	SPI_I2S_SendData(SPI1, (uint16_t)d);
	//SPI_I2S_SendData(SPI1, (uint8_t)data);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET); 
	value=(uint16_t)SPI_I2S_ReceiveData(SPI1);
	return value;  
}*/

uint8_t gyroTransmit(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //indicates when the TX buffer is empty and ready for new data
	SPI_I2S_SendData(SPI1, (uint8_t)data);
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE)==RESET); 
	return (uint8_t)SPI_I2S_ReceiveData(SPI1);  
}
