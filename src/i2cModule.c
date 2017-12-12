#include "i2cModule.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"

void i2cInit()
{
	I2C_InitTypeDef I2C_InitStruct;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	/* initialize the I2C_ClockSpeed member */
  I2C_InitStruct.I2C_ClockSpeed = 100000;
  /* Initialize the I2C_Mode member */
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  /* Initialize the I2C_DutyCycle member */
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  /* Initialize the I2C_OwnAddress1 member */
  I2C_InitStruct.I2C_OwnAddress1 = 0;
  /* Initialize the I2C_Ack member */
  I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
  /* Initialize the I2C_AcknowledgedAddress member */
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	I2C_Init(I2C1, &I2C_InitStruct); //this function applies the configurations made above
	
	I2C_Cmd(I2C1, ENABLE); //enable the I2C
}

void i2cWrite(uint8_t devAddr, uint8_t regAddr, uint8_t val)
{
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY ));
	I2C_GenerateSTART(I2C1, ENABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT ));
	I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Transmitter );

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED ));
	I2C_SendData(I2C1, regAddr);

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING ));
	I2C_SendData(I2C1, val);

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING ));
	I2C_GenerateSTOP(I2C1, ENABLE);
}

uint8_t i2cRead(uint8_t devAddr, uint8_t regAddr)
{
  uint8_t reg;
  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY ));
	I2C_GenerateSTART(I2C1, ENABLE);

  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT )); // wait for I2C1 EV5 --> Slave has acknowledged start condition
	I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Transmitter );

	I2C_SendData(I2C1, regAddr);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED )); // wait for I2C1 EV6, check if Slave has acknowledged Master transmitter 

  while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF ) == RESET);
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE); //repeated start

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT ));
	I2C_Send7bitAddress(I2C1, devAddr, I2C_Direction_Receiver );

  /*while (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR ) == RESET);
	I2C_AcknowledgeConfig(I2C1, DISABLE);*/

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED ));
	reg = I2C_ReceiveData(I2C1);

  I2C_GenerateSTOP(I2C1, ENABLE);

	return reg;
}

