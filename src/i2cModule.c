#include "i2cModule.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"

void i2cInit()
{
	I2C_InitTypeDef I2C_InitStruct;
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	/* initialize the I2C_ClockSpeed member */
  I2C_InitStruct.I2C_ClockSpeed = 5000;
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

