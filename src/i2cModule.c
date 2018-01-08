#include "i2cModule.h"
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>
#include <misc.h>
#include <stm32f4xx_gpio.h>

	// I2C: PB8 -> SCL, PB9 -> SDA,	INT -> PD7

//static void DMA_init(void);
volatile static unsigned short int bufferRx0[256];
volatile static unsigned short int bufferRx1[256];
volatile static unsigned short int bufferTx0[256];
volatile static unsigned short int bufferTx1[256];
// DMA_GetCurrentMemoryTarget() returns the index of the Memory target currently in use by the DMA Stream.




void i2c_init(){
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// Enabling GPIO peripheral clock ? I2C: PB6 for SCL, PB7 for SDA
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);	// Enabling I2C1 peripheral clock

// GPIO peripheral properties specification for I2C
	GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9; 		// Pin6 (SCL), Pin7 (SDA)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 							// Alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 					// Clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; 						// Open drain
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;// GPIO_PuPd_NOPULL;							// No pull resistors active
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);											// Setting GPIO peripheral corresponding bits
// Connection of GPIO to peripheral
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);	// SDA
	
// I2C struct initialization
	I2C_InitStructure.I2C_ClockSpeed = 400000; 								// 400kHz
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; 								// I2C mode
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 				// 50% duty cycle --> standard
	I2C_InitStructure.I2C_OwnAddress1 = 0x00; 								// own address, not relevant in master mode
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 							// enable acknowledge
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;		// set address length to 7 bit addresses
	
// Starting I2C
	I2C_Init(I2C1, &I2C_InitStructure);												// Setting I2C peripheral corresponding bits
	I2C_Cmd(I2C1, ENABLE);																		// Enables the complete I2C1 peripheral

	//DMA_init();

}





void EXTI_PD7(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /* Tell system that you will use PD0 for EXTI_Line7 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);
    
    /* PD0 is connected to EXTI_Line7 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line7;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
 
    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line7, which has EXTI7_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}











