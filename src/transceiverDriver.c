/* Includes ------------------------------------------------------------------*/
#include "transceiverDriver.h"
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "RFM69registers.h"
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "string.h"

//#include "startup_stm32f40_41xxx.s"

/******************************************************************************
*								Public Variables
*******************************************************************************/

#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         1 // XTAL OFF
#define RF69_MODE_STANDBY       2 // XTAL ON
#define RF69_MODE_LISTEN       3 // Listen Mode
#define RF69_MODE_RX            4 // RX MODE
#define RF69_MODE_TX            5 // TX MODE
//chyper key -> ea db da 47 a6 5a 59 6c d8 3d f1 08 fe 51 38 d8 
#define RF_LISTEN2_COEF_IDLE 	1//Duration of the Idle phase in Listen mode. tListenIdle = ListenCoefIdle* ListenResolIdle
#define RF_LISTEN3_COEF_RX 		0x26//Duration of the Rx phase in Listen mode (startup timeincluded)tListenRx = ListenCoefRx* ListenResolRx
//gPIO PD11 interrupt pin for transceiver
#define DUMMYBYTE 00
/******************************************************************************
*								Private Variables
*******************************************************************************/

   static volatile uint8_t _mode=0; //current mode
	 static const int serverNode=3;
	 static int ackReceived=0;
	typedef enum {NOTRECEIVED = 0, RECEIVED = 1} ackState;
/******************************************************************************
*								Private Headers
*******************************************************************************/

static void sgpioInit(void);
	void r_gpioInit();
static void r_writeReg(uint8_t addr, uint8_t value);
 //uint8_t  readReg(uint8_t addr);
static void sendFrame(const void *buffer);
static void timInit(void);
	short int r_spiTransmit(short int data);
 short int message[10];

/*****************************************************************************
*			Private Functions
******************************************************************************/
void r_spiInit()
{
	r_gpioInit();
	
	SPI_InitTypeDef SPI_InitStruct;
	SPI_StructInit(&SPI_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); //enable peripheral clock

	/* Initialize the SPI_Direction member */
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_Mode member */
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  /* initialize the SPI_DataSize member */
  //SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16  ; //5MHz clock
  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI3, &SPI_InitStruct); //this function applies the configurations made above

	/**
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2, 3, 4, 5 or 6 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
	SPI_Cmd(SPI3, ENABLE);
	
}

void r_gpioInit()
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //enable SCK, MOSI, MISO and NSS GPIO clocks
	
	/*enable NSS GPIO clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; //configuring SCK, MISO, MOSI and NSS
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); //SCK
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3); //MISO
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3); //MOSI
    	
	//SPI NSS pin configuration 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive	
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_8; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
GPIO_SetBits(GPIOA, GPIO_Pin_8); //put NSS pin to HIGH



}
	
static void sgpioInit(void)
{//PD11
	
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

	

  /* Enable clock for GPIOD */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);



	
	//interrupt pin configuration
	 /* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
    
	/* Tell system that you will use PD11 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
    
	/* PD11 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line10;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	//EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);
			EXTI_ClearITPendingBit(EXTI_Line10);
 
	/* Add IRQ vector to NVIC */
	/* PD11 is connected to EXTI_Line11, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn ;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
	
	
	


}


static void writeReg(uint8_t addr, uint8_t value)
{
	//chip select 
	int addr2=(((addr|0x80)<<8)|value);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8); //put NSS pin to LOW
	r_spiTransmit( addr2 );
		GPIO_SetBits(GPIOA, GPIO_Pin_8); //put NSS pin to HIGH
}

 int  readReg(int addr)
{

 short	int value=01;
	int addr2=(addr&0x7f)<<8;
	//chip select 
	GPIO_ResetBits(GPIOA, GPIO_Pin_8); //put NSS pin to LOW
value =	r_spiTransmit( addr2|0x00 );
	
//	value =spiTransmit( 0xAA);//read value that is stored in the register
	
	GPIO_SetBits(GPIOA, GPIO_Pin_8); //put NSS pin to HIGH
	return (value&0xff);

}

static void sendFrame(const void *buffer)
{
	short int bufferSize=0;
	short int message[9]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
	changeMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
	int i=8;
//	while(i!= RF_DIOMAPPING1_DIO0_00 )
//	i=readReg(REG_DIOMAPPING1);
	i=0;

	//First send address 

	writeReg(REG_FIFO ,serverNode);

	
	for(int i=0; i<7;i++)
	{
		writeReg(REG_FIFO ,message[i]);//transmit the message through the spi
	}
	
	changeMode(RF69_MODE_TX);//change the mode to transmit the value that we wrote in to the FIFO
	
}
short int r_spiTransmit(short int data)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET); //indicates when the TX buffer is empty and ready for new data
	SPI_I2S_SendData(SPI3, (short int)data);
	
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE)==RESET); 
	return (short int)SPI_I2S_ReceiveData(SPI3);  
}


void transceiverInit(void)
{
	//ver melhor sitio para por estas configura?oes
	/*configurations for the transceiver*/
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },//turn on the sequencer, turn off listen, put the transceiver on standby
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_50000   }, // speed of 250kbps->might be to fast testing needed
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_50000},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},	
    /* 0x07 */ { REG_FRFMSB, (uint8_t) RF_FRFMSB_433    },//put frequency of 433 MH<
    /* 0x08 */ { REG_FRFMID, (uint8_t)RF_FRFMID_433 },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) RF_FRFLSB_433 },
		/* 0x0D*/  { REG_LISTEN1, RF_LISTEN1_RESOL_RX_262000  |RF_LISTEN1_RESOL_IDLE_64 |RF_LISTEN1_CRITERIA_RSSIANDSYNC |RF_LISTEN1_END_01},//it goes to MODE when  PayloadReady or Timeout interrupt occurs	
    /* 0x0E*/  { REG_LISTEN2, RF_LISTEN2_COEF_IDLE}, // ListenCoefIdle is 1
		/* 0x0F*/	 { REG_LISTEN3, RF_LISTEN3_COEFRX_VALUE}, // ListenCoefRX is 0x26
		// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
		/* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
		/* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw) -> need to test this configuration
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    /* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE }, // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
		/* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // sync value
    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
   /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODE },
		/* 0x38 */ { REG_PAYLOADLENGTH, 0x07 }, // packet length of 10 bytes -> also needs testing
		/* 0x39 */ { REG_NODEADRS, 4 }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty 
   /* 0x3D */ 	{ REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_64BITS   | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF   }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent), AES encryption disable
	  /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
 };
		/*we initialize the pins we need*/
	 r_spiInit();


	int i=0;
 int value=(RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);

 while(i!= 0x24 )
	i=readReg(REG_VERSION);
  
		
  //	while(i!=0x2)
//	i=readReg(0x0C);
	//	  	while(i!=0xb)
//	i=readReg(0x04);
		//			while(i!=0x1a)
//	i=readReg(0x3);
 //  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1)!=0xAA );
//  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1)!=0x55 );
//	 do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1)!=0xAA );
		
 /*write our configurations in the transceiver*/
 for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
   			 writeReg(CONFIG[i][0], CONFIG[i][1]);


value=(RF_FDEVLSB_50000);
sgpioInit();
// changeMode(RF69_MODE_TX);
  while(i!= value )
	i=readReg(REG_FDEVLSB);
}

void changeMode(int newMode)
{
	if(newMode==_mode)
	{
		return;
	}
		
		switch(newMode)
		{
			/*change mode to transmiter*/
			case RF69_MODE_TX:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_TRANSMITTER));
				
			break;
			/*change mode to receiver*/
			case RF69_MODE_RX:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_RECEIVER));
				
			break;
			/*change mode to sleep*/
			case RF69_MODE_SLEEP:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_SLEEP));
				
			break;
			/*change mode to standby*/
			case RF69_MODE_STANDBY:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_STANDBY));
				
			break;
			/*change mode to listen*/
			case RF69_MODE_LISTEN:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_STANDBY));
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_LISTEN_ON));
				
			break;
	}
		
	//qualquer coisa para limpar semaphore se ouver algum taken
}


void transmitData(const void *transferBuffer)
{
	/*first and most important we stop the scheduler because the transmission of data is the most critical part 
	of our system and we can't have problems in this part	*/ //->review if this should be a critical section or not
	//to be implemented later
//while(ackReceived!=RECEIVED)
{
	/*send the frame through the transeiver using the sendFrame() function*/
	sendFrame(transferBuffer);

	int i=0;
	while(i!=0x8)
		i=(readReg(REG_IRQFLAGS2)&0x08);
	/*
		if the message was received we end the function and leave the critical section->->review if this should be a critical section or not
		if we did not receive the ACK we retransmit the message
	*/
	/*here we leave the critical section*/
	//to be implemented later


}
}


/* Handle PD11 interrupt */
void EXTI15_10_IRQHandler(void)
	{
		short int message[10]={0};
		int i=0;
		i++;
			/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		/* Do your stuff when PD0 is changed */
			
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}