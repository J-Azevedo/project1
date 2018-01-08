/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "event_groups.h"               // ARM.FreeRTOS::RTOS:Event Groups

#include "transceiverDriver.h"
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "RFM69registers.h"
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
#include "string.h"
#include "spiModule.h"
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
#define PACKET_SENT 1<<0
#define ACK_PENDING 1<<1
//gPIO PD11 interrupt pin for transceiver
/******************************************************************************
*								Private Variables
*******************************************************************************/
	 static EventGroupHandle_t xAckStatusFlags;
   static volatile uint8_t _mode=0; //current mode
	 static const int serverNode=3;
//	 static int ackReceived=0;
	typedef enum {NOTRECEIVED = 0, RECEIVED = 1} ackState;
/******************************************************************************
*								Private Headers
*******************************************************************************/


static void writeReg(uint8_t addr, uint8_t value);
static void sendFrame(const void *buffer);
//static void timInit(void);
 short int tSpiTransmit(short int data);


	
/*****************************************************************************
*			Private Functions
*******************************************************************************/
	
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
void eventGroupInitialization()
{
	xAckStatusFlags=xEventGroupCreate();

}
//static void timInit(void)
//{
//  NVIC_InitTypeDef NVIC_InitStruct;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	
//	//uint16_t PrescalerValue =  0xFA00; //get this right to the specific clock value
////	uint16_t period=665;
//	uint16_t PrescalerValue =  (42000-1); //get this right to the specific clock value
//	//uint16_t period=2000-1; ->2s
//	uint16_t period=20000-1;//->10s
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	 /* Compute the prescaler value */
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	int i=SystemCoreClock;
//  //PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;//need to be reviewed later to get real values for our project
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = period;
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	

//	/*timer interrupt configuration*/
//	TIM_ITConfig(TIM3, TIM_IT_Update,ENABLE);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//}

//static void timInit(void)
//{
//  NVIC_InitTypeDef NVIC_InitStruct;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	
//	//uint16_t PrescalerValue =  0xFA00; //get this right to the specific clock value
////	uint16_t period=665;
//	uint16_t PrescalerValue =  (42000-1); //get this right to the specific clock value
//	//uint16_t period=2000-1; ->2s
//	uint16_t period=20000-1;//->10s
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	 /* Compute the prescaler value */
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	int i=SystemCoreClock;
//  //PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;//need to be reviewed later to get real values for our project
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = period;
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//	

//	/*timer interrupt configuration*/
//	TIM_ITConfig(TIM3, TIM_IT_Update,ENABLE);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//	
//	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
//}

static void writeReg(uint8_t addr, uint8_t value)
{
	//chip select 

	GPIO_ResetBits(GPIOA, GPIO_Pin_8); //put NSS pin to LOW
	int spivalue=((addr|0x80)<<8)|value;
	tSpiTransmit(spivalue ); //send register to write data on

		GPIO_SetBits(GPIOA, GPIO_Pin_8); //put NSS pin to HIGH
}

 int  readReg(int addr)
{

 short	int value=01;
	int addr2=(addr&0x7f)<<8;
	//chip select 
	GPIO_ResetBits(GPIOA, GPIO_Pin_8); //put NSS pin to LOW
	value =	tSpiTransmit( addr2|0x00 );

	
	GPIO_SetBits(GPIOA, GPIO_Pin_8); //put NSS pin to HIGH
	return value&0xff;
}

static void sendFrame(const void *buffer)
{

	short int message[9]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
	changeMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
//  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
	int i=8;
	while(i!= RF_DIOMAPPING1_DIO0_00 )
	i=readReg(REG_DIOMAPPING1);
	i=0;


	//First send address 
	writeReg(REG_FIFO ,serverNode);


	for(int i=0; i<7;i++)
	{
		writeReg(REG_FIFO ,message[i]);//transmit the message through the spi
	}
	xEventGroupSetBits(xAckStatusFlags,PACKET_SENT|ACK_PENDING);//still need to see if this is needed
	changeMode(RF69_MODE_TX);//change the mode to transmit the value that we wrote in to the FIFO
	
}




 short int tSpiTransmit(short int data)
{
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET); //indicates when the TX buffer is empty and ready for new data
	SPI_I2S_SendData(SPI3, (short int)data);
	
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE)==RESET); 
	return (short int)SPI_I2S_ReceiveData(SPI3);  
}

/*****************************************************************************
*			Public Functions
******************************************************************************/

void transceiverInit(void)
{
	//ver melhor sitio para por estas configuraçoes
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
    /* 0x38 */ { REG_PAYLOADLENGTH, 0x0A }, // packet length of 10 bytes -> also needs testing
		/* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty ->also need test
		//need to test de?ay
   /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_64BITS   | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_ON }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent), AES encryption enabled
	// /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_64BITS   | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF },
	/*0x3E  */ {RegAesKey1,CHYPHER_KEY_1},//16 byte chypher key
		/*0x3f  */ {RegAesKey2,CHYPHER_KEY_2},
		/*0x40  */ {RegAesKey3,CHYPHER_KEY_3},
		/*0x41  */ {RegAesKey4,CHYPHER_KEY_4},
		/*0x42  */ {RegAesKey5,CHYPHER_KEY_5},
		/*0x43  */ {RegAesKey6,CHYPHER_KEY_6},
		/*0x44  */ {RegAesKey7,CHYPHER_KEY_7},
		/*0x45  */ {RegAesKey8,CHYPHER_KEY_8},
		/*0x46  */ {RegAesKey9,CHYPHER_KEY_9},
		/*0x47  */ {RegAesKey10,CHYPHER_KEY_10},
		/*0x48  */ {RegAesKey11,CHYPHER_KEY_11},
		/*0x49  */ {RegAesKey12,CHYPHER_KEY_12},
		/*0x4A  */ {RegAesKey13,CHYPHER_KEY_13},
		/*0x4b  */ {RegAesKey14,CHYPHER_KEY_14},
		/*0x4c  */ {RegAesKey15,CHYPHER_KEY_15},
		/*0x4d  */ {RegAesKey16,CHYPHER_KEY_16},		
	  /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
 };



 
	int i=0;
 //here we initialize the spi peripheral and the corrensponding GPIO pins
	tSpiInit();
	eventGroupInitialization();

// int value=(RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);

 while(i!= 0x24 )
	i=readReg(REG_VERSION);
/*do this operations to guarantee that the programing of the transceiver is working*/
  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1)!=0xAA );
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1)!=0x55 );

 /*write our configurations in the transceiver*/
 for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
   			 writeReg(CONFIG[i][0], CONFIG[i][1]);

 
changeMode(RF69_MODE_STANDBY);//put the transceiver in sleep
		sgpioInit();

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
	while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
	//qualquer coisa para limpar semaphore se ouver algum taken
}


/* Handle PD11 interrupt */
void EXTI15_10_IRQHandler(void)
{
		short int message[10]={0};
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		/* Do your stuff when PD0 is changed */
		

		
		/*we indicate that the ack was received*/
		if(xEventGroupGetBitsFromISR(xAckStatusFlags)==PACKET_SENT)
		{
			xEventGroupClearBitsFromISR(xAckStatusFlags,PACKET_SENT);
			
		}
		else
		{

		
			for(int i=0; i<0x2f; i++)
			{
				message[i]=readReg(REG_FIFO);
			
			}
			xEventGroupClearBitsFromISR(xAckStatusFlags,ACK_PENDING);
		
		}
				/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}



void transmitData(const void *transferBuffer)
{
	/*first and most important we stop the scheduler because the transmission of data is the most critical part 
	of our system and we can't have problems in this part	*/ //->review if this should be a critical section or not
	//to be implemented later
//while(ackReceived!=RECEIVED)
	EventBits_t uxBits;
//const TickType_t xTicksToWait = 5000 / portTICK_PERIOD_MS;
	
	while((uxBits&ACK_PENDING)!=ACK_PENDING)
	{
		


	/*send the frame through the transeiver using the sendFrame() function*/
	sendFrame(transferBuffer);
	

	//for now do this way for testing later use eventgroupgetbits			
		int i=0;
		while(i!=0x8)
		i=(readReg(REG_IRQFLAGS2)&0x08);
	
	
//	xEventGroupGetBits->use this later
	  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // DIO1 is "PayloadReady"
	/*
		changeMode(RF69_MODE_RX);
	*/
	//wait for a 1 second to get the values o the bits in the event group
	 uxBits = xEventGroupWaitBits(xAckStatusFlags,ACK_PENDING,pdTRUE,pdFALSE,(1000/ portTICK_PERIOD_MS));

	
	/*
	after this happened it will go to the beggining of the while and it will test the flag of the event group
	correspondent to the ack received flag, if the flag is on the transmission ocorred successfully and we can continue the program execution
	if the bit was not set we resend the message again
*/



	}
		/*->critical secticon is to be talked and discussed with Apu
		if the message was received we end the function and leave the critical section->->review if this should be a critical section or not
		if we did not receive the ACK we retransmit the message
	*/
	/*here we leave the critical section and do something to indicate the end of the message, maybe*/
	//to be implemented later 
}
