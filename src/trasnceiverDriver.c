/* Includes ------------------------------------------------------------------*/
#include "transceiverDriver.h"
#include "stm32f4xx_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "RFM69registers.h"
#include "stm32f4xx_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI


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
/******************************************************************************
*								Private Variables
*******************************************************************************/

   static volatile uint8_t _mode=0; //current mode
/******************************************************************************
*								Private Headers
*******************************************************************************/

static void gpioInit(void);
static void writeReg(uint8_t addr, uint8_t value);
static uint8_t  readReg(uint8_t addr);



/*****************************************************************************
*			Private Functions
******************************************************************************/

static void gpioInit(void)
{//PD11
	
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
    
	/*enable NSS GPIO clocks*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
  /* Enable clock for GPIOD */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	
	//SPI NSS pin configuration 
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_15; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI1);
	
	//interrupt pin configuration
	 /* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
    
	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
    
	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);
 
	/* Add IRQ vector to NVIC */
	/* PD1 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
	

	
	//falta meter pino para interrupçao

}
static void writeReg(uint8_t addr, uint8_t value)
{
	//chip select 
	GPIO_SetBits(GPIOA, GPIO_Pin_15); //put NSS pin to HIGH

	SPI_I2S_SendData(SPI1, addr|0x80); //send register to write data on
	SPI_I2S_SendData(SPI1, value); //send register to write data on
	GPIO_ResetBits(GPIOA, GPIO_Pin_15); //put NSS pin to LOW
}

static uint8_t  readReg(uint8_t addr)
{

	uint8_t value;
	//chip select 
	GPIO_SetBits(GPIOA, GPIO_Pin_15); //put NSS pin to HIGH
	value =(uint8_t)SPI_I2S_ReceiveData(SPI1);//read value that is stored in the register
	GPIO_ResetBits(GPIOA, GPIO_Pin_15); //put NSS pin to LOW
	return value;

}






/*****************************************************************************
*			Public Functions
******************************************************************************/

void transceiverInit(void)
{
	//ver melhor sitio para por estas configuraçoes
  const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_250000   }, // speed of 250kbps->might be to fast testing needed
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_250000},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},	
    /* 0x07 */ { REG_FRFMSB, (uint8_t) RF_FRFMSB_433    },
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

		//may be wrong
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw) -> need to test this configuration

    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
  //  /* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE }, // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // sync value
    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_NODE },
    /* 0x38 */ { REG_PAYLOADLENGTH, 0xa }, // packet length of 10 bytes -> also needs testing
		/* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty ->also need test
		//need to test deçay
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_64BITS   | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_ON }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent), AES encryption enabled
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


	gpioInit();//falta meter aqui interrupçao do pino

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);
 
	changeMode(RF69_MODE_SLEEP);


}


void changeMode(int newMode)
{
	if(newMode==_mode)
	{
		return;
		
		switch(newMode)
		{
			case RF69_MODE_TX:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_TRANSMITTER));
				
			break;
			
			case RF69_MODE_RX:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_RECEIVER));
				
			break;
			
			case RF69_MODE_SLEEP:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_SLEEP));
				
			break;
			
			case RF69_MODE_STANDBY:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_STANDBY));
				
			break;
			
			case RF69_MODE_LISTEN:
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_STANDBY));
				writeReg(REG_OPMODE, ((readReg(REG_OPMODE)&0xe3)|RF_OPMODE_LISTEN_ON));
				
			break;
		}
	}
}

/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Do your stuff when PD0 is changed */
		
		
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
