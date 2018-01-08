#include "oximeterDriver.h"
#include "i2cModule.h"
#include "MAX30100_Registers.h"


/******************************************************************************
*								Public Variables
*******************************************************************************/

float irACValueSqSum;
float redACValueSqSum;
uint16_t samplesRecorded;
uint16_t pulsesDetected;
float currentSaO2Value;
uint8_t FIFO[256], fifoptr=0, Sendptr=0;
/******************************************************************************
*								Private Variables
//*******************************************************************************/
//static uint8_t currentPulseDetectorState;

//static float currentBPM;
//static float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
//static float valuesBPMSum;
//static uint8_t valuesBPMCount;
//static uint8_t bpmIndex;
//static uint32_t lastBeatThreshold;

//static SamplingRate samplingRate;
//static Mode mode;
//static LEDPulseWidth pulseWidth; 
//static LEDCurrent IrLedCurrent;

//static dcFilter_t dcFilterIR;
//static dcFilter_t dcFilterRed;
//static butterworthFilter_t lpbFilterIR;
//static meanDiffFilter_t meanDiffIR;




void EXTI_PD7(void)
{
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




int oximeterStart()
{
	char c=0;
		i2cInit();

//	IrLedCurrent = DEFAULT_IR_LED_CURRENT; //50mA
//	pulseWidth = DEFAULT_LED_PULSE_WIDTH; //1600us
//	mode = MAX30100_MODE_HR_ONLY;
//	samplingRate = DEFAULT_SAMPLING_RATE; //100hz
//	currentPulseDetectorState = PULSE_IDLE;
	if(ResetMAX()!=1)
		c++;
	EXTI_PD7();
	if(ModuleConfig()!=1)
		c++;

//	c=i2cRead(0x12,0xff);
//	c=ReadByte(0xff);
 // setMode( mode );

  //Check table 8 in datasheet on page 19. You can't just throw in sample rate and pulse width randomly. 100hz + 1600us is max for that resolution
 // setSamplingRate( samplingRate );
 // setLEDPulseWidth( pulseWidth ); //resolution 16 bits ADC for 1600us pulse width
//	setInterrupt( 0x80|0x20 ); //ALMOST_FULL and HR_RDY interrupts

//  IrLedCurrent = MAX30100_LED_CURRENT_27_1MA;
////	setLEDCurrents(0, IrLedCurrent );

//  dcFilterIR.w = 0;
//  dcFilterIR.result = 0;

//  dcFilterRed.w = 0;
//  dcFilterRed.result = 0;


//  lpbFilterIR.v[0] = 0;
//  lpbFilterIR.v[1] = 0;
//  lpbFilterIR.result = 0;

//  meanDiffIR.index = 0;
//  meanDiffIR.sum = 0;
//  meanDiffIR.count = 0;


//  valuesBPM[0] = 0;
//  valuesBPMSum = 0;
//  valuesBPMCount = 0;
//  bpmIndex = 0;
//  

//  irACValueSqSum = 0;
//  redACValueSqSum = 0;
//  samplesRecorded = 0;
//  pulsesDetected = 0;
//  currentSaO2Value = 0;

//  lastBeatThreshold = 0;

	
	return 0;
}

/*************************************************************************************
 *																READ FIFO FUNCTION	  														 *
 *************************************************************************************/
int ReadFIFO(){
	unsigned short int i=0;
	uint8_t buff[4];
	
	if(!i2cStart(Write)) return 0;
//--------------------------------------- SEND FIFO_DATA ADDRESS ------------------------------------------------
	if(!i2cSetAddr(MAX30100_REG_FIFO_DATA)) return 0;
	
//------------------------------------------- REPEAT START ------------------------------------------------------	
	if(!i2cStart(Read)) return 0;
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	while (i<16){		
		if(i!=15) FIFO[(fifoptr++)&0xFF] = I2CReadACK();
		else FIFO[(fifoptr++)&0xFF] = I2CReadNACK();
		i++;
	}
	i2cStop();
	
//-------------------------------------------- STOP ------------------------------------------------- 
//	I2C_AcknowledgeConfig(I2C1, DISABLE);
//	I2C_GenerateSTOP(I2C1, ENABLE);
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
//		/* If the timeout delay is exeeded, exit with error code */
//		if ((timeout--) == 0) return 0;
//	} timeout = I2C_TIMEOUT_MAX;
	
	return 1;
}


/* Handle PD7 interrupt */
void EXTI9_5_IRQHandler(void) {
	static uint8_t MAX2=0, READ_PTR=0, WRITE_PTR=0, flag=0;
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {		/* Make sure that interrupt flag is set */
		MAX2= i2cReadByte(0x00);
//		WRITE_PTR= ReadByte(0x02);
    i2cStart(Write);
		i2cWrite(0x02, 0x00 );
		I2C_GenerateSTOP(I2C1, ENABLE);
		i2cStart(Write);
		i2cWrite(0x03, 0x00 );
		I2C_GenerateSTOP(I2C1, ENABLE);
		i2cStart(Write);
		i2cWrite(0x04, 0x00 );
		I2C_GenerateSTOP(I2C1, ENABLE);
		
		ReadFIFO();
		
		i2cStart(Write);
		i2cWrite(0x04, 0x00);
		I2C_GenerateSTOP(I2C1, ENABLE);
//		READ_PTR= ReadByte(0x04);
//			//MAX2= ReadByte(0x00);
//    WRITE_PTR= ReadByte(0x02);
	//	USART_SendData(USART3,FIFO[Sendptr++]);
		
    EXTI_ClearITPendingBit(EXTI_Line7);					/* Clear interrupt flag */
	}

}
/*************************************************************************************
 *														MODULE CONFIGURATION																	 *
 *************************************************************************************/
int ModuleConfig()
	{
	Mode mode = MAX30100_MODE_SPO2_HR;
	LEDPulseWidth led = MAX30100_SPC_PW_1600US_16BITS ;
	SamplingRate sp = MAX30100_SAMPRATE_200HZ;
	LEDCurrent IRcurr = MAX30100_LED_CURR_20_8MA ;
	LEDCurrent Redcurr = MAX30100_LED_CURR_20_8MA ;	
	uint8_t mod, inter, spo,l;

//----------------------------------------- SET MODE --------------------------------------------------------------	
	if(!i2cStart(Write)) return 0;
	if(!i2cWrite(MAX30100_REG_MODE_CONFIGURATION, mode)) return 0;	
//------------------------------------- SET INTERRUPTION ----------------------------------------------------------	
	if(!i2cStart(Write)) return 0;
	if(!i2cWrite(MAX30100_REG_INTERRUPT_ENABLE, 0x80)) return 0;				// Enable FIFO full interruption
//---------------- SET LEDS PULSE WIDTH, SAMPLING RATE AND SETTING THE HIGH RESOLUTION ----------------------------
	if(!i2cStart(Write)) return 0;
//	if(!write(MAX30100_REG_SPO2_CONFIGURATION, (led | (sp<<2) | MAX30100_SPC_SPO2_HI_RES_EN) ) ) return 0;	
	if(!i2cWrite(MAX30100_REG_SPO2_CONFIGURATION, 0x4D )) return 0;	
//-------------------------------------- SET LEDS CURRENT ---------------------------------------------------------
	if(!i2cStart(Write)) return 0;
	if(!i2cWrite(MAX30100_REG_LED_CONFIGURATION, (IRcurr | Redcurr<<4) ) ) return 0;	
	
	i2cStop();
	
	
mod= i2cReadByte(MAX30100_REG_MODE_CONFIGURATION);
inter= i2cReadByte(MAX30100_REG_INTERRUPT_ENABLE);
spo= i2cReadByte(MAX30100_REG_SPO2_CONFIGURATION);
l= i2cReadByte(MAX30100_REG_LED_CONFIGURATION);

	
	return 1;
}
/*************************************************************************************
 *															  RESET MODULE   																		 *
 *************************************************************************************/
int ResetMAX()
	{
	if(!i2cStart(Write)) return 0;
	if(!i2cWrite(MAX30100_REG_MODE_CONFIGURATION, MAX30100_MC_RESET)) return 0;	
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 1;
}


/*
void readFrom(max30100Registers_t reg, int num, uint8_t* _buff)
{
	int k;
  int i = 0;
	I2C_GenerateSTART(I2C1, ENABLE); //beginTransmission(MAX30100_DEVICE)
	I2C_Send7bitAddress(I2C1, MAX30100_DEVICE, I2C_Direction_Transmitter); //send device address in write mode
	I2C_SendData(I2C1, reg);
	I2C_GenerateSTART(I2C1, ENABLE); //repeated start
	I2C_Send7bitAddress(I2C1, MAX30100_DEVICE, I2C_Direction_Receiver);	//send device address in read mode

  for(k=0; k<4; k++) // device may send less than requested (abnormal)
  {
    _buff[i++] = I2C_ReceiveData(I2C1);// faz increase automatico??
  }

  I2C_GenerateSTOP(I2C1, ENABLE); // end transmission
}
*/
/*int detectPulse(float sensor_value)
{
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD)
  {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return 0;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {
        uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;

        //This method sometimes glitches, it's better to go through whole moving average everytime
        //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
        //valuesBPMSum -= valuesBPM[bpmIndex];
        //valuesBPM[bpmIndex] = rawBPM;
        //valuesBPMSum += valuesBPM[bpmIndex];

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;
        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;
				
        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;
        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return 1;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }
      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return 0;
}*/
