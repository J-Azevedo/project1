#include "oximeterDriver.h"
#include "i2cModule.h"

/******************************************************************************
*								Public Variables
*******************************************************************************/

float irACValueSqSum;
float redACValueSqSum;
uint16_t samplesRecorded;
uint16_t pulsesDetected;
float currentSaO2Value;


/******************************************************************************
*								Private Variables
*******************************************************************************/
static uint8_t currentPulseDetectorState;

static float currentBPM;
static float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
static float valuesBPMSum;
static uint8_t valuesBPMCount;
static uint8_t bpmIndex;
static uint32_t lastBeatThreshold;

static SamplingRate samplingRate;
static Mode mode;
static LEDPulseWidth pulseWidth; 
static LEDCurrent IrLedCurrent;

static dcFilter_t dcFilterIR;
static dcFilter_t dcFilterRed;
static butterworthFilter_t lpbFilterIR;
static meanDiffFilter_t meanDiffIR;


void oximeterInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //enable SDA, SCL  and SMBA (when used) GPIO clocks
	
	/*(#) Peripherals alternate function: 
        (++) Connect the pin to the desired peripherals' Alternate 
             Function (AF) using GPIO_PinAFConfig() function
        (++) Configure the desired pin in alternate function by:
             GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
        (++) Select the type, pull-up/pull-down and output speed via 
             GPIO_PuPd, GPIO_OType and GPIO_Speed members
        (++) Call GPIO_Init() function
             Recommended configuration is Push-Pull, Pull-up, Open-Drain.
             Add an external pull up if necessary (typically 4.7 KOhm).      */
	
		//here i could use GPIO_StructInit() to fill each GPIO_InitStruct member with its default value
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // push/pull 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // pullup/pulldown resistors inactive
		
	/* I2C1 SCL pin configuration */	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; // GPIO pin		
	// Setting GPIO peripheral corresponding bits
	GPIO_Init(GPIOB, &GPIO_InitStruct);		
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_I2C1); 
	
	/* I2C1 SDA pin configuration */
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_9; //isto sao masks logo n se substituem
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
}

int oximeterStart()
{
	oximeterInit();
	IrLedCurrent = DEFAULT_IR_LED_CURRENT; //50mA
	pulseWidth = DEFAULT_LED_PULSE_WIDTH; //1600us
	mode = MAX30100_MODE_HR_ONLY;
	samplingRate = DEFAULT_SAMPLING_RATE; //100hz
	currentPulseDetectorState = PULSE_IDLE;

  setMode( mode );

  //Check table 8 in datasheet on page 19. You can't just throw in sample rate and pulse width randomly. 100hz + 1600us is max for that resolution
  setSamplingRate( samplingRate );
  setLEDPulseWidth( pulseWidth ); //resolution 16 bits ADC for 1600us pulse width
	setInterrupt( 0x80|0x20 ); //ALMOST_FULL and HR_RDY interrupts

  IrLedCurrent = MAX30100_LED_CURRENT_27_1MA;
	setLEDCurrents(0, IrLedCurrent );

  dcFilterIR.w = 0;
  dcFilterIR.result = 0;

  dcFilterRed.w = 0;
  dcFilterRed.result = 0;


  lpbFilterIR.v[0] = 0;
  lpbFilterIR.v[1] = 0;
  lpbFilterIR.result = 0;

  meanDiffIR.index = 0;
  meanDiffIR.sum = 0;
  meanDiffIR.count = 0;


  valuesBPM[0] = 0;
  valuesBPMSum = 0;
  valuesBPMCount = 0;
  bpmIndex = 0;
  

  irACValueSqSum = 0;
  redACValueSqSum = 0;
  samplesRecorded = 0;
  pulsesDetected = 0;
  currentSaO2Value = 0;

  lastBeatThreshold = 0;

	oximeterWrite(MAX30100_FIFO_WRITE, 0);
	oximeterWrite(MAX30100_FIFO_READ, 0); //clear write and read pointers
	return 0;
}

// Writes val to address register on device
void oximeterWrite(max30100Registers_t reg, uint8_t data)
{
	I2C_GenerateSTART(I2C1, ENABLE); //beginTransmission(MAX30100_DEVICE)
	I2C_Send7bitAddress(I2C1, MAX30100_DEVICE, I2C_Direction_Transmitter);
	I2C_SendData(I2C1, reg);
	I2C_SendData(I2C1, data); //send value to write
	I2C_GenerateSTOP(I2C1, ENABLE);//endTransmission
}

uint8_t oximeterRead(max30100Registers_t reg)
{
	uint8_t readuint8_t;
	I2C_GenerateSTART(I2C1, ENABLE); //beginTransmission(MAX30100_DEVICE)
	I2C_Send7bitAddress(I2C1, MAX30100_DEVICE, I2C_Direction_Transmitter);
	I2C_SendData(I2C1, reg);
	
	I2C_GenerateSTART(I2C1, ENABLE);
	I2C_Send7bitAddress(I2C1, MAX30100_DEVICE, I2C_Direction_Receiver);
	readuint8_t=I2C_ReceiveData(I2C1); //requestFrom(MAX30100_DEVICE)
	
	I2C_GenerateSTOP(I2C1, ENABLE);//endTransmission
	return readuint8_t;
}

fifo_t oximeterReadFIFO() //read one sample
{
	fifo_t result;
  uint8_t buffer[4];
	int tmp;
	while(!tmp) //wait for HR_RDY flag set
	{
		tmp=oximeterRead(MAX30100_INT_STATUS);
		tmp&=HR_RDY;
	}
	/*uint8_t NUM_AVAILABLE_SAMPLES;
	int writePointer, readPointer;
	
	writePointer=oximeterRead(MAX30100_FIFO_WRITE);
	readPointer=oximeterRead(MAX30100_FIFO_READ);
	NUM_AVAILABLE_SAMPLES=writePointer-readPointer;*/
	
  readFrom( MAX30100_FIFO_DATA, 4, buffer );
  result.rawIR = (buffer[0] << 8) | buffer[1];
  result.rawRed = (buffer[2] << 8) | buffer[3];

  return result;
}

// Reads num uint8_ts starting from address register on device in to _buff array
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

void setLEDCurrents(uint8_t redLedCurrent, uint8_t IRLedCurrent)
{
  oximeterWrite( MAX30100_LED_CONF, (redLedCurrent << 4) | IRLedCurrent );
}

void setInterrupt(uint8_t interrupt)
{
	oximeterWrite(MAX30100_INT_ENABLE, 0x80|0x20); //enable ALMOST_FULL and HD_RDY interrupts
}

void setMode(Mode mode)
{
  uint8_t currentModeReg = oximeterRead(MAX30100_MODE_CONF);
  oximeterWrite( MAX30100_MODE_CONF, (currentModeReg & 0xF8) | mode );
}

void setSamplingRate(SamplingRate rate)
{
  uint8_t currentSpO2Reg = oximeterRead( MAX30100_SPO2_CONF );
  oximeterWrite( MAX30100_SPO2_CONF, ( currentSpO2Reg & 0xE3 ) | (rate<<2) );
}

void setLEDPulseWidth(LEDPulseWidth pw)
{
  uint8_t currentSpO2Reg = oximeterRead( MAX30100_SPO2_CONF );
  oximeterWrite( MAX30100_SPO2_CONF, ( currentSpO2Reg & 0xFC ) | pw );
}
