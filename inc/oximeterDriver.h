/**
  ******************************************************************************
  * @file    oximeterDriver.h
  * @author  Joao 
  * @date    15-November-2017
  * @brief   This file contains all the functions prototypes for the heart rate and 
	oximeter sensor driver.
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OXIMETER_H__
#define __OXIMETER_H__

#include "stm32f4xx.h"

/* MAX30100 parameters */
#define DEFAULT_OPERATING_MODE            MAX30100_MODE_SPO2_HR    
/*!!!IMPORTANT
 * You can't just throw these two values at random. Check Check table 8 in datasheet on page 19.
 * 100hz + 1600us is max for that resolution
 */
#define DEFAULT_SAMPLING_RATE             MAX30100_SAMPLING_RATE_100HZ
#define DEFAULT_LED_PULSE_WIDTH           MAX30100_PULSE_WIDTH_1600US_ADC_16

#define DEFAULT_IR_LED_CURRENT            MAX30100_LED_CURRENT_50MA

/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_MS           500

/* SaO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES     4

/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha value
#define MEAN_FILTER_SIZE        15  

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD         100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1

#define PULSE_BPM_SAMPLE_SIZE       10 //Moving average size

#define HR_RDY (1<<5) //HR_RDY flag mask

/* Enums, data structures and typdefs. DO NOT EDIT */
struct pulseoxymeter_t {
  int pulseDetected;
  float heartBPM;

  float irCardiogram;

  float irDcValue;
  float redDcValue;

  float SaO2;

  uint32_t lastBeatThreshold;

  float dcFilteredIR;
  float dcFilteredRed;
};

typedef enum PulseStateMachine {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PulseStateMachine;

typedef struct fifo_t {
  uint16_t rawIR;
  uint16_t rawRed;
} fifo_t;

typedef struct dcFilter_t {
  float w;
  float result;
} dcFilter_t;

typedef struct butterworthFilter_t
{
  float v[2];
  float result;
} butterworthFilter_t;

typedef struct meanDiffFilter_t
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
} meanDiffFilter_t;

typedef enum {
		/* MAX30100 register and bit, DO NOT EDIT */
		MAX30100_DEVICE                   =0x57,

		//Part ID Registers
		MAX30100_REV_ID                   =0xFE,
		MAX30100_PART_ID                  =0xFF,

		//status registers
		MAX30100_INT_STATUS               =0x00,
		MAX30100_INT_ENABLE               =0x01,

		//Fifo registers
		MAX30100_FIFO_WRITE               =0x02,
		MAX30100_FIFO_OVERFLOW_COUNTER    =0x03,
		MAX30100_FIFO_READ                =0x04,
		MAX30100_FIFO_DATA                =0x05,

		//Config registers
		MAX30100_MODE_CONF                =0x06,
		MAX30100_SPO2_CONF                =0x07,
		MAX30100_LED_CONF                 =0x09,

		//Temperature registers
		MAX30100_TEMP_INT                 =0x16,
		MAX30100_TEMP_FRACTION            =0x17
} max30100Registers_t;

//Bit defines MODE Regsiter
#define MAX30100_MODE_SHDN                (1<<7)
#define MAX30100_MODE_RESET               (1<<6)
#define MAX30100_MODE_TEMP_EN             (1<<3)

typedef enum Mode {
    MAX30100_MODE_HR_ONLY                 = 0x02,
    MAX30100_MODE_SPO2_HR                 = 0x03
} Mode;

//Bit defines SpO2 register
#define MAX30100_SPO2_HI_RES_EN           (1 << 6)
typedef enum SamplingRate {
    MAX30100_SAMPLING_RATE_50HZ           = 0x00,
    MAX30100_SAMPLING_RATE_100HZ          = 0x01,
    MAX30100_SAMPLING_RATE_167HZ          = 0x02,
    MAX30100_SAMPLING_RATE_200HZ          = 0x03,
    MAX30100_SAMPLING_RATE_400HZ          = 0x04,
    MAX30100_SAMPLING_RATE_600HZ          = 0x05,
    MAX30100_SAMPLING_RATE_800HZ          = 0x06,
    MAX30100_SAMPLING_RATE_1000HZ         = 0x07
} SamplingRate;

typedef enum LEDPulseWidth {
    MAX30100_PULSE_WIDTH_200US_ADC_13     = 0x00,
    MAX30100_PULSE_WIDTH_400US_ADC_14     = 0x01,
    MAX30100_PULSE_WIDTH_800US_ADC_15     = 0x02,
    MAX30100_PULSE_WIDTH_1600US_ADC_16    = 0x03,
} LEDPulseWidth;

typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05,
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} LEDCurrent;

extern float irACValueSqSum;
extern float redACValueSqSum;
extern uint16_t samplesRecorded;
extern uint16_t pulsesDetected;
extern float currentSaO2Value;

void oximeterInit(void);
int oximeterStart(void);
void oximeterWrite(max30100Registers_t, uint8_t);
uint8_t oximeterRead(max30100Registers_t);
fifo_t oximeterReadFIFO(void);
void readFrom(max30100Registers_t, int, uint8_t*);

int detectPulse(float);

void setMode(Mode);
void setSamplingRate(SamplingRate);
void setLEDPulseWidth(LEDPulseWidth);
void setLEDCurrents(uint8_t, uint8_t);
void setInterrupt(uint8_t);

#endif /*__OXIMETER_H__*/
