#include "MAX30100.h"
#include <stdint.h>
#include <math.h>



#define I2C_TIMEOUT_MAX 0xFFFF
#define N 63
void parsing(uint8_t);

uint8_t FIFO[256], fifoptr=0, Sendptr=0;
uint16_t IR[64], RED[64], Filtered_IR[66];
uint8_t WRITE_PTR=0;


	max30100Data lastAcquiredData;//variable that stores the last data that was acquired
///unfiltered data for red and ir
uint16_t red[0xfff]={0}, ir[0xfff]={0};
//output data for red
float outputRED[4080];	
//final output data of ir
float outputIR[4080];
//auxiliar output data
float outputIR2[4080];
	static short int m=0;//index that indicates the number of samples read
//filter specifications	
#define NZEROS 5
#define NPOLES 5
#define GAIN   1.052148876e+00
static float xv[NZEROS + 1], yv[NPOLES + 1];
//filter specifications	
//lowpass 10 order 10
#define NZEROSLOW 10
#define NPOLES2LOW 10
#define GAINLOW   5.939718719e+05

static float xv2[NZEROSLOW + 1], yv2[NPOLES2LOW + 1];
	
	
	
enum I2C_direction {
	Read = 0,
	Write
};


int start(enum I2C_direction direction);
int write(uint8_t reg, uint8_t data);


/* Handle PD7 interrupt */
void EXTI9_5_IRQHandler(void) {
	static uint8_t MAX2=0, flag=0;
	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {		/* Make sure that interrupt flag is set */
		MAX2= ReadByte(0x00);
		WRITE_PTR= ReadByte(0x02);
//    start(Write);
//		write(0x02, 0x00 );
//		I2C_GenerateSTOP(I2C1, ENABLE);
//		start(Write);
//		write(0x03, 0x00 );
//		I2C_GenerateSTOP(I2C1, ENABLE);
//		start(Write);
//		write(0x04, 0x00 );
//		I2C_GenerateSTOP(I2C1, ENABLE);
		
		ReadFIFO();
		
		start(Write);
		write(0x04, 0x00);
		I2C_GenerateSTOP(I2C1, ENABLE);

		
    EXTI_ClearITPendingBit(EXTI_Line7);					/* Clear interrupt flag */
	}

}




/*************************************************************************************
 *														MODULE CONFIGURATION																	 *
 *************************************************************************************/
int ModuleConfig(){
	Mode mode = MAX30100_MODE_SPO2_HR;
	LEDPulseWidth led = MAX30100_SPC_PW_1600US_16BITS ;
	SamplingRate sp = MAX30100_SAMPRATE_200HZ;
	LEDCurrent IRcurr = MAX30100_LED_CURR_20_8MA ;
	LEDCurrent Redcurr = MAX30100_LED_CURR_27_1MA ;	
	uint8_t mod, inter, spo,l;
	m=0;
	fifoptr=0;

//----------------------------------------- SET MODE --------------------------------------------------------------	
	if(!start(Write)) return 0;
	if(!write(MAX30100_REG_MODE_CONFIGURATION, mode)) return 0;	
//------------------------------------- SET INTERRUPTION ----------------------------------------------------------	
	if(!start(Write)) return 0;
	if(!write(MAX30100_REG_INTERRUPT_ENABLE, 0x80)) return 0;				// Enable FIFO full interruption
//---------------- SET LEDS PULSE WIDTH, SAMPLING RATE AND SETTING THE HIGH RESOLUTION ----------------------------
	if(!start(Write)) return 0;
//	if(!write(MAX30100_REG_SPO2_CONFIGURATION, (led | (sp<<2) | MAX30100_SPC_SPO2_HI_RES_EN) ) ) return 0;	
	if(!write(MAX30100_REG_SPO2_CONFIGURATION, 0x4D )) return 0;	
//-------------------------------------- SET LEDS CURRENT ---------------------------------------------------------
	if(!start(Write)) return 0;
	if(!write(MAX30100_REG_LED_CONFIGURATION, (IRcurr | Redcurr<<4) ) ) return 0;	
	
	I2CStop();
	
	
mod= ReadByte(MAX30100_REG_MODE_CONFIGURATION);
inter= ReadByte(MAX30100_REG_INTERRUPT_ENABLE);
spo= ReadByte(MAX30100_REG_SPO2_CONFIGURATION);
l= ReadByte(MAX30100_REG_LED_CONFIGURATION);

	
	return 1;
}



/*************************************************************************************
 *																READ FIFO FUNCTION	  														 *
 *************************************************************************************/
int ReadFIFO(){
	unsigned short int i=0;

	
	if(!start(Write)) return 0;
//--------------------------------------- SEND FIFO_DATA ADDRESS ------------------------------------------------
	if(!setAddr(MAX30100_REG_FIFO_DATA)) return 0;
	
//------------------------------------------- REPEAT START ------------------------------------------------------	
	if(!start(Read)) return 0;
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	while (i<16){		
		if(i!=(15)) FIFO[(fifoptr++)&0xFF] = I2CReadACK();
		else FIFO[(fifoptr++)&0xFF] = I2CReadNACK();
		i++;
	}
	I2CStop();

//every time the FIFO gets full it calls the parsing function
	if(fifoptr>=0xf0)
	parsing(fifoptr);
//-------------------------------------------- STOP ------------------------------------------------- 
//	I2C_AcknowledgeConfig(I2C1, DISABLE);
//	I2C_GenerateSTOP(I2C1, ENABLE);
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
//		/* If the timeout delay is exeeded, exit with error code */
//		if ((timeout--) == 0) return 0;
//	} timeout = I2C_TIMEOUT_MAX;
	
	return 1;
}








/*********************************************************************************************************/
//----------------------------------- MINOR FUNCTIONS -----------------------------------------------------
/*********************************************************************************************************/



/*************************************************************************************
 *															 START FUNCTION 																		 *
 *************************************************************************************/
int start(enum I2C_direction direction){
	unsigned short int timeout = I2C_TIMEOUT_MAX;
	
//	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {				// wait until I2C1 is not busy anymore
//		if ((timeout--) == 0) return 0;
//	} timeout = I2C_TIMEOUT_MAX;
	
	
	I2C_GenerateSTART(I2C1, ENABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
					if ((timeout--) == 0) return 0;
		} timeout = I2C_TIMEOUT_MAX;
	
	
//------------------------ SEND DIRECTION ------------------------
	if(direction==Write){
		I2C_Send7bitAddress(I2C1, MAX30100_I2C_ADDRESS , I2C_Direction_Transmitter); 
			while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){			// Wait Slave has acknowledged address
				if ((timeout--) == 0) return 0;
			} timeout = I2C_TIMEOUT_MAX;				
	}	
	else {
		I2C_Send7bitAddress(I2C1, MAX30100_I2C_ADDRESS , I2C_Direction_Receiver); 
			while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){			// Wait Slave has acknowledged address
				if ((timeout--) == 0) return 0;
		} timeout = I2C_TIMEOUT_MAX;
	}
	return 1;
}



/*************************************************************************************
 *															 WRITE FUNCTION 																		 *
 *************************************************************************************/
int write(uint8_t reg, uint8_t data){
	unsigned short int timeout = I2C_TIMEOUT_MAX;
	I2C_SendData(I2C1, reg);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
			if ((timeout--) == 0) return 0;
		} timeout = I2C_TIMEOUT_MAX;
	
	
	I2C_SendData(I2C1, data);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {	
			if ((timeout--) == 0) return 0;	/* If the timeout delay is exeeded, exit with error code */
		} timeout = I2C_TIMEOUT_MAX;
	return 1;
}



/*************************************************************************************
 *														SEND ADDRESS FUNCTION 																 *
 *************************************************************************************/
int setAddr (uint8_t address){
	unsigned short int timeout = I2C_TIMEOUT_MAX;
	I2C_SendData(I2C1, address);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) return 0;	/* If the timeout delay is exeeded, exit with error code */
	} timeout = I2C_TIMEOUT_MAX;
	return 1;
}



/*************************************************************************************
 *																		READ ACK				  														 *
 *************************************************************************************/
/* This function reads one byte from the slave device and acknowledges the byte (requests another byte)*/
uint8_t I2CReadACK(){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );		//while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2C1);
	return data;
}



/*************************************************************************************
 *																		READ NACK		 		 															 *
 *************************************************************************************/
/* This function reads one byte from the slave device and doesn't acknowledge the recieved data */
uint8_t I2CReadNACK(){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_NACKPositionConfig(I2C1,I2C_NACKPosition_Current);
	I2C_GenerateSTOP(I2C1, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );		//while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2C1);
	return data;
}



/*************************************************************************************
 *																			STOP			  																 *
 *************************************************************************************/
void I2CStop(){
	I2C_GenerateSTOP(I2C1, ENABLE);	// Send I2C1 STOP Condition 
}



/*************************************************************************************
 *																	READ 1 BYTE		  																 *
 *************************************************************************************/
uint8_t ReadByte (uint8_t address){							/*	CORRECTED	*/
	uint8_t data=0;
	unsigned short int timeout = I2C_TIMEOUT_MAX;
	
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {				// wait until I2C1 is not busy anymore
		if ((timeout--) == 0) return 0;
	} timeout = I2C_TIMEOUT_MAX;
	
	
	I2C_GenerateSTART(I2C1, ENABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
			if ((timeout--) == 0) return 0;
		} timeout = I2C_TIMEOUT_MAX;
	
	
	I2C_Send7bitAddress(I2C1, MAX30100_I2C_ADDRESS , I2C_Direction_Transmitter); 
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){			// Wait Slave has acknowledged address
			if ((timeout--) == 0) return 0;
		} timeout = I2C_TIMEOUT_MAX;	
		
	if(!setAddr(address)) return 0;	
		
		
	I2C_GenerateSTART(I2C1, ENABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
			if ((timeout--) == 0) return 0;
		} timeout = I2C_TIMEOUT_MAX;
	
	
	I2C_Send7bitAddress(I2C1, MAX30100_I2C_ADDRESS , I2C_Direction_Receiver); 
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){			// Wait Slave has acknowledged address
		if ((timeout--) == 0) return 0;
	} timeout = I2C_TIMEOUT_MAX;	

	data = I2CReadNACK();
	
	I2C_GenerateSTOP(I2C1, ENABLE);	
	return data;
}



/*************************************************************************************
 *															  RESET MODULE   																		 *
 *************************************************************************************/
int ResetMAX(){
	if(!start(Write)) return 0;
	if(!write(MAX30100_REG_MODE_CONFIGURATION, MAX30100_MC_RESET)) return 0;	
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 1;
}









/*********************************************************************************************************/
//----------------------------------- DATA PROCESSING ----------------------------------------------------
/*********************************************************************************************************/



/*************************************************************************************
 *															  PARSING FIFO  																		 *
 *************************************************************************************/
void parsing(uint8_t ptr){



	int aux=0;
	int i=0;
//it reads the complete FIFO and parses it to the raw output arrays
	while(i<ptr)
	{
		aux=(FIFO[i++]<<8);
		ir[m]=aux;
		
		aux=FIFO[i++];
		ir[m]|=aux;
		aux=(FIFO[i++]<<8);
		red[m]=aux;
		aux=(FIFO[i++]);
		
		red[m]|=aux;
		m++;
	}
	if(m>=0xff0)
	{
		dataProcessing();
		m=0;
	}
	
}



/*************************************************************************************
* 						  DC FILTER  BUTTERWORTH HIGHPASS ORDER 5 FC=0.5  											*
*								FOR IR AND RED DATA																										*
 *************************************************************************************/


static void filterloop()
{
	//ir highpass filter
	for (int i = 0; i<4096; i++)
	{
		xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5];
		xv[5] = ir[i] / GAIN;
		yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5];
		yv[5] = (xv[5] - xv[0]) + 5 * (xv[1] - xv[4]) + 10 * (xv[3] - xv[2])
			+ (0.9033282853 * yv[0]) + (-4.6084763585 * yv[1])
			+ (9.4053079892 * yv[2]) + (-9.5984970908 * yv[3])
			+ (4.8983371457 * yv[4]);
		outputIR[i] = yv[5];
	}
//red highpass filter
	for (int i = 0; i<4096; i++)
	{
		xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5];
		xv[5] = ir[i] / GAIN;
		yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5];
		yv[5] = (xv[5] - xv[0]) + 5 * (xv[1] - xv[4]) + 10 * (xv[3] - xv[2])
			+ (0.9033282853 * yv[0]) + (-4.6084763585 * yv[1])
			+ (9.4053079892 * yv[2]) + (-9.5984970908 * yv[3])
			+ (4.8983371457 * yv[4]);
		outputRED[i] = yv[5];
	}
}


/*************************************************************************************
* 							 LOWPASS FILTER BUTTERWORTH 	ORDER 10 FC=10												 *
*                FOR IR DATA THAT WAS ALREADY FILTERED OF DC PART                    *
 *************************************************************************************/
//lowpass 10 order 10
static void filterloop2()
{
	for (int i = 0; i<4096; i++)
	{
		xv2[0] = xv2[1]; xv2[1] = xv2[2]; xv2[2] = xv2[3]; xv2[3] = xv2[4]; xv2[4] = xv2[5]; xv2[5] = xv2[6]; xv2[6] = xv2[7]; xv2[7] = xv2[8]; xv2[8] = xv2[9]; xv2[9] = xv2[10];
		xv2[10] = outputIR[i] / GAINLOW;
		yv2[0] = yv2[1]; yv2[1] = yv2[2]; yv2[2] = yv2[3]; yv2[3] = yv2[4]; yv2[4] = yv2[5]; yv2[5] = yv2[6]; yv2[6] = yv2[7]; yv2[7] = yv2[8]; yv2[8] = yv2[9]; yv2[9] = yv2[10];
		yv2[10] = (xv2[0] + xv2[10]) + 10 * (xv2[1] + xv2[9]) + 45 * (xv2[2] + xv2[8])
			+ 120 * (xv2[3] + xv2[7]) + 210 * (xv2[4] + xv2[6]) + 252 * xv2[5]
			+ (-0.0164796305 * yv2[0]) + (0.2309193459 * yv2[1])
			+ (-1.4737279370 * yv2[2]) + (5.6470743441 * yv2[3])
			+ (-14.4056874260 * yv2[4]) + (25.6017495970 * yv2[5])
			+ (-32.1597564880 * yv2[6]) + (28.2587879000 * yv2[7])
			+ (-16.6721933230 * yv2[8]) + (5.9875896298 * yv2[9]);
		outputIR2[i] = yv2[10];
	}
}

/*************************************************************************************
*			 											MEAN DIFFERENCE FILTER																	 *
* 													TO MAKE DATA MORE REGULAR																 *
 *************************************************************************************/
void meanDiff()
{
		int k = 0;
		outputIR[0]= outputIR[1]= outputIR[2]= outputIR[3]= outputIR[4]=
			outputIR[5]= outputIR[6]= (outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++]
			+ outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++]
			+ outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++]) / 15;
		for (k=7 ; k < 4073; k++)
		{
			float sum = 0;;
			for (int j = k - 7; j < k + 8; j++)
				sum += outputIR2[j];
			outputIR[k] = sum / 15;

		}
		k -= 7;
		outputIR[4077] = outputIR[4078] = outputIR[4079] = outputIR[4076] = outputIR[4073] =
			outputIR[4074] = outputIR[4075] = (outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++]
				+ outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++]
				+ outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++] + outputIR2[k++]) / 15;
}

/*************************************************************************************
*							BEAT PER MINUTE CALCULATOR																							*
**************************************************************************************/
int bpmCalculator()
{
	float bpm = 0;
	for (int i = 1000; i < 1500; i++)
	{
		if (outputIR[i] >= 0 && outputIR[i - 1] < 0)
			bpm++;
	}
	for (int i = 2000; i < 2500; i++)
	{
		if (outputIR[i] >= 0 && outputIR[i - 1] < 0)
			bpm++;
	}
	bpm = bpm / 2;
	return(int) bpm *12;
}


/*************************************************************************************
*							SPO2 CALCULATOR									   																			*
**************************************************************************************/
float spo2Calculator(void)
{
	float valueIR = 0;
	float valueRED = 0;
	float R = 0;
	float SPo2 = 0;
	for (int i = 2000; i < 2050; i++)
	{
		valueIR += outputIR[i];
		valueRED += outputRED[i];
	}
	valueIR =fabsf( valueIR / 50);
	valueRED = fabsf(valueRED / 50);

	R = (log(valueIR)*650E-9) / (log(valueRED)*950E-9);
	return SPo2 = (110 - 25 * R);
}


/*************************************************************************************
*							DATA PROCESSEMENT								   																			*
**************************************************************************************/

void dataProcessing(void)
{
	filterloop();
	lastAcquiredData.spo2=spo2Calculator();
	filterloop2();
	meanDiff();
	lastAcquiredData.bpm=bpmCalculator();
	
	//xSemaphoreTake( xSemaphore, ( TickType_t ) 0 )

}







