/**
  ******************************************************************************
  * @file    transceiverDriver.h
  * @author  Joao Azevedo

  * @brief   This file contains all the functions prototypes for the SPI 
  *          firmware library. 
  ****************************************************************************** 


 Define to prevent recursive inclusion -------------------------------------*/


#ifndef __TRANSCEIVER_DRIVER
#define __TRANSCEIVER_DRIVER

/*******************************************************************************
* 														Public Defines          												  *
********************************************************************************/
//frequencies
#define RF69_433MHZ            43
//networwk id
#define networkID  						  10

//node ID
#define nodeID                  4
//chyper key-> 
#define CHYPHER_KEY_1 0xea
#define CHYPHER_KEY_2 0xdb
#define CHYPHER_KEY_3 0xda
#define CHYPHER_KEY_4 0x47
#define CHYPHER_KEY_5 0xa6
#define CHYPHER_KEY_6 0x5a 
#define CHYPHER_KEY_7 0x59 
#define CHYPHER_KEY_8 0x6c
#define CHYPHER_KEY_9 0xd8
#define CHYPHER_KEY_10 0x3d 
#define CHYPHER_KEY_11 0xf1
#define CHYPHER_KEY_12 0x08
#define CHYPHER_KEY_13 0xfe 
#define CHYPHER_KEY_14 0x51 
#define CHYPHER_KEY_15 0x38
#define CHYPHER_KEY_16 0xd8
               


/*******************************************************************************
* 														Public Function Headers 												  *
********************************************************************************/

void transceiverInit(void);
void changeMode(int newMode);
void transmitData(const void *transferBuffer);

 int  readReg(int addr);


















#endif //__TRANSCEIVER_DRIVER
