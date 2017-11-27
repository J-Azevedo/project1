/**
  ******************************************************************************
  * @file    gyroscopeDriver.h
  * @author  Joao 
  * @date    15-November-2017
  * @brief   This file contains all the functions prototypes for the gyroscope 
	(L3GD20) driver.
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "stm32f4xx.h"

#define L3GD20_ADDRESS                (0x6B)        // 110101+1 0x35+SDO on
#define L3GD20_POLL_TIMEOUT           (100)         // Maximum number of read attempts
#define L3GD20_ID                     0xD4
#define L3GD20H_ID                    0xD7

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)      // Roughly 22/256 for fixed point match
#define L3GD20_SENSITIVITY_500DPS  (0.0175F)       // Roughly 45/256
#define L3GD20_SENSITIVITY_2000DPS (0.070F)        // Roughly 18/256
#define L3GD20_DPS_TO_RADS         (0.017453293F)  // degress/s to rad/s multiplier

#define DUMMY_BYTE 										0xFF

typedef enum {ONE_TIME, MULTIPLE_TIMES} readingType;
  
typedef enum
{                                               // DEFAULT    TYPE
      L3GD20_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
      L3GD20_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
      L3GD20_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
      L3GD20_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
      L3GD20_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
      L3GD20_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
      L3GD20_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
      L3GD20_REGISTER_OUT_TEMP            = 0x26,   //            r
      L3GD20_REGISTER_STATUS_REG          = 0x27,   //            r
      L3GD20_REGISTER_OUT_X_L             = 0x28,   //            r
      L3GD20_REGISTER_OUT_X_H             = 0x29,   //            r
      L3GD20_REGISTER_OUT_Y_L             = 0x2A,   //            r
      L3GD20_REGISTER_OUT_Y_H             = 0x2B,   //            r
      L3GD20_REGISTER_OUT_Z_L             = 0x2C,   //            r
      L3GD20_REGISTER_OUT_Z_H             = 0x2D,   //            r
      L3GD20_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
      L3GD20_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
      L3GD20_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
      L3GD20_REGISTER_INT1_SRC            = 0x31,   //            r
      L3GD20_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
      L3GD20_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
      L3GD20_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
      L3GD20_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
      L3GD20_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
      L3GD20_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
      L3GD20_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
} l3gd20Registers_t;

typedef struct l3gd20Data_t
{
  float x;
  float y;
  float z;
} l3gd20Data;

extern l3gd20Data data;
		
void gyroInit(void);
int gyroStart(void);
void gyroWrite(l3gd20Registers_t, uint8_t);
uint8_t gyroRead(l3gd20Registers_t, readingType);
//uint16_t gyroTransmit(uint8_t);
uint8_t gyroTransmit(uint8_t);


#endif /*__GYROSCOPE_H__*/


