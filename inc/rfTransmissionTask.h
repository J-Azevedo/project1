/**
  ******************************************************************************
  * @file    gyroAcquisitionTask.h
  * @author  Joao Azevedo
  * @date    15-December-2017
  * @brief   This file contains all the functions prototypes for the gyroscope 
	(L3GD20) driver.
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RFTRANSMISSIONTASK_H__
#define __RFTRANSMISSIONTASK_H__

typedef struct rfMessage_t
{
  char type;
	char priority;
} rfMessage;

void rfTransmissionTask(void);


#endif /*__RFTRANSMISSIONTASK_H__*/


