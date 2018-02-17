/**
  ******************************************************************************
  * @file    rfTransmissionTask.h
  * @author  Joao Azevedo and Joao Reis
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


