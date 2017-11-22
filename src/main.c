/* Scheduler includes. */
/*#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>*/

/* Library includes. */
#include <stm32f4xx.h>
#include "gyroscopeDriver.h"
#include "oximeterDriver.h"
#include "spiModule.h"
#include "i2cModule.h"
#include "usartModule.h"


int main()
{
	//usartInit();
	//i2cInit();
	if(!gyroStart()) //if it doesnt get the right slave ID
		return 0;
	//oximeterStart();
	gyroRead(L3GD20_REGISTER_OUT_X_L, MULTIPLE_TIMES); //first parameter is ignored
	//USART_SendData(USART1, 5);

	return 0;
}


