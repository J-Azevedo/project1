/* Includes ------------------------------------------------------------------*/
#include "gyroscopeDriver.h"
#include "gyroAcquisitionTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "main.h"

extern xQueueHandle xQGyroData;

float sampleData[] =	{ 2.730000,7.087500,4.243750,6.265000,16.441250,15.610000,20.440001,28.131250,25.611250,25.287500,24.307501,26.101250,24.508751,25.121250,26.748751,20.545000,19.871250,14.481250,11.112500,13.020000,18.296249,18.252501,19.512501,17.027500,13.545000,13.790000,15.758750,15.636250,18.462500,16.178751,11.926250,11.445000,5.923750,1.960000,0.542500,-0.603750,-2.703750,0.910000,-4.690000,-8.260000,-10.587500,-10.613750,-8.933750,-11.795000,-2.458750,-6.553750,-9.371250,-15.793750,-16.301250,-20.606251,-23.677500,-28.236250,-23.563749,-18.392500,-16.695000,-17.482500,-13.842500,-17.508751,-27.002501,-26.976250,-24.535000,-27.291250,-28.577499,-23.712500,-24.150000,-21.315001,-16.738750,-17.622499,-18.663750,-20.606251,-20.921249,-21.603750,-18.943750,-16.161249,-14.752501,-13.982500,-11.970000,-8.671250,-7.175000,-0.796250,4.637500,5.311250,5.355000,10.158750,0.770000,-0.551250,-2.231250,-4.252500,-1.741250,-4.453750,-3.080000,-5.416250,-3.368750,-1.863750,-0.883750,-0.393750,1.015000,1.505000,1.207500 };
void gyroAcquisitionTask()
{
static int i=0;
	//l3gd20Data data;
//static	l3gd20Data data[100];
//	static int dx[500];
//		static int dy[500];
	//	static int dz[500];
	l3gd20Data sdata;
float x=sampleData[i];
	//i++;
	//data[i++]=gyroReadAxisValue();

	i++;
	xQueueSendToBack(xQGyroData,&x,(TickType_t)50);
	vTaskDelay(50 / portTICK_RATE_MS);	
	/*if(i==99)
	{
			i++;
	}*/
	if(i==99)
		i=0;
	
//4.28
	//4.60
	//4.44
	//3.53
	//4.2125
//log > truedata02.log

}