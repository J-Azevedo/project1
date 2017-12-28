/* Includes ------------------------------------------------------------------*/
#include "gyroscopeDriver.h"
#include "gyroProcessingTask.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stdlib.h>
#include "main.h"
 
 #define INVALID_DATA -1
extern xQueueHandle xQGyroData;
static l3gd20Data lastcaptures[20];
float minimum[] = { 0.76125,0.4375,-1.14625,-0.5775,3.045,4.655,7.86625,9.02125,6.27375,3.255,3.59625,2.86125,5.985,4.47125,6.86875,9.47625,9.38875,8.30375,5.81875,5.39,2.3975,7.67375,5.03125,3.94625,7.7875,0.98,8.23375,4.76,4.59375,6.39625,1.435,4.76,1.56625,1.91625,0.5425,-0.60375,-2.70375,-1.6625,-4.69,-8.26,-10.5875,-10.61375,-8.93375,-13.475,-17.17625,-29.434999,-36.846249,-42.096249,-37.528751,-30.59,-26.48625,-28.23625,-26.311251,-22.7325,-31.9025,-37.563751,-40.118752,-30.870001,-29.268749,-30.309999,-39.19125,-41.046249,-38.228748,-33.451252,-28.647501,-26.827499,-31.66625,-30.380001,-26.06625,-30.0825,-39.235001,-47.888752,-27.1075,-26.57375,-37.47625,-34.352501,-36.994999,-30.266251,-27.83375,-22.653749,-21.77,-27.553751,-30.546249,-25.94375,-19.7925,-21.35,-19.55625,-21.4025,-19.5825,-21.647501,-20.282499,-18.34,-15.91625,-19.6,-15.05,-19.45125,-20.081249,-22.102501,-28.32375 };
float maximum[] = { 21.74375,33.634998,35.743752,34.168751,28.393749,23.598749,29.785,39.05125,37.40625,44.598751,31.9025,29.452499,32.165001,28.18375,26.748751,22.557501,23.782499,27.483749,24.237499,18.252501,18.296249,18.7775,19.512501,18.19125,21.139999,23.467501,20.641251,19.057501,18.4625,19.696251,18.0425,17.8325,14.70875,13.60625,14.3675,16.2575,18.252501,16.747499,13.06375,12.0225,12.3725,10.31625,10.26375,11.34875,13.02,11.4275,8.3125,5.83625,3.28125,2.485,2.75625,2.37125,2.05625,3.43,4.095,8.0675,8.925,7.39375,6.16875,2.5025,2.14375,2.4325,1.82875,2.3275,1.4,1.40875,1.1025,0.72625,-0.4375,-1.26875,-4.865,-10.0625,-2.8175,12.075,15.68,10.96375,3.75375,-1.0675,4.445,3.91125,4.6375,5.31125,5.355,10.15875,6.0025,2.835,5.9675,7.07,9.66,6.895,4.1475,6.67625,1.1025,3.78,1.44375,4.01625,6.11625,7.6475,6.3175 };
struct processementData
{
	//	l3gd20Data data;
	float f;	
	struct processementData *nextValue;
		
};

struct processementData *firstValue=NULL;
struct processementData *lastValue=NULL;

static int dataProcessement()
{
		 int i=0;
		 for(struct processementData *tmp=firstValue ;tmp->nextValue!=NULL;tmp=tmp->nextValue)
		 {
			 if( tmp->f<=(minimum[i]-0.1) ||tmp->f>=(maximum[i]+0.1))
				 return INVALID_DATA;
				i++;
		 }
		 return 0;

}

	//struct processementData values[100];
void gyroProcessingTask()
{
	static int i=0;
	int k=0;
	 
	struct processementData 	*newdata;
	struct processementData *auxiliarPointer;
	int heap= xPortGetFreeHeapSize() ;
newdata = ( struct processementData *)pvPortMalloc(sizeof(struct processementData)); 
	 if (newdata== NULL)
	 {
			k++;
        return;
	 }
	 xQueueReceive(xQGyroData,&newdata->f,portMAX_DELAY);
	 
	 if(i>99)
	 {
			lastValue->nextValue=newdata;
			lastValue=newdata;
			
			auxiliarPointer=firstValue->nextValue;
			vPortFree(firstValue);
			firstValue=auxiliarPointer;

		 if(dataProcessement()!=INVALID_DATA)
		 {
				//semaphore to sinalize correct movement
			 //and to initializa microphone recording
			 
			 k++;
		 }
		 

	 
	 }
	else
	{
		if(firstValue==NULL)
		 {
			firstValue=newdata;
			lastValue=newdata;
		 }
		 else
		 {
			 lastValue->nextValue=newdata;
			 lastValue=newdata;
			 if(i==98)
			 {
					 if(dataProcessement()!=INVALID_DATA)
		 {
				//semaphore to sinalize correct movement
			 k++;
		 }
			}
			 
		 }
		 i++;
	
	}
	


	



}

