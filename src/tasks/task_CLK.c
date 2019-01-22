/*******************************************************************************
 * @file 				task_CLK.c
 *
 * @brief       Implementation of CLK task
 *
 * @author			
 *
 * @year				2019.
 ******************************************************************************/
#include "task_CLK.h"
#define TIMER_PERIOD      1000 //inace treba u utitlies.h
#define CLOCK_TASK_DELAY 1000 //inace treba u configuration.h
#define ASCII2TIME(A,B) (((A)-'0')*10 + ((B)-'0'))

TaskHandle_t clock_task_handle;
TimerHandle_t clock_increment_timer_handle;
 
volatile myTime myClock = {
	.hours = 0,
	.mins = 0,
	.secs = 0,
};
 
void clock_increment_timer_callback (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	myClock.secs+=1;
}

/* Shared resource! */
char clockString[9];

static void clock_time_function (void * pvParameter)
{
	char clockString[9];
	while (true)
	{
		updateTime();
		// Show time
		setFormat(clockString, myClock);
		NRF_LOG_INFO("%s", clockString);
		NRF_LOG_INFO("\r\n");
		
		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(CLOCK_TASK_DELAY));		
	}
}

void updateTime() {
	if (myClock.secs > 59) {
		myClock.secs = 0;
		myClock.mins+=1;
	}
	if (myClock.mins > 59){
		myClock.mins = 0;
		myClock.hours+=1;
	}
	if (myClock.hours > 23) {
		myClock.hours  = 0;
	}	
}

void setFormat(char *chr, myTime clock)
{
  chr[0] = clock.hours/10 + '0';
  chr[1] = clock.hours%10 + '0';
  chr[2] = ':';
  chr[3] = clock.mins/10 + '0';
  chr[4] = clock.mins%10 + '0';
  chr[5] = ':';
  chr[6] = clock.secs/10 + '0';
  chr[7] = clock.secs%10 + '0';
  chr[8] = '\0';
}

void split(char * line) {
  myClock.hours = ASCII2TIME(line[0],line[1]);
  myClock.mins = ASCII2TIME(line[3],line[4]);
  myClock.secs = ASCII2TIME(line[6],line[7]);  
}

void Task_CLK_Init(void)
{
	clock_increment_timer_handle = xTimerCreate( "SEC", TIMER_PERIOD, pdTRUE, NULL, clock_increment_timer_callback);
	xTaskCreate(clock_time_function, "CLOCK", 256, NULL, 3, &clock_task_handle);
	UNUSED_VARIABLE(xTimerStart(clock_increment_timer_handle, 0));
}
