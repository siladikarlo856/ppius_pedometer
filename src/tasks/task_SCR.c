/*******************************************************************************
 * @file 				task_SCR.c
 *
 * @brief       Implementation of SCR task.
 *							This task is responsible for output on the screen.
 *
 * @author			(FER)
 *
 * @year				2019.
 ******************************************************************************/
#include "task_SCR.h"
#include "task_CLK.h"

extern bool bt_connected;

TaskHandle_t screen_task_handle;

extern uint16_t steps_taken;
extern uint8_t screen_number;
extern float temp;
extern volatile myTime myClock;

/*******************************************************************************
 * @fn          screen_task_function
 *
 * @brief       Task witch displays data on the screen.
 *
 * @param				pvParameter Is void pointer, not used.
 *
 * @return      None.
******************************************************************************/
static void screen_task_function (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	uint16_t steps;
	int8_t screen_id;
	float current_temp;
	char tmp_clock_string[9];
	
	while (true)
	{
		vTaskSuspendAll();
		{
			steps = steps_taken;
			screen_id = screen_number;
			current_temp = temp;
			setFormat(tmp_clock_string, myClock);
		}
		xTaskResumeAll();
		
		NRF_LOG_INFO("Screen id: %d", screen_id);
		
		switch (screen_id)
		{
			case SCREEN_OFF:
				
				break;
			case SCREEN_CLOCK:
				NRF_LOG_INFO("Clock: %s", tmp_clock_string);
				break;
	
			case SCREEN_STEPS:
				NRF_LOG_INFO("Steps taken: %d", steps);
				break;
			
			case SCREEN_TEMP:
				NRF_LOG_INFO("Temperature: %d", (int)current_temp);	
				break;
		}
		
		
		
		
		
		if(bt_connected)
		{
			printf("Steps taken: %d\r\n", steps);
		}
		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(SCREEN_TASK_DELAY));
	}
}

/*******************************************************************************
 * @fn          Task_SCR_Init
 *
 * @brief       Creates SCR task.
 *
 * @param				None.
 *
 * @return      None.
 ******************************************************************************/
void Task_SCR_Init(void)
{
	if (pdPASS != xTaskCreate(screen_task_function, "SCR", 256, NULL, 3, &screen_task_handle))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
}
