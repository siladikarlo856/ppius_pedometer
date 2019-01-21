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

extern bool bt_connected;

TaskHandle_t screen_task_handle;


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
	while (true)
	{
		
		switch (screen_id)
		{
			case SCREEN_OFF:
			{
			}
			case SCREEN_CLOCK:
			{
				
			}
			case SCREEN_STEPS:
			{
				
			}
			case SCREEN_TEMP:
			{
				
			}
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
