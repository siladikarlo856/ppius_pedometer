/*******************************************************************************
 * @file 				task_IMU.c
 *
 * @brief       Implementation of IMU task.
 *
 * @author			(FER)
 *
 * @year				2019.
 ******************************************************************************/
#include "task_IMU.h"

extern bool bt_connected;

TaskHandle_t temperature_task_handle;
TaskHandle_t accel_task_handle;
TaskHandle_t pedo_task_handle;

/**@brief pedo task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void pedo_task_function (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	uint16_t steps;
	while (true)
	{
		steps = readSteps();
		SEGGER_RTT_WriteString(0, "Steps taken: ");
		SEGGER_RTT_printf(0, "%d\r\n", steps);
		
		if(bt_connected)
		{
			printf("Steps taken: %d\r\n", steps);
		}
		
		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(PEDO_TASK_DELAY));
	}
}

/**@brief temperature read task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void temperature_task_function (void * pvParameter)
{
	float temp = 0;
	temperature_units temp_unit = DEFAULT_TEMP_UNIT;
	
	while (true)
	{
		temp = readTemp(temp_unit);
		SEGGER_RTT_WriteString(0, "Current temperature: ");
		print_float(temp);
		if (temp_unit == CELSIUS) {SEGGER_RTT_WriteString(0, " °C\r\n");}
		else {SEGGER_RTT_WriteString(0, " °F\r\n");}
//		xQueueSendToBack(xQueue, &temp, 0);
		
		if(bt_connected)
		{
			printf("Current temperature: %.2f\r\n", temp);
		}
		
		// TODO remove!!!
		/**************************************************/
		uint8_t data;
		readRegister(&data, LSM6DS3_ACC_GYRO_D6D_SRC);
    /**************************************************/
		
		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(TEMPERATURE_TASK_DELAY));
	}
}

/**@brief acceleration read task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void accel_task_function (void * pvParameter)
{
	float average_accel;
	float accel_x, accel_y, accel_z, total;
	uint16_t steps_taken = 0;
	uint16_t last_steps_taken = 0;
	
	average_accel = get_avg_accel();
	
	while (true)
	{
		accel_x = readFloatAccel(IMU_ACCEL_X);
		accel_y = readFloatAccel(IMU_ACCEL_Y);
		accel_z = readFloatAccel(IMU_ACCEL_Z);
		total = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
		if (detect_step(total))
		{
			last_steps_taken = steps_taken;
			steps_taken++;
			printf("Steps: %d\r\n", steps_taken);
		}
			
		SEGGER_RTT_WriteString(0, "Total acceleration: ");
		print_float(total - average_accel);
		SEGGER_RTT_WriteString(0, " --- Steps: ");
		SEGGER_RTT_printf(0, "%d", steps_taken);
		SEGGER_RTT_WriteString(0, "\r\n");
		
		
		
		if(bt_connected)
		{
			//printf("Steps: %d\r\n", steps_taken);
		}

		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(ACCEL_TASK_DELAY));
	}
}


void Task_IMU_Init(void)
{
	/******************* Initialize I2C interface *******************/
		I2C_Init(SCL_PIN, SDA_PIN);
		NRF_LOG_INFO("TWI init... DONE!");
		/*********************** Initialize IMU *************************/
		NRF_LOG_INFO("IMU init...");
		IMU_Init();
		IMU_Init_Orient();
		IMU_Init_HW_Tap();
//	IMU_Init_Pedo();
	
		NRF_LOG_INFO("DONE!\r\n");
		NRF_LOG_INFO("Calibrating IMU...");
		calibrate_IMU();
		NRF_LOG_INFO("DONE!\r\n");
	
		if (pdPASS != xTaskCreate(temperature_task_function, "TMPT", 256, NULL, 3, &temperature_task_handle))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
		if (pdPASS != xTaskCreate(accel_task_function, "ACCEL", 256, NULL, 3, &accel_task_handle))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
		if (pdPASS != xTaskCreate(pedo_task_function, "PEDO", configMINIMAL_STACK_SIZE, NULL, 3, &pedo_task_handle))
		{
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
		}
}
