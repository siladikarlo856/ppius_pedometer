/**
 * @file		pedometer.c
 * @brief		Functions related to step counting algorithm
 * @author  Marko Krizmancic
 */
#include "FreeRTOS.h"
#include "task.h"
#include "IMU.h"
#include "pedometer.h"
#include "configuration.h"
#include "utilities.h"

static float history[3] = {0};
static float avg_accel_value;
static portTickType t1 = 0;

/**
 * @fn          inc_average
 * @brief       Incremental average
 * @param				New value to add to the cumulative average
 * @return      Current cumulative average
*/
float inc_average(float value)
{
	static uint16_t num_of_items;
	static float old_average;
	float new_average;
	
	num_of_items++;
	new_average = old_average + (value - old_average) / num_of_items;
	old_average = new_average;
	
	return new_average;
}

/**
 * @fn          calibrate_IMU
 * @brief       Store average total acceleration for first 30 consecutive readings
 * @param				none
 * @return      none
*/
void calibrate_IMU(void)
{
	float total;
	for (int i=0; i<30; i++)
	{
		total = readTotalAccel();
		avg_accel_value = inc_average(total);
		nrf_delay_ms(50);
	}
}

/**
 * @fn          get_avg_accel
 * @brief       Get average total acceleration
 * @param				none
 * @return      Average total acceleration
*/
float get_avg_accel(void)
{
	return avg_accel_value;
}

/**
 * @fn          detect_step
 * @brief       Step detection algorithm
 * @param				Total acceleration at given time
 * @return      Step detected or not
*/
bool detect_step(float total_accel)
{
	portTickType t2;
	uint32_t time_passed;
	float peak;
	
	t2 = xTaskGetTickCount();
	time_passed = ticks_to_ms(t2 - t1);
	
	history[0] = history[1];
	history[1] = history[2];
	history[2] = total_accel - avg_accel_value;
	
	peak = history[1];
	
	if (peak > history[0] && peak > history[2])
	{
		if (peak < UPPER_TRESHOLD && peak > LOWER_TRESHOLD)
		{
			t1 = xTaskGetTickCount();
			if (time_passed >= MIN_TIME)
			{
				return 1;
			}
		}
	}
	
	return 0;
}
