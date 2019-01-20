/*******************************************************************************
 * @file 				task_IMU.h
 *
 * @brief       API to IMU task.
 *
 * @author			(FER)
 *
 * @year				2018.
 ******************************************************************************/
#ifndef __TASK_IMU_H__
#define __TASK_IMU_H__

/*	nrf LOG	*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "I2C.h"
#include "IMU.h"
#include "pedometer.h"
#include "SparkFunLSM6DS3.h"

#include "pin_definitions.h"
#include "addresses.h"
#include "configuration.h"
#include "utilities.h"

void Task_IMU_Init(void);

#endif //__TASK_IMU_H__
