/*******************************************************************************
 * @file 				task_SCR.h
 *
 * @brief       API to SCR task.
 *
 * @author			(FER)
 *
 * @year				2019.
 ******************************************************************************/
#ifndef __TASK_SCR_H__
#define __TASK_SCR_H__

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

#include "pin_definitions.h"
#include "addresses.h"
#include "configuration.h"
#include "utilities.h"

typedef enum {
	SCREEN_CLOCK	= 0,
	SCREEN_STEPS	= 1,
	SCREEN_TEMP		= 2,
	SCREEN_OFF		= (-1),
} SCREEN_ID_t;

void Task_IMU_Init(void);

#endif //__TASK_SCR_H__
