/*******************************************************************************
 * @file 				task_LOG.c
 *
 * @brief       API to LOG task.
 *
 * @author			(FER)
 *
 * @year				2019.
 ******************************************************************************/
#ifndef __TASK_LOG_H__
#define __TASK_LOG_H__

/*	nrf LOG	*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

void Task_LOG_Init(void);

#endif //__TASK_LOG_H__
