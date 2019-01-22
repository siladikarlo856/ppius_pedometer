/*******************************************************************************
 * @file 				task_CLK.h
 *
 * @brief       API to CLK task.
 *
 * @author			(FER)
 *
 * @year				2018.
 ******************************************************************************/
#ifndef __TASK_CLK_H__
#define __TASK_CLK_H__

/*	nrf LOG	*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "configuration.h"
#include "utilities.h"

 typedef struct {
  uint8_t hours;
  uint8_t mins;
  uint8_t secs;
} myTime;
 
void setFormat(char *chr, myTime clock);

void Task_CLK_Init(void);
void updateTime();
void split(char * line);

#endif //__TASK_CLK_H__
