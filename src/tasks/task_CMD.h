/*******************************************************************************
 * @file 				task_CMD.c
 *
 * @brief       API to CMD task.
 *
 * @author			Jurica Martincevic (FER)
 *
 * @year				2018.
 ******************************************************************************/

#ifndef TASK_CMD_H_
#define TASK_CMD_H_

/* standard libraries */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CMD_TASK_STACK_SIZE		512

#define CMD_BUFFER_SIZE				32

/**
 * this macro is used for sending response to commad. Command implementation is
 * independeant of physical layer and communication protocol.
 * R is string witch to send.
 */
#define CMS_SEND_RESPONSE(R)	printf((R))

void Task_CMD_Init(void);

#endif
