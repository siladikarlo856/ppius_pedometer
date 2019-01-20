/*******************************************************************************
 * @file 				cmd.h
 *
 * @brief       API and definitions for command support.
 *
 * @author			Jurica Martincevic (FER)
 *
 * @year				2018.
 ******************************************************************************/

#ifndef CMD_H_
#define CMD_H_

/* standard libraries */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum
{
	CMD_OK     				= 0,
	CMD_ERROR 				= 1,
	CMD_UNKNOWN 			= 2,
	CMD_FEW_PARAM 		= 3,
	CMD_MANY_PARAM 		= 4
}cmd_ret_code_e;

typedef struct
{
	char* command_string;
	char* command_info;
	cmd_ret_code_e (*command_function)(uint8_t arg_cnt, uint8_t **args);
}command_t;

cmd_ret_code_e CMD_Run(uint8_t *command_string, uint8_t arg_cnt, uint8_t **args);

#endif
