/*******************************************************************************
 * @file 				cmd.c
 *
 * @brief       Implementation of command support.
 *
 * @author			(FER)
 *
 * @year				2019.
 ******************************************************************************/

#include "cmd.h"

/* standard libraries */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Uncomment line bellow to see debug logs on BLE terminal */
//#define BLE_TERMINAL_DEBUG

/**
 * external functions, implemented in commands.c file.
 */
cmd_ret_code_e Help( uint8_t arg_cnt, uint8_t **args);

#define NUMBER_OF_COMMANDS	1

/**
 * in this part commands are registered, to add command add another entry
 * to commands array and increment NUMBER_OF_COMMANDS macro.
 * example of commands array entry:
 *		{
 *			.command_string = "example",
 *			.command_function = example,
 *			.command_info = "this is a example"
 *		}
 * when string "example" is received, function example is executed. Function
 * example must be defined outside and linked vith keyword extern.
 */
static command_t commands[NUMBER_OF_COMMANDS] = {
	{
		.command_string = "help",
		.command_function = Help,
		.command_info = "   - displays all commands\r\n"
	},
};

/*******************************************************************************
 * @fn          Help
 *
 * @brief       Prints all commands and info about them on BLE.
 *
 * @param				void
 *
 * @return      Return code, look for cmd_ret_code_e in cmd.h
 ******************************************************************************/
cmd_ret_code_e Help( uint8_t arg_cnt, uint8_t **args)
{
	uint8_t i;
	for(i = 0; i < NUMBER_OF_COMMANDS; i++)
	{
		printf("\r\n%s%s" , commands[i].command_string, commands[i].command_info);
	}
	
	return CMD_OK;
}

/*******************************************************************************
 * @fn          CMD_Run
 *
 * @brief       Function finds and executes command.
 *
 * @param				command_string Is pointer to string representing command.
 *
 * @return      Return code, look for cmd_ret_code_e in cmd.h
 ******************************************************************************/
cmd_ret_code_e CMD_Run(uint8_t *command_string, uint8_t arg_cnt, uint8_t **args)
{
	uint8_t i;
	
	#ifdef BLE_TERMINAL_DEBUG 
		printf(" Command string: %s", command_string);
		printf(" Param cnt: %d", arg_cnt);
		
		if(arg_cnt > 0) {
			for(i = 0; i < arg_cnt; i++){
				printf(" Param: %d:%d",i, *args[i] );		
			}
		}
	#endif
	
	for(i = 0; i < NUMBER_OF_COMMANDS; i++)
	{
		if(!strcmp(commands[i].command_string, (const char*)command_string))
		{
			return (commands[i].command_function)(arg_cnt, args);
		}
	}
	
	return CMD_UNKNOWN;
}
