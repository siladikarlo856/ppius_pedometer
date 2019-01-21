/*******************************************************************************
 * @file 				task_CMD.c
 *
 * @brief       Implementation of CMD task, this task runs commands received
 *							from bluetooth.
 *
 * @author			
 *
 * @year				2019.
 ******************************************************************************/
#include "task_CMD.h"
#include "cmd.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


/* bluetooth */
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_nus.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"

#include "nrf_sdh.h"


//FreeRTOS custom defines 
#define TASK_PRIO_HIGH			5
#define TASK_PRIO_NORMAL		4
#define TASK_PRIO_LOW				3

/* this defines are used for freertos semaphores and mutexes */
#define FREERTOS_SEMAPHORE_WAIT_LONG			1000
#define FREERTOS_SEMAPHORE_WAIT_SHORT			100


/**
 * task handle for task witch execurtes commands form bluetooth,
 * used only for retreiving high water mark for task. This variable
 * is used only when debuging.
 */
static TaskHandle_t cmd_task_handle;

/**
 * this is queue witch is used to send charactersto CMD task by other tasks.
 */
QueueHandle_t cmd_queue;

/* internal buffer used by task for constructing command string */
static uint8_t cmd[CMD_BUFFER_SIZE];
static uint8_t cmd_index = 0;

/* constants witch are sent back as response to command */
static char *ok_str = "CMD_OK\r\n";
static char *error_str = "CMD_EROR\r\n";
static char *unknown_str = "CMD_UNKNOWN\r\n";


/*******************************************************************************
 * @fn          CMD_Send_Response
 *
 * @brief       This function sends response to command.
 *
 * @param				cmd_status Is status witch to sent as response to command.
 *
 * @return      None.
 ******************************************************************************/
static void CMD_Send_Response(cmd_ret_code_e cmd_status)
{
	switch(cmd_status)
	{
		case CMD_OK:
		{
			CMS_SEND_RESPONSE(ok_str);
			break;
		}
		case CMD_ERROR:
		{
			CMS_SEND_RESPONSE(error_str);
			break;
		}
		case CMD_UNKNOWN:
		{
			CMS_SEND_RESPONSE(unknown_str);
			break;
		}
		default:
		{
			
		}
	}
}

/*******************************************************************************
 * @fn          task_CMD
 *
 * @brief       Task witch executes commands.
 *
 * @param				p Is void pointer, not used.
 *
 * @return      None.
 ******************************************************************************/
static void task_CMD(void *p)
{
	cmd_ret_code_e cmd_status;
	uint8_t argc, i = 0;
	uint8_t *argv[30];
	
	while(1)
	{
		if(pdTRUE == xQueueReceive(cmd_queue, &cmd[cmd_index], FREERTOS_SEMAPHORE_WAIT_SHORT))
		{
			if(cmd[cmd_index] == '\0')
			{
		
				// parse the command line statement and break it up into space-delimited
				// strings. the array of strings will be saved in the argv array.
				i = 0;
				argv[i] = (uint8_t *)strtok((char *)cmd, " ");
				do
				{
						argv[++i] = (uint8_t *)strtok(NULL, " ");
				} while ((i < 30) && (argv[i] != NULL));

				// save off the number of arguments for the particular command.
				argc = i-1; // -1 because argv[i] = NULL
				
			 // parse DONE
				cmd_status = CMD_Run(argv[0], argc, &argv[1]);
				CMD_Send_Response(cmd_status);
				
				cmd_index = 0;
				memset(cmd, 0, sizeof(uint8_t) * CMD_BUFFER_SIZE);
			}
			else
			{
				cmd_index++;
			}
		}
	}
}

/*******************************************************************************
 * @fn          Task_CMD_Init
 *
 * @brief       Creates CMD task and initializes queue over witch CMD task
 *							receives characters to interpret as commands.
 *
 * @param				None.
 *
 * @return      None.
 ******************************************************************************/
void Task_CMD_Init(void)
{
	cmd_queue = xQueueCreate(128, sizeof(uint8_t));
	
	if (pdPASS != xTaskCreate(task_CMD,
														"CMD",
														CMD_TASK_STACK_SIZE,
														NULL,
														TASK_PRIO_NORMAL,
														&cmd_task_handle))
	{
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
#ifdef TASK_CMD_DEBUG
	IO_Send_Str("Task CMD created.\r\n");
#endif
}
