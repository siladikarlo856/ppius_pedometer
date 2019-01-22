#ifndef __MAIN_H_
#define __MAIN_H_

/*
		BLE FreeRTOS template

*/
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_nus.h"
#include "app_timer.h"


#include "pin_definitions.h"
#include "addresses.h"
#include "configuration.h"
#include "utilities.h"
#include "IMU_interrupts.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* tasks */
#include "task_CMD.h"
#include "task_BLE.h"
#include "task_LOG.h"
#include "task_IMU.h"
#include "task_CLK.h"
#include "task_SCR.h"

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

//FreeRTOS custom defines 
#define TASK_PRIO_HIGH			5
#define TASK_PRIO_NORMAL		4
#define TASK_PRIO_LOW				3

/* this defines are used for freertos semaphores and mutexes */
#define FREERTOS_SEMAPHORE_WAIT_LONG			1000
#define FREERTOS_SEMAPHORE_WAIT_SHORT			100

#endif //__MAIN_H_
