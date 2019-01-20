#ifndef __TASK_UART_H__
#define __TASK_UART_H__


#include "app_uart.h"
#include "ble_nus.h"
#include "boards.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define UART_TX_BUF_SIZE                256  /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256  /**< UART RX buffer size. */


#endif // __TASK_UART_H__
