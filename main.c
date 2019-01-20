#include "main.h"

extern bool bt_connected;

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
TaskHandle_t temperature_task_handle;
TaskHandle_t accel_task_handle;
TaskHandle_t pedo_task_handle;

// screen number to show on display
uint8_t screen_number = 0;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
		ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

		nrf_gpio_cfg_output(BLUE_LED);
		nrf_gpio_pin_set(BLUE_LED);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */




/**@brief pedo task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void pedo_task_function (void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	uint16_t steps;
	while (true)
	{
		steps = readSteps();
		SEGGER_RTT_WriteString(0, "Steps taken: ");
		SEGGER_RTT_printf(0, "%d\r\n", steps);
		
		if(bt_connected)
		{
			printf("Steps taken: %d\r\n", steps);
		}
		
		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(PEDO_TASK_DELAY));
	}
}

/**@brief temperature read task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void temperature_task_function (void * pvParameter)
{
	float temp = 0;
	temperature_units temp_unit = DEFAULT_TEMP_UNIT;
	
	while (true)
	{
		temp = readTemp(temp_unit);
		SEGGER_RTT_WriteString(0, "Current temperature: ");
		print_float(temp);
		if (temp_unit == CELSIUS) {SEGGER_RTT_WriteString(0, " °C\r\n");}
		else {SEGGER_RTT_WriteString(0, " °F\r\n");}
//		xQueueSendToBack(xQueue, &temp, 0);
		
		if(bt_connected)
		{
			printf("Current temperature: %.2f\r\n", temp);
		}
		
		// TODO remove!!!
		/**************************************************/
		uint8_t data;
		readRegister(&data, LSM6DS3_ACC_GYRO_D6D_SRC);
    /**************************************************/
		
		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(TEMPERATURE_TASK_DELAY));
	}
}

/**@brief acceleration read task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void accel_task_function (void * pvParameter)
{
	float average_accel;
	float accel_x, accel_y, accel_z, total;
	uint16_t steps_taken = 0;
	uint16_t last_steps_taken = 0;
	
	average_accel = get_avg_accel();
	
	while (true)
	{
		accel_x = readFloatAccel(IMU_ACCEL_X);
		accel_y = readFloatAccel(IMU_ACCEL_Y);
		accel_z = readFloatAccel(IMU_ACCEL_Z);
		total = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
		if (detect_step(total))
		{
			last_steps_taken = steps_taken;
			steps_taken++;
			printf("Steps: %d\r\n", steps_taken);
		}
			
		SEGGER_RTT_WriteString(0, "Total acceleration: ");
		print_float(total - average_accel);
		SEGGER_RTT_WriteString(0, " --- Steps: ");
		SEGGER_RTT_printf(0, "%d", steps_taken);
		SEGGER_RTT_WriteString(0, "\r\n");
		
		
		
		if(bt_connected)
		{
			//printf("Steps: %d\r\n", steps_taken);
		}

		/* Delay a task for a given number of ticks */
		vTaskDelay(APP_TIMER_TICKS(ACCEL_TASK_DELAY));
	}
}

void int_1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    screen_number++;
		screen_number%=MAX_SCREEN_NUMBER;
		NRF_LOG_INFO("In1 num: %d", screen_number);
		NRF_LOG_FLUSH();
}

void int_2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    screen_number++;
		screen_number%=MAX_SCREEN_NUMBER;
		NRF_LOG_INFO("Int 2 num: %d", screen_number);
		NRF_LOG_FLUSH();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize modules.
    log_init();
    clock_init();

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Initialize modules.
    timers_init();
    buttons_leds_init(&erase_bonds);
		/******************* Initialize BLE *******************/
		/**
		* this task must be initialized  early because it initializes softdevice whos
		* functions are used by all other tasks. It also initializes BLE for
		* bluetooth communication witch is used for sending commands to sensor.
		*/
		task_ble_init(&erase_bonds);
		/******************* Initialize I2C interface *******************/
		I2C_Init(SCL_PIN, SDA_PIN);
		NRF_LOG_INFO("TWI init... DONE!");
		/*********************** Initialize IMU *************************/
		NRF_LOG_INFO("IMU init...");
		IMU_Init();
		IMU_Init_Orient();
		IMU_Init_HW_Tap();
    
//		IMU_Init_Pedo();

		ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
		
		nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_INT_1, &in_config, int_1_handler);
    APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_gpiote_in_init(PIN_INT_2, &in_config, int_2_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_INT_1, true);
		nrf_drv_gpiote_in_event_enable(PIN_INT_2, true);

		
		NRF_LOG_INFO("DONE!\r\n");
		NRF_LOG_INFO("Calibrating IMU...");
		calibrate_IMU();
		NRF_LOG_INFO("DONE!\r\n");
		
		
		Task_CMD_Init();
		
  	  UNUSED_VARIABLE(xTaskCreate(temperature_task_function, "TMPT", 256, NULL, 3, &temperature_task_handle));
//		UNUSED_VARIABLE(xTaskCreate(accel_task_function, "ACCEL", 256, NULL, 3, &accel_task_handle));
//		UNUSED_VARIABLE(xTaskCreate(pedo_task_function, "pedo", configMINIMAL_STACK_SIZE, NULL, 3, &pedo_task_handle));

    NRF_LOG_INFO("HRS FreeRTOS example started.");
	 
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


