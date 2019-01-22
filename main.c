#include "main.h"

// screen number to show on display
extern uint8_t screen_number;


#if NRF_LOG_ENABLED
extern TaskHandle_t m_logger_thread;
#endif
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

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize modules.
    clock_init();
		Task_LOG_Init();
		
		// The best solution is to start the OS before any other initalisation.
		/******************* Initialize BLE *******************/
		/**
		* this task must be initialized  early because it initializes softdevice whos
		* functions are used by all other tasks. It also initializes BLE for
		* bluetooth communication witch is used for sending commands to sensor.
		*/
		Task_BLE_Init();
		Task_CMD_Init();
		Task_IMU_Init();
		
		IMU_Interrupts_Init();
		Task_CLK_Init();
		Task_SCR_Init();
	
    NRF_LOG_INFO("HRS FreeRTOS example started.");
	 
		// Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


