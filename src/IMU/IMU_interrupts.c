#include "IMU_interrupts.h"

// screen number to show on display
uint8_t screen_number = 0;


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

void IMU_Interrupts_Init(void)
{
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
}
