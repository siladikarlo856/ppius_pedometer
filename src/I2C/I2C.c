/**
 * @file		I2C.c
 * @brief		I2c driver for nrf5 series
 * @author  Ilija Domislovic
 */
 
#include "I2C.h"
#include "nrf_twi.h"
#include <stdint.h>

//varijable
static uint8_t readBuffer[2];
static ret_code_t ret;
static uint8_t send[32];
static uint8_t currentRegValue;
static uint8_t send_data[2];


#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

static const nrf_drv_twi_t	twi_instance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static volatile bool twi_xfer_done = false;

/*******************************************************************************
 * @fn          I2C_init
 *
 * @brief       Initialize I2C driver
 *
 * @param				The clock and data pins that will be used
 *
 * @return      none
 ******************************************************************************/

void I2C_Init(uint8_t scl, uint8_t sda)
{
	const nrf_drv_twi_config_t Config = 
	{
		 .scl                = scl,
		 .sda                = sda,
		 .frequency          = NRF_DRV_TWI_FREQ_100K,
		 .interrupt_priority = APP_IRQ_PRIORITY_THREAD,
		 .clear_bus_init     = false
	};
		
	ret = nrf_drv_twi_init(&twi_instance, &Config, Handler, NULL);	
	APP_ERROR_CHECK(ret); 
	nrf_drv_twi_enable(&twi_instance); 
}

/*******************************************************************************
 * @fn          Handler
 *
 * @brief       Callback function for events of I2C driver
 *
 * @param				The event type flag and the context of the event
 *
 * @return      none
 ******************************************************************************/

void Handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

	switch (p_event->type)
	{
			case NRF_DRV_TWI_EVT_DONE:
				
				if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) //RX transfer 
				{
					//read_data_handler(readBuffer);
					//printf("1\r\n");
				}
				else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX) //TX transfer
				{
					//write_data_handler(writeBuffer);
					//printf("2\r\n");
				}
				else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX) //TX transfer followed by RX with repeated start
				{
					//read_data_handler(readBuffer);
					//printf("3\r\n");
				}

				else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXTX) //TX transfer followed by TX with repeated start
				{
					//write_data_handler(writeBuffer);
					//printf("4\r\n");
				}
				else
				{
					//printf("Something's wrong with handler\r\n");
				}
				twi_xfer_done = true;
				
				break;
			case NRF_DRV_TWI_EVT_ADDRESS_NACK:
				//printf("Error-> NACK received after sending the address\r\n");
				twi_xfer_done = true; // Continue after error
				break;
			case NRF_DRV_TWI_EVT_DATA_NACK:
				//printf("Error-> NACK received after sending the data byte\r\n");
				twi_xfer_done = true; // Continue
				break;
			default:
				//NRF_LOG_INFO("\nError I2C\n");
				twi_xfer_done = true; // Continue
				break;
	}
}

/*******************************************************************************
 * @fn          I2C_Read
 *
 * @brief       Reads a single byte from specified slave address
 *
 * @param				The I2C slave address and the register address being read
 *
 * @return      The value ont the address (uint8_t)
 ******************************************************************************/

uint8_t I2C_Read(uint8_t u8SlaveAddress, uint8_t u8RegAddress)
{
	twi_xfer_done = false;
	
	ret = nrf_drv_twi_tx(&twi_instance, u8SlaveAddress, &u8RegAddress, 1, true);
	APP_ERROR_CHECK(ret);
	
	while (twi_xfer_done == false);

	if (ret == NRF_SUCCESS)
	{
		twi_xfer_done = false;
		ret = nrf_drv_twi_rx(&twi_instance, u8SlaveAddress, readBuffer, 1);
		APP_ERROR_CHECK(ret);
		
		while (twi_xfer_done == false);

		
		return *readBuffer;		
	}
	else
	{
		return 0;
	}

}

/*******************************************************************************
 * @fn          I2C_Is_Ready
 *
 * @brief       Function that signalizes if the I2C driver is in use
 *
 * @param				none
 *
 * @return      true if driver is not in use false atherwise (bool)
 ******************************************************************************/

bool I2C_Is_Ready(void){
	return twi_xfer_done;
}

/*******************************************************************************
 * @fn          I2C-Multi_Read
 *
 * @brief       Function used to read multiple bytes from slave
 *
 * @param				The slave address, the address inside the slave, 
								the buffer for data storage and number of bytes to read
 *
 * @return      none
 ******************************************************************************/

void I2C_Multi_Read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length)
{

	twi_xfer_done = false;

	ret = nrf_drv_twi_tx(&twi_instance, address,&reg, 1, true);
	APP_ERROR_CHECK(ret);

	while (twi_xfer_done == false);

	twi_xfer_done = false;
	ret = nrf_drv_twi_rx(&twi_instance, address, buffer, length);
	APP_ERROR_CHECK(ret);

	while (twi_xfer_done == false);

	return ;

}

/*******************************************************************************
 * @fn          I2C_Multi_Write
 *
 * @brief       Function used to send multiple bytes to a slave, MAX 32 bytes
 *
 * @param				The slave address, the address inside the slave, 
								the buffer of data to send and the nuber of bytes to send
 *
 * @return       none
 ******************************************************************************/

void I2C_Multi_Write(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length)
{

	send[0]=reg;
	for(int i=1;i<=length;i++){
		send[i]=buffer[i-1];
	}
	twi_xfer_done = false;

	ret = nrf_drv_twi_tx(&twi_instance, address,send, length+1, true);
	APP_ERROR_CHECK(ret);

	while (twi_xfer_done == false);
		
	return ;

}

/*******************************************************************************
 * @fn          I2C_Write
 *
 * @brief       Function that send a single byte to a slave
 *
 * @param				The slave address, the address in slave where the data will be sent,
								the data to send and will the old data be erased or preserved
 *
 * @return      none
 ******************************************************************************/

void I2C_Write(uint8_t u8SlaveAddress, uint8_t u8RegAddress, uint8_t u8Data, bool append) //if append is false, writing is done with whole fresh value, 
{
	
	twi_xfer_done = false;
	
	if (append == false)
	{
		
		send_data[0] = u8RegAddress;
		send_data[1] = u8Data;
		
		ret = nrf_drv_twi_tx(&twi_instance, u8SlaveAddress, (uint8_t *)send_data, 2, false);
		APP_ERROR_CHECK(ret);
		
		while (twi_xfer_done == false);
		
	}
	else if (append == true)
	{
		currentRegValue = I2C_Read(u8SlaveAddress, u8RegAddress);
		
		send_data[0] = u8RegAddress;
		send_data[1] = (currentRegValue | u8Data);
		
		ret = nrf_drv_twi_tx(&twi_instance, u8SlaveAddress, (uint8_t *)send_data, 2, false);
		APP_ERROR_CHECK(ret);
		
		while (twi_xfer_done == false);
	}
}



