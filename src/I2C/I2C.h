#ifndef _I2C_H_
#define _I2C_H_

#include "nrf_drv_twi.h"


typedef struct
{
	uint8_t wReg;
	uint8_t rReg;
	
} CurrentReg_t;


void I2C_Init(uint8_t scl, uint8_t sda);
void Handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

// I2C func. za citanje i pisanje REGISTRA
void I2C_Write(uint8_t u8SlaveAddress, uint8_t u8RegAddress, uint8_t u8Data,bool append);
void I2C_Multi_Read(uint8_t address,uint8_t u8RegAddress, uint8_t * u8Data,uint8_t lenght);
void I2C_Multi_Write(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length);
uint8_t I2C_Read(uint8_t u8SlaveAddress, uint8_t u8RegAddress);

bool I2C_Is_Ready(void);


#endif //_I2C_H_
