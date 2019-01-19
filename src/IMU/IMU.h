#ifndef _IMU_H_
#define _IMU_H_

#include "SparkFunLSM6DS3.h"
#include <math.h>

/*** IMU temperature unit definitions ***/
typedef enum {CELSIUS, FARENHEIT} temperature_units;
/****************************************/

//OVO TU JE VEC SVE DEFINIRANO, STA CEMO?
/*** IMU hw tap definitions ***/
// TAP_CFG register
#define PEDO_ENABLE 0x40
#define TAP_DIR_X 	0x08
#define TAP_DIR_Y 	0x04
#define TAP_DIR_Z 	0x02
#define TAP_LIR			0x01
// MD1_CFG register
#define INT1_SINGLE_TAP 0x40
#define INT1_DOUBLE_TAP	0x08
// WAKE_UP_THS register
#define SINGLE_DOUBLE_TAP	0x80
// CTRL10_C register
#define FUNC_EN				0x04
#define PEDO_RST_STEP	0x02
/******************************/


/* IMU axis definitions */
#define IMU_ACCEL_X LSM6DS3_ACC_GYRO_OUTX_L_XL
#define IMU_ACCEL_Y LSM6DS3_ACC_GYRO_OUTY_L_XL
#define IMU_ACCEL_Z LSM6DS3_ACC_GYRO_OUTZ_L_XL
#define IMU_GYRO_X	LSM6DS3_ACC_GYRO_OUTX_L_G
#define IMU_GYRO_Y	LSM6DS3_ACC_GYRO_OUTY_L_G
#define IMU_GYRO_Z	LSM6DS3_ACC_GYRO_OUTZ_L_G

// Operation status enumerator
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	//...
} status_t;

//This struct holds the settings the driver uses to do calculations
typedef struct
{
	//Gyro settings
	uint8_t gyroEnabled;
	uint16_t gyroRange;
	uint16_t gyroSampleRate;
	uint16_t gyroBandWidth;

	uint8_t gyroFifoEnabled;
	uint8_t gyroFifoDecimation;

	//Accelerometer settings
	uint8_t accelEnabled;
	uint8_t accelODROff;
	uint16_t accelRange;
	uint16_t accelSampleRate;
	uint16_t accelBandWidth;
	
	uint8_t accelFifoEnabled;
	uint8_t accelFifoDecimation;
	
//	//FIFO control data
//	uint16_t fifoThreshold;
//	int16_t fifoSampleRate;
//	uint8_t fifoModeWord;
} IMU_Settings;

void IMU_Init(void);
void IMU_Init_Pedo(void);
void IMU_Init_HW_Tap(void);
void IMU_Init_Orient(void);

status_t readRegisterRegion(uint8_t* outputPointer , uint8_t offset, uint8_t length);
status_t readRegister(uint8_t* outputPointer, uint8_t offset);
status_t readRegisterInt16(int16_t* outputPointer, uint8_t offset);
status_t writeRegister(uint8_t offset, uint8_t dataToWrite);

float readFloatAccel(uint8_t axis);
float readTotalAccel(void);
float readFloatGyro(uint8_t axis);
float readTemp(uint8_t temp_unit);
uint16_t readSteps(void);

#endif
