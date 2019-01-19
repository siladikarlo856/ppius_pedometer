/**
 * @file		IMU.c
 * @brief		Functions related IMU
 * @author  Marko Krizmancic
 */

#include "IMU.h"
#include "I2C.h"
#include "addresses.h"

// Fill out the settings structure
const IMU_Settings settings =
{
	// Gyro settings
	.gyroEnabled = 0,  //Can be 0 or 1
	.gyroRange = 2000,   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	.gyroSampleRate = 416,   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	.gyroBandWidth = 400,  //Hz.  Can be: 50, 100, 200, 400,
	.gyroFifoEnabled = 1,  //Set to include gyro in FIFO
	.gyroFifoDecimation = 1,  //set 1 for on /1

	// Accelerometer setting
	.accelEnabled = 1,
	.accelODROff = 1,
	.accelRange = 2,      //Max G force readable.  Can be: 2, 4, 8, 16
	.accelSampleRate = 416,  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
	.accelBandWidth = 200,  //Hz.  Can be: 50, 100, 200, 400,
	.accelFifoEnabled = 1,  //Set to include accelerometer in the FIFO
	.accelFifoDecimation = 1,  //set 1 for on /1s

//	//FIFO control data
//	.fifoThreshold = 3000,  //Can be 0 to 4096 (16 bit bytes)
//	.fifoSampleRate = 10,  //default 10Hz
//	.fifoModeWord = 0,  //Default off
};

/**
 * @fn          IMU_Init
 * @brief       Initialize the IMU
 * @param				none
 * @return      none
*/
void IMU_Init(void)
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable
	
	//Setup the accelerometer******************************
	if ( settings.accelEnabled == 1) {
		//Build config reg
		//First patch in filter bandwidth
		switch (settings.accelBandWidth) {
		case 50:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
			break;
		case 100:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
			break;
		case 200:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
			break;
		default:  //set default case to max passthrough
		case 400:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
			break;
		}
		//Next, patch in full scale
		switch (settings.accelRange) {
		case 2:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
			break;
		case 4:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
			break;
		case 8:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
			break;
		default:  //set default case to 16(max)
		case 16:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
			break;
		}
		//Lastly, patch in accelerometer ODR
		switch (settings.accelSampleRate) {
		case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
			break;
		case 13330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}

	//Now, write the patched together data
	writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
	if ( settings.accelODROff == 1) {
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}
	writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);
	
	//Setup the gyroscope**********************************************
	dataToWrite = 0; //Start Fresh!
	if ( settings.gyroEnabled == 1) {
		//Build config reg
		//First, patch in full scale
		switch (settings.gyroRange) {
		case 125:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
			break;
		case 245:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
			break;
		case 500:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
			break;
		case 1000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
			break;
		default:  //Default to full 2000DPS range
		case 2000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
			break;
		}
		//Lastly, patch in gyro ODR
		switch (settings.gyroSampleRate) {
		case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}
	//Write the byte
	writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);
}

/**
 * @fn          IMU_Init_Pedo
 * @brief       Enable built-in pedometer
 * @param				none
 * @return      none
*/
void IMU_Init_Pedo(void)
{
	uint8_t errorAccumulator = 0;
	uint8_t data;
	
	// Enable embedded functions -- ALSO clears the pedo step count
	readRegister(&data, LSM6DS3_ACC_GYRO_CTRL10_C);
	data |= FUNC_EN | PEDO_RST_STEP;
	errorAccumulator += writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, data);
	
	// Enable pedometer algorithm
	readRegister(&data, LSM6DS3_ACC_GYRO_TAP_CFG1);
	data |= PEDO_ENABLE;
	errorAccumulator += writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, data);
}

/**
 * @fn          IMU_Init_HW_Tap
 * @brief       Enable interrupt on tap
 * @param				none
 * @return      none
*/
void IMU_Init_HW_Tap(void)
{
	//Error accumulation variable
	uint8_t errorAccumulator = 0;
	uint8_t data;

	// Enable tap detection on Z axis, and latch output
	readRegister(&data, LSM6DS3_ACC_GYRO_TAP_CFG1);
	data |= TAP_DIR_Z | TAP_LIR;
	errorAccumulator += writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, data );
	 
	// Set tap threshold
	// 1 LSB = FS_XL / 2^5
	readRegister(&data, LSM6DS3_ACC_GYRO_TAP_THS_6D);
	data |= 0x0c;
	errorAccumulator += writeRegister( LSM6DS3_ACC_GYRO_TAP_THS_6D, data );

	// Set Duration, Quiet and Shock time windows
	// Duration:	00 -> 16*ODR_XL; ** -> 1 LSB = 32*ODR_XL
	// Quiet:			00 ->  2*ODR_XL; ** -> 1 LSB =  4*ODR_XL
	// Shock:			00 ->  4*ODR_XL; ** -> 1 LSB =  8*ODR_XL	
	errorAccumulator += writeRegister( LSM6DS3_ACC_GYRO_INT_DUR2, 0x7f );
	
	// Single and double tap enabled (SINGLE_DOUBLE_TAP = 1)
	readRegister(&data, LSM6DS3_ACC_GYRO_WAKE_UP_THS);
	data |= SINGLE_DOUBLE_TAP;
	errorAccumulator += writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, data );
	
	// Single and double tap interrupt driven to INT1 pin -- enable latch
	readRegister(&data, LSM6DS3_ACC_GYRO_MD1_CFG);
	data |= INT1_SINGLE_TAP;// | INT1_DOUBLE_TAP;
	errorAccumulator += writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, data );
}

/**
 * @fn          IMU_Init_Orient
 * @brief       Enable interrupt on orientation change
 * @param				none
 * @return      none
*/
void IMU_Init_Orient(void)
{	
	uint8_t data;
	
	//Set 6D treshold
	readRegister(&data, LSM6DS3_ACC_GYRO_TAP_THS_6D);
	data |= 0x40;
	writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, data);
	
	//Enable LPF2 filter and latch interrupt
	readRegister(&data, LSM6DS3_ACC_GYRO_TAP_CFG1);
	//data |= 0x11;
	data |= 0x10;
	writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, data);
	
	//Apply LPF2 filter to 6D function
	readRegister(&data, LSM6DS3_ACC_GYRO_CTRL8_XL);
	data |= 0x01;
	writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, data);
	
	//Drive interrupt to INT2 pin
	readRegister(&data, LSM6DS3_ACC_GYRO_MD2_CFG);
	data |= 0x04;
	writeRegister(LSM6DS3_ACC_GYRO_MD2_CFG, data);
}

/**
 * @fn          writeRegister
 * @brief       Write data to a register on IMU
 * @param				Register address and 8 bit data to write
 * @return      Status
*/
status_t writeRegister(uint8_t offset, uint8_t dataToWrite) {
	status_t returnError = IMU_SUCCESS;

	I2C_Write(IMU_I2C_ADDRESS, offset, dataToWrite, false);

	return returnError;
}


/**
 * @fn          readRegisterRegion
 * @brief       Read multiple bytes of data from register on IMU
 * @param				Variable to save read data, register address and number of bytes to read
 * @return      Status
*/
status_t readRegisterRegion(uint8_t* outputPointer , uint8_t offset, uint8_t length)
{
	//Return value
	status_t returnError = IMU_SUCCESS;
	
	I2C_Multi_Read(IMU_I2C_ADDRESS, offset, outputPointer, length);

	return returnError;
}

/**
 * @fn          readRegister
 * @brief       Read one byte of data from register on IMU
 * @param				Variable to save read data and register address
 * @return      Status
*/
status_t readRegister(uint8_t* outputPointer, uint8_t offset)
{
	//Return value
	status_t returnError = IMU_SUCCESS;
	
	*outputPointer = I2C_Read(IMU_I2C_ADDRESS, offset);
	
	return returnError;
}

/**
 * @fn          readRegisterInt16
 * @brief       Read two bytes of data from register on IMU
 * @param				Variable to save read data and register address
 * @return      Status
*/
status_t readRegisterInt16(int16_t* outputPointer, uint8_t offset)
{
	uint8_t myBuffer[2];
	status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);
	
	*outputPointer = output;
	return returnError;
}

/**
 * @fn          readFloatAccel
 * @brief       Read acceleration in float format
 * @param				Acceleration axis (x, y or z)
 * @return      Acceleration value
*/
float readFloatAccel(uint8_t axis)
{
	int16_t raw_output;
	float calc_output;
	
	// Read raw data
	status_t errorLevel = readRegisterInt16(&raw_output, axis);
	
	// Calculate float from raw data
	calc_output = (float)raw_output * 0.061f * (settings.accelRange >> 1) / 1000;
	
	return calc_output;
}

/**
 * @fn          readTotalAccel
 * @brief       Read total acceleration (all three axis) in float format
 * @param				none
 * @return      Acceleration value
*/
float readTotalAccel(void)
{
	float accel_x, accel_y, accel_z, total;
	//Read acceleration data
	accel_x = readFloatAccel(IMU_ACCEL_X);
	accel_y = readFloatAccel(IMU_ACCEL_Y);
	accel_z = readFloatAccel(IMU_ACCEL_Z);
	
	total = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));
	return total;
}

/**
 * @fn          readFloatGyro
 * @brief       Read gyro in float format
 * @param				Gyro axis (x, y or z)
 * @return      Gyro value
*/
float readFloatGyro(uint8_t axis)
{
	uint8_t gyroRangeDivisor = settings.gyroRange / 125;
	int16_t raw_output;
	float calc_output;
	
	if ( settings.gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}
	
	// Read raw data
	status_t errorLevel = readRegisterInt16(&raw_output, axis);
	
	// Calculate float from raw data
	calc_output = (float)raw_output * 4.375f * (gyroRangeDivisor) / 1000;
	
	return calc_output;
}

/**
 * @fn          readTemp
 * @brief       Read temperature in float format
 * @param				Units (celsius or faranheit)
 * @return      Temperature value
*/
float readTemp(uint8_t temp_unit)
{
	int16_t raw_output;
	float calc_output;
	
	readRegisterInt16(&raw_output, LSM6DS3_ACC_GYRO_OUT_TEMP_L);
	calc_output = (float)raw_output / 16 + 25;
	
	if (temp_unit == FARENHEIT)
		calc_output = calc_output * 9 / 5 + 32;
	
	return calc_output;
}

/**
 * @fn          readSteps
 * @brief       Read number of steps taken from built-in pedometer
 * @param				none
 * @return      Steps taken
*/
uint16_t readSteps(void)
{
	uint8_t readDataByte = 0;
	uint16_t stepsTaken = 0;
	
	//Read the 16bit value by two 8bit operations
	readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
	stepsTaken = ((uint16_t)readDataByte) << 8;
	
	readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
	stepsTaken |= readDataByte;
	
	return stepsTaken;
}
