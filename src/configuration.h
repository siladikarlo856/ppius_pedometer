#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include "IMU.h"

// PEDOMETER CONFIGURATION
#define UPPER_TRESHOLD 1.0f		// upper treshold for step detection
#define LOWER_TRESHOLD 0.5f		// lower treshold for step detection
#define MIN_TIME 500					// minimal time between two consecutive steps

// SCREEN CONFIGURATION
#define NUMBER_OF_SCREENS 4		// Number of screens on the watch

// TASK DELAY CONFIGURATION
#define WHOAMI_TASK_DELAY 1000
#define TEMPERATURE_TASK_DELAY 5000
#define ACCEL_TASK_DELAY 50
#define PEDO_TASK_DELAY 1000

// UNITS CONFIGURATION
const static temperature_units DEFAULT_TEMP_UNIT = CELSIUS;	// Temperature unit: CELSIUS or FARENHEIT

#endif
