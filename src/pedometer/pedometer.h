#ifndef _PEDOMETER_H_
#define _PEDOMETER_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "nrf_delay.h"

float inc_average(float value);
void calibrate_IMU(void);
float get_avg_accel(void);
bool detect_step(float total_accel);

#endif
