#ifndef __IMU_INTERRUPTS_H__
#define __IMU_INTERRUPTS_H__

#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "configuration.h"
#include "pin_definitions.h"

void IMU_Interrupts_Init(void);

#endif // __IMU_INTERRUPTS_H__
