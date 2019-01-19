#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include "FreeRTOS.h"

#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))
#define APP_TIMER_TICKS(MS) (uint32_t)ROUNDED_DIV((MS)*configTICK_RATE_HZ,1000)

void print_float(float input);
uint32_t ticks_to_ms(portTickType ticks);

#endif
