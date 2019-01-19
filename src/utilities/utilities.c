#include <stdint.h>
#include "SEGGER_RTT.h"
#include "FreeRTOSConfig.h"
#include "utilities.h"

void print_float(float input)
{
	int32_t whole, decimal;
	
	whole = (int) input * (input < 0 ? -1 : 1);
	decimal = ((int) (input * 100)) % 100 * (input < 0 ? -1 : 1);
	
	if (input < 0)
	    SEGGER_RTT_printf(0, "-%d.%02d", whole, decimal);
	else
	    SEGGER_RTT_printf(0, "%d.%02d", whole, decimal);
}

uint32_t ticks_to_ms(portTickType ticks)
{
	float ms;
	
	ms = 1000.0f / configTICK_RATE_HZ * ticks;
	return (uint32_t) ms;
}
