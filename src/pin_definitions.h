#ifndef _PIN_DEFINITIONS_H_
#define _PIN_DEFINITIONS_H_

#include "boards.h"

/* PIN definitions */
#define SCL_PIN 		12
#define SDA_PIN 		11

#define PIN_INT_1 	14
#define PIN_INT_2 	7

#define BLUE_LED		23

// PIN_OUT_1 BSP_LED_0
#ifdef BSP_LED_0
    #define PIN_OUT_1 BSP_LED_0
#endif
#ifndef PIN_OUT_1
    #error "Please indicate output pin"
#endif
// PIN_OUT_2 BSP_LED_1
#ifdef BSP_LED_1
    #define PIN_OUT_2 BSP_LED_1
#endif
#ifndef PIN_OUT_2
    #error "Please indicate output pin"
#endif
#endif
