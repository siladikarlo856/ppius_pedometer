#ifndef _PIN_DEFINITIONS_H_
#define _PIN_DEFINITIONS_H_

#include "boards.h"

/* PIN definitions */
#define ARDUINO_SHIELD_SCL_PIN 31
#define ARDUINO_SHIELD_SDA_PIN 30

// PIN_IN_1 - ARDUINO_2_PIN
#ifdef ARDUINO_2_PIN
    #define PIN_IN_1 ARDUINO_2_PIN
#endif
#ifndef PIN_IN_1
    #error "Please indicate input pin"
#endif
// PIN_IN_2 - ARDUINO_3_PIN
#ifdef ARDUINO_3_PIN
		#define PIN_IN_2 ARDUINO_3_PIN
#endif
#ifndef PIN_IN_2
    #error "Please indicate input pin"
#endif
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
