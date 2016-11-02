#ifndef GPA_RC5_H_
#define GPA_RC5_H_ 
#include "stdint.h"

#define RC5_TICKS_PER_US /2
//#define RC5_TICKS_PER_US *4

#define RC5_SHORT_MINPULSE 	 (444 RC5_TICKS_PER_US)
#define RC5_SHORT_MAXPULSE 	(1333 RC5_TICKS_PER_US)
#define RC5_LONG_MINPULSE 	(1334 RC5_TICKS_PER_US)
#define RC5_LONG_MAXPULSE 	(2222 RC5_TICKS_PER_US)

typedef enum
{
	RC5_STATE_START = 0, 
	RC5_STATE_LO = 1, 
    RC5_STATE_LO2 = 2, 
    RC5_STATE_HI = 3, 
    RC5_STATE_HI2 = 4, 
    
    RC5_STATE_ERR = 255
} rc5_state;

#define RC5_PIN 2

extern void rc5_on_recieve(uint16_t code);
extern void rc5_on_recieve2(uint16_t code);


void rc5_init();

void rc5_reset();


uint8_t rc5_update(uint16_t d, uint8_t pulse);

#endif