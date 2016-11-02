#ifndef GPA_SONAR_STEREO_H_
#define GPA_SONAR_STEREO_H_ 
#include <avr/io.h>
#include <avr/interrupt.h>

#define SONAR_PULSE_COUNT 5


#define SONAR_PORT PORTD

void sonar_init();

void sonar_gen_pulse(uint8_t ch);

void sonar_trigger(uint8_t ch);


#endif
