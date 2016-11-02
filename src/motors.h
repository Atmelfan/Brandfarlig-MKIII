#ifndef BRANDFARLIG_MOTORS
#define BRANDFARLIG_MOTORS
#include "avr/io.h"

#define MOTOR_PORT PORTC
#define MOTOR_DIRS 0x0F

#define FAST_DECAY 1
#define SLOW_DECAY 0

typedef enum {
	MOTOR_BRAKE,
	MOTOR_COAST,
	MOTOR_FORWD,
	MOTOR_REVRS
}motor_dir;

void motors_init();

motor_dir motors_set(uint8_t channel, motor_dir dir);

void motors_pwm(uint8_t channel, int16_t speed, uint8_t decay);

void motors_coast(uint8_t channel);

void motors_force_disable();

void motors_force_enable();


#endif