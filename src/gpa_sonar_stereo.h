#ifndef GPA_SONAR_STEREO_H_
#define GPA_SONAR_STEREO_H_ 
#include <avr/io.h>
#include <avr/interrupt.h>

#define SONAR_PULSE_COUNT 5

#define SONAR_NUM_CHANNELS 4

#define SONAR_PORT PORTD

//#define SONAR_TIMER TCD0

#define PULSE_MIN 100

#define SONAR_DT_MAX 110

#define SONAR_US_PER_TICK 2

#define SONAR_SOUND_VELOCITY 342.0

#define SONAR_MILLIMETERS_PER_TICK ((SONAR_US_PER_TICK)*0.34029)//0.34029mm per us, [mm/T]
/*
	
	float mm = sonar_get_range(ch)*SONAR_MILLIMETERS_PER_TICK;

	if(sonar_get_range(ch) < 1000/SONAR_MILLIMETERS_PER_TICK){}
*/

typedef enum
{
		SONAR_READY = 0,
		SONAR_MEAS_RIGHT,
		SONAR_MEAS_LEFT
} sonar_state;

typedef struct {
	uint16_t r_range;	//Time it took for the signal to return 
	uint16_t l_range;	//Time it took for the signal to return
} sonar_channel_data;


/**
 * Initiate measurement on channel $ch. If freerun == 1, it will automatically loop through all channels
 */
void sonar_trigger_free(uint8_t ch, uint8_t freerun);

/**
 * Initiate measurement on channel $ch
 */
uint8_t sonar_status(uint8_t ch);

/**
 * Returns 1 if range is available
 */
uint8_t sonar_has_range(uint8_t ch);

/**
 * Returns 1 if direction is available
 */
uint8_t sonar_has_direction(uint8_t ch);

/**
 * Calculates average range (if possible) or closest range  
 */
uint16_t sonar_get_range(uint8_t ch);

/**
 * Calculates average direction or 0 if not possible   
 */
int16_t sonar_get_direction(uint8_t ch);


void sonar_init();

void sonar_gen_pulse(uint8_t ch);

void sonar_trigger(uint8_t ch);


#endif
