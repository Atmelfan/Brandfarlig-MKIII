#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <limits.h>
#include "motors.h"
#include "adc.h"

#define MOTOR_RIGHT 0
#define MOTOR_LEFT  1
#define MOTOR_STILL 0
#define MOTOR_FULL_FORW INT_MAX
#define MOTOR_FULL_REVS INT_MIN

#define DEBUG_MODE 0

#define EDGE_SENSOR_COUNT 4
#define EDGE_SENSOR_OFFST 0
#define EDGE_SENSOR_HYSTS 250
//Edge sensors
#define EDGE_SENSOR_FRNTL 3
#define EDGE_SENSOR_FRNTR 2
#define EDGE_SENSOR_BACKL 1
#define EDGE_SENSOR_BACKR 0

#define EDGE_SENSOR_DELAY 100

//Distance sensors
#define DIST_SENSOR_FRNTL 4
#define DIST_SENSOR_FRNTR 6
#define DIST_SENSOR_BACKL 7
#define DIST_SENSOR_BACKR 5

#define DIST_SENSOR_THRESHOLD 1750
#define DIST_TURN_FACTOR 4
#define DIST_FORW_FACTOR 1

#define SHRT_SENSOR_FRNT 0
#define SHRT_SENSOR_BACK 3

#define DIST_SENSOR_DELAY 0

typedef struct {
	int16_t value;
	int16_t background;
}edge_sensor;

edge_sensor edge_sensors[EDGE_SENSOR_COUNT]; 

/*********************************************/
//Function prototypes
/*********************************************/
void edges_read(edge_sensor sensors[], uint8_t n);

void edges_calibrate(edge_sensor sensors[], uint8_t n);

uint8_t edges_check(edge_sensor* sensor);

uint8_t start_ok();

void motors_brake(){
	motors_pwm(MOTOR_RIGHT, MOTOR_STILL, FAST_DECAY);
	motors_pwm(MOTOR_LEFT,  MOTOR_STILL, FAST_DECAY);
}

/*********************************************/
//Init
/*********************************************/
void setup(){
	/*********************************************/
	//Configure motors
	/*********************************************/
	motors_init();
	motors_brake();
	/*********************************************/
	//Configure ADC for edge & distance sensors
	/*********************************************/
	adc_init(&ADCA, ADC_REFSEL_VCC_gc, ADC_PRESCALER_DIV128_gc, ADC_RESOLUTION_12BIT_gc, 0x20, 0x21);
	adc_config(&ADCA.CH0, ADC_CH_GAIN_1X_gc, ADC_CH_INPUTMODE_SINGLEENDED_gc, 0);
	edges_calibrate(edge_sensors, EDGE_SENSOR_COUNT);

	_delay_ms(100);
#if DEBUG_MODE
	_delay_ms(4900);
#endif
}




uint8_t reverse = 1;
void update(){
#if !DEBUG_MODE
	if(!start_ok()){
		motors_brake();
		return;
	}
#endif

	//Read edge sensors
	edges_read(edge_sensors, EDGE_SENSOR_COUNT);
	uint8_t br = edges_check(&edge_sensors[EDGE_SENSOR_BACKR]),
			bl = edges_check(&edge_sensors[EDGE_SENSOR_BACKL]),
			fr = edges_check(&edge_sensors[EDGE_SENSOR_FRNTR]),
			fl = edges_check(&edge_sensors[EDGE_SENSOR_FRNTL]);

	//Read distance sensors
	uint8_t bool_dfr = adc_read_pin(&ADCA.CH0, DIST_SENSOR_FRNTR) > DIST_SENSOR_THRESHOLD,
			bool_dfl = adc_read_pin(&ADCA.CH0, DIST_SENSOR_FRNTL) > DIST_SENSOR_THRESHOLD,
			bool_dbr = adc_read_pin(&ADCA.CH0, DIST_SENSOR_BACKR) > DIST_SENSOR_THRESHOLD,
			bool_dbl = adc_read_pin(&ADCA.CH0, DIST_SENSOR_BACKL) > DIST_SENSOR_THRESHOLD;

	//Check and act upon front edge sensors
	if(fr || fl){
		motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW, FAST_DECAY);
		motors_pwm(MOTOR_LEFT,  MOTOR_FULL_FORW, FAST_DECAY);
		_delay_ms(EDGE_SENSOR_DELAY*2);
		if(fr && bl){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS, FAST_DECAY);
			motors_pwm(MOTOR_LEFT,  MOTOR_FULL_FORW, FAST_DECAY);
			_delay_ms(EDGE_SENSOR_DELAY);
		}else if(fr){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW, FAST_DECAY);
			motors_pwm(MOTOR_LEFT,  MOTOR_FULL_REVS, FAST_DECAY);
		}else if(fl){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS, FAST_DECAY);
			motors_pwm(MOTOR_LEFT,  MOTOR_FULL_FORW, FAST_DECAY);

		}
		_delay_ms(EDGE_SENSOR_DELAY);
	//Check and act upon back edge sensors
	}else if(br || bl){
		motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS, FAST_DECAY);
		motors_pwm(MOTOR_LEFT,  MOTOR_FULL_REVS, FAST_DECAY);
		_delay_ms(EDGE_SENSOR_DELAY*2);
		if(br && bl){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW, FAST_DECAY);
			motors_pwm(MOTOR_LEFT,  MOTOR_FULL_REVS, FAST_DECAY);
			_delay_ms(EDGE_SENSOR_DELAY);
		}else if(br){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS, FAST_DECAY);
			motors_pwm(MOTOR_LEFT,  MOTOR_FULL_FORW, FAST_DECAY);
		}else if(bl){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW, FAST_DECAY);
			motors_pwm(MOTOR_LEFT,  MOTOR_FULL_REVS, FAST_DECAY);
		}
		_delay_ms(EDGE_SENSOR_DELAY);

	}else{
		//Check and act upon distance sensors
		if((bool_dfr && bool_dfl) || PORTB.IN & (1 << SHRT_SENSOR_BACK)){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW/DIST_FORW_FACTOR, FAST_DECAY);
			motors_pwm(MOTOR_LEFT , MOTOR_FULL_FORW/DIST_FORW_FACTOR, FAST_DECAY);
		}else if((bool_dbr && bool_dbl) || PORTB.IN & (1 << SHRT_SENSOR_FRNT)){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS/DIST_FORW_FACTOR, FAST_DECAY);
			motors_pwm(MOTOR_LEFT , MOTOR_FULL_REVS/DIST_FORW_FACTOR, FAST_DECAY);
		}else if(bool_dfr || bool_dbl){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW/DIST_TURN_FACTOR, FAST_DECAY);
			motors_pwm(MOTOR_LEFT , MOTOR_FULL_REVS/DIST_TURN_FACTOR, FAST_DECAY);
			reverse = 1;
		}else if(bool_dfl || bool_dbr){
			motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS/DIST_TURN_FACTOR, FAST_DECAY);
			motors_pwm(MOTOR_LEFT , MOTOR_FULL_FORW/DIST_TURN_FACTOR, FAST_DECAY);
			reverse = 0;
		}else{
			if(reverse){
				motors_pwm(MOTOR_RIGHT, MOTOR_FULL_FORW/DIST_TURN_FACTOR, FAST_DECAY);
				motors_pwm(MOTOR_LEFT,  MOTOR_FULL_REVS/DIST_TURN_FACTOR, FAST_DECAY);
			}else{
				motors_pwm(MOTOR_RIGHT, MOTOR_FULL_REVS/DIST_TURN_FACTOR, FAST_DECAY);
				motors_pwm(MOTOR_LEFT , MOTOR_FULL_FORW/DIST_TURN_FACTOR, FAST_DECAY);
			}
			
		}
	}
}

void setClockTo32MHz() {
    CCP = CCP_IOREG_gc;              // disable register security for oscillator update
    OSC.CTRL = OSC_RC32MEN_bm;       // enable 32MHz oscillator
    while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for oscillator to be ready
    CCP = CCP_IOREG_gc;              // disable register security for clock update
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}

int main(void){
	setClockTo32MHz();
	setup();
	while(1){
		update();
	}
	return 1;
}

void edges_read(edge_sensor sensors[], uint8_t n){
	for (int t = 0; t < n; t++)
	{
		sensors[t].value = adc_read_pin(&ADCA.CH0, t+EDGE_SENSOR_OFFST);
	}
}

void edges_calibrate(edge_sensor sensors[], uint8_t n){
	for (int t = 0; t < n; t++)
	{
		sensors[t].background = adc_read_pin(&ADCA.CH0, t+EDGE_SENSOR_OFFST);
	}
}

uint8_t edges_check(edge_sensor* sensor){
	return sensor->value < (sensor->background - EDGE_SENSOR_HYSTS);
}

uint8_t start_ok(){
	return PORTB.IN & (1 << 1);
}




