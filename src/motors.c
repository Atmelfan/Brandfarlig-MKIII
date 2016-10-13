#include "motors.h"

void motors_init(){
	MOTOR_PORT.DIRSET = (MOTOR_DIRS);
	TCC0_CTRLB = TC_WGMODE_SS_gc;
	TCC0_CCA = 0;
	TCC0_CCB = 0;
	TCC0_CCC = 0;
	TCC0_CCD = 0;
	TCC0_CTRLA = TC_CLKSEL_DIV1_gc;
}

#define ISREVERSE(x) (x < 0)

motor_dir motors_set(uint8_t channel, motor_dir dir){
	int8_t xin1 = channel*2 + 0;
	uint8_t xin2 = channel*2 + 1;
	switch(dir){
		case MOTOR_BRAKE:
			MOTOR_PORT.OUTSET = (1 << xin1);
			MOTOR_PORT.OUTSET = (1 << xin2);
		break;
		case MOTOR_COAST:
			MOTOR_PORT.OUTCLR = (1 << xin1);
			MOTOR_PORT.OUTCLR = (1 << xin2);
		break;
		case MOTOR_REVRS:
			MOTOR_PORT.OUTSET = (1 << xin1);
			MOTOR_PORT.OUTCLR = (1 << xin2);
		break;
		case MOTOR_FORWD:
			MOTOR_PORT.OUTCLR = (1 << xin1);
			MOTOR_PORT.OUTSET = (1 << xin2);
		break;
	}
	return dir;
}

/*	xin1 	xin2 	Out
 *  PWM 	0 		Forward, slow decay
 *  1 		PWM 	Forward, fast decay
 *	0 		PWM     Reverse, fast decay
 *  PWM 	1       Reverse, slow decay
 *  xin1 = forward
 *  xin2 = 
 */
void motors_pwm(uint8_t channel, int16_t speed, uint8_t fast){
	uint8_t xin1 = channel*2 + 0;
	uint8_t xin2 = channel*2 + 1;

	//xin1 = forward
	//xin2 = !forward
	if(ISREVERSE(speed)){
		MOTOR_PORT.OUTSET = (1 << xin1);
		MOTOR_PORT.OUTCLR = (1 << xin2);
	}else{
		MOTOR_PORT.OUTCLR = (1 << xin1);
		MOTOR_PORT.OUTSET = (1 << xin2);
	}
	//xin2pwm = fast
	//xin1pwm = !fast
	speed = speed < 0 ? -speed : speed;//Absolute value of speed 
	if(fast){//Enable xin2 pwm
		if(channel == 0){
			TCC0_CCB = speed*2;
			TCC0_CTRLB &= ~TC0_CCAEN_bm;//Disable PC0 pwm (xin1)
			TCC0_CTRLB |= TC0_CCBEN_bm;//Enable PC1 pwm (xin2)
		}else{
			TCC0_CCD = speed*2;
			TCC0_CTRLB &= ~TC0_CCCEN_bm;//Disable PC2 pwm
			TCC0_CTRLB |= TC0_CCDEN_bm;//Enable PC3 pwm
		}
		
	}else{ //Enable xin1 pwm
		if(channel == 0){
			TCC0_CCA = speed*2;
			TCC0_CTRLB &= ~TC0_CCBEN_bm;//Disable PC1 pwm (xin2)
			TCC0_CTRLB |= TC0_CCAEN_bm;//Enable PC0 pwm (xin1)
		}else{
			TCC0_CCC = speed*2;
			TCC0_CTRLB &= ~TC0_CCDEN_bm;//Disable PC3 pwm
			TCC0_CTRLB |= TC0_CCCEN_bm;//Enable PC2 pwm
		}
	}

}
