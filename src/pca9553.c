#include "pca9533.h"

uint8_t psc0 = 0x00, psc1 = 0x00, pwm0 = 0x00, pwm1 = 0x00, ls = 0x00;

void pca9533_init(uint8_t _psc0, uint8_t _psc1){
	psc0 = _psc0;
	psc1 = _psc1;
}

void pca9533_reset(){
	pwm0 = 0x00;
	pwm1 = 0x00;
	ls = 0x00;
}

void pca9533_pwm0(uint8_t duty){
	pwm0 = duty;
}

void pca9533_pwm1(uint8_t duty){
	pwm1 = duty;
}

void pca9533_set(uint8_t pin, pca9533_pin_enum mode){
	ls &= ~(0x03 << 2*pin);
	ls |= (mode << 2*pin);
}

void pca9533_update(){
	
}
