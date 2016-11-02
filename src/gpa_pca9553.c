#include "gpa_pca9553.h"

uint8_t psc0 = 0x00, psc1 = 0x00, pwm0 = 0x00, pwm1 = 0x00, ls = 0x00;

void pca9553_init(uint8_t _psc0, uint8_t _psc1){
	psc0 = _psc0;
	psc1 = _psc1;
	pca9553_reset();
	pca9553_update();
}

void pca9553_reset(){
	pwm0 = 0x00;
	pwm1 = 0x00;
	ls = 0x00;
}

void pca9553_pwm0(uint8_t duty){
	pwm0 = duty;
}

void pca9553_pwm1(uint8_t duty){
	pwm1 = duty;
}

void pca9553_set(uint8_t pin, pca9553_pin_enum mode){
	pin = 2*pin;
	ls &= ~(0x03 << pin);
	ls |= (mode << pin);
}

uint8_t send[6]; 
void pca9553_update(){
	
	send[0] = 0x10 | PCA9553_REG_PSC0;
	send[1] = psc0;
	send[2] = pwm0;
	send[3] = psc1;
	send[4] = pwm1;
	send[5] = ls;
	while(!twi_ready());
	twi_send(PCA9553_ADDR, send, 6);
	
}

uint8_t pca9553_input(){
	uint8_t ret;
	uint8_t snd = PCA9553_REG_INPUT;

	twi_send_read(PCA9553_ADDR, &snd, 1, &ret, 1);
	return ret;
}