#ifndef PCA9533_H_
#define PCA9533_H_ 
#include <stdint.h>

#define PCA9553_01_ADDR	0xC4
#define PCA9553_02_ADDR	0xC6

#define PCA9553_REG_INPUT 	0x00//Input register
#define PCA9553_REG_PSC0 	0x01//Frequency prescaler 0
#define PCA9553_REG_PWM0 	0x02//PWM 0
#define PCA9553_REG_PSC1 	0x03//Frequency prescaler 1
#define PCA9553_REG_PWM1	0x04//PWM 1
#define PCA9553_REG_LS		0x05//LED selector

typedef enum pca9533_pin_enum
{
	PCA9553_LS_OFF 	= 0x00,//Pin high(led off)
	PCA9553_LS_ON 	= 0x01,//Pin low(led on)
	PCA9553_LS_PWM0 = 0x02,//Pin high(led off)
	PCA9553_LS_PWM1 = 0x03//Pin high(led off)
};

void pca9533_init(uint8_t _psc0, uint8_t _psc1);

void pca9533_reset();

void pca9533_pwm0(uint8_t duty);

void pca9533_pwm1(uint8_t duty);

void pca9533_set(uint8_t pin, pca9533_pin_enum mode);

void pca9533_update();

#endif