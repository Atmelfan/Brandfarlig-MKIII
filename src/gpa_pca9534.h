#ifndef PCA9534_H_
#define PCA9534_H_ 

#include "gpa_twi_generic.h"
#include <stdint.h>

#define PCA9534_ADDR	0x20

#define PCA9534_REG_INPUT 		0x00//Input register
#define PCA9534_REG_OUTPUT		0x01//Frequency prescaler 0
#define PCA9534_REG_POLARITY 	0x02//PWM 0
#define PCA9534_REG_CONFIG 		0x03//Frequency prescaler 1


/*Does nothing!*/
void pca9534_init(uint8_t addr);

/*Clears ouput, sets all pins to inputs, clears polarity*/
void pca9534_reset(uint8_t addr);

/*Reads input register*/
uint8_t pca9534_in(uint8_t addr);

/*Writes $out to output*/
void pca9534_out(uint8_t addr, uint8_t out);

/*Writes io direction*/
void pca9534_dir(uint8_t addr, uint8_t dir);

/*Sets polarity*/
void pca9534_polarity(uint8_t addr, uint8_t pol);



/*Sets bits of $set on output*/
void pca9534_set(uint8_t addr, uint8_t set);

/*Clears bits of $set on output*/
void pca9534_clr(uint8_t addr, uint8_t clr);



#endif